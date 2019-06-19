#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
class StateEstimator{
  private:
  	// ros related
  	string node_name;
  	ros::NodeHandle nh, pnh;
  	ros::Subscriber sub_pc;
  	ros::Publisher pub_pc;
  	ros::Publisher pub_odom;
  	ros::Publisher pub_current_pose;
  	ros::Publisher pub_target_pose;
  	// params
  	const double height_max = 0.3;
  	const double height_min = -0.3;

  public:
  	//constructor
  	void cbpc(const sensor_msgs::PointCloud2ConstPtr&);
  	void pc_handler(const pcl::PCLPointCloud2::Ptr, pcl::PCLPointCloud2::Ptr);

  	StateEstimator(ros::NodeHandle &n, ros::NodeHandle& pn){
	nh = n;
	pnh = pn;
	node_name = ros::this_node::getName();
	ROS_INFO("Node class constructed");
	sub_pc = pnh.subscribe("/camera/depth/color/points", 1, &StateEstimator::cbpc, this);
	pub_pc = pnh.advertise<sensor_msgs::PointCloud2> ("pc_out", 1);
	pub_current_pose = pnh.advertise<std_msgs::Float32MultiArray>("current_pose", 1000);
	pub_target_pose = pnh.advertise<std_msgs::Float32MultiArray>("target_pose", 1000);
	pub_odom = pnh.advertise<nav_msgs::Odometry> ("odom", 1);

}
};





void StateEstimator::cbpc(const sensor_msgs::PointCloud2ConstPtr& pc_msg){
	pcl::PCLPointCloud2::Ptr cloud_in(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_out(new pcl::PCLPointCloud2());

	//std::cerr << pc_msg->fields[0] << std::endl; 
	/*
	sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_rgb(*pc_msg, "rgb");
	for(;iter_rgb != iter_rgb.end(); ++iter_rgb){
		std::cerr << iter_rgb[0] << " " << iter_rgb[1] << " " << iter_rgb[2] << std::endl;
	}
	*/
	
	pcl_conversions::toPCL (*pc_msg, *cloud_in);
	pc_handler(cloud_in, cloud_out);
	pub_pc.publish(cloud_out);
	
}

void StateEstimator::pc_handler(const pcl::PCLPointCloud2::Ptr cloud_in, pcl::PCLPointCloud2::Ptr cloud_out){
	


	//downsample
	//std::cerr << "PointCloud before filtering: " << cloud_in->width * cloud_in->height << " data points (" << pcl::getFieldsList (*cloud_in) << ").";
	pcl::PCLPointCloud2::Ptr cloud_ds(new pcl::PCLPointCloud2());
	pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
	vox.setInputCloud(cloud_in);
	vox.setLeafSize (0.1f, 0.1f, 0.1f);
	vox.filter(*cloud_ds);

	PointCloudXYZ::Ptr cloud(new PointCloudXYZ()); 
	pcl::fromPCLPointCloud2 (*cloud_ds, *cloud);

	
	//remove NaN
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
	
	
	//rotate pointcloud
	float theta = -0.3;
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
	pcl::transformPointCloud (*cloud, *cloud, transform);

	
	//Final goal: find the possible shape of the current pointcloud. 

	
	//filter height limit
	
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(this->height_min, this->height_max);
	pass.filter(*cloud);
	
	//find closest side-wall?
	//iterate thrcugh all the points and find if there is a wall, follow wall
	cv::Mat image = cv::Mat::zeros(200, 200, CV_8U);
	cv::Mat image_r = cv::Mat::zeros(200, 200, CV_8UC3);
	for(PointCloudXYZ::iterator it=cloud->begin(); it!=cloud->end(); it++){
		//cout << it->x << " " << it->z << endl;
		if (abs(it->x) < 5 && abs(it->z) < 10){
			cv::Point pt = cv::Point(int((it->x)*20)+100, int((it->z)*20));
			cv::circle( image, pt, 1, cv::Scalar(255), -1, 8 );
			
		}
		
	}
	//cv::flip(image, image, 0);


	vector<cv::Vec4i> lines;
	cv::HoughLinesP(image, lines, 1, CV_PI/180, 80, 60, 20 );
	//void HoughLinesP(InputArray image, OutputArray lines, double rho, double theta, int threshold, double minLineLength=0, double maxLineGap=0 )
	//cout << lines.size() << endl;
	
	float left_wall[2] = {0};
	float right_wall[2] = {0};
	float front_wall[2] = {0};
	int right_wall_count = 0;
	int left_wall_count = 0;
	int front_wall_count = 0;
	int front_wall_dist = 0;
	double acc_right_ang = 0;
	double acc_right_slope = 0;
	double acc_right_intercept = 0;
	double acc_left_ang = 0;
	double acc_left_slope = 0;
	double acc_left_intercept = 0;
	double acc_front_ang = 0;
	double acc_right_dis = 0;
	double acc_left_dis = 0;
	double acc_front_dis = 0;
	double max_right_point[2] = {0};
	double max_left_point[2] = {0};
	int right_corner = 0;
	int left_corner = 0;
	vector<float> current_pose(2, 0);
	vector<float> target_pose(2, 0);
	int policy_mode = 0;
	cv::cvtColor(image, image_r, CV_GRAY2BGR);
	for( int i = 0; i < lines.size(); i++ )
	{

		cv::Vec4i l = lines[i];
		if (l[1] < l[3]){
			int tmpx = l[0];
			int tmpy = l[1];
			l[0] = l[2];
			l[1] = l[3];
			l[2] = tmpx;
			l[3] = tmpy;
		}
		double ang = atan2((l[1]-l[3]), (l[0]-l[2]));
		double slope;

		if (abs(l[0]-l[2]) > 0.001){
			slope = double((l[1]-l[3]))/double((l[0]-l[2]));
			//std::cout << "debug " << l[1]-l[3] << " " << l[0]-l[2] << " " << slope <<std::endl;
		}
		else{
			slope = 1000;
		}
		double intercept = l[1] - slope* l[0];
		double dis_to_line = abs(((slope*100 + intercept)/sqrt(slope*slope+1))/20);
		//std::cout << ang << " " << dis_to_line << std::endl;
		//side wall
		double x_intercept = -intercept/slope;
		//std::cout << x_intercept << " " << intercept << " " << slope << std::endl;
		//std::cout << l[0] << " " << l[1] << " " << l[2] << " " << l[3] << std::endl;
		if(abs(ang) < 0.3 || abs(ang-3.14) < 0.3){
			cv::arrowedLine( image_r, cv::Point(l[2], l[3]), cv::Point(l[0], l[1]), cv::Scalar(0, 0, 255), 1, CV_AA);
			acc_front_ang += ang-1.57;
			acc_front_dis += dis_to_line;
			front_wall_count += 1;
			std::cout << "front wall detected dist:" << intercept/20 << std::endl;
			front_wall_dist = (intercept/20);
			//cout << ((l[2]-max_right_point[0])*(l[2]-max_right_point[0])+(l[3]-max_right_point[1])*(l[3]-max_right_point[1])) << endl;
			//cout << ((l[0]-max_right_point[0])*(l[0]-max_right_point[0])+(l[1]-max_right_point[1])*(l[1]-max_right_point[1])) << endl;
			if (((l[2]-max_right_point[0])*(l[2]-max_right_point[0])+(l[3]-max_right_point[1])*(l[3]-max_right_point[1])) < 3000){
				right_corner = 1;
			}
			if (((l[0]-max_right_point[0])*(l[0]-max_right_point[0])+(l[1]-max_right_point[1])*(l[1]-max_right_point[1])) < 3000){
				right_corner = 1;
			}
			if (((l[2]-max_left_point[0])*(l[2]-max_left_point[0])+(l[3]-max_left_point[1])*(l[3]-max_left_point[1])) < 3000){
				left_corner = 1;
			}
			if (((l[0]-max_left_point[0])*(l[0]-max_left_point[0])+(l[1]-max_left_point[1])*(l[1]-max_left_point[1])) < 3000){
				left_corner = 1;
			}

		}

		else {
			if(l[3] > 100)
				continue;
			//std::cout << "l1 l3 " << l[1] << " " << l[3] << std::endl;
			if (x_intercept > 100){ //right wall
				//std::cout << "r intercept " << x_intercept << std::endl;
				cv::arrowedLine( image_r, cv::Point(l[2], l[3]), cv::Point(l[0], l[1]), cv::Scalar(0, 255, 0), 1, CV_AA);
				acc_right_slope += slope; 
				acc_right_intercept += intercept; 
				acc_right_ang += ang-1.57;
				acc_right_dis += dis_to_line;
				right_wall_count += 1;
				if (l[3] > max_right_point[1]){
					max_right_point[0] = l[2];
					max_right_point[1] = l[3];
				}
				
			} 
			else{ //left wall
				//std::cout << "l intercept " << x_intercept << std::endl;
				cv::arrowedLine( image_r, cv::Point(l[2], l[3]), cv::Point(l[0], l[1]), cv::Scalar(255, 0, 0), 1, CV_AA);
				acc_left_slope += slope; 
				acc_left_intercept += intercept; 
				acc_left_ang += ang-1.57;
				acc_left_dis += dis_to_line;
				left_wall_count += 1;
				if (l[3] > max_left_point[1]){
					max_left_point[0] = l[2];
					max_left_point[1] = l[3];
				}
			}

			//cout << ang << " " << dis_to_line << endl;
		} 
		



	}
	
	
	
	if (left_wall_count != 0){
		left_wall[0] = acc_left_ang/left_wall_count;
		left_wall[1] = acc_left_dis/left_wall_count;
		double slope = acc_left_slope/left_wall_count;
		double intercept = acc_left_intercept/left_wall_count;
		cout << "left "<< left_wall[0] << " " << left_wall[1] << endl;
		if (abs(slope)>0.05)
			cv::line( image_r, cv::Point(0, int(intercept)), cv::Point(int(-intercept/slope), 0), cv::Scalar(100, 0, 0), 2, CV_AA);
		else
			cv::line( image_r, cv::Point(0, int(intercept)), cv::Point(1, int(intercept)), cv::Scalar(100, 0, 0), 2, CV_AA);
	}
	if (right_wall_count != 0){		
		right_wall[0] = acc_right_ang/right_wall_count;
		right_wall[1] = acc_right_dis/right_wall_count;
		double slope = acc_right_slope/right_wall_count;
		double intercept = acc_right_intercept/right_wall_count;
		cout << "right "<< right_wall[0] << " " << right_wall[1] << endl;
		if (abs(slope)>0.05)
			cv::line( image_r, cv::Point(0, int(intercept)), cv::Point(int(-intercept/slope), 0), cv::Scalar(0, 100, 0), 2, CV_AA);
		else
			cv::line( image_r, cv::Point(0, int(intercept)), cv::Point(1, int(intercept)), cv::Scalar(0, 100, 0), 2, CV_AA);

	}
	if (front_wall_count != 0){
		front_wall[0] = acc_front_ang/front_wall_count;
		front_wall[1] = acc_front_dis/front_wall_count;
		//cout << "front "<< front_wall[0] << " " << front_wall[1] << endl;
	}
	if (left_corner == 1){
		cout << "left corner detected" << endl; 
	}
	if (right_corner == 1){
		cout << "right corner detected" << endl; 
	}

	/*
	cv::flip(image_r, image_r, 0);
	cv::imshow("sliced pc", image_r);
	cv::waitKey(10); 
	*/

	// Policy
	// first step: find current pose
	//if left and right walls are detected check if parallel yes-> generate pose/ no-> get pose from left wall
	// second step: find target pose
	//make sure if there is free space ahead if no-> send move signal/ yes -> find if there are exits yes -> modify pose no-> spin around

	//policy mode: stupid. Turn right if possible, else go straight. If encounter dead end, rotate 180 degrees.

	if(left_wall_count > 0 && right_wall_count > 0){
		//check if parallel and tunnel confirmed
		if(abs(left_wall[0]-right_wall[0]) < 0.25 and right_wall[1]+left_wall[1] > 2){
			current_pose[0] = -(left_wall[0]+right_wall[0])/2;
			current_pose[1] = left_wall[1]-right_wall[1];
			std::cout << "tunnel detected" <<" current pose: " << current_pose[0] << " " << current_pose[1] << std::endl;
		}
		else{//get pose from right wall
			current_pose[0] = -right_wall[0];
			current_pose[1] = 1.3-right_wall[1];
		}
	}
	else if(left_wall_count > 0){
		current_pose[0] = -left_wall[0];
		current_pose[1] = left_wall[1]-1.3;
		std::cout << "only left wall detected" <<" current pose: " << current_pose[0] << " " << current_pose[1] << std::endl;
	}
	else if(right_wall_count > 0){
		current_pose[0] = -right_wall[0];
		current_pose[1] = 1.3-right_wall[1];
		std::cout << "only right wall detected" <<" current pose: " << current_pose[0] << " " << current_pose[1] << std::endl;
	}

	//publish current pose
	std_msgs::Float32MultiArray cmsg;
	cmsg.data = current_pose;
	pub_current_pose.publish(cmsg);

	if (policy_mode == 0){
		if(front_wall_count == 0){//if no front wall go straight
			if(1){//detect if there is a intersection
				target_pose[0] = 0;
				target_pose[1] = 0;
			}
		else{
			if(front_wall_dist < 2.5){// turn right
				target_pose[0] = -1.57;
				target_pose[1] = 0;
			}
		}
		std_msgs::Float32MultiArray tmsg;
		tmsg.data = target_pose;
		pub_target_pose.publish(tmsg);
	}

	}
	



	/*
	nav_msgs::Odometry odom_msg;

	odom_msg.header.frame_id = "camera_link";
	odom_msg.header.stamp = ros::Time::now();
	odom_msg.pose.pose.position.x = 0; 
	odom_msg.pose.pose.position.y = 0;
	odom_msg.pose.pose.position.z = 0;

	tf2::Quaternion q;
	q.setRPY(0, 0, 0);
	geometry_msgs::Quaternion quat_msg;
	quat_msg = tf2::toMsg(q);
	odom_msg.pose.pose.orientation = quat_msg;
	pub_odom.publish(odom_msg);
	*/

	//check if there are junctions
	// corner detector



	
	


	pcl::toPCLPointCloud2(*cloud, *cloud_out);
	//*cloud_out = *cloud_ds;
	

}

