# include "state_estimator.hpp"

int main(int argc, char** argv){
	ros::init(argc, argv, "state_estimator_node");
	ros::NodeHandle nh, pnh("~");
	StateEstimator s(nh, pnh);
	while(ros::ok()) ros::spinOnce();
	return 0;
}