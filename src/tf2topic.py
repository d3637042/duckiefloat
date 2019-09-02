#!/usr/bin/env python
import numpy as np
import cv2
import roslib
import rospy
import tf
import struct
import math
import time

from tf import TransformListener,TransformerROS
from tf import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point
import rospkg

class tf2topic():
	def __init__(self):
		self.map_frame = "/slam_map"
		self.robot_frame = "/base_link"
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing " %(self.node_name))
		self.pub_pose = rospy.Publisher('/robot_pose', PoseStamped, queue_size=1)
		self.get_tf()

	def get_tf(self):
		while not rospy.is_shutdown():
			t = rospy.Time(0)
			try:
				(trans, rot) = listener.lookupTransform(self.map_frame, self.robot_frame, t)
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
			transpose_matrix = transformer.fromTranslationRotation(trans, rot)
			# print(rot)
			p = np.array([0, 0, 0, 1])
			new_p = np.dot(transpose_matrix, p)
			posestamped = PoseStamped()
			posestamped.header.frame_id = self.map_frame
			posestamped.header.stamp = t
			posestamped.pose.position.x = new_p[0]
			posestamped.pose.position.y = new_p[1]
			posestamped.pose.position.z = new_p[2]
			posestamped.pose.orientation.x = rot[0]
			posestamped.pose.orientation.y = rot[1]
			posestamped.pose.orientation.z = rot[2]
			posestamped.pose.orientation.w = rot[3]
			self.pub_pose.publish(posestamped)
			rospy.sleep(0.1)

if __name__ == '__main__':
	rospy.init_node('tf2topic')
	listener = TransformListener()
	transformer = TransformerROS()
	foo = tf2topic()
	rospy.spin()