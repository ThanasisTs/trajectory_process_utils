#!/usr/bin/env python 
import rospy
from geometry_msgs.msg import PointStamped
from keypoint_3d_matching_msgs.msg import Keypoint3d_list
from std_msgs.msg import Bool

import numpy as np

pub = None
x, y, z = [], [], []
x_mov, y_mov, z_mov = [], [], []
init_flag, start = False, False
count = 0
end = False
halt_motion = False

def motion_control_callback(msg):
	global halt_motion
	halt_motion = msg.data

def callback(msg):
	global pub, x, y, z, init_flag, x_mov, y_mov, start, end, count, halt_motion
	
	if not halt_motion:
		count += 1
		# Get RWrist keypoint
		for i in range(len(msg.keypoints)):
			if msg.keypoints[i].name == "RWrist":
				x_point = msg.keypoints[i].points.point.x
				y_point = msg.keypoints[i].points.point.y
				z_point = msg.keypoints[i].points.point.z
				time_point = msg.keypoints[i].points.header.stamp
				if len(x) > 1:
					if abs(x[-1]-x_point) < 0.1 and abs(y[-1]-y_point) < 0.1 and abs(z[-1]-z_point) < 0.1:
						x.append(x_point)
						y.append(y_point)
						z.append(z_point)
				else:
					x.append(x_point)
					y.append(y_point)
					z.append(z_point)
		
		# Average the 15 first points to get the first point 
		# in order to avoid the case where the first point is outlier
		if not init_flag and len(x) == 15:
			init_point = PointStamped()
			init_point.header.stamp = time_point
			init_point.point.x = np.mean(x)
			init_point.point.y = np.mean(y)
			init_point.point.z = np.mean(z)
			pub.publish(init_point)
			init_flag = True
			rospy.loginfo("Published initial point")
			print (init_point)

		if init_flag:
			# Check for outliers or zeros (invalid trajectory points)
			if len(x_mov) == 0:
				x_mov.append(x_point)
				y_mov.append(y_point)
				z_mov.append(z_point)
			else:
				if x_point == 0.0 and y_point == 0.0 and z_point == 0.0:
					valid_point = False
				elif abs(x_mov[-1] - x_point) < 0.1 and abs(y_mov[-1] - y_point) < 0.1 and abs(z_mov[-1] - z_point) < 0.1:
					# rospy.loginfo("Valid point")
					# rospy.loginfo(count)
					x_mov.append(x_point)
					y_mov.append(y_point)
					z_mov.append(z_point)
					valid_point = True
				else:
					valid_point = False

			# If valid trajectory point, check if the motion has started
			if len(x_mov) > 1 and valid_point:
				if len(x_mov) == 25:
					del x_mov[0]
					del y_mov[0]
					del z_mov[0]
				std_x = np.std(x_mov)
				std_y = np.std(y_mov)
				std_z = np.std(z_mov)
				if (not start) and (std_x > 0.01 or std_y > 0.01 or std_z > 0.01):
					rospy.loginfo("Motion started at sample %d"%count)
					rospy.loginfo('STD: %f, %f, %f'%(std_x, std_y, std_z))
					start = True
				if start:
					point = PointStamped()
					point.header.stamp = time_point
					point.point.x = x_point
					point.point.y = y_point
					point.point.z = z_point
					pub.publish(point)

def main():
	global pub
	rospy.init_node("raw_points")
	sub = rospy.Subscriber("transform_topic", Keypoint3d_list, callback)
	sub_motion_control = rospy.Subscriber("/check_keypoints_placement_topic", Bool, motion_control_callback)
	pub = rospy.Publisher("trajectory_points", PointStamped, queue_size=10, latch=True)
	rospy.spin()

main()
