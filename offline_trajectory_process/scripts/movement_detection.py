#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from keypoint_3d_matching_msgs.msg import Keypoint3d_list
from trajectory_custom_msgs.msg import PointStampedArray
from trajectory_process_utils_srvs.srv import *

import numpy as np
import matplotlib.pyplot as plt

xRaw, yRaw, zRaw = [], [], []
xV_tmp, yV_tmp, zV_tmp, tV_tmp = [], [], [], []
x, y, z, t = [], [], [], []
outliers_count = []
count = 0
movement_start = False
movement_end = False
movement_recording = True
invalid_movement = False
num_outliers, num_points_std, std_threshold = None, None, None
timenow = None
times = []
def callback(data, args):
	global timenow, xRaw, yRaw, zRaw, x, y, z, t, xV_tmp, yV_tmp, zV_tmp, tV_tmp, movement_start, movement_end, count, movement_recording, outliers_count, invalid_movement, num_points_std, std_threshold, num_outliers
	if movement_recording:

		# Comment this block of code if not using input of type geometry_msgs/Point 
		x_tmp = data.x
		y_tmp = data.y
		z_tmp = data.z
		timestamp = rospy.get_time()
		count += 1
		try:
			# rospy.loginfo("Time duration: %f"%(rospy.Time.now().to_sec() - timenow))
			times.append(rospy.Time.now().to_sec() - timenow)
		except Exception as e:
			rospy.logwarn(e)
		timenow = rospy.Time.now().to_sec()

		# Uncomment this block of code when using input of type Keypoints_list
		# for i in range(len(data.keypoints)):
		# 	if (data.keypoints[i].name == "RWrist"):
		# 		x_tmp = data.keypoints[i].points.point.x
		# 		y_tmp = data.keypoints[i].points.point.y
		# 		z_tmp = data.keypoints[i].points.point.z
		# 		timestamp = rospy.get_time()
		# 		count += 1
		# 		break
		if x_tmp != 0 and y_tmp != 0 and z_tmp != 0:
			if len(xRaw) == 0 or (len(xRaw) >= 1 and abs(xRaw[-1] - x_tmp) < 0.1 and abs(yRaw[-1] - y_tmp) < 0.1 and abs(zRaw[-1] - z_tmp) < 0.1):
				xRaw.append(x_tmp)
				yRaw.append(y_tmp)
				zRaw.append(z_tmp)
				if abs(x_tmp) < 0.6 and abs(y_tmp) < 0.6 and abs(z_tmp) < 0.6:
					if len(xV_tmp) == num_points_std:
						del xV_tmp[0]
						del yV_tmp[0]
						del zV_tmp[0]
						del tV_tmp[0]
					xV_tmp.append(x_tmp)
					yV_tmp.append(y_tmp)
					zV_tmp.append(z_tmp)
					tV_tmp.append(timestamp)
					if len(xV_tmp) >= 2:
						std_x = np.std(xV_tmp)
						std_y = np.std(yV_tmp)
						std_z = np.std(zV_tmp)
						if (not movement_start) and (std_x > std_threshold or std_y > std_threshold or std_z > std_threshold):
							rospy.loginfo("Start movement at sample %d" %count)
							for k in xrange(len(xV_tmp)-num_points_std, len(xV_tmp)-1):
								x.append(xV_tmp[k])
								y.append(yV_tmp[k])
								z.append(zV_tmp[k])
								t.append(tV_tmp[k])
							movement_start = True
						if movement_start:
							x.append(x_tmp)
							y.append(y_tmp)
							z.append(z_tmp)
							t.append(timestamp)
							if std_x <= std_threshold and std_y <= std_threshold and std_z <= std_threshold:
								rospy.loginfo("End movement at sample %d" %count)
								movement_end = True
								movement_recording = False
			else:
				outliers_count.append(count)
				if len(outliers_count) == num_outliers and len(range(min(outliers_count), max(outliers_count)+1)) == num_outliers:
					rospy.logwarn("Invalid movement. Please record a new movement")
					invalid_movement = True
					movement_recording = False
	# else:
	# 	rospy.logerr("Cannot record movement")
					
def movement_detection_node():
	rospy.init_node("movement_detection_node")
	global times, x, y, z, t, xRaw, yRaw, zRaw, xV_tmp, yV_tmp, zV_tmp, tV_tmp, movement_recording, movement_start, movement_end, count, invalid_movement, outliers_count, num_points_std, std_threshold, num_outliers
	rospy.loginfo("Ready to record NEW movement")
	
	smooth_flag = rospy.get_param("movement_detection_node/smooth", False)
	filter_flag = rospy.get_param("movement_detection_node/filter", False)
	num_points_std = rospy.get_param("movement_detection_node/num_points_std", 24)
	std_threshold = rospy.get_param("movement_detection_node/std_threshold", 0.01)
	num_outliers = rospy.get_param("movement_detection_node/num_outliers", 10)
	smooth_service_name = rospy.get_param("movement_detection_node/smooth_service_name", "trajectory_smoothing")
	filter_service_name = rospy.get_param("movement_detection_node/filter_service_name", "static_points_filtering")

	# Use the following subscription if you use the movement detection function
	# using Openpose and the custom message Keypoint3d_list
	# sub = rospy.Subscriber('raw_points_online', Keypoint3d_list, callback, num_points_std)
	
	# Use the following subscription if you use the movement detection function
	# using a geometry_msgs/Point msg for each point
	sub = rospy.Subscriber('raw_points', Point, callback, num_points_std)
	
	pub = rospy.Publisher('/trajectory_points', PointStampedArray, queue_size=10)
	raw_pub = rospy.Publisher('/raw_movement_points', Point, queue_size=10)
	x_raw, y_raw, z_raw = [], [], []

	msg = PointStampedArray()
	while not rospy.is_shutdown():
		if (not movement_recording and not invalid_movement):
			rospy.loginfo("Mean value of time steps: %f"%np.mean(times))
			times = []
			timenow = None
			for i in xrange(len(x)):
				x_raw.append(x[i])
				y_raw.append(y[i])
				z_raw.append(z[i])

				point = Point()
				point.x = x[i]
				point.y = y[i]
				point.z = z[i]
				raw_pub.publish(point)
				rospy.sleep(0.05)
			if filter_flag:
				try:
					rospy.wait_for_service(filter_service_name)
					filtering = rospy.ServiceProxy(filter_service_name, Filtering)
					resp = filtering(x, y, z)
					x = resp.x
					y = resp.y
					z = resp.z
					# t = resp.t
					rospy.loginfo("Filtered the points")
				except rospy.ServiceException, e:
					rospy.logerr("Cleaning service call failed: %s"%e)				

			if smooth_flag:
				try:
					rospy.wait_for_service(smooth_service_name)
					smoothing = rospy.ServiceProxy(smooth_service_name, Smoothing)
					resp = smoothing(x, y, z)
					x = resp.x_smooth
					y = resp.y_smooth
					z = resp.z_smooth
					rospy.loginfo("Smoothed the trajectory")
				except rospy.ServiceException, e:
					rospy.logerr("Smoothing service call failed: %s"%e)	

			fig = plt.figure()
			ax = plt.axes()
			ax.scatter(x_raw, y_raw, c='blue', s=20)			
			ax.scatter(x, y, c='orange', s=20)
			ax.set_xlabel('x(m)')
			ax.set_ylabel('y(m)')
			ax.grid()
			plt.show()
			x_raw, y_raw, z_raw = [], [], []

			for i in xrange(len(x)):
				point = PointStamped()
				point.point.x = x[i]
				point.point.y = y[i]
				point.point.z = z[i]
				msg.points.append(point)
			pub.publish(msg)
			# msg_flag = True
			msg.points = []
			movement_recording = True
			movement_start = False
			movement_end = False
			count = 0
			x, y, z, t = [], [], [], []
			xV_tmp, yV_tmp, zV_tmp, tV_tmp = [], [], [], []
			xRaw, yRaw, zRaw = [], [], []
			outliers_count = []			
			rospy.sleep(1)
			rospy.loginfo("Ready to record NEW movement")

		if invalid_movement:
			rospy.loginfo("Sleep for 10 secs to setup the movement")
			rospy.sleep(10)
			invalid_movement = False
			movement_recording = True
			movement_start = False
			movement_end = False
			count = 0
			x, y, z, t = [], [], [], []
			xV_tmp, yV_tmp, zV_tmp, tV_tmp = [], [], [], []
			xRaw, yRaw, zRaw = [], [], []
			outliers_count = []
			rospy.loginfo("Ready to record NEW movement")

if __name__ == '__main__':
	movement_detection_node()
