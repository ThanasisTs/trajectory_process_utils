#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from keypoint_3d_matching_msgs.msg import Keypoint3d_list
import math
import numpy as np
from scipy.spatial import distance

x, y, z = [], [], []
xRaw, yRaw, zRaw = [], [], []
xV_tmp, yV_tmp, zV_tmp = [], [], []
pub, pub_stamp = None, None
ds_thres = 0.012
ip_thres = 0.024
start_flag = False
end_flag = False
init_point = True
start_threshold = 24
count = 0
sum_time = 0
num_points = 0
times = []
init_point = True
count_points = []
count_inter = 0
time_debug = {}
timestamp = None
x_init, y_init, z_init = [], [], []

# Interpolate points in the line segment between p1 and p2
def interpolation(p1, p2, dis, dur):
	global timestamp, x, y, z, ds_thres, pub, num_points, count_inter
	num_inter_points = dis//ds_thres
	pub_rate = dur/(num_inter_points+1)
	# rospy.loginfo("Num of points %d"%num_inter_points)
	rospy.loginfo("Duration and number of points and pub rate: %f %d %f"%(dur, num_points, pub_rate))
	# rospy.loginfo("Duration and fake Duration %f %f"%(dur, num_points*0.047))
	# pub_rate = (num_points+1)*0.047/(num_inter_points)
	try:
		time_inter = np.linspace(times[-1], timestamp, num_inter_points)
	except Exception as e:
		rospy.loginfo(e)

	for index, i in enumerate(np.linspace(0,1,num_inter_points + 1)):
		if i==0 or i==1:
			continue
		x.append((1-i)*p1[0] + i*p2[0])
		y.append((1-i)*p1[1] + i*p2[1])
		z.append((1-i)*p1[2] + i*p2[2])
		times.append(time_inter[index])
		point = PointStamped()
		point.point.x = (1-i)*p1[0] + i*p2[0]
		point.point.y = (1-i)*p1[1] + i*p2[1]
		point.point.z = (1-i)*p1[2] + i*p2[2]
		rospy.sleep(pub_rate)
		pub.publish(point)

def callback(data):
	global timestamp, init_point, times, num_points, sum_time, x, y, z, xRaw, yRaw, zRaw, xV_tmp, yV_tmp, zV_tmp, start_threshold, ds_thres, ip_thres, init_point, end_flag, start_flag, count
	global x_init, y_init, z_init
	start_time = rospy.get_time()

	# Get RWrist keypoint
	for i in range(len(data.keypoints)):
		if (data.keypoints[i].name == "RWrist"):
			x_tmp = data.keypoints[i].points.point.x
			y_tmp = data.keypoints[i].points.point.y
			z_tmp = data.keypoints[i].points.point.z
			timestamp = data.keypoints[i].points.header.stamp.to_sec()
			if len(x_init) >= 1 and abs(x_init[-1]-x_tmp) < 0.1 and abs(y_init[-1]-y_tmp) < 0.1 and abs(z_init[-1]-z_tmp) < 0.1:
				x_init.append(x_tmp)
				y_init.append(y_tmp)
				z_init.append(z_tmp)
			elif len(x_init) == 0:
				x_init.append(x_tmp)
				y_init.append(y_tmp)
				z_init.append(z_tmp)
	
	count_points.append(count)
	count += 1
	# Average the 15 first points to get the first point 
	# in order to avoid the case where the first point is outlier	
	if init_point and len(x_init) == 15:
		point = PointStamped()
		point.point.x = np.mean(x_init)
		point.point.y = np.mean(y_init)
		point.point.z = np.mean(y_init)
		pub.publish(point)
		init_point = False
		rospy.loginfo("Published initial point")

	if not init_point:
		if not end_flag:
			# Check for outliers or zeros (invalid trajectory points)
			if x_tmp != 0 and y_tmp != 0 and z_tmp != 0:
				if len(xRaw) == 0 or (len(xRaw) >= 1 and abs(xRaw[-1] - x_tmp) < 0.1 and abs(yRaw[-1] - y_tmp) < 0.1 and abs(zRaw[-1] - z_tmp) < 0.1):
					xRaw.append(x_tmp)
					yRaw.append(y_tmp)
					zRaw.append(z_tmp)
					if len(xV_tmp) == start_threshold:
						del xV_tmp[0]
						del yV_tmp[0]
						del zV_tmp[0]
					xV_tmp.append(x_tmp)
					yV_tmp.append(y_tmp)
					zV_tmp.append(z_tmp)
					if len(xV_tmp) >= 2:
						std_x = np.std(xV_tmp)
						std_y = np.std(yV_tmp)
						std_z = np.std(zV_tmp)
						if (not start_flag) and (std_x > 0.01 or std_y > 0.01 or std_z > 0.01):
							print("Start movement at sample %d" %count)
							start_flag = True
						
						# If motion has started, check if there is a need to downsample 
						# the points (high points density) or interpolate points (sparse points)
						if start_flag:
							if len(x) == 0:
								x.append(x_tmp)
								y.append(y_tmp)
								z.append(z_tmp)
								times.append(timestamp)
								time_debug[count] = timestamp
								point = PointStamped()
								point.point.x = x_tmp
								point.point.y = y_tmp
								point.point.z = z_tmp
								pub.publish(point)
							else:
								dis = distance.euclidean(list(zip(x, y, z))[-1], [x_tmp, y_tmp, z_tmp])
								if dis > ds_thres:
									if dis < ip_thres:
										x.append(x_tmp)
										y.append(y_tmp)
										z.append(z_tmp)
										times.append(timestamp)
										point = PointStamped()
										point.point.x = x_tmp
										point.point.y = y_tmp
										point.point.z = z_tmp
										time_debug[count] = timestamp
										pub.publish(point)
									else:
										# print (time_debug.items()[-1])
										# print (count, timestamp)
										# print (timestamp - time_debug.values()[-1])
										# print (num_points)
										interpolation(list(zip(x, y, z))[-1], [x_tmp, y_tmp, z_tmp], dis, timestamp-times[-1])
										num_points = 0
										end_time = rospy.get_time()
										sum_time += end_time - start_time
								else:
									num_points += 1
									# print (num_points)
							# Check if motion has ended
							if std_x <= 0.01 and std_y <= 0.01 and std_z <= 0.01:
								print("End movement at sample %d" %count)
								rospy.loginfo("Time elapsed: %f" %sum_time)
								end_flag = True

def movement_detection_node():
	global pub
	rospy.init_node("movement_detection_downsampling_node")
	print ("Started node")
	pub = rospy.Publisher("trajectory_points", PointStamped, queue_size=10, latch=True)
	sub = rospy.Subscriber("transform_topic", Keypoint3d_list, callback)
	rospy.spin()



if __name__ == "__main__":
	movement_detection_node()