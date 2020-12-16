#!/usr/bin/env python
import rospy
from trajectory_execution_msgs.msg import PointArray
from keypoint_3d_matching_msgs.msg import Keypoint3d_list
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from trajectory_process_utils_srvs.srv import *

import numpy as np
from scipy.spatial import distance

xV_tmp, yV_tmp, zV_tmp = [], [], []
x, y, z = [], [], []
xFinal, yFinal, zFinal = [], [], []
xRaw, yRaw, zRaw = [], [], []
xMov, yMov, zMov = [], [], []
count = 0
start_flag = False
end_flag = False
start_threshold = None
pub = None
sum_time = 0
second_point = False
init_x, init_y, init_z = None, None, None
start_bz_time = None
init_point = True

def callback(data):
	global init_point, start_bz_time, init_x, init_y, init_z, second_point, sum_time, count, pub, xV_tmp, yV_tmp, zV_tmp, x, y, z, xFinal, yFinal, zFinal, xRaw, yRaw, zRaw, xMov, yMov, zMov, start_threshold, start_flag, end_flag

	# Get the RWrist keypoint
	for i in range(len(data.keypoints)):
		if (data.keypoints[i].name == "RWrist"):
			print ('received point')
			x_tmp = data.keypoints[i].points.point.x
			y_tmp = data.keypoints[i].points.point.y
			z_tmp = data.keypoints[i].points.point.z
			timestamp = data.keypoints[i].points.header.stamp.to_sec()
			break

	# Store the points after the end of the motion
	if end_flag:
		if x_tmp != 0 and y_tmp != 0 and z_tmp != 0:
			if len(xRaw) == 0 or (len(xRaw) >= 1 and abs(xRaw[-1] - x_tmp) < 0.1 and abs(yRaw[-1] - y_tmp) < 0.1 and abs(zRaw[-1] - z_tmp) < 0.1):
				xRaw.append(x_tmp)
				yRaw.append(y_tmp)
				zRaw.append(z_tmp)

	count += 1
	# Publish first point
	if init_point and x_tmp < 0 and x_tmp >= -0.45 and y_tmp < 0 and y_tmp >= -0.45:
		point = PointStamped()
		point.point.x = x_tmp
		point.point.y = y_tmp
		point.point.z = z_tmp
		point.header.stamp = rospy.Time.now()
		init_x = x_tmp
		init_y = y_tmp
		init_z = z_tmp
		pub.publish(point)
		rospy.loginfo("Num of control points: %d"%count)
		second_point = True
		init_point = False

	# if the motion has not ended, remove outliers and zeros (invalid trajectory points)
	if not end_flag:
		if x_tmp != 0 and y_tmp != 0 and z_tmp != 0:
			if len(xRaw) == 0 or (len(xRaw) >= 1 and abs(xRaw[-1] - x_tmp) < 0.1 and abs(yRaw[-1] - y_tmp) < 0.1 and abs(zRaw[-1] - z_tmp) < 0.1):
				xRaw.append(x_tmp)
				yRaw.append(y_tmp)
				zRaw.append(z_tmp)

				# If valid trajectory points, check if the motion has started
				if abs(x_tmp) < 0.6 and abs(y_tmp) < 0.6 and abs(z_tmp) < 0.6:
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
						if start_flag:
							xMov.append(x_tmp)
							yMov.append(y_tmp)
							zMov.append(z_tmp)
							x.append(x_tmp)
							y.append(y_tmp)
							z.append(z_tmp)
							
							# If the motion has started, smooth the movement using 4 trajectory points with
							# one overlapping point (the final point of one quadraple is the first of the next)
							if len(x) == 4:
								try:
									rospy.wait_for_service("trajectory_smoothing")
									smoothing = rospy.ServiceProxy("trajectory_smoothing", Smoothing)
									resp = smoothing(x, y, z)
									x = resp.x_smooth
									y = resp.y_smooth
									z = resp.z_smooth
									x_all = resp.x_smooth_all
									y_all = resp.y_smooth_all
									z_all = resp.z_smooth_all
									
									rospy.loginfo("Smoothed the trajectory")
									xFinal.extend(x)
									yFinal.extend(y)
									zFinal.extend(z)
									if len(x) > 1:
										pub_rate = (3*0.047)/(len(x)-1)
									for i in xrange(1, len(x)):
										point = PointStamped()
										point.point.x = x[i]
										point.point.y = y[i]
										point.point.z = z[i]
										point.header.stamp = rospy.Time.now()
										if second_point:
											second_point = False
											dis = distance.euclidean([x[i], y[i], z[i]], [init_x, init_y, init_z])
											num_points = dis//0.012 - 1
											for j in np.linspace(0, 1, num_points):
												if j!= 0:
													point = PointStamped()
													point.point.x = (1-j)*init_x + j*x[i]
													point.point.y = (1-j)*init_y + j*y[i]
													point.point.z = (1-j)*init_z + j*z[i]
													point.header.stamp = rospy.Time.now()
													pub.publish(point)
													rospy.sleep(0.001)
										else:
											pub.publish(point)
											rospy.sleep(pub_rate)
									
									end_time = rospy.get_time()
									x = [x[-1]]
									y = [y[-1]]
									z = [z[-1]]
								except rospy.ServiceException, e:
									rospy.logerr("Service call failed: %s"%e)	
							# Check if motion has ended
							if std_x <= 0.01 and std_y <= 0.01 and std_z <= 0.01:
								print("End movement at sample %d" %count)
								rospy.loginfo("Time elapsed: %f" %sum_time)
								end_flag = False
					
def movement_detection_node():
	rospy.init_node("movement_detection_node")
	global pub, start_threshold
	rospy.loginfo("Ready to record NEW movement")
	start_threshold = 24
	pub = rospy.Publisher("trajectory_points", PointStamped, queue_size=10, latch=True)	
	sub = rospy.Subscriber("transform_topic", Keypoint3d_list, callback)
	rospy.spin()


if __name__ == '__main__':
	movement_detection_node()

