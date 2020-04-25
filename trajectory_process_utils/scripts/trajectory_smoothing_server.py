#!/usr/bin/env python
import rospy
import sys
from trajectory_point_process_msgs.srv import *
from scipy.spatial import distance

from smooth_files import trajectory_smoothing_function, functions

def handle_smoothing(req):
	x = req.x
	y = req.y
	z = req.z

	x_1,y_1,z_1 = trajectory_smoothing_function.main(x,y,z)
	x_smooth,y_smooth,z_smooth = [x_1[0]],[y_1[0]],[z_1[0]]

	a = (x_1[0],y_1[0],z_1[0])
	for i in range(len(x_1)):
		b = (x_1[i],y_1[i],z_1[i])
		dst = distance.euclidean(a, b)
		
		if dst > 0.0055:
			x_smooth.append(x_1[i])
			y_smooth.append(y_1[i])
			z_smooth.append(z_1[i])
			a = (x_1[i],y_1[i],z_1[i])

	rospy.loginfo("Wait to plot the trajectory")
	functions.print2D([x,y,z],[x_smooth,y_smooth,z_smooth])
	functions.print3D([x,y,z],[x_smooth,y_smooth,z_smooth])

	return SmoothingResponse(x_smooth,y_smooth,z_smooth)


def trajectory_smoothing_server():
	rospy.init_node('trajectory_smoothing_server')
	s = rospy.Service('trajectory_smoothing',Smoothing,handle_smoothing)
	rospy.loginfo("Ready to smooth trajectory")
	rospy.spin()

if __name__ == '__main__':
	trajectory_smoothing_server()