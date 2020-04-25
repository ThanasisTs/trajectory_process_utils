#!/usr/bin/env python
import rospy
from trajectory_point_process_msgs.srv import *
from statistics import median


median_threshold = None
boundX = None
boundY = None
boundZ = None

def handle_filtering(req):
	global median_threshold, boundX, boundY, boundZ
	x = req.x
	y = req.y
	z = req.z

	for i in xrange(median_threshold, len(x)-1):
		if abs(x[i] - median(x[0:i])) > boundX or abs(y[i] - median(y[0:i])) > boundY or abs(z[i] - median(z[0:i])) > boundZ:
			break

	for j in xrange(len(x)-median_threshold, 1, -1):
		if abs(x[j] - median(x[j:len(x)])) > boundX or abs(y[j] - median(y[j:len(y)])) > boundY:
			break

	x = x[i-3:j+3]
	y = y[i-3:j+3]
	z = z[i-3:j+3]
	return FilteringResponse(x, y, z)


def static_points_filtering_server():
	rospy.init_node("static_points_filtering_server")
	global median_threshold, boundX, boundY, boundZ
	median_threshold = rospy.get_param("/static_points_filtering_server/median_threshold", 12)
	boundX = rospy.get_param("/static_points_filtering_server/boundX", 0.012)
	boundY = rospy.get_param("/static_points_filtering_server/boundY", 0.012)
	boundZ = rospy.get_param("/static_points_filtering_server/boundZ", 0.012)
	s = rospy.Service('static_points_filtering', Filtering, handle_filtering)
	rospy.loginfo("Ready to remove redundant points")
	rospy.spin()



if __name__=="__main__":
	static_points_filtering_server()