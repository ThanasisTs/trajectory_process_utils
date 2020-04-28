#!/usr/bin/env python
import rospy
from trajectory_process_utils_srvs.srv import *
from statistics import median


num_median_points = None
thresX = None
thresY = None
thresZ = None

def handle_filtering(req):
	global num_median_points, thresX, thresY, thresZ
	x = req.x
	y = req.y
	z = req.z

	for i in xrange(num_median_points, len(x)-1):
		if abs(x[i] - median(x[0:i])) > thresX or abs(y[i] - median(y[0:i])) > thresY or abs(z[i] - median(z[0:i])) > thresZ:
			break

	for j in xrange(len(x)-num_median_points, 1, -1):
		if abs(x[j] - median(x[j:len(x)])) > thresX or abs(y[j] - median(y[j:len(y)])) > thresY:
			break

	x = x[i-3:j+3]
	y = y[i-3:j+3]
	z = z[i-3:j+3]
	return FilteringResponse(x, y, z)


def static_points_filtering_server():
	rospy.init_node("static_points_filtering_server")
	global num_median_points, thresX, thresY, thresZ
	num_median_points = rospy.get_param("/static_points_filtering_server/num_median_points", 12)
	thresX = rospy.get_param("/static_points_filtering_server/thresX", 0.012)
	thresY = rospy.get_param("/static_points_filtering_server/thresY", 0.012)
	thresZ = rospy.get_param("/static_points_filtering_server/thresZ", 0.012)
	s = rospy.Service('static_points_filtering', Filtering, handle_filtering)
	rospy.loginfo("Ready to remove redundant points")
	rospy.spin()



if __name__=="__main__":
	static_points_filtering_server()