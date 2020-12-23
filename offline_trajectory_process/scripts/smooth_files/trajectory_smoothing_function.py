#!/usr/bin/env python
import rospy
import smoother
from timeit import default_timer as timer

def main(x,y,z):

	x_raw,y_raw,z_raw = x[:],y[:],z[:]

	start = timer()
	x_smoothed,y_smoothed,z_smoothed = smoother.Bezier(x,y,z)
	end = timer()
	time_elapsed = end - start

	rospy.loginfo("Time Elapsed for Smoothing: %.4f seconds."%time_elapsed)
	

	return x_smoothed,y_smoothed,z_smoothed


if __name__ == '__main__':
	main()
