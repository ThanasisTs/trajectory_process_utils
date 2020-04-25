#!/usr/bin/env python
import python
import actionlib
import trajectory_points_process.msg


case = None

class BezierAction(object):
	_feedback = trajectory_points_process.msg.BezierFeedback()
	_result = trajectory_points_process.msg.BezierResult()

	"""docstring for BezierAction"""
	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, trajectory_points_process.msg.BezierAction,
		 execute_cb=self.execute_cb, auto_start=False)
		self._as.start()

	def execute_cb(self, goal):
		x = goal.x_raw
		y = goal.y_raw
		z = goal.z_raw
		global case

		x_1 ,y_1 , z_1 = trajectory_smoothing_function.main(x, y, z, case)
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
		if success:
			self._result.x_smooth = x_smooth
			self._result.y_smooth = y_smooth
			self._result.z_smooth = z_smooth
			rospy.loginfo("%s: Succeeded"%self.action_name)
			self._as.set_succeeded(self._result)



if __name__ == "__main__":
	rospy.init_node("bezier_action_server")
	case = rospy.get_param("bezier_action_server/case", 0)
	server = BezierAction(rospy.get_name())
	rospy.spin()
