# The ur3 trajectory process package

A ROS package for converting a 3D cartesian trajectory to be executable by a UR3 Cobot

## Description:
* Computes an translsation offset using the first point of the trajectory and the end effector's current position to map the input trajectory to the robot's workspace.
* Checks if the trajectory points satisfy the robot's limits, namely if they lead the robot to overextension or self-collision. If yes, then these points are discarded.
* Online recalibration, namely recomputation of the translation offset, if asked by the operator.
