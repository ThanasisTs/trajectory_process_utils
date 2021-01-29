# The offline trajectory process package

A ROS package for offline trajectory smoothing and post processing trajectories resulting from dynamic movements (e.g. dynamic movements)

## Description

* `movement_detection.py`: A node that detects, filters and sends for smoothing a dynamic movement.
  * movement detection: The detection of the movement onset is based on the standard deviation of the x, y and z coordinates. Using a sliding window of length N, if the standard deviation of at least one coordinate is greater than a predefined threshold, then we assume that the motion has started. The same criterion is used for the detection of the end of the motion. In the meantime, outliers are removed. Outliers are considered NaN values and values which correspond to erroneous measurements (points whose distance from the last valid point is greater than a predetermined threshold).
  * extra movement filtering: `static_points_filtering_server` is called to further clean the end of the movement.
  * movement smoothing: `trajectory_smoothing_server` is called to smooth the trajectory.

* `static_points_filtering_server.py`: A server for removing points from the end of a recorded trajectory. It accepts three vectors containing the x, y, z coordinates in the form of a service request and returns the corresponding cleaned trajectory in the form of a service response. The declaration of the services are [here](https://github.com/ThanasisTs/trajectory_process_utils/tree/master/trajectory_process_utils_srvs).

* `trajectory_smoothing_server.py`: A server for smoothing a trajectory using Bezier curves. It accepts three vectors containing the x, y, z coordinates in the form of a service request and returns the corresponding Bezier curve in the form of a service response. The declaration of the services are [here](https://github.com/ThanasisTs/trajectory_process_utils/tree/master/trajectory_process_utils_srvs).

In the following plot, orange points correspond to the raw final filtered trajectory and blue to the smoothed trajectory produced by the `trajectory_smoothing_server`

<img src="https://github.com/ThanasisTs/trajectory_process_utils/blob/master/offline_trajectory_process/raw_smooth.png" width="1000">
