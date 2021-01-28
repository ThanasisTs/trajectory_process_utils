# The offline trajectory process package

A ROS package for offline trajectory processing

## Description
* `trajectory_smoothing_server.py`: A server for smoothing a trajectory using Bezier curves. It accepts three vectors containing the x, y, z coordinates in the form of a
service request and returns the corresponding Bezier curve in the form of a service response. The declaration of the services are [here]()
* `static_points_filtering_server.py`: A server for removing points from the end of a recorded movement which correspond to station. It accepts three vectors containing the x, y, z coordinates in the form of a
service request and returns the corresponding cleaned trajectory in the form of a service response. The declaration of the services are [here]()
* `movement_detection.py`: A script containing the complete pipeline for movement detection and movement preprocessing
  * movement detection: The detection of the movement is based on the standard deviation of the x, y and z coordinates. Using a sliding window of length N, is the
  standard deviation of at least one coordinate is greater than a predefined threshold, then we assume that the motion has started. The same criterion is used for
  the detection of the end of the motion. In the meantime, outliers are removed. Outliers are considered NaN values of values which correspond to erroneous measurements.
  Erroneous points are supposed to be points whose distance from the last valid point is greater than a predetermined threshold.
  * movement preprocessing: Once the movement has been obtained, a call to the `static_points_filtering_server` is made to get rid of redundant points at the end of the
  motion. After that a call to the `trajectory_smoothing_server` is made to smooth the trajectory.

In the following plot, blue points correspond to all the recorded points, orange to the final filtered trajectory before passing it to the `trajectory_smoothing_server`
red to the points indicating the start of the motion and green the points filtered by the `static_points_filtering_server`

<img src="https://github.com/ThanasisTs/trajectory_process_utils/blob/master/offline_trajectory_process/md_z_n.png" width="1000" height="600">

In the following plot, orange points correspond to the raw final filtered trajectory and blue to the smoothed trajectory produced by the `trajectory_smoothing_server`

<img src="https://github.com/ThanasisTs/trajectory_process_utils/blob/master/offline_trajectory_process/raw_smooth.png" width="1000">

In the following figure, a block diagram of the whole pipeline implemented in `movement_detection.py` is shown

<img src="https://github.com/ThanasisTs/trajectory_process_utils/blob/master/offline_trajectory_process/block_diagram.png" width="500">
