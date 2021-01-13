# The online trajectory process package
A ROS package for online trajectory processing

## Description
* `raw_points_process.py`: Detects the start of the motion. The detection of the movement is based on the standard deviation of the x, y and z coordinates. 
Using a sliding window of length N, is the standard deviation of at least one coordinate is greater than a predefined threshold, then we assume that the motion
has started. In the meantime, outliers are removed. Outliers are considered NaN values of values which correspond to erroneous measurements. Erroneous points
are supposed to be points whose distance from the last valid point is greater than a predetermined threshold.
* `piecewise_bezier_process.py`: The detection of the motion and the removal of NaN points and outliers is done just like in the `raw_points_process.py`.
Additionally, every four points a call to the [smoothing server](https://github.com/ThanasisTs/trajectory_process_utils/tree/master/offline_trajectory_process)
is done. This results in a piecewize Bezier smoothed traejectory.
* `downsampling_interpolation.py`: The detection of the motion and the removal of NaN points and outliers is done just like in the `raw_points_process.py`.
Additionally, when the distance between consecutive points is greater than a predetermined threshold, artificial points are interpolated across the line segment
connecting these two points. If the distance is smaller than another predefined threshold, then the second point is discarded. This procedure yields a trajectory
whose points are more equidistant than the raw one.
