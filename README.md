# The trajectory point process ROS package

This package offers two services:
* <b>static_points_filtering</b>: This service removes static points from the beginning and the end of lists. The distance of each point is compared to the median of the previous points, starting from point <b>num_median_points</b>.  When a distance (in any axis) is greater than <b>traj{}</b> that marks the end of static points. The three precedent points (three last static points) in each case are kept while the rest are discarded. This routine is applied twice at the beginning of the list (and moving forward), and at the end of the list (and moving backwards).


* <b>trajectory_smoothing</b>: Given three lists of cartesian coordinates which correspond to a recorded trajectory, it smooths the trajectory using Bezier Curves. The implementation of the Bezier curves is based on [this repo](https://github.com/Hrisi/Python---Spline-curves).

In the first figure the the raw points and the filtered ones of a recorded trajectory are shown, while in the second one the filtered points and the corresponding smooth trajectory are presented. In the third figure a block diagram of the framework is shown.

<img src="https://github.com/ThanasisTs/trajectory_process_utils/blob/master/md_z_n.png" width="1000" height="600">
<img src="https://github.com/ThanasisTs/trajectory_process_utils/blob/master/raw_smooth.png" width="1000">
<img src="https://github.com/ThanasisTs/trajectory_process_utils/blob/master/block_diagram.png" width="500">
