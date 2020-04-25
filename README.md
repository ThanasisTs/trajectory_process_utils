# The trajectory point process ROS package

This package offers two services:
* <b>static_points_filtering</b>: Given three lists of cartesian coordinates, it removes points from the beginning and the end of the lists which could be considered redundant. For that purpose a median-based criterion is utilized. For the first N consecutive points, the distance between their coordinates and the coordinates of the median of the first N/2 points in each is computed and the points which correspond to distances lower than a predetermined value in at least one axis are removed. The respective procedure is followed for the end of the movement.

* <b>trajectory_smoothing</b>: Given three lists of cartesian coordinates which correspond to a recorded trajectory, it smooths the trajectory using Bezier Curves. The implementation of the Bezier curves is based on [this repo](https://github.com/Hrisi/Python---Spline-curves).

