#include <ur3_trajectory_process/utils.h>


// Check if trajectory point violates the robot limits
bool check_robot_limits (const geometry_msgs::PointConstPtr candidate_point){
	// Self-collision checking
	if (sqrt(pow(candidate_point->x, 2) + pow(candidate_point->y, 2)) < self_collision_limit 
		and candidate_point->z < z_limit){
		ROS_WARN_STREAM("Control point leading to self collision. Waiting for valid control point");
		return false;
	}
	// Overextension checking
	else if (sqrt(pow(candidate_point->x, 2) + pow(candidate_point->y, 2) + 
		pow(candidate_point->z, 2)) > overextension_limit){
		ROS_WARN_STREAM("Control point leading to overextention Waiting for valid control point");
		return false;
	}
	else{
		return true;
	}
}

// std calculator
void calculateSTD (const trajectory_custom_msgs::PointStampedArray::ConstPtr points_array){
	float sumx=0, sumy=0, sumz=0;
	for (auto point : points_array->points){
		sumx += point.point.x;
		sumy += point.point.y;
		sumz += point.point.z;
	}

	float meanx = sumx/points_array->points.size();
	float meany = sumy/points_array->points.size();
	float meanz = sumz/points_array->points.size();

	double varx=0, vary=0, varz=0; 
	for (auto point : points_array->points){
		varx += pow(point.point.x-meanx, 2);
		vary += pow(point.point.y-meany, 2);
		varz += pow(point.point.z-meanz, 2);
	}

	points_std.push_back(sqrt(varx/(points_array->points.size()-1)));
	points_std.push_back(sqrt(vary/(points_array->points.size()-1)));
	points_std.push_back(sqrt(varz/(points_array->points.size()-1)));
}

// Compute new translation offset
void compute_new_offset (const geometry_msgs::PointStamped::ConstPtr new_trajectory_point){
	new_trajectory_points_array->points.push_back(*new_trajectory_point);
	if (new_trajectory_points_array->points.size() == new_offset_points){
		calculateSTD(new_trajectory_points_array);
		new_trajectory_points_array->points.erase(new_trajectory_points_array->points.begin());
		ROS_WARN("STD: %f, %f, %f", points_std[0], points_std[1], points_std[2]);
		if (points_std[0] < 0.01 and points_std[1] < 0.01 and points_std[2] < 0.01){
			ROS_WARN_STREAM("The human halted its motion. Ready to compute new offsets");
			xOffset = robot_pose->pose.position.x - new_trajectory_point->point.x;
			yOffset = robot_pose->pose.position.y - new_trajectory_point->point.y;
			zOffset = robot_pose->pose.position.z - new_trajectory_point->point.z;
			ROS_INFO("New Offsets: %f, %f, %f", xOffset, yOffset, zOffset);
			new_trajectory_points_array->points.clear();
		}
	}
}
