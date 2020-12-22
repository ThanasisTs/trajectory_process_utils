#include <ur3_trajectory_points_process/ur3_trajectory_points_process.h>
// Check if trajectory point is leading to overextension or self collision
void check_trajectory_point(const geometry_msgs::PointConstPtr candidate_point){
	// Self-collision checking
	if (sqrt(pow(candidate_point->x, 2) + pow(candidate_point->y, 2)) < self_collision_limit 
		and candidate_point->z < z_limit){
		ROS_WARN_STREAM("Control point leading to self collision. Waiting for valid control point");
	}
	// Overextension checking
	else if (sqrt(pow(candidate_point->x, 2) + pow(candidate_point->y, 2) + 
		pow(candidate_point->z, 2)) > overextension_limit){
		ROS_WARN_STREAM("Control point leading to overextention Waiting for valid control point");
	}
	else{
		// Publish the trajectory point if the robot is not in self-collision / overextension state
		// and the point does not lead to such a state
		control_point->point.x = candidate_point->x;
		control_point->point.y = candidate_point->y;
		control_point->point.z = candidate_point->z;
		control_points_pub.publish(*control_point);
		
	}
}

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


void halt_motion_callback(const std_msgs::Bool::ConstPtr halt_motion_msg){
	halt_motion = halt_motion_msg->data;
}

void ee_state_callback(const cartesian_state_msgs::PoseTwist::ConstPtr state_msg){
	robot_pose->pose.position.x = state_msg->pose.position.x;
	robot_pose->pose.position.y = state_msg->pose.position.y;
	robot_pose->pose.position.z = state_msg->pose.position.z;
	robot_pose->header.stamp = state_msg->header.stamp;
}

void trajectory_points_callback(const geometry_msgs::PointStamped::ConstPtr trajectory_point){
	if (not halt_motion){
		// If it is the first trajectory point, compute the translation offset
		if (init_point_flag){
			xOffset = robot_pose->pose.position.x - trajectory_point->point.x;
			yOffset = robot_pose->pose.position.y - trajectory_point->point.y;
			zOffset = robot_pose->pose.position.z - trajectory_point->point.z;
			ROS_INFO("First Offsets: %f, %f, %f", xOffset, yOffset, zOffset);
			init_point_flag = false;
		}
		else{
			candidate_point->x = trajectory_point->point.x + xOffset;
			candidate_point->y = trajectory_point->point.y + yOffset;
			candidate_point->z = trajectory_point->point.z + zOffset;
			
			check_trajectory_point(candidate_point);
		}
	}
	else{
		compute_new_offset(trajectory_point);
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "ur3_trajectory_points_process_node");
	ros::NodeHandle nh;

	nh.param("reactive_control_node/state_topic", ee_state_topic, std::string("ur3_cartesian_velocity_controller/ee_state"));

	// Self collision distances
	nh.param("ur3_trajectory_points_process_node/self_collision_limit", self_collision_limit, 0.0f);
	nh.param("ur3_trajectory_points_process_node/z_limit", z_limit, 0.0f);

	// Extention distance
	nh.param("ur3_trajectory_points_process_node/overextension_limit", overextension_limit, 0.0f);

	nh.param("ur3_trajectory_points_process_node/new_offset_points", new_offset_points, 25);

	control_points_pub = nh.advertise<geometry_msgs::PointStamped>("/control_points_topic", 100);

	ros::Subscriber ee_state_sub = nh.subscribe(ee_state_topic, 100, ee_state_callback);
	ros::Subscriber trajectory_points_sub = nh.subscribe("/trajectory_points", 100, trajectory_points_callback);
	ros::Subscriber check_keypoints_placement_sub = nh.subscribe("/check_keypoints_placement_topic", 100, halt_motion_callback);

	ros::spin();
}