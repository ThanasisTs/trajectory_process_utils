#include <ur3_trajectory_process/ur3_trajectory_process.h>

extern bool check_robot_limits (const geometry_msgs::PointConstPtr candidate_point);
extern void calculateSTD (const trajectory_custom_msgs::PointStampedArray::ConstPtr points_array);
extern void compute_new_offset (const geometry_msgs::PointStamped::ConstPtr new_trajectory_point);


void halt_motion_callback(const std_msgs::Bool::ConstPtr halt_motion_msg){
	halt_motion = halt_motion_msg->data;
	halt_motion = false;
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
			
			// The trajectory point if valid if it doed not lead the robot
			// to self-collision or overextension
			if (check_robot_limits(candidate_point)){
				control_point->point.x = candidate_point->x;
				control_point->point.y = candidate_point->y;
				control_point->point.z = candidate_point->z;
				control_points_pub.publish(*control_point);
			}
		}
	}
	else{
		// Compute new offset if the human has halted the robot motion
		compute_new_offset(trajectory_point);
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "ur3_trajectory_process_node");
	ros::NodeHandle nh;

	// ee state topic
	nh.param("reactive_control_node/state_topic", ee_state_topic, std::string("ur3_cartesian_velocity_controller/ee_state"));

	// Self collision limits
	nh.param("ur3_trajectory_process_node/self_collision_limit", self_collision_limit, 0.0f);
	nh.param("ur3_trajectory_process_node/z_limit", z_limit, 0.0f);

	// Overextention limits
	nh.param("ur3_trajectory_process_node/overextension_limit", overextension_limit, 0.0f);

	// Number of points to consider for recalibration
	nh.param("ur3_trajectory_process_node/new_offset_points", new_offset_points, 25);

	// Control points publisher
	control_points_pub = nh.advertise<geometry_msgs::PointStamped>("/control_points_topic", 100);

	// ee state subscriber
	ros::Subscriber ee_state_sub = nh.subscribe(ee_state_topic, 100, ee_state_callback);
	
	// Trajectory points subscriber
	ros::Subscriber trajectory_points_sub = nh.subscribe("/trajectory_points", 100, trajectory_points_callback);
	
	// Halt motion (left arm) subscriber
	ros::Subscriber check_keypoints_placement_sub = nh.subscribe("/check_keypoints_placement_topic", 100, halt_motion_callback);

	ros::spin();
}