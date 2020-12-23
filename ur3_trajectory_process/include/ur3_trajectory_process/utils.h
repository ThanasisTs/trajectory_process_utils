#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_custom_msgs/PointStampedArray.h>

#include <vector>

extern ros::Publisher control_points_pub;

extern geometry_msgs::PoseStampedPtr robot_pose;
extern geometry_msgs::PointStampedPtr control_point;
extern boost::shared_ptr<trajectory_custom_msgs::PointStampedArray> new_trajectory_points_array;

extern float xOffset, yOffset, zOffset;
extern float self_collision_limit, z_limit, overextension_limit;
extern int new_offset_points;
std::vector<double> points_std;
