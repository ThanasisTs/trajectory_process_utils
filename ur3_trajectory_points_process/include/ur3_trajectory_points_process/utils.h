#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_custom_msgs/PointStampedArray.h>

#include <vector>

ros::Publisher control_points_pub;

geometry_msgs::PoseStampedPtr robot_pose = boost::make_shared<geometry_msgs::PoseStamped>();
geometry_msgs::PointStampedPtr control_point = boost::make_shared<geometry_msgs::PointStamped>();
boost::shared_ptr<trajectory_custom_msgs::PointStampedArray> new_trajectory_points_array = boost::make_shared<trajectory_custom_msgs::PointStampedArray>();

float xOffset, yOffset, zOffset;
float self_collision_limit, z_limit, overextension_limit;
int new_offset_points;
std::vector<double> points_std;
