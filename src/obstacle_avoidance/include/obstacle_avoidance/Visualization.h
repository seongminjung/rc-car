#include <vector>

#include "obstacle_avoidance/Point.h"
#include "ros/ros.h"

void visualize(std::vector<std::vector<Point>> walls, float target_angle, float target_speed,
               ros::Publisher marker_pub_, ros::Publisher goal_pub_);
void visualize_planes(std::vector<std::vector<Point>> walls, ros::Publisher marker_pub_);
void visualize_goal(float target_angle, float target_speed, ros::Publisher goal_pub_);