#include <vector>

#include "obstacle_avoidance/Point.h"
#include "ros/ros.h"

void visualize_planes(std::vector<std::vector<Point>> walls, ros::Publisher marker_pub_);
void visualize_goal(float local_goal_angle, float local_goal_speed, ros::Publisher goal_pub_);