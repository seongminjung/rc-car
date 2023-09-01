#include "obstacle_avoidance/ObstacleAvoidance.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "obstacle_avoidance_node");
  ObstacleAvoidance obstacle_avoidance;
  ros::spin();
  return 0;
}
