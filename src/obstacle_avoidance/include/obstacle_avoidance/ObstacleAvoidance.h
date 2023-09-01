#include <vector>

#include "ackermann_msgs/AckermannDrive.h"
#include "obstacle_avoidance/SplitAndMerge.h"
#include "obstacle_avoidance/Visualization.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/MarkerArray.h"

class ObstacleAvoidance {
 private:
  ros::NodeHandle n_;
  ros::Publisher ack_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher goal_pub_;
  ros::Subscriber sub_;
  std::vector<float> ir;
  int state = 0;
  // 0: straight
  // 1: wall parallel - adjust wall parallel
  // 2: two wall parallel - adjust wall parallel and distance
  // 3: else - find local goal & avoid front obstacle
  float target_speed = 0;
  float target_angle = 0;
  bool emergency_stop = false;
  bool direction_lock = false;
  int prev_turn = 0;
  SplitAndMerge split_and_merge;
  std::vector<std::vector<Point>> walls;

 public:
  ObstacleAvoidance();

  void ir_callback(const sensor_msgs::LaserScan::ConstPtr &scan_in);

  void control_once(int throttle, int servo);

  void update_state();

  void get_target_angle();

  void follow_wall_single();

  void follow_wall_double();

  void guide_to_empty_space();

  void get_target_speed();

  void follow_goal();
};
