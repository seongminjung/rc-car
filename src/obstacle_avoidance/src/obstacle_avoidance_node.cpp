#include <iostream>
#include <vector>

#include "ackermann_msgs/AckermannDrive.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define IR_MAX 60 + 14
#define IR_MIN 20 + 14
#define IR_THRESHOLD 40 + 14
#define THROTTLE_FORWARD 500
#define THROTTLE_IDLE 0
#define SERVO_LEFT 7878
#define SERVO_CENTER 0
#define SERVO_RIGHT -7878

class ObstacleAvoidance {
 public:
  ObstacleAvoidance() {
    pub_ = n_.advertise<ackermann_msgs::AckermannDrive>("/rc_car/ackermann_cmd",
                                                        1);
    sub_ = n_.subscribe("/rc_car/ir", 1, &ObstacleAvoidance::ir_callback, this);
  }

  void ir_callback(const sensor_msgs::LaserScan::ConstPtr &scan_in) {
    ir.clear();
    for (int i = 0; i < 9; i++) {
      ir.push_back(
          std::max(std::min(float(scan_in->ranges[i] * 100), (float)IR_MAX),
                   (float)IR_MIN) -
          14);
      //   std::printf("%.2f  ", ir[i]);
    }
    // std::printf("\n");
    update_state();
    find_local_goal();
    // do_action();
  }
  void control_once(float throttle, int servo) {
    ackermann_msgs::AckermannDrive msg;
    msg.speed = throttle / 1000.0;
    msg.steering_angle = servo / 10000.0;
    pub_.publish(msg);
  }
  void update_state() {
    if (ir[4] == IR_MAX && ir[6] == IR_MAX)
      state = 0;  // front open
    else
      state = 1;  // front blocked
  }
  void find_local_goal() {
    int max_distance = 0;
    int max_idx = 0;
    vector<int> max_group;
    for (int i = 1; i < 9; i++) {
      if () }
    // float angle = max_dist_i
  }
  // void do_action() {}

 private:  // private으로 NodeHandle과 publisher, subscriber를 선언한다.
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  int state = 0;
  int substate = 0;
  std::vector<float> ir;
  float local_goal;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_avoidance_node");
    ObstacleAvoidance OA;
    ros::spin();
    return 0;
}
