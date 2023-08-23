#include <vector>

#include "ackermann_msgs/AckermannDrive.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define IR_MAX 80
#define IR_MIN 20
#define IR_OFFSET 14
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
          IR_OFFSET);
    }
    // if front three irs are less than 20, stop
    emergency_stop = ir[4] < IR_MIN && ir[5] < IR_MIN && ir[6] < IR_MIN;

    find_local_goal();
    adjust_wall_distance();
    follow_goal();
  }

  void control_once(float throttle, int servo) {
    ackermann_msgs::AckermannDrive msg;
    msg.speed = throttle / 1000.0;
    msg.steering_angle = servo / 10000.0;
    pub_.publish(msg);
  }

  void find_local_goal() {
    float max_distance = 0;
    std::vector<int> max_group;

    // find the IR sensors with furthest distance
    for (int i = 0; i < 10; i++) {
      if (ir[i] > max_distance) {
        max_distance = ir[i];
        max_group.clear();
        max_group.push_back(i);
      } else if (ir[i] == max_distance) {
        max_group.push_back(i);
      }
    }
    // print the index of IR sensors with furthest distance
    for (int i = 0; i < max_group.size(); i++) {
      std::printf("%d  ", max_group[i]);
    }
    std::printf("\n");

    if (max_group.size() > 0) {
      // group the consecutive numbers. e.g. (2,3), (5,6,7,8)
      std::vector<std::vector<int>> groups;
      std::vector<int> group;

      group.push_back(max_group[0]);

      for (int i = 1; i < max_group.size(); i++) {
        if (max_group[i] == group.back() + 1) {
          group.push_back(max_group[i]);
        } else {
          groups.push_back(group);
          group.clear();
          group.push_back(max_group[i]);
        }
      }
      groups.push_back(group);
      std::printf("number of groups: %ld\t", groups.size());

      // find the biggest group, and if two groups have the same size, find the
      // group with bigger average distance
      int max_group_size = 0;
      int max_group_idx = 0;
      float max_group_avg_distance = 0;

      for (int i = 0; i < groups.size(); i++) {
        if (groups[i].size() > max_group_size) {
          max_group_size = groups[i].size();
          max_group_idx = i;
        }
      }

      std::printf("biggest group index: %d\n", max_group_idx);

      // find the center of the biggest group
      int max_group_center = 0;
      for (int i = 0; i < groups[max_group_idx].size(); i++) {
        max_group_center += groups[max_group_idx][i];
      }
      max_group_center /= groups[max_group_idx].size();
      local_goal = (max_group_center - 5) * 22.5;

      std::printf("local_goal: %.2f\n", local_goal);
    }
  }

  void adjust_wall_distance() {
    // compare ir[0] and ir[8], which are the distances from left and right
    // respectively, and adjust the servo motor proportional to the difference
    // between the two.
    float diff = ir[0] - ir[8];
    local_goal -= int(SERVO_CENTER + diff * 10.0);
    std::printf("diff: %.2f\n", diff);
  }

  void follow_goal() {
    int angle =
        std::min(std::max(int(SERVO_CENTER - local_goal * SERVO_LEFT / 90.0),
                          SERVO_RIGHT),
                 SERVO_LEFT);
    if (emergency_stop) {
      control_once(THROTTLE_IDLE, SERVO_CENTER);
    } else {
      control_once(THROTTLE_FORWARD, angle);
    }
  }

 private:  // private으로 NodeHandle과 publisher, subscriber를 선언한다.
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  int state = 0;
  int substate = 0;
  std::vector<float> ir;
  float local_goal;
  bool emergency_stop = false;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "obstacle_avoidance_node");
  ObstacleAvoidance OA;
  ros::spin();
  return 0;
}
