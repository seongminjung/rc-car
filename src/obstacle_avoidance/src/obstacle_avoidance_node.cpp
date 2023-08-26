#include <vector>

#include "ackermann_msgs/AckermannDrive.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define IR_MAX 70
#define IR_MIN 20
#define IR_OFFSET 7
#define THROTTLE_FORWARD 1000  // 1000 is converted to 1
#define THROTTLE_IDLE 0
#define SERVO_LEFT -800  // 1000 is converted to 1
#define SERVO_CENTER 0
#define SERVO_RIGHT 800  // 1000 is converted to 1

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
      if (std::isinf(scan_in->ranges[i])) {
        // for Gazebo simulation
        ir.push_back((float)IR_MAX);
      } else {
        ir.push_back(
            std::max(std::min(float(scan_in->ranges[i] * 100), (float)IR_MAX),
                     (float)IR_MIN) -
            IR_OFFSET);
      }
    }
    // if front three irs are less than 20, stop
    emergency_stop = ir[4] < IR_MIN && ir[5] < IR_MIN && ir[6] < IR_MIN;

    find_local_goal();
    adjust_wall_distance();
    adjust_wall_parallel();
    get_local_goal_speed();
    follow_goal();
  }

  void control_once(int throttle, int servo) {
    ackermann_msgs::AckermannDrive msg;
    // convert to actual input value
    msg.speed = throttle / 1000.0;
    msg.steering_angle = servo / 1000.0;
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

      // find the biggest group
      int max_group_size = 0;
      int max_group_idx = 0;
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
      local_goal_angle = (max_group_center - 4) * 22.5;  // multiply -1

      std::printf("local_goal_angle: %.2f\n", local_goal_angle);
    }
  }

  void adjust_wall_distance() {
    // compare ir[0] and ir[8], which are the distances from left and right
    // respectively, and adjust the servo motor proportional to the difference
    // between the two.
    float diff = (ir[0] - ir[8]) * 0.5;
    local_goal_angle -= diff;
    std::printf("distance adjust amount: %.2f\n", diff);
  }

  void adjust_one_side_parallel(int first, int second, int third,
                                int direction) {
    if (ir[first] == IR_MAX || ir[second] == IR_MAX || ir[third] == IR_MAX ||
        ir[first] == IR_MIN || ir[second] == IR_MIN || ir[third] == IR_MIN)
      return;  // cannot determine car-wall angle

    float a = ir[first], b1 = ir[second], b2 = ir[third];
    float c1, c2;
    float theta = 22.5 * 3.14159 / 180.0;

    // triangle between ir[0] and ir[1]
    c1 = sqrt(a * a + b1 * b1 - 2 * a * b1 * cos(theta));
    float alpha =
        acos((a * a + c1 * c1 - b1 * b1) / (2 * a * c1)) * 180.0 / 3.14159;

    // triangle between ir[0] and ir[2]
    c2 = sqrt(a * a + b2 * b2 - 2 * a * b2 * cos(theta * 2));
    float beta =
        acos((a * a + c2 * c2 - b2 * b2) / (2 * a * c2)) * 180.0 / 3.14159;

    if (abs(alpha - beta) > 20) return;  // not parallel

    float ave_angle = (alpha + beta) * 0.5;
    float diff_angle =
        direction * (ave_angle - 90) * 0.5;  // left wall: +, right wall: -
    local_goal_angle -= diff_angle;
    std::printf("parallel adjust amount: %.2f\n", diff_angle);
  }

  void adjust_wall_parallel() {
    adjust_one_side_parallel(0, 1, 2, 1);
    adjust_one_side_parallel(8, 7, 6, -1);
  }

  void get_local_goal_speed() {
    float angle_rad = local_goal_angle * 3.14159 / 180.0;
    float a = 1.5, b = 0.5;
    float r = (a * b) / sqrt(b * b * cos(angle_rad) * cos(angle_rad) +
                             a * a * sin(angle_rad) * sin(angle_rad));
    local_goal_speed = r;
  }

  void follow_goal() {
    local_goal_angle =
        std::min(std::max(local_goal_angle, float(-90.0)), float(90.0));
    int throttle = int(local_goal_speed * THROTTLE_FORWARD);
    int servo = int(SERVO_LEFT * local_goal_angle / 90.0);
    std::printf("final_goal_speed: %.2f\n", local_goal_speed);
    std::printf("final_goal_angle: %.2f\n", local_goal_angle);
    if (emergency_stop) {
      control_once(THROTTLE_IDLE, SERVO_CENTER);
    } else {
      control_once(throttle, servo);
    }
  }

 private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  std::vector<float> ir;
  float local_goal_speed;
  float local_goal_angle;
  bool emergency_stop = false;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "obstacle_avoidance_node");
  ObstacleAvoidance OA;
  ros::spin();
  return 0;
}
