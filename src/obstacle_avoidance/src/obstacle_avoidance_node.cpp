#include <vector>

#include "ackermann_msgs/AckermannDrive.h"
#include "obstacle_avoidance/SplitAndMerge.h"
#include "obstacle_avoidance/Visualization.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/MarkerArray.h"

#define IR_MAX 70
#define IR_MIN 20
#define IR_OFFSET 7
#define THROTTLE_FORWARD 1000  // 1000 is converted to 1
#define THROTTLE_IDLE 0
#define SERVO_LEFT 800  // 1000 is converted to 1
#define SERVO_CENTER 0
#define SERVO_RIGHT -800  // 1000 is converted to 1

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
  // 1:  wall parallel - adjust wall parallel
  // 2: two wall parallel - adjust wall parallel and distance
  // 3: else - find local goal & avoid front obstacle
  float target_speed = 0;
  float target_angle = 0;
  bool emergency_stop = false;
  int prev_turn = 0;
  SplitAndMerge split_and_merge;
  std::vector<std::vector<Point>> walls;

 public:
  ObstacleAvoidance() {
    ack_pub_ = n_.advertise<ackermann_msgs::AckermannDrive>("/rc_car/ackermann_cmd", 1);
    marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("/rc_car/visualization_marker", 1);
    goal_pub_ = n_.advertise<visualization_msgs::Marker>("/rc_car/local_goal", 1);
    sub_ = n_.subscribe("/rc_car/ir", 1, &ObstacleAvoidance::ir_callback, this);
  }

  void ir_callback(const sensor_msgs::LaserScan::ConstPtr &scan_in) {
    ir.clear();
    for (int i = 0; i < 9; i++) {
      if (std::isinf(scan_in->ranges[i])) {
        // for Gazebo simulation
        ir.push_back((float)IR_MAX);
      } else {
        ir.push_back(std::max(std::min(float(scan_in->ranges[i] * 100 - IR_OFFSET), float(IR_MAX)), float(IR_MIN)));
      }
    }
    // if front three irs are less than 20, stop
    emergency_stop = ir[3] == IR_MIN && ir[4] == IR_MIN && ir[5] == IR_MIN;

    walls = split_and_merge.grabData(ir);
    update_state();
    get_target_angle();
    get_target_speed();
    // follow_goal();
    visualize(walls, target_angle, target_speed, marker_pub_, goal_pub_);
  }

  void control_once(int throttle, int servo) {
    ackermann_msgs::AckermannDrive msg;
    // convert to actual input value
    msg.speed = throttle / 1000.0;
    msg.steering_angle = servo / 1000.0;
    ack_pub_.publish(msg);
  }

  void update_state() {
    if (walls.size() == 0) {
      state = 0;
    } else if (walls.size() == 1) {
      // angle between wall and car
      float angle;
      if (walls[0][0].y > 0)
        angle =
            atan2(walls[0][walls[0].size() - 1].y - walls[0][0].y, walls[0][walls[0].size() - 1].x - walls[0][0].x) *
            180.0 / 3.14159;
      else
        angle =
            atan2(walls[0][0].y - walls[0][walls[0].size() - 1].y, walls[0][0].x - walls[0][walls[0].size() - 1].x) *
            180.0 / 3.14159;
      if (abs(angle) < 20) {
        state = 1;
      } else {
        state = 3;
      }
    } else if (walls.size() == 2) {
      // angle between wall and car
      float angle1 =
          atan2(walls[0][0].y - walls[0][walls[0].size() - 1].y, walls[0][0].x - walls[0][walls[0].size() - 1].x) *
          180.0 / 3.14159;
      float angle2 =
          atan2(walls[1][0].y - walls[1][walls[1].size() - 1].y, walls[1][0].x - walls[1][walls[1].size() - 1].x) *
          180.0 / 3.14159;
      if (abs(angle1) < 20 && abs(angle2) < 20) {
        state = 2;
      } else {
        state = 3;
      }
    } else {
      state = 3;
    }
    // std::printf("number of walls: %ld\t", walls.size());
    // std::printf("state: %d\n", state);
  }

  void get_target_angle() {
    target_angle = 0;
    switch (state) {
      case 0:
        target_angle = 0;
        break;
      case 1:
        adjust_parallel();
        break;
      case 2:
        adjust_wall_distance();
        break;
      case 3:
        guide_to_empty();
        break;
    }
  }

  void guide_to_empty() {
    if (ir[3] == IR_MAX && ir[4] == IR_MAX && ir[5] == IR_MAX) {
      // if front three irs are all max, go straight
      target_angle = 0;
      return;
    }

    float max_distance = 0;
    std::vector<int> max_group;

    // find the IR sensors with furthest distance
    for (int i = 0; i < 9; i++) {
      if (ir[i] > max_distance) {
        max_distance = ir[i];
        max_group.clear();
        max_group.push_back(i);
      } else if (ir[i] == max_distance) {
        max_group.push_back(i);
      }
    }
    // print the index of IR sensors with furthest distance
    // for (int i = 0; i < max_group.size(); i++) {
    //   std::printf("%d  ", max_group[i]);
    // }
    // std::printf("\n");

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
      // std::printf("number of groups: %ld\t", groups.size());

      // find the biggest group
      int max_group_size = 0;
      int max_group_idx = 0;
      for (int i = 0; i < groups.size(); i++) {
        if (groups[i].size() > max_group_size) {
          max_group_size = groups[i].size();
          max_group_idx = i;
        } else if (groups[i].size() == max_group_size) {
          max_group_idx = ir[3] > ir[5] ? 0 : 1;
          if (abs(ir[3] - ir[5]) < 5) max_group_idx = prev_turn;
          prev_turn = max_group_idx;
        }
      }
      // std::printf("biggest group index: %d\n", max_group_idx);

      // find the center of the biggest group
      int max_group_center = 0;
      for (int i = 0; i < groups[max_group_idx].size(); i++) {
        max_group_center += groups[max_group_idx][i];
      }
      max_group_center /= groups[max_group_idx].size();
      target_angle = -1 * (max_group_center - 4) * 22.5;

      // std::printf("target_angle: %.2f\n", target_angle);
    }
  }

  void adjust_wall_distance() {
    // compare ir[0] and ir[8], which are the distances from left and right
    // respectively, and adjust the servo motor proportional to the difference
    // between the two.
    if ((ir[0] == IR_MAX) != (ir[8] == IR_MAX)) return;  // better to follow wall
    float diff = (ir[0] - ir[8]) * 0.5;
    target_angle = diff;
    std::printf("distance adjust amount: %.2f\n", diff);
  }

  void adjust_parallel() {
    // angle between wall and car
    float angle;
    if (walls[0][0].y > 0)
      angle = atan2(walls[0][walls[0].size() - 1].y - walls[0][0].y, walls[0][walls[0].size() - 1].x - walls[0][0].x) *
              180.0 / 3.14159;
    else
      angle = atan2(walls[0][0].y - walls[0][walls[0].size() - 1].y, walls[0][0].x - walls[0][walls[0].size() - 1].x) *
              180.0 / 3.14159;
    target_angle = angle;
  }

  void get_target_speed() {
    if (emergency_stop) {
      target_angle = 0;
      target_speed = 0;
      return;
    }
    float angle_rad = target_angle * 3.14159 / 180.0;
    float a = 1.0, b = 0.5;
    float r = (a * b) / sqrt(b * b * cos(angle_rad) * cos(angle_rad) + a * a * sin(angle_rad) * sin(angle_rad));
    target_speed = r;
  }

  void follow_goal() {
    target_angle = std::min(std::max(target_angle, float(-90.0)), float(90.0));
    int throttle = int(target_speed * THROTTLE_FORWARD);
    int servo = int(SERVO_LEFT * target_angle / 90.0);
    // std::printf("final_goal_speed: %.2f\n", target_speed);
    // std::printf("final_goal_angle: %.2f\n", target_angle);
    control_once(throttle, servo);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "obstacle_avoidance_node");
  ObstacleAvoidance OA;
  ros::spin();
  return 0;
}
