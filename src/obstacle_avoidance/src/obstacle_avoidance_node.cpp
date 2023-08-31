#include <vector>

#include "ackermann_msgs/AckermannDrive.h"
#include "obstacle_avoidance/SplitAndMerge.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/MarkerArray.h"

#define IR_MAX 70
#define IR_MIN 20
#define IR_OFFSET 7
#define THROTTLE_FORWARD 1000  // 1000 is converted to 1
#define THROTTLE_IDLE 0
#define SERVO_LEFT -800  // 1000 is converted to 1
#define SERVO_CENTER 0
#define SERVO_RIGHT 800  // 1000 is converted to 1

class ObstacleAvoidance {
 private:
  ros::NodeHandle n_;
  ros::Publisher ack_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher goal_pub_;
  ros::Subscriber sub_;
  std::vector<float> ir;
  float local_goal_speed = 0;
  float local_goal_angle = 0;
  bool emergency_stop = false;
  int prev_turn = 0;
  SplitAndMerge split_and_merge;

 public:
  ObstacleAvoidance() {
    ack_pub_ = n_.advertise<ackermann_msgs::AckermannDrive>("/rc_car/ackermann_cmd", 1);
    marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("/rc_car/visualization_marker", 1);
    goal_pub_ = n_.advertise<visualization_msgs::Marker>("/rc_car/local_goal", 1);
    sub_ = n_.subscribe("/rc_car/ir", 1, &ObstacleAvoidance::ir_callback, this);
  }

  void visualize_planes(std::vector<std::vector<Point>> lines) {
    visualization_msgs::MarkerArray marker_array;

    // remove all markers
    visualization_msgs::Marker deleteall_marker;
    deleteall_marker.ns = "plane";
    deleteall_marker.id = 1;
    deleteall_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(deleteall_marker);
    marker_pub_.publish(marker_array);

    marker_array.markers.clear();

    for (int i = 0; i < lines.size(); i++) {
      visualization_msgs::Marker marker;
      // Set the frame ID and timestamp.
      marker.header.frame_id = "base_link";
      marker.header.stamp = ros::Time::now();
      // Set the namespace and id for this marker. This serves to create a unique
      // ID Any marker sent with the same namespace and id will overwrite the old
      // one
      marker.ns = "plane";
      marker.id = i;
      // Set the marker type. Initially this is CUBE, and cycles between that and
      // SPHERE, ARROW, and CYLINDER
      marker.type = visualization_msgs::Marker::CUBE;
      // Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3
      // (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;
      // Set the pose of the marker.  This is a full 6DOF pose relative to the
      // frame/time specified in the header

      Point start = lines[i][0];
      Point end = lines[i][lines[i].size() - 1];
      float length = sqrt(pow(start.x - end.x, 2) + pow(start.y - end.y, 2));
      float angle = -1 * atan2(end.y - start.y, end.x - start.x);

      tf2::Quaternion q;
      q.setRPY(0, 0, angle);
      // q = q.normalize();

      marker.pose.position.x = (start.x + end.x) / 200.0;
      marker.pose.position.y = -1 * (start.y + end.y) / 200.0;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = length / 100.0;
      marker.scale.y = 0.05;
      marker.scale.z = 0.5;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration();

      marker_array.markers.push_back(marker);
    }
    marker_pub_.publish(marker_array);
  }

  void visualize_goal() {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "base_link";
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "goal";
    arrow.id = 0;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    tf2::Quaternion q;
    q.setRPY(0, 0, -1 * local_goal_angle * 3.141592 / 180);

    arrow.pose.position.x = 0.0;
    arrow.pose.position.y = 0.0;
    arrow.pose.position.z = 0.0;
    arrow.pose.orientation.x = q.x();
    arrow.pose.orientation.y = q.y();
    arrow.pose.orientation.z = q.z();
    arrow.pose.orientation.w = q.w();

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    arrow.scale.x = local_goal_speed;
    arrow.scale.y = 0.05;
    arrow.scale.z = 0.05;

    // Set the color -- be sure to set alpha to something non-zero!
    arrow.color.r = 1.0f;
    arrow.color.g = 0.0f;
    arrow.color.b = 0.0f;
    arrow.color.a = 1.0;

    arrow.lifetime = ros::Duration();

    goal_pub_.publish(arrow);
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

    std::vector<std::vector<Point>> result = split_and_merge.grabData(ir);
    visualize_planes(result);
    find_local_goal();
    adjust_wall_distance();
    adjust_wall_parallel();
    get_local_goal_speed();
    visualize_goal();
    follow_goal();
  }

  void control_once(int throttle, int servo) {
    ackermann_msgs::AckermannDrive msg;
    // convert to actual input value
    msg.speed = throttle / 1000.0;
    msg.steering_angle = servo / 1000.0;
    ack_pub_.publish(msg);
  }

  void find_local_goal() {
    if (ir[3] == IR_MAX && ir[4] == IR_MAX && ir[5] == IR_MAX) {
      // if front three irs are all max, go straight
      local_goal_angle = 0;
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
        } else if (groups[i].size() == max_group_size) {
          max_group_idx = ir[3] > ir[5] ? 0 : 1;
          if (abs(ir[3] - ir[5]) < 5) max_group_idx = prev_turn;
          prev_turn = max_group_idx;
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
    if ((ir[0] == IR_MAX) != (ir[8] == IR_MAX)) return;  // better to follow wall
    float diff = (ir[0] - ir[8]) * 0.5;
    local_goal_angle -= diff;
    std::printf("distance adjust amount: %.2f\n", diff);
  }

  void adjust_one_side_parallel(int first, int second, int third, int direction) {
    if (ir[first] == IR_MAX || ir[second] == IR_MAX || ir[third] == IR_MAX || ir[first] == IR_MIN ||
        ir[second] == IR_MIN || ir[third] == IR_MIN)
      return;  // cannot determine car-wall angle

    float a = ir[first], b1 = ir[second], b2 = ir[third];
    float c1, c2;
    float theta = 22.5 * 3.14159 / 180.0;

    // triangle between ir[0] and ir[1]
    c1 = sqrt(a * a + b1 * b1 - 2 * a * b1 * cos(theta));
    float alpha = acos((a * a + c1 * c1 - b1 * b1) / (2 * a * c1)) * 180.0 / 3.14159;

    // triangle between ir[0] and ir[2]
    c2 = sqrt(a * a + b2 * b2 - 2 * a * b2 * cos(theta * 2));
    float beta = acos((a * a + c2 * c2 - b2 * b2) / (2 * a * c2)) * 180.0 / 3.14159;

    if (abs(alpha - beta) > 20) return;  // not parallel

    float ave_angle = (alpha + beta) * 0.5;
    float diff_angle = direction * (ave_angle - 90) * 1.0;  // left wall: +, right wall: -
    local_goal_angle -= diff_angle;
    std::printf("parallel adjust amount: %.2f\n", diff_angle);
  }

  void adjust_wall_parallel() {
    adjust_one_side_parallel(0, 1, 2, 1);
    adjust_one_side_parallel(8, 7, 6, -1);
  }

  void get_local_goal_speed() {
    if (emergency_stop) {
      std::printf("emergency stop!\n");
      local_goal_speed = 0;
      return;
    }
    float angle_rad = local_goal_angle * 3.14159 / 180.0;
    float a = 1.0, b = 0.5;
    float r = (a * b) / sqrt(b * b * cos(angle_rad) * cos(angle_rad) + a * a * sin(angle_rad) * sin(angle_rad));
    local_goal_speed = r;
  }

  void follow_goal() {
    local_goal_angle = std::min(std::max(local_goal_angle, float(-90.0)), float(90.0));
    int throttle = int(local_goal_speed * THROTTLE_FORWARD);
    int servo = int(SERVO_LEFT * local_goal_angle / 90.0);
    std::printf("final_goal_speed: %.2f\n", local_goal_speed);
    std::printf("final_goal_angle: %.2f\n", local_goal_angle);
    control_once(throttle, servo);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "obstacle_avoidance_node");
  ObstacleAvoidance OA;
  ros::spin();
  return 0;
}
