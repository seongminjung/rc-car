#include "obstacle_avoidance/Visualization.h"

#include <vector>

#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/MarkerArray.h"

#define IR_BODY_GAP 10

void visualize_planes(std::vector<std::vector<Point>> walls, ros::Publisher marker_pub_) {
  visualization_msgs::MarkerArray marker_array;

  // remove all markers
  visualization_msgs::Marker deleteall_marker;
  deleteall_marker.ns = "plane";
  deleteall_marker.id = 1;
  deleteall_marker.action = visualization_msgs::Marker::DELETEALL;
  marker_array.markers.push_back(deleteall_marker);
  marker_pub_.publish(marker_array);

  marker_array.markers.clear();

  for (int i = 0; i < walls.size(); i++) {
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

    Point start = walls[i][0];
    Point end = walls[i][walls[i].size() - 1];
    float length = sqrt(pow(start.x - end.x, 2) + pow(start.y - end.y, 2));
    float angle = -1 * atan2(end.y - start.y, end.x - start.x);

    tf2::Quaternion q;
    q.setRPY(0, 0, angle);
    // q = q.normalize();

    marker.pose.position.x = (start.x + end.x) / 200.0 + IR_BODY_GAP / 100.0;
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
    marker.color.g = 1.0f * walls[i].size() / 6.0;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_array.markers.push_back(marker);
  }
  marker_pub_.publish(marker_array);
}

void visualize_goal(float local_goal_angle, float local_goal_speed, ros::Publisher goal_pub_) {
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
