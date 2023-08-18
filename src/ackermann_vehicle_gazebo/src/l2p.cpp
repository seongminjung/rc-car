#include <sstream>

#include "ros/ros.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"


class Laser2Point
{
public:
  Laser2Point()
  {
    pub_ = n_.advertise<sensor_msgs::PointCloud2>("/rc_car/pointcloud", 1);
    sub_ = n_.subscribe("/rc_car/laser/scan", 1, &Laser2Point::callback, this);
  }

  void callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud2 output;
    projector_.projectLaser(*scan_in, output);
    pub_.publish(output);
  }

private: //private으로 NodeHandle과 publisher, subscriber를 선언한다.
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  laser_geometry::LaserProjection projector_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "l2p");
  Laser2Point L2Pbject; //클래스 객체 선을 하게 되면 모든게 된다.
  ros::spin();
  return 0;
}
