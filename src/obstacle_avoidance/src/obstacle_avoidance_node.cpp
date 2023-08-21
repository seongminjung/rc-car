#include <sstream>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "ackermann_msgs/AckermannDrive.h"

#define IR_MAX 60
#define IR_MIN 20
#define IR_THRESHOLD 40
#define THROTTLE_FORWARD 500
#define THROTTLE_IDLE 0
#define SERVO_LEFT 7878
#define SERVO_CENTER 0
#define SERVO_RIGHT -7878

class ObstacleAvoidance
{
public:
    ObstacleAvoidance()
    {
        pub_ = n_.advertise<ackermann_msgs::AckermannDrive>("/rc_car/ackermann_cmd", 1);
        subs[0] = n_.subscribe("/rc_car/ir/left_front", 1, &ObstacleAvoidance::callback0, this);
        subs[1] = n_.subscribe("/rc_car/ir/front_left", 1, &ObstacleAvoidance::callback1, this);
        subs[2] = n_.subscribe("/rc_car/ir/front_right", 1, &ObstacleAvoidance::callback2, this);
        subs[3] = n_.subscribe("/rc_car/ir/right_front", 1, &ObstacleAvoidance::callback3, this);
    }

    void callback0(const sensor_msgs::LaserScan::ConstPtr &scan_in)
    {
        distance[0] = std::max(std::min(int(scan_in->ranges[0] * 100), (int)IR_MAX), (int)IR_MIN);
        update_state();
        do_action();
    }
    void callback1(const sensor_msgs::LaserScan::ConstPtr &scan_in)
    {
        distance[1] = std::max(std::min(int(scan_in->ranges[0] * 100), (int)IR_MAX), (int)IR_MIN);
    }
    void callback2(const sensor_msgs::LaserScan::ConstPtr &scan_in)
    {
        distance[2] = std::max(std::min(int(scan_in->ranges[0] * 100), (int)IR_MAX), (int)IR_MIN);
    }
    void callback3(const sensor_msgs::LaserScan::ConstPtr &scan_in)
    {
        distance[3] = std::max(std::min(int(scan_in->ranges[0] * 100), (int)IR_MAX), (int)IR_MIN);
    }
    void control_once(float throttle, int servo)
    {
        ackermann_msgs::AckermannDrive msg;
        msg.speed = throttle / 1000.0;
        msg.steering_angle = servo / 10000.0;
        pub_.publish(msg);
    }
    void update_state()
    {
        if (distance[1] == IR_MAX && distance[2] == IR_MAX)
            state = 0; // front open
        else
            state = 1; // front blocked
    }
    void do_action()
    {
        switch (state)
        {
        case 0:
        {
            // go straight
            float diff = distance[0] - distance[3];
            int corrected_servo = int(SERVO_CENTER + diff * 200);
            control_once(THROTTLE_FORWARD, corrected_servo);
            break;
        }
        case 1:
        {
            if (distance[0] == IR_MAX && distance[3] < IR_MAX) // left side open
            {
                for (int cnt = 0; cnt < 10000; cnt++)
                {
                    control_once(THROTTLE_FORWARD, SERVO_LEFT);
                }
            }
            else if (distance[0] < IR_MAX && distance[3] == IR_MAX) // right side open
            {
                for (int cnt = 0; cnt < 10000; cnt++)
                {
                    control_once(THROTTLE_FORWARD, SERVO_RIGHT);
                }
            }
            else
            // else if (distance[0] == IR_MAX && distance[3] == IR_MAX) // both side open
            {
                float diff = distance[1] - distance[2];
                if (abs(diff) > 1) // if one front IR is bigger than other by 3cm
                {
                    int corrected_servo = int(SERVO_CENTER + (1 / diff) * SERVO_LEFT * 1.5);
                    control_once(THROTTLE_FORWARD, corrected_servo);
                }
                else // two front IRs are similar
                {
                    control_once(THROTTLE_IDLE, SERVO_CENTER);
                }
            }
            // else // both side closed
            // {
            //     control_once(THROTTLE_IDLE, SERVO_CENTER);
            // }
            break;
        }
        }
    }

private: // private으로 NodeHandle과 publisher, subscriber를 선언한다.
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber subs[4];
    int state = 0;
    int distance[4];
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_avoidance_node");
    ObstacleAvoidance OA;
    ros::spin();
    return 0;
}
