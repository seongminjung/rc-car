import rospy
from time import time

from typing import Optional, Union

from pynput import keyboard
from pynput.keyboard import Key, KeyCode
from geometry_msgs.msg import Twist


LINEAR_ACCEL = 1.3
LINEAR_DECEL = 1.8
ANGULAR_ACCEL = 2.5
ANGULAR_DECEL = 3.0

class KeyboardDriverNode():
    def __init__(self):
        self.ns = rospy.get_namespace()
        self.cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.forward = rospy.get_param(self.ns + 'forward') # e.g. /robot1/forward
        self.backward = rospy.get_param(self.ns + 'backward')
        self.left = rospy.get_param(self.ns + 'left')
        self.right = rospy.get_param(self.ns + 'right')

        self.des_vel = Twist()
        self.prev_msg = Twist()
        self.status = {
            "forward": 0,
            "backward": 0,
            "left": 0,
            "right": 0,
        }
        rospy.Timer(rospy.Duration(0.01), self.pub_twist)
        self.prev_time = time() - 0.01
        self.cur_time = time()

    def on_press(self, key: Optional[Union[Key, KeyCode]]):
        try:
            if isinstance(key, KeyCode):
                key = key.char
            elif isinstance(key, Key):
                key = key.name

            if key == self.forward:
                self.status["forward"] = 1
            if key == self.backward:
                self.status["backward"] = 1
            if key == self.left:
                self.status["left"] = 1
            if key == self.right:
                self.status["right"] = 1

            self.des_vel.linear.x = self.status["forward"] - self.status["backward"]
            self.des_vel.angular.z = self.status["left"] - self.status["right"]

            # Make sure this listener node is destroyed when the node is shutdown
            if rospy.is_shutdown():
                return False

        except rospy.ROSInterruptException:
            pass

    def on_release(self, key: Optional[Union[Key, KeyCode]]):
        try:
            if isinstance(key, KeyCode):
                key = key.char
            elif isinstance(key, Key):
                key = key.name

            if key == self.forward:
                self.status["forward"] = 0
            if key == self.backward:
                self.status["backward"] = 0
            if key == self.left:
                self.status["left"] = 0
            if key == self.right:
                self.status["right"] = 0

            self.des_vel.linear.x = self.status["forward"] - self.status["backward"]
            self.des_vel.angular.z = self.status["left"] - self.status["right"]

        except rospy.ROSInterruptException:
            pass

    def pub_twist(self, event=None):
        self.cur_time = time()
        dt = self.cur_time - self.prev_time
        dvx = self.des_vel.linear.x - self.prev_msg.linear.x
        dvz = self.des_vel.angular.z - self.prev_msg.angular.z

        msg = Twist()
        if dvx > LINEAR_ACCEL * dt:
            # only when the difference is greater than the acceleration, we will accelerate
            msg.linear.x = self.prev_msg.linear.x + LINEAR_ACCEL * dt
        elif dvx < -LINEAR_DECEL * dt:
            msg.linear.x = self.prev_msg.linear.x - LINEAR_DECEL * dt
        else:
            # otherwise, we will just set the velocity to the desired velocity directly
            msg.linear.x = self.des_vel.linear.x
            
        if dvz > ANGULAR_ACCEL * dt:
            msg.angular.z = self.prev_msg.angular.z + ANGULAR_ACCEL * dt
        elif dvz < -ANGULAR_DECEL * dt:
            msg.angular.z = self.prev_msg.angular.z - ANGULAR_DECEL * dt
        else:
            msg.angular.z = self.des_vel.angular.z

        self.cmd_vel_publisher.publish(msg)
        self.prev_time = self.cur_time
        self.prev_msg = msg


if __name__ == '__main__':
    rospy.init_node('keyboard_driver_node', anonymous=True)
    node = KeyboardDriverNode()
    while not rospy.is_shutdown():
        with keyboard.Listener(
            on_press=node.on_press, on_release=node.on_release
        ) as listener:
            listener.join()
