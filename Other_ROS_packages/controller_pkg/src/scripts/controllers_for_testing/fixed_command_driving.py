#!/usr/bin/env python3

'''
Controller --> to go round
Author: Arjun Pradeep
'''


import os
import argparse
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
import yaml
import sys


class CONTROLLER_FIXED_COMMAND(object):
    def __init__(self, unit_id, steering, vel):
        self.unit_id = unit_id

        self.topic_pub = "/tractor" + str(unit_id) + "/cmd_tractor" + str(unit_id)
        self.topic_sub = "/tractor" + str(unit_id) + "/odom"
        self.pub_rate = 5 # hz
        self.cmd_msg = Twist()
        self.steering_min = -25 * np.pi / 180
        self.steering_max = 25 * np.pi / 180

        self.collision_flag = False

        # Constant Values
        self.steering_bias = 0.0 # deg
        if unit_id == 1:
            self.steering_bias = 7.0 # deg
        else:
            self.steering_bias = 0.0

        self.cmd_msg.linear.x = vel
        steering_set_value = (steering + self.steering_bias) * np.pi/180
        self.cmd_msg.angular.z = np.clip(steering_set_value, self.steering_min, self.steering_max)


    def activate_controller(self):
        rospy.init_node(f"NODE_Activate_Controller_Fixed_Command_{self.unit_id}")
        rospy.logwarn(f"Tractor_{self.unit_id} controller --> Activated")

        rate = rospy.Rate(self.pub_rate)
        pub_scan = rospy.Publisher(self.topic_pub, Twist, queue_size=10)

        # rospy.Subscriber(self.topic_sub_collision, Bool, self.check_for_collision)

        while not rospy.is_shutdown():
            try:
                pub_scan.publish(self.cmd_msg)
                rate.sleep()
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")
            except rospy.ROSTimeMovedBackwardsException:
                rospy.logerr("ROS Time Backwards! Just ignore the exception!")

        rospy.logwarn(f"Tractor_{self.unit_id} controller --> Deactivated")


def main(args):
    controller = CONTROLLER_FIXED_COMMAND(unit_id=args.unit_id, steering=args.steering, vel=args.vel)
    controller.activate_controller()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="lidar node activation script")
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Tractor to be controlled', type=int)
    parser.add_argument('-steering', '--steering', help='Provide steering angle in degrees', default=0.0, type=float)
    parser.add_argument('-vel', '--vel', help='Provide velocity in m/s', default=0.0, type=float)
    args = parser.parse_args()
    main(args)
