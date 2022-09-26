#!/usr/bin/env python3

'''
Script: Node for Odometry pre-processing
Author: Arjun Pradeep
'''


import argparse
import os
import rospy
import roslaunch
from sensor_msgs.msg import LaserScan
from perception_pkg.msg import lidarmsg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
import matplotlib.pyplot as plt
import yaml
from pathlib import Path
from scipy.spatial.transform import Rotation as R
import sys


def read_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)

class ODOMETRY_PREPROCESSOR(object):
    def __init__(self, config_file_path, dc_id, unit_type, unit_id):
        config_info = read_yaml(config_file_path)
        self.dc_id = dc_id
        self.dc_namespace = config_info[f"DC_{self.dc_id}"]["NAMESPACE"]
        self.unit_type = unit_type
        self.unit_id = unit_id
        self.unit_namespace = config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"]["NAMESPACE"]
        self.unit_type_str = config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"]["TYPE"]
        self.config_info = config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"]

        if self.unit_type_str != "TRACTOR":
            raise ValueError("Received unacceptable value for 'unit_type'. Must be 'v'.\n Please note that the Odometry is only configured for vehicles!")

        self.topic_sub = self.config_info["ODOMETRY"]["TOPIC_SUB"]
        self.topic_pub = "/" + str(self.dc_namespace) + "/" + str(self.unit_namespace) + "/odometry"
        self.pub_rate = self.config_info["ODOMETRY"]["PUBLISH_RATE"]
        self.odom_msg = Odometry()

    def listen_and_preprocess(self, msg):
        self.odom_msg = msg

        ##########
        # Any pre-processing required could be added here
        ##########

    def activate_preprocessing(self):
        rospy.init_node(f"NODE_Odometry_Preprocessor_{self.dc_namespace}_{self.unit_namespace}")
        rospy.logwarn(f"{self.dc_namespace}/{self.unit_namespace}/odometry --> Activated")
        rospy.Subscriber(self.topic_sub, Odometry, self.listen_and_preprocess)

        rate = rospy.Rate(self.pub_rate)
        pub_scan = rospy.Publisher(self.topic_pub, Odometry, queue_size=10)

        while not rospy.is_shutdown():
            try:
                pub_scan.publish(self.odom_msg)
                rate.sleep()
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")

        rospy.logwarn(f"{self.dc_namespace}/{self.unit_namespace}/odometry --> Deactivated")


def main(args):
    ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
    PATH_CONFIG_SYSDEF = Path(ROOT_DIR / "user_scripts" / "user_config" / "system_definition" / "dc_1_system_definition.yaml")

    odometry = ODOMETRY_PREPROCESSOR(config_file_path=PATH_CONFIG_SYSDEF, dc_id=args.dc_id, unit_type=args.unit_type, unit_id=args.unit_id)
    odometry.activate_preprocessing()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="node odometry preprocessor")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_type', '--unit_type', help='Provide unit type, Vehicle-v or Infrastructure-i', type=str, default="v")
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle/Infrastructure', type=int, default=1)
    args = parser.parse_args()
    main(args)
