#!/usr/bin/env python3

'''
Script: Node for LiDAR pre-processing
Author: Arjun Pradeep
'''


import argparse
import os
import rospy
import roslaunch
from sensor_msgs.msg import LaserScan
from perception_pkg.msg import lidarmsg
import numpy as np
import matplotlib.pyplot as plt
import yaml
from pathlib import Path
import sys


def read_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)

class LIDAR_PREPROCESSOR(object):
    def __init__(self, config_file_path, dc_id, unit_type, unit_id, lidar_id):
        config_info = read_yaml(config_file_path)
        self.dc_id = dc_id
        self.dc_namespace = config_info[f"DC_{self.dc_id}"]["NAMESPACE"]
        self.unit_type = unit_type

        if self.unit_type == 'v':
            self.unit_id = unit_id
            self.unit_namespace = config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"]["NAMESPACE"]
            self.lidar_id = lidar_id
            self.lidar_namespace = config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"][f"LIDAR_{self.lidar_id}"]["NAMESPACE"]
            self.config_info = config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"][f"LIDAR_{self.lidar_id}"]
        elif self.unit_type == 'i':
            self.unit_id = unit_id
            self.unit_namespace = config_info[f"DC_{self.dc_id}"][f"INFRASTRUCTURE_{self.unit_id}"]["NAMESPACE"]
            self.lidar_id = lidar_id
            self.lidar_namespace = config_info[f"DC_{self.dc_id}"][f"INFRASTRUCTURE_{self.unit_id}"][f"LIDAR_{self.lidar_id}"]["NAMESPACE"]
            self.config_info = config_info[f"DC_{self.dc_id}"][f"INFRASTRUCTURE_{self.unit_id}"][f"LIDAR_{self.lidar_id}"]
        else:
            raise ValueError("Received unacceptable value for 'unit_type'. Must be either 'v' or 'i'.")

        self.angle_min_view = self.config_info["ANGLE_MIN_VIEW"] * np.pi / 180
        self.angle_max_view = self.config_info["ANGLE_MAX_VIEW"] * np.pi / 180
        self.range_min = self.config_info["RANGE_MIN"]
        self.range_max = self.config_info["RANGE_MAX"]
        self.animate_rate = 1 / self.config_info["FEED_RATE"]
        self.topic_sub = self.config_info["TOPIC_SUB"]
        self.topic_pub = "/" + str(self.dc_namespace) + "/" + str(self.unit_namespace) + "/" + str(self.lidar_namespace) + "/lidarscan"
        self.pub_rate = self.config_info["PUBLISH_RATE"]
        self.lidar_data = []
        self.intensities = []
        self.theta = []
        self.lidar_msg = lidarmsg()
        self.run_once = True

    def listen_and_preprocess(self, msg):
        if self.run_once == True:
            self.angle_inc = msg.angle_increment
            self.angle_min = msg.angle_min
            self.angle_max = msg.angle_max
            self.time_inc = msg.time_increment
            self.range_min = msg.range_min
            self.range_max = msg.range_max
            self.ind_min = int((self.angle_min_view - self.angle_min) // self.angle_inc)
            self.ind_max = int((self.angle_max_view - self.angle_min) // self.angle_inc)
            self.theta = np.arange(self.angle_min_view, self.angle_min_view + (self.ind_max - self.ind_min)*self.angle_inc, self.angle_inc) - np.pi/2
            self.lidar_data = [float(j) if not np.isinf(float(j)) else np.nan for j in msg.ranges]
            self.lidar_data = np.array(self.lidar_data[self.ind_min:self.ind_max])
            self.intensities = np.array(msg.intensities[self.ind_min:self.ind_max])

            # If using Gazebo simulation, then to correct the array length
            if self.topic_sub == "/scan":
                self.lidar_data = self.lidar_data[:-1]
                self.intensities = self.intensities[:-1]
                self.theta = self.theta[:-1]

            if self.config_info["REMOVE_CABIN_LIDAR_POINTS"]:
                x = np.array(self.lidar_data * np.cos(self.theta))
                y = np.array(self.lidar_data * np.sin(self.theta))
                mask = np.logical_and(np.logical_and(x < 0.18, x > -0.18), np.logical_and(y < 0.05, y > -0.25))

                self.lidar_data[mask] = np.nan
                self.theta[mask] = np.nan
                self.intensities[mask] = np.nan

            self.run_once = False

        else:
            self.lidar_data = [float(j) if not np.isinf(float(j)) else np.nan for j in msg.ranges]
            self.lidar_data = np.array(self.lidar_data[self.ind_min:self.ind_max])
            self.intensities = np.array(msg.intensities[self.ind_min:self.ind_max])
            self.theta = np.arange(self.angle_min_view,
                                   self.angle_min_view + (self.ind_max - self.ind_min) * self.angle_inc,
                                   self.angle_inc) - np.pi / 2

            # If using Gazebo simulation, then to correct the array length
            if self.topic_sub == "/scan":
                self.lidar_data = self.lidar_data[:-1]
                self.theta = self.theta[:-1]

            if self.config_info["REMOVE_CABIN_LIDAR_POINTS"]:
                x = np.array(self.lidar_data * np.cos(self.theta))
                y = np.array(self.lidar_data * np.sin(self.theta))
                mask = np.logical_and(np.logical_and(x < 0.18, x > -0.18), np.logical_and(y < 0.05, y > -0.25))

                self.lidar_data[mask] = np.nan
                self.theta[mask] = np.nan
                self.intensities[mask] = np.nan

        self.lidar_msg.header = msg.header
        self.lidar_msg.angle_min_view = self.angle_min_view
        self.lidar_msg.angle_max_view = self.angle_max_view
        self.lidar_msg.angle_increment = self.angle_inc
        self.lidar_msg.ranges = self.lidar_data
        self.lidar_msg.angles = self.theta
        self.lidar_msg.intensities = self.intensities

    def activate_preprocessing(self):
        rospy.init_node(f"NODE_LiDAR_Activate_{self.dc_namespace}_{self.unit_namespace}_{self.lidar_namespace}")
        rospy.logwarn(f"{self.dc_namespace}/{self.unit_namespace}/{self.lidar_namespace} --> Activated")
        rospy.Subscriber(self.topic_sub, LaserScan, self.listen_and_preprocess)

        rate = rospy.Rate(self.pub_rate)
        pub_scan = rospy.Publisher(self.topic_pub, lidarmsg, queue_size=10)

        while not rospy.is_shutdown():
            try:
                pub_scan.publish(self.lidar_msg)
                rate.sleep()
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")
            except rospy.ROSTimeMovedBackwardsException:
                rospy.logerr("ROS Time Backwards! Just ignore the exception!")

        rospy.logwarn(f"{self.dc_namespace}/{self.unit_namespace}/{self.lidar_namespace} --> Deactivated")


def main(args):
    ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
    PATH_CONFIG_SYSDEF = Path(ROOT_DIR / "user_scripts" / "user_config" / "system_definition" / "dc_1_system_definition.yaml")

    lidar = LIDAR_PREPROCESSOR(config_file_path=PATH_CONFIG_SYSDEF, dc_id=args.dc_id, unit_type=args.unit_type, unit_id=args.unit_id, lidar_id=args.lidar_id)
    lidar.activate_preprocessing()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="node lidar preprocessor")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_type', '--unit_type', help='Provide unit type, Vehicle-v or Infrastructure-i', type=str, default="v")
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle/Infrastructure', type=int, default=1)
    parser.add_argument('-lidar_id', '--lidar_id', help='Provide ID number of LiDAR unit', type=int, default=1)
    args = parser.parse_args()
    main(args)
