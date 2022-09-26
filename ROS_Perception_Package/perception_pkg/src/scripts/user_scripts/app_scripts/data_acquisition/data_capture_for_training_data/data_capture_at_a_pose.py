#!/usr/bin/env python3

'''
Script: Data collector
Author: Arjun Pradeep
'''

from pathlib import Path
SCRIPTS_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")

import argparse
import os
import rospy
import roslaunch
import message_filters
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage
from perception_pkg.msg import lidarmsg, ogmmsg
import subprocess
import numpy as np
import cv2
import matplotlib.pyplot as plt
import yaml
import sys


def read_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)

class DATA_CAPTURER():
    def __init__(self, duration, filepath_sysconfig, folderpath_bags, dc_id, unit_type, unit_id, lidar_id, obj_name, cap_num):
        config_info_sysdef = read_yaml(filepath_sysconfig)
        self.duration = duration
        bag_name = "bag_" + obj_name + f"_{cap_num}.bag"
        self.bag_file_name = str(Path(folderpath_bags / bag_name))

        self.dc_id = dc_id
        self.dc_namespace = config_info_sysdef[f"DC_{self.dc_id}"]["NAMESPACE"]
        self.unit_type = unit_type

        if self.unit_type == 'v':
            self.unit_id = unit_id
            self.unit_namespace = config_info_sysdef[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"]["NAMESPACE"]
            self.lidar_id = lidar_id
            self.lidar_namespace = \
            config_info_sysdef[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"][f"LIDAR_{self.lidar_id}"]["NAMESPACE"]
        elif self.unit_type == 'i':
            self.unit_id = unit_id
            self.unit_namespace = config_info_sysdef[f"DC_{self.dc_id}"][f"INFRASTRUCTURE_{self.unit_id}"]["NAMESPACE"]
            self.lidar_id = lidar_id
            self.lidar_namespace = \
            config_info_sysdef[f"DC_{self.dc_id}"][f"INFRASTRUCTURE_{self.unit_id}"][f"LIDAR_{self.lidar_id}"][
                "NAMESPACE"]
        else:
            raise ValueError("Received unacceptable value for 'unit_type'. Must be either 'v' or 'i'.")

        self.topic_list_lidar = [f"/{self.dc_namespace}/{self.unit_namespace}/{self.lidar_namespace}/lidarscan"]
        self.topic_list_optitrack = [f"/{self.dc_namespace}/{self.unit_namespace}/optitrack"]

        self.topic_list_full = np.concatenate([self.topic_list_lidar, self.topic_list_optitrack], axis=0)
        self.topic_list_full_str = ""
        for item in self.topic_list_full:
            self.topic_list_full_str += " " + str(item)


    def capture(self):
        # Records the required data for capture_number 'cap_num' of object 'obj_name'
        default_cmd = 'gnome-terminal --tab -- '
        pkg_cmd = 'rosbag record '
        file_path = self.bag_file_name
        duration = self.duration

        def run_cmd_in_new_terminal():
            cmd = default_cmd + pkg_cmd + "-O " + file_path + " --duration=" + str(duration) + self.topic_list_full_str
            program = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=None, shell=True)
            program.wait(timeout=None)

        # Activating: Record data
        run_cmd_in_new_terminal()


def main(args):
    ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
    PATH_CONFIG_SYSDEF = Path(ROOT_DIR / "user_scripts" / "user_config" / "system_definition" / "dc_1_system_definition.yaml")
    PATH_FOLDER_BAGS = Path(ROOT_DIR / "data" / "data_captured_for_training_data" / "captured_data_bags")

    data_capturer = DATA_CAPTURER(duration=args.duration, filepath_sysconfig=PATH_CONFIG_SYSDEF, folderpath_bags=PATH_FOLDER_BAGS,
                                    dc_id=args.dc_id, unit_type=args.unit_type, unit_id=args.unit_id, lidar_id=args.lidar_id, cap_num=args.cap_num, obj_name=args.obj_name)
    data_capturer.capture()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="LiDAR capture")
    parser.add_argument('-duration', '--duration', help='Provide the duration of capture in seconds', type=float, default=10)
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_type', '--unit_type', help='Provide unit type, Vehicle-v or Infrastructure-i', type=str, default="v")
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle/Infrastructure', type=int, default=1)
    parser.add_argument('-lidar_id', '--lidar_id', help='Provide ID number of LiDAR', type=int, default=1)
    parser.add_argument('-obj_name', '--obj_name', help='Provide the object name for the capture, please given the exact name as "boundary" for the boundary', type=str)
    parser.add_argument('-cap_num', '--cap_num', help='Provide the capture number', type=int, default=1)
    args = parser.parse_args()
    main(args)
