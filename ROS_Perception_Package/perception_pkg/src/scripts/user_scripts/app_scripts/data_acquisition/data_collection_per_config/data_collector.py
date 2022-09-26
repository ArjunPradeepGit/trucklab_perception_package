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

class DATACOLLECTOR():
    def __init__(self, path_datacollection_config, dc_id, path_dc_config):
        self.config_dc = read_yaml(path_dc_config)
        self.config_datacollection = read_yaml(path_datacollection_config)

        self.dc_id = dc_id
        self.dc_namespace = self.config_dc[f"DC_{self.dc_id}"]["NAMESPACE"]
        # self.unit_type = 'v'

        self.sub_rate = self.config_datacollection["meta_data"]["frequency"]
        self.bag_file_name = self.config_datacollection["meta_data"]["file_name"]

        self.topic_list_lidar = []
        self.topic_list_camera = []
        self.topic_list_optitrack = []

        # Topic list
        for vehicle_num in range(1, self.config_datacollection["meta_data"]["total_num_vehicles"]+1):
            if self.config_datacollection["data_to_be_collected"][f"vehicle_{vehicle_num}"]["type"] == "TRACTOR":
                try:
                    self.unit_namespace = self.config_datacollection["data_to_be_collected"][f"vehicle_{vehicle_num}"]["namespace"]
                    for lidar_num in range(1, self.config_datacollection["data_to_be_collected"][f"vehicle_{vehicle_num}"]["num_of_lidars"]+1):
                        try:
                            if self.config_datacollection["data_to_be_collected"][f"vehicle_{vehicle_num}"][f"lidar_{lidar_num}"]["record"]:
                                self.lidar_namespace = self.config_datacollection["data_to_be_collected"][f"vehicle_{vehicle_num}"][f"lidar_{lidar_num}"]["namespace"]
                                self.topic_list_lidar.append(f"/{self.dc_namespace}/{self.unit_namespace}/{self.lidar_namespace}/lidarscan")
                        except:
                            continue
                except:
                    continue

                try:
                    self.unit_namespace = self.config_datacollection["data_to_be_collected"][f"vehicle_{vehicle_num}"]["namespace"]
                    for camera_num in range(1, self.config_datacollection["data_to_be_collected"][f"vehicle_{vehicle_num}"]["num_of_cameras"]+1):
                        try:
                            if self.config_datacollection["data_to_be_collected"][f"vehicle_{vehicle_num}"][f"camera_{camera_num}"]["record"]:
                                self.camera_namespace = self.config_datacollection["data_to_be_collected"][f"vehicle_{vehicle_num}"][f"camera_{camera_num}"]["namespace"]
                                self.topic_list_camera.append(f"/{self.dc_namespace}/{self.unit_namespace}/{self.camera_namespace}/cameraimage")
                        except:
                            continue
                except:
                    continue

                try:
                    self.unit_namespace = self.config_datacollection["data_to_be_collected"][f"vehicle_{vehicle_num}"]["namespace"]
                    if self.config_datacollection["data_to_be_collected"][f"vehicle_{vehicle_num}"]["optitrack_record"]:
                        self.topic_list_optitrack.append(f"/{self.dc_namespace}/{self.unit_namespace}/optitrack")
                except:
                    continue

            elif self.config_datacollection["data_to_be_collected"][f"vehicle_{vehicle_num}"]["type"] == "SEMITRAILER":
                try:
                    self.unit_namespace = self.config_datacollection["data_to_be_collected"][f"vehicle_{vehicle_num}"]["namespace"]
                    if self.config_datacollection["data_to_be_collected"][f"vehicle_{vehicle_num}"]["optitrack_record"]:
                        self.topic_list_optitrack.append(f"/{self.dc_namespace}/{self.unit_namespace}/optitrack")
                except:
                    continue

        self.topic_list_full = np.concatenate([self.topic_list_lidar, self.topic_list_camera, self.topic_list_optitrack], axis=0)
        self.topic_list_full_str = ""
        for item in self.topic_list_full:
            self.topic_list_full_str += " " + str(item)


    def collect_data(self):
        default_cmd = 'gnome-terminal --tab -- '
        pkg_cmd = 'rosbag record '
        file_path = str(Path(self.config_datacollection["meta_data"]["folder_location"] / Path(self.config_datacollection["meta_data"]["file_name"]))) + ".bag"
        duration = self.config_datacollection["meta_data"]["max_duration"]

        def run_cmd_in_new_terminal():
            cmd = default_cmd + pkg_cmd + "-O " + file_path + " --duration=" + str(duration) + self.topic_list_full_str
            print(cmd)
            program = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=None, shell=True)
            program.wait(timeout=None)

        # Activating: Record data
        run_cmd_in_new_terminal()


def main(args):
    PATH_CONFIG_DC = Path(SCRIPTS_DIR / "user_scripts" / "user_config" / "system_definition" / f"dc_{args.dc_id}_system_definition.yaml")
    PATH_CONFIG_DATACOLLECTION = Path(SCRIPTS_DIR / "user_scripts" / "user_config" / "data_collection" / "datacollection.yaml")
    data_collector = DATACOLLECTOR(path_datacollection_config=PATH_CONFIG_DATACOLLECTION, dc_id=args.dc_id, path_dc_config=PATH_CONFIG_DC)
    data_collector.collect_data()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="data collector node - script")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center for data collection', type=int, default=1)
    args = parser.parse_args()
    main(args)