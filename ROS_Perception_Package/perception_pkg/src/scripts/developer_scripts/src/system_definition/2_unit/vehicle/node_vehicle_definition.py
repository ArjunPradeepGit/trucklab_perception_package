#!/usr/bin/env python3

'''
Script: Vehicle definition
Author: Arjun Pradeep
'''

import os
import argparse
import subprocess
import time
import numpy as np
import yaml
import sys
from pathlib import Path


def read_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)

class VEHICLE_DEFINITION(object):
    def __init__(self, config_file_path, dc_id, unit_id):
        config_info = read_yaml(config_file_path)
        self.dc_id = dc_id
        self.unit_type = 'v'
        self.unit_id = unit_id
        self.config_info_vehicle = config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"]

    def define_sensors(self):
        for i in range(self.config_info_vehicle["NUMBER_OF_LIDARS"]):
            lidar_id = i + 1
            lidar_cmd = f'gnome-terminal --tab -- rosrun perception_pkg lidar_activation.py -dc_id {self.dc_id} -unit_type {self.unit_type} -unit_id {self.unit_id} -lidar_id {lidar_id}'
            lidar_process = subprocess.Popen(lidar_cmd, stdout=subprocess.PIPE, stderr=None, shell=True)
            lidar_process.wait(timeout=None)

        for i in range(self.config_info_vehicle["NUMBER_OF_CAMERAS"]):
            camera_id = i + 1
            camera_cmd = f'gnome-terminal --tab -- rosrun perception_pkg camera_activation.py -dc_id {self.dc_id} -unit_type {self.unit_type} -unit_id {self.unit_id} -camera_id {camera_id}'
            camera_process = subprocess.Popen(camera_cmd, stdout=subprocess.PIPE, stderr=None, shell=True)
            camera_process.wait(timeout=None)

        odom_cmd = f'gnome-terminal --tab -- rosrun perception_pkg odometry_activation.py -dc_id {self.dc_id} -unit_type {self.unit_type} -unit_id {self.unit_id}'
        odom_process = subprocess.Popen(odom_cmd, stdout=subprocess.PIPE, stderr=None, shell=True)
        odom_process.wait(timeout=None)

        imu_cmd = f'gnome-terminal --tab -- rosrun perception_pkg imu_activation.py -dc_id {self.dc_id} -unit_type {self.unit_type} -unit_id {self.unit_id}'
        imu_process = subprocess.Popen(imu_cmd, stdout=subprocess.PIPE, stderr=None, shell=True)
        imu_process.wait(timeout=None)



def main(args):
    ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
    PATH_CONFIG_SYSDEF = Path(ROOT_DIR / "user_scripts" / "user_config" / "system_definition" / "dc_1_system_definition.yaml")

    vehicle = VEHICLE_DEFINITION(config_file_path=PATH_CONFIG_SYSDEF, dc_id=args.dc_id, unit_id=args.unit_id)
    vehicle.define_sensors()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="node vehicle definition")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle', type=int, default=1)
    args = parser.parse_args()
    main(args)



