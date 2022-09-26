#!/usr/bin/env python3

'''
Script: Obstacle detection activation - at Lidar level
Author: Arjun Pradeep
'''

import os
import yaml
import argparse
import subprocess
import time
import sys
from pathlib import Path

def read_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)

def main(args):
    ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
    PATH_CONFIG_OBSDET = Path(ROOT_DIR / "user_scripts" / "user_config" / "features" / "obstacle_detection" / "obstacle_detection.yaml")
    config_info = read_yaml(PATH_CONFIG_OBSDET)

    default_cmd = 'gnome-terminal --tab -- '
    pkg_cmd = 'rosrun perception_pkg '

    def run_lidar_file_in_new_terminal(node_name, dc_id, unit_type, unit_id, lidar_id):
        file_name = default_cmd + pkg_cmd + node_name
        args = f' -dc_id {dc_id} -unit_type {unit_type} -unit_id {unit_id} -lidar_id {lidar_id}'
        cmd = default_cmd + file_name + args
        program = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=None, shell=True)
        program.wait(timeout=None)

    # Activating: Obstacle Detection LiDAR
    node_obstacledetection = "node_obstacledetection_definition.py"
    run_lidar_file_in_new_terminal(node_name=node_obstacledetection, dc_id=args.dc_id, unit_type=args.unit_type, unit_id=args.unit_id, lidar_id=args.lidar_id)

    # Activating: Livefeed of OGM
    if config_info["SHOW_PLOT"]:
        time.sleep(2)
        node_ogm_plot = "node_obstacledetection_plot.py"
        run_lidar_file_in_new_terminal(node_name=node_ogm_plot, dc_id=args.dc_id, unit_type=args.unit_type, unit_id=args.unit_id, lidar_id=args.lidar_id)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="node obstacle detection lidar activation")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_type', '--unit_type', help='Provide unit type, Vehicle-v or Infrastructure-i', type=str, default='v')
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle/Infrastructure', type=int, default=1)
    parser.add_argument('-lidar_id', '--lidar_id', help='Provide ID number of LiDAR unit', type=int, default=1)
    args = parser.parse_args()
    main(args)
