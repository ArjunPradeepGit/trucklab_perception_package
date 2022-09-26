#!/usr/bin/env python3

'''
Script: Obstacle detection - Collision Alert activation - at Lidar level
Author: Arjun Pradeep
'''

import os
import yaml
import argparse
import subprocess
import time
import sys
from pathlib import Path


def main(args):
    default_cmd = 'gnome-terminal --tab -- '
    pkg_cmd = 'rosrun perception_pkg '

    def run_lidar_file_in_new_terminal(node_name, dc_id, unit_type, unit_id, lidar_id):
        file_name = default_cmd + pkg_cmd + node_name
        args = f' -dc_id {dc_id} -unit_type {unit_type} -unit_id {unit_id} -lidar_id {lidar_id}'
        cmd = default_cmd + file_name + args
        program = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=None, shell=True)
        program.wait(timeout=None)

    # Activating: Obstacle Detection - Collision Alert - LiDAR
    node_collision_alert = "node_collision_alert.py"
    run_lidar_file_in_new_terminal(node_name=node_collision_alert, dc_id=args.dc_id, unit_type=args.unit_type,
                                   unit_id=args.unit_id, lidar_id=args.lidar_id)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="node obstacle detection lidar activation")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_type', '--unit_type', help='Provide unit type, Vehicle-v or Infrastructure-i', type=str, default='v')
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle/Infrastructure', type=int, default=1)
    parser.add_argument('-lidar_id', '--lidar_id', help='Provide ID number of LiDAR unit', type=int, default=1)
    args = parser.parse_args()
    main(args)
