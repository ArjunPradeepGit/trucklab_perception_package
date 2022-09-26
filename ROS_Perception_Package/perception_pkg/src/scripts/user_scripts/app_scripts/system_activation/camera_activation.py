#!/usr/bin/env python3

'''
Script: Camera activation file
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
    PATH_CONFIG_SYSDEF = Path(ROOT_DIR / "user_scripts" / "user_config" / "system_definition" / "dc_1_system_definition.yaml")

    config_info = read_yaml(PATH_CONFIG_SYSDEF)
    if args.unit_type == 'v':
        config_info = config_info[f"DC_{args.dc_id}"][f"VEHICLE_{args.unit_id}"][f"CAMERA_{args.camera_id}"]
    elif args.unit_type == 'i':
        config_info = config_info[f"DC_{args.dc_id}"][f"INFRASTRUCTURE_{args.unit_id}"][f"CAMERA_{args.camera_id}"]

    default_cmd = 'gnome-terminal --tab -- '
    pkg_cmd = 'rosrun perception_pkg '

    def run_camera_file_in_new_terminal(node_name, dc_id, unit_type, unit_id, camera_id):
        file_name = default_cmd + pkg_cmd + node_name
        args = f' -dc_id {dc_id} -unit_type {unit_type} -unit_id {unit_id} -camera_id {camera_id}'
        cmd = file_name + args
        program = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=None, shell=True)
        program.wait(timeout=None)

    # Activating: Preprocessing Camera
    node_preprocessor = "node_camera_preprocessor.py"
    run_camera_file_in_new_terminal(node_name=node_preprocessor, dc_id=args.dc_id, unit_type=args.unit_type, unit_id=args.unit_id, camera_id=args.camera_id)

    # Activating: Livefeed of Camera
    if config_info["SHOW_FEED_WHILE_ACTIVATION"]:
        time.sleep(2)
        node_livefeed = "node_camera_livefeed.py"
        run_camera_file_in_new_terminal(node_name=node_livefeed, dc_id=args.dc_id, unit_type=args.unit_type, unit_id=args.unit_id, camera_id=args.camera_id)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="node camera activation")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_type', '--unit_type', help='Provide unit type, Vehicle-v or Infrastructure-i', type=str, default="v")
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle/Infrastructure', type=int, default=1)
    parser.add_argument('-camera_id', '--camera_id', help='Provide ID number of Camera unit', type=int, default=1)
    args = parser.parse_args()
    main(args)
