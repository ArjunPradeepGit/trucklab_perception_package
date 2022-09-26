#!/usr/bin/env python3

'''
Script: Vehicle activation
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

    def run_file_in_new_terminal(node_name, dc_id, unit_id):
        file_name = default_cmd + pkg_cmd + node_name
        args = f' --dc_id {dc_id} --unit_id {unit_id}'
        cmd = file_name + args
        program = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=None, shell=True)
        program.wait(timeout=None)

    # Activating: Defining Sensors
    node_define_sensors = "node_vehicle_definition.py"
    run_file_in_new_terminal(node_name=node_define_sensors, dc_id=args.dc_id, unit_id=args.unit_id)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="node vehicle activation")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle', type=int, default=1)
    args = parser.parse_args()
    main(args)
