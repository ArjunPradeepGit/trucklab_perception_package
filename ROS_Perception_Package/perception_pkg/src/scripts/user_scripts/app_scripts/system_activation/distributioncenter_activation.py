#!/usr/bin/env python3

'''
Script: DistributionCenter activation file
Author: Arjun Pradeep
'''

import os
import yaml
import argparse
import subprocess
import time
import sys


def main(args):

    # Activating optitrack system
    optitrack_cmd = r"gnome-terminal --tab -- roslaunch vrpn_client_ros sample.launch"
    optitrack_program = subprocess.Popen(optitrack_cmd, stdout=subprocess.PIPE, stderr=None, shell=True)
    optitrack_program.wait(timeout=None)
    time.sleep(5)

    # Activating distribution center
    default_cmd = 'gnome-terminal --tab -- '
    pkg_cmd = 'rosrun perception_pkg '

    def run_distributioncenter_file_in_new_terminal(node_name, dc_id):
        args = f' --dc_id {dc_id}'
        cmd = default_cmd + pkg_cmd + node_name + args
        program = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=None, shell=True)
        program.wait(timeout=None)

    # Activating: Defining Sensors
    node_define_sensors = "node_distributioncenter_definition.py"
    run_distributioncenter_file_in_new_terminal(node_name=node_define_sensors, dc_id=args.dc_id)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="distribution center activation")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    args = parser.parse_args()
    main(args)
