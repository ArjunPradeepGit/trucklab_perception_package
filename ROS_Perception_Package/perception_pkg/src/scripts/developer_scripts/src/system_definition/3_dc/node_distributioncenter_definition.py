#!/usr/bin/env python3

'''
Script: DistributionCenter definition
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

class DISTRIBUTIONCENTER_DEFINITION(object):
    def __init__(self, config_file_path, dc_id):
        config_info = read_yaml(config_file_path)
        self.dc_id = dc_id
        self.config_info = config_info[f"DC_{self.dc_id}"]

    def define_actors(self):
        if self.config_info["NUMBER_OF_VEHICLES"] > 0:
            for i in range(self.config_info["NUMBER_OF_VEHICLES"]):
                vehicle_id = i+1
                vehicle_cmd = f'gnome-terminal --tab -- rosrun perception_pkg vehicle_activation.py -dc_id {self.dc_id} -unit_id {vehicle_id}'
                vehicle_process = subprocess.Popen(vehicle_cmd, stdout=subprocess.PIPE, stderr=None, shell=True)
                vehicle_process.wait(timeout=None)

        if self.config_info["NUMBER_OF_INFRASTRUCTURES"] > 0:
            for i in range(self.config_info["NUMBER_OF_INFRASTRUCTURES"]):
                infrastructure_id = i + 1
                infrastructure_cmd = f'gnome-terminal --tab -- rosrun perception_pkg infrastructure_activation.py -dc_id {self.dc_id} -unit_id {infrastructure_id}'
                infrastructure_process = subprocess.Popen(infrastructure_cmd, stdout=subprocess.PIPE, stderr=None, shell=True)
                infrastructure_process.wait(timeout=None)


def main(args):
    ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
    PATH_CONFIG_SYSDEF = Path(ROOT_DIR / "user_scripts" / "user_config" / "system_definition" / "dc_1_system_definition.yaml")

    dc = DISTRIBUTIONCENTER_DEFINITION(config_file_path=PATH_CONFIG_SYSDEF, dc_id=args.dc_id)
    dc.define_actors()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="node distribution center definition")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    args = parser.parse_args()
    main(args)



