#!/usr/bin/env python3

'''
Script: Camera livefeed file
Author: Arjun Pradeep
'''

import os
import argparse
import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import matplotlib.pyplot as plt
import yaml
from matplotlib.animation import FuncAnimation
import sys
from pathlib import Path


def read_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)


class CAMERA_LIVEFEED(object):
    def __init__(self, config_file_path, dc_id, unit_type, unit_id, camera_id):
        config_info = read_yaml(config_file_path)
        self.dc_id = dc_id
        self.dc_namespace = config_info[f"DC_{self.dc_id}"]["NAMESPACE"]
        self.unit_type = unit_type

        if self.unit_type == 'v':
            self.unit_id = unit_id
            self.unit_namespace = config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"]["NAMESPACE"]
            self.camera_id = camera_id
            self.camera_namespace = \
            config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"][f"CAMERA_{self.camera_id}"]["NAMESPACE"]
            self.config_info = config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"][f"CAMERA_{self.camera_id}"]
        elif self.unit_type == 'i':
            self.unit_id = unit_id
            self.unit_namespace = config_info[f"DC_{self.dc_id}"][f"INFRASTRUCTURE_{self.unit_id}"]["NAMESPACE"]
            self.camera_id = camera_id
            self.camera_namespace = \
            config_info[f"DC_{self.dc_id}"][f"INFRASTRUCTURE_{self.unit_id}"][f"CAMERA_{self.camera_id}"]["NAMESPACE"]
            self.config_info = config_info[f"DC_{self.dc_id}"][f"INFRASTRUCTURE_{self.unit_id}"][
                f"CAMERA_{self.camera_id}"]
        else:
            raise ValueError("Received unacceptable value for 'unit_type'. Must be either 'v' or 'i'.")

        self.topic_sub = "/" + str(self.dc_namespace) + "/" + str(self.unit_namespace) + "/" + str(self.camera_namespace) + "/cameraimage"
        self.camera_msg = CompressedImage
        self.image = None
        self.img_count = 1

    def show_image(self, msg):
        img_arr = np.fromstring(msg.data, np.uint8)
        self.image = cv2.imdecode(img_arr, 1)
        cv2.imshow(f"Video capture of {self.dc_namespace}_{self.unit_namespace}_{self.camera_namespace}", self.image)
        cv2.waitKey(10)

    def live_feed(self):
        rospy.init_node(f"NODE_LiveFeed_{self.dc_namespace}_{self.unit_namespace}_{self.camera_namespace}")
        rospy.logwarn(f"Generating live feed of: {self.dc_namespace}/{self.unit_namespace}/{self.camera_namespace}...")
        rospy.Subscriber(self.topic_sub, CompressedImage, self.show_image)
        rospy.spin()
        rospy.logwarn(f"Closing live feed of : {self.dc_namespace}_{self.unit_namespace}_{self.camera_namespace}...")


def main(args):
    ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
    PATH_CONFIG_SYSDEF = Path(ROOT_DIR / "user_scripts" / "user_config" / "system_definition" / "dc_1_system_definition.yaml")

    camera = CAMERA_LIVEFEED(config_file_path=PATH_CONFIG_SYSDEF, dc_id=args.dc_id, unit_type=args.unit_type, unit_id=args.unit_id, camera_id=args.camera_id)
    camera.live_feed()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="node camera livefeed")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_type', '--unit_type', help='Provide unit type, Vehicle-v or Infrastructure-i', type=str, default="v")
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle/Infrastructure', type=int, default=1)
    parser.add_argument('-camera_id', '--camera_id', help='Provide ID number of Camera unit', type=int, default=1)
    args = parser.parse_args()
    main(args)
