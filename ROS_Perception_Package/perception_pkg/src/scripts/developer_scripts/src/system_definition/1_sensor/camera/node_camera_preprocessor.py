#!/usr/bin/env python3

'''
Script: Camera preprocessor
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

class CAMERA_PREPROCESSOR(object):
    def __init__(self, config_file_path, dc_id, unit_type, unit_id, camera_id):
        config_info = read_yaml(config_file_path)
        self.dc_id = dc_id
        self.dc_namespace = config_info[f"DC_{self.dc_id}"]["NAMESPACE"]
        self.unit_type = unit_type

        if self.unit_type == 'v':
            self.unit_id = unit_id
            self.unit_namespace = config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"]["NAMESPACE"]
            self.camera_id = camera_id
            self.camera_namespace = config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"][f"CAMERA_{self.camera_id}"]["NAMESPACE"]
            self.config_info = config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"][f"CAMERA_{self.camera_id}"]
        elif self.unit_type == 'i':
            self.unit_id = unit_id
            self.unit_namespace = config_info[f"DC_{self.dc_id}"][f"INFRASTRUCTURE_{self.unit_id}"]["NAMESPACE"]
            self.camera_id = camera_id
            self.camera_namespace = config_info[f"DC_{self.dc_id}"][f"INFRASTRUCTURE_{self.unit_id}"][f"CAMERA_{self.camera_id}"]["NAMESPACE"]
            self.config_info = config_info[f"DC_{self.dc_id}"][f"INFRASTRUCTURE_{self.unit_id}"][f"CAMERA_{self.camera_id}"]
        else:
            raise ValueError("Received unacceptable value for 'unit_type'. Must be either 'v' or 'i'.")

        self.topic_sub = self.config_info["TOPIC_SUB"]
        self.topic_pub = "/" + str(self.dc_namespace) + "/" + str(self.unit_namespace) + "/" + str(self.camera_namespace) + "/cameraimage"
        self.pub_rate = self.config_info["PUBLISH_RATE"]
        self.camera_msg = CompressedImage
        self.image = None
        self.img_count = 1

    def listen_and_preprocess(self, msg):
        msg.format = "bgr8; jpeg compressed bgr8"   # For Matlab, part of TruckLab Learning Module
        self.camera_msg = msg
        self.img_count += 1
        self.image = []

    def activate_preprocessing(self):
        rospy.init_node(f"NODE_Camera_Activate_{self.dc_namespace}_{self.unit_namespace}_{self.camera_namespace}")
        rospy.logwarn(f"{self.dc_namespace}/{self.unit_namespace}/{self.camera_namespace} --> Activated")
        rospy.Subscriber(self.topic_sub, CompressedImage, self.listen_and_preprocess)

        rate = rospy.Rate(self.pub_rate)
        pub_scan = rospy.Publisher(self.topic_pub, CompressedImage, queue_size=10)

        while not rospy.is_shutdown():
            try:
                pub_scan.publish(self.camera_msg)
                rate.sleep()
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")
            except rospy.ROSTimeMovedBackwardsException:
                rospy.logerr("ROS Time Backwards! Just ignore the exception!")

        rospy.logwarn(f"{self.dc_namespace}/{self.unit_namespace}/{self.camera_namespace} --> Deactivated")


def main(args):
    ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
    PATH_CONFIG_SYSDEF = Path(ROOT_DIR / "user_scripts" / "user_config" / "system_definition" / "dc_1_system_definition.yaml")

    camera = CAMERA_PREPROCESSOR(config_file_path=PATH_CONFIG_SYSDEF, dc_id=args.dc_id, unit_type=args.unit_type, unit_id=args.unit_id, camera_id=args.camera_id)
    camera.activate_preprocessing()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="node camera preprocessor script")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_type', '--unit_type', help='Provide unit type, Vehicle-v or Infrastructure-i', type=str, default="v")
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle/Infrastructure', type=int, default=1)
    parser.add_argument('-camera_id', '--camera_id', help='Provide ID number of Camera unit', type=int, default=1)
    args = parser.parse_args()
    main(args)
