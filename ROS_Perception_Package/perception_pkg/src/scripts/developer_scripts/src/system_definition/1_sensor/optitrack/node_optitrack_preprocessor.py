#!/usr/bin/env python3

'''
Script: Node for Opti-track pre-processing
Author: Arjun Pradeep
'''


import argparse
import os
import rospy
import roslaunch
from sensor_msgs.msg import LaserScan
from perception_pkg.msg import lidarmsg
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
import matplotlib.pyplot as plt
import yaml
from pathlib import Path
from scipy.spatial.transform import Rotation as R
import sys


def read_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)

class OPTITRACK_PREPROCESSOR(object):
    def __init__(self, config_file_path, dc_id, unit_type, unit_id):
        config_info = read_yaml(config_file_path)
        self.dc_id = dc_id
        self.dc_namespace = config_info[f"DC_{self.dc_id}"]["NAMESPACE"]
        self.unit_type = unit_type
        self.unit_id = unit_id
        self.unit_namespace = config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"]["NAMESPACE"]
        self.unit_type_str = config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"]["TYPE"]
        self.config_info = config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"]

        if self.unit_type_str == "TRACTOR":
            self.unit_num = self.unit_namespace.split("_")[-1]
            self.topic_sub = f"/vrpn_client_node/tractor{self.unit_num}/pose"
        elif self.unit_type_str == "SEMITRAILER":
            self.unit_num = self.unit_namespace.split("_")[-1]
            self.topic_sub = f"/vrpn_client_node/trailer{self.unit_num}/pose"
        else:
            raise ValueError("Received unacceptable value for 'unit_type'. Must be 'v'.\n Please note that the Opti-track is not yet configured for infrastructure!")

        self.topic_pub = "/" + str(self.dc_namespace) + "/" + str(self.unit_namespace) + "/optitrack"
        self.pub_rate = self.config_info["OPTITRACK_PUBLISH_RATE"]
        self.optitrack_msg = PoseStamped()
        self.correction_longitudinal = self.config_info["OPTITRACK_CORRECTION_LONGITUDINAL_LENGTH"]
        self.correction_lateral = self.config_info["OPTITRACK_CORRECTION_LATERAL_LENGTH"]

    def listen_and_preprocess(self, msg):
        self.optitrack_msg = msg

        # Read values from Opti-Track
        self.X_global = msg.pose.position.z
        self.Y_global = msg.pose.position.x
        self.Z_global = msg.pose.position.y
        r = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.theta_Z_global, self.theta_X_global, self.theta_Y_global = r.as_euler('yzx', degrees=False)

        # Apply corrections to the raw opti-track values
        r_corrected = R.from_euler('zyx', [self.theta_Z_global, self.theta_Y_global, self.theta_X_global], degrees=False)
        self.quat_x_corrected, self.quat_y_corrected, self.quat_z_corrected, self.quat_w_corrected = r_corrected.as_quat()
        self.X_global_corrected = self.X_global + self.correction_longitudinal * np.cos(self.theta_Z_global) - self.correction_lateral * np.sin(self.theta_Z_global)
        self.Y_global_corrected = self.Y_global + self.correction_longitudinal * np.sin(self.theta_Z_global) + self.correction_lateral * np.cos(self.theta_Z_global)
        self.Z_global_corrected = self.Z_global

        # Update the corrected values
        self.optitrack_msg.pose.position.x = self.X_global_corrected
        self.optitrack_msg.pose.position.y = self.Y_global_corrected
        self.optitrack_msg.pose.position.z = self.Z_global_corrected
        self.optitrack_msg.pose.orientation.x = self.quat_x_corrected
        self.optitrack_msg.pose.orientation.y = self.quat_y_corrected
        self.optitrack_msg.pose.orientation.z = self.quat_z_corrected
        self.optitrack_msg.pose.orientation.w = self.quat_w_corrected

    def activate_preprocessing(self):
        rospy.init_node(f"NODE_Optitrack_Preprocessor_{self.dc_namespace}_{self.unit_namespace}")
        rospy.logwarn(f"{self.dc_namespace}/{self.unit_namespace}/Optitrack --> Activated")
        rospy.Subscriber(self.topic_sub, PoseStamped, self.listen_and_preprocess)

        rate = rospy.Rate(self.pub_rate)
        pub_scan = rospy.Publisher(self.topic_pub, PoseStamped, queue_size=10)

        while not rospy.is_shutdown():
            try:
                pub_scan.publish(self.optitrack_msg)
                rate.sleep()
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")

        rospy.logwarn(f"{self.dc_namespace}/{self.unit_namespace}/Optitrack --> Deactivated")


def main(args):
    ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
    PATH_CONFIG_SYSDEF = Path(ROOT_DIR / "user_scripts" / "user_config" / "system_definition" / "dc_1_system_definition.yaml")

    optitrack = OPTITRACK_PREPROCESSOR(config_file_path=PATH_CONFIG_SYSDEF, dc_id=args.dc_id, unit_type=args.unit_type, unit_id=args.unit_id)
    optitrack.activate_preprocessing()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="node optitrack preprocessor")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_type', '--unit_type', help='Provide unit type, Vehicle-v or Infrastructure-i', type=str, default="v")
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle/Infrastructure', type=int, default=1)
    args = parser.parse_args()
    main(args)
