#!/usr/bin/env python3

'''
Script: Collision Alert Flag - Sensor level
Author: Arjun Pradeep
'''


import argparse
import os
import rospy
import roslaunch
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from perception_pkg.msg import lidarmsg, ogmmsg
import numpy as np
import cv2
import matplotlib.pyplot as plt
import yaml
import sys
from pathlib import Path


def read_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)

class OGM_LIDAR_COLLISION_ALERT():
    def __init__(self, config_sys_def, config_obs_det, dc_id, unit_type, unit_id, lidar_id):
        config_info_sysdef = read_yaml(config_sys_def)
        self.dc_id = dc_id
        self.dc_namespace = config_info_sysdef[f"DC_{self.dc_id}"]["NAMESPACE"]
        self.unit_type = unit_type

        if self.unit_type == 'v':
            self.unit_id = unit_id
            self.unit_namespace = config_info_sysdef[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"]["NAMESPACE"]
            self.lidar_id = lidar_id
            self.lidar_namespace = config_info_sysdef[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"][f"LIDAR_{self.lidar_id}"]["NAMESPACE"]
        elif self.unit_type == 'i':
            self.unit_id = unit_id
            self.unit_namespace = config_info_sysdef[f"DC_{self.dc_id}"][f"INFRASTRUCTURE_{self.unit_id}"]["NAMESPACE"]
            self.lidar_id = lidar_id
            self.lidar_namespace = config_info_sysdef[f"DC_{self.dc_id}"][f"INFRASTRUCTURE_{self.unit_id}"][f"LIDAR_{self.lidar_id}"]["NAMESPACE"]
        else:
            raise ValueError("Received unacceptable value for 'unit_type'. Must be either 'v' or 'i'.")

        self.topic_sub = "/" + str(self.dc_namespace) + "/" + str(self.unit_namespace) + "/" + str(self.lidar_namespace) + "/ogm"
        config_info_obsdet = read_yaml(config_obs_det)
        self.pub_rate = config_info_obsdet["PUBLISH_RATE"]
        self.topic_pub = "/" + str(self.dc_namespace) + "/" + str(self.unit_namespace) + "/" + str(self.lidar_namespace) + "/collision_alert"
        self.collision_msg = False
        self.wait_until_clearance_dist = 0
        self.collision_alert_check_dist = config_info_obsdet["COLLISION_ALERT_CHECK_DISTANCE"]

        self.ogm_grid_size = config_info_obsdet["OGM_GRID_SIZE"]
        self.ogm_img_size = config_info_obsdet["OGM_IMG_SIZE"]
        self.ogm_grid_to_px = config_info_obsdet["OGM_GRID_TO_PX"]


    def check_for_collision_alert(self):
        rospy.init_node(f"NODE_Collision_Alert_{self.dc_namespace}_{self.unit_namespace}_{self.lidar_namespace}")
        rospy.logwarn(f"Collision Alert of: {self.dc_namespace}/{self.unit_namespace}/{self.lidar_namespace} --> Activated")
        rospy.Subscriber(self.topic_sub, ogmmsg, self.check_distance)

        rate = rospy.Rate(self.pub_rate)
        pub_scan = rospy.Publisher(self.topic_pub, Bool, queue_size=10)

        while not rospy.is_shutdown():
            try:
                pub_scan.publish(self.collision_msg)
                rate.sleep()
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")
            except rospy.ROSTimeMovedBackwardsException:
                rospy.logerr("ROS Time Backwards! Just ignore the exception!")

        rospy.logwarn(f"Collision Alert of: {self.dc_namespace}/{self.unit_namespace}/{self.lidar_namespace} --> Deactivated")

    def check_distance(self, msg):
        rows = msg.rows
        cols = msg.cols
        ogm = np.array(msg.data)
        ogm = ogm.reshape([rows, cols])
        delta_rows = np.ceil((self.collision_alert_check_dist + self.wait_until_clearance_dist)/self.ogm_grid_size[0])
        delta_cols = np.ceil((self.collision_alert_check_dist + self.wait_until_clearance_dist)/self.ogm_grid_size[1])
        ogm_check_region = ogm[int(np.ceil((rows/2)-delta_rows)):int(np.ceil((rows/2)+delta_rows)),
                           int(np.ceil((cols/2)-delta_cols)):int(np.ceil((cols/2)+delta_cols))]
        if np.any(ogm_check_region == 0):
            self.collision_msg = True
            self.wait_until_clearance_dist = self.collision_alert_check_dist * 0.2
        else:
            self.collision_msg = False
            self.wait_until_clearance_dist = 0.0

def main(args):
    ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
    PATH_CONFIG_SYSDEF = Path(ROOT_DIR / "user_scripts" / "user_config" / "system_definition" / "dc_1_system_definition.yaml")
    PATH_CONFIG_OBSDET = Path(ROOT_DIR / "user_scripts" / "user_config" / "features" / "obstacle_detection" / "obstacle_detection.yaml")

    ogm_lidar = OGM_LIDAR_COLLISION_ALERT(config_sys_def=PATH_CONFIG_SYSDEF, config_obs_det=PATH_CONFIG_OBSDET, dc_id=args.dc_id, unit_type=args.unit_type.lower(), unit_id=args.unit_id, lidar_id=args.lidar_id)
    ogm_lidar.check_for_collision_alert()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="node collision alert")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_type', '--unit_type', help='Provide unit type, Vehicle-v or Infrastructure-i', type=str, default='v')
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle/Infrastructure', type=int, default=1)
    parser.add_argument('-lidar_id', '--lidar_id', help='Provide ID number of LiDAR unit', type=int, default=1)
    args = parser.parse_args()
    main(args)
