#!/usr/bin/env python3

'''
Script: Obstacle Detection - Sensor level
Author: Arjun Pradeep
'''


import argparse
import os
import rospy
import roslaunch
from sensor_msgs.msg import LaserScan
from perception_pkg.msg import lidarmsg, ogmmsg
import numpy as np
import matplotlib.pyplot as plt
import yaml
import sys
from pathlib import Path


def read_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)

class OGM_LIDAR():
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

        self.topic_sub = "/" + str(self.dc_namespace) + "/" + str(self.unit_namespace) + "/" + str(self.lidar_namespace) + "/lidarscan"
        self.topic_pub = "/" + str(self.dc_namespace) + "/" + str(self.unit_namespace) + "/" + str(self.lidar_namespace) + "/ogm"

        config_info_obsdet = read_yaml(config_obs_det)
        self.pub_rate = config_info_obsdet["PUBLISH_RATE"]

        self.lidar_x = []
        self.lidar_y = []

        self.ogm_grid_size = config_info_obsdet["OGM_GRID_SIZE"]
        self.ogm_img_size = config_info_obsdet["OGM_IMG_SIZE"]
        self.ogm_grid_to_px = config_info_obsdet["OGM_GRID_TO_PX"]

        self.ogm_px_to_xy = [self.ogm_grid_size[0] / self.ogm_grid_to_px[0], self.ogm_grid_size[1] / self.ogm_grid_to_px[1]]
        self.ogm_map_limits = [[0, self.ogm_img_size[0] * self.ogm_px_to_xy[0]], [0, self.ogm_img_size[1] * self.ogm_px_to_xy[1]]]

        self.ogm_total_cols = int((self.ogm_map_limits[0][1] - self.ogm_map_limits[0][0]) / self.ogm_grid_size[0])
        self.ogm_total_rows = int((self.ogm_map_limits[1][1] - self.ogm_map_limits[1][0]) / self.ogm_grid_size[1])
        self.ogm = np.ones((self.ogm_total_rows, self.ogm_total_cols), dtype=float) * (-1)
        self.ogm_img = np.ones((self.ogm_img_size[1], self.ogm_img_size[0]), np.uint8) * 255
        self.ogm_msg = ogmmsg()

    def update_ogm(self):
        rospy.init_node(f"NODE_OGM_{self.dc_namespace}_{self.unit_namespace}_{self.lidar_namespace}")
        rospy.logwarn(f"OGM generation of: {self.dc_namespace}/{self.unit_namespace}/{self.lidar_namespace} --> Activated")
        rospy.Subscriber(self.topic_sub, lidarmsg, self.listen_and_update)

        rate = rospy.Rate(self.pub_rate)
        pub_scan = rospy.Publisher(self.topic_pub, ogmmsg, queue_size=10)

        while not rospy.is_shutdown():
            try:
                pub_scan.publish(self.ogm_msg)
                rate.sleep()
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")
            except rospy.ROSTimeMovedBackwardsException:
                rospy.logerr("ROS Time Backwards! Just ignore the exception!")

        rospy.logwarn(f"OGM generation of: {self.dc_namespace}/{self.unit_namespace}/{self.lidar_namespace} --> Deactivated")


    def listen_and_update(self, msg):
        self.ogm = np.ones((self.ogm_total_rows, self.ogm_total_cols), dtype=float) * (-1)
        ranges = np.array(msg.ranges)
        angles = np.array(msg.angles)
        mask = np.logical_not(np.isnan(ranges))
        ranges = np.array(ranges[mask])
        angles = np.array(angles[mask])
        self.lidar_x = ranges * np.cos(angles) + ((self.ogm_map_limits[0][1] - self.ogm_map_limits[0][0]) / 2)
        self.lidar_y = (ranges * np.sin(angles))*(-1) + ((self.ogm_map_limits[1][1] - self.ogm_map_limits[1][0]) / 2)
        mask_fit_ogm = np.logical_and(np.logical_and(self.lidar_x > self.ogm_map_limits[0][0], self.lidar_x < self.ogm_map_limits[0][1]),
                                      np.logical_and(self.lidar_y > self.ogm_map_limits[1][0], self.lidar_y < self.ogm_map_limits[1][1]))
        self.lidar_x_masked = self.lidar_x[mask_fit_ogm]
        self.lidar_y_masked = self.lidar_y[mask_fit_ogm]
        cols = [int(np.ceil(value / self.ogm_grid_size[0])) for value in self.lidar_x_masked]
        rows = [int(np.ceil(value / self.ogm_grid_size[1])) for value in self.lidar_y_masked]
        for row, col in zip(rows, cols):
            if row < self.ogm_total_rows+1 and col < self.ogm_total_cols:
                self.ogm[row, col] = 0.0
        self.ogm_msg.rows = self.ogm_total_rows
        self.ogm_msg.cols = self.ogm_total_cols
        self.ogm_msg.data = self.ogm.flatten()


def main(args):
    ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
    PATH_CONFIG_SYSDEF = Path(ROOT_DIR / "user_scripts" / "user_config" / "system_definition" / "dc_1_system_definition.yaml")
    PATH_CONFIG_OBSDET = Path(ROOT_DIR / "user_scripts" / "user_config" / "features" / "obstacle_detection" / "obstacle_detection.yaml")

    ogm_lidar = OGM_LIDAR(config_sys_def=PATH_CONFIG_SYSDEF, config_obs_det=PATH_CONFIG_OBSDET, dc_id=args.dc_id, unit_type=args.unit_type.lower(), unit_id=args.unit_id, lidar_id=args.lidar_id)
    ogm_lidar.update_ogm()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="node obstacle detection lidar")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_type', '--unit_type', help='Provide unit type, Vehicle-v or Infrastructure-i', type=str, default='v')
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle/Infrastructure', type=int, default=1)
    parser.add_argument('-lidar_id', '--lidar_id', help='Provide ID number of LiDAR unit', type=int, default=1)
    args = parser.parse_args()
    main(args)
