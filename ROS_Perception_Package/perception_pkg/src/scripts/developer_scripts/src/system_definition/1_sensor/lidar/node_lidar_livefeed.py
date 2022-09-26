#!/usr/bin/env python3

'''
Script: Node for LiDAR live-feed
Author: Arjun Pradeep
'''


import argparse
import os
import rospy
import roslaunch
from sensor_msgs.msg import LaserScan
from perception_pkg.msg import lidarmsg
import numpy as np
import matplotlib.pyplot as plt
import time
import yaml
from matplotlib.animation import FuncAnimation
from matplotlib.ticker import MultipleLocator
from pathlib import Path
import sys
import math


def read_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)

class LIDAR_LIVEFEED(object):
    def __init__(self, config_file_path, dc_id, unit_type, unit_id, lidar_id):
        config_info = read_yaml(config_file_path)
        self.dc_id = dc_id
        self.dc_namespace = config_info[f"DC_{self.dc_id}"]["NAMESPACE"]
        self.unit_type = unit_type

        if self.unit_type == 'v':
            self.unit_id = unit_id
            self.unit_namespace = config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"]["NAMESPACE"]
            self.lidar_id = lidar_id
            self.lidar_namespace = config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"][f"LIDAR_{self.lidar_id}"]["NAMESPACE"]
            self.config_info = config_info[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"][f"LIDAR_{self.lidar_id}"]
        elif self.unit_type == 'i':
            self.unit_id = unit_id
            self.unit_namespace = config_info[f"DC_{self.dc_id}"][f"INFRASTRUCTURE_{self.unit_id}"]["NAMESPACE"]
            self.lidar_id = lidar_id
            self.lidar_namespace = config_info[f"DC_{self.dc_id}"][f"INFRASTRUCTURE_{self.unit_id}"][f"LIDAR_{self.lidar_id}"]["NAMESPACE"]
            self.config_info = config_info[f"DC_{self.dc_id}"][f"INFRASTRUCTURE_{self.unit_id}"][f"LIDAR_{self.lidar_id}"]
        else:
            raise ValueError("Received unacceptable value for 'unit_type'. Must be either 'v' or 'i'.")

        self.angle_min_view = self.config_info["ANGLE_MIN_VIEW"] * np.pi / 180
        self.angle_max_view = self.config_info["ANGLE_MAX_VIEW"] * np.pi / 180
        self.range_min = self.config_info["RANGE_MIN"]
        self.range_max = self.config_info["RANGE_MAX"]
        self.animate_rate = 1 / self.config_info["FEED_RATE"]
        self.topic_sub = "/" + str(self.dc_namespace) + "/" + str(self.unit_namespace) + "/" + str(self.lidar_namespace) + "/lidarscan"
        self.lidar_data = []
        self.theta = []
        self.lidar_msg = lidarmsg()
        self.run_once = True
        self.fig, self.ax = plt.subplots()

    def listen_to_plot(self, msg):
        mask = np.logical_not(msg.ranges == np.nan)
        self.plot_lidar_data = np.array(msg.ranges)[mask]
        self.plot_theta = np.array(msg.angles)[mask]
        self.plot_angle_max_view = msg.angle_max_view
        self.plot_angle_min_view = msg.angle_min_view
        self.plot_range_max = 3.5

    def plot(self, i):
        if self.run_once:
            time.sleep(5)
            self.run_once = False

        x = self.plot_lidar_data * np.cos(self.plot_theta)
        y = self.plot_lidar_data * np.sin(self.plot_theta)
        self.ax.clear()
        self.ax.scatter(x, y, c='r', s=1, label='LiDAR detections')
        self.ax.plot(0.0, 0.0, 'bo', markersize=5, label='LiDAR location')
        self.ax.plot([self.plot_range_max*np.cos(self.plot_angle_max_view - np.pi/2), 0, self.plot_range_max*np.cos(self.plot_angle_min_view - np.pi/2)],
                     [self.plot_range_max*np.sin(self.plot_angle_max_view - np.pi/2), 0, self.plot_range_max*np.sin(self.plot_angle_min_view - np.pi/2)],
                     linewidth=1, linestyle='dashed')
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_title(f'LIVE LiDAR PLOT of {self.dc_namespace}/{self.unit_namespace}/{self.lidar_namespace}', fontsize=12)
        self.ax.set_xlabel('X - axis (m)', fontsize=10)
        self.ax.set_ylabel('Y - axis (m)', fontsize=10)
        self.ax.legend(loc='lower left')
        self.ax.set_aspect(1.0)
        self.ax.xaxis.set_major_locator(MultipleLocator(1))
        self.ax.yaxis.set_major_locator(MultipleLocator(1))
        self.ax.xaxis.set_minor_locator(MultipleLocator(0.2))
        self.ax.yaxis.set_minor_locator(MultipleLocator(0.2))
        self.ax.grid(which='major', color='#CCCCCC', linestyle='--')
        self.ax.grid(which='minor', color='#CCCCCC', linestyle=':')

    def live_feed(self):
        rospy.init_node(f"NODE_LiDAR_LiveFeed_{self.dc_namespace}_{self.unit_namespace}_{self.lidar_namespace}")
        rospy.logwarn(f"Generating live feed of: {self.dc_namespace}/{self.unit_namespace}/{self.lidar_namespace}...")
        rospy.Subscriber(self.topic_sub, lidarmsg, self.listen_to_plot)

        ani = FuncAnimation(self.fig, self.plot, interval=self.animate_rate * 10)
        plt.show()
        rospy.spin()

        rospy.logwarn(f"Closing live feed of : {self.dc_namespace}/{self.unit_namespace}/{self.lidar_namespace}...")

def main(args):
    ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
    PATH_CONFIG_SYSDEF = Path(ROOT_DIR / "user_scripts" / "user_config" / "system_definition" / "dc_1_system_definition.yaml")

    lidar = LIDAR_LIVEFEED(config_file_path=PATH_CONFIG_SYSDEF, dc_id=args.dc_id, unit_type=args.unit_type, unit_id=args.unit_id, lidar_id=args.lidar_id)
    lidar.live_feed()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="node lidar livefeed")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_type', '--unit_type', help='Provide unit type, Vehicle-v or Infrastructure-i', type=str, default="v")
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle/Infrastructure', type=int, default=1)
    parser.add_argument('-lidar_id', '--lidar_id', help='Provide ID number of LiDAR unit', type=int, default=1)
    args = parser.parse_args()
    main(args)
