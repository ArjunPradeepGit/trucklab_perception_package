#!/usr/bin/env python3

'''
Script: Analyze Obstacle Detection
Author: Arjun Pradeep
'''

from pathlib import Path
SCRIPTS_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")


import argparse
# import os
# import rospy
# import roslaunch
# import message_filters
# from std_msgs.msg import Bool
# from sensor_msgs.msg import LaserScan
# from sensor_msgs.msg import CompressedImage
# from perception_pkg.msg import lidarmsg, ogmmsg
import subprocess
import numpy as np
import cv2
import matplotlib.pyplot as plt
import yaml
import sys
import bagpy
from bagpy import bagreader
import pandas as pd
from scipy.spatial.transform import Rotation as R

def read_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)


def main(args):
    ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
    PATH_CONFIG_SYSDEF = Path(ROOT_DIR / "user_scripts" / "user_config" / "system_definition" / "dc_1_system_definition.yaml")
    PATH_FOLDER_BAGS = Path(ROOT_DIR / "developer_scripts" / "test_scripts" / "test_obstacle_detection" / "bags")

    config_info_sysdef = read_yaml(PATH_CONFIG_SYSDEF)

    dc_namespace = config_info_sysdef[f"DC_{args.dc_id}"]["NAMESPACE"]
    if args.unit_type == 'v':
        unit_namespace = config_info_sysdef[f"DC_{args.dc_id}"][f"VEHICLE_{args.unit_id}"]["NAMESPACE"]
        lidar_namespace = config_info_sysdef[f"DC_{args.dc_id}"][f"VEHICLE_{args.unit_id}"][f"LIDAR_{args.lidar_id}"]["NAMESPACE"]
    elif args.unit_type == 'i':
        unit_namespace = config_info_sysdef[f"DC_{args.dc_id}"][f"INFRASTRUCTURE_{args.unit_id}"]["NAMESPACE"]
        lidar_namespace = config_info_sysdef[f"DC_{args.dc_id}"][f"INFRASTRUCTURE_{args.unit_id}"][f"LIDAR_{args.lidar_id}"]["NAMESPACE"]
    else:
        raise ValueError("Received unacceptable value for 'unit_type'. Must be either 'v' or 'i'.")


    topic_lidar = f"/{dc_namespace}/{unit_namespace}/{lidar_namespace}/lidarscan"
    topic_optitrack = f"/{dc_namespace}/{unit_namespace}/optitrack"

    df_lidar_captures = pd.DataFrame()

    range_cols = []
    range_means = []
    range_stds = []
    angle_cols = []
    angle_means = []
    angle_stds = []
    for cap_num in range(1, args.tot_cap+1):
        bag_name = "bag_" + args.obj_name + f"_{cap_num}.bag"
        path_bag_file = str(Path(PATH_FOLDER_BAGS / bag_name))
        bag_file = bagreader(path_bag_file)

        LIDAR_MSG = bag_file.message_by_topic(topic_lidar)
        df_lidar = pd.read_csv(LIDAR_MSG)
        for i in range(360):
            try:
                if len(df_lidar[f"ranges_{i}"]) > 0:
                    range_cols.append(f"ranges_{i}")
                    range_means.append(df_lidar[f"ranges_{i}"].mean())
                    range_stds.append(df_lidar[f"ranges_{i}"].std())
                    angle_cols.append(f"angles_{i}")
                    angle_means.append(df_lidar[f"angles_{i}"].mean())
                    angle_stds.append(df_lidar[f"angles_{i}"].std())
            except:
                continue

    range_means = np.array(range_means)
    range_stds = np.array(range_stds)
    angle_means = np.array(angle_means)
    angle_stds = np.array(angle_stds)

    x_means = range_means * np.cos(angle_means)
    y_means = range_means * np.sin(angle_means)

    # x_means = x_means[y_means < 1.0]
    # y_means = y_means[y_means < 1.0]

    font_size = 18
    plt.figure(1)
    plt.rcParams['font.size'] = '16'
    plt.title("LiDAR plot", fontsize=font_size+5)
    means_plot = plt.plot(x_means, y_means, 'b+')
    plt.xlabel("Mean of x values with respect to LiDAR Frame (m)", fontsize=font_size)
    plt.ylabel("Mean of y values with respect to LiDAR Frame (m)", fontsize=font_size)
    plt.axis("equal")
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="LiDAR capture - reade and update")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_type', '--unit_type', help='Provide unit type, Vehicle-v or Infrastructure-i', type=str, default="v")
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle/Infrastructure', type=int, default=1)
    parser.add_argument('-lidar_id', '--lidar_id', help='Provide ID number of LiDAR', type=int, default=1)
    parser.add_argument('-tot_cap', '--tot_cap', help='Provide the number of total captures', type=int, default=1)
    parser.add_argument('-obj_name', '--obj_name', help='Provide the name of the object', type=str, default="lidar_object_error")
    args = parser.parse_args()
    main(args)

