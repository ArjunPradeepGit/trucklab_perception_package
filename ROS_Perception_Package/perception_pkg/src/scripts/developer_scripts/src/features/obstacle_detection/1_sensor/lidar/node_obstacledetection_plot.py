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
import cv2
import matplotlib.pyplot as plt
import yaml
import sys
from pathlib import Path


def read_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)

class OGM_LIDAR_PLOT():
    def __init__(self, config_sys_def, config_obs_det, dc_id, unit_type, unit_id, lidar_id):
        config_info_sysdef = read_yaml(config_sys_def)
        self.dc_id = dc_id
        self.dc_namespace = config_info_sysdef[f"DC_{self.dc_id}"]["NAMESPACE"]
        self.unit_type = unit_type

        if self.unit_type == 'v':
            self.unit_id = unit_id
            self.unit_namespace = config_info_sysdef[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"]["NAMESPACE"]
            self.lidar_id = lidar_id
            self.lidar_namespace = \
            config_info_sysdef[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"][f"LIDAR_{self.lidar_id}"]["NAMESPACE"]
        elif self.unit_type == 'i':
            self.unit_id = unit_id
            self.unit_namespace = config_info_sysdef[f"DC_{self.dc_id}"][f"INFRASTRUCTURE_{self.unit_id}"]["NAMESPACE"]
            self.lidar_id = lidar_id
            self.lidar_namespace = \
            config_info_sysdef[f"DC_{self.dc_id}"][f"INFRASTRUCTURE_{self.unit_id}"][f"LIDAR_{self.lidar_id}"][
                "NAMESPACE"]
        else:
            raise ValueError("Received unacceptable value for 'unit_type'. Must be either 'v' or 'i'.")


        self.topic_sub = "/" + str(self.dc_namespace) + "/" + str(self.unit_namespace) + "/" + str(self.lidar_namespace) + "/ogm"
        config_info_obsdet = read_yaml(config_obs_det)
        self.pub_rate = config_info_obsdet["PUBLISH_RATE"]

        self.ogm_grid_size = config_info_obsdet["OGM_GRID_SIZE"]
        self.ogm_img_size = config_info_obsdet["OGM_IMG_SIZE"]
        self.ogm_grid_to_px = config_info_obsdet["OGM_GRID_TO_PX"]

        scale = config_info_obsdet["IMG_RESIZE_FACTOR"]
        width = int(self.ogm_img_size[1] * scale)
        height = int(self.ogm_img_size[0] * scale)
        self.resize_factors = (width, height)

    def plot_ogm(self):
        rospy.init_node(f"NODE_OGM_plot_{self.dc_namespace}_{self.unit_namespace}_{self.lidar_namespace}")
        rospy.logwarn(f"OGM plot of: {self.dc_namespace}/{self.unit_namespace}/{self.lidar_namespace} --> Activated")
        rospy.Subscriber(self.topic_sub, ogmmsg, self.show_ogm_2D)
        rospy.spin()
        rospy.logwarn(f"OGM plot of: {self.dc_namespace}/{self.unit_namespace}/{self.lidar_namespace} --> Deactivated")

    def show_ogm_2D(self, msg):
        rows = msg.rows
        cols = msg.cols
        ogm = np.array(msg.data)
        ogm = np.reshape(ogm, (rows, cols))
        self.ogm_img = np.ones((self.ogm_img_size[1], self.ogm_img_size[0]), np.uint8) * 255
        for row in range(rows):
            for col in range(cols):
                if ogm[row, col] == 0:
                    self.ogm_img[(row) * self.ogm_grid_to_px[1]: (row + 1) * self.ogm_grid_to_px[1],
                    (col) * self.ogm_grid_to_px[0]: (col + 1) * self.ogm_grid_to_px[0]] = 0

        self.ogm_img_resized = cv2.resize(self.ogm_img, self.resize_factors, interpolation = cv2.INTER_AREA)
        cv2.imshow(f"OGM of {self.dc_namespace}_{self.unit_namespace}_{self.lidar_namespace}", self.ogm_img_resized)
        cv2.waitKey(10)

    # def show_ogm_3D(self):
    #     rows = int((self.map_limits[1][1] - self.map_limits[1][0]) / self.ogm_grid_size[1])
    #     cols = int((self.map_limits[0][1] - self.map_limits[0][0]) / self.ogm_grid_size[0])
    #     x = np.arange(self.map_limits[0][0], self.map_limits[0][1], self.ogm_grid_size[0])
    #     y = np.arange(self.map_limits[1][1], self.map_limits[1][0], -self.ogm_grid_size[1])
    #
    #     X = np.tile(x, rows)
    #     Y = np.repeat(y, cols)
    #     Z = 0
    #
    #     dx = self.ogm_grid_size[0]  # Width of each bar
    #     dy = self.ogm_grid_size[1]  # Depth of each bar
    #     dz = (self.ogm.flatten() + 1)  # Height of each bar
    #     clr = ['m' if value > 0 else 'navy' for value in dz]
    #
    #     def set_axes_equal(ax):
    #         '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    #         cubes as cubes, etc..  This is one possible solution to Matplotlib's
    #         ax.set_aspect('equal') and ax.axis('equal') not working for 3D.
    #
    #         Input
    #           ax: a matplotlib axis, e.g., as output from plt.gca().
    #         '''
    #
    #         x_limits = ax.get_xlim3d()
    #         y_limits = ax.get_ylim3d()
    #         z_limits = ax.get_zlim3d()
    #
    #         x_range = abs(x_limits[1] - x_limits[0])
    #         x_middle = np.mean(x_limits)
    #         y_range = abs(y_limits[1] - y_limits[0])
    #         y_middle = np.mean(y_limits)
    #         z_range = abs(z_limits[1] - z_limits[0])
    #         z_middle = np.mean(z_limits)
    #
    #         # The plot bounding box is a sphere in the sense of the infinity
    #         # norm, hence I call half the max range the plot radius.
    #         plot_radius = 0.5 * max([x_range, y_range, z_range])
    #
    #         ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    #         ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    #         ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])
    #
    #     self.fig = plt.figure(figsize=(10, 10))
    #     self.ax = self.fig.add_subplot(projection='3d')
    #     self.ax.bar3d(X, Y, Z, dx, dy, dz, color=clr)
    #     set_axes_equal(self.ax)
    #     self.ax.grid(False)
    #     self.fig.suptitle('Occupancy Grid Map', fontsize=18)
    #     self.ax.set_xlabel('X-axis (m)', fontsize=10)
    #     self.ax.set_ylabel('Y-axis (m)', fontsize=10)
    #     plt.show()


def main(args):
    ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
    PATH_CONFIG_SYSDEF = Path(ROOT_DIR / "user_scripts" / "user_config" / "system_definition" / "dc_1_system_definition.yaml")
    PATH_CONFIG_OBSDET = Path(ROOT_DIR / "user_scripts" / "user_config" / "features" / "obstacle_detection" / "obstacle_detection.yaml")

    ogm_lidar = OGM_LIDAR_PLOT(config_sys_def=PATH_CONFIG_SYSDEF, config_obs_det=PATH_CONFIG_OBSDET, dc_id=args.dc_id, unit_type=args.unit_type.lower(), unit_id=args.unit_id, lidar_id=args.lidar_id)
    ogm_lidar.plot_ogm()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="node lidar obstacle detection plot")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_type', '--unit_type', help='Provide unit type, Vehicle-v or Infrastructure-i', type=str, default='v')
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle/Infrastructure', type=int, default=1)
    parser.add_argument('-lidar_id', '--lidar_id', help='Provide ID number of LiDAR unit', type=int, default=1)
    args = parser.parse_args()
    main(args)
