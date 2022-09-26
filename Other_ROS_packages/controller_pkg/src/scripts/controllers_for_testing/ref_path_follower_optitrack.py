#!/usr/bin/env python3

'''
Controller --> to follow a reference path
Author: Arjun Pradeep
'''

import os
import argparse
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from path_planning_pkg.msg import pathmsg
import numpy as np
import yaml
import sys
import math
from shapely.geometry import Point, LineString
from shapely.geometry.polygon import Polygon
from shapely.ops import nearest_points
from path_generator import b_spline_from_pts, change_x_and_y
from scipy.spatial.transform import Rotation as R
from pathlib import Path

NUM_STABILITY = 0.1 ** 5

# def path_generator(choice):
#     # Path based out of nodes
#
#     if choice == 1:
#         nodes_around_trucklab = np.array([[1.5, 1.5], [1.5, 5.0], [5.0, 5.0], [5.0, 1.5], [1.5, 1.5]])
#     elif choice == 2:
#         nodes_around_trucklab = np.array(
#             [[5.386, 5.217], [5.905, 4.533], [6.000, 3.993], [6.000, 2.558], [5.130, 2.487],
#              [1.659, 2.487], [1.301, 2.558], [1.301, 4.835], [2.041, 5.403], [2.587, 5.952], [3.348, 6.136],
#              [4.542, 6.165], [5.386, 5.217]])
#     else:
#         raise ValueError("Choice value does not exist!")
#     pts_path, pts_nodes = b_spline_from_pts(nodes_around_trucklab)
#     return LineString(pts_path)

def read_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)


class CONTROLLER_REF_PATH_FOLLOWER(object):
    def __init__(self, config_sys_def, dc_id, unit_type, unit_id, lidar_id, vel, path_choice):
        config_info_sysdef = read_yaml(config_sys_def)
        self.dc_id = dc_id
        self.dc_namespace = config_info_sysdef[f"DC_{self.dc_id}"]["NAMESPACE"]
        self.unit_type = unit_type
        self.unit_id = unit_id
        self.unit_namespace = config_info_sysdef[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"]["NAMESPACE"]
        self.unit_type_str = config_info_sysdef[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"]["TYPE"]
        self.path_choice = path_choice

        if self.unit_type == 'v' and self.unit_type_str == "TRACTOR":
            self.lidar_id = lidar_id
            self.lidar_namespace = config_info_sysdef[f"DC_{self.dc_id}"][f"VEHICLE_{self.unit_id}"][f"LIDAR_{self.lidar_id}"]["NAMESPACE"]
        else:
            raise ValueError(f"Received unacceptable value for 'unit_type' or check if this vehicle id: {self.unit_id} is a tractor.")

        self.topic_pub = "/tractor" + str(unit_id) + "/cmd_tractor" + str(unit_id)

        self.topic_sub_path = f"/PATH/{self.path_choice}"
        self.topic_sub_optitrack = "/" + str(self.dc_namespace) + "/" + str(self.unit_namespace) + "/optitrack"
        self.topic_sub_collision = "/" + str(self.dc_namespace) + "/" + str(self.unit_namespace) + "/" + str(self.lidar_namespace) + "/collision_alert"

        self.pub_rate = 5  # hz
        self.cmd_msg = Twist()

        # self.ref_path_poly = path_generator(self.path_choice)

        self.stanley_kn = 1.5
        self.stanley_kd = 1.0
        self.steering_min = -25 * np.pi / 180
        self.steering_max = 25 * np.pi / 180
        self.stop = False
        self.collision_flag = False

        # Constant Values
        self.x_vel_set = vel

        self.first_run = True


    def get_path(self, msg):
        path_x = msg.x
        path_y = msg.y
        pts_path = []
        for x, y in zip(path_x, path_y):
            pt_path = (x, y)
            pts_path.append(pt_path)
        self.ref_path_poly = LineString(pts_path)
        self.last_ref_point = Point(self.ref_path_poly.coords[-1])

    def find_e_and_heading_error(self):
        self.current_pose_local = Point(self.X_global, self.Y_global)
        pt_nearest, _ = nearest_points(self.ref_path_poly, self.current_pose_local)
        # print(f"     The nearest point is (local): ({pt_nearest.xy[0][0]}, {pt_nearest.xy[1][0]})")

        min_dist = 10000.0
        min_index = 0
        for ind, point in enumerate(self.ref_path_poly.coords):
            pt = Point(point[0], point[1])
            dist = pt.distance(pt_nearest)
            if dist < min_dist:
                min_dist = dist
                min_index = ind
        pt_nearest_in_path = Point(self.ref_path_poly.coords[min_index][0], self.ref_path_poly.coords[min_index][1])
        # print("     Min index is: ", min_index)
        # print(f"     IN path - The nearest point is (local): {pt_nearest_in_path.xy[0][0]}, {pt_nearest_in_path.xy[1][0]})")
        if min_index > len(self.ref_path_poly.coords)-10:
            self.stop = True
            e_value, heading_error = 0.0, 0.0
            # print(f"     Heading error: {heading_error},  e_value: {e_value}")
            return e_value, heading_error
        else:
            self.stop = False
            pt_next_nearest_in_path = Point(self.ref_path_poly.coords[min_index + 1][0], self.ref_path_poly.coords[min_index + 1][1])
            slope_of_nearest_line_segment = math.atan2((pt_next_nearest_in_path.xy[1][0] - pt_nearest_in_path.xy[1][0]),
                                                       (pt_next_nearest_in_path.xy[0][0] - pt_nearest_in_path.xy[0][0] + NUM_STABILITY))

            polygon_for_check = Polygon(self.ref_path_poly)
            if polygon_for_check.contains(self.current_pose_local):
                factor = 1.0
            else:
                factor = -1.0
            # print("     factor: ", factor)
            e_value = pt_nearest.distance(self.current_pose_local) * factor

            heading_error = slope_of_nearest_line_segment - self.theta_Z_global
            # print(f"     Heading error BEFORE: {heading_error * 180/np.pi},  e_value: {e_value * 180/np.pi}")
            if np.abs(heading_error) > np.pi:
                # print("     greater!!!")
                heading_error = heading_error - 2 * np.pi * np.sign(heading_error)
            # print(f"     Heading error AFTER: {heading_error * 180/np.pi},  e_value: {e_value * 180/np.pi}")
            return e_value, heading_error

    def get_steering_value(self):
        e_value, heading_error = self.find_e_and_heading_error()
        # steering_set_value = heading_error + math.atan2(self.stanley_kn * e_value, self.x_vel_local_read + self.stanley_kd)
        steering_set_value = heading_error + math.atan2(self.stanley_kn * e_value, self.x_vel_set + self.stanley_kd)
        return steering_set_value

    def listen_optitrack(self, msg):
        self.X_global = msg.pose.position.x
        self.Y_global = msg.pose.position.y
        r = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.theta_Z_global, _, _ = r.as_euler('zxy', degrees=False)
        print("Current: ", self.X_global, self.Y_global, self.theta_Z_global * 180/np.pi)

        steering_set_value = self.get_steering_value()

        if self.stop or self.collision_flag:
            self.cmd_msg.linear.x = 0.0
            self.cmd_msg.angular.z = 0.0
            # print(f"Steering value is : {0.0}")
        else:
            self.cmd_msg.linear.x = self.x_vel_set
            self.cmd_msg.angular.z = np.clip(steering_set_value, self.steering_min, self.steering_max)
            # print(f"Steering value is : {np.clip(steering_set_value, self.steering_min, self.steering_max)*180/np.pi}")

    def check_for_collision(self, msg):
        self.collision_flag = msg.data


    def activate_controller(self):
        rospy.init_node(f"NODE_Activate_Controller_GoToPoint_{self.unit_id}")
        rospy.logwarn(f"Tractor_{self.unit_id} controller --> Activated")

        rospy.Subscriber(self.topic_sub_path, pathmsg, self.get_path)
        rospy.Subscriber(self.topic_sub_optitrack, PoseStamped, self.listen_optitrack)
        rospy.Subscriber(self.topic_sub_collision, Bool, self.check_for_collision)

        rate = rospy.Rate(self.pub_rate)
        pub_scan = rospy.Publisher(self.topic_pub, Twist, queue_size=10)

        while not rospy.is_shutdown():
            try:
                pub_scan.publish(self.cmd_msg)
                rate.sleep()
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")
            except rospy.ROSTimeMovedBackwardsException:
                rospy.logerr("ROS Time Backwards! Just ignore the exception!")

        rospy.logwarn(f"Tractor_{self.unit_id} controller --> Deactivated")


def main(args):
    ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
    PATH_CONFIG_SYSDEF = Path(ROOT_DIR / "user_scripts" / "user_config" / "system_definition" / "dc_1_system_definition.yaml")
    # PATH_CONFIG_OBSDET = Path(ROOT_DIR / "user_config" / "features" / "obstacle_detection" / "obstacle_detection.yaml")

    controller = CONTROLLER_REF_PATH_FOLLOWER(config_sys_def=PATH_CONFIG_SYSDEF, dc_id=args.dc_id, unit_type=args.unit_type.lower(), unit_id=args.unit_id, lidar_id=args.lidar_id, vel=args.vel, path_choice=args.path_choice)
    controller.activate_controller()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="lidar node activation script")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_type', '--unit_type', help='Provide unit type, Vehicle-v or Infrastructure-i', type=str, default='v')
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle/Infrastructure', type=int, default=1)
    parser.add_argument('-lidar_id', '--lidar_id', help='Provide ID number of LiDAR unit', type=int, default=1)
    parser.add_argument('-vel', '--vel', help='Provide velocity value in m/s', type=float, default=0.1)
    parser.add_argument('-path_choice', '--path_choice', help='Provide choice of path', type=int, default=1)
    args = parser.parse_args()
    main(args)
