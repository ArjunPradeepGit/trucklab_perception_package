#!/usr/bin/env python3

'''
Script: Path generator
Author: Arjun Pradeep
'''

import rospy
import roslaunch
from sensor_msgs.msg import LaserScan
from perception_pkg.msg import lidarmsg, ogmmsg, objdetmsg
from path_planning_pkg.msg import pathmsg
from geometry_msgs.msg import Twist, PoseStamped
import argparse
from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import yaml
import sys
from scipy.spatial.transform import Rotation as R

# from src.scripts.path_generator.user_config.config_path_definition import *

ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "path_planning_pkg" / "src" / "scripts")
PATH_CONFIG_PATHDEF = Path(ROOT_DIR / "path_generator" / "user_config")
sys.path.insert(0, str(PATH_CONFIG_PATHDEF))
from config_path_definition import *

def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def read_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)


class PATH_GENERATOR():
    def __init__(self, path_choice, config_sys_def, dc_id, unit_type, unit_id, lidar_id, update_with_objdet):
        self.update_path_option = update_with_objdet
        self.path_choice = path_choice
        self.topic_pub = f"/PATH/{self.path_choice}"
        self.pub_rate = PUB_RATE
        self.path_msg = pathmsg()

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

        self.topic_sub_objdet = "/" + str(self.dc_namespace) + "/" + str(self.unit_namespace) + "/" + str(self.lidar_namespace) + "/objdet"
        self.topic_sub_optitrack = "/" + str(self.dc_namespace) + "/" + str(self.unit_namespace) + "/optitrack"

    def activate(self):
        rospy.init_node(f"NODE_PathGeneration_choice_{self.path_choice}")
        rospy.logwarn(f"Path Generation: Choice_{self.path_choice} --> Activated")

        # For first run
        self.generate_path()
        self.path_msg.x = self.path_x
        self.path_msg.y = self.path_y

        if self.update_path_option:
            rospy.Subscriber(self.topic_sub_optitrack, PoseStamped, self.listen_optitrack)
            rospy.Subscriber(self.topic_sub_objdet, objdetmsg, self.update_path)

        rate = rospy.Rate(self.pub_rate)
        pub = rospy.Publisher(self.topic_pub, pathmsg, queue_size=10)

        while not rospy.is_shutdown():
            try:
                # self.generate_path()
                self.path_msg.x = self.path_x
                self.path_msg.y = self.path_y
                pub.publish(self.path_msg)
                rate.sleep()
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")

        rospy.logwarn(f"Path Generation: Choice_{self.path_choice} --> Deactivated")


    def generate_path(self):
        path_nodes = PATH[f"CHOICE_{self.path_choice}"]
        pts_path, pts_nodes = self.b_spline_from_pts(path_nodes)
        self.path_x_org = np.array([x for x, _ in pts_path])
        self.path_y_org = np.array([y for _, y in pts_path])
        self.path_x = self.path_x_org.copy()
        self.path_y = self.path_y_org.copy()

    def listen_optitrack(self, msg):
        self.x_vehicle_global = msg.pose.position.x
        self.y_vehicle_global = msg.pose.position.y
        r = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.theta_z_vehicle_global, _, _ = r.as_euler('zxy', degrees=False)

    def update_path(self, msg):
        try:
            for ind in range(np.size(msg.index)):
                if msg.score[ind] > 0.5:

                    print("DETECTED:")

                    # x_obj_local = msg.y[ind]
                    # y_obj_local = -msg.x[ind]
                    #
                    # x_obj_global = self.x_vehicle_global \
                    #                + x_obj_local*np.cos(self.theta_z_vehicle_global) - y_obj_local*np.sin(self.theta_z_vehicle_global)
                    # y_obj_global = self.y_vehicle_global \
                    #                + x_obj_local*np.sin(self.theta_z_vehicle_global) + y_obj_local*np.cos(self.theta_z_vehicle_global)

                    x_obj_global = msg.x[ind]
                    y_obj_global = msg.y[ind]

                    print(f"   Object location: ({x_obj_global}, {y_obj_global})")


                    delta_x = 0.4
                    avoid_dist_x = 1.0
                    avoid_dist_y = 1.0
                    mask_x = np.logical_and(self.path_x_org < x_obj_global + avoid_dist_x - delta_x, self.path_x_org > x_obj_global - avoid_dist_x - delta_x)
                    mask_y = np.logical_and(self.path_y_org < y_obj_global + avoid_dist_y, self.path_y_org > y_obj_global - avoid_dist_y)
                    mask = np.logical_and(mask_x, mask_y)

                    adjust_y = y_obj_global - avoid_dist_y
                    smooth_adjust_y = []
                    for i, value in enumerate(self.path_y_org[mask]):
                        if i < len(self.path_y_org[mask])/2:
                            smooth_adjust_y = np.hstack((smooth_adjust_y, (value - adjust_y) * np.tanh(3 * i / (len(self.path_y_org[mask])/2))))
                        else:
                            smooth_adjust_y = np.hstack((smooth_adjust_y, (value - adjust_y) * np.tanh(3 * (len(self.path_y_org[mask]) - i) / (len(self.path_y_org[mask])/2))))

                    self.path_y[mask] = self.path_y_org[mask] - smooth_adjust_y

            # plt.plot(-self.path_x, -self.path_y, "-bo")
            # plt.plot(-self.path_x[mask], -self.path_y[mask], "-ro")
            # plt.plot(-self.x_vehicle_global, -self.y_vehicle_global, 'gD')
            # plt.plot(-x_obj_global, -y_obj_global, 'rD')
            # plt.legend(["path", "path_corrected", "vehicle", "object"])
            # plt.xlabel("X_trucklab (inversed to 4th quadrant) (m)")
            # plt.ylabel("Y_trucklab (inversed to 4th quadrant) (m)")
            # plt.axis("equal")
            # plt.show()
        except:
            print("No object detected!")


    def add_pts_to_nodes(self, nodes, num=5):
        x_values, y_values = [], []
        for i in range(len(nodes)-1):
            x_values = np.concatenate([x_values[:-1], np.linspace(nodes[i, 0], nodes[i + 1, 0], num)])
            y_values = np.concatenate([y_values[:-1], np.linspace(nodes[i, 1], nodes[i + 1, 1], num)])
        pts = np.vstack((x_values, y_values)).T
        return pts

    def b_spline_from_pts(self, nodes):
        pts_nodes = self.add_pts_to_nodes(nodes, num=10)
        x = pts_nodes[:, 0]
        y = pts_nodes[:, 1]
        pts_nodes = [(pt_node[0], pt_node[1]) for pt_node in pts_nodes]

        tck, *u = interpolate.splprep([x, y], k=3, s=0.01)
        x_smooth, y_smooth = interpolate.splev(np.linspace(0, 1, 800), tck, der=0)

        pts_path = []
        for x_value, y_value in zip(x_smooth, y_smooth):
            pts_path.append((x_value, y_value))

        return pts_path, pts_nodes


def main(args):
    try:
        nodes_try = PATH[f"CHOICE_{args.path_choice}"]
    except:
        raise ValueError("Choice value provided do not match with the already fed choices for path! ")

    ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
    PATH_CONFIG_SYSDEF = Path(ROOT_DIR / "user_scripts" / "user_config" / "system_definition" / "dc_1_system_definition.yaml")

    path_generator = PATH_GENERATOR(path_choice=args.path_choice, config_sys_def=PATH_CONFIG_SYSDEF, dc_id=args.dc_id, unit_type=args.unit_type.lower(), unit_id=args.unit_id, lidar_id=args.lidar_id, update_with_objdet=args.update_objdet)
    path_generator.activate()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="lidar object detection node script")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_type', '--unit_type', help='Provide unit type, Vehicle-v or Infrastructure-i', type=str, default='v')
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle/Infrastructure', type=int, default=1)
    parser.add_argument('-lidar_id', '--lidar_id', help='Provide ID number of LiDAR unit', type=int, default=1)
    parser.add_argument('-path_choice', '--path_choice', help='Provide choice of path', type=int, default=1)
    parser.add_argument('-update_objdet', '--update_objdet', help='If to update path with object detection, give True or False', type=str2bool, default=False)
    args = parser.parse_args()
    main(args)



