#!/usr/bin/env python3

'''
Script: Object Detection - Sensor level
Author: Arjun Pradeep
'''


import argparse
import rospy
import roslaunch
from sensor_msgs.msg import LaserScan
from perception_pkg.msg import lidarmsg, ogmmsg, objdetmsg
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64MultiArray
import numpy as np
import cv2
import matplotlib.pyplot as plt
import yaml
import sys
from pathlib import Path
import os
import time
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
os.environ['CUDA_VISIBLE_DEVICES'] = '0'


ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
PATH_OBJDET_FILES = Path(ROOT_DIR / "developer_scripts" / "src" / "features" / "object_detection" / "1_sensor" / "lidar" / "trucklab_object_detection")
sys.path.insert(0, str(PATH_OBJDET_FILES))
from utils import detect_image, Load_Yolo_model, draw_bbox
from configs import *


def read_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)

class OGM_OBJECT_DETECTION():
    def __init__(self, config_sys_def, config_obj_det, dc_id, unit_type, unit_id, lidar_id):
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
        self.topic_pub = "/" + str(self.dc_namespace) + "/" + str(self.unit_namespace) + "/" + str(self.lidar_namespace) + "/objdet"

        self.topic_sub_unit_optitrack = "/" + str(self.dc_namespace) + "/" + str(self.unit_namespace) + "/optitrack"

        config_info_objdet = read_yaml(config_obj_det)
        self.sub_rate = config_info_objdet["SUBSCRIBE_RATE"]
        self.show_plot = config_info_objdet["SHOW_OBJECT_DETECTION_PLOT"]
        self.objdet_msg = objdetmsg()

        self.ogm_grid_size = config_info_objdet["OGM_GRID_SIZE"]
        self.ogm_img_size = config_info_objdet["OGM_IMG_SIZE"]
        self.ogm_grid_to_px = config_info_objdet["OGM_GRID_TO_PX"]
        self.ogm_px_to_xy = [self.ogm_grid_size[0]/self.ogm_grid_to_px[0], self.ogm_grid_size[1]/self.ogm_grid_to_px[1]]
        self.yolo = Load_Yolo_model()

        # Technique to subscribe at a given rate
        self.processing = False
        self.new_msg = False


    def update_unit_position(self, msg):
        self.x_unit = msg.pose.position.x
        self.y_unit = msg.pose.position.y
        self.z_unit = msg.pose.position.z
        self.r_x_unit = msg.pose.orientation.x
        self.r_y_unit = msg.pose.orientation.y
        self.r_z_unit = msg.pose.orientation.z
        self.r_w_unit = msg.pose.orientation.w
        r = R.from_quat([self.r_x_unit, self.r_y_unit, self.r_z_unit, self.r_w_unit])
        self.theta_unit, _, _ = r.as_euler('zxy', degrees=False)

    def local_to_global(self, x_local, y_local):
        x_global = self.x_unit + y_local * np.cos(self.theta_unit) + x_local * np.sin(self.theta_unit)
        y_global = self.y_unit + y_local * np.sin(self.theta_unit) - x_local * np.cos(self.theta_unit)
        return x_global, y_global

    def objdet_ogm(self):
        rospy.init_node(f"NODE_ObjectDetection_{self.dc_namespace}_{self.unit_namespace}_{self.lidar_namespace}")
        rospy.logwarn(f"Object Detection of: {self.dc_namespace}/{self.unit_namespace}/{self.lidar_namespace} --> Activated")
        rospy.Subscriber(self.topic_sub, ogmmsg, self.detect_objects)
        rospy.Subscriber(self.topic_sub_unit_optitrack, PoseStamped, self.update_unit_position)

        rate = rospy.Rate(self.sub_rate)
        pub = rospy.Publisher(self.topic_pub, objdetmsg, queue_size=10)

        while not rospy.is_shutdown():
            try:
                if self.new_msg:
                    self.processing = True
                    self.new_msg = False
                    pub.publish(self.objdet_msg)
                    rate.sleep()
                    self.processing = False
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")
            except rospy.ROSTimeMovedBackwardsException:
                rospy.logerr("ROS Time Backwards! Just ignore the exception!")

        rospy.logwarn(f"Object Detection of: {self.dc_namespace}/{self.unit_namespace}/{self.lidar_namespace} --> Deactivated")

    def detect_objects(self, msg):
        if not self.processing:
            self.new_msg = True
            rows = msg.rows
            cols = msg.cols
            ogm = np.array(msg.data)
            ogm = ogm.reshape([rows, cols])
            self.ogm_img = np.ones((self.ogm_img_size[1], self.ogm_img_size[0]), np.uint8) * 255
            for row in range(rows):
                for col in range(cols):
                    if ogm[row, col] == 0:
                        self.ogm_img[(row) * self.ogm_grid_to_px[1]: (row + 1) * self.ogm_grid_to_px[1],
                        (col) * self.ogm_grid_to_px[0]: (col + 1) * self.ogm_grid_to_px[0]] = 0

            self.detections, original_image, bboxes_processed, ogm_px_to_xy, CLASSES, rectangle_colors = \
                detect_image(model=self.yolo, input_img=self.ogm_img, ogm_px_to_xy=self.ogm_px_to_xy, input_size=YOLO_INPUT_SIZE, CLASSES=YOLO_TRUCKLAB_CLASSES, rectangle_colors=(255, 0, 0), show_plot=self.show_plot)

            # order of detections info --> code ICXYS
            x_global_values, y_global_values = [], []
            if np.size(self.detections) > 0:
                self.objdet_msg.index = [int(item) for item in self.detections[:, 0]]
                self.objdet_msg.class_num = [int(item) for item in self.detections[:, 1]]
                for item_x, item_y in zip(self.detections[:, 2], self.detections[:, 3]):
                    x_global, y_global = self.local_to_global(item_x, item_y)
                    x_global_values.append(x_global)
                    y_global_values.append(y_global)
                self.objdet_msg.x = x_global_values
                self.objdet_msg.y = y_global_values
                # self.objdet_msg.y = [float(item) for item in self.detections[:, 3]]
                self.objdet_msg.score = [float(item) for item in self.detections[:, 4]]
            else:
                self.objdet_msg.index = []
                self.objdet_msg.class_num = []
                self.objdet_msg.x = []
                self.objdet_msg.y = []
                self.objdet_msg.score = []

            if self.show_plot:
                self.img_detected = draw_bbox(original_image, bboxes_processed, self.objdet_msg.x, self.objdet_msg.y, self.x_unit, self.y_unit, ogm_px_to_xy=ogm_px_to_xy, CLASSES=CLASSES, rectangle_colors=rectangle_colors)
                cv2.imshow(f"Object Detection of {self.dc_namespace}/{self.unit_namespace}/{self.lidar_namespace}", self.img_detected)
                # cv2.imwrite("/home/emeinder/catkin_ws/src/perception_pkg/src/scripts_2/config_files/features/sensors/lidar/trucklab_object_detection/detected_images/image_1.jpg", self.ogm_img)
                cv2.waitKey(10)


def main(args):
    ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
    PATH_CONFIG_SYSDEF = Path(ROOT_DIR / "user_scripts" / "user_config" / "system_definition" / "dc_1_system_definition.yaml")
    PATH_CONFIG_OBJDET = Path(ROOT_DIR / "user_scripts" / "user_config" / "features" / "object_detection" / "object_detection.yaml")

    ogm_objdet = OGM_OBJECT_DETECTION(config_sys_def=PATH_CONFIG_SYSDEF, config_obj_det=PATH_CONFIG_OBJDET, dc_id=args.dc_id, unit_type=args.unit_type.lower(), unit_id=args.unit_id, lidar_id=args.lidar_id)
    ogm_objdet.objdet_ogm()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="lidar object detection node script")
    parser.add_argument('-dc_id', '--dc_id', help='Provide ID number of Distribution Center', type=int, default=1)
    parser.add_argument('-unit_type', '--unit_type', help='Provide unit type, Vehicle-v or Infrastructure-i', type=str, default='v')
    parser.add_argument('-unit_id', '--unit_id', help='Provide ID number of Vehicle/Infrastructure', type=int, default=1)
    parser.add_argument('-lidar_id', '--lidar_id', help='Provide ID number of LiDAR unit', type=int, default=1)
    args = parser.parse_args()
    main(args)
