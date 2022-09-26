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


def main():
    # Optitrack measurements
    x_tractor = 2.49664
    y_tractor = 4.81054
    r_x_tractor = 0.0013726
    r_y_tractor = 0.0025953
    r_z_tractor = 0.0288498
    r_w_tractor = 0.9995794
    r = R.from_quat([r_x_tractor, r_y_tractor, r_z_tractor, r_w_tractor])
    theta_tractor, _, _ = r.as_euler('zxy', degrees=False)

    x_semitrailer = 3.51981
    y_semitrailer = 5.257769
    r_x_semitrailer = -0.02323425
    r_y_semitrailer = -0.02786593
    r_z_semitrailer = 0.72019526
    r_w_semitrailer = 0.69282208
    r = R.from_quat([r_x_semitrailer, r_y_semitrailer, r_z_semitrailer, r_w_semitrailer])
    theta_semitrailer, _, _ = r.as_euler('zxy', degrees=False)


    # Detection measurements
    x_detection_local = -0.38
    y_detection_local = 1.09 + 0.035

    x_detection_global = x_tractor + y_detection_local*np.cos(theta_tractor) + x_detection_local*np.sin(theta_tractor)
    y_detection_global = y_tractor + y_detection_local * np.sin(theta_tractor) - x_detection_local * np.cos(theta_tractor)

    print(f"Tractor:       ({x_tractor}, {y_tractor})")
    print(f"Semitrailer:   ({x_semitrailer}, {y_semitrailer})")
    print(f"Semitrailer Detected:   ({x_detection_global}, {y_detection_global})")

    print(f"Error:   ({x_detection_global - x_semitrailer}, {y_detection_global - y_semitrailer})")


if __name__ == "__main__":
    main()

