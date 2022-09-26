#!/usr/bin/env python3

import subprocess
from pathlib import Path
from termcolor import colored

default_cmd = 'gnome-terminal --tab -- '
make_exec_cmd = 'chmod +x '

def make_file_executable(file_path):
    cmd = default_cmd + make_exec_cmd + file_path
    program = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=None, shell=True)
    program.wait(timeout=None)

ROOT_DIR = Path.cwd()
DIR_SCRIPTS = ROOT_DIR.parent.parent

DIR_USER_SCRIPTS = Path(DIR_SCRIPTS / "user_scripts")
DIR_DEVELOPER_SCRIPTS = Path(DIR_SCRIPTS / "developer_scripts")


print(colored("Process started: Converting all scripts as executables within the ROS package...", "yellow"))
print(colored("This will take a couple of mins...\n\n", "yellow"))
########################################################################################################################
##### USER SCRIPTS #####
path_user_app_scripts= Path(DIR_USER_SCRIPTS / "app_scripts")

nodes = [
    "data_acquisition/data_capture_for_training_data/data_capture_at_a_pose.py",
    "data_acquisition/data_capture_for_training_data/data_read_bag_and_update_captures.py",

    "data_acquisition/data_collection_per_config/data_collector.py",
    "data_acquisition/data_collection_per_config/data_collector.py",

    "features/object_detection/1_sensor/lidar/lidar_object_detection_activation.py",
    "features/obstacle_detection/1_sensor/lidar/lidar_collision_alert_activation.py",
    "features/obstacle_detection/1_sensor/lidar/lidar_obstacle_detection_activation.py",

    "system_activation/camera_activation.py",
    "system_activation/distributioncenter_activation.py",
    "system_activation/infrastructure_activation.py",
    "system_activation/lidar_activation.py",
    "system_activation/optitrack_activation.py",
    "system_activation/odometry_activation.py",
    "system_activation/imu_activation.py",
    "system_activation/vehicle_activation.py",
]

for node in nodes:
    node_str = str(Path(path_user_app_scripts / node))
    make_file_executable(node_str)
    print(colored(f"Converted: '{node_str}'", "yellow"))



########################################################################################################################
##### DEVELOPER SCRIPTS #####
path_developer_src_scripts= Path(DIR_DEVELOPER_SCRIPTS / "src")
nodes = [
    "features/object_detection/1_sensor/lidar/node_objectdetection.py",
    "features/obstacle_detection/1_sensor/lidar/node_collision_alert.py",
    "features/obstacle_detection/1_sensor/lidar/node_obstacledetection_definition.py",
    "features/obstacle_detection/1_sensor/lidar/node_obstacledetection_plot.py",

    "system_definition/1_sensor/camera/node_camera_livefeed.py",
    "system_definition/1_sensor/camera/node_camera_preprocessor.py",
    "system_definition/1_sensor/lidar/node_lidar_livefeed.py",
    "system_definition/1_sensor/lidar/node_lidar_preprocessor.py",
    "system_definition/1_sensor/lidar/node_lidar_pass_full.py",
    "system_definition/1_sensor/optitrack/node_optitrack_preprocessor.py",
    "system_definition/1_sensor/odometry/node_odometry_preprocessor.py",
    "system_definition/1_sensor/imu/node_imu_preprocessor.py",
    "system_definition/2_unit/infrastructure/node_infrastructure_definition.py",
    "system_definition/2_unit/vehicle/node_vehicle_definition.py",
    "system_definition/3_dc/node_distributioncenter_definition.py",
]

for node in nodes:
    node_str = str(Path(path_developer_src_scripts / node))
    make_file_executable(node_str)
    print(colored(f"Converted: '{node_str}'", "yellow"))




path_developer_test_scripts = Path(DIR_DEVELOPER_SCRIPTS / "test_scripts")
nodes = [
    "test_lidar_accuracy/analyze_lidar_static.py",
    "test_object_detection/analyze_object_detection.py",
    "test_obstacle_detection/analyze_obstacle_detection.py",
]

for node in nodes:
    node_str = str(Path(path_developer_test_scripts / node))
    make_file_executable(node_str)
    print(colored(f"Converted: '{node_str}'", "yellow"))


print(colored("\nAll required scripts --> converted as executables", "green"))
print(colored("\nSuccessfully completed!", "green"))





