#!/usr/bin/env python3

import subprocess
from pathlib import Path, PurePath

default_cmd = 'gnome-terminal --tab -- '
make_exec_cmd = 'chmod +x '

def make_file_executable(file_path):
    cmd = default_cmd + make_exec_cmd + file_path
    program = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=None, shell=True)
    program.wait(timeout=None)

ROOT_DIR = Path.cwd()
PATH_SCRIPTS = ROOT_DIR.parent.parent


########################################################################################################################
##### APP SCRIPTS #####
# path_app_scripts_features_objdet = Path(PATH_SCRIPTS / "app_scripts" / "features" / "object_detection" / "1_sensor" / "lidar")
# total_nodes = 1
# node = [None] * total_nodes
# node[0] = "lidar_object_detection_activation.py"
# for i in range(total_nodes):
#     node[i] = str(Path(path_app_scripts_features_objdet / node[i]))
#     # Activation
#     make_file_executable(node[i])


########################################################################################################################
##### SRC #####
path_src_pathgen = Path(PATH_SCRIPTS / "path_generator" / "src")
total_nodes = 1
node = [None] * total_nodes
node[0] = "node_path_generator.py"
for i in range(total_nodes):
    node[i] = str(Path(path_src_pathgen / node[i]))
    # Activation
    make_file_executable(node[i])

