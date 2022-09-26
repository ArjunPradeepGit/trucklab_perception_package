"""
YOLOv3 Configuration File
"""
import os
from pathlib import Path

ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
PATH_OBJDET_FILES = Path(ROOT_DIR / "developer_scripts" / "src" / "features" / "object_detection" / "1_sensor" / "lidar" / "trucklab_object_detection")
PATH_CLASS_NAMES = Path(PATH_OBJDET_FILES / "trucklab_classes.names")
PATH_MODEL_DATA = Path(ROOT_DIR / "data" / "model_data")
# print(ROOT_DIR)

# YOLO options
YOLO_V3_MODEL               = Path(PATH_MODEL_DATA / "model_current" / "yolov3_trucklab")
YOLO_INPUT_SIZE             = 416
YOLO_STRIDES                = [8, 16, 32]
YOLO_ANCHORS                = [[[10,  13], [16,   30], [33,   23]],
                                [[30,  61], [62,   45], [59,  119]],
                                [[116, 90], [156, 198], [373, 326]]]
YOLO_TRUCKLAB_CLASSES       = str(PATH_CLASS_NAMES)

CONFIDENCE_THRESHOLD = 0.49