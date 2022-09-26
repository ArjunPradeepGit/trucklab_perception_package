"""
About: Config files to define required properties for our custom "Dataset"
Author: Arjun Pradeep
"""
import os
import numpy as np

# ********************************************************************************************************************

def rchop(s, suffix):
    if suffix and s.endswith(suffix):
        return s[:-len(suffix)]
    return s

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = rchop(ROOT_DIR, "\yolov3_files")
# ROOT_DIR = r"D:\Local_PDEng_ASD\TruckLab_LOCAL\TruckLab_Object_Detection_from_scratch_GIT\LiDAR_2D_Object_Detection\TrainModel_TruckLab"
# print(ROOT_DIR)

# YOLO details
YOLO_STRIDES                = [8, 16, 32]
YOLO_ANCHORS                = [[[10,  13], [16,   30], [33,   23]],
                               [[30,  61], [62,   45], [59,  119]],
                               [[116, 90], [156, 198], [373, 326]]]
YOLO_ANCHOR_PER_SCALE       = 3
YOLO_MAX_BBOX_PER_SCALE     = 100

# Data folder
DATA_FOLDER = "training_data"

# Train info
TRAIN_ANNOT_PATH            = os.path.abspath(os.path.join(ROOT_DIR, "training_data\Dataset_train.txt"))
TRAIN_INPUT_SIZE            = 416
TRAIN_BATCH_SIZE            = 8
TRAIN_DATA_AUG              = False
TRAIN_CLASSES               = os.path.abspath(os.path.join(ROOT_DIR, "training_data\Dataset_names.txt"))
TRAIN_LOAD_IMAGES_TO_RAM    = True # With True faster training, but need more RAM
TRAIN_LR_INIT               = 1e-4
TRAIN_LR_END                = 1e-6
TRAIN_WARMUP_EPOCHS         = 3
TRAIN_EPOCHS                = 10
TRAIN_TRANSFER_FROM_DARKNET = False


# Test info
TEST_ANNOT_PATH             = os.path.abspath(os.path.join(ROOT_DIR, "training_data\Dataset_test.txt"))
TEST_INPUT_SIZE             = TRAIN_INPUT_SIZE
TEST_BATCH_SIZE             = 8
TEST_DATA_AUG               = False

# YOLO info
YOLOV3_TRUCKLAB_WEIGHTS                 = os.path.join(ROOT_DIR, "model_current\yolov3_trucklab")
YOLOV3_TRUCKLAB_SAVE_TRAINED_WEIGHTS    = os.path.abspath(os.path.join(ROOT_DIR, "model_saved_after_training\yolov3_trucklab"))
YOLO_IOU_LOSS_THRESH                    = 0.5

YOLOV3_ORIGINAL_WEIGHTS                 = os.path.abspath(os.path.join(ROOT_DIR, "model_original_darknet\yolov3_original.h5"))
YOLO_COCO_CLASSES                       = os.path.abspath(os.path.join(ROOT_DIR, "model_original_darknet\coco.names"))




# some pre-processing constants --> for making code-contexts meaningful
STRIDES                     = np.array(YOLO_STRIDES)
YOLO_TRUCKLAB_CLASSES       = TRAIN_CLASSES
YOLO_TRUCKLAB_CLASSES_OLD   = os.path.abspath(os.path.join(ROOT_DIR, "training_data\Dataset_names_old.txt"))
YOLO_V3_MODEL               = YOLOV3_TRUCKLAB_WEIGHTS
YOLO_INPUT_SIZE             = TRAIN_INPUT_SIZE

TEST_SCORE_THRESHOLD        = 0.3
TEST_IOU_THRESHOLD          = 0.45
YOLO_FRAMEWORK              = "tf"