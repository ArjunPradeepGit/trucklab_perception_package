"""
About: Learning to create a custom "Dataset" with iterator
Author: Arjun Pradeep
"""

# ********************************************************************************************************************

"""
All necessary imports
"""

import os
os.environ['CUDA_VISIBLE_DEVICES'] = '0'
os.environ['TF_FORCE_GPU_ALLOW_GROWTH'] = 'true'
from tensorflow.python.client import device_lib
print(device_lib.list_local_devices())
import shutil
import numpy as np
import tensorflow as tf
from yolov3_files.dataset import Dataset
from yolov3_files.yolov3 import Create_Yolo, compute_loss
from yolov3_files.configs import *
from evaluate_mAP import get_mAP


def main():
    trainset = Dataset('train')
    testset = Dataset('test')

    steps_per_epoch = len(trainset)
    global_steps = tf.Variable(0, trainable=False, dtype=tf.int64)
    warmup_steps = TRAIN_WARMUP_EPOCHS * steps_per_epoch
    total_steps = TRAIN_EPOCHS * steps_per_epoch

    if TRAIN_TRANSFER_FROM_DARKNET:
        Darknet_yolo = Create_Yolo(input_size=YOLO_INPUT_SIZE, CLASSES=YOLO_COCO_CLASSES)
        Darknet_yolo.load_weights(YOLOV3_ORIGINAL_WEIGHTS)

        yolo = Create_Yolo(input_size=YOLO_INPUT_SIZE, training=True, CLASSES=YOLO_TRUCKLAB_CLASSES)
        for i, l in enumerate(Darknet_yolo.layers):
            layer_weights = l.get_weights()
            if layer_weights != []:
                try:
                    yolo.layers[i].set_weights(layer_weights)
                except:
                    print("skipping", yolo.layers[i].name)

    else:
        yolo_old = Create_Yolo(input_size=YOLO_INPUT_SIZE, training=True, CLASSES=YOLO_TRUCKLAB_CLASSES_OLD)
        yolo_old.load_weights(YOLOV3_TRUCKLAB_WEIGHTS)

        yolo = Create_Yolo(input_size=YOLO_INPUT_SIZE, training=True, CLASSES=YOLO_TRUCKLAB_CLASSES)
        for i, l in enumerate(yolo_old.layers):
            layer_weights = l.get_weights()
            if layer_weights != []:
                try:
                    yolo.layers[i].set_weights(layer_weights)
                except:
                    print("skipping", yolo.layers[i].name)
    
    optimizer = tf.keras.optimizers.Adam()


    def train_step(image_data, target):
        with tf.GradientTape() as tape:
            pred_result = yolo(image_data, training=True)
            giou_loss=conf_loss=prob_loss=0

            # optimizing process
            grid = 3
            for i in range(grid):
                conv, pred = pred_result[i*2], pred_result[i*2+1]
                loss_items = compute_loss(pred, conv, *target[i], i, CLASSES=TRAIN_CLASSES)
                giou_loss += loss_items[0]
                conf_loss += loss_items[1]
                prob_loss += loss_items[2]

            total_loss = giou_loss + conf_loss + prob_loss

            gradients = tape.gradient(total_loss, yolo.trainable_variables)
            optimizer.apply_gradients(zip(gradients, yolo.trainable_variables))

            # about warmup: https://arxiv.org/pdf/1812.01187.pdf&usg=ALkJrhglKOPDjNt6SHGbphTHyMcT0cuMJg
            global_steps.assign_add(1)
            if global_steps < warmup_steps:# and not TRAIN_TRANSFER:
                lr = global_steps / warmup_steps * TRAIN_LR_INIT
            else:
                lr = TRAIN_LR_END + 0.5 * (TRAIN_LR_INIT - TRAIN_LR_END)*(
                    (1 + tf.cos((global_steps - warmup_steps) / (total_steps - warmup_steps) * np.pi)))
            optimizer.lr.assign(lr.numpy())
            
        return global_steps.numpy(), optimizer.lr.numpy(), giou_loss.numpy(), conf_loss.numpy(), prob_loss.numpy(), total_loss.numpy()

    def validate_step(image_data, target):
        with tf.GradientTape() as tape:
            pred_result = yolo(image_data, training=False)
            giou_loss=conf_loss=prob_loss=0

            # optimizing process
            grid = 3
            for i in range(grid):
                conv, pred = pred_result[i*2], pred_result[i*2+1]
                loss_items = compute_loss(pred, conv, *target[i], i, CLASSES=TRAIN_CLASSES)
                giou_loss += loss_items[0]
                conf_loss += loss_items[1]
                prob_loss += loss_items[2]

            total_loss = giou_loss + conf_loss + prob_loss
            
        return giou_loss.numpy(), conf_loss.numpy(), prob_loss.numpy(), total_loss.numpy()

    mAP_model = Create_Yolo(input_size=YOLO_INPUT_SIZE, CLASSES=TRAIN_CLASSES)  # create second model to measure mAP

    for epoch in range(TRAIN_EPOCHS):
        for image_data, target in trainset:
            results = train_step(image_data, target)
            cur_step = results[0] % steps_per_epoch
            print("epoch:{:2.0f} step:{:5.0f}/{}, lr:{:.6f}, giou_loss:{:7.2f}, conf_loss:{:7.2f}, prob_loss:{:7.2f}, total_loss:{:7.2f}"
                  .format(epoch+1, cur_step, steps_per_epoch, results[1], results[2], results[3], results[4], results[5]))

        if len(testset) == 0:
            print("configure TEST options to validate model")
            yolo.save_weights(YOLOV3_TRUCKLAB_SAVE_TRAINED_WEIGHTS)
            continue
        
        count, giou_val, conf_val, prob_val, total_val = 0., 0, 0, 0, 0
        for image_data, target in testset:
            results = validate_step(image_data, target)
            count += 1
            giou_val += results[0]
            conf_val += results[1]
            prob_val += results[2]
            total_val += results[3]
            
        print("\n\ngiou_val_loss:{:7.2f}, conf_val_loss:{:7.2f}, prob_val_loss:{:7.2f}, total_val_loss:{:7.2f}\n\n".
              format(giou_val/count, conf_val/count, prob_val/count, total_val/count))

        yolo.save_weights(YOLOV3_TRUCKLAB_SAVE_TRAINED_WEIGHTS)
        print(f"model saved after epoch: {epoch + 1}")

    # measure mAP of trained custom model
    try:
        mAP_model.load_weights(YOLOV3_TRUCKLAB_SAVE_TRAINED_WEIGHTS) # use keras weights
        get_mAP(mAP_model, testset, score_threshold=TEST_SCORE_THRESHOLD, iou_threshold=TEST_IOU_THRESHOLD)
    except UnboundLocalError:
        print("You don't have saved model weights to measure mAP, check TRAIN_SAVE_BEST_ONLY and TRAIN_SAVE_CHECKPOINT lines in configs.py")

if __name__ == '__main__':
    main()
