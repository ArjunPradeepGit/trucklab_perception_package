
import cv2
import time
import random
import colorsys
import numpy as np
import tensorflow as tf
import sys
from pathlib import Path

ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
PATH_OBJDET_FILES = Path(ROOT_DIR / "src" / "features" / "object_detection" / "1_sensor" / "lidar" / "trucklab_object_detection")
sys.path.insert(0, str(PATH_OBJDET_FILES))
from configs import *
from yolov3 import *

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'


def Load_Yolo_model():
    # print("Loading custom weights from:", YOLO_V3_MODEL)
    yolo = Create_Yolo(input_size=YOLO_INPUT_SIZE, CLASSES=YOLO_TRUCKLAB_CLASSES)
    yolo.load_weights(YOLO_V3_MODEL)  # use custom weights
    return yolo

def image_preprocess(image, target_size, gt_boxes=None):
    ih, iw    = target_size
    h,  w, _  = image.shape

    scale = min(iw/w, ih/h)
    nw, nh  = int(scale * w), int(scale * h)
    image_resized = cv2.resize(image, (nw, nh))

    image_paded = np.full(shape=[ih, iw, 3], fill_value=128.0)
    dw, dh = (iw - nw) // 2, (ih-nh) // 2
    image_paded[dh:nh+dh, dw:nw+dw, :] = image_resized
    image_paded = image_paded / 255.

    if gt_boxes is None:
        return image_paded

    else:
        gt_boxes[:, [0, 2]] = gt_boxes[:, [0, 2]] * scale + dw
        gt_boxes[:, [1, 3]] = gt_boxes[:, [1, 3]] * scale + dh
        return image_paded, gt_boxes

def postprocess_boxes(pred_bbox, original_image, input_size, score_threshold):
    valid_scale=[0, np.inf]
    pred_bbox = np.array(pred_bbox)

    pred_xywh = pred_bbox[:, 0:4]
    pred_conf = pred_bbox[:, 4]
    pred_prob = pred_bbox[:, 5:]

    # 1. (x, y, w, h) --> (xmin, ymin, xmax, ymax)
    pred_coor = np.concatenate([pred_xywh[:, :2] - pred_xywh[:, 2:] * 0.5,
                                pred_xywh[:, :2] + pred_xywh[:, 2:] * 0.5], axis=-1)
    # 2. (xmin, ymin, xmax, ymax) -> (xmin_org, ymin_org, xmax_org, ymax_org)
    org_h, org_w = original_image.shape[:2]
    resize_ratio = min(input_size / org_w, input_size / org_h)

    dw = (input_size - resize_ratio * org_w) / 2
    dh = (input_size - resize_ratio * org_h) / 2

    pred_coor[:, 0::2] = 1.0 * (pred_coor[:, 0::2] - dw) / resize_ratio
    pred_coor[:, 1::2] = 1.0 * (pred_coor[:, 1::2] - dh) / resize_ratio

    # 3. clip some boxes those are out of range
    pred_coor = np.concatenate([np.maximum(pred_coor[:, :2], [0, 0]),
                                np.minimum(pred_coor[:, 2:], [org_w - 1, org_h - 1])], axis=-1)
    invalid_mask = np.logical_or((pred_coor[:, 0] > pred_coor[:, 2]), (pred_coor[:, 1] > pred_coor[:, 3]))
    pred_coor[invalid_mask] = 0

    # 4. discard some invalid boxes
    bboxes_scale = np.sqrt(np.multiply.reduce(pred_coor[:, 2:4] - pred_coor[:, 0:2], axis=-1))
    scale_mask = np.logical_and((valid_scale[0] < bboxes_scale), (bboxes_scale < valid_scale[1]))

    # 5. discard boxes with low scores
    classes = np.argmax(pred_prob, axis=-1)
    scores = pred_conf * pred_prob[np.arange(len(pred_coor)), classes]
    score_mask = scores > score_threshold
    mask = np.logical_and(scale_mask, score_mask)
    coors, scores, classes = pred_coor[mask], scores[mask], classes[mask]

    return np.concatenate([coors, scores[:, np.newaxis], classes[:, np.newaxis]], axis=-1)

def bboxes_iou(boxes1, boxes2):
    boxes1 = np.array(boxes1)
    boxes2 = np.array(boxes2)

    boxes1_area = (boxes1[..., 2] - boxes1[..., 0]) * (boxes1[..., 3] - boxes1[..., 1])
    boxes2_area = (boxes2[..., 2] - boxes2[..., 0]) * (boxes2[..., 3] - boxes2[..., 1])

    left_up       = np.maximum(boxes1[..., :2], boxes2[..., :2])
    right_down    = np.minimum(boxes1[..., 2:], boxes2[..., 2:])

    inter_section = np.maximum(right_down - left_up, 0.0)
    inter_area    = inter_section[..., 0] * inter_section[..., 1]
    union_area    = boxes1_area + boxes2_area - inter_area
    ious          = np.maximum(1.0 * inter_area / union_area, np.finfo(np.float32).eps)

    return ious

def nms(bboxes, iou_threshold, sigma=0.3, method='nms'):
    classes_in_img = list(set(bboxes[:, 5]))
    best_bboxes = []

    for cls in classes_in_img:
        cls_mask = (bboxes[:, 5] == cls)
        cls_bboxes = bboxes[cls_mask]
        # Process 1: Determine whether the number of bounding boxes is greater than 0
        while len(cls_bboxes) > 0:
            # Process 2: Select the bounding box with the highest score according to socre order A
            max_ind = np.argmax(cls_bboxes[:, 4])
            best_bbox = cls_bboxes[max_ind]
            best_bboxes.append(best_bbox)
            cls_bboxes = np.concatenate([cls_bboxes[: max_ind], cls_bboxes[max_ind + 1:]])
            # Process 3: Calculate this bounding box A and
            # Remain all iou of the bounding box and remove those bounding boxes whose iou value is higher than the threshold
            iou = bboxes_iou(best_bbox[np.newaxis, :4], cls_bboxes[:, :4])
            weight = np.ones((len(iou),), dtype=np.float32)

            assert method in ['nms', 'soft-nms']

            if method == 'nms':
                iou_mask = iou > iou_threshold
                weight[iou_mask] = 0.0

            if method == 'soft-nms':
                weight = np.exp(-(1.0 * iou ** 2 / sigma))

            cls_bboxes[:, 4] = cls_bboxes[:, 4] * weight
            score_mask = cls_bboxes[:, 4] > 0.
            cls_bboxes = cls_bboxes[score_mask]

    return best_bboxes

def draw_bbox(image, bboxes_processed, x_detections_global, y_detections_global, x_ego, y_ego, ogm_px_to_xy, CLASSES=YOLO_TRUCKLAB_CLASSES, show_label=True, show_confidence = True, Text_colors=(0,0,0), rectangle_colors=''):
    NUM_CLASS = read_class_names(CLASSES)
    num_classes = len(NUM_CLASS)
    image_h, image_w, _ = image.shape
    hsv_tuples = [(1.0 * x / num_classes, 1., 1.) for x in range(num_classes)]
    #print("hsv_tuples", hsv_tuples)
    colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
    colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), colors))

    random.seed(0)
    random.shuffle(colors)
    random.seed(None)

    # put ego vehicle information
    label_ego = f"Ego Location: ({x_ego:.2f}, {y_ego:.2f})m"
    x_ego_img = int(YOLO_INPUT_SIZE / 2)
    y_ego_img = int(YOLO_INPUT_SIZE / 2)
    bbox_thick = int(0.6 * (image_h + image_w) / 1000)
    if bbox_thick < 1: bbox_thick = 1
    fontScale = 0.5 * bbox_thick
    cv2.circle(image, (x_ego_img, y_ego_img), 2, (0, 255, 0), -1)
    cv2.putText(image, label_ego, (x_ego_img, y_ego_img + 12), cv2.FONT_HERSHEY_COMPLEX_SMALL,
                fontScale, Text_colors, bbox_thick, lineType=cv2.LINE_AA)

    print("\n\n")
    for i, bbox in enumerate(bboxes_processed):
        score = bbox[4]
        # if score > CONFIDENCE_THRESHOLD:
        coor = np.array(bbox[:4], dtype=np.int32)
        class_ind = int(bbox[5])
        bbox_color = rectangle_colors if rectangle_colors != '' else colors[class_ind]
        bbox_thick = int(0.6 * (image_h + image_w) / 1000)
        if bbox_thick < 1: bbox_thick = 1
        fontScale = 0.5 * bbox_thick
        (x1, y1), (x2, y2) = (coor[0], coor[1]), (coor[2], coor[3])
        xc = (x1 + x2) / 2
        yc = (y1 + y2) / 2

        # x_from_ego = (xc - YOLO_INPUT_SIZE / 2) * ogm_px_to_xy[0]
        # y_from_ego = (yc - YOLO_INPUT_SIZE / 2) * ogm_px_to_xy[1] * (-1)


        # put object rectangle
        cv2.rectangle(image, (x1, y1), (x2, y2), bbox_color, bbox_thick*2)

        if show_label:
            # get text label
            score_str = "score: {:.2f}".format(score) if show_confidence else ""

            try:
                print(i)
                print(x_detections_global)
                x_global_value = x_detections_global[i]
                y_global_value = y_detections_global[i]
                # label = "{}".format(NUM_CLASS[class_ind]) + score_str
                label_1 = f"{NUM_CLASS[class_ind]} ({score_str})" if show_confidence else f"{NUM_CLASS[class_ind]}"
                label_2 = f"Location: ({x_global_value:.2f}, {y_global_value:.2f})m"
            except KeyError:
                print("You received KeyError, this might be that you are trying to use yolo original weights")
                print("while using custom classes, if using custom model in configs.py set YOLO_CUSTOM_WEIGHTS = True")

            # get text size
            (text_width, text_height), baseline = cv2.getTextSize(label_1, cv2.FONT_HERSHEY_COMPLEX_SMALL,
                                                                  fontScale, thickness=bbox_thick)
            # # put filled text rectangle
            # cv2.rectangle(image, (x1, y1), (x1 + text_width, y1 - text_height - baseline), bbox_color, thickness=cv2.FILLED)

            # put text above rectangle
            cv2.putText(image, label_1, (x1, y1 - 24), cv2.FONT_HERSHEY_COMPLEX_SMALL,
                        fontScale, Text_colors, bbox_thick, lineType=cv2.LINE_AA)
            cv2.putText(image, label_2, (x1, y1 - 8), cv2.FONT_HERSHEY_COMPLEX_SMALL,
                        fontScale, Text_colors, bbox_thick, lineType=cv2.LINE_AA)

    return image

def detect_image(model, input_img, ogm_px_to_xy, input_size=416, CLASSES=YOLO_TRUCKLAB_CLASSES, score_threshold=0.3, iou_threshold=0.45, rectangle_colors=(255, 0, 0), show_plot=False):
    original_image = input_img
    original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
    original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)

    image_data = image_preprocess(np.copy(original_image), [input_size, input_size])
    image_data = image_data[np.newaxis, ...].astype(np.float32)

    pred_bbox = model.predict(image_data)

    pred_bbox = [tf.reshape(x, (-1, tf.shape(x)[-1])) for x in pred_bbox]
    pred_bbox = tf.concat(pred_bbox, axis=0)

    bboxes = postprocess_boxes(pred_bbox, original_image, input_size, score_threshold)
    bboxes = nms(bboxes, iou_threshold, method='nms')

    detections = []
    detections = np.empty(shape=[0, 5])
    bboxes_processed = []
    # NUM_CLASS = read_class_names(CLASSES)
    for i, bbox in enumerate(bboxes):
        score = bbox[4]
        if score > CONFIDENCE_THRESHOLD:
            coor = np.array(bbox[:4], dtype=np.int32)
            class_num = bbox[5]
            xc = (coor[0] + coor[2]) / 2
            yc = (coor[1] + coor[3]) / 2
            x_from_ego = (xc - YOLO_INPUT_SIZE / 2) * ogm_px_to_xy[0]
            y_from_ego = (yc - YOLO_INPUT_SIZE / 2) * ogm_px_to_xy[1] * (-1)
            detections = np.concatenate([detections, np.reshape([int(i+1), int(class_num), float(x_from_ego), float(y_from_ego), float(score)], (1, 5))])  # order of info --> code ICXYS
            bboxes_processed.append(bbox)

    # if show_plot:
    #     image = draw_bbox(original_image, bboxes, ogm_px_to_xy=ogm_px_to_xy, CLASSES=CLASSES, rectangle_colors=rectangle_colors)
    # else:
    #     image = []

    return detections, original_image, bboxes_processed, ogm_px_to_xy, CLASSES, rectangle_colors


