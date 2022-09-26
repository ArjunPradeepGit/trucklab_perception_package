"""
About: To generate training data for object detection & localization
Author: Arjun Pradeep
"""


import cv2
import numpy as np
import pickle
import matplotlib.pyplot as plt
from random import random, randint
import shapely
import os
from shapely.geometry import Polygon, Point, mapping
import glob
from pathlib import Path


def initialize_boundary(ROOT_DIR):
    img_path = f"scene_details/s3_map_for_scenes/map_boundary.jpg"
    img_boundary_path = os.path.abspath(os.path.join(ROOT_DIR, img_path))
    details_path = f"scene_details/s3_map_for_scenes/map_details_boundary.csv"
    details_boundary_path = os.path.abspath(os.path.join(ROOT_DIR, details_path))
    img_boundary = cv2.imread(img_boundary_path)
    img_boundary = cv2.cvtColor(img_boundary, cv2.COLOR_BGR2RGB)
    with open(details_boundary_path, 'rb') as f:
        data = f.read()
    details_boundary = pickle.loads(data)
    return img_boundary, details_boundary

def initialize_object(object, img_objects, details_objects, ROOT_DIR):
    img_path = f"scene_details/s3_map_for_scenes/map_" + str(object) + ".jpg"
    img_object_path = os.path.abspath(os.path.join(ROOT_DIR, img_path))
    details_path = f"scene_details/s3_map_for_scenes/map_details_" + str(object) + ".csv"
    details_object_path = os.path.abspath(os.path.join(ROOT_DIR, details_path))
    img_object = cv2.imread(img_object_path)
    img_object = cv2.cvtColor(img_object, cv2.COLOR_BGR2RGB)
    img_objects.append(img_object)
    with open(details_object_path, 'rb') as f:
        data = f.read()
    details_objects.append(pickle.loads(data))
    return img_objects, details_objects

def generate_random_position(boundary_dict, object_dict, polygon_list):
    bool_within_boundary = False
    bool_object_intersect = True
    while not bool_within_boundary or bool_object_intersect:
        polygon_boundary = Polygon(boundary_dict['polygon'])
        polygon_current_object = Polygon(object_dict['polygon'])

        xc = randint(0, boundary_dict["image_size"][0])
        yc = randint(0, boundary_dict["image_size"][1])
        theta = 0.0 + random() * 360.0
        pose = [xc, yc, theta]

        rotated_polygon = shapely.affinity.rotate(polygon_current_object, theta, origin='center', use_radians=False)
        transformed_polygon = shapely.affinity.translate(rotated_polygon, xoff=xc - object_dict["center_coords"][0], yoff=yc - object_dict["center_coords"][1])

        bool_within_boundary = polygon_boundary.contains(transformed_polygon)

        if bool_within_boundary:
            bool_object_intersect = False
            for i in range(len(polygon_list)):
                bool_object_intersect = polygon_list[i].intersects(transformed_polygon)
                if bool_object_intersect:
                    break

    polygon_detail = transformed_polygon
    return pose, polygon_detail

def generate_random_lidar_point(boundary_dict, polygon_list):
    bool_within_boundary = False
    bool_object_intersect = True
    polygon_boundary = Polygon(boundary_dict["polygon"])
    delta_correction = 200
    while not bool_within_boundary or bool_object_intersect:
        xc = randint(0 + delta_correction, boundary_dict["image_size"][0] - delta_correction)
        yc = randint(0 + delta_correction, boundary_dict["image_size"][1] - delta_correction)
        theta = 0.0 + random() * 360.0
        pt = Point(xc, yc)
        bool_within_boundary = polygon_boundary.contains(pt)
        if bool_within_boundary:
            bool_object_intersect = False
            for i in range(len(polygon_list)):
                bool_object_intersect = polygon_list[i].contains(pt)
                if bool_object_intersect:
                    break
    return xc, yc, theta

def lidar_sense_scene(img_scene,
                      pt_lidar_scene, pose_angle_scene,
                      lidar_max_range, lidar_angle_limits, lidar_angle_inc, lidar_max_error,
                      scene_xy_to_px, scene_px_to_xy,
                      ogm_map_limits, ogm_grid_size, ogm_total_cols, ogm_total_rows,
                      ogm_img_size, ogm_grid_to_px):
    scene_lidar_points, scene_correct_lidar_points_wrt_lidar, scene_lidar_points_wrt_lidar = [], [], []
    real_lidar_points_x, real_lidar_points_y = [], []
    real_lidar_points_x_rotated, real_lidar_points_y_rotated = [], []
    real_lidar_points_x_transformed, real_lidar_points_y_transformed = [], []

    img_scene[img_scene >= 200] = 255
    img_scene[img_scene < 200] = 0
    img_scene_lidar = img_scene.copy()
    img_scene_lidar = cv2.circle(img_scene_lidar, (pt_lidar_scene[0], pt_lidar_scene[1]), radius=8, color=(255, 0, 0), thickness=-1)

    max_range = min(int((img_scene.shape[0] + img_scene.shape[1]) / 2), lidar_max_range * scene_xy_to_px[0])
    for angle in np.linspace((pose_angle_scene - lidar_angle_limits[0]), (pose_angle_scene + lidar_angle_limits[1]), int((lidar_angle_limits[0]+lidar_angle_limits[1])/lidar_angle_inc), False):
        x_max, y_max = (pt_lidar_scene[0] + max_range * np.cos(angle), pt_lidar_scene[1] + max_range * np.sin(angle))
        for i in range(1000):
            breaker = False
            u = i / 1000
            x_check = int(x_max * u + pt_lidar_scene[0] * (1 - u))
            y_check = int(y_max * u + pt_lidar_scene[1] * (1 - u))
            if 0 < x_check < img_scene.shape[0] and 0 < y_check < img_scene.shape[1]:
                color = img_scene[y_check, x_check, :]
                if (color[0], color[1], color[2]) == (0, 0, 0):
                    error = np.random.uniform(-0.5*lidar_max_error, 1.5*lidar_max_error) * scene_xy_to_px[0]
                    scene_lidar_points.append([x_check, y_check])
                    scene_correct_lidar_points_wrt_lidar.append([int(x_check - pt_lidar_scene[0]), int(y_check - pt_lidar_scene[1])])
                    scene_lidar_points_wrt_lidar.append([int(x_check - pt_lidar_scene[0] + error*np.cos(angle)), int(y_check - pt_lidar_scene[1] + error*np.sin(angle))])
                    img_scene_lidar = cv2.circle(img_scene_lidar, (int(x_check + error*np.cos(angle)), int(y_check + error*np.sin(angle))), radius=3, color=(0, 0, 255), thickness=-1)
                    breaker = True
                    break
            if breaker:
                break

    img_scene_w_lidar = cv2.circle(img_scene_lidar, (pt_lidar_scene[0], pt_lidar_scene[1]), radius=int(max_range), color=(0, 255, 0), thickness=2)
    img_scene_w_lidar = cv2.line(img_scene_w_lidar, (pt_lidar_scene[0], pt_lidar_scene[1]), (int(pt_lidar_scene[0]+(np.cos(pose_angle_scene - lidar_angle_limits[0]))*max_range), int(pt_lidar_scene[1]+(np.sin(pose_angle_scene - lidar_angle_limits[0]))*max_range)), (255, 0, 0), 3)
    img_scene_w_lidar = cv2.line(img_scene_w_lidar, (pt_lidar_scene[0], pt_lidar_scene[1]), (int(pt_lidar_scene[0] + (np.cos(pose_angle_scene + lidar_angle_limits[1]))*max_range), int(pt_lidar_scene[1] + (np.sin(pose_angle_scene + lidar_angle_limits[1]))*max_range)), (255, 0, 0), 3)

    # Converting from Scene --> XY real map coordinates
    for i, _ in enumerate(scene_lidar_points_wrt_lidar):
        real_lidar_points_x.append(scene_lidar_points_wrt_lidar[i][0] * scene_px_to_xy[0])
        real_lidar_points_y.append(scene_lidar_points_wrt_lidar[i][1] * scene_px_to_xy[1])

    # Rotating the xy lidar points to pose angle '-90' degrees (to face top)
    theta = - pose_angle_scene - np.pi / 2
    for i, _ in enumerate(real_lidar_points_x):
        real_lidar_points_x_rotated.append(real_lidar_points_x[i] * np.cos(theta) - real_lidar_points_y[i] * np.sin(theta))
        real_lidar_points_y_rotated.append(real_lidar_points_x[i] * np.sin(theta) + real_lidar_points_y[i] * np.cos(theta))

    # Translating the rotated points to center of OGM real map
    for i, _ in enumerate(real_lidar_points_x_rotated):
        if 0 < real_lidar_points_x_rotated[i] + ogm_map_limits[0][1]/2 < ogm_map_limits[0][1] and 0 < real_lidar_points_y_rotated[i] + ogm_map_limits[1][1]/2 < ogm_map_limits[1][1]:
            real_lidar_points_x_transformed.append(real_lidar_points_x_rotated[i] + ogm_map_limits[0][1]/2)
            real_lidar_points_y_transformed.append(real_lidar_points_y_rotated[i] + ogm_map_limits[1][1]/2)

    # Finding OGM values in OGM real map
    ogm = np.ones((ogm_total_rows, ogm_total_cols), dtype=int) * (-1)
    cols = [int(np.ceil(value / ogm_grid_size[0])) for value in real_lidar_points_x_transformed]
    rows = [int(np.ceil(value / ogm_grid_size[1])) for value in real_lidar_points_y_transformed]
    for i in range(len(rows)):
        ogm[rows[i], cols[i]] = 0

    # Generating OGM image
    ogm_img = np.ones((ogm_img_size[0], ogm_img_size[1], ogm_img_size[2]), np.uint8) * 255
    for row in range(ogm_total_rows):
        for col in range(ogm_total_cols):
            if ogm[row, col] == 0:
                ogm_img[(row) * ogm_grid_to_px[1]: (row + 1) * ogm_grid_to_px[1], (col) * ogm_grid_to_px[0]: (col + 1) * ogm_grid_to_px[0]] = 0

    return ogm_img, img_scene_w_lidar, scene_lidar_points



if __name__ == "__main__":

    ######### USER CHOICES ############
    # Training data generation
    choice = 2  # 1: for train data, 2: for test data

    # Training data generation
    total_num_images = 200
    min_num_objects = 10
    max_num_objects = 12



    ########## DEVELOPER CHOICES #######
    # Real Map properties
    real_map_x_limits = [0.0, 6.99]    # m
    real_map_y_limits = [0.0, 6.99]    # m

    # Scene Image properties
    scene_img_width = 800

    # OGM Map properties
    ogm_grid_size = [0.03, 0.03]       # m

    # OGM Image properties
    ogm_img_size = [416, 416, 3]
    grid_map_to_px = [2, 2]                 # [x, y] here means --> the above grid size (in meters) will correspond to 'x' by 'y' pixels in ogm-image

    # LiDAR properties
    lidar_max_range = 3.5   # m
    lidar_angle_low_limit  = 60 * np.pi / 180
    lidar_angle_high_limit = 60 * np.pi / 180
    lidar_angle_inc = 0.017501922  # radians
    lidar_max_error = 0.045  # m

    # While generating, user options:
    # Please make sure both are false for final data generation.
    show_generated_scenes = False
    show_lidar_location_on_ogm = False
    override_object_names = False


    ##################################################################################################################
    """
    Pre-processing data
    """
    # File path details

    PATH_FOLDER = Path(Path.cwd() / "scene_details"/ "s3_map_for_scenes")
    PATH_FILES = Path(PATH_FOLDER / f"*.jpg")
    files_map_images = glob.glob(str(PATH_FILES))
    object_names = [Path(file).name.lstrip("map_").rstrip(".jpg") for file in files_map_images]

    ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
    data_directory = os.path.abspath(os.path.join(ROOT_DIR, "generated_data"))
    if choice == 1: data_type = "Dataset_train"
    elif choice == 2: data_type = "Dataset_test"

    # Real Map
    real_map_limits = [real_map_x_limits, real_map_y_limits]

    # Scene Image
    scene_img_size = [int(scene_img_width), int(scene_img_width * (real_map_limits[1][1] - real_map_limits[1][0])/(real_map_limits[0][1] - real_map_limits[0][0])), 3]
    scene_x_to_px = scene_img_size[0] / (real_map_limits[0][1] - real_map_limits[0][0])
    scene_y_to_px = scene_img_size[1] / (real_map_limits[1][1] - real_map_limits[1][0])
    scene_xy_to_px = [scene_x_to_px, scene_y_to_px]
    scene_px_to_xy = [1 / value for value in scene_xy_to_px]

    # OGM Image
    ogm_x_to_px = grid_map_to_px[0] / ogm_grid_size[0]
    ogm_y_to_px = grid_map_to_px[1] / ogm_grid_size[1]
    ogm_xy_to_px = [ogm_x_to_px, ogm_y_to_px]
    ogm_px_to_xy = [1 / value for value in ogm_xy_to_px]
    ogm_map_limits = [[0, ogm_img_size[0] * ogm_px_to_xy[0]], [0, ogm_img_size[1] * ogm_px_to_xy[1]]]
    print("Range covered within ogm image: ", ogm_map_limits)
    ogm_grid_to_px = grid_map_to_px

    # OGM Map
    ogm_total_cols = int((ogm_map_limits[0][1] - ogm_map_limits[0][0]) / ogm_grid_size[0])
    ogm_total_rows = int((ogm_map_limits[1][1] - ogm_map_limits[1][0]) / ogm_grid_size[1])

    # LiDAR
    lidar_angle_limits = [lidar_angle_low_limit, lidar_angle_high_limit]


    ##################################################################################################################
    """
    Boundary & Objects Initialization
    """
    # Boundary --> Initialization
    img_boundary, details_boundary = initialize_boundary(ROOT_DIR=ROOT_DIR)

    # Objects --> Initialization --> OVERRIDE
    if override_object_names:
        object_names = ["tire", "semitrailer"]

    img_objects, details_objects = [], []
    if "boundary" in object_names:
        object_names.remove("boundary")
    print(object_names)
    for object_name in object_names:
        img_objects, details_objects = initialize_object(object=object_name, img_objects=img_objects, details_objects=details_objects, ROOT_DIR=ROOT_DIR)


    ##################################################################################################################
    """
    Data Generation
    """
    total_labels = []
    labels = ""

    # for img_num in range(1, total_num_images + 1):
    img_num = 1
    while img_num < total_num_images + 1:
        print("Image number: ", img_num)

        # Fixing image paths
        img_path = data_directory + "\\" + data_type + f"\{img_num:06}" + ".jpg"
        img_path_saveas = data_type + f"\{img_num:06}" + ".jpg"

        # Some initializations
        polygon_list = []
        object_num_list = []
        img_scene = img_boundary.copy()
        num_objects_in_scene = randint(min_num_objects, max_num_objects)
        bool_scene_good = True

        # Place random objects at randomly generated poses
        for i in range(num_objects_in_scene):
            # Generate random poses for the objects
            object_num = randint(1, len(object_names))
            pose, polygon_detail = generate_random_position(details_boundary, details_objects[object_num-1], polygon_list)
            polygon_list.append(polygon_detail)
            object_num_list.append(object_num-1)

            # Transform the images of objects to the generated poses
            M = np.float32([[1, 0, pose[0] - details_objects[object_num-1]["center_coords"][0]], [0, 1, pose[1] - details_objects[object_num-1]["center_coords"][1]]])
            img_shifted = cv2.warpAffine(img_objects[object_num-1].copy(), M, (img_boundary.shape[1], img_boundary.shape[0]), flags=cv2.INTER_LANCZOS4, borderValue=(255, 255, 255))
            center_of_rot = (pose[0], pose[1])
            M = cv2.getRotationMatrix2D(center_of_rot, -pose[2], 1.0)
            img_transformed = cv2.warpAffine(img_shifted, M, (img_boundary.shape[1], img_boundary.shape[0]), flags=cv2.INTER_LANCZOS4, borderValue=(255, 255, 255))

            # Reflect placed objects on to the scene image
            img_scene[img_transformed <= 200] = 0

        pose_lidar = generate_random_lidar_point(details_boundary, polygon_list)
        pt_lidar_scene = pose_lidar[0], pose_lidar[1]
        pose_angle_scene = pose_lidar[2] * np.pi / 180

        img_ogm, img_scene_w_lidar, scene_lidar_points = lidar_sense_scene(img_scene=img_scene,
                                                                  pt_lidar_scene=pt_lidar_scene, pose_angle_scene=pose_angle_scene,
                                                                  lidar_max_range=lidar_max_range, lidar_angle_limits=lidar_angle_limits, lidar_angle_inc=lidar_angle_inc, lidar_max_error=lidar_max_error,
                                                                  scene_xy_to_px=scene_xy_to_px, scene_px_to_xy=scene_px_to_xy,
                                                                  ogm_map_limits=ogm_map_limits, ogm_grid_size=ogm_grid_size, ogm_total_cols=ogm_total_cols, ogm_total_rows=ogm_total_rows,
                                                                  ogm_img_size=ogm_img_size, ogm_grid_to_px=ogm_grid_to_px)

        if show_lidar_location_on_ogm:
            img_ogm = cv2.circle(img_ogm, (int(ogm_img_size[0]/2), int(ogm_img_size[1]/2)), radius=3, color=(0, 255, 0), thickness=2)


        # Find which polygons were seen by the LiDAR
        polygon_activation_indices = []
        for pnt in scene_lidar_points:
            pt = Point(pnt[0], pnt[1])
            for i in range(len(polygon_list)):
                if polygon_list[i].contains(pt):
                    polygon_activation_indices.append(i)
        polygon_activation_indices = list(set(polygon_activation_indices))

        # Reading and generating label information (info of bounding boxes)
        labels_str = str(img_path_saveas) + str(" ")
        for i in polygon_activation_indices:
            poly_mapped = mapping(polygon_list[i])
            poly_coordinates = poly_mapped["coordinates"][0]

            x_vals = [row[0] for row in poly_coordinates]
            y_vals = [row[1] for row in poly_coordinates]

            x_scaled = [(val - pose_lidar[0]) * (ogm_xy_to_px[0] / scene_xy_to_px[0]) for val in x_vals]
            y_scaled = [(val - pose_lidar[1]) * (ogm_xy_to_px[1] / scene_xy_to_px[1]) for val in y_vals]

            x_vals_lidar = []
            y_vals_lidar = []
            theta = - pose_angle_scene - np.pi / 2
            for j in range(len(x_scaled)):
                x_vals_lidar.append(x_scaled[j] * np.cos(theta) - y_scaled[j] * np.sin(theta) + ogm_img_size[0] / 2)
                y_vals_lidar.append(x_scaled[j] * np.sin(theta) + y_scaled[j] * np.cos(theta) + ogm_img_size[1] / 2)

            min_x = int(min(x_vals_lidar))
            min_y = int(min(y_vals_lidar))
            max_x = int(max(x_vals_lidar))
            max_y = int(max(y_vals_lidar))

            if 0 < min_x < ogm_img_size[0] and 0 < min_y < ogm_img_size[1] and 0 < max_x < ogm_img_size[0] and 0 < max_y < ogm_img_size[1]:
                bool_scene_good = True
                label = [min_x, min_y, max_x, max_y, object_num_list[i]]
                labels = labels + str(label).strip("[]").replace(" ", "") + str(" ")
            else:
                bool_scene_good = False
                break

        if bool_scene_good:
            labels_str = labels_str + labels
            total_labels.append(labels_str)
            labels = ""
            img_num = img_num + 1

            cv2.imwrite(img_path, img_ogm)
        else:
            labels = ""
            bool_scene_good = True

        if show_generated_scenes:
            cv2.imshow("Scene generated", img_scene)
            cv2.imshow("LiDAR detections", img_scene_w_lidar)
            cv2.imshow("OGM image", img_ogm)
            cv2.waitKey(0)

    with open(data_directory + "\\" + data_type + ".txt", 'w') as f:
        for line in total_labels:
            f.write(line + '\n')