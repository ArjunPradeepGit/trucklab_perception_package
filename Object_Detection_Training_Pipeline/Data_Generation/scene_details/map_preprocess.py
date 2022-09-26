import cv2
import numpy as np
import pickle
import glob
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

import matplotlib.pyplot as plt
from shapely.geometry import Polygon

def read_map_boundary(img_read_path, img_write_path, map_details_path, scale, object_name):
    img = cv2.imread(img_read_path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img[img >= 200] = 255
    img[img < 200] = 0
    pt1 = np.min(np.where(img == 0), axis=1)
    pt2 = np.max(np.where(img == 0), axis=1)
    img = img[pt1[0]:pt2[0], pt1[1]:pt2[1]]
    dim = (int(img.shape[1] * scale[0]), int(img.shape[0] * scale[1]))
    img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
    cv2.imwrite(str(img_write_path), img)

    polygon_object = []
    max_range = int((dim[0] + dim[1])/2)
    xc, yc = int(dim[0]/2), int(dim[1]/2)
    for angle in np.linspace(0, 2 * np.pi, 360, False):
        x_max, y_max = (xc + max_range * np.cos(angle), yc - max_range * np.sin(angle))
        for i in range(100):
            u = i / 100
            x_check = int(x_max * u + xc * (1 - u))
            y_check = int(y_max * u + yc * (1 - u))
            if 0 < x_check < dim[0] and 0 < y_check < dim[1]:
                color = img[y_check, x_check, :]
                if (color[0], color[1], color[2]) == (0, 0, 0):
                    polygon_object.append((x_check, y_check))
                    break

    center_coords = [int(img.shape[1]/2), int(img.shape[0]/2)]
    img_size = [int(img.shape[1]), int(img.shape[0])]
    details = {'object_name': object_name,
               'center_coords': center_coords,
               'image_size': img_size,
               'polygon': polygon_object}
    with open(map_details_path, 'wb') as f:
        pickle.dump(details, f)

    polygon_boundary = Polygon(polygon_object)
    x1, y1 = polygon_boundary.exterior.xy
    plt.plot(x1, y1, c="blue")
    plt.gca().set_aspect('equal', adjustable='box')

    return img


def read_map_object(img_read_path, img_write_path, map_details_path, scale, object_name):
    img = cv2.imread(img_read_path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img[img >= 200] = 255
    img[img < 200] = 0
    pt1 = np.min(np.where(img == 0), axis=1)
    pt2 = np.max(np.where(img == 0), axis=1)
    img = img[pt1[0]:pt2[0], pt1[1]:pt2[1]]
    dim = (int(img.shape[1] * scale[0]), int(img.shape[0] * scale[1]))
    img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
    cv2.imwrite(str(img_write_path), img)
    polygon_object = [(0, 0), (img.shape[1], 0), (img.shape[1], img.shape[0]), (0, img.shape[0])]
    center_coords = [int(img.shape[1]/2), int(img.shape[0]/2)]
    img_size = [int(img.shape[1]), int(img.shape[0])]
    details = {'object_name': object_name,
               'center_coords': center_coords,
               'image_size': img_size,
               'polygon': polygon_object}
    with open(map_details_path, 'wb') as f:
        pickle.dump(details, f)

    return img




if __name__ == "__main__":

    show_image = False

    PATH_FOLDER = Path(Path.cwd() / "s2_map_drawn")
    PATH_FILES = Path(PATH_FOLDER / f"*.jpg")
    files_map_images = glob.glob(str(PATH_FILES))
    object_names = [Path(file).name.lstrip("map_").rstrip(".jpg") for file in files_map_images]
    paths_write_image = [Path(Path(file).parent.parent / "s3_map_for_scenes" / Path("map_" + object_names[ind] + ".jpg")) for ind, file in enumerate(files_map_images)]
    paths_write_csv = [Path(Path(file).parent.parent / "s3_map_for_scenes" / Path("map_details_" + object_names[ind] + ".csv")) for ind, file in enumerate(files_map_images)]

    map_width_boundary = 7  # m
    map_depth_boundary = 7  # m

    for ind, object_name in enumerate(object_names):
        print(f"Processing --> {object_name}")
        if object_name == "boundary":
            map_width = 7   # m
            map_depth = 7   # m
            scale = [1, 1]
            img = read_map_boundary(files_map_images[ind], paths_write_image[ind], paths_write_csv[ind], scale, object_name)
        else:
            map_width = 3  # m
            map_depth = 3  # m
            scale = [map_width / map_width_boundary, map_depth / map_depth_boundary]
            img = read_map_object(files_map_images[ind], paths_write_image[ind], paths_write_csv[ind], scale, object_name)

        if show_image:
            cv2.imshow("Image", img)
            plt.show()
            cv2.waitKey(0)

    print(f"Completed preprocessing maps!")
