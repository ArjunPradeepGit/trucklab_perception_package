import argparse
import cv2
import numpy as np
import glob
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

def draw_grid(img, pxstep=50, grid_size=1, line_color=(230, 230, 230), thickness=1, type_=cv2.LINE_AA):
    x = pxstep
    y = pxstep
    while x < img.shape[1]:
        cv2.line(img, (x, 0), (x, img.shape[0]), color=line_color, lineType=type_, thickness=thickness)
        x += pxstep

    while y < img.shape[0]:
        cv2.line(img, (0, y), (img.shape[1], y), color=line_color, lineType=type_, thickness=thickness)
        y += pxstep

    position = (img.shape[0]-150, 25)
    cv2.putText(
        img,  # numpy array on which text is written
        f"Grid size = {grid_size} m",  # text
        position,  # position at which writing has to start
        cv2.FONT_HERSHEY_SIMPLEX,  # font family
        0.5,  # font size
        line_color)  # font color

    return img

def create_map_img(width, depth, grid_pix, grid_size):
    im_size = (width, depth, 3)
    img = np.zeros(im_size, np.uint8) + 255
    img = draw_grid(img, grid_pix, grid_size)
    return img

def convert_pts_to_image(pts, img_width, img_height, map_to_img_ratio):
    x_mean = np.mean(pts[:, 0])
    y_mean = np.mean(pts[:, 1])
    img_pts = [0, 0]
    for pt in pts:
        img_pt = [int((pt[0] - x_mean) * map_to_img_ratio[0] + img_width/2), int((pt[1] - y_mean) * map_to_img_ratio[1] + img_height/2)]
        img_pts = np.vstack([img_pts, img_pt])
    img_pts = img_pts[1:, :]
    return img_pts

def main(args):
    show_process = True
    PATH_FOLDER = Path(Path.cwd() / "lidar_capture_data_to_generate_map")
    PATH_FILES = Path(PATH_FOLDER / f"*.csv")
    files_lidar = glob.glob(str(PATH_FILES))
    object_names = [Path(file).name.rstrip(".csv") for file in files_lidar]
    paths_write = [Path(Path(file).parent.parent / "s2_map_drawn" / Path("map_" + object_names[ind] + ".jpg")) for ind, file in enumerate(files_lidar)]
    file_class_name = Path(Path.cwd().parent / "generated_data" / "Dataset_names.txt")

    # Writing class names
    f = open(file_class_name, "w")
    for object_name in object_names:
        if object_name != "boundary":
            f.write(object_name + "\n")
    f.close()

    for ind, object_name in enumerate(object_names):
        if object_name == "boundary":
            map_width = 7  # m
            map_depth = 7  # m
        else:
            map_width = 3  # m
            map_depth = 3  # m

        grid_size = 1  # m

        """
        Image Properties
        """
        img_width = 800  # pixels
        img_height = int(map_depth / map_width * img_width)

        map_to_img_ratio = [img_width / map_width, img_height / map_depth]
        grid_px = int(grid_size * map_to_img_ratio[0])
        img = create_map_img(img_height, img_width, grid_px, grid_size)

        df_lidar_captures = pd.read_csv(files_lidar[ind])
        rad_of_interest = args.rad_of_interest

        x_lidar, y_lidar = [], []
        for index, _ in df_lidar_captures.iterrows():
            for i in range(360):
                try:
                    x_current = float(df_lidar_captures[f"ranges_{i}"].loc[index].strip("[]")) * np.cos(
                        float(df_lidar_captures[f"angles_{i}"].loc[index].strip("[]")))
                    y_current = float(df_lidar_captures[f"ranges_{i}"].loc[index].strip("[]")) * np.sin(
                        float(df_lidar_captures[f"angles_{i}"].loc[index].strip("[]")))

                    if x_current < rad_of_interest and x_current > -rad_of_interest and y_current < rad_of_interest:
                        x_trans = float(df_lidar_captures["x"].loc[index])
                        y_trans = float(df_lidar_captures["y"].loc[index])
                        theta_trans = float(df_lidar_captures["theta"].loc[index]) - np.pi / 2

                        x_transformed = x_trans + x_current * np.cos(theta_trans) - y_current * np.sin(theta_trans)
                        y_transformed = y_trans + x_current * np.sin(theta_trans) + y_current * np.cos(theta_trans)

                        x_lidar.append(x_transformed)
                        y_lidar.append(y_transformed)

                except:
                    continue

        x_lidar = np.array(x_lidar)
        y_lidar = np.array(y_lidar)

        pts = [0, 0]
        for x, y in zip(x_lidar, y_lidar):
            pts = np.vstack([pts, [x, y]])
        pts = pts[1:, :]
        hull = ConvexHull(pts)

        pts_convex = [0, 0]
        for vertex in hull.vertices:
            pt_convex = [pts[vertex, 0], pts[vertex, 1]]
            pts_convex = np.vstack([pts_convex, pt_convex])
        pts_convex = pts_convex[1:, :]

        img_pts = convert_pts_to_image(pts=pts_convex, img_width=img_width, img_height=img_height,
                                       map_to_img_ratio=map_to_img_ratio)
        for index, _ in enumerate(img_pts):
            cv2.line(img, (img_pts[index][0], img_pts[index][1]), (img_pts[index - 1][0], img_pts[index - 1][1]),
                     color=(0, 0, 0), lineType=cv2.LINE_AA, thickness=2)

        cv2.imwrite(str(paths_write[ind]), img)
        print(f"Map for {object_name} --> successfully created")

        if show_process:
            plt.figure(1)
            plt.plot(x_lidar, y_lidar, 'b+')
            plt.plot(pts_convex[:, 0], pts_convex[:, 1], '-k')
            plt.plot([pts_convex[-1, 0], pts_convex[0, 0]], [pts_convex[-1, 1], pts_convex[0, 1]], '-k')
            plt.axis("equal")
            plt.show()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="create map format script")
    parser.add_argument('-rad_of_interest', '--rad_of_interest', help='Provide the radius of interest of lidar captures', type=float, default=1.5)
    args = parser.parse_args()
    main(args)


