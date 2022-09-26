#!/usr/bin/env python3

'''
Script: Generate map image for object
Author: Arjun Pradeep
'''


import numpy as np
import cv2
import pandas as pd
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull, convex_hull_plot_2d


def main():
    ROOT_DIR = Path(Path.cwd() / "catkin_ws" / "src" / "perception_pkg" / "src" / "scripts")
    PATH_FILE_LIDAR_CAPTURES = Path(ROOT_DIR / "data" / "data_captured_for_training_data" / "processed_data_files" / "tire.csv")

    df_lidar_captures = pd.read_csv(PATH_FILE_LIDAR_CAPTURES)
    rad_of_interest = 1.0

    x_lidar, y_lidar = [], []

    for index, _ in df_lidar_captures.iterrows():
        for i in range(360):
            try:
                x_current = float(df_lidar_captures[f"ranges_{i}"].loc[index].strip("[]")) * np.cos(float(df_lidar_captures[f"angles_{i}"].loc[index].strip("[]")))
                y_current = float(df_lidar_captures[f"ranges_{i}"].loc[index].strip("[]")) * np.sin(float(df_lidar_captures[f"angles_{i}"].loc[index].strip("[]")))

                if x_current < rad_of_interest and x_current > -rad_of_interest and y_current < rad_of_interest:
                    x_trans = float(df_lidar_captures["x"].loc[index])
                    y_trans = float(df_lidar_captures["y"].loc[index])
                    theta_trans = float(df_lidar_captures["theta"].loc[index]) - np.pi/2

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


    # Plots
    plt.figure(1)
    plt.plot(x_lidar, y_lidar, 'bo')
    plt.axis("equal")

    plt.figure(2)
    plt.plot(pts[:, 0], pts[:, 1], 'o')
    for simplex in hull.simplices:
        plt.plot(pts[simplex, 0], pts[simplex, 1], 'k-')

    plt.axis("equal")
    plt.show()




if __name__ == "__main__":
    main()
