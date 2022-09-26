#!/usr/bin/env python3

'''
Script: Path trial for developers
Author: Arjun Pradeep
'''


import argparse
from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt

def plot_xy_inversed(pts_smooth, pts_nodes, plot_type="data", fig_num=1):
    x_node, y_node = [], []
    for item in pts_nodes:
        x_node.append(-item[0])
        y_node.append(-item[1])

    x_smooth, y_smooth = [], []
    for item in pts_smooth:
        x_smooth.append(-item[0])
        y_smooth.append(-item[1])

    plt.figure(fig_num)
    if plot_type == "border":
        plt.plot(x_node, y_node, 'bo', x_smooth, y_smooth, 'b-')
    else:
        plt.plot(x_node, y_node, 'go', x_smooth, y_smooth, 'g-')

def add_pts_to_nodes(nodes, num=5):
    x_values, y_values = [], []
    for i in range(len(nodes)-1):
        x_values = np.concatenate([x_values[:-1], np.linspace(nodes[i, 0], nodes[i + 1, 0], num)])
        y_values = np.concatenate([y_values[:-1], np.linspace(nodes[i, 1], nodes[i + 1, 1], num)])
    pts = np.vstack((x_values, y_values)).T
    return pts

def b_spline_from_pts(nodes):
    pts_nodes = add_pts_to_nodes(nodes, num=10)
    x = pts_nodes[:, 0]
    y = pts_nodes[:, 1]
    pts_nodes = [(pt_node[0], pt_node[1]) for pt_node in pts_nodes]

    tck, *u = interpolate.splprep([x, y], k=3, s=0.01)
    x_smooth, y_smooth = interpolate.splev(np.linspace(0, 1, 800), tck, der=0)

    pts_path = []
    for x_value, y_value in zip(x_smooth, y_smooth):
        pts_path.append((x_value, y_value))

    return pts_path, pts_nodes


def main(args):
    border_nodes = np.array([[0.0, 0.0], [0.0, 6.5], [6.5, 6.5], [6.5, 0.0], [0.0, 0.0]])
    border_pts_path, border_pts_nodes = b_spline_from_pts(border_nodes)
    plot_xy_inversed(pts_smooth=border_pts_path, pts_nodes=border_pts_nodes, plot_type="border", fig_num=1)

    if args.choice == 1:
        global_nodes = np.array(
            [[5.386, 5.217], [5.905, 4.533], [6.000, 3.993], [6.000, 2.558], [5.130, 2.487],
             [1.659, 2.487], [1.301, 2.558], [1.301, 4.835], [2.041, 5.403], [2.587, 5.952], [3.348, 6.136],
             [4.542, 6.165], [5.386, 5.217]])
    elif args.choice == 2:
        global_nodes = np.array([[5.608, 4.986], [5.608, 1.794], [1.608, 1.794], [1.608, 4.986], [2.608, 4.986], [3.608, 4.986], [5.608, 4.986]])
    else:
        raise ValueError("Choice value provided do not match with the already fed choices for path! ")
    pts_path, pts_nodes = b_spline_from_pts(global_nodes)
    plot_xy_inversed(pts_smooth=pts_path, pts_nodes=pts_nodes, plot_type=f"path-choice-{args.choice}", fig_num=1)

    plt.legend(["border", "border-spline", "path", "path-spline"])
    plt.axis('equal')
    plt.title("Path generated - XY axes inversed for visualization")
    plt.xlabel("X - global (m)")
    plt.ylabel("Y - global (m)")
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="lidar object detection node script")
    parser.add_argument('-choice', '--choice', help='Provide choice of path', type=int, default=1)
    args = parser.parse_args()
    main(args)



