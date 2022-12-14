import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2
import os

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
data_directory = os.path.abspath(os.path.join(ROOT_DIR, "generated_data"))
print(data_directory)

data_type = "Dataset_train"
# data_type = "Dataset_test"

file_path = data_directory + "\\" + data_type + ".txt"
object_names = ["semitrailer", "tire", "tractor"]

with open(file_path) as file:
    lines = file.readlines()
    for line in lines:
        line = line.strip("\n")
        line = line.strip(" ")
        line = line.split(" ")
        img_path = data_directory + "\\" + line[0].strip("\n")
        print(img_path)
        im = cv2.imread(img_path)
        values = line[1:]
        if len(values) != 0:
            for value in values:
                boxes = value.split(",")
                print(boxes)
                box = [int(boxes[0]), int(boxes[1]), int(boxes[2]), int(boxes[3])]
                object_name = object_names[int(boxes[4])]
                im = cv2.rectangle(im, box[0:2], box[2:4], (255, 0, 0), 2)
                im = cv2.putText(im, object_name, (box[0], box[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1, cv2.LINE_AA)

        cv2.imshow("image", im)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        im = []






