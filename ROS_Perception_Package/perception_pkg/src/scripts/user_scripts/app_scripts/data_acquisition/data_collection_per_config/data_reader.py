import bagpy
from bagpy import bagreader
import pandas as pd

b = bagreader('/home/emeinder/catkin_ws/src/perception_pkg/src/scripts/data/data_collected_per_config/collected_bags/bag_lidar_static.bag')

# replace the topic name as per your need
LASER_MSG = b.message_by_topic('/DC_1/TRACTOR_1/LIDAR_1/lidarscan')
df_laser = pd.read_csv(LASER_MSG)

OPTI_MSG = b.message_by_topic('/DC_1/TRACTOR_1/optitrack')
df_opti = pd.read_csv(OPTI_MSG)

print(df_laser)
print(df_opti)
