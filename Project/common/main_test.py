import os
import sys
import time
import serial
import math
import numpy
from Thymio import Thymio
import robot
import vision
import four_point_transform
import map_creation
import cv2
import global_nav_new
#%%
# Adding the src folder in the current directory as it contains the script
# with the Thymio class
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))

th = Thymio.serial(port="/dev/cu.usbmodem14101", refreshing_rate=0.1)

#%%image
distance_two_features_m = 0.1
thymio_clearance_m = 0.2
filename = 'media/thymio_square.png'
image = cv2.imread(filename, cv2.IMREAD_COLOR)
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

vis = vision(camera_ip='http://192.168.100.16:8080/video', plot=True)
warped = vis.warp_map(image) # or warped = vis.warp_map(vis.get_camera_image())

# Load the feature images
feature_image_1 = cv2.imread('media/thymio_star.png')
feature_image_2 = cv2.imread('media/thymio_thunder.png')
# Convert the features images to RGB
feature_image_1 = cv2.cvtColor(feature_image_1, cv2.COLOR_BGR2RGB)
feature_image_2 = cv2.cvtColor(feature_image_2, cv2.COLOR_BGR2RGB)

feature_color_1 = [0x41, 0x57, 0x33]
feature_color_2 = [0x33, 0x5D, 0x19]
thymio_x_m, thymio_y_m, thymio_theta_rad, scaling_px_2_m = vis.locate_thymio(warped, feature_image_1, feature_color_1, feature_image_2, feature_color_2, distance_two_features_m)

obstacle_map = vis.create_obstacle_map(warped)
thymio_clearance_px = int(thymio_clearance_m / scaling_px_2_m)
dilated_obstacle_map = vis.dilate_obstacle_map(obstacle_map, thymio_clearance_px)

target_color = [0x86, 0x33, 0x26]
target_position_px = vis.locate_target_px(warped, target_color)
target_position_m = target_position_px * scaling_px_2_m

obstacle_vertices_m = vis.get_obstacle_vertices(dilated_obstacle_map, scaling_px_2_m)
   
#%%
x_0 = thymio_x_m
y_0 = thymio_y_m
theta_0 = thymio_theta_rad
sp = global_nav_new.build_visibility_graph(obstacle_vertices_m,[x_0,y_0],target_position_m )
#%%

my_robot = robot.robot(th,sp,x_0,y_0,theta_0)

#%% boucle du prog
while my_robot.on_goal() == False:
    my_robot.find_next_target_point()
    my_robot.turn_to_target_point()  
    my_robot.advance_to_target_point()
  
my_robot.stop()
print('arrivé')
print(my_robot.x,my_robot.y)


#%% stop
my_robot.stop()




