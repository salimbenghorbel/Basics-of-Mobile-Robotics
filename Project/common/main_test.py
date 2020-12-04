import os
import sys
import time
import serial
import math
import numpy
from Thymio import Thymio
import robot
from vision import vision
import utilities
import cv2
import global_nav_new
#%%
# Adding the src folder in the current directory as it contains the script
# with the Thymio class
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))

th = Thymio.serial(port="/dev/cu.usbmodem14101", refreshing_rate=0.1)

#%%image
map_x = 0.7
map_y = 1.07 
thymio_clearance_m = 0.10
filename = 'media/thymio_square.png'
image = cv2.imread(filename, cv2.IMREAD_COLOR)
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

vis = vision(camera_ip='http://192.168.100.16:8080/video', plot=True)
warped = vis.warp_map(image)
warped = cv2.flip(warped, 0)
warped, scaling_px_2_m = vis.scale_map(warped, map_x, map_y)

# Load the feature images
feature_image_1 = cv2.imread('media/thymio_triangle.png')
feature_image_2 = cv2.imread('media/thymio_rond.png')
# Convert the features images to RGB
feature_image_1 = cv2.cvtColor(feature_image_1, cv2.COLOR_BGR2RGB)
feature_image_2 = cv2.cvtColor(feature_image_2, cv2.COLOR_BGR2RGB)

feature_color_1 = [98, 169, 87]
feature_color_2 = [66, 148, 169]
thymio_x_m, thymio_y_m, thymio_theta_rad = vis.locate_thymio(warped, feature_image_1, feature_color_1, feature_image_2, feature_color_2, scaling_px_2_m)

obstacle_map = vis.create_obstacle_map(warped)
thymio_clearance_px = int(thymio_clearance_m / scaling_px_2_m)
dilated_obstacle_map = vis.dilate_obstacle_map(obstacle_map, thymio_clearance_px)

target_color = [200, 90, 23]
target_position_px = vis.locate_target_px(warped, target_color)
target_position_m = target_position_px * scaling_px_2_m

obstacle_vertices_m = vis.get_obstacle_vertices(dilated_obstacle_map, scaling_px_2_m)
   

x_0 = thymio_x_m
y_0 = thymio_y_m 
theta_0 = thymio_theta_rad

obstacles = global_nav_new.build_obstacles(obstacle_vertices_m)
graph = global_nav_new.build_visgraph(obstacles)
sp = global_nav_new.apply_astar(graph, [x_0,y_0],target_position_m)

all_target_points = []
for i in range(0,len(sp)):
    all_target_points.append([sp[i].x,sp[i].y])
#%%
v = 0.03250
b = 0.095
my_robot = robot.robot(th,all_target_points,x_0,y_0,theta_0,v,b,obstacles)
utilities.draw_path(all_target_points, scaling_px_2_m,warped)

#%% boucle du prog
while my_robot.on_goal() == False:
    my_robot.find_next_target_point()
    my_robot.turn_to_target_point()  
    my_robot.advance_to_target_point()
    if my_robot.on_target_point() == False:
        print("pas trouvé le tg")
        print(my_robot.x,my_robot.y)
        #my_robot.turn_to_target_point()  
        #my_robot.advance_to_target_point()
    
        
    else: 
        print("trouvé le TG")
    

print('arrivé')
print(my_robot.x,my_robot.y)


#%% stop
my_robot.stop()
#%%
def draw_odometry(log,img):
    for i in range(0,len(log)-1):
        cv2.line(img, (int(log[i][0]/scaling_px_2_m), int(log[i][1]/scaling_px_2_m)), (int(log[i+1][0]/scaling_px_2_m), int(log[i+1][1]/scaling_px_2_m)), (0, 0, 255), 20)
    plt.imshow(img)

draw_odometry(my_robot.log,warped)

