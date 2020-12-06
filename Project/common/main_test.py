import time
import serial
import math
import numpy as np
from Thymio import Thymio
import robot
from vision import vision
import utilities
import cv2
import global_navigation as nav
import matplotlib.pyplot as plt
import pyvisgraph as vg
#%% Establish serial connection with Thymio
windows = True
if windows:
    thymio_port = "\\.\COM5"
else:
    thymio_port = "/dev/cu.usbmodem14101"

th = Thymio.serial(port=thymio_port, refreshing_rate=0.1)

#%% Global navigation

# physical dimensions of the map.
map_x = 0.89 
map_y = 0.84 
# clearance in m to take into account thymio dimensions in path planning.
thymio_clearance_m = 0.12
# a webcam app is used to stream smartphone camera data on a local webserver
camera_ip = 'http://192.168.1.9:8080/video' 

# declare vision class:
vis = vision(camera_ip=camera_ip, plot=True)
# take snapshot from camera feed
image = vis.get_camera_image()
# extract image map with warped perspective.
warped = vis.warp_map(image)
# perform a vertical flip of warped to use direct rotation referencial and coordinates
warped = cv2.flip(warped, 0)
# scale warped to match physical dimension aspect ratio
warped, scaling_px_2_m = vis.scale_map(warped, map_x, map_y)

# Load the feature images
feature_image_1 = cv2.imread('media/thymio_circle_green.png')
feature_image_2 = cv2.imread('media/thymio_circle_blue.png')
# Convert the features images to RGB
feature_image_1 = cv2.cvtColor(feature_image_1, cv2.COLOR_BGR2RGB)
feature_image_2 = cv2.cvtColor(feature_image_2, cv2.COLOR_BGR2RGB)

feature_color_1 = [8, 72, 47]
feature_color_2 = [39, 52, 104]

# locate thymio on map and extract orientation and coordinates.
x_0, y_0, theta_0 = vis.locate_thymio(warped, feature_image_1, feature_color_1, feature_image_2, feature_color_2, scaling_px_2_m)

# create obstacle map
obstacle_map = vis.create_obstacle_map(warped)
# dilate obstacle map with thymio clearance.
thymio_clearance_px = int(thymio_clearance_m / scaling_px_2_m)
dilated_obstacle_map = vis.dilate_obstacle_map(obstacle_map, thymio_clearance_px)

# locate target on map.
target_color = [148, 18, 28]
target_position_px = vis.locate_target_px(warped, target_color)
target_position_m = target_position_px * scaling_px_2_m

# extract position of obstacle vertices.
obstacle_vertices_m = vis.get_obstacle_vertices(dilated_obstacle_map, scaling_px_2_m)
# build obstacles and build visibility graph.
obstacles = nav.build_obstacles(obstacle_vertices_m, map_x, map_y, thymio_clearance_m)
graph = nav.build_visgraph(obstacles)
# apply A* to extract optimal path and target points coordinates.
sp = nav.apply_astar(graph, [x_0,y_0],target_position_m)
all_target_points = []
for i in range(0,len(sp)):
    all_target_points.append([sp[i].x,sp[i].y])
    
# draw robot path: final result of global path planning
utilities.draw_path(all_target_points, scaling_px_2_m,warped)

obstacle_vertices_m = vis.get_obstacle_vertices(dilated_obstacle_map, scaling_px_2_m)

# obstacles = global_nav_new.build_obstacles(obstacle_vertices_m)
#obstacles = global_nav_new.build_obstacles(obstacle_vertices_m, warped, scaling_px_2_m)
obstacles = nav.build_obstacles(obstacle_vertices_m, map_x, map_y, thymio_clearance_m)
graph = nav.build_visgraph(obstacles)

# draw robot path: final result of global path planning
utilities.draw_path(all_target_points, scaling_px_2_m,warped)
#%% Declare robot object.
v = 0.03275
my_robot = robot.robot(th,all_target_points,x_0,y_0,theta_0,v)
#%% Main program loop: moving to target points and local obstacle avoidance.

# Loop until target is reached
while my_robot.on_goal() == False:
    # Set next target point
    my_robot.find_next_target_point()
    print("\t Pos [x,y] = ",my_robot.x,my_robot.y)
    # Rotate orientation to face target point.
    my_robot.turn_to_target_point()
    # Advance to target point in a straight line.
    my_robot.advance_to_target_point()
    
    # Check whether robot reached target point. Break if so.
    if my_robot.on_goal():
        break
    
    # Check whether robot correclty estimated its motion and successfully 
    # attained target point.
    
    # Take camera snapshot to locate thymio new position and orientation.
    # A second chance is given to the user in case of a misaligned picture.
    for i in range(2):
        try:
            print("Taking new camera snaphot.")
            image = vis.get_camera_image()
            # warp perspective to image.
            warped = vis.warp_map(image)
            # flip vertically
            warped = cv2.flip(warped, 0)
            # rescale map to physical aspect ratio
            warped, scaling_px_2_m = vis.scale_map(warped, map_x, map_y)
            # locate thymio and update its coordinates and orientation.
            my_robot.x, my_robot.y, my_robot.theta = vis.locate_thymio(warped, feature_image_1, feature_color_1, feature_image_2, feature_color_2, scaling_px_2_m)
            break
        except: 
            # wait a certain amount of time to give the user the chance to 
            # extract a new image.
            time.sleep(3)
            pass
    
    # apply A* algorithm again with thymio updated position.
    sp = nav.apply_astar(graph, [my_robot.x,my_robot.y], target_position_m)            
    # extract new target points for optimal path.
    new_all_target_points = []
    for i in range(0,len(sp)):
            new_all_target_points.append([sp[i].x,sp[i].y])
    my_robot.all_target_points = new_all_target_points
    
    my_robot.target_point = [new_all_target_points[0][0],new_all_target_points[0][1]]
    
    print("Found the target point.")
    print("\t Pos [x,y] = ",my_robot.x,my_robot.y)

print('Arrived on target point. Hurray!')
print("\t Pos [x,y] = ",my_robot.x,my_robot.y)


def draw_odometry(log,img):
    """
    This method draws the set of points taken by the robot based off its position
    estimation from odometry.
    """
    image = img.copy()
    for i in range(0,len(log)-1):
        cv2.line(image, (int(log[i][0]/scaling_px_2_m), int(log[i][1]/scaling_px_2_m)), (int(log[i+1][0]/scaling_px_2_m), int(log[i+1][1]/scaling_px_2_m)), (0, 0, 255), 20)
    plt.imshow(image)
    
# draw odometry estimation data on map.
utilities.draw_odometry(my_robot.log,warped)
#%% stop robot
my_robot.stop()


