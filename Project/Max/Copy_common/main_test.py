import time
import serial
import math
import numpy as np
from Thymio import Thymio
import robot
from vision import vision
import utilities
import cv2
import global_nav_new1 as nav
import matplotlib.pyplot as plt
import pyvisgraph as vg
#%%
#th = Thymio.serial(port="/dev/cu.usbmodem14101", refreshing_rate=0.1)
th = Thymio.serial(port="\\.\COM5", refreshing_rate=0.1)
#%%image
map_x = 0.89 #valeur Ben = 0.7
map_y = 0.84 #valeur Ben = 1.07
thymio_clearance_m = 0.12
# filename = 'media/thymio_square3.jpg'
# image = cv2.imread(filename, cv2.IMREAD_COLOR)
# image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

vis = vision(camera_ip='http://192.168.1.9:8080/video', plot=True)
image = vis.get_camera_image()
#image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
warped = vis.warp_map(image)
warped = cv2.flip(warped, 0)
warped, scaling_px_2_m = vis.scale_map(warped, map_x, map_y)

# Load the feature images
feature_image_1 = cv2.imread('media/thymio_circle_green.png')
feature_image_2 = cv2.imread('media/thymio_circle_blue.png')
# Convert the features images to RGB
feature_image_1 = cv2.cvtColor(feature_image_1, cv2.COLOR_BGR2RGB)
feature_image_2 = cv2.cvtColor(feature_image_2, cv2.COLOR_BGR2RGB)

# feature_color_2 = [72, 96, 46]
feature_color_1 = [8, 72, 47]
feature_color_2 = [39, 52, 104]

thymio_x_m, thymio_y_m, thymio_theta_rad = vis.locate_thymio(warped, feature_image_1, feature_color_1, feature_image_2, feature_color_2, scaling_px_2_m)

obstacle_map = vis.create_obstacle_map(warped)
thymio_clearance_px = int(thymio_clearance_m / scaling_px_2_m)
dilated_obstacle_map = vis.dilate_obstacle_map(obstacle_map, thymio_clearance_px)

target_color = [148, 18, 28]
target_position_px = vis.locate_target_px(warped, target_color)
target_position_m = target_position_px * scaling_px_2_m

obstacle_vertices_m = vis.get_obstacle_vertices(dilated_obstacle_map, scaling_px_2_m)
   

x_0 = thymio_x_m
y_0 = thymio_y_m 
theta_0 = thymio_theta_rad

# obstacles = global_nav_new.build_obstacles(obstacle_vertices_m)
#obstacles = global_nav_new.build_obstacles(obstacle_vertices_m, warped, scaling_px_2_m)
obstacles = nav.build_obstacles(obstacle_vertices_m, map_x, map_y, thymio_clearance_m)
graph = nav.build_visgraph(obstacles)

#edge_one = vg.Edge(vg.Point(0,1),vg.Point(0,0))
#edge_two = vg.Edge(vg.Point(0.01,0),vg.Point(warped.shape[0]*scaling_px_2_m,0))
#edge_three = vg.Edge(vg.Point(warped.shape[0]*scaling_px_2_m,0.01),vg.Point(warped.shape[0]*scaling_px_2_m,warped.shape[1]*scaling_px_2_m))
#edge_four = vg.Edge(vg.Point(warped.shape[0]*scaling_px_2_m-0.01,warped.shape[1]*scaling_px_2_m),vg.Point(0.01,warped.shape[1]*scaling_px_2_m))
#graph.graph.add_edge(edge_one)
#graph.graph.add_edge(edge_two)
#graph.graph.add_edge(edge_three)
#graph.graph.add_edge(edge_four)

sp = nav.apply_astar(graph, [x_0,y_0],target_position_m)

all_target_points = []
for i in range(0,len(sp)):
    all_target_points.append([sp[i].x,sp[i].y])
utilities.draw_path(all_target_points, scaling_px_2_m,warped)

v = 0.03275
b = 0.095
my_robot = robot.robot(th,all_target_points,x_0,y_0,theta_0,v,b,obstacles)
# my_robot.theta = 0
#my_robot.turn(np.pi)
# my_robot.run_forward(0.2)
#%%
while my_robot.on_goal() == False:
    
    my_robot.find_next_target_point()
    print("\t Pos [x,y] = ",my_robot.x,my_robot.y)
    my_robot.turn_to_target_point()  
    my_robot.advance_to_target_point()
    print("t'as 0 sec pour ta photo")
    
    if my_robot.on_goal():
        break
    
    time.sleep(0)
    for i in range(2):
        try:
            image = vis.get_camera_image()
            #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            warped = vis.warp_map(image)
            warped = cv2.flip(warped, 0)
            warped, scaling_px_2_m = vis.scale_map(warped, map_x, map_y)
            my_robot.x, my_robot.y, my_robot.theta = vis.locate_thymio(warped, feature_image_1, feature_color_1, feature_image_2, feature_color_2, scaling_px_2_m)
            # my_robot.x, my_robot.y, theta_vision = vis.locate_thymio(warped, feature_image_1, feature_color_1, feature_image_2, feature_color_2, scaling_px_2_m)
            # print("Theta robot = ", my_robot.theta)
            # print("Theta vision =", theta_vision)
            break
        except: 
            time.sleep(3)
            pass
        
    sp = nav.apply_astar(graph, [my_robot.x,my_robot.y], target_position_m)            
    
    new_all_target_points = []
    for i in range(0,len(sp)):
            new_all_target_points.append([sp[i].x,sp[i].y])
    my_robot.all_target_points = new_all_target_points
    
    my_robot.target_point = [new_all_target_points[0][0],new_all_target_points[0][1]]
    
    print("trouvé le tg")
    print("\t Pos [x,y] = ",my_robot.x,my_robot.y)

print('arrivé c la fête')
print(my_robot.x,my_robot.y)

def draw_odometry_on_warped(log,img):
    for i in range(0,len(log)-1):
        cv2.line(img, (int(log[i][0]/scaling_px_2_m), int(log[i][1]/scaling_px_2_m)), (int(log[i+1][0]/scaling_px_2_m), int(log[i+1][1]/scaling_px_2_m)), (0, 0, 255), 20)
    plt.imshow(img)
def draw_odometry(log,img):
    image = img.copy()
    for i in range(0,len(log)-1):
        cv2.line(image, (int(log[i][0]/scaling_px_2_m), int(log[i][1]/scaling_px_2_m)), (int(log[i+1][0]/scaling_px_2_m), int(log[i+1][1]/scaling_px_2_m)), (0, 0, 255), 20)
    plt.imshow(image)

draw_odometry(my_robot.log,warped)
#%% stop
my_robot.stop()


