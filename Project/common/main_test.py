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
import global_pos
#%%
# Adding the src folder in the current directory as it contains the script
# with the Thymio class
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))

th = Thymio.serial(port="/dev/cu.usbmodem14101", refreshing_rate=0.1)

#%%image

image = vision.get_image()

#%% Initialisation à remplacer 
image = cv2.imread("saved_img.png", cv2.IMREAD_COLOR)
#%%
warped = four_point_transform.four_mat(image)
#%%
dilation = map_creation.create_map(warped)
#%%
obstacles_vertices = vision.get_obstacle_vertices(dilation,dilation)
#%%sorti vision:
#sort départ, arrivé et tous les points en mètres 

#%%
sp = global_pos.build_visibility_graph(obstacles_vertices,[1000,1500],[2100,50])
#%%
all_target_points = [[0,0],[0.34,0.33],[0.59,0.87]]
theta_0 = 0
x_0 = 0
y_0 = 0 


my_robot = robot.robot(th,all_target_points,x_0,y_0,theta_0)

#%% boucle du prog
while my_robot.on_goal() == False:
    my_robot.find_next_target_point()
    my_robot.turn_to_target_point()  
    my_robot.advance_to_target_point()
  
my_robot.stop()
print('arrivé')
print(my_robot.x,my_robot.y)

#%% test calibaration
i = 0
while i <5000:
    th.set_var("motor.left.target", 2**16-100)
    L = th.get_var('motor.left.speed')
    L = L/(2**16) - 100
    print(L)
    i = i+1
th.set_var("motor.left.target",0)

#%% stop
my_robot.stop()




