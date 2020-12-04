import os
import sys
import time
import serial
import math
import numpy
from Thymio import Thymio
import robot

#%%
th = Thymio.serial(port="\\.\COM5", refreshing_rate=0.1)
#%% Initialisation à remplacer 

all_target_points = [[0,0],[0.30,0],[0,0.10]]
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
print('Thymio has arrived')
print(my_robot.x,my_robot.y)

#%%
my_robot.stop()
#%% 
my_robot.forward()
#%%
time.sleep(0.5)
Thymio.close(th)
 
