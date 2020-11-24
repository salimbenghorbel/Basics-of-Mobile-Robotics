import os
import sys
import time
import serial
import math
import numpy
from Thymio import Thymio
import robot

#%%
# Adding the src folder in the current directory as it contains the script
# with the Thymio class
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))

th = Thymio.serial(port="/dev/cu.usbmodem14101", refreshing_rate=0.1)

#%% Initialisation à remplacer 

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

