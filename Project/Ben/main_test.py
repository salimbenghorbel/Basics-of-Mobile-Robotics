import os
import sys
import time
import serial
import math
import numpy
from Thymio import Thymio
import robot


# Adding the src folder in the current directory as it contains the script
# with the Thymio class
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))

th = Thymio.serial(port="/dev/cu.usbmodem14101", refreshing_rate=0.1)

#%% Test


all_target_points = [[0,0],[0.34,0.33],[0.64,0.87]]

my_robot = robot.robot(all_target_points)

while my_robot.on_goal() == False:

    my_robot.find_next_target_point()
    my_robot.turn_to_target_point(th)  
    my_robot.advance_to_target_point(th)
  
my_robot.stop(th)
print('arrivé')
print(my_robot.x,my_robot.y)

