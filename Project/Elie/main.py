import os
import sys
import time
import serial

# importing our modules

import global_pos
import vision
import navigation as nav

# Adding the src folder in the current directory as it contains the script
# with the Thymio class
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))

from Thymio import Thymio

th = Thymio.serial(port="/dev/cu.usbmodem14101", refreshing_rate=0.1)

# definning flags and global variables
thymio_coords = [0,0,0] #x, y and theta
goal = 0

# getting picture of terrain from vision module
vision.get_image()

# highlighting thymio, obstacles, edges of the map, goal and origin
vision.filter_image()

# defining a coordonate system and calculating coordonates of the thymio, 
# obstacle, edges of the map, goal and origin
global_pos.define_coordonates()

# applying A* algo (or other path-finding algo) and calculating coordonates
# of target_points
target_point_array = global_pos.get_path()

while !nav.onGoal(x,y,allTargetPoints):
    
    if nav.onTargetPoint(x,y,targetPoint):
        
        if allTargetPoints.targetPoint == objective:
            goal = 1
        else:
            target_point = findNextTargetPoint(allTargetPoints,targetPoint)
            nav.turnToTargetPoint(theta,targetPoint)
    else:
        advanceToTargetPoint(x,y,theta,targetPoint)