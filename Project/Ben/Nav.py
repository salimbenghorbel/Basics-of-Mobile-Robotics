import os
import sys
import time
import serial
import math
import numpy
from Thymio import Thymio


# Adding the src folder in the current directory as it contains the script
# with the Thymio class
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))

th = Thymio.serial(port="/dev/cu.usbmodem14101", refreshing_rate=0.1)

#%% Test

x = 0
y = 0
theta = 0
allTargetPoints = [ [0.1,0.1],[0.3,0.5],[0.5,0.5]]
targetPoint=[0,0]
#%% test boucle
while onGoal(x,y,allTargetPoints)==False:
    if onTargetPoint:
        targetPoint = findNextTargetPoint(allTargetPoints,targetPoint)
        alpha = turnToTargetPoint(theta,targetPoint)  
        advanceToTargetPoint(x,y,theta,targetPoint)
        x = targetPoint[0]
        y = targetPoint[1]
        theta = math.atan2(y,x)
      
        

#%%
def findNextTargetPoint(allTargetPoints,targetPoint):
    if targetPoint == [0,0]:
        targetPoint[0] = allTargetPoints[0][0]
        targetPoint[1] = allTargetPoints[0][1]
    else:
        i = allTargetPoints.index(targetPoint)
        targetPoint[0] = allTargetPoints[i+1][0]
        targetPoint[1] = allTargetPoints[i+1][1]
    return targetPoint
        
def turnToTargetPoint(theta,targetPoint):
    theta_goal = math.atan2(targetPoint[1], targetPoint[0])
    alpha = theta_goal-theta
    turn(alpha)
    print(alpha)
    return alpha
    

def advanceToTargetPoint(x,y,theta,targetPoint):
    d_x = targetPoint[0] - x
    d_y = targetPoint[1] - y
    d = math.sqrt(math.pow(d_x,2)+math.pow(d_y,2))
    runForward(d)
    
    

def onTargetPoint(x,y,targetPoint):
    R = 0.05
    d_x = targetPoint[0] - x
    d_y = targetPoint[1] - y
    d = math.sqrt(math.pow(d_x,2)+math.pow(d_y,2))
    if d < R:
        return True
    else:
        return False

def onGoal(x,y,allTargetPoints):
    R = 0.05
    d_x = allTargetPoints[-1][0] - x
    d_y = allTargetPoints[-1][1] - y
    d = math.sqrt(math.pow(d_x,2)+math.pow(d_y,2))
    if d < R:
        return True
    else:
        return False
 #%%

def forward():
    th.set_var("motor.left.target", 100)
    th.set_var("motor.right.target", 100)

def stop():
    th.set_var("motor.left.target", 0)
    th.set_var("motor.right.target", 0)


def clockwise():
    th.set_var("motor.left.target", 2**16-100)
    th.set_var("motor.right.target", 100)
    return

def anticlockwise():
    th.set_var("motor.left.target", 100)
    th.set_var("motor.right.target", 2**16-100)
    return


def runForward(d):
    v = 0.03375
    dt = d/v 
    t0 = time.time()
    t1 = 0
    
    while t1 - t0 < dt:
        forward()
        t1 = time.time()
    stop()
   

def turn(alpha):
    R = 0.047
    v = 0.03375
    dt = abs(R * alpha / v)
    t0 = time.time()
    t1 = 0
    if alpha > 0:
      
        while t1 - t0 < dt:
            clockwise()
            t1 = time.time()
        stop()
        
    else:
        
         while t1 - t0 < dt:
            anticlockwise()
            t1 = time.time()
         stop()
  
    
#%% 

