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
allTargetPoints = [[0.34,0.33],[0.64,0.87]]
targetPoint=[0,0]



while onGoal(x,y,allTargetPoints)==False:
    if onTargetPoint(x,y,targetPoint):
        print(theta)
        targetPoint = findNextTargetPoint(allTargetPoints,targetPoint)
        alpha = turnToTargetPoint(theta,targetPoint)  
        advanceToTargetPoint(x,y,theta,targetPoint)
    else:
        stop()
print('arrivé')
print(x,y)


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
    global x
    global y
    theta_goal = math.atan2(targetPoint[1]-y, targetPoint[0]-x)
    alpha = theta_goal-theta
    turn(alpha)
    return alpha
    

def advanceToTargetPoint(x,y,theta,targetPoint):
    d_x = targetPoint[0] - x
    d_y = targetPoint[1] - y
    print(x,y)
    d = math.sqrt(math.pow(d_x,2)+math.pow(d_y,2))
    runForward(d)
    
    

def onTargetPoint(x,y,targetPoint):
    R = 0.05
    d_x = targetPoint[0] - x
    d_y = targetPoint[1] - y
    d = math.sqrt(math.pow(d_x,2)+math.pow(d_y,2))
    if d < R:
        print('true')
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

def odometryForward(delta_t,v,dt):
    global x, y, theta
    x = x + v * delta_t * math.cos(theta)
    y = y + v * delta_t * math.sin(theta)

def odometryAngle(delta_t,alpha,dt):
    global theta
    theta = theta + delta_t * alpha /dt
 

def forward():
    th.set_var("motor.left.target", 100)
    th.set_var("motor.right.target", 100)

def stop():
    th.set_var("motor.left.target", 0)
    th.set_var("motor.right.target", 0)


def clockwise():
    th.set_var("motor.left.target", 2**16-100)
    th.set_var("motor.right.target", 102)
    return

def anticlockwise():
    th.set_var("motor.left.target", 100)
    th.set_var("motor.right.target", 2**16-102)
    return


def runForward(d):
    v = 0.03375
    dt = d/v 
    t0 = time.time()
    t1 = 0
    t = 0
    
     
    while t < dt:
        t1 = time.time()
        delta_t = t1 - t0 - t
        forward()
        t = t1 - t0
        odometryForward(delta_t,v,dt)
    stop()
   

def turn(alpha):
    R = 0.047
    v = 0.03050
    dt = abs(R * alpha / v)
    t0 = time.time()
    t1 = 0
    t = 0
    print(alpha)
    if alpha > 0:
      
        while t < dt:
            t1 = time.time()
            delta_t = t1 - t0 - t 
            clockwise()
            t = t1 - t0
            odometryAngle(delta_t,alpha,dt)
        stop()
        
    else:
        
         while t < dt:
            t1 = time.time()
            delta_t = t1 - t0 - t  
            anticlockwise()
            t = t1 - t0
            odometryAngle(delta_t,alpha,dt)
         stop()
  
    
#%% 

