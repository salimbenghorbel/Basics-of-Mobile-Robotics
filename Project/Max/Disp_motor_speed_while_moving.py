import time
import math
import numpy as np
from Thymio import Thymio
import robot

#%% 
th = Thymio.serial(port="\\.\COM4", refreshing_rate=0.1)

#%%
all_target_points = [[0,0],[0.15,0.20],[0.30,0]]
theta_0 = 0
x_0 = 0
y_0 = 0 


my_robot = robot.robot(all_target_points,x_0,y_0,theta_0)

def stop():
    
    """Shutdown all the Thymio motors"""
    
    th.set_var("motor.left.target", 0)
    th.set_var("motor.right.target", 0)
#%%

tic = time.time()
t = time.time()
operating_time = 7

while t-tic <= operating_time:
    my_robot.forward(th)
    print("Left motor: ",th["motor.left.speed"])
    print("Right motor: ",th["motor.right.speed"])
    t = time.time()

stop()

#%%
stop()
#%%
Thymio.__del__(th)