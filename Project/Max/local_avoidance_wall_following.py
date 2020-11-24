import os
import sys
import time
import serial
import math
import numpy as np
from Thymio import Thymio
import robot

# Adding the src folder in the current directory as it contains the script
# with the Thymio class
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))

#%% 
th = Thymio.serial(port="\\.\COM5", refreshing_rate=0.1)

#%%
all_target_points = [[0,0],[0.15,0.20],[0.30,0]]
theta_0 = 0
x_0 = 0
y_0 = 0 


my_robot = robot.robot(all_target_points,x_0,y_0,theta_0)
#%%

def stop():
    
    """Shutdown all the Thymio motors"""
    
    th.set_var("motor.left.target", 0)
    th.set_var("motor.right.target", 0)
    

def local_avoidance(prox_sensors,th): #Local avoidance strategy
    
    """
    Implement sequential strategy
    
    Input: proximity sensor values
    Output: Thymio dodges obstacle with a predefined sequence of movements
    """
    
    l_speed = 0
    r_speed = 0
    
# =============================================================================
#     if max(prox_sensors[1:5]) != 0: #rotates to only detect with one sensor
#         l_speed = 300
#         r_speed = 2**16-300
#     
#     elif max(prox_sensors[2:5]) == 0 and prox_sensors[0] != 0: 
#         l_speed = 20
#         r_speed = 90
#     
#     th.set_var("motor.left.target", l_speed)
#     th.set_var("motor.right.target", r_speed)
# 
# =============================================================================
    
    my_robot.turn(np.pi/3,th)
    my_robot.run_forward(0.2, th)
    my_robot.turn(-np.pi/3,th)
    my_robot.run_forward(0.2, th)

    
def FSM(verbose=True): #Finite State Machine
    """
    Checks at regular intervals the prox sensor and enters local avoidance if obstacle detected
    """
    prox = False
    tic = time.time()
    t =  time.time()
    operating_time = 20#sec

    while t-tic <= operating_time:
    
        prox_sensors = th["prox.horizontal"][0:5]
        

        if max(prox_sensors)!= 0 and prox == False: #obstacle detected while still in global navigation mode
            
            if verbose: print("Obstacle go local")
            prox = True
        
        elif max(prox_sensors) == 0 and prox == True: #no obstacle in front in local avoidance --> go back to global
            
            if verbose: print("No obstalce go global")
            prox = False

        
        if prox:
            
            local_avoidance(prox_sensors,th)
        
        elif not prox:
            
            my_robot.forward(th)
        
        t = time.time()
    
    stop()
#%% 
FSM()

#%% 
stop()

#%%
Thymio.close(th)