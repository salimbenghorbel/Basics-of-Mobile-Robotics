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

def stop():
    
    """Shutdown all the Thymio motors"""
    
    th.set_var("motor.left.target", 0)
    th.set_var("motor.right.target", 0)
    

def Thymio_translate(l_speed = 150, r_speed = 150):
    """
    Input: speed Thymio = 150 (default)
    Output: Thymio moves (default = forward) at speed indicated (default = 150)  
    """
    r_wheel = 43/2*1e-3 #radius of a wheel [m]
    
    """Calibration part"""
    cal_R = 6 # calibration term for right motor
    cal_L = 0 # calibration term for left motor
    
    if l_speed < 0: l_speed = 2**16 - (l_speed+cal_L)
    elif l_speed > 0: l_speed = l_speed + cal_L
            
    if r_speed < 0: r_speed = 2**16 - (l_speed+cal_R)
    elif r_speed > 0: r_speed = r_speed + cal_R
            
    th.set_var("motor.left.target", l_speed)
    th.set_var("motor.right.target", r_speed)
    
def local_avoidance(prox_sensors): #Local avoidance strategy
    
    """
    Implement Braitenberg strategy
    
    Input: proximity sensor values
    Output: Thymio dodges obstacle by quickly rotating wheel near obstacle
    """
    
    W=np.matrix([[2, 2, -1, -2, -2], 
                    [-2, -2, -1, 2, 2]]) # wight matrix for each front sensor
    
    y = W.dot(prox_sensors)
    y = np.divide(y,10)
    print("y = ",y)

    l_speed = int(np.round(y[0,0]))
    r_speed = int(np.round(y[0,1]))
    
    speed = np.array([l_speed, r_speed])
    
    speed[speed > 500] = 500 # Saturation condition to avoid "bytes must be in range(0, 256)" error
    speed[speed < -500] = -500
    
    print("speed = ", speed)
    
    
    if (l_speed < 0): l_speed = 2**16-abs(l_speed) #convert negative values for Thymio.py
    if (r_speed < 0): r_speed = 2**16-abs(r_speed)

    th.set_var("motor.left.target", l_speed)
    th.set_var("motor.right.target", r_speed)

    
def FSM(verbose=True): #Finite State Machine
    """
    Checks at regular intervals the prox sensor and enters local avoidance if obstacle detected
    """
    prox = False
    tic = time.time()
    t =  time.time()
    operating_time = 10 #sec

    while t-tic <= operating_time:
    
        prox_sensors = th["prox.horizontal"][0:5]
        

        if max(prox_sensors)!=0 and prox == False: #obstacle detected while still in global navigation mode
            
            if verbose: print("Obstacle go local")
            prox = True
        
        elif max(prox_sensors) ==0 and prox == True: #no obstacle in front in local avoidance --> go back to global
            
            if verbose: print("No obstalce go global")
            prox = False

        
        if prox:
            
            local_avoidance(prox_sensors)
        
        elif not prox:
            
            Thymio_translate()
        
        t = time.time()
    
    stop()
#%% 
FSM()