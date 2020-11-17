# -*- coding: utf-8 -*-
"""
Created on Sun Nov 15 16:18:30 2020

@author: Maxime
"""
from Thymio import Thymio
import numpy as np
import time


#th = Thymio.serial(port="\\.\COM4", refreshing_rate=0.1)

vreal_max = 0.1525
def vThymio2vReal(vThymio):
    """
    Input: vThymio = 0 - 500
    Output: vReal = 0 - 0.14 m/s
    """
    
    if vThymio > 500:
        print("Warning: vThymio above 500, saturating vReal to 0.14")
        vReal = vreal_max

    elif vThymio < 0:
        print("Warning: vThymio negative")
        vThymio = 0
    
    else:
        vReal = vThymio*vreal_max/500
    
    return vReal 

def vReal2vThymio(vReal):
    """
    Input: vReal = 0 - 0.14 m/s
    Output: vThymio = 0 - 500
    """
    if vReal > vreal_max:
        print("Warning: vReal above 0.14 m/s, saturating vThymio to 500")
        vThymio = 500

    elif vReal < 0:
        print("Warning: vReal negative")
        vThymio = 0
        
    else:
        vThymio = vReal*500/vreal_max
       
        
    return vThymio

def stop():
    
 th.set_var("motor.left.target", 0)
 th.set_var("motor.right.target", 0)
    
def Thymio_rotate(angle, vThymio = 150):

    vReal = vThymio2vReal(vThymio)
    cal_R = 10
    cal_L = 0

    dist_wheel = 95*1e-3 # dist between wheels [m]
    dt = abs(angle)*vReal/(dist_wheel/2)

    if angle > 0:
        th.set_var("motor.left.target", 2**16-(vThymio + cal_L))
        th.set_var("motor.right.target", vThymio + cal_R)
    elif angle < 0:
        th.set_var("motor.left.target", vThymio + cal_L)
        th.set_var("motor.right.target", 2**16-(vThymio + cal_R))
    elif angle == 0:
        stop()
        disp("Warning: angle = 0")


    time.sleep(dt)
    stop()
    
    return dt

def Thymio_translate(d, vThymio = 150):
    """
    Input: d = distance to travel [m], vThymio = 150 (default)
    Output: Thymio moves (default = forward) through a distance d at speed vThymio 
    """
    r_wheel = 43/2*1e-3 #radius of a wheel [m]
    cal_R = 6
    cal_L = 0
    
    vReal = vThymio2vReal(vThymio)
    dt = abs(d)/vReal
    if d > 0: #moves forward
        th.set_var("motor.left.target", vThymio + cal_L)
        th.set_var("motor.right.target", vThymio + cal_R)
        
    elif d < 0: #moves backward
        th.set_var("motor.left.target", 2**16-(vThymio + cal_L))
        th.set_var("motor.right.target", 2**16-(vThymio + cal_R))
        
    elif d == 0:
        stop()
        disp("Warning: dist = 0")
    
    time.sleep(dt)
    stop()
    return dt
