import os
import sys
import time
import serial
import numpy as np

from Thymio import Thymio

z_scale = 0.4306409130816506  # positive when downward
x_scale = 0.5096103896103896  # positive when right
y_scale = 0.46758817921830315 # positive when forward

z_std = 0.2553522687593768
x_std = 0.355816090465905
y_std = 0.31002189417769344

def signed_sensor_data(x):
    return (x + 2**15)%(2**16) - 2**15


def calibrate_accelerometer_z(th):
    g_constant = 9.81
    # thymio must be put on its wheels.
    acc_data = []
    for i in range(100):
        acc_data.append(abs(signed_sensor_data(th["acc"][2])))
        time.sleep(0.1)
    
    scale = g_constant/np.mean(acc_data)
    return scale, np.std([scale * i for i in acc_data])

def calibrate_accelerometer_x(th):
    g_constant = 9.81
    # thymio must be put on its sides (left or right).
    acc_data = []
    for i in range(100):
        acc_data.append(abs(signed_sensor_data(th["acc"][0])))
        time.sleep(0.1)
    
    scale = g_constant/np.mean(acc_data)
    return scale, np.std([scale * i for i in acc_data])


def calibrate_accelerometer_y(th):
    g_constant = 9.81
    # thymio must be put on its sides (front or back).
    acc_data = []
    for i in range(100):
        acc_data.append(abs(signed_sensor_data(th["acc"][1])))
        time.sleep(0.1)
    
    scale = g_constant/np.mean(acc_data)
    return scale, np.std([scale * i for i in acc_data])



# script
th = Thymio.serial(port="/dev/cu.usbmodem14201", refreshing_rate=0.1)
#z_scale, z_std = calibrate_accelerometer_z(th)
#x_scale, x_std = calibrate_accelerometer_x(th)
#y_scale, y_std = calibrate_accelerometer_y(th)

th.close()