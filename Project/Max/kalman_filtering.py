import os
import sys
import time
import serial
import numpy as np
import matplotlib.pyplot as plt
import robot

from Thymio import Thymio

#%%
th = Thymio.serial(port="\\.\COM5", refreshing_rate=0.1)
#%%

all_target_points = [[0,0],[0.15,0.20],[0.30,0]]
theta_0 = 0
x_0 = 0
y_0 = 0
v = 0.03275

my_robot = robot.robot(th, all_target_points,x_0,y_0,theta_0, v, obstacle_vertices_m)

#%%

th.set_var("motor.left.target", 0)
th.set_var("motor.right.target", 0)
#%%
 

def remove_outliers(x): #Get rid of the zero values in speed measurements
    
    x = np.array(x)
    k = np.where(x>70)
    x = x[k]
    x = x.tolist()  
    return x

l_wheel = []
r_wheel = [] 

def get_speed_standard_deviation(l_wheel, r_wheel):
    
      
    tic = time.time()
    t = time.time()
    operating_time = 7
    
    while t-tic <= operating_time:
        my_robot.forward()
        l_wheel.append(th["motor.left.speed"])
        r_wheel.append(th["motor.right.speed"])     
        t = time.time()
    
    my_robot.stop()
    
    l_wheel = remove_outliers(l_wheel)
    r_wheel = remove_outliers(r_wheel)
    
    sig_L = np.std(l_wheel)
    sig_R = np.std(r_wheel)
    mu_L = np.mean(l_wheel)
    mu_R = np.mean(r_wheel)
    
    
    return l_wheel, r_wheel, sig_L, sig_R, mu_L, mu_R

def gaussian(x, mu, sig):
    return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))

[l_wheel, r_wheel, sig_L, sig_R, mu_L, mu_R] =  get_speed_standard_deviation(l_wheel, r_wheel)

if True: #Gaussian plot of left and right speeds
    plt.figure()
    plt.scatter(l_wheel, gaussian(l_wheel, mu_L, sig_L), label = 'left wheel')
    plt.scatter(r_wheel, gaussian(r_wheel, mu_R, sig_L), label = 'right wheel')
    plt.legend()
    plt.grid()
    
    plt.show()

#%%

Ts = 0.1
std_speed = 12.3*1e-6
qv = std_speed/2 # variance on speed state
rv = std_speed/2 # variance on speed measurement

std_acc = 0.31002189417769344
qa = std_acc/2 # variance on acceleration state
ra = std_acc/2 # variance on acceleration measurement 

# states = [position, speed]
A = np.array([[1, Ts], [0, 1]])
Q = np.array([[qv, 0], [0, qa]])
speed_conv_factor = 0.00033878326996197714
acceleration_conv_factor = 0.46758817921830315 


def signed_sensor_data(x):
    return (x + 2**15)%(2**16) - 2**15


def kalman_filter(speed, acceleration, x_est_prev, P_est_prev):
    """
    Estimates the current state using input sensor data and the previous state
    
    param speed: measured speed (Thymio units)
    param acceleration: measured acceleration (Thymio units)
    param x_est_prev: previous state a posteriori estimation
    param P_est_prev: previous state a posteriori covariance
    
    return x_est: new a posteriori state estimation
    return P_est: new a posteriori state covariance
    """
    
    ## Prediciton through the a priori estimate
    # estimated mean of the state
    x_est_a_priori = np.dot(A, x_est_prev)
    
    # Estimated covariance of the state
    P_est_a_priori = np.dot(A, np.dot(P_est_prev, A.T)) + Q
    
    ## Update         
    # y, C, and R for a posteriori estimate, depending on transition
        # transition detected
    
    y = np.array([[speed*speed_conv_factor],[acceleration*acceleration_conv_factor]])
    H = np.array([[1, 0],[0, 1]])
    R = np.array([[rv, 0],[0, ra]])

    # innovation / measurement residual
    i = y - np.dot(H, x_est_a_priori);
    # measurement prediction covariance
    S = np.dot(H, np.dot(P_est_a_priori, H.T)) + R;
             
    # Kalman gain (tells how much the predictions should be corrected based on the measurements)
    K = np.dot(P_est_a_priori, np.dot(H.T, np.linalg.inv(S)));
    
    
    # a posteriori estimate
    x_est = x_est_a_priori + np.dot(K,i);
    P_est = P_est_a_priori - np.dot(K,np.dot(H, P_est_a_priori));
     
    return x_est, P_est

x = [speed, acceleration]
x(k+1) = A*x(k) + ..
A = []

x_new = [x, y, theta, speed]
u

# script
# =============================================================================
# th = Thymio.serial(port="/dev/cu.usbmodem14201", refreshing_rate=0.1)
# time.sleep(3)
# t_start = 5
# t_move = 20
# dt = 0.1
# steps_start = int(t_start/dt)
# steps_move = int(t_move/dt)
# 
# x_est = [np.array([[0], [0]])]
# P_est = [1000 * np.ones(2)]
# acceleration = []
# speed = []
# 
# 
# for i in range(steps_start):
#     acceleration.append(signed_sensor_data(th['acc'][1]))
#     speed.append( (signed_sensor_data(th['motor.left.speed']) + signed_sensor_data(th['motor.right.speed']))/2 )
#     
#     new_x_est, new_P_est = kalman_filter(speed[-1], acceleration[-1], x_est[-1], P_est[-1])
#     x_est.append(new_x_est)
#     P_est.append(new_P_est)
#     time.sleep(dt)
#     
# th.set_var("motor.left.target", 200)
# th.set_var("motor.right.target", 200)
# for i in range(steps_move):
#     acceleration.append(signed_sensor_data(th['acc'][1]))
#     speed.append( (signed_sensor_data(th['motor.left.speed']) + signed_sensor_data(th['motor.right.speed']))/2 )
#     
#     new_x_est, new_P_est = kalman_filter(speed[-1], acceleration[-1], x_est[-1], P_est[-1])
#     x_est.append(new_x_est)
#     P_est.append(new_P_est)
#     time.sleep(dt)
#     
# th.set_var("motor.left.target", 0)
# th.set_var("motor.right.target", 0)
# time.sleep(0.5)   
#     
# th.close()
# =============================================================================
