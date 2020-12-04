#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec  4 14:12:14 2020

@author: eliechelly
"""
import cv2
import matplotlib.pyplot as plt
import vision

import global_nav_new as nav

def draw_path(all_target_points, scaling_px_2_m,img_path):
    img = cv2.imread(img_path)
    target_points_pixel = []
    for i in range(0,len(all_target_points)):
        target_points_pixel.append([all_target_points[i][0]/scaling_px_2_m,all_target_points[i][1]/scaling_px_2_m])
        
    for i in range(0,len(target_points_pixel)-1):
        cv2.line(img, (int(target_points_pixel[i][0]), int(target_points_pixel[i][1])), (int(target_points_pixel[i+1][0]), int(target_points_pixel[i+1][1])), (0, 255, 0), 50)
    plt.imshow(img)
    
    

start = [0,3500]
goal = [2800,0]
img_path = 'saved_img.jpg'
scaling_px_2_m = 1
vertices_array = vision.get_obstacle_vertices()
obstacles = nav.build_obstacles(vertices_array)
graph = nav.build_visgraph(obstacles)
shortest_path = nav.apply_astar(graph, start, goal)
all_target_points = nav.point2list(shortest_path)
draw_path(all_target_points, scaling_px_2_m,img_path)