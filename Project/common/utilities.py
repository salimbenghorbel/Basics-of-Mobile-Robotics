"""
This module contains different methods used for global navigation and plotting
followed paths
"""

import cv2
import matplotlib.pyplot as plt
import vision

import global_navigation as nav

def draw_path(all_target_points, scaling_px_2_m,img):
    """
    This method draws path between target points on warped map.
    """
    target_points_pixel = []
    for i in range(0,len(all_target_points)):
        target_points_pixel.append([all_target_points[i][0]/scaling_px_2_m,all_target_points[i][1]/scaling_px_2_m])
        
    for i in range(0,len(target_points_pixel)-1):
        cv2.line(img, (int(target_points_pixel[i][0]), int(target_points_pixel[i][1])), (int(target_points_pixel[i+1][0]), int(target_points_pixel[i+1][1])), (0, 255, 0), 20)
    plt.imshow(img)

def draw_odometry(log,img):
    """
    This method draws the set of points taken by the robot based off its position
    estimation from odometry.
    """
    image = img.copy()
    for i in range(0,len(log)-1):
        cv2.line(image, (int(log[i][0]/scaling_px_2_m), int(log[i][1]/scaling_px_2_m)), (int(log[i+1][0]/scaling_px_2_m), int(log[i+1][1]/scaling_px_2_m)), (0, 0, 255), 20)
    plt.imshow(image)
    
if False:
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