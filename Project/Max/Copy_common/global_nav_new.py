#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#https://github.com/TaipanRex/pyvisgraph

import pyvisgraph as vg

# def build_obstacles(vertices_array):
#     obstacles = []
#     for i in range(0,len(vertices_array)):
#         polygon = []
#         for j in range(0,len(vertices_array[i])):
#             polygon.append(vg.Point(vertices_array[i][j][0],vertices_array[i][j][1] ))
#         obstacles.append(polygon)
#     return obstacles

def build_obstacles(vertices_array, map_x, map_y, thymio_clearance_m):
    '''
        This function takes a list of lists of points and affects it to
        a list of the class polygon from pyvisgraph
        
        Parameters
        ----------
        vertices_array : list
            List containing lists of points defining polygons
        warped : image
            Image of the warped map
        scaling_px_2_m : float
            scale to convert pixel to meters
        
        Returns
        -------
        obstacles : list of polygon
            list of polygons
        '''
    
    obstacles = []
    for i in range(0,len(vertices_array)):
        polygon = []
        for j in range(0,len(vertices_array[i])):
            x = vertices_array[i][j][0]
            y = vertices_array[i][j][1]
            # if x > thymio_clearance_m and map_x - x > thymio_clearance_m and \
            # y > thymio_clearance_m and map_y - y > thymio_clearance_m:
            polygon.append(vg.Point(x, y ))
        obstacles.append(polygon)
    
    # polygon = []
    
    # polygon.append( vg.Point(-0.001,0.001))
    # polygon.append(vg.Point(-0.001,warped.shape[1]*scaling_px_2_m ))
    # polygon.append(vg.Point(0.0001,warped.shape[1]*scaling_px_2_m))
    # polygon.append(vg.Point(0.0001,0.0001))
    
    # obstacles.append(polygon)
    
    # polygon = []
    
    # polygon.append( vg.Point(warped.shape[0]*scaling_px_2_m,0) )
    # polygon.append( vg.Point(warped.shape[0]*scaling_px_2_m, warped.shape[1]*scaling_px_2_m) )
    # polygon.append( vg.Point(1+warped.shape[0]*scaling_px_2_m, warped.shape[1]*scaling_px_2_m) )
    # polygon.append( vg.Point(1+warped.shape[0]*scaling_px_2_m, 0) )
    
    # obstacles.append(polygon)
    
    return obstacles

def build_visgraph(obstacles):
    graph = vg.VisGraph()
    graph.build(obstacles)
    return graph

def apply_astar(graph, start, goal):
    start_point = vg.Point(start[0],start[1])
    goal_point = vg.Point(goal[0],goal[1])
    return graph.shortest_path(start_point,goal_point)