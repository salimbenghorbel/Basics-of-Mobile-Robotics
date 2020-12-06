#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#https://github.com/TaipanRex/pyvisgraph

import pyvisgraph as vg

def build_obstacles(vertices_array, map_x, map_y, thymio_clearance_m):
    obstacles = []
    
    # add borders
    border0 = vg.Point(thymio_clearance_m, thymio_clearance_m)
    border1 = vg.Point(map_x - thymio_clearance_m, thymio_clearance_m)
    border2 = vg.Point(map_x - thymio_clearance_m, map_y - thymio_clearance_m)
    border3 = vg.Point(thymio_clearance_m, map_y - thymio_clearance_m)
    obstacles.append([border0, border1, border2, border3])
    
    for i in range(0,len(vertices_array)):
        polygon = []
        for j in range(0,len(vertices_array[i])):
            x = vertices_array[i][j][0]
            y = vertices_array[i][j][1]
            polygon.append(vg.Point(x, y ))
        obstacles.append(polygon)
    return obstacles

def build_visgraph(obstacles):
    graph = vg.VisGraph()
    graph.build(obstacles)
    return graph

def apply_astar(graph, start, goal):
    start_point = vg.Point(start[0],start[1])
    goal_point = vg.Point(goal[0],goal[1])
    return graph.shortest_path(start_point,goal_point)