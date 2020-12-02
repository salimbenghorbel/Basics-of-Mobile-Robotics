#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#https://github.com/TaipanRex/pyvisgraph

import pyvisgraph as vg

def build_obstacles(vertices_array):
    obstacles = []
    for i in range(0,len(vertices_array)):
        polygon = []
        for j in range(0,len(vertices_array[i])):
            polygon.append(vg.Point(vertices_array[i][j][0],vertices_array[i][j][1] ))
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