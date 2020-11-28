import math
import itertools 

def ccw(A,B,C):
	return (C.y-A.y)*(B.x-A.x) > (B.y-A.y)*(C.x-A.x)

def intersect(A,B,C,D):
    if ([A.x,A.y] == [C.x,C.y] or [B.x,B.y] == [D.x,D.y]) or ([A.x,A.y] == [D.x,D.y] or [B.x,B.y] == [C.x,C.y]):
        #print('same segment or point in common')
        return False       
    else:
        #print('different segment')
        return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
    
class Point:
	def __init__(self,x,y):
		self.x = x
		self.y = y
        
def build_point(i,j, vertices_array):
    return Point(vertices_array[i][j][0],vertices_array[i][j][1])

def segment_length(A,B):
    return math.sqrt((A.x-B.x)**2+(A.y-B.y)**2)

class segment:
    def __init__(self,A,B):
        self.p1 = A
        self.p2 = B
    def length(self):
        return segment_length(self.p1,self.p2)
    
class polygon:
    
    def __init__(self, vertices_array, polygon_id):
        self.vertices = vertices_array[polygon_id];
        self.nvertices = len(self.vertices) 
        self.point = []
        for i in range(0,self.nvertices):
            self.point.append(build_point(polygon_id,i, vertices_array))

    def intersect(self,A,B):
        
        segments = list(itertools.combinations(self.point,2))
        for i in range(0,len(segments)):
            #print('iteration')
            if intersect(A,B,segments[i][0],segments[i][1]):
                #print('intersection!',segments[i][0].x,segments[i][0].y,';',segments[i][1].x,segments[i][1].y)
                return True
                break 
            
        return False
        
        # if intersect(A, B, self.point[0], self.point[self.nvertices-1]):
        #     return True
        
        # for i in range(0,self.nvertices-1):
        #     if intersect(A,B,self.point[i],self.point[i+1]):
        #         return True
        #         break 
            
        # return False
    
def build_obstacles(vertices_array):
    obstacle_table = []
    for i in range(0,len(vertices_array)):
        obstacle_table.append(polygon(vertices_array,i))
    return obstacle_table

def visible_points(A, vertices_array):
    
    obstacles = build_obstacles(vertices_array)
    vis_points = []
    
    for i in range(0,len(obstacles)):
        
        for j in range(0,len(obstacles[i].point)):
            visibility = [True]*len(obstacles)
            
            for k in range(0,len(obstacles)):
                
                if obstacles[k].intersect(A,obstacles[i].point[j]):
                    visibility[k] = False
                    
            if min(visibility) and [A.x,A.y] != [obstacles[i].point[j].x,obstacles[i].point[j].y]:
                vis_points.append(segment(A,obstacles[i].point[j]))
                
    return vis_points

def print_visible_points(vis_points):
    for i in range(0, len(vis_points)):
        print('[',vis_points[i].p2.x,',',vis_points[i].p2.y,']')
        
def print_point_table_coordinates(point_table):
    for i in range(0, len(point_table)):
        print('[',point_table[i].x,',',point_table[i].y,']')
        
def point_talbe_to_list(point_table):
    coord_list = []
    for i in range(0, len(point_table)):
        coord_list.append((point_table[i].x,point_table[i].y))
    return coord_list
    
def intersection(lst1, lst2): 
    return list(set(lst1) & set(lst2)) 

def build_visibility_graph(vertices_array, start_xy, goal_xy):
    
    # vertices_array is a list that contains polygon vertices which are
    # themselves lists of points[[[x,y],[x,y],[x,y]],[[x,y],[x,y],[x,y]]]
        
    # start is the starting position in the form [x,y]  
    # goal is the target position in the form [x,y]
    
    start = Point(start_xy[0],start_xy[1])
    goal = Point(goal_xy[0],goal_xy[1])
    
    # getting visible points from starting poiint
    vis_points = visible_points(start, vertices_array)
    visible_points_from_start = []
    for i in range(0, len(vis_points)):
        visible_points_from_start.append((vis_points[i].p2.x,vis_points[i].p2.y))
    

    # getting visible points from goal
    vis_points = visible_points(goal, vertices_array)
    visible_points_from_goal = []
    for i in range(0, len(vis_points)):
        visible_points_from_goal.append((vis_points[i].p2.x,vis_points[i].p2.y))
    
    inter_table = intersection(visible_points_from_start, visible_points_from_goal)
    inter_points = []
    valid_paths = []
    for i in range(0, len(inter_table)):
        inter_points.append(Point(inter_table[i][0],inter_table[i][1]))
        valid_paths.append((start,inter_points[i],goal))
    
    valid_paths_lengths = []
    for i in range(0, len(valid_paths)):
        valid_paths_lengths.append(segment_length(valid_paths[i][0],valid_paths[i][1])+segment_length(valid_paths[i][1],valid_paths[i][2]))
        
    min_index = valid_paths_lengths.index(min(valid_paths_lengths))
    
    set_points = []
    for i in range(0,len(valid_paths[min_index])):
        set_points.append([valid_paths[min_index][i].x,valid_paths[min_index][i].y])
        
    return set_points
#def get_path():

    # input :
    #_____________________________________________________________________
    # return : list of coordonates of target points ==> target_point_array