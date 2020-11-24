import math
import itertools 

def ccw(A,B,C):
	return (C.y-A.y)*(B.x-A.x) > (B.y-A.y)*(C.x-A.x)

def intersect(A,B,C,D):
    if ([A.x,A.y] == [C.x,C.y] and [B.x,B.y] == [D.x,D.y]) or ([A.x,A.y] == [D.x,D.y] and [B.x,B.y] == [C.x,C.y]):
        return False
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

    def intersect_polygon(self,A,B):
        
        segments = list(itertools.combinations(self.point,2))
        for i in range(0,len(segments)):
            if intersect(A,B,segments[i][0],segments[i][1]):
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
    visible_points = []
    
    for i in range(0,len(obstacles)):
        
        for j in range(0,len(obstacles[i].point)):
            visibility = [True]*len(obstacles)
            
            for k in range(0,len(obstacles)):
                
                if obstacles[k].intersect_polygon(A,obstacles[i].point[j]):
                    visibility[k] = False
                    
            if min(visibility):
                visible_points.append(segment(A,obstacles[i].point[j]))
                
    return visible_points

#def print_visible_points(visible_points):
    

#def build_visibility_graph(vertices_array, start_pos, goal_pos):
        # vertices_array is a list that contains polygon vertices which are
        # themselves lists of points[[[x,y],[x,y],[x,y]],[[x,y],[x,y],[x,y]]]
        
        # start is the starting position in the form [x,y]  
        # goal is the target position in the form [x,y]
        
    
    
#def get_path():

    # input :
    #_____________________________________________________________________
    # return : list of coordonates of target points ==> target_point_array