import math
import time
import numpy as np
import pyvisgraph as vg

class robot:
    
    
    def __init__(self,th, all_target_points, x_0, y_0, theta_0, v, vertices_array,verbose = True):
        self.x = x_0
        self.y = y_0
        self.theta = theta_0
        self.all_target_points = all_target_points
        self.target_point = [0,0]
        self.th = th
        self.verbose = verbose
        self.v = v
        self.vertices_array = vertices_array
        self.obstacles = vg.build_obstacles(vertices_array)
      
       
    
    def find_next_target_point(self):
        i = self.all_target_points.index(self.target_point)
        self.target_point[0] = self.all_target_points[i+1][0]    
        self.target_point[1] = self.all_target_points[i+1][1]
        print('j')
        
    def turn_to_target_point(self):
        theta_goal = math.atan2(self.target_point[1] - self.y, self.target_point[0] - self.x)
        alpha = theta_goal - self.theta
        self.turn(alpha)
        print(alpha)
        return alpha
    

    def advance_to_target_point(self):
        d_x = self.target_point[0] - self.x
        d_y = self.target_point[1] - self.y
  
        d = math.sqrt(math.pow(d_x,2)+math.pow(d_y,2))
        self.run_forward(d)
    

    def on_target_point(self):
        R = 0.05
        d_x = self.target_point[0] - self.x
        d_y = self.target_point[1] - self.y
        d = math.sqrt(math.pow(d_x,2)+math.pow(d_y,2))
        if d < R:
            return True
        else:
            return False

    def on_goal(self):
        R = 0.05
        d_x = self.all_target_points[-1][0] - self.x
        d_y = self.all_target_points[-1][1] - self.y
        d = math.sqrt(math.pow(d_x,2)+math.pow(d_y,2))
        if d < R:
            return True
        else:
            return False
    
    def check_prox(self):
        prox_sensors = self.th["prox.horizontal"][0:5]
        "Checks if horizontal proximity sensors see something"
        
        if max(prox_sensors) != 0:
        
            return True
        else: 
       
            return False
    
    def local_avoidance(self):
             
        """
        Implement sequential strategy
        
        Input: proximity sensor values
        Output: Thymio dodges obstacle with a predefined sequence of movements
        """
        print('local avoidance')
        if self.verbose: print("\t Entering local avoidance")
        """ --- Storing sensors value for convenience --- """
        prox_sensors = self.th["prox.horizontal"][0:5]
        front_sensor = prox_sensors[2]
        left_sensors = prox_sensors[0:2]
        right_sensors = prox_sensors[3:5]
        side_sensors = []
        for i in range(len(prox_sensors)):
            if i == 2: continue
            else: side_sensors .append(prox_sensors[i])
        
        d = 0
        angle = 0
        
        """ --- Choosing avoidance strategy --- """
        if front_sensor!=0 and max(side_sensors) == 0:   
            if self.verbose: print("\t Saw something in right in front, turn")
            d = 0.1
            angle = np.pi/3 #has to turn a lot to be sure to avoid obstacle
            
        elif max(left_sensors)!=0:
            if self.verbose: print("Saw something at left side, turning right")
            d = 0.1
            angle = -np.pi/4 # obstacle seen at Thymio's side = no need for big angle to avoid it
            
        elif max(right_sensors)!=0:
            if self.verbose: print("Saw something at right side, turning left")
            d = 0.1
            angle = np.pi/4 # obstacle seen at Thymio's side = no need for big angle to avoid it
       
        """ --- Launching avoidance strategy --- """
        self.dodge_sequence(d,angle)
        if self.verbose: 
            print("\t Obstacle dodged, going back to global navigation")
    
    def dodge_sequence(self, d, angle):
        """
        Inputs: d = distance to travel straight [m], angle = angle to turn and avoid obstacle [rad]
        Output: Thymio executes dodging sequence
        
        """

        self.turn(angle)
        self.run_forward(d)
        self.turn_to_target_point()
        self.advance_to_target_point()
   
   
        
    def forward(self):
        self.th.set_var("motor.left.target", 100)
        self.th.set_var("motor.right.target", 100)
    
    def stop(self):
        self.th.set_var("motor.left.target", 0)
        self.th.set_var("motor.right.target", 0)

    def clockwise(self):
        self.th.set_var("motor.left.target", 2**16-100)
        self.th.set_var("motor.right.target", 100)
    
    def anticlockwise(self):
        self.th.set_var("motor.left.target", 100)
        self.th.set_var("motor.right.target", 2**16-100)


    def run_forward(self,d):
        v = self.v
        dt = d/v 
        t0 = time.time()
        t1 = 0
        t = 0   
        while t < dt and not self.check_prox():# and self.avoid_global_obstacle():
            t1 = time.time()
            delta_t = t1 - t0 - t
            self.forward()
            t = t1 - t0
            self.odometry(delta_t)
        if t>= dt:
            self.stop()
        else: #if got out of "while" because saw something, enter local avoidance
            self.local_avoidance()
   
    def turn(self,alpha):
    
        t0 = time.time()
        t1 = 0
        t = 0
        theta_init = self.theta
        if alpha > 0:
            while self.theta < theta_init + alpha:
                t1 = time.time()
                delta_t = t1 - t0 - t
                self.clockwise()
                t = t1 - t0
                self.odometry(delta_t)
            self.stop()
        else:
        
            while self.theta > theta_init + alpha:
                t1 = time.time()
                delta_t = t1 - t0 - t
                self.anticlockwise()
                t = t1 - t0
                self.odometry(delta_t)
            self.stop()

    #def avoid_global_obstacle(self):
     #  p = vg.Point( self.x + 0.05 * math.cos(self.theta), self.y + 0.05 * math.sin(self.theta))
      #  graph = vg.VisGraph()
       # graph.build(self.obstacles)
        #obstacle = vg.point_in_polygon(p, graph)
        ##   return True
        #else: 
         #   return False
          #  print('global obstacle')
        
    def odometry(self,delta_t):
        b = 0.095
        v = self.v
        s_r = self.th.get_var('motor.right.speed')
        s_l = self.th.get_var('motor.left.speed') 
        if s_r > 2**8:
            s_r = s_r/2**16 - 100
        if s_l > 2**8:
            s_l = s_l/2**16 - 100
        d_r = s_r/100 * v * delta_t
        d_l = s_l/100 * v * delta_t
      
        d_s = (d_r + d_l)/2
    
        d_theta = (d_r - d_l)/b
        self.x = self.x  + d_s * math.cos(self.theta + d_theta/2)
        self.y = self.y  + d_s * math.sin(self.theta + d_theta/2)
        self.theta = self.theta + d_theta

    
    
    
    
        