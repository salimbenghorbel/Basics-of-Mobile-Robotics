import math
import time
import numpy as np

class robot:
    
    
    def __init__(self, th, all_target_points, x_0, y_0, theta_0, verbose = True):
        self.x = x_0
        self.y = y_0
        self.theta = theta_0
        self.all_target_points = all_target_points
        self.target_point = [0,0]
        self.vreal_max = 0.1525 # max measured speed of thymio in [m/s]
        self.th = th
        self.verbose = verbose
        self.cL = None
        self.cR = None
    
    def find_next_target_point(self):
              
        if self.verbose: print("Next target point in list")
        i = self.all_target_points.index(self.target_point)
        self.target_point[0] = self.all_target_points[i+1][0]    
        self.target_point[1] = self.all_target_points[i+1][1]
        
    def turn_to_target_point(self):
        
        if self.verbose: print("\t Orienting toward target point")
        theta_goal = math.atan2(self.target_point[1] - self.y, self.target_point[0] - self.x)
        alpha = theta_goal - self.theta
        self.turn(alpha)
        return alpha
    

    def advance_to_target_point(self):
        if self.verbose: print("\t Going to target point")
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
        
        if max(prox_sensors) != 0:
            return True
        else:
            return False

 
    def forward(self, l_speed = 100, r_speed = 100):
        self.th.set_var("motor.left.target", l_speed)
        self.th.set_var("motor.right.target", r_speed)
    
    def stop(self):
        self.th.set_var("motor.left.target", 0)
        self.th.set_var("motor.right.target", 0)

    def clockwise(self, l_speed = 100, r_speed = 100):
        self.th.set_var("motor.left.target", 2**16-l_speed)
        self.th.set_var("motor.right.target", r_speed)
    
    def anticlockwise(self, l_speed = 100, r_speed = 100):
        self.th.set_var("motor.left.target", l_speed)
        self.th.set_var("motor.right.target", 2**16-r_speed)
        
    def dodge_sequence(self, d, angle):
        """
        Inputs: d = distance to travel straight [m], angle = angle to turn and avoid obstacle [rad]
        Output: Thymio executes dodging sequence
        
        """
        if self.verbose: print("\t Performing dodge sequence")
        self.turn(angle)
        self.run_forward(d)
        #self.turn(-angle)
        #self.run_forward(d)
        self.turn_to_target_point()
        self.advance_to_target_point()
        
    
    def local_avoidance(self):
        
        """
        Implement sequential strategy
        
        Input: proximity sensor values
        Output: Thymio dodges obstacle with a predefined sequence of movements
        """
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
        d_front = 0.2
        d_side = 0.1
        angle = 0
        angle_side = np.pi/2
        angle_front = np.pi/2
        
        """ --- Choosing avoidance strategy --- """
        if front_sensor!=0 and max(side_sensors) == 0:   
            if self.verbose: print("\t Saw something in right in front, turn")
            d = d_front
            angle = angle_front #has to turn a lot to be sure to avoid obstacle
            
        elif max(left_sensors)!=0:
            if self.verbose: print("Saw something at left side, turning right")
            d = d_side
            angle = -angle_side # obstacle seen at Thymio's side = no need for big angle to avoid it
            
        elif max(right_sensors)!=0:
            if self.verbose: print("Saw something at right side, turning left")
            d = d_side
            angle = angle_side # obstacle seen at Thymio's side = no need for big angle to avoid it
       
        """ --- Launching avoidance strategy --- """
        self.dodge_sequence(d,angle)
        
        if self.verbose: print("\t Obstacle dodged, going back to global navigation")
        
    def run_forward(self,d):
        v = 0.03375
        dt = d/v 
        t0 = time.time()
        t1 = 0
        t = 0   
        while t < dt and not self.check_prox():
            t1 = time.time()
            delta_t = t1 - t0 - t
            self.forward()
            t = t1 - t0
            self.new_odometry(delta_t)
        if t>= dt:
            self.stop()
        else: #if got out of "while" because saw something, enter local avoidance
            self.local_avoidance()
   
    def turn(self,alpha):
        R = 0.047
        v = 0.03050
        dt = abs(R * alpha / v)
        t0 = time.time()
        t1 = 0
        t = 0
        if alpha > 0:
            while t < dt:
                t1 = time.time()
                delta_t = t1 - t0 - t 
                self.clockwise()
                t = t1 - t0
                self.new_odometry(delta_t)
            self.stop()
            
        else:
        
            while t < dt:
                t1 = time.time()
                delta_t = t1 - t0 - t  
                self.anticlockwise()
                t = t1 - t0
                self.new_odometry(delta_t)
            self.stop()
           
    def odometry_forward(self,delta_t,v):
        self.x = self.x + v * delta_t * math.cos(self.theta)
        self.y = self.y + v * delta_t * math.sin(self.theta)

    def odometry_angle(self,delta_t,alpha,dt):
        self.theta = self.theta + delta_t * alpha /dt
        
    def new_odometry(self,delta_t):
        b = 0.095
        v = 0.03050
        d_r = self.th.get_var('motor.right.speed')/100 * v * delta_t
        d_l = self.th.get_var('motor.left.speed')/100 * v * delta_t
        d_s = (d_r + d_l)/2 
        d_theta = (d_r - d_l)/b
        d_theta = math.radians(d_theta)
        print(d_theta)
        self.x = self.x  + d_s * math.cos(self.theta + d_theta/2)
        self.y = self.y  + d_s * math.sin(self.theta + d_theta/2)
        self.theta = self.theta + d_theta
     
        
        
        
        
        
        
        
        
        
        