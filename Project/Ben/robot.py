import math
import time

class robot:
    
    
    def __init__(self,th, all_target_points, x_0, y_0, theta_0):
        self.x = x_0
        self.y = y_0
        self.theta = theta_0
        self.all_target_points = all_target_points
        self.target_point = [0,0]
        self.th = th
       
    
    def find_next_target_point(self):
        i = self.all_target_points.index(self.target_point)
        self.target_point[0] = self.all_target_points[i+1][0]    
        self.target_point[1] = self.all_target_points[i+1][1]
        
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
        print('local avoidance')
        self.turn(1.507)
        self.run_forward(0.1)
        self.stop()
        
    def odometry_forward(self,delta_t,v,dt):
        self.x = self.x + v * delta_t * math.cos(self.theta)
        self.y = self.y + v * delta_t * math.sin(self.theta)

    def odometry_angle(self,delta_t,alpha,dt):
        self.theta = self.theta + delta_t * alpha /dt
 
    def forward(self):
        self.th.set_var("motor.left.target", 100)
        self.th.set_var("motor.right.target", 100)
    
    def stop(self):
        self.th.set_var("motor.left.target", 0)
        self.th.set_var("motor.right.target", 0)

    def clockwise(self):
        self.th.set_var("motor.left.target", 2**16-100)
        self.th.set_var("motor.right.target", 102)
    
    def anticlockwise(self):
        self.th.set_var("motor.left.target", 100)
        self.th.set_var("motor.right.target", 2**16-102)


    def run_forward(self,d):
        v = 0.03375
        dt = d/v 
        t0 = time.time()
        t1 = 0
        t = 0   
        while t < dt and self.check_prox() == False:
            t1 = time.time()
            delta_t = t1 - t0 - t
            self.forward()
            t = t1 - t0
            self.odometry_forward(delta_t,v,dt)
        if t>= dt:
            self.stop()
        else:
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
                self.odometry_angle(delta_t,alpha,dt)
            self.stop()
        else:
        
            while t < dt:
                t1 = time.time()
                delta_t = t1 - t0 - t  
                self.anticlockwise()
                t = t1 - t0
                self.odometry_angle(delta_t,alpha,dt)
            self.stop()
  
    
        