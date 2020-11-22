import math
import time

class robot:
    
    
    def __init__(self, all_target_points):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.all_target_points = all_target_points
        self.target_point = [0,0]
    
    def find_next_target_point(self):
      
        i = self.all_target_points.index(self.target_point)
        self.target_point[0] = self.all_target_points[i+1][0]    
        self.target_point[1] = self.all_target_points[i+1][1]
        
    def turn_to_target_point(self,th):
    
        theta_goal = math.atan2(self.target_point[1] - self.y, self.target_point[0] - self.x)
        alpha = theta_goal - self.theta
        self.turn(alpha,th)
        print(alpha)
        return alpha
    

    def advance_to_target_point(self,th):
        d_x = self.target_point[0] - self.x
        d_y = self.target_point[1] - self.y
  
        d = math.sqrt(math.pow(d_x,2)+math.pow(d_y,2))
        self.run_forward(d,th)
    

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
        return True
    
    def local_avoidance(self,th):
        self.stop(th)
        
    def odometry_forward(self,delta_t,v,dt):
        self.x = self.x + v * delta_t * math.cos(self.theta)
        self.y = self.y + v * delta_t * math.sin(self.theta)

    def odometry_angle(self,delta_t,alpha,dt):
        self.theta = self.theta + delta_t * alpha /dt
 
    def forward(self,th):
        th.set_var("motor.left.target", 100)
        th.set_var("motor.right.target", 100)
    
    def stop(self,th):
        th.set_var("motor.left.target", 0)
        th.set_var("motor.right.target", 0)

    def clockwise(self,th):
        th.set_var("motor.left.target", 2**16-100)
        th.set_var("motor.right.target", 102)
    
    def anticlockwise(self,th):
        th.set_var("motor.left.target", 100)
        th.set_var("motor.right.target", 2**16-102)


    def run_forward(self,d,th):
        v = 0.03375
        dt = d/v 
        t0 = time.time()
        t1 = 0
        t = 0   
        while t < dt and self.check_prox():
            t1 = time.time()
            delta_t = t1 - t0 - t
            self.forward(th)
            t = t1 - t0
            self.odometry_forward(delta_t,v,dt)
        if t>= dt:
            self.stop(th)
        else:
            self.local_avoidance(th)
   
    def turn(self,alpha,th):
        R = 0.047
        v = 0.0250
        dt = abs(R * alpha / v)
        t0 = time.time()
        t1 = 0
        t = 0
        if alpha > 0:
      
            while t < dt:
                t1 = time.time()
                delta_t = t1 - t0 - t 
                self.clockwise(th)
                t = t1 - t0
                self.odometry_angle(delta_t,alpha,dt)
                self.stop(th)
        else:
        
             while t < dt:
                t1 = time.time()
                delta_t = t1 - t0 - t  
                self.anticlockwise(th)
                t = t1 - t0
                self.odometry_angle(delta_t,alpha,dt)
                self.stop(th)
  
    
        