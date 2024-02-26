import numpy as np
import time
import control as ctl
import numpy as np
from collections import deque
import asyncio
import math
class PidController: 
     def __init__(self,Pre_offset=None,Pre_time=None,Pre_I=None,Pre_D=None):
            self.Pre_offset = Pre_offset
            self.Pre_time = Pre_time
            self.Pre_I = Pre_I
            self.Pre_D = Pre_D
    
     @staticmethod
     def calculate_parameters(Center_X,set_point):
         Offset_x = set_point-Center_X     
         return Offset_x 
    
    
     def PID_controller(self,Offset,P=0,I=0,D=0,scale_devide=1, margin=0):
          time_now = time.time()
          P = P/scale_devide
          I = I/scale_devide
          D = D/scale_devide
          margin = margin/scale_devide
          if self.Pre_time is None:
               Output = P*Offset
               u_I = 0
               u_D = 0
          else:
               u_I=self.Pre_I+I*Offset*(time_now-self.Pre_time)
               u_D = D*(Offset-self.Pre_offset)
               Output = P*Offset+u_I+u_D

          self.Pre_offset = Offset
          self.Pre_time = time_now
          self.Pre_I = u_I
          self.Pre_D = u_D

          if abs(Output) < margin:
               Output = 0.0
          return Output


# class transfer_funtion_class:
#     def __init__(self, numerator, denominator):
#         ss = ctl.tf2ss(numerator,denominator)
#         self.A = ss.A
#         self.B = ss.B
#         self.C = ss.C
#         self.D  = ss.D
#         self.x = np.zeros((self.A.shape[0],))
#         self.pre_time = 0

#     def impliment_transfer_function(self, input):
#         time_now = time.time()
#         t_s = time_now - self.pre_time

#         # State update equation: x(k+1) = Ax(k) + Bu(k)
#         self.x = np.dot(self.A, self.x) * t_s + np.dot(self.B, input) * t_s

#         # Output equation: y(k) = Cx(k) + Du(k)
#         output = np.dot(self.C, self.x) + self.D * input

#         self.pre_time = time_now
#         return output[0][0]

class transfer_funtion_class: 
    def __init__(self,numerator,denominator): 
        self.numerator = numerator
        self.denominator = denominator
        ss = ctl.tf2ss(numerator,denominator)
        self.A = np.array(np.array([[-5.731, -6.473, -0.003582, -0.00167],
                          [7.195, -0.02194, -8.354, -0.7413],
                          [-2.26, 8.258, -13.42, -83.66],
                          [0.3076, -0.002601, -1.61, -10.72]]))
        print(self.A)
        self.B = np.array(np.array([[0.5814],[-0.5942],[0.5476],[-0.003237]]))
        self.C = np.array(np.array([[8.979, -0.02903, 0.0001338, 4.864e-7]]))
        self.D  = np.array(np.array([[0]]))
        self.x = np.zeros((self.A.shape[0],))
        self.pre_time = 0
        

    def impliment_transfer_function(self,input): 
        time_now = time.time()
        t_s = time_now-self.pre_time
          
        # State update equation: x(k+1) = Ax(k) + Bu(k)      
        dx = np.dot(self.A,self.x) + np.dot(self.B,input)  
        self.x = self.x + dx * t_s #Discrete-time state update
        # output equation: y(k) = Cx(k) + Du(k)
        output = np.dot(self.C,self.x) + self.D * input  
        self.pre_time=time_now
        return output[0][0]
    
class Quaters:
     @staticmethod
     def quaternion_to_euler(x, y, z, w):
          """
          Convert a quaternion into euler angles (roll, pitch, yaw)
          roll is rotation around x in radians (counterclockwise)
          pitch is rotation around y in radians (counterclockwise)
          yaw is rotation around z in radians (counterclockwise)
          """
          t0 = +2.0 * (w * x + y * z)
          t1 = +1.0 - 2.0 * (x * x + y * y)
          roll_x = math.atan2(t0, t1)
          t2 = +2.0 * (w * y - z * x)
          t2 = +1.0 if t2 > +1.0 else t2
          t2 = -1.0 if t2 < -1.0 else t2
          pitch_y = math.asin(t2)
          t3 = +2.0 * (w * z + x * y)
          t4 = +1.0 - 2.0 * (y * y + z * z)
          yaw_z = math.atan2(t3, t4)
          return roll_x, pitch_y, yaw_z