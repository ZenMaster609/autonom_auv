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
     def calculate_parameters(measured_value,set_point):
         Offset = set_point-measured_value  
         return Offset
    
    
     def PID_controller(self,e,P=0,I=0,D=0,T_f=0.5,scale_devide=1, margin=0):
          time_now = time.time()
          P = P/scale_devide
          I = I/scale_devide
          D = D/scale_devide
          margin = margin/scale_devide
          
          if self.Pre_time is None:
               Output = P*e
               u_I = 0
               u_D = 0
               e_f=e 
          else:
               T_s = time_now-self.Pre_time
               u_P = P*e 
               u_I=self.Pre_I+I*e*T_s/2
               e_f = (1/(1+(T_s/T_f)))*self.Pre_e_f +((T_s/T_f)/(1+(T_s/T_f)))*e 
               u_D = D*(e-self.Pre_offset)
               Output = u_P+u_I+u_D


          self.Pre_offset = e
          self.Pre_time = time_now
          self.Pre_e_f = e_f
          self.Pre_I = u_I
          self.Pre_D = u_D

          if abs(Output) < margin:
               Output = 0.0
          return Output

class transfer_funtion_class: 
    def __init__(self,numerator,denominator): 
        self.numerator = numerator
        self.denominator = denominator
        TF = ctl.TransferFunction(numerator,denominator)
        ctl.c2d(TF,0.25,method="zoh")
        self.ss = ctl.tf2ss(TF)
        self.A = self.ss.A
        self.x = np.zeros(self.A.shape[0],)
        self.pre_time = time.time()
        

    def impliment_transfer_function(self,input): 
        time_now = time.time()
        t_s = time_now-self.pre_time
        ss = ctl.c2d(self.ss,t_s,"foh")
        self.A =ss.A
        self.B =ss.B
        self.C =ss.C
        self.D =ss.D
        # State update equation: x(k+1) = Ax(k) + Bu(k)      
        self.x  = np.dot(self.A,self.x) + np.dot(self.B,input)  
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