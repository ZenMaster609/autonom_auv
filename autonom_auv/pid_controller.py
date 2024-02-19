import numpy as np
import time


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

    