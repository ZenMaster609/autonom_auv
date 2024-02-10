import cv2 
import cv2.aruco as aruco
import numpy as np
import time


class PidControllerNode: 

    def calculate_parameters(Center_X,dimensions):
         Offset_x= dimensions[1]/2-Center_X     
         return Offset_x 

    #def PID_controller(Offset_x,P=0,I=0,D=0):
        
    