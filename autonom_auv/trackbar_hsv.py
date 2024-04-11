import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os
from .image_methods import ImageMethods
def nothing(x):
    pass

class DynamicDisplay:

    @staticmethod
    def save_parameters(my_parameters):
        """Saves current parameters in tex file in config folder under install"""
        config_path = os.path.join(get_package_share_directory('autonom_auv'), 'config', 'image_settings.tex')
        my_parameters_str = str(my_parameters)
        with open(config_path, 'a') as file:
            file.write(my_parameters_str + '\n')

    @staticmethod
    def trackbar_init():
        """Initialises trackbars"""
        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
        cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
        cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
        cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

    @staticmethod
    def find_hsv(image):
        """use this to find HSV ranges live using trackbars"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")
        lower_range = np.array([l_h, l_s, l_v])
        upper_range = np.array([u_h, u_s, u_v])
        mask = cv2.inRange(hsv, lower_range, upper_range)
        res = cv2.bitwise_and(image, image, mask=mask)
        # Converting the binary mask to 3 channel scaled_image
        mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        stacked = np.hstack((mask_3, image, res))
        cv2.imshow('Trackbars', cv2.resize(stacked, None, fx=0.3, fy=0.3))
        key = cv2.waitKey(1)
        if key == ord('s'):
            thearray = [[l_h,l_s,l_v],[u_h, u_s, u_v]]
            DynamicDisplay.save_parameters(thearray)
