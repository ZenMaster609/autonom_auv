import math
import cv2 
import cv2.aruco as aruco
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os
import time
class ImageMethods: 

    @staticmethod    
    def scale_image(image, scale_factor = 0.5):
        """Scales image"""
        new_width = int(image.shape[1] * scale_factor)
        new_height = int(image.shape[0] * scale_factor)
        resized_image = cv2.resize(image, (new_width, new_height))
        return resized_image

    @staticmethod
    def fix_hsv(image):
        """Gives HSV image a third dimention so it can be merged with color pictures for dual display"""
        fixed_hsv = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        return fixed_hsv
    
    @staticmethod
    def saveImage(image):
        "saves image"
        photos_path = os.path.join(get_package_share_directory('autonom_auv'), 'photos', f"cam2_{time.time()}.jpg")
        cv2.imwrite(photos_path, image)

    @staticmethod
    def showImage(image,scale=1):
        """Saves displayed image by pressing 's' """
        image = cv2.resize(image, (0, 0),fx=scale,fy=scale)
        cv2.imshow("window", image)
        key = cv2.waitKey(1)
        if key == ord('s'):
            ImageMethods.saveImage(image)

    @staticmethod
    def close_image(image, rate):
        """Fills holes in image"""
        kernel = np.ones((rate, rate), np.uint8)  # Kernel size affects the amount of merging
        closed_image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)
        return closed_image

    @staticmethod
    def stack_images(images,):
        """Takes a list of images and stacks them"""
        images = [img for img in images if img is not None]
        rows = (len(images) + 1) // 2
        cols = min(2, len(images))
        row_images = []
        for i in range(0, len(images), 2):
            imgs_to_stack = images[i:i+2]
            if len(imgs_to_stack) == 1:  
                imgs_to_stack.append(np.zeros_like(images[0])) 
            row_images.append(np.hstack(imgs_to_stack))
        image_show = np.vstack(row_images) if len(row_images) > 1 else row_images[0]
        return image_show


    @staticmethod 
    def color_filter(image, range:list, debug = False):
        "Takes in an image and a HSV range, return a black and white image"  
        image_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        HSV_lower = np.array(range[0])
        HSV_upper = np.array(range[1])
        mask = cv2.inRange(image_HSV,HSV_lower,HSV_upper)
        maskM = cv2.medianBlur(mask, 5)
        if debug:return maskM, image_HSV
        else:return maskM


    
    @staticmethod
    def find_boxes(hsv_image, Original_image, min_box_size, draw:bool):
        """
        Takes in a black and white image and the original image, returns the original image with drawn boxes, 
        an image with box approximation, and a list of boxes that have an area greater than min_box_size.
        """
        box_list = []
        contours, _ = cv2.findContours(hsv_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours is not None:
            for cnt in contours:
                rect = cv2.minAreaRect(cnt)
                box = np.intp(cv2.boxPoints(rect))
                area = cv2.contourArea(box)
                if area > min_box_size:
                    box_list.append(box)
                    if draw:
                        cv2.drawContours(Original_image, [box], -1, (0, 0, 255), thickness=2)
        return box_list

 
    @staticmethod
    def find_biggest_box(image, boxes:list, draw:bool):
        """Finds biggest box in a list of boxes"""
        max_area = 0
        biggest_box = None
        for box in boxes:
            area = cv2.contourArea(box)
            if area > max_area:
                biggest_box = box
                max_area = area

        if biggest_box is not None and draw:
            cv2.drawContours(image, [biggest_box], -1, (0, 255, 0), thickness=3)
        return biggest_box 



    @staticmethod
    def find_highest_box(boxes):
        "Takes in a list of boxes return the boxes highest in the picture"
        min_y_value=1920
        if len(boxes)>0:
            for i in range(len(boxes)):
                y_value_list = boxes[i][:,1]
                if min(y_value_list)<=min_y_value: 
                    min_y_value = min(y_value_list)
                    box_index = i 
            box=boxes[box_index]
            return box 
            
        else: return None

    @staticmethod
    def get_box_info(box):
        """
        Takes a box as an argument and returns the positions of each of its four corners and the middle points of each of its four lines.
        Parameters:
        - box: A NumPy array of shape (4, 2) representing the box's four corners.
        Returns:
        - A dictionary with keys 'top_left', 'top_right', 'bottom_right', 'bottom_left'
        corresponding to the coordinates of each corner and 'middle_top', 'middle_right', 'middle_bottom', 'middle_left' for the middle points of each line.
        """
        area = cv2.contourArea(box)
        # Sort the box points based on their x-coordinates (helps in identifying left/right)
        sorted_box = sorted(box, key=lambda x: x[0])
        # Split the sorted points into leftmost and rightmost
        left_points = sorted_box[:2]
        right_points = sorted_box[2:]
        # Sort the left_points and right_points by their y-coordinates to separate top/bottom
        top_left, bottom_left = sorted(left_points, key=lambda x: x[1])
        top_right, bottom_right = sorted(right_points, key=lambda x: x[1])

        # Calculate the middle points of each side
        middle_top = ((top_left[0] + top_right[0]) / 2, (top_left[1] + top_right[1]) / 2)
        middle_right = ((top_right[0] + bottom_right[0]) / 2, (top_right[1] + bottom_right[1]) / 2)
        middle_bottom = ((bottom_left[0] + bottom_right[0]) / 2, (bottom_left[1] + bottom_right[1]) / 2)
        middle_left = ((top_left[0] + bottom_left[0]) / 2, (top_left[1] + bottom_left[1]) / 2)
        
        # Calculate the center of the box
        center_x = (top_left[0] + top_right[0] + bottom_left[0] + bottom_right[0]) / 4
        center_y = (top_left[1] + top_right[1] + bottom_left[1] + bottom_right[1]) / 4
        center = (center_x, center_y)

        # Return the corners and middle points in a structured dictionary
        positions = {
            'top_left': tuple(top_left),
            'top_right': tuple(top_right),
            'bottom_right': tuple(bottom_right),
            'bottom_left': tuple(bottom_left),
            'middle_top': middle_top,
            'middle_right': middle_right,
            'middle_bottom': middle_bottom,
            'middle_left': middle_left,
            'center' : center
        }
        return positions, area





    @staticmethod
    def find_Center(image, box, draw:bool): 
        "Draw a dot in the center of a box, return Center cordinates"  
        if box is not None:
            center=((box[0]+box[2])/2)
            Center_X=int(center[0])
            Center_Y=int(center[1])
            cv2.circle(image,(Center_X,Center_Y),10,(0,0,255),-1)
            return Center_X,Center_Y
        else: return 960,600


    @staticmethod
    def find_angle_box(box,offset=0, width= 960):
        if box is not None:
            "Finds the angle of the object between the horizontal frame and the longest vector"
            Vec_1=box[0]-box[1]
            Vec_2=box[1]-box[2]
            lenght_Vec_1=math.sqrt(Vec_1[0]**2+Vec_1[1]**2)
            lenght_Vec_2=math.sqrt(Vec_2[0]**2+Vec_2[1]**2)

            if lenght_Vec_1>lenght_Vec_2:
                diff = (box[0][0]+box[1][0]-width)   
                Vector = Vec_1
                lenght_Vec = lenght_Vec_1   
            else:
                diff=(box[1][0]+box[2][0]-width)
                Vector = Vec_2
                lenght_Vec = lenght_Vec_2
                
            direction=np.sign(Vector[1])*(-1)
            if direction==0:
                if diff>0:
                    direction=-1
                else: direction=1
            angle=math.acos(Vector[0]/lenght_Vec)
            angle_deg=(angle*360/2/math.pi-offset)*direction
            angle = math.radians(angle_deg)
            return angle, angle_deg
        else: return 0 

    @staticmethod
    def angle_cooldown(angle,Cooldown):
        """makes sure angle calculating functions dont confuse themselves about 90 and -90 degrees"""
        if Cooldown == 0:
            if angle==-math.pi/2:
                Cooldown = -40
            elif angle == math.pi/2:
                Cooldown = 40

        if angle==-math.pi/2 or angle ==math.pi/2:
            if Cooldown > 0:
                angle=abs(angle)
                Cooldown -= 1 
            if Cooldown < 0:
                angle=-abs(angle)
                Cooldown += 1
        else:
            if Cooldown > 0:
                Cooldown -= 1 
            elif Cooldown < 0:
                Cooldown += 1
            else:
                Cooldown = 0
        return angle, Cooldown

    @staticmethod
    def read_AruCo(image,id_list):
        "Takes the image and reads the aruco code and adds the Id to the given list"
        gray= cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        dict= aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        corners, ids, rejected = aruco.detectMarkers(gray, dict)
        
        if ids is not None and len(ids) > 0:
            aruco.drawDetectedMarkers(image,corners,ids)
            id_list.append(ids[0][0])
        return id_list
    

    
    @staticmethod
    def filtered_ids_list(list):
        "Filter the list and gives back a list without duplicates"
        filtered_list=[]
        for i in range(len(list)):
            if filtered_list.count(list[i]) < 1:
                    filtered_list.append(list[i])
        return filtered_list
    

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
    

    @staticmethod
    def pixles_to_meters(pixles, distance_from_object, dimension):
        meters = pixles * distance_from_object / dimension
        return meters 
    
    @staticmethod
    def pixles_to_meters69(pixles):
        meters = pixles * 0.01355
        return meters 
    



    
    


    
