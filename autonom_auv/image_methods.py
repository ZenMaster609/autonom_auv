import math
import cv2 
import cv2.aruco as aruco
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os
import time
class ImageMethods: 

    @staticmethod
    def scale_image(image, scale_factor):
        new_width = int(image.shape[1] * scale_factor)
        new_height = int(image.shape[0] * scale_factor)
        resized_image = cv2.resize(image, (new_width, new_height))
        return resized_image

    @staticmethod
    def fix_hsv(image):
        fixed_hsv = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        return fixed_hsv
    
    @staticmethod
    def saveImage(image):
        photos_path = os.path.join(get_package_share_directory('autonom_auv'), 'photos', f"cam2_{time.time()}.jpg")
        cv2.imwrite(photos_path, image)

    @staticmethod
    def showImage(image):
        cv2.imshow("window", image)
        key = cv2.waitKey(1)
        if key == ord('s'):
            ImageMethods.saveImage(image)

    @staticmethod
    def close_image(image, rate):
        kernel = np.ones((rate, rate), np.uint8)  # Kernel size affects the amount of merging
        closed_image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)
        return closed_image

    @staticmethod
    def stack_images(images, scale=0.5):
        """
        Stacks the given images in a grid with a maximum of 2 images per row.
        :param images: List of images to stack. None values are skipped.
        :param scale: Scale factor for resizing the final stacked image.
        :return: The stacked image.
        """
        # Filter out None values from the images list
        images = [img for img in images if img is not None]
        # Calculate the number of rows and columns for the grid
        rows = (len(images) + 1) // 2
        cols = min(2, len(images))
        # Placeholder for row images
        row_images = []
        # Iterate over the rows
        for i in range(0, len(images), 2):
            # Horizontal stack for each row (max 2 images per row)
            imgs_to_stack = images[i:i+2]
            if len(imgs_to_stack) == 1:  # If there's only one image in the last row
                imgs_to_stack.append(np.zeros_like(images[0]))  # Add a blank image
            row_images.append(np.hstack(imgs_to_stack))
        image_show = np.vstack(row_images) if len(row_images) > 1 else row_images[0]
        image_show = cv2.resize(image_show, (0, 0), None, scale, scale)
        return image_show


    @staticmethod 
    def color_filter(Image_inn, lower:list, upper:list):
        "Takes in an image and HSV range, return a black and white image"  
        image_HSV = cv2.cvtColor(Image_inn, cv2.COLOR_BGR2HSV)
        HSV_lower = np.array(lower)
        HSV_upper = np.array(upper)
        mask = cv2.inRange(image_HSV,HSV_lower,HSV_upper)
        maskM = cv2.medianBlur(mask, 5)
        return maskM 


    
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
    def make_stricter_boxes(hsv_image, Original_image, draw:bool):
        contours, _ = cv2.findContours(hsv_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_rectangle = None
        max_area = 0
        for cnt in contours:
            # Approximate the contour to a polygon
            epsilon = 0.05 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            # Check for rectangular shape (4 sides) and aspect ratio
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = float(w) / h
                area = cv2.contourArea(approx)
                # You can adjust the aspect ratio range according to the expected shape
                if 2.0 < aspect_ratio < 4 and area > max_area:
                    best_rectangle = approx
                    max_area = area
        if best_rectangle is not None and draw:
            cv2.drawContours(Original_image, [best_rectangle], -1, (0, 255, 0), thickness=3)
        return best_rectangle   
    

        
    @staticmethod
    def find_biggest_box(image, boxes:list, draw:bool):
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
    def find_the_box(Box_list):
        "Takes in a list of boxes return the boxes highest in the picture"
        min_y_value=1920
        
        if len(Box_list)>0:
            for i in range(len(Box_list)):
                y_value_list = Box_list[i][:,1]
                if min(y_value_list)<=min_y_value: 
                    min_y_value = min(y_value_list)
                    box_index = i 
            box=Box_list[box_index]
            return box 
        else: return None

    @staticmethod
    def get_box_info(box):
        """
        Takes a box as an argument and returns the positions of each of its four corners.
        Parameters:
        - box: A NumPy array of shape (4, 2) representing the box's four corners.
        Returns:
        - A dictionary with keys 'top_left', 'top_right', 'bottom_right', 'bottom_left'
        corresponding to the coordinates of each corner.
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
        # Return the corners in a structured dictionary
        corners = {
            'top_left': tuple(top_left),
            'top_right': tuple(top_right),
            'bottom_right': tuple(bottom_right),
            'bottom_left': tuple(bottom_left)
        }
        return corners, area




    @staticmethod
    def find_Center(Image_inn,The_box, draw:bool): 
        "Draw a dot in the center of the box, return Center cordinates"  
        if The_box is not None:
            center=((The_box[0]+The_box[2])/2)
            Center_X=int(center[0])
            Center_Y=int(center[1])
            cv2.circle(Image_inn,(Center_X,Center_Y),10,(0,0,255),-1)
            return Center_X,Center_Y
        else: return 960,600





    @staticmethod
    def read_AruCo(image_in,Ids_list):
        "Takes the image and reads the aruco code and adds the Id to the given list"
        gray= cv2.cvtColor(image_in,cv2.COLOR_BGR2GRAY)
        dict= aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        corners, ids, rejected = aruco.detectMarkers(gray, dict)

        if ids is not None and len(ids) > 0:
            Ids_list.append(ids[0][0])
        return Ids_list
    
    @staticmethod
    def filtered_Ids_list(Ids_list):
        "Filter the list and gives back a list without duplicates"
        filtered_list=[]
        for i in range(len(list)):
            if filtered_list.count(list[i]) < 1 and list.count(list[i])>10:
                    filtered_list.append(list[i])
        return filtered_list
    



    @staticmethod
    def find_bench(image, lower:list, upper:list):
        hsv_image = ImageMethods.color_filter(image,lower,upper) 
        ImageMethods.showImage(ImageMethods.stack_images)
        return hsv_image
    


    
