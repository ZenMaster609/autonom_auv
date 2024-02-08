import math
import cv2 
import cv2.aruco as aruco
import numpy as np


class image_prosessing: 

    #Looks for the color range and makes a black and white picture 
    @staticmethod 
    def color_filter(Image_inn,H_min,S_min,V_min,H_max,S_max,V_max):
        "Takes in an image and HSV range, return a black and white image"  
        image_HSV = cv2.cvtColor(Image_inn, cv2.COLOR_BGR2HSV)
        HSV_lower = np.array([H_min,S_min,V_min])
        HSV_upper = np.array([H_max,S_max,V_max])
        mask = cv2.inRange(image_HSV,HSV_lower,HSV_upper)
        maskM = cv2.medianBlur(mask, 5)
        return maskM 

    @staticmethod
    def make_boxes(Black_White_Image, Original_image):
        "Takes in black and white image, return orginal image with drawn boxes, image with box approximation and a list of boxes "
        dimensions = Black_White_Image.shape
        Box_Image=np.zeros((dimensions[0],dimensions[1],3),np.uint8)
        Box_list = []
        contours, _ = cv2.findContours(Black_White_Image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in range(len(contours)):
            if len(contours[cnt])>1:
                rect = cv2.minAreaRect(contours[cnt])
                Box = np.intp(cv2.boxPoints(rect))
                Box_list.append(Box)
                cv2.drawContours(Box_Image,[Box],0,(255,255,255),-1)
                cv2.drawContours(Original_image, [Box], -1, (0, 0, 255),thickness=2) 
        return Original_image,Box_Image,Box_list 
    
    #Finds the box furthes away from the camera 
    @staticmethod
    def find_the_box(Box_list):
        "Takes in a list of boxes return the boxes highest in the picture"
        min_y_value=1920
        
        for i in range(len(Box_list)):
            y_value_list = Box_list[i][:,1]
            if min(y_value_list)<min_y_value: 
                min_y_value = min(y_value_list)
                box_index = i 
        box=Box_list[box_index]
        return box 
        

#Find a point in the middle of the object
    @staticmethod
    def Draw_Center(Image_inn,The_box):    
        "Draw a dot in the center of the bo, return Center cordinates"
        center=((The_box[0]+The_box[2])/2)
        Center_X=int(center[0])
        Center_Y=int(center[1])
        cv2.circle(Image_inn,(Center_X,Center_Y),10,(0,0,255),-1)
        return Image_inn,Center_X,Center_Y

    @staticmethod
    def makes2x2_image(image1,image2,image3,image4):
        image_show = np.hstack((image1,image2,))
        image_show2 = np.hstack((image3,image4))
        image_show = np.vstack((image_show,image_show2))
        image_show = cv2.resize(image_show, (0, 0), fx = 0.4, fy = 0.4)
        return image_show


    @staticmethod
    def read_AruCo(image_in,Ids_list):
        gray= cv2.cvtColor(image_in,cv2.COLOR_BGR2GRAY)
        dict= aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        corners, ids, rejected = aruco.detectMarkers(gray, dict)
        if ids is not None and len(ids) > 0:
            Ids_list.append(ids[0][0])
        return Ids_list

    def filtered_Ids_list(Ids_list):
        filtered_list=[]
        for i in range(len(list)):
            if filtered_list.count(list[i]) < 1 and list.count(list[i])>10:
                    filtered_list.append(list[i])
        return filtered_list



image = cv2.imread("HjorneSim.png")
image_edit =image.copy()
dimensions = image.shape

maskM = image_prosessing.color_filter(image,30,114,114,30,255,238)
maskM = cv2.line(maskM,(0,200),(dimensions[1],200),(0,0,0),10)
image_edit,Box_Image,Box_list = image_prosessing.make_boxes(maskM,image_edit)
The_box =image_prosessing.find_the_box(Box_list)
image_with_dot,Center_X,Center_Y = image_prosessing.Draw_Center(image_edit,The_box)

maskM=cv2.cvtColor(maskM,cv2.COLOR_BAYER_BG2BGR)
image_show=image_prosessing.makes2x2_image(image,maskM,Box_Image,image_with_dot)



cv2.imshow("Bilde",image_show)
cv2.waitKey(0)
cv2.destroyAllWindows()