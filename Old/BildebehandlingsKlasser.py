import cv2
import numpy as np
import math

class Find_HSV_Range:
    def __init__(self,HSV_image):
        self.HSV_image=HSV_image
    def HSV_Range(self):
        Hlist=[]; Slist=[];Vlist=[]
        HSV=cv2.cvtColor(self.HSV_image, cv2.COLOR_BGR2HSV)
        dimensions = HSV.shape
        for y in range(dimensions[1]):
            for x in range(dimensions[0]):
                Hlist.append(HSV[x, y, 0])
                Slist.append(HSV[x, y, 1]) 
                Vlist.append(HSV[x, y, 2])
            
        H_min=min(Hlist);S_min=min(Slist);V_min=min(Vlist)
        H_max=max(Hlist);S_max=max(Slist);V_max=max(Vlist)
        HSV_lower=np.array([H_min,S_min,V_min])
        HSV_upper=np.array([H_max,S_max,V_max])
        return np.array(HSV_lower),np.array(HSV_upper)

class image_prosessing:

    def __init__(self,image_in,HSV_lower=[77,43,165],HSV_upper=[99,116,207]):
        self.image_in=image_in
        self.HSV_lower=np.array(HSV_lower)
        self.HSV_upper=np.array(HSV_upper)

        self.box_list=[]
        self.boxl_y=[]
        self.HSV=0
        self.Myimage=0
        
    
    def create_images(self):
        self.contour_image = self.image_in.copy()
        self.dim=self.image_in.shape
        self.Myimage= np.zeros((self.dim[0],self.dim[1],3),np.uint8)
         

    #Looks for the color range and makes a black and white picture 
    def color_filter(self):  
        self.HSV = cv2.cvtColor(self.image_in, cv2.COLOR_BGR2HSV)
        self.mask=cv2.inRange(self.HSV,self.HSV_lower,self.HSV_upper)
        self.maskM=cv2.medianBlur(self.mask, 5) 
        print("HEI")
        cv2.circle(self.maskM,(500,500),200,(0,0,0),-1)
        #cv2.line(self.mask,(0,self.dim[0]),(self.dim[1],0),(0,0,0),10)


    #Makes a box around the contours with a lenght greater than 400
    def make_boxes(self):
        contours, _ = cv2.findContours(self.maskM, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in range(len(contours)):
            if len(contours[cnt])>400:
                rect = cv2.minAreaRect(contours[cnt])
                box = np.intp(cv2.boxPoints(rect))
                self.box_list.append(box)
                self.boxl_y.append(box[:,1])
                cv2.drawContours(self.Myimage,[box],0,(255,255,255),-1)


    def find_closest_box(self):
        max_y_value=0
        for i in range(len(self.box_list)):
            if max(self.boxl_y[i])>max_y_value: 
                max_y_value=max(self.boxl_y[i])
                box_index=i 
        self.box=self.box_list[box_index]

    def find_contour_box(self):
        gray = cv2.cvtColor(self.Myimage, cv2.COLOR_BGR2GRAY)
        self.contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(self.contour_image,self.contours,-1,(0,0,255),3)

     #Find a point in the middle of the object   
    def find_middle_of_box(self):    
        center=((self.box[0]+self.box[2])/2)
        cX=int(center[0])
        cY=int(center[1])
        cv2.circle(self.contour_image,(cX,cY),10,(0,0,0),-1)
        return cX
    #Finding the angle of the object between the horizonal frame and the longest vector
    def find_angle(self):
        Vec_1=self.box[0]-self.box[1]
        Vec_2=self.box[1]-self.box[2]
        lenght_Vec_1=math.sqrt(Vec_1[0]**2+Vec_1[1]**2)
        lenght_Vec_2=math.sqrt(Vec_2[0]**2+Vec_2[1]**2)
        if lenght_Vec_1>lenght_Vec_2:
            angle=math.acos(Vec_1[0]/lenght_Vec_1)
        else:angle=math.acos(Vec_2[0]/lenght_Vec_2)
        angelDeg=angle*360/2/math.pi
        cv2.putText(self.contour_image, (str(int(angelDeg))),[50,70], cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 0), 2, cv2.LINE_AA)
        return angle

    def make_image_out(self):
        combined=np.hstack((self.image_in, self.HSV))
        combined2=np.hstack((cv2.cvtColor(self.mask, cv2.COLOR_BAYER_BG2BGR), cv2.cvtColor(self.maskM, cv2.COLOR_BAYER_BG2BGR)))
        combined3=np.hstack((self.Myimage,self.contour_image))
        combined=np.vstack((combined,combined2,combined3))
        combined = cv2.resize(combined, (0, 0), fx = 0.3, fy = 0.3)
        return combined

class Reg_parameters:
    def __init__(self,cX,angle,dimCamera_x):
        self.cX=cX
        self.angle=angle
        self.dimCamera_x=dimCamera_x
    def find_angle_vel(self):
        offsett_x=self.dimCamera_x/2-self.cX
        angle_vel=offsett_x/(self.dimCamera_x/2)
        return angle_vel
    

#Finding HVS range for the given object 
# image_HSV = cv2.imread('PipelineinspeksjonHVS2.png')
# range_HSV=Find_HSV_Range(image_HSV)
# lower,upper=range_HSV.HSV_Range()


# image=cv2.imread('Pipelineinspeksjon1.png')
# #print(image.shape)
# img_p=image_prosessing(image)
# img_p.create_images()
# img_p.color_filter()
# img_p.make_boxes()
# img_p.find_closest_box()
# img_p.find_contour_box()
# cX=img_p.find_middle_of_box()
# angle=img_p.find_angle()
# image_out=img_p.make_image_out()

# print(cX)
# REG1= Reg_parameters(cX,angle,1856)
# angle_vel=REG1.find_angle_vel()
# print(angle_vel)

# cv2.imshow('Contours FUCK', image_out)
# cv2.waitKey(0)
# cv2.destroyAllWindows()













