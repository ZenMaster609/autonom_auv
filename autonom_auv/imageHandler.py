import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory
import cv2.aruco as aruco
from std_msgs.msg import Float32
import numpy as np


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor') 
        self.subscription = self.create_subscription(Image,'/camera/image_raw',  self.listener_callback,10)
        self.subscription  # Prevent unused variable warning
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Float32, '/angular_velocity', 10)
        self.ids_list = []
  

    def publish_float(self, value):
        msg = Float32()
        msg.data = value
        self.publisher_.publish(msg)
        #self.get_logger().info(f'Publishing: {msg.data}')

    def listener_callback(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.contour_image=self.cv_image.copy()
        self.Myimage= np.zeros((1080,1920,3),np.uint8)
        self.read_AruCo()
        self.color_filter()
        self.make_boxes()
        if len(self.box_list)>0:
            self.find_closest_box()
            self.find_contour_box()
            self.find_middle_of_box()
            self.find_angle_vel()
            self.show_image()
         
    

   #Looks for the color range and makes a black and white picture 
    def color_filter(self):  
        self.HSV = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        HSV_lower = np.array([30,114,114])
        HSV_upper = np.array([30,255,238])
        self.mask = cv2.inRange(self.HSV,HSV_lower,HSV_upper)
        self.maskM = cv2.medianBlur(self.mask, 5)
        self.maskM = cv2.line(self.mask,(0,600),(1920,600),(0,0,0),10)

    
    #Makes a box around the contours with a lenght greater than 400
    def make_boxes(self):
        self.box_list = []
        self.boxl_y = []
        contours, _ = cv2.findContours(self.maskM, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in range(len(contours)):
            #if len(contours[cnt])>1:
            rect = cv2.minAreaRect(contours[cnt])
            box = np.intp(cv2.boxPoints(rect))
            self.box_list.append(box)
            self.boxl_y.append(box[:,1])
            cv2.drawContours(self.Myimage,[box],0,(255,255,255),-1)

#Finds the box neares the camera 
    def find_closest_box(self):
        min_y_value=1920
        for i in range(len(self.box_list)):
            if min(self.boxl_y[i])<min_y_value: 
                min_y_value = min(self.boxl_y[i])
                box_index = i 
        self.box=self.box_list[box_index]

#Makes the contour around the box
    def find_contour_box(self):
        gray = cv2.cvtColor(self.Myimage, cv2.COLOR_BGR2GRAY)
        self.contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(self.contour_image,self.contours,-1,(0,0,255),3)

#Find a point in the middle of the object
    def find_middle_of_box(self):    
        center=((self.box[0]+self.box[2])/2)
        self.cX=int(center[0])
        cY=int(center[1])
        cv2.circle(self.contour_image,(self.cX,cY),10,(0,0,0),-1)
        cv2.circle(self.Myimage,(self.cX,cY),10,(0,0,0),-1)


    def show_image(self):
        cv2.drawContours(self.contour_image,[self.box],0,(0,0,255),-1)
        image_show = np.hstack((self.cv_image,cv2.cvtColor(self.maskM, cv2.COLOR_BAYER_BG2BGR)))
        image_show2 = np.hstack((self.Myimage,self.contour_image))
        image_show = np.vstack((image_show,image_show2))
        image_show = cv2.resize(image_show, (0, 0), fx = 0.4, fy = 0.4)
        cv2.imshow("Camera Image", image_show)
        cv2.waitKey(1)


    def find_angle_vel(self):
        offsett_x=1920/2-self.cX
        angle_vel=(offsett_x/(1920/2))
        self.publish_float(angle_vel)

    
    def read_AruCo(self):
        gray= cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2GRAY)
        dict= aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        corners, ids, rejected = aruco.detectMarkers(gray, dict)
        if ids is not None and len(ids) > 0:
           self.ids_list.append(ids[0][0])
            

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    cv2.destroyAllWindows()
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
