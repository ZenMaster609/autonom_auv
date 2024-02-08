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
from .ImageProssesingClass import image_prosessing


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor') 
        self.subscription = self.create_subscription(Image,'/camera/image_raw',  self.listener_callback,10)
        self.subscription  # Prevent unused variable warning
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Float32, '/angular_velocity', 10)
  

    def publish_float(self, value):
        msg = Float32()
        msg.data = value
        self.publisher_.publish(msg)
        #self.get_logger().info(f'Publishing: {msg.data}')

    def listener_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image_edit =cv_image.copy()
        dimensions = cv_image.shape
       
        maskM = image_prosessing.color_filter(cv_image,30,114,114,30,255,238)
        
        maskM = cv2.line(maskM,(0,600),(dimensions[1],600),(0,0,0),10)
        image_edit,Box_Image,Box_list = image_prosessing.make_boxes(maskM,image_edit)
       
        The_box =image_prosessing.find_the_box(Box_list)
        
        image_with_dot,Center_X,Center_Y = image_prosessing.Draw_Center(image_edit,The_box)
       
        maskM=cv2.cvtColor(maskM,cv2.COLOR_BAYER_BG2BGR)
        image_show=image_prosessing.makes2x2_image(cv_image,maskM,Box_Image,image_with_dot)
        self.find_angle_vel(Center_X)

        cv2.imshow("window",image_show)
        cv2.waitKey(1)
        
        


    def find_angle_vel(self,Center_X):
        offsett_x=1920/2-Center_X
        angle_vel=(offsett_x/(1920/2))
        self.publish_float(angle_vel)           


def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    cv2.destroyAllWindows()
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
