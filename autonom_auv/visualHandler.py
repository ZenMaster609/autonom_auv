import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock 
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory
import cv2.aruco as aruco
from std_msgs.msg import Float32
import numpy as np
from .ImageProssesingClass import image_prosessing
from .ControllerClass import Controllers
from geometry_msgs.msg import Twist
import time

class VisualProcessor(Node):
    def __init__(self):
        super().__init__('visual_processor') 
        self.cam1_subscription = self.create_subscription(Image,'/camera/image_raw',  self.cam1_callback,10)
        self.cam1_subscription  # Prevent unused variable warning
        self.cam2_subscription = self.create_subscription(Image,'/camera2/image_raw',  self.cam2_callback,10)
        self.cam2_subscription  # Prevent unused variable warning
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_image1 = None
        self.pub_image1 = None
        self.imgcount = 0

    def send_movement(self,ang_vel):
        move_cmd = Twist()
        move_cmd.linear.x = 0.4
        move_cmd.angular.z =ang_vel
        self.publisher_.publish(move_cmd)
       
    def cam1_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image_edit =cv_image.copy()
        dimensions = cv_image.shape
        self.pub_image1 = image_edit

    def cam2_callback(self, data):
        self.imgcount += 1
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image_edit =cv_image.copy()
        dimensions = cv_image.shape
        self.pub_image2 = image_edit
        if self.imgcount > 3:
            self.imgcount = 0
        filename = os.path.join("..", "photos", f"cam2_{time.time()}.jpg")
        cv2.imwrite(filename, cv_image)
        cv2.imshow("window", image_edit)
        cv2.waitKey(1)



        
def main(args=None):
    rclpy.init(args=args)
    visual_processor = VisualProcessor()
    rclpy.spin(visual_processor)
    cv2.destroyAllWindows()
    visual_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # photos_path = os.path.join("..", "photos")
    # if not os.path.exists(photos_path):
    #     os.makedirs(photos_path)
    main()
    
