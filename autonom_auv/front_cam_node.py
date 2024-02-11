import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import time 
from .image_handler import ImageHandler
from .image_methods import ImageMethods
from .dynamic_display import DynamicDisplay

class FrontCamNode(Node):
    def __init__(self):
        super().__init__('valve_image_node')
        self.create_subscription(Image,'/camera2/image_raw',  self.cam2_callback,10)
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Twist, '/pid_info', 10)
        self.handler = ImageHandler(save_freq = 8)
        #DynamicDisplay.trackbar_init()
        

    def cam2_callback(self, data):
        cam_feed = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.handler.feed_image = cam_feed
        #DynamicDisplay.find_hsv(cam_feed)
        self.handler.find_bench()




        
def main(args=None):
    rclpy.init(args=args)
    node = FrontCamNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
