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
from .pipeline_image_methods import PipelineImageMethods
from geometry_msgs.msg import Twist
import time

class ValveImageNode(Node):
    def __init__(self):
        super().__init__('valve_image_node')
        self.create_subscription(Image,'/camera/image_raw',  self.cam1_callback,10)
        self.create_subscription(Image,'/camera2/image_raw',  self.cam2_callback,10)
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_image1 = None
        self.pub_image1 = None
        self.imgcount = 0
        

    def cam1_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image_edit =cv_image.copy()
        dimensions = cv_image.shape
        self.pub_image1 = image_edit

    def cam2_callback(self, data):
        photos_path = os.path.join(get_package_share_directory('autonom_auv'), 'photos', f"cam2_{time.time()}.jpg")
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image_edit =cv_image.copy()
        dimensions = cv_image.shape
        self.pub_image2 = image_edit

            # self.imgcount += 1
            # if self.imgcount > 20:
            #     self.imgcount = 0
            #     cv2.imwrite(photos_path, cv_image)
        cv2.imshow("window", image_edit)
        cv2.waitKey(1)



        
def main(args=None):
    rclpy.init(args=args)
    node = ValveImageNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
