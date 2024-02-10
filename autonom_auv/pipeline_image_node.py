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
from .pid_controller_node import PidControllerNode
from geometry_msgs.msg import Twist


class PipelineImageNode(Node):
    def __init__(self):
        super().__init__('image_processor') 
        self.subscription = self.create_subscription(Image,'/camera/image_raw',  self.listener_callback,10)
        self.subscription  # Prevent unused variable warning
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)


    def send_movement(self,ang_vel):
        move_cmd = Twist()
        move_cmd.linear.x = 0.4
        move_cmd.angular.z =ang_vel
        self.publisher_.publish(move_cmd)
       

    def listener_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image_edit =cv_image.copy()
        dimensions = cv_image.shape
       
        maskM = PipelineImageMethods.color_filter(cv_image,30,114,114,30,255,238)
        
        maskM = cv2.line(maskM,(0,600),(dimensions[1],600),(0,0,0),10)
        image_edit,Box_Image,Box_list = PipelineImageMethods.make_boxes(maskM,image_edit)
       
        The_box = PipelineImageMethods.find_the_box(Box_list)
        
        image_with_dot,Center_X,Center_Y = PipelineImageMethods.Draw_Center(image_edit,The_box)
       
        maskM=cv2.cvtColor(maskM,cv2.COLOR_BAYER_BG2BGR)
        image_show=PipelineImageMethods.stack_images(cv_image,maskM,Box_Image,image_with_dot)
        Offsett_x = PidControllerNode.calculate_parameters(Center_X,dimensions)
        angle_vel=(Offsett_x/(1920/2))
        self.send_movement(angle_vel)

        cv2.imshow("window",image_show)
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    image_processor = PipelineImageNode()
    rclpy.spin(image_processor)
    cv2.destroyAllWindows()
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
