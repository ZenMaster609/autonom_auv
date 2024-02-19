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
from .image_handler import ImageHandler, logging_data
from .image_methods import ImageMethods
from .dynamic_display import DynamicDisplay
from .pid_controller import PidController
import signal


class FrontCamNode(Node):
    def __init__(self):
        super().__init__('valve_image_node')
        self.create_subscription(Image,'/camera2/image_raw',  self.cam2_callback,10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.desired_distance = 30
        self.mode = 1
        self.bridge = CvBridge()
        self.logger = logging_data()
        self.handler = ImageHandler()
        self.yaw_controller = PidController()
        self.x_controller = PidController()
        self.y_controller = PidController()
        self.z_controller = PidController()
        
                                
    def cam2_callback(self, data):
        cam_feed = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.handler.feed_image = cam_feed
        yaw_offset, size, middle_left, middle_right = self.handler.find_bench_info()
        distance_offset = size - self.desired_distance
        if self.mode == 0: #Reis til venstre hjørne
            y_offset = 960 - middle_left[0]
            if distance_offset < 5 and y_offset < 100: #hvis innen 3 cm distanse og innen 50 piksler fra venstre linje
                self.mode = 1 #bytt til modus 1
        elif self.mode == 1:
            y_offset = 960 - middle_right[0]
        angle_vel= self.yaw_controller.PID_controller(yaw_offset,10,0.0,0.0,100, 1)
        x_vel = self.x_controller.PID_controller(distance_offset,10,0.0,0.0,5000, 10)
        y_vel = self.y_controller.PID_controller(y_offset,5,0.0,0.0,5000, 10)
        self.logger.log_data(angle_vel,x_vel,y_offset, y_offset)
        # self.get_logger().info(f"angle_vel = {angle_vel}, x_vel = {x_vel}, y_vel = {y_vel} ")
        self.get_logger().info(f"distance_offset = {distance_offset}, y_offset = {y_offset}, y_vel = {y_vel}")
        #self.send_movement(angle_vel, x_vel, y_vel)



    #def turn_angle(self, angle):


    def send_movement(self,ang_z = 0.0, lin_x = 0.0, lin_y = 0.0):
        move_cmd = Twist()
        move_cmd.linear.x = lin_x
        move_cmd.linear.y = lin_y
        move_cmd.angular.z = ang_z
        self.publisher_.publish(move_cmd)

    def custom_cleanup(self):
        self.logger.plot_data()

def main(args=None):
    rclpy.init(args=args)
    node = FrontCamNode()
    signal.signal(signal.SIGINT, lambda sig, frame: node.custom_cleanup())
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
