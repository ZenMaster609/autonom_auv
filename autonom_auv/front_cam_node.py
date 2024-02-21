import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time 
from .image_handler import ImageHandler, logging_data
from .image_methods import ImageMethods
from .dynamic_display import DynamicDisplay
from .pid_controller import PidController
import signal
from example_interfaces.srv import AddTwoInts


class FrontCamNode(Node):
    def __init__(self):
        super().__init__('front_cam_node')
        self.create_subscription(Image,'/camera2/image_raw',  self.cam2_callback,10)
        self.create_subscription(Bool,'/done_moving',  self.serv_checker, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_serv = self.create_publisher(Twist, '/move_serv', 10)
        self.desired_distance = 30
        self.mode = 2
        self.bridge = CvBridge()
        self.logger = logging_data()
        self.handler = ImageHandler()
        self.yaw_controller = PidController()
        self.x_controller = PidController()
        self.y_controller = PidController()
        self.z_controller = PidController()
        self.serv_bool = False
        
                                
    def cam2_callback(self, data):
        cam_feed = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.handler.feed_image = cam_feed
        size, middle_left, middle_right = self.handler.find_bench()
        distance_offset = size - self.desired_distance
        if self.mode == 3:
            return
        if self.mode == 0: #Reis til venstre hjørne
            y_offset = 960 - middle_left[0]
            if abs(distance_offset) < 5 and abs(y_offset) < 150: #hvis innen 3 cm distanse og innen 50 piksler fra venstre linje
                self.mode = 1 #bytt til modus 1
        elif self.mode == 1:
            y_offset = 960 - middle_right[0] - 300
            if abs(distance_offset) < 5 and abs(y_offset) < 150:
                self.mode = 2
        elif self.mode == 2:
            response = self.call_move(x = -7.0, y = 2.0, yaw = 180.0)
            self.get_logger().info(response.sum)
            self.mode = 3
            return
        x_vel = self.x_controller.PID_controller(distance_offset,20,0.0,0.0,5000, 10)
        y_vel = self.y_controller.PID_controller(y_offset,10,0.0,0.0,5000, 10)
        #self.get_logger().info(f"distance_offset = {distance_offset}, y_offset = {y_offset}, y_vel = {y_vel}")
        self.send_movement(x_vel, y_vel)
 
    

    def send_movement(self, lin_x = 0.0, lin_y = 0.0):
        move_cmd = Twist()
        move_cmd.linear.x = lin_x
        move_cmd.linear.y = lin_y
        self.publisher_.publish(move_cmd)


    def call_move(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.x = roll
        msg.angular.y = pitch
        msg.angular.z = yaw
        self.publisher_serv.publish(msg)

    def serv_checker(self, msg):
        self.serv_bool = msg.data
        self.get_logger().info(self.serv_bool)

def main(args=None):
    rclpy.init(args=args)
    node = FrontCamNode()
    #signal.signal(signal.SIGINT, lambda sig, frame: node.custom_cleanup())
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
