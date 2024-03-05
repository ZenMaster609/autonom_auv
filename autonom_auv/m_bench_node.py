import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time 
from .image_handler import ImageHandler, logging_data
from .image_methods import ImageMethods
from .dynamic_display import DynamicDisplay
from .pid_controller import PidController, Quaters
import signal
from example_interfaces.srv import AddTwoInts
import math

class MBenchNode(Node):
    def __init__(self):
        super().__init__('mission_bench_node')
        self.create_subscription(Image,'/camera2/image_raw',  self.cam2_callback,10)
        self.create_subscription(Image,'/camera/image_raw',  self.cam1_callback,10)
        self.create_subscription(Bool, '/move_bool', self.bool_callback, 10)
        self.publisher1 = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher2 = self.create_publisher(Float32, '/up_down', 10)
        self.publisher3 = self.create_publisher(Twist, '/target', 10)
        self.desired_distance = 20
        self.mode = 0
        self.bridge = CvBridge()
        self.handler = ImageHandler()
        self.x_controller = PidController()
        self.y_controller = PidController()
        self.move_bool = False
        self.front = True


    def send_movement(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.x = roll
        msg.angular.y = pitch
        msg.angular.z = yaw
        self.publisher1.publish(msg)

    def publish_z(self, value):
        msg = Float32()
        msg.data = value
        self.publisher2.publish(msg)


    def bool_callback(self, msg):
        self.move_bool = msg.data

    def cam1_callback(self,data):
        self.handler.feed_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def cam2_callback(self, data):
        #try:
        self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.handler.feed_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        size, positions = self.handler.find_bench(self.front)
        distance_offset = size - self.desired_distance
        #except Exception as e:
             #self.handler.show_image(True)
        if self.move_bool == False:
            if self.mode == 0: #Reis til venstre hjørne
                y_offset = 960 - positions['middle_left'][0]
                if abs(distance_offset) < 5 and abs(y_offset) < 150: #hvis innen 3 cm distanse og innen 50 piksler fra venstre linje
                    self.mode = 1 #bytt til modus 1
            elif self.mode == 1:
                y_offset = 960 - positions['middle_right'][0]
                if abs(distance_offset) < 5 and abs(y_offset) < 150:
                    self.mode = 2
            if self.mode < 2:
                x_vel = self.x_controller.PID_controller(distance_offset,20,0.0,0.0,5000)
                y_vel = self.y_controller.PID_controller(y_offset,3,0.0,0.0,5000)
                self.get_logger().info(f"distance_offset = {distance_offset}, y_offset = {y_offset}, y_vel = {y_vel}")
                self.send_movement(x=x_vel, y=y_vel)
            elif self.mode == 2:
                self.move_pos(1,-1) # 3m y
                self.mode = 3
            elif self.mode == 3:
                self.move_pos(5,90) # 90 grader venstre 
                if self.front == True:self.mode = 4
                else:self.mode = 6
            elif self.mode == 4:
                self.move_pos(1,-3.7) # 3m y
                self.mode = 5
            elif self.mode == 5:
                self.move_pos(5,90) #90 grader venstre 
                self.mode = 0
                self.front = False #Now we are behind the bench
            elif self.mode == 6:
                self.move_pos(1, -1.5)
                self.mode = 7
            elif self.mode == 7:
                y_offset = 960 - positions['center'][0] - 30
                y_vel = self.y_controller.PID_controller(y_offset,3,0.0,0.0,5000)
                self.get_logger().info(f"y_offset = {y_offset}, y_vel = {y_vel}")
                self.send_movement(y=y_vel)
                if abs(y_offset) < 5:
                    self.publish_z(1.6)
                    self.mode = 8
            elif self.mode == 8:
                self.move_pos(5, 90.2)
                self.mode = 9
            elif self.mode ==9:
                self.move_pos(1,-3)
                self.handler.filter_arucos #filtrer koder
                self.mode =10
            elif self.mode ==10:
                self.get_logger().info(f"Aruco list: {self.handler.filtered_list}")
                self.send_movement(0.0,0.0,0.0,0.0,0.0,0.0)

    def move_pos(self, axis, distance):
        msg = Twist() 
        msg.linear.x = float(axis)
        msg.linear.y = float(distance)
        self.move_bool = True
        self.publisher3.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MBenchNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
