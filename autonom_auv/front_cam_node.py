import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
import time 
from .image_handler import ImageHandler, logging_data
from .image_methods import ImageMethods
from .dynamic_display import DynamicDisplay
from .pid_controller import PidController, Quaters
import signal
from example_interfaces.srv import AddTwoInts
import math

class FrontCamNode(Node):
    def __init__(self):
        super().__init__('front_cam_node')
        self.create_subscription(Image,'/camera2/image_raw',  self.cam2_callback,10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.desired_distance = 30
        self.mode = 2
        self.bridge = CvBridge()
        self.logger = logging_data()
        self.handler = ImageHandler()
        self.pitch_controller = PidController()
        self.roll_controller = PidController()
        self.yaw_controller = PidController()
        self.x_controller = PidController()
        self.y_controller = PidController()
        self.z_controller = PidController()
        self.bx_controller = PidController()
        self.by_controller = PidController()
        self.pos = [None,None,None]
        self.ori = [None,None,None]
        self.target_ori = [None,None,None]
        self.target_pos = [None,None,None]
        self.move_bool = False

    def send_movement(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.x = roll
        msg.angular.y = pitch
        msg.angular.z = yaw
        self.publisher_.publish(msg)

    def odom_callback(self, msg):
        self.pos[0] = msg.pose.pose.position.x
        self.pos[1] = msg.pose.pose.position.y
        self.pos[2] = msg.pose.pose.position.z
        quaternion = msg.pose.pose.orientation
        self.ori[0], self.ori[1], self.ori[2] = Quaters.quaternion_to_euler(quaternion.x, quaternion.y, quaternion.z, quaternion.w)

    def cam2_callback(self, data):
        cam_feed = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.handler.feed_image = cam_feed
        size, middle_left, middle_right = self.handler.find_bench()
        distance_offset = size - self.desired_distance

        if self.mode == 0: #Reis til venstre hjørne
            y_offset = 960 - middle_left[0]
            if abs(distance_offset) < 5 and abs(y_offset) < 150: #hvis innen 3 cm distanse og innen 50 piksler fra venstre linje
                self.mode = 1 #bytt til modus 1
        elif self.mode == 1:
            y_offset = 960 - middle_right[0] 
            if abs(distance_offset) < 5 and abs(y_offset) < 150:
                self.mode = 2
        elif self.mode == 2:
            self.move_bool = True
            self.move_pos(1,2)
            self.mode = 3
        elif self.mode == 3 and self.move_bool == False:
            self.move_pos(0,4)
        if self.mode < 2:
            x_vel = self.x_controller.PID_controller(distance_offset,20,0.0,0.0,5000, 10)
            y_vel = self.y_controller.PID_controller(y_offset,10,0.0,0.0,5000, 10)
            #self.get_logger().info(f"distance_offset = {distance_offset}, y_offset = {y_offset}, y_vel = {y_vel}")
            self.send_movement(x_vel, y_vel)
 
    def move_pos(self, axis, distance):
        self.target_pos[axis] = self.pos[axis] + distance

    def turn_angle(self, axis, angle):
        rad_angle = math.radians(angle)
        self.target_ori[axis] = self.ori[axis] + rad_angle

    def timer_callback(self):
        self.check_goal_pose(0) #sjekk x
        self.check_goal_pose(1) #sjekk y
        

    def check_goal_pose(self, axis):
        if self.target_pos[axis] is None:return
        if axis == 0 and abs(self.ori[2]) > 3.1:pos_fixed = -self.pos[axis]
        else:pos_fixed = self.pos[axis]
        
        if abs(self.target_pos[axis] - pos_fixed) < 0.1: # Check if we are close to the target
            if axis == 0:self.send_movement(x=0.0)
            elif axis ==1:self.send_movement(y=0.0)  # Stop turning
            else:self.send_movement(z=0.0)
            self.target_pos[axis] = None
            self.move_bool = False
        else:
            offset = round(self.target_pos[axis] - pos_fixed, 4)
            if axis ==0:
                vel = round(self.bx_controller.PID_controller(offset, 80, 0.0, 0.0, 100, 0), 4)
                self.send_movement(x=vel)
            else:
                vel = round(self.by_controller.PID_controller(offset, 80, 0.0, 0.0, 100, 0), 4)
                self.send_movement(y=vel)
            self.get_logger().info(f"goal = {self.target_pos[axis]}, offset = {offset}, odom = {round(pos_fixed),4}, vel = {vel}") 











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
    
