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

class VisualInspectionNode(Node):
    def __init__(self):
        super().__init__('visual_inspection_node')
        self.create_subscription(Image,'/camera2/image_raw',  self.cam2_callback,10)
        self.create_subscription(Image,'/camera/image_raw',  self.cam1_callback,10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher2 = self.create_publisher(Float32, '/up_down', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.desired_distance = 20
        self.mode = 10
        self.bridge = CvBridge()
        self.logger = logging_data()
        self.handler = ImageHandler()
        self.x_controller = PidController()
        self.y_controller = PidController()
        self.blind_pid = [PidController() for _ in range(6)]
        self.pos = [None,None,None,None,None,None]
        self.target_pos = [None,None,None,None,None,None]
        self.move_bool = False
        self.front = True
        self.speed_scale = 2

    def send_movement(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, axis = 6, magnitude = 0.0):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.x = roll
        msg.angular.y = pitch
        msg.angular.z = yaw
        if axis == 0:msg.linear.x = magnitude
        elif axis == 1:msg.linear.y = magnitude
        elif axis == 2:msg.linear.z = magnitude
        elif axis == 3:msg.angular.x = magnitude
        elif axis == 4:msg.angular.y = magnitude
        elif axis == 5:msg.angular.z = magnitude
        self.publisher_.publish(msg)

    def publish_z(self, value):
        msg = Float32()
        msg.data = value
        self.publisher2.publish(msg)

    def odom_callback(self, msg):
        self.pos[0] = msg.pose.pose.position.x
        self.pos[1] = msg.pose.pose.position.y
        self.pos[2] = msg.pose.pose.position.z
        quaternion = msg.pose.pose.orientation
        self.pos[3], self.pos[4], self.pos[5] = Quaters.quaternion_to_euler(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        if self.pos[5] < 0:self.pos[5] = 2*math.pi - abs(self.pos[5])  

    def cam1_callback(self,data):
        self.handler.feed_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def cam2_callback(self, data):
        try:
            self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.handler.feed_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            size, positions = self.handler.find_bench(self.front)
            distance_offset = size - self.desired_distance
        except Exception as e:
             self.handler.show_image(True)
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
                self.mode =10
            elif self.mode ==10:
                self.get_logger().info(f"Gå hjem!")
                self.send_movement(0.0,0.0,0.0,0.0,0.0,0.0)

                

                
            
        
 
    def move_pos(self, axis, distance):
        if axis > 2:distance = math.radians(distance)
        self.target_pos[axis] = round((self.remap_axes(axis) + distance), 4)
        self.move_bool = True
        self.get_logger().info(f"new target = {self.target_pos[axis]}, pos = {self.pos[axis]}, distance = {distance}")


    def timer_callback(self):
        self.check_goal_pose(0) #sjekk x
        self.check_goal_pose(1) #sjekk y
        self.check_goal_pose(5) #sjekk yaw

    def remap_axes(self, axis):
        if axis ==0:
            if  3.2 > self.pos[5] > 3.11:pos_fixed = -self.pos[axis] 
            elif 1.6 > self.pos[5] > 1.5:pos_fixed = -self.pos[1]
            elif 4.65 > self.pos[5] > 4.75:pos_fixed = self.pos[1]
            else:pos_fixed = self.pos[axis]
        elif axis ==1:
            if 3.2 > self.pos[5] > 3.1:pos_fixed = -self.pos[axis]
            elif 1.6 > self.pos[5] > 1.5:pos_fixed = -self.pos[0]
            elif 4.75 > self.pos[5] > 4.65:pos_fixed = self.pos[0]
            else:pos_fixed = self.pos[axis]
        else:pos_fixed = self.pos[axis]
        return pos_fixed

    def check_goal_pose(self, axis):
        if self.target_pos[axis] is None:return
        pos_fixed = self.remap_axes(axis)
        
        if abs(self.target_pos[axis] - pos_fixed) < 0.005: # Check if we are close to the target
            self.send_movement(axis=axis, magnitude =0.0)
            self.target_pos[axis] = None
            self.move_bool = False
        else:
            offset = round(self.target_pos[axis] - pos_fixed, 4)
            vel = round(self.blind_pid[axis].PID_controller(offset, 80, 0.0, 0.0, 100, 0), 4)
            self.send_movement(axis=axis, magnitude = vel)
            self.get_logger().info(f"axis = {axis} goal = {self.target_pos[axis]}, offset = {offset}, odom = {round(pos_fixed, 4)}, vel = {vel}, rot = {self.pos[5]}") 
            #self.get_logger().info(f"odoms: {self.pos}") 











def main(args=None):
    rclpy.init(args=args)
    node = VisualInspectionNode()
    #signal.signal(signal.SIGINT, lambda sig, frame: node.custom_cleanup())
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
