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

class FakeDvlNode(Node):
    def __init__(self):
        super().__init__('fake_dvl_node')
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/target', self.move_pos_callback, 10)
        self.publisher1 = self.create_publisher(Twist, '/tf_movement', 10)
        self.publisher2 = self.create_publisher(Bool, '/move_bool', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.blind_pid = [PidController() for _ in range(6)]
        self.pos = [None,None,None,None,None,None]
        self.target_pos = [None,None,None,None,None,None]
        self.speed_scale = 1
        self.zero_yaw = False
        self.top_speed = 3.0
        self.pid = [80, 0.05, 0]

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
        self.publisher1.publish(msg)

    def odom_callback(self, msg):
        self.pos[0] = msg.pose.pose.position.x
        self.pos[1] = msg.pose.pose.position.y
        self.pos[2] = msg.pose.pose.position.z
        quaternion = msg.pose.pose.orientation
        self.pos[3], self.pos[4], self.pos[5] = Quaters.quaternion_to_euler(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        if self.pos[5] < 0:self.pos[5] = 2*math.pi - abs(self.pos[5])  

    def send_false(self):
        msg = Bool()
        msg.data = False
        self.publisher2.publish(msg)

    def home1(self):
        self.target_pos[5] = 0
        self.zero_yaw = True
        self.get_logger().info(f"HOMING")
        
    
    def home2(self):
        if self.zero_yaw and abs(self.pos[5]) < 0.005:
            self.target_pos[0] = 0
            self.target_pos[1] = 0
            self.zero_yaw = False
    
    def reset_pi(self):
        if self.target_pos[5] is not None:
            if self.target_pos[5] > 2*math.pi and self.pos[5] < 1:
                self.get_logger().info("RESET PI")
                self.target_pos[5] = self.target_pos[5] - 2*math.pi
                

    def timer_callback(self):
        self.check_goal_pose(0) #sjekk x
        self.check_goal_pose(1) #sjekk y
        self.reset_pi()
        self.check_goal_pose(5) #sjekk yaw
        self.home2()

    def xytrig(self):
        yaw, x, y = self.pos[5], self.pos[0], self.pos[1]
        local_x = x * math.cos(yaw) + y * math.sin(yaw)
        local_y = -x * math.sin(yaw) + y * math.cos(yaw)
        return local_x, local_y

    

    def move_pos_callback(self, msg):
        axis = int(msg.linear.x); distance = msg.linear.y
        if distance == 0.0:
            self.home1()
            return
        pos_fixed = self.pos[axis] 
        if axis > 2:
            distance = math.radians(distance)
        elif axis == 0: pos_fixed, _ = self.xytrig()
        elif axis == 1: _, pos_fixed = self.xytrig()          
        self.target_pos[axis] = pos_fixed + distance
        self.get_logger().info(f"new target = {self.target_pos[axis]}, pos = {pos_fixed}, distance = {distance}")

    def check_goal_pose(self, axis):
        if self.target_pos[axis] is None:return
        if axis == 0:pos_fixed, _ = self.xytrig()
        elif axis == 1:_, pos_fixed = self.xytrig() 
        else:pos_fixed = self.pos[axis]
        
        if abs(self.target_pos[axis] - pos_fixed) < 0.005: # Check if we are close to the target
            self.send_movement(axis=axis, magnitude =0.0)
            self.target_pos[axis] = None
            self.get_logger().info(f"reached goal in axis {axis}")
            self.send_false()
        else:
            offset = round(self.target_pos[axis] - pos_fixed, 4)
            vel = self.blind_pid[axis].PID_controller(offset, self.pid[0], self.pid[1], self.pid[2], 100, 0.0)
            if vel > self.top_speed:vel = self.top_speed
            self.send_movement(axis=axis, magnitude = vel)
            self.get_logger().info(f"axis = {axis} goal = {self.target_pos[axis]}, offset = {offset}, odom = {round(pos_fixed, 4)}, vel = {round(vel,4)}, rot = {self.pos[5]}") 








def main(args=None):
    rclpy.init(args=args)
    node = FakeDvlNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    



