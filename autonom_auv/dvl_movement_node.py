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
from .trackbar_hsv import DynamicDisplay
from .controller import PidController
import signal
from example_interfaces.srv import AddTwoInts
import math

class DvlMovementNode(Node):
    def __init__(self):
        super().__init__('dvl_movement_node')
        #Topic pub/sub + timer declaration
        self.create_subscription(Odometry, '/odom', self.odom_callback, 40)
        self.create_subscription(Twist, '/target', self.move_pos_callback, 40)
        self.publisher1 = self.create_publisher(Twist, '/tf_movement', 40)
        self.publisher2 = self.create_publisher(Bool, '/move_bool', 40)
        self.timer = self.create_timer(0.2, self.timer_callback)
        #pid controller list declaration
        self.blind_pid = [PidController() for _ in range(6)]
        #Node variable declarations
        self.pos = [None,None,None,None,None,None]
        self.target_pos = [None,None,None,None,None,None]
        self.speed_scale = 1
        self.treshold = [0.05,0.05,0.05,0.001,0.001,0.01]
        self.zero_yaw = False
        self.homing = False
        self.top_speed = [0.3, 0.3, 0.3, 0.1, 0.1, 0.1]
        pid_gir = [0.14, 0.0, 0.0]
        pid_jag = [1,0.00, 0.0] 
        pid_svai = [1,0.0, 0.0]
        pid_gir = [1.5, 0.3, 0.1]
        pid_jag = [1,0.19, 0.19] 
        pid_svai = [6,0.19, 0.13]
        self.pid = [pid_jag,pid_svai,pid_svai,pid_gir,pid_gir,pid_gir]
        self.margin = [0.05,0.05,0.05, 0.001, 0.001, 0.001]
        self.u_I_max = [0.1, 0.1,0.1, 0.1, 0.1, 0.01]

    def send_movement(self,x = 0.0, y = 0.0, yaw =0.0, axis = 6, magnitude = 0.0):
        """Sends movements to the movement node"""
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = yaw

        if axis == 0:msg.linear.x = magnitude
        elif axis == 1:msg.linear.y = magnitude
        elif axis == 2:msg.linear.z = magnitude
        elif axis == 3:msg.angular.x = magnitude
        elif axis == 4:msg.angular.y = magnitude
        elif axis == 5:msg.angular.z = magnitude
        self.publisher1.publish(msg)

    def odom_callback(self, msg):
        """Fetches odometry of the ROV"""
        self.pos[0] = msg.pose.pose.position.x
        self.pos[1] = msg.pose.pose.position.y
        self.pos[2] = msg.pose.pose.position.z
        quaternion = msg.pose.pose.orientation
        self.pos[3], self.pos[4], self.pos[5] = ImageMethods.quaternion_to_euler(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        if self.pos[5] < 0:self.pos[5] = 2*math.pi - abs(self.pos[5])  

    def send_false(self):
        """Tells the m_bench_node that movement command has been executed"""
        msg = Bool()
        msg.data = False
        self.publisher2.publish(msg)

    def home1(self):
        """Sets yaw x and y targets to 0 to return home"""
        self.target_pos[5] = 0.0
        self.target_pos[0] = 0.0
        self.target_pos[1] = 0.0
        self.get_logger().info(f"HOMING")
    
    def reset_pi(self):
        """Corrects for simulator trigonometry to allow ROV to fully rotate further than 360 degrees"""
        if self.target_pos[5] is not None:
            if self.target_pos[5] > 2*math.pi and self.pos[5] < 1:
                self.get_logger().info("RESET PI")
                self.target_pos[5] = self.target_pos[5] - 2*math.pi
                
    def timer_callback(self):   
        """Calls appropriate functions on a timer to regulate the ROV postition towards its targets"""
        self.reset_pi()
        x = self.check_goal_pose(0) #sjekk x
        y = self.check_goal_pose(1) #sjekk y
        yaw = self.check_goal_pose(5) #sjekk yaw
        self.send_movement(x=x, y=y, yaw=yaw)
 
    def xytrig(self):
        """Remaps x and y coordinates using trigonometry"""
        yaw, x, y = self.pos[5], self.pos[0], self.pos[1]
        local_x = x * math.cos(yaw) + y * math.sin(yaw)
        local_y = -x * math.sin(yaw) + y * math.cos(yaw)
        return local_x, local_y

    def move_pos_callback(self, msg):
        """Sets targets when called"""
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
        """Checks targets vs position when called, call continously to regulate so that target = position."""
        if self.target_pos[axis] is None:return 0.0
        if axis == 0:pos_fixed, _ = self.xytrig()
        elif axis == 1:_, pos_fixed = self.xytrig() 
        else:pos_fixed = self.pos[axis]
        
        if abs(self.target_pos[axis] - pos_fixed) < self.treshold[axis]: # Check if we are close to the target
            self.send_movement(axis=axis, magnitude =0.0)
            self.target_pos[axis] = None
            self.get_logger().info(f"reached goal in axis {axis}")
            self.send_false()
            return 0.0
        else:
            offset = round(self.target_pos[axis] - pos_fixed, 4)
            vel = self.blind_pid[axis].PID_controller
                (offset,*self.pid[axis],self.u_I_max[axis]
                 ,margin=self.margin[axis])
            if abs(vel) > self.top_speed[axis]:vel = self.top_speed[axis]*np.sign(vel)
            self.get_logger().info(f"axis = {axis} goal = {self.target_pos[axis]}, offset = {offset}, odom = {round(pos_fixed, 4)}, vel = {round(vel,4)}, rot = {self.pos[5]}") 
            return vel




def main(args=None):
    rclpy.init(args=args)
    node = DvlMovementNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    




