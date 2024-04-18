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

class MBenchNode(Node):
    def __init__(self):
        super().__init__('mission_bench_node')
        #Topic pub/sub + timer
        self.create_subscription(Image,'/camera2/image_raw',  self.cam2_callback,10)
        self.create_subscription(Image,'/camera/image_raw',  self.cam1_callback,10)
        self.create_subscription(Bool, '/move_bool', self.bool_callback, 10)
        self.create_timer(0.05, self.timer_callback)
        self.publisher1 = self.create_publisher(Twist, '/tf_movement', 10)
        self.publisher2 = self.create_publisher(Float32, '/up_down', 10)
        self.publisher3 = self.create_publisher(Twist, '/target', 10)
        #object declarations
        self.bridge = CvBridge()
        self.handler = ImageHandler()
        self.x_controller = PidController()
        self.y_controller = PidController()
        self.yaw_controller = PidController()
        self.logger = logging_data()
        #variable declarations
        self.desired_distance = 30
        self.mode = 5
        self.move_bool = False
        self.front = True
        self.size = None
        self.positions = None
        self.angle = None
        self.angle_list = []
        self.dvl_zeroed = False
        self.found_bench = False

    def zero_dvl(self):
        """Pretend zeroing DVL"""
        self.dvl_zeroed = True
        self.get_logger().info("ZERO DVL")

    def send_movement(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        """Sends movements which are then processed by the movement node"""
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.x = roll
        msg.angular.y = pitch
        msg.angular.z = yaw
        self.publisher1.publish(msg)

    def publish_z(self, value):
        """Teleports on the z axis"""
        msg = Float32()
        msg.data = value
        self.publisher2.publish(msg)

    def bool_callback(self, msg):
        #Changes movebool if DVL movement node is finished moving
        self.move_bool = msg.data
        self.get_logger().info(f"BOOL CALLBACK: {msg.data}")

    #Fetch image feeds
    def cam1_callback(self,data):
        self.handler.feed_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def cam2_callback(self, data):
        self.handler.feed_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    
    def timer_callback(self): #Runs the main sequence on a timer
        self.run_image_strategy()

    def cam_info_get(self):
        """Retrieves positional data on the bench relative to the ROV"""
        try:
            self.size, self.positions, self.angle = self.handler.find_bench(self.front, self.mode)
            self.angle_list.append(self.angle)
            self.found_bench = True
        except Exception as e:
            self.found_bench = False
        # gir = yaw, svai = y, jag = x

    def camera_regulator(self, key:str, accuracy = 1.0):
        """Regulates postion relative to the bench using imageprocessing data. 
        Different key arguements lead to different methods of regulation and different offsets.
        Increases mode by 1"""
        if self.found_bench == True:
            if key == 'slide_in':
                y_offset = self.handler.dims[1]/2 - self.positions['center'][0] - 30
                y_vel = self.y_controller.PID_controller(y_offset,P=20, I=0.001, D=0.0, T_f=0.5, scale_devide=5000, margin=0.2)
                self.get_logger().info(f"y_offset = {y_offset}, y_vel = {y_vel}")
                self.send_movement(y=y_vel)
                if abs(y_offset) < 5:
                    self.publish_z(1.6)
                    self.mode += 1
                return 
            elif key == 'align':
                    self.get_logger().info(f"angle = {self.angle}")
                    yaw_vel = self.yaw_controller.PID_controller(self.angle,P=20, I=0.001, D=0.0, T_f=0.5, scale_devide=5000, margin=0.2)
                    self.get_logger().info(f"angle = {self.angle}, yaw_vel = {yaw_vel}")
                    self.send_movement(yaw = yaw_vel)
                    if len(self.angle_list) > 10:
                        filtered_angle = sum(self.angle_list[-10:]) / 10
                        if abs(filtered_angle) < 0.3:
                            self.mode+=1
            else:
                distance_offset = self.size - self.desired_distance
                y_offset = self.handler.dims[1]/2 - self.positions[key][0]
                x_vel = self.x_controller.PID_controller(distance_offset,P=20, I=0.001, D=0.0, T_f=0.5, scale_devide=5000, margin=0.2)
                y_vel = self.y_controller.PID_controller(y_offset, P =6, I=0.0001, D=0.0, T_f=0.5, scale_devide=5000, margin=0.1)
                self.get_logger().info(f"distance_offset = {round(distance_offset,4)}, x_vel = {round(x_vel,4)}, y_offset = {round(y_offset,4)}, y_vel = {round(y_vel,4)}")
                self.send_movement(x=x_vel, y=y_vel)
                if abs(distance_offset) < 5/accuracy and abs(y_offset) < self.handler.dims[1]/8*accuracy:
                    self.mode += 1  

    def run_image_strategy(self):
            """Contains and controls the sequence for the mission, mode variable indicates where in the sequence the ROV is"""
            if self.handler.feed_image is not None:
                self.cam_info_get()
            if self.move_bool == False:
                if self.mode == 0:
                    self.camera_regulator('align')
                elif self.mode == 1:
                    self.camera_regulator('center', 5)
                elif self.mode == 2:
                    self.camera_regulator('align')
                elif self.mode == 3:
                    self.camera_regulator('center', 20) #find center with high accuracy
                elif self.mode == 4:
                    if self.front:self.zero_dvl()
                    self.camera_regulator('middle_left')
                elif self.mode == 5:
                    self.camera_regulator('middle_right')
                elif self.mode == 6:
                    self.move_pos(1,-1.3)  #Slide further to the right after finding the right side of the bench
                elif self.mode == 7:
                    self.move_pos(5,90) # turn 90 deg 
                    if not self.front:self.mode += 3 #Check if were behind the bench, if so skip to mode 10. 
                elif self.mode == 8:
                    self.move_pos(1,-3.5)  #Slide to the right to position behind the bench #-2.8
                elif self.mode == 9:
                    self.move_pos(5,90) #rotate 90 deg
                    self.front = False #Now the ROV is behind the bench
                elif self.mode ==10:
                    self.move_pos(1, -2) #sideways slide to get closer to middle of the side of the bench before trying to locate it via imageprocessing.
                    self.mode = 0
                elif self.mode == 11:
                    self.camera_regulator('slide_in') #Find an exact position for sliding in between the bench and teleport upwards
                elif self.mode == 12:
                    self.move_pos(5, 90) # turn around 90 deg to face the bench top front.
                elif self.mode ==13:
                    self.move_pos(1,-3.5) #Slide along the top of the bench to find the last codes.
                elif self.mode ==14:
                    self.handler.filter_arucos() #filter codes
                    self.get_logger().info(f"Aruco list: {self.handler.filtered_list}") #print codes
            
    
    def move_pos(self, axis, distance):
        """Sends movement command to the dvl_movement_node and increase mode by 1"""
        self.move_bool = True
        msg = Twist() 
        msg.linear.x = float(axis)
        msg.linear.y = float(distance)
        self.publisher3.publish(msg)
        self.mode += 1 


def main(args=None):
    rclpy.init(args=args)
    node = MBenchNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
