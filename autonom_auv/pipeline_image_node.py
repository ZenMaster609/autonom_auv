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
from .image_methods import ImageMethods
from .pid_controller_node import PidControllerNode
from .movement_node import compute_speed

from geometry_msgs.msg import Twist
import math

class PipelineImageNode(Node):
    def __init__(self,mode):
        super().__init__('image_processor') 
        self.create_subscription(Image,'/camera/image_raw',  self.listener_callback,10)
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.Ids_list= []
        self.angeleuar_controller = PidControllerNode()
        self.y_controller = PidControllerNode()
        self.compute_speed1 = compute_speed()
        self.cooldown = 0
        self.mode=mode

    def send_movement(self,ang_vel=0.0,linear_y_vel=0.0):
        move_cmd = Twist()
        move_cmd.linear.x = 0.6
        ang_vel = self.compute_speed1.real_speed(ang_vel, 0.05)
        move_cmd.angular.z = ang_vel
        move_cmd.linear.y = linear_y_vel
        self.publisher_.publish(move_cmd)



    def listener_callback(self, data):

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image_edit =cv_image.copy()
        dimensions = cv_image.shape
       
     
        mask = ImageMethods.color_filter(cv_image,[30,114,114],[30,255,255])
        mask = cv2.line(mask,(0,600),(dimensions[1],600),(0,0,0),10)
        box_list = ImageMethods.find_boxes(mask, image_edit, 70000, True)
        the_box = ImageMethods.find_the_box(box_list)
        
        angle_deg = ImageMethods.find_angle_box(the_box,90)
        angle_deg, self.cooldown = ImageMethods.angel_cooldown(angle_deg,self.cooldown)

        center_x,center_y = ImageMethods.find_Center(image_edit,the_box, True)


        cv2.putText(image_edit, f"{int(angle_deg)}",[1600,1050], cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 0, 255), 2, cv2.LINE_AA)
        offsett_x = PidControllerNode.calculate_parameters((center_x),960)

        if self.mode ==1:
            angle_vel =self.angeleuar_controller.PID_controller(angle_deg,(0.02),0.0000,0.0000)
            linear_y_vel =  self.y_controller.PID_controller(offsett_x,(2/1920),0.000005,0.000005)
            self.send_movement(angle_vel,linear_y_vel)
        else:
            angle_vel= self.angeleuar_controller.PID_controller(offsett_x,(1.5/1920),0.000005,0.000005)
            self.send_movement(angle_vel)

        self.Ids_list= ImageMethods.read_AruCo(cv_image,image_edit,self.Ids_list)
        if the_box is None:
            ids = ImageMethods.filtered_ids_list(self.Ids_list)
            self.get_logger().info(f"{ids}")
      
        image_show = cv2.resize(image_edit, (0, 0),fx=0.7, fy=0.7)
        ImageMethods.showImage(image_show)



        
def main(args=None):
    rclpy.init(args=args)
    image_processor = PipelineImageNode(1)
    rclpy.spin(image_processor)
    cv2.destroyAllWindows()
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
