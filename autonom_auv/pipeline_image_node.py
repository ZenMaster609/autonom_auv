import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32
import numpy as np
from .image_methods import ImageMethods
from .pid_controller import PidController
from .pid_controller import transfer_funtion_class
from .movement_node import compute_speed
from .image_handler import ImageHandler
from .image_handler import logging_data
import signal
from geometry_msgs.msg import Twist
import asyncio
import time
from sensor_msgs.msg import Imu

class PipelineImageNode(Node):
    def __init__(self,mode):
        super().__init__('image_processor') 
        self.create_subscription(Image,'/camera/image_raw',  self.listener_callback,10)
        self.bridge = CvBridge()
        self.create_subscription(Imu,"/imu/data", self.imu_callback,10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.angeleuar_controller = PidController()
        self.y_controller = PidController()
        self.compute_speed1 = compute_speed()
        self.image_pipe = ImageHandler()
        self.logger = logging_data()
        self.mode=mode
        self.yaw_tf =  transfer_funtion_class([1007],[180.5, 131.043,1507])
        self.k = 0
        self.loop = asyncio.get_event_loop()


    def custom_cleanup(self):
        self.logger.plot_data_table(self.colum1,self.colum2,self.filtered_ids,self.plot_names)
        self.get_logger().info(f'I ran')

    def imu_callback(self,data):
        angular_velocity = data.angular_velocity
        self.get_logger().info(f"Angular velocity: x={angular_velocity.x}, y={angular_velocity.y}, z={angular_velocity.z}")


    def send_movement(self,ang_vel=0.0,linear_y_vel=0.0):
        move_cmd = Twist()
        move_cmd.linear.x = 0.2
        #time.sleep(0.2)
        # if self.k > 10:
        #     await asyncio.sleep(0.2)
        move_cmd.angular.z = ang_vel
        move_cmd.linear.y = linear_y_vel
        self.publisher_.publish(move_cmd)
        self.k += 1


    def listener_callback(self, data):

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image_edit = cv_image.copy()

        the_box = self.image_pipe.find_box(cv_image,image_edit,"pipeline_sim",70000,True)
        angle_deg,center_x,center_y = self.image_pipe.find_box_info(the_box,image_edit,90,True)

        offsett_x = PidController.calculate_parameters((center_x),960)


        if self.mode ==1:
            angle_vel =self.angeleuar_controller.PID_controller(angle_deg,(2),0.0,0.0,100)
            angle_vel =angle_deg/90
            linear_y_vel =  self.y_controller.PID_controller(offsett_x,(10.62),0.05,0.05,10000)
            real_angle_vel = self.yaw_tf.impliment_transfer_function(angle_vel)
            self.send_movement(real_angle_vel,linear_y_vel)
        else:
            angle_vel= self.angeleuar_controller.PID_controller(offsett_x,(7.8125),0.05,0.05,10000)
            self.send_movement(angle_vel)

        


        self.filtered_ids = self.image_pipe.aruco_handler(cv_image,image_edit,the_box)
        ImageMethods.showImage(image_edit,0.7)
    
        self.plot_names=["","Angel offset in degrees","Ideal Angeluar Velocity","Real Angeluar Velocity"]
        self.logger.log_data(angle_deg,angle_vel,real_angle_vel)
        self.colum1 = ["P","I","D","Acceleration","min area box"]
        self.colum2 = [2,0,0,0.4654,70000]




        
def main(args=None):
    rclpy.init(args=args)
    image_processor = PipelineImageNode(1)
    signal.signal(signal.SIGINT, lambda sig, frame: image_processor.custom_cleanup())
    rclpy.spin(image_processor)
    image_processor.custom_cleanup()  # Ensure cleanup is called if exit wasn't due to SIGINT
    rclpy.shutdown()
    cv2.destroyAllWindows()
    image_processor.destroy_node()



if __name__ == '__main__':
    main()
    
