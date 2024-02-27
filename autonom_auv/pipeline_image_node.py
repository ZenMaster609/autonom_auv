import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ament_index_python.packages import get_package_share_directory
from .image_methods import ImageMethods
from .pid_controller import PidController
from .image_handler import ImageHandler
from .image_handler import logging_data
import signal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class PipelineImageNode(Node):
    def __init__(self,mode):
        super().__init__('image_processor') 
        self.create_subscription(Image,'/camera/image_raw',  self.listener_callback,10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Twist, '/movement', 10)
        self.angeleuar_controller = PidController()
        self.y_controller = PidController()
        self.image_pipe = ImageHandler()
        self.logger = logging_data()
        self.mode=mode
        

    def custom_cleanup(self):
        self.logger.plot_data_table(self.colum1,self.colum2,self.filtered_ids,self.plot_names)
        self.get_logger().info(f'I ran')



    def send_movement(self,ang_vel=0.0,linear_y_vel=0.0):
        movement = Twist()
        movement.linear.x = 0.4
        movement.angular.z = ang_vel
        movement.linear.y = linear_y_vel
        self.publisher_.publish(movement)
    
    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_z = msg.pose.pose.position.z
        self.odom_roll = msg.pose.pose.orientation.x
        self.odom_pitch = msg.pose.pose.orientation.y
        self.angular_yaw = msg.twist.twist.angular.z

    def listener_callback(self, data):
        self.time_start = time.time()
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image_edit = cv_image.copy()
        print(f"time1: {self.time_start-time.time()}")
        the_box = self.image_pipe.find_box(cv_image,image_edit,"pipeline_sim",70000,True)
        angle_deg,center_x,center_y = self.image_pipe.find_box_info(the_box,image_edit,90,True)
        offsett_x = PidController.calculate_parameters((center_x),960)

        if self.mode ==1:
            angle_vel =self.angeleuar_controller.PID_controller(angle_deg,(15),0.0,0.0,1000)
            linear_y_vel =  self.y_controller.PID_controller(offsett_x,(10.62),0.05,0.05,10000)
            self.send_movement(angle_vel,linear_y_vel)            
        else:
            angle_vel= self.angeleuar_controller.PID_controller(offsett_x,(7.8125),0.05,0.05,10000)
            self.send_movement(angle_vel)

        self.filtered_ids = self.image_pipe.aruco_handler(cv_image,image_edit,the_box)
        ImageMethods.showImage(image_edit,0.7)
    
        self.plot_names=["","Angel offset in degrees","Ideal Angeluar Velocity","Real Angeluar Velocity"]
        self.logger.log_data(angle_deg,angle_vel,self.angular_yaw )
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
    
