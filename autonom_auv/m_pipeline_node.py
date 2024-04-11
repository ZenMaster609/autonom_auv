import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ament_index_python.packages import get_package_share_directory
from .image_methods import ImageMethods
from .controller import PidController
from .image_handler import ImageHandler
from .image_handler import logging_data
import signal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class PipelineImageNode(Node):
    def __init__(self,mode):
        super().__init__('mission_pipeline_node') 
        #Topic pub/sub and timer declaration
        self.create_subscription(Image,'/camera/image_raw',  self.listener_callback,10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/target', 10)
        self.publisher1 = self.create_publisher(Twist, '/tf_movement', 10)
        self.create_timer(0.05, self.timer_callback1)  

        #Variable declaration
        self.bridge = CvBridge()
        self.angular_controller = PidController()
        self.y_controller = PidController()
        self.handler = ImageHandler()
        self.logger = logging_data()
        self.mode=mode
        self.time_start = time.time()
        self.cv_image = None
        self.state = 0
        self.pid_gir = [1, 0.001, 0.2]
        self.pid_svai = [0.1, 0.0, 0.0]
        self.super_start = time.time()

    def move_pos(self, axis, distance):
        """Request movement through the DVL_movement_node"""
        msg = Twist() 
        msg.linear.x = float(axis)
        msg.linear.y = float(distance)
        self.publisher.publish(msg)

    def custom_cleanup(self):
        """Cleans up certain error messages when closing node"""
        self.logger.plot_data_table(self.colum1,self.colum2,self.handler.filter_arucos(),self.plot_names)
        self.get_logger().info(f'I ran')

    def send_movement(self,ang_vel=0.0,linear_y_vel=0.0):
        """Request movement via the Movement_node"""
        movement = Twist()
        movement.linear.x = 0.6
        movement.angular.z = ang_vel
        movement.linear.y = linear_y_vel
        self.testmeg = movement.angular.z
        self.publisher1.publish(movement)
    
    def odom_callback(self, msg):
        """Fetch the odom of the ROV"""
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_z = msg.pose.pose.position.z
        self.odom_roll = msg.pose.pose.orientation.x
        self.odom_pitch = msg.pose.pose.orientation.y
        self.angular_yaw = -msg.twist.twist.angular.z

    def listener_callback(self, data): #Get imagefeed.
        self.handler.feed_image = self.bridge.imgmsg_to_cv2(data, "bgr8") 

    def timer_callback1(self): #Timer callback to run the tasks for the mission.
        if self.handler.feed_image is not None:
            time_elapsed = time.time() - self.super_start
            self.time_start = time.time()
            angle_deg,center_x, done = self.handler.find_pipeline(75000) #Get info about the pipeline position
            if self.state == 1:return #if done stop camera based movement
            self.get_logger().info(f"time elapsed = {time_elapsed}")
            if done and time_elapsed > 4:
                self.get_logger().info(f"Aruco List:{self.handler.filter_arucos()}")
                self.state = 1
                self.move_pos(0,0.0) #tell the DVl movement node you want to go home
                return
            offsett_x = -PidController.calculate_offset((center_x),self.handler.dims[1]/2)
            if self.mode ==1:
                angle_vel = self.angular_controller.PID_controller(angle_deg,*self.pid_gir, scale_devide=100)  #PID-regulate the ROV yaw
                linear_y_vel = self.y_controller.PID_controller(-offsett_x,*self.pid_svai, scale_devide=100) #Do the same for ROV Y posistion
                if linear_y_vel > 1:linear_y_vel=1.0
                self.send_movement(angle_vel,linear_y_vel)   #send regulated values
                self.get_logger().info(f"y = {self.odom_y}, yaw = {self.angular_yaw}")
            else:
                angle_vel= self.angular_controller.PID_controller(offsett_x,(7.8125),0.05,0.05,0.5,10000)  #PID-regulate the ROV yaw
                self.send_movement(angle_vel) #send regulated value
            # gir = yaw, svai = y, jag = x
            #push logging data once per tick
            self.plot_names=["","angle offset in degrees","Ideal angleuar Velocity","Real angleuar Velocity"]
            self.logger.log_data(angle_deg,angle_vel,self.angular_yaw )
            self.colum1 = ["P","I","D","Acceleration","min area box"]
            self.colum2 = [17,0.1,0,0.1,.4654,75000]
            #print(f"time5: {time.time()-self.time_start}")
        
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
    
