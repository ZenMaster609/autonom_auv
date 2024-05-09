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
import math

class PipelineImageNode(Node):
    def __init__(self,mode):
        super().__init__('mission_pipeline_node') 
        #Topic pub/sub and timer declaration
        self.create_subscription(Image,'/camera/image_raw',  self.listener_callback,40)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 40)
        self.publisher = self.create_publisher(Twist, '/target', 40)
        self.publisher1 = self.create_publisher(Twist, 'tf_movement', 40) #/tf_movement
        self.create_timer(0.05, self.timer_callback1)  

        #Variable declaration
        self.bridge = CvBridge()
        self.angular_controller = PidController()
        self.y_controller = PidController()
        self.handler = ImageHandler()
        self.logger = logging_data()
        self.logger_y = logging_data()
        self.logger_dvl = logging_data()
        self.mode=mode
        self.time_start = time.time()
        self.cv_image = None
        self.state = 0
        self.pid_gir =[1, 0.19, 0.19] #[0.56, 2.89, 0.18] [1, 0.19, 0.19]
        self.pid_svai =[6,0.19, 0.14] #[0.55, 2.84, 0.13]
        self.super_start = time.time()
        self.colum1 = ["P","I","D","Acceleration","min area box"]
        self.colum2 = [17,0.1,0,0.1,.4654,75000]
        self.counter = 0 

    def move_pos(self, axis, distance):
        """Request movement through the DVL_movement_node"""
        msg = Twist() 
        msg.linear.x = float(axis)
        msg.linear.y = float(distance)
        self.publisher.publish(msg)

    def custom_cleanup(self):
        """Cleans up certain error messages when closing node"""
        self.logger.plot_data_table("gir", self.colum1,self.pid_gir,self.handler.filter_arucos(),self.plot_names)
        self.logger_y.plot_data_table("svai" ,self.colum1,self.pid_svai,self.handler.filter_arucos(),self.plot_names_y)
        self.logger_dvl.plot_data("xy" ,self.plot_names_dvl)
        self.get_logger().info(f'I ran')

    def send_movement(self,ang_vel=0.0,linear_y_vel=0.0):
        """Request movement via the Movement_node"""
        movement = Twist()
        movement.linear.x = 0.6
        # if time.time()-self.super_start >= 6:
        #     movement.linear.x = 2.0
        # self.get_logger().info(f"time:{time.time()-self.super_start} x vel:{self.velocity_x}")
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
        self.odom_yaw ,a,b= ImageMethods.quaternion_to_euler(msg.pose.pose.orientation.z,0,0,msg.pose.pose.orientation.w)
        self.angular_yaw = -msg.twist.twist.angular.z
        self.velocity_y = msg.twist.twist.linear.y
        self.velocity_x = msg.twist.twist.linear.x

    def listener_callback(self, data): #Get imagefeed.
        self.handler.feed_image = self.bridge.imgmsg_to_cv2(data, "bgr8") 

    def timer_callback1(self): #Timer callback to run the tasks for the mission.
        if self.handler.feed_image is not None:
            time_elapsed = time.time() - self.super_start
            self.time_start = time.time()
            angle, center_x, done = self.handler.find_pipeline(75000) #Get info about the pipeline position'
            distance_pipe = self.odom_z-0.2
            center_x_meters = ImageMethods.pixles_to_meters(center_x,distance_pipe,self.handler.dims[1])
            #center_x_meters = ImageMethods.pixles_to_meters69(center_x)

            if self.state == 1:
                self.plot_names_dvl=["X cordinates","Y cordinates","Yaw",""]
                self.logger_dvl.log_data(self.odom_x,self.odom_y,self.odom_yaw)
                self.colum1 = ["P","I","D","Acceleration","min area box"]
                self.colum2 = [17,0.1,0,0.1,.4654,75000]
                return #if done stop camera based movement
            
            #self.get_logger().info(f"time elapsed = {time_elapsed}")
            if done and time_elapsed > 4:
                self.get_logger().info(f"Aruco List:{self.handler.filter_arucos()}")
                self.state = 1
                self.move_pos(0.0,0.0) #tell the DVl movement node you want to go home
                return
            
            set_point = ImageMethods.pixles_to_meters(self.handler.dims[1]/2,distance_pipe,self.handler.dims[1])
            offsett_x = -PidController.calculate_offset((center_x_meters),set_point)
            if self.mode ==1:
                angle_vel = self.angular_controller.PID_controller(angle,*self.pid_gir, scale_devide=1, u_I_max=0.10)  #PID-regulate the ROV yaw
                linear_y_vel = self.y_controller.PID_controller(-offsett_x,*self.pid_svai, scale_devide=1,u_I_max=0.02) #Do the same for ROV Y posistion
                if linear_y_vel > 1:linear_y_vel=1.0
                if abs(angle)> math.pi/4: linear_y_vel = 0.0
                self.send_movement(angle_vel,linear_y_vel)   #send regulated values
                #self.get_logger().info(f" angle= {angle}, angel degrees = {angle*180/math.pi}")
            else:
                angle_vel= self.angular_controller.PID_controller(offsett_x,(7.8125),0.05,0.05,0.5,10000)  #PID-regulate the ROV yaw
                self.send_movement(linear_y_vel) #send regulated value
            # gir = yaw, svai = y, jag = x
            #push logging data once per tick
           
            self.plot_names=["","Angle offset in degrees","Ideal angular velocity","Real angular velocity"]
            self.logger.log_data(angle,-angle_vel, self.angular_yaw )

            self.plot_names_y=["","Offsett meters","Ideal Velocity","Real Velocity"]
            self.logger_y.log_data(offsett_x,linear_y_vel,self.velocity_y)
            self.counter += 1 
            self.get_logger().info(f'Count pipeline: {self.counter}')
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
    
