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
        self.create_subscription(Image,'/camera/image_raw',  self.listener_callback,10)
        self.cap = cv2.VideoCapture(r'/home/master/Desktop/VideoRor.mp4')
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video file")
            return
        # Use a timer to read and process frames at a desired rate
        self.create_timer(0.05, self.process_video_frame)


        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/target', 10)
        self.create_timer(0.05, self.timer_callback1)
        #self.create_timer(1, self.timer_callback2)
        self.publisher1 = self.create_publisher(Twist, '/tf_movement', 10)  
        self.bridge = CvBridge()
        self.angular_controller = PidController()
        self.y_controller = PidController()
        self.handler = ImageHandler()
        self.logger = logging_data()
        self.mode=mode
        self.time_start = time.time()
        self.cv_image = None
        self.state = 0
        
    def move_pos(self, axis, distance):
        msg = Twist() 
        msg.linear.x = float(axis)
        msg.linear.y = float(distance)
        self.publisher.publish(msg)

    def custom_cleanup(self):
        self.get_logger().info(f'Aruco {self.handler.filter_arucos()}')
        self.logger.plot_data_table(self.colum1,self.colum2,self.handler.filter_arucos(),self.plot_names)
        self.get_logger().info(f'I ran')

    def send_movement(self,ang_vel=0.0,linear_y_vel=0.0):
        movement = Twist()
        movement.linear.x = 1.1
        movement.angular.z = ang_vel
        movement.linear.y = linear_y_vel
        self.testmeg = movement.angular.z
        self.publisher1.publish(movement)
    
    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_z = msg.pose.pose.position.z
        self.odom_roll = msg.pose.pose.orientation.x
        self.odom_pitch = msg.pose.pose.orientation.y
        self.angular_yaw = msg.twist.twist.angular.z

    def listener_callback(self, data):
        self.handler.feed_image = self.bridge.imgmsg_to_cv2(data, "bgr8") 

    def timer_callback1(self):
        if self.handler.feed_image is not None:
            #print(f"t_s: {time.time()-self.time_start}")
            self.time_start = time.time()
            #print(f"time1: {self.time_start-time.time()}")
            #print(f"time2: {self.time_start-time.time()}")
            angle_deg,center_x, done = self.handler.find_pipeline()
            if self.state == 1:return
            if done:
                self.state = 1
                print("done")
                self.move_pos(0,0.0) #if distance = 0.0, home
                return
            #print(f"time3: {self.time_start-time.time()}")
            set_point = (self.handler.dims[0])/2
            offsett_x = PidController.calculate_offset((center_x),self.handler.dims[1]/2)
            # print(f"time4: {self.time_start-time.time()}")
            #if not done:
            if self.mode ==1:
                angle_vel =self.angular_controller.PID_controller(angle_deg,(15),0.0,0.0,0.5,1000)
                linear_y_vel =  self.y_controller.PID_controller(offsett_x,(10.62),0.05,0.05,0.5,10000)
                self.send_movement(angle_vel,linear_y_vel)            
            else:
                angle_vel= self.angular_controller.PID_controller(offsett_x,(7.8125),0.05,0.05,0.5,10000)
                self.send_movement(angle_vel)
            self.plot_names=["","angle offset in degrees","Ideal angleuar Velocity","Real angleuar Velocity"]
            self.logger.log_data(angle_deg,angle_vel,self.angular_yaw )
            self.colum1 = ["P","I","D","Acceleration","min area box"]
            self.colum2 = [2,0,0,0.4654,70000]
            # else:
            #     self.state = 1
            #     print("done")
            #     self.move_pos(0,0.0) #if distance = 0.0, home
                
            #print(f"time5: {self.time_start-time.time()}")
            #self.handler.show_image(False)
            #print(f"time6: {self.time_start-time.time()}")
            
            #print(f"time7: {self.time_start-time.time()}")



        
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
    
