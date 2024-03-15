import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time 
import numpy as np
from .pid_controller import PidController
from .pid_controller import transfer_funtion_class
from nav_msgs.msg import Odometry

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Twist,'/tf_movement', self.movement_callback,10)
        self.yaw_controller = PidController()
        self.yaw_tf =  transfer_funtion_class([3.74, 61.48, 2546], [1, 38.65, 519.6, 2533])  
        self.x_tf = transfer_funtion_class([3.606,40.12,1421],[1, 16.98, 273.3, 1411])
        self.y_tf = transfer_funtion_class([4.014, 83.39, 2173],[1, 31.09, 357.9, 2170])
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0  

    def publish_movement(self):
        cmd_vel = Twist()
        self.yaw = self.yaw_tf.implement_transfer_function(self.yaw)
        self.x = self.x_tf.implement_transfer_function(self.x)
        self.y = self.y_tf.implement_transfer_function(self.y)
        cmd_vel.linear.x = self.x
        cmd_vel.linear.y = self.y
        cmd_vel.linear.z = self.z
        cmd_vel.angular.x = self.roll
        cmd_vel.angular.y = self.pitch
        cmd_vel.angular.z = self.yaw
        self.publisher.publish(cmd_vel)

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_z = msg.pose.pose.position.z
        self.odom_roll = msg.pose.pose.orientation.x
        self.odom_yaw = msg.pose.pose.orientation.z

    def movement_callback(self,msg):
        self.x = msg.linear.x
        self.y = msg.linear.y
        self.z = msg.linear.z
        self.roll = msg.angular.x
        self.pitch = msg.angular.y
        self.yaw = msg.angular.z
        self.publish_movement()
        







class compute_speed:
    def __init__(self):
        self.pre_speed = 0
        self.pre_time = time.time()

    def real_speed(self,speed_in,acceleration):
        sign = np.sign(speed_in)
        time_now = time.time()
        t_s = time_now - self.pre_time
        speed_out = self.pre_speed+(acceleration*t_s*sign)
        if sign >=0:    
            if speed_out > speed_in: speed_out = speed_in
        elif sign < 0:
            if speed_out < speed_in: speed_out = speed_in
        self.pre_time = time_now
        self.pre_speed = speed_out
        return speed_out


def main(args=None):
    rclpy.init(args=args)
    movement = MovementNode()
    rclpy.spin(movement)

    # Destroy the node explicitly
    movement.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
