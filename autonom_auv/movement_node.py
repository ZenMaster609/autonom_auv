import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time 
import numpy as np
from .pid_controller import PidController
from nav_msgs.msg import Odometry

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.yaw_controller = PidController()

    def publish_movement(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.x = roll
        msg.angular.y = pitch
        msg.angular.z = yaw
        self.publisher.publish(msg)

    def odom_callback(self, msg):
        self.odom_x = msg.pose.position.x
        self.odom_y = msg.pose.position.y
        self.odom_z = msg.pose.position.z
        self.odom_roll = msg.pose.pose.orientation.x
        self.odom_pitch = msg.pose.pose.orientation.y
        self.odom_yaw = msg.pose.pose.orientation.z

    def turn_angle(self, angle):
        offset = angle - self.odom_x







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
