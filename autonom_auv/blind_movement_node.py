
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time 
import numpy as np
from .pid_controller import PidController
from nav_msgs.msg import Odometry
import math
from example_interfaces.srv import AddTwoInts
from std_msgs.msg import Bool
class BlindMoveNode(Node):
    def __init__(self):
        super().__init__('movement_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_bool = self.create_publisher(Bool, '/done_moving', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/move_serv', self.move_serv_callback, 10)

        self.yaw_controller = PidController()
        self.x_controller = PidController()
        self.target_yaw = None 
        self.target_x = None 



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
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_z = msg.pose.pose.position.z
        quaternion = msg.pose.pose.orientation
        self.odom_roll, self.odom_pitch, self.odom_yaw = quaternion_to_euler(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        self.check_yaw_goal()
        if self.target_yaw == None:
            self.check_x_goal()
        #self.get_logger().info(f"odom_x = {self.odom_x}")

    def check_yaw_goal(self, axis):
        if self.target_yaw is None or self.odom_yaw is None:
            return  # Do nothing if we don't have a target or current state
        if abs(self.target_yaw - self.odom_yaw) < 0.005: # Check if we are close to the target
            self.publish_movement(yaw=0.0)  # Stop turning
            self.target_yaw = None  # Reset target
        else:
            yaw_offset = self.target_yaw - self.odom_yaw
            angle_vel = self.yaw_controller.PID_controller(yaw_offset, 100, 0.0, 0.0, 100, 0)
            self.publish_movement(yaw=angle_vel)
            self.get_logger().info(f"goal = {self.target_yaw}, yaw_offset = {yaw_offset}, odom_yaw = {self.odom_yaw}, angle_vel = {angle_vel}")

    def check_x_goal(self):
        if self.target_x is None or self.odom_x is None:
            return  # Do nothing if we don't have a target or current state
        if abs(self.odom_yaw) < 0.1: #hvis vanlig orientasjon
            odom_x_fixed = self.odom_x
        elif abs(self.odom_yaw) > 3.1:
            odom_x_fixed = -self.odom_x
        else:
            self.get_logger().error('Current yaw not within limits for decent odometric calculations')

        if abs(self.target_x - odom_x_fixed) < 0.1: # Check if we are close to the target
            self.publish_movement(x=0.0)  # Stop turning
            self.target_x = None  # Reset target
        else:
            x_offset = self.target_x - odom_x_fixed
            x_vel = self.x_controller.PID_controller(x_offset, 80, 0.0, 0.0, 100, 0)
            self.publish_movement(x=x_vel)
            self.get_logger().info(f"goal = {self.target_x}, x_offset = {x_offset}, odom_x = {odom_x_fixed}, x_vel = {x_vel}")
        
    def turn_angle(self, angle):
        if self.odom_yaw is None:
            self.get_logger().error('Current yaw not known, cannot turn.')
            return
        rad_angle = math.radians(angle)
        self.target_yaw = self.odom_yaw + rad_angle 


        


def main(args=None):
    rclpy.init(args=args)
    node = BlindMoveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

def quaternion_to_euler(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z