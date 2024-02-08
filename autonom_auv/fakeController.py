import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from .mathClasses import vectorCalculator
import time

class FakeControllerNode(Node):
    def __init__(self):
        super().__init__('fakeController')
        #self.publisher_ = self.create_publisher(Twist, '/desired_vel_cmd', 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(5)
        self.publish_fake_movement(z=1.0)
        time.sleep(5)
        self.publish_fake_movement(z=0.5)
        time.sleep(5)
        self.publish_fake_movement(z=-1.0)


    def publish_fake_movement(self, x=0.0, y=0.0, z=0.0, xt=0.0, yt=0.0, zt=0.0):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.x = xt
        msg.angular.y = yt
        msg.angular.z = zt
        self.publisher_.publish(msg)


    
def main(args=None):
    rclpy.init(args=args)
    fakeController = FakeControllerNode()
    rclpy.spin(fakeController)
    fakeController.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()