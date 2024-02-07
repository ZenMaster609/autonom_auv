import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Float32

class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Float32,'/angular_velocity', self.ang_vel_callback,10)
        self.subscription  # Prevent unused variable warning
        

    def ang_vel_callback(self,msg):
        angular_velocity = round(msg.data,3)
        self.get_logger().info(f'Received: {angular_velocity}')
        self.move_in_square(angular_velocity)


    def move_in_square(self,ang_vel):
        move_cmd = Twist()
        move_cmd.linear.x = 0.2
        move_cmd.angular.z =ang_vel
        self.publisher_.publish(move_cmd)
            


def main(args=None):
    rclpy.init(args=args)
    square_mover = SquareMover()
    rclpy.spin(square_mover)

    # Destroy the node explicitly
    square_mover.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
