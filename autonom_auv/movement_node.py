import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float32,'/angular_velocity', self.ang_vel_callback,10)
 
        

    def ang_vel_callback(self,msg):
        angular_velocity = round(msg.data,3)
        self.send_movement(angular_velocity)


    def send_movement(self,ang_vel):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z =ang_vel
        self.publisher_.publish(move_cmd)
            


def main(args=None):
    rclpy.init(args=args)
    movement = MovementNode()
    rclpy.spin(movement)

    # Destroy the node explicitly
    movement.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
