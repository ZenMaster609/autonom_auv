import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from .mathClasses import vectorCalculator

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller')
        self.velocity = Vector3(x=0.0,y=0.0,z=0.0)
        self.torque = Vector3(x=0.0,y=0.0,z=0.0)
        self.publisher_ = self.create_publisher(Twist, '/force_torque_cmd', 10)


        self.subscription = self.create_subscription(Twist, '/desired_vel_cmd', self.publish_force_torque_callback, 10)
        self.subscription 
    
    
    def publish_force_torque_callback(self, msg):
        sendmsg = Twist()
        desiredVel = Vector3(x=msg.linear.x,y=msg.linear.y,z=msg.linear.z)
        desiredTorque = Vector3(x=msg.angular.x, y=msg.angular.y, z=msg.angular.z)
        addedTorque = vectorCalculator.subtract_two_vectors(desiredTorque, self.torque)
        addedVelocity = vectorCalculator.subtract_two_vectors(desiredVel, self.velocity)
        # self.get_logger().info(f'Previous speed: {self.velocity}')
        # self.get_logger().info(f'Desired speed: {desiredVel}')
        # self.get_logger().info(f'Added Speed: {addedVelocity}')
        # self.get_logger().info(f'Final speed: {self.velocity}')
        sendmsg.linear.x = addedVelocity.x  
        sendmsg.linear.y = addedVelocity.y
        sendmsg.linear.z = addedVelocity.z
        sendmsg.angular.x = addedTorque.x
        sendmsg.angular.y = addedTorque.y
        sendmsg.angular.z = addedTorque.z
        self.velocity = vectorCalculator.add_two_vectors(self.velocity, addedVelocity)
        self.torque = vectorCalculator.add_two_vectors(self.torque, addedTorque)
        self.publisher_.publish(sendmsg)




def main(args=None):
    rclpy.init(args=args)
    controller = ControllerNode()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()