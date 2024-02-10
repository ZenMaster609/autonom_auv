import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetEntityState


class UpDownNode(Node):
    def __init__(self):
        super().__init__('up_down_node')
        self.last_linear_z = None
        self.current_x = None
        self.current_y = None
        self.current_orientation = None

        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.set_state_client = self.create_client(SetEntityState, '/demo/set_entity_state')
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /demo/set_entity_state service...')


    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_orientation = msg.pose.pose.orientation

    def cmd_vel_callback(self, msg):
        if self.last_linear_z != msg.linear.z:
            self.last_linear_z = msg.linear.z
            self.update_my_bot_position()

    def update_my_bot_position(self):
        if None in (self.current_x, self.current_y, self.current_orientation):
            self.get_logger().error('Current position or orientation not yet received.')
            return
        set_req = SetEntityState.Request()
        set_req.state.name = 'my_bot'
        set_req.state.pose.position.x = self.current_x
        set_req.state.pose.position.y = self.current_y
        set_req.state.pose.position.z = self.last_linear_z
        set_req.state.pose.orientation = self.current_orientation
        future = self.set_state_client.call_async(set_req)
        future.add_done_callback(self.set_state_callback)

    def set_state_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Successfully updated position of my_bot.')
            else:
                self.get_logger().error('Failed to update position of my_bot.')
        except Exception as e:
            self.get_logger().error(f'Exception while updating position: {e}')




def main(args=None):
    rclpy.init(args=args)
    node = UpDownNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
