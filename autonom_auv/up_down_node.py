import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetEntityState
from std_msgs.msg import Float32

class UpDownNode(Node):
    def __init__(self):
        super().__init__('up_down_node')
        #variable declaration
        self.last_z = None
        self.current_x = None
        self.current_y = None
        self.current_orientation = None
        #Topics
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Float32, '/up_down', self.up_down_callback, 10)
        
        #Services
        self.set_state_client = self.create_client(SetEntityState, '/demo/set_entity_state')
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /demo/set_entity_state service...')


    def odom_callback(self, msg): #Fetches odometry
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_orientation = msg.pose.pose.orientation

    def up_down_callback(self, msg): #Checks if movement command has been recieved in z axis and teleports bot accordingly
        if self.last_z != msg.data:
            self.last_z = msg.data
            self.update_my_bot_position()

    def update_my_bot_position(self):
        """Copies the position of the ROV but uses a given Z value to teleport the bot"""
        if None in (self.current_x, self.current_y, self.current_orientation):
            self.get_logger().error('Current position or orientation not yet received.')
            return
        set_req = SetEntityState.Request()
        set_req.state.name = 'my_bot'
        set_req.state.pose.position.x = self.current_x
        set_req.state.pose.position.y = self.current_y
        set_req.state.pose.position.z = self.last_z
        set_req.state.pose.orientation = self.current_orientation
        future = self.set_state_client.call_async(set_req)
        future.add_done_callback(self.set_state_callback)

    def set_state_callback(self, future):
        """Tries to update the ROV posisiton"""
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
