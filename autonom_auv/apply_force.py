import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Vector3
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration

class ForceApplierNode(Node):
    def __init__(self):
        super().__init__('force_applier_node')
        # Create a service client for the ApplyBodyWrench service
        self.apply_body_wrench_client = self.create_client(ApplyBodyWrench, '/gazebo/apply_body_wrench')
        self.get_logger().info('Waiting for /gazebo/apply_body_wrench service...')
        self.apply_body_wrench_client.wait_for_service()
        self.get_logger().info('/gazebo/apply_body_wrench service is available.')

    def apply_force(self):
        request = ApplyBodyWrench.Request()
        request.body_name = 'object_link'  # Replace with the actual link name of your object
        request.reference_frame = 'world'  # or your specific reference frame
        request.wrench.force.x = 10.0  # Specify the X component of the force
        request.wrench.force.y = 0.0  # Specify the Y component of the force
        request.wrench.force.z = 0.0  # Specify the Z component of the force
        request.start_time = self.get_clock().now().to_msg()  # Apply force immediately
        request.duration = Duration(sec=1, nanosec=0)  # Apply force for a specified duration

        # Asynchronously send the request and handle the response
        future = self.apply_body_wrench_client.call_async(request)
        future.add_done_callback(self.force_applied_callback)

    def force_applied_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))
        else:
            if response.success:
                self.get_logger().info('Force applied successfully')
            else:
                self.get_logger().error('Failed to apply force')

def main(args=None):
    rclpy.init(args=args)
    node = ForceApplierNode()
    # Apply force immediately after node creation and service availability
    node.apply_force()
    rclpy.spin(node)
    # Cleanup and shutdown after node has been stopped
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
