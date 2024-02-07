import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Float32
from gazebo_msgs.srv import ApplyLinkWrench
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy

class Movement(Node):
    def __init__(self):
        super().__init__('movement')
        self.now_sec = 0
        self.now_nano = 0
        
        clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.clock_subscription = self.create_subscription(Clock, '/clock', self.clock_callback, clock_qos)
        self.clock_subscription

        self.wrench_client = self.create_client(ApplyLinkWrench, '/apply_link_wrench')
        while not self.wrench_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/apply_link_wrench service not available, waiting again...')



    def apply_wrench(self, z_value):
        self.get_logger().info('PROVER APPLY WRENCH')
        req = ApplyLinkWrench.Request()
        req.link_name = 'base_footprint'
        req.reference_frame = 'world'
        req.wrench.force.x = 0.0
        req.wrench.force.y = 0.0
        req.wrench.force.z = z_value
        req.wrench.torque.x = 0.0
        req.wrench.torque.y = 0.0
        req.wrench.torque.z = 0.0
        req.start_time.sec = self.now_sec 
        req.start_time.nanosec = self.now_nano + 100000
        req.duration.sec = 1   
        req.duration.nanosec = 0
        self.wrench_client.call_async(req)
        self.get_logger().info('APPLY WRENCH SUKKSESS')
    



    def clock_callback(self, msg):
        current_simulation_time = msg.clock
        self.now_sec = current_simulation_time.sec
        self.now_nano = current_simulation_time.nanosec
        self.get_logger().info(f'Current Simulation Time: {current_simulation_time.sec}')
        self.get_logger().info(f'Current Simulation Time2: {self.now_sec}')



def main(args=None):
    rclpy.init(args=args)
    movement = Movement()
    rclpy.spin(movement)
    movement.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
