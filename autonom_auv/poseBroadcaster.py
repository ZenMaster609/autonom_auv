import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class PoseBroadcaster(Node):
    def __init__(self):
        super().__init__('pose_broadcaster')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        # Assume you have a way to periodically update the robot's pose
        # For example, a timer callback that updates pose based on sensor data
        self.timer = self.create_timer(0.1, self.update_pose)

    def update_pose(self):
        # Example: Update the robot's pose here
        x, y, theta = self.get_current_pose()

        # Create a TransformStamped message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # Convert theta (yaw) to a quaternion and set rotation
        q = self.yaw_to_quaternion(theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Broadcast the transform
        self.broadcaster.sendTransform(t)

    def get_current_pose(self):
        # Placeholder: Get the robot's current position and orientation (yaw)
        return (0.0, 0.0, 0.0)

    def yaw_to_quaternion(self, yaw):
        # Convert a yaw angle (in radians) to a quaternion
        return [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]



def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    pose_broadcaster = PoseBroadcaster()  # Create an instance of your node
    rclpy.spin(pose_broadcaster)  # Keep your node running

    # Cleanup and shutdown
    pose_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()