import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Quaternion
import tf2_ros
from tf2_ros import Buffer, TransformListener
import numpy as np
from .mathClasses import vectorCalculator

class RelativeForceNode(Node):
    def __init__(self):
        super().__init__('relativeForce')
        self.velWorld = Vector3(x=0.0, y=0.0, z=0.0)
        self.torqueWorld = Vector3(x=0.0, y=0.0, z=0.0)
        self.publisher_ = self.create_publisher(Twist, '/force_torque_cmd', 10)
        self.subscription = self.create_subscription(Twist, '/desired_vel_cmd', self.publish_force_torque_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def publish_force_torque_callback(self, msg):
        orientation = self.get_robot_orientation()
        if orientation is None:
            self.get_logger().error('No orientation available.')
            return
        
        desiredVel_world = self.transform_vector_to_world_frame(msg.linear, orientation)
        desiredTorque_world = self.transform_vector_to_world_frame(msg.angular, orientation)
        addedVel_world = vectorCalculator.subtract_two_vectors(desiredVel_world, self.velWorld)
        addedTorque_world = vectorCalculator.subtract_two_vectors(desiredTorque_world, self.torqueWorld)

        self.velWorld = vectorCalculator.add_two_vectors(self.velWorld, addedVel_world)
        self.torqueWorld = vectorCalculator.add_two_vectors(self.torqueWorld, addedTorque_world)
        
        sendmsg = Twist()
        sendmsg.linear = addedVel_world
        sendmsg.angular = addedTorque_world
        self.publisher_.publish(sendmsg)

    def get_robot_orientation(self):
        try:
            trans = self.tf_buffer.lookup_transform('world', 'base_footprint', rclpy.time.Time())
            return trans.transform.rotation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Failed to get transform: {str(e)}")
            return None

    def transform_vector_to_world_frame(self, vector, orientation):
        # Assuming a simple rotation around Z-axis
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler_z = self.quaternion_to_euler_z(q)
        
        rotated_vector = self.rotate_vector(vector, euler_z)
        return rotated_vector

    @staticmethod
    def quaternion_to_euler_z(quaternion):
        """
        Convert a quaternion to Euler angle on the Z axis.
        """
        x, y, z, w = quaternion
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y*y + z*z)
        euler_z = np.arctan2(t3, t4)
        return euler_z

    @staticmethod
    def rotate_vector(vector, angle):
        """
        Rotate a vector by a given angle around the Z-axis.
        """
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        
        rotated_x = vector.x * cos_angle - vector.y * sin_angle
        rotated_y = vector.x * sin_angle + vector.y * cos_angle
        rotated_z = vector.z  # No change in Z-axis in 2D rotation
        
        return Vector3(x=rotated_x, y=rotated_y, z=rotated_z)

def main(args=None):
    rclpy.init(args=args)
    relativeForce = RelativeForceNode()
    rclpy.spin(relativeForce)
    relativeForce.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
