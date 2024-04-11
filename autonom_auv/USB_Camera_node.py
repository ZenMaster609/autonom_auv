import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import signal
import subprocess
import time

class USB_Camera(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        self.publisher = self.create_publisher(Image, 'usb_camera', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.cv_bridge = CvBridge()
        self.calibrate = False
        self.video_writer = None

        # Run system commands to configure camera and Cheese settings
        self.run_system_commands()

        # Initialize video capture
        self.initialize_video_capture()

    def run_system_commands(self):
        # Set gsettings for Cheese (optional, as Cheese settings do not affect OpenCV)
        # These commands configure Cheese's preferences and do not need to be run if you're not using Cheese alongside
        subprocess.run(['gsettings', 'set', 'org.gnome.Cheese', 'video-x-resolution', '1920'], check=True)
        subprocess.run(['gsettings', 'set', 'org.gnome.Cheese', 'video-y-resolution', '1080'], check=True)

        # Set v4l2 controls for the camera
        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl', 'power_line_frequency=1'], check=True)
        #subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl', 'focus_auto=0'], check=True)
        #subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl', 'zoom_absolute=500'], check=True)

    def initialize_video_capture(self):
        # Initialize OpenCV video capture with the V4L2 backend
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        # Set the resolution for OpenCV video capture
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            self.publisher.publish(self.cv_bridge.cv2_to_imgmsg(frame, "bgr8"))
            # Further processing and video writing logic...

    def custom_cleanup(self):
        if self.video_writer:
            self.video_writer.release()
        self.cap.release()
        self.get_logger().info('Camera cleanup ran')

def main(args=None):
    rclpy.init(args=args)
    camera_node = USB_Camera()
    signal.signal(signal.SIGINT, lambda sig, frame: camera_node.custom_cleanup())
    rclpy.spin(camera_node)
    camera_node.custom_cleanup()
    rclpy.shutdown()
    camera_node.destroy_node()

if __name__ == '__main__':
    main()
