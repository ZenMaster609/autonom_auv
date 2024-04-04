import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class USB_Camera(Node):
    def __init__(self):
         super().__init__('usb_camera_node') 
         self.publisher = self.create_publisher(Image, 'usb_camera', 10)
         self.timer = self.create_timer(0.1, self.timer_callback)
         self.cap = cv2.VideoCapture(0)
         self.cv_bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret == True:
            self.publisher.publish(self.cv_bridge.cv2_to_imgmsg(frame,"bgr8"))


def main(args=None):
    rclpy.init(args=args)
    Camera =  USB_Camera()
    rclpy.spin(Camera)
    rclpy.shutdown()
    Camera.destroy_node()

if __name__ == '__main__':
    main()
    
