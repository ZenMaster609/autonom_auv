import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import signal


class USB_Camera(Node):
    def __init__(self):
         super().__init__('usb_camera_node') 
         self.publisher = self.create_publisher(Image, 'usb_camera', 10)
         self.timer = self.create_timer(0.1, self.timer_callback)
         self.cap = cv2.VideoCapture(0)
         self.cv_bridge = CvBridge()
         self.calibrate = False
         self.record = False
         if self.calibrate:
            self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            self.chessboard_size = (7, 9)  # 7 corners in width, 6 in height
            # Prepare object points based on the chessboard size
            self.objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
            self.objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
            # Arrays to store object points and image points from all the images.
            self.objpoints = [] # 3d point in real world space
            self.imgpoints = [] # 2d points in image plane.
         self.video_writer = None

    def custom_cleanup(self):
            if self.video_writer:
                self.video_writer.release()
            if self.calibrate:
                self.get_logger().info(f"{self.mtx}")
                self.get_logger().info(f"{self.dist}")

    def impliment_matrix(self,camera_port): 
        ret, frame = self.cap.read(camera_port)

        if ret == True:
            if self.calibrate:
                self.gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # Find the chess board corners
                ret2, corners = cv2.findChessboardCorners(self.gray, (7,9), None)
                # If found, add object points, image points (after refining them)
                if ret2 == True:
                    self.objpoints.append(self.objp)
                    corners2 = cv2.cornerSubPix(self.gray,corners, (11,11), (-1,-1), self.criteria)
                    self.imgpoints.append(corners2)
                    # Draw and display the corners
                    cv2.drawChessboardCorners(frame, (7,9), corners2, ret)
                    ret, self.mtx, self.dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, self.gray.shape[::-1], None, None)

            mtx = np.array(((426.17685011,   0,         320.38487224),
             (0,          429.76884458, 225.27715136),
             (0,           0,           1       ),))
            dist = np.array(((-0.35653578,  0.11731714,  0.01052246,  0.00376304, -0.01392377),))

            img = frame.copy()
            h, w = frame.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
            mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
            dst1 = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

            # crop the image
            x, y, w, h = roi
            dst = dst1[y:y+h, x:x+w]
            height = max(img.shape[0], dst.shape[0])
            width_img = int(img.shape[1] * (height / img.shape[0]))
            width_dst = int(dst.shape[1] * (height / dst.shape[0]))
            img_resized = cv2.resize(img, (width_img, height))
            dst_resized = cv2.resize(dst, (width_dst, height))
            
            return dst_resized

    def timer_callback(self):
            
            dst_image = self.impliment_matrix(0)
            self.publisher.publish(
                self.cv_bridge.cv2_to_imgmsg(dst_image,"bgr8"))

            if self.record:
                # Write frame to video file
                if self.video_writer is None:
                    # Define the codec and create VideoWriter object
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    self.video_writer = cv2.VideoWriter('output.mp4', fourcc, 20.0, (dst_resized.shape[1], dst_resized.shape[0]))
                # Write the frame
                self.video_writer.write(dst_image)


def main(args=None):
    rclpy.init(args=args)
    Camera =  USB_Camera()
    signal.signal(signal.SIGINT, lambda sig, frame: Camera.custom_cleanup())
    rclpy.spin(Camera)
    Camera.custom_cleanup()  # Ensure cleanup is called if exit wasn't due to SIGINT
    rclpy.shutdown()
    Camera.destroy_node()

if __name__ == '__main__':
    main()
    
