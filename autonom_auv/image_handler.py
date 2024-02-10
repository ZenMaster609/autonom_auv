from autonom_auv.autonom_auv.pipeline_image_methods import ImageMethods
import cv2

# class ImageHandler:



#     def cam2_callback(self, data):
#             photos_path = os.path.join(get_package_share_directory('autonom_auv'), 'photos', f"cam2_{time.time()}.jpg")
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#             image_edit =cv_image.copy()
#             dimensions = cv_image.shape
#             self.pub_image2 = image_edit
#             if self.save_images:
#                 self.imgcount += 1
#                 if self.imgcount > 7:
#                     self.imgcount = 0
#                     cv2.imwrite(photos_path, cv_image)
#             cv2.imshow("window", image_edit)
#             cv2.waitKey(1)
