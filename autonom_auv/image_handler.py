try:
    from .image_methods import ImageMethods  # Attempt relative import for package context
except ImportError:
    from image_methods import ImageMethods  # Fallback to direct import for standalone execution

import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import time


class ImageHandler:
    def __init__(self, save_freq):
        self.save_freq = save_freq
        self.feed_image = None
        self.hsv_image = None
        

    def find_bench(self, close:bool):
        if close:
            hsv_lower = [0, 0, 0] #long distance
            hsv_upper = [40, 200, 170] #long distance
        else:
            hsv_lower = [0, 0, 30] #short distance
            hsv_upper = [86, 0, 120] #short distance
        image_edit = self.feed_image.copy()
        hsv_image = ImageMethods.color_filter(image_edit , hsv_lower, hsv_upper)
        show_hsv = ImageMethods.fix_hsv(hsv_image)
        boxes = ImageMethods.find_boxes(hsv_image, image_edit, 500, True)
        bench = ImageMethods.find_biggest_box(image_edit, boxes, True) 
        stacked = ImageMethods.stack_images([show_hsv,image_edit])
        ImageMethods.showImage(stacked)
        return bench


    def find_bench_info():
        image_edit = self.feed_image.copy()
        bench = self.find_bench(True)
        corners, area = ImageMethods.get_box_info(bench)
        #bruk vektorer for å regne ut tilt i grader/radianer
        
        #if tilt ok, gå til venstre linje
        #gå til høyre linje mens du scanner for koder







