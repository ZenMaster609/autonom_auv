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
        self.image_count = 0
        self.bench_distance = None
        self.valves = None
        self.mode = None
        self.orientation = None

    def find_bench(self):
        hsv_lower = [0, 0, 0]
        hsv_upper = [40, 200, 170]
        image_edit = self.feed_image.copy()
        hsv_image = ImageMethods.color_filter(image_edit , hsv_lower, hsv_upper)
        show_hsv = ImageMethods.fix_hsv(hsv_image)
        boxes, closed_image = ImageMethods.make_boxes2(hsv_image, image_edit, 1, False)
        show_closed = ImageMethods.fix_hsv(closed_image)
        biggest_box = ImageMethods.find_biggest_box(image_edit, boxes, True)
        center_image, cx, cy = ImageMethods.Draw_Center(image_edit, biggest_box)
        stacked = ImageMethods.stack_images([show_hsv, show_closed, image_edit, center_image])
        ImageMethods.showImage(stacked)
        














