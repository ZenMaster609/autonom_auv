try:
    from .image_methods import ImageMethods  # Attempt relative import for package context
except ImportError:
    from image_methods import ImageMethods  # Fallback to direct import for standalone execution

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
import time
import subprocess
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import cv2.aruco as aruco


class ImageHandler:
    def __init__(self):
        self.hsv_range_bib = {
            "pipeline_sim" : [[30,114,114],[30,255,255]],
            "visual_short_distance" : [[0, 0, 30],[86, 0, 120]],
            "visual_long_distance" : [[0, 0, 0],[40, 200, 170]],
            "pink_pipeline" : [[126, 45, 87],[179, 255, 255]]
        }
        self.feed_image = None
        self.feed_image2 = None
        self.show_hsv = None
        self.cooldown = 0
        self.Id_list= []   
        self.aruco_printed = 0 
        self.bench_box_image = None
        self.scale_factor = 1
        
    def show_image(self, double):
        if not double:
            ImageMethods.showImage(self.feed_image)
        else:
            showImage = ImageMethods.stack_images([self.feed_image,self.feed_image2])
            ImageMethods.showImage(showImage)
        
    def find_bench(self, front, mode):
        image_edit = ImageMethods.scale_image(self.feed_image.copy(), scale_factor=self.scale_factor)
        image_edit2 = ImageMethods.scale_image(self.feed_image2.copy(), scale_factor=self.scale_factor)
        self.aruco_handler(image_edit, image_edit2)
        self.dims = image_edit.shape
        hsv_range = self.hsv_range_bib["visual_long_distance"]
        hsv_image = ImageMethods.color_filter(image_edit , hsv_range)
        try:
            boxes = ImageMethods.find_boxes(hsv_image, image_edit, 500, False)
            bench = ImageMethods.find_biggest_box(image_edit, boxes, True) 
            positions, area = ImageMethods.get_box_info(bench)
            angle = ImageMethods.find_angle_box(bench,180, self.dims[1])
            if front:size = (self.scale_factor**2)*8000000/area
            else: size = (self.scale_factor**2)*14000000/area
        except Exception as e:_ = e
        cv2.putText(image_edit, f"Mode:{mode}",[700,525], cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2, cv2.LINE_AA)
        showImage = ImageMethods.stack_images([image_edit,image_edit2])
        ImageMethods.showImage(showImage)
        return size, positions, angle
        

    def find_pipeline(self,min_box_sice):
        image_edit = ImageMethods.scale_image(self.feed_image.copy(), scale_factor=self.scale_factor)
        self.dims = image_edit.shape
        hsv_range = self.hsv_range_bib["pink_pipeline"]
        hsv_image = ImageMethods.color_filter(image_edit , hsv_range)
        self.aruco_handler(image_edit)
        try:
            cv2.line(hsv_image,(0,int(self.dims[0]/2)),(self.dims[1],int(self.dims[0]/2)),(0,0,0),10)     
            box_list = ImageMethods.find_boxes(hsv_image, image_edit, (self.scale_factor**2)*min_box_sice, True)
            highest_box = ImageMethods.find_highest_box(box_list)
            if highest_box is None:done = True
            else:done=False
            angle_deg = ImageMethods.find_angle_box(highest_box,90, self.dims[1])
            angle_deg, self.cooldown = ImageMethods.angle_cooldown(angle_deg,self.cooldown)
            center_x,center_y = ImageMethods.find_Center(image_edit,highest_box, True)
        except Exception as e:_ = e
        cv2.putText(image_edit, f"{int(angle_deg)}",[800,525], cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 0, 255), 2, cv2.LINE_AA)
        ImageMethods.showImage(image_edit)
        return angle_deg,center_x, done,image_edit
  



    def aruco_handler(self,image1, image2=None):
        self.Id_list= ImageMethods.read_AruCo(image1,self.Id_list)
        if image2 is not None:
            self.Id_list = ImageMethods.read_AruCo(image2,self.Id_list)

    def filter_arucos(self):
        filtered_list = []
        filtered_list = ImageMethods.filtered_ids_list(self.Id_list)
        return filtered_list 

    


class logging_data: 
    def __init__(self): 
        self.time = []
        self.data1 = []
        self.data2 = []
        self.data3 = []
        self.data4 = []
        self.start_time = time.time()
    
    def log_data(self,value1=None,value2=None,value3=None,value4=None):
        time_now = time.time()-self.start_time
        time_now = round(time_now,3)
        self.time.append(time_now)
        value1 = round(value1,3)
        self.data1.append(value1)
        if value2 is not None:
            value2 = round(value2,3)
            self.data2.append(value2)
        if value3 is not None:
            value3 = round(value3,3)
            self.data3.append(value3)
        if value4 is not None:self.data4.append(value4)



    def plot_data(self): 
        fig = make_subplots(rows=2, cols=2)
        fig.add_trace(go.Scatter(x=self.time, y=self.data1),row=1, col=1)
        if self.data2 is not None:
            fig.add_trace(go.Scatter(x=self.time, y=self.data2),row=1, col=2)
        if self.data3 is not None:
            fig.add_trace(go.Scatter(x=self.time, y=self.data3),row=2, col=1)
        if self.data4 is not None:
            fig.add_trace(go.Scatter(x=self.time, y=self.data4),row=2, col=2)
        fig.write_html("plot.html")
        image_path = "plot.html"
        subprocess.run(['xdg-open', image_path], check=True)

    def plot_data_table(self,colum1, colum2,colum3=[],plot_names=["","","",""]):
        fig = make_subplots(
            rows=2, cols=2,
            specs=[[{"type": "table"}, {"type": "scatter"}],
                [{"type": "scatter"}, {"type": "scatter"}]],
        subplot_titles=(plot_names[0],plot_names[1],plot_names[2],plot_names[3]))
        fig.add_trace(go.Table(header=dict(values=['Name', 'Value',"AruCo Codes"]),
                                cells=dict(values=[colum1, colum2,colum3])),
                                row=1, col=1)
        fig.add_trace(go.Scatter(x=self.time, y=self.data1),row=1, col=2)
        if self.data2 is not None:
            fig.add_trace(go.Scatter(x=self.time, y=self.data2),row=2, col=1)
        if self.data3 is not None:
            fig.add_trace(go.Scatter(x=self.time, y=self.data3),row=2, col=2)
        fig.write_html("plot.html")
        image_path = "plot.html"
        subprocess.run(['xdg-open', image_path], check=True)
