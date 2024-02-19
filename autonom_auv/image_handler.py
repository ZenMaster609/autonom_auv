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

class ImageHandler:
    def __init__(self):
        self.hsv_range_bib = {
            "pipeline_sim" : [[30,114,114],[30,255,255]],
            "visual_short_distance" : [[0, 0, 30],[86, 0, 120]],
            "visual_long_distance" : [[0, 0, 0],[40, 200, 170]],
        }
        self.feed_image = None
        self.show_hsv = None
        self.cooldown = 0
        self.Ids_list= []   
        self.aruco_printed = 0 
        self.bench_box_image = None
        

        


    def find_bench(self):
        image_edit = self.feed_image.copy()
        hsv_range = self.hsv_range_bib["visual_long_distance"]
        hsv_image = ImageMethods.color_filter(image_edit , hsv_range)
        self.show_hsv = ImageMethods.fix_hsv(hsv_image)
        boxes = ImageMethods.find_boxes(hsv_image, image_edit, 500, False)
        bench = ImageMethods.find_biggest_box(image_edit, boxes, True) 
        self.bench_box_image = image_edit
        stacked = ImageMethods.stack_images([self.show_hsv,image_edit])
        #ImageMethods.showImage(stacked)
        return bench


    def find_bench_info(self):
        bench = self.find_bench()
        yaw_offset, cx, cy = self.find_box_info(bench, self.bench_box_image, 180, True)
        positions, area = ImageMethods.get_box_info(bench)
        size = 8000000/area
        middle_left = positions["middle_left"]
        middle_right = positions["middle_right"]

        

        
        stacked = ImageMethods.stack_images([self.show_hsv, self.bench_box_image])
        ImageMethods.showImage(stacked)
        return yaw_offset, size, middle_left, middle_right


    def find_box(self,cv_image,image_edit,hsv_range_name,min_box_area,draw:bool):
        dimensions = cv_image.shape 
        hsv_range = self.hsv_range_bib[hsv_range_name]
        mask = ImageMethods.color_filter(cv_image,hsv_range)
        mask = cv2.line(mask,(0,600),(dimensions[1],600),(0,0,0),10)     
        box_list = ImageMethods.find_boxes(mask, image_edit, min_box_area, draw)
        the_box = ImageMethods.find_the_box(box_list)
        return the_box
    

    def find_box_info(self,the_box,image_edit, angel_offset,draw:bool):
        angle_deg = ImageMethods.find_angle_box(the_box,angel_offset)
        angle_deg, self.cooldown = ImageMethods.angel_cooldown(angle_deg,self.cooldown)
        center_x,center_y = ImageMethods.find_Center(image_edit,the_box, draw)
        cv2.putText(image_edit, f"{int(angle_deg)}",[1600,1050], cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 0, 255), 2, cv2.LINE_AA)
        return angle_deg,center_x,center_y
    

    def aruco_handler(self,cv_image,image_edit,the_box):
        #reads AruCo codes and print them if there noe more pipeline
        self.Ids_list= ImageMethods.read_AruCo(cv_image,image_edit,self.Ids_list)
        ids = ImageMethods.filtered_ids_list(self.Ids_list)
        return ids


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