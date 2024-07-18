import numpy as np
import math

class DistanceEstimator:
    def __init__(self, hfov, vfov, real_qr_height, real_qr_width, img_width, img_height):
        self.hfov = math.radians(hfov)
        self.vfov = math.radians(vfov)
        self.real_qr_height = real_qr_height
        self.real_qr_width = real_qr_width
        self.img_width = img_width
        self.img_height = img_height

        self.F = 1488.7
        
    
    def estimate_distance(self, bbox):
        top_left = bbox[0]
        top_right = bbox[1]
        bottom_left = bbox[3]
        bottom_right = bbox[2]
        bbox_width_high = abs(top_right[0] - top_left[0])
        bbox_width_low = abs(bottom_right[0] - bottom_left[0])
        bbox_width = (bbox_width_high + bbox_width_low) / 2
        bbox_height_left = abs(bottom_left[1] - top_left[1])
        bbox_height_right = abs(bottom_right[1] - top_right[1])
        bbox_height= (bbox_height_left + bbox_height_right) / 2
        distance_height = (self.real_qr_height * self.img_height) / (2 * bbox_height * np.tan(self.vfov/2))
        distance_width = (self.real_qr_width * self.img_width) / (2 * bbox_width * np.tan(self.hfov /2))
        distance = (distance_height + distance_width) / 2
        return distance_height, distance_width, distance
    
    def estimate_distance2(self, bbox):
        top_left = bbox[0]
        top_right = bbox[1]
        bottom_left = bbox[3]
        bottom_right = bbox[2]
        bbox_width_high = abs(top_right[0] - top_left[0])
        bbox_width_low = abs(bottom_right[0] - bottom_left[0])
        bbox_width = (bbox_width_high + bbox_width_low) / 2
        bbox_height_left = abs(bottom_left[1] - top_left[1])
        bbox_height_right = abs(bottom_right[1] - top_right[1])
        bbox_height= (bbox_height_left + bbox_height_right) / 2
        distance_height = self.F * self.real_qr_height / bbox_height
        distance_width = self.F * self.real_qr_width / bbox_width
        distance = (distance_height + distance_width) / 2
        return distance_height, distance_width, distance