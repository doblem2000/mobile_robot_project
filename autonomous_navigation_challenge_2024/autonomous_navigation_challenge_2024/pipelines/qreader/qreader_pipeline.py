from qreader import QReader
from cv_bridge import CvBridge
import cv2
import numpy as np
from autonomous_navigation_challenge_2024.pipelines.pipeline import Pipeline
import json
import os
from autonomous_navigation_challenge_2024.distance_estimator import DistanceEstimator

class QReaderPipeline(Pipeline):
    def __init__(self):
        self.qreader = QReader(model_size='n', min_confidence=0.3)
        
        with open("src/mobile-robots-project-work/autonomous_navigation_challenge_2024/autonomous_navigation_challenge_2024/config/camera_parameter.json", 'r') as f:
            self.config = json.load(f)
            
        real_qr_dimension_height=self.config['real_qr_dimension_height']
        real_qr_dimension_width=self.config['real_qr_dimension_width']

        vfov = np.deg2rad(self.config['vfov'])
        hfov = np.deg2rad(self.config['hfov'])
        
        self._distance_estimator = DistanceEstimator(hfov, vfov, real_qr_dimension_height, real_qr_dimension_width, 1774, 1440)
    
    def draw_informations(self, cv_image, text, bbox, confidence, distance_height, distance_width, distance):
        cv_image = cv2.polylines(cv_image, [bbox], isClosed=True, color=(0, 255, 0), thickness=2)
        x1, y1 = bbox[0]
        y2 = bbox[3, 1]  # y-coordinate del lato inferiore del bounding box  
        cv_image = cv2.putText(cv_image, f"Distance Height: {round(distance_height,3)}", (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
        cv_image = cv2.putText(cv_image, f"Distance Width: {round(distance_width,3)}", (x1, y2 + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
        cv_image = cv2.putText(cv_image, f"Distance: {round(distance,3)}", (x1, y2 + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
        cv_image = cv2.putText(cv_image, f"Confidence: {round(confidence,3)}", (x1, y2 + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
        if text is not None:
            cv_image = cv2.putText(cv_image, text, (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        return cv_image
            
    def process_image(self, cv_image, draw_results=False):
        detections = self.qreader.detect_and_decode(cv_image, return_detections=True)
        detections = zip(*detections)
        detection_result = []
        detected = False
        for text, metadata in detections:
            dict_result={}
            bbox = metadata["bbox_xyxy"].astype(int)
            confidence = metadata["confidence"]
            top_left = [bbox[0], bbox[1]]
            top_right = [bbox[2], bbox[1]]
            bottom_right = [bbox[2], bbox[3]]
            bottom_left = [bbox[0], bbox[3]]
            bbox = np.stack([top_left, top_right, bottom_right, bottom_left])
            
            distance_height, distance_width, distance = self._distance_estimator.estimate_distance2(bbox)
            
            dict_result['text'] = text
            dict_result['bbox'] = bbox
            dict_result['confidence'] = confidence
            dict_result['distance_height'] = distance_height
            dict_result['distance_width'] = distance_width
            dict_result['distance'] = distance
            
            detection_result.append(dict_result)
            detected = True
            
            if draw_results:
                cv_image = self.draw_informations(cv_image, text, bbox, confidence, distance_height, distance_width, distance)
        return detected, cv_image, detection_result