from qreader import QReader
from patched_yolo_infer import MakeCropsDetectThem, CombineDetections
from patched_yolo_infer import visualize_results
from cv_bridge import CvBridge
import cv2
import numpy as np
from autonomous_navigation_challenge_2024.pipelines.pipeline import Pipeline
from autonomous_navigation_challenge_2024.pipelines.qreader_boost.from_patch_to_det import FromPatchToDet
import json
import os
from autonomous_navigation_challenge_2024.distance_estimator import DistanceEstimator

class QReaderBoostPipeline(Pipeline):
    def __init__(self):
        self.qreader = QReader(model_size='n', min_confidence=0.3)
        
        with open("src/mobile-robots-project-work/autonomous_navigation_challenge_2024/autonomous_navigation_challenge_2024/config/camera_parameter.json", 'r') as f:
            self.config = json.load(f)
            
        real_qr_dimension_height=self.config['real_qr_dimension_height']
        real_qr_dimension_width=self.config['real_qr_dimension_width']

        vfov = np.deg2rad(self.config['vfov'])
        hfov = np.deg2rad(self.config['hfov'])
        
        self._distance_estimator = DistanceEstimator(hfov, vfov, real_qr_dimension_height, real_qr_dimension_width, 1774, 1440)

        self.from_patch_to_det = FromPatchToDet()
    
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
        element_crops = MakeCropsDetectThem(
            image=cv_image,
            model_path="src/mobile-robots-project-work/autonomous_navigation_challenge_2024/autonomous_navigation_challenge_2024/models/qrdet-n.pt",
            segment=False,
            shape_x=640,
            shape_y=640,
            overlap_x=50,
            overlap_y=50,
            #show_crops=True,
            conf=0.3,
            iou=0.7,
            resize_initial_size=True,
        )

        result = CombineDetections(element_crops, nms_threshold=0.25, match_metric='IOU')  
        detections = self.from_patch_to_det.convert_result(cv_image, result)
        decoded_qrs = tuple(self.qreader.decode(image=cv_image, detection_result=detection) for detection in detections)


        detection_result = []
        detected = False
        for text, metadata in zip(decoded_qrs, detections):
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