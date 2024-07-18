import numpy as np
from ultralytics import YOLO
from time import time
import os

from autonomous_navigation_challenge_2024.pipelines.pipeline import Pipeline

class YOLO1Pipeline(Pipeline):
    def __init__(self):
        self.model_path = os.path.join(os.path.dirname(__file__), 'best2.pt')
        self.classes = ['Speed limit (20km/h)',
                'Speed limit (30km/h)',
                'Speed limit (50km/h)',
                'Speed limit (60km/h)',
                'Speed limit (70km/h)',
                'Speed limit (80km/h)',
                'End of speed limit (80km/h)',
                'Speed limit (100km/h)',
                'Speed limit (120km/h)',
                'No Overtaking',
                'No Overtaking for Heavy Vehicles',
                'Right-of-Way at next Intersection',
                'Priority Road',
                'Yield',
                'Stop',
                'No Vehicles',
                'Heavy Vehicles Prohibited',
                'No Entry',
                'General Caution',
                'Dangerous Left Curve',
                'Dangerous Right Curve',
                'Double Curve',
                'Bumpy Road',
                'Slippery Road',
                'Narrowing Road',
                'Road Work',
                'Traffic Signals',
                'Pedestrian',
                'Children',
                'Bike',
                'Snow',
                'Deer',
                'End of Limits',
                'Turn Right Ahead',
                'Turn Left Ahead',
                'Ahead Only',
                'Go Straight or Right',
                'Go Straight or Left',
                'Keep Right',
                'Keep Left',
                'Roundabout Mandatory',
                'End of No Overtaking',
                'End of No Overtaking for Heavy Vehicles']
        self.import_model()

    def import_model(self):
        t1 = time()
        self.detector = YOLO(self.model_path)
        t2 = time()
        print(f"Loaded YOLO model. Took {t2 - t1} seconds.")
    
    def getClassName(self, classIndex):
        return self.classes[classIndex]
    
    def process_image(self, image, draw_results = False):
        img = np.asarray(image)
        results = self.detector.predict(source=img, verbose=True, imgsz=1792)[0]
        boxes = results.boxes.xyxy.cpu().numpy()  # Bounding boxes
        confidences = results.boxes.conf.cpu().numpy()  # Confidences
        class_ids = results.boxes.cls.cpu().numpy().astype(int)  # Class IDs
        detections = []
        for i in range(len(boxes)):
            x1, y1, x2, y2 = map(int, boxes[i])
            top_left = (x1, y1)
            top_right = (x2, y1)
            bottom_left = (x1, y2)
            bottom_right = (x2, y2)
            label = self.getClassName(class_ids[i])
            confidence = confidences[i]
            detections.append((label, (top_left, top_right, bottom_right, bottom_left)))
        if draw_results:
            img = results.plot()
        if len(detections) > 0:
            return True, img, detections
        return False, img, None