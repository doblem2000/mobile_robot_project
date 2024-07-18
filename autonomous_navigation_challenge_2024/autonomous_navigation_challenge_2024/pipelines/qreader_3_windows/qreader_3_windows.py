import cv2
import numpy as np
from qreader import QReader
import json
import os
from autonomous_navigation_challenge_2024.distance_estimator import DistanceEstimator

from autonomous_navigation_challenge_2024.pipelines.pipeline import Pipeline

class QReader3WsPipeline(Pipeline):
    def __init__(self):
        self.qreader = QReader(model_size='n', min_confidence=0.3)

        with open("src/mobile-robots-project-work/autonomous_navigation_challenge_2024/autonomous_navigation_challenge_2024/config/camera_parameter.json", 'r') as f:
            self.config = json.load(f)
        
        real_qr_dimension_height=self.config['real_qr_dimension_height']
        real_qr_dimension_width=self.config['real_qr_dimension_width']

        vfov = np.deg2rad(self.config['vfov'])
        hfov = np.deg2rad(self.config['hfov'])
        
        self._distance_estimator = DistanceEstimator(hfov, vfov, real_qr_dimension_height, real_qr_dimension_width, 1774, 1440)


    def calculate_area(self, rect):
        #rect = ((x1, y1), (x2, y2))
        (x1,y1) = rect[0]
        (x2,y2) = rect[1]

        # Calcola l'area del rettangolo: area = (larghezza) * (altezza)
        return (x2 - x1) * (y2 - y1)


    def are_rectangles_overlapping(self, rect1, rect2):
        # rect: ((x1, y1), (x2, y2))

        overlap_x = not (rect1[1][0] < rect2[0][0] or rect2[1][0] < rect1[0][0])
        overlap_y = not (rect1[1][1] < rect2[0][1] or rect2[1][1] < rect1[0][1])
        return overlap_x and overlap_y



    def find_largest_overlapping_rectangles(self, rectangles):
        # Lista per memorizzare i rettangoli da tenere
        rectangles_unless_overlap = []

        # Insieme per tenere traccia degli indici dei rettangoli da eliminare
        to_remove = set()

        # Trova i rettangoli sovrapposti e calcola le loro aree
        for i in range(len(rectangles)):
            for j in range(i + 1, len(rectangles)):
                rect1 = rectangles[i]
                rect2 = rectangles[j]

                # Verifica se i rettangoli si sovrappongono
                if self.are_rectangles_overlapping(rect1, rect2):
                    #print(f"indices: {i,j}")
                    #print(f"rectagnles: {rect1,rect2}")
                    # Calcola le aree dei rettangoli sovrapposti
                    area1 = self.calculate_area(rect1)
                    area2 = self.calculate_area(rect2)

                    # Determina quale rettangolo eliminare
                    if area1 < area2:
                        #print(f'elimino il rettangolo {i}')
                        to_remove.add(i)
                    else:
                        #print(f'elimino il rettangolo {j}')
                        to_remove.add(j)

        # Aggiungi solo i rettangoli che non sono stati segnati per l'eliminazione
        for i in range(len(rectangles)):
            if i not in to_remove:
                rectangles_unless_overlap.append(rectangles[i])

        return rectangles_unless_overlap

    

    def draw_informations(self, cv_image, text, bbox, confidence, distance_height, distance_width, distance):
        cv_image = cv2.polylines(cv_image, [bbox], isClosed=True, color=(0, 255, 0), thickness=2)
        x1, y1 = bbox[0]
        y2 = bbox[3, 1]  # y-coordinate del lato inferiore del bounding box
        cv_image = cv2.putText(cv_image, f"Distance Height: {round(distance_height,3)}", (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
        cv_image = cv2.putText(cv_image, f"Distance Width: {round(distance_width,3)}", (x1, y2 + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
        cv_image = cv2.putText(cv_image, f"Distance: {round(distance,3)}", (x1, y2 + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
        cv_image = cv2.putText(cv_image, f"Confidence: {round(confidence,3)}", (x1, y2 + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
        if text is not None:
            print(f"text: {text}")
            cv_image = cv2.putText(cv_image, text, (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        return cv_image

    def process_image(self, cv_image, draw_results=False):
        height, width = cv_image.shape[:2]  # Ottieni altezza e larghezza dell'immagine

        # Definisci le dimensioni delle finestre
        window_width = height  # Larghezza della finestra

        # Calcola le coordinate x per ciascuna finestra
        x_left = 0  # Estremo sinistro
        x_center = (width - window_width) // 2  # Centro
        x_right = width - window_width  # Estremo destro

        # Estrai le tre finestre dall'immagine
        window_left = cv_image[:, x_left:x_left + window_width]
        window_center = cv_image[:, x_center:x_center + window_width]
        window_right = cv_image[:, x_right:x_right + window_width]

        windows = [window_left, window_center, window_right]

        bboxes = []
        detection_res_windows = []


        for window, x in zip(windows, [x_left, x_center, x_right]):

            texts_windows, metadatas_windows = self.qreader.detect_and_decode(window, return_detections=True)

            if len(texts_windows) == 0:
                continue

            for text_window, detection_data_window in zip(texts_windows, metadatas_windows):
                bbox = detection_data_window["bbox_xyxy"]
                confidence = detection_data_window["confidence"]
                x1, y1, x2, y2 = bbox.astype(int)
                
                # Translate bbox to the original image coordinates
                x1 += x
                y1 += 0  # Y rimane invariato poiché rappresenta il punto in alto a sinistra della finestra
                x2 += x
                y2 += 0  # Y rimane invariato

                top_left = [x1, y1]
                top_right = [x2, y1]
                bottom_right = [x2, y2]
                bottom_left = [x1, y2]
                bbox = np.stack([top_left, top_right, bottom_right, bottom_left])

                distance_height, distance_width, distance = self._distance_estimator.estimate_distance2(bbox)

                detection_res_windows.append((text_window,bbox,confidence,distance_height, distance_width, distance))
                bboxes.append(((x1, y1), (x2, y2)))


        detections = []
        detected = False
        filtered_bboxes = self.find_largest_overlapping_rectangles(bboxes)
        for idx, det_res in enumerate(detection_res_windows):
            if bboxes[idx] in filtered_bboxes:
                dict_result={}
                dict_result['text'] = det_res[0]
                dict_result['bbox'] = det_res[1]
                dict_result['confidence'] = det_res[2]
                dict_result['distance_height'] = det_res[3]
                dict_result['distance_width'] = det_res[4]
                dict_result['distance'] = det_res[5]
                detected = True

                detections.append(dict_result)
                if draw_results:
                    cv_image = self.draw_informations(cv_image, dict_result['text'], dict_result['bbox'], dict_result['confidence'], dict_result['distance_height'], dict_result['distance_width'] , dict_result['distance'])
        return detected, cv_image, detections