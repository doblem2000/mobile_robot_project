import cv2

from autonomous_navigation_challenge_2024.pipelines.pipeline import Pipeline

class BaselinePipeline(Pipeline):
    def __init__(self):
        self.qr_detector = cv2.QRCodeDetector()
    
    def display_detections(self, cv_image, detections):
        for info, bb in detections:
            pt1 = (int(bb[0][0]), int(bb[0][1]))
            pt2 = (int(bb[1][0]), int(bb[1][1]))
            pt3 = (int(bb[2][0]), int(bb[2][1]))
            pt4 = (int(bb[3][0]), int(bb[3][1]))
            cv_image = cv2.line(cv_image, pt1, pt2, (0, 0, 255), 10)
            cv_image = cv2.line(cv_image, pt2, pt3, (0, 0, 255), 10)
            cv_image = cv2.line(cv_image, pt3, pt4, (0, 0, 255), 10)
            cv_image = cv2.line(cv_image, pt4, pt1, (0, 0, 255), 10)
            cv_image = cv2.putText(cv_image, info, pt1, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        return cv_image
    
    def process_image(self, cv_image, draw_results = False):
        retval, decoded_info, points, straight_qrcode = self.qr_detector.detectAndDecodeMulti(cv_image)
        detections = []
        if retval and draw_results:
            detections = list(zip(decoded_info, points))
            detections = [{'text': text, 'bbox': bb, 'confidence': None, 'distance_height': None, 'distance_width': None, 'distance': 0.0} for text, bb in detections]
            cv_image = self.display_detections(cv_image, zip(decoded_info, points))
        return True if retval else False, cv_image, detections