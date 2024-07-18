import warnings
warnings.filterwarnings('ignore')
import numpy as np
import os
import tensorflow as tf
import glob as glob
import cv2

from autonomous_navigation_challenge_2024.pipelines.pipeline import Pipeline

class FRCNNPipeline(Pipeline):
    def __init__(self):
        self.model_path = os.path.join(os.path.dirname(__file__), 'faster_rcnn_inception_v2')
        self.chkpt_path = os.path.join(self.model_path,'inference_graph/frozen_inference_graph.pb')
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.compat.v1.GraphDef() 
            with tf.compat.v2.io.gfile.GFile(self.chkpt_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            self.sess = tf.compat.v1.Session(graph=self.detection_graph)
    
    def process_image(self, image, draw_results = False):
        img = np.asarray(image)
        image_np_expanded = np.expand_dims(img, axis=0)
        with self.detection_graph.as_default():
            with self.sess.as_default():
                image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
                (boxes, scores, classes, num_detections) = self.sess.run(
                    [boxes, scores, classes, num_detections],
                    feed_dict={image_tensor: image_np_expanded})
        detections = []
        for box, score in zip(boxes[0], scores[0]):
            if score < 0.1:
                break
            pt1 = (int(box[1]*img.shape[1]), int(box[0]*img.shape[0]))
            pt2 = (int(box[3]*img.shape[1]), int(box[2]*img.shape[0]))
            top_left = pt1
            top_right = (pt2[0], pt1[1])
            bottom_right = pt2
            bottom_left = (pt1[0], pt2[1])
            detections.append(('', (top_left, top_right, bottom_right, bottom_left)))
            if draw_results:
                img=cv2.rectangle(img, pt1, pt2, (0, 255, 0), 3)
        if len(num_detections) > 0:
            return True, img, detections
        return False, img, None