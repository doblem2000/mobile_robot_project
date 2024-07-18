import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
from autonomous_navigation_challenge_2024_msgs.msg import PerceptedSignpostList, PerceptedSignpost
from rclpy.qos import qos_profile_sensor_data
import importlib

#######################################################
import numpy as np
SHOW = True
DEBUG = False

direction_dict = {1:'forward and slightly to the left',2:'forward',3:'forward and slightly to the right',4:'forward to the left',5:'forward',6:'forward to the right',7:'left',8:'forward',9:'right'}

def is_image_sharp(image, threshold=100):
    # Converti l'immagine in scala di grigi
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Calcola la varianza dei pixel nell'immagine
    variance = cv2.Laplacian(gray, cv2.CV_64F).var()
    
    # Verifica se la varianza supera la soglia per considerare l'immagine a fuoco
    return variance > threshold
#######################################################

PIPELINES = {
    "baseline": ("autonomous_navigation_challenge_2024.pipelines.baseline.baseline_pipeline", "BaselinePipeline"),
    "pyzbar": ("autonomous_navigation_challenge_2024.pipelines.pyzbar.pyzbar_pipeline", "PyzbarPipeline"),
    "qreader": ("autonomous_navigation_challenge_2024.pipelines.qreader.qreader_pipeline", "QReaderPipeline"),
    "qreader_3_windows": ("autonomous_navigation_challenge_2024.pipelines.qreader_3_windows.qreader_3_windows", "QReader3WsPipeline"),
    "qreader_sliding_window": ("autonomous_navigation_challenge_2024.pipelines.qreader_sliding_window.qreader_sw_pipeline", "QReaderSWPipeline"),
    "qreader_boost": ("autonomous_navigation_challenge_2024.pipelines.qreader_boost.qreader_boost_pipeline", "QReaderBoostPipeline"),
    "cv2_sliding_window": ("autonomous_navigation_challenge_2024.pipelines.cv2_sliding_window.cv2_sw_pipeline", "CV2SWPipeline"),
    "yolo1": ("autonomous_navigation_challenge_2024.pipelines.yolo1.yolo1_pipeline", "YOLO1Pipeline"),
    "frcnn": ("autonomous_navigation_challenge_2024.pipelines.frcnn.frcnn_pipeline", "FRCNNPipeline"),
}


class TestDetector(Node):
    def __init__(self):
        super().__init__("test_detector")  # Init node
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.get_logger().info("Detector test node initialized")

        # ROS2 parameters
        self.declare_parameter("topic", "/oakd/rgb/preview/image_raw/compressed")
        topic = self.get_parameter("topic").value
        self.declare_parameter("pipeline", "qreader_3_windows")
        self.pipeline = self.get_parameter("pipeline").value
        if self.pipeline not in PIPELINES:
            self.get_logger().error(f"Invalid pipeline: {self.pipeline}")
            self.destroy_node()
            exit()
        self.declare_parameter("debug", True)
        self.debug = self.get_parameter("debug").value
        

        self.bridge = CvBridge()
        
        try:
            pipeline_module = importlib.import_module(PIPELINES[self.pipeline][0])
            self.pipeline_class = pipeline_module.__getattribute__(PIPELINES[self.pipeline][1])()
        except Exception as e:
            self.get_logger().error(f"Error loading pipeline: {e}")
            self.destroy_node()
            exit()

        self.raw_image_subscription = self.create_subscription(
            CompressedImage, topic, self.process_frame, qos_profile=qos_profile_sensor_data
        )

    def process_frame(self, msg):
        self.get_logger().info('Received raw image')
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_image = cv2.resize(cv_image, None, fx=16/9., fy=1.0, interpolation=cv2.INTER_CUBIC)
        height, width = cv_image.shape[:2]
        cv_image = cv_image[height//4:height, :]
                
        # Verifica se l'immagine è nitida
        # sharp = is_image_sharp(cv_image)
    
        
        if self.pipeline_class:
            detected, cv_image, detections = self.pipeline_class.process_image(cv_image, draw_results = True)

        for detection in detections:
            bbox = detection['bbox']
            cv_image = cv2.polylines(cv_image, [bbox], isClosed=True, color=(0, 255, 0), thickness=2)
            
        
        cv2.imshow("Test Detector", cv_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            self.get_logger().info("Terminating node as 'q' key was pressed")
            rclpy.shutdown()
            

def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    node = TestDetector()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

