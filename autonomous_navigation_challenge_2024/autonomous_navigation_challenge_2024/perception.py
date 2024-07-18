import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from autonomous_navigation_challenge_2024_msgs.msg import PerceptedSignpostList, PerceptedSignpost
from vision_msgs.msg import BoundingBox2D, Pose2D, Point2D
from rclpy.qos import qos_profile_sensor_data
import importlib
from .change_parameters import ChangeParameters
from std_srvs.srv import Trigger
from .config.camera_parameter import (
    i_keep_preview_aspect_ratio, i_resolution, i_width, i_height,
    i_framerate, i_preview_size, vfov, hfov
)
import json

PIPELINES = {
    "baseline": ("autonomous_navigation_challenge_2024.pipelines.baseline.baseline_pipeline", "BaselinePipeline"),
    "pyzbar": ("autonomous_navigation_challenge_2024.pipelines.pyzbar.pyzbar_pipeline", "PyzbarPipeline"),
    "qreader": ("autonomous_navigation_challenge_2024.pipelines.qreader.qreader_pipeline", "QReaderPipeline"),
    "qreader_3_windows": ("autonomous_navigation_challenge_2024.pipelines.qreader_3_windows.qreader_3_windows", "QReader3WsPipeline"),
    "qreader_sliding_window": ("autonomous_navigation_challenge_2024.pipelines.qreader_sliding_window.qreader_sw_pipeline", "QReaderSWPipeline"),
    "qreader_boost": ("autonomous_navigation_challenge_2024.pipelines.qreader_boost.qreader_boost_pipeline", "QReaderBoostPipeline"),
    "cv2_sliding_window": ("autonomous_navigation_challenge_2024.pipelines.cv2_sliding_window.cv2_sw_pipeline", "CV2SWPipeline"),
    "yolo1": ("autonomous_navigation_challenge_2024.pipelines.yolo1.yolo1_pipeline", "YOLO1Pipeline")
}


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.get_logger().info("Perception node initialized")
        
        with open("src/mobile-robots-project-work/autonomous_navigation_challenge_2024/autonomous_navigation_challenge_2024/config/camera_parameter.json", 'r') as f:
            self.config = json.load(f)

        # ROS2 parameters
        self.declare_parameter("topic", "/oakd/rgb/preview/image_raw/compressed")
        topic = self.get_parameter("topic").value
        self.declare_parameter("pipeline", "qreader_3_windows")
        self.pipeline = self.get_parameter("pipeline").value
        if self.pipeline not in PIPELINES:
            self.get_logger().error(f"Invalid pipeline: {self.pipeline}")
            self.destroy_node()
            exit()

        self.declare_parameter("on_turtlebot", False)
        self.on_turtlebot = self.get_parameter("on_turtlebot").value

        self.declare_parameter("debug", False)
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

        self.perception_publisher = self.create_publisher(
            PerceptedSignpostList,
            '/perception',
            10
        )
        
        if self.on_turtlebot:
            self._change_camera_parameters("/oakd", "rgb.i_keep_preview_aspect_ratio", self.config['keep_preview_aspect_ratio'])
            self._change_camera_parameters("/oakd", "rgb.i_resolution", self.config['resolution'])
            self._change_camera_parameters("/oakd", "rgb.i_width", self.config['width'])
            self._change_camera_parameters("/oakd", "rgb.i_height", self.config['height'])
            self._change_camera_parameters("/oakd", "rgb.i_framerate", self.config['framerate'])
            self._change_camera_parameters("/oakd", "rgb.i_preview_size", self.config['preview_size'])
            self._start_camera(stop_camera=True)
            self._start_camera()


    def _change_camera_parameters(self, node_name, parameter_name, parameter_value):
        node = ChangeParameters(node_name, parameter_name, parameter_value)
        rclpy.spin_once(node)
        node.destroy_node()


    def _start_camera(self, stop_camera=False):
        node=Node("start_camera")
        if stop_camera:
            cli = node.create_client(Trigger, "/oakd/stop_camera")
        else:
            cli = node.create_client(Trigger, "/oakd/start_camera")
        while not cli.wait_for_service(timeout_sec=2.0):
            node.get_logger().info('service not available, waiting again...')
        req = Trigger.Request()
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(node)
        if future.result() is not None:
            node.get_logger().info('Result of stop_camera: %r' % future.result().success)
        else:
            node.get_logger().error('Exception while calling service: %r' % future.exception())
        node.destroy_node()


    def _sign_to_id(self, sign):
        if sign is None:
            return PerceptedSignpost.SIGNPOST_NONE
        elif sign == "STRAIGHTON":
            return PerceptedSignpost.SIGNPOST_FORWARD
        elif sign == "LEFT":
            return PerceptedSignpost.SIGNPOST_LEFT
        elif sign == "RIGHT":
            return PerceptedSignpost.SIGNPOST_RIGHT
        elif sign == "GOBACK":
            return PerceptedSignpost.SIGNPOST_BACKWARD
        elif sign == "STOP":
            return PerceptedSignpost.SIGNPOST_STOP
        else:
            return PerceptedSignpost.SIGNPOST_NONE


    def create_msg_list(self, texts, bboxs, distances):
        msg_list = PerceptedSignpostList()
        signpost_list = []
        for text, bbox, distance in zip(texts, bboxs, distances):
            #(text, bbox,confidence)  = detection
            msg = PerceptedSignpost()
            msg.bounding_box = BoundingBox2D()

            if text is None:
                self.get_logger().info('No QR code detected')
            else:
                self.get_logger().info('QR code detected: ' + text)
                
            sign = self._sign_to_id(text)
            msg.signpost_type = sign
            msg.distance = distance

            top_left = bbox[0]
            top_right = bbox[1]
            bottom_right = bbox[2]
            bottom_left = bbox[3]

            center_x = (top_left[0] + top_right[0] + bottom_right[0] + bottom_left[0]) / 4.0
            center_y = (top_left[1] + top_right[1] + bottom_right[1] + bottom_left[1]) / 4.0

            width = np.linalg.norm(top_right - top_left)
            height = np.linalg.norm(bottom_left - top_left)

            msg.bounding_box.center = Pose2D()
            msg.bounding_box.center.position = Point2D()
            msg.bounding_box.center.position.x = float(center_x)
            msg.bounding_box.center.position.y = float(center_y)
            msg.bounding_box.center.theta = float(0.0)
            msg.bounding_box.size_x = float(width)
            msg.bounding_box.size_y = float(height)

            signpost_list.append(msg)
        # Sort the percepted signpost list
        signpost_list.sort(key = lambda signpost: -1.0/(signpost.distance+0.0001) if signpost.signpost_type != PerceptedSignpost.SIGNPOST_NONE else signpost.distance)
        msg_list.signposts = signpost_list
        return msg_list


    def process_frame(self, msg):
        self.get_logger().info('Received raw image')
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_image = cv2.resize(cv_image, None, fx=16/9., fy=1.0, interpolation=cv2.INTER_CUBIC)
        height, width = cv_image.shape[:2]
        cv_image = cv_image[height // 4:height, :]
        
        if self.pipeline_class:
            time = self.get_clock().now()
            detected, cv_image, detections = self.pipeline_class.process_image(cv_image, draw_results=self.debug)
            
            texts=[]
            bboxs=[]
            distances=[]
            
            for detection in detections:
                texts.append(detection['text'])
                bboxs.append(detection['bbox']) 
                distances.append(detection['distance'])
                        

            msg_list=self.create_msg_list(texts, bboxs, distances)
            elapsed_nanoseconds = (self.get_clock().now() - time).nanoseconds
            elapsed_seconds = elapsed_nanoseconds * 1e-9
            self.get_logger().info(f"Elaboration time (seconds): {elapsed_seconds} - FPS: {1.0 / elapsed_seconds}")
            
            if isinstance(msg_list, PerceptedSignpostList):
                self.perception_publisher.publish(msg_list)

            if self.debug:
                cv2.imshow("Perception", cv_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            self.get_logger().info("Terminating node as 'q' key was pressed")
            rclpy.shutdown()

def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    node = PerceptionNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
