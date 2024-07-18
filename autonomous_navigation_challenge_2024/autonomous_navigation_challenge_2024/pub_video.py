import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
import rosbag2_py
from rclpy.serialization import deserialize_message
import cv2
from cv_bridge import CvBridge

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.declare_parameter("video", None)
        self.declare_parameter("topic", "/oakd/rgb/preview/image_raw/compressed")
        self.publisher = self.create_publisher(CompressedImage, self.get_parameter("topic").value, 10)
        self.reader = rosbag2_py.SequentialReader()
        self.get_logger().info(f"Opening video file: {self.get_parameter('video').value}")
        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()
        self.reader.open(
            rosbag2_py.StorageOptions(uri=self.get_parameter("video").value, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
        )
        self.create_timer(1/15., self.pub_new_frame)

    def pub_new_frame(self):
        try:
            topic, data, timestamp = self.reader.read_next()
        except RuntimeError as e:
            self.get_logger().info("End of video file reached. Exiting.")
            self.destroy_node()
            return
        if topic != '/oakd/rgb/preview/image_raw/compressed':
            return
        self.get_logger().info(f"Publishing frame from topic: {topic}")
        img = deserialize_message(data, CompressedImage)
        self.publisher.publish(img)
        cv_image = self.bridge.compressed_imgmsg_to_cv2(img)
        cv_image = cv2.resize(cv_image, None, fx=16/9., fy = 1.0, interpolation=cv2.INTER_CUBIC)

        ret, points, code = self.qr_detector.detectAndDecode(cv_image)
        if ret != '':
            pt1 = (int(points[0][0][0]), int(points[0][0][1]))
            pt2 = (int(points[0][1][0]), int(points[0][1][1]))
            pt3 = (int(points[0][2][0]), int(points[0][2][1]))
            pt4 = (int(points[0][3][0]), int(points[0][3][1]))
            cv_image = cv2.line(cv_image, pt1, pt2, (0, 0, 255) , 10)
            cv_image = cv2.line(cv_image, pt2, pt3, (0, 0, 255) , 10) 
            cv_image = cv2.line(cv_image, pt3, pt4, (0, 0, 255) , 10) 
            cv_image = cv2.line(cv_image, pt4, pt1, (0, 0, 255) , 10)

        cv2.imshow('Camera', cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
