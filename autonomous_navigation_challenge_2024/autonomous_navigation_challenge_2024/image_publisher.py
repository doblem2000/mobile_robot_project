import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.get_logger().info("Image publisher node initialized")

        # ROS2 parameters
        self.declare_parameter("topic", "/oakd/rgb/preview/image_raw/compressed")
        self.topic = self.get_parameter("topic").value
        self.declare_parameter("img")
        self.img_path = self.get_parameter("img").value

        self.bridge = CvBridge()
        cv_image = cv2.imread(self.img_path)
        if cv_image is None:
            self.get_logger().error(f"Failed to load image from {self.img_path}")
            self.destroy_node()
            exit()
        self.image_msg = self.bridge.cv2_to_compressed_imgmsg(cv_image)

        self.publisher = self.create_publisher(CompressedImage, self.topic, qos_profile=qos_profile_sensor_data)
        self.timer = self.create_timer(1/30., self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Publishing image')
        self.publisher.publish(self.image_msg)

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
