import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

class ViewCamera(Node):
    def __init__(self):
        super().__init__('view_camera')
        self.raw_image_subscription = self.create_subscription(CompressedImage, '/oakd/rgb/preview/image_raw/compressed', self.detector_callback, qos_profile=qos_profile_sensor_data)
        self.bridge = CvBridge()

    def detector_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        cv_image = cv2.resize(cv_image, None, fx=16/9., fy = 1.0, interpolation=cv2.INTER_CUBIC)
        cv2.imshow('Camera', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    camera_viewer = ViewCamera()
    rclpy.spin(camera_viewer)
    camera_viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
