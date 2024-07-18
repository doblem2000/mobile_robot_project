import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import termios
import tty
import sys
from select import select

from autonomous_navigation_challenge_2024_msgs.msg import PerceptedSignpostList, PerceptedSignpost

def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

class MockSignposts(Node):
    def __init__(self):
        super().__init__("mock_signposts_publisher") 
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.get_logger().info("Mock Signposts Publisher node initialized")

        self.settings = saveTerminalSettings()
        
        self.pub = self.create_publisher(PerceptedSignpostList, '/perception', 10)
        self.timer = self.create_timer(0.01, self.publish_signpost)
        self.get_logger().info("Mock Signposts Publisher node ready")
        self.get_logger().info("Press 'w' to publish a forward signpost")
        self.get_logger().info("Press 'a' to publish a left signpost")
        self.get_logger().info("Press 'd' to publish a right signpost")
        self.get_logger().info("Press 's' to publish a backward signpost")
        self.get_logger().info("Press 'e' to publish a stop signpost")
        self.get_logger().info("Press 'i' to publish an invalid signpost")
        self.get_logger().info("Press 'q' to quit")

    def publish_signpost(self):
        signpost_list = PerceptedSignpostList()
        signpost = PerceptedSignpost()
        signpost.signpost_type = PerceptedSignpost.SIGNPOST_NONE
        signpost.distance = 0.0
        key = getKey(self.settings, 0.01)
        if key == 'q':
            self.get_logger().info("Quitting")
            self.destroy_node()
            sys.exit()
        elif key == 'w':
            signpost.signpost_type = PerceptedSignpost.SIGNPOST_FORWARD
            text = "forward"
        elif key == 'a':
            signpost.signpost_type = PerceptedSignpost.SIGNPOST_LEFT
            text = "left"
        elif key == 'd':
            signpost.signpost_type = PerceptedSignpost.SIGNPOST_RIGHT
            text = "right"
        elif key == 's':
            signpost.signpost_type = PerceptedSignpost.SIGNPOST_BACKWARD
            text = "backward"
        elif key == 'e':
            signpost.signpost_type = PerceptedSignpost.SIGNPOST_STOP
            text = "stop"
        elif key == 'i':
            signpost.signpost_type = PerceptedSignpost.SIGNPOST_NONE
            text = "invalid"
        else:
            self.pub.publish(signpost_list)
            return
        signpost_list.signposts.append(signpost)
        self.pub.publish(signpost_list)
        self.get_logger().info(f"Published percepted signpost {text}")

def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    node = MockSignposts()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()

if __name__ == '__main__':
    main()