import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.executors import SingleThreadedExecutor
from irobot_create_msgs.msg import KidnapStatus
import threading
import termios
import tty
import sys
from select import select

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


class MockKidnap(Node):
    def __init__(self):
        super().__init__("mock_kidnap")
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.get_logger().info("Initializing mock_kidnap node")
        self.get_logger().info("Press 'k' to publish kidnap")
        self.get_logger().info("Press 'n' to publish not kidnap")
        self.get_logger().info("Press 'q' to quit")
        
        self.settings = saveTerminalSettings()
        
        self._kidnap=False
        #self._timer=None
        self.publisher = self.create_publisher(KidnapStatus, "/kidnap_status_mock", 10)
        self._timer=self.create_timer(0.016, self.publish_message)
        self.timer_switch = self.create_timer(0.01, self.timer_switch_callback)
        
        
    def publish_message(self):
        msg = KidnapStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.is_kidnapped = self._kidnap
        self.publisher.publish(msg)
        # self.get_logger().info("Kidnap message published")


    def timer_switch_callback(self):            
        key = getKey(self.settings, 0.016)
        if key == 'q':
            self.get_logger().info("Quitting")
            self.destroy_node()
            sys.exit()
        elif key == 'k':
            self._kidnap=True 
            self.get_logger().info(f"Published kidnap {self._kidnap}")
        elif key == 'n':
            self._kidnap=False 
            self.get_logger().info(f"Published NOT kidnap {self._kidnap}")
        # msg = KidnapStatus()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.is_kidnapped = self._kidnap
        # self.publisher.publish(msg) 
        #self._timer=self.create_timer(0.016, self.publish_kidnap)  # Create a timer that calls publish_kidnap_false every 0.016 seconds

    
def main():
    rclpy.init()
    executor = SingleThreadedExecutor()

    node = MockKidnap()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()

if __name__ == '__main__':
    main()
