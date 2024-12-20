import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node

class ImagePublisher(Node):
    def init(self) -> None:
        super().init("cam_pub")
        self.__publisher = self.create_publisher(Image, "frames", 10)
        self.__timer = self.create_timer(0.1, self.__timer_callback)
        self.__cap = cv2.VideoCapture(0)
        self.__br = CvBridge()

    def __timer_callback(self) -> None:
        ret, frame = self.__cap.read()
        if ret:
            self.__publisher.publish(self.__br.cv2_to_imgmsg(frame, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.__cap.release()
    node.destroy_node()
    rclpy.shutdown()