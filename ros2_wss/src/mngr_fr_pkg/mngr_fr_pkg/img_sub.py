import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
from mngr_fr_pkg.mngr_facerec import FaceRecognizer  # ������ ������ ������ ������������� ���

class ImageSubscriber(Node):

    def init(self):
        super().init('img_sub')
        # ������� �������� �� ����� ��������� � �������������
        self.create_subscription(
            Image, 
            'frames',  # ��� ������ � �������������
            self.callback, 
            10
        )
        # ��������� ��������� � ����� ����� ���������
        self.__publisher = self.create_publisher(String, 'recognized_user', 10)
        
        # ��� �������������� ����������� OpenCV -> ROS2
        self.__br = CvBridge()

        # �������������� ������ ������������� ���
        path = os.path.join(os.path.expanduser('~'), 'mngr_fr_pkg')  # ������ ����
        self.__fr = FaceRecognizer(os.path.join(path, "Images"), os.path.join(path, "encodings.pkl"), os.path.join(path, "users.json"))

    def callback(self, data):
        self.get_logger().info('Received an image for face recognition')

        # ����������� ROS2 ����������� � OpenCV
        current_frame = self.__br.imgmsg_to_cv2(data)

        # ���������� ����
        self.__fr.recognize_faces(current_frame)
        recognized_id = self.__fr.get_user_id()

        # ���������, ���������� �� �� ����
        if recognized_id == "-1":
            self.get_logger().info("New user detected, creating user...")
            self.__fr.create_user(current_frame)
            return

        # ���������� ��� ������������� ������������
        if recognized_id != "-2":
            json_data = self.__fr.get_json_data()
            msg = String()
            msg.data = json_data[recognized_id]["name"]
            self.__publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()