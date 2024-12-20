import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
from mngr_fr_pkg.mngr_facerec import FaceRecognizer  # Импорт вашего класса распознавания лиц

class ImageSubscriber(Node):

    def init(self):
        super().init('img_sub')
        # Создаем подписку на поток сообщений с изображениями
        self.create_subscription(
            Image, 
            'frames',  # Имя потока с изображениями
            self.callback, 
            10
        )
        # Публикуем результат в новый поток сообщений
        self.__publisher = self.create_publisher(String, 'recognized_user', 10)
        
        # Для преобразования изображений OpenCV -> ROS2
        self.__br = CvBridge()

        # Инициализируем объект распознавания лиц
        path = os.path.join(os.path.expanduser('~'), 'mngr_fr_pkg')  # Пример пути
        self.__fr = FaceRecognizer(os.path.join(path, "Images"), os.path.join(path, "encodings.pkl"), os.path.join(path, "users.json"))

    def callback(self, data):
        self.get_logger().info('Received an image for face recognition')

        # Преобразуем ROS2 изображение в OpenCV
        current_frame = self.__br.imgmsg_to_cv2(data)

        # Распознаем лица
        self.__fr.recognize_faces(current_frame)
        recognized_id = self.__fr.get_user_id()

        # Проверяем, распознали ли мы лицо
        if recognized_id == "-1":
            self.get_logger().info("New user detected, creating user...")
            self.__fr.create_user(current_frame)
            return

        # Отправляем имя распознанного пользователя
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