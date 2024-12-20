import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mngr_fr_pkg.mngr_facerec import FaceRecognizer  # Импорт вашего класса распознавания лиц

class UpdateUsers(Node):

    def init(self):
        super().init('update_users')
        self.create_subscription(
            String,  # Получаем строку с именем пользователя
            'recognized_user',  # Поток сообщений, куда публикуется имя пользователя
            self.callback,
            10
        )

        # Инициализируем объект для работы с базой данных пользователей
        self.__fr = FaceRecognizer('/path/to/images', '/path/to/encodings.pkl', '/path/to/users.json')

    def callback(self, msg):
        self.get_logger().info(f'User recognized: {msg.data}')
        # Обновляем информацию о пользователе
        self.__fr.update_users()

def main(args=None):
    rclpy.init(args=args)
    node = UpdateUsers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()