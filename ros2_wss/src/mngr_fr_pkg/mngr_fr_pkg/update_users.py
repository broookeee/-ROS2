import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mngr_fr_pkg.mngr_facerec import FaceRecognizer  # ������ ������ ������ ������������� ���

class UpdateUsers(Node):

    def init(self):
        super().init('update_users')
        self.create_subscription(
            String,  # �������� ������ � ������ ������������
            'recognized_user',  # ����� ���������, ���� ����������� ��� ������������
            self.callback,
            10
        )

        # �������������� ������ ��� ������ � ����� ������ �������������
        self.__fr = FaceRecognizer('/path/to/images', '/path/to/encodings.pkl', '/path/to/users.json')

    def callback(self, msg):
        self.get_logger().info(f'User recognized: {msg.data}')
        # ��������� ���������� � ������������
        self.__fr.update_users()

def main(args=None):
    rclpy.init(args=args)
    node = UpdateUsers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()