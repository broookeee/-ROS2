1. Подготовка среды:
   
   1.1. Установите ROS2:
   
    ·Если ROS2 еще не установлен, следуйте официальной документации ROS2 для установки подходящей версии для вашей системы.
    ·Убедитесь, что установлены зависимости: colcon, python3, pip.
   
   1.2. Установите необходимые библиотеки:
   
    sudo apt update
    sudo apt install python3-pip python3-venv python3-colcon-common-extensions
    pip3 install setuptools empy gttts pygame opencv-python-headless
   
2. Создание структуры пакета:
   
   2.1. Создайте директорию проекта ROS2.
         mkdir -p ~/ros2_ws/src/mngr_fr_pkg
         cd ~/ros2_ws/src/mngr_fr_pkg
   
   2.2. Создайте следующие файлы и каталоги:
        · package.xml
        · setup.py
        · setup.cfg
        · resource/mngr_fr_pkg (пустой файл с именем пакета)
        · Каталог mngr_fr_pkg с файлами __init__.py, img_sub.py, cam_pub.py, update_users.py, greet_user.py.
   
3. Файлы пакета:
   
   3.1. Файл setup.py:

         from setuptools import setup
         
         package_name = "mngr_fr_pkg"
         
         setup(
             name=package_name,
             version="0.0.1",
             packages=[package_name],
             data_files=[
                 ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                 ("share/" + package_name, ["package.xml"]),
             ],
             install_requires=["setuptools"],
             entry_points={
                 "console_scripts": [
                     "img_sub = " + package_name + ".img_sub:main",
                     "cam_pub = " + package_name + ".cam_pub:main",
                     "update_users = " + package_name + ".update_users:main",
                     "greet_user = " + package_name + ".greet_user:main",
                 ],
             },
             description="A face recognition package for the office manager robot",
         )
   
   3.2. Файл package.xml:
         
         <?xml version="1.0"?>
         <package format="3">
             <name>mngr_fr_pkg</name>
             <version>0.0.1</version>
             <description>A face recognition package for the office manager robot</description>
             <maintainer email="youremail@example.com">Your Name</maintainer>
             <license>Apache 2.0</license>
             <exec_depend>rclpy</exec_depend>
             <exec_depend>std_msgs</exec_depend>
             <exec_depend>sensor_msgs</exec_depend>
             <export>
                 <build_type>ament_python</build_type>
             </export>
         </package>
   
   3.3. Файл setup.cfg:

         [develop]
         script_dir=$base/lib/mngr_fr_pkg
         
         [install]
         install_scripts=$base/lib/mngr_fr_pkg


4. Реализация узлов ROS2:
   
   4.1. Файл cam_pub.py:

         import cv2
         from cv_bridge import CvBridge
         from sensor_msgs.msg import Image
         import rclpy
         from rclpy.node import Node
         
         class ImagePublisher(Node):
             def __init__(self) -> None:
                 super().__init__("cam_pub")
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

   4.2. Файл greet_user.py:

         from gtts import gTTS
         import pygame
         import time
         from io import BytesIO
         from std_msgs.msg import String
         import rclpy
         from rclpy.node import Node
         
         class GreetUser(Node):
             def __init__(self) -> None:
                 super().__init__("greet_user")
                 self.create_subscription(String, "greetings", self.__callback, 10)
         
             def __callback(self, msg: String) -> None:
                 self.__speak_text(f"Здравствуйте, {msg.data}")
         
             def __speak_text(self, text: str) -> None:
                 pygame.mixer.init()
                 io = BytesIO()
                 tts = gTTS(text, lang="ru")
                 tts.write_to_fp(io)
                 io.seek(0)
                 sound = pygame.mixer.Sound(io)
                 sound.play()
                 while pygame.mixer.get_busy():
                     time.sleep(1)
         
         def main(args=None):
             rclpy.init(args=args)
             node = GreetUser()
             rclpy.spin(node)
             node.destroy_node()
             rclpy.shutdown()

   4.3. Файл img_sub.py:

         import rclpy
         from rclpy.node import Node
         from sensor_msgs.msg import Image
         from std_msgs.msg import String
         from cv_bridge import CvBridge
         import cv2
         import os
         from mngr_fr_pkg.mngr_facerec import FaceRecognizer 
         
         class ImageSubscriber(Node):
         
             def __init__(self):
                 super().__init__('img_sub')
                 self.create_subscription(
                     Image, 
                     'frames',
                     self.callback, 
                     10
                 )
                 self.__publisher = self.create_publisher(String, 'recognized_user', 10)
                 
                 self.__br = CvBridge()
         
                 path = os.path.join(os.path.expanduser('~'), 'mngr_fr_pkg') 
                 self.__fr = FaceRecognizer(os.path.join(path, "Images"), os.path.join(path, "encodings.pkl"), os.path.join(path, "users.json"))
         
             def callback(self, data):
                 self.get_logger().info('Received an image for face recognition')
         
                 current_frame = self.__br.imgmsg_to_cv2(data)
         
                 self.__fr.recognize_faces(current_frame)
                 recognized_id = self.__fr.get_user_id()
         
                 if recognized_id == "-1":
                     self.get_logger().info("New user detected, creating user...")
                     self.__fr.create_user(current_frame)
                     return
         
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

   
   4.4. Файл update_users.py:

         import rclpy
         from rclpy.node import Node
         from std_msgs.msg import String
         from mngr_fr_pkg.mngr_facerec import FaceRecognizer  
         
         class UpdateUsers(Node):
         
             def __init__(self):
                 super().__init__('update_users')
                 self.create_subscription(
                     String,  
                     'recognized_user', 
                     self.callback,
                     10
                 )
         
                 self.__fr = FaceRecognizer('/path/to/images', '/path/to/encodings.pkl', '/path/to/users.json')
         
             def callback(self, msg):
                 self.get_logger().info(f'User recognized: {msg.data}')
                 self.__fr.update_users()
         
         def main(args=None):
             rclpy.init(args=args)

5. Сборка и запуск пакета:
   
   5.1. Перейдите в рабочую область ROS2:

      cd ~/ros2_ws

   5.2. Соберите пакет:

      colcon build --symlink-install

   5.3. Активируйте рабочую среду:

      source install/setup.bash

6. Отладка и тестирование:

   ros2 topic list
   ros2 topic echo /frames


