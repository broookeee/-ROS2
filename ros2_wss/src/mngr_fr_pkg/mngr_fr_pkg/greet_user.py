from gtts import gTTS
import pygame
import time
from io import BytesIO
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class GreetUser(Node):
    def init(self) -> None:
        super().init("greet_user")
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