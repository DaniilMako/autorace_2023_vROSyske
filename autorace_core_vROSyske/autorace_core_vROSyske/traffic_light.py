import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from cv_bridge import CvBridge

import numpy as np
import cv2


class Traffic_Light_Handler(Node):
    """ 
    Определение зеленого цвета светофора. 
    После обнаружения посылает сигнал о начале старта.
    """

    def __init__(self):
        super().__init__('Traffic_Light_Handler')

        # Создаем издателя для сигнала о начале движения
        self.enable_following_pub = self.create_publisher(Bool, '/enable_following', 1)
        
        # Создаем подписчика для получения изображения с камеры
        self.image_sub = self.create_subscription(Image, '/color/image', self.find_green, 1)

        # Инициализируем мост для преобразования ROS-изображений в OpenCV
        self.cv_bridge = CvBridge()

    def find_green(self, msg):
        """ Метод для поиска зеленого цвета на изображении. """

        # Считывание изображения и перевод в формат HSV
        image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Маскирование зеленого цвета в диапазоне HSV
        green_mask = cv2.inRange(image, (50, 100, 100), (70, 255, 255))

        # Если на изображении есть зеленый цвет, отправляем сигнал на начало движения
        if np.any(green_mask != 0):
            self.enable_following_pub.publish(Bool(data = True))
            rclpy.shutdown()  # Завершаем работу узла после обнаружения зеленого цвета


def main():
    """ Основная функция для запуска узла. """
    rclpy.init()  # Инициализация ROS 2

    node = Traffic_Light_Handler()  # Создаем экземпляр узла
    rclpy.spin(node)  # Запуск обработки сообщений
    
    node.destroy_node()  # Уничтожаем узел после завершения работы
    rclpy.shutdown()  # Завершаем работу ROS 2