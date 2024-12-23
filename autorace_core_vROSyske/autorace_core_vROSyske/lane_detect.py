import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import cv2

class LineDetector(Node):
    """ 
    Определение белой и желтой линий на трансформированном изображении, 
    поступающего с камеры. Публикует координату x, на которой находится центр между линиями.
    """

    def __init__(self):
        super().__init__('LineDetector')

        # Издатели
        self.center_lane_pub = self.create_publisher(Float64, '/center_lane', 1)  # Издатель для координаты центра между линиями

        # Подписчики
        self.img_proj_sub = self.create_subscription(Image, '/color/image_projected', self.image_processing, 1)  # Подписчик на трансформированное изображение

        self.cv_bridge = CvBridge()  # Мост для преобразования изображений между ROS и OpenCV
        self.is_parking = False  # Флаг парковки (не используется в данном коде)
        
    def image_processing(self, msg):
        """Обработка изображения для определения центров линий."""
        
        # Считывание изображения и перевод в HSV
        image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Центры линий
        centers_x = []

        # Маска желтой линии
        yellow_mask = cv2.inRange(hsv_image, (20, 100, 100), (30, 255, 255))  # Выделение желтого цвета
        yellow_mask = cv2.blur(yellow_mask, (3, 3))  # Размытие для устранения шума
        yellow_mask[yellow_mask != 0] = 255  # Бинаризация маски

        # Маска белой линии
        white_mask = cv2.inRange(hsv_image, (0, 0, 230), (255, 0, 255))  # Выделение белого цвета
        white_mask = cv2.blur(white_mask, (3, 3))  # Размытие для устранения шума
        white_mask[white_mask != 0] = 255  # Бинаризация маски
        
        # Определение центров линий
        M_yellow = cv2.moments(yellow_mask, binaryImage = True)  # Моменты для желтой линии
        M_white = cv2.moments(white_mask, binaryImage = True)  # Моменты для белой линии

        # Координата x центра желтой линии
        yellow_center_x = 0 if M_yellow['m00'] == 0 else M_yellow['m10'] // M_yellow['m00']
        # Координата x центра белой линии
        white_center_x = hsv_image.shape[1] if M_white['m00'] == 0 else M_white['m10'] // M_white['m00']

        # Пропускаем те случаи, когда желтая линия не левее белой
        if white_center_x < yellow_center_x:
            white_center_x = image.shape[1]  # Устанавливаем координату белой линии за пределами изображения

        centers_x.append(yellow_center_x)
        centers_x.append(white_center_x)

        # Публикация центра между линиями
        self.center_lane_pub.publish(Float64(data = np.sum(centers_x) / len(centers_x)))


def main():
    """Основная функция для запуска узла."""
    rclpy.init()  # Инициализация ROS 2

    node = LineDetector()  # Создание экземпляра узла
    rclpy.spin(node)  # Запуск обработки сообщений
    
    node.destroy_node()  # Уничтожение узла после завершения работы
    rclpy.shutdown()  # Завершение работы ROS 2