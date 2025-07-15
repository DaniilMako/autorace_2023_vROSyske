import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import torch
import cv2

import warnings
warnings.filterwarnings("ignore", category=FutureWarning)


class Sign_detection(Node):

    """"
    Детекция знаков с помощью YOLOv5.
    Модель выдает только один знак и только тогда, когда площадь bbox превышает заданный порог (знак близко к роботу).
    """

    def __init__(self):
        super().__init__('Sign_detection')

        # Издатели
        self.class_pub = self.create_publisher(String, '/sign', 1)  # Издатель для отправки распознанного знака
        self.image_pub = self.create_publisher(Image, '/color/detect', 1)  # Издатель для отправки изображения с аннотациями
        self.stop_autorace = self.create_publisher(Bool, '/enable_following', 1)
        self.robot_finish = self.create_publisher(String, '/robot_finish', 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)  # Издатель для команд управления движением
        
        # Подписчики
        self.image_sub = self.create_subscription(Image, '/color/image', self.subscription_callback, 1)  # Подписчик на изображения с камеры
        
        # Загрузка модели YOLOv5
        self.model_path = self.declare_parameter('model_path', 'model').get_parameter_value().string_value 
        self.model = torch.hub.load(os.path.join(self.model_path, 'yolov5'), 'custom', path = os.path.join(self.model_path, 'best.pt'), source = 'local') 

        self.classes = self.model.names  # Список классов, которые может распознать модель

        # Минимальная площадь bbox для детекции знака
        self.min_square = self.declare_parameter('min_square', 0).get_parameter_value().integer_value   

        # Количество детекций для усреднения ответа
        self.detects_cnt = self.declare_parameter('detects_cnt', 0).get_parameter_value().integer_value   
        self.detects = []  # Список детектированных знаков для усреднения

        self.cvBridge = CvBridge()  # Мост для преобразования изображений между ROS и OpenCV

        # Логирование: узел запущен
        # self.get_logger().info("Узел Sign_detection запущен.")

        # Счетчик для уменьшения частоты вывода логов
        self.log_counter = 0

    def subscription_callback(self, image_msg):
        """ Обработчик изображений с камеры. """
        # Увеличиваем счетчик
        self.log_counter += 1

        # Логирование: получено новое изображение
        if self.log_counter % 30 == 0:
            self.get_logger().info("Получено новое изображение для обработки.")

        # Преобразуем ROS-изображение в формат OpenCV
        image = self.cvBridge.imgmsg_to_cv2(image_msg, image_msg.encoding)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Прогоняем изображение через модель YOLOv5
        result = self.model(image)    
        bbox = result.pandas().xyxy[0]  # Получаем данные о bbox

        # Проверка на детекцию
        if len(bbox['class'].values) != 0:
            # Вычисляем площадь bbox
            sqr = (bbox['xmax'].values[0] - bbox['xmin'].values[0]) * (bbox['ymax'].values[0] - bbox['ymin'].values[0]) 
            
            # Если площадь bbox превышает заданный порог
            if sqr > self.min_square:
                image = result.render()[0]  # Отображаем результаты детекции на изображении
                cur_sign = self.classes[result.pandas().xyxy[0]['class'].values[0]]  # Получаем класс знака

                # Логирование: обнаружен знак
                self.get_logger().info(f"О Б Н А Р У Ж Е Н знак: {cur_sign}")

                # Усредняем результат по заданному количеству детекций
                if len(self.detects) != self.detects_cnt:
                    self.detects.append(cur_sign)  # Добавляем текущий знак в список
                else:
                    # Находим самый встречаемый класс
                    unique, pos = np.unique(self.detects, return_inverse = True)
                    cur_sign = unique[np.bincount(pos).argmax()] 

                    self.detects = []  # Очищаем список детекций
                    if cur_sign == "tunnel_sign":
                        self.robot_finish.publish(String(data = "vROSyske"))
                        self.stop_autorace.publish(Bool(data = False))
                        twist = Twist()   
                        twist.linear.x = 0
                        twist.angular.z = 0
                        self.cmd_vel_pub.publish(twist)  # Публикация команды управления 
                    self.class_pub.publish(String(data = cur_sign))  # Публикуем распознанный знак

                    # Логирование: опубликован распознанный знак
                    self.get_logger().info(f"Опубликован распознанный знак: {cur_sign}")

        # Преобразуем изображение обратно в формат ROS
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) 
        self.image_pub.publish(self.cvBridge.cv2_to_imgmsg(image,  image_msg.encoding))

        # Логирование: изображение с аннотациями опубликовано
        if self.log_counter % 30 == 0:
            self.get_logger().info("Изображение с аннотациями опубликовано.")
                

def main():
    """ Основная функция для запуска узла. """
    rclpy.init()  # Инициализация ROS 2

    node = Sign_detection()  # Создаем экземпляр узла
    rclpy.spin(node)  # Запуск обработки сообщений
    
    node.destroy_node()  # Уничтожаем узел после завершения работы
    rclpy.shutdown()  # Завершаем работу ROS 2