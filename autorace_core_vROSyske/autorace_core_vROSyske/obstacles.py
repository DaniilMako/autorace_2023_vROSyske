import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool, String, Int8
from robot_rotate_interface.msg import Rotate
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import numpy as np


class Obstacles_Handler(Node):
    """
    Объезд препятствий.
    С помощью лидара отслеживается расстояние перед роботом до препятсвия. 
    Принудительно поворачивает робота "по змейке". 
    """

    def __init__(self):
        super().__init__('Obstacles_Handler')

        # Издатели
        self.enable_following_pub = self.create_publisher(Bool, '/enable_following', 1)  # Издатель для включения следования по полосе
        self.max_vel_pub = self.create_publisher(Float64, '/max_vel', 1)  # Издатель для установки максимальной скорости
        self.rotate_pub = self.create_publisher(Rotate, '/rotate', 1)  # Издатель для отправки команды поворота
        
        # Подписчики
        self.rotate_done_sub = self.create_subscription(Int8, '/rotate_done', self.set_rotate_done, 1)  # Подписчик на сигнал о завершении поворота
        self.sign_sub = self.create_subscription(String, '/sign', self.handle_sign, 1)  # Подписчик на распознанные знаки
        self.laser_scan_sub = self.create_subscription(LaserScan, '/scan', self.get_distance, 1)  # Подписчик на данные лидара
        self.enable_detection_pub = self.create_subscription(Bool, '/enable_detection', self.get_detection_state, 1)  # Подписчик на состояние детекции знаков
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.get_odom, 1)  # Подписчик на данные одометрии

        self.ID = 1  # Идентификатор ноды

        self.enable_detection = False  # Разрешена ли детекция знака

        self.dir = True  # Направление поворота: True - налево, False - направо
        self.enable = False  # Режим объезда препятствий
        self.turned = False  # Флаг окончания поворота

        # Скорости
        self.in_speed = self.declare_parameter('in_speed', 0.0).get_parameter_value().double_value
        self.out_speed = self.declare_parameter('out_speed', 0.0).get_parameter_value().double_value

        # Углы поворота
        self.angle_L = self.declare_parameter('angle_L', 0.0).get_parameter_value().double_value
        self.angle_R = self.declare_parameter('angle_R', 0.0).get_parameter_value().double_value

        # Линейные скорости при повороте
        self.linear_x_L = self.declare_parameter('linear_x_L', 0.0).get_parameter_value().double_value
        self.linear_x_R = self.declare_parameter('linear_x_R', 0.0).get_parameter_value().double_value

        # Угловые скорости при повороте
        self.angular_z_L = self.declare_parameter('angular_z_L', 0.0).get_parameter_value().double_value
        self.angular_z_R = self.declare_parameter('angular_z_R', 0.0).get_parameter_value().double_value

        # Дистанция для начала поворота
        self.distance = self.declare_parameter('distance', 0.0).get_parameter_value().double_value

        # Логирование: узел запущен
        # self.get_logger().info("Узел Obstacles_Handler запущен.")

    def handle_sign(self, msg):
        """Обработчик распознанных знаков."""
        
        # Действия при проезде знака дорожных работ
        if msg.data == 'road_works_sign' and self.enable_detection:
            self.max_vel_pub.publish(Float64(data = self.in_speed))  # Устанавливаем скорость для проезда зоны работ
            self.get_logger().info("О Б Н А Р У Ж Е Н знак 'road_works_sign'. Установлена скорость для проезда зоны работ.")

    def get_distance(self, msg):
        """Обработчик данных лидара."""
        
        # При нахождении знака определяем расстояние до препятствия 
        if self.enable:
            front_distance = np.min(np.concatenate((msg.ranges[345:360], msg.ranges[0:15]), axis = 0))  # Определяем минимальное расстояние перед роботом

            if front_distance < self.distance:
                self.enable_following_pub.publish(Bool(data = False))  # Отключаем следование по полосе
                self.get_logger().info(f"О Б Н А Р У Ж Е Н О препятствие на расстоянии {front_distance}. Отключено следование по полосе.")

                # Поворот налево 
                if self.dir:
                    self.rotate_pub.publish(Rotate(angle = self.angle_L, linear_x = self.linear_x_L, angular_z = self.angular_z_L, id = self.ID))  # Отправляем команду поворота налево
                    self.get_logger().info("Отправлена команда на поворот НАЛЕВО.")

                # Поворот направо 
                if not self.dir and self.turned:
                    self.rotate_pub.publish(Rotate(angle = self.angle_R, linear_x = self.linear_x_R, angular_z = self.angular_z_R, id = self.ID))  # Отправляем команду поворота направо
                    self.get_logger().info("Отправлена команда на поворот НАПРАВО.")
               
    def set_rotate_done(self, msg):
        """Обработчик сигнала о завершении поворота."""
        
        if msg.data == self.ID:
            self.turned = True  # Устанавливаем флаг окончания поворота
            self.get_logger().info("Поворот завершен.")

            # Смена направления поворота
            if self.dir:
                self.dir = False  # Меняем направление на поворот направо
                self.get_logger().info("Смена направления поворота: НАПРАВО.")
            else:
                # Отключение ноды после завершения второго поворота
                self.enable_following_pub.publish(Bool(data = True))  # Включаем следование по полосе
                self.max_vel_pub.publish(Float64(data = self.out_speed))  # Устанавливаем скорость для выезда из зоны препятствий
                self.get_logger().info("В К Л следование по полосе. Установлена скорость для выезда из зоны препятствий.")
                self.get_logger().info("В Ы К Л режим объезда препятствий.")
                self.get_logger().info("Узел Obstacles_Handler З А В Е Р Ш И Л работу.")
                self.get_logger().info("Узел Obstacles_Handler З А В Е Р Ш И Л работу.")
                self.get_logger().info("Узел Obstacles_Handler З А В Е Р Ш И Л работу.")
                rclpy.shutdown()  # Завершаем работу узла

    def get_detection_state(self, msg):
        """Обработчик состояния детекции знаков."""
        self.enable_detection = msg.data  # Устанавливаем состояние детекции знаков
        self.get_logger().info(f"Состояние детекции знаков: {self.enable_detection}.")

    def get_odom(self, msg):
        """Обработчик данных одометрии."""

        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y

        if pose_x >= 0.74 and pose_y >= 2.0:
            self.enable = True  # Включаем режим объезда препятствий
            self.get_logger().info("В К Л режим объезда препятствий.")


def main():
    """Основная функция для запуска узла."""
    rclpy.init()  # Инициализация ROS 2

    node = Obstacles_Handler()  # Создание экземпляра узла
    rclpy.spin(node)  # Запуск обработки сообщений
    
    node.destroy_node()  # Уничтожение узла после завершения работы
    rclpy.shutdown()  # Завершение работы ROS 2