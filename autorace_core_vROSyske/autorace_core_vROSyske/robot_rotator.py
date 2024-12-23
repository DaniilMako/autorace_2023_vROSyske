import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_rotate_interface.msg import Rotate

import numpy as np

class Rotator(Node):
    """
    Совершает поворот робота на заданный угол с заданными угловой и линейной скоростями.
    По завершении поворота посылает сообщение по топику.
    """

    def __init__(self):
        super().__init__('Rotator')

        # Издатели
        self.rotate_done_pub = self.create_publisher(Int8, '/rotate_done', 1)  # Издатель для сигнала о завершении поворота
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)  # Издатель для управления движением робота

        # Подписчики
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.get_odom, 1)  # Подписчик на данные одометрии
        self.rotate_sub = self.create_subscription(Rotate, '/rotate', self.get_data, 1)  # Подписчик на команды поворота

        self.timer = self.create_timer(0.1, self.rotate_robot)  # Таймер для регулярного вызова функции поворота

        self.cur_angle = None   # Текущий абсолютный угол поворота
        self.start_angle = None # Абсолютный угол поворота при начале поворота
        
        self.angle = None       # Относительный угол поворота
        self.linear_x = None    # Линейная скорость при повороте
        self.angular_z = None   # Угловая скорость при повороте
        self.id = None          # Идентификатор вызывающей ноды

        # Логирование: узел запущен
        # self.get_logger().info("Узел Rotator запущен.")

    def rotate_robot(self):
        """Функция для выполнения поворота робота."""
        
        # Принудительно повернуть робота на заданный угол
        if self.angle is not None and self.start_angle is not None and self.cur_angle is not None:
            
            # Определить текущий относительный поворот
            diff = self.cur_angle - self.start_angle
            diff += -360 if diff > 180 else 360 if diff < -180 else 0

            # Проверка на окончание поворота
            if np.abs(diff) >= np.abs(self.angle):
                
                self.rotate_done_pub.publish(Int8(data = self.id))  # Отправка сигнала о завершении поворота

                # Логирование: поворот завершен
                self.get_logger().info(f"Поворот завершен. Отправлен сигнал о завершении поворота с ID: {self.id}.")

                # Сброс переменных
                self.angle = None
                self.start_angle = None
                self.cur_angle = None
                self.id = None
        
            else:
                twist = Twist()
                twist.linear.x = self.linear_x  # Установка линейной скорости
                twist.angular.z = self.angular_z * np.sign(self.angle)  # Установка угловой скорости

                self.cmd_vel_pub.publish(twist)  # Отправка команды на движение

                # Логирование: отправлена команда на движение
                self.get_logger().info(f"Отправлена команда на движение: линейная скорость = {twist.linear.x}, угловая скорость = {twist.angular.z}.")

    def get_odom(self, msg):
        """Обработчик данных одометрии."""
        
        if self.angle is not None:

            # Текущий угол поворота в градусах
            self.cur_angle = self.euler_from_quaternion(msg.pose.pose.orientation)[2]
            self.cur_angle = np.degrees(self.cur_angle)
            self.cur_angle = 360 + self.cur_angle if self.cur_angle < 0 else self.cur_angle

            # Логирование: текущий угол поворота
            self.get_logger().info(f"Текущий угол поворота: {self.cur_angle} градусов.")

            # Фиксация начального угла перед поворотом робота
            if self.start_angle is None:
                self.start_angle = self.cur_angle

                # Логирование: начальный угол поворота
                self.get_logger().info(f"Фиксация начального угла поворота: {self.start_angle} градусов.")

    def euler_from_quaternion(self, quaternion):
        """Перевод кватерниона в углы Эйлера."""
        
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def get_data(self, msg):
        """Обработчик команды на поворот."""

        self.angle = msg.angle  # Угол поворота
        self.linear_x = msg.linear_x  # Линейная скорость
        self.angular_z = msg.angular_z  # Угловая скорость
        self.id = msg.id  # Идентификатор вызывающей ноды

        # Логирование: получена команда на поворот
        self.get_logger().info(f"Получена команда на поворот: угол = {self.angle}, линейная скорость = {self.linear_x}, угловая скорость = {self.angular_z}, ID = {self.id}.")


def main():
    """Основная функция для запуска узла."""
    rclpy.init()  # Инициализация ROS 2

    node = Rotator()  # Создание экземпляра узла
    rclpy.spin(node)  # Запуск обработки сообщений
    
    node.destroy_node()  # Уничтожение узла после завершения работы
    rclpy.shutdown()  # Завершение работы ROS 2