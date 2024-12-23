import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float64, Int8
from nav_msgs.msg import Odometry
from robot_rotate_interface.msg import Rotate


class Intersection_Handler(Node):
    """
    Проезд перекрестка.
    При знаке "поворот налево" или "поворот направо" принудительно поворачивает робота в соответствующую сторону,
    после чего робот снова следует вдоль полосы по PID. 
    """

    def __init__(self):
        super().__init__('Intersection_Handler')

        # Издатели
        self.enable_following_pub = self.create_publisher(Bool, '/enable_following', 1)  # Издатель для включения следования по полосе
        self.enable_detection_pub = self.create_publisher(Bool, '/enable_detection', 1)  # Издатель для включения детекции знаков
        self.max_vel_pub = self.create_publisher(Float64, '/max_vel', 1)  # Издатель для установки максимальной скорости
        self.offset_pub = self.create_publisher(Float64, '/offset', 1)  # Издатель для установки смещения
        self.rotate_pub = self.create_publisher(Rotate, '/rotate', 1)  # Издатель для отправки команды поворота
        
        # Подписчики
        self.rotate_done_sub = self.create_subscription(Int8, '/rotate_done', self.set_rotate_done, 1)  # Подписчик на сигнал о завершении поворота
        self.sign_sub = self.create_subscription(String, '/sign', self.handle_sign, 1)  # Подписчик на распознанные знаки
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.get_odom, 1)  # Подписчик на данные одометрии
        
        self.ID = 0  # Идентификатор ноды

        self.rotated = False  # Был ли робот повернут принудительно
        self.enable_detection = False  # Разрешена ли детекция знаков поворота

        # Скорости движения по перекрестку
        self.in_speed = self.declare_parameter('in_speed', 0.0).get_parameter_value().double_value
        self.speed_L = self.declare_parameter('speed_L', 0.0).get_parameter_value().double_value
        self.speed_R = self.declare_parameter('speed_R', 0.0).get_parameter_value().double_value
        self.out_speed = self.declare_parameter('out_speed', 0.0).get_parameter_value().double_value

        # Смещения
        self.in_offset = self.declare_parameter('in_offset', 0.0).get_parameter_value().double_value
        self.L_offset = self.declare_parameter('L_offset', 0.0).get_parameter_value().double_value
        self.R_offset = self.declare_parameter('R_offset', 0.0).get_parameter_value().double_value
        self.out_offset = self.declare_parameter('out_offset', 0.0).get_parameter_value().double_value       

        # Углы поворота
        self.angle_L = self.declare_parameter('angle_L', 0.0).get_parameter_value().double_value
        self.angle_R = self.declare_parameter('angle_R', 0.0).get_parameter_value().double_value

        # Линейные скорости при повороте
        self.linear_x_L = self.declare_parameter('linear_x_L', 0.0).get_parameter_value().double_value
        self.linear_x_R = self.declare_parameter('linear_x_R', 0.0).get_parameter_value().double_value

        # Угловые скорости при повороте
        self.angular_z_L = self.declare_parameter('angular_z_L', 0.0).get_parameter_value().double_value
        self.angular_z_R = self.declare_parameter('angular_z_R', 0.0).get_parameter_value().double_value         

    def handle_sign(self, msg):
        """Обработчик распознанных знаков."""
        
        cur_sign = msg.data

        if not self.rotated:

            # Действия при проезде знака перекрестка
            if cur_sign == 'intersection_sign':
                self.max_vel_pub.publish(Float64(data = self.in_speed))  # Устанавливаем скорость для проезда перекрестка
                self.offset_pub.publish(Float64(data = self.in_offset))  # Устанавливаем смещение для проезда перекрестка

            # Действия при повороте налево или направо
            if (cur_sign == 'turn_left_sign' or cur_sign == 'turn_right_sign') and self.enable_detection:

                self.enable_detection_pub.publish(Bool(data = False))  # Отключаем детекцию знаков
                self.enable_following_pub.publish(Bool(data = False))  # Отключаем следование по полосе

                self.rotated = True  # Устанавливаем флаг, что робот был повернут

                if cur_sign == 'turn_left_sign':
                    self.max_vel_pub.publish(Float64(data = self.speed_L))  # Устанавливаем скорость для поворота налево
                    self.offset_pub.publish(Float64(data = self.L_offset))  # Устанавливаем смещение для поворота налево
                    self.rotate_pub.publish(Rotate(angle = self.angle_L, linear_x = self.linear_x_L, angular_z = self.angular_z_L, id = self.ID))  # Отправляем команду поворота налево

                elif cur_sign == 'turn_right_sign':
                    self.max_vel_pub.publish(Float64(data = self.speed_R))  # Устанавливаем скорость для поворота направо
                    self.offset_pub.publish(Float64(data = self.R_offset))  # Устанавливаем смещение для поворота направо
                    self.rotate_pub.publish(Rotate(angle = self.angle_R, linear_x = self.linear_x_R, angular_z = self.angular_z_R, id = self.ID))  # Отправляем команду поворота направо

    def get_odom(self, msg):
        """Обработчик данных одометрии."""

        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y

        if pose_x <= 0.93 and pose_y >= 0.86:
            self.enable_detection = True  # Разрешаем детекцию знаков при входе на перекресток

        # Изменить скорость и смещение при выезде из перекрестка
        if pose_x <= -0.55:
            self.offset_pub.publish(Float64(data = self.out_offset))  # Устанавливаем смещение для выезда
            self.max_vel_pub.publish(Float64(data = self.out_speed))  # Устанавливаем скорость для выезда
            self.enable_detection_pub.publish(Bool(data = True))  # Включаем детекцию знаков
            self.get_logger().info("Узел Intersection_Handler З А В Е Р Ш И Л работу.")
            self.get_logger().info("Узел Intersection_Handler З А В Е Р Ш И Л работу.")
            self.get_logger().info("Узел Intersection_Handler З А В Е Р Ш И Л работу.")
            rclpy.shutdown()  # Завершаем работу узла

    def set_rotate_done(self, msg):
        """Обработчик сигнала о завершении поворота."""
        if msg.data == self.ID:
            self.enable_following_pub.publish(Bool(data = True))  # Включаем следование по полосе после поворота


def main():
    """Основная функция для запуска узла."""
    rclpy.init()  # Инициализация ROS 2

    node = Intersection_Handler()  # Создание экземпляра узла
    rclpy.spin(node)  # Запуск обработки сообщений
    
    node.destroy_node()  # Уничтожение узла после завершения работы
    rclpy.shutdown()  # Завершение работы ROS 2