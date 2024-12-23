import os

from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Получаем пути к пакетам для настройки робота
    pkg_autorace_core = get_package_share_directory('autorace_core_vROSyske')
    pkg_autorace_camera = get_package_share_directory('autorace_camera')
        
    # Загрузка конфигурационных файлов для различных нодов
    base_config = os.path.join(pkg_autorace_core, 'config', 'base.yaml')
    intersection_config = os.path.join(pkg_autorace_core, 'config', 'intersection.yaml')
    obstacles_config = os.path.join(pkg_autorace_core, 'config', 'obstacles.yaml')

    # Запуск калибровки камеры для корректного распознавания окружающей среды
    camera_calibration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_autorace_camera, 'launch', 'extrinsic_camera_calibration.launch.py')))


    # Нода для обнаружения полосы движения
    lane_detect = Node(
        package = 'autorace_core_vROSyske',
        executable = 'lane_detect',
        name = 'lane_detect',
        parameters = [base_config])
    
    # Нода для следования по полосе движения
    lane_follow = Node(
        package = 'autorace_core_vROSyske',
        executable = 'lane_follow',
        name = 'lane_follow',
        parameters = [base_config])
    
    # Нода для управления поворотом робота
    robot_rotator = Node(
        package = 'autorace_core_vROSyske',
        executable = 'robot_rotator',
        name = 'robot_rotator')
    
    # Нода для распознавания знаков
    sign_detection = Node(
        package = 'autorace_core_vROSyske',
        executable = 'sign_detection',
        name = 'sign_detection',
        parameters=[
            {'model_path': os.path.join(pkg_autorace_core, 'model')},  # Путь к модели для распознавания знаков
            base_config])

    # Нода для распознавания светофоров
    traffic_light = Node(
        package = 'autorace_core_vROSyske',
        executable = 'traffic_light',
        name = 'traffic_light')
    
    # Нода для обработки перекрестков
    intersection = Node(
        package = 'autorace_core_vROSyske',
        executable = 'intersection',
        name = 'intersection',
        parameters = [intersection_config])
    
    # Нода для обнаружения препятствий
    obstacles = Node(
        package = 'autorace_core_vROSyske',
        executable = 'obstacles',
        name = 'obstacles',
        parameters = [obstacles_config])
    
    # Описание запускаемой системы
    return LaunchDescription([
        camera_calibration,  # Запуск калибровки камеры

        # Постоянно работающие ноды
        lane_detect,     # Обнаружение полосы движения
        lane_follow,     # Следование по полосе
        robot_rotator,   # Управление поворотом
        sign_detection,  # Распознавание знаков

        # Ноды для прохождения конкретных испытаний
        traffic_light,   # Распознавание светофоров
        intersection,    # Проезд перекрестка
        obstacles,       # Проезд препятствий
    ])