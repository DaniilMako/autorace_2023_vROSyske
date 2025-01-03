import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'autorace_core_vROSyske'

data_files = []

start_point = os.path.join('model')
for root, dirs, files in os.walk(start_point):
    root_files = [os.path.join(root, i) for i in files]
    data_files.append((os.path.join('share', package_name, root), root_files))

data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append((os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))))
data_files.append((os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))))
data_files.append((os.path.join('share', package_name, 'model'), glob(os.path.join('model', '*.pt'))))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DaniilMako',
    maintainer_email='d.makovetskii@g.nsu.ru',
    description='Package controlling the robot at autorace 2023',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "lane_detect = autorace_core_vROSyske.lane_detect:main",
            "lane_follow = autorace_core_vROSyske.lane_follow:main",
            "sign_detection = autorace_core_vROSyske.sign_detection:main",
            "robot_rotator = autorace_core_vROSyske.robot_rotator:main",
            "traffic_light = autorace_core_vROSyske.traffic_light:main",
            "intersection = autorace_core_vROSyske.intersection:main",
            "obstacles = autorace_core_vROSyske.obstacles:main",
        ],
    },
)
