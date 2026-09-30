import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aluno',
    maintainer_email='aluno@todo.todo',
    description='Controle de pose, missão por waypoints e trajetória em 8 (Prática 03).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_controller = robot_control.pose_controller_node:main',
            'pose_controller_3m = robot_control.pose_controller_node:main_3m',
            'mission_follower = robot_control.mission_follower_node:main',
            'trajectory_follower = robot_control.trajectory_follower_node:main',
        ],
    },
)
