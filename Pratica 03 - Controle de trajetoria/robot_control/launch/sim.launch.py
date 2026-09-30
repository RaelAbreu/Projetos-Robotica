import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('robot_control'),
                              'urdf', 'my_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen',
               parameters=[{'robot_description': robot_description,
                            'use_sim_time': True}])

    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-entity', 'my_robot', '-topic', 'robot_description',
                            '-x', '0.0', '-y', '0.0', '-z', '0.1'])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager-timeout', '60'])

    jsb, ddc = spawner('joint_state_broadcaster'), spawner('diff_drive_controller')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        gazebo, rsp, spawn,
        # encadeia: spawn -> joint_state_broadcaster -> diff_drive_controller
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[ddc])),
    ])