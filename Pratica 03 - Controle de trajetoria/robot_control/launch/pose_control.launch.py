import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    pkg = get_package_share_directory('robot_control')
    ctrl = LaunchConfiguration('controller').perform(context)
    mission = LaunchConfiguration('mission').perform(context).lower() == 'true'
    log = LaunchConfiguration('log_file').perform(context)
    exe, name = (('pose_controller', 'pose_controller') if ctrl == 'continuous'
                 else ('pose_controller_3m', 'pose_controller_3m'))
    actions = [Node(package='robot_control', executable=exe, name=name,
                    output='screen',
                    parameters=[os.path.join(pkg, 'config', 'pose_controller.yaml')])]
    if mission:
        params = [os.path.join(pkg, 'config', 'mission.yaml')]
        if log:
            params.append({'log_file': log})
        actions.append(Node(package='robot_control', executable='mission_follower',
                            output='screen', parameters=params))
    return [TimerAction(period=8.0, actions=actions)]  # espera os controladores subirem


def generate_launch_description():
    pkg = get_package_share_directory('robot_control')
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='continuous',
                              description='continuous | three_maneuvers'),
        DeclareLaunchArgument('mission', default_value='false'),
        DeclareLaunchArgument('log_file', default_value=''),
        DeclareLaunchArgument('gui', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui')}.items()),
        OpaqueFunction(function=_setup),
    ])
