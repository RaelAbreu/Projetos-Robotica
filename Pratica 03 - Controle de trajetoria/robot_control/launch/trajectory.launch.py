import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

OVERRIDES = {'mode': str, 'k_ff': float, 'omega': float, 'A': float, 'B': float,
             'n_periods': float, 'log_file': str}


def _setup(context):
    pkg = get_package_share_directory('robot_control')
    over = {}
    for key, cast in OVERRIDES.items():
        val = LaunchConfiguration(key).perform(context)
        if val != '':
            over[key] = cast(val)
    auto_exit = LaunchConfiguration('auto_exit').perform(context).lower() == 'true'
    if auto_exit:
        over['shutdown_on_finish'] = True
    node = Node(package='robot_control', executable='trajectory_follower',
                output='screen',
                parameters=[os.path.join(pkg, 'config', 'trajectory.yaml'), over])
    actions = [TimerAction(period=8.0, actions=[node])]
    if auto_exit:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=[EmitEvent(event=Shutdown())])))
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('robot_control')
    args = [DeclareLaunchArgument(k, default_value='') for k in OVERRIDES]
    return LaunchDescription(args + [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('auto_exit', default_value='false',
                              description='encerra tudo ao fim da trajetória'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui')}.items()),
        OpaqueFunction(function=_setup),
    ])
