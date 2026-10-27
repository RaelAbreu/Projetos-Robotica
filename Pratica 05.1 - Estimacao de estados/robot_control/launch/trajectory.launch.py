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
SIM_ARGS = {'noise_scale': '1.0', 'imu_noise': '1.0', 'gps_noise': '1.0',
            'lidar_noise': '1.0', 'camera_noise': '1.0', 'odom_noise': '1.0',
            'ekf': 'true', 'rviz': 'false'}


def _setup(context):
    pkg = get_package_share_directory('robot_control')
    over = {}
    for key, cast in OVERRIDES.items():
        val = LaunchConfiguration(key).perform(context)
        if val != '':
            over[key] = cast(val)

    source = LaunchConfiguration('state_source').perform(context).lower()
    if source == 'sensors':
        over['odom_topic'] = '/estimated_odom'
    elif source == 'wheels':
        over['odom_topic'] = '/diff_drive_controller/odom'
        over['frame_id'] = 'odom'
    elif source != 'ekf':
        raise RuntimeError(f"state_source inválido: {source} (use ekf, sensors ou wheels)")
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
        DeclareLaunchArgument('state_source', default_value='ekf',
                              description='estado usado no controle: ekf | sensors | wheels'),
        *[DeclareLaunchArgument(k, default_value=v) for k, v in SIM_ARGS.items()],
        DeclareLaunchArgument('auto_exit', default_value='false',
                              description='encerra tudo ao fim da trajetória'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui'),
                              **{k: LaunchConfiguration(k) for k in SIM_ARGS}}.items()),
        OpaqueFunction(function=_setup),
    ])
