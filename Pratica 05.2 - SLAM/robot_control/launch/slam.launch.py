import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NOISE_CASES = {
    'none':  (False, 'ground_truth'),
    'lidar': (True,  'ground_truth'),
    'both':  (True,  'wheels'),
}


def _setup(context):
    pkg = get_package_share_directory('robot_control')
    cfg = lambda name: os.path.join(pkg, 'config', name)       
    arg = lambda name: LaunchConfiguration(name).perform(context)   

    mode = arg('mode').lower()
    if mode not in ('mapping', 'localization'):
        raise RuntimeError(f"mode inválido: {mode} (use mapping ou localization)")
    case = arg('noise_case').lower()
    if case not in NOISE_CASES:
        raise RuntimeError(f"noise_case inválido: {case} (use {', '.join(NOISE_CASES)})")
    lidar_noisy, odom_source = NOISE_CASES[case]
    map_file = os.path.expanduser(arg('map_file'))
    if mode == 'localization' and not map_file:
        raise RuntimeError('mode:=localization exige map_file:=<caminho sem extensão> do '
                           'grafo de poses serializado (arquivos .posegraph e .data)')
    drive = arg('drive').lower() or ('mission' if mode == 'mapping' else 'none')
    if drive not in ('mission', 'teleop', 'none'):
        raise RuntimeError(f"drive inválido: {drive} (use mission, teleop ou none)")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={
            'slam': 'true',
            'gui': arg('gui'),
            'rviz': arg('rviz'),
            'world': arg('world') or os.path.join(pkg, 'worlds', 'slam_world.world'),
            'odom_source': odom_source,
            'noise_scale': '1.0',
            'lidar_noise': arg('lidar_noise') if lidar_noisy else '0.0',
            'odom_noise': arg('odom_noise'),
            'imu_noise': '0.0', 'gps_noise': '0.0', 'camera_noise': '0.0',   
        }.items())

    slam_params = {'use_sim_time': True, 'mode': mode}
    if mode == 'mapping':
        executable = 'async_slam_toolbox_node'
    else:
        executable = 'localization_slam_toolbox_node'
        slam_params.update({'map_file_name': map_file,
                            'map_start_at_dock': True})
    slam = Node(package='slam_toolbox', executable=executable, name='slam_toolbox',
                output='screen', parameters=[cfg('slam_toolbox.yaml'), slam_params])

    eval_params = [cfg('slam_nodes.yaml')]
    if arg('log_file'):          # string vazia não é repassada como override
        eval_params.append({'log_file': os.path.expanduser(arg('log_file'))})
    evaluator = Node(package='robot_control', executable='slam_eval', output='screen',
                     parameters=eval_params)

    drive_nodes = []
    if drive == 'mission':
        drive_nodes = [
            Node(package='robot_control', executable='pose_controller', output='screen',
                 parameters=[cfg('pose_controller.yaml')]),
            Node(package='robot_control', executable='mission_follower', output='screen',
                 parameters=[cfg('slam_mission.yaml')]),
        ]

    msgs = [LogInfo(msg=f'[05.2] modo={mode} | noise_case={case} (lidar '
                        f'{"COM" if lidar_noisy else "SEM"} ruído, odometria={odom_source}) | '
                        f'drive={drive}')]
    if drive == 'teleop':
        msgs.append(LogInfo(msg='[05.2] teleoperação, em OUTRO terminal: ros2 run '
                                'teleop_twist_keyboard teleop_twist_keyboard --ros-args '
                                '-r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped'))
    return msgs + [
        sim,
        TimerAction(period=8.0, actions=[slam, evaluator]),
        TimerAction(period=14.0, actions=drive_nodes),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping',
                              description='mapping | localization (usa mapa já salvo)'),
        DeclareLaunchArgument('noise_case', default_value='both',
                              description='none | lidar | both  (situações do Ex. 2.2)'),
        DeclareLaunchArgument('lidar_noise', default_value='3.0',
                              description='multiplicador do σ do lidar (base 0,01 m)'),
        DeclareLaunchArgument('odom_noise', default_value='1.0',
                              description='multiplicador do erro da odometria (ver slam_nodes.yaml)'),
        DeclareLaunchArgument('map_file', default_value='',
                              description='localization: grafo serializado, sem extensão'),
        DeclareLaunchArgument('drive', default_value='',
                              description='mission | teleop | none (vazio: mission no mapeamento, '
                                          'none na localização)'),
        DeclareLaunchArgument('log_file', default_value='',
                              description='CSV do slam_eval (erro vs ground truth)'),
        DeclareLaunchArgument('world', default_value='',
                              description='vazio = worlds/slam_world.world'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
