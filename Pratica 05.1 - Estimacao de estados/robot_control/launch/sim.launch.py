import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NOISE_ARGS = ['noise_scale', 'imu_noise', 'gps_noise', 'lidar_noise', 'camera_noise']
EXTRA_NOISE_ARGS = ['odom_noise']


def _setup(context):
    pkg = get_package_share_directory('robot_control')
    cfg = os.path.join(pkg, 'config', 'sensors.yaml')
    sim_time = {'use_sim_time': True}

    noise = {k: LaunchConfiguration(k).perform(context) for k in NOISE_ARGS}
    robot_description = xacro.process_file(
        os.path.join(pkg, 'urdf', 'my_robot.urdf.xacro'), mappings=noise).toxml()

    robot_description = re.sub(r'<!--.*?-->', '', robot_description, flags=re.S)

    # --- Gazebo + robô ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui'),
                          'world': LaunchConfiguration('world')}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-entity', 'my_robot', '-topic', 'robot_description',
                            '-x', '0.0', '-y', '0.0', '-z', '0.1'])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager-timeout', '60'])

    jsb, ddc = spawner('joint_state_broadcaster'), spawner('diff_drive_controller')


    map_to_odom = Node(package='tf2_ros', executable='static_transform_publisher',
                       name='map_to_odom_tf',
                       arguments=['--x', '0', '--y', '0', '--z', '0',
                                  '--roll', '0', '--pitch', '0', '--yaw', '0',
                                  '--frame-id', 'map', '--child-frame-id', 'odom'],
                       parameters=[sim_time],
                       condition=UnlessCondition(LaunchConfiguration('ekf')))

    # --- processamento dos sensores ---
    ns = float(noise['noise_scale'])
    imu_scale = ns * float(noise['imu_noise'])
    gps_scale = ns * float(noise['gps_noise'])
    odom_scale = ns * float(LaunchConfiguration('odom_noise').perform(context))
    gps_to_xyz = Node(package='robot_control', executable='gps_to_xyz',
                      output='screen', parameters=[cfg, {'noise_scale': gps_scale}])
    imu_noise = Node(package='robot_control', executable='imu_noise',
                     output='screen', parameters=[cfg, {'noise_scale': imu_scale}])
    wheel_noise = Node(package='robot_control', executable='wheel_odom_noise',
                       output='screen', parameters=[cfg, {'noise_scale': odom_scale}])
    ekf = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
               output='screen', parameters=[os.path.join(pkg, 'config', 'ekf.yaml')],
               condition=IfCondition(LaunchConfiguration('ekf')))
    estimator = Node(package='robot_control', executable='sensor_pose_estimator',
                     output='screen', parameters=[cfg, sim_time])

    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', os.path.join(pkg, 'rviz', 'sensors.rviz')],
                parameters=[sim_time], condition=IfCondition(LaunchConfiguration('rviz')))

    return [
        gazebo, rsp, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[ddc])),
        map_to_odom, gps_to_xyz, imu_noise, wheel_noise, estimator, ekf, rviz,
    ]


def generate_launch_description():
    pkg = get_package_share_directory('robot_control')
    args = [DeclareLaunchArgument('gui', default_value='true'),
            DeclareLaunchArgument('rviz', default_value='false',
                                  description='abre o RViz com a config da Prática 04'),
            DeclareLaunchArgument('world', default_value=os.path.join(
                pkg, 'worlds', 'sensor_world.world'))]
    args += [DeclareLaunchArgument(k, default_value='1.0',
                                   description='multiplicador do ruído dos sensores')
             for k in NOISE_ARGS + EXTRA_NOISE_ARGS]
    args.append(DeclareLaunchArgument('ekf', default_value='true',
                                      description='liga o filtro de Kalman (robot_localization)'))
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
