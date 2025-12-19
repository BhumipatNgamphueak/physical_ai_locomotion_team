#!/usr/bin/env python3
"""
Knee Link Only - Simulation Launch File
Simple pendulum: just knee_link hanging from world joint
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    desc_pkg = get_package_share_directory('test_station_description')
    sim_pkg = get_package_share_directory('test_station_simulation')
    install_share_dir = os.path.dirname(desc_pkg)
    
    gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_share_dir + ':' + os.path.join(desc_pkg, 'meshes') + ':' +
              os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )
    
    ign_model_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=install_share_dir + ':' + os.path.join(desc_pkg, 'meshes') + ':' +
              os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    )

    xacro_file = PathJoinSubstitution([
        FindPackageShare('test_station_description'),
        'robot', 'visual', 'knee_link_only.xacro'
    ])
    
    controller_config = PathJoinSubstitution([
        FindPackageShare('test_station_simulation'),
        'config', 'knee_pendulum_controller.yaml'
    ])
    
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' use_sim_time:=', use_sim_time,
        ' controller_config_file:=', controller_config
    ])
    
    robot_description = ParameterValue(robot_description_content, value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_knee_link',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'knee_link_test',
            '-allow_renaming', 'true',
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ]
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    pendulum_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='knee_pendulum_controller_spawner',
        output='screen',
        arguments=['knee_pendulum_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rviz_config_file = os.path.join(sim_pkg, 'rviz', 'display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Data Logger - starts automatically with small delay
    data_logger = Node(
        package='test_station_simulation',
        executable='log_data.py',
        name='data_logger',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'joint_name': 'world_to_knee'},
            {'duration': 5.0},
            {'output_dir': 'sim_signal'}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        gz_model_path,
        ign_model_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        TimerAction(period=0.0, actions=[joint_state_broadcaster]),
        TimerAction(period=0.0, actions=[pendulum_controller]),
        rviz
    ])
