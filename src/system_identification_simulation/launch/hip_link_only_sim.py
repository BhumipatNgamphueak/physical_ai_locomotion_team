#!/usr/bin/env python3
"""
Hip Link Only - Simulation Launch File
Simple pendulum: just hip_Link hanging from world joint
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
    
    desc_pkg = get_package_share_directory('system_identification_description')
    sim_pkg = get_package_share_directory('system_identification_simulation')
    install_share_dir = os.path.dirname(desc_pkg)
    
    # Gazebo resource paths
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

    # XACRO file (hip only)
    xacro_file = PathJoinSubstitution([
        FindPackageShare('system_identification_description'),
        'robot', 'visual', 'hip_link_only.xacro'
    ])
    
    # Controller config
    controller_config = PathJoinSubstitution([
        FindPackageShare('system_identification_simulation'),
        'config', 'hip_pendulum_controller.yaml'
    ])
    
    # Process xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' use_sim_time:=', use_sim_time,
        ' controller_config_file:=', controller_config
    ])
    
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    # Robot State Publisher
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

    # Spawn Entity (with delay to ensure Gazebo is ready)
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_hip_link',
                output='screen',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'hip_link_test',
                    '-allow_renaming', 'true',
                    '-x', '0.0', '-y', '0.0', '-z', '0.0'
                ]
            )
        ]
    )

    # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Pendulum Controller
    pendulum_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='hip_pendulum_controller_spawner',
        output='screen',
        arguments=['hip_pendulum_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz (optional)
    rviz_config_file = os.path.join(sim_pkg, 'rviz', 'display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Gravity Enabler - enables gravity after delay for clean start
    gravity_enabler = Node(
        package='system_identification_simulation',
        executable='gravity_enabler.py',
        name='gravity_enabler',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'delay_seconds': 1.0},  # Wait 1 second before enabling gravity
            {'entity_name': 'hip_link_test'},
            {'link_name': 'hip_Link'}
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
        TimerAction(period=5.0, actions=[joint_state_broadcaster]),
        TimerAction(period=7.0, actions=[pendulum_controller]),
        TimerAction(period=8.0, actions=[gravity_enabler]),  # Enable gravity after controllers ready
        rviz
    ])
