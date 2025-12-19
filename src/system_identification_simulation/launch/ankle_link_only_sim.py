#!/usr/bin/env python3
"""
Ankle Pendulum Test - With Proper Joint States Bridge
FIXES: Bridges Gazebo /joint_states to ROS2 /joint_states
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    desc_pkg = get_package_share_directory('test_station_description')
    install_share_dir = os.path.dirname(desc_pkg)
    
    # Gazebo resource paths
    gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_share_dir + ':' + 
              os.path.join(desc_pkg, 'meshes') + ':' +
              os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )
    
    ign_model_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=install_share_dir + ':' + 
              os.path.join(desc_pkg, 'meshes') + ':' +
              os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    )

    # Robot description XACRO file
    xacro_file = PathJoinSubstitution([
        FindPackageShare('test_station_description'),
        'robot',
        'visual',
        'ankle_pendulum_complete.xacro'
    ])
    
    # Process xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file
    ])
    
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )

        # Spawn entity
    # Initial position matches real data: -0.523843 rad (-30 degrees from horizontal)
    # Coordinate system: 0 = horizontal, -π/2 = vertical down
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'ankle_test',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3',
            '-J', 'world_to_ankle', '-0.523843'  # -30 degrees from horizontal
        ]
    )

    # ========================================
    # ROS-Gazebo Bridge - CRITICAL FIX!
    # Bridge BOTH clock AND joint_states
    # ========================================
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'  # ✅ ADD THIS!
        ],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gz_model_path,
        ign_model_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge
    ])