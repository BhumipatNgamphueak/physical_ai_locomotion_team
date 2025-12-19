#!/usr/bin/env python3
"""
Test Station Simulation Launch File - FIXED VERSION
Fixes:
1. Controller configuration path handling
2. Proper event handling for controller spawning
3. Correct controller names matching YAML
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, 
    FindExecutable, 
    LaunchConfiguration, 
    PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Use simulated time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Package shares
    desc_pkg = get_package_share_directory('test_station_description')
    sim_pkg = get_package_share_directory('test_station_simulation')

    # Get the parent directory (install/share) to make model:// URIs work
    install_share_dir = os.path.dirname(desc_pkg)
    
    # Set Gazebo resource paths
    gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_share_dir + ':' + 
              os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )
    
    ign_model_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=install_share_dir + ':' + 
              os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    )

    # Robot description (xacro)
    xacro_file = PathJoinSubstitution([
        FindPackageShare('test_station_description'), 
        'robot',
        'visual',
        'test_station.xacro'
    ])
    
    # IMPORTANT: Pass the controller config path as xacro parameter
    controller_config = PathJoinSubstitution([
        FindPackageShare('test_station_simulation'),
        'config',
        'controller_full.yaml'
    ])
    
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' use_sim_time:=', use_sim_time,
        ' controller_config_file:=', controller_config  # Pass to xacro
    ])
    
    robot_description_param = ParameterValue(robot_description_content, value_type=str)

    # Gazebo simulation launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 
                         'launch', 
                         'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    # Robot state publisher
    state_pub = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher',
        name='robot_state_publisher', 
        output='screen',
        parameters=[
            {'robot_description': robot_description_param},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Spawn entity in Gazebo
    # Height calculated to place base_link bottom at ground level (z=0)
    # while keeping leg elevated above ground
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create', 
        name='spawn_entity', 
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'test_station',
            '-allow_renaming', 'true',
            '-x', '0.0', 
            '-y', '0.0', 
            '-z', '0.01'  # Base sits on ground, leg elevated at ~0.31m
        ]
    )

    # Controller manager spawners
    # 1. Joint State Broadcaster (spawn after robot appears)
    jsb_spawner = Node(
        package='controller_manager', 
        executable='spawner', 
        name='spawner_joint_state_broadcaster',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 2. Effort Controller (spawn after JSB is ready)
    effort_spawner = Node(
        package='controller_manager', 
        executable='spawner', 
        name='spawner_leg_effort_controller',
        arguments=[
            'leg_effort_controller',
            '--controller-manager', '/controller_manager'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ROS <-> Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge', 
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz (optional)
    rviz_config = os.path.join(sim_pkg, 'rviz', 'display.rviz')
    if os.path.exists(rviz_config):
        rviz = Node(
            package='rviz2', 
            executable='rviz2', 
            name='rviz', 
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    else:
        rviz = Node(
            package='rviz2', 
            executable='rviz2', 
            name='rviz', 
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        gz_model_path,
        ign_model_path,
        gz_sim,
        state_pub,
        spawn_entity,
        bridge,
        
        # Use TimerAction instead of OnProcessExit for more reliable spawning
        TimerAction(
            period=3.0,  # Wait 3 seconds after spawn
            actions=[jsb_spawner]
        ),
        TimerAction(
            period=5.0,  # Wait 5 seconds total before effort controller
            actions=[effort_spawner]
        ),
        
        rviz
    ])