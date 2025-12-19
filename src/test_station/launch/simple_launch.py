#!/usr/bin/env python3
"""
Test Station Control Launch File - FIXED VERSION
Single leg control system for test station

Fixes:
1. Corrected executable names (should match Python file names)
2. Fixed topic remapping to match controller output
3. Proper namespace handling
4. Correct sequencing of trajectory nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    nodes_to_launch = []
    
    # =================================================================
    # TARGET GENERATION
    # =================================================================
    
    # Target Generator - generates desired end effector positions
    nodes_to_launch.append(
        Node(
            package='test_station',
            executable='target_generator.py',  # FIXED: match actual filename
            name='target_generator',
            namespace='test_station',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'update_rate': 50.0,
                'default_x': 0.15,
                'default_y': 0.0,
                'default_z': -0.20,
                'trajectory_type': 'circle',
            }],
            remappings=[
                # Input (optional - from GUI or topic)
                ('target_pose', '/test_station/target_pose'),
                # Output
                ('end_effector_target', '/test_station/end_effector_setpoint'),  # FIXED
                ('end_effector_velocity', '/test_station/end_effector_velocity_raw'),
            ],
            output='screen'
        )
    )
    
    # =================================================================
    # TRAJECTORY PLANNING
    # =================================================================
    
    # Trajectory Generator - smooth interpolation
    nodes_to_launch.append(
        Node(
            package='test_station',
            executable='trajectory_planning.py',  # FIXED: match actual filename
            name='trajectory_generator',
            namespace='test_station',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'interpolation_method': 'cubic',
                'trajectory_rate': 100.0,
                'swing_clearance': 0.03,
            }],
            remappings=[
                # Input
                ('end_effector_setpoint', '/test_station/end_effector_setpoint'),
                # Output (smoothed)
                ('end_effector_target', '/test_station/end_effector_target_smooth'),
                ('end_effector_velocity', '/test_station/end_effector_velocity_smooth'),
            ],
            output='screen'
        )
    )
    
    # =================================================================
    # INVERSE KINEMATICS
    # =================================================================
    
    # Inverse Position Kinematics
    nodes_to_launch.append(
        Node(
            package='test_station',
            executable='inverse_position_kinematic.py',  # FIXED: match actual filename
            name='inverse_kinematics',
            namespace='test_station',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_analytical_ik': True,
                'max_iterations': 100,
                'tolerance': 0.001,
            }],
            remappings=[
                # Input
                ('joint_states', '/joint_states'),
                ('end_effector_target', '/test_station/end_effector_target_smooth'),
                # Output
                ('joint_position_target', '/test_station/joint_position_target'),
            ],
            output='screen'
        )
    )
    
    # Inverse Velocity Kinematics
    nodes_to_launch.append(
        Node(
            package='test_station',
            executable='inverse_velocity_kinematic.py',  # FIXED: match actual filename
            name='inverse_velocity_kinematics',
            namespace='test_station',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'update_rate': 100.0,
            }],
            remappings=[
                # Input
                ('joint_states', '/joint_states'),
                ('end_effector_velocity', '/test_station/end_effector_velocity_smooth'),
                # Output
                ('joint_velocity_feedforward', '/test_station/joint_velocity_feedforward'),
            ],
            output='screen'
        )
    )
    
    # =================================================================
    # CONTROL LOOPS
    # =================================================================
    
    # Position PID Controller
    nodes_to_launch.append(
        Node(
            package='test_station',
            executable='pid_position_controller.py',  # FIXED: match actual filename
            name='position_pid_controller',
            namespace='test_station',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'position_kp': [10.0, 10.0, 10.0],
                'position_ki': [0.1, 0.1, 0.1],
                'position_kd': [1.0, 1.0, 1.0],
                'velocity_limit': 5.0,
                'control_rate': 100.0,
            }],
            remappings=[
                # Input
                ('joint_states', '/joint_states'),
                ('joint_position_target', '/test_station/joint_position_target'),
                # Output
                ('joint_velocity_target', '/test_station/joint_velocity_target'),
            ],
            output='screen'
        )
    )
    
    # Velocity PID Controller
    nodes_to_launch.append(
        Node(
            package='test_station',
            executable='pid_velocity_controller.py',  # FIXED: match actual filename
            name='velocity_pid_controller',
            namespace='test_station',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'velocity_kp': [5.0, 5.0, 5.0],
                'velocity_ki': [0.5, 0.5, 0.5],
                'velocity_kd': [0.1, 0.1, 0.1],
                'effort_limit': 10.0,
                'control_rate': 100.0,
            }],
            remappings=[
                # Input
                ('joint_states', '/joint_states'),
                ('joint_velocity_target', '/test_station/joint_velocity_target'),
                ('joint_velocity_feedforward', '/test_station/joint_velocity_feedforward'),
                # Output (to Gazebo controller) - FIXED
                ('commands', '/leg_effort_controller/commands'),
            ],
            output='screen'
        )
    )
    
    # =================================================================
    # MONITORING (Optional)
    # =================================================================
    
    # Forward Kinematics - for monitoring actual foot position
    nodes_to_launch.append(
        Node(
            package='test_station',
            executable='forward_position_kinematic.py',  # FIXED: match actual filename
            name='forward_kinematics',
            namespace='test_station',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'update_rate': 100.0,
            }],
            remappings=[
                # Input
                ('joint_states', '/joint_states'),
                # Output
                ('end_effector_position', '/test_station/end_effector_position'),
            ],
            output='screen'
        )
    )
    
    # Return launch description
    return LaunchDescription([
        use_sim_time_arg,
        *nodes_to_launch
    ])