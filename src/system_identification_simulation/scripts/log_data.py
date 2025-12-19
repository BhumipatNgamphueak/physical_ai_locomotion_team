#!/usr/bin/env python3
"""
Joint State Data Collector for Simulation
Logs position, velocity, and effort from /joint_states topic
Saves to timestamped CSV file for comparison with real experiments
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv
import os
from datetime import datetime
import numpy as np


class JointStateLogger(Node):
    def __init__(self):
        super().__init__('joint_state_logger')
        
        # Parameters
        self.declare_parameter('output_dir', 'sim_signal')
        self.declare_parameter('joint_name', 'world_to_hip')
        self.declare_parameter('duration', 20.0)  # Duration in seconds
        self.declare_parameter('log_rate', 0.0)   # 0 = log every message, >0 = specific rate
        
        # Get parameters
        self.output_dir = self.get_parameter('output_dir').value
        self.joint_name = self.get_parameter('joint_name').value
        self.duration = self.get_parameter('duration').value
        self.log_rate = self.get_parameter('log_rate').value
        
        # State
        self.start_time = None
        self.last_log_time = 0.0
        self.data_buffer = []
        self.message_count = 0
        
        # Generate filename with joint name for easy recognition
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        # Clean joint name for filename (replace special chars with underscore)
        clean_joint_name = self.joint_name.replace('/', '_').replace('\\', '_')
        self.output_file = os.path.join(
            self.output_dir,
            f'{clean_joint_name}_sim_{timestamp}.csv'
        )
        
        # Create output directory if needed
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Open CSV file
        self.csv_file = open(self.output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write header
        self.csv_writer.writerow([
            'timestamp',
            'sim_time_sec',
            'position_rad',
            'velocity_rad_s',
            'effort_Nm',
            'acceleration_rad_s2'
        ])
        
        # Previous state for acceleration calculation
        self.prev_velocity = None
        self.prev_time = None

        # Simulation time offset (will be set from first message)
        self.sim_time_offset = None

        # Subscription
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            100  # Queue size
        )
        
        # Timer to check duration
        self.timer = self.create_timer(0.1, self.check_duration)
        
        self.get_logger().info(f'Joint State Logger started')
        self.get_logger().info(f'Joint name: {self.joint_name}')
        self.get_logger().info(f'Output file: {self.output_file}')
        self.get_logger().info(f'Duration: {self.duration}s')
        if self.log_rate > 0:
            self.get_logger().info(f'Log rate: {self.log_rate} Hz')
        else:
            self.get_logger().info(f'Log rate: Every message')

    def joint_state_callback(self, msg):
        """Process incoming joint state messages"""
        
        # Find our joint in the message
        try:
            joint_idx = msg.name.index(self.joint_name)
        except ValueError:
            # Joint not in this message
            return
        
        # Get current time
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Initialize start time on first message
        if self.start_time is None:
            self.start_time = current_time
            self.get_logger().info('Started logging data!')
        
        # Calculate elapsed time
        elapsed = current_time - self.start_time
        
        # Check if we should log this message (rate limiting)
        if self.log_rate > 0:
            time_since_last_log = current_time - self.last_log_time
            min_interval = 1.0 / self.log_rate
            if time_since_last_log < min_interval:
                return  # Skip this message
        
        # Extract data
        position = msg.position[joint_idx] if joint_idx < len(msg.position) else 0.0
        velocity = msg.velocity[joint_idx] if joint_idx < len(msg.velocity) else 0.0
        effort = msg.effort[joint_idx] if joint_idx < len(msg.effort) else 0.0
        
        # Calculate acceleration (numerical derivative of velocity)
        acceleration = 0.0
        if self.prev_velocity is not None and self.prev_time is not None:
            dt = current_time - self.prev_time
            if dt > 1e-6:  # Avoid division by very small numbers
                acceleration = (velocity - self.prev_velocity) / dt
        
        # Update previous values
        self.prev_velocity = velocity
        self.prev_time = current_time

        # Get simulation time from message header
        sim_time_raw = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        # Initialize sim_time_offset from first message to start at 0
        if self.sim_time_offset is None:
            self.sim_time_offset = sim_time_raw
            self.get_logger().info(f'Sim time offset set to: {self.sim_time_offset:.3f}s (data will start at t=0)')

        # Reset sim_time to start from 0
        sim_time = sim_time_raw - self.sim_time_offset

        # Write to CSV
        self.csv_writer.writerow([
            current_time,
            sim_time,
            position,
            velocity,
            effort,
            acceleration
        ])
        
        # Flush periodically
        self.message_count += 1
        if self.message_count % 100 == 0:
            self.csv_file.flush()
            self.get_logger().info(
                f'Logged {self.message_count} messages | '
                f't={elapsed:.2f}s | '
                f'pos={position:.3f} rad | '
                f'vel={velocity:.3f} rad/s'
            )
        
        self.last_log_time = current_time

    def check_duration(self):
        """Check if duration has elapsed"""
        if self.start_time is None:
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time
        
        if elapsed >= self.duration:
            self.get_logger().info(f'Duration reached: {elapsed:.2f}s')
            self.get_logger().info(f'Total messages logged: {self.message_count}')
            self.get_logger().info(f'Data saved to: {self.output_file}')
            self.cleanup()
            rclpy.shutdown()

    def cleanup(self):
        """Close file and cleanup"""
        if hasattr(self, 'csv_file'):
            self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        logger = JointStateLogger()
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            logger.get_logger().info('Shutting down...')
            logger.cleanup()
            rclpy.shutdown()


if __name__ == '__main__':
    main()