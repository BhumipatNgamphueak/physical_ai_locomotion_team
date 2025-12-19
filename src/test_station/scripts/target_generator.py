#!/usr/bin/env python3
"""
Target Generator Node - SKELETON (Test Station)
Generates desired end effector positions for single leg testing
INPUT: /target_pose (PointStamped - from GUI/ROS2 topic)
OUTPUT: /end_effector_target (PointStamped - desired foot position)
OUTPUT: /end_effector_velocity (Vector3Stamped - desired foot velocity)
Frequency: 50 Hz
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Vector3Stamped
import numpy as np


class TargetGenerator(Node):
    def __init__(self):
        super().__init__('target_generator')
        
        # Parameters
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('default_x', 0.15)
        self.declare_parameter('default_y', 0.0)
        self.declare_parameter('default_z', -0.20)
        self.declare_parameter('trajectory_type', 'circle')  # circle, line, point
        
        update_rate = self.get_parameter('update_rate').value
        
        # INPUT: Subscribe to target commands
        self.target_sub = self.create_subscription(
            PointStamped,
            'target_pose',
            self.target_callback,
            10
        )
        
        # OUTPUT: Publishers
        self.target_pub = self.create_publisher(
            PointStamped,
            'end_effector_target',
            10
        )
        self.velocity_pub = self.create_publisher(
            Vector3Stamped,
            'end_effector_velocity',
            10
        )
        
        # State
        self.current_target = np.array([
            self.get_parameter('default_x').value,
            self.get_parameter('default_y').value,
            self.get_parameter('default_z').value
        ])
        
        # Timer - 50 Hz
        self.timer = self.create_timer(1.0 / update_rate, self.publish_target)
        
        self.get_logger().info('Target Generator initialized')
    
    def target_callback(self, msg):
        """INPUT: Receive target position from user/GUI"""
        # TODO: Store target position
        # TODO: Optionally interpolate to new target
        pass
    
    def publish_target(self):
        """OUTPUT: Publish target position - 50 Hz"""
        # TODO: Generate trajectory (circle, line, or fixed point)
        # TODO: Compute velocity
        # TODO: Publish target and velocity
        pass


def main(args=None):
    rclpy.init(args=args)
    node = TargetGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()