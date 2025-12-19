#!/usr/bin/env python3
"""
Gravity Enabler Node
Waits for specified delay, then enables gravity on the hip_Link
This ensures the pendulum starts at exactly 0 rad before dropping
"""

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
import time


class GravityEnabler(Node):
    def __init__(self):
        super().__init__('gravity_enabler')

        # Parameters
        self.declare_parameter('delay_seconds', 1.0)
        self.declare_parameter('entity_name', 'hip_link_test')
        self.declare_parameter('link_name', 'hip_Link')

        # Get parameters
        self.delay = self.get_parameter('delay_seconds').value
        self.entity_name = self.get_parameter('entity_name').value
        self.link_name = self.get_parameter('link_name').value

        self.get_logger().info(f'Gravity Enabler initialized')
        self.get_logger().info(f'  Entity: {self.entity_name}')
        self.get_logger().info(f'  Link: {self.link_name}')
        self.get_logger().info(f'  Delay: {self.delay}s')
        self.get_logger().info(f'  Gravity will be enabled after {self.delay}s')

        # Create timer to enable gravity after delay
        self.timer = self.create_timer(self.delay, self.enable_gravity_callback)
        self.gravity_enabled = False

    def enable_gravity_callback(self):
        """Enable gravity after delay"""
        if self.gravity_enabled:
            return

        try:
            # Use gz service to enable gravity
            import subprocess

            # Build the full entity name with link
            full_entity_name = f'{self.entity_name}::{self.link_name}'

            self.get_logger().info(f'⏰ Delay complete! Enabling gravity on {full_entity_name}...')

            # Call gz service to enable gravity
            # gz service -s /world/empty/set_gravity --reqtype gz.msgs.Gravity --reptype gz.msgs.Boolean --timeout 1000 --req "gravity: {x: 0, y: 0, z: -9.81}"
            # Note: We need to use entity-specific gravity control

            # Method 1: Use gz model command to set gravity
            cmd = f'gz model -m {self.entity_name} -l {self.link_name} -g true'
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

            if result.returncode == 0:
                self.get_logger().info(f'✅ Gravity enabled successfully!')
                self.get_logger().info(f'🚀 Pendulum released - free fall begins!')
                self.gravity_enabled = True
            else:
                self.get_logger().error(f'❌ Failed to enable gravity: {result.stderr}')
                self.get_logger().warn(f'Trying alternative method...')

                # Method 2: Alternative using topic
                # This publishes a message to enable gravity
                cmd2 = f'gz topic -t /model/{self.entity_name}/link/{self.link_name}/enable_gravity -m gz.msgs.Boolean -p "data: true"'
                result2 = subprocess.run(cmd2, shell=True, capture_output=True, text=True)

                if result2.returncode == 0:
                    self.get_logger().info(f'✅ Gravity enabled via topic!')
                    self.gravity_enabled = True
                else:
                    self.get_logger().error(f'❌ Failed with alternative method: {result2.stderr}')

            # Cancel timer
            self.timer.cancel()

        except Exception as e:
            self.get_logger().error(f'Exception enabling gravity: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    try:
        enabler = GravityEnabler()
        rclpy.spin(enabler)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
