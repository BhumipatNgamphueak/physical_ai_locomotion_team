#!/usr/bin/env python3
"""
Pendulum Release Controller
Holds the pendulum at 0 rad using effort control, then releases it after a delay
This ensures clean starting conditions for comparison with real experiments
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np


class PendulumReleaseController(Node):
    def __init__(self):
        super().__init__('pendulum_release_controller')

        # Parameters
        self.declare_parameter('hold_duration', 2.0)  # How long to hold before release
        self.declare_parameter('joint_name', 'world_to_hip')
        self.declare_parameter('target_position', 0.0)  # Hold at 0 rad (horizontal)
        self.declare_parameter('kp', 50.0)  # Proportional gain
        self.declare_parameter('kd', 5.0)   # Derivative gain

        # Get parameters
        self.hold_duration = self.get_parameter('hold_duration').value
        self.joint_name = self.get_parameter('joint_name').value
        self.target_position = self.get_parameter('target_position').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value

        # State
        self.start_time = None
        self.released = False
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.joint_states_received = False

        # Publishers and Subscribers
        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            '/hip_pendulum_controller/commands',
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Control timer (100 Hz)
        self.control_timer = self.create_timer(0.01, self.control_loop)

        self.get_logger().info('='*60)
        self.get_logger().info('Pendulum Release Controller Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'  Joint: {self.joint_name}')
        self.get_logger().info(f'  Target position: {self.target_position:.3f} rad ({np.degrees(self.target_position):.1f}°)')
        self.get_logger().info(f'  Hold duration: {self.hold_duration:.1f}s')
        self.get_logger().info(f'  Control gains: Kp={self.kp:.1f}, Kd={self.kd:.1f}')
        self.get_logger().info('='*60)
        self.get_logger().info('🔒 HOLDING pendulum at target position...')
        self.get_logger().info(f'   Will release after {self.hold_duration:.1f}s')
        self.get_logger().info('='*60)

    def joint_state_callback(self, msg):
        """Update current joint state"""
        try:
            idx = msg.name.index(self.joint_name)
            self.current_position = msg.position[idx]
            self.current_velocity = msg.velocity[idx] if idx < len(msg.velocity) else 0.0

            if not self.joint_states_received:
                self.joint_states_received = True
                self.start_time = self.get_clock().now()
                self.get_logger().info(f'✓ Joint states received - starting hold control')
        except ValueError:
            pass

    def control_loop(self):
        """Main control loop - holds position then releases"""

        if not self.joint_states_received or self.start_time is None:
            return

        # Calculate elapsed time
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # Create command message
        effort_msg = Float64MultiArray()

        if elapsed < self.hold_duration:
            # HOLD MODE: Apply PD control to hold at target position
            position_error = self.target_position - self.current_position
            velocity_error = 0.0 - self.current_velocity

            # PD controller
            effort = self.kp * position_error + self.kd * velocity_error

            # Limit effort
            max_effort = 50.0
            effort = np.clip(effort, -max_effort, max_effort)

            effort_msg.data = [effort]

            # Log status every second
            if int(elapsed) != int(elapsed - 0.01):  # New second
                remaining = self.hold_duration - elapsed
                self.get_logger().info(
                    f'🔒 HOLDING | t={elapsed:.1f}s | '
                    f'pos={self.current_position:.4f} rad ({np.degrees(self.current_position):.1f}°) | '
                    f'err={position_error:.4f} rad | effort={effort:.2f} Nm | '
                    f'release in {remaining:.1f}s'
                )
        else:
            # RELEASE MODE: Set effort to 0
            if not self.released:
                self.get_logger().info('='*60)
                self.get_logger().info('⏰ RELEASE TIME!')
                self.get_logger().info('='*60)
                self.get_logger().info(f'🚀 PENDULUM RELEASED at t={elapsed:.1f}s')
                self.get_logger().info(f'   Release position: {self.current_position:.4f} rad ({np.degrees(self.current_position):.1f}°)')
                self.get_logger().info(f'   Release velocity: {self.current_velocity:.4f} rad/s')
                self.get_logger().info('='*60)
                self.get_logger().info('📊 FREE PENDULUM MOTION - Logging clean data!')
                self.get_logger().info('='*60)
                self.released = True

            # Apply zero effort (free swing)
            effort_msg.data = [0.0]

        # Publish command
        self.effort_pub.publish(effort_msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        controller = PendulumReleaseController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
