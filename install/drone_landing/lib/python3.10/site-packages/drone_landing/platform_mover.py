#!/usr/bin/env python3
"""
Platform Mover Node
-------------------
Moves the landing platform along a sinusoidal trajectory in the XY plane.
Uses the Gazebo set_entity_state service to update the platform pose at 50 Hz.
"""

import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Point, Quaternion


class PlatformMover(Node):

    def __init__(self):
        super().__init__('platform_mover')

        # Declare parameters
        self.declare_parameter('amplitude_x', 2.0)
        self.declare_parameter('amplitude_y', 1.5)
        self.declare_parameter('frequency_x', 0.1)    # Hz
        self.declare_parameter('frequency_y', 0.15)   # Hz
        self.declare_parameter('platform_height', 0.025)
        self.declare_parameter('update_rate', 50.0)

        self.amp_x = self.get_parameter('amplitude_x').value
        self.amp_y = self.get_parameter('amplitude_y').value
        self.freq_x = self.get_parameter('frequency_x').value
        self.freq_y = self.get_parameter('frequency_y').value
        self.height = self.get_parameter('platform_height').value
        rate = self.get_parameter('update_rate').value

        # Service client for moving platform
        self.client = self.create_client(
            SetEntityState, '/gazebo/set_entity_state')
        self.get_logger().info('Waiting for /gazebo/set_entity_state ...')
        self.client.wait_for_service(timeout_sec=30.0)
        self.get_logger().info('Service available.')

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info(
            f'Platform mover started  amp=({self.amp_x},{self.amp_y})  '
            f'freq=({self.freq_x},{self.freq_y}) Hz')

    # ------------------------------------------------------------------
    def timer_callback(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        omega_x = 2.0 * math.pi * self.freq_x
        omega_y = 2.0 * math.pi * self.freq_y

        # Position (sinusoidal)
        x = self.amp_x * math.sin(omega_x * t)
        y = self.amp_y * math.sin(omega_y * t)

        # Velocity (analytical derivative)
        vx = self.amp_x * omega_x * math.cos(omega_x * t)
        vy = self.amp_y * omega_y * math.cos(omega_y * t)

        state = EntityState()
        state.name = 'landing_platform'
        state.pose.position = Point(x=x, y=y, z=self.height)
        state.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        # Zero twist — we set position directly each tick;
        # non-zero twist would cause physics-engine drift between teleports.
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.reference_frame = 'world'

        req = SetEntityState.Request()
        req.state = state
        self.client.call_async(req)          # fire-and-forget


def main(args=None):
    rclpy.init(args=args)
    node = PlatformMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
