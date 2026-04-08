#!/usr/bin/env python3
"""
Landing Controller Node
-----------------------
PID-based state machine that lands the drone on a moving platform.

States:
  WAITING    → hold at spawn position for 3 s
  TRACKING   → follow platform XY at cruise altitude
  DESCENDING → aligned — descend at 0.3 m/s
  LANDING    → final slow approach at 0.1 m/s
  LANDED     → lock drone on platform

Subscribes:  /drone/pose, /platform/pose
Publishes:   /landing/status
Uses:        /gazebo/set_entity_state
"""

import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String


# ======================================================================
class PIDController:
    """1-D PID with output clamping and integral anti-windup."""

    def __init__(self, kp, ki, kd, out_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_max = out_max
        self._integral = 0.0
        self._prev_err = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_err = 0.0

    def compute(self, error, dt):
        if dt <= 0.0:
            return 0.0
        self._integral += error * dt
        # Anti-windup
        if self.ki > 1e-9:
            i_max = self.out_max / self.ki
            self._integral = max(-i_max, min(i_max, self._integral))
        derivative = (error - self._prev_err) / dt
        self._prev_err = error
        output = (self.kp * error
                  + self.ki * self._integral
                  + self.kd * derivative)
        return max(-self.out_max, min(self.out_max, output))


# ======================================================================
class LandingController(Node):

    WAITING = 'WAITING'
    TRACKING = 'TRACKING'
    DESCENDING = 'DESCENDING'
    LANDING = 'LANDING'
    LANDED = 'LANDED'

    def __init__(self):
        super().__init__('landing_controller')

        # ---------- parameters ----------
        self.declare_parameter('pid_xy_kp', 3.0)
        self.declare_parameter('pid_xy_ki', 0.05)
        self.declare_parameter('pid_xy_kd', 0.8)
        self.declare_parameter('pid_z_kp', 1.2)
        self.declare_parameter('pid_z_ki', 0.0)
        self.declare_parameter('pid_z_kd', 0.5)
        self.declare_parameter('max_horizontal_vel', 3.0)
        self.declare_parameter('max_vertical_vel', 1.0)
        self.declare_parameter('cruise_altitude', 3.0)
        self.declare_parameter('alignment_threshold', 0.5)
        self.declare_parameter('align_hold_time', 1.5)
        self.declare_parameter('descent_speed', 0.3)
        self.declare_parameter('final_descent_speed', 0.10)
        self.declare_parameter('landing_height', 0.08)
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_z', 5.0)

        kp_xy = self.get_parameter('pid_xy_kp').value
        ki_xy = self.get_parameter('pid_xy_ki').value
        kd_xy = self.get_parameter('pid_xy_kd').value
        kp_z = self.get_parameter('pid_z_kp').value
        ki_z = self.get_parameter('pid_z_ki').value
        kd_z = self.get_parameter('pid_z_kd').value
        max_h = self.get_parameter('max_horizontal_vel').value
        max_v = self.get_parameter('max_vertical_vel').value
        self.cruise_alt = self.get_parameter('cruise_altitude').value
        self.align_thresh = self.get_parameter('alignment_threshold').value
        self.align_hold = self.get_parameter('align_hold_time').value
        self.descent_speed = self.get_parameter('descent_speed').value
        self.final_speed = self.get_parameter('final_descent_speed').value
        self.land_height = self.get_parameter('landing_height').value
        rate = self.get_parameter('update_rate').value
        self.spawn_x = self.get_parameter('spawn_x').value
        self.spawn_y = self.get_parameter('spawn_y').value
        self.spawn_z = self.get_parameter('spawn_z').value

        self.dt = 1.0 / rate

        # ---------- PID controllers ----------
        self.pid_x = PIDController(kp_xy, ki_xy, kd_xy, max_h)
        self.pid_y = PIDController(kp_xy, ki_xy, kd_xy, max_h)
        self.pid_z = PIDController(kp_z, ki_z, kd_z, max_v)

        # ---------- state ----------
        self.state = self.WAITING
        self.drone_pose = None
        self.platform_pose = None
        self.start_time = None
        self.aligned_duration = 0.0
        self.target_altitude = self.spawn_z
        self._tick = 0

        # ---------- ROS interfaces ----------
        self.create_subscription(
            PoseStamped, '/drone/pose', self._drone_cb, 10)
        self.create_subscription(
            PoseStamped, '/platform/pose', self._platform_cb, 10)

        self.set_client = self.create_client(
            SetEntityState, '/gazebo/set_entity_state')
        self.get_logger().info('Waiting for /gazebo/set_entity_state ...')
        self.set_client.wait_for_service(timeout_sec=30.0)
        self.get_logger().info('Service available.')

        self.status_pub = self.create_publisher(String, '/landing/status', 10)

        self.timer = self.create_timer(self.dt, self._control_loop)
        self.get_logger().info('Landing controller started')

    # ------------------------------------------------------------------
    def _drone_cb(self, msg):
        self.drone_pose = msg.pose

    def _platform_cb(self, msg):
        self.platform_pose = msg.pose

    # ------------------------------------------------------------------
    def _control_loop(self):
        if self.drone_pose is None or self.platform_pose is None:
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        dp = self.drone_pose.position
        pp = self.platform_pose.position

        err_x = pp.x - dp.x
        err_y = pp.y - dp.y
        horiz_err = math.sqrt(err_x ** 2 + err_y ** 2)
        height_above = dp.z - pp.z

        # ---- State machine ----
        if self.state == self.WAITING:
            self._do_waiting(elapsed, dp)
        elif self.state == self.TRACKING:
            self._do_tracking(err_x, err_y, dp, pp, horiz_err)
        elif self.state == self.DESCENDING:
            self._do_descending(err_x, err_y, dp, pp, height_above)
        elif self.state == self.LANDING:
            self._do_landing(err_x, err_y, dp, pp, height_above)
        elif self.state == self.LANDED:
            self._do_landed(dp, pp)

        # Publish status
        s = String()
        s.data = (f'{self.state} | xy_err={horiz_err:.2f} m | '
                  f'alt_above={height_above:.2f} m')
        self.status_pub.publish(s)

        # Debug log once per second
        self._tick += 1
        if self._tick % 50 == 0:
            self.get_logger().info(s.data)

    # ------------------------------------------------------------------
    def _do_waiting(self, elapsed, dp):
        """Hold at spawn position for 3 s."""
        target_x = self.spawn_x
        target_y = self.spawn_y
        target_z = self.spawn_z
        self._move_toward(dp, target_x, target_y, target_z)

        if elapsed > 3.0:
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()
            self.state = self.TRACKING
            self.get_logger().info('>> TRACKING — following platform XY')

    def _do_tracking(self, err_x, err_y, dp, pp, horiz_err):
        """Follow platform XY, hold cruise altitude."""
        target_x = pp.x
        target_y = pp.y
        target_z = self.cruise_alt
        self._move_toward(dp, target_x, target_y, target_z)

        if horiz_err < self.align_thresh:
            self.aligned_duration += self.dt
        else:
            self.aligned_duration = 0.0

        if self.aligned_duration >= self.align_hold:
            self.target_altitude = dp.z
            self.pid_z.reset()
            self.state = self.DESCENDING
            self.get_logger().info(
                f'>> DESCENDING — aligned for {self.align_hold:.1f}s, '
                f'starting descent from {dp.z:.2f} m')

    def _do_descending(self, err_x, err_y, dp, pp, height_above):
        """Track XY, reduce altitude at descent_speed."""
        self.target_altitude -= self.descent_speed * self.dt
        floor = pp.z + 0.30
        self.target_altitude = max(self.target_altitude, floor)

        target_x = pp.x
        target_y = pp.y
        target_z = self.target_altitude
        self._move_toward(dp, target_x, target_y, target_z)

        if height_above < 0.35:
            self.state = self.LANDING
            self.get_logger().info('>> LANDING — final approach')

    def _do_landing(self, err_x, err_y, dp, pp, height_above):
        """Slow descent to platform surface."""
        target_x = pp.x
        target_y = pp.y
        target_z = pp.z + 0.065

        vx = self.pid_x.compute(target_x - dp.x, self.dt)
        vy = self.pid_y.compute(target_y - dp.y, self.dt)
        vz = self.pid_z.compute(target_z - dp.z, self.dt)
        # Limit descent speed during final approach
        vz = max(vz, -self.final_speed)

        new_x = dp.x + vx * self.dt
        new_y = dp.y + vy * self.dt
        new_z = dp.z + vz * self.dt
        self._set_entity(new_x, new_y, new_z)

        if height_above < self.land_height:
            self.state = self.LANDED
            self.get_logger().info(
                '========================================')
            self.get_logger().info(
                '   LANDED SUCCESSFULLY ON PLATFORM')
            self.get_logger().info(
                '========================================')

    def _do_landed(self, dp, pp):
        """Lock drone on top of platform."""
        self._set_entity(pp.x, pp.y, pp.z + 0.065)

    # ------------------------------------------------------------------
    def _move_toward(self, dp, target_x, target_y, target_z):
        """PID → velocity → new_pos = feedback_pos + vel*dt → teleport."""
        vx = self.pid_x.compute(target_x - dp.x, self.dt)
        vy = self.pid_y.compute(target_y - dp.y, self.dt)
        vz = self.pid_z.compute(target_z - dp.z, self.dt)

        new_x = dp.x + vx * self.dt
        new_y = dp.y + vy * self.dt
        new_z = dp.z + vz * self.dt
        self._set_entity(new_x, new_y, new_z)

    def _set_entity(self, x, y, z):
        state = EntityState()
        state.name = 'quadrotor'
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = float(z)
        state.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.reference_frame = 'world'

        req = SetEntityState.Request()
        req.state = state
        self.set_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = LandingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
