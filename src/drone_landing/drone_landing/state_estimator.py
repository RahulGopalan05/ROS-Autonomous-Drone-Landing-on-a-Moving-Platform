#!/usr/bin/env python3
"""
State Estimator Node
--------------------
Subscribes to /gazebo/model_states (published by the gazebo_ros_state plugin)
and extracts the drone and platform poses.  Publishes:
  /drone/pose          (geometry_msgs/PoseStamped)
  /platform/pose       (geometry_msgs/PoseStamped)
  /relative_pose       (geometry_msgs/PoseStamped)  — platform minus drone
  /drone/height_above_platform  (std_msgs/Float32)
  TF: map → drone/base_link
  TF: map → platform

Using the topic (instead of get_entity_state service) gives real-time
updates with no async / stale-cache issues.
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster


class StateEstimator(Node):

    DRONE_NAME = 'quadrotor'
    PLATFORM_NAME = 'landing_platform'

    def __init__(self):
        super().__init__('state_estimator')

        # Subscribe to Gazebo model states topic
        self.create_subscription(
            ModelStates, '/gazebo/model_states',
            self._model_states_cb, 10)

        # Publishers
        self.drone_pub = self.create_publisher(PoseStamped, '/drone/pose', 10)
        self.plat_pub = self.create_publisher(PoseStamped, '/platform/pose', 10)
        self.rel_pub = self.create_publisher(PoseStamped, '/relative_pose', 10)
        self.height_pub = self.create_publisher(
            Float32, '/drone/height_above_platform', 10)

        # TF broadcaster
        self.tf_bc = TransformBroadcaster(self)

        self.get_logger().info(
            'State estimator started — listening on /gazebo/model_states')

    # ------------------------------------------------------------------
    def _model_states_cb(self, msg):
        """Called every time Gazebo publishes model states."""
        drone_pose = None
        plat_pose = None

        for i, name in enumerate(msg.name):
            if name == self.DRONE_NAME:
                drone_pose = msg.pose[i]
            elif name == self.PLATFORM_NAME:
                plat_pose = msg.pose[i]

        if drone_pose is None or plat_pose is None:
            return

        now = self.get_clock().now().to_msg()

        # --- Publish drone pose ---
        msg_d = PoseStamped()
        msg_d.header.stamp = now
        msg_d.header.frame_id = 'map'
        msg_d.pose = drone_pose
        self.drone_pub.publish(msg_d)

        # --- Publish platform pose ---
        msg_p = PoseStamped()
        msg_p.header.stamp = now
        msg_p.header.frame_id = 'map'
        msg_p.pose = plat_pose
        self.plat_pub.publish(msg_p)

        # --- Relative pose (platform − drone) ---
        msg_r = PoseStamped()
        msg_r.header.stamp = now
        msg_r.header.frame_id = 'map'
        msg_r.pose.position.x = plat_pose.position.x - drone_pose.position.x
        msg_r.pose.position.y = plat_pose.position.y - drone_pose.position.y
        msg_r.pose.position.z = plat_pose.position.z - drone_pose.position.z
        msg_r.pose.orientation.w = 1.0
        self.rel_pub.publish(msg_r)

        # --- Height above platform ---
        h = Float32()
        h.data = float(drone_pose.position.z - plat_pose.position.z)
        self.height_pub.publish(h)

        # --- TF broadcast ---
        self._send_tf('map', 'drone/base_link', drone_pose, now)
        self._send_tf('map', 'platform', plat_pose, now)

    # ------------------------------------------------------------------
    def _send_tf(self, parent, child, pose, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.tf_bc.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
