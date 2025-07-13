#!/usr/bin/env python3
"""
Publish a single latched initial-pose message.

Parameters you may override from a launch file:
  • x          (default 0.0  m)
  • y          (default 0.0  m)
  • yaw        (default 0.0  rad)
  • frame_id   (“map”)
  • use_sim_time (bool)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


class InitialPosePub(Node):
    def __init__(self):
        super().__init__("initial_pose_pub")

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        )
        self.pub = self.create_publisher(PoseWithCovarianceStamped,
                                         "initialpose", qos)

        # parameters (can be set from the launch file)
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("frame_id", "map")

        # give AMCL a moment to finish activating
        self.create_timer(0.5, self._publish_once)
        self._done = False

    def _publish_once(self):
        if self._done:
            rclpy.shutdown()
            return

        x   = self.get_parameter("x").value
        y   = self.get_parameter("y").value
        yaw = self.get_parameter("yaw").value

        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("frame_id").value
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # loose covariances – tune if you like
        msg.pose.covariance[0]  = 0.25      # x
        msg.pose.covariance[7]  = 0.25      # y
        msg.pose.covariance[35] = 0.07      # yaw

        self.pub.publish(msg)
        self.get_logger().info(f"✔ initial pose published  x={x:.2f} y={y:.2f} yaw={yaw:.2f}")
        self._done = True


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePub()
    rclpy.spin(node)
