#!/usr/bin/env python3
"""
pose_init_once.py  – publish exactly **one** /initialpose, then quit.
• Waits until the AMCL node is **active** AND is actually subscribed
  (→ no race, no lost message).
• Publishes with QoS *transient_local* so the message is latched.
• Exits cleanly – no crash, no lingering executor.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import PoseWithCovarianceStamped
from builtin_interfaces.msg import Time
import math
import sys

AMCL_NS = "/j100_0000/amcl"
POSE = {  # warehouse origin you used before
    "x": -0.50,
    "y":  0.00,
    "yaw": math.pi / 2.0,
}

class PoseInitOnce(Node):
    def __init__(self):
        super().__init__("pose_init_once")

        # Latched publisher
        qos = QoSProfile(depth=1,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, "/j100_0000/initialpose", qos)

        # client to AMCL lifecycle service
        self.get_state_cli = self.create_client(
            GetState, f"{AMCL_NS}/get_state")

        # keep polling until ready, then publish
        self.timer = self.create_timer(0.5, self._try_publish)

    # ------------------------------------------------------------------
    def _try_publish(self):
        # 1) is AMCL’s get_state service available?
        if not self.get_state_cli.wait_for_service(timeout_sec=0.0):
            self.get_logger().info("…waiting for AMCL")
            return

        # 2) is AMCL ACTIVE?
        future = self.get_state_cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.result() is None or future.result().current_state.id != 3:
            self.get_logger().info("…AMCL not active yet")
            return

        # 3) does AMCL actually subscribe to /initialpose?
        if self.pub.get_subscription_count() == 0:
            self.get_logger().info("…waiting for AMCL subscriber")
            return

        # 4) All green → publish and quit
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = Time()   # zero → “now” for latched msg
        msg.pose.pose.position.x = POSE["x"]
        msg.pose.pose.position.y = POSE["y"]
        msg.pose.pose.orientation.z = math.sin(POSE["yaw"] / 2.0)
        msg.pose.pose.orientation.w = math.cos(POSE["yaw"] / 2.0)
        self.pub.publish(msg)
        self.get_logger().info("✅ Initial pose sent – exiting")
        rclpy.shutdown()            # ends spin(), exits 0

# ----------------------------------------------------------------------
def main():
    rclpy.init()
    node = PoseInitOnce()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
