#!/usr/bin/env python3

import rclpy
from rclpy.node        import Node
from rclpy.action      import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action  import NavigateToPose
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class MobileManipulationCoordinator(Node):

    def __init__(self):
        super().__init__("mobile_manipulation_coordinator")

        # -------- action clients --------
        self.nav_client = ActionClient(
            self, NavigateToPose,
            "/j100_0000/navigate_to_pose")
        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            "/j100_0000/arm_0_joint_trajectory_controller/follow_joint_trajectory")

        # state flag to avoid overlapping jobs
        self.nav_in_progress = False

        # subscribe to goals coming from llm_interface
        self.create_subscription(
            PoseStamped, "/task_goal", self._task_goal_cb, 10)

        # arm joints
        self.arm_joint_names = [
            "arm_0_joint_1", "arm_0_joint_2", "arm_0_joint_3",
            "arm_0_joint_4", "arm_0_joint_5", "arm_0_joint_6"
        ]

        self.get_logger().info("Coordinator ready – waiting for /task_goal …")

    # --------------------------------------------------------- callbacks
    def _task_goal_cb(self, pose: PoseStamped):
        """Called every time chat‑LLM publishes a new PoseStamped."""
        if self.nav_in_progress:
            self.get_logger().warn(
                "Navigation already in progress; ignoring new goal")
            return

        self.nav_in_progress = True
        self.get_logger().info(
            f"Received task goal → ({pose.pose.position.x:.2f}, "
            f"{pose.pose.position.y:.2f}) yaw≈{pose.pose.orientation.w:.2f}")
        self._send_navigation_goal(pose)

    # --------------------------------------------------------- navigation
    def _send_navigation_goal(self, pose: PoseStamped):
        goal_msg       = NavigateToPose.Goal()
        goal_msg.pose  = pose
        self.get_logger().info("Waiting for Nav2 action server…")
        self.nav_client.wait_for_server()

        self.get_logger().info("Sending navigation goal")
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal was rejected")
            self.nav_in_progress = False
            return

        goal_handle.get_result_async().add_done_callback(
            self._nav_result_cb)

    def _nav_result_cb(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:   # SUCCEEDED
            self.get_logger().info("Navigation succeeded – moving the arm")
            self._send_manipulation_goal()
        else:
            self.get_logger().error(f"Navigation failed with status {status}")
            self.nav_in_progress = False

    # --------------------------------------------------------- manipulation
    def _send_manipulation_goal(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joint_names

        pt = JointTrajectoryPoint()
        pt.positions = [0.1, -0.2, 0.3, 0.0, -0.1, 0.2]
        pt.time_from_start.sec = 3
        goal.trajectory.points.append(pt)

        self.arm_client.wait_for_server()
        self.get_logger().info("Sending arm trajectory")
        future = self.arm_client.send_goal_async(goal)
        future.add_done_callback(self._arm_goal_response_cb)

    def _arm_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Arm goal was rejected")
            self.nav_in_progress = False
            return

        goal_handle.get_result_async().add_done_callback(
            self._arm_result_cb)

    def _arm_result_cb(self, future):
        status = future.result().status
        if status == 4:   # SUCCEEDED
            self.get_logger().info("Manipulation complete ✔")
        else:
            self.get_logger().error(f"Manipulation failed with status {status}")
        # ready for the next /task_goal
        self.nav_in_progress = False



def main(args=None):
    rclpy.init(args=args)
    node = MobileManipulationCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
