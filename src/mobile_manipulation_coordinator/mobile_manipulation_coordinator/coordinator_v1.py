#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import rclpy
from rclpy.node   import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action  import NavigateToPose
from jackal_perception_interfaces.action import DetectObject

try:
    from mmc_msgs.msg import TaskRequest as MMC_TaskRequest
except Exception:
    MMC_TaskRequest = None


class MobileManipulationCoordinator(Node):
    S_IDLE        = "IDLE"
    S_NAVIGATING  = "NAVIGATING"
    S_DETECTING   = "DETECTING"

    M_NAV_ONLY            = "NAV_ONLY"
    M_PERCEPTION_ONLY     = "PERCEPTION_ONLY"
    M_NAV_THEN_PERCEPTION = "NAV_THEN_PERCEPTION"
    M_PERCEPTION_THEN_NAV = "PERCEPTION_THEN_NAV"  # parsed; we still execute sequentially

    def __init__(self) -> None:
        super().__init__("mobile_manipulation_coordinator")

        # ------------------- Parameters -------------------
        # Nav2 action server
        self.declare_parameter("nav_action_name", "/j100_0000/navigate_to_pose")

        # DetectObject action server
        self.declare_parameter("detect_action_name", "/perception/detect_object")

        # Expected frame of DetectObject result (advisory; server decides the actual frame)
        self.declare_parameter("detect_expected_frame", "base_link")

        # Default detection request (used by legacy /task_goal as fallback)
        self.declare_parameter("detect_class_id",     "red_ball")
        self.declare_parameter("detect_target_color", "red")
        self.declare_parameter("detect_detector",     "yolo")

        self.declare_parameter("show_debug_reminder", True)

        # params
        self.nav_action_name  = self.get_parameter("nav_action_name").get_parameter_value().string_value
        self.detect_action_name = self.get_parameter("detect_action_name").get_parameter_value().string_value
        self.detect_expected_frame = self.get_parameter("detect_expected_frame").get_parameter_value().string_value

        self.default_detect = {
            "class_id":     self.get_parameter("detect_class_id").get_parameter_value().string_value,
            "target_color": self.get_parameter("detect_target_color").get_parameter_value().string_value,
            "detector":     self.get_parameter("detect_detector").get_parameter_value().string_value,
        }
        self.show_debug_reminder = self.get_parameter("show_debug_reminder").get_parameter_value().bool_value

        # ------------------- Action clients -------------------
        self.nav_client    = ActionClient(self, NavigateToPose, self.nav_action_name)
        self.detect_client = ActionClient(self, DetectObject,   self.detect_action_name)

        # ------------------- Subscriptions -------------------
        # Legacy: PoseStamped FOR NAV2
        self.create_subscription(PoseStamped, "/task_goal", self._on_legacy_pose_goal, 10)

        # New: TaskRequest
        if MMC_TaskRequest is not None:
            self.create_subscription(MMC_TaskRequest, "/task_request", self._on_task_request, 10)
            self.get_logger().info("Subscribed to /task_request (MMC_TaskRequest).")
        else:
            self.get_logger().warn("MMC_TaskRequest not available; only legacy /task_goal is enabled.")

        # ------------------- State -------------------
        self.state = self.S_IDLE
        self.current_nav_goal: PoseStamped | None = None
        self.current_detect_goal: DetectObject.Goal | None = None
        self._chain_detect_after_nav = False

        self.get_logger().info(
            f"Coordinator v1 ready. Nav action: '{self.nav_action_name}', Detect action: '{self.detect_action_name}', "
            f"expecting detection pose in '{self.detect_expected_frame}'."
        )
        if self.show_debug_reminder:
            self.get_logger().info(
                "Tip: view overlay with\n"
                "  ros2 run image_view image_view --ros-args -r image:=/perception/debug_image"
            )

    # ======================================================================
    # Incoming requests
    # ======================================================================

    def _on_legacy_pose_goal(self, pose: PoseStamped) -> None:
        if not self._ensure_idle():
            return
        task = {
            "mode": self.M_NAV_THEN_PERCEPTION,
            "pose": pose,
            "detect": self.default_detect.copy(),
        }
        self._start_task(task, origin="legacy:/task_goal")

    def _on_task_request(self, msg) -> None:
        if not self._ensure_idle():
            return
        task = self._coerce_task(msg)
        self._start_task(task, origin="/task_request")

    # ======================================================================
    # Task decoding / validation
    # ======================================================================

    def _coerce_task(self, msg) -> dict:
        """
        Normalize TaskRequest to:
        {
          "mode": M_*,
          "pose": PoseStamped | None,
          "detect": {"class_id": str, "target_color": str, "detector": str}
        }
        """
        # Flags
        do_nav  = bool(getattr(msg, "do_navigate", False))
        do_det  = bool(getattr(msg, "do_perception", False))

        # Pose is in nav_target (PoseStamped)
        pose = getattr(msg, "nav_target", None)
        if not isinstance(pose, PoseStamped):
            pose = None
        else:
            # Ensure frame is set (Nav2 needs a valid frame_id)
            if not pose.header.frame_id:
                pose.header.frame_id = "map"

        # Map sequence_mode → mode when both tasks are requested
        seq_mode = int(getattr(msg, "sequence_mode", 0))
        if do_nav and do_det:
            if seq_mode == 1:
                mode = self.M_PERCEPTION_THEN_NAV
            else:
                mode = self.M_NAV_THEN_PERCEPTION  # default
        elif do_nav:
            # if sender forgot the flag but provided a pose, we still accept nav-only
            mode = self.M_NAV_ONLY if pose is not None else self.M_PERCEPTION_ONLY
        elif do_det:
            mode = self.M_PERCEPTION_ONLY
        else:
            # No flags: infer from presence of pose
            mode = self.M_NAV_ONLY if pose is not None else self.M_PERCEPTION_ONLY

        class_id     = getattr(msg, "class_id",     "") or self.default_detect["class_id"]
        target_color = getattr(msg, "target_color", "") or self.default_detect["target_color"]
        detector     = getattr(msg, "detector",     "") or self.default_detect["detector"]

        return {
            "mode": mode,
            "pose": pose,
            "detect": {
                "class_id": str(class_id),
                "target_color": str(target_color),
                "detector": str(detector),
            },
        }

    # ======================================================================
    # Orchestration
    # ======================================================================

    def _start_task(self, task: dict, *, origin: str) -> None:
        self.get_logger().info(
            f"New task from {origin}: mode={task['mode']}, "
            f"detect={task['detect']}, pose={'yes' if task['pose'] else 'no'}"
        )

        mode = task["mode"]
        pose = task["pose"]
        det  = task["detect"]

        self.current_nav_goal    = pose
        self.current_detect_goal = self._make_detect_goal(det)

        if mode == self.M_NAV_ONLY:
            if not pose:
                self._fail_fast("NAV_ONLY requested but no PoseStamped provided.")
                return
            self._begin_navigation(pose, then_detect=False)

        elif mode == self.M_PERCEPTION_ONLY:
            self._begin_detection(self.current_detect_goal)

        elif mode == self.M_NAV_THEN_PERCEPTION:
            if not pose:
                self._fail_fast("NAV_THEN_PERCEPTION requested but no PoseStamped provided.")
                return
            self._begin_navigation(pose, then_detect=True)

        elif mode == self.M_PERCEPTION_THEN_NAV:
            # Execute sequentially
            if not pose:
                self.get_logger().warn("PERCEPTION_THEN_NAV requested but no PoseStamped provided; running perception only.")
                self._begin_detection(self.current_detect_goal)
            else:
                # We'll run detection now; when it finishes, start nav.
                self._chain_detect_after_nav = False
                self._pending_nav_after_detection = True
                self._begin_detection(self.current_detect_goal)

        else:
            self._fail_fast(f"Unsupported mode: {mode}")

    # ======================================================================
    # Navigation
    # ======================================================================

    def _begin_navigation(self, pose: PoseStamped, *, then_detect: bool) -> None:
        if not self._transition(self.S_IDLE, self.S_NAVIGATING):
            return

        self._chain_detect_after_nav = bool(then_detect)

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f"Waiting for Nav2 action server at '{self.nav_action_name}' …")
        self.nav_client.wait_for_server()

        self.get_logger().info(
            f"Sending NAV goal → x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}, "
            f"frame='{pose.header.frame_id}'"
        )
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_nav_goal_response)

    def _on_nav_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Navigation goal was rejected.")
            self._reset()
            return
        goal_handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        try:
            result_wrapper = future.result()
        except Exception as e:
            self.get_logger().error(f"NAV result exception: {e}")
            self._reset()
            return

        status = result_wrapper.status
        if status == 4:  # SUCCEEDED
            self.get_logger().info("Navigation succeeded.")
            if self._chain_detect_after_nav and self.current_detect_goal is not None:
                self._begin_detection(self.current_detect_goal)
            else:
                self._reset()
        else:
            self.get_logger().error(f"Navigation failed (status={status}).")
            self._reset()

    # ======================================================================
    # Perception
    # ======================================================================

    def _make_detect_goal(self, detect_dict: dict) -> DetectObject.Goal:
        g = DetectObject.Goal()
        g.class_id     = detect_dict.get("class_id", self.default_detect["class_id"])
        g.target_color = detect_dict.get("target_color", self.default_detect["target_color"])
        g.detector     = detect_dict.get("detector", self.default_detect["detector"])
        return g

    def _begin_detection(self, goal: DetectObject.Goal) -> None:
        if self.state == self.S_IDLE:
            ok = self._transition(self.S_IDLE, self.S_DETECTING)
        elif self.state == self.S_NAVIGATING:
            self.state = self.S_DETECTING
            ok = True
        else:
            ok = False

        if not ok:
            self.get_logger().warn("Cannot start detection in the current state.")
            self._reset()
            return

        self.get_logger().info(f"Waiting for DetectObject server at '{self.detect_action_name}' …")
        self.detect_client.wait_for_server()

        self.get_logger().info(
            f"Sending DetectObject goal: class_id='{goal.class_id}', "
            f"target_color='{goal.target_color}', detector='{goal.detector}' "
            f"(expecting pose in '{self.detect_expected_frame}')"
        )
        future = self.detect_client.send_goal_async(goal)
        future.add_done_callback(self._on_detect_goal_response)

    def _on_detect_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("DetectObject goal was rejected.")
            self._reset()
            return
        goal_handle.get_result_async().add_done_callback(self._on_detect_result)

    def _on_detect_result(self, future) -> None:
        try:
            result_wrapper = future.result()
        except Exception as e:
            self.get_logger().error(f"DetectObject result exception: {e}")
            self._reset()
            return

        status = result_wrapper.status
        res    = result_wrapper.result

        if status == 4 and getattr(res, "success", False):
            frame_id = getattr(res.pose.header, "frame_id", "")
            p = res.pose.pose.position
            self.get_logger().info(
                f"Perception success ✔  ({res.message})  "
                f"{frame_id} xyz=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}), conf={res.confidence:.2f}"
            )
            if self.detect_expected_frame and frame_id and frame_id != self.detect_expected_frame:
                self.get_logger().warn(
                    f"DetectObject pose returned in '{frame_id}' (expected '{self.detect_expected_frame}'). "
                    "Downstream consumers should transform if required."
                )
        else:
            self.get_logger().warn(
                f"Perception finished (status={status}) – success={getattr(res,'success',False)}, "
                f"message='{getattr(res,'message','')}'"
            )

        # If we had requested PERCEPTION_THEN_NAV, continue with nav now
        if getattr(self, "_pending_nav_after_detection", False) and self.current_nav_goal is not None:
            self._pending_nav_after_detection = False
            self._begin_navigation(self.current_nav_goal, then_detect=False)
        else:
            self._reset()

    # ======================================================================
    # State helpers
    # ======================================================================

    def _ensure_idle(self) -> bool:
        if self.state != self.S_IDLE:
            self.get_logger().warn(f"Busy (state={self.state}); ignoring new request.")
            return False
        return True

    def _transition(self, expected: str, next_state: str) -> bool:
        if self.state != expected:
            self.get_logger().warn(f"Cannot transition {self.state} → {next_state} (expected {expected}).")
            return False
        self.state = next_state
        return True

    def _reset(self) -> None:
        self.state = self.S_IDLE
        self.current_nav_goal = None
        self.current_detect_goal = None
        self._chain_detect_after_nav = False
        self._pending_nav_after_detection = False
        self.get_logger().info("Ready for next task.")


def main(args=None):
    rclpy.init(args=args)
    node = MobileManipulationCoordinator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
