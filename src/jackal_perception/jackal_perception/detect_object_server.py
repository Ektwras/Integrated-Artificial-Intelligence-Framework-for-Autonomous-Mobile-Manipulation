#!/usr/bin/env python3
import math
import time
from typing import Optional, Tuple, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header

from cv_bridge import CvBridge
import cv2
import numpy as np

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

from jackal_perception_interfaces.action import DetectObject

_HAVE_YOLO = True
try:
    from ultralytics import YOLO
except Exception as e:
    _HAVE_YOLO = False
    _YOLO_ERR = repr(e)


def _depth_to_meters(depth_patch: np.ndarray) -> Optional[float]:
    if depth_patch is None:
        return None
    vals = depth_patch.astype(np.float32).reshape(-1)
    vals = vals[np.isfinite(vals)]
    if vals.size == 0:
        return None
    if depth_patch.dtype == np.uint16:
        vals = vals / 1000.0
    return float(np.median(vals))


def _pixel_to_camera_ray(u: float, v: float, K: np.ndarray) -> Tuple[float, float]:
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    x = (u - cx) / fx
    y = (v - cy) / fy
    return x, y


def _parse_aliases(alias_list) -> Dict[str, str]:
    mapping = {}
    if alias_list is None:
        return mapping
    try:
        for line in list(alias_list):
            if not isinstance(line, str) or '=' not in line:
                continue
            key, val = line.split('=', 1)
            k = key.strip().lower()
            v = val.strip()
            if k:
                mapping[k] = v
    except Exception:
        pass
    return mapping


class DetectObjectServer(Node):
    def __init__(self):
        super().__init__("perception_server")

        # --- Parameters (names unchanged) ---
        self.declare_parameter("color_image_topic", "/j100_0000/sensors/camera_0/color/image")
        self.declare_parameter("depth_image_topic", "/j100_0000/sensors/camera_0/depth/image")
        self.declare_parameter("camera_info_topic", "/j100_0000/sensors/camera_0/color/camera_info")

        self.declare_parameter("action_name", "perception/detect_object")
        self.declare_parameter("target_frame", "map")

        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("iou", 0.45)

        self.declare_parameter("show_debug_view", True)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "perception/debug_image")

        self.declare_parameter("timeout_sec", 10.0)

        # IMPORTANT: non-empty STRING_ARRAY default (Humble treats [] as BYTE_ARRAY)
        self.declare_parameter("class_aliases", [''])
        self.declare_parameter("default_class_id", "sports ball")

        # Read params
        self.color_topic = self.get_parameter("color_image_topic").value
        self.depth_topic = self.get_parameter("depth_image_topic").value
        self.cam_info_topic = self.get_parameter("camera_info_topic").value

        action_name_param = self.get_parameter("action_name").value or "perception/detect_object"
        self.action_name = action_name_param if action_name_param.startswith("/") else f"/{action_name_param}"

        self.target_frame = self.get_parameter("target_frame").value

        self.model_path = self.get_parameter("model_path").value
        self.conf_thresh = float(self.get_parameter("confidence").value)
        self.iou_thresh = float(self.get_parameter("iou").value)

        self.show_debug = bool(self.get_parameter("show_debug_view").value)
        self.publish_debug = bool(self.get_parameter("publish_debug_image").value)

        debug_topic_param = self.get_parameter("debug_image_topic").value or "perception/debug_image"
        self.debug_topic = debug_topic_param if debug_topic_param.startswith("/") else f"/{debug_topic_param}"

        self.default_timeout = float(self.get_parameter("timeout_sec").value)
        self.class_map = _parse_aliases(self.get_parameter("class_aliases").value)
        self.default_class = str(self.get_parameter("default_class_id").value or "sports ball")

        self.get_logger().info(f"class_map loaded: {self.class_map}")

        # Bridge & buffers
        self.bridge = CvBridge()
        self._rgb: Optional[Tuple[Image, np.ndarray]] = None
        self._depth: Optional[Tuple[Image, np.ndarray]] = None
        self._K: Optional[np.ndarray] = None
        self._rgb_frame_id: Optional[str] = None
        self._saw_rgb = False
        self._saw_depth = False
        self._saw_caminfo = False

        # TF
        self.tf_buf = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buf, self)

        # QoS for CameraInfo: **match typical camera publishers** (RELIABLE + VOLATILE)
        qos_caminfo = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subs
        self.create_subscription(Image, self.color_topic, self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.cam_info_topic, self._on_caminfo, qos_caminfo)

        # Debug publisher
        self.pub_debug = self.create_publisher(Image, self.debug_topic, 10) if self.publish_debug else None

        # YOLO (lazy)
        self._yolo = None
        if not _HAVE_YOLO:
            self.get_logger().warn(
                f"Ultralytics YOLO not available ({_YOLO_ERR}). Goals will be rejected."
            )

        # Action server
        self._action_srv = ActionServer(
            self,
            DetectObject,
            self.action_name,
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb
        )

        self.get_logger().info(
            "Perception server ready.\n"
            f"  RGB:   {self.color_topic}\n"
            f"  Depth: {self.depth_topic}\n"
            f"  Info:  {self.cam_info_topic}\n"
            f"  Action: {self.action_name}  -> poses in '{self.target_frame}'\n"
            f"  Debug:  {self.debug_topic}"
        )

    # --- Callbacks ---
    def _on_rgb(self, msg: Image):
        try:
            cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge RGB conversion failed: {e}")
            return
        self._rgb = (msg, cv)
        self._rgb_frame_id = msg.header.frame_id
        if not self._saw_rgb:
            self._saw_rgb = True
            self.get_logger().info("RGB stream connected.")

    def _on_depth(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge Depth conversion failed: {e}")
            return
        self._depth = (msg, depth)
        if not self._saw_depth:
            self._saw_depth = True
            self.get_logger().info("Depth stream connected.")

    def _on_caminfo(self, msg: CameraInfo):
        self._K = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        if not self._saw_caminfo:
            self._saw_caminfo = True
            self.get_logger().info("CameraInfo received (intrinsics ready).")

    # --- Action helpers ---
    def _goal_cb(self, _goal_req):
        return GoalResponse.REJECT if not _HAVE_YOLO else GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _lazy_load_yolo(self):
        if self._yolo is not None:
            return
        model_to_load = (self.model_path or "").strip() or "yolov8n.pt"
        self.get_logger().info(f"Loading YOLO model: {model_to_load}")
        self._yolo = YOLO(model_to_load)
        self._yolo.overrides["conf"] = self.conf_thresh
        self._yolo.overrides["iou"] = self.iou_thresh

    def _publish_debug(self, img_bgr: np.ndarray, frame_id: Optional[str]):
        if self.pub_debug is None or img_bgr is None:
            return
        try:
            msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id or (self._rgb_frame_id or self.target_frame)
            self.pub_debug.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish debug image: {e}")

    def _execute_cb(self, goal_handle):
        result = DetectObject.Result()
        req = goal_handle.request

        try:
            self.get_logger().info(
                f"Goal received: class_id='{getattr(req,'class_id','')}', "
                f"target_color='{getattr(req,'target_color','')}', "
                f"detector='{getattr(req,'detector','')}'"
            )
        except Exception:
            pass

        target_req = (
            getattr(req, "class_id", None)
            or getattr(req, "target_class", None)
            or getattr(req, "label", None)
            or ""
        )
        target_req = str(target_req or "").strip()
        timeout = float(getattr(req, "timeout", self.default_timeout))

        if target_req:
            yolo_label = self.class_map.get(target_req.lower(), target_req).strip()
        else:
            yolo_label = self.default_class

        if not _HAVE_YOLO:
            goal_handle.abort()
            result.success = False
            result.message = "YOLO not available on this system"
            result.pose = PoseStamped()
            result.confidence = 0.0
            return result

        try:
            self._lazy_load_yolo()
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO: {e}")
            goal_handle.abort()
            result.success = False
            result.message = f"YOLO init error: {e}"
            result.pose = PoseStamped()
            result.confidence = 0.0
            return result

        self.get_logger().info(
            f"Detecting '{target_req}' (YOLO label: '{yolo_label}') for up to {timeout:.1f}s"
        )

        # Use wall clock for timeout so we never hang if sim time isn't ticking.
        t0 = time.monotonic()
        last_debug_pub = 0.0
        best_pose_map = None
        best_conf = 0.0
        warned_missing = False

        while rclpy.ok() and (time.monotonic() - t0) < timeout:
            if self._rgb is None:
                if not warned_missing:
                    self.get_logger().info("Waiting for RGB frames…")
                    warned_missing = True
                time.sleep(0.02)
                continue

            rgb_msg, rgb = self._rgb
            depth = self._depth[1] if self._depth is not None else None
            K = self._K
            frame_id = self._rgb_frame_id or rgb_msg.header.frame_id

            # Run detector on RGB
            try:
                rgb_for_net = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
                preds = self._yolo.predict(rgb_for_net, verbose=False)
            except Exception as e:
                self.get_logger().warn(f"YOLO inference error: {e}")
                time.sleep(0.05)
                continue

            # Prepare debug image every ~0.1s regardless of depth/K availability
            debug_img = rgb.copy()
            now_sec = time.monotonic()

            chosen = None
            for r in preds:
                if r.boxes is None or len(r.boxes) == 0:
                    continue
                boxes = r.boxes
                xyxy = boxes.xyxy.cpu().numpy()
                clsi = boxes.cls.cpu().numpy().astype(int)
                conf = boxes.conf.cpu().numpy()
                names = r.names

                for i in range(xyxy.shape[0]):
                    label = names[int(clsi[i])]
                    x1, y1, x2, y2 = xyxy[i].tolist()
                    c = float(conf[i])

                    # Draw all boxes for visual feedback
                    if self.publish_debug or self.show_debug:
                        cv2.rectangle(debug_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.putText(
                            debug_img, f"{label} {c:.2f}",
                            (int(x1), max(10, int(y1) - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
                        )

                    if yolo_label and (label.lower() == yolo_label.lower()):
                        if (chosen is None) or (c > chosen["conf"]):
                            chosen = {"conf": c, "bbox": (x1, y1, x2, y2), "label": label}

            # publish debug continuously while searching
            if (self.publish_debug or self.show_debug) and (now_sec - last_debug_pub > 0.1):
                self._publish_debug(debug_img, frame_id)
                last_debug_pub = now_sec

            # If no matching box yet, keep looping
            if chosen is None:
                time.sleep(0.02)
                continue

            # If we don't yet have depth or intrinsics, show label but wait for pose
            if depth is None or K is None:
                x1, y1, x2, y2 = chosen["bbox"]
                u = int((x1 + x2) * 0.5)
                v = int((y1 + y2) * 0.5)
                if self.publish_debug or self.show_debug:
                    dbg = debug_img
                    cv2.circle(dbg, (u, v), 3, (0, 255, 255), -1)
                    cv2.putText(
                        dbg, "[no depth/K yet]", (u + 5, v + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1
                    )
                    self._publish_debug(dbg, frame_id)
                time.sleep(0.02)
                continue

            # Compute depth at box center
            x1, y1, x2, y2 = chosen["bbox"]
            u = int((x1 + x2) * 0.5)
            v = int((y1 + y2) * 0.5)
            half = 3
            uh0 = max(0, u - half); uh1 = min(depth.shape[1], u + half + 1)
            vh0 = max(0, v - half); vh1 = min(depth.shape[0], v + half + 1)
            Z = _depth_to_meters(depth[vh0:vh1, uh0:uh1])
            if Z is None or not math.isfinite(Z) or Z <= 0.0:
                time.sleep(0.02)
                continue

            xnorm, ynorm = _pixel_to_camera_ray(float(u), float(v), K)
            X = xnorm * Z
            Y = ynorm * Z

            cam_pose = PoseStamped()
            cam_pose.header = Header()
            cam_pose.header.stamp = rgb_msg.header.stamp
            cam_pose.header.frame_id = frame_id
            cam_pose.pose.position = Point(x=X, y=Y, z=Z)
            cam_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            try:
                tf = self.tf_buf.lookup_transform(self.target_frame, cam_pose.header.frame_id, Time())
                map_pose = do_transform_pose(cam_pose, tf)
            except Exception as e:
                self.get_logger().warn(f"TF transform {cam_pose.header.frame_id} -> {self.target_frame} failed: {e}")
                time.sleep(0.05)
                continue

            if self.publish_debug or self.show_debug:
                dbg = debug_img
                cv2.circle(dbg, (u, v), 3, (0, 255, 255), -1)
                txt = f"{chosen['label']} {chosen['conf']:.2f} Z={Z:.2f}m"
                cv2.putText(dbg, txt, (int(x1), max(10, int(y1) - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                self._publish_debug(dbg, frame_id)

            best_pose_map = map_pose
            best_conf = float(chosen["conf"])
            break

        # Finish
        if best_pose_map is None:
            self.get_logger().info("Detection timed out / not found or no depth/K available.")
            goal_handle.abort()
            result.success = False
            result.message = "not_found_or_missing_depth_K"
            result.pose = PoseStamped()
            result.confidence = 0.0
        else:
            self.get_logger().info(
                f"Detection OK → map xyz=({best_pose_map.pose.position.x:.2f}, "
                f"{best_pose_map.pose.position.y:.2f}, {best_pose_map.pose.position.z:.2f})"
            )
            goal_handle.succeed()
            result.success = True
            result.message = "ok"
            result.pose = best_pose_map
            result.confidence = best_conf

        return result


def main():
    rclpy.init()
    node = DetectObjectServer()
    try:
        # Single-threaded is fine here; the loop yields with time.sleep.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
