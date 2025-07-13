#!/usr/bin/env python3

import os
import sys
import json
import threading
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from openai import OpenAI
from ament_index_python.packages import get_package_share_directory

from llm_interface.helpers import yaw_to_quat

# ─────────────────────────────────────────────────────────────────── constants
SYSTEM_PROMPT = """
Translate the user's warehouse command into a JSON dict:
{"x": <float>, "y": <float>, "yaw": <float radians>}
Return ONLY the JSON (no preamble, no code block).
Coordinates must match the map used by the Jackal robot.
""".strip()

# Landmarks file
LANDMARKS_FILE = os.path.join(
    get_package_share_directory("llm_interface"),
    "config",
    "landmarks.yaml",
)

# ──────────────────────────────────────────────────────────── utilities/helpers
def load_landmarks() -> dict:
    try:
        with open(LANDMARKS_FILE, "r") as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        return {}

def landmark_lookup(sentence: str, table: dict):
    
    return table.get(sentence.strip().lower())

def query_openai(user_msg: str) -> dict:
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY not set in the environment")

    client = OpenAI(api_key=api_key)

    rsp = client.chat.completions.create(
        model="gpt-3.5-turbo",
        temperature=0.2,
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user",   "content": user_msg.strip()},
        ],
    )
    return json.loads(rsp.choices[0].message.content)

# ────────────────────────────────────────────────────────────────────── ROS node
class ChatLLM(Node):
    def __init__(self):
        super().__init__("chat_llm")
        self.pub = self.create_publisher(PoseStamped, "/task_goal", 10)
        self.landmarks = load_landmarks()

        self.get_logger().info(
            "🤖  Provide a task or type 'quit' to terminate…"
        )

    
    def repl(self):
        while rclpy.ok():
            try:
                text = input("🗣  ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not text:
                continue
            if text.lower() in {"quit", "exit"}:
                break

            try:
                pose_dict = landmark_lookup(text, self.landmarks)
                if pose_dict is None:
                    pose_dict = query_openai(text)
                self.publish_pose(pose_dict)
            except Exception as exc:
                self.get_logger().error(f"❌ {exc}")

    def publish_pose(self, d: dict):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(d["x"])
        pose.pose.position.y = float(d["y"])
        pose.pose.orientation = yaw_to_quat(float(d["yaw"]))
        self.pub.publish(pose)
        self.get_logger().info(f"✅ Published {d}")

# ──────────────────────────────────────────────────────────────────────── main
def main():
    rclpy.init()
    node = ChatLLM()
    t = threading.Thread(target=node.repl, daemon=True)
    t.start()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
