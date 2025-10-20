#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import threading
import uuid

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from mmc_msgs.msg import TaskRequest
from openai import OpenAI

from llm_interface.helpers import yaw_to_quat


# ─────────────────────────────────────────────────────────────────── system prompt
SYSTEM_PROMPT = """
SYSTEM ROLE:
You are a semantic parser for a Jackal robot. Return ONE compact JSON object ONLY (no prose, no markdown, no code fences). 

Schema (all keys required):
{
  "do_navigate": <true|false>,
  "location": {
    "name": <string|null>,           // canonical place name or null
    "x": <number|null>,              // meters in 'map' frame
    "y": <number|null>,              // meters in 'map' frame
    "yaw": <number|null>             // radians, ENU
  },
  "do_perception": <true|false>,
  "class_id": <string|null>,         // snake_case class label, e.g., "green_cube"
  "target_color": <string|null>,     // lowercase, e.g., "green"
  "detector": <string|null>,         // e.g., "yolo"
  "sequence": <"NAV_THEN_PERCEPTION"|"NAV_ONLY"|"PERCEPTION_ONLY">
}

Ground rules:
1) Known places → fill exact poses and set location.name. Known: 
   - "shelves": {"x": 9.379, "y": 3.925, "yaw": 0.641, "aliases": ["shelf", "the shelves"]}
   - "chairs": {"x": 13.162, "y": -3.706, "yaw": -0.186, "aliases": ["chair", "the chairs", "seats", "seat"]} 
   - "base_start": {"x": 0.000, "y": 0.000, "yaw": 0.000, "aliases": ["base", "home", "start", "dock", "starting point"]} 
   - "objects": {"x": 8.250, "y": -1.893, "yaw": -0.302, "aliases": ["object area", "bins", "bin area", "object zone", "the objects"]}
2) If user gives numeric coordinates, use them exactly (set location.name=null).
3) Perception phrases map to {class_id, target_color}:
   - “green cube” → {"class_id":"green_cube","target_color":"green"}
   - “red ball”   → {"class_id":"red_ball","target_color":"red"}
   Use lowercase; multiword classes become snake_case.
4) Detector: default "yolo" if user mentions detection but no model.
5) If user asks both nav and perception, sequence="NAV_THEN_PERCEPTION".
   Nav only → "NAV_ONLY". Perception only → "PERCEPTION_ONLY".
6) If any field is unknown, set it to null. Do NOT invent values.
7) Output MUST be valid, minified JSON on one line. No trailing commas, no comments.

Few-shot examples:

User: go to the objects
JSON: {"do_navigate":true,"location":{"name":"objects","x":8.25,"y":-1.893,"yaw":-0.302},"do_perception":false,"class_id":null,"target_color":null,"detector":null,"sequence":"NAV_ONLY"}

User: go to the chairs and detect the green cube
JSON: {"do_navigate":true,"location":{"name":"chairs","x":13.162,"y":-3.706,"yaw":-0.186},"do_perception":true,"class_id":"green_cube","target_color":"green","detector":"yolo","sequence":"NAV_THEN_PERCEPTION"}

User: just detect the red ball here
JSON: {"do_navigate":false,"location":{"name":null,"x":null,"y":null,"yaw":null},"do_perception":true,"class_id":"red_ball","target_color":"red","detector":"yolo","sequence":"PERCEPTION_ONLY"}

User: go to x=2.0, y=-1.5, yaw=1.5708 and detect green cube
JSON: {"do_navigate":true,"location":{"name":null,"x":2.0,"y":-1.5,"yaw":1.5708},"do_perception":true,"class_id":"green_cube","target_color":"green","detector":"yolo","sequence":"NAV_THEN_PERCEPTION"}

User: return to base
JSON: {"do_navigate":true,"location":{"name":"base_start","x":0.0,"y":0.0,"yaw":0.0},"do_perception":false,"class_id":null,"target_color":null,"detector":null,"sequence":"NAV_ONLY"}
""".strip()


# ──────────────────────────────────────────────────────────── OpenAI call
def call_openai(user_msg: str) -> dict:
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY not set")
    client = OpenAI(api_key=api_key)
    rsp = client.chat.completions.create(
        model="gpt-3.5-turbo",
        temperature=0.1,
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user",   "content": user_msg.strip()},
        ],
    )
    content = rsp.choices[0].message.content.strip()
    return json.loads(content)


# ───────────────────────────────────────────────────────────── ROS node
class ChatLLM(Node):
    def __init__(self):
        super().__init__("chat_llm")
        # Publish TaskRequest (coordinator_v1 subscribes to it)
        self.pub = self.create_publisher(TaskRequest, "/task_request", 10)
        self.get_logger().info("🤖  Type a command or 'quit' to exit.")

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
                parsed = call_openai(text)
                self.publish_task_request(parsed)
            except Exception as exc:
                self.get_logger().error(f"❌ {exc}")

    def publish_task_request(self, d: dict):
        """
        Map the LLM JSON to mmc_msgs/TaskRequest.
        The coordinator decides sequencing; we just set the flags + pose + perception fields.
        """
        msg = TaskRequest()
        # Stamp header for traceability
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.request_id = str(uuid.uuid4())[:8]  # short id for logs

        # Navigation
        msg.do_navigate = bool(d.get("do_navigate", False))
        if msg.do_navigate:
            loc = d.get("location") or {}
            x = loc.get("x")
            y = loc.get("y")
            yaw = loc.get("yaw")
            if x is not None and y is not None:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                if yaw is not None:
                    pose.pose.orientation = yaw_to_quat(float(yaw))
                else:
                    pose.pose.orientation.w = 1.0
                msg.nav_target = pose
            else:
                self.get_logger().warn("LLM returned do_navigate=True but missing x/y → ignoring nav.")
                msg.do_navigate = False

        # Perception
        msg.do_perception = bool(d.get("do_perception", False))
        if msg.do_perception:
            msg.class_id = (d.get("class_id") or "").strip()
            msg.target_color = (d.get("target_color") or "").strip()
            det = (d.get("detector") or "").strip()
            msg.detector = det if det else "yolo"

        # Sequence mode (explicit & robust)
        if msg.do_navigate and msg.do_perception:
            msg.sequence_mode = TaskRequest.SEQ_NAV_THEN_PERCEPTION
        elif msg.do_navigate:
            msg.sequence_mode = TaskRequest.SEQ_NAV_THEN_PERCEPTION  # ignored by coordinator, fine
        else:
            msg.sequence_mode = TaskRequest.SEQ_NAV_THEN_PERCEPTION  # ignored

        # Publish
        self.pub.publish(msg)

        # ACTION SUMMARY LOG
        if msg.do_navigate:
            px = msg.nav_target.pose.position.x
            py = msg.nav_target.pose.position.y
            nav_txt = f"NAV → ({px:.2f}, {py:.2f})"
        else:
            nav_txt = "NAV → none"
        perc_txt = (
            f"DETECT → class='{msg.class_id}', color='{msg.target_color}', det='{msg.detector}'"
            if msg.do_perception else "DETECT → none"
        )
        self.get_logger().info(
            f"✅ Sent TaskRequest[{msg.request_id}]  {nav_txt};  {perc_txt};  seq={'NAV_THEN_PERCEPTION' if msg.do_navigate and msg.do_perception else ('NAV_ONLY' if msg.do_navigate else 'PERCEPTION_ONLY')}"
        )


# ───────────────────────────────────────────────────────────────────── main
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
