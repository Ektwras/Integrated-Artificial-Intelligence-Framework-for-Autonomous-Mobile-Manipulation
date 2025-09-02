#!/usr/bin/env python3
"""
Inject the red ball and green box as CollisionObjects into MoveIt's
planning scene, referenced in the global (odom) frame.
"""

import rclpy
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene

GLOBAL_FRAME = "odom"          # planning-scene world frame

# name, shape, size (radius or (x,y,z)), pose xyz (m)
OBJECT_SPECS = [
    ("red_ball",  "sphere", 0.05,            (9.0, -2.0, 0.05)),
    ("green_box", "box",    (0.1, 0.1, 0.1), (9.0, -2.2, 0.05)),
]


def make_collision_object(name: str, shape: str, size, xyz) -> CollisionObject:
    """Build a MoveIt CollisionObject message."""
    co = CollisionObject()
    co.id = name
    co.header.frame_id = GLOBAL_FRAME
    co.operation = CollisionObject.ADD

    # geometry
    if shape == "sphere":
        prim = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[size])  # radius
    else:  # box
        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))  # (x,y,z)
    co.primitives.append(prim)

    # pose
    p = PoseStamped()
    p.header.frame_id = GLOBAL_FRAME
    p.pose.position.x, p.pose.position.y, p.pose.position.z = xyz
    p.pose.orientation.w = 1.0
    co.primitive_poses.append(p.pose)

    return co


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("add_scene_objects")

    srv_name = "/j100_0000/apply_planning_scene"
    client = node.create_client(ApplyPlanningScene, srv_name)

    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(f"Service '{srv_name}' unavailable")
        rclpy.shutdown()
        return

    req = ApplyPlanningScene.Request()
    req.scene.is_diff = True
    req.scene.world.collision_objects = [
        make_collision_object(*spec) for spec in OBJECT_SPECS
    ]

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if future.done() and future.result() and future.result().success:
        node.get_logger().info(f"Objects spawned in frame '{GLOBAL_FRAME}'.")
    else:
        node.get_logger().error("Failed to apply planning scene.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
