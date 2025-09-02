#!/usr/bin/env python3
"""
Inject a floating virtual joint (odom ↔ base_link) into the SRDF
*after* move_group has started, so MoveIt keeps the mobile base
connected to the global frame.

Call once, any time after /j100_0000/move_group is up.
"""

import rclpy
import xml.etree.ElementTree as ET
from rclpy.parameter import Parameter as RclParam          # ← fix 1
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue

FRAME = "odom"        # parent TF frame
CHILD = "base_link"   # robot root link


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("inject_vj")

    # ---- 1. Contact move_group’s parameter server --------------------------
    cli = node.create_client(SetParameters,
                             "/j100_0000/move_group/set_parameters")
    cli.wait_for_service()

    # ---- 2. Grab the current SRDF string -----------------------------------
    desc_param = node.get_parameter_or(
        "robot_description_semantic",
        RclParam(
            "robot_description_semantic",
            RclParam.Type.STRING,
            ""                         # default empty SRDF if not set yet
        )
    ).value
    root = ET.fromstring(desc_param)

    # ---- 3. Add the virtual joint only if it isn't there already ----------
    if root.find(".//virtual_joint[@name='world_joint']") is None:
        ET.SubElement(
            root, "virtual_joint",
            name="world_joint",
            type="floating",           # use "planar" if Z/roll/pitch should be locked
            parent_frame=FRAME,
            child_link=CHILD
        )
        new_desc = ET.tostring(root, encoding="unicode")

        # ---- 4. Send the updated SRDF back to move_group -------------------
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name="robot_description_semantic",
            value=ParameterValue(
                type=ParameterValue.TYPE_STRING,
                string_value=new_desc
            )
        )]
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        node.get_logger().info("Injected floating virtual joint into SRDF")

    else:
        node.get_logger().info("SRDF already contains world_joint – nothing to do")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
