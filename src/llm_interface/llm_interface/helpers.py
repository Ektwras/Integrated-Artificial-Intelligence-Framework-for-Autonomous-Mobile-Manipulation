import math
from geometry_msgs.msg import Quaternion

def yaw_to_quat(yaw: float) -> Quaternion:
    """Convert a yaw angle (rad) into geometry_msgs/Quaternion (ROS ENU)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q
