from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    chat_node = Node(
        package="llm_interface",
        executable="chat",
        # Open a wider terminal with a custom title; all flags are safe no-ops
        # if the system doesn't support extras. No color/profile tweaks here to
        # avoid dependency on local GNOME profiles.
        prefix="gnome-terminal --wait --profile=jackal --title='JACKAL AI' --",
        output="screen",
        parameters=[{"use_sim_time": False}],  # set True if you run with /clock
    )

    return LaunchDescription([chat_node])
