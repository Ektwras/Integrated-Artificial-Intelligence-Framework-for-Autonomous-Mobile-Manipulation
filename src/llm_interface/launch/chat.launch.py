from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    chat_node = Node(
        package="llm_interface",
        executable="chat",
        # Open a separate terminal so the REPL has stdin/stdout
        prefix="gnome-terminal --wait --",    # KDE → 'konsole -e', headless → drop prefix
        output="screen",
        parameters=[{"use_sim_time": False}],  # set True if you run with /clock
    )

    return LaunchDescription([chat_node])
