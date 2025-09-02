#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('jackal_perception')
    params_file = os.path.join(pkg, 'params', 'perception.yaml')

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )

    return LaunchDescription([
        arg_use_sim_time,
        Node(
            package='jackal_perception',
            executable='detect_object_server',
            name='perception_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            # inherit whatever namespace you pass with --ros-args -r __ns:=/j100_0000
        )
    ])
