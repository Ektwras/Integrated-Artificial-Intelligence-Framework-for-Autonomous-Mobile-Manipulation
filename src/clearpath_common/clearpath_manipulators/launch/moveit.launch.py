#!/usr/bin/env python3
#
# (BSD licence header unchanged)
#

import os
import xacro
import xml.etree.ElementTree as ET
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from clearpath_config.clearpath_config import ClearpathConfig


def launch_setup(context, *args, **kwargs):
    # ------------------------------------------------------------------
    # Launch-file arguments
    # ------------------------------------------------------------------
    setup_path       = LaunchConfiguration('setup_path')
    use_sim_time     = LaunchConfiguration('use_sim_time')
    extra_semantic   = LaunchConfiguration('extra_semantic')
    setup_path_ctx   = setup_path.perform(context)
    overlay_path_str = extra_semantic.perform(context)         # may be ''

    # ------------------------------------------------------------------
    # Namespace
    # ------------------------------------------------------------------
    namespace = ClearpathConfig(
        os.path.join(setup_path_ctx, 'robot.yaml')
    ).get_namespace()

    # ------------------------------------------------------------------
    # URDF
    # ------------------------------------------------------------------
    robot_description = {
        'robot_description': xacro.process_file(
            os.path.join(setup_path_ctx, 'robot.urdf.xacro')
        ).toxml()
    }

    # ------------------------------------------------------------------
    # SRDF  (merge overlay if provided)
    # ------------------------------------------------------------------
    main_srdf_str = xacro.process_file(
        os.path.join(setup_path_ctx, 'robot.srdf')
    ).toxml()

    root = ET.fromstring(main_srdf_str)

    if overlay_path_str:
        overlay_path = Path(overlay_path_str)
        if overlay_path.is_file():
            snippet = f'<root>{overlay_path.read_text()}</root>'
            for elem in ET.fromstring(snippet):
                # avoid duplicate virtual-joints on re-launch
                if root.find(f"./{elem.tag}[@name='{elem.get('name')}']") is None:
                    root.append(elem)

    merged_srdf = ET.tostring(root, encoding='unicode')

    robot_description_semantic = {
        'robot_description_semantic': merged_srdf
    }

    # ------------------------------------------------------------------
    # move_group node
    # ------------------------------------------------------------------
    return [
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='log',
            namespace=namespace,
            parameters=[
                os.path.join(
                    setup_path_ctx,
                    'manipulators', 'config', 'moveit.yaml'),
                robot_description,
                robot_description_semantic,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('joint_states', 'platform/joint_states'),
            ]

        )
    ]


def generate_launch_description():
    arg_setup_path = DeclareLaunchArgument(
        'setup_path',
        default_value='/etc/clearpath/',
        description='Clearpath setup path'
    )
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='use_sim_time'
    )
    # NEW – optional SRDF overlay
    arg_extra_semantic = DeclareLaunchArgument(
        'extra_semantic',
        default_value='',
        description='Absolute path of SRDF snippet to prepend (may be empty)'
    )

    ld = LaunchDescription()
    ld.add_action(arg_setup_path)
    ld.add_action(arg_use_sim_time)
    ld.add_action(arg_extra_semantic)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
