# =============================================================================
# MASTER LAUNCH FILE FOR JACKAL MOBILE MANIPULATION
#
# Date: June 25, 2025
#
# Description:
# This launch file provides a robust, staged startup for the entire
# simulation stack. It uses TimerActions to ensure each critical
# component has sufficient time to initialize before the next stage begins,
# preventing race conditions and ensuring a consistent launch.
# =============================================================================

import os
import shutil
import subprocess
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
    OpaqueFunction,
    LogInfo
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _clean_up(_context, *_, **__):
    """A one-shot cleanup function to be called on shutdown."""
    print("[MasterLaunch] Performing shutdown cleanup...")
    time.sleep(2.0)
    patterns = ["ignition.*gz", "ros_gz_bridge", "gz_ros2_control", "rviz2"]
    for p in patterns:
        subprocess.call(["pkill", "-15", "-f", p])
    time.sleep(2.0)
    for p in patterns:
        subprocess.call(["pkill", "-9", "-f", p])
    subprocess.call(["ros2", "daemon", "stop"])
    subprocess.call(["ros2", "daemon", "start"])
    log_dir = os.path.join(os.path.expanduser('~'), '.ros', 'log', 'latest')
    shutil.rmtree(log_dir, ignore_errors=True)
    print("[MasterLaunch] Shutdown cleanup complete.")


def generate_launch_description():

    # =========================================================================
    # === STAGE 0: CONFIGURATION & PATHS
    # =========================================================================
    LogInfo(msg="[MasterLaunch] Stage 0: Reading configuration and paths...")

    
    ws_path = Path.home() / "ros2_clearpath_ws"
    ws_str = f"{ws_path.as_posix()}/"

    pkg_clearpath_nav2_demos = ws_path / "src" / "clearpath_nav2_demos"

    config_dir = pkg_clearpath_nav2_demos / "config" / "j100"

    nav2_params_file         = str(config_dir / "nav2.yaml")
    localization_params_file = str(config_dir / "localization.yaml")


    # Locate required packages using their share directory
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_clearpath_nav2_demos = get_package_share_directory('clearpath_nav2_demos')
    pkg_clearpath_manipulators = get_package_share_directory('clearpath_manipulators')
    pkg_clearpath_viz = get_package_share_directory('clearpath_viz')
    pkg_jackal_bringup = get_package_share_directory('jackal_bringup')
    pkg_llm_interface = get_package_share_directory('llm_interface')
    pkg_mobile_manipulation_coordinator = get_package_share_directory('mobile_manipulation_coordinator')
    # --- ADDED: jackal_perception package path
    pkg_jackal_perception = get_package_share_directory('jackal_perception')
    

    
    # Timings for each sequential stage.
    stage1_delay = 20.0
    stage2_delay = 28.0
    stage3_delay = 15.0
    

    # =========================================================================
    # === STAGE 1 (t = 0s): SIMULATION
    # =========================================================================
    launch_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_clearpath_gz, 'launch', 'simulation.launch.py')),
        launch_arguments={'setup_path': ws_str}.items()
    )


    # =========================================================================
    # === STAGE 2 (t = 20s): CORE CONTROL & LOCALIZATION
    # =========================================================================
    launch_core_systems = TimerAction(
        period=stage1_delay,
        actions=[
            LogInfo(msg=f"--- [MasterLaunch] Starting Stage 2 at {stage1_delay}s: Core Systems ---"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_clearpath_nav2_demos, 'launch', 'localization.launch.py')),
                launch_arguments={
                    'setup_path': ws_str, 
                    'namespace': 'j100_0000', 
                    'use_sim_time': 'true',
                    'params_file': localization_params_file
                }.items()
            ),
            TimerAction(
                period=10.0,
                actions=[
                    Node(
                        package='jackal_bringup',
                        executable='initial_pose_pub',
                        namespace='j100_0000',
                        output='screen',
                        parameters=[{"use_sim_time": True, "x": 0.0, "y": 0.0, "yaw": 0.0, "frame_id": "map"}]
                    )
                ]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_clearpath_manipulators, "launch", "moveit.launch.py")),
                launch_arguments={
                    "setup_path": ws_str,
                    "use_sim_time": "true",
                    "extra_semantic":
                        str(Path.home() /
                            "ros2_clearpath_ws/src/clearpath_common/clearpath_manipulators/config/world_joint.srdf")
                }.items()
            )
        ]
    )


    # =========================================================================
    # === STAGE 3 (t = 40s): NAVIGATION STACK
    # =========================================================================
    total_stage2_delay = stage1_delay + stage2_delay
    launch_navigation = TimerAction(
        period=total_stage2_delay,
        actions=[
            LogInfo(msg=f"--- [MasterLaunch] Starting Stage 3 at {total_stage2_delay}s: Navigation Stack ---"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_clearpath_nav2_demos, 'launch', 'nav2.launch.py')),
                launch_arguments={
                    'setup_path': ws_str,
                    'namespace': 'j100_0000',
                    'use_sim_time': 'true',
                    'params_file': nav2_params_file
                }.items()
            )
        ]
    )

    
    # =========================================================================
    # === STAGE 4 (t = 55s): APPLICATION & VISUALIZATION
    # =========================================================================
    total_stage3_delay = total_stage2_delay + stage3_delay
    launch_application_layer = TimerAction(
        period=total_stage3_delay,
        actions=[
            LogInfo(msg=f"--- [MasterLaunch] Starting Stage 4 at {total_stage3_delay}s: Application Layer ---"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_clearpath_viz, 'launch', 'view_navigation.launch.py')),
                launch_arguments={'namespace': 'j100_0000', 'use_sim_time': 'true'}.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_clearpath_viz, 'launch', 'view_moveit.launch.py')),
                launch_arguments={'namespace': 'j100_0000', 'use_sim_time': 'true'}.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_jackal_perception, 'launch', 'perception.launch.py')),
                launch_arguments={'use_sim_time': 'true'}.items()
            ),
            Node(
                package='mobile_manipulation_coordinator',
                executable='coordinator_v1',
                namespace='j100_0000',
                output='screen',
                parameters=[{"use_sim_time": True}]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_llm_interface, 'launch', 'chat.launch.py'))
            ),
            
            TimerAction(
                period=5.0,          
                actions=[
                    Node(
                        package='jackal_bringup',
                        executable='add_scene_objects',  
                        namespace='j100_0000',
                        output='screen',
                        parameters=[{"use_sim_time": True}])
                ])
        ]
    )


    # =========================================================================
    # === DESCRIPTION ASSEMBLY
    # =========================================================================
    return LaunchDescription([
        LogInfo(msg="[MasterLaunch] Starting the launch sequence..."),
        launch_simulation,
        launch_core_systems,
        launch_navigation,
        launch_application_layer,
        RegisterEventHandler(
            event_handler=OnShutdown(on_shutdown=[OpaqueFunction(function=_clean_up)])
        )
    ])
