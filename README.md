# Integrated Artificial Intelligence Framework for Autonomous Mobile Manipulation
**ROS 2 Humble · colcon · v0.2.0**

## Abstract
This repository provides the reference implementation for an integrated AI–robotics framework that translates **semantic tasking** (natural-language goals) into **navigation** and **manipulation** behaviors on a mobile robot. The system couples a language-aware coordinator with classical motion planning and visual perception to realize end-to-end **mobile manipulation** in simulation on a Clearpath Jackal with a 6-DoF arm. The codebase is organized for **reproducible builds**, and **extensible experimentation**.

## Research Objectives
- **O1 – Language to action:** Map high-level, human-readable commands to an executable plan (navigate → perceive → grasp/place), with explicit state transitions and error handling.
- **O2 – Perception-guided manipulation:** Detect and localize target objects and condition grasp/planning on perception quality and spatial uncertainty.
- **O3 – Reproducibility:** Provide a self-contained ROS 2 Humble workspace that can be cloned and built from scratch, with pinned external dependencies and runnable demos.

## System Architecture 
- **Coordinator (LLM interface & task graph):** Parses goals, selects behaviors, sequences Nav2 and MoveIt 2 actions, and supervises failure recovery.
- **Navigation (Nav2):** Goal reaching via global/local planning, costmaps, and behavior trees.
- **Manipulation (MoveIt 2):** Motion planning, collision checking, and execution for the arm; SRDF/URDF/Xacro define kinematics and scene.
- **Perception (YOLOv8-based):** Object detection and 3D pose estimation; results injected into grasp and place decisions.
- **Simulation (Gazebo/ign & Clearpath stack):** Jackal base with a manipulator, world assets, and demo launch files.

## Key Contributions
1. **Coordinator node** converting semantic commands to ROS 2 action pipelines with explicit state machine transitions.  
2. **Perception-conditioned manipulation** where detection confidence and pose drive grasp selection and placement feasibility.  
3. **Reproducible workspace** (Humble/colcon) with vendored modifications of Clearpath packages and scripts for one-command build/run.


## How to Recreate (Essentials Only)

**Requirements**
- Ubuntu 22.04
- ROS 2 Humble at `/opt/ros/humble`
- `colcon` build tool:
  ```bash
  sudo apt update
  sudo apt install -y python3-colcon-common-extensions

├── src/                                   # ROS 2 packages
│   ├── jackal_bringup/                    # simulation bringup
│   ├── jackal_perception/                 # YOLOv8-based perception (models/, params/, launch/)
│   ├── jackal_perception_interfaces/      # action/message definitions
│   ├── mobile_manipulation_coordinator/   # task sequencing/state machine
│   ├── llm_interface/                     # LLM integration and landmarks
│   ├── clearpath_common/                  # vendored Clearpath packages (modified)
│   ├── clearpath_simulator/               # simulator worlds/launchers
│   ├── clearpath_platform_*               # platform descriptions, msgs, URDF/Xacro
│   └── ...                                # other support pkgs
├── scripts/                               # reproducible workflow
│   ├── build.sh                           # rosdep + pip + colcon build
│   ├── install_rosdeps.sh                 # system/ROS deps
│   └── run_demo.sh                        # default demo launcher
├── robot.urdf.xacro                       # robot model (URDF/Xacro)
├── robot.srdf.xacro                       # semantic robot (SRDF/Xacro)
├── robot.yaml                             # robot configuration
├── requirements.txt                       # Python deps (e.g., openai)
└── (build/ install/ log/ ignored by git)

**Steps**
# 1) Clone (SSH shown; HTTPS+PAT also works)
git clone git@github.com:Ektwras/Integrated-Artificial-Intelligence-Framework-for-Autonomous-Mobile-Manipulation.git ws
cd ws

# 2) Build (installs deps → colcon build)
./scripts/build.sh

# 3) Source overlay (if sourcing manually)
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

# 4) Run demo
./scripts/run_demo.sh
# (launches: ros2 launch jackal_bringup jackal_sim.launch.py)


## Citation

If this framework is useful in your research, please cite:

@software{sofiannopoulos2025ai_mobmanip,
  author  = {Ektoras Sofianopoulos},
  title   = {Integrated AI Framework for Autonomous Mobile Manipulation},
  year    = {2025},
  version = {v0.2.0},
  url     = {https://github.com/Ektwras/Integrated-Artificial-Intelligence-Framework-for-Autonomous-Mobile-Manipulation}
}

## License

Apache License 2.0 © 2025 Ektoras Sofianopoulos
