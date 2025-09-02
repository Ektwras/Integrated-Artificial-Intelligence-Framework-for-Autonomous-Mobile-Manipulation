# Integrated Artificial Intelligence Framework for Autonomous Mobile Manipulation
**ROS 2 Humble · colcon · v0.2.0**

## Abstract
This repository provides the reference implementation for an integrated AI–robotics framework that translates **semantic tasking** (natural-language goals) into **navigation** and **manipulation** behaviors on a mobile robot. The system couples a language-aware coordinator with classical motion planning and modern visual perception to realize end-to-end **mobile manipulation** in simulation on a Clearpath Jackal with a 6-DoF arm. The codebase is organized for **reproducible builds**, **transparent configuration**, and **extensible experimentation**.

## Research Objectives
- **O1 – Language to action:** Map high-level, human-readable commands to an executable plan (navigate → perceive → grasp/place), with explicit state transitions and error handling.
- **O2 – Perception-guided manipulation:** Detect and localize target objects and condition grasp/planning on perception quality and spatial uncertainty.
- **O3 – Reproducibility:** Provide a self-contained ROS 2 Humble workspace that can be cloned and built from scratch, with pinned external dependencies and runnable demos.

## System Architecture (High-Level)
- **Coordinator (LLM interface & task graph):** Parses goals, selects behaviors, sequences Nav2 and MoveIt 2 actions, and supervises failure recovery.
- **Navigation (Nav2):** Goal reaching via global/local planning, costmaps, and behavior trees.
- **Manipulation (MoveIt 2):** Motion planning, collision checking, and execution for the arm; SRDF/URDF/Xacro define kinematics and scene.
- **Perception (YOLOv8-based):** Object detection and 3D pose estimation; results injected into grasp and place decisions.
- **Simulation (Gazebo/ign & Clearpath stack):** Jackal base with a manipulator, world assets, and demo launch files.

## Key Contributions
1. **Coordinator node** converting semantic commands to ROS 2 action pipelines with explicit state machine transitions.  
2. **Perception-conditioned manipulation** where detection confidence and pose drive grasp selection and placement feasibility.  
3. **Reproducible workspace** (Humble/colcon) with vendored modifications of Clearpath packages and scripts for one-command build/run.  

## Repository Layout
```plaintext
├── build/            # local build artifacts (ignored)
├── install/          # local install artifacts (ignored)
├── log/              # local logs (ignored)
├── manipulators/     # manipulator configs/aux files
├── maps/             # navigation maps (if used)
├── platform/         # platform descriptions/config
├── sensors/          # sensor configs
├── src/              # ROS 2 packages (authored + vendored)
├── dependencies.repos
├── robot.srdf.xacro  # SRDF/Xacro (semantic robot)
├── robot.urdf.xacro  # URDF/Xacro  (robot model)
└── robot.yaml        # configuration parameters
