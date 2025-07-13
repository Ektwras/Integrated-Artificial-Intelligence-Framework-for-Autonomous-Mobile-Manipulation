# Integrated Artificial Intelligence Framework for Autonomous Mobile Manipulation

**ROS 2 Humble · colcon**

## Overview

A unified codebase for semantic tasking → motion planning → manipulation → visual perception.

## Prerequisites

- **Ubuntu 22.04**
- **ROS 2 Humble**
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/ros2_clearpath_ws/install/setup.bash
  ```
- **Colcon**
  ```bash
  sudo apt update
  sudo apt install python3-colcon-common-extensions
  ```
- **System dependencies** (e.g., OpenCV, PCL)

## Workspace layout

```plaintext
├── build/            # local build artifacts (ignored)
├── install/          # local install artifacts (ignored)
├── log/              # local logs (ignored)
├── manipulators/     
├── maps/             # map files
├── platform/         # platform configs
├── sensors/          # sensor setups
├── src/               # ROS 2 packages 
├── dependencies.repos
├── robot.srdf.xacro
├── robot.urdf.xacro
└── robot.yaml
```

## Installation

```bash
git clone https://github.com/Ektwras/Integrated-Artificial-Intelligence-Framework-for-Autonomous-Mobile-Manipulation.git
cd Integrated-Artificial-Intelligence-Framework-for-Autonomous-Mobile-Manipulation

# vendor any external packages listed in dependencies.repos
vcs import src < dependencies.repos

# build the workspace
colcon build --symlink-install
```

## Usage

```bash
source install/setup.bash
ros2 launch <your_package> <your_launch_file>.launch.py
```

## Contributing

1. Fork this repo & create a branch off `main`.
2. Make changes & `git commit -m "feat: …"`.
3. Push & open a Pull Request against `main`.

## License

Apache 2.0 © 2025 Ektwras

