# fm-robot

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

Robot layer for First Motive's ROS2 stack. Groups the URDF description, the
`ros2_control` controllers, and the sensor drivers — the packages that describe
and drive the physical robot.

Part of First Motive's ROS2 (Humble) stack. Builds standalone here; assembled
with the other six package repos by
[`fm-ros2`](https://github.com/first-motive/fm-ros2).

## Packages

| Package | Build | Role |
|---------|-------|------|
| `fm_description` | ament_cmake | URDF/xacro, meshes, and Foxglove layouts for the robot |
| `fm_control` | ament_cmake | `ros2_control` controllers and hardware wiring |
| `fm_sensors` | ament_python | Sensor drivers |
| `fm_robot` | ament_cmake | Metapackage tying the three together for a single install |

## Standalone Build

Clone into a colcon workspace's `src/`, pull dependencies, then build:

```bash
mkdir -p ws/src && cd ws/src
git clone https://github.com/first-motive/fm-robot.git
vcs import < fm-robot/fm-robot.repos     # externals (OpenArm + Unitree descriptions)
cd .. && colcon build --symlink-install
colcon test && colcon test-result --verbose
```

## Architecture

Three concerns stack one on the next: `fm_description` is the foundation,
`fm_control` adds the control layer, and the hardware abstraction lets the same
control stack drive a mock, three simulators, or real hardware behind one
`ros2_control` interface.

![hardware](docs/diagrams/hardware.svg)

Full diagrams and the design contract: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Governance

Owner-free-on-main — see [CONTRIBUTING.md](CONTRIBUTING.md) and
[`.github/CODEOWNERS`](.github/CODEOWNERS).
