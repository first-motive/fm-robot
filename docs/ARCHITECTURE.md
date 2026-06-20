# Architecture

The robot layer of First Motive's ROS2 (Humble) stack: the packages that
describe and drive the physical robot. Three concerns stack one on the next —
**description** is the foundation, **control** sits on it, and the **hardware
abstraction** lets the same control stack drive a mock, three simulators, or real
hardware behind one interface.

This repo is the robot layer in isolation. For the full-system view — how the
robot layer fits under bringup, sim, teleop, and the data/policy loop — see
[`fm-ros2`](https://github.com/first-motive/fm-ros2).

## Packages

| Package | Build | Responsibility |
|---------|-------|----------------|
| `fm_description` | ament_cmake | URDF/xacro, mesh handling, multi-robot registry (G1-D, SO101, OpenArm), Foxglove layouts |
| `fm_control` | ament_cmake | Backend-selectable `ros2_control` description (mock/mujoco/gazebo/isaac/real) |
| `fm_sensors` | ament_python | Multi-sensor capture layer (placeholder stub) |
| `fm_robot` | ament_cmake (meta) | Metapackage tying the three together for a single install |

The dependency direction is the design contract: **`fm_description` is the
foundation**, and `fm_control` adds the control layer on top of it. Layers above
this repo (`fm_bringup` in [`fm-app`](https://github.com/first-motive/fm-app))
orchestrate both, but neither package here depends upward.

## Robot State

`fm_description/view_robot.launch.py` publishes robot state only:
`joint_state_publisher` turns panel commands into joint states;
`robot_state_publisher` turns those into TF + the URDF. A viz client subscribes
these topics, launched separately — not here.

![view_robot](diagrams/view_robot.svg)

The model source feeds in from the top: a URDF source file is expanded by `xacro`
into the robot description the node publishes. Links, joints, and geometries live
there. Source: [`diagrams/view_robot.d2`](diagrams/view_robot.d2).

| Topic | Type | Direction |
|-------|------|-----------|
| `/joint_command` | `sensor_msgs/JointState` | panel → joint_state_publisher |
| `/joint_states` | `sensor_msgs/JointState` | joint_state_publisher → robot_state_publisher |
| `/tf`, `/tf_static` | `geometry_msgs/TransformStamped` | robot_state_publisher → viz |
| `/robot_description` | URDF (XML) | robot_state_publisher → viz |

## Control

`fm_control` brings up the `ros2_control` graph. Controllers run inside the
`controller_manager` as loaded plugins: the robot controller subscribes a command;
`joint_state_broadcaster` publishes `/joint_states`. `robot_state_publisher`
subscribes `/joint_states` and publishes `/robot_description` + TF.

![controllers](diagrams/controllers.svg)

The `controller_manager` receives the description as a launch parameter, so there
is no `/robot_description` topic edge into it. Source:
[`diagrams/controllers.d2`](diagrams/controllers.d2).

## Hardware Abstraction Layer

This is the architectural crux. `fm_control` emits one `ros2_control` system whose
`<hardware>` plugin is chosen by the `sim_backend` argument. Everything above the
hardware interface — controllers, servo, description, teleop — is identical across
all backends.

![hardware](diagrams/hardware.svg)

Source: [`diagrams/hardware.d2`](diagrams/hardware.d2).

| Backend | Plugin | Host | Use |
|---------|--------|------|-----|
| `mock` | `mock_components/GenericSystem` | any CPU | State echo, no physics — fast smoke tests |
| `mujoco` | `mujoco_ros2_control/MujocoSystemInterface` | CPU (arm64 ok) | **Daily driver**, incl. macOS M5 |
| `gazebo` | `gz_ros2_control/GazeboSimSystem` | Linux GPU | Higher-fidelity sim |
| `isaac` | `topic_based_ros2_control/TopicBasedSystem` | Linux GPU + external Isaac | Photoreal sim over ROS topics |
| `real` | `openarm_hardware/OpenArmHW` | Linux native | CAN-FD to DM motors |

The mock, mujoco, gazebo, and isaac plugins are provided by
[`fm-sim`](https://github.com/first-motive/fm-sim); `real` binds OpenArm hardware
vendored as an external. This repo defines the interface they all bind to.

The xacro layering that makes this work:

```
openarm.sim.urdf.xacro          (top level)
  ├─ openarm_description geometry + preset YAML   → links, joints, meshes
  └─ openarm.ros2_control.xacro                   → one <ros2_control> per arm
       └─ hardware block selected by sim_backend  → plugin above
```

Because the swap happens at the `<hardware>` boundary, switching from MuJoCo to
real hardware is a launch argument, not a code change.

## Robot Registry

`fm_description` carries a registry that abstracts over three robots. Each entry
defines its description source, variants, and mesh strategy. The viewer and
launchers select by `robot:=` and `variant:=`.

```mermaid
flowchart TD
    reg[Robot Registry<br/>fm_description]
    reg --> g1[g1_d · default<br/>Unitree wheeled G1-D]
    reg --> so[so101<br/>LeRobot SO-ARM100]
    reg --> oa[openarm<br/>bimanual]

    g1 --> g1v[variants: g1_d · g1_29dof_rev_1_0]
    oa --> oav[variants: right_arm · default_bimanual ·<br/>*_with_pinch_gripper]

    g1 -.flat URDF + STL.-> g1mesh[vendored meshes]
    so -.flat URDF + STL.-> somesh[vendored meshes]
    oa -.DAE → STL convert.-> oamesh[openarm_meshes/*.stl]
```

Mesh handling differs by source: G1-D and SO101 ship flat URDF + STL files
vendored into the package, while OpenArm visual `.dae` meshes are converted to
`.stl` at build time so the Foxglove bridge can fetch them over the `package://`
scheme.

## Design Notes

| Principle | How it shows up | Payoff |
|-----------|-----------------|--------|
| **One interface, many backends** | `sim_backend` selects the `ros2_control` hardware plugin | Sim ↔ real is a launch arg; controllers and teleop never change |
| **Layered, one-way deps** | `description → control`; nothing depends upward | Lower layers stay testable and reusable; no cycles |
| **Description as foundation** | `fm_description` registry abstracts robot + variant + meshes | New robot is a registry entry, not a fork |

Per-package detail lives in each `<package>/README.md`.
