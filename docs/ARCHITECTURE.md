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
| `fm_description` | ament_cmake | URDF/xacro, mesh handling, multi-robot registry (G1-D, SO101, OpenArm, Axol), Foxglove layouts |
| `fm_control` | ament_cmake | Backend-selectable `ros2_control` description (mock/mujoco/gazebo/isaac/real) |
| `fm_sensors` | ament_python | Multi-sensor capture layer (placeholder stub) |
| `fm_robot` | ament_cmake (meta) | Metapackage tying the three together for a single install |

The dependency direction is the design contract: **`fm_description` is the
foundation**, and `fm_control` adds the control layer on top of it. Layers above
this repo (`fm_bringup` in [`fm-app`](https://github.com/first-motive/fm-app))
orchestrate both, but neither package here depends upward.

![packages](diagrams/packages.svg)

Arrows are ament dependencies (build + exec, from each `package.xml`). `fm_robot`
is a metapackage — it builds nothing itself, it aggregates the three.
`fm_control` layers on `fm_description` (its `ros2_control` xacro includes the
robot URDF); `fm_sensors` is independent. Source:
[`diagrams/packages.d2`](diagrams/packages.d2).

## Layered Detail

Each layer's ROS graph, topics, and design notes live with the package that owns
it — diagrams sit in that package's `doc/diagrams/`:

- **Description** — robot state publishing, the view-robot graph, and the robot
  registry: [`fm_description/README.md`](../fm_description/README.md).
- **Control** — the `ros2_control` graph, the simulation drive loop, the hardware
  abstraction layer, and xacro composition:
  [`fm_control/README.md`](../fm_control/README.md).
- **Sensors** — capture-layer stub: [`fm_sensors/README.md`](../fm_sensors/README.md).

## Design Notes

| Principle | How it shows up | Payoff |
|-----------|-----------------|--------|
| **One interface, many backends** | `sim_backend` selects the `ros2_control` hardware plugin | Sim ↔ real is a launch arg; controllers and teleop never change |
| **Layered, one-way deps** | `description → control`; nothing depends upward | Lower layers stay testable and reusable; no cycles |
| **Description as foundation** | `fm_description` registry abstracts robot + variant + meshes | New robot is a registry entry, not a fork |

Per-package detail lives in each `<package>/README.md`.
