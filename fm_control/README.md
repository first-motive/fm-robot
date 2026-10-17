# fm_control

The backend-selectable `ros2_control` descriptions for the OpenArm, SO101, G1-D,
and Axol, the G1-D teleop bridge nodes, and a sim-only Jazzy MoveIt Servo launch
for the OpenArm. C++ (`ament_cmake`, C++17).

## Role

One control stack drives each robot across every simulator and the real hardware.
Each robot has a top-level xacro that wraps its geometry and injects First Motive's
own `<ros2_control>` system, so the controllers and interfaces never change — only the
`<hardware>` System plugin swaps, chosen by the `sim_backend` arg.

```
fm_bringup launches -> controllers -> ros2_control system (plugin per backend) -> sim or hardware
```

## Control Graph

ROS2 Control is two managers: `controller_manager` loads the controllers — the
robot controller subscribes the command, `joint_state_broadcaster` publishes
`/joint_states` — and `resource_manager` owns the hardware interfaces.
`robot_state_publisher` turns `/joint_states` into `/robot_description`.

![control](doc/diagrams/control.svg)

In simulation the joint states come from the sim, not a panel. The sim loads
`/robot_description` and runs a control plugin: it writes joint references into the
sim robot, reads the actual state back (the closed loop), drives from `/cmd_vel`,
publishes `/joint_states`, and emits the odom → base_link transform on `/tf`.

![control_robot](doc/diagrams/control_robot.svg)

## Layout

```
urdf/
  openarm.sim.urdf.xacro / openarm.ros2_control.xacro   OpenArm (7-DOF, preset-driven)
  so101.sim.urdf.xacro   / so101.ros2_control.xacro     SO101 (5-DOF arm + gripper)
  g1.sim.urdf.xacro      / g1.ros2_control.xacro        G1-D right arm (7-DOF)
  axol.sim.urdf.xacro    / axol.ros2_control.xacro      Axol bimanual (two 7-DOF arms, 14 joints)
src/
  control_node.cpp        placeholder node (unused by the backends below)
  g1_arm_sdk_bridge.cpp   G1-D real arm: Servo JointTrajectory -> unitree_hg/LowCmd
  g1_base_teleop.cpp      G1-D base: Twist -> Unitree AGV Move RPC
  pose_tracking_node.cpp  vision mirror: MoveIt Servo PoseTracking host (Humble only;
                          Jazzy's moveit_servo dropped PoseTracking, so it is not built)
include/fm_control/       pure command-building logic for the two bridge nodes
config/openarm/           OpenArm SRDF, KDL kinematics and sim Servo parameters
launch/
  openarm_sim_servo.launch.py  OpenArm right-arm MoveIt Servo (Jazzy, sim only)
test/                     gtest for the bridge logic; pytest for the OpenArm xacro
                          and the OpenArm sim Servo config and launch
```

## Backend → System Plugin

`sim_backend` selects one `<hardware>` plugin; everything above the
`ros2_control` system interface is identical across backends. The five plugins
all implement the same interface — the dashed block expands the real-hardware
case below.

![hardware](doc/diagrams/hardware.svg)

```
sim_backend   openarm                     so101                            g1_d                      axol
  mock          mock_components/GenericSystem (all robots)
  mujoco        mujoco_ros2_control/MujocoSystemInterface (all robots)
  gazebo        gz_ros2_control/GazeboSimSystem (all robots)
  isaac         topic_based_ros2_control/TopicBasedSystem (all robots)
  real          openarm_hardware/OpenArmHW   feetech_ros2_driver/FeetechHardwareInterface   (no plugin — bridge)   (deferred — no CAN plugin yet)
```

The sim backends are identical across robots: only the geometry + joint set differ.
The `real` backend is where the robots diverge — see the G1-D asymmetry below.

Every robot's `sim_backend` also accepts `gz`, the simulation stack's name for
Gazebo; `gz` and `gazebo` select the same plugin. Any value a robot does not list
above fails the xacro run rather than emitting a system with no `<hardware>`, so
`real` fails for the Axol. `urdf/sim_backend.xacro` holds that rule for all four.

### OpenArm Arguments

| `sim_backend` | System plugin | Notes |
|---------------|---------------|-------|
| `mock` (default) | `mock_components/GenericSystem` | perfect state echo |
| `mujoco` | `mujoco_ros2_control/MujocoSystemInterface` | MJCF from `mujoco_model:=` |
| `gz` or `gazebo` | `gz_ros2_control/GazeboSimSystem` | adds the `<gazebo>` plugin, controllers from `gazebo_controllers_file:=` |
| `isaac` | `topic_based_ros2_control/TopicBasedSystem` | `/isaac_joint_commands`, `/isaac_joint_states` |
| `real` | `openarm_hardware/OpenArmHW` | SocketCAN `can_interface:=`, `can_fd:=` |

```bash
xacro $(ros2 pkg prefix fm_control)/share/fm_control/urdf/openarm.sim.urdf.xacro \
  robot_preset:=default_bimanual sim_backend:=gz emit_grasp_frame:=true
```

`emit_grasp_frame` defaults to `false`. Pass `emit_grasp_frame:=true` for the
fm-skills `fm_openarm_v2` robot profile, whose `ee_frame` is
`openarm_*_grasp_frame`; without it that frame does not exist.

`hand` picks the gripper on a preset that has one:

| `hand` | Gripper | Arm limits | Source |
|--------|---------|------------|--------|
| `loader` (default) | parallel jaw: `openarm_*_hand`, prismatic `finger_joint1`/`2` over 0..0.05 m, `grasp_frame` 0.1801 m out from link7, approaching along its +z | the loader's: joint1 ±2.3562, joint6 -1.22173..0.785398 (left mirrored), lower velocities | `fm_description/config/openarm/loader_presets/`, `fm_description/urdf/openarm_parallel_hand.xacro` |
| `pinch` | openarm_description v2.0 pinch gripper: revolute fingers over ±0.7854 rad, `grasp_frame` 0.138 m out, approaching along its +x | openarm_description v2.0's | the stock `robot_preset` |

`loader` reproduces the arm fm-rob-01's loader runs, from the loader URDF snapshot in
fm-skills. Its arm kinematics are v2.0's; only the hand, the wrist and the limits
differ. v2.0 puts the wrist on its end effector, so `openarm_*_ee_base_link` carries the
loader's v1.0 link7 mesh and its 0.466 kg inertial. Only `default_bimanual` has a
loader variant. `right_arm` and `left_arm` carry no gripper and stay stock under
either value; any other gripper preset needs `hand:=pinch`.

The bimanual preset spawns both arms in a mirrored ready pose (the `initial_value` of
each joint). The 168 mm loader hands touch at the pinch pose, so with `hand:=loader`
joint2 opens from 0.0265 to 0.13 rad: the grasp frames sit 210 mm apart and the arms
keep at least 40 mm clear, fingers open or closed.

The OpenArm description also carries the chest and right wrist camera frames from
`fm_description/urdf/openarm_cameras.xacro`, on whichever links the preset enables.

## How It Is Built

A per-robot `{robot}.sim.urdf.xacro` includes the geometry and the
`{robot}.ros2_control.xacro`; that second file holds the `<hardware>` block the
`sim_backend` argument resolves.

![xacro](doc/diagrams/xacro.svg)

The launches process the xacro per robot + backend, e.g.:

```bash
xacro $(ros2 pkg prefix fm_control)/share/fm_control/urdf/so101.sim.urdf.xacro \
  sim_backend:=mujoco
```

Each emits its `<ros2_control>` system with joint names matching the geometry exactly,
and (for gazebo) the `<gazebo>` world plugin that hosts the controller_manager inside
the sim. SO101, G1-D, and Axol merge a vendored flat URDF from `fm_description`'s
share; OpenArm parses its preset-driven `openarm_description` xacro. The robot name
matches each MoveIt SRDF so Servo accepts the pair. Axol is bimanual, 14 joints
total.

The bimanual robots emit one system per arm on `mock` and `isaac`
(`openarm_left_system` + `openarm_right_system`, `axol_left_system` +
`axol_right_system`), and OpenArm also on `real`. On `mujoco` and `gazebo` both
arms share one system (`openarm_system`, `axol_system`). Each
`MujocoSystemInterface` loads its own copy of the MJCF, so per-arm systems would
step the arms in separate worlds, and gz_ros2_control never finishes initialising
two `GazeboSimSystem` systems. OpenArm's real hardware keeps per-arm systems
because each one drives its own CAN arm; Axol has no real backend yet.

Consumed by `fm_bringup`'s `sim.launch.py`, `servo.launch.py`, and `teleop.launch.py`.

## G1-D Real Arm: a Bridge, Not a ros2_control Plugin

No `ros2_control` hardware interface exists for the Unitree G1 upstream, so the G1-D
`real` arm is **not** a System plugin. Instead `g1.ros2_control.xacro` emits no system
for `sim_backend:=real`, and two nodes drive the robot directly:

```
g1_arm_sdk_bridge   /g1_right_arm_controller/joint_trajectory (Servo)
                      -> unitree_hg/LowCmd on rt/arm_sdk, 50 Hz
                      -> right-arm joints on motor_cmd[22..28], engagement weight on [29]

g1_base_teleop      /cmd_vel (Twist) -> unitree_api/Request (AGV Move, api_id 1001)
                      -> rt/api/agv/request
```

Both mirror the `unitree_sdk2` G1 examples. Reaching real hardware needs the Unitree
CycloneDDS RMW (so `rt/...` maps to the robot's DDS topics). They are plumbed,
build-checked, and unit-tested, but **untested on hardware** — none on hand yet.

## OpenArm MoveIt Servo (Jazzy, Sim Only)

`launch/openarm_sim_servo.launch.py` starts `/servo_node` for a simulated OpenArm
right arm, so JointJog on `/servo_node/delta_joint_cmds` (the fm-teleop `arm_joint`
channel) or TwistStamped on `/servo_node/delta_twist_cmds` drives the arm through
`/openarm_right_arm_controller/joint_trajectory`.

**Sim only.** Collision checking is off and `config/openarm/openarm.srdf` has no
collision matrix; the launch logs a warning saying so. Never run it against a real
arm. A real-arm Servo needs its own reviewed profile: a collision matrix with
collision checking on, and DDS domain isolation.

```bash
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  ros2 launch fm_control openarm_sim_servo.launch.py sim_only:=true command_type:=joint_jog
```

| Argument | Values | Effect |
|----------|--------|--------|
| `sim_only` | must be `true` (default `false`) | refuses to start otherwise |
| `command_type` | `joint_jog` (default), `twist` | calls `~/switch_command_type`; the launch shuts down if Servo rejects it |
| `robot_description` | URDF string, default empty | empty builds the URDF locally; a passed one must contain `openarm_body_link0` and `openarm_right_grasp_frame` |

Servo always runs on sim time (`use_sim_time: true`). The local URDF is
`default_bimanual` with `emit_grasp_frame:=true` and the default loader hand, the
model the SRDF names; the pinch hand gives the same chain. Its
`right_arm` group is a chain from `openarm_body_link0` to
`openarm_right_grasp_frame`, so Servo's planning frame and end effector are the
`fm_openarm_v2` profile's `base_frame` and right `ee_frame`. The running robot must
publish that URDF's joints on `/joint_states`; verified against the `mock` backend.

The launch refuses to start, with an error naming the cause, unless both guards hold:

- `sim_only:=true` is passed, so the caller states the arm is simulated (fm-sim's
  include passes it).
- Discovery is host-local, since anything on the same DDS domain could command the
  arm through `/servo_node`: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (the fm-sim
  agent sets it on every run) or `ROS_LOCALHOST_ONLY=1`.

Behaviour of Jazzy's Servo (moveit_servo 2.12.4), measured against the mock arm at
joint1 1.0 unitless (0.5 rad/s commanded):

- Servo accepts no command until a command type is selected; the launch selects it.
- Each command is applied until its header stamp is `incoming_command_timeout`
  (0.1 s) old on Servo's clock, then Servo halts smoothly. When a stream stamped
  with the current time stops, the arm stopped within 0.25 s and moved 0.05 rad
  further.
- Future stamps are not rejected: a command stamped 1 s ahead kept the arm moving
  for about 1.1 s after the stream stopped (0.34 rad further). Publishers must stamp
  with Servo's clock.
- Servo starts only after a joint state that changes once its loop is running. A
  simulator's physics provides that; a `mock` robot holds still, so move it once
  (any trajectory to its JTC) before jogging.
- `publish_period` must be under a third of `max_expected_latency`, or Servo exits
  at start.

## control_node

A stub (`ros2 run fm_control control_node`) — the sim/real backends above do not use
it. Keep or replace if a non-ros2_control path is ever needed.
