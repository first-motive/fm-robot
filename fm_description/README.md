# fm_description

Robot description: URDF, xacro, and meshes. Feeds robot state / URDF to the graph.

## Role

`fm_description` owns the robot model and publishes robot state. The model is
authored as split-up URDF/xacro files — links, joints, geometries. `xacro`
processes them into one URDF that `robot_state_publisher` loads and publishes as
`/robot_description`.

![robot_state_publisher](doc/diagrams/robot_state_publisher.svg)

`view_robot.launch.py` publishes robot state only: `joint_state_publisher` turns
panel commands into joint states; `robot_state_publisher` turns those into TF and
the URDF. A viz client (RViz or Foxglove) subscribes these topics.

![view_robot](doc/diagrams/view_robot.svg)

| Topic | Type | Direction |
|-------|------|-----------|
| `/joint_command` | `sensor_msgs/JointState` | panel → joint_state_publisher |
| `/joint_states` | `sensor_msgs/JointState` | joint_state_publisher → robot_state_publisher |
| `/tf`, `/tf_static` | `geometry_msgs/TransformStamped` | robot_state_publisher → viz |
| `/robot_description` | URDF (XML) | robot_state_publisher → viz |

## Layout

```
urdf/     fm_robot.urdf.xacro   (placeholder — replace with the real robot)
meshes/   visual + collision geometry
launch/   description-only launch helpers
```

## Build Type

`ament_cmake`. Installs `urdf/`, `meshes/`, `launch/` to the package share.

## View Robots

One entry point renders any supported robot. `scripts/view-robot.sh` brings the
container stack up and serves the view to Foxglove Studio on the host;
`launch/view_robot.launch.py` holds an inline `ROBOTS` registry, one entry per
robot. Default robot is **g1_d** (the wheeled G1-D). Adding a robot is one new
registry entry plus one row in the table below — no new launch file or wrapper.

```
scripts/view-robot.sh --robot <key>
        │  docker compose up -d  →  ros2 launch fm_description view_robot.launch.py robot:=<key>
        ▼
ROBOTS[<key>].build_description(share, variant)  →  URDF (mesh refs rewritten to package://)
        ▼
robot_state_publisher → /robot_description, /tf, /tf_static
joint_state_publisher → /joint_states (default pose; source_list ← /joint_command)
foxglove_bridge       → ws://8765  (Foxglove Studio on the host renders it)
```

### Usage

Vendor the sources and build once, then launch any robot:

```bash
./scripts/import-externals.sh    # vendor / import robot sources into external/ (once)
docker compose -f docker/compose.yaml -f docker/compose.macos.yaml \
  run --rm fm_ros2 colcon build --symlink-install
./scripts/view-robot.sh                                  # g1_d wheeled G1-D (default)
./scripts/view-robot.sh --robot g1_d --variant g1_29dof_rev_1_0   # bipedal body
./scripts/view-robot.sh --robot so101
./scripts/view-robot.sh --robot openarm                  # right_arm
./scripts/view-robot.sh --robot openarm --variant default_bimanual
./scripts/view-robot.sh --robot axol                     # bimanual (two 7-DOF arms)
./scripts/view-robot.sh use_rviz:=true                   # RViz (needs an X display)
```

Then connect Foxglove Studio to `ws://localhost:8765`. `--robot` accepts hyphen
or underscore (`g1-d` == `g1_d`); any extra args pass straight through to
`ros2 launch`.

### Robots

The registry abstracts four robots — each entry defines its description source,
variants, and mesh strategy; the viewer and launchers select by `robot:=` and
`variant:=`.

![registry](doc/diagrams/registry.svg)

| `--robot` | `--variant` (default first) | Source | Mesh rewrite |
|-----------|------------------------------|--------|--------------|
| `g1_d` | `g1_d`, `g1_29dof_rev_1_0` | `unitree_ros` flat URDF, vendored into share | `meshes/` → `package://fm_description/<desc>/meshes/` |
| `so101` | _(none — single description)_ | `SO-ARM100` flat URDF, vendored into share | `assets/` → `package://fm_description/so101_description/assets/` |
| `openarm` | `right_arm`, `left_arm`, `default_bimanual`, `*_with_pinch_gripper` | `openarm_description` built ament_cmake package | visual `.dae` → `package://fm_description/openarm_meshes/*.stl` |
| `axol` | `bimanual` | Almond Bot axol (flat URDF + STL, vendored) | `package://assembly/` → `package://fm_description/axol_description/` |

- **g1_d** — both the wheeled G1-D and the bipedal 29 DOF body are installed; pick
  with `--variant`. The bipedal `g1_description` ships hand variants too (e.g.
  `g1_29dof_rev_1_0_with_inspire_hand_FTP`); the G1-D hand is not yet locked
  (Inspire U6 leading).
- **so101** — the upstream (`TheRobotStudio/SO-ARM100`) ships plain files, not a
  ROS package; `--variant` is ignored.
- **openarm** — `--variant` is the xacro `robot_preset` (mirrors upstream
  `display_openarm.launch.py`). The default `right_arm` disables the body and left
  arm. The preset's ros2_control include runs with fake hardware and is harmless
  for a view, so no disable flag is needed.
- **axol** — the upstream (`almond-bot/axol`) ships a flat URDF + STL, not a ROS
  package; the single `bimanual` description carries both 7-DOF arms, so
  `--variant` is ignored.

Common launch args (every robot):

| Arg | Default | Meaning |
|-----|---------|---------|
| `robot` | `g1_d` | registry key: `g1_d`, `so101`, `openarm`, `axol` |
| `variant` | _(empty → entry default)_ | robot sub-form (see the table above) |
| `use_foxglove` | `true` | start foxglove_bridge on `ws://8765` |
| `use_rviz` | `false` | start RViz (needs an X display) |
| `use_jsp` | `true` | start joint_state_publisher so non-fixed joints get TF |
| `panel_topic` | `/joint_command` | topic jsp subscribes to (point the Foxglove panel here) |

An unknown `--robot` key (shell) or `robot:=` value (launch) fails loud, listing
the valid keys.

### Saved Views

Each robot ships a saved default view for both viewers: base frame selected, an
Orbit camera at a working distance, `/robot_description` visible. The files live
under this package's share (`rviz/`, `foxglove/`).

| `--robot` (variant) | Base frame | RViz | Foxglove |
|---------------------|------------|------|----------|
| `g1_d` | `AGV_link` | `rviz/g1_d.rviz` | `foxglove/g1_d_view.json` |
| `g1_d` (`g1_29dof_rev_1_0`) | `pelvis` | `rviz/g1_29dof.rviz` | `foxglove/g1_29dof_view.json` |
| `so101` | `base_link` | `rviz/so101.rviz` | `foxglove/so101_view.json` |
| `axol` | `base` | `rviz/axol.rviz` | `foxglove/axol_view.json` |
| `openarm` | `openarm_right_base_link` | `rviz/openarm.rviz` | `foxglove/openarm_view.json` |

- **RViz** loads its view automatically. `use_rviz:=true` picks the config for the
  selected `robot`/`variant` and starts `rviz2 -d`; an unmapped variant opens bare
  RViz rather than failing the launch.
- **Foxglove** reads its layout host-side, so import it once in Foxglove Studio
  (Layouts → import from file). The layout pre-sets the 3D panel: Z-up meshes,
  follow the base frame, `/robot_description` visible.
- Add the **Joint State Publisher** panel by hand to drive movable joints — point
  it at `/joint_command` (see the flip-flop gotcha below). The teleop views under
  `fm_teleop_vision` add a camera panel on the vision image topic.

### Mesh Resolution

Every registry entry rewrites its mesh references to `package://fm_description/...`
before publishing the URDF. Foxglove Studio routes `package://` (and only
`package://`) to foxglove_bridge, which resolves it inside the container and
streams the bytes to Studio. Other schemes (`file://`, `http://`) are fetched
host-side by Studio and cannot see container files, so they fail. CMakeLists
installs every description into the package share to make the `package://` path
resolve; the bridge's default `asset_uri_allowlist` permits the simple paths
(OpenArm widens it — see below).

The g1_d and so101 entries read a flat vendored URDF and do a relative-path
rewrite (`meshes/` or `assets/` → `package://`). OpenArm instead ships as a built
ament_cmake package, so its entry processes the xacro at launch with
`xacro.process_file` and rewrites visual `.dae` references onto a converted STL
set — detailed below.

### Foxglove Gotcha: Meshes Tipped 90° About X

If a robot renders with correct link positions but every mesh rotated 90° about X,
set the Foxglove 3D panel's mesh up-axis to match ROS. Foxglove defaults to Y-up
and rotates meshes +90° about X; the vendored meshes are Z-up, so they end up
over-rotated. This is a display setting, not a URDF/TF issue (RViz is unaffected):

```
3D panel → settings → Scene → Mesh "up" axis → Z-up   (then Ctrl-R to refresh)
```

To skip this each time, import the robot's ready-made layout under `foxglove/`
(e.g. `foxglove/g1_d_view.json`; see [Saved Views](#saved-views) for the full
list). Each pre-sets the 3D panel: Z-up meshes, follow the base frame,
`/robot_description` visible.

### Foxglove Gotcha: Joint State Publisher Panel Flips Between Poses

If the robot oscillates between two poses when you open Foxglove's **Joint State
Publisher** panel, the panel and the headless `joint_state_publisher` node are both
publishing `/joint_states`, and `robot_state_publisher` interleaves them:

```
joint_state_publisher node ──► /joint_states (default pose, 10 Hz)
                                  ▲
Foxglove panel ───────────────────┘ (slider values)   → two publishers race → flip-flop
```

The launch wires `joint_state_publisher` with `source_list:=[/joint_command]`, so it
is the only `/joint_states` publisher and the panel feeds it instead. Point the panel
at that topic once:

```
Joint State Publisher panel → settings → Publish topic → /joint_command
```

Now the panel publishes `/joint_command`, jsp holds the last value and republishes a
single consistent `/joint_states`, and the flip-flop is gone. Override the topic with
`panel_topic:=<topic>` if you prefer a different name.

### OpenArm: Visual Mesh Conversion to STL

OpenArm's upstream visual meshes are COLLADA (`.dae`) with **inconsistent declared
up-axes**: the arm and pinch-gripper meshes are `Y_UP`, the body and parallel-link
meshes are `Z_UP`. The declared axes disagree, but the vertices are all authored in
one shared Z-up world frame — the `up_axis` tags are stale metadata, not a real
per-file rotation. Foxglove needs plain STL (it fetches `package://` STL through
foxglove_bridge and cannot render the `.dae`).

The fix is to **ignore `up_axis` and export the raw geometry**, which keeps every
mesh in the shared Z-up frame the URDF `<visual>` origins already expect. trimesh
does this — `trimesh.load(..., force="mesh")` applies the COLLADA node transforms
and concatenates geometry without re-rotating for `up_axis`:

```
.dae (any declared up_axis)  ──trimesh load + export (raw geometry)──▶  STL
```

An earlier converter used assimp, which **honoured** each file's `up_axis` on
export. That rotated the `Z_UP` body mesh 90 degrees relative to the `Y_UP` arm
meshes, so the assembled robot rendered with its stand lying flat while the arms
sat upright. A canary assertion on the body mesh (its long axis must stay Z) guards
against a future trimesh that starts applying `up_axis`.

`fm_description`'s build (see `CMakeLists.txt` +
`scripts/convert_openarm_visual_meshes.py`) runs this for every visual mesh and
installs the results into `share/fm_description/openarm_meshes/`, mirroring the
upstream path; the launch rewrites visual references onto them. Collision meshes are
already STL and are not rendered by default, so they are left pointing at
`openarm_description`.

### OpenArm: Foxglove Asset Allowlist and Send-Buffer Limit

OpenArm's mesh URIs run through a dotted directory, e.g.
`package://openarm_description/assets/robot/openarm_v2.0/meshes/arm/visual/link1.dae`.
foxglove_bridge's default `asset_uri_allowlist` regex permits only `[\w-]` in path
segments, so the dot in `openarm_v2.0` makes it reject every mesh with `Asset URI
not allowed` — the links show load errors and nothing appears. The openarm entry's
`bridge_params` widen the allowlist to permit dots (`[-\w.]`). The g1_d/so101 paths
have no dotted dirs, so they keep the default.

The same entry raises `send_buffer_limit` from its 10 MB default to 128 MB. The
`default_bimanual` preset includes a ~10.8 MB body mesh (`body_link0.dae`) that
exceeds the default; over the limit the bridge silently drops that asset and
resets the asset channel, so neighbouring meshes fail to load too. The default
`right_arm` preset stays well under 10 MB, but the raised limit lets every preset
render.

### Adding a Robot

Each robot is one entry in the `ROBOTS` dict in `launch/view_robot.launch.py`:

1. Vendor or build the source — flat URDF into the package share (like g1_d /
   so101), or a built ROS package left out of `COLCON_IGNORE` (like openarm) —
   and ensure the meshes install into the package share so `package://` resolves.
2. Add a `build_description(share, variant) -> urdf_xml` callable that reads the
   source and rewrites its mesh references to `package://fm_description/...`.
3. Register an entry: `label`, `default_variant`, `build_description`, and
   `bridge_params` (start from the default `{port, address}`; extend only if the
   vendor needs it, as openarm does for the dotted-path allowlist and send buffer).
4. Add a row to the [Robots](#robots) table here, and update the valid-key list in
   `scripts/view-robot.sh`.
