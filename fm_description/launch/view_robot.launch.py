# Copyright 2026 First Motive
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

r"""
View any supported robot URDF as a robot_description from one launch file.

This unifies the former per-robot views (G1, SO101, OpenArm) behind a single
inline ROBOTS registry. Each registry entry owns its quirks: where the
description comes from, how meshes are rewritten, and any foxglove_bridge tweaks.
Select an entry with the `robot` arg; pick a sub-form with `variant`. Adding a
new robot is one new registry entry — no new launch file.

Mesh resolution: every entry rewrites mesh references to
`package://fm_description/...`. That is the only scheme Foxglove Studio fetches
through the bridge — Studio routes `package://` to the bridge's
resource_retriever (in the container) and resolves every other scheme (file://,
http://) host-side, which cannot see container files. CMakeLists installs each
description into this package's share, so the package:// path resolves. The
bridge's default asset_uri_allowlist already permits package:// for the simple
paths; OpenArm widens it (dotted dir) — see its registry entry.

Each entry exposes:
  - label            short echo string
  - default_variant  used when the `variant` arg is empty
  - build_description (share, variant) -> urdf_xml callable
  - bridge_params    merged into the foxglove_bridge node's parameters
  - rviz_config      basename of the saved view under this package's rviz/ share.
                     Loaded with `rviz2 -d` so RViz opens on the robot's base
                     frame at a working distance instead of an empty scene. The
                     matching Foxglove layouts live under foxglove/ and are
                     imported host-side in Foxglove Studio (Layouts -> import).

Joint control: exactly one joint-state publisher runs and it is the SOLE
publisher of /joint_states. Headless joint_state_publisher is the default,
seeded at the robot's home pose (config/home_poses.yaml -> its `zeros` param) and
subscribed to /joint_command via source_list so the Foxglove Joint State
Publisher panel drives the joints without racing it. The rviz path (use_rviz)
swaps it for joint_state_publisher_gui (a native slider window) automatically,
since use_jsp_gui defaults to "auto" and follows the viewer — every frontend gets
the right joint control from the viewer choice alone. This is the description-view
path only — never add a standalone jsp against sim.launch.py, where
joint_state_broadcaster owns /joint_states.
"""

import os
import re

import xacro
import yaml

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "fm_description"

# Default foxglove_bridge parameters. Entries may extend (never shrink) these.
_DEFAULT_BRIDGE_PARAMS = {"port": 8765, "address": "0.0.0.0"}

# Home poses keyed robot -> variant -> {joint: radians}, installed into this
# package's share by CMakeLists. Only non-zero overrides are listed; jsp defaults
# every other movable joint to zero.
_HOME_POSES_REL = os.path.join("config", "home_poses.yaml")


def _load_home_pose(share, robot, variant):
    """Return the {joint: radians} home pose for robot/variant, or {} if none.

    A missing file, robot, or variant yields an empty pose — jsp then defaults
    every joint to zero, which is the correct rest pose for robots that omit an
    entry. Passed to jsp as its `zeros` param so the first /joint_states message
    already holds the pose.
    """
    path = os.path.join(share, _HOME_POSES_REL)
    if not os.path.isfile(path):
        return {}
    with open(path, "r") as f:
        poses = yaml.safe_load(f) or {}
    return poses.get(robot, {}).get(variant, {})


# --- G1 -------------------------------------------------------------------

# The G1 descriptions are installed into this package's share by CMakeLists,
# sourced from the vcs-imported unitree_ros (run import-externals.sh, then build).
# Default is g1_d_description: the wheeled G1-D (AGV base + arms). Switch to the
# bipedal 29 DOF body with variant:=g1_29dof_rev_1_0.
_G1_VARIANT_DIRS = {
    "g1_d": "g1_d_description",
    "g1_29dof_rev_1_0": "g1_description",
}


def _build_g1(share, variant):
    """Load a flat G1 URDF and rewrite relative mesh paths to package://."""
    desc_dir = _G1_VARIANT_DIRS.get(variant)
    if desc_dir is None:
        raise RuntimeError(
            f"Unknown G1 variant: {variant!r}. "
            f"Valid variants: {sorted(_G1_VARIANT_DIRS)}"
        )

    root = os.path.join(share, desc_dir)
    urdf_path = os.path.join(root, f"{variant}.urdf")
    if not os.path.isfile(urdf_path):
        raise RuntimeError(
            f"G1 URDF not found: {urdf_path}\n"
            "Import externals then build: ./scripts/import-externals.sh && colcon build"
        )

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # Rewrite relative mesh paths to package:// so Foxglove fetches them via the
    # bridge. The meshes live in this package's share (installed by CMakeLists)
    # under a dir named after the variant's description (g1_d_description, ...).
    return robot_description.replace(
        'filename="meshes/', f'filename="package://{PKG}/{desc_dir}/meshes/'
    )


# --- SO101 ----------------------------------------------------------------

# The SO101 description is installed into this package's share by CMakeLists,
# sourced from the vcs-imported SO-ARM100 working copy (run import-externals.sh,
# then build). It is a single flat URDF (so101_new_calib.urdf) plus assets/.


def _build_so101(share, variant):
    """Load the flat SO101 URDF and rewrite relative mesh paths to package://."""
    # SO101 has a single description; the variant arg is ignored.
    root = os.path.join(share, "so101_description")
    urdf_path = os.path.join(root, "so101_new_calib.urdf")
    if not os.path.isfile(urdf_path):
        raise RuntimeError(
            f"SO101 URDF not found: {urdf_path}\n"
            "Import externals then build: ./scripts/import-externals.sh && colcon build"
        )

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # Rewrite relative mesh paths to package:// so Foxglove fetches them via the
    # bridge. The meshes live in this package's share (installed by CMakeLists)
    # under so101_description/assets/.
    return robot_description.replace(
        'filename="assets/', f'filename="package://{PKG}/so101_description/assets/'
    )


# --- Axol -----------------------------------------------------------------

# The Axol description is installed into this package's share by CMakeLists,
# sourced from the vcs-imported axol working copy (run import-externals.sh, then
# build). It is a single flat bimanual URDF (axol.urdf) plus meshes/. The URDF
# already references meshes as package://assembly/meshes/... (its Onshape export
# name); we rewrite the "assembly" package to fm_description/axol_description so
# the bridge resolves them from this package's share.

# Match the yaw on the root joint (root -> base) to reorient the heading. Group 1
# keeps the opening tag and the xyz origin; the trailing rp=".." is replaced. Any
# whitespace between attributes is tolerated so a re-export cannot break the match.
_AXOL_ROOT_JOINT_RE = re.compile(
    r'(<joint name="fixed_node_to_root_joint_0"[^>]*>\s*<origin xyz="[^"]*")\s*rpy="[^"]*"'
)


def _build_axol(share, variant):
    """Load the flat Axol URDF and rewrite its package:// meshes to fm_description."""
    # Axol has a single bimanual description; the variant arg is ignored.
    root = os.path.join(share, "axol_description")
    urdf_path = os.path.join(root, "axol.urdf")
    if not os.path.isfile(urdf_path):
        raise RuntimeError(
            f"Axol URDF not found: {urdf_path}\n"
            "Import externals then build: ./scripts/import-externals.sh && colcon build"
        )

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # Rewrite the exported "assembly" package to this package's share so Foxglove
    # fetches the meshes via the bridge. The meshes live under
    # axol_description/meshes/ (installed by CMakeLists).
    robot_description = robot_description.replace(
        'filename="package://assembly/',
        f'filename="package://{PKG}/axol_description/',
    )

    # Rename the robot. The Onshape export names the URDF after its assembly
    # document ("assembly"); every other robot's URDF names itself after the
    # robot (G1_D, so101_new_calib, openarm_v20). Rename to "axol" so the
    # published /robot_description is consistent.
    robot_description = robot_description.replace(
        '<robot name="assembly">', '<robot name="axol">'
    )

    # Reorient the heading. The Onshape export lays the robot out facing -Y, so
    # it renders 90 degrees clockwise about Z relative to every other robot (ROS
    # convention is +X forward). The tree hangs off the fixed root joint
    # `root -> base`, which the export leaves at zero yaw; set its yaw to +90
    # degrees to rotate the whole robot anti-clockwise back to +X forward. Anchor
    # on the joint name (not its float xyz) so a re-export cannot silently no-op.
    return _AXOL_ROOT_JOINT_RE.sub(
        r'\1 rpy="0 0 1.5707963267948966"', robot_description
    )


# --- OpenArm --------------------------------------------------------------

# OpenArm differs from the G1 and SO101 views in how the description is sourced.
# The G1/SO101 paths vendor flat URDF files into this package's share and rewrite
# relative mesh paths to package://fm_description/... at launch. OpenArm instead
# ships as a real, built ament_cmake ROS package (enactic/openarm_description):
# import-externals.sh leaves it OUT of COLCON_IGNORE, so colcon build compiles it
# into the workspace. We therefore process its xacro at launch.
#
# Visual mesh conversion to STL. The upstream visual meshes are COLLADA (.dae) with
# inconsistent declared up-axes (arm and pinch gripper Y_UP, body and parallel-link
# Z_UP), but their vertices are all authored in one shared Z-up world frame — the
# up_axis tags are stale metadata. Foxglove needs plain STL (it cannot render the
# .dae). fm_description's build converts every OpenArm visual mesh with trimesh,
# which ignores up_axis and exports the raw geometry, keeping every mesh in the
# Z-up frame the URDF <visual> origins expect (see CMakeLists.txt). An earlier
# assimp converter honoured up_axis and rotated the Z_UP body 90 degrees, so the
# stand rendered flat while the arms stayed upright. We rewrite each visual
# reference from package://openarm_description/<rel>.dae to
# package://fm_description/openarm_meshes/<rel>.stl. Collision meshes are already
# STL, are not rendered by default, and are left pointing at openarm_description.

# Relative path to the xacro entry point inside the built openarm_description
# share. The package is an upstream ament_cmake package, not vendored here.
_OPENARM_XACRO_REL = "assets/robot/openarm_v2.0/urdf/openarm_v20.urdf.xacro"

# Rewrite visual COLLADA mesh references onto the Z-up STL set vendored into this
# package's share at build (see CMakeLists.txt). Only visual meshes are .dae, so
# matching the .dae suffix targets them without touching collision STL refs.
_VISUAL_MESH_RE = re.compile(r"package://openarm_description/([^\"']+?)\.dae")
_VISUAL_MESH_SUB = r"package://fm_description/openarm_meshes/\1.stl"

# Recolour OpenArm. The v2.0 xacro emits one <material> per visual, all set to a
# flat light grey (rgba 0.753) — it does NOT read the real colours baked into the
# COLLADA visual meshes (dominant diffuse "palette_01_matte_black" 0.247, silver
# 0.65 accents), and the DAE->STL conversion drops those baked colours too. So
# every OpenArm link renders flat grey, while G1 and SO101 carry real per-link
# colours in their URDFs and render coloured. We override the grey with OpenArm's
# dominant matte black. STL holds one colour per mesh, so the black+silver
# two-tone collapses to a single flat colour per link. A visual missing a
# material entirely is also covered (defensive — upstream currently gives all
# seven a material).
_OPENARM_COLOR = "0.247 0.247 0.247 1.0"
_OPENARM_MATERIAL = (
    f'<material name="openarm_matte_black">'
    f'<color rgba="{_OPENARM_COLOR}"/></material>'
)

# Override the colour inside every existing OpenArm <material> (the xacro names
# them openarm_*_material). Group 1 ends at rgba=", group 2 is the closing "/>.
_OPENARM_COLOR_RE = re.compile(r'(<material name="openarm[^"]*">\s*<color rgba=")[^"]*("\s*/>)')
_VISUAL_BLOCK_RE = re.compile(r"<visual\b[^>]*>.*?</visual>", re.DOTALL)


def _add_openarm_material(match):
    """Inject the OpenArm material into a <visual> block that lacks one."""
    block = match.group(0)
    if "<material" in block:
        return block
    return block.replace("</visual>", _OPENARM_MATERIAL + "</visual>")


def _build_openarm(share, variant):
    """Process the OpenArm xacro for a preset and rewrite .dae visuals to STL."""
    # Locate the built openarm_description package. It must be imported (without
    # COLCON_IGNORE) and built into the workspace before this launch can run.
    try:
        openarm_share = get_package_share_directory("openarm_description")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "openarm_description not found. It is a built ament_cmake package, "
            "not vendored into fm_description. Import and build it:\n"
            "  ./scripts/import-externals.sh   # imports openarm_description "
            "without COLCON_IGNORE\n"
            "  colcon build --symlink-install"
        ) from exc

    xacro_path = os.path.join(openarm_share, _OPENARM_XACRO_REL)
    if not os.path.isfile(xacro_path):
        raise RuntimeError(
            f"OpenArm xacro not found: {xacro_path}\n"
            "Import externals then build: "
            "./scripts/import-externals.sh && colcon build --symlink-install"
        )

    # The v2.0 xacro selects links through robot_preset (string). This is the
    # only model arg upstream display_openarm.launch.py passes for v2.0; the
    # ros2_control include runs with fake hardware and needs no disable flag.
    mappings = {"robot_preset": variant}

    try:
        doc = xacro.process_file(xacro_path, mappings=mappings)
        robot_description = doc.toxml()
        # Point visual meshes at the Z-up STL set in this package's share.
        robot_description = _VISUAL_MESH_RE.sub(_VISUAL_MESH_SUB, robot_description)
        # Recolour: override the flat grey the xacro assigns each link, then cover
        # any visual that has no material at all (see above).
        robot_description = _OPENARM_COLOR_RE.sub(
            rf"\g<1>{_OPENARM_COLOR}\g<2>", robot_description
        )
        robot_description = _VISUAL_BLOCK_RE.sub(
            _add_openarm_material, robot_description
        )
    except Exception as exc:
        raise RuntimeError(
            f"Failed to process OpenArm xacro: {xacro_path}\n"
            f"mappings={mappings}\n"
            "Valid robot_preset values: default_bimanual, right_arm, left_arm, "
            "right_arm_with_pinch_gripper, left_arm_with_pinch_gripper. See "
            "upstream display_openarm.launch.py for the reference mappings."
        ) from exc

    return robot_description


# OpenArm's bridge params extend the default. Its package:// paths run through a
# dotted directory (openarm_v2.0), and foxglove_bridge's default
# asset_uri_allowlist regex permits only [\w-] in path segments — so it rejects
# every mesh with "Asset URI not allowed", and nothing renders. [-\w.] permits the
# dot. send_buffer_limit is raised above its 10 MB default because the
# default_bimanual preset includes a ~10.8 MB body mesh (body_link0.dae) that
# exceeds it, which silently drops that asset (and resets the asset channel, so
# sibling meshes fail too). 128 MB covers every preset with headroom.
_OPENARM_BRIDGE_PARAMS = {
    **_DEFAULT_BRIDGE_PARAMS,
    "send_buffer_limit": 134217728,
    "asset_uri_allowlist": [
        r"^package://(?:[-\w.]+/)*[-\w.]+"
        r"\.(?:dae|stl|obj|glb|gltf|mtl|png|jpe?g|tiff?)$"
    ],
}


# --- Registry -------------------------------------------------------------

# RViz view configs live under this package's rviz/ share (installed by
# CMakeLists). Each sets Fixed Frame to the robot's base link and an Orbit camera
# at a working distance. The g1_d entry serves two bodies with different roots
# (wheeled AGV_link vs bipedal pelvis), so its config is keyed by variant; an
# unmapped variant falls back to bare RViz. The other entries share one root
# across variants, so a plain basename covers them.
ROBOTS = {
    "g1_d": {
        "label": "Unitree G1 (G1-D)",
        "default_variant": "g1_d",
        "build_description": _build_g1,
        "bridge_params": _DEFAULT_BRIDGE_PARAMS,
        "rviz_config": {
            "g1_d": "g1_d.rviz",
            "g1_29dof_rev_1_0": "g1_29dof.rviz",
        },
    },
    "so101": {
        "label": "LeRobot SO101",
        "default_variant": "so101",
        "build_description": _build_so101,
        "bridge_params": _DEFAULT_BRIDGE_PARAMS,
        "rviz_config": "so101.rviz",
    },
    "axol": {
        "label": "Almond Bot Axol",
        "default_variant": "bimanual",
        "build_description": _build_axol,
        "bridge_params": _DEFAULT_BRIDGE_PARAMS,
        "rviz_config": "axol.rviz",
    },
    "openarm": {
        "label": "Enactic OpenArm",
        "default_variant": "right_arm",
        "build_description": _build_openarm,
        "bridge_params": _OPENARM_BRIDGE_PARAMS,
        "rviz_config": "openarm.rviz",
    },
}


def _resolve_rviz_config(entry, variant, share):
    """Return the absolute path to the entry's RViz view, or None.

    The registry value is a basename shared across variants, or a dict mapping
    variant -> basename when one robot key serves multiple roots. A missing key
    or absent file yields None so RViz opens bare rather than failing the launch.
    """
    config = entry.get("rviz_config")
    if isinstance(config, dict):
        config = config.get(variant)
    if not config:
        return None
    path = os.path.join(share, "rviz", config)
    return path if os.path.isfile(path) else None


def _launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    variant = LaunchConfiguration("variant").perform(context)
    use_foxglove = LaunchConfiguration("use_foxglove").perform(context) == "true"
    use_rviz = LaunchConfiguration("use_rviz").perform(context) == "true"
    use_jsp = LaunchConfiguration("use_jsp").perform(context) == "true"
    panel_topic = LaunchConfiguration("panel_topic").perform(context)

    # Resolve which joint-state publisher runs, if any, into (run_jsp, use_gui):
    #   run_jsp  a publisher runs at all
    #   use_gui  it is joint_state_publisher_gui (native sliders) vs headless jsp
    # use_jsp_gui defaults to "auto": follow the viewer. RViz has no joint panel
    # of its own, so the rviz path needs the gui; the foxglove path keeps headless
    # jsp and drives joints from the in-panel Joint State Publisher. Deriving this
    # here — not in each frontend — means every entryway (TUI, CLI, run.sh, FM
    # Desktop) gets the right joint control from the viewer choice alone. auto
    # respects use_jsp (so use_jsp:=false still runs nothing). Explicit "true"
    # forces the gui even over use_jsp:=false; "false" forces headless.
    raw_jsp_gui = LaunchConfiguration("use_jsp_gui").perform(context).strip().lower()
    if raw_jsp_gui == "true":
        run_jsp, use_gui = True, True
    elif raw_jsp_gui == "false":
        run_jsp, use_gui = use_jsp, False
    else:  # auto
        run_jsp, use_gui = use_jsp, (use_jsp and use_rviz)

    entry = ROBOTS.get(robot)
    if entry is None:
        raise RuntimeError(
            f"Unknown robot: {robot!r}. Valid robots: {sorted(ROBOTS)}"
        )

    if not variant:
        variant = entry["default_variant"]

    share = get_package_share_directory(PKG)
    robot_description = entry["build_description"](share, variant)

    # Home pose for this robot/variant. Passed to jsp as `zeros` so its first
    # /joint_states message already holds the pose (no post-hoc repositioning).
    # Empty for robots that omit an entry — jsp then defaults every joint to zero.
    home_pose = _load_home_pose(share, robot, variant)
    # Only include `zeros` when the pose is non-empty. launch_ros flattens a dict
    # param into zeros.<joint> entries; an empty dict is a value of ambiguous type.
    jsp_params = {
        "robot_description": robot_description,
        "source_list": [panel_topic],
    }
    if home_pose:
        jsp_params["zeros"] = home_pose

    # Load the robot's saved RViz view (Fixed Frame + framed Orbit camera) when
    # one exists; otherwise RViz opens bare. Foxglove reads its layout host-side,
    # so only RViz is wired here.
    rviz_config = _resolve_rviz_config(entry, variant, share)
    rviz_args = ["-d", rviz_config] if rviz_config else []

    # At most one joint-state publisher runs (resolved above into run_jsp +
    # use_gui), and it is the SOLE publisher of /joint_states. Headless jsp and
    # jsp_gui are mutually exclusive by construction — never both — so the
    # single-publisher invariant holds on either viewer path. Both take the same
    # params:
    #   - source_list=[panel_topic] subscribes to the Foxglove panel's
    #     /joint_command so the panel drives joints WITHOUT publishing
    #     /joint_states itself (two publishers race; the robot flips between
    #     poses). jsp holds the last value and republishes one consistent stream.
    #   - zeros seeds the home pose so the first /joint_states message is upright.
    joint_pub = None
    if run_jsp and use_gui:
        joint_pub = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            parameters=[jsp_params],
        )
    elif run_jsp:
        joint_pub = Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[jsp_params],
        )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
    ]

    if use_rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=rviz_args,
            )
        )

    # The chosen joint-state publisher (jsp or jsp_gui), when enabled.
    if joint_pub is not None:
        nodes.append(joint_pub)

    if use_foxglove:
        nodes.append(
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[entry["bridge_params"]],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot",
                default_value="g1_d",
                description=(
                    "Registry key selecting the robot: g1_d, so101, axol, openarm."
                ),
            ),
            DeclareLaunchArgument(
                "variant",
                default_value="",
                description=(
                    "Robot sub-form (empty uses the entry's default): G1 URDF "
                    "basename (g1_d, g1_29dof_rev_1_0) or OpenArm robot_preset "
                    "(right_arm, left_arm, default_bimanual, *_with_pinch_gripper). "
                    "Ignored for so101 and axol."
                ),
            ),
            DeclareLaunchArgument(
                "use_foxglove",
                default_value="true",
                description="Start foxglove_bridge on ws://0.0.0.0:8765.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Start RViz (needs an X display; Foxglove is the macOS path).",
            ),
            DeclareLaunchArgument(
                "use_jsp",
                default_value="true",
                description="Start a joint-state publisher so non-fixed joints get TF.",
            ),
            DeclareLaunchArgument(
                "use_jsp_gui",
                default_value="auto",
                description=(
                    "auto (default) follows the viewer: rviz gets "
                    "joint_state_publisher_gui (native sliders), foxglove keeps "
                    "headless joint_state_publisher (driven by the in-panel Joint "
                    "State Publisher). true/false force it. Wins over use_jsp; the "
                    "two never run together, so /joint_states keeps one publisher."
                ),
            ),
            DeclareLaunchArgument(
                "panel_topic",
                default_value="/joint_command",
                description=(
                    "Topic joint_state_publisher subscribes to via source_list. "
                    "Set the Foxglove Joint State Publisher panel to publish here "
                    "(not /joint_states) so it feeds jsp instead of racing it."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
