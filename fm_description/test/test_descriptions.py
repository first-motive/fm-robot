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

"""
Smoke-test every robot description across all variants.

Each registry entry in ``view_robot.launch.py`` builds a robot_description: the
G1 and SO101 entries read a flat URDF vendored into this package's share at build
time (see ``CMakeLists.txt``), and the OpenArm entry processes the built
``openarm_description`` xacro. A description "runs" when its build function
returns a non-empty URDF with mesh paths rewritten to ``package://``.

This guards the vendoring. When the external path broke on the nav2 flatten, the
URDFs were absent from share and launch raised ``G1 URDF not found`` — but the
build stayed green because vendoring is a guarded warning, not an error. These
tests turn that silent gap into a failing test by exercising the real build path
for every robot and variant.
"""

import importlib.util
import os
import re
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory

PKG = "fm_description"

# Every (robot, variant, mesh_marker) the registry supports. ``mesh_marker`` is
# the exact package:// prefix that robot/variant's build function rewrites mesh
# paths to — a per-case marker, not a generic one, so a build that returns the
# wrong robot's URDF or skips the rewrite fails instead of passing on an
# unrelated match. G1 variants come from ``_G1_VARIANT_DIRS``; SO101 ignores its
# variant; OpenArm presets mirror the validation list in ``_build_openarm`` (its
# visual .dae meshes are rewritten to fm_description; collision STL meshes stay
# on openarm_description by design). Panda ignores its variant and rewrites its
# visuals the same way, onto panda_meshes/. The coverage tests below assert this list
# stays in sync with the registry, so a robot or G1 variant added or removed
# without updating CASES fails CI.
CASES = [
    ("g1_d", "g1_d", "package://fm_description/g1_d_description/meshes/"),
    ("g1_d", "g1_29dof_rev_1_0", "package://fm_description/g1_description/meshes/"),
    ("so101", "so101", "package://fm_description/so101_description/assets/"),
    ("axol", "bimanual", "package://fm_description/axol_description/meshes/"),
    ("openarm", "right_arm", "package://fm_description/openarm_meshes/"),
    ("openarm", "left_arm", "package://fm_description/openarm_meshes/"),
    ("openarm", "default_bimanual", "package://fm_description/openarm_meshes/"),
    ("openarm", "right_arm_with_pinch_gripper", "package://fm_description/openarm_meshes/"),
    ("openarm", "left_arm_with_pinch_gripper", "package://fm_description/openarm_meshes/"),
    ("panda", "panda", "package://fm_description/panda_meshes/robots/fer/visual/"),
]


def _load_launch_module():
    """
    Load the installed view_robot launch file as an importable module.

    Launch files are not importable packages, so load by path from this
    package's share — the same files CI installs after the vendoring step.
    """
    share = get_package_share_directory(PKG)
    path = os.path.join(share, "launch", "view_robot.launch.py")
    spec = importlib.util.spec_from_file_location("view_robot_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, share


@pytest.mark.parametrize("robot,variant,mesh_marker", CASES)
def test_description_builds(robot, variant, mesh_marker):
    """Each robot/variant produces a valid URDF with its mesh paths rewritten."""
    module, share = _load_launch_module()
    entry = module.ROBOTS[robot]

    urdf = entry["build_description"](share, variant)

    assert urdf and "<robot" in urdf, f"{robot}/{variant}: empty or invalid URDF"
    assert mesh_marker in urdf, (
        f"{robot}/{variant}: mesh paths not rewritten to {mesh_marker}"
    )


def test_every_robot_has_a_case():
    """CASES covers exactly the registry's robots — no gaps, no stale entries."""
    module, _ = _load_launch_module()
    covered = {robot for robot, _, _ in CASES}
    assert covered == set(module.ROBOTS), (
        f"CASES robots out of sync with registry: "
        f"{covered ^ set(module.ROBOTS)}"
    )


def test_every_g1_variant_has_a_case():
    """CASES covers exactly the registry's G1 variants — no gaps, no stale entries."""
    module, _ = _load_launch_module()
    covered = {variant for robot, variant, _ in CASES if robot == "g1_d"}
    assert covered == set(module._G1_VARIANT_DIRS), (
        f"CASES G1 variants out of sync with registry: "
        f"{covered ^ set(module._G1_VARIANT_DIRS)}"
    )


def test_panda_uses_fer_joints_and_hand():
    """The Panda description carries the fer_* arm joints and both hand fingers."""
    module, share = _load_launch_module()
    urdf = module.ROBOTS["panda"]["build_description"](share, "panda")

    for i in range(1, 8):
        assert f'<joint name="fer_joint{i}"' in urdf
    assert '<joint name="fer_finger_joint1"' in urdf
    assert '<joint name="fer_finger_joint2"' in urdf
    assert "package://fm_description/panda_meshes/robot_ee/franka_hand_white/" in urdf
    assert ".dae" not in urdf, "a Panda visual .dae was left unrewritten"


def test_panda_visual_meshes_are_installed():
    """Every rewritten Panda visual URI names an STL the build installed."""
    module, share = _load_launch_module()
    urdf = module.ROBOTS["panda"]["build_description"](share, "panda")

    prefix = "package://fm_description/"
    uris = set(re.findall(r'"(package://fm_description/panda_meshes/[^"]+)"', urdf))
    assert uris, "no Panda visual mesh points at panda_meshes/"
    missing = [u for u in uris if not os.path.isfile(os.path.join(share, u[len(prefix):]))]
    assert not missing, f"Panda visual meshes not installed: {sorted(missing)}"


def test_every_panda_visual_has_a_material():
    """Each Panda visual names a material, so RViz does not fall back to red."""
    module, share = _load_launch_module()
    urdf = ET.fromstring(module.ROBOTS["panda"]["build_description"](share, "panda"))

    visuals = [(link.get("name"), visual) for link in urdf.iter("link")
               for visual in link.findall("visual")]
    assert visuals, "the Panda description has no visuals"
    # fer declares no materials of its own, so each one is the injected Franka white.
    other = [name for name, visual in visuals
             if visual.find("material") is None
             or visual.find("material").get("name") != "franka_white"]
    assert not other, f"Panda visuals without the Franka white material: {other}"


def test_panda_home_pose_names_real_joints():
    """Every joint in the Panda home pose exists in its description."""
    module, share = _load_launch_module()
    urdf = module.ROBOTS["panda"]["build_description"](share, "panda")

    pose = module._load_home_pose(share, "panda", "panda")
    assert pose, "panda home pose missing from installed config"
    unknown = [j for j in pose if f'<joint name="{j}"' not in urdf]
    assert not unknown, f"home pose names joints the Panda lacks: {unknown}"
