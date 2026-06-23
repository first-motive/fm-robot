"""Smoke-test every robot description across all variants.

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
# on openarm_description by design). The coverage tests below assert this list
# stays in sync with the registry, so a robot or G1 variant added or removed
# without updating CASES fails CI.
CASES = [
    ("g1_d", "g1_d", "package://fm_description/g1_d_description/meshes/"),
    ("g1_d", "g1_29dof_rev_1_0", "package://fm_description/g1_description/meshes/"),
    ("so101", "so101", "package://fm_description/so101_description/assets/"),
    ("openarm", "right_arm", "package://fm_description/openarm_meshes/"),
    ("openarm", "left_arm", "package://fm_description/openarm_meshes/"),
    ("openarm", "default_bimanual", "package://fm_description/openarm_meshes/"),
    ("openarm", "right_arm_with_pinch_gripper", "package://fm_description/openarm_meshes/"),
    ("openarm", "left_arm_with_pinch_gripper", "package://fm_description/openarm_meshes/"),
]


def _load_launch_module():
    """Load the installed view_robot launch file as an importable module.

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
