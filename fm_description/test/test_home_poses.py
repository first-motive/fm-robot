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
Home-pose seeding for view_robot.launch.py.

``_load_home_pose`` resolves a robot/variant to the {joint: radians} pose the
launch passes to joint_state_publisher as its ``zeros`` parameter. It must
return an empty pose (jsp then defaults every joint to zero) for a missing file,
robot, or variant, and the authored pose for a listed one. The installed config
must carry the g1_29dof upright override — the reason the seeding exists.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory

PKG = "fm_description"


def _load_launch_module():
    """Load the installed view_robot launch file as an importable module."""
    share = get_package_share_directory(PKG)
    path = os.path.join(share, "launch", "view_robot.launch.py")
    spec = importlib.util.spec_from_file_location("view_robot_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, share


def _write_poses(share_dir, body):
    """Write a home_poses.yaml under share_dir/config and return share_dir."""
    config_dir = os.path.join(share_dir, "config")
    os.makedirs(config_dir, exist_ok=True)
    with open(os.path.join(config_dir, "home_poses.yaml"), "w") as f:
        f.write(body)
    return share_dir


def test_missing_file_yields_empty(tmp_path):
    """No config file at all → empty pose (jsp defaults every joint to zero)."""
    module, _ = _load_launch_module()
    assert module._load_home_pose(str(tmp_path), "g1_d", "g1_29dof_rev_1_0") == {}


def test_missing_robot_yields_empty(tmp_path):
    """A robot absent from the file → empty pose."""
    module, _ = _load_launch_module()
    share = _write_poses(tmp_path, "g1_d:\n  g1_29dof_rev_1_0:\n    x: 1.0\n")
    assert module._load_home_pose(share, "so101", "so101") == {}


def test_missing_variant_yields_empty(tmp_path):
    """A variant absent under a listed robot → empty pose."""
    module, _ = _load_launch_module()
    share = _write_poses(tmp_path, "g1_d:\n  g1_29dof_rev_1_0:\n    x: 1.0\n")
    assert module._load_home_pose(share, "g1_d", "g1_d") == {}


def test_listed_variant_returns_pose(tmp_path):
    """A listed robot/variant returns its authored {joint: radians} pose."""
    module, _ = _load_launch_module()
    share = _write_poses(
        tmp_path, "g1_d:\n  g1_29dof_rev_1_0:\n    a_joint: 0.2\n    b_joint: -0.3\n"
    )
    pose = module._load_home_pose(share, "g1_d", "g1_29dof_rev_1_0")
    assert pose == {"a_joint": 0.2, "b_joint": -0.3}


def test_installed_config_has_g1_29dof_override():
    """The shipped config carries the g1_29dof upright pose the seeding exists for."""
    module, share = _load_launch_module()
    pose = module._load_home_pose(share, "g1_d", "g1_29dof_rev_1_0")
    assert pose, "g1_29dof_rev_1_0 home pose missing from installed config"
    assert all(isinstance(v, (int, float)) for v in pose.values())
