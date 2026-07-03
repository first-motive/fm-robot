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
Joint-state-publisher selection in view_robot.launch.py.

The launch picks at most one joint-state publisher and keeps it the sole
/joint_states publisher. use_jsp_gui defaults to "auto": it follows the viewer so
every frontend (TUI, CLI, run.sh, FM Desktop) gets the right joint control from
the viewer choice alone — rviz gets joint_state_publisher_gui (native sliders),
foxglove keeps headless joint_state_publisher (driven by the in-panel Foxglove
Joint State Publisher). These tests pin that resolution across the matrix so a
regression (e.g. two publishers, or the wrong one on a viewer) fails CI.
"""

import importlib.util
import os

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

PKG = "fm_description"


def _module():
    share = get_package_share_directory(PKG)
    path = os.path.join(share, "launch", "view_robot.launch.py")
    spec = importlib.util.spec_from_file_location("view_robot_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _joint_pub(use_rviz, use_jsp, use_jsp_gui):
    """Run _launch_setup headless and return the joint-pub executable (or None)."""
    module = _module()
    ctx = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            default = entity.default_value[0].perform(ctx) if entity.default_value else ""
            ctx.launch_configurations.setdefault(entity.name, default)
    ctx.launch_configurations.update(
        {
            "robot": "g1_d",
            "variant": "g1_29dof_rev_1_0",
            "use_foxglove": "false",
            "use_rviz": use_rviz,
            "use_jsp": use_jsp,
            "use_jsp_gui": use_jsp_gui,
        }
    )
    execs = [node.node_executable for node in module._launch_setup(ctx)]
    joint = [e for e in execs if "joint_state_publisher" in e]
    assert len(joint) <= 1, f"more than one joint publisher: {joint}"
    return joint[0] if joint else None


# (use_rviz, use_jsp, use_jsp_gui) -> expected joint-pub executable (or None).
CASES = [
    # auto follows the viewer: foxglove -> headless, rviz -> gui.
    ("false", "true", "auto", "joint_state_publisher"),
    ("true", "true", "auto", "joint_state_publisher_gui"),
    # explicit overrides win over auto.
    ("true", "true", "false", "joint_state_publisher"),
    ("false", "true", "true", "joint_state_publisher_gui"),
    # use_jsp:=false opts out entirely — auto must not resurrect a publisher on
    # the rviz path (a second /joint_states source is the flicker we avoid).
    ("true", "false", "auto", None),
    ("false", "false", "auto", None),
    # explicit gui still forces a publisher even when use_jsp is off.
    ("false", "false", "true", "joint_state_publisher_gui"),
]


@pytest.mark.parametrize("use_rviz,use_jsp,use_jsp_gui,expected", CASES)
def test_joint_publisher_selection(use_rviz, use_jsp, use_jsp_gui, expected):
    assert _joint_pub(use_rviz, use_jsp, use_jsp_gui) == expected
