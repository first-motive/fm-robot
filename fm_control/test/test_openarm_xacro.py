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
Process the OpenArm xacro for every sim_backend value and check what it emits.

Runs against the installed share, so it catches a broken include or a macro typo
that only surfaces when a launch processes the description.
"""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro

PLUGIN_BY_BACKEND = {
    'mock': 'mock_components/GenericSystem',
    'mujoco': 'mujoco_ros2_control/MujocoSystemInterface',
    'gz': 'gz_ros2_control/GazeboSimSystem',
    'gazebo': 'gz_ros2_control/GazeboSimSystem',
    'isaac': 'topic_based_ros2_control/TopicBasedSystem',
    'real': 'openarm_hardware/OpenArmHW',
}

CAMERA_LINKS = {
    'openarm_cam_chest_optical',
    'openarm_right_cam_wrist_mount',
    'openarm_right_cam_wrist_optical',
}


def _process(**mappings):
    path = os.path.join(
        get_package_share_directory('fm_control'), 'urdf', 'openarm.sim.urdf.xacro'
    )
    return ET.fromstring(xacro.process_file(path, mappings=mappings).toxml())


def _plugins(robot):
    return {p.text for p in robot.findall('ros2_control/hardware/plugin')}


def _links(robot):
    return {link.get('name') for link in robot.findall('link')}


@pytest.mark.parametrize('backend,plugin', PLUGIN_BY_BACKEND.items())
def test_sim_backend_selects_plugin(backend, plugin):
    robot = _process(robot_preset='default_bimanual', sim_backend=backend)
    assert _plugins(robot) == {plugin}
    assert CAMERA_LINKS <= _links(robot)


def test_default_backend_is_mock():
    assert _plugins(_process()) == {PLUGIN_BY_BACKEND['mock']}


@pytest.mark.parametrize('backend', ['gz', 'gazebo'])
def test_gazebo_hosts_controller_manager(backend):
    robot = _process(sim_backend=backend)
    assert robot.find('gazebo/plugin').get('filename') == 'gz_ros2_control-system'


def test_unknown_backend_fails():
    with pytest.raises(xacro.XacroException, match='unknown sim_backend: gazeb'):
        _process(sim_backend='gazeb')


def test_isaac_topics():
    params = {
        p.get('name'): p.text
        for p in _process(sim_backend='isaac').findall('ros2_control/hardware/param')
    }
    assert params['joint_commands_topic'] == '/isaac_joint_commands'
    assert params['joint_states_topic'] == '/isaac_joint_states'


@pytest.mark.parametrize(
    'preset,collapse',
    [('default_bimanual', 'true'), ('default_bimanual', 'false'), ('right_arm', 'true')],
)
def test_wrist_camera_rides_link7_frame(preset, collapse):
    # Whatever link the preset names, the wrist pose is measured from joint7's child.
    robot = _process(robot_preset=preset, collapse_internal_empty_links=collapse)
    mount = robot.find("joint[@name='openarm_right_cam_wrist_mount_joint']")
    joint7 = robot.find("joint[@name='openarm_right_joint7']")
    assert mount.find('parent').get('link') == joint7.find('child').get('link')


def test_cameras_follow_preset():
    # right_arm has no body to carry the chest camera; left_arm has no right wrist.
    assert 'openarm_cam_chest_optical' not in _links(_process(robot_preset='right_arm'))
    assert not CAMERA_LINKS & _links(_process(robot_preset='left_arm'))
