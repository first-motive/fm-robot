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
Process the Panda xacro for each sim_backend and check the system it emits.

Runs against the installed share; needs the built franka_description (imported by
fm-robot.repos).
"""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
import yaml

PLUGIN_BY_BACKEND = {
    'mock': 'mock_components/GenericSystem',
    'mujoco': 'mujoco_ros2_control/MujocoSystemInterface',
    'gz': 'gz_ros2_control/GazeboSimSystem',
    'gazebo': 'gz_ros2_control/GazeboSimSystem',
    'isaac': 'topic_based_ros2_control/TopicBasedSystem',
}

COMMANDED_JOINTS = [f'fer_joint{i}' for i in range(1, 8)] + ['fer_finger_joint1']


def _process(**mappings):
    path = os.path.join(
        get_package_share_directory('fm_control'), 'urdf', 'panda.sim.urdf.xacro'
    )
    return ET.fromstring(xacro.process_file(path, mappings=mappings).toxml())


@pytest.mark.parametrize('backend,plugin', PLUGIN_BY_BACKEND.items())
def test_sim_backend_selects_plugin(backend, plugin):
    robot = _process(sim_backend=backend)
    assert robot.get('name') == 'panda'
    assert [s.get('name') for s in robot.findall('ros2_control')] == ['panda_system']
    assert [p.text for p in robot.findall('ros2_control/hardware/plugin')] == [plugin]


@pytest.mark.parametrize('backend', PLUGIN_BY_BACKEND)
def test_system_joints_exist_in_geometry(backend):
    robot = _process(sim_backend=backend)
    geometry = {j.get('name') for j in robot.findall('joint')}
    commanded = [
        j.get('name') for j in robot.findall('ros2_control/joint')
        if j.find('command_interface') is not None
    ]
    assert commanded == COMMANDED_JOINTS
    assert set(commanded) <= geometry


def test_spawn_pose_is_within_joint_limits():
    robot = _process(sim_backend='mock')
    limits = {
        j.get('name'): j.find('limit') for j in robot.findall('joint')
        if j.find('limit') is not None
    }
    for joint in robot.findall('ros2_control/joint'):
        name = joint.get('name')
        value = joint.find("state_interface[@name='position']/param[@name='initial_value']")
        if value is None:
            continue
        limit = limits.get(name)
        assert limit is not None, f'{name} has an initial_value but no <limit>'
        lower, upper = float(limit.get('lower')), float(limit.get('upper'))
        assert lower <= float(value.text) <= upper, f'{name} spawns outside its limits'


def test_gazebo_mimics_second_finger():
    robot = _process(sim_backend='gazebo')
    mimic = robot.find("ros2_control/joint[@name='fer_finger_joint2']")
    assert mimic is not None
    assert mimic.find('command_interface') is None
    assert robot.find('gazebo/plugin').get('filename') == 'gz_ros2_control-system'


@pytest.mark.parametrize('backend', ['real', 'gazeb'])
def test_unsupported_backend_fails(backend):
    with pytest.raises(xacro.XacroException, match=f'unknown sim_backend: {backend}'):
        _process(sim_backend=backend)


def test_spawn_pose_matches_description_home_pose():
    """The sim spawns where fm_description's view seeds the Panda (home_poses.yaml)."""
    path = os.path.join(
        get_package_share_directory('fm_description'), 'config', 'home_poses.yaml'
    )
    with open(path) as poses_file:
        home = yaml.safe_load(poses_file)['panda']['panda']
    robot = _process(sim_backend='mock')
    spawn = {
        j.get('name'): float(v.text) for j in robot.findall('ros2_control/joint')
        if (v := j.find("state_interface[@name='position']/param[@name='initial_value']"))
        is not None and float(v.text) != 0.0
    }
    assert spawn == home
