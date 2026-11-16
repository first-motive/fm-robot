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
Process the Axol xacro for each sim_backend and check the ros2_control systems.

Each arm contributes seven joints and its finger (16 in all); every joint takes a
position command and reports position, velocity and effort.

Runs against the installed share; needs the vendored axol description in
fm_description's share (imported by fm-robot.repos).
"""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro

ARM_JOINTS = ('s1', 's2', 's3', 'e1', 'e2', 'w1', 'w2')


def _process(**mappings):
    path = os.path.join(
        get_package_share_directory('fm_control'), 'urdf', 'axol.sim.urdf.xacro'
    )
    return ET.fromstring(xacro.process_file(path, mappings=mappings).toxml())


def _systems(robot):
    return {
        system.get('name'): [joint.get('name') for joint in system.findall('joint')]
        for system in robot.findall('ros2_control')
    }


def _initial_values(robot):
    return {
        joint.get('name'): {
            state.get('name'): float(state.find("param[@name='initial_value']").text)
            for state in joint.findall('state_interface')
        }
        for joint in robot.findall('ros2_control/joint')
    }


def _side_joints(side):
    return [f'{side}_{j}_0' for j in ARM_JOINTS] + [f'{side}_finger_1_joint']


ALL_JOINTS = set(_side_joints('left') + _side_joints('right'))


@pytest.mark.parametrize('backend', ['mujoco', 'gz', 'gazebo', 'real'])
def test_shared_backends_load_one_system_for_both_arms(backend):
    # One MujocoSystemInterface is one MJCF world, gz_ros2_control does not
    # initialise two GazeboSimSystem systems, and on real one shim owns both arms.
    systems = _systems(_process(sim_backend=backend))
    assert list(systems) == ['axol_system']
    assert len(systems['axol_system']) == 16
    assert set(systems['axol_system']) == ALL_JOINTS


@pytest.mark.parametrize('backend', ['mock', 'isaac'])
def test_other_backends_keep_one_system_per_arm(backend):
    systems = _systems(_process(sim_backend=backend))
    assert systems == {
        'axol_left_system': _side_joints('left'),
        'axol_right_system': _side_joints('right'),
    }


@pytest.mark.parametrize('backend', ['mock', 'mujoco', 'gazebo', 'isaac', 'real'])
def test_every_joint_takes_position_and_reports_effort(backend):
    for joint in _process(sim_backend=backend).findall('ros2_control/joint'):
        commands = [c.get('name') for c in joint.findall('command_interface')]
        states = [s.get('name') for s in joint.findall('state_interface')]
        assert commands == ['position'], joint.get('name')
        assert states == ['position', 'velocity', 'effort'], joint.get('name')


def test_mimic_fingers_are_left_to_the_description():
    robot = _process(sim_backend='real')
    claimed = {joint.get('name') for joint in robot.findall('ros2_control/joint')}
    assert not {'left_finger_2_joint', 'right_finger_2_joint'} & claimed


def test_real_backend_talks_to_the_axol_shim():
    robot = _process(sim_backend='real')
    hardware = robot.find('ros2_control/hardware')
    params = {p.get('name'): p.text for p in hardware.findall('param')}
    assert hardware.find('plugin').text == 'topic_based_ros2_control/TopicBasedSystem'
    assert params == {
        'joint_commands_topic': '/axol/joint_commands',
        'joint_states_topic': '/axol/joint_states',
    }


def test_isaac_keeps_its_own_topics():
    params = {
        p.get('name'): p.text
        for p in _process(sim_backend='isaac').findall('ros2_control/hardware/param')
    }
    assert params == {
        'joint_commands_topic': '/isaac_joint_commands',
        'joint_states_topic': '/isaac_joint_states',
    }


@pytest.mark.parametrize('backend', ['mujoco', 'gazebo', 'real'])
def test_single_system_spawns_the_per_arm_pose(backend):
    single = _initial_values(_process(sim_backend=backend))
    assert single == _initial_values(_process(sim_backend='mock'))
    assert single['left_e1_0']['position'] == 1.0
    assert single['right_e1_0']['position'] == -1.0


@pytest.mark.parametrize('backend', ['gz', 'gazebo'])
def test_gazebo_hosts_controller_manager(backend):
    robot = _process(sim_backend=backend)
    assert {p.text for p in robot.findall('ros2_control/hardware/plugin')} == {
        'gz_ros2_control/GazeboSimSystem'
    }
    assert robot.find('gazebo/plugin').get('filename') == 'gz_ros2_control-system'


def test_default_mount_is_the_ox_cart():
    links = {link.get('name') for link in _process().findall('link')}
    assert {'world', 'cart', 'root'} <= links


def test_mount_arg_selects_the_upstream_urdf():
    links = {link.get('name') for link in _process(mount='standalone').findall('link')}
    assert 'cart' not in links
    assert 'root' in links


def test_meshes_stay_relative_to_the_upstream_urdf():
    # Consumers (fm_bringup, fm_sim_bringup, view_robot) rewrite this prefix.
    meshes = [mesh.get('filename') for mesh in _process().iter('mesh')]
    assert meshes and all(m.startswith('meshes/') for m in meshes)
    assert any(m.startswith('meshes/ox_cart/') for m in meshes)


def test_unknown_backend_fails():
    with pytest.raises(xacro.XacroException, match='unknown sim_backend: gazeb'):
        _process(sim_backend='gazeb')
