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


@pytest.mark.parametrize('backend', ['mujoco', 'gz', 'gazebo'])
def test_simulators_load_one_system_for_both_arms(backend):
    # One MujocoSystemInterface is one MJCF world, and gz_ros2_control does not
    # initialise two GazeboSimSystem systems; both get every arm in one.
    systems = _systems(_process(sim_backend=backend))
    assert list(systems) == ['axol_system']
    expected = {f'{side}_{j}_0' for side in ('left', 'right') for j in ARM_JOINTS}
    assert len(systems['axol_system']) == 14
    assert set(systems['axol_system']) == expected


@pytest.mark.parametrize('backend', ['mock', 'isaac'])
def test_other_backends_keep_one_system_per_arm(backend):
    systems = _systems(_process(sim_backend=backend))
    assert sorted(systems) == ['axol_left_system', 'axol_right_system']
    assert all(len(joints) == 7 for joints in systems.values())


@pytest.mark.parametrize('backend', ['mujoco', 'gazebo'])
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


@pytest.mark.parametrize('backend', ['gazeb', 'real'])
def test_unknown_backend_fails(backend):
    # Axol has no real hardware plugin yet, so real is as unknown as a typo.
    with pytest.raises(xacro.XacroException, match=f'unknown sim_backend: {backend}'):
        _process(sim_backend=backend)
