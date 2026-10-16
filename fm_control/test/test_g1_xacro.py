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
Process the G1-D xacro for each sim_backend and check the system it emits.

Runs against the installed share; needs the vendored G1-D description in
fm_description's share (imported by fm-robot.repos).
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
}


def _process(**mappings):
    path = os.path.join(
        get_package_share_directory('fm_control'), 'urdf', 'g1.sim.urdf.xacro'
    )
    return ET.fromstring(xacro.process_file(path, mappings=mappings).toxml())


@pytest.mark.parametrize('backend,plugin', PLUGIN_BY_BACKEND.items())
def test_sim_backend_selects_plugin(backend, plugin):
    robot = _process(sim_backend=backend)
    assert [s.get('name') for s in robot.findall('ros2_control')] == ['g1_system']
    assert [p.text for p in robot.findall('ros2_control/hardware/plugin')] == [plugin]


def test_real_emits_no_ros2_control():
    # The arm_sdk bridge drives the real arms out of band.
    assert _process(sim_backend='real').findall('ros2_control') == []


@pytest.mark.parametrize('backend', ['gz', 'gazebo'])
def test_gazebo_hosts_controller_manager(backend):
    robot = _process(sim_backend=backend)
    assert robot.find('gazebo/plugin').get('filename') == 'gz_ros2_control-system'


def test_unknown_backend_fails():
    with pytest.raises(xacro.XacroException, match='unknown sim_backend: gazeb'):
        _process(sim_backend='gazeb')
