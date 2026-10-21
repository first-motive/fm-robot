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
Process openarm_loader.sim.urdf.xacro for every sim_backend and check what it adds.

The real description is private, so most tests include a stand-in: a small URDF with the
loader's link and joint names. The last test checks the real one where it is installed
and skips elsewhere.
"""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import pytest
import xacro

REAL = 'openarm_v2_anvil_description'
SIDES = ('follower_l_', 'follower_r_')
CAMERA_LINKS = ('chest_cam', 'waist_cam', 'left_wrist_cam', 'right_wrist_cam')
INITIAL_POSITION = 'state_interface[@name="position"]/param'
PROFILE_CAMERAS = {'cam_chest_optical': 'chest_cam', 'cam_wrist_r_optical': 'right_wrist_cam'}
PLUGIN_BY_BACKEND = {
    'mock': 'mock_components/GenericSystem',
    'mujoco': 'mujoco_ros2_control/MujocoSystemInterface',
    'gz': 'gz_ros2_control/GazeboSimSystem',
    'gazebo': 'gz_ros2_control/GazeboSimSystem',
    'isaac': 'topic_based_ros2_control/TopicBasedSystem',
}


def _stand_in_urdf():
    chain = [('world', 'follower_body_link0', 'follower_body_world_joint', 'fixed')]
    cameras = {'chest_cam': 'follower_body_link0', 'waist_cam': 'follower_body_link0',
               'left_wrist_cam': 'follower_l_hand', 'right_wrist_cam': 'follower_r_hand'}
    for side in SIDES:
        chain.append(('follower_body_link0', f'{side}link0',
                      f'{side}follower_body_link0_joint', 'fixed'))
        chain += [(f'{side}link{i - 1}', f'{side}link{i}', f'{side}joint{i}', 'revolute')
                  for i in range(1, 8)]
        chain += [(f'{side}link7', f'{side}hand', f'{side}openarm_hand_joint', 'fixed'),
                  (f'{side}hand', f'{side}hand_tcp', f'{side}hand_tcp_joint', 'fixed'),
                  (f'{side}hand', f'{side}right_finger', f'{side}finger_joint1', 'prismatic'),
                  (f'{side}hand', f'{side}left_finger', f'{side}finger_joint2', 'prismatic')]
    chain += [(parent, camera, f'{camera}_joint', 'fixed') for camera, parent in cameras.items()]
    links = ['world'] + [child for _, child, _, _ in chain]
    joints = [
        f'<joint name="{name}" type="{kind}"><parent link="{parent}"/><child link="{child}"/>'
        + ('<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>'
           if kind != 'fixed' else '')
        + '</joint>'
        for parent, child, name, kind in chain
    ]
    return ('<robot name="openarm">' + ''.join(f'<link name="{link}"/>' for link in links)
            + ''.join(joints) + '</robot>')


@pytest.fixture(scope='module')
def stand_in(tmp_path_factory):
    urdf = tmp_path_factory.mktemp('stand_in') / 'robot.urdf'
    urdf.write_text(_stand_in_urdf())
    return urdf


def _process(**mappings):
    path = os.path.join(
        get_package_share_directory('fm_control'), 'urdf', 'openarm_loader.sim.urdf.xacro'
    )
    return ET.fromstring(xacro.process_file(path, mappings=mappings).toxml())


def _names(robot, tag):
    return {element.get('name') for element in robot.findall(tag)}


def _system_joints(robot):
    return {system.get('name'): [joint.get('name') for joint in system.findall('joint')]
            for system in robot.findall('ros2_control')}


def _arm_joints(side):
    return [f'{side}joint{i}' for i in range(1, 8)] + [f'{side}finger_joint1']


@pytest.mark.parametrize('backend,plugin', PLUGIN_BY_BACKEND.items())
def test_sim_backend_selects_plugin(stand_in, backend, plugin):
    robot = _process(description=str(stand_in), sim_backend=backend)
    assert {p.text for p in robot.findall('ros2_control/hardware/plugin')} == {plugin}


def test_the_real_backend_is_refused(stand_in):
    with pytest.raises(xacro.XacroException, match='unknown sim_backend'):
        _process(description=str(stand_in), sim_backend='real')


@pytest.mark.parametrize('backend', ['mujoco', 'gazebo'])
def test_simulators_get_one_system_over_both_arms(stand_in, backend):
    robot = _process(description=str(stand_in), sim_backend=backend)
    both_arms = _arm_joints('follower_l_') + _arm_joints('follower_r_')
    assert _system_joints(robot) == {'openarm_system': both_arms}


@pytest.mark.parametrize('backend', ['mock', 'isaac'])
def test_other_backends_get_one_system_per_arm(stand_in, backend):
    robot = _process(description=str(stand_in), sim_backend=backend)
    assert _system_joints(robot) == {f'{side}system': _arm_joints(side) for side in SIDES}


def test_the_arms_spawn_as_mirrored_ready_poses(stand_in):
    robot = _process(description=str(stand_in), sim_backend='mock')
    initial = {joint.get('name'): float(joint.find(INITIAL_POSITION).text)
               for joint in robot.findall('ros2_control/joint')}
    right = [initial[f'follower_r_joint{i}'] for i in range(1, 8)]
    left = [initial[f'follower_l_joint{i}'] for i in range(1, 8)]
    assert right == [0.6816, 0.13, -0.3161, 1.1622, 0.0, 0.3, 0.0]
    assert left == [q if i == 3 else -q for i, q in enumerate(right)]


def test_the_description_passes_through_with_only_the_profile_cameras_added(stand_in):
    robot = _process(description=str(stand_in), sim_backend='mock')
    description = ET.parse(stand_in).getroot()
    assert _names(robot, 'link') == _names(description, 'link') | set(PROFILE_CAMERAS)
    camera_joints = {f'{camera}_joint' for camera in PROFILE_CAMERAS}
    assert _names(robot, 'joint') == _names(description, 'joint') | camera_joints
    parents = {joint.find('child').get('link'): joint.find('parent').get('link')
               for joint in robot.findall('joint')}
    assert {camera: parents[camera] for camera in PROFILE_CAMERAS} == PROFILE_CAMERAS


def test_the_real_description_passes_through():
    try:
        share = get_package_share_directory(REAL)
    except PackageNotFoundError:
        pytest.skip(f'{REAL} is private and not installed here')
    # A generated, flat URDF: no xacro or $(find) left in it to expand.
    description = ET.parse(os.path.join(share, 'urdf', 'robot.urdf')).getroot()
    robot = _process(sim_backend='mujoco')
    assert _names(robot, 'link') == _names(description, 'link') | set(PROFILE_CAMERAS)
    assert set(CAMERA_LINKS) <= _names(robot, 'link')
