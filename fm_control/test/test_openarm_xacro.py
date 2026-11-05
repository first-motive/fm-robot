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
import numpy as np
import pytest
from urdf_kinematics import rotation_rpy, world_pose
import xacro

PLUGIN_BY_BACKEND = {
    'mock': 'mock_components/GenericSystem',
    'mujoco': 'mujoco_ros2_control/MujocoSystemInterface',
    'gz': 'gz_ros2_control/GazeboSimSystem',
    'gazebo': 'gz_ros2_control/GazeboSimSystem',
    'isaac': 'topic_based_ros2_control/TopicBasedSystem',
    'real': 'openarm_hardware/OpenArmHW',
}

BODY_CAMERA_LINKS = {'openarm_cam_chest_optical', 'openarm_cam_waist_optical'}
CAMERA_LINKS = BODY_CAMERA_LINKS | {f'openarm_{side}_cam_wrist_optical'
                                    for side in ('right', 'left')}
# Anvil's CAD URDF (openarm_v2_anvil_description): waist_cam_joint on body_link0, and
# left_wrist_cam_joint and right_wrist_cam_joint, which share one origin, on each hand.
CAD_WAIST_CAMERA = ([0.0827, 0.0, 0.3898], [-1.5708, 0.0, -1.5708])
CAD_WRIST_CAMERA = ([0.074, 0.001, 0.009], [-0.8255, 0.0, 1.5708])
# The loader hand frame from link7 on either arm (openarm_parallel_hand.xacro).
HAND_IN_LINK7 = ([0.0, 0.0, -0.1001], [3.141592653589793, 0.0, 0.0])


def _process(**mappings):
    path = os.path.join(
        get_package_share_directory('fm_control'), 'urdf', 'openarm.sim.urdf.xacro'
    )
    return ET.fromstring(xacro.process_file(path, mappings=mappings).toxml())


def _plugins(robot):
    return {p.text for p in robot.findall('ros2_control/hardware/plugin')}


def _links(robot):
    return {link.get('name') for link in robot.findall('link')}


def _pose(xyz, rpy):
    pose = np.eye(4)
    pose[:3, :3] = rotation_rpy(*rpy)
    pose[:3, 3] = xyz
    return pose


def _relative_pose(robot, parent, child):
    # Every joint at zero, so the pose is the description's fixed geometry.
    return np.linalg.inv(world_pose(robot, parent, {})) @ world_pose(robot, child, {})


@pytest.mark.parametrize('backend,plugin', PLUGIN_BY_BACKEND.items())
def test_sim_backend_selects_plugin(backend, plugin):
    robot = _process(robot_preset='default_bimanual', sim_backend=backend)
    assert _plugins(robot) == {plugin}
    assert CAMERA_LINKS <= _links(robot)


def _systems(robot):
    return {
        system.get('name'): [joint.get('name') for joint in system.findall('joint')]
        for system in robot.findall('ros2_control')
    }


@pytest.mark.parametrize('backend', ['mujoco', 'gz', 'gazebo'])
def test_simulators_load_one_system_for_both_arms(backend):
    # One MujocoSystemInterface is one MJCF world, and gz_ros2_control does not
    # initialise two GazeboSimSystem systems; both get every arm in one.
    systems = _systems(_process(robot_preset='default_bimanual', sim_backend=backend))
    assert list(systems) == ['openarm_system']
    joints = systems['openarm_system']
    # Gazebo also lists each finger_joint2, the mimic gz_ros2_control drives.
    mimic = backend in ('gz', 'gazebo')
    assert len(joints) == len(set(joints)) == (18 if mimic else 16)
    for side in ('left', 'right'):
        assert {f'openarm_{side}_joint{i}' for i in range(1, 8)} <= set(joints)
        assert f'openarm_{side}_finger_joint1' in joints
        assert (f'openarm_{side}_finger_joint2' in joints) == mimic


@pytest.mark.parametrize('backend', ['mock', 'isaac', 'real'])
def test_other_backends_keep_one_system_per_arm(backend):
    systems = _systems(_process(robot_preset='default_bimanual', sim_backend=backend))
    assert sorted(systems) == ['openarm_left_system', 'openarm_right_system']
    assert all(len(joints) == 8 for joints in systems.values())


def _initial_values(robot):
    # A mimic joint carries no initial value: it follows the joint it mimics.
    return {
        joint.get('name'): {
            state.get('name'): float(state.find("param[@name='initial_value']").text)
            for state in joint.findall('state_interface')
        }
        for joint in robot.findall('ros2_control/joint')
        if joint.find("state_interface/param[@name='initial_value']") is not None
    }


@pytest.mark.parametrize('backend', ['mujoco', 'gazebo'])
@pytest.mark.parametrize('preset', ['default_bimanual', 'right_arm'])
def test_single_system_spawns_the_per_arm_pose(backend, preset):
    # Same joints, same mirrored ready pose, whichever layout emits them.
    single = _initial_values(_process(robot_preset=preset, sim_backend=backend))
    assert single == _initial_values(_process(robot_preset=preset, sim_backend='mock'))


def test_left_arm_spawns_mirrored():
    # joint4 is excluded from the reflect; every other arm joint is sign-flipped.
    values = _initial_values(_process(robot_preset='default_bimanual', sim_backend='mujoco'))

    def position(side, joint):
        return values[f'openarm_{side}_joint{joint}']['position']

    assert position('left', 4) == position('right', 4)
    for joint in (1, 2, 3, 5, 6, 7):
        assert position('left', joint) == -position('right', joint), f'joint{joint}'


def test_real_hardware_names_each_arm():
    robot = _process(robot_preset='default_bimanual', sim_backend='real')
    prefixes = {
        p.text for p in robot.findall("ros2_control/hardware/param[@name='arm_prefix']")
    }
    assert prefixes == {'openarm_left_', 'openarm_right_'}


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
    'preset,collapse,side',
    [('default_bimanual', 'true', 'right'), ('default_bimanual', 'false', 'right'),
     ('default_bimanual', 'true', 'left'), ('default_bimanual', 'false', 'left'),
     ('right_arm', 'true', 'right'), ('left_arm', 'true', 'left')],
)
def test_wrist_camera_rides_link7_frame(preset, collapse, side):
    # Without the loader hand, the wrist pose is measured from joint7's child,
    # whatever link the preset names.
    robot = _process(robot_preset=preset, collapse_internal_empty_links=collapse, hand='pinch')
    mount = robot.find(f"joint[@name='openarm_{side}_cam_wrist_mount_joint']")
    joint7 = robot.find(f"joint[@name='openarm_{side}_joint7']")
    assert mount.find('parent').get('link') == joint7.find('child').get('link')


@pytest.mark.parametrize('preset,side', [('right_arm', 'right'), ('left_arm', 'left')])
def test_cameras_follow_preset(preset, side):
    # A single-arm preset has no body for the chest and waist cameras, and only its
    # own wrist camera.
    assert CAMERA_LINKS & _links(_process(robot_preset=preset)) == {
        f'openarm_{side}_cam_wrist_optical'}


@pytest.mark.parametrize('hand', ['loader', 'pinch'])
def test_waist_camera_sits_at_the_cad_pose(hand):
    robot = _process(robot_preset='default_bimanual', hand=hand)
    assert _relative_pose(robot, 'openarm_body_link0', 'openarm_cam_waist_optical') == (
        pytest.approx(_pose(*CAD_WAIST_CAMERA), abs=1e-9))


@pytest.mark.parametrize('hand', ['loader', 'pinch'])
@pytest.mark.parametrize('side', ['right', 'left'])
def test_wrist_camera_sits_at_the_cad_pose(side, hand):
    # Measured from joint7's child, the camera is the CAD wrist pose on the loader hand
    # frame, whether the loader hand or the stand-in mount carries it.
    robot = _process(robot_preset='default_bimanual', hand=hand)
    joint7 = robot.find(f"joint[@name='openarm_{side}_joint7']")
    camera = _relative_pose(
        robot, joint7.find('child').get('link'), f'openarm_{side}_cam_wrist_optical')
    assert camera == pytest.approx(
        _pose(*HAND_IN_LINK7) @ _pose(*CAD_WRIST_CAMERA), abs=1e-9)
