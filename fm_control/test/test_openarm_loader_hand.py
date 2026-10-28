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
Check the OpenArm loader-hand preset against the loader's own description.

The loader is the stack fm-rob-01 runs. Its URDF snapshot lives in fm-skills
(src/openarm_v2_skills_config/urdf/openarm_anvil.urdf at 4c5b6a7, loader image
ros2:1.2.16); the limits and poses below were read from it, with the loader's
follower_<r|l>_* names mapped to openarm_<right|left>_*.
"""

import glob
import itertools
import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
import xacro

# The loader's joint limits per side: (lower, upper, effort, velocity).
LOADER_LIMITS = {
    'right': {
        1: (-2.3562, 2.3562, 40, 2.5),
        2: (-0.1745, 3.3161, 40, 2.5),
        3: (-1.570796, 1.570796, 27, 3.0),
        4: (0.0, 2.443461, 27, 5.5),
        5: (-1.570796, 1.570796, 7, 15.0),
        6: (-1.22173, 0.785398, 7, 16.5),
        7: (-1.570796, 1.570796, 7, 16.5),
    },
    'left': {
        1: (-2.3562, 2.3562, 40, 2.5),
        2: (-3.3161, 0.1745, 40, 2.5),
        3: (-1.570796, 1.570796, 27, 3.0),
        4: (0.0, 2.443461, 27, 5.5),
        5: (-1.570796, 1.570796, 7, 15.0),
        6: (-0.785398, 1.22173, 7, 16.5),
        7: (-1.570796, 1.570796, 7, 16.5),
    },
}

# Loader forward kinematics in world: (side, joints 1..7, link7 position, hand_tcp
# position, hand_tcp x axis, hand_tcp z axis). The joints are the loader's, which map
# to openarm_* 1:1. Rows: zero, the fm-rob-01 rest pose, the bimanual ready pose, and
# two poses near the limits, mirrored for the left arm.
LOADER_FK = [
    ('right', (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     (0.0, -0.153498, 0.262), (0.0, -0.153498, 0.0819),
     (1.0, 0.0, 0.0), (0.0, 0.000004, -1.0)),
    ('right', (0.0, 0.168, 0.012, 1.609, 0.0, -0.046, -0.016),
     (0.215827, -0.19146, 0.489664), (0.395218, -0.193891, 0.505437),
     (-0.084098, 0.16761, 0.98226), (0.996066, -0.013499, 0.087584)),
    ('right', (0.6816, 0.0265, -0.3161, 1.1622, 0.0, 0.3, 0.0),
     (0.339943, -0.100006, 0.578022), (0.5154, -0.060617, 0.56805),
     (0.0055, 0.222343, 0.974953), (0.974219, 0.218703, -0.055372)),
    ('right', (-1.9, 2.9, 1.2, 0.4, -1.3, -1.1, 1.4),
     (0.392834, -0.177612, 0.531584), (0.421238, -0.353444, 0.5049),
     (-0.987271, -0.159016, -0.003098), (0.157715, -0.976306, -0.148163)),
    ('right', (2.2, 0.9, -1.0, 2.3, 1.1, 0.7, -1.2),
     (0.07286, -0.128848, 0.858676), (0.11913, -0.13606, 1.032581),
     (-0.862953, 0.440315, 0.24786), (0.256912, -0.040043, 0.965605)),
    ('left', (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     (0.0, 0.153498, 0.262), (0.0, 0.153498, 0.0819),
     (1.0, 0.0, 0.0), (0.0, -0.000004, -1.0)),
    ('left', (0.0, -0.168, -0.012, 1.609, 0.0, 0.046, 0.016),
     (0.215827, 0.19146, 0.489664), (0.395218, 0.193891, 0.505437),
     (-0.084098, -0.16761, 0.98226), (0.996066, 0.013499, 0.087584)),
    ('left', (-0.6816, -0.0265, 0.3161, 1.1622, 0.0, -0.3, 0.0),
     (0.339943, 0.100006, 0.578022), (0.5154, 0.060617, 0.56805),
     (0.0055, -0.222343, 0.974953), (0.974219, -0.218703, -0.055372)),
    ('left', (1.9, -2.9, -1.2, 0.4, 1.3, 1.1, -1.4),
     (0.392834, 0.177612, 0.531584), (0.421238, 0.353444, 0.5049),
     (-0.987271, 0.159016, -0.003098), (0.157715, 0.976306, -0.148163)),
    ('left', (-2.2, -0.9, 1.0, 2.3, -1.1, -0.7, 1.2),
     (0.07286, 0.128848, 0.858676), (0.11913, 0.13606, 1.032581),
     (-0.862953, -0.440315, 0.24786), (0.256912, 0.040043, 0.965605)),
]

# The loader's link7 per side, which carries the wrist: its hand joint (xyz, rpy from
# link7), its mesh origin and scale_y, and its inertial (com, mass, ixx iyy izz ixy ixz
# iyz). The left arm's mesh and inertial are the right's mirrored in y.
LOADER_LINK7 = {
    'right': {
        'hand': ((0.0, 0.0, 0.1001), (0.0, 0.0, 1.5708)),
        'mesh': (0.0, 0.0, -0.5585), 'scale_y': 1,
        'com': (6.875510271106056e-05, 0.01266175250761268, 0.06951945409987448),
        'mass': 0.4659771327380578,
        'inertia': (0.000639, 0.000497, 0.000342, 1e-06, 1e-06, 8.9e-05),
    },
    'left': {
        'hand': ((0.0, 0.0, 0.1001), (0.0, 0.0, -1.5708)),
        'mesh': (0.0, 0.0, -0.5585), 'scale_y': -1,
        'com': (6.875510271106056e-05, -0.01266175250761268, 0.06951945409987448),
        'mass': 0.4659771327380578,
        'inertia': (0.000639, 0.000497, 0.000342, -1e-06, 1e-06, -8.9e-05),
    },
}

# The bimanual spawn pose per hand, right arm; the left mirrors it. The pinch pose
# makes the 168 mm loader hands touch, so the loader spreads joint2.
READY_BIMANUAL = {
    'pinch': (0.6816, 0.0265, -0.3161, 1.1622, 0.0, 0.3, 0.0),
    'loader': (0.6816, 0.13, -0.3161, 1.1622, 0.0, 0.3, 0.0),
}
# Clearance the loader ready pose keeps between the two arms, fingers anywhere. The
# collision meshes are sampled at SAMPLE_SPACING_M, which can overstate a gap by at
# most the spacing; the pose keeps about 53 mm.
READY_CLEARANCE_M = 0.04
SAMPLE_SPACING_M = 0.004

# The reference table carries 6 decimals; the loader writes pi/2 as 1.5708.
POSITION_TOLERANCE_M = 1e-5
AXIS_TOLERANCE = 1e-4


def _process(**mappings):
    path = os.path.join(
        get_package_share_directory('fm_control'), 'urdf', 'openarm.sim.urdf.xacro'
    )
    return ET.fromstring(xacro.process_file(path, mappings=mappings).toxml())


@pytest.fixture(scope='module')
def loader():
    # The default hand, as the sim builds it.
    return _process(robot_preset='default_bimanual', emit_grasp_frame='true')


@pytest.fixture(scope='module')
def pinch():
    return _process(robot_preset='default_bimanual', emit_grasp_frame='true', hand='pinch')


def _joint(robot, name):
    joint = robot.find(f"joint[@name='{name}']")
    assert joint is not None, name
    return joint


def _limit(joint):
    limit = joint.find('limit')
    return tuple(float(limit.get(key)) for key in ('lower', 'upper', 'effort', 'velocity'))


def _floats(element, attribute, default):
    text = None if element is None else element.get(attribute)
    return np.array(default if text is None else text.split(), dtype=float)


def _rotation_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _rotation_axis_angle(axis, angle):
    x, y, z = axis / np.linalg.norm(axis)
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * skew @ skew


def _world_pose(robot, link, positions):
    """Return the 4x4 world pose of `link` with the named joints at `positions`."""
    joint_by_child = {j.find('child').get('link'): j for j in robot.findall('joint')}
    pose = np.eye(4)
    while link in joint_by_child:
        joint = joint_by_child[link]
        origin = joint.find('origin')
        to_joint = np.eye(4)
        to_joint[:3, :3] = _rotation_rpy(*_floats(origin, 'rpy', [0, 0, 0]))
        to_joint[:3, 3] = _floats(origin, 'xyz', [0, 0, 0])
        motion = np.eye(4)
        value = positions.get(joint.get('name'), 0.0)
        axis = _floats(joint.find('axis'), 'xyz', [1, 0, 0])
        if joint.get('type') in ('revolute', 'continuous'):
            motion[:3, :3] = _rotation_axis_angle(axis, value)
        elif joint.get('type') == 'prismatic':
            motion[:3, 3] = axis * value
        pose = to_joint @ motion @ pose
        link = joint.find('parent').get('link')
    return pose


def _origin(element):
    pose = np.eye(4)
    pose[:3, :3] = _rotation_rpy(*_floats(element, 'rpy', [0, 0, 0]))
    pose[:3, 3] = _floats(element, 'xyz', [0, 0, 0])
    return pose


def _link(robot, name):
    link = robot.find(f"link[@name='{name}']")
    assert link is not None, name
    return link


def _read_stl(path):
    """Return the triangles of a binary STL as an (n, 3, 3) array."""
    with open(path, 'rb') as stl:
        data = stl.read()
    count = int(np.frombuffer(data, dtype='<u4', count=1, offset=80)[0])
    assert len(data) == 84 + 50 * count, f'{path} is not a binary STL'
    record = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', 9), ('attribute', '<u2')])
    triangles = np.frombuffer(data, dtype=record, count=count, offset=84)['vertices']
    return triangles.reshape(count, 3, 3).astype(float)


def _package_path(uri):
    package, relative = uri[len('package://'):].split('/', 1)
    return os.path.join(get_package_share_directory(package), relative)


def _surface_points(triangles, spacing):
    """Sample each triangle on a grid no coarser than `spacing` along its longest edge."""
    points = []
    for a, b, c in triangles:
        steps = max(1, math.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - b),
                                     np.linalg.norm(a - c)) / spacing))
        weights = np.array([(i, j) for i in range(steps + 1) for j in range(steps + 1 - i)])
        along_b = np.outer(weights[:, 0] / steps, b - a)
        along_c = np.outer(weights[:, 1] / steps, c - a)
        points.append(a + along_b + along_c)
    return np.concatenate(points)


def _collision_points(robot, side, positions):
    """Return {link: world surface points} for one arm's collision meshes."""
    points = {}
    for link in robot.findall('link'):
        name = link.get('name')
        collision = link.find('collision')
        if not name.startswith(f'openarm_{side}_') or collision is None:
            continue
        mesh = collision.find('geometry/mesh')
        triangles = _read_stl(_package_path(mesh.get('filename'))) * _floats(
            mesh, 'scale', [1, 1, 1])
        pose = _world_pose(robot, name, positions) @ _origin(collision.find('origin'))
        local = _surface_points(triangles, SAMPLE_SPACING_M)
        points[name] = local @ pose[:3, :3].T + pose[:3, 3]
    return points


def _nearest(a, b):
    """Return the smallest distance between two point sets."""
    best = math.inf
    for start in range(0, len(a), 512):
        chunk = a[start:start + 512]
        squared = (chunk ** 2).sum(1)[:, None] + (b ** 2).sum(1)[None, :] - 2 * chunk @ b.T
        best = min(best, math.sqrt(max(float(squared.min()), 0.0)))
    return best


def _arm_clearance(robot, positions):
    """Return the smallest gap between the two arms' collision meshes."""
    right = _collision_points(robot, 'right', positions)
    left = _collision_points(robot, 'left', positions)
    best = math.inf
    for a, b in itertools.product(left.values(), right.values()):
        # Only points within the current best of the other set's box can beat it.
        reach = min(best, 0.1)
        a_near = a[((a > b.min(0) - reach) & (a < b.max(0) + reach)).all(1)]
        b_near = b[((b > a.min(0) - reach) & (b < a.max(0) + reach)).all(1)]
        if len(a_near) and len(b_near):
            best = min(best, _nearest(a_near, b_near))
    return best


def _spawn_positions(robot):
    """Return the ros2_control initial position of every joint the URDF commands."""
    positions = {}
    for joint in robot.findall('ros2_control/joint'):
        value = joint.find("state_interface[@name='position']/param[@name='initial_value']")
        positions[joint.get('name')] = float(value.text)
    return positions


def _mirrored(right):
    # joint4 is not reflected; every other joint's angle flips for the left arm.
    return tuple(q if i == 4 else -q for i, q in enumerate(right, start=1))


def _with_fingers(robot, positions, opening):
    positions = dict(positions)
    for joint in robot.findall('joint'):
        if joint.get('name').endswith(('finger_joint1', 'finger_joint2')):
            positions[joint.get('name')] = opening
    return positions


@pytest.mark.parametrize('side', ['right', 'left'])
def test_loader_joint_limits(loader, side):
    for index, expected in LOADER_LIMITS[side].items():
        joint = _joint(loader, f'openarm_{side}_joint{index}')
        assert _limit(joint) == pytest.approx(expected), f'{side} joint{index}'


@pytest.mark.parametrize('side,joints,flange,tcp,tcp_x,tcp_z', LOADER_FK)
def test_fk_matches_loader(loader, side, joints, flange, tcp, tcp_x, tcp_z):
    positions = {f'openarm_{side}_joint{i}': q for i, q in enumerate(joints, start=1)}
    flange_pose = _world_pose(loader, f'openarm_{side}_ee_base_link', positions)
    tcp_pose = _world_pose(loader, f'openarm_{side}_grasp_frame', positions)
    assert flange_pose[:3, 3] == pytest.approx(flange, abs=POSITION_TOLERANCE_M)
    assert tcp_pose[:3, 3] == pytest.approx(tcp, abs=POSITION_TOLERANCE_M)
    assert tcp_pose[:3, 0] == pytest.approx(tcp_x, abs=AXIS_TOLERANCE)
    assert tcp_pose[:3, 2] == pytest.approx(tcp_z, abs=AXIS_TOLERANCE)


@pytest.mark.parametrize('collapse', ['true', 'false'])
@pytest.mark.parametrize('side', ['right', 'left'])
def test_tcp_sits_on_the_hand_axis(side, collapse):
    # grasp_frame is 0.1801 m out from the flange, approaching along its own +z.
    robot = _process(
        robot_preset='default_bimanual', emit_grasp_frame='true',
        collapse_internal_empty_links=collapse,
    )
    joint7 = _joint(robot, f'openarm_{side}_joint7')
    tcp = np.linalg.inv(_world_pose(robot, joint7.find('child').get('link'), {})) @ (
        _world_pose(robot, f'openarm_{side}_grasp_frame', {})
    )
    assert tcp[:3, 3] == pytest.approx((0.0, 0.0, -0.1801), abs=1e-9)
    assert tcp[:3, 2] == pytest.approx((0.0, 0.0, -1.0), abs=1e-9)


@pytest.mark.parametrize('side', ['right', 'left'])
def test_fingers_are_prismatic_mimics(loader, side):
    prefix = f'openarm_{side}_'
    for name, axis in (('finger_joint1', (0, -1, 0)), ('finger_joint2', (0, 1, 0))):
        joint = _joint(loader, prefix + name)
        assert joint.get('type') == 'prismatic'
        assert joint.find('parent').get('link') == prefix + 'hand'
        assert _limit(joint) == pytest.approx((0.0, 0.05, 333, 10.0))
        assert tuple(_floats(joint.find('axis'), 'xyz', [1, 0, 0])) == axis
    assert _joint(loader, prefix + 'finger_joint1').find('mimic') is None
    mimic = _joint(loader, prefix + 'finger_joint2').find('mimic')
    assert mimic.get('joint') == prefix + 'finger_joint1'
    assert float(mimic.get('multiplier')) == 1.0
    assert float(mimic.get('offset')) == 0.0


@pytest.mark.parametrize('side', ['right', 'left'])
def test_wrist_is_the_loader_link7(loader, side):
    # ee_base_link carries the loader's link7 mesh and inertial. The loader's link7
    # frame in ee_base_link follows from the hand, which both descriptions place.
    expected = LOADER_LINK7[side]
    prefix = f'openarm_{side}_'
    hand_in_link7 = np.eye(4)
    hand_in_link7[:3, :3] = _rotation_rpy(*expected['hand'][1])
    hand_in_link7[:3, 3] = expected['hand'][0]
    link7 = _origin(_joint(loader, prefix + 'hand_joint').find('origin')) @ np.linalg.inv(
        hand_in_link7)
    rotation = link7[:3, :3]
    wrist = _link(loader, prefix + 'ee_base_link')

    mesh_in_link7 = np.eye(4)
    mesh_in_link7[:3, 3] = expected['mesh']
    for element, folder, mesh in (('visual', 'visual', 'link7.dae'),
                                  ('collision', 'collision', 'link7_symp.stl')):
        geometry = wrist.find(element)
        assert _origin(geometry.find('origin')) == pytest.approx(
            link7 @ mesh_in_link7, abs=1e-5), element
        uri = geometry.find('geometry/mesh').get('filename')
        assert uri == ('package://openarm_description/assets/robot/openarm_v1.0/mesh/arm/'
                       f'{folder}/{mesh}')
        assert os.path.isfile(_package_path(uri))
        assert tuple(_floats(geometry.find('geometry/mesh'), 'scale', [1, 1, 1])) == (
            pytest.approx((0.001, 0.001 * expected['scale_y'], 0.001)))

    inertial = wrist.find('inertial')
    assert float(inertial.find('mass').get('value')) == pytest.approx(expected['mass'])
    origin = _origin(inertial.find('origin'))
    assert origin[:3, :3] == pytest.approx(np.eye(3))
    assert origin[:3, 3] == pytest.approx(rotation @ expected['com'], abs=1e-6)
    ixx, iyy, izz, ixy, ixz, iyz = expected['inertia']
    tensor = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
    element = inertial.find('inertia')
    actual = {key: float(element.get(key)) for key in element.keys()}
    actual = np.array([[actual['ixx'], actual['ixy'], actual['ixz']],
                       [actual['ixy'], actual['iyy'], actual['iyz']],
                       [actual['ixz'], actual['iyz'], actual['izz']]])
    assert actual == pytest.approx(rotation @ tensor @ rotation.T, abs=1e-9)


@pytest.mark.parametrize('hand', ['loader', 'pinch'])
def test_bimanual_spawn_pose(loader, pinch, hand):
    robot = {'loader': loader, 'pinch': pinch}[hand]
    positions = _spawn_positions(robot)
    ready = READY_BIMANUAL[hand]
    for side, ready in (('right', ready), ('left', _mirrored(ready))):
        for index, q in enumerate(ready, start=1):
            name = f'openarm_{side}_joint{index}'
            assert positions[name] == pytest.approx(q), name
            if hand == 'loader':
                lower, upper = LOADER_LIMITS[side][index][:2]
                assert lower <= q <= upper, name


@pytest.mark.parametrize('opening', [0.0, 0.05])
def test_loader_spawn_pose_keeps_the_hands_apart(loader, opening):
    positions = _with_fingers(loader, _spawn_positions(loader), opening)
    assert _arm_clearance(loader, positions) >= READY_CLEARANCE_M


def test_pinch_spawn_pose_would_close_the_loader_hands(loader):
    # The check sees contact: the pinch pose puts the loader hands into each other.
    right = READY_BIMANUAL['pinch']
    positions = {f'openarm_right_joint{i}': q for i, q in enumerate(right, start=1)}
    positions.update(
        {f'openarm_left_joint{i}': q for i, q in enumerate(_mirrored(right), start=1)})
    assert _arm_clearance(loader, _with_fingers(loader, positions, 0.05)) < 0.01


def test_no_pinch_gripper_links(loader):
    links = {link.get('name') for link in loader.findall('link')}
    assert not {name for name in links if '_ee_link' in name}


def test_ros2_control_drives_finger_joint1_without_a_gazebo_mimic(loader):
    # The default backend; gazebo adds each finger_joint2 as a mimic
    # (test_openarm_loader_xacro).
    joints = {j.get('name') for j in loader.findall('ros2_control/joint')}
    assert {'openarm_left_finger_joint1', 'openarm_right_finger_joint1'} <= joints
    assert not {name for name in joints if name.endswith('finger_joint2')}


def test_wrist_camera_rides_the_hand(loader, pinch):
    # The camera hangs from the loader hand and lands where the pinch preset's
    # stand-in mount put it, which is where it sits on the real arm.
    optical = _joint(loader, 'openarm_right_cam_wrist_optical_joint')
    assert optical.find('parent').get('link') == 'openarm_right_hand'
    assert loader.find("link[@name='openarm_right_cam_wrist_mount']") is None
    positions = {f'openarm_right_joint{i}': 0.3 * i for i in range(1, 8)}
    assert _world_pose(loader, 'openarm_right_cam_wrist_optical', positions) == pytest.approx(
        _world_pose(pinch, 'openarm_right_cam_wrist_optical', positions), abs=1e-9
    )


def test_pinch_preset_stays_stock(pinch):
    finger = _joint(pinch, 'openarm_right_finger_joint1')
    assert finger.get('type') == 'revolute'
    assert finger.find('parent').get('link') == 'openarm_right_ee_base_link'
    assert _limit(_joint(pinch, 'openarm_right_joint1')) == pytest.approx(
        (-1.3963, 3.4907, 40, 16.755)
    )
    assert pinch.find("link[@name='openarm_right_grasp_frame']") is not None
    assert pinch.find("link[@name='openarm_right_hand']") is None


def test_grasp_frame_absent_by_default():
    robot = _process(robot_preset='default_bimanual')
    assert robot.find("link[@name='openarm_right_hand']") is not None
    assert robot.find("link[@name='openarm_right_grasp_frame']") is None


def test_every_loader_preset_is_selectable():
    # A variant the xacro does not list would never be built.
    directory = os.path.join(
        get_package_share_directory('fm_description'), 'config', 'openarm', 'loader_presets'
    )
    names = [os.path.basename(p)[:-len('.yaml')] for p in glob.glob(f'{directory}/*.yaml')]
    assert names
    for name in names:
        robot = _process(robot_preset=name)
        assert robot.find("link[@name='openarm_right_hand']") is not None, name


def test_single_arm_presets_stay_stock():
    # right_arm carries no gripper, so the hand arg leaves it as openarm_description has it.
    robot = _process(robot_preset='right_arm')
    assert robot.find("link[@name='openarm_right_hand']") is None
    assert _limit(_joint(robot, 'openarm_right_joint1')) == pytest.approx(
        (-1.3963, 3.4907, 40, 16.755)
    )


def test_gripper_preset_without_loader_variant_fails():
    with pytest.raises(xacro.XacroException, match='has no loader-hand variant'):
        _process(robot_preset='right_arm_with_pinch_gripper')
    _process(robot_preset='right_arm_with_pinch_gripper', hand='pinch')


def test_unknown_hand_fails():
    with pytest.raises(xacro.XacroException, match='unknown hand: parallel'):
        _process(robot_preset='default_bimanual', hand='parallel')


@pytest.mark.parametrize('preset', ['../default_bimanual', 'default bimanual', '_x', ''])
def test_invalid_preset_name_fails(preset):
    with pytest.raises(xacro.XacroException, match='invalid robot_preset'):
        _process(robot_preset=preset)
