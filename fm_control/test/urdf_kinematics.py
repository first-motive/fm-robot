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

"""Pose arithmetic over a processed URDF, shared by the xacro tests."""

import math

import numpy as np


def floats(element, attribute, default):
    """Return an attribute's space-separated numbers, or `default` when it is absent."""
    text = None if element is None else element.get(attribute)
    return np.array(default if text is None else text.split(), dtype=float)


def rotation_rpy(roll, pitch, yaw):
    """Return the URDF fixed-axis roll, pitch, yaw rotation."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def rotation_axis_angle(axis, angle):
    """Return the rotation by `angle` about `axis` (Rodrigues)."""
    x, y, z = axis / np.linalg.norm(axis)
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * skew @ skew


def world_pose(robot, link, positions):
    """Return the 4x4 world pose of `link` with the named joints at `positions`."""
    joint_by_child = {j.find('child').get('link'): j for j in robot.findall('joint')}
    pose = np.eye(4)
    while link in joint_by_child:
        joint = joint_by_child[link]
        origin = joint.find('origin')
        to_joint = np.eye(4)
        to_joint[:3, :3] = rotation_rpy(*floats(origin, 'rpy', [0, 0, 0]))
        to_joint[:3, 3] = floats(origin, 'xyz', [0, 0, 0])
        motion = np.eye(4)
        value = positions.get(joint.get('name'), 0.0)
        axis = floats(joint.find('axis'), 'xyz', [1, 0, 0])
        if joint.get('type') in ('revolute', 'continuous'):
            motion[:3, :3] = rotation_axis_angle(axis, value)
        elif joint.get('type') == 'prismatic':
            motion[:3, 3] = axis * value
        pose = to_joint @ motion @ pose
        link = joint.find('parent').get('link')
    return pose


def origin_pose(element):
    """Return the 4x4 pose an <origin> element describes."""
    pose = np.eye(4)
    pose[:3, :3] = rotation_rpy(*floats(element, 'rpy', [0, 0, 0]))
    pose[:3, 3] = floats(element, 'xyz', [0, 0, 0])
    return pose
