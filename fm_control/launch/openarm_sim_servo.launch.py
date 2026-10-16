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
Start MoveIt Servo (Jazzy) for a SIMULATED OpenArm right arm.

    ros2 launch fm_control openarm_sim_servo.launch.py command_type:=joint_jog

Sim only: collision checking is off and the SRDF has no collision matrix, so this
must never drive a real arm. A real-arm Servo needs its own reviewed profile. The
launch refuses to start unless ``sim_only:=true`` is passed and the ROS graph is
host-local.

Brings up ``/servo_node`` on sim time with the description, SRDF, KDL kinematics and
``config/openarm/servo_sim.yaml``, then selects its command type: Jazzy's Servo
accepts no command until ``~/switch_command_type`` is called. If Servo rejects the
type, the launch shuts down.

    sim_only           must be 'true' (default 'false'): the caller states this is a sim
    command_type       joint_jog | twist (default joint_jog)
    robot_description  URDF Servo plans with (default: built here as
                       default_bimanual with the grasp frame, the model
                       openarm.srdf names). A passed URDF must carry the
                       right_arm chain links.

Servo's topics are reachable by anything on the same DDS domain, so the launch
refuses to start unless discovery is host-local: ``ROS_AUTOMATIC_DISCOVERY_RANGE=
LOCALHOST`` (what the fm-sim agent sets on every run) or ``ROS_LOCALHOST_ONLY=1``.
"""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, Shutdown,
)
from launch.event_handlers import OnProcessExit, OnProcessIO
import launch.logging
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml

# The model openarm.srdf names; test_openarm_sim_servo checks the SRDF against it.
DESCRIPTION_ARGS = {'robot_preset': 'default_bimanual', 'emit_grasp_frame': 'true'}
# The right_arm chain ends; a passed robot_description must contain both.
CHAIN_LINKS = ('openarm_body_link0', 'openarm_right_grasp_frame')
# moveit_msgs/srv/ServoCommandType values this launch offers.
COMMAND_TYPES = {'joint_jog': 0, 'twist': 1}
# fm-teleop's contract publishes on /servo_node/delta_*_cmds.
SERVO_NODE = 'servo_node'
# What `ros2 service call` prints for an accepted ServoCommandType request.
ACCEPTED_REPLY = 'ServoCommandType_Response(success=True)'

_logger = launch.logging.get_logger('openarm_sim_servo')


def share_file(*parts):
    """Return a path in fm_control's install share."""
    return os.path.join(get_package_share_directory('fm_control'), *parts)


def robot_description():
    """Return the OpenArm URDF the SRDF is written for."""
    path = share_file('urdf', 'openarm.sim.urdf.xacro')
    return xacro.process_file(path, mappings=DESCRIPTION_ARGS).toxml()


def checked_description(passed):
    """Return ``passed``, or the local build when empty; fail if it lacks the chain."""
    description = passed or robot_description()
    links = {link.get('name') for link in ET.fromstring(description).findall('link')}
    missing = [link for link in CHAIN_LINKS if link not in links]
    if missing:
        raise ValueError(
            f'robot_description lacks {", ".join(missing)}, which openarm.srdf needs '
            '(build it with robot_preset:=default_bimanual emit_grasp_frame:=true)'
        )
    return description


def check_host_local(environ):
    """Raise unless ``environ`` keeps ROS discovery on this host."""
    if environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST':
        return
    if environ.get('ROS_LOCALHOST_ONLY') == '1':
        return
    raise RuntimeError(
        'refusing to start: set ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST or '
        'ROS_LOCALHOST_ONLY=1, or any host on this DDS domain can command the arm '
        f'through /{SERVO_NODE}'
    )


def on_select_exit(replies):
    """Return the exit handler: shut the launch down unless Servo accepted the type."""
    def handler(event, context):
        if ACCEPTED_REPLY in ''.join(replies):
            return None
        reason = (
            f'/{SERVO_NODE}/switch_command_type did not accept the command type '
            f'(exit code {event.returncode}); Servo would ignore every command'
        )
        _logger.error(reason)
        return [Shutdown(reason=reason)]
    return handler


def _select_command_type(value):
    call = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            f'/{SERVO_NODE}/switch_command_type',
            'moveit_msgs/srv/ServoCommandType',
            f'{{command_type: {value}}}',
        ],
        output='screen',
    )
    replies = []
    return [
        # `ros2 service call` waits for the service, so this needs no start delay.
        call,
        RegisterEventHandler(OnProcessIO(
            target_action=call, on_stdout=lambda event: replies.append(event.text.decode()),
        )),
        RegisterEventHandler(OnProcessExit(target_action=call, on_exit=on_select_exit(replies))),
    ]


def _launch_setup(context):
    sim_only = LaunchConfiguration('sim_only').perform(context)
    if sim_only != 'true':
        raise RuntimeError(
            f'refusing to start: sim_only must be true, got {sim_only!r}. This Servo has '
            'collision checking off and must never drive a real arm.'
        )
    check_host_local(os.environ)
    command_type = LaunchConfiguration('command_type').perform(context)
    if command_type not in COMMAND_TYPES:
        raise ValueError(
            f'command_type must be one of: {", ".join(COMMAND_TYPES)}; got {command_type!r}'
        )
    description = checked_description(
        LaunchConfiguration('robot_description').perform(context)
    )
    with open(share_file('config', 'openarm', 'openarm.srdf')) as handle:
        semantic = handle.read()
    with open(share_file('config', 'openarm', 'kinematics.yaml')) as handle:
        kinematics = yaml.safe_load(handle)

    _logger.warning(
        'OpenArm Servo is SIM ONLY: collision checking is off. Never run it against a '
        'real arm.'
    )

    servo = Node(
        package='moveit_servo',
        executable='servo_node',
        name=SERVO_NODE,
        output='screen',
        parameters=[
            share_file('config', 'openarm', 'servo_sim.yaml'),
            {
                'robot_description': description,
                'robot_description_semantic': semantic,
                'robot_description_kinematics': kinematics,
                'use_sim_time': True,
            },
        ],
    )
    return [servo, *_select_command_type(COMMAND_TYPES[command_type])]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'sim_only', default_value='false',
                description="Must be 'true': confirms the arm is simulated.",
            ),
            DeclareLaunchArgument(
                'command_type', default_value='joint_jog',
                description='Command Servo accepts: joint_jog or twist.',
            ),
            DeclareLaunchArgument(
                'robot_description', default_value='',
                description='URDF for Servo; empty builds default_bimanual with the '
                'grasp frame from openarm.sim.urdf.xacro.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
