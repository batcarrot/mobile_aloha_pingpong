# Copyright 2025 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    use_flir_camera = LaunchConfiguration('use_flir_camera')
    use_flir_camera_val = use_flir_camera.perform(context) == 'true'

    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('trossen_arm_description'),
            'urdf',
            'mobile_ai.urdf.xacro',
        ]), ' ',
        'ros2_control_hardware_type:=', LaunchConfiguration('ros2_control_hardware_type'), ' ',
        'follower_left_ip:=', LaunchConfiguration('follower_left_ip'), ' ',
        'follower_right_ip:=', LaunchConfiguration('follower_right_ip'), ' ',
        'use_flir_camera:=', LaunchConfiguration('use_flir_camera'),
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
        }],
        output={'both': 'screen'},
    )

    ros2_control_controllers_config_parameter_file = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('trossen_arm_bringup'),
            'config',
            'mobile_ai_controllers.yaml',
        ]),
        allow_substs=True,
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            ros2_control_controllers_config_parameter_file,
            {'robot_description': ParameterValue(robot_description, value_type=str)},
        ],
        remappings=[('~/robot_description', '/robot_description')],
        output={'both': 'screen'},
    )

    controller_spawner_nodes = [
        Node(
            name=f'{name}_spawner',
            package='controller_manager',
            executable='spawner',
            arguments=[name],
            output={'both': 'screen'},
        )
        for name in [
            'follower_left_arm_controller',
            'follower_left_gripper_controller',
            'follower_right_arm_controller',
            'follower_right_gripper_controller',
            'joint_state_broadcaster',
        ]
    ]

    actions = [
        robot_state_publisher_node,
        controller_manager_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=controller_manager_node,
                on_start=controller_spawner_nodes,
            )
        ),
    ]

    if use_flir_camera_val:
        flir_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('spinnaker_camera_driver'),
                    'launch',
                    'driver_node.launch.py',
                ])
            ),
            launch_arguments={
                'camera_name': LaunchConfiguration('camera_name'),
                'camera_type': LaunchConfiguration('camera_type'),
                'serial': LaunchConfiguration('camera_serial'),
            }.items(),
        )
        actions.append(flir_launch)

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output={'both': 'screen'},
    )
    actions.append(rviz_node)

    return actions


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'ros2_control_hardware_type',
            default_value='real',
            choices=('real', 'mock_components'),
            description='Use real arms or mock hardware.',
        ),
        DeclareLaunchArgument(
            'follower_left_ip',
            default_value='192.168.1.2',
            description='IP address of the left follower arm.',
        ),
        DeclareLaunchArgument(
            'follower_right_ip',
            default_value='192.168.1.3',
            description='IP address of the right follower arm.',
        ),
        DeclareLaunchArgument(
            'use_flir_camera',
            default_value='false',
            choices=('true', 'false'),
            description='If true, include FLIR camera in URDF and launch Spinnaker driver.',
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='flir_cam',
            description='ROS node name for the FLIR camera (use flir_cam to match URDF frame).',
        ),
        DeclareLaunchArgument(
            'camera_type',
            default_value='blackfly_s',
            description='Spinnaker camera type (e.g. blackfly_s, chameleon).',
        ),
        DeclareLaunchArgument(
            'camera_serial',
            default_value="'0'",
            description='FLIR camera serial number (in quotes, e.g. "\'20435008\'").',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='Launch RViz.',
        ),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([
                FindPackageShare('trossen_arm_bringup'),
                'rviz',
                'dual_arm.rviz',
            ]),
            description='Path to RViz config file.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
