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

"""Launch both FLIR Blackfly S cameras (cam0, cam1) with shared config: 720×540 @ 200 Hz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # GenTL path required for Spinnaker to discover USB cameras
    gentl_cti = PathJoinSubstitution([
        FindPackageShare('spinnaker_camera_driver'),
        '..', '..', 'lib', 'spinnaker-gentl', 'Spinnaker_GenTL.cti',
    ])
    config_file = PathJoinSubstitution([
        FindPackageShare('trossen_arm_bringup'),
        'config',
        'flir_dual_camera.yaml',
    ])
    params = ParameterFile(param_file=config_file, allow_substs=True)
    node_map = PathJoinSubstitution([
        FindPackageShare('spinnaker_camera_driver'),
        'config',
        'blackfly_s.yaml',
    ])

    return LaunchDescription([
        SetEnvironmentVariable(name='SPINNAKER_GENTL64_CTI', value=gentl_cti),
        DeclareLaunchArgument(
            'use_cam0',
            default_value='true',
            description='Launch cam0.',
        ),
        DeclareLaunchArgument(
            'use_cam1',
            default_value='true',
            description='Launch cam1.',
        ),
        DeclareLaunchArgument(
            'cam0_serial',
            default_value="'25505853'",
            description='Serial for cam0 (string).',
        ),
        DeclareLaunchArgument(
            'cam1_serial',
            default_value="'25505854'",
            description='Serial for cam1 (string).',
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('use_cam0')),
            package='spinnaker_camera_driver',
            executable='camera_driver_node',
            name='cam0',
            output='screen',
            parameters=[
                params,
                {
                    'parameter_file': node_map,
                    'serial_number': [LaunchConfiguration('cam0_serial')],
                },
            ],
            remappings=[('~/control', '/exposure_control/control')],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('use_cam1')),
            package='spinnaker_camera_driver',
            executable='camera_driver_node',
            name='cam1',
            output='screen',
            parameters=[
                params,
                {
                    'parameter_file': node_map,
                    'serial_number': [LaunchConfiguration('cam1_serial')],
                },
            ],
            remappings=[('~/control', '/exposure_control/control')],
        ),
    ])
