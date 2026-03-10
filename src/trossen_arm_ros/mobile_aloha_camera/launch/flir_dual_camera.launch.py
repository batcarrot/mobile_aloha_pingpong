"""Launch both FLIR Blackfly S cameras (cam0, cam1) with shared config: 720×540 @ 200 Hz."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gentl_cti = PathJoinSubstitution([
        FindPackageShare('spinnaker_camera_driver'),
        '..', '..', 'lib', 'spinnaker-gentl', 'Spinnaker_GenTL.cti',
    ])
    config_file = PathJoinSubstitution([
        FindPackageShare('mobile_aloha_camera'),
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
        DeclareLaunchArgument('use_cam0', default_value='true', description='Launch cam0.'),
        DeclareLaunchArgument('use_cam1', default_value='true', description='Launch cam1.'),
        DeclareLaunchArgument('cam0_serial', default_value="'25505853'", description='Serial for cam0 (string).'),
        DeclareLaunchArgument('cam1_serial', default_value="'25505854'", description='Serial for cam1 (string).'),
        Node(
            condition=IfCondition(LaunchConfiguration('use_cam0')),
            package='spinnaker_camera_driver',
            executable='camera_driver_node',
            name='cam0',
            output='screen',
            parameters=[
                params,
                {'parameter_file': node_map, 'serial_number': [LaunchConfiguration('cam0_serial')]},
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
                {'parameter_file': node_map, 'serial_number': [LaunchConfiguration('cam1_serial')]},
            ],
            remappings=[('~/control', '/exposure_control/control')],
        ),
    ])
