from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    gentl_cti = PathJoinSubstitution([
        FindPackageShare('spinnaker_camera_driver'),
        '..', '..', 'lib', 'spinnaker-gentl', 'Spinnaker_GenTL.cti',
    ])
    return LaunchDescription([
        SetEnvironmentVariable(name='SPINNAKER_GENTL64_CTI', value=gentl_cti),
        DeclareLaunchArgument('camera_name', default_value='flir_cam'),
        DeclareLaunchArgument('camera_type', default_value='blackfly_s'),
        DeclareLaunchArgument('camera_serial', default_value="'0'"),
        DeclareLaunchArgument('parameter_file', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('spinnaker_camera_driver'),
                    'launch', 'driver_node.launch.py',
                ])
            ),
            launch_arguments={
                'camera_name': LaunchConfiguration('camera_name'),
                'camera_type': LaunchConfiguration('camera_type'),
                'serial': LaunchConfiguration('camera_serial'),
                'parameter_file': LaunchConfiguration('parameter_file'),
            }.items(),
        ),
    ])
