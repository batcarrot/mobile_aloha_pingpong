# Copyright 2025 Trossen Robotics

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    calibration_file = LaunchConfiguration("calibration_file", default="")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("mobile_aloha_camera"),
                    "launch",
                    "flir_dual_camera.launch.py",
                ])
            ),
        ),
        Node(
            package="ball_detection_ros",
            executable="ball_position_node.py",
            name="ball_position_node",
            output="screen",
            parameters=[{"calibration_file": calibration_file}],
        ),
    ])
