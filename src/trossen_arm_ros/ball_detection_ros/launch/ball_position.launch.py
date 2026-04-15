# Copyright 2025 Trossen Robotics

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    calibration_file = LaunchConfiguration("calibration_file", default="")
    use_rgb_ball_filter = LaunchConfiguration("use_rgb_ball_filter", default="false")

    return LaunchDescription([
        DeclareLaunchArgument(
            "calibration_file",
            default_value="",
            description="Path to calibration.npz (stereo cam0/cam1).",
        ),
        DeclareLaunchArgument(
            "use_rgb_ball_filter",
            default_value="false",
            description="If true, require D405 HSV ball patch before publishing stereo/map outputs.",
        ),
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
            parameters=[
                {"calibration_file": calibration_file},
                {"use_rgb_ball_filter": use_rgb_ball_filter},
            ],
        ),
    ])
