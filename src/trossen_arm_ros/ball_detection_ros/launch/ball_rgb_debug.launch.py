# Copyright 2025 Trossen Robotics
# FLIR stereo + D405 color + ball_position + RGB capture (PNG+JSON, bounded).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    calibration_file = LaunchConfiguration("calibration_file", default="")
    use_rgb_ball_filter = LaunchConfiguration("use_rgb_ball_filter", default="false")
    debug_save_dir = LaunchConfiguration("debug_save_dir", default="/tmp/rgb_ball_captures")
    debug_max_captures = LaunchConfiguration("debug_max_captures", default="10")
    roi_radius = LaunchConfiguration("roi_radius", default="32")
    launch_d405 = LaunchConfiguration("launch_d405", default="true")

    return LaunchDescription([
        DeclareLaunchArgument(
            "calibration_file",
            default_value="",
            description="calibration.npz with cam0, cam1, cam2",
        ),
        DeclareLaunchArgument(
            "debug_save_dir",
            default_value="/tmp/rgb_ball_captures",
            description="Directory for capture_NNN.png and .json",
        ),
        DeclareLaunchArgument(
            "debug_max_captures",
            default_value="10",
            description="Stop saving after this many stereo hits",
        ),
        DeclareLaunchArgument(
            "launch_d405",
            default_value="true",
            description="Start realsense2_camera (d405_color.launch.py)",
        ),
        DeclareLaunchArgument(
            "roi_radius",
            default_value="32",
            description="Half-size in pixels of RGB HSV ROI around projected ball (square ~2r+1).",
        ),
        DeclareLaunchArgument(
            "use_rgb_ball_filter",
            default_value="false",
            description="If true, ball_position_node gates stereo on D405 HSV (needs D405 + cam2 in npz).",
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("mobile_aloha_camera"),
                    "launch",
                    "d405_color.launch.py",
                ])
            ),
            condition=IfCondition(launch_d405),
        ),
        Node(
            package="ball_detection_ros",
            executable="ball_position_node.py",
            name="ball_position_node",
            output="screen",
            parameters=[
                {"calibration_file": calibration_file},
                {"publish_stereo_ball_hypothesis": True},
                {"use_rgb_ball_filter": use_rgb_ball_filter},
            ],
        ),
        Node(
            package="ball_detection_ros",
            executable="ball_rgb_capture_node.py",
            name="ball_rgb_capture_node",
            output="screen",
            parameters=[
                {"calibration_file": calibration_file},
                {"debug_save_dir": debug_save_dir},
                {"debug_max_captures": debug_max_captures},
                {"hypothesis_topic": "/ball_position_node/stereo_ball_hypothesis"},
                # Match realsense2_camera: rectified color + CameraInfo (driver often has no color/image_raw).
                {"rgb_topic": "/d405/d405/color/image_rect_raw"},
                {"camera_info_topic": "/d405/d405/color/camera_info"},
                {"roi_radius": roi_radius},
            ],
        ),
    ])
