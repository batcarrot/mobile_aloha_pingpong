"""Intel RealSense D405 color stream for ball RGB gating (ball_detection_ros).

Default image topic: /d405/d405/color/image_raw (camera_namespace and camera_name both d405).

`enable_depth` defaults to true (same as before this package added the knob). Some D405 /
driver combinations behave more reliably with depth enabled even if you only subscribe
to color. Pass enable_depth:=false to try saving USB bandwidth when stable.

Install: sudo apt install ros-${ROS_DISTRO}-realsense2-camera
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_no",
            default_value="''",
            description="RealSense serial (empty = first device).",
        ),
        DeclareLaunchArgument(
            "camera_namespace",
            default_value="d405",
            description="ROS namespace for the camera node.",
        ),
        DeclareLaunchArgument(
            "camera_name",
            default_value="d405",
            description="Camera node name (full topic prefix: /ns/name/...).",
        ),
        DeclareLaunchArgument(
            "depth_module.color_profile",
            default_value="848x480x30",
            description="D405 color profile (width x height x fps).",
        ),
        DeclareLaunchArgument(
            "enable_depth",
            default_value="true",
            description="Match upstream realsense default; set false to disable Z16 (less USB load).",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("realsense2_camera"),
                    "launch",
                    "rs_launch.py",
                ]),
            ]),
            launch_arguments=[
                ("serial_no", LaunchConfiguration("serial_no")),
                ("camera_namespace", LaunchConfiguration("camera_namespace")),
                ("camera_name", LaunchConfiguration("camera_name")),
                ("enable_color", "true"),
                ("enable_depth", LaunchConfiguration("enable_depth")),
                ("enable_infra1", "false"),
                ("enable_infra2", "false"),
                (
                    "depth_module.color_profile",
                    LaunchConfiguration("depth_module.color_profile"),
                ),
                ("publish_tf", "true"),
            ],
        ),
    ])
