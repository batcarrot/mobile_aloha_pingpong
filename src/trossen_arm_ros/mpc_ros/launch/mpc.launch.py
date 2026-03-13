# Copyright 2025 Trossen Robotics

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mpc_ros",
            executable="mpc_node.py",
            name="mpc_node",
            output="screen",
            parameters=[{}],
        ),
    ])
