#!/usr/bin/env python3

import sys
from pathlib import Path

# When run via symlink (e.g. ros2 run), __file__ is the source path (scripts/);
# the mpc_ros package lives at package_root/mpc_ros/, so add package_root to path.
_script_dir = Path(__file__).resolve().parent
_pkg_root = _script_dir.parent if _script_dir.name == "scripts" else _script_dir
if str(_pkg_root) not in sys.path:
    sys.path.insert(0, str(_pkg_root))

import rclpy

from mpc_ros.nodes import BallPredictionNode


def main():
    rclpy.init()
    node = BallPredictionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

