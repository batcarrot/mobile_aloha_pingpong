#!/usr/bin/env python3
"""Save one D405 color frame + intrinsics from ROS (matches ball_rgb_capture / realsense2_camera).

Use this when your driver publishes ``image_rect_raw`` + ``camera_info`` (no ``image_raw``).
Feed the PNG + JSON K/D into your ``calibrate()`` / web tool the same way you used
pyrealsense ``get_intrinsics()`` — so calibration matches runtime.

Example::

    ros2 launch mobile_aloha_camera d405_color.launch.py
    # In another terminal:
    ros2 run ball_detection_ros d405_calibration_snapshot.py --ros-args \\
        -p output_dir:=/tmp/d405_cal_snap

Writes under output_dir:
    - d405_frame.png   (BGR, same pixels as image_rect_raw)
    - d405_intrinsics.json   (K 3x3, D[5], width, height, topics)
"""
import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class D405CalibrationSnapshot(Node):
    def __init__(self):
        super().__init__("d405_calibration_snapshot")
        self.declare_parameter("output_dir", "/tmp/d405_cal_snap")
        self.declare_parameter("image_topic", "/d405/d405/color/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/d405/d405/color/camera_info")
        self._out = Path(self.get_parameter("output_dir").value).expanduser()
        self._img_topic = self.get_parameter("image_topic").value
        self._ci_topic = self.get_parameter("camera_info_topic").value
        self._out.mkdir(parents=True, exist_ok=True)
        self._bridge = CvBridge()
        self._last_ci = None
        self._done = False

        self.create_subscription(
            CameraInfo, self._ci_topic, self._cb_ci, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, self._img_topic, self._cb_img, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Waiting for one matching frame+CameraInfo → {self._out} "
            f"(image={self._img_topic!r}, camera_info={self._ci_topic!r})"
        )

    def _cb_ci(self, msg: CameraInfo):
        if self._done:
            return
        self._last_ci = msg

    def _cb_img(self, msg: Image):
        if self._done:
            return
        ci = self._last_ci
        if ci is None:
            return
        if int(msg.width) != int(ci.width) or int(msg.height) != int(ci.height):
            return

        enc = (msg.encoding or "").lower()
        arr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        arr = np.asarray(arr, dtype=np.uint8)
        if enc in ("rgb8", "rgba8"):
            code = (
                cv2.COLOR_RGBA2BGR
                if arr.ndim == 3 and arr.shape[2] == 4
                else cv2.COLOR_RGB2BGR
            )
            bgr = cv2.cvtColor(arr, code)
        else:
            bgr = arr

        K = np.asarray(ci.k, dtype=float).reshape(3, 3)
        d = np.asarray(ci.d, dtype=float).reshape(-1) if ci.d else np.zeros(0)
        if d.size < 5:
            d = np.pad(d, (0, 5 - d.size))
        d = d[:5]

        png = self._out / "d405_frame.png"
        meta = self._out / "d405_intrinsics.json"
        cv2.imwrite(str(png), bgr)
        payload = {
            "image_topic": self._img_topic,
            "camera_info_topic": self._ci_topic,
            "width": int(ci.width),
            "height": int(ci.height),
            "K": K.tolist(),
            "dist": d.tolist(),
            "note": "Use K and dist as realsense_K / realsense_dist in calibrate(); "
            "click corners on d405_frame.png (same stream as ROS).",
        }
        meta.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        self._done = True
        self.get_logger().info(f"Wrote {png} and {meta}")


def main(args=None):
    rclpy.init(args=args)
    node = D405CalibrationSnapshot()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
