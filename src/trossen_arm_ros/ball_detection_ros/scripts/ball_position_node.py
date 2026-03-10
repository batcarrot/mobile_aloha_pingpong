#!/usr/bin/env python3
import os
import sys
from pathlib import Path

_script_dir = Path(__file__).resolve().parent
if str(_script_dir) not in sys.path:
    sys.path.insert(0, str(_script_dir))

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3
from cv_bridge import CvBridge

from ball_detection_ros.core import (
    get_kalman_filter,
    detect_3d,
    median_frame,
    HAS_KALMAN,
)


class BallPositionNode(Node):
    def __init__(self):
        super().__init__("ball_position_node")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("median_frames", 30)
        self.declare_parameter("no_ball_reset_frames", 30)
        self.declare_parameter("max_intersect_error", 0.1)
        self.declare_parameter("use_kalman", True)
        self.declare_parameter("cam0_topic", "/cam0/image_raw")
        self.declare_parameter("cam1_topic", "/cam1/image_raw")

        cal_path = self.get_parameter("calibration_file").value
        if not cal_path or not os.path.isfile(cal_path):
            self.get_logger().error(
                "calibration_file must point to a calibration.npz (e.g. from robotpingpong/camera). "
                "Set it with calibration_file:=/path/to/calibration.npz"
            )
            raise SystemExit(1)

        data = np.load(cal_path, allow_pickle=True)
        self.K = data["K"]
        cam0 = data["cam0"].item()
        cam1 = data["cam1"].item()
        self.R0 = cam0["R"]
        self.t0 = cam0["t"]
        self.R1 = cam1["R"]
        self.t1 = cam1["t"]

        self.median_frames = self.get_parameter("median_frames").value
        self.no_ball_reset_frames = self.get_parameter("no_ball_reset_frames").value
        self.max_intersect_error = self.get_parameter("max_intersect_error").value
        self.use_kalman = self.get_parameter("use_kalman").value and HAS_KALMAN
        if self.get_parameter("use_kalman").value and not HAS_KALMAN:
            self.get_logger().warn("pykalman not installed; publishing raw position only.")

        self.bridge = CvBridge()
        self._frames0 = []
        self._frames1 = []
        self._latest0 = None
        self._latest1 = None
        self._latest0_stamp = None
        self._latest1_stamp = None
        self._max_stamp_diff = 0.05

        self._reference_pos = None
        self._no_ball_count = 0
        self._kf = get_kalman_filter() if self.use_kalman else None
        self._state_mean = None
        self._state_cov = None

        cam0_topic = self.get_parameter("cam0_topic").value
        cam1_topic = self.get_parameter("cam1_topic").value
        self._sub0 = self.create_subscription(Image, cam0_topic, self._cb_cam0, 10)
        self._sub1 = self.create_subscription(Image, cam1_topic, self._cb_cam1, 10)
        self._pub_pos = self.create_publisher(PointStamped, "~/ball_position", 10)
        self._pub_vel = self.create_publisher(Vector3, "~/ball_velocity", 10)

        self.get_logger().info(
            f"Ball position node: calibration={cal_path}, "
            f"median_frames={self.median_frames}, use_kalman={self.use_kalman}"
        )

    def _to_grayscale(self, msg):
        cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if cv.ndim == 3:
            return np.asarray(cv2.cvtColor(cv, cv2.COLOR_BGR2GRAY), dtype=np.uint8)
        return np.asarray(cv, dtype=np.uint8)

    def _cb_cam0(self, msg):
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        gray = self._to_grayscale(msg)
        self._latest0 = gray
        self._latest0_stamp = stamp
        self._frames0.append(gray)
        if len(self._frames0) > self.median_frames + 50:
            self._frames0.pop(0)
        self._process_pair()

    def _cb_cam1(self, msg):
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        gray = self._to_grayscale(msg)
        self._latest1 = gray
        self._latest1_stamp = stamp
        self._frames1.append(gray)
        if len(self._frames1) > self.median_frames + 50:
            self._frames1.pop(0)
        self._process_pair()

    def _process_pair(self):
        if self._latest0 is None or self._latest1 is None:
            return
        if self._latest0_stamp is None or self._latest1_stamp is None:
            return
        if abs(self._latest0_stamp - self._latest1_stamp) > self._max_stamp_diff:
            return

        median0 = median_frame(self._frames0, k=self.median_frames)
        median1 = median_frame(self._frames1, k=self.median_frames)
        if median0 is None or median1 is None:
            return

        pos = detect_3d(
            self._latest0, self._latest1,
            median0, median1,
            self.K, self.R0, self.t0, self.R1, self.t1,
            reference_pos=self._reference_pos,
            max_intersect_error=self.max_intersect_error,
        )

        if pos is None:
            self._no_ball_count += 1
            if self._no_ball_count >= self.no_ball_reset_frames:
                self._state_mean = None
                self._state_cov = None
                self._reference_pos = None
            return

        self._no_ball_count = 0
        z = np.asarray(pos, dtype=float)

        if self._kf is not None and self._state_mean is not None:
            self._state_mean, self._state_cov = self._kf.filter_update(
                self._state_mean, self._state_cov, observation=z
            )
            pos_out = self._state_mean[:3]
            vel_out = self._state_mean[3:]
        else:
            if self._kf is not None:
                self._state_mean = np.array(
                    [z[0], z[1], z[2], 0.0, 0.0, 0.0], dtype=float
                )
                self._state_cov = np.eye(6) * 1e-2
            pos_out = z
            vel_out = np.zeros(3)

        self._reference_pos = pos_out.copy()

        t = (self._latest0_stamp + self._latest1_stamp) / 2.0
        sec = int(t)
        nsec = int((t - sec) * 1e9)

        out = PointStamped()
        out.header.stamp.sec = sec
        out.header.stamp.nanosec = nsec
        out.header.frame_id = "object_frame"
        out.point.x = float(pos_out[0])
        out.point.y = float(pos_out[1])
        out.point.z = float(pos_out[2])
        self._pub_pos.publish(out)

        vel_msg = Vector3()
        vel_msg.x = float(vel_out[0])
        vel_msg.y = float(vel_out[1])
        vel_msg.z = float(vel_out[2])
        self._pub_vel.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = BallPositionNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
