#!/usr/bin/env python3
import os
import sys
from pathlib import Path

# When run via symlink (e.g. ros2 run), __file__ is the source path (scripts/);
# the ball_detection_ros package lives at package_root/ball_detection_ros/.
_script_dir = Path(__file__).resolve().parent
_pkg_root = _script_dir.parent if _script_dir.name == "scripts" else _script_dir
if str(_pkg_root) not in sys.path:
    sys.path.insert(0, str(_pkg_root))

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3
from cv_bridge import CvBridge
import time
import pickle

from ball_detection_ros.core import (
    get_kalman_filter,
    detect_3d,
    median_frame,
    HAS_KALMAN,
    table_x, table_surface_z
)

from ball_detection_ros.ball_state_estimator import BallStateEstimatorNoSpin
from ball_state_msgs.msg import BallState


class BallPositionNode(Node):
    def __init__(self):
        super().__init__("ball_position_node")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("median_frames", 30)
        self.declare_parameter("no_ball_reset_frames", 30)
        self.declare_parameter("max_intersect_error", 0.1)
        self.declare_parameter("use_kalman", False)
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
        self._median_frame_list0 = []
        self._median_frame_list1 = []
        self._median_frame0 = None
        self._median_frame1 = None

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
        self._pub_state = self.create_publisher(BallState, "~/ball_state", 10)
        self._pub_pos = self.create_publisher(PointStamped, "~/ball_pos", 10)

        self.get_logger().info(
            f"Ball position node: calibration={cal_path}, "
            f"median_frames={self.median_frames}, use_kalman={self.use_kalman}"
        )

        self.k_D = 0.1487858280740126
        self.k_M = 0.01013947865393271
        self._state_estimator = BallStateEstimatorNoSpin()
        self._current_trajectory = []
        self._current_trajectory_times = []

        self.z_list = []

        self.data = []
        self.last_save = 0

        self._needs_reset = False


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

        if self._median_frame0 is None:
            self._median_frame_list0.append(gray)
            if len(self._median_frame_list0) >= self.median_frames:
                self._median_frame0 = median_frame(self._median_frame_list0, k=self.median_frames)
        self._process_pair()

    def _cb_cam1(self, msg):
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        gray = self._to_grayscale(msg)
        self._latest1 = gray
        self._latest1_stamp = stamp

        if self._median_frame1 is None:
            self._median_frame_list1.append(gray)
            if len(self._median_frame_list0) >= self.median_frames:
                self._median_frame1 = median_frame(self._median_frame_list1, k=self.median_frames)
        self._process_pair()

    def _process_pair(self):
        new_t = time.time()
        if self._latest0 is None or self._latest1 is None:
            return
        if self._latest0_stamp is None or self._latest1_stamp is None:
            return
        if abs(self._latest0_stamp - self._latest1_stamp) > self._max_stamp_diff:
            return
        if self._median_frame0 is None or self._median_frame1 is None:
            return

        median0 = self._median_frame0
        median1 = self._median_frame1
        if median0 is None or median1 is None:
            return

        pos = detect_3d(
            self._latest0, self._latest1,
            median0, median1,
            self.K, self.R0, self.t0, self.R1, self.t1,
            reference_pos=self._reference_pos,
            max_intersect_error=self.max_intersect_error,
        )

        self.data.append((pos, new_t))
        if len(self.data) - self.last_save >= 10:
            self.last_save = len(self.data)
            with open('det_data.pkl', 'wb') as f:
                pickle.dump(self.data, f)
            

        if pos is None:
            self._no_ball_count += 1
            if self._no_ball_count >= self.no_ball_reset_frames:
                self._state_mean = None
                self._state_cov = None
                self._reference_pos = None

                self._current_trajectory = []
                self._current_trajectory_times = []
                self._state_estimator.reset()
                self.z_list = []

                msg = BallState()
                msg.has_ball = False
                self._pub_state.publish(msg)
            return

        self._no_ball_count = 0
        z = np.asarray(pos, dtype=float) + np.array([0.9, 0., table_surface_z])

        # if self._kf is not None and self._state_mean is not None:
        #     self._state_mean, self._state_cov = self._kf.filter_update(
        #         self._state_mean, self._state_cov, observation=z
        #     )
        #     pos_out = self._state_mean[:3]
        #     vel_out = self._state_mean[3:]
        # else:
        #     if self._kf is not None:
        #         self._state_mean = np.array(
        #             [z[0], z[1], z[2], 0.0, 0.0, 0.0], dtype=float
        #         )
        #         self._state_cov = np.eye(6) * 1e-2
        #     pos_out = z
        #     vel_out = np.zeros(3)

        # self._current_trajectory.append(z)
        # self._current_trajectory_times.append(new_t)
        
        # self.z_list.append(z[2])
        
        # # finding bounces
        # if len(self.z_list) > 15:
        #     min_z_idx = np.argmin(self.z_list)
        #     if min_z_idx > 3 and min_z_idx < len(self.z_list) - 4:
        #         self._current_trajectory = []
        #         self._current_trajectory_times = []
        #         self._state_estimator = None
        #         self.z_list = []

        state = self._state_estimator.add_point(new_t, z)
        if state is None:
            return
        
        # estimated_pos, estimated_vel, estimated_ang_vel = self._state_estimator.predict(new_t)
        pos_out = state[0]
        vel_out = state[1]

        self._reference_pos = pos_out.copy()

        t = (self._latest0_stamp + self._latest1_stamp) / 2.0
        sec = int(t)
        nsec = int((t - sec) * 1e9)

        

        out = BallState()
        out.header.stamp.sec = sec
        out.header.stamp.nanosec = nsec
        out.header.frame_id = "map"
        out.position.x = float(pos_out[0])
        out.position.y = float(pos_out[1])
        out.position.z = float(pos_out[2])

        out.velocity.x = float(vel_out[0])
        out.velocity.y = float(vel_out[1])
        out.velocity.z = float(vel_out[2])

        out.has_ball = True
        self._pub_state.publish(out)

        out = PointStamped()
        out.header.stamp.sec = sec
        out.header.stamp.nanosec = nsec
        out.header.frame_id = "map"
        out.point.x = float(pos_out[0])
        out.point.y = float(pos_out[1])
        out.point.z = float(pos_out[2])
        self._pub_pos.publish(out)


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
