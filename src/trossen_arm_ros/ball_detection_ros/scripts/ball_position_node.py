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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import time

from ball_detection_ros.core import (
    get_kalman_filter,
    detect_3d,
    median_frame,
    HAS_KALMAN,
    table_surface_z,
)
from ball_detection_ros.rgb_capture_core import (
    project_object_point_to_uv,
    rgb_ball_good_fraction,
    stereo_point_to_cam2_object,
    stamp_to_float,
)
from ball_detection_ros.ball_state_estimator import BallStateEstimatorNoSpin
from ball_state_msgs.msg import BallState


def _param_bool(node, name: str) -> bool:
    v = node.get_parameter(name).value
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        return v.strip().lower() in ("true", "1", "yes", "on")
    return bool(v)


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
        self.declare_parameter("publish_stereo_stamp_diff", True)
        self.declare_parameter("stereo_stamp_diff_rate_hz", 10.0)
        self.declare_parameter("log_stereo_stamp_diff", False)
        self.declare_parameter("publish_stereo_ball_hypothesis", True)
        # When true, require D405 HSV ball patch around projected stereo point before
        # publishing stereo_hypothesis, ball_pos, or feeding the state estimator.
        self.declare_parameter("use_rgb_ball_filter", False)
        self.declare_parameter("rgb_topic", "/d405/d405/color/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/d405/d405/color/camera_info")
        self.declare_parameter("rgb_sync_max_dt", 0.08)
        self.declare_parameter("rgb_gate_allow_without_image", True)
        self.declare_parameter("rgb_roi_radius", 32)
        self.declare_parameter("rgb_min_good_fraction", 0.12)
        self.declare_parameter("rgb_h_orange_lo", 5)
        self.declare_parameter("rgb_h_orange_hi", 28)
        self.declare_parameter("rgb_s_orange_min", 50)
        self.declare_parameter("rgb_white_s_max", 45)
        self.declare_parameter("rgb_white_v_min", 180)

        cal_path = self.get_parameter("calibration_file").value
        if not cal_path or not os.path.isfile(cal_path):
            self.get_logger().error(
                "calibration_file must point to a calibration.npz (e.g. from robotpingpong/camera). "
                "Set it with calibration_file:=/path/to/calibration.npz"
            )
            raise SystemExit(1)

        data = np.load(cal_path, allow_pickle=True)
        self.K = np.asarray(data["K"], dtype=float)
        cam0 = data["cam0"].item()
        cam1 = data["cam1"].item()
        self.R0 = cam0["R"]
        self.t0 = cam0["t"]
        self.R1 = cam1["R"]
        self.t1 = cam1["t"]
        self.K0 = (
            np.asarray(cam0["K"], dtype=float)
            if isinstance(cam0.get("K"), np.ndarray)
            else self.K
        )
        self.K1 = (
            np.asarray(cam1["K"], dtype=float)
            if isinstance(cam1.get("K"), np.ndarray)
            else self.K
        )

        self._rgb_filter_enabled = _param_bool(self, "use_rgb_ball_filter")
        self._cam2 = None
        self._T_stereo_to_cam2 = np.eye(4, dtype=float)
        self._ci_K = None
        self._ci_dist = None
        self._rgb_bgr = None
        self._rgb_stamp = None
        self._rgb_sync_max = float(self.get_parameter("rgb_sync_max_dt").value)
        self._rgb_allow_without_image = _param_bool(
            self, "rgb_gate_allow_without_image"
        )
        self._rgb_roi = int(self.get_parameter("rgb_roi_radius").value)
        self._rgb_min_good = float(self.get_parameter("rgb_min_good_fraction").value)
        self._rgb_h_lo = int(self.get_parameter("rgb_h_orange_lo").value)
        self._rgb_h_hi = int(self.get_parameter("rgb_h_orange_hi").value)
        self._rgb_s_min = int(self.get_parameter("rgb_s_orange_min").value)
        self._rgb_ws = int(self.get_parameter("rgb_white_s_max").value)
        self._rgb_wv = int(self.get_parameter("rgb_white_v_min").value)
        self._rgb_reject_log_t = 0.0

        if self._rgb_filter_enabled:
            if "cam2" not in data.files:
                self.get_logger().error(
                    "use_rgb_ball_filter requires cam2 in calibration.npz (pose + intrinsics)."
                )
                raise SystemExit(1)
            self._cam2 = data["cam2"].item()
            if "T_cam2_from_stereo" in data.files:
                self._T_stereo_to_cam2 = np.asarray(
                    data["T_cam2_from_stereo"], dtype=float
                ).reshape(4, 4)
            ci_topic = (self.get_parameter("camera_info_topic").value or "").strip()
            rgb_tp = self.get_parameter("rgb_topic").value
            self.create_subscription(
                Image, rgb_tp, self._cb_rgb_filter, qos_profile_sensor_data
            )
            if ci_topic:
                self.create_subscription(
                    CameraInfo, ci_topic, self._cb_ci_filter, qos_profile_sensor_data
                )
            self.get_logger().info(
                f"RGB ball filter ON: rgb={rgb_tp!r}, camera_info={ci_topic or 'npz K only'}, "
                f"roi_r={self._rgb_roi}, min_good={self._rgb_min_good}, "
                f"allow_without_image={self._rgb_allow_without_image}"
            )

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

        self._pub_stereo_hyp = None
        if self.get_parameter("publish_stereo_ball_hypothesis").value:
            self._pub_stereo_hyp = self.create_publisher(
                PointStamped, "~/stereo_ball_hypothesis", 10
            )

        self._pub_stereo_stamp_diff = None
        if self.get_parameter("publish_stereo_stamp_diff").value:
            self._pub_stereo_stamp_diff = self.create_publisher(
                Float64, "~/stereo_stamp_diff_sec", 10
            )
            hz = float(self.get_parameter("stereo_stamp_diff_rate_hz").value)
            hz = max(0.5, min(hz, 200.0))
            self.create_timer(1.0 / hz, self._timer_publish_stereo_stamp_diff)
        if self.get_parameter("log_stereo_stamp_diff").value:
            self.create_timer(1.0, self._timer_log_stereo_stamp_diff)

        self.get_logger().info(
            f"Ball position node: calibration={cal_path}, "
            f"median_frames={self.median_frames}, use_kalman={self.use_kalman}"
        )

        self.k_D = 0.16
        self._state_estimator = BallStateEstimatorNoSpin(k_D=self.k_D)
        self._current_trajectory = []
        self._current_trajectory_times = []

        self.z_list = []

        self.data = []
        self.last_save = 0

        self._needs_reset = False

        self._has_published = False
        self._n_data = 0

    def _to_grayscale(self, msg):
        cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if cv.ndim == 3:
            return np.asarray(cv2.cvtColor(cv, cv2.COLOR_BGR2GRAY), dtype=np.uint8)
        return np.asarray(cv, dtype=np.uint8)

    def _imgmsg_to_bgr(self, msg):
        enc = (msg.encoding or "").lower()
        cv = np.asarray(
            self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"),
            dtype=np.uint8,
        )
        if enc in ("rgb8", "rgba8"):
            code = (
                cv2.COLOR_RGBA2BGR
                if cv.ndim == 3 and cv.shape[2] == 4
                else cv2.COLOR_RGB2BGR
            )
            cv = cv2.cvtColor(cv, code)
        return cv

    def _cb_rgb_filter(self, msg):
        self._rgb_bgr = self._imgmsg_to_bgr(msg)
        self._rgb_stamp = stamp_to_float(msg.header.stamp)

    def _cb_ci_filter(self, msg: CameraInfo):
        K = np.asarray(msg.k, dtype=float).reshape(3, 3)
        if not np.any(K):
            return
        d = np.asarray(msg.d, dtype=float).reshape(-1) if msg.d else np.zeros(0)
        if d.size < 5:
            d = np.pad(d, (0, 5 - d.size))
        self._ci_K = K
        self._ci_dist = d[:5]

    def _projection_K_dist_for_rgb(self):
        if self._ci_K is not None:
            return self._ci_K, np.asarray(self._ci_dist, dtype=float)
        K2 = np.asarray(self._cam2["K"], dtype=float)
        dist = np.asarray(self._cam2.get("dist", np.zeros(5)), dtype=float).reshape(-1)
        if dist.size < 5:
            dist = np.pad(dist, (0, 5 - dist.size))
        return K2, dist[:5]

    def _rgb_ball_gate_pass(self, pos, stereo_t: float) -> bool:
        if pos is None or not self._rgb_filter_enabled:
            return True
        if self._rgb_bgr is None or self._rgb_stamp is None:
            return self._rgb_allow_without_image
        if abs(self._rgb_stamp - stereo_t) > self._rgb_sync_max:
            return False
        p_obj = stereo_point_to_cam2_object(
            np.asarray(pos, dtype=float), self._T_stereo_to_cam2
        )
        K2, dist = self._projection_K_dist_for_rgb()
        uv = project_object_point_to_uv(
            p_obj, self._cam2["R"], self._cam2["t"], K2, dist
        )
        if uv is None:
            return False
        u, v = uv
        gf = rgb_ball_good_fraction(
            self._rgb_bgr,
            u,
            v,
            self._rgb_roi,
            self._rgb_h_lo,
            self._rgb_h_hi,
            self._rgb_s_min,
            self._rgb_ws,
            self._rgb_wv,
        )
        return gf >= self._rgb_min_good

    def _timer_publish_stereo_stamp_diff(self):
        if self._pub_stereo_stamp_diff is None:
            return
        if self._latest0_stamp is None or self._latest1_stamp is None:
            return
        msg = Float64()
        msg.data = float(abs(self._latest0_stamp - self._latest1_stamp))
        self._pub_stereo_stamp_diff.publish(msg)

    def _timer_log_stereo_stamp_diff(self):
        if self._latest0_stamp is None or self._latest1_stamp is None:
            return
        dt = float(abs(self._latest0_stamp - self._latest1_stamp))
        self.get_logger().info(
            f"stereo |cam0_stamp - cam1_stamp| = {dt:.6f} s ({dt * 1e3:.3f} ms)"
        )

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
            if len(self._median_frame_list1) >= self.median_frames:
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
            K0=self.K0,
            K1=self.K1,
        )
        stereo_t = (self._latest0_stamp + self._latest1_stamp) * 0.5
        rgb_pass = self._rgb_ball_gate_pass(pos, stereo_t)
        if self._rgb_filter_enabled and pos is not None and not rgb_pass:
            now = time.time()
            if now - self._rgb_reject_log_t > 2.0:
                self._rgb_reject_log_t = now
                self.get_logger().info(
                    "RGB ball filter rejected stereo hit (HSV/sync/projection)."
                )

        if pos is not None and rgb_pass and self._pub_stereo_hyp is not None:
            s = int(np.floor(stereo_t))
            ns = int(round((stereo_t - s) * 1e9))
            if ns >= 1_000_000_000:
                s += 1
                ns -= 1_000_000_000
            hyp = PointStamped()
            hyp.header.frame_id = "stereo_calib"
            hyp.header.stamp.sec = s
            hyp.header.stamp.nanosec = ns
            p = np.asarray(pos, dtype=float).reshape(3)
            hyp.point.x = float(p[0])
            hyp.point.y = float(p[1])
            hyp.point.z = float(p[2])
            self._pub_stereo_hyp.publish(hyp)

        if pos is not None and rgb_pass:
            pos_publish = np.asarray(pos, dtype=float) + np.array([0.9, 0., table_surface_z])
            out = PointStamped()
            sec = int(new_t)
            nsec = int((new_t - sec) * 1e9)
            out.header.stamp.sec = sec
            out.header.stamp.nanosec = nsec
            out.header.frame_id = "map"
            out.point.x = float(pos_publish[0])
            out.point.y = float(pos_publish[1])
            out.point.z = float(pos_publish[2])
            self._pub_pos.publish(out)

        if pos is None or not rgb_pass:
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

                self._has_published = False
                self._n_data = 0
            return

        self._no_ball_count = 0
        z = np.asarray(pos, dtype=float) + np.array([0.9, 0., table_surface_z])

        self._n_data += 1
        state = self._state_estimator.add_point(new_t, z)
        if state is None:
            return

        if self._has_published:
            return

        if self._n_data < 50:
            return

        self._has_published = True

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
