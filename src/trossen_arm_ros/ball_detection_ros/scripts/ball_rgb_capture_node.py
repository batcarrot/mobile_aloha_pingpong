#!/usr/bin/env python3
"""Subscribe to stereo ball hypotheses + D405 color; save annotated PNG + JSON (bounded count)."""
import os
import sys
import time
from pathlib import Path

_script_dir = Path(__file__).resolve().parent
_pkg_root = _script_dir.parent if _script_dir.name == "scripts" else _script_dir
if str(_pkg_root) not in sys.path:
    sys.path.insert(0, str(_pkg_root))

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from ball_detection_ros.rgb_capture_core import (
    build_debug_bgr,
    project_object_point_to_uv,
    stamp_to_float,
    stereo_point_to_cam2_object,
    write_capture_meta,
)

STEREO_FRAME_ID = "stereo_calib"


class BallRgbCaptureNode(Node):
    def __init__(self):
        super().__init__("ball_rgb_capture_node")
        self.declare_parameter("calibration_file", "")
        # Many realsense2_camera builds only publish color as image_rect_raw (+ camera_info).
        # For raw SDK-style images, set rgb_topic to .../color/image_raw and clear camera_info.
        self.declare_parameter("rgb_topic", "/d405/d405/color/image_rect_raw")
        self.declare_parameter(
            "hypothesis_topic",
            "/ball_position_node/stereo_ball_hypothesis",
        )
        # Empty = npz cam2 K,dist only. Set to .../color/camera_info when using image_rect_raw.
        self.declare_parameter(
            "camera_info_topic", "/d405/d405/color/camera_info"
        )
        self.declare_parameter("debug_save_dir", "/tmp/rgb_ball_captures")
        self.declare_parameter("debug_max_captures", 10)
        self.declare_parameter("debug_min_interval_sec", 0.15)
        self.declare_parameter("sync_max_dt", 0.08)
        # Pixels around projected (u,v) for HSV mask (square side ≈ 2*roi_radius+1).
        self.declare_parameter("roi_radius", 32)
        self.declare_parameter("min_good_fraction", 0.12)
        self.declare_parameter("h_orange_lo", 5)
        self.declare_parameter("h_orange_hi", 28)
        self.declare_parameter("s_orange_min", 50)
        self.declare_parameter("white_s_max", 45)
        self.declare_parameter("white_v_min", 180)

        cal_path = self.get_parameter("calibration_file").value
        if not cal_path or not os.path.isfile(cal_path):
            self.get_logger().error("calibration_file must point to calibration.npz with cam2.")
            raise SystemExit(1)

        data = np.load(cal_path, allow_pickle=True)
        if "cam2" not in data.files:
            self.get_logger().error("calibration.npz must contain cam2 (D405 RGB intrinsics + pose).")
            raise SystemExit(1)
        self._cam2 = data["cam2"].item()
        self._T = np.eye(4, dtype=float)
        if "T_cam2_from_stereo" in data.files:
            self._T = np.asarray(data["T_cam2_from_stereo"], dtype=float).reshape(4, 4)
        else:
            self.get_logger().warn(
                "No T_cam2_from_stereo in npz; using identity (stereo frame == D405 PnP frame)."
            )

        self._debug_dir = Path(self.get_parameter("debug_save_dir").value).expanduser()
        self._max_cap = int(self.get_parameter("debug_max_captures").value)
        self._min_int = float(self.get_parameter("debug_min_interval_sec").value)
        self._sync_max = float(self.get_parameter("sync_max_dt").value)
        self._roi_r = int(self.get_parameter("roi_radius").value)
        self._min_good = float(self.get_parameter("min_good_fraction").value)
        self._h_lo = int(self.get_parameter("h_orange_lo").value)
        self._h_hi = int(self.get_parameter("h_orange_hi").value)
        self._s_min = int(self.get_parameter("s_orange_min").value)
        self._ws = int(self.get_parameter("white_s_max").value)
        self._wv = int(self.get_parameter("white_v_min").value)

        self._debug_dir.mkdir(parents=True, exist_ok=True)
        self._captures = 0
        self._last_save_wall = 0.0
        self._bridge = CvBridge()
        self._rgb_bgr = None
        self._rgb_stamp = None
        self._ci_K = None
        self._ci_dist = None
        self._ci_wh = None
        self._logged_ci = False
        self._warned_k_mismatch = False
        self._warned_no_ci_yet = False
        self._logged_first_rgb = False
        self._hyp_no_rgb_count = 0

        rgb_topic = self.get_parameter("rgb_topic").value
        hyp_topic = self.get_parameter("hypothesis_topic").value
        ci_topic = (self.get_parameter("camera_info_topic").value or "").strip()
        self._ci_topic = ci_topic
        self._rgb_topic = rgb_topic
        if "image_rect" in rgb_topic or "rect_raw" in rgb_topic:
            if not ci_topic:
                self.get_logger().warn(
                    "RGB topic looks rectified but camera_info_topic is empty: "
                    "set camera_info_topic to the matching color/camera_info "
                    "(e.g. /d405/d405/color/camera_info) or projection will be wrong "
                    "if npz cam2 K/dist are for raw images."
                )
            else:
                self.get_logger().info(
                    f"Using CameraInfo from {ci_topic!r} for K,D (matches rectified RGB)."
                )
        elif not ci_topic:
            self.get_logger().info(
                "Using cam2 K,dist from calibration npz only (camera_info_topic unset); "
                "rgb_topic resolution must match calibration."
            )
        # RealSense (and many camera drivers) publish with sensor QoS (often BEST_EFFORT).
        # Default rclpy subscription is RELIABLE → no images delivered, no saves.
        self.create_subscription(
            Image, rgb_topic, self._cb_rgb, qos_profile_sensor_data
        )
        self.create_subscription(PointStamped, hyp_topic, self._cb_hypothesis, 10)
        if ci_topic:
            self.create_subscription(
                CameraInfo, ci_topic, self._cb_camera_info, qos_profile_sensor_data
            )

        self.get_logger().info(
            f"ball_rgb_capture: dir={self._debug_dir}, max={self._max_cap}, "
            f"rgb={rgb_topic}, hyp={hyp_topic}, camera_info={ci_topic or 'off'}"
        )

    def _to_bgr(self, msg):
        enc = (msg.encoding or "").lower()
        cv = np.asarray(
            self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"),
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

    def _cb_rgb(self, msg):
        self._rgb_bgr = self._to_bgr(msg)
        self._rgb_stamp = stamp_to_float(msg.header.stamp)
        if not self._logged_first_rgb:
            self._logged_first_rgb = True
            h, w = self._rgb_bgr.shape[:2]
            self.get_logger().info(
                f"First RGB frame: {w}x{h} (topic matched; sensor QoS subscription)."
            )

    @staticmethod
    def _K_dist_from_camera_info(msg: CameraInfo):
        K = np.asarray(msg.k, dtype=float).reshape(3, 3)
        if not np.any(K):
            return None, None
        d = np.asarray(msg.d, dtype=float).reshape(-1) if msg.d else np.zeros(0)
        if d.size < 5:
            d = np.pad(d, (0, 5 - d.size))
        d = d[:5]
        return K, d

    def _cb_camera_info(self, msg: CameraInfo):
        K, d = self._K_dist_from_camera_info(msg)
        if K is None:
            return
        self._ci_K = K
        self._ci_dist = d
        self._ci_wh = (int(msg.width), int(msg.height))
        if not self._logged_ci:
            self._logged_ci = True
            self.get_logger().info(
                f"CameraInfo K: size={self._ci_wh} fx={K[0,0]:.2f} fy={K[1,1]:.2f} "
                f"cx={K[0,2]:.2f} cy={K[1,2]:.2f}"
            )

    def _projection_K_dist(self):
        if self._ci_K is not None:
            return self._ci_K, np.asarray(self._ci_dist, dtype=float), "camera_info"
        K2 = np.asarray(self._cam2["K"], dtype=float)
        dist = np.asarray(self._cam2.get("dist", np.zeros(5)), dtype=float).reshape(-1)
        if dist.size < 5:
            dist = np.pad(dist, (0, 5 - dist.size))
        return K2, dist[:5], "npz"

    def _cb_hypothesis(self, msg: PointStamped):
        if self._captures >= self._max_cap:
            return
        if msg.header.frame_id != STEREO_FRAME_ID:
            self.get_logger().warn(
                f"Ignoring hypothesis frame_id={msg.header.frame_id!r} "
                f"(expected {STEREO_FRAME_ID!r})"
            )
            return

        now = time.time()
        if now - self._last_save_wall < self._min_int:
            return

        stereo_stamp = stamp_to_float(msg.header.stamp)
        pos = np.array(
            [msg.point.x, msg.point.y, msg.point.z], dtype=float
        )

        meta = {
            "capture_index": self._captures,
            "stereo_stamp_sec": stereo_stamp,
            "rgb_stamp_sec": None,
            "stereo_rgb_dt_sec": None,
            "intrinsics_source": None,
            "u": None,
            "v": None,
            "good_frac": None,
            "pass_threshold": None,
            "reason": None,
        }

        if self._rgb_bgr is None or self._rgb_stamp is None:
            self._hyp_no_rgb_count += 1
            if self._hyp_no_rgb_count == 1 or self._hyp_no_rgb_count % 200 == 0:
                self.get_logger().warn(
                    f"Stereo hypothesis #{self._hyp_no_rgb_count} but no RGB yet "
                    f"(check rgb_topic={self._rgb_topic!r} and that the D405 node is running)."
                )
            return

        meta["rgb_stamp_sec"] = self._rgb_stamp
        meta["stereo_rgb_dt_sec"] = abs(self._rgb_stamp - stereo_stamp)
        if meta["stereo_rgb_dt_sec"] > self._sync_max:
            meta["reason"] = "sync_bad"
            vis = self._rgb_bgr.copy()
            cv2.putText(
                vis,
                f"sync_bad dt={meta['stereo_rgb_dt_sec']:.4f}",
                (8, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            self._write_pair(meta, vis)
            return

        p_obj = stereo_point_to_cam2_object(pos, self._T)
        if self._ci_topic and self._ci_K is None and not self._warned_no_ci_yet:
            self._warned_no_ci_yet = True
            self.get_logger().warn(
                f"No CameraInfo on {self._ci_topic!r} yet; using npz K,dist until it arrives."
            )
        K2, dist, k_src = self._projection_K_dist()
        meta["intrinsics_source"] = k_src
        if self._ci_wh is not None and self._rgb_bgr is not None:
            h, w = self._rgb_bgr.shape[:2]
            if (w, h) != self._ci_wh and not self._warned_k_mismatch:
                self._warned_k_mismatch = True
                self.get_logger().warn(
                    f"RGB image size {(w, h)} != CameraInfo {self._ci_wh}; "
                    "intrinsics may not match this stream."
                )
        uv = project_object_point_to_uv(
            p_obj, self._cam2["R"], self._cam2["t"], K2, dist
        )
        if uv is None:
            meta["reason"] = "behind_camera"
            self._write_pair(meta, self._rgb_bgr.copy())
            return

        u, v = uv
        meta["u"] = u
        meta["v"] = v
        vis, good_frac = build_debug_bgr(
            self._rgb_bgr,
            u,
            v,
            self._roi_r,
            self._h_lo,
            self._h_hi,
            self._s_min,
            self._ws,
            self._wv,
        )
        meta["good_frac"] = good_frac
        meta["pass_threshold"] = self._min_good
        meta["pass"] = good_frac >= self._min_good
        meta["reason"] = "ok"
        h_vis = vis.shape[0]
        ytxt = h_vis - 8 if h_vis > 16 else 24
        cv2.putText(
            vis,
            f"g={good_frac:.3f} st={stereo_stamp:.4f} rgb={self._rgb_stamp:.4f}",
            (8, ytxt),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        self._write_pair(meta, vis)

    def _write_pair(self, meta, vis_bgr):
        idx = self._captures
        base = self._debug_dir / f"capture_{idx:03d}"
        write_capture_meta(Path(str(base) + ".json"), meta)
        if vis_bgr is not None:
            cv2.imwrite(str(base) + ".png", vis_bgr)
        self._captures += 1
        self._last_save_wall = time.time()
        self.get_logger().info(
            f"Saved {base}.json (+png if image) [{self._captures}/{self._max_cap}]"
        )


def main(args=None):
    rclpy.init(args=args)
    try:
        node = BallRgbCaptureNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
