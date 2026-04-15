"""RGB capture helpers: project 3D→D405, HSV ball mask, debug composite image."""

import json
from pathlib import Path

import cv2
import numpy as np


def stamp_to_float(stamp) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def project_object_point_to_uv(point_obj, R, t, K, dist):
    p = np.asarray(point_obj, dtype=np.float64).reshape(3)
    rvec, _ = cv2.Rodrigues(np.asarray(R, dtype=np.float64))
    tvec = np.asarray(t, dtype=np.float64).reshape(3, 1)
    K = np.asarray(K, dtype=np.float64)
    dist = np.asarray(dist, dtype=np.float64).reshape(-1)
    if dist.size < 5:
        dist = np.pad(dist, (0, 5 - dist.size))
    dist = dist[:5]
    img_pts, _ = cv2.projectPoints(p.reshape(1, 3), rvec, tvec, K, dist)
    uv = np.asarray(img_pts, dtype=np.float64).reshape(-1, 2)[0]
    u, v = float(uv[0]), float(uv[1])
    zc = (R @ p.reshape(3, 1) + tvec).flatten()[2]
    if zc <= 0.01:
        return None
    return u, v


def stereo_point_to_cam2_object(pos_stereo, T_4x4: np.ndarray) -> np.ndarray:
    p = np.append(np.asarray(pos_stereo, dtype=float).reshape(3), 1.0)
    q = T_4x4 @ p
    return q[:3]


def ball_color_mask_bgr(
    bgr_patch,
    h_orange_lo,
    h_orange_hi,
    s_orange_min,
    white_s_max,
    white_v_min,
):
    hsv = cv2.cvtColor(bgr_patch, cv2.COLOR_BGR2HSV)
    ch, cs, cv_ = cv2.split(hsv)
    orange = (ch >= h_orange_lo) & (ch <= h_orange_hi) & (cs >= s_orange_min)
    white = (cs <= white_s_max) & (cv_ >= white_v_min)
    return (orange | white).astype(np.uint8) * 255


def roi_bounds(u, v, roi_radius, w, h):
    ui, vi = int(round(u)), int(round(v))
    x0 = max(0, ui - roi_radius)
    x1 = min(w, ui + roi_radius + 1)
    y0 = max(0, vi - roi_radius)
    y1 = min(h, vi + roi_radius + 1)
    return x0, x1, y0, y1, ui, vi


def rgb_ball_good_fraction(
    bgr_full,
    u,
    v,
    roi_radius,
    h_orange_lo,
    h_orange_hi,
    s_orange_min,
    white_s_max,
    white_v_min,
):
    """HSV orange/white fraction in ROI around (u,v); same mask as build_debug_bgr, no viz."""
    h, w = bgr_full.shape[:2]
    x0, x1, y0, y1, _, _ = roi_bounds(u, v, roi_radius, w, h)
    if x1 <= x0 or y1 <= y0:
        return 0.0
    patch = bgr_full[y0:y1, x0:x1]
    mask = ball_color_mask_bgr(
        patch,
        h_orange_lo,
        h_orange_hi,
        s_orange_min,
        white_s_max,
        white_v_min,
    )
    return float(np.mean(mask > 0))


def build_debug_bgr(
    bgr_full,
    u,
    v,
    roi_radius,
    h_orange_lo,
    h_orange_hi,
    s_orange_min,
    white_s_max,
    white_v_min,
):
    """
    Full-frame BGR: ROI rectangle, crosshair at (u,v), green overlay where mask is true (ROI only).
    Returns (vis_bgr, good_pixel_fraction_in_roi).
    """
    out = bgr_full.copy()
    h, w = out.shape[:2]
    x0, x1, y0, y1, ui, vi = roi_bounds(u, v, roi_radius, w, h)
    if x1 <= x0 or y1 <= y0:
        cv2.putText(
            out,
            "projection OOB",
            (8, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return out, 0.0

    patch = bgr_full[y0:y1, x0:x1]
    mask = ball_color_mask_bgr(
        patch,
        h_orange_lo,
        h_orange_hi,
        s_orange_min,
        white_s_max,
        white_v_min,
    )
    good_frac = float(np.mean(mask > 0))

    roi = out[y0:y1, x0:x1].astype(np.float32)
    green = np.zeros_like(roi)
    green[:, :] = (0, 255, 0)
    m = (mask > 0)[..., np.newaxis]
    blended = np.where(m, 0.55 * roi + 0.45 * green, roi).astype(np.uint8)
    out[y0:y1, x0:x1] = blended

    cv2.rectangle(out, (x0, y0), (x1 - 1, y1 - 1), (255, 0, 0), 1)
    if 0 <= ui < w and 0 <= vi < h:
        cv2.drawMarker(
            out, (ui, vi), (0, 255, 255), cv2.MARKER_CROSS, 16, 2
        )

    return out, good_frac


def write_capture_meta(path: Path, meta: dict):
    path.write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
