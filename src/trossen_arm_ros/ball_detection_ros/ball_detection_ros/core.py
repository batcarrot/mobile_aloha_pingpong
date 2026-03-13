import numpy as np
import cv2

FPS = 1 / 2e-3
DT = 1 / FPS
G = 9.81

try:
    from pykalman import KalmanFilter
    HAS_KALMAN = True
except ImportError:
    HAS_KALMAN = False

table_x = 1.37 + 0.9
table_surface_z = 0.76

def get_kalman_filter():
    if not HAS_KALMAN:
        return None
    F = np.array([
        [1, 0, 0, DT, 0, 0],
        [0, 1, 0, 0, DT, 0],
        [0, 0, 1, 0, 0, DT],
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1],
    ], dtype=float)
    H = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
    ], dtype=float)
    b = np.array([
        0.0, 0.0, -0.5 * G * DT**2,
        0.0, 0.0, -G * DT,
    ], dtype=float)
    Q = np.diag([1e-5, 1e-5, 1e-5, 1e-2, 1e-2, 1e-2])
    R = np.diag([1e-4, 1e-4, 1e-4])
    return KalmanFilter(
        transition_matrices=F,
        observation_matrices=H,
        transition_offsets=b,
        transition_covariance=Q,
        observation_covariance=R,
    )


def epipolar_ray_rect(pixel, K, R, t):
    u, v = pixel
    ray_cam = np.linalg.inv(K) @ np.array([u, v, 1.0])
    dir_rect = R.T @ ray_cam
    dir_rect = dir_rect / np.linalg.norm(dir_rect)
    origin_rect = -R.T @ t
    return origin_rect, dir_rect


def get_intersect(epi0, epi1):
    o0, d0 = epi0
    o1, d1 = epi1
    d0 = d0 / np.linalg.norm(d0)
    d1 = d1 / np.linalg.norm(d1)
    w0 = o0 - o1
    a = np.dot(d0, d0)
    b = np.dot(d0, d1)
    c = np.dot(d1, d1)
    d = np.dot(d0, w0)
    e = np.dot(d1, w0)
    denom = a * c - b * b
    if abs(denom) < 1e-9:
        t0 = 0
        t1 = e / c
    else:
        t0 = (b * e - c * d) / denom
        t1 = (a * e - b * d) / denom
    p0 = o0 + t0 * d0
    p1 = o1 + t1 * d1
    p = 0.5 * (p0 + p1)
    error = np.linalg.norm(p0 - p1)
    return p, error


def median_frame(img_list, k=20):
    if len(img_list) == 0:
        return None
    stack = np.stack(img_list[-k:], axis=0)
    return np.median(stack, axis=0).astype(np.uint8)


def background_subtract(frame, background):
    fg_mask = cv2.absdiff(frame, background)
    _, fg_mask = cv2.threshold(fg_mask, 30, 255, cv2.THRESH_BINARY)
    return fg_mask


def clean(frame):
    kernel = np.ones((4, 4), np.uint8)
    for _ in range(3):
        frame = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)
    return frame


def bounding_box(frame):
    contours, _ = cv2.findContours(
        frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if contours:
        rects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)

            valid = True
            valid &= area > 5
            valid &= perimeter > 0
            valid &= (perimeter ** 2 / area) < 20

            if valid:
                rects.append(cv2.boundingRect(contour))

        return [cv2.boundingRect(c) for c in contours]
    return None


def pipeline(frame, K, R, t, median):
    if median is None:
        return None
    fg_mask = background_subtract(frame, median)
    cleaned = clean(fg_mask)
    bbox = bounding_box(cleaned)
    if bbox is None:
        return None
    epipolar_lines = []
    for x, y, _, _ in bbox:
        origin, direction = epipolar_ray_rect((x, y), K, R, t)
        epipolar_lines.append((origin, direction))
    return epipolar_lines


def detect_3d(
    frame0, frame1, median0, median1,
    K, R0, t0, R1, t1,
    reference_pos=None,
    max_intersect_error=0.1,
    temporal_weight=1.0,
):
    epi0 = pipeline(frame0, K, R0, t0, median0)
    epi1 = pipeline(frame1, K, R1, t1, median1)
    if epi0 is None or epi1 is None:
        return None

    best_pair = None
    best_error = float('inf')
    best_intersect_error = float('inf')
    best_pos = None
    for e0, e1 in [(a, b) for a in epi0 for b in epi1]:
        pos, err_intersect = get_intersect(e0, e1)
        error = err_intersect
        if reference_pos is not None:
            error += temporal_weight * np.linalg.norm(pos - reference_pos)
        if error < best_error:
            best_error = error
            best_intersect_error = err_intersect
            best_pos = pos
            best_pair = (e0, e1)

    if best_intersect_error > max_intersect_error or best_pos is None:
        return None
    return np.asarray(best_pos, dtype=float)
