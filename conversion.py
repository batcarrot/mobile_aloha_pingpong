#!/usr/bin/env python3
"""
Convert a ROS 2 MCAP recording of /mpc_step (mpc_msgs/MpcStep) into the same
pickle layout as mpc_data.pkl: a list of dicts with keys
t, ball_pos, ball_vel, p_des, t_strike, q_traj, qd_traj, qdd_traj,
cur_q, cur_qd, mpc_time, mpc_error.

Dependencies (use a venv): pip install mcap mcap-ros2-support numpy
"""

from __future__ import annotations

import argparse
import math
import pickle
import re
import sys
from pathlib import Path
from typing import Any

import numpy as np

try:
    from mcap_ros2.reader import read_ros2_messages
except ImportError:
    print(
        "Missing dependency. Install with:\n"
        "  python3 -m venv .venv && .venv/bin/pip install mcap mcap-ros2-support numpy",
        file=sys.stderr,
    )
    raise


def _nan_to_none_float(x: Any) -> float | None:
    if x is None:
        return None
    v = float(x)
    if math.isnan(v):
        return None
    return v


def _point_to_vec(p: Any) -> np.ndarray | None:
    arr = np.array([p.x, p.y, p.z], dtype=np.float64)
    if np.all(np.isnan(arr)):
        return None
    return arr


def _reshape_traj(flat: list[float], n_joints: int, n_points: int) -> np.ndarray | None:
    if not flat or n_points == 0 or n_joints == 0:
        return None
    a = np.asarray(flat, dtype=np.float64)
    expected = n_points * n_joints
    if a.size != expected:
        raise ValueError(
            f"Trajectory size mismatch: got {a.size} floats, expected {expected} "
            f"(n_points={n_points} * n_joints={n_joints})"
        )
    return a.reshape((n_points, n_joints))


def _mpc_step_to_record(msg: Any) -> dict[str, Any]:
    n_joints = int(msg.n_joints)
    n_points = int(msg.n_points)

    q_traj = _reshape_traj(msg.q_traj_flat, n_joints, n_points)
    qd_traj = _reshape_traj(msg.qd_traj_flat, n_joints, n_points)
    qdd_traj = _reshape_traj(msg.qdd_traj_flat, n_joints, n_points)

    cur_q = np.asarray(msg.cur_q, dtype=np.float64) if msg.cur_q else None
    cur_qd = np.asarray(msg.cur_qd, dtype=np.float64) if msg.cur_qd else None

    me = _nan_to_none_float(msg.mpc_error)
    mpc_error = np.float64(me) if me is not None else None

    return {
        "t": float(msg.t_wall),
        "ball_pos": _point_to_vec(msg.ball_pos),
        "ball_vel": _point_to_vec(msg.ball_vel),
        "p_des": _point_to_vec(msg.p_des),
        "t_strike": _nan_to_none_float(msg.t_strike),
        "q_traj": q_traj,
        "qd_traj": qd_traj,
        "qdd_traj": qdd_traj,
        "cur_q": cur_q,
        "cur_qd": cur_qd,
        "mpc_time": _nan_to_none_float(msg.mpc_time),
        "mpc_error": mpc_error,
    }


def mcap_to_mpc_data(mcap_path: Path) -> list[dict[str, Any]]:
    data: list[dict[str, Any]] = []
    for wrapped in read_ros2_messages(mcap_path, topics=["/mpc_step"]):
        data.append(_mpc_step_to_record(wrapped.ros_msg))
    return data


# e.g. mpc_run_20260325_203249_0.mcap -> 20260325_203249
_TS_IN_NAME = re.compile(r"(\d{8}_\d{6})")


def _timestamp_from_mcap_name(name: str) -> str | None:
    m = _TS_IN_NAME.search(name)
    return m.group(1) if m else None


def _mcap_sort_key(path: Path) -> tuple:
    ts = _timestamp_from_mcap_name(path.name)
    if ts is not None:
        return (0, ts)
    return (1, path.stat().st_mtime)


def find_latest_mcap(folder: Path) -> Path:
    """Newest .mcap in ``folder`` by YYYYMMDD_HHMMSS in the filename; else by mtime."""
    folder = folder.expanduser().resolve()
    mcaps = sorted(folder.glob("*.mcap"))
    if not mcaps:
        print(f"No .mcap files in {folder}", file=sys.stderr)
        sys.exit(1)
    return max(mcaps, key=_mcap_sort_key)


def default_output_path(mcap_path: Path) -> Path:
    """``<mcap_dir>/pkl/mpc_data_<timestamp>.pkl`` using the date/time segment from the mcap name."""
    mcap_path = mcap_path.resolve()
    ts = _timestamp_from_mcap_name(mcap_path.name)
    tag = ts if ts is not None else mcap_path.stem
    out_dir = mcap_path.parent / "pkl"
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir / f"mpc_data_{tag}.pkl"


def main() -> None:
    _here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="MCAP /mpc_step -> mpc_data.pkl (latest mcap in folder by name timestamp)"
    )
    parser.add_argument(
        "--dir",
        type=Path,
        default=_here,
        help=f"Folder containing .mcap files (default: { _here })",
    )
    parser.add_argument(
        "--mcap",
        type=Path,
        default=None,
        help="Explicit input .mcap (default: latest in --dir by YYYYMMDD_HHMMSS in the filename)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output .pkl path (default: <mcap_dir>/pkl/mpc_data_<timestamp>.pkl)",
    )
    args = parser.parse_args()

    scan_dir = args.dir.expanduser().resolve()
    if args.mcap is None:
        mcap_path = find_latest_mcap(scan_dir)
    else:
        mcap_path = args.mcap.expanduser().resolve()
    if not mcap_path.is_file():
        print(f"File not found: {mcap_path}", file=sys.stderr)
        sys.exit(1)

    out_path = args.output
    if out_path is None:
        out_path = default_output_path(mcap_path)
    else:
        out_path = args.output.expanduser().resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)

    data = mcap_to_mpc_data(mcap_path)
    with open(out_path, "wb") as f:
        pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)
    print(f"Wrote {len(data)} records from {mcap_path.name} to {out_path}")


if __name__ == "__main__":
    main()
