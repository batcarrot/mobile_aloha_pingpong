#!/usr/bin/env python3
"""
Split ROS 2 MCAP ball position data into per-shot runs.

Default behavior:
- Topic: /ball_position_node/ball_pos
- New run when timestamp gap > 0.5 seconds
- Trim first X frames and last Y frames from each run

Output:
- Pickle file containing metadata + list of runs
"""

from __future__ import annotations

import argparse
import pickle
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

try:
    from mcap_ros2.reader import read_ros2_messages
except ImportError as exc:
    raise SystemExit(
        "Missing dependency. Install with:\n"
        "  python3 -m venv .venv && .venv/bin/pip install mcap mcap-ros2-support numpy"
    ) from exc


_TS_IN_NAME = re.compile(r"(\d{8}_\d{6})")


@dataclass(frozen=True)
class RunBounds:
    start: int
    end: int

    @property
    def n_frames(self) -> int:
        return self.end - self.start + 1


def _timestamp_from_name(name: str) -> str | None:
    m = _TS_IN_NAME.search(name)
    return m.group(1) if m else None


def default_output_path(mcap_path: Path) -> Path:
    tag = _timestamp_from_name(mcap_path.name) or mcap_path.stem
    out_dir = mcap_path.parent / "pkl"
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir / f"ball_pos_runs_{tag}.pkl"


def load_topic_data(mcap_path: Path, topic: str) -> tuple[np.ndarray, np.ndarray]:
    ts: list[float] = []
    pos: list[tuple[float, float, float]] = []
    for wrapped in read_ros2_messages(mcap_path, topics=[topic]):
        msg = wrapped.ros_msg
        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        ts.append(t)
        pos.append((float(msg.point.x), float(msg.point.y), float(msg.point.z)))

    if not ts:
        raise RuntimeError(f"No messages found on topic: {topic}")

    return np.asarray(ts, dtype=np.float64), np.asarray(pos, dtype=np.float64)


def split_runs(ts: np.ndarray, gap_s: float) -> list[RunBounds]:
    dt = np.diff(ts)
    cut_idx = np.where(dt > gap_s)[0]
    starts = np.r_[0, cut_idx + 1]
    ends = np.r_[cut_idx, len(ts) - 1]
    return [RunBounds(start=int(s), end=int(e)) for s, e in zip(starts, ends)]


def trim_run(
    run: RunBounds,
    trim_start_frames: int,
    trim_end_frames: int,
    min_frames_after_trim: int,
) -> RunBounds | None:
    s = run.start + trim_start_frames
    e = run.end - trim_end_frames
    if e < s:
        return None
    trimmed = RunBounds(start=s, end=e)
    if trimmed.n_frames < min_frames_after_trim:
        return None
    return trimmed


def build_output(
    ts: np.ndarray,
    pos: np.ndarray,
    raw_runs: list[RunBounds],
    trimmed_runs: list[RunBounds],
    args: argparse.Namespace,
) -> dict[str, Any]:
    runs_payload: list[dict[str, Any]] = []
    for run_id, (raw, trimmed) in enumerate(zip(raw_runs, trimmed_runs)):
        if trimmed is None:
            continue
        t_run = ts[trimmed.start : trimmed.end + 1]
        p_run = pos[trimmed.start : trimmed.end + 1]
        runs_payload.append(
            {
                "run_id": run_id,
                "raw_start_idx": raw.start,
                "raw_end_idx": raw.end,
                "trimmed_start_idx": trimmed.start,
                "trimmed_end_idx": trimmed.end,
                "n_frames": int(trimmed.n_frames),
                "t_start": float(t_run[0]),
                "t_end": float(t_run[-1]),
                "duration_s": float(t_run[-1] - t_run[0]),
                "t": t_run,
                "pos": p_run,
            }
        )

    return {
        "meta": {
            "topic": args.topic,
            "gap_s": args.gap_s,
            "trim_start_frames": args.trim_start_frames,
            "trim_end_frames": args.trim_end_frames,
            "min_frames_after_trim": args.min_frames_after_trim,
            "n_messages_total": int(len(ts)),
            "n_runs_raw": int(len(raw_runs)),
            "n_runs_kept": int(len(runs_payload)),
        },
        "runs": runs_payload,
    }


def parse_args() -> argparse.Namespace:
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="Split /ball_pos data into runs by timestamp gaps and trim each run."
    )
    parser.add_argument(
        "--mcap",
        type=Path,
        required=True,
        help="Path to input .mcap file",
    )
    parser.add_argument(
        "--topic",
        type=str,
        default="/ball_position_node/ball_pos",
        help="Topic to read (default: /ball_position_node/ball_pos)",
    )
    parser.add_argument(
        "--gap-s",
        type=float,
        default=0.5,
        help="Split run when dt > gap-s (default: 0.5)",
    )
    parser.add_argument(
        "--trim-start-frames",
        type=int,
        default=0,
        help="Trim first X frames from each run (default: 0)",
    )
    parser.add_argument(
        "--trim-end-frames",
        type=int,
        default=0,
        help="Trim last Y frames from each run (default: 0)",
    )
    parser.add_argument(
        "--min-frames-after-trim",
        type=int,
        default=1,
        help="Drop run if trimmed length is below this (default: 1)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help=f"Output pkl path (default: {here}/<mcap_dir>/pkl/ball_pos_runs_<timestamp>.pkl)",
    )
    args = parser.parse_args()

    if args.gap_s <= 0:
        raise SystemExit("--gap-s must be > 0")
    if args.trim_start_frames < 0 or args.trim_end_frames < 0:
        raise SystemExit("trim frame counts must be >= 0")
    if args.min_frames_after_trim <= 0:
        raise SystemExit("--min-frames-after-trim must be > 0")
    return args


def main() -> None:
    args = parse_args()
    mcap_path = args.mcap.expanduser().resolve()
    if not mcap_path.is_file():
        raise SystemExit(f"File not found: {mcap_path}")

    out_path = (
        args.output.expanduser().resolve()
        if args.output is not None
        else default_output_path(mcap_path)
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)

    ts, pos = load_topic_data(mcap_path, args.topic)
    raw_runs = split_runs(ts, args.gap_s)
    trimmed_runs = [
        trim_run(
            run,
            trim_start_frames=args.trim_start_frames,
            trim_end_frames=args.trim_end_frames,
            min_frames_after_trim=args.min_frames_after_trim,
        )
        for run in raw_runs
    ]
    payload = build_output(ts, pos, raw_runs, trimmed_runs, args)

    with open(out_path, "wb") as f:
        pickle.dump(payload, f, protocol=pickle.HIGHEST_PROTOCOL)

    print(
        "Wrote "
        f"{payload['meta']['n_runs_kept']} trimmed runs "
        f"(raw runs: {payload['meta']['n_runs_raw']}) to {out_path}"
    )


if __name__ == "__main__":
    main()
