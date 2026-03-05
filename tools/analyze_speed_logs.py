#!/usr/bin/env python3
"""
Analyze ROS2 CSV logs for speed-control tests.

Expected default CSV format:
- cmd.csv from:  ros2 topic echo /ackermann_cmd --csv > cmd.csv
- odom.csv from: ros2 topic echo /odom --csv > odom.csv
"""

from __future__ import annotations

import argparse
import csv
import statistics
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Segment:
    start_t: float
    end_t: float
    samples: list[float]

    @property
    def duration(self) -> float:
        return self.end_t - self.start_t


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Analyze speed test CSV logs")
    p.add_argument("--odom", required=True, help="Path to odom.csv")
    p.add_argument("--cmd", help="Path to cmd.csv (optional, for reference speeds)")
    p.add_argument(
        "--odom-vel-col",
        type=int,
        default=48,
        help="1-based column index of linear speed in odom.csv (default: 48)",
    )
    p.add_argument(
        "--cmd-speed-col",
        type=int,
        default=6,
        help="1-based column index of commanded speed in cmd.csv (default: 6)",
    )
    p.add_argument(
        "--move-threshold",
        type=float,
        default=0.01,
        help="Speed threshold to detect moving segments (m/s)",
    )
    p.add_argument(
        "--min-segment-sec",
        type=float,
        default=0.3,
        help="Minimum duration for a valid moving segment (sec)",
    )
    p.add_argument(
        "--trim-sec",
        type=float,
        default=0.5,
        help="Trim this time from both start/end of each segment for steady-state stats",
    )
    p.add_argument(
        "--target-err-pct",
        type=float,
        default=3.0,
        help="PASS threshold for absolute speed error percent",
    )
    p.add_argument(
        "--target-std-mps",
        type=float,
        default=0.02,
        help="PASS threshold for speed ripple std (m/s)",
    )
    return p.parse_args()


def read_odom(path: Path, vel_col_1based: int) -> list[tuple[float, float]]:
    vel_idx = vel_col_1based - 1
    rows: list[tuple[float, float]] = []
    with path.open() as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) <= vel_idx:
                continue
            try:
                sec = float(row[0])
                nsec = float(row[1])
                v = float(row[vel_idx])
            except ValueError:
                continue
            t = sec + nsec * 1e-9
            rows.append((t, v))
    return rows


def read_cmd_refs(path: Path, speed_col_1based: int) -> list[float]:
    speed_idx = speed_col_1based - 1
    refs: list[float] = []
    with path.open() as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) <= speed_idx:
                continue
            try:
                refs.append(float(row[speed_idx]))
            except ValueError:
                continue

    if not refs:
        return []

    unique_runs: list[float] = []
    prev = refs[0]
    for v in refs[1:]:
        if v != prev:
            unique_runs.append(prev)
            prev = v
    unique_runs.append(prev)

    # Keep non-zero command targets only, in order.
    return [v for v in unique_runs if abs(v) > 1e-9]


def detect_moving_segments(
    odom: list[tuple[float, float]], threshold: float, min_sec: float
) -> list[Segment]:
    segs: list[Segment] = []
    in_seg = False
    start_idx = 0

    for i, (t, v) in enumerate(odom):
        moving = abs(v) > threshold
        if moving and not in_seg:
            in_seg = True
            start_idx = i
        elif not moving and in_seg:
            end_idx = i - 1
            in_seg = False
            st, _ = odom[start_idx]
            et, _ = odom[end_idx]
            if et - st >= min_sec:
                segs.append(
                    Segment(
                        start_t=st,
                        end_t=et,
                        samples=[x for _, x in odom[start_idx : end_idx + 1]],
                    )
                )

    if in_seg and odom:
        end_idx = len(odom) - 1
        st, _ = odom[start_idx]
        et, _ = odom[end_idx]
        if et - st >= min_sec:
            segs.append(
                Segment(
                    start_t=st,
                    end_t=et,
                    samples=[x for _, x in odom[start_idx : end_idx + 1]],
                )
            )

    return segs


def trimmed_window_values(
    odom: list[tuple[float, float]], seg: Segment, trim_sec: float
) -> list[float]:
    a = seg.start_t + trim_sec
    b = seg.end_t - trim_sec
    vals = [v for t, v in odom if a <= t <= b]
    return vals if len(vals) >= 10 else list(seg.samples)


def main() -> int:
    args = parse_args()
    odom_path = Path(args.odom)
    cmd_path = Path(args.cmd) if args.cmd else None

    odom = read_odom(odom_path, args.odom_vel_col)
    if not odom:
        raise SystemExit("No valid odom rows found.")

    refs = read_cmd_refs(cmd_path, args.cmd_speed_col) if cmd_path else []
    segs = detect_moving_segments(odom, args.move_threshold, args.min_segment_sec)

    if not segs:
        raise SystemExit("No moving segments detected. Check threshold/columns.")

    print(f"odom rows: {len(odom)}")
    print(f"moving segments: {len(segs)}")
    if refs:
        print(f"reference targets from cmd (non-zero runs): {refs}")
    print("")
    print("idx,ref_mps,duration_s,n,mean_mps,std_mps,err_pct,result")

    all_pass = True
    for i, seg in enumerate(segs, start=1):
        vals = trimmed_window_values(odom, seg, args.trim_sec)
        mean_v = sum(vals) / len(vals)
        std_v = statistics.pstdev(vals) if len(vals) > 1 else 0.0
        ref = refs[i - 1] if i - 1 < len(refs) else None
        err = abs(mean_v - ref) / abs(ref) * 100.0 if ref and abs(ref) > 1e-12 else None
        err_ok = (err is not None) and (err <= args.target_err_pct)
        std_ok = std_v <= args.target_std_mps
        seg_pass = err_ok and std_ok
        all_pass = all_pass and seg_pass
        ref_s = f"{ref:.6f}" if ref is not None else ""
        err_s = f"{err:.2f}" if err is not None else ""
        result = "PASS" if seg_pass else "FAIL"
        print(
            f"{i},{ref_s},{seg.duration:.3f},{len(vals)},"
            f"{mean_v:.6f},{std_v:.6f},{err_s},{result}"
        )

    print("")
    print(
        f"criteria: err<={args.target_err_pct:.2f}% and std<={args.target_std_mps:.4f} m/s"
    )
    print(f"overall: {'PASS' if all_pass else 'FAIL'}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
