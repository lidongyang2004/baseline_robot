#!/usr/bin/env python3
"""Time-align trajectory CSVs for the default identity world -> odom transform."""

import argparse
from bisect import bisect_left
import csv
import json
import math
from pathlib import Path


def read_trajectory(filename):
    streams = {"filtered": [], "truth": []}
    skipped = 0
    frames = {"filtered": "baseline_robot/odom", "truth": "baseline_robot/world"}
    with Path(filename).open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            source = row.get("source")
            if source not in streams:
                continue
            try:
                # Keep nanosecond differences exact until interpolation.
                stamp = int(row["stamp_sec"])*1_000_000_000 + int(row["stamp_nanosec"])
                position = tuple(float(row[c]) for c in ("x", "y", "z"))
                if not all(math.isfinite(v) for v in position):
                    raise ValueError("Non-finite position")
            except (TypeError, ValueError, KeyError):
                skipped += 1
                continue
            if (row["frame_id"] != frames[source]
                    or row["child_frame_id"] != "baseline_robot/base_link"):
                raise ValueError("CSV does not use the default world/odom/base_link frames")
            if streams[source] and stamp <= streams[source][-1][0]:
                raise ValueError("Non-increasing timestamps; evaluate each simulation run separately")
            streams[source].append((stamp, position))
    if not all(streams.values()):
        raise ValueError("CSV must contain both filtered and truth positions")
    return streams, skipped


def aligned_errors(streams, warmup=5.0, max_gap=1.0, start_time=None, end_time=None):
    estimated = streams["filtered"]
    stamps = [sample[0] for sample in estimated]
    first = max(stamps[0], streams["truth"][0][0]) + round(warmup*1e9)
    if start_time is not None:
        first = max(first, round(start_time*1e9))
    last = min(stamps[-1], streams["truth"][-1][0])
    if end_time is not None:
        last = min(last, round(end_time*1e9))
    pairs = []
    for stamp, truth in streams["truth"]:
        if not first <= stamp <= last:
            continue
        index = bisect_left(stamps, stamp)
        if index < len(stamps) and stamps[index] == stamp:
            position = estimated[index][1]
        elif 0 < index < len(stamps):
            before, after = estimated[index-1], estimated[index]
            if after[0] - before[0] > round(max_gap*1e9):
                continue
            fraction = (stamp - before[0]) / (after[0] - before[0])
            position = tuple(a + fraction*(b-a) for a, b in zip(before[1], after[1]))
        else:
            continue
        error = tuple(a-b for a, b in zip(position, truth))
        pairs.append((stamp, position, truth, error))
    if not pairs:
        raise ValueError("No overlapping samples after warmup/time/gap filtering")
    return pairs


def summarize(pairs):
    squared_xy = [sum(v*v for v in row[3][:2]) for row in pairs]
    squared_xyz = [sum(v*v for v in row[3]) for row in pairs]
    return {
        "samples": len(pairs),
        "first_stamp": pairs[0][0] / 1e9,
        "last_stamp": pairs[-1][0] / 1e9,
        "duration_sec": (pairs[-1][0] - pairs[0][0]) / 1e9,
        "horizontal_rmse_m": math.sqrt(sum(squared_xy) / len(pairs)),
        "rmse_3d_m": math.sqrt(sum(squared_xyz) / len(pairs)),
        "max_horizontal_error_m": math.sqrt(max(squared_xy)),
        "max_error_3d_m": math.sqrt(max(squared_xyz)),
        "final_horizontal_error_m": math.sqrt(squared_xy[-1]),
        "final_error_3d_m": math.sqrt(squared_xyz[-1]),
        "final_error_xyz_m": pairs[-1][3],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--warmup", type=float, default=5.0)
    parser.add_argument("--max-gap", type=float, default=1.0)
    parser.add_argument("--start-time", type=float)
    parser.add_argument("--end-time", type=float)
    parser.add_argument("--json", type=Path, dest="json_path")
    parser.add_argument("--check", action="store_true", help="Check the near-zero-noise acceptance limits")
    args = parser.parse_args()
    if (not math.isfinite(args.warmup) or args.warmup < 0
            or not math.isfinite(args.max_gap) or args.max_gap <= 0):
        parser.error("warmup must be nonnegative and max-gap must be positive")
    try:
        streams, skipped = read_trajectory(args.csv_path)
        pairs = aligned_errors(streams, args.warmup, args.max_gap,
                               args.start_time, args.end_time)
    except ValueError as exc:
        parser.error(str(exc))
    summary = summarize(pairs)
    summary["csv_path"] = str(args.csv_path.resolve())
    summary["skipped_incomplete_rows"] = skipped
    summary["passed"] = (summary["horizontal_rmse_m"] < 0.10
                         and summary["rmse_3d_m"] < 0.15
                         and summary["final_error_3d_m"] < 0.10)
    output = json.dumps(summary, ensure_ascii=False, indent=2)
    if args.json_path:
        args.json_path.write_text(output + "\n", encoding="utf-8")
    print(output)
    if args.check and not summary["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
