#!/usr/bin/env python3
"""Audit recorded sensor variances and evaluate reported EKF marginal intervals."""

import argparse
from bisect import bisect_left
from collections import defaultdict
import csv
import json
import math
from pathlib import Path

from evaluate_navigation import read_trajectory


def covariance_samples(filename):
    samples = []
    with Path(filename).open() as stream:
        for row in csv.DictReader(stream):
            if row["source"] != "filtered":
                continue
            stamp = int(row["stamp_sec"])*1_000_000_000 + int(row["stamp_nanosec"])
            position = tuple(float(row[k]) for k in ("x", "y", "z"))
            xx, xy, xz, yy, yz, zz = (float(row[k]) for k in
                                       ("cov_xx", "cov_xy", "cov_xz", "cov_yy", "cov_yz", "cov_zz"))
            covariance = (xx, xy, xz, xy, yy, yz, xz, yz, zz)
            if not valid_covariance(covariance):
                raise ValueError("Invalid reported EKF position covariance")
            samples.append((stamp, position, covariance))
    return samples


def valid_covariance(c):
    if len(c) != 9 or not all(v is not None and math.isfinite(v) for v in c) or any(c[i] <= 0 for i in (0, 4, 8)):
        return False
    if any(not math.isclose(c[i], c[j], rel_tol=1e-7, abs_tol=1e-12)
           for i, j in ((1, 3), (2, 6), (5, 7))):
        return False
    tolerance = 1e-10*max(c[0], c[4], c[8])**2
    minors = (c[0]*c[4]-c[1]*c[3], c[0]*c[8]-c[2]*c[6], c[4]*c[8]-c[5]*c[7])
    determinant = (c[0]*(c[4]*c[8]-c[5]*c[7]) - c[1]*(c[3]*c[8]-c[5]*c[6])
                   + c[2]*(c[3]*c[7]-c[4]*c[6]))
    return min(minors) >= -tolerance and determinant >= -tolerance*max(c[0], c[4], c[8])


def aligned_uncertainty(filename, start_time=None, end_time=None, warmup=5.0, max_gap=1.0):
    streams, _ = read_trajectory(filename)
    estimates = covariance_samples(filename)
    truth = streams["truth"]
    stamps = [sample[0] for sample in truth]
    first = max(estimates[0][0], stamps[0]) + round(warmup*1e9)
    if start_time is not None:
        first = max(first, round(start_time*1e9))
    last = min(estimates[-1][0], stamps[-1])
    if end_time is not None:
        last = min(last, round(end_time*1e9))
    rows = []
    for stamp, position, covariance in estimates:
        if not first <= stamp <= last:
            continue
        index = bisect_left(stamps, stamp)
        if index < len(stamps) and stamps[index] == stamp:
            reference = truth[index][1]
        elif 0 < index < len(stamps):
            before, after = truth[index-1], truth[index]
            if after[0]-before[0] > round(max_gap*1e9):
                continue
            fraction = (stamp-before[0])/(after[0]-before[0])
            reference = tuple(a+fraction*(b-a) for a, b in zip(before[1], after[1]))
        else:
            continue
        error = tuple(a-b for a, b in zip(position, reference))
        sigma = tuple(math.sqrt(covariance[i]) for i in (0, 4, 8))
        rows.append((stamp, error, sigma))
    if not rows:
        raise ValueError("No overlapping uncertainty samples")
    return rows


def interval_summary(rows):
    return {
        "samples": len(rows),
        "alignment": "ground truth interpolated to each filtered timestamp; P used at its own timestamp",
        "coverage_2sigma": {axis: sum(abs(row[1][i]) <= 2*row[2][i] for row in rows)/len(rows)
                            for i, axis in enumerate("xyz")},
        "mean_sigma_m": {axis: sum(row[2][i] for row in rows)/len(rows)
                         for i, axis in enumerate("xyz")},
        "note": "Marginal coverage is reported, not a claim of calibrated covariance or joint 95% coverage.",
    }


def audit_sensors(filename):
    counts = defaultdict(lambda: {"total": 0, "valid": 0, "invalid": 0})
    mismatches, examples, models = 0, [], None

    def compare(source, actual, expected):
        nonlocal mismatches
        if (len(actual) != len(expected) or not all(
                a is not None and math.isfinite(a) and a > 0
                and math.isclose(a, e, rel_tol=1e-6, abs_tol=1e-12)
                for a, e in zip(actual, expected))):
            mismatches += 1
            if len(examples) < 5:
                examples.append({"source": source, "actual": actual, "expected": expected})

    with Path(filename).open() as stream:
        for line in stream:
            row = json.loads(line)
            if row["record_type"] == "noise_models":
                models = row["models"]
                continue
            if models is None:
                raise ValueError("Missing noise model metadata")
            source, data = row["source"], row["data"]
            counts[source]["total"] += 1
            counts[source]["valid" if row["valid"] else "invalid"] += 1
            if source in ("imu_raw", "imu"):
                for field, key in (("orientation_covariance", "orientation_stddev_rad"),
                                   ("angular_velocity_covariance", "angular_velocity_stddev_rad_s"),
                                   ("linear_acceleration_covariance", "linear_acceleration_stddev_m_s2")):
                    compare(source, [data[field][i] for i in (0, 4, 8)], [s*s for s in models["imu"][key]])
            elif source in ("dvl_raw", "dvl"):
                c = data["velocity_covariance"] if source == "dvl_raw" else data["twist"]["covariance"]
                compare(source, [c[i] for i in ((0, 4, 8) if source == "dvl_raw" else (0, 7, 14))],
                        [models["dvl"]["velocity_stddev_m_s"]**2]*3)
            elif source == "dvl_altitude":
                compare(source, [data["pose"]["covariance"][14]], [models["dvl"]["altitude_stddev_m"]**2])
            elif source == "pressure":
                compare(source, [data["variance"]], [models["pressure"]["stddev_pa"]**2])
            elif source == "gps":
                compare(source, [data["position_covariance"][i] for i in (0, 4)],
                        [models["gps"]["horizontal_stddev_m"]**2]*2)
            elif source == "usbl_raw":
                compare(source, row["primitive_variances"], [models["usbl"][k]**2 for k in
                        ("range_stddev_m", "azimuth_stddev_rad", "elevation_stddev_rad")])
            elif source in ("depth", "usbl"):
                c = data["pose"]["covariance"]
                valid = (c[14] is not None and math.isfinite(c[14]) and c[14] > 0) if source == "depth" else valid_covariance(
                    [c[i] for i in (0, 1, 2, 6, 7, 8, 12, 13, 14)])
                if not valid:
                    mismatches += 1
    required = {"imu_raw", "imu", "dvl_raw", "dvl", "dvl_altitude", "pressure", "depth",
                "usbl_raw", "usbl", "truth_raw"}
    missing = sorted(source for source in required if counts[source]["valid"] == 0)
    if counts["gps"]["total"] == 0:
        missing.append("gps messages (NO_FIX is allowed)")
    return {"noise_models": models, "counts": dict(counts), "covariance_mismatches": mismatches,
            "mismatch_examples": examples, "missing_sources": missing,
            "passed": mismatches == 0 and not missing}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--sensors", type=Path)
    parser.add_argument("--start-time", type=float)
    parser.add_argument("--end-time", type=float)
    parser.add_argument("--json", type=Path, dest="json_path")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    try:
        summary = interval_summary(aligned_uncertainty(args.csv_path, args.start_time, args.end_time))
        sensors = args.sensors or args.csv_path.with_name(args.csv_path.stem + "-sensors.jsonl")
        summary["sensor_audit"] = audit_sensors(sensors)
    except (ValueError, KeyError, IndexError) as exc:
        parser.error(str(exc))
    text = json.dumps(summary, indent=2)
    if args.json_path:
        args.json_path.write_text(text + "\n")
    print(text)
    if args.check and not summary["sensor_audit"]["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
