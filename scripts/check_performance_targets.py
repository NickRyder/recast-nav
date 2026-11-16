#!/usr/bin/env python3
"""Check per-workload Rust time/allocated-byte targets against matched C++ data."""
import argparse
from collections import Counter
from fractions import Fraction
import hashlib
import itertools
import json
import math
from pathlib import Path
import re
import statistics

from bench_libraries import CASES

# Keep the original workload set when later workloads are reorganized.
INITIAL_CASES = {
    "recast", "detour", "tilecache", "crowd", "recast-monotone", "recast-layers",
    "detour-spatial", "tilecache-shapes", "proximity-grid", "avoidance-grid",
    "avoidance-adaptive", "detour-lifecycle", "detour-search", "detour-sliced",
    "tilecache-codec", "tilecache-builder", "tilecache-stream",
}


def check(report, diagnostic):
    if report.get("schema_version") != 3:
        raise ValueError("requires a complete schema-3 benchmark report")
    policy = report["measurement_policy"]
    if policy["validate_only"]:
        raise ValueError("validation-only reports contain no timing evidence")
    if report.get("allocation_calibration", {}).get("delta") != {
        "calls": 600, "requested_bytes": 66800, "frees": 500,
    }:
        raise ValueError("missing successful allocator calibration")
    missing = sorted((INITIAL_CASES | set(CASES)) - report["workloads"].keys())
    limits = []
    if report.get("tilecache_snapshot_policy") != "borrowed-tile-bytes-v1":
        limits.append("TileCache snapshots do not establish matched borrowed-output ownership")
    if missing:
        limits.append("missing workloads: " + ", ".join(missing))
    if policy["repetitions"] < 12 or policy["target_ms"] < 1000:
        limits.append("acceptance requires at least 12 balanced repetitions and a 1000 ms sample target")
    if not re.fullmatch(r"[0-9a-f]{40}", report.get("head") or ""):
        limits.append("missing source revision")
    clean = report.get("source_diff_sha256") == hashlib.sha256(b"").hexdigest()
    archive = report.get("source_origin") == "archive-or-unversioned" and report.get("source_diff_sha256") is None
    if not (clean or archive):
        limits.append("measurement source has uncommitted changes")
    if not report.get("platform", "").startswith("macOS-"):
        limits.append("report is not from the requested macOS platform")
    if report.get("rust_policy") != "release opt-level=3; default codegen units; no LTO; fma/polyref64 disabled":
        limits.append("unexpected Rust build policy")
    if report.get("native_policy") != "-O3 -fno-fast-math -ffp-contract=off -fno-builtin-sinf -fno-builtin-cosf; unmodified pinned library translation units":
        limits.append("unexpected native build policy")
    rows = []
    for case, data in report["workloads"].items():
        roles = list(data["samples"])
        if not {"rust", "native"}.issubset(roles):
            raise ValueError(f"{case}: missing workers")
        if set(data["validation"]) != set(roles):
            raise ValueError(f"{case}: missing worker validation")
        if any(v["case"] != case or v["iterations"] != 1 for v in data["validation"].values()):
            raise ValueError(f"{case}: wrong validation workload or count")
        if len({v["checksum"] for v in data["validation"].values()}) != 1:
            raise ValueError(f"{case}: initial checksum mismatch")
        orders = Counter(tuple(order) for order in data["sample_orders"])
        if (sum(orders.values()) != policy["repetitions"]
                or set(orders) != set(itertools.permutations(roles))
                or len(set(orders.values())) != 1):
            limits.append(f"{case}: unbalanced sample orders")
        medians = {}
        for role, samples in data["samples"].items():
            if len(samples) != policy["repetitions"]:
                raise ValueError(f"{case}: wrong sample count")
            times = []
            for sample in samples:
                if sample["case"] != case or sample["iterations"] != data["iterations_per_sample"] or sample["iterations"] <= 0 or sample["elapsed_ns"] <= 0:
                    raise ValueError(f"{case}: invalid timed episode count or duration")
                times.append(Fraction(sample["elapsed_ns"], sample["iterations"]))
            medians[role] = statistics.median(times)
            if not math.isclose(float(medians[role]), data["median_ns_per_episode"][role], rel_tol=1e-12):
                raise ValueError(f"{case}: derived median disagrees with raw samples")
        for repetition in range(policy["repetitions"]):
            if len({data["samples"][role][repetition]["checksum"] for role in roles}) != 1:
                raise ValueError(f"{case}: timed checksum mismatch")
        allocations = {}
        for role in ("rust", "native"):
            values = data["allocations"][role]
            if len(values["samples"]) != 3 or any(s != values["per_episode"] for s in values["samples"]):
                raise ValueError(f"{case}: unstable allocation slopes")
            if any(v < 0 for v in values["per_episode"].values()):
                raise ValueError(f"{case}: negative allocation count")
            allocations[role] = values["per_episode"]
        speed = medians["rust"] <= medians["native"]
        size = allocations["rust"]["requested_bytes"] <= allocations["native"]["requested_bytes"]
        rows.append({"workload": case, "rust_over_cpp_time": float(medians["rust"] / medians["native"]),
                     "rust_requested_bytes": allocations["rust"]["requested_bytes"],
                     "cpp_requested_bytes": allocations["native"]["requested_bytes"],
                     "speed_target_met": speed, "allocation_size_target_met": size,
                     "both_targets_met": speed and size})
    passed = all(row["both_targets_met"] for row in rows) and not limits
    return {"source_commit": report["head"], "accepted": passed and not diagnostic,
            "diagnostic": diagnostic, "acceptance_limits": limits,
            "workloads_meeting_both": sum(row["both_targets_met"] for row in rows),
            "workloads_measured": len(rows), "workloads": rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("--diagnostic", action="store_true", help="report partial/short-run progress without claiming acceptance")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        result = check(json.loads(args.report.read_text()), args.diagnostic)
    except (KeyError, TypeError, ValueError) as error:
        parser.error(str(error))
    for row in result["workloads"]:
        print(f"{row['workload']}: time {row['rust_over_cpp_time']:.3f}x C++; bytes {row['rust_requested_bytes']}/{row['cpp_requested_bytes']}; {'PASS' if row['both_targets_met'] else 'FAIL'}")
    for limit in result["acceptance_limits"]:
        print("Acceptance pending: " + limit)
    print(f"{result['workloads_meeting_both']}/{result['workloads_measured']} measured workloads meet both targets.")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
    # A diagnostic report can never stand in for the final acceptance gate.
    raise SystemExit(0 if result["accepted"] and not args.diagnostic else 1)


if __name__ == "__main__":
    main()
