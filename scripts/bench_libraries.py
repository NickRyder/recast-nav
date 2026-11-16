#!/usr/bin/env python3
"""Measure already-built, checksum-matched workers; run under bb exclusive."""
import argparse
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import platform
import re
import statistics
import subprocess
import sys
import time

CASES = ("recast", "detour", "tilecache", "crowd", "recast-monotone", "recast-layers",
         "detour-spatial", "tilecache-shapes", "proximity-grid", "avoidance-grid", "avoidance-adaptive", "detour-lifecycle", "detour-search", "detour-sliced", "tilecache-codec", "tilecache-builder", "tilecache-stream", "recast-utilities", "recast-stacked", "recast-stacked-monotone", "recast-stacked-layers", "recast-layer-capacity", "crowd-small", "crowd-dense", "crowd-offmesh", "crowd-corridor", "crowd-path-queue")


def command(args, env=None):
    return subprocess.run(args, check=True, text=True, capture_output=True, env=env)


def source_metadata(root):
    root = root.resolve()
    git = ["git", "-C", str(root)]
    try:
        top = command([*git, "rev-parse", "--show-toplevel"]).stdout.strip()
        if Path(top).resolve() == root:
            return {"head": command([*git, "rev-parse", "HEAD"]).stdout.strip(),
                    "source_diff_sha256": hashlib.sha256(command([*git, "diff", "HEAD"]).stdout.encode()).hexdigest(),
                    "source_origin": "git"}
    except (FileNotFoundError, subprocess.CalledProcessError):
        pass
    revision = root / "SOURCE_REVISION"
    value = revision.read_text().strip() if revision.is_file() else ""
    return {"head": value if re.fullmatch(r"[0-9a-f]{40}", value) else None,
            "source_diff_sha256": None, "source_origin": "archive-or-unversioned"}


def allocations(stderr):
    lines = [s.removeprefix("ALLOCATIONS ") for s in stderr.splitlines() if s.startswith("ALLOCATIONS ")]
    if len(lines) != 1:
        raise RuntimeError("allocation profiler did not report exactly once")
    return json.loads(lines[0])


def worker(exe, case, fixtures, n, profiler=None, rss=False):
    args = [str(exe), case, str(fixtures), str(n)]
    env = os.environ.copy()
    env.pop("DYLD_INSERT_LIBRARIES", None)
    env.pop("RECAST_ALLOC_STACKS", None)
    if profiler:
        env["DYLD_INSERT_LIBRARIES"] = str(profiler)
    if rss:
        if platform.system() == "Darwin":
            args = ["/usr/bin/time", "-l", *args]
        elif platform.system() == "Linux":
            args = ["/usr/bin/time", "-f", "PEAK_RSS_KIB %M", *args]
    p = command(args, env)
    value = json.loads(p.stdout)
    if value["case"] != case or value["iterations"] != n:
        raise RuntimeError("worker returned the wrong workload")
    if profiler:
        value["allocations"] = allocations(p.stderr)
    if rss:
        match = re.search(r"(\d+)\s+maximum resident set size", p.stderr)
        if match:
            value["peak_rss_bytes"] = int(match[1])
        else:
            match = re.search(r"PEAK_RSS_KIB (\d+)", p.stderr)
            if match:
                value["peak_rss_bytes"] = int(match[1]) * 1024
    return value


def calibrate(profiler, executable):
    env = {**os.environ, "DYLD_INSERT_LIBRARIES": str(profiler)}
    env.pop("RECAST_ALLOC_STACKS", None)
    first, second = [allocations(command([str(executable), str(n)], env).stderr) for n in (100, 200)]
    delta = {key: second[key] - first[key] for key in first}
    if delta != {"calls": 600, "requested_bytes": 66800, "frees": 500}:
        raise RuntimeError(f"allocation calibration failed: {delta}")
    return {"counts_100": first, "counts_200": second, "delta": delta}


def allocation_samples(exe, case, fixtures, profiler):
    samples = []
    for _ in range(3):
        first, second = [worker(exe, case, fixtures, n, profiler) for n in (3, 6)]
        delta = {key: second["allocations"][key] - first["allocations"][key] for key in first["allocations"]}
        if any(v < 0 or v % 3 for v in delta.values()):
            raise RuntimeError(f"unstable allocation slope: {case} {delta}")
        samples.append({key: value // 3 for key, value in delta.items()})
    if any(s != samples[0] for s in samples):
        raise RuntimeError(f"allocation counts vary between runs: {case} {samples}")
    return {"per_episode": samples[0], "samples": samples, "method": "main-thread successful allocator calls; slope of 3 versus 6 episodes; three identical repetitions"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("rust", "native", "fixtures", "output"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    parser.add_argument("--profiler", type=Path)
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--baseline-rust", type=Path)
    parser.add_argument("--baseline-revision")
    parser.add_argument("--cases", nargs="+", choices=CASES, default=CASES)
    parser.add_argument("--repetitions", type=int, default=7)
    parser.add_argument("--target-ms", type=float, default=250.)
    args = parser.parse_args()
    if args.repetitions < 3 or not math.isfinite(args.target_ms) or args.target_ms <= 0 or bool(args.profiler) != bool(args.calibration):
        parser.error("need at least three repetitions, a positive target time, and paired profiler/calibration paths")
    if args.baseline_revision and (not args.baseline_rust or not re.fullmatch(r"[0-9a-f]{40}", args.baseline_revision)):
        parser.error("baseline revision needs a baseline Rust binary and a full hexadecimal commit ID")
    executables = {"rust": args.rust.resolve(), "native": args.native.resolve()}
    if args.baseline_rust:
        executables["baseline_rust"] = args.baseline_rust.resolve()
    args.fixtures = args.fixtures.resolve()
    build_info = json.loads(command([str(executables["rust"]), "build-info", str(args.fixtures)]).stdout)
    report = {"schema_version": 3, "detour_output_policy": "all workers allocate reusable outputs before timing: original paths 256 entries; spatial outputs 256/64/18; search entries 1024 and path backing 1025, with advertised budgets hashed per call", "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "platform": platform.platform(), "machine": platform.machine(),
              **source_metadata(Path(__file__).resolve().parents[1]),
              "tilecache_snapshot_policy": build_info.get("tilecache_snapshot_policy"),
              "measurement_policy": {"cases": args.cases, "repetitions": args.repetitions, "target_ms": args.target_ms, "validate_only": args.validate_only},
              "rustc": build_info["rustc"],
              "cxx": build_info["cxx"],
              "rust_policy": "release opt-level=3; default codegen units; no LTO; fma/polyref64 disabled",
              "native_policy": "-O3 -fno-fast-math -ffp-contract=off -fno-builtin-sinf -fno-builtin-cosf; unmodified pinned library translation units",
              "executables": {key: {"path": str(exe), "sha256": hashlib.sha256(exe.read_bytes()).hexdigest()} for key, exe in executables.items()},
              "fixtures": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(args.fixtures.glob("*.bin"))}, "workloads": {}}
    if args.baseline_rust:
        report["baseline_revision"] = args.baseline_revision
        report["baseline_build_info"] = json.loads(command([str(executables["baseline_rust"]), "build-info", str(args.fixtures)]).stdout)
    if {"tilecache", "tilecache-shapes"}.intersection(args.cases) and args.baseline_rust:
        if report["tilecache_snapshot_policy"] != report["baseline_build_info"].get("tilecache_snapshot_policy"):
            raise RuntimeError("baseline and current TileCache snapshot ownership policies differ")
    if platform.system() == "Darwin":
        report["hardware"] = command(["sysctl", "-n", "machdep.cpu.brand_string"]).stdout.strip()
    # Validate every workload before starting any measurements.
    for case in args.cases:
        initial = {key: worker(exe, case, args.fixtures, 1) for key, exe in executables.items()}
        if len({value["checksum"] for value in initial.values()}) != 1:
            raise RuntimeError(f"checksum mismatch: {case}: {initial}")
        report["workloads"][case] = {"validation": initial}
        print(f"{case}: checksum {initial['rust']['checksum']} matches", flush=True)
    if args.profiler:
        report["allocation_calibration"] = calibrate(args.profiler, args.calibration)
        for exe in executables.values():
            symbols = command(["nm", "-u", str(exe)]).stdout
            if re.search(r"malloc_type|reallocf|reallocarray|malloc_zone_", symbols):
                raise RuntimeError("uncovered allocator imports")
    if not args.validate_only:
        for case in args.cases:
            data = report["workloads"][case]
            quickest = min(v["elapsed_ns"] for v in data["validation"].values())
            n = max(1, math.ceil(args.target_ms * 1_000_000 / quickest))
            data["iterations_per_sample"] = n
            data["samples"] = {key: [] for key in executables}
            # Initial validation warmed each worker's filesystem pages. Each
            # timed child repeats complete episodes with process setup excluded.
            orders = list(itertools.permutations(executables))
            data["sample_orders"] = []
            for repetition in range(args.repetitions):
                order = orders[repetition % len(orders)]
                data["sample_orders"].append(order)
                pair = {}
                for key in order:
                    pair[key] = worker(executables[key], case, args.fixtures, n, rss=True)
                    data["samples"][key].append(pair[key])
                if len({value["checksum"] for value in pair.values()}) != 1:
                    raise RuntimeError(f"timed checksum mismatch: {case}")
            data["median_ns_per_episode"] = {key: statistics.median(s["elapsed_ns"] / n for s in values) for key, values in data["samples"].items()}
            data["rust_over_native_time"] = data["median_ns_per_episode"]["rust"] / data["median_ns_per_episode"]["native"]
            if args.baseline_rust:
                data["baseline_over_native_time"] = data["median_ns_per_episode"]["baseline_rust"] / data["median_ns_per_episode"]["native"]
                data["rust_over_baseline_time"] = data["median_ns_per_episode"]["rust"] / data["median_ns_per_episode"]["baseline_rust"]
            if args.profiler:
                data["allocations"] = {key: allocation_samples(exe, case, args.fixtures, args.profiler) for key, exe in executables.items()}
            print(f"{case}: median Rust/native time {data['rust_over_native_time']:.3f}x", flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(f"Saved {args.output}", flush=True)


if __name__ == "__main__":
    main()
