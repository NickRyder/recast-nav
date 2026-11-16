#!/usr/bin/env python3
"""Attribute calibrated requested bytes to call stacks, separately from timing."""
import argparse
from collections import defaultdict
import hashlib
import json
import os
from pathlib import Path
import platform
import re

from bench_libraries import CASES, allocations, command, source_metadata


def capture(executable, arguments, profiler, raw_path):
    env = {**os.environ, "DYLD_INSERT_LIBRARIES": str(profiler), "RECAST_ALLOC_STACKS": "1"}
    process = command([str(executable), *map(str, arguments)], env)
    raw_path.write_text(process.stderr)
    counts = allocations(process.stderr)
    lines = [s.removeprefix("ALLOCATION_STACKS ") for s in process.stderr.splitlines() if s.startswith("ALLOCATION_STACKS ")]
    if len(lines) != 1:
        raise RuntimeError("expected one allocation stack report")
    trace = json.loads(lines[0])
    if trace["dropped_calls"] or trace["dropped_bytes"]:
        raise RuntimeError("allocation stack table overflow or failed unwind")
    for key in ("calls", "requested_bytes"):
        if sum(s[key] for s in trace["stacks"]) != counts[key]:
            raise RuntimeError("allocation trace does not cover all counter totals")
    stacks = defaultdict(lambda: {"calls": 0, "requested_bytes": 0})
    addresses = {}
    for stack in trace["stacks"]:
        frames = []
        for frame in stack["frames"]:
            module = frame["module"]
            if module == profiler.name:
                continue
            offset = int(frame["pc"], 16) - int(frame["base"], 16) if module != "unknown" else 0
            key = (module, offset)
            frames.append(key)
            addresses.setdefault(key, (frame["pc"], frame["base"]))
        for key in ("calls", "requested_bytes"):
            stacks[tuple(frames)][key] += stack[key]
    return counts, dict(stacks), addresses, json.loads(process.stdout) if process.stdout.strip() else None, hashlib.sha256(process.stderr.encode()).hexdigest()


def slope(first, second):
    delta = {key: second[key] - first[key] for key in first}
    if any(v < 0 or v % 3 for v in delta.values()):
        raise RuntimeError(f"unstable allocation slope: {delta}")
    return {key: value // 3 for key, value in delta.items()}


def symbolize(executable, addresses):
    groups = defaultdict(list)
    for (module, offset), (pc, base) in addresses.items():
        if module == executable.name:
            groups[base].append((offset, pc))
    symbols = {}
    for base, values in groups.items():
        result = command(["/usr/bin/atos", "-o", str(executable), "-l", "0x" + base, *["0x" + pc for _, pc in values]])
        lines = result.stdout.splitlines()
        if len(lines) != len(values):
            raise RuntimeError("symbolizer returned the wrong number of frames")
        for (offset, _), line in zip(values, lines):
            symbols[(executable.name, offset)] = line.split(" (in ", 1)[0].strip()
    return symbols


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("rust", "native", "fixtures", "profiler", "calibration", "output"):
        parser.add_argument("--" + name, required=True, type=Path)
    parser.add_argument("--worker-revision", required=True)
    parser.add_argument("--cases", nargs="+", choices=CASES, default=["recast-layer-capacity", "recast-stacked", "tilecache-builder", "tilecache-stream", "crowd", "detour-lifecycle", "avoidance-adaptive"])
    args = parser.parse_args()
    if platform.system() != "Darwin" or not re.fullmatch("[0-9a-f]{40}", args.worker_revision):
        parser.error("requires macOS and a full frozen worker revision")
    workers = {"rust": args.rust.resolve(), "native": args.native.resolve()}
    profiler = args.profiler.resolve()
    fixtures = args.fixtures.resolve()
    directory = args.output.with_suffix("").resolve()
    directory.mkdir(parents=True, exist_ok=True)
    report = {"schema_version": 1, **source_metadata(Path(__file__).resolve().parents[1]),
              "worker_revision": args.worker_revision, "platform": platform.platform(),
              "method": "main-thread successful allocation stacks; three identical slopes of six versus three episodes; unwind/symbolization excluded from timing",
              "limits": "Requested allocation traffic, not retained/peak bytes. Fixed 24-frame stacks; reject dropped captures. Unknown frames are marked, not interpreted as source attribution. Raw captures stay outside published reports.",
              "profiler_sha256": hashlib.sha256(profiler.read_bytes()).hexdigest(),
              "executables": {k: {"path": p.name, "sha256": hashlib.sha256(p.read_bytes()).hexdigest()} for k, p in workers.items()},
              "fixtures": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(fixtures.glob("*.bin"))},
              "workloads": {}}
    calibration = []
    calibration_hashes = []
    for count in (100, 200):
        values, _, _, _, raw = capture(args.calibration.resolve(), [count], profiler, directory / f"calibration-{count}.stderr")
        calibration.append(values)
        calibration_hashes.append(raw)
    delta = {k: calibration[1][k] - calibration[0][k] for k in calibration[0]}
    if delta != {"calls": 600, "requested_bytes": 66800, "frees": 500}:
        raise RuntimeError(f"allocator calibration failed: {delta}")
    report["allocation_calibration"] = {"counts_100": calibration[0], "counts_200": calibration[1], "delta": delta, "raw_sha256": calibration_hashes}
    for case in args.cases:
        result = {}
        for role, executable in workers.items():
            samples, totals, hashes, checksums, addresses = [], [], [], [], {}
            for repetition in range(3):
                pair = []
                for count in (3, 6):
                    item = capture(executable, [case, fixtures, count], profiler, directory / f"{role}-{case}-{count}-{repetition}.stderr")
                    counts, stacks, frames, value, raw = item
                    if value["case"] != case or value["iterations"] != count:
                        raise RuntimeError("worker returned the wrong workload")
                    hashes.append(raw)
                    checksums.append(value["checksum"])
                    addresses.update(frames)
                    pair.append((counts, stacks))
                total = slope(pair[0][0], pair[1][0])
                stacks = {}
                zero = {"calls": 0, "requested_bytes": 0}
                for stack in pair[0][1].keys() | pair[1][1].keys():
                    value = slope(pair[0][1].get(stack, zero), pair[1][1].get(stack, zero))
                    if any(value.values()):
                        stacks[stack] = value
                for key in zero:
                    if sum(v[key] for v in stacks.values()) != total[key]:
                        raise RuntimeError("attributed stack slopes do not sum to the episode total")
                samples.append(stacks)
                totals.append(total)
            if any(s != samples[0] for s in samples) or any(t != totals[0] for t in totals):
                raise RuntimeError(f"unstable stack attribution: {case} {role}")
            if len(set(checksums[::2])) != 1 or len(set(checksums[1::2])) != 1:
                raise RuntimeError("inconsistent repeated checksums")
            symbols = symbolize(executable, addresses)
            rows = [{**value, "frames": [{"module": module, "image_offset": hex(offset), "symbol": symbols.get((module, offset))} for module, offset in stack]} for stack, value in samples[0].items()]
            rows.sort(key=lambda v: (-v["requested_bytes"], -v["calls"], json.dumps(v["frames"], sort_keys=True)))
            result[role] = {"per_episode": totals[0], "stacks": rows, "raw_sha256": hashes,
                            "three_episode_checksum": checksums[0], "six_episode_checksum": checksums[1]}
            print(case, role, totals[0], flush=True)
        for count in ("three_episode_checksum", "six_episode_checksum"):
            if result["rust"][count] != result["native"][count]:
                raise RuntimeError(f"native checksum mismatch: {case}")
        report["workloads"][case] = result
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print("Saved", args.output)


if __name__ == "__main__":
    main()
