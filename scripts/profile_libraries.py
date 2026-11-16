#!/usr/bin/env python3
"""Sample prebuilt macOS workers under an external exclusive machine reservation."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import subprocess
import time

from bench_libraries import CASES, command, source_metadata, worker


def summarize(path):
    text = path.read_text()
    total = re.search(r"^\s*(\d+) Thread_\d+.*com.apple.main-thread", text, re.M)
    start = text.index("Sort by top of stack")
    end = text.index("Binary Images:", start)
    rows = []
    for line in text[start:end].splitlines()[1:]:
        match = re.fullmatch(r"\s*(.*?)\s+\(in ([^)]+)\)\s+(\d+)\s*", line)
        if match:
            symbol, module, count = match.groups()
            rows.append({"symbol": re.sub(r"::h[0-9a-f]{16}$", "", symbol), "module": module, "samples": int(count)})
    if total is None or not rows:
        raise RuntimeError("sample report lacks a main-thread total or symbol rows")
    return {"main_thread_samples": int(total[1]), "top_of_stack": rows,
            "raw_sample_sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("rust", "native", "fixtures", "output"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    parser.add_argument("--cases", nargs="+", choices=CASES, default=CASES)
    parser.add_argument("--seconds", type=int, default=3)
    args = parser.parse_args()
    if platform.system() != "Darwin" or args.seconds < 1:
        parser.error("requires macOS and a positive sample duration")
    executables = {"rust": args.rust.resolve(), "native": args.native.resolve()}
    fixtures = args.fixtures.resolve()
    env = os.environ.copy()
    env.pop("DYLD_INSERT_LIBRARIES", None)
    directory = args.output.with_suffix("").resolve()
    directory.mkdir(parents=True, exist_ok=True)
    report = {"schema_version": 1, **source_metadata(Path(__file__).resolve().parents[1]),
              "build_info": json.loads(command([str(executables["rust"]), "build-info", str(fixtures)]).stdout),
              "tool": "/usr/bin/sample", "sample_seconds": args.seconds, "interval_ms": 1,
              "limits": "Sample counts identify hotspots; sampling overhead and short captures preclude precise timing attribution.",
              "executables": {key: {"path": exe.name, "sha256": hashlib.sha256(exe.read_bytes()).hexdigest()} for key, exe in executables.items()},
              "workloads": {}}
    for case in args.cases:
        initial = {key: worker(exe, case, fixtures, 1) for key, exe in executables.items()}
        if len({value["checksum"] for value in initial.values()}) != 1:
            raise RuntimeError(f"initial checksum mismatch: {case}")
        episodes = max(1, math.ceil((args.seconds + 3) * 1e9 / min(v["elapsed_ns"] for v in initial.values())))
        results = {}
        for key, exe in executables.items():
            path = directory / f"{key}-{case}.sample.txt"
            with (directory / f"{key}-{case}.json").open("w") as out:
                child = subprocess.Popen([str(exe), case, str(fixtures), str(episodes)], stdout=out, env=env)
                try:
                    time.sleep(.2)
                    sampled = subprocess.run(["/usr/bin/sample", str(child.pid), str(args.seconds), "1", "-mayDie", "-file", str(path)], capture_output=True, text=True)
                    if sampled.returncode:
                        raise RuntimeError(sampled.stdout + sampled.stderr)
                    if child.wait() != 0:
                        raise RuntimeError(f"worker failed: {key} {case}")
                finally:
                    if child.poll() is None:
                        child.terminate()
                        child.wait()
            value = json.loads((directory / f"{key}-{case}.json").read_text())
            if value["case"] != case or value["iterations"] != episodes:
                raise RuntimeError("profiled worker returned the wrong workload")
            results[key] = {**summarize(path), "iterations": episodes, "checksum": value["checksum"]}
            print(f"{key} {case}: sampled {episodes} episodes", flush=True)
        if len({value["checksum"] for value in results.values()}) != 1:
            raise RuntimeError(f"profiled checksum mismatch: {case}")
        report["workloads"][case] = results
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
