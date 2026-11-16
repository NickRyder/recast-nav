"""Check locked package sources, the local trust boundary, and core isolation."""
import argparse
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parent.parent
REGISTRY = "registry+https://github.com/rust-lang/crates.io-index"
LOCAL = {
    "recast-nav": "crates/recast-nav",
    "bevy-recast-nav": "crates/bevy-recast-nav",
}
APPROVED_LICENSES = {
    "(MIT OR Apache-2.0) AND Unicode-3.0",
    "(MIT OR Apache-2.0) AND Zlib AND BSD-3-Clause",
    "Apache-2.0 OR MIT", "Apache-2.0/MIT", "MIT", "MIT OR Apache-2.0",
    "MIT OR Apache-2.0 OR Zlib", "MIT/Apache-2.0", "Unlicense OR MIT",
    "Zlib", "Zlib OR Apache-2.0 OR MIT",
}


def check(offline=False, target=None):
    lock = (ROOT / "Cargo.lock").read_text()
    registry_packages = 0
    for package in lock.split("[[package]]")[1:]:
        sources = re.findall(r'^source = "([^"]+)"$', package, re.MULTILINE)
        if sources:
            if sources != [REGISTRY]:
                raise SystemExit(f"Unapproved dependency source: {sources}")
            if not re.search(r'^checksum = "[0-9a-f]{64}"$', package, re.MULTILINE):
                raise SystemExit("Registry package has no SHA-256 lockfile checksum")
            registry_packages += 1
    for name, directory in LOCAL.items():
        source = (ROOT / directory / "src/lib.rs").read_text()
        if "#![forbid(unsafe_code)]" not in source.splitlines():
            raise SystemExit(f"{name} must forbid unsafe Rust at its crate root")
    if target is None:
        version = subprocess.check_output(["rustc", "-vV"], text=True)
        target = re.search(r"^host: (.+)$", version, re.MULTILINE)[1]
    command = ["cargo", "metadata", "--locked", "--format-version", "1",
               "--filter-platform", target]
    if offline:
        command.append("--offline")
    metadata = json.loads(subprocess.check_output(command, cwd=ROOT, text=True))
    packages = {package["id"]: package for package in metadata["packages"]}
    local_ids = {}
    for identifier, package in packages.items():
        if package["license"] not in APPROVED_LICENSES:
            raise SystemExit(f"Unreviewed license for {package['name']}: {package['license']}")
        minimum = package.get("rust_version")
        if minimum and tuple(map(int, minimum.split(".")[:2])) > (1, 95):
            raise SystemExit(f"{package['name']} requires Rust {minimum}, above our MSRV")
        source = package["source"]
        if source is not None:
            if source != REGISTRY:
                raise SystemExit(f"Unapproved source for {package['name']}: {source}")
        else:
            directory = LOCAL.get(package["name"])
            expected = ROOT / directory / "Cargo.toml" if directory else None
            if expected is None or Path(package["manifest_path"]).resolve() != expected.resolve():
                raise SystemExit(f"Unapproved local package: {package['name']}")
            local_ids[package["name"]] = identifier
    if set(local_ids) != set(LOCAL):
        raise SystemExit("Expected local core and adapter packages")
    # metadata's workspace-wide resolution may list inactive optional edges.
    # Ask Cargo for the actual core-only normal dependency graph instead.
    tree_command = ["cargo", "tree", "--locked", "--target", target,
                    "-p", "recast-nav", "-e", "normal", "--prefix", "none",
                    "--format", "{p}"]
    if offline:
        tree_command.append("--offline")
    tree = subprocess.check_output(tree_command, cwd=ROOT, text=True)
    core_names = {line.split()[0] for line in tree.splitlines() if line.strip()}
    if core_names != {"recast-nav", "glam"}:
        raise SystemExit(f"The default core must depend only on glam; review additions: {sorted(core_names)}")
    return {
        "target": target,
        "locked_registry_packages": registry_packages,
        "local_packages": sorted(local_ids),
        "core_normal_dependency_graph": sorted(core_names),
        "licenses": sorted({p["license"] or "UNDECLARED" for p in packages.values()}),
        "unsafe_policy": "forbidden in local Rust crates; external implementations are separate",
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--offline", action="store_true")
    parser.add_argument("--target")
    parser.add_argument("--report", type=Path)
    arguments = parser.parse_args()
    report = check(arguments.offline, arguments.target)
    payload = json.dumps(report, indent=2) + "\n"
    if arguments.report:
        arguments.report.write_text(payload)
    print(payload, end="")
