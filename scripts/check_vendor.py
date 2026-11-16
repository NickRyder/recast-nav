"""Verify the pinned C++ reference and account for every Rust source alteration."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parent.parent
vendored_rust = sorted(path.relative_to(root).as_posix() for path in (root / "vendor").rglob("*.rs"))
if vendored_rust:
    raise SystemExit(f"Rust source belongs in the core package, not vendor: {vendored_rust}")
origins = json.loads((root / "vendor/origins.json").read_text())
expected = origins["cpp"]["sha256"]
actual = {
    path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
    for path in (root / "vendor/recastnavigation").rglob("*")
    if path.is_file()
}
if actual != expected:
    raise SystemExit("C++ reference differs from its pinned source manifest")

expected = origins["apple_libc"]["sha256"]
actual = {
    path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
    for path in (root / "vendor/apple-libc").rglob("*")
    if path.is_file()
}
if actual != expected:
    raise SystemExit("Apple libc sort sources differ from their pinned source manifest")

rust_root = root / origins["rust"]["source_root"]
original = origins["rust"]["original_sha256"]
actual = {
    name: hashlib.sha256((rust_root / name).read_bytes()).hexdigest()
    for name in origins["rust"]["source_files"]
}
changed = {name for name in original.keys() | actual.keys() if original.get(name) != actual.get(name)}
if changed != set(origins["rust"]["modified_source_files"]):
    raise SystemExit(f"Unrecorded Rust source alterations: {sorted(changed)}")
print("Pinned C++ and libc sources verified; Rust alterations match the provenance record.")
