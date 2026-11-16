# Dependencies and unsafe-code policy

## Local Rust code

`recast-nav` and the thin `bevy-recast-nav` adapter forbid unsafe Rust at their
crate roots. Workspace Cargo lint policies also cover tests, examples and build
scripts. Generation code is integrated directly into the core package; there is
no separate vendored Rust backend or backend path dependency.

No unsafe Rust source was found in those packages during the cleanup audit.
The core runtime uses Rust APIs and has no dependency on the C++ oracle. Native
C++ and allocator-interposition code are confined to opt-in external test and
benchmark executables.

This is a guarantee about local source, not a claim that Rust's standard library
or third-party crates have no unsafe implementation code. `glam` and Bevy use
unsafe internally behind their APIs. A whole-tree unsafe-free policy
would require different dependencies and a separate integration decision.

## Dependency boundaries

The default core depends only on `glam`, with no other normal transitive
dependencies. Span keys, integer flags and errors use standard Rust instead of
`slotmap`, `bitflags` and `thiserror`. Key and span storage sizes remain part of
the allocation contract. The policy check rejects additional default runtime
dependencies unless this boundary is deliberately reviewed.

Optional `serde`, `tracing` and `libm` features are engine-independent and add
dependencies only when selected. They retain serialization, tracing integration
and alternative math functionality. `serde_json` is test tooling; `cc` is enabled
only for native test and benchmark builds. Bevy app/ECS dependencies belong to
the separate headless adapter, with default features disabled. Rendering and
physics are not required.

The `libm` feature selects alternative generation math implementations; compiling
it does not establish bitwise C++ parity for that arithmetic configuration.

Commit `Cargo.lock` and use `--locked` for verification. Registry package versions
and SHA-256 checksums are locked; compatible manifest ranges permit reviewed
patch updates. The integrated generation source retains its upstream revision,
alterations and licenses in
[third-party provenance](../THIRD_PARTY.md) and `vendor/origins.json`.

Only crates.io and the two known workspace packages are allowed by the dependency
policy check. Adding Git dependencies, another registry, an external path package
or an unreviewed license expression requires an explicit policy review. Declared
dependency MSRVs must not exceed Rust 1.95. The check also protects the core's
normal dependency graph from Bevy and native build-tool dependencies.

## Verification and updates

```sh
python3 scripts/check_dependencies.py
python3 scripts/check_vendor.py
cargo audit --deny warnings --file Cargo.lock
```

Use `--offline` on the policy check when the target dependencies are cached.
Advisory checks require the separate `cargo-audit` tool; CI pins its tool version
and fetches the current RustSec database. A clean audit means no matching known
advisories at that database revision, not a general safety proof. Review update
release notes, MSRV, licenses, enabled features and API changes; preserve native
behavior and repeat affected differential checks after updates.

The 2026-11-14 audit found no advisories or warnings in the locked graph. License
metadata in the macOS graph consists of permissive MIT, Apache-2.0, Zlib, BSD,
Unicode and Unlicense alternatives. Local license texts and altered-source
notices remain part of source distribution. The deployment target's graph and
current advisories should be checked again before release.
