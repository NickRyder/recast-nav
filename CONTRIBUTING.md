# Contributing

The target is faithful RecastNavigation behavior. Start with the pinned upstream
implementation, identify the supported input/configuration domain, and add a
focused C++ differential fixture before changing an algorithm. Preserve ordering,
status flags, capacities and handle semantics. Do not weaken a comparison or
alter the reference to make a mismatch disappear.

Keep the runtime entirely Rust, the core engine-independent, and convenience
policies separate from reference algorithms. Keep scope/status claims aligned
with tested functionality. Port new features incrementally; existing behavior
and the reference version must remain explicit.

Use the pinned Rust toolchain. The default core depends only on `glam`; optional
integrations must stay opt-in, and Bevy integration belongs in its separate
adapter. Keep local Rust free of handwritten unsafe code and update public
documentation when behavior or configuration changes.

Before submitting:

```sh
python3 scripts/check_vendor.py
python3 scripts/check_dependencies.py
cargo fmt --all -- --check
CXX=clang++ cargo clippy --locked -p recast-nav -p bevy-recast-nav --tests --examples --features recast-nav/cpp-oracle -- -D warnings
CXX=clang++ cargo test --locked -p recast-nav -p bevy-recast-nav --features recast-nav/cpp-oracle
CXX=clang++ cargo test --locked -p recast-nav -p bevy-recast-nav --features recast-nav/cpp-oracle,recast-nav/fma
```

Run focused Clippy under the FMA feature as well when modifying numeric code.
Preserve upstream license notices and update provenance for reused source changes.
See [scope](docs/scope.md), [conformance](docs/conformance.md), and
[third-party code](THIRD_PARTY.md).

See [release preparation](docs/releasing.md) for source and registry packaging.
Dependency updates also require `cargo audit --deny warnings --file Cargo.lock`;
see the [dependency policy](docs/dependencies.md). Use locked dependency versions
for build/test commands and keep the integrated generation source and its original notices together.
