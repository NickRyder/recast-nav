# Native Windows validation — 2026-11-15

## Revision and environment

The revisions below identify the original Windows validation runs, before the
repository history was reset. The `v0.1.0` source release retains the same
runtime, native harness and test sources; only documentation changed during the
reset. The original revision identifiers are historical and may not resolve in
the current repository.

- Baseline: `9167e4cc8d5706affedf80e2a66f62d7403f0240`.
- Tested implementation: `fd9e66b6c419e89d652b145e873d461451f5a3c6`, on
  `test/windows-conformance`. Subsequent report edits change documentation only.
- Unmodified C++ reference: RecastNavigation v1.6.0, commit
  `6dc1667f580357e8a2154c28b7867bea7e8ad3a7`, checked against its pinned manifest.
- Windows 10 Home 22H2, build 19045.6466, x64; FMA hardware support confirmed.
- Native Rust `1.97.1 (8bab26f4f 2026-07-14)`, host
  `x86_64-pc-windows-msvc`; Cargo `1.97.1 (c980f4866 2026-06-30)`;
  rustc LLVM 22.1.6, rustfmt 1.9.0, Clippy 0.1.97.
- Native Clang `22.1.0`, LLVM revision
  `4434dabb69916856b824f68a64b029c67175e532`, GNU-style `clang++.exe`, target
  `x86_64-pc-windows-msvc`; MSVC 14.42.34433 and Windows SDK 10.0.22621.0,
  initialized with Visual Studio 2022 Community's x64 developer environment.
- WSL supplied GitHub SSH/CLI authentication for source retrieval and publishing
  only. Every Cargo check, Rust test, example and C++ oracle ran on Windows.

Read `README.md`, `CONTRIBUTING.md`, `docs/validation.md`,
`docs/conformance.md` and the core build script before validation. No global
installation, CI, repository visibility, or upstream-reference changes were made.

## Blockers and fixes

1. The default Windows C++ driver is MSVC-style and intentionally rejected by
   `build.rs`. Selecting existing native GNU-style Clang retains the required
   floating-point switches and FMA compiler policy. `clang-cl.exe` remains
   unsupported.
2. C++11 fails inside the installed MSVC standard-library headers. The build
   harness now selects C++14 for MSVC targets, for both oracle and optional
   benchmark compilation. Other targets retain C++11. Executable paths explicitly
   include `.exe` on Windows.
3. Clang linking needs the developer environment's library paths; without them,
   `libcpmt.lib` is not found. The test launcher initialized this environment
   locally, without changing the installation or global environment.
4. The provenance script compared Windows backslash paths with slash-delimited
   manifest keys. It now uses POSIX keys while still hashing exact bytes. All
   49 pinned RecastNavigation files already matched the manifest before this
   fix; neither the reference nor its manifest was edited.
5. Actual oracle execution exposed UCRT equal-key sorting differences: the first
   complete separate/32-bit run failed four BV/data tests, direct record order,
   and a seeded TileCache lifecycle test. A safe Windows/MSVC record-swap sort
   now follows the native first-maximum selection and partition tie choices.
   The runtime remains Rust, with no unsafe code or CRT FFI.
6. The split-axis regression also hardcoded stable equal-key order. Its second
   polygon now has a distinct X key: the correct unsigned-short X extent yields
   `[0,1]`, while incorrect signed-extent selection yields `[1,0]` on every CRT.
   Complete native byte comparison remains. Direct sort coverage was expanded
   around small-array and partition boundaries; no comparison was relaxed.

No numeric algorithm, float tolerance, status/reference comparison, callback
ordering comparison, or existing field-local NaN policy was changed.

## Commands

From an x64 Visual Studio Developer PowerShell in the repository, select the
installed Clang explicitly:

```powershell
$env:CXX = 'C:\Program Files\LLVM\bin\clang++.exe'
python scripts/check_vendor.py
python scripts/check_dependencies.py
cargo +1.97.1 fmt --all -- --check
cargo +1.97.1 test --locked --no-fail-fast -p recast-nav -p bevy-recast-nav
cargo +1.97.1 clippy --locked -p recast-nav -p bevy-recast-nav --tests --examples --features recast-nav/cpp-oracle -- -D warnings
cargo +1.97.1 clippy --locked -p recast-nav -p bevy-recast-nav --tests --examples --features recast-nav/cpp-oracle,recast-nav/fma -- -D warnings
cargo +1.97.1 clippy --locked -p recast-nav --lib --features serialize,tracing,libm,polyref64,fma -- -D warnings
cargo +1.97.1 check --locked -p recast-nav -p bevy-recast-nav --all-targets --all-features
cargo +1.97.1 run --locked -p recast-nav --example pathfinding
cargo +1.97.1 run --locked -p bevy-recast-nav --example headless
$env:RUSTDOCFLAGS = '-D warnings'
cargo +1.97.1 doc --locked -p recast-nav -p bevy-recast-nav --no-deps
```

The differential matrix uses the complete core and Bevy suites, including
doctests, without filters:

```powershell
$modes = @(
    'recast-nav/cpp-oracle',
    'recast-nav/cpp-oracle,recast-nav/fma',
    'recast-nav/cpp-oracle,recast-nav/polyref64',
    'recast-nav/cpp-oracle,recast-nav/polyref64,recast-nav/fma'
)
foreach ($features in $modes) {
    cargo +1.97.1 test --locked --no-fail-fast -p recast-nav -p bevy-recast-nav --features $features
    cargo +1.97.1 test --locked --no-fail-fast -p recast-nav -p bevy-recast-nav --features $features --release
}
```

Differential driver flags remain `-O0 -fno-fast-math`, with `-ffp-contract=off` for separate
arithmetic or `-ffp-contract=on -mfma` for FMA. `DT_POLYREF64=1` matches the Rust
reference-width feature. Release means optimized Rust; optimized C++ reference
drivers are outside this evidence, as specified by the existing conformance
policy. Optional all-feature compilation builds the native benchmark executable
but never executes it. The full performance benchmark suite was not run.

## Results

| Rust profile | Arithmetic | Polygon references | Full differential suite |
| --- | --- | --- | --- |
| Debug | Separate | 32-bit | Pass: 168 passed, 1 existing ignored doctest |
| Debug | FMA | 32-bit | Pass: 168 passed, 1 existing ignored doctest |
| Debug | Separate | 64-bit | Pass: 168 passed, 1 existing ignored doctest |
| Debug | FMA | 64-bit | Pass: 168 passed, 1 existing ignored doctest |
| Release | Separate | 32-bit | Pass: 168 passed, 1 existing ignored doctest |
| Release | FMA | 32-bit | Pass: 168 passed, 1 existing ignored doctest |
| Release | Separate | 64-bit | Pass: 168 passed, 1 existing ignored doctest |
| Release | FMA | 64-bit | Pass: 168 passed, 1 existing ignored doctest |

The tag's Rust-only baseline passed 26 tests with one existing ignored doctest.
Formatting, strict Clippy, optional-feature compilation, provenance and dependency
checks, and both public examples passed. Pathfinding printed two waypoints;
headless Bevy built four navigation polygons, returned two waypoints and reported
`reaches goal: true`.

The seven native drivers are `recast-oracle.exe`, `runtime-oracle.exe`,
`recast-utilities-oracle.exe`, `detour-mesh-oracle.exe`,
`detour-data-oracle.exe`, `tile-cache-oracle.exe`, and `crowd-oracle.exe`.
Their Windows PE/x64 headers were inspected, and live Windows processes for all
seven driver names were captured. The differential fixtures spawn these executable
paths, require successful process exits, parse their output, and compare it to
Rust. Passing driver-dependent suites therefore establish execution, not merely
compilation. The initial exact mismatches are additional evidence that the C++
comparison path was active.

## Logs and limits

Raw command logs, exit codes, elapsed times, differential snapshots and PE/hash
inventory remain locally under ignored `target/windows-validation/`. Sanitized
copies replace workspace/user paths, emails and credential patterns before
sharing. Raw logs and authentication configuration are not committed or attached
to the PR.

This is local Windows/MSVC-target evidence for the specified compiler, CRT,
features and finite corpus. Windows CI remains Rust-only. It does not establish
all-input equivalence, other Windows CRT/toolchain behavior, MinGW parity,
optimized native-reference equivalence, optional `libm` runtime parity, or a
performance guarantee.
