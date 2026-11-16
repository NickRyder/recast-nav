# Validation

The reference is pinned RecastNavigation 1.6.0. Rust tests launch external C++
oracle executables and compare logical outputs; the Rust runtime does not link
to those executables. The reference and its original licenses are included in
the source distribution.

The repository pins Rust 1.97.1 for development and declares Rust 1.95 as its
minimum version. Earlier local validation targets macOS arm64; the separate
[Windows report](windows-validation.md) records native Windows/MSVC-target results.

## Automated platform checks

The [Conformance workflow](https://github.com/NickRyder/recast-nav/actions/workflows/conformance.yml)
runs native differential tests on macOS arm64 and Linux x86_64 with separate
and explicit FMA arithmetic. Each job runs the core and Bevy test corpus,
strict Clippy, documentation, dependency checks and the headless Bevy example.
Linux additionally runs optimized `float_comparison`, `detour_common`,
`detour_data`, `recast_utilities` and `crowd` targets. These hosted native runs
use the default 32-bit reference layout; they do not replace the separate
64-bit reference checks described below.

A Windows x86_64/MSVC job runs Rust-only core and Bevy tests and both public
examples. It does not enable `cpp-oracle` or `native-bench`. Native Windows C++
differential suites passed locally in all eight debug/release, separate/FMA and
32/64-bit-reference configurations; the [Windows report](windows-validation.md)
records those results separately from CI. The dependency-audit job checks the
committed lockfile against the RustSec advisory database.

CI results identify the exact tested commit. Check the workflow result for the
revision and feature set you intend to use rather than treating a configured
job as execution evidence. Neither application builds nor a successful compile
substitute for native differential tests.

Float comparisons follow the [conformance policy](conformance.md). Rounding
explicitly quiets signaling NaNs while retaining their sign and payload.
Only explicitly identified arithmetic outputs permit differing quiet-NaN
representations; input copies, serialized values, references, statuses and
non-NaN float bits retain their exact comparisons.

## Run the checks

Ordinary Rust builds need no C++ compiler. Enable `cpp-oracle` to build the
native test drivers; Clang is required for the FMA comparison policy.

```sh
python3 scripts/check_vendor.py
python3 scripts/check_dependencies.py
cargo fmt --all -- --check
CXX=clang++ cargo test --locked -p recast-nav -p bevy-recast-nav --features recast-nav/cpp-oracle
CXX=clang++ cargo test --locked -p recast-nav -p bevy-recast-nav --features recast-nav/cpp-oracle,recast-nav/fma
```

To check optimized behavior, add `--release`. Add `recast-nav/polyref64` to either
command to exercise the 64-bit reference layout. This gives four arithmetic and
reference configurations. Test the minimum supported Rust version separately
when changing dependencies or using new language/library features.

### Native Windows C++ setup

Use an x64 Visual Studio Developer PowerShell with the C++ tools and Windows SDK
available. Set `CXX` to a native Windows **GNU-style `clang++` driver** targeting
`x86_64-pc-windows-msvc`, rather than `cl.exe` or `clang-cl.exe`. For example:

```powershell
$env:CXX = 'C:\Program Files\LLVM\bin\clang++.exe'
& $env:CXX --version
cargo +1.97.1 test --locked --no-fail-fast -p recast-nav -p bevy-recast-nav --features recast-nav/cpp-oracle
```

The developer shell supplies the MSVC/SDK headers and libraries, including
`libcpmt.lib`. The runner selects C++14 for MSVC-targeting builds because current
MSVC standard-library headers require it, and publishes explicit `.exe` paths.
The vendored C++ reference remains unchanged. The existing separate/FMA flags
and `-O0` native-driver policy apply on Windows too; `--release` optimizes Rust,
not the reference drivers. Repeat with `recast-nav/fma`, `recast-nav/polyref64`,
and both together, in debug and release. FMA requires host hardware support.

Run these commands with Windows Cargo and execute the Windows oracle processes.
Using WSL to fetch source is compatible with this procedure; Linux oracle runs
or successful Windows compilation alone do not establish Windows conformance.
Keep raw local logs under ignored `target/windows-validation/` and remove local
paths, account details and credentials before sharing excerpts.

## Coverage

| Area | Test targets |
| --- | --- |
| Generation stages, partitioning, configuration | `conformance`, `recast_utilities` |
| Packed fields, callbacks, numeric limits, span storage | `recast_utilities` |
| Tile data, ownership, links, state, reference lifetimes | `detour_mesh`, `detour_data` |
| Geometry/math helpers and node pools/queues | `detour_common`, `detour_nodes` |
| Complete runtime builds, queries, obstacle updates | `runtime_conformance` |
| TileCache builders, codecs, obstacle and tile lifecycle | `tile_cache`, `tile_cache_object` |
| Crowd updates, path queues, avoidance and boundaries | `crowd` |
| Input validation and error chains | `validation` |
| Bevy resource integration and failed rebuild retention | `bevy-recast-nav` library tests |
| Public examples | README doctests and runnable examples |

The [coverage ledger](coverage.md) and [conformance policy](conformance.md)
describe tested domains and checked adaptations in detail. The
[public API inventory](public-api.md) records declaration coverage separately.
None of these establishes every-input equivalence, C ABI compatibility, or a
performance guarantee. Measure performance separately using the
[benchmark tools](performance.md).
