# recast-nav

Build navigation meshes and find paths through 3D environments in Rust.
`recast-nav` implements the [RecastNavigation 1.6.0](https://github.com/recastnavigation/recastnavigation/tree/v1.6.0)
libraries, including tiled navigation, dynamic obstacles, and crowd simulation.
The core is independent of any game engine and has no C++ runtime dependency.

## Capabilities

| Library | What it does |
| --- | --- |
| Recast | Turns triangle geometry into polygon navigation meshes, with watershed, monotone, or layer partitioning and optional detail meshes. |
| Detour | Loads and links navigation tiles; provides A* and sliced searches, straight paths, raycasts, nearest-point queries, and off-mesh connections. |
| TileCache | Rebuilds affected navigation tiles as cylinder, box, and oriented-box obstacles are added or removed. |
| Crowd | Manages agents, path corridors, local steering, separation, obstacle avoidance, and off-mesh traversal. |

Use the `runtime::Navigation` facade for building a navigable world and managing
keyed obstacles. Lower-level modules expose the generation stages, tile formats,
queries, cache, and crowd algorithms separately. Geometry extraction, physics,
rendering, and application scheduling remain the caller's responsibility.

## Getting started

The crates are available from Git and are not published to crates.io. Rust 1.95
or newer is required; development uses the pinned Rust 1.97.1 toolchain.

After checking out the source, run the example:

```sh
cargo run --locked -p recast-nav --example pathfinding
```

To use the core from another project, pin the source release:

```toml
[dependencies]
recast-nav = { git = "https://github.com/NickRyder/recast-nav", tag = "v0.1.0" }
```

No system Recast installation, navigation-specific CMake setup, or generated
bindings are needed. The tag identifies a source release; the crates are not
published to crates.io.

Build a navigation mesh from an upward-facing floor, then request a complete
route across it:

```rust
use recast_nav::{TriangleMesh, runtime::{Navigation, NavigationSettings}};

fn main() -> Result<(), recast_nav::BuildError> {
    let mesh = TriangleMesh {
        vertices: vec![
            [0., 0., 0.], [0., 0., 16.],
            [16., 0., 16.], [16., 0., 0.],
        ],
        triangles: vec![[0, 1, 2], [0, 2, 3]],
    };
    let nav = Navigation::build(&mesh, NavigationSettings::default())?;
    let path = nav.path([2., 0., 2.], [14., 0., 14.], false)?;
    assert!(path.is_some_and(|path| path.reaches_goal));
    Ok(())
}
```

Coordinates are Y-up, with XZ as the ground plane. Geometry, cell dimensions, and
agent dimensions must use consistent world units. See the [usage guide](docs/usage.md)
for configuration, obstacle clearance, partial paths, and update scheduling.

## Bevy integration

The separate `bevy-recast-nav` package provides a Bevy 0.20 plugin and a
`NavigationWorld` resource. It supports headless use and preserves the existing
navigation world when a rebuild fails. The core does not depend on Bevy.

Add the adapter using the same source release:

```toml
[dependencies]
bevy-recast-nav = { git = "https://github.com/NickRyder/recast-nav", tag = "v0.1.0" }
```

To run its example from this checkout:

```sh
cargo run --locked -p bevy-recast-nav --example headless
```

## Features and dependencies

The default core's only external runtime dependency is `glam`. The crates forbid
handwritten unsafe Rust; dependencies and the standard library may use unsafe
internally. See the [dependency policy](docs/dependencies.md).

| Core feature | Purpose |
| --- | --- |
| `serialize` | Serde serialization for generation data. |
| `tracing` | Optional diagnostic events. |
| `libm` | Alternative generation math backend. |
| `fma` | Explicit fused multiply-add arithmetic. |
| `polyref64` | 64-bit Detour polygon references instead of the default 32-bit layout. |

The bundled, unmodified C++ source is the reference for the opt-in `cpp-oracle`
tests and `native-bench` tools. Ordinary builds do not compile or link it.
Arithmetic policy and reference width must match when exchanging native data or
comparing implementations.

## Compatibility and validation

The Rust implementation uses ownership and error handling in place of native
pointer and allocation APIs. It is not a C ABI wrapper.
The tests compare ordered geometry, float bits, statuses, callbacks, handle
lifetimes, and serialized data against the pinned C++ reference. Float comparisons
follow the [conformance policy](docs/conformance.md), including narrowly scoped
exceptions for arithmetic-produced quiet NaN representations. Undefined native
input domains use documented checked Rust contracts.

The [conformance workflow](https://github.com/NickRyder/recast-nav/actions/workflows/conformance.yml)
checks the library independently of any application:

| Platform | Automated coverage |
| --- | --- |
| macOS arm64 | Native differential tests with separate and FMA arithmetic, optional-feature checks, and the headless Bevy example. |
| Linux x86_64 | The same native differential tests, plus focused optimized numeric tests. |
| Windows x86_64/MSVC | Rust-only core/Bevy tests and pathfinding/headless examples. Native C++ differential tests are not run in CI. |

Local native Windows C++ differential tests passed all eight combinations of
debug/release, separate/FMA arithmetic and 32/64-bit polygon references. The
[native Windows validation report](docs/windows-validation.md) records the
compiler requirements, tested revisions and evidence limits.
See the [validation notes](docs/validation.md) for configurations and evidence
limits. Test results do not imply every-input or every-configuration equivalence.
Performance comparisons require measurements of the source and workload being
used.

- [API and behavior guide](docs/usage.md)
- [Feature coverage and qualifications](docs/coverage.md)
- [Conformance tests](docs/validation.md)
- [Benchmark methodology](docs/performance.md)
- [Contributing](CONTRIBUTING.md)

## License

Original work is available under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE).
Integrated translations also carry [Zlib](LICENSE-ZLIB) and
[BSD-3-Clause](LICENSE-BSD-3-Clause) obligations. Original authorship and license
notices are retained; see [third-party provenance](THIRD_PARTY.md).
