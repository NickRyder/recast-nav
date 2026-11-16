# Scope

The target is a faithful Rust port of RecastNavigation 1.6.0, commit
`6dc1667f580357e8a2154c28b7867bea7e8ad3a7`. The scoped operational library port
is implemented; API coverage and differential evidence remain distinct claims.
The demo/editor is outside the library scope. The detailed [coverage ledger](coverage.md)
tracks implementation, exposed APIs, configurations and differential evidence separately.

The primary local validation platform is macOS arm64. Separate/FMA arithmetic
and 32/64-bit references are supported configurations. CI is configured for
macOS and Linux; other platforms require their own validation.

| Upstream library | Implemented | Qualifications / further evidence |
| --- | --- | --- |
| Recast | Indexed 16/32-bit and nonindexed triangles; direct span insertion/counting; slope classification and triangle clearing; rasterization and filters; compact heightfields; erosion; area marking and median filtering; polygon offsets; heightfield layers; distance fields; watershed, monotone and layer regions; contours, polygon and detail meshes; mesh copy/merging | Broader storage/arithmetic/parameter-domain evidence |
| Detour | Native v7 tiles and runtime load/export/removal/state; detail/BV data and off-mesh connections; native 32/64-bit refs and cross-tile links; persistent A*/sliced/any-angle searches and node state; point/polygon/custom-filter queries; funnel crossing options, costed raycasts, surface movement, Dijkstra searches, neighborhoods, walls and random-point queries | Broader input/configuration/platform evidence |
| DetourTileCache | Layer regions, contours and polygons; raw AABB/cylinder/oriented-box marking; streaming cache pools and salted handles; deferred updates for all three shapes; native v1 compressed-layer format/endian conversion; compressor/mesh-process extensions | Broader input-domain evidence; native allocation machinery is replaced by Rust ownership |
| DetourCrowd | Full agent lifecycle/configuration and deterministic update: requests/replanning, corridors, neighbor ordering, steering/separation, avoidance/debug data, integration/collisions and off-mesh animation; standalone components and pinned sources | Broader operation-domain and platform evidence; local optimized performance/allocation results are recorded |
| DebugUtils | Test snapshots and debug polygon data only | Upstream drawing/dump APIs excluded |

`recast-nav` owns the engine-independent algorithms. `bevy-recast-nav` is an
optional adapter; it initializes a navigation resource and atomically replaces
it after a successful build. Rendering, geometry extraction, physics, steering,
and application scheduling remain caller responsibilities.

## Faithfulness

Preserve upstream algorithm order, limits, status flags, handle lifetimes, and
geometry. Compare raw logical output against the pinned C++ reference. Do not
add shortcuts or application-specific optimizations to make a scenario pass.
Any reference update requires a reviewed provenance change and renewed tests.

Rust ownership and data structures replace C++ allocation/pointer machinery;
this does not imply ABI, binary-format, memory-use, or performance equivalence.
The target excludes reproducing undefined behavior and uninitialized memory.
Known ordering/platform qualifications are recorded in `THIRD_PARTY.md` and
`docs/conformance.md`.

The `runtime` module adds convenience policies: keyed obstacle maps, update
revisions, complete/partial paths, proxy-goal projection, and agent-start
projection. Those policies are separate from the lower-level algorithm port.
Their behavior and current limits are documented in `docs/usage.md`.

## Ongoing maintenance

1. Maintain the source-anchored public API map and expand Recast/cache input-domain evidence.
2. Expand randomized operation coverage and minimize any discrepancies.
3. Maintain macOS arm64 toolchain/configuration validation and matched performance/allocation evidence.

Each feature needs a matching C++ oracle path and focused regression coverage.
A checked-in implementation or a passing small corpus alone is insufficient to
claim full upstream equivalence.
