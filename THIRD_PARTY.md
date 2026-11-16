# Third-party code and alterations

## Reused Rust generation

The integrated generation modules under `crates/recast-nav/src` originate from
[janhohenheim/rerecast](https://github.com/janhohenheim/rerecast), commit
`1d5abc74d44beefbdfa13b2a99607276c9dd0434`, whose core manifest identifies itself
as 0.5.0. The commit is the actual snapshot identity. Author: Jan Hohenheim and
upstream contributors. Original MIT/Apache-2.0 license texts are retained.

The generation code is part of the `recast-nav` package, not a vendored Rust
crate or external backend dependency. Original license texts are retained in
`licenses/generation-*`; upstream identity and the integrated source-file list
are recorded in `vendor/origins.json`. Original notices and authorship remain
applicable after integration.

The package uses std, forbids unsafe Rust, and directly declares its generation
dependencies. Serialization, tracing and alternative libm operations remain
optional engine-independent features. Backend-only no_std shims and Bevy
reflection derives are omitted; engine integration stays in the thin adapter.
Default arithmetic implementations and generation algorithms are retained.
Source changes:

- `span.rs`, `region.rs`, `contours.rs` and generation error types: standard Rust
  replaces the slotmap key, bitflags and thiserror helpers. Span-key generations,
  compact pool storage, flag bits and error context/source chains are retained.

- `pre_filter.rs`, `compact_heightfield.rs`, `heightfield.rs`, `rasterize.rs`,
  `config.rs`, `lib.rs`: native signed height/climb parameters, exact ledge
  comparisons and checked arithmetic failures; permit compact neighbor layer 62
  and promote span-top additions before comparing headroom.
- `erosion.rs`, `config.rs`, `lib.rs`: signed native erosion radii and wrapped
  byte thresholds; checked signed multiplication/allocation failures before mutation.
- `config.rs`: documentation wording, 32-bit region/polygon budgets, and widened
  region-size squaring in the Rust convenience builder. Region-area thresholds
  retain signed native comparisons, including negative values.
- `watershed_build_regions.rs`: context entry points and real expansion/flood/filter
  scopes, explicit watershed timer lifetime, native overflow/overlap diagnostics
  and filter-phase metadata assignment.
- `watershed_build_regions.rs`, `watershed_distance_field.rs`, `contours.rs`:
  preserve border-neighbor conflict scanning, promoted distance comparisons,
  wrapped initial watershed levels, and exact 39999-iteration contour cutoffs.
- Grid fields/coordinates in `heightfield.rs`, `compact_heightfield.rs`,
  `config.rs`, `contours.rs`, `watershed_build_regions.rs`, `detail_mesh.rs`,
  `erosion.rs`, `mark_convex_poly_area.rs`, `watershed_distance_field.rs`, `math.rs`
  and `poly_mesh.rs` use native signed 32-bit values. Contours remain signed
  through triangulation before explicit 16-bit mesh welding. Detail patches and
  sampling indices retain native signed ranges; preserve lower-bound checks.
  Heightfield creation checks native conversion/allocation domains and signed
  flat indexing preserves defined row crossings.
- `pre_filter.rs`, `rasterize.rs`, `watershed_distance_field.rs`: context timer
  entry points for solid filters/rasterization and actual nested distance/blur
  phases, with buffers allocated before their computation timers.
- `poly_mesh.rs`, `detail_mesh.rs`: native diagnostic callbacks and construction
  timers, signed triangulation outcomes, and a checked error for nonempty detail
  meshes whose disabled edge sampling leaves native hull storage uninitialized.
  Empty zero-width meshes retain native counts, iteration and detail early return.
- `contours.rs`, `contour_holes.rs`: contour trace/simplification timers and native
  capacity/outline/bridge diagnostics; native Apple sort for equal-key candidates.
- `context.rs`, `lib.rs`, `compact_heightfield.rs`, `erosion.rs`: native optional
  diagnostic hooks, timer gates/scopes and bounded log formatting; compact
  connection overflow logs but succeeds, with native linear column scans and
  linked-only counts, checked allocation/storage errors.
- `compact_cell.rs`, `compact_span.rs`: native 24-bit index/connection assignment,
  8-bit count wrapping, height-byte preservation and checked connection shifts.
- `span.rs`: C++ default area ID 63 and 13-bit solid span maximum 8191.
- `contour_holes.rs`, `lib.rs`, `contours.rs`: missing C++ contour-hole merging;
  signed native edge lengths and checked squared-length overflow. Contour building
  now returns `ContourError` for native undefined arithmetic domains.
- `rasterize.rs`: share indexed batching over borrowed triangle indices and aligned
  vertices, preserving the single native batch timer; specialize clipping by axis
  without changing arithmetic; preserve native ordered clamping with reversed
  limits at the packed solid-span ceiling; check native float-to-int conversion domains.
- `watershed_build_regions.rs`: retain bounded per-column layer-region scratch
  and iterate floor prefixes directly instead of cloning them; reuse native-width
  previous-row counters within each monotone/layer partition call.
- `detail_mesh.rs`: compact bounded private Delaunay edge indices and retain
  face scratch capacity within one detail build, preserving reset/removal order;
  match native fused segment-distance and barycentric expressions in FMA mode.
- `math.rs`, `poly_mesh.rs`: use bounded circular-index steps instead of integer
  remainder in polygon scoring/merging, preserving traversal and tie order;
  specialize the six-vertex scoring width while keeping other widths general.
- `poly_mesh.rs`: preserve signed squared lengths rather than truncating to `u16`;
  use a 32-bit polygon-width budget and the native Recast `0xfffe` contour-vertex cutoff.
  Repair cyclic removal edges, initialized edge counts, hole assembly and vertex-zero
  removal; retain partial triangulation output and logical region counts. A separate
  TileCache mode uses strict triangulation, native removal-buffer limits and actual
  wrapped ushort vertex indices and triangulation flags, independently of the
  native integer count, instead of the Recast summed-contour cutoff; check
  nonterminating wrapped vertex chains;
  expose initialized partial mesh output after native removal-buffer failure.
- `watershed_build_regions.rs`: add translated monotone/layer partitioning and
  layer merging/filtering, sharing the identical sweep algorithm. Preserve native
  sweep capacity with a checked error instead of an out-of-bounds write. Widen
  the region-record count before adding one, matching C++ integer promotion;
  preserve 32-bit nonnegative region-area budgets.
- `lib.rs`: also export region-building, polygon-mesh and rasterization errors,
  and span-insertion types.
- `heightfield.rs`: expose span insertion, preserve native 13-bit height and
  6-bit area fields, and use the signed native flag-merge threshold.
- `rasterize.rs`: expose a signed flag-merge threshold, preserve native optional
  fused clipping-plane coordinates, clear both output counts when clipping an empty polygon;
  optional fused interpolation matching the native C++ policy.
- `compact_heightfield.rs` and `detail_mesh.rs`: optional fused metadata/edge
  interpolation matching the reference policy.

Original hashes and the complete altered-source filename set are recorded in
`vendor/origins.json`. Hole merging is an altered translation of the zlib-licensed
C++ algorithm and retains that notice. Its predicates use widened integers and
platform-specific equal-distance ordering; C++ `qsort` equal-key ordering is not specified
across platforms. The translation uses endpoint equality to exclude incident
edges instead of repeating C++ 1.6.0's potentially invalid `diags[i]` access.
Undefined behavior is not reproduced.

## C++ reference and translated runtime

`vendor/recastnavigation/{Recast,Detour,DetourTileCache,DetourCrowd}` and `License.txt` are
unmodified files from
[recastnavigation/recastnavigation](https://github.com/recastnavigation/recastnavigation),
v1.6.0, commit `6dc1667f580357e8a2154c28b7867bea7e8ad3a7`. Copyright (c)
2009–2010 Mikko Mononen, under zlib. Source headers and license text are retained.

`crates/recast-nav/src/{layers,cache,common,nav,nav_data,nodes,query,runtime,recast}.rs` contain altered Rust
translations of the relevant layer, tile-cache builder/lifecycle, native tile
loading/link pools/state, detail/off-mesh navigation/query algorithms, and Recast area-marking, median-filtering
polygon-offset utilities, polygon/detail mesh merging, and triangle-area and
bounds/grid input utilities. The production triangle classifier uses native
degree conversion and normalization operation order, including the documented
floating-point contraction policy. Their source headers identify the origin
and zlib notice. C++ allocator/raw-pointer layouts are replaced by Rust ownership;
logical ordering, reference salts, status flags, and geometry remain differential
comparison targets. Verified generation primitives are shared where their
logical output matches the separate C++ tile-cache builder.

`nav_data.rs` translates native Detour tile-data creation, packed records, BV-tree
construction and endian conversion. Byte offsets replace native pointer casts;
safe slice/size checks reject invalid memory domains. Native omissions and integer
packing are preserved, including the off-mesh user-ID endian omission and wrapped
16-bit detail offsets. Runtime loading owns decoded sections and preserves the
native link pool, free-list/list order, snapped vertices, state and reserved bytes
when exporting/removing tile data. Safe topology checks replace invalid access.
`nodes.rs` translates native node packing/hash chains and queue priority rules,
using checked handles instead of pointers. `query.rs` retains query allocation and
search records; meshes are supplied per operation instead of storing a raw pointer.
Sliced searches retain incremental state, use native any-angle raycasts, and
reverse parent/detached flags during full or partial finalization. Mesh/filter
lifetimes are caller contracts across operations. Owned output vectors preserve
the native extra final append after an exactly full detached ray without writing
beyond caller storage. Failed midpoint lookups retain initialized node positions.
Fresh query storage accepts native budgets 1–65535. Budgets below four retain
the zero-sized main hash table: independent/tiny-pool queries and same-polygon
shortcuts remain available, while main-table access is checked. Reinitialization
retains a larger allocated capacity as upstream. Rust memory
estimates are independent of native struct/allocator sizes.
`common.rs` translates ordered segment/polygon half-plane clipping, including
partial outputs, and unclamped XZ line-intersection parameters. Straight-path
translation includes ordered area/all portal crossings and partial recovery
using the original endpoint. Raycast translation retains native cost/height interpolation,
partial portals, output reset rules and zero-reference pointer contexts. Internal
decoded reference lookups retain defined stale-slot behavior with checked bounds;
public salt/reference validation stays separate. Cache conversion retains native
world-space agent header metadata.
Surface movement retains the separate 64-node pool, bounded FIFO and per-edge
neighbor limits, native filter order and wall-height interpolation. Scratch
vertices/neighbors use fixed Rust arrays instead of native stack pointers.
Circle/shape Dijkstra searches retain native portal predicates, priority/cost
updates and node records for later path extraction. A failed portal lookup is
checked instead of consuming uninitialized native local coordinates.
`crates/recast-nav/src/query/walls.rs` translates wall segments and distance
searches, preserving bounded interval insertion/sentinels, first-link eligibility,
native normal arithmetic and caller-initialized output retention. Neighborhood
queries preserve small-pool/FIFO limits and capacity-dependent overlap checks;
`common.rs` includes the separating-axis polygon-overlap predicate.
`crates/recast-nav/src/query/random.rs` translates tile/polygon reservoir
sampling, circle-constrained graph search and detail projection. Caller-owned
Rust random callbacks replace the C function pointer while preserving draw order.
`common.rs` includes convex triangle-fan sampling and its scratch-write behavior.
`cache.rs` includes byte-area AABB, cylinder and oriented-box marking with native
voxel bounds, half-cell expansion and auxiliary-rotation arithmetic. Safe grid
and float-conversion checks replace undefined native access/conversion domains.
Cache builders expose signed climb thresholds, native status details, empty layers,
zero polygon flags and independent contour-to-mesh construction. Simplification
uses the selected fused arithmetic policy. Independent region/contour/polygon
stages retain native initialized partial output and unused grid tails. Signed
contour coordinates preserve defined row-crossing flat indices, while checked
reads replace accesses outside the region buffer.
`cache_data.rs` translates native compressed-layer construction, decompression
and header endian conversion. Explicit offsets preserve the 56-byte header and
padding without Rust struct casts. A safe compressor trait replaces virtual
function pointers; owned contiguous grids preserve native zero initialization,
section offsets, callback status handling and ignored decompressed-size behavior.
`tile_cache.rs` translates the standalone streaming cache, hash/free pools, salted
handles, all obstacle queues/shapes, bounds queries and rebuild lifecycle. Owned
creation arrays and a Rust mesh-process trait replace borrowed native pointers;
retained pending/touched arrays, failure advancement and reinitialization queues
remain observable. Checked references and conversions exclude native undefined
accesses. Allocation failure leaves Rust reinitialization state intact.

`crates/recast-nav/src/crowd/proximity_grid.rs` is an altered translation of
`DetourProximityGrid.cpp`, preserving native hash/cell order, 16-bit storage,
capacity exhaustion and bounds. Hash multiplication explicitly wraps; safe
float conversion and chain traversal checks replace undefined conversion and
nonterminating wrapped chains. Rust owns the grid allocations and does not
reproduce native reinitialization leaks.

`crowd/local_boundary.rs` translates the bounded local wall cache, preserving
segment insertion/tie order, native neighborhood/wall query limits, validity
checks and initialized backing segments after reset. Optional Rust slots track
which native entries have been initialized; uninitialized values are not read.

`crowd/path_queue.rs` translates the eight-slot deferred search queue, shared
iteration budget, handle/expiry lifecycle and result status rules. Caller-owned
filter keys resolve to current borrowed filters instead of stored raw pointers.
The query module also exposes optional-filter initialization with native null
validation. Rust initializes fresh slot metadata and owns replacement buffers;
native uninitialized free-slot reads and partial allocation failures are excluded.

`crowd/path_corridor.rs` translates maintained path corridors and all three raw
merge helpers, with fixed owned storage and a separate used count. Native
visibility scaling, corner pruning, topology budgets, distinct movement-height
rules and off-mesh pruning/failure mutations are preserved. Logical capacity is
separate from checked physical slice bounds. Rust initializes fresh state rather
than exposing native uninitialized positions or allocation contents.

`crowd/avoidance.rs` translates both velocity samplers, circle/segment preparation,
penalty early-outs and the public avoidance debug-data API. Native float-operation
boundaries, sample order, retained obstacle records and debug array normalization
are preserved. Optional Rust debug entries track initialization after buffer
replacement. Checked adaptive pattern entries reject upstream uninitialized-slot
reads for even clamped divisions and its full-size even-pattern overflow, rather
than consuming uninitialized memory.

## Native sort ordering

`vendor/apple-libc/{qsort,heapsort}.c` are unmodified BSD-3-Clause files from
[Apple Libc](https://github.com/apple-oss-distributions/Libc), commit
`71bbe350ab79eef58113991d817ccc6165061a64`, paths `stdlib/FreeBSD/`. Hashes and
original notices are retained. `crates/recast-nav/src/native_sort.rs` is an altered
safe Rust translation shared by contour merging and Detour BV-tree construction
on macOS to preserve native observable equal-key order. It
replaces byte/pointer arithmetic with record swaps/copies; no libc calls or C
runtime code are linked. The full copyright, contributor notice and license are
also retained in `LICENSE-BSD-3-Clause` and `licenses/generation-LICENSE-BSD-3-Clause`.
On Windows/MSVC, an original safe record-swap implementation reproduces the
observed UCRT ordering: first-maximum selection for up to eight records,
median-of-three partitioning for larger ranges, and omission of the equal-key
partition suffix. It uses no CRT FFI or copied Windows SDK source. Existing
native record-order, complete tile-byte and lifecycle fixtures validate the
result, including the small-sort cutoff. Other targets use Rust stable sorting.
These policies target the tested CRT implementations, not a universal guarantee
about the unspecified equal-key order of C `qsort`.

The C++ code is compiled only into test executables, not runtime libraries.
Original project scaffolding is MIT OR Apache-2.0; combined core/reused-port
package metadata also includes Zlib and BSD-3-Clause. All license texts are included.
This project is independent of both upstream projects.
