# Usage

## Geometry and units

`TriangleMesh` contains indexed, world-space triangles in a right-handed, Y-up
coordinate system. Use counterclockwise winding when viewed from the walkable
side. Bounds and coordinates must be finite; indices must reference vertices.
The caller chooses world units and supplies geometry from its own scene or
physics adapter. No renderer or physics engine is required.

`BuildConfig` uses voxel units for walkable height, climb, radius, border and
maximum edge length. Its bounds include tile padding. `NavigationSettings`
uses world units for agent dimensions and cell dimensions; `tile_voxels` is the
number of cells along a tile edge, not the number of application grid cells.
The XZ world size of a tile is `tile_voxels * cell_size`.

Generation converts height/climb/radius with `ceil`. The tile-cache climb
conversion truncates to voxel units, matching upstream. Fractional origins and
non-binary cell sizes are included in the differential corpus.

## Custom generation

`BuildConfig::partitioning` selects `RegionPartitioning::Watershed` (the default),
`Monotone` or `Layers`. The standard observed pipeline retains its distance-field
stage in every mode. Custom compact pipelines can call `build_regions_monotone`
or `build_layer_regions` without first building a distance field. Layer partitioning
uses the minimum region area; it does not use the merge-region-area setting.
`BuildConfig::contour_flags` selects solid-wall and/or area-edge tessellation.
Region-area budgets and `max_vertices_per_polygon` accept native nonnegative
signed-32-bit values, with a minimum polygon width of three. Recast can generate
polygons with more than six vertices; Detour retains its separate six-vertex tile
format. The polygon builder rejects 65,534 or more total contour vertices,
matching the upstream capacity check.
Both alternatives retain upstream's grid-sized sweep capacity and report a checked
error if a row exceeds it.

The degree-based `recast::mark_walkable_triangles` and
`clear_unwalkable_triangles` entry points preserve custom IDs on triangles outside
their respective tests. Marking uses a strict normal-Y threshold; clearing uses
the inclusive reverse comparison, preserving degenerate NaN normals as upstream.
`calc_bounds` preserves native bound bits and `calc_grid_size` uses native rounding.

`recast::add_span` inserts solid spans in voxel units; native 13-bit heights and
6-bit area IDs are retained. Flag merge thresholds are signed, and negative values
disable area-ID merging. `heightfield_span_count` counts non-null spans.
The `rasterize_triangle`, indexed `rasterize_triangles_u16` / `_u32`, and
nonindexed `rasterize_triangles` functions preserve input order and take one
area ID per triangle. Vertex indices must be valid.

The `recast` module exposes low-level heightfield/compact types and standalone
box, cylinder and convex area marking, median filtering and polygon offsets.
`merge_poly_meshes` welds vertices in upstream hash-chain order, retains outer
portals and rebuilds adjacency; inputs must use compatible grids and polygon
sizes. Use `recast::copy_poly_mesh` for the upstream mesh-copy operation: it sets
`max_polygons` to the live count and omits initialized spare storage. Rust
`Clone` preserves all fields, including `max_polygons` and the `unused_*` arrays.
`merge_detail_meshes` preserves local triangle indices and float bits while
rebasing submesh offsets, and skips `None` entries.
Area operations use world units and preserve null spans. Cylinder boundaries
exclude cell centers exactly on the radius. The median filter reads the original
field for an entire pass. `offset_poly` writes into a caller-provided buffer;
upstream requires one spare vertex slot and returns zero on exhaustion, retaining
any vertices already written. Use these operations explicitly in a custom
pipeline; the standard `build_tile` pipeline does not assign custom areas.

## Layers and references

`max_tiles` counts cached heightfield layers, not XZ grid positions. A bridge
above a lower floor can require multiple slots at the same position. Allocate
capacity for the expected layer count; exhausted capacity is a build error.
`NavigationSettings::max_polygons_per_tile` defaults to 4096 and is configurable.
The low-level constructor accepts the upstream 32-bit capacity domain: the sum
of the rounded-up tile and polygon bit counts must leave at least ten salt
bits. There is no separate 1024-tile guard. Non-power-of-two budgets reserve the
same reference bits as the next power of two.

The optional `polyref64` feature selects upstream's fixed 16 salt / 28 tile /
20 polygon bit layout for `PolyRef` and `TileRef`. This layout does not depend
on the configured budgets. Like upstream, 64-bit initialization does not reject
budgets beyond those fields; callers must keep actual indices representable.
Tile-cache and obstacle handles remain 32-bit in both modes. Tile replacement
invalidates old polygon references until the finite salt field eventually wraps.

## Native tile data

`nav_data::create` builds a Detour v7 tile from quantized vertices, Recast neighbor
codes, flags/areas and optional detail/off-mesh data. Bounds and walkable dimensions
use world units. Detail vertices are independently supplied: upstream uses them
for off-mesh start-height culling even without detail-mesh records. Ordinary polygons
use 3–6 vertices; initialized thin 0–2-width domains are also preserved
as described below. Source vertex count must be below 65,535. Evaluated array
reads and native signed buffer sizes are checked. The returned buffer includes native zero padding,
reserved links and the unused final BV node.

`nav_data::TileData::parse` provides checked section views without interpreting
topology or changing bytes. `bytes()` preserves the complete supplied buffer.
The producer's reference width must match the `polyref64` selection: native tile
headers do not encode this choice. Rust-built buffers are loaded by real C++
Detour in fixtures, and C++-built buffers are loaded by the Rust runtime.

`NavMesh::from_tile_data` initializes a single-tile mesh. For a tiled mesh, use
`add_tile_data` with zero for ordinary allocation or a former tile reference to
restore its free slot and salt. These methods copy native input into Rust-owned
storage and validate section ranges and structural linking requirements. Polygon
indices, BV nodes and detail metadata are retained without eager index scans:
linking and geometry queries interpret only the records they visit, including
native unsigned detail-offset wrapping. Loading therefore does not certify
every potential geometry read; records used by an operation must have initialized
backing. Ground polygons with zero edges perform no internal-link iterations;
loading retains them when the tile has initialized link storage. Rust bounds
checks still prevent out-of-buffer reads. Polygon type/count combinations are
not certified globally: internal linking skips off-mesh types before reading
their edge count. `PolygonView::vertex` addresses any of the six stored vertex
slots, including inactive slots; `vertices()` still iterates the active prefix.
Native off-mesh linking and endpoint queries use the first two stored slots
independently of the active count.
BV trees retain native traversal order. Off-mesh links
snap to the current mesh and honor direction flags across local/neighbor tiles.
`tile_data` (or `Tile::data`) exports the current native buffer; `remove_tile_data` returns that
buffer and releases the slot. `Tile::links(poly)` follows native list order through
the tile's link pool. Reserved/unused bytes and freed link contents are retained.

`closest_boundary` projects onto polygon edges without detail height; `poly_height`
uses imported detail triangles, with native edge fallback and off-mesh interpolation.
Height queries use XZ and accept a non-finite Y value. Other point queries require
finite coordinates. `off_mesh_endpoints` orients snapped endpoints by the previous
polygon reference; `off_mesh_connection` returns the original record and user ID.
Flags/area getters and setters retain native status behavior and area-bit masking.

`tile_state_size`, `store_tile_state` and `restore_tile_state` persist polygon
flags/areas separately from geometry. Storage writes into a caller-provided buffer
and retains its padding/trailing bytes. State is tied to the exact tile reference;
restoring the former reference with `last_ref` makes its state usable again.

For foreign-endian export call `swap_data_endian` then `swap_header_endian`;
import reverses that order. Data swapping requires a native header. These functions
preserve native omissions: link records, polygon first-link fields and off-mesh
user IDs are not swapped. Truncated buffers are rejected without partial writes.
BV-tree and contour-hole/diagonal tie ordering follows the pinned Apple libc algorithm on macOS and stable
ordering elsewhere; validation of other platform libc versions is outstanding.

## Obstacles and updates

The low-level `Filter::default()` includes every flag, excludes none and assigns
cost 1 to all 64 areas, matching Detour. A polygon with flags zero always fails;
an excluded flag takes precedence over an included flag. The `Navigation` facade
explicitly selects flag 1 for its generated walkable polygons.

Custom queries can implement `QueryFilter::pass_filter` and `cost`. Each
`FilterPolygon` contains the raw reference and owning tile/polygon; cost callbacks
receive previous/current/next contexts, using `None` for missing native pointers. The implementation
can replace both default flag eligibility and area-cost logic. Query methods accept
concrete filters or `&dyn QueryFilter`.

Context presence follows native pointer presence. Raycast callbacks can receive
a context with reference zero, including the initial previous polygon and a
rejected next candidate at a wall. Its tile/polygon fields remain meaningful.

`query_polygons` returns ordered references up to a chosen capacity, including zero;
additional matches set `BUFFER_TOO_SMALL`. Filtering continues after the output is
full. `query_polygons_with` invokes a callback in batches of at most 32 polygons,
flushing at each tile boundary. It preserves native filter/process call order,
including duplicate references produced by a stored trailing BV leaf.
`nearest_with_over` also returns the native over-polygon flag in `NearestPoint`.

`query::NavMeshQuery` retains the node pool/open queue between A* searches. Supply
the mesh and filter for each operation; Rust does not store an attached raw mesh
pointer. `new` accepts 4–65,535 nodes. `reinitialize` grows storage when needed and
otherwise clears and retains its larger capacity/hash size, including for a zero
request. `node_pool` exposes logical search records; `is_in_closed_list` reports
the last search. `is_valid_poly_ref` checks current mesh/filter eligibility. An
invalid or same-polygon path request retains the existing search records, as native.

Sliced searches use `init_sliced_find_path`, repeated `update_sliced_find_path`,
then `finalize_sliced_find_path` or `finalize_sliced_find_path_partial`. Updates
accept a signed iteration budget and an optional completed-iteration output.
`FINDPATH_ANY_ANGLE` enables radius-limited raycast shortcuts. Keep the same mesh
and filter throughout this lifecycle; tile changes and changes to that filter's
contents are observed by later steps. Operations that overwrite query storage,
including another search, reinitialization or direct pool mutation, invalidate an
active search. Finalization reverses retained node parent links and resets sliced
state. Invalid output budgets leave that state intact. The native regular append
sets `BUFFER_TOO_SMALL` even when the path exactly fits. A detached ray that fills
the budget exactly can also be followed by one final polygon, so the owned result
can contain `capacity + 1` references; no caller buffer is written past its bounds.

`straight_path_with_options` accepts `STRAIGHTPATH_AREA_CROSSINGS` or
`STRAIGHTPATH_ALL_CROSSINGS` to insert portal crossings along the funnel's straight
segments. When both bits are set, native area filtering takes precedence.
`straight_path` keeps the default funnel policy; `NavMeshQuery::straight_path`
accepts options explicitly. Points, flags and entered-polygon references retain
native order. Partial recovery clamps the requested endpoint to the last
reachable polygon. It can append a final point after crossing insertion fills
the budget, so that owned result can also contain `capacity + 1` points.

The `nodes` module exposes standalone pools and queues. Pool constructors take
1–65,535 nodes and a nonzero power-of-two hash size. Pool operations return
one-based handles; hash-chain slots use zero-based `u16` indices and `NULL_INDEX`.
Node parent/state/flag setters preserve native 24/2/3-bit packing. Pool clearing
resets buckets/count and retains record/next-link storage. Queues use handles from
the same pool and preserve native equal/unordered-cost priority comparisons.
Memory estimates describe Rust struct/vector storage rather than native ABI size.

`raycast` exposes the legacy no-cost cast; `raycast_with_options` accepts
`RAYCAST_USE_COSTS`, a previous reference and path capacity. `raycast_into` writes
into a reusable `RaycastHit`, preserving the native reset/partial-write rules.
The cast checks XZ geometry and returns `f32::MAX` when the endpoint is reached.
Costed casts correct intermediate edge heights and retain the supplied endpoint Y
in the final cost. A full path buffer sets `BUFFER_TOO_SMALL` while casting continues.
`common::intersect_segment_polygon_2d` exposes native clipping parameters and
entering/leaving edges, including partial outputs on a rejected intersection.
`common::intersect_segments_2d` returns native XZ line-intersection parameters;
the parameters can fall outside `[0, 1]` and are not clamped to segment bounds.

`NavMeshQuery::move_along_surface` returns a status and `SurfaceMove` containing
the reached position and ordered visited references. It uses a separate 64-node
pool and a 48-entry FIFO, preserving main-search records. Movement is constrained
to a circle around the requested segment and considers at most eight passing
neighbors per border edge. Filters are still called for additional candidates.
It checks XZ containment: a reached endpoint keeps its requested Y, while a wall
result interpolates the wall edge height. Use `poly_height` to project the result
onto the surface. Exact visited capacity sets `BUFFER_TOO_SMALL`; exhausting the
small search pool does not set `OUT_OF_NODES`. The `_into` form clears the visited
list and preserves the supplied position on invalid input.

`find_polygons_around_circle` and `find_polygons_around_shape` run Dijkstra searches
through connected portals. `DijkstraSearch::entries` stores each visited reference,
its parent and accumulated cost in native priority order. Searches continue after
the result buffer fills, including at zero capacity; retained main-pool records
can still be used with `path_from_dijkstra_search`. Path extraction accepts zero
capacity, preserves the pool and returns the prefix that fits. Geometry tests use
XZ while cost calculations use the circle center or shape's 3D vertex average.
Shape input follows native validation, which does not reject non-finite vertex Y.
A failed portal lookup returns a checked failure where native code would proceed
with uninitialized local coordinates; that undefined memory domain is excluded
from parity comparisons.

`find_local_neighborhood` uses the separate small pool, skips off-mesh neighbors
and returns ordered reference/parent pairs. It marks candidates visited before
testing projected overlaps. Native overlap tests consult only the references
stored in the bounded result, so changing capacity can change later traversal.
`common::overlap_polygons_2d` exposes the same separating-axis predicate and epsilon.
It returns `None` when exactly one polygon is empty, avoiding the native invalid
projection read.

`poly_wall_segments` returns ordered segment endpoints and neighboring references.
Pass `store_portals = false` for walls alone. Border processing preserves native
16-interval insertion and truncation, including overlapping/reversed segments and
omitted sentinels when the interval storage fills. `find_distance_to_wall` uses
the main pool, shrinks its search radius on each wall hit and returns `WallHit`.
Wall eligibility on a border checks only its first matching link. Hit height
comes from polygon edges; the normal includes the center's Y offset. The `_into`
form preserves all fields on invalid input. If no wall is found, it retains the
supplied position and computes the normal from that initialized position; callers
should use the returned distance to determine whether the search found a wall.

`find_random_point` and `find_random_point_around_circle` accept a caller-owned
`FnMut() -> f32` producing values in `[0, 1]`. Native draw order is preserved.
Global sampling selects a tile uniformly before filtering, then selects a ground
polygon by area; it can fail after selecting a tile with no eligible polygons.
Circle sampling searches connected polygons through portals and uses the main
pool. It samples the entire selected polygon, so the returned point can lie
outside the circle. Both methods project onto detail geometry and return a status
with `RandomPoint`; their `_into` forms retain supplied outputs on failure.
`common::random_point_in_convex_polygon` exposes triangle-fan sampling with caller
scratch areas. It preserves scratch entries 0/1 and the unused tail, including the
native area-floor selection behavior; two vertices form the native line case.

The cache-mesh `add_tile` helper takes world-space walkable height, radius and
climb. The runtime facade supplies all three agent settings so exported headers
and query policies use the native metadata.

`add_box` and `remove_obstacle` expose the reference's queued, salted-handle
behavior. `add_or_replace_box`, `remove_key`, and `retain_only` add keyed tracking.
An unchanged keyed box is a no-op. A replacement queues removal plus insertion.

Raw cache-layer marking uses `cache::mark_box_area`, `mark_cylinder_area` and
`mark_oriented_box_area`, each returning a native status and accepting an arbitrary
byte area ID. `mark_box` is the area-zero convenience. The layer's lower bounds and
cell sizes supply the native origin/scale. Cylinder marking expands its radius by
half a voxel. Oriented boxes accept native auxiliary rotation values directly;
`[0, 0.5]` is the identity, and the bounding approximation remains `1.41`.
These APIs check byte-sized layer dimensions, grid storage and native float-to-int
conversion domains before mutation. The `runtime::Navigation` convenience layer
queues axis-aligned boxes. Use `tile_cache::TileCache` for native streaming
lifecycles with cylinders, axis-aligned boxes and oriented boxes. Raw marking
functions mutate layer data directly.

`cache_data` builds and decompresses native TileCache v1 layer buffers using a
caller-owned `Compressor`. The 56-byte `Header` preserves its two padding bytes;
the payload contains the codec's compressed heights, areas and connections.
The format identifies neither the codec nor cell sizes. `Layer::heightfield`
accepts those cell sizes to expose decoded data to cache builders. `grids` and
`grids_mut` provide heights, areas, connections and regions in native pointer
order. Decompression starts with four zeroed grids, permits writes to the region
grid and ignores the reported output size, as native. Only a callback's failure
status bit aborts the operation; other status bits are discarded. Compression's
reported size must fit its provided buffer. `swap_header_endian` converts header
fields alone, preserving padding and the codec payload; import requires converting
foreign-endian headers before decompression. Checked slice/size guards replace
invalid native memory domains. `tile_cache::TileCache` exposes the native
streaming cache, queues, callbacks, salted pools and deferred rebuilding.

The upstream defaults are 64 requests, 64 pending tile updates, eight touched
tiles per obstacle, and 32 layers per tile lookup. The obstacle pool capacity is
independent of the request queue. Callers must batch changes or retry rejected
requests; do not discard errors. Large obstacle coverage is not automatically
expanded past the reference's touch limit.

Box marking uses the supplied bounds directly. Unlike static heightfield
erosion, it does not add travelling-agent clearance. If a box describes a
physical obstacle, the caller must reserve sufficient clearance for the moving
agent and its collision tolerance. The library does not choose an application
collision policy or inflate boxes implicitly.

`update` advances one tile-cache update. `update_until_up_to_date` drains all
pending work and can rebuild several layers synchronously. `revision` advances
after successful rebuilds; `requested_revision` tracks keyed request changes.
Queued changes must not be mistaken for already published geometry.

## Queries

Low-level `NavMesh` queries expose Detour references, status bits, corridors,
straight-path flags and explicit budgets. A returned polygon corridor is a query
result; `crowd::PathCorridor` provides persistent corridor maintenance and movement
feedback. Fresh node budgets are 1–65535. Budgets 1–3 produce the native zero-sized
main hash table, while same-polygon shortcuts, independent queries and tiny-pool
operations remain usable. Calls requiring the zero-sized table return checked
failures where native would access invalid storage. Path/waypoint buffer
exhaustion is reported through status flags.

The convenience `Navigation::path` projects endpoints with half-extents
`[1, 100, 1]`. Partial queries can search for a proxy goal with `[4, 100, 4]`.
These are world-unit policies, not universal actor/floor tolerances. Callers
requiring stricter projection should use `project_point` or the low-level
queries and check the displacement. `reaches_goal` describes reaching the
projected goal without truncation/exhaustion; a proxy remains partial.

`path_for_agent` projects the start near a keyed obstacle before querying. It
does not temporarily remove that obstacle or change the navigation mesh.
Navigation waypoints do not provide collision response, animation or locomotion.

## Bevy

Add `RecastNavigationPlugin` to initialize `NavigationWorld`. `rebuild` builds a
world synchronously and preserves the previous value if construction fails.
For background work, build a `Navigation` on a caller-owned task pool and insert
it with `replace`; navigation values are `Send + Sync`. Scoped generation jobs
share a process-wide four-worker ceiling and assemble results in tile order.

Run the complete headless example with:

```sh
cargo run -p bevy-recast-nav --example headless
```

## Streaming TileCache

`tile_cache::TileCache::new` accepts `tile_cache::Params`, a caller-owned
`cache_data::Compressor`, and an optional boxed `tile_cache::MeshProcess`.
Insert native v1 layer bytes with `add_tile`; `build_tile` or `build_tiles_at`
creates the corresponding tiles in a `nav::NavMesh`. Areas and flags initially
match the native cache builder, including zero polygon flags. A mesh processor
receives `nav_data::OwnedCreateParams` and can edit metadata and geometry or add
off-mesh connections before serialization.

Cylinder, AABB and oriented-box additions return 32-bit obstacle references.
`update(dt, &mut mesh)` returns `(status, up_to_date)` and processes at most one
queued tile rebuild. The time step is unused, as upstream. A failed rebuild is
still consumed and advances obstacle state. Removing a compressed layer does
not automatically remove its navigation tile. `FREE_DATA` suppresses returned
bytes on layer removal; Rust owns the allocation in either case.

Pool slices expose native slot order, salts, next indices and complete retained
obstacle arrays. Ref lookups may return an empty slot with a current salt.
Reinitialization clears pools and requests but retains queued tile references,
matching upstream. Rust does not reproduce leaked allocations or partially
initialized state after allocation failure. Invalid storage, invalid pool
capacities and undefined native float-to-int conversions return checked errors.

The cache builder also exposes independent `build_regions_into`,
`build_contours_into` and `build_polygon_mesh_into` stages. Region output uses a
caller-provided byte buffer and region count; an overflow retains the previous
count, partially written grid and unused buffer tail. Contour and polygon output
parameters retain initialized partial results on native fixed-buffer failures.
`build_cache_mesh` composes these same stages into an owned successful result.

## Crowd path queue

`crowd::PathQueue<K>` shares one persistent sliced search across eight request
slots. Supply a caller-defined filter key, such as a filter-array index, when
requesting a path. `update(&mesh, iterations, |key| &filters[*key])` resolves
current filter values for each active request; it does not snapshot them.
A request with no filter key models native null-filter validation.

Use `request_status` to poll and `path_result` or `path_result_into` to consume
a request. Consuming an unfinished request is allowed and frees its slot.
Completed requests expire after more than two subsequent visits. Result copying
preserves existing status-detail flags; shortening that copy adds no new flag.
Unknown requests leave `_into` output unchanged. Reinitialization retains the
handle counter and initialized metadata while replacing search/path storage.
Rust initializes fresh free-slot metadata; native zero-handle reads of fresh
uninitialized slots are outside the conformance domain.

## Maintained corridors

`crowd::PathCorridor::new(capacity)` creates a path buffer. Call `reset` with
an initial reference and position, then `set_corridor` with a target and path.
The native asserted contract requires that path to be nonempty and strictly
shorter than the buffer capacity. The corridor supports local position/target
movement, corner finding, visibility/topology optimization, validity checks,
start repair and invalid-path trimming. Moving the position projects its height;
moving the target retains native surface-movement height behavior.

`move_over_off_mesh_connection_into` preserves output retention and native
partial mutation on failure. A failed endpoint lookup may already have pruned
the path and written references. The convenience method returns an optional
`OffMeshMove`. The three standalone merge helpers operate on a fixed slice,
a used count, an independent logical capacity and visited references; unused
buffer entries remain unchanged.

## Obstacle avoidance

`crowd::ObstacleAvoidanceQuery` owns bounded circle and segment pools. Additions
silently stop at capacity, and reset retains prepared values in allocated slots.
Supply `VelocityInput` and `AvoidanceParams` to either sampler; results contain
sample count and selected velocity. Optional `AvoidanceDebugData` records sample
velocity, size and five penalties (total, desired velocity, current velocity,
preferred side, collision time). Early-rejected samples still count toward the
sampler total but produce no debug record. Debug storage is independently bounded.

The adaptive sampler checks uninitialized pattern slots in the pinned native
implementation: positive depth with an even clamped division count reaches such
a slot, and 32 divisions/four rings overflows its pattern array. These return a
checked error. Defined depth-zero cases remain supported. Debug reinitialization
retains its sample count; fresh entries return `None` until initialized, and
normalizing uninitialized entries returns a checked failure.


## Crowd simulation

`crowd::Crowd::new(max_agents, max_agent_radius)` creates fixed agent slots,
16 filters, eight avoidance presets and native-sized query/corridor pools.
Supply the same `NavMesh` to `add_agent` and `update`; tiles may be mutated
between operations. Agent parameter defaults are zero; callers provide radius,
height, acceleration, speed, query/optimization ranges, separation weight and
behavior flags. Crowd avoidance presets have the native nonzero defaults.

`add_agent` returns a slot or `-1` when full. An unplaceable agent still occupies
a slot with `STATE_INVALID`. Request a target reference/position or a raw velocity,
then call `update(mesh, dt, debug)`. The update processes native phases and
budgets in order, including deferred path requests and off-mesh animations.
`CrowdDebugInfo` optionally captures visibility targets and avoidance samples.
Its native index compares the active-agent ordinal, which may differ from the
pool slot after removal.

`agent` and `editable_agent` expose pooled state, including inactive slots.
Removal only deactivates a slot; reuse retains the native fields not reset by
addition, including an unfinished animation. Reinitialization replaces agent
storage and avoidance presets but retains filters, path-handle progression and
the last velocity-sample count. Invalid indices return `None`/false or leave
configuration unchanged, as appropriate. Updates return a checked status for
native invalid memory/conversion domains; they preserve defined zero, negative
and non-finite numeric behavior rather than imposing extra time-step policy.


## Mesh accessors and common utilities

`NavMesh::tile(index)` returns an allocated `TileSlot` even when its `tile` field
is `None`. `tile_by_ref` checks salt and slot bounds, ignores polygon bits and
also permits unloaded slots, matching the native getter. `tile_reference` accepts
an optional slot index (`None` returns zero); it also supplies the native polygon
base reference. `tile_and_polygon_by_ref` validates the loaded polygon and
returns the native status. `polygon_from_bits` deliberately ignores salt/zero
checks while checking Rust memory bounds, providing the native unchecked getter's
defined contexts. `tiles_at_bounded` honors caller capacity; the existing
`tiles_at` convenience method retains the internal 32-neighbor limit.

The `common` module exposes native triangle, polygon, vector, bounds, scalar and
integer helpers. Floating-point helpers preserve native operation order and
comparison semantics rather than using Rust min/max NaN policies. Normalizing a
zero vector and finding the center of zero indices retain native NaN results.
Distance-to-edge helpers retain caller buffer tails; failed triangle-height
queries retain the supplied height. Invalid array indices and native signed
arithmetic/conversion overflow domains return `None`. Rust arrays, assignment,
`std::mem::swap`, and integer `swap_bytes` also provide the native set/copy/swap
operations; explicit endian wrappers include `f32` bit swapping.
`common::floor` and `common::ceil` preserve native signaling-NaN quieting while
keeping its sign and payload. Other arithmetic NaN signs/payloads are not a
portable compatibility guarantee; raw copies and scalar selections retain bits.



`CompactHeightfield::erode_walkable_area` accepts a signed native radius and
returns `Result<(), ErosionError>`. Recast uses the low byte of twice that radius
as its distance threshold: 128 gives zero, 129 gives two, and -1 gives 254.
`BuildConfig::walkable_radius` exposes the same behavior. Signed multiplication
overflow and allocation failure return an error before changing area IDs. The
higher-level tiled `NavigationSettings` keeps its separate practical radius limit.


Recast filter and compact-builder height/climb parameters use native signed
32-bit integers, including the corresponding `BuildConfig` fields. The ledge
filter uses a strict headroom comparison; low-height filtering uses its distinct
native comparison. `filter_ledge_spans` returns `FilterError` for native signed
arithmetic overflow. Compact connections permit native layer indices 0–62;
63 remains the no-connection sentinel. Layer merge thresholds retain native
ushort truncation before promoted signed range comparisons; all signed height
inputs participate through their low 16 bits.


`BuildConfig::max_edge_len` and compact `build_contours` use native signed
32-bit edge lengths. Nonpositive values disable edge tessellation. Contour
building returns `Result<ContourSet, ContourError>` and checks signed squared
lengths only when native tessellation evaluates them; extreme values remain
valid with tessellation disabled. Vertical world bounds may exceed the packed
13-bit solid-span range. Rasterization preserves native clamping/bit packing at
that ceiling and returns `RasterizationError::IntegerConversion` for undefined
native float-to-int conversion domains.


Heightfield, compact-field and contour dimensions/borders now retain native
signed 32-bit grid fields. Solid span insertion uses signed coordinates and
checks the resulting flat allocation index; defined row crossings retain native
behavior. `BuildConfig` applies its explicit `max_grid_cells` allocation policy
and checks signed index arithmetic, without the old 16383-cell axis cap.
`Contour` vertices use `glam::IVec3`: triangulation precedes native 16-bit polygon
vertex welding. Detail patch bounds and interior sampling indices also use signed
integers, so high-coordinate samples do not truncate at 65535.


Region minimum/merge area thresholds use native signed integers. Negative
thresholds retain native comparison behavior; they are not converted to unsigned
counts. Watershed and contour traversal preserve native border handling, ushort
level wrapping and the exact 39999-iteration traversal cutoff, including the
resulting truncated contour data.


The standalone cache polygon builder retains native integer vertex counts while
its mesh indices wrap as unsigned shorts. Output may therefore contain the
65535 sentinel as a newly produced vertex index, overwrite earlier rows after
wrapping, and retain zero-initialized unused vertex rows. Triangulation preserves
native ushort index/ear flags past 32768 contour vertices. A wrapped vertex hash
chain that would never terminate returns a checked invalid-parameter error.


`add_tile_data_with_flags` retains native flags, including `TILE_FREE_DATA`.
`add_tile_data_sized` additionally retains the declared signed data size separately
from the actual safe backing slice, as native Detour does. Matching single-tile
constructors are available. `Tile` and `TileSlot` expose flags and declared size;
`TileSlot::data` also returns retained bytes from an unloaded caller-owned slot.
All byte storage is owned safely by Rust, and returned buffers are independent
copies.

`remove_tile_data_into` and `remove_tile_data_with_ownership` model native outputs
with `RemovedTileData`: free-data tiles return `None` and size zero; other tiles
return bytes and the declared size. Failed `_into` calls retain both outputs.
Caller-owned removal retains native vacant-slot data/size until replacement.
The existing `remove_tile_data` convenience method always returns a byte backup.
Generated navigation and standalone TileCache tiles carry the native free-data
flag, so ordinary cache replacement does not retain their removed bytes.

Compact-cell index assignments retain their native low 24 bits. Counts wrap at
8 bits, including compact construction with more than 255 spans in a column.
`CompactSpan::connections_raw` and `set_connections_raw` expose the independent
24-bit connection field; `con_raw` / `set_con_raw` correspond to native connection
helpers with checked undefined-shift domains. Connection writes preserve the
height byte. The `recast::direction_offset_x`, `direction_offset_z` and
`direction_for_offset` helpers preserve the native lookup and arithmetic rules;
the last returns `None` outside the defined native domain.

The `recast` module exports the native scalar and vector helper equivalents,
including comparison-ordered `minimum` / `maximum` / `clamp`, signed-zero-preserving
`absolute`, `square` / `square_root`, cross/dot products, scaled addition,
componentwise arithmetic/min/max, distance and normalization. These share the
source-identical Detour translations, checked directly against both libraries.
Normalization preserves native zero-length and non-finite results.

`NavMesh::from_params` accepts native signed `NavMeshParams` and retains all
origin/dimension bits. Initialization does not reject zero, negative or non-finite
float dimensions. `max_polygons` is signed reference-budget metadata; no polygon
array is allocated from it. Zero tile budgets create an empty pool, while
negative tile counts return an invalid-storage error. The `new` convenience API
accepts nonnegative capacities within the native signed range.

Native unsigned salt-budget arithmetic can accept extreme 32-bit budgets whose
combined tile/polygon bit counts later cause undefined shifts. Use
`ReferenceLayout::try_encode` / `try_decode` to check that domain; the existing
`encode` / `decode` methods panic explicitly outside it. Native coordinate
conversion is checked separately: `calc_tile_location` returns `None` for
undefined float-to-int conversions, and polygon queries return `INVALID_PARAM`.

`nav_data::create` retains native float payloads, including NaNs, infinities,
signed zero, reversed bounds and nonpositive scales when the evaluated native
operations are defined. BV-tree construction checks the actual float-to-int or
float-to-ushort conversions, rather than rejecting every non-finite input.
Tile import also retains float bits; individual queries and link construction
still require their own native arithmetic and memory domains. Direct fixtures
cross-load and export non-finite payloads without BV trees, portals or off-mesh
links, where loading performs no numeric conversion.

Runtime `nav::Poly` and parsed `nav_data::Polygon` expose the native
`area_and_type` byte. Use `area()` / `kind()` for the masked values and
`set_area()` / `set_kind()` to preserve the other field. Area writes retain six
bits, and type writes retain two bits, matching native `dtPoly` even for byte
values outside the documented area/type enumerations. State restore and query
filters use these accessors.

`recast::Context::with_hooks(enabled, &sink)` accepts caller-owned `ContextHooks`.
The sink supplies log storage and timer values; the default context has no output
and reports -1 for accumulated time. Log and timer switches are independent.
`scoped_timer` stops at drop using the current timer switch. `log_bytes` preserves
arbitrary formatted bytes, including embedded NUL; `log_format` replaces C varargs
with Rust formatting and uses the same 511-byte payload limit. Overlong messages
emit `Log message was truncated` at error level before the original truncated
message, including partial UTF-8 sequences when the byte boundary cuts a character.

Context variants currently cover compact construction (`into_compact_with_context`),
erosion, standalone area marking/median, mesh merging, all solid filters and
rasterization overloads, heightfield layers and distance-field distance/blur
phases, all region strategies, contour trace/simplification phases, and polygon/detail
construction. Use `into_polygon_mesh_with_context` and `DetailNavmesh::new_with_context`
for standalone mesh builds. `build_tile_with_context`,
`build_tile_observed_with_context`, and `build_layers_with_context` thread one
caller-owned sink through the complete pipeline. Compact connection indices above
62 produce a diagnostic and leave those links unset while returning success,
matching native Recast. Construction scans the signed linear grid and reachable
column lists; unused span-pool entries do not contribute to the result.

`layers::build_heightfield_layers_with_context` emits native layer timers and
region/overlap failure diagnostics. Borders that leave no interior produce an
empty successful result. Layer merging uses `(walkable_height as u16).wrapping_mul(4)`
before adding it to promoted region heights: 16384 wraps the
threshold to zero, while -1 yields 65532. Disconnected surfaces therefore merge
or remain separate according to that wrapped threshold.

`build_contours_with_context` preserves the initial trace scope, per-contour
trace/simplification pairs, capacity warnings, and hole-collection diagnostics.
Pointer-bearing bridge-failure messages contain actual Rust contour addresses;
those process-specific addresses are outside byte-equivalence comparisons.

Region methods also have `_with_context` variants. Watershed callbacks preserve
the native expansion/flood loop scopes and region-filter scope. Its watershed
timer is manually stopped: a region-ID overflow returns an error with public
field output retained and leaves that timer open. Overlapping regions instead
emit an error diagnostic and retain successful output, including native region
count metadata. Monotone and layer partitioning use the shared outer/filter
scopes without watershed events.

Nonempty detail meshes require a positive sample distance. Pinned Recast skips
hull initialization at zero, negative, or NaN distances and then reads that
storage during triangulation. Rust returns `DetailNavmeshError::UninitializedHull`
in this domain. Empty meshes still return success before inspecting the distance.
Polygon triangulation preserves native partial triangles and emits a warning
for zero as well as negative native triangle counts. Polygon merging keeps full
integer vertex counts; exceeding 65535 emits an error diagnostic but succeeds.

Detour status helpers in `nav` test bits independently, so a raw status containing
several high bits can satisfy several predicates. `status_has_detail` tests
whether any bit of the supplied mask is present. `detail_triangle_edge_flags`
returns `Some` for native defined shift indices 0–15, including indices beyond
the three geometric edges, and `None` for undefined signed multiplication/shifts.

Native tile creation accepts polygon widths 0–6 when the supplied initialized
detail data makes the operation defined. One- and two-vertex polygons can be
loaded; explicit initialized detail triangles also permit the tested geometry
queries. Zero-width fixtures have no link storage, so loading them would make
native Detour initialize `links[-1]`; the safe loader rejects that domain.
Packed polygon types 2 and 3 are retained for ordinary polygon storage/querying;
native random-point selection excludes them.

Creation preserves supplied detail triangle bases and byte-wrapped counts
without dereferencing those per-polygon ranges. The global triangle array is
copied separately. A zero stored triangle count ignores its base while loading.
A source vertex base is ignored when no extra vertices are copied and BV
construction is disabled. When copying, native signed base conversion and
skipping the polygon's own vertices can make bases -1, -2 or -3 valid. Actual
copy and BV reads remain checked against initialized backing arrays.

Empty polygon meshes have zero decoded polygons at width zero, including
`PolygonNavmesh::default()`. Counting and iteration return zero/empty, and detail
construction returns success before inspecting sampling parameters.

`NodePool::node`/`node_mut` address logical allocated nodes. Native one-based
backing access uses `node_at_index`/`node_at_index_mut`, including records retained
after `clear`. Fresh unused native record fields are uninitialized; Rust exposes
safe defaults there without claiming equivalence for native uninitialized reads.

`NodeQueue::top` exposes initialized native heap slot zero, so it can retain
`Some(handle)` after `clear` or the final pop. Check `is_empty` for logical state;
`pop` returns `None` when empty. A fresh queue has no initialized top. `Some(0)`
represents an initialized native null singleton, which can be peeked, modified,
cleared and popped without dereferencing a node. A second insertion while null
is queued is rejected because native comparison would dereference null. The
native allocation has one pointer beyond the declared capacity; this physical
spare slot is accepted, and insertions beyond it are rejected.

Crowd `Agent::neighbors` and `Agent::corners` are fixed backing arrays of six and
four entries. Use `neighbors()` and `corners()` for their live prefixes, or
`neighbor_count` / `corner_count` with the backing arrays to inspect retained
slots. Neighbor indices retain the native signed integer type. Adding an agent
resets its neighbor count while retaining backing entries and the existing
corner count, matching native slot reuse. `PathCorridor::find_corners_into` takes
a caller-owned corner slice and reusable `StraightPath` scratch storage; its
return value is the live count and initialized slots after it are retained.

`recast::Config` is the native parameter aggregation: slope is stored in
`slope_degrees`, integer fields (including explicit grid/tile sizes and polygon
width) retain signed 32-bit storage, and `Default` zero-initializes the record.
It stores values for individual low-level calls. `BuildConfig::default()` provides
the high-level pipeline's configured defaults and derives grid dimensions from
bounds. The reused crate's radian-based configuration builder has different units
and is not the native `recast::Config` type.

`NavMeshQuery::find_path_into` fills a reusable `Corridor`, retaining its vector
capacity between searches. `find_path` remains an owned-result convenience
wrapper over the same implementation. Invalid input clears the logical result;
query pool/status ordering is unchanged. Pair it with `straight_path_into` to
reuse both stages of query output.

`find_local_neighborhood_into` and `poly_wall_segments_into` likewise retain
caller-owned result vectors. LocalBoundary uses these forms internally and keeps
its neighborhood/wall scratch across updates. `NavMesh::add_tile_data_owned`
accepts a tile byte vector without duplicating its backing bytes; native flags
and removal semantics remain independent of this Rust ownership choice.

`find_polygons_around_circle_into` and `find_polygons_around_shape_into` reuse
`DijkstraSearch::entries`. `path_from_dijkstra_search_into` reuses a `Corridor`;
invalid endpoint references or invalid budgets preserve its previous path, while
a valid endpoint without a closed search node clears it, matching native output
initialization order. `finalize_sliced_find_path_into` and its `_partial_into`
variant likewise retain corridor capacity; invalid output budgets clear the
logical output while retaining sliced-search state. Owned methods delegate to
the same implementation. PathQueue reuses its request storage internally.

`Tile::data_into` and `NavMesh::tile_data_into` export complete native tile bytes
into a reusable caller vector. Padding and initialized spare records are retained,
and current runtime fields are applied in the same order as owned exports. The
mesh form leaves the caller's buffer unchanged for an invalid tile reference.

Runtime `nav::Poly` now aliases the fixed `nav_data::Polygon` record. Its
`vertices` and `neighbors` fields retain all six native entries; `vertex_count`
is the active byte count. Use `vertices()` and `neighbors()` for geometry and
edge traversal. `Link::edge` retains the native `u8` width. The explicit byte
serializer remains the native-format contract.

Solid heightfield spans use `Span::new(min, max, area)` and `min()`, `max()`,
`area()` accessors. `set_min`, `set_max` and `set_area` truncate to native
13/13/6-bit widths. The public `next` link retains a generational `SpanKey`.
`Spans` provides insertion/removal, checked shared/mutable lookup, indexing,
live iteration and clear/reuse without exposing its allocation layout.

`ProximityGrid::query_items_slice(bounds, &mut ids)` accepts a fixed caller
buffer and returns the used prefix length. Entries after that prefix retain
their previous values. The owned `query_items` and retained-Vec
`query_items_into` forms keep the same ordered unique results.
