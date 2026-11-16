# Library coverage ledger

Reference: RecastNavigation v1.6.0, commit
`6dc1667f580357e8a2154c28b7867bea7e8ad3a7`. Baseline: `7d68bf0`.
Implementation, public API, differential evidence, binary compatibility and
performance are independent claims. All scoped operational library families are
implemented and pass the recorded macOS arm64 acceptance matrix. This is a
source/API and tested-behavior claim, not proof of equivalence on every input.

Legend: **partial** means a restricted implementation or input domain;
**yes** means present for the domain described, not all-input equivalence;
**no** means missing. Rows group upstream overloads with the same purpose.
Rust ownership replaces allocation/free pointer machinery. Context hooks and
algorithm instrumentation are tracked explicitly below.
Demo, editor, importers and DebugUtils drawing/dump APIs are excluded.

The [source-anchored public API map](public-api.md) inventories all 20 library
headers in both reference-width modes, including overloads, fields, constants,
type aliases and operational macros. It records ownership/language/contract
adaptations explicitly. Run `python3 scripts/check_public_api.py` to verify the
native declaration inventory; behavioral evidence remains separate below.

## Recast

| Upstream feature/API | Implementation | Exposed API | Differential evidence |
| --- | --- | --- | --- |
| Scalar/vector helpers | yes, shared with identical Detour expressions | recast scalar/vector exports and square_root; arrays, assignment and std::mem::swap cover copy/set/swap | direct native Recast bits for seeded arithmetic, zero/negative-zero, quiet/signaling NaN operand combinations, infinities and subnormals under separate/FMA policies; shared scalar templates also compare native Recast/Detour integer promotion/narrowing, bool and f64 bits |
| Build context/logging/timers | yes within native initialized-memory and allocation qualifications | recast::Context / ContextHooks / LogCategory / TimerLabel / ScopedTimer; context variants for all timed Recast operations and full tile/layer pipelines | direct native enable/reset/get/start/stop/scope callback order and caller timer values, all timer labels, byte-preserving 511-byte truncation with its preceding error log; full generation corpus compares the complete callback stream alongside stage data. Error paths cover compact overflow success, layer failure, watershed timer left open on region failure, region overlaps, contour capacity/multiple outlines, partial/zero triangulation, vertex cutoff and successful 65536-vertex merge diagnostics |
| Bounds/grid calculation, heightfield creation, addSpan/span count | yes with signed native grid dimensions | recast input utilities, heightfield builder, add_span and SpanInsertion | native bound/grid bits, ordered insertion/merging, packed fields, signed thresholds and non-null span counts; 65537-cell axes, checked conversion and defined signed flat-index row crossings |
| Mark/clear walkable triangles | yes for indexed arrays | recast::mark_walkable_triangles / clear_unwalkable_triangles; production pipeline uses degree API | direct custom-area/degenerate/seeded threshold fixtures; minimized slope-rounding pipeline regression |
| Rasterize triangle, indexed 16/32-bit and nonindexed triangles | yes with signed native grid dimensions; checked native float-to-int domains | recast rasterize_triangle / rasterize_triangles / _u16 / _u32 | real upstream overloads; ordered spans after batch calls and individual triangles; custom IDs and signed thresholds; packed-ceiling/reversed-clamp behavior at and above 8191 in every overload |
| Low hanging, ledge, low-height span filters | yes with signed native parameters and checked ledge arithmetic overflow | dependency and pipeline; FilterError | direct full ordered span snapshots for 77 signed height/climb combinations, zero-height spans, exact strict headroom comparisons and complete generation corpus |
| Compact heightfield construction/connections | yes with signed metadata and native packed fields | dependency and pipeline; CompactCell / CompactSpan raw field helpers; recast direction helpers | full ordered cells/spans/connections and metadata/bounds under signed parameters; native last valid neighbor index 62; excess indices log and succeed with unset connections; signed linear columns and reachable-only span counting; promoted span-top addition; packed four-byte cells, direct 24-bit assignments, 8-bit count wrap, connection/height separation and direction operations; complete 255/256/257-span columns |
| Erode walkable area | yes for signed radii whose doubled value fits native int | CompactHeightfield::erode_walkable_area / ErosionError; signed BuildConfig::walkable_radius | exact ordered compact data across negative/large radii, native byte threshold wrapping, saturated distances and complete pipeline; atomic checked overflow/allocation failure |
| Box/cylinder/convex area marking, median filter | yes | recast module free functions | direct C++ ordered full-field snapshots after each operation; seeded stacked surfaces |
| Offset polygon | yes | recast::offset_poly | direct C++ float bits, bevels, degenerate/empty/one/two-vertex inputs, partial writes and capacities; 10,000 independent mixed geometry/offset tuples retain native NaN signs and operand order |
| Distance field; watershed regions | yes within checked native storage/arithmetic domains | dependency and pipeline | ordered distance/region stages; direct signed area thresholds, border conflict scans and maximum-distance promotion/wrapped levels |
| Monotone regions; layer regions | yes within checked native storage/arithmetic domains | compact methods, BuildConfig::partitioning / RegionPartitioning | direct ordered fields and signed area thresholds through i32 limits; complete 13-stage corpus in every strategy |
| Heightfield layers | yes within checked grid/storage domains; native ushort merge-threshold wrap and promoted height sums | layers module, build_heightfield_layers_with_context | ordered metadata/grids, 13 signed thresholds on stacked surfaces and disconnected height islands; actual one/two-layer discrimination, empty interior borders through int limits and native region-overflow status/diagnostics |
| Contours, flags and hole merging | all flags and signed edge lengths; native Apple equal-key ties, qualified widened hole predicates | compact methods, build_contours_with_context and BuildConfig::contour_flags | direct libc tie-record corpus; ring/hole and disconnected outline context fixtures, all four flag combinations, raw/simplified area boundaries and full pipeline option fixtures; negative edge lengths, exact large signed squares and tessellation-disabled extremes; checked native square overflow and exact truncated raw data at the 39999-iteration walk limit |
| Polygon mesh and adjacency | yes within checked native storage/arithmetic domains, native polygon-width budgets | dependency and pipeline | ordered mesh/areas/regions/flags; actual eight-vertex polygons, 65536-wide storage, exact 65534-vertex rejection and full data at 65533 vertices; vertex-zero border removal, partial/empty triangulation warnings and logical region counts; empty widths 0/1/2/6/8/65536 and zero-width default count/iteration/detail early return |
| Polygon mesh copy/merge | yes | Clone / recast::merge_poly_meshes / merge_poly_meshes_with_context | direct C++ copies, weld ordering/height threshold, portal preservation and rebuilt adjacency, 3/4/6 vertices; 65536 vertices retained with native success/error diagnostic |
| Detail mesh construction | yes within initialized native hull and checked storage domains | DetailNavmesh::new / new_with_context / DetailNavmeshError and pipeline | ordered submeshes/vertices/triangles/bits; native high-coordinate interior refinement above sample index 65535 |
| Detail mesh merge | yes | recast::merge_detail_meshes | direct C++ submesh rebasing, null entries, local triangle IDs, flags and vertex bits |

## Detour mesh and common utilities

| Upstream feature/API | Implementation | Exposed API | Differential evidence |
| --- | --- | --- | --- |
| Mesh initialization/reference budgets | yes for nonnegative tile allocations; signed polygon metadata and checked reference shifts | NavMesh::from_params / new; native signed NavMeshParams; ReferenceLayout::from_native_capacities / try_encode / try_decode | direct status/parameter-bit/slot/reference snapshots for zero and signed/extreme budgets, unsigned salt-budget wrapping, zero/negative/non-finite dimensions/origins; all actual fixture tile allocations capped at eight |
| Mesh create params/createNavMeshData | yes with checked array/size and evaluated BV conversion domains | nav_data::CreateParams / create | native whole-buffer bytes; 0–6 polygon widths with initialized explicit details, one/two-vertex load/query domains, details, off-mesh records, portals, BV tree and creation rejection; raw non-finite/signed-zero metadata and geometry, NaN comparison/operand order, reversed bounds/scales and unsigned-short narrowing of promoted BV extents; ignored detail-copy bases, signed bases after skipping polygon vertices, arbitrary triangle metadata and zero-declared-count BV backing reads |
| Full detail mesh import and height/edge queries | yes for checked tile data | add_tile_data / from_tile_data, closest_point / poly_height | direct native imported detail heights, boundary projection and edge fallback, float bits |
| Polygon packed properties | yes | nav::Poly / nav_data::Polygon area_and_type; area / kind / set_area / set_kind | every initial byte with six area/type assignment combinations, raw bytes and both getters after each write, directly against native dtPoly; runtime/state/filter paths share masked access |
| Off-mesh connections, direction/endpoints/user IDs | yes | nav_data creation/parsing; runtime linking, off_mesh_connection / off_mesh_endpoints | native classification/culling, endpoint snapping and pool allocation, local/cross-tile links, one/bidirectional paths and funnel flags, both insertion orders and reloads |
| Init/capacities/encode/decode refs | yes: native rounded 32-bit capacities; configurable runtime polygon budget | NavMesh, ReferenceLayout | direct capacity/bit fixtures, runtime budget fixtures, removal and salt rollover |
| Native 64-bit refs (16 salt/28 tile/20 poly bits) | yes | polyref64 feature, PolyRef/TileRef | native DT_POLYREF64 drivers, direct refs and runtime corpus |
| Tile add/remove, free list, cross-tile links | yes for checked backing arrays; flags/declared sizes retained with Rust-owned copies | add_tile_data / add_tile_data_with_flags / add_tile_data_sized; remove_tile_data_into / remove_tile_data_with_ownership; coordinate helpers, Tile::data / TileSlot::data | packed types 2/3 load/export/query and native random exclusion; exact post-load/export/removal bytes/pool/free list, lastRef (including zero salt), stale refs, slot failures/budgets, bounded layer-disconnect behavior; flags/free-data null returns and retained vacant-slot bytes/sizes, independent declared sizes and failure-output retention |
| Status and detail-triangle edge helpers | yes, checked undefined shifts | nav::status_succeeded / status_failed / status_in_progress / status_has_detail / detail_triangle_edge_flags | direct native status masks including combined high bits and all 256 flag bytes at every defined promoted shift, indices 0–15; checked negative/oversized/overflowing indices |
| Tile lookup/location/ref accessors | yes within checked float-to-int domains | NavMesh::params / calc_tile_location / max_tiles / tile / tile_by_ref / tile_at / tile_ref_at / tile_reference / tiles_at_bounded / tile_and_polygon_by_ref / polygon_from_bits / is_valid_poly_ref; TileSlot preserves allocated-but-unloaded slots | native public overloads across 35 layers and caller capacities above 32, exact original params/refs/statuses, unloaded slots, stale salts, unchecked decoded contexts, removal/reload and location bits |
| Polygon flags/area accessors and mutation | yes | poly_flags / poly_area / set_poly_flags / set_poly_area | native accessors/statuses, area masking, zero/stale refs and serialized mutation |
| Store/restore tile state | yes | tile_state_size / store_tile_state / restore_tile_state | exact native bytes with caller-filled padding/tail, size/status precedence, magic/version/ref checks, salt invalidation/restoration |
| BV tree build/query | yes for current query paths | nav_data creation/parsing; internal candidate traversal | ordered nodes/quantization, seeded ties and eight-equal-bound regression; imported nearest/off-mesh candidates; native ordered clamping for defined reversed boxes and ordered min/max/overlap for NaN polygon bounds |
| Native tile headers/data, endian conversion | yes for v7 | nav_data views/swaps; runtime load/export/remove and native tile state | full pre/post-load bytes in both ref widths; bidirectional native loading, retained reserved bytes, endian round trips and native user-ID omission |
| Common geometry/vector/integer helpers | yes with checked native memory/signed-overflow domains | common module: triangle projection/height, point-in-polygon, point-to-segment/edge distances, center, clipping/intersection/overlap/random sampling; vector/scalar/bounds/power/log/alignment/direction/endian helpers. Rust arrays, assignment and std::mem::swap replace set/copy/swap machinery | direct native raw bits/partial and retained outputs, all triangle regions, degenerate/empty polygons, zero/negative-zero/NaN/infinite vectors, seeded arithmetic, unsigned wrap boundaries and all endian types. Runtime/Crowd share arithmetic helpers |
| Standard math backend wrappers | common::cosine / sine / floor / ceil plus Rust f32 operations | abs / sqrt / floor / ceil / cos / sin / atan2 / is_finite | all eight native dtMath wrappers; 617 raw-bit input pairs including both zero signs, quiet/signaling NaN payloads, infinities, subnormals and seeded words on macOS arm64 and Linux x86_64; rounding quiets signaling NaNs while retaining their sign/payload |
| Node pool/queue | yes with checked handles and Rust memory estimates | nodes::Node / NodePool / NodeQueue; node_at_index / node_at_index_mut expose native one-based backing; production A* uses them | native hash chains, state/parent/flag packing, find/find-many order, capacity, clear/reuse, priority ties/NaNs/infinities, modify/duplicates/retained backing storage; direct initialized post-clear pool reads/writes, post-clear/final-pop queue head, physical spare slot and null singleton |

## Detour query

| Upstream feature/API | Implementation | Exposed API | Differential evidence |
| --- | --- | --- | --- |
| Query init/node budgets | persistent storage, native fresh budgets 1–65535 and retained larger reinit capacity; checked zero-table hash access | query::NavMeshQuery, stateless mesh wrappers | full native node records/hash chains/positions/cost bits/flags after searches and reinitialization, exhaustion and early-return retention; budgets 1/2/3 initialize with zero hash size and preserve same-poly paths/slices, tiny-pool movement/neighborhood and independent random selection |
| Query filter flags, area costs, custom pass/cost extensions | yes in implemented query paths | Filter / QueryFilter / FilterPolygon | native defaults/masks, changed-cost routes, eligibility overrides, exact callback order and previous/current/next tile/poly/point contexts against DT_VIRTUAL_QUERYFILTER |
| FindPath A* | yes for ground/off-mesh polygons | find_path | raw ordered refs/statuses, seeded terrains, imported one/bidirectional off-mesh fixtures; retained initialized positions after a failed portal midpoint |
| Straight path funnel and crossing options | yes | straight_path / straight_path_with_options; NavMeshQuery::straight_path | raw points/bits/flags/refs, area/all/both option precedence, slopes, turns, off-mesh transitions, truncation, partial recovery and original endpoint clamping |
| Nearest/closest point | yes | nearest / nearest_with_over, closest_point | imported details/BV, custom filters, over-polygon flag, off-mesh projection, zero/stale refs, numeric bits |
| Closest boundary, poly height, validity, closed-list inspection | yes | closest_boundary / poly_height / resolve; NavMeshQuery::is_valid_poly_ref / is_in_closed_list / node_pool | native boundary/height/status bits, filter validity, closed-list/node records, stale references and ignored non-finite Y for height |
| queryPolygons buffer/callback overloads | yes | query_polygons / query_polygons_with | exact refs, 32-polygon batches per tile and filter/process call interleaving, zero/full/overflow capacities, layer ordering and trailing BV duplicate |
| Sliced init/update/finalize/partial finalize, any-angle | yes with mesh/filter supplied per operation | NavMeshQuery::init_sliced_find_path / update_sliced_find_path / finalize_sliced_find_path / finalize_sliced_find_path_partial | exact statuses, callback order, node records/parent reversal/detached flags; step budgets, exhaustion, radius-limited shortcuts, partial paths, buffer edge cases, filter mutation and disappearing current/parent/grandparent tiles |
| Raycast overloads/options and cost | yes | raycast / raycast_with_options / raycast_into on mesh/query, RaycastHit | native hit/normal/edge/path/cost bits, reused output fields, legacy/options, partial portals both axes, filters and retained zero-ref candidate contexts, seeded slanted normals; distinct quiet/signaling NaN area costs retain native cumulative operand order |
| moveAlongSurface | yes with separate native 64-node small pool | NavMeshQuery::move_along_surface / move_along_surface_into, SurfaceMove | exact visited refs/statuses/result bits and filter callback order; walls/slopes/retained endpoint Y, off-mesh starts, pool exhaustion, zero/full capacities, main-pool retention and layered eight-neighbor cap |
| Circle/shape search and Dijkstra path extraction | yes with checked undefined portal-failure domain | NavMeshQuery::find_polygons_around_circle / find_polygons_around_shape / path_from_dijkstra_search, DijkstraSearch | exact ordered refs/parents/cost bits, complete retained node records, custom filter/cost callbacks, portal geometry, zero/full buffers, exhaustion, shape winding/non-finite Y, off-mesh traversal and defined stale-slot contexts |
| Local neighborhood | yes with separate native small pool and checked portal-failure domain | NavMeshQuery::find_local_neighborhood, Neighborhood | exact refs/parents/filter callbacks, 64-node/48-FIFO limits, retained main pool, layered/folded overlaps and capacity-dependent comparisons against stored results |
| Distance to wall / wall segments | yes | NavMeshQuery::find_distance_to_wall / find_distance_to_wall_into / poly_wall_segments; NavMesh::poly_wall_segments | exact ordered segments/ref/point bits, 16-interval truncation/sentinels/overlaps, first-link wall eligibility, normals/distance/statuses, filter calls, retained caller output and complete main-pool records |
| Random point / random around circle | yes with caller RNG and checked portal-failure domain | NavMeshQuery::find_random_point / find_random_point_around_circle and _into forms, RandomPoint | exact RNG/filter interleaving and draw counts, returned refs/point bits, retained outputs/main-pool state, unequal polygon areas, tile sampling before filtering, zero refs, off-mesh traversal and detail-height projection |

## DetourTileCache

| Upstream feature/API | Implementation | Exposed API | Differential evidence |
| --- | --- | --- | --- |
| Layer regions/contours/polymesh | yes with byte layers, native integer counts and ushort index/flag wrapping | build_cache_mesh / build_regions_into / build_contours_into / build_polygon_mesh / build_polygon_mesh_into, CacheBuildError | ordered stages, signed climb, empty layers, simplification bits, region overflow, strict triangulation, border removal, fixed removal-buffer failure, complete output through 65537 vertices and the 32769-contour triangulation boundary; retained partial grids/counts/contours/mesh after failures and defined flat-index row crossings; nonterminating wrapped vertex chains return checked errors |
| Box/cylinder/oriented box marking | yes for checked native byte-sized layers | mark_box_area / mark_cylinder_area / mark_oriented_box_area; mark_box area-zero convenience | native sequential full-grid snapshots across scales, byte IDs, height 255, zero/negative dimensions, shifted bounds, rotations, FMA, byte-sized boundaries and zero width; checked invalid storage/conversion domains |
| Queued obstacle lifecycle, salted handles | yes for cylinder/AABB/oriented box, checked float-to-int domains | tile_cache::TileCache add_cylinder / add_box / add_oriented_box / remove_obstacle / update | ordered states, full retained union/touched/pending arrays, salted free lists, request/update/touched limits, no-touch states, failed rebuild advancement and queued removal |
| Native compressed layer build/decompress/endian | yes for v1 with caller codec | cache_data::Header / build_layer / decompress_layer / swap_header_endian, Layer | complete bytes/padding/payload, independent native header construction, cross-loading both producers, four-grid decompression/partial writes, callback status/size quirks and endian round trips |
| Compressor/mesh-process extension points | yes with owned Rust arrays | cache_data::Compressor / CompressionResult; tile_cache::MeshProcess, nav_data::OwnedCreateParams | exact compressor inputs/statuses, callback order/mesh inputs, area/flag edits, coordinate changes, creation failures, BV metadata and off-mesh injection |
| Tile add/remove/streaming/ref accessors | yes with owned bytes and checked capacities | tile_cache::TileCache pools / add_tile / remove_tile / tile_at / tiles_at / ref helpers / reinitialize | full slot bytes/flags/salts/hash/free chains, ownership flags, duplicate/magic/version/exhaustion statuses, raw ref packing, free-slot lookup and retained rebuilds across reinitialization |
| Tile query/tight bounds/obstacle bounds | yes within checked float-to-int domains | TileCache::query_tiles / tight_tile_bounds; Obstacle::bounds | bounded hash order, 32-layer cap, ordered coordinate traversal, inclusive overlap/NaN Y, exact bounds/shape float bits and native byte promotion |
| Build one tile/build tiles at/update | yes within checked builder domains | TileCache::build_tile / build_tiles_at / update | complete linked navmesh bytes after each operation, empty removal, callback mutation/failure, full tile replacement and add failures, non-binary scales, all shape updates |

## DetourCrowd

The pinned Crowd sources are vendored unmodified. The complete update loop and
its components are ported; coverage remains qualified by checked native undefined
memory/conversion domains and the configuration/platform evidence below.

| Component | Implementation/API | Differential evidence |
| --- | --- | --- |
| Crowd configuration and agent lifecycle | crowd::Crowd new / reinitialize, agent/editable_agent, active_agents, filters and avoidance presets, add/remove/parameter updates, target/velocity requests and reset | Complete public defined agent state, defaults, all filters/presets, capacity/getter behavior, slot reuse, invalid placement, retained fields, query pools and reinitialization |
| Complete update | Crowd::update with caller mesh/time step and optional CrowdDebugInfo | Per-command/per-step raw bits, refs, neighbors/corners/boundaries, both query pools and avoidance debug samples; all five behavior flags separately/together, dense 12/36-agent collision/neighbor limits, deferred request budgets/replanning, holes in active slots, tile removal/reload, off-mesh animation completion/pause/reuse, zero parameters and zero/negative/NaN time steps |
| Obstacle avoidance and sample debug data | crowd::ObstacleAvoidanceQuery, AvoidanceParams, VelocityInput, VelocitySample, AvoidanceDebugData / AvoidanceDebugSample | Grid/adaptive sample order, winner/count/cost bits, early-outs, prepared circle vectors/touch flags and retained pooled records; debug capacities/normalization/reinit; zero/negative/non-finite inputs and defined depth-zero even divisions. Native uninitialized adaptive pattern slots and pattern overflow return checked errors |
| Maintained corridor and path merges | crowd::PathCorridor reset / set_corridor / find_corners / move_position / move_target_position / optimize_visibility / optimize_topology / fix_path_start / trim_invalid_path / is_valid / move_over_off_mesh_connection and _into; three merge_corridor helpers | Exact refs/position/target/corner bits, full query pools, movement height rules, native visibility scaling/topology search, fix/trim/filter/stale refs, off-mesh first/middle/final-index and failure mutations; raw merge duplicate selection, independent logical capacities and retained full buffers |
| Path queue | crowd::PathQueue request / update / request_status / path_result / path_result_into / reinitialize / query | Eight slots, shared iteration budgets, mutable caller filter identities/null filters, full retained node pools, zero/negative budgets, early reads, capacity/status details, expiry, handle persistence across reinit, stale mesh refs and defined released-slot zero-handle reads |
| Local boundary | crowd::LocalBoundary default / reset / update / is_valid / center / segment_count / segment / segments | Native center/segment bits and validity after each update, eight-segment and sixteen-polygon limits, tie ordering, filtering/mutation, off-mesh starts, stale refs and initialized backing slots retained after reset/shorter updates |
| Proximity grid | crowd::ProximityGrid new / reinitialize / clear / add_item / query_items / query_items_into / query_items_slice / item_count_at / bounds / cell_size | Ordered IDs, duplicate suppression, bounded insertion and declared bounds after exhaustion, clear/reinit, non-binary/infinite cell sizes, 65536-slot sentinel behavior; checked invalid conversion/cyclic-chain domains |

## Configuration and validation matrix

| Configuration/claim | Recorded evidence | Evidence limits / further coverage |
| --- | --- | --- |
| Recast integer/config domain | signed 32-bit grid/border/height/climb/radius/edge fields, 32-bit region/polygon budgets, signed contours and detail sample indices; explicit contour flags | remaining native storage/arithmetic and parameter-domain audit |
| Separate operations | macOS and Linux arm64 Clang, pinned reference corpus | new APIs, extreme inputs, other compilers |
| Explicit FMA | macOS and Linux arm64 Clang, same corpus | each new numeric API; no universal compiler claim |
| 32-bit references | native capacity/bit fixtures, runtime budgets and salt rollover | remaining input/API/platform audit |
| 64-bit references | native fixed layout, raw refs, runtime corpus and native tile data | remaining input/API/platform audit |
| Binary compatibility | native Detour v7 and cache-layer v1 construction/parsing/endian; runtime mesh loading/query/export/removal and tile state; complete bytes/cross-loading in both ref modes | broader operations/platforms |
| Crowd determinism | Full public defined per-step state, bits, ordering and debug data under deterministic command/time streams in all four local configurations | broader randomized lifecycle sequences and platforms |
| Random operations/geometry | 32 seeded terrains × 32 path queries; 64 Crowd streams × 160 mixed operations and 32 TileCache streams × 192 operations, plus setup/reinit/draining; full native snapshots after every command | broader geometry/domain/seed sets; minimize discrepancies when found |
| Linux/macOS CI | core+Bevy native corpus, strict Clippy/docs/dependency checks with separate/FMA arithmetic on macOS arm64 and Linux x86_64; focused optimized Linux numeric targets; see [validation](validation.md) and the workflow result for the selected revision | default 32-bit references in hosted native CI; broader configurations remain separately qualified |
| Linux arm64/glibc | full debug core+Bevy and optimized core native suites, all four modes on Rust 1.95.0 and 1.97.1; Clang 18.1.3, glibc 2.39, local Ubuntu VM; see validation.md | other architectures/libcs and hosted evidence |
| Minimum Rust 1.95 | complete core+Bevy debug and core optimized native suites in all four modes on macOS and Linux arm64; Windows MSVC-target core+Bevy compilation default/combined | other platforms and Windows execution |
| Windows/MSVC | Native C++ differential suites passed locally in all eight debug/release × separate/FMA × 32/64-bit-reference configurations using MSVC-targeting Clang; see the [Windows report](windows-validation.md). Hosted CI runs Rust-only tests and examples. | native C++ differential tests are not run in CI; MSVC-style compiler drivers remain unsupported |
| Bevy/README | existing headless tests/example/doctest | preserve through API changes |
| Performance/allocations | optimized four-library time/RSS and calibrated allocation traffic on macOS arm64; see performance.md | other workloads/platforms |

## Completed implementation sequence

1. Recast custom pipeline utilities and direct function fixtures.
2. Mesh data/details/off-mesh/ref configurations and binary interoperability.
3. Query state/filter extensions, algorithms and options.
4. Complete cache formats/shapes/extensions/streaming.
5. Crowd dependencies and complete deterministic update loop.
6. Audit API/configuration coverage, expand randomized operations and record
   the macOS acceptance results. Every in-scope operational row has evidence;
   retain this ledger when extending domains or changing implementation.

Optimized core conformance also passes all four numeric/reference configurations
on Rust 1.95.0 on macOS arm64, including mixed-sign nonfinite adaptive patterns.

Straight-path queries expose `straight_path_into` for retained caller-owned
point/flag/reference vectors. Proximity-grid queries likewise expose
`query_items_into` and borrowed `query_items_slice`. Native operation streams compare whole initialized output buffers across
shrinking/empty results. Crowd retains neighbor/corner, visibility-raycast and
surface-movement buffers across updates;
the two tiny-pool neighborhood searches use the native bounded 48-entry FIFO.
All four optimized query/Crowd native configurations remain exact.

Optimized four-library time/RSS and calibrated allocation evidence is recorded in
[performance methodology and local results](performance.md). This closes the
previous absence of local performance evidence; platform/API audits remain.

Polygon meshes retain native `maxpolys` metadata and initialized spare polygon,
neighbor, region and area slots in explicit `unused_*` arrays. Native merge
leaves `maxpolys` zero; native copy sets it to the live polygon count and omits
spare storage. `recast::copy_poly_mesh` implements that operation while Rust
`Clone` preserves every Rust field. Direct build/copy fixtures and the complete
generation corpus compare this metadata and every initialized spare slot.

Polygon building also exposes `ContourSet::build_polygon_mesh_into`, retaining
native metadata writes and prior initialized buffers on the conservative vertex
capacity rejection. Direct native fixtures cover empty and populated output.

Crowd agents expose fixed six-neighbor and four-corner backing arrays alongside
live counts and `neighbors()` / `corners()` prefix accessors. All initialized
entries are retained across shorter updates, removal and slot reuse. Native
per-step fixtures compare every backing slot, including direct edits to signed
unused neighbor indices, NaN payloads, signed zero and full-width corner refs.
Standalone corridor corner output compares whole caller buffers; repeated native
pruning moves preserve the initialized tail rather than using one batch drain.

The low-level Recast surface exposes native `Config` with degree-based slope
units and signed grid/tile/polygon-width fields, and reexports `RegionVertexId`,
`SpanKey` and `Spans` for public field storage. Solid `Span::new` and
`min`/`max`/`area` getters with matching `set_*` methods preserve the native
13/13/6-bit fields, including truncation on assignment. `Spans` owns fixed-size
pools and rejects removed key generations; generation metadata is excluded from
logical span equality. The generation API's separate
radian-based builder configuration is not used as the native configuration. `crowd::AgentAnimation` exposes the standalone native
animation record; attached animation storage remains owned by Crowd, as upstream
provides no attached-animation getter.

Public common geometry helpers also compare 1,000 mixed-operand triples through
triangle projection, edge distances and line intersections, with nonfinite
convex sampling cases. Native product/addition operand order and rounded-product
subtraction are retained, including distinct NaN payloads. Use the
four-configuration matrix in [validation](validation.md) when checking optimized
builds.

Stateful cost fixtures compare 36 ordered normal/nonfinite area-cost pairs with
concrete default filters and virtual custom filters. Full A*, Dijkstra, direct
costed rays and sliced/any-angle searches compare complete logical node records.
Costed rays preserve the existing accumulator as the left addend when distinct
NaN payloads meet. The Detour data/query corpus exercises these cases under the
selected arithmetic and reference configuration.
