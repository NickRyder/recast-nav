# Conformance policy

The reference is unmodified C++ RecastNavigation v1.6.0, commit
`6dc1667f580357e8a2154c28b7867bea7e8ad3a7`. Recast, Detour, DetourTileCache and DetourCrowd
sources and licenses are vendored. `scripts/check_vendor.py` verifies their
hashes and accounts for local alterations to reused Rust source. Updating the
reference requires an explicit version decision and reviewed provenance changes.

## Comparisons

The 27-case base generation corpus runs each case under watershed, monotone and
layer partitioning, observing the production pipeline at 13 stages:
triangle classification, rasterization, low-hanging filtering, ledge filtering,
headroom filtering, compact construction, erosion, heightfield layers, distance
field, selected regions, contours, polygons, and detail mesh. On eligible layers,
the tile-cache builder is also compared at region, contour, and polygon stages,
both unchanged and after box marking. Additional configuration fixtures exercise
32-bit region-area/polygon-width budgets and selected contour flags.

Snapshots include logical used data and metadata, excluding allocator capacity,
addresses, and timers. Connections are decoded losslessly. C++ interleaved
polygon/neighbor storage is split without reordering. Default/null area IDs are
63/0. Integers and non-NaN IEEE-754 float bits must match exactly, including signed
zero and infinity signs. Copied, selected, and serialized input float bits remain
exact even for NaNs. Arithmetic-produced quiet NaNs may differ in sign/payload:
the native result itself varies with compiler, target, and optimization. Tests
allow this only at explicitly identified arithmetic output fields, preserving
exact status/count/reference checks; signaling NaNs do not receive this exception.
The `common::floor` and `common::ceil` adapters explicitly quiet signaling NaNs
while retaining their sign and payload, matching the native math backend rather
than assuming Rust's rounding primitives always quiet them. Floating-point
exception flags, traps, and errno are outside these snapshot comparisons.
There is no geometry sorting, ID remapping, or finite floating-point tolerance.
Every C++ stage must be consumed.
Direct input utilities compare triangle marking/clearing with pre-existing custom
area IDs, degenerate triangles and seeded normals near slope cutoffs. A minimized
33.23423-degree regression verifies native angle conversion and normalization
operation order in the production pipeline. Bounds include signed-zero bit
comparisons; grid-size rounding is compared directly. Span fixtures call
`rcAddSpan` and `rcGetHeightFieldSpanCount` after every insertion, comparing
packed fields, overlap ordering and signed flag-merge thresholds. Raster fixtures
call every indexed/nonindexed/single-triangle upstream overload and retain
ordered column/span data, including custom-area truncation and null spans.

Failed generation snapshots are saved under ignored `target/parity-failures/`.

The runtime oracle uses the real `dtTileCache`, `dtNavMesh`, and `dtNavMeshQuery`:

- Ordered tile/polygon data, raw salted refs, link order, sides and portal bounds.
- Obstacle salts, states, touched tiles and pending updates after each operation.
- Projection, A* corridors, funnel waypoints/flags/refs, and status bits.
- Complete, unreachable, proxy-goal and agent-start projection queries.
- Deferred requests, unchanged boxes, replacement/removal, overlapping obstacles,
  fully carved/restored meshes, stale handles and capacity exhaustion.

Direct filter fixtures compare native include/exclude defaults and all 64 area
costs, then compare native `passFilter` against Rust nearest-query eligibility
for zero, low/high bits, overlapping masks and an empty inclusion mask.

The native data/query driver enables upstream's supported `DT_VIRTUAL_QUERYFILTER`
configuration. A derived test filter changes eligibility and costs and records all
callback references, polygon areas, tile coordinates/user IDs and input point bits.
Rust trait-object dispatch must match those logs, changed-cost routes and statuses,
including local/cross-tile off-mesh transitions. The polygon-query overloads also
compare interleaved filter/process calls, native batches of up to 32 per tile,
zero/full/overflow capacities, stacked tile ordering and duplicate BV leaves.
The separate runtime driver retains the ordinary nonvirtual default filter build.
A direct `dtNodeQueue` fixture compares pop ordering for ties, NaNs and infinities;
Rust uses the native strict comparison rather than its non-equivalent complement.
Standalone node-pool fixtures compare all hash heads/next slots, one-based handles,
find/find-many ordering, packed fields, capacity and storage retained after clear.
Only initialized positions are observed. Queue sequences also cover modify, clear,
duplicate handles and live backing records after pool clear. Production A* uses
these same Rust pool/queue implementations. Persistent-query fixtures compare
complete used node records, cost/position bits, closed-list membership, retained
larger reinit capacity, exhaustion, early-return behavior and current filter/ref
validity after tile removal/restoration. Memory-layout/allocator totals are excluded.

Raycast fixtures call both native overloads, retaining raw hit/normal/edge/path/cost
bits, output-field reset rules, truncation, previous references, filter callbacks
and partial portals on both axes. Slanted-wall cases retain seeded numeric inputs.
Native cost callbacks may carry a zero reference with non-null polygon pointers;
these contexts are preserved, including rejected next candidates. Clipping utility
fixtures compare entering/leaving edges and partial parameters on rejection.
Runtime tile snapshots also compare native walkable-height/radius/climb header bits
after generation and every obstacle rebuild.

A minimized 34-layer lifecycle fixture retains upstream's bounded 32-layer
disconnection behavior and subsequent defined stale-slot query output. Public
reference checks validate salt; native internal decoded-context paths preserve
their distinct behavior while checking memory bounds. A caller-supplied zero-salt
lastRef fixture covers zero-ref internal ray candidates and off-mesh basing.
Invalid memory domains are rejected instead of reproducing native undefined access.

The corpus includes ramps, seams, stacked floors, cliffs, fragmented boundaries,
fractional origins and scales, rotated geometry, sequential/parallel generation,
and 32 seeded terrains with 32 path queries each. The standalone corpus is the
source of public conformance claims; external application tests are not required.

The direct Recast utility runner calls `rcMarkBoxArea`, `rcMarkCylinderArea`,
`rcMarkConvexPolyArea`, `rcMedianFilterWalkableArea`, and `rcOffsetPoly`. It
compares complete ordered cell/span data after each area operation on seeded
stacked surfaces, and raw offset vertex bits including partial buffer writes
and upstream's spare-slot capacity requirement. These calls do not alter the
standard generation pipeline. Direct region fixtures also exercise stacked floors,
area boundaries, holes, border clipping and small-region filtering. All contour flag combinations are compared on area boundaries. A capacity
fixture compares full polygon data at 65,533 contour vertices and native rejection
at 65,534/65,535. Rust returns
a checked error for rows exceeding the upstream sweep allocation; native
out-of-bounds writes are outside the conformance domain. Mesh utility fixtures also call `rcCopyPolyMesh`,
`rcMergePolyMeshes`, and `rcMergePolyMeshDetails`, comparing ordered welds,
portal retention/adjacency, metadata and detail submesh offsets without remapping.

The native Detour data driver calls `dtCreateNavMeshData`, both endian utilities,
and `dtNavMesh::init`. The entire created byte buffer must match, including zero
padding/reserved link space, unused BV nodes, all detail triangles, off-mesh IDs
and header fields. Fixtures cover optional/orphan detail vertices, every off-mesh
side, start-height rejection, discarded-endpoint link counts, 3–6 vertex polygons,
border direction codes, BV clamping and native 16-bit detail-offset/8-bit count
packing. C++ loads Rust-created blobs; Rust parses C++-created blobs. Endian checks
also use populated native link/first-link records from C++ loading. No fields are
normalized, including upstream's unswapped off-mesh user IDs. Rust runtime loading
also matches the complete post-load buffer, including pool indices and stale
link contents after disconnection/reloading. Functional fixtures combine detail
heights, BV traversal, local/cross-tile off-mesh links, directions, insertion order,
projection, A* and funnel flags/truncation. State fixtures retain caller-filled
padding/tails and compare native size/error precedence, flag/area accessors,
stale references and lastRef restoration. Truncated/invalid memory domains are
rejected by Rust and are not sent to unsafe native entry points.

An eight-equal-bound BV fixture minimizes a seeded ordering mismatch. On macOS,
pure Rust reproduces the pinned Apple libc quicksort/heap fallback, retaining its
equal-key order for both BV trees and contour holes/diagonals. Direct native
record fixtures cover ties, integer extrema, sorted/reverse/organ-pipe and seeded
inputs across insertion/pivot thresholds. The glibc comparison uses stable
sorting, matching its normal merge-sort tie policy. The complete Linux
arm64/glibc 2.39 corpus passes with Clang 18.1.3 on Rust 1.95.0 and 1.97.1.
Windows/MSVC uses a safe record-swap implementation of UCRT ordering, including
its first-maximum selection for short arrays and median-of-three partitioning.
The direct ordered-record fixture covers both sides of the eight-record cutoff;
the BV and TileCache fixtures retain their complete byte comparisons. Other libc
implementations remain unvalidated. The BSD-licensed sort source and translation
provenance are retained separately from
the unchanged RecastNavigation reference.

The standalone TileCache driver compares every slot's compressed bytes, flags,
header, salt and next index, complete initialized obstacle union/touched/pending
storage, codec and mesh-process callback logs, and linked native navmesh bytes
following every command. Cases include all three shapes, raw cache/obstacle ref
packing, bounded hash/layer queries, exact tight/obstacle bounds, native queue
saturation and exhaustion, ignored time steps, failed rebuild advancement,
coordinate-changing callbacks, off-mesh injection and queued updates retained by
reinitialization. Allocation addresses, leaks and partially initialized objects
after allocator failure are excluded; Rust checks invalid access/conversion
inputs before they could reach undefined native behavior.

TileCache vertex welding uses the native 256-bucket hash table; Recast retains
its 4096 buckets. A boundary fixture inserts the ushort sentinel, then looks up
a vertex sharing only the low eight hash bits. Native clears the whole bucket
chain and recreates that vertex at a wrapped index; full mesh outputs must match.

## Floating-point policies

A faithful port must identify its reference compiler policy. Contraction can
change raster quantization and therefore the generated navigation geometry.

| Rust policy | Native reference |
| --- | --- |
| Default: separate multiply/add | `-fno-fast-math -ffp-contract=off` |
| `fma`: explicit fused interpolation, coordinate and query operations | Clang `-fno-fast-math -ffp-contract=on` |

Native drivers build at `-O0`. This tested contraction policy preserves function
boundaries: for example, `rcSqr` products in normal normalization remain separate.
Optimized native builds require separate validation; these results do not imply
equivalence for every inlining/contraction combination.

The x86_64 FMA oracle adds `-mfma` and requires a host with FMA support. AArch64
has native FMA instructions. The FMA runner requires Clang because contraction
policies differ between compilers. The runner supports GNU-compatible compiler
drivers, including native Windows `clang++` targeting MSVC with a Visual Studio
developer environment; MSVC-style `cl.exe` and `clang-cl.exe` drivers are not
supported. See [Windows setup](validation.md#native-windows-c-setup).

CI is configured to use Clang and both policies on Linux/macOS. Report platform
results as tested, rather than assuming them from a compatibility target. Never
hide an unexplained reference mismatch with an arbitrary epsilon.

The fractional-ramp regression exposed stale output counts when a clipped
polygon became empty. A separate tiny-triangle regression checks Detour's
edge-height fallback. These are exact behavior checks, not allowed rounding drift.

## Limits and differences

The `BuildConfig` convenience pipeline requires finite positive-extent bounds,
positive finite cell dimensions, nonnegative padding with a remaining interior,
and a grid within its explicit cell budget and signed indexing range. The default
budget is 4,194,304 cells. Low-level Recast APIs preserve signed native grid,
height/climb, radius, edge-length and region-area parameters; checked errors
cover undefined arithmetic, conversion and storage domains. There is no 16383-cell
axis cap or blanket 8191-voxel vertical-extent rejection. Contours retain signed
coordinates until native ushort mesh welding. Detour polygons have a separate
six-vertex format. Detailed per-operation domains are recorded in the coverage
ledger and usage documentation.

Detour initialization retains signed capacity metadata and every float bit.
The native 32-bit reference layout uses rounded capacities and unsigned salt
arithmetic; `try_encode` / `try_decode` check the shift domain independently.
`polyref64` selects the fixed 16/28/20-bit layout; cache/obstacle refs remain
32-bit. Native v7 tile creation/import/export and tile state have whole-buffer
comparisons. Creation checks actual BV integer conversions and retains defined
non-finite payloads. Using a stored tile still requires the arithmetic/memory
domain of the individual operation.

Native cache-layer v1 construction/decompression/endian conversion has separate
whole-buffer/cross-load fixtures with caller-supplied codecs. Streaming cache
lifecycle, deferred obstacle updates and mesh-process integration also compare
complete native state and linked mesh bytes.

Fresh query node budgets are 1–65535. Native initialization with budgets 1–3
succeeds with a zero-sized main hash table; independent/tiny-pool queries and
same-polygon shortcuts remain defined. Rust retains these operations and checks
main-table accesses that would otherwise be invalid. A zero request may reuse
existing query storage without shrinking it. Undefined memory behavior, uninitialized
data, allocator failure behavior and out-of-bounds writes are not equivalence
targets. In particular, disabling detail sampling on a nonempty mesh leaves the
native hull array uninitialized; Rust returns a checked error for zero, negative,
or NaN sampling distances. Empty meshes take the native successful early return.
Extreme input sizes still warrant further fuzzing.

C++ `qsort` does not specify the order of equal-distance contour diagonals. The
hole-merging translation uses the pinned Apple algorithm on macOS, UCRT ordering
on Windows/MSVC, stable ties elsewhere, and widened predicates; its other
safety qualifications are recorded in `THIRD_PARTY.md`. A finite corpus cannot
establish all-input or cross-platform equivalence.

High-level query policies and publication counters are conveniences, not upstream
API guarantees. See `docs/usage.md`. Implemented functionality, public API
coverage, binary formats and performance are separate compatibility claims.

## Changes

- Add a failing differential fixture before changing algorithm behavior.
- Preserve exact comparisons and intermediate ordering. A semantic comparison
  needs an explicit rationale and retained raw differences.
- Minimize random failures into named fixtures; keep seeds reproducible.
- Run the relevant suites under both floating-point policies.
- Keep convenience policies separate from reference algorithms.
- Never change the C++ reference to make incorrect Rust pass.

## Reference modes

The full corpus, README doctest and headless Bevy test can run in all four
reference/numeric configurations. Native test drivers define `DT_POLYREF64`
only when the Rust `polyref64` feature is enabled:

```sh
CXX=clang++ cargo test -p recast-nav -p bevy-recast-nav --features recast-nav/cpp-oracle,recast-nav/polyref64
CXX=clang++ cargo test -p recast-nav -p bevy-recast-nav --features recast-nav/cpp-oracle,recast-nav/polyref64,recast-nav/fma
```

The existing CI matrix covers the two numeric policies in the default reference
mode. The 64-bit runs are local evidence until separately added to CI.

## Run

```sh
python3 scripts/check_vendor.py
cargo fmt --all -- --check
cargo clippy -p recast-nav -p bevy-recast-nav --tests --examples --features recast-nav/cpp-oracle -- -D warnings
cargo test -p recast-nav -p bevy-recast-nav --features recast-nav/cpp-oracle
CXX=clang++ cargo clippy -p recast-nav -p bevy-recast-nav --tests --examples --features recast-nav/cpp-oracle,recast-nav/fma -- -D warnings
CXX=clang++ cargo test -p recast-nav -p bevy-recast-nav --features recast-nav/cpp-oracle,recast-nav/fma
cargo run -p bevy-recast-nav --example headless
```

The complete generation corpus records the native context callback stream as
well as all stage data. Polygon/detail build scopes include empty successful
builds. Direct fixtures cover zero/partial triangulation warnings, vertex-budget
rejection and a successful 65536-vertex merge with the native error diagnostic.

The declared Rust 1.95 minimum is validated locally with installed Rust 1.95.0:
complete core and optional Bevy suites, including native comparisons, pass with
all four combinations of 32/64-bit references and separate/FMA arithmetic. These use the same runtime
build target and profiles as the Rust 1.97.1 matrix. This does not establish
Windows/MSVC or hosted Linux execution evidence.

Data-domain fixtures compare initialized thin polygons, packed types 2/3, and
ignored or byte-wrapped detail metadata. Creator-only cases whose result has
zero link storage are not sent to the native loader. Native geometry calls that
would use uninitialized triangle-edge state are also excluded; thin query cases
supply initialized detail triangles. Empty Recast mesh metadata is compared
independently through polygon/detail context callbacks, including width zero.

Candidate-selection fixtures also cover NaN polygon bounds with no BV tree,
preserving native ordered min/max and overlap comparisons, and reversed BV
bounds whose quantized conversions remain zero/one. These compare bounded
polygon lists and callback batches, including filter order and zero capacities.
They do not call geometry routines on uninitialized triangle-edge domains.

Node queue fixtures call native `top` after initialized clears/final pops rather
than replacing those results based on logical emptiness. They also cover the
allocated spare pointer slot and defined null-singleton operations. Fresh
uninitialized native peeks are rejected by the fixture driver. Pool backing
records are observed and edited after clear only after their fields were
initialized, including retained position bits used by subsequent allocations.

The eight `DetourMath.h` wrappers use Rust `f32` operations and explicit
`common::floor`/`common::ceil` adapters for native signaling-NaN quieting.
Sine and cosine use separate, non-inlined call boundaries: LLVM's combined
sine/cosine call changes finite result bits and cosine NaN signs on this platform. A direct native fixture compares all returned bits/classifications for
617 input pairs, including signed zeros, quiet/signaling NaNs, infinities,
subnormals, special angles and seeded raw words. This is local standard-math
backend evidence rather than a cross-platform libm guarantee.

BV split axes follow `longestAxis(unsigned short, unsigned short, unsigned short)`:
promoted extent differences narrow to 16 bits before comparison. A minimized
mixed-extent fixture has signed differences `[-8, 2, 1]`, which become
`[65528, 2, 1]` and select X. Complete creator bytes and post-load/export bytes
retain the resulting native leaf order. Comparing signed differences alone
incorrectly selected Y even though all underlying conversions were defined.

Optimized core conformance is checked in the release profile as well as debug,
for both reference widths and separate/FMA arithmetic. Adaptive avoidance keeps
pattern additions and subtractions behind non-inlined arithmetic
boundaries. This prevents the optimizer from moving a negation from a rounded
product to its finite coefficient and retains source operation grouping.
Arithmetic quiet-NaN representation follows the field-local policy above;
non-NaN results remain bit-exact.

The allocating `NavMesh::find_path` convenience method also accepts native node
budgets 1–3: same-polygon paths succeed with a zero-sized hash table, while
operations requiring that table return a checked invalid-parameter failure.

Recast polygon snapshots include `maxpolys` and the initialized slots after the
live polygon count in `polys`, `regs` and `areas`. Rust retains those slots in
`unused_polygons`, `unused_polygon_neighbors`, `unused_regions` and
`unused_areas`, without changing the logical arrays used by algorithms. Native
merge leaves `maxpolys` at its constructor value zero despite allocating live
polygons. `copy_poly_mesh` sets it to `npolys` and copies only live data; ordinary
Rust cloning keeps the stored capacity and spare slots. Native build allocates
flags for `npolys`, so flags beyond that count are not read even when the header
comment describes a larger array.

`ContourSet::build_polygon_mesh_into` preserves output from native defined
capacity failures. At the 65,534-contour-vertex cutoff, bounds, cell scales,
border size and edge error are updated while previous counts, polygon width,
capacity, live buffers and initialized spare slots remain intact. The native
fixture checks both a fresh output and prepopulated buffers; successful builds
still replace output and emit the same scoped context events.

Crowd agent neighbor and corner storage is compared at its fixed native extent,
not only its live count. Native value-initialization starts those arrays at zero;
updates, removal and agent reuse retain initialized entries beyond the count.
The fixture edits unused slots through the real native editable-agent accessor,
including a signed negative neighbor index, NaN payloads, signed zero and the
maximum reference word, then observes subsequent updates and reuse. Standalone
`PathCorridor::find_corners_into` compares the entire supplied corner buffer.
Leading-point pruning repeats the native moves so its retained tail slots also
match; both APIs expose live prefixes separately from fixed storage.


Detail sampling retains native rounding at two sensitive decisions. The minimum
polygon extent takes a square root before its strict comparison with twice the
sample distance; a right triangle with squared extent 40.5 distinguishes this
from a squared threshold. Its regression covers adjacent representable sampling
distances as well as the boundary. Three-dimensional edge deviations evaluate
the native scalar/FMA product and addition order, including projection and
ordered clamping. A seeded rough-heightfield fixture reaches the vertex cap and
distinguishes the former extra boundary sample from the native result. Both
fixtures compare complete detail vertex bits, triangle indices/flags, context
events and heightfield state through the public detail-build API against the
unmodified pinned native implementation.

Raw tile export copies the allocated backing buffer without traversing mutable
header counts or polygon topology. The record fixture now compares polygon count
bytes 0/2/6/7/255 and all eight header layout counts at zero and all-one bits,
restoring valid records before structural access. The old export rejected both
the out-of-range polygon count and the altered header counts even though native
completed the byte copy. Traversal of invalid active prefixes remains outside
that export fixture; native fixed-slot off-mesh operations are covered below. Complete loaded snapshots with link capacities 1/2/3/4 also cover
exhaustion during internal/off-mesh attachment, including retained spare bytes,
reverse edge insertion, first-link resets and free-list ordering. These extensions
remain in the existing record-view test rather than adding redundant test families.

`Tile::data_parts` exposes a serialized header and a borrowed backing body whose
concatenation matches complete owned export. Native record fixtures compare both
after raw metadata, link, vertex, detail, triangle and BV mutations, retaining
padding, spare records and exceptional float bits. The header is a fixed snapshot;
the body borrow prevents mutation while it is observed.

TileCache scratch is bounded by the already validated byte dimensions: at most
255 sweeps/regions and `(255 + 255) * 4` raw contour vertices. An additional raw
slot preserves the existing append-before-capacity-failure sequence; successful
simplification uses only the 2,040-entry index extent. New cases extend the existing
stage fixture to 255 live sweeps, a maximum-size single region and maximum-grid
contour overflow with prior initialized contours retained. Neighbor ordering,
partial region labels, signed climb behavior, float evaluation and row-crossing
semantics retain their existing native comparisons.

Contour walks retain signed coordinates separately from the incremental flat
index. Native connections can cross row boundaries while still addressing valid
storage; emitted coordinates are narrowed only when stored. Vertex merging
compares full signed coordinates against stored byte coordinates before that
narrowing. Multiple row crossings with nonuniform heights distinguish the former
extra merge from native output. A directed in-buffer walk at the iteration limit
also covers repeated simplification extrema at zero and nonzero error; the
one-pass extrema scan retains the first native tie. These cases extend the
existing row-crossing fixture and compare complete status and contour output.

Neighborhood connectivity uses an immutable link-slice iterator and a bit mask
that can prove absence only. Possible matches retain the exact native reference
scan. The folded-corridor fixture has five active polygons with reference indices
0, 1, 66, 2 and 3: the fifth overlaps the first, whose mask bit collides with a
different connected reference. Both reference widths reject that polygon and
retain native callback, status, parent and query-state outputs. Removing the
exact fallback demonstrably fails the fixture. No geometry or connectivity cache
persists across calls or mesh mutation.

Detail-data loading preserves metadata and triangle indices that native loading
does not dereference. A direct fixture compares load/export/state mutations with
unused out-of-range triangle metadata, boundary-free triangles skipped before
vertex reads, early successful height queries with an unused triangle tail, and
unsigned triangle/detail-vertex offset wrapping into initialized backing. Only
visited records participate in geometry access; native-invalid reads remain
outside the parity domain and Rust indexing remains bounds checked.

Native loading also preserves polygon vertex/local-neighbor indices and BV leaf
or escape indices that linking does not dereference. A single-tile fixture with
no off-mesh connections compares complete loaded bytes, state store/restore,
flag/area mutations and removal with independently out-of-range indices,
including the most negative signed BV index, plus a ground polygon with zero
active edges and an initialized nonempty link pool. Geometry queries are
deliberately not run on these fixtures: their unused records establish load/lifecycle parity,
not permission for native-invalid traversal.

The unused-loading-metadata fixture also covers off-mesh polygon types before
the header's off-mesh base, with stored counts 0/1/2/4/7/255. Only load/export,
state, flags and removal are exercised on those records; native connection lookup
would assert for an index outside its connection table and is not invoked.
A separate fixture retains a valid connection table and ground-only BV tree,
then varies the connection polygon's count through 0/1/2/3/6/7/255. Native linking
and endpoint queries use its first two initialized vertex slots regardless of
that count. Non-off-mesh types 0/2/3 at the connection index also retain complete
native loading/state/removal bytes and return native endpoint failure. There
are no neighboring tiles, so oversized edge counts never enter external edge
traversal. These cases exposed both an eager INVALID_PARAM rejection and an
active-count bounds failure before fixed-slot reads were corrected.
