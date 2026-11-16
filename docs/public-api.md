# Pinned public API map

[public-api.json](public-api.json) maps declarations in all 20 public headers of
Recast, Detour, DetourTileCache and DetourCrowd at commit
`6dc1667f580357e8a2154c28b7867bea7e8ad3a7`. Each entry records its native source
file, line and byte offset, and its Rust counterpart or explicit adaptation.
Header SHA-256 hashes are part of the record.

| Declaration group | Recorded entries |
| --- | ---: |
| Public callables and protected Recast context hooks | 509 |
| Public fields | 361 |
| Named constants | 130 |
| Type-alias declarations | 18 |
| Operational macro definitions | 9 |

Overloads and const/non-const methods remain separate entries. Constructors,
destructors, allocator wrappers and public container methods are included.
Clang-generated implicit members, instantiated template duplicates, private
members and include guards are excluded. Nine macro entries include alternative
definitions of assertion and branch-hint macros. The 18 type-alias entries
include both physical declarations of each width-dependent reference typedef.
The map covers both `DT_POLYREF64` settings with `DT_VIRTUAL_QUERYFILTER` enabled
to include the optional virtual filter destructor and extension points.

Re-extract and compare the native declarations with the reviewed record:

```sh
python3 scripts/check_public_api.py
```

The script requires Clang with JSON AST output. It compiles no executable, uses
temporary files, verifies source tokens against the actual header bytes, and
checks both reference widths. `--clang PATH` selects a Clang driver. This is an
audit tool; ordinary Rust runtime builds have no Clang dependency. The script
checks declaration coverage and source anchors. Rust target descriptions are
reviewed manually and are not resolved or type-checked by this script.

The map is an inventory, not proof of equivalence. The [coverage ledger](coverage.md)
records behavioral domains and differential evidence. A `mapped` callable has a
corresponding Rust operation; that label does not assert identical signatures,
all-input behavior or C ABI compatibility. Slash-separated targets identify
alternatives within the same Rust module or grouped native overloads.

## Explicit adaptations

Rust ownership replaces native pointer allocation/free, allocator hints,
allocator-vector containers, raw attached-mesh pointers, and TileCache temporary
allocator callbacks. Native global allocator/assertion hooks and their callback
sequences are not exposed. Native allocation-failure intermediate state is not a
general guarantee. Node memory-usage accessors report Rust storage, so their byte
totals differ from native `sizeof` totals. Node pointer/index conversion uses
one-based Rust handles. Queries take an explicit mesh borrow on each operation.

Array assignment, Rust swaps, checked slices and native-format readers replace
pointer/template utilities. Context byte logging preserves already formatted
bytes; Rust formatting replaces the C variadic `printf` interface. Native
assertion-violating, uninitialized-memory, invalid-pointer, nonterminating and
undefined conversion/arithmetic domains use checked Rust contracts where
described in the ledger. Native `rcVectorBase::operator=` is declared publicly
but intentionally has no upstream definition; the map labels it `native-disabled`.

Public field entries describe data and access, not C++ object layout or arbitrary
pointer mutation. Counts often become slice/vector lengths; unions become an
explicit shape buffer; packed fields use typed accessors or bounded values.
Mutations must preserve the Rust operation's valid storage and index invariants.

Detour polygons use `Tile::polygon_count`, `polygon`, `polygons`, `polygon_mut`,
`polygons_mut` and `set_polygon` instead of a decoded public vector.
`FilterPolygon::polygon` is a borrowed `nav_data::PolygonView`: scalar fields have
accessors, active vertices/neighbors are iterators of `u16` values, and indexed
reads use `vertex`/`neighbor`. `to_owned` preserves all six words, including spare
slots. `PolygonMut` writes flags, packed area/type and link heads in place;
`set_polygon` replaces a complete record. Invalid indices return None/error
without mutation. Record counts remain fixed; restore valid topology before
queries. The owned `nav_data::Polygon` creation/inspection type remains available.

Detour tile links use `Tile::link_count`, `Tile::link`, `Tile::set_link` and
`Tile::links` instead of a decoded public vector. The iterator yields borrowed
`nav_data::LinkView` records with field accessors; indexed reads return owned
`nav::Link` values. Checked writes update the owned native-format backing,
including initialized free records, without allocating a second link array.
Detail triangles use fixed-length borrowed slices through `detail_triangles` and
`detail_triangles_mut`. BV nodes use `bv_nodes`, `bv_node_count`, `bv_node` and
`set_bv_node`; these read or mutate the original native records without a decoded
array. Record counts stay fixed, as in the attached native tile buffer. Vertices,
extra detail vertices, detail descriptors and off-mesh connections likewise use
count, iterator, indexed-read and indexed-write methods. Reads return owned
fixed-size values; writes preserve native padding and leave bytes intact on an
invalid index. `NavMesh::off_mesh_connection` also returns an owned record.
The former public geometry vectors are no longer independently resizable.

Native pointer aliasing, bitfield assignment syntax and raw object ABI are not
claimed. Fixed native file formats are handled separately by byte readers and
writers and are covered by interoperability fixtures.

The audit found and closed missing public configuration units/types, polygon
capacity and initialized spare storage, complete initialized Crowd neighbor and
corner arrays, public flag/capacity constants, and scalar helper template
instantiations beyond `f32`. The operation streams compare
retained storage as well as live results. Remaining platform and input-domain
evidence is tracked in the ledger; these inventory counts do not close it.
