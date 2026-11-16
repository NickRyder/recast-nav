#![cfg(feature = "cpp-oracle")]
#[path = "support/float_comparison.rs"]
mod float_comparison;
#[path = "support/parity_trace.rs"]
mod parity_trace;
use recast_nav::{crowd::ProximityGrid, nav::*};
use serde_json::{Value, json};
use std::{
    fmt::Write as _,
    io::Write as _,
    process::{Command, Stdio},
};
fn oracle(input: String) -> Value {
    let mut child = Command::new(env!("CROWD_ORACLE"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    let mut stdin = child.stdin.take().unwrap();
    let out = std::thread::scope(|scope| {
        let writer = scope.spawn(move || stdin.write_all(input.as_bytes()).unwrap());
        let out = child.wait_with_output().unwrap();
        writer.join().unwrap();
        out
    });
    assert!(out.status.success());
    serde_json::from_slice(&out.stdout).unwrap()
}
enum Op {
    Add(u16, usize, [[f32; 2]; 2]),
    Query([[f32; 2]; 2], usize),
    Count(i32, i32),
    Clear,
    Init(usize, f32),
}
fn grid_snapshot(grid: &ProximityGrid) -> Value {
    json!([grid.bounds(), grid.cell_size().to_bits()])
}
fn check_grid(size: usize, cs: f32, ops: Vec<Op>) {
    let mut grid = ProximityGrid::new(size, cs).unwrap();
    let mut input = format!("grid {size} {} {} ", cs.to_bits(), ops.len());
    let mut actual = vec![json!([null, grid_snapshot(&grid)])];
    let mut ids = Vec::new();
    let mut backing = [0xface; 32];
    for op in ops {
        let value = match op {
            Op::Add(id, count, b) => {
                write!(input, "A {id} {count} ").unwrap();
                for f in b.into_iter().flatten() {
                    write!(input, "{} ", f.to_bits()).unwrap();
                }
                for _ in 0..count {
                    assert_eq!(grid.add_item(id, b), SUCCESS);
                }
                Value::Null
            }
            Op::Query(b, cap) => {
                input.push_str("B ");
                for f in b.into_iter().flatten() {
                    write!(input, "{} ", f.to_bits()).unwrap();
                }
                write!(input, "{cap} ").unwrap();
                grid.query_items_into(b, cap, &mut ids).unwrap();
                let count = grid.query_items_slice(b, &mut backing[..cap]).unwrap();
                assert_eq!(&ids, &backing[..count]);
                json!([count, backing])
            }
            Op::Count(x, y) => {
                write!(input, "N {x} {y} ").unwrap();
                json!(grid.item_count_at(x, y).unwrap())
            }
            Op::Clear => {
                input.push_str("C ");
                grid.clear();
                Value::Null
            }
            Op::Init(n, c) => {
                write!(input, "I {n} {} ", c.to_bits()).unwrap();
                grid.reinitialize(n, c).unwrap();
                json!(1)
            }
        };
        actual.push(json!([value, grid_snapshot(&grid)]));
    }
    let expected = oracle(input);
    for (i, a) in actual.into_iter().enumerate() {
        assert_eq!(
            a, expected[i],
            "proximity grid operation {i}, pool={size}, cs={cs}"
        );
    }
}
#[test]
fn native_crowd_proximity_grid_order_duplicates_exhaustion_and_reinitialization() {
    for pool in [1, 7, 32, 128] {
        for cs in [0.3, 1., 2.7, f32::INFINITY] {
            let mut ops = vec![
                Op::Query([[-2., -2.], [2., 2.]], 0),
                Op::Add(65535, 1, [[1., 1.], [-1., -1.]]),
            ];
            let mut seed = 0x5728ba19u32;
            for i in 0..80 {
                let mut next = || {
                    seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                    ((seed >> 16) % 100) as f32 * 0.02 - 1.
                };
                let x = next();
                let y = next();
                let w = next().abs();
                let h = next().abs();
                let id = match i % 17 {
                    13 => 63,
                    14 => 64,
                    15 => u16::MAX,
                    _ => (i % 13) as u16,
                };
                ops.push(Op::Add(id, 1, [[x, y], [x + w, y + h]]));
                ops.push(Op::Count(i % 7 - 3, i % 5 - 2));
                for cap in [0, 1, 4, 20] {
                    ops.push(Op::Query([[-2., -2.], [2., 2.]], cap));
                }
                if i % 17 == 16 {
                    ops.push(Op::Clear);
                }
            }
            ops.push(Op::Init(7, cs));
            ops.push(Op::Add(17, 1, [[0., 0.], [0., 0.]]));
            ops.push(Op::Count(0, 0));
            ops.push(Op::Query([[0., 0.], [0., 0.]], 7));
            ops.push(Op::Clear);
            ops.push(Op::Query([[0., 0.], [0., 0.]], 7));
            check_grid(pool, cs, ops);
        }
    }
    // The last u16 pool index is the native sentinel. Its insertion hides this
    // bucket, despite the memory access itself remaining within the pool.
    let point = [[0.; 2]; 2];
    check_grid(
        65536,
        1.,
        vec![
            Op::Add(42, 65535, point),
            Op::Count(0, 0),
            Op::Query(point, 2),
            Op::Add(43, 1, point),
            Op::Count(0, 0),
            Op::Query(point, 2),
            Op::Clear,
            Op::Add(44, 1, point),
            Op::Query(point, 2),
        ],
    );
}
#[test]
fn proximity_grid_checks_undefined_conversion_and_nonterminating_chain_domains() {
    assert_eq!(
        ProximityGrid::new(0, 1.).unwrap_err(),
        FAILURE | INVALID_PARAM
    );
    let mut grid = ProximityGrid::new(65537, 1.).unwrap();
    let original = grid.bounds();
    assert_eq!(
        grid.add_item(1, [[f32::NAN, 0.], [0., 0.]]),
        FAILURE | INVALID_PARAM
    );
    assert_eq!(grid.bounds(), original);
    let point = [[0.; 2]; 2];
    for _ in 0..65535 {
        assert_eq!(grid.add_item(1, point), SUCCESS);
    }
    assert_eq!(grid.add_item(2, [[1.; 2]; 2]), SUCCESS);
    assert_eq!(grid.add_item(1, point), SUCCESS);
    assert_eq!(grid.query_items(point, 2), Err(FAILURE | INVALID_PARAM));
    assert_eq!(grid.item_count_at(0, 0), Err(FAILURE | INVALID_PARAM));
    grid.clear();
    assert_eq!(grid.query_items(point, 2), Ok(vec![]));
}

use recast_nav::{
    crowd::LocalBoundary,
    nav_data::{self, CreateOffMeshConnection, CreatePolygon, OwnedCreateParams},
    query::NavMeshQuery,
};
fn mesh_tile(n: usize, x: i32, off_mesh: bool) -> Vec<u8> {
    let mut p = OwnedCreateParams {
        vertices_per_polygon: 6,
        coordinate: [x, 0, 0],
        bounds: [
            [-1.3 + x as f32 * n as f32 * 0.3, 0.13, 2.1],
            [
                -1.3 + (x + 1) as f32 * n as f32 * 0.3,
                5.,
                2.1 + n as f32 * 0.3,
            ],
        ],
        cell_size: 0.3,
        cell_height: 0.2,
        walkable_height: 1.8,
        walkable_radius: 0.3,
        walkable_climb: 1.,
        build_bv_tree: x == 0,
        ..OwnedCreateParams::default()
    };
    for z in 0..=n {
        for vx in 0..=n {
            p.vertices.push([
                vx as u16,
                ((vx + x as usize * n + z * 2) % 3) as u16,
                z as u16,
            ]);
        }
    }
    for z in 0..n {
        for x in 0..n {
            let v = (x + z * (n + 1)) as u16;
            let i = x + z * n;
            p.polygons.push(CreatePolygon {
                vertices: [
                    v,
                    v + (n + 1) as u16,
                    v + (n + 2) as u16,
                    v + 1,
                    u16::MAX,
                    u16::MAX,
                ],
                neighbors: [
                    if x > 0 { (i - 1) as u16 } else { 0x8000 },
                    if z + 1 < n { (i + n) as u16 } else { 0x8001 },
                    if x + 1 < n { (i + 1) as u16 } else { 0x8002 },
                    if z > 0 { (i - n) as u16 } else { 0x8003 },
                    u16::MAX,
                    u16::MAX,
                ],
                flags: 1 << ((x + z) % 3),
                area: ((x + z) % 3) as u8,
            });
        }
    }
    if off_mesh {
        p.off_mesh_connections.push(CreateOffMeshConnection {
            positions: [
                [-1.15, 0.5, 2.25],
                [
                    -1.3 + n as f32 * 0.3 - 0.15,
                    0.5,
                    2.1 + n as f32 * 0.3 - 0.15,
                ],
            ],
            radius: 0.7,
            flags: 1,
            area: 2,
            direction: 1,
        });
        p.off_mesh_user_ids = Some(vec![37]);
    }
    nav_data::create(&p.as_params()).unwrap()
}
enum BoundaryOp {
    Update(PolyRef, Point, f32),
    Reset,
    Filter(u16, u16),
    Remove(TileRef),
    Flags(PolyRef, u16),
}
fn boundary_snapshot(
    boundary: &LocalBoundary,
    mesh: &NavMesh,
    query: &NavMeshQuery,
    filter: &Filter,
) -> Value {
    json!([
        boundary.center().map(f32::to_bits),
        boundary.segment_count(),
        (0..8)
            .map(|i| boundary.segment(i).map(|s| s
                .positions
                .into_iter()
                .flatten()
                .map(f32::to_bits)
                .collect::<Vec<_>>()))
            .collect::<Vec<_>>(),
        u8::from(boundary.is_valid(mesh, query, filter))
    ])
}
#[test]
fn native_crowd_local_boundary_segments_ties_filters_resets_and_stale_refs() {
    for n in [3, 8] {
        for off_mesh in [false, true] {
            let tiles = [mesh_tile(n, 0, off_mesh), mesh_tile(n, 1, false)];
            let origin = [-1.3, 0.13, 2.1];
            let tile_width = n as f32 * 0.3;
            let mut mesh = NavMesh::new(origin, tile_width, tile_width, 4, 128).unwrap();
            let mut input = "boundary ".to_owned();
            for f in origin.into_iter().chain([tile_width, tile_width]) {
                write!(input, "{} ", f.to_bits()).unwrap();
            }
            input.push_str("4 128 128 2 ");
            for data in &tiles {
                write!(input, "{} ", data.len()).unwrap();
                for b in data {
                    write!(input, "{b} ").unwrap();
                }
                mesh.add_tile_data(data, 0).unwrap();
            }
            let start = mesh.reference(0, 0);
            let other = mesh.reference(1, 0);
            let mut ops = vec![
                BoundaryOp::Update(start, [-1.15, 0.7, 2.25], 10.),
                BoundaryOp::Reset,
                BoundaryOp::Update(start, [-1.3, 0.7, 2.1], 10.),
                BoundaryOp::Update(start, [-1.15, 0.7, 2.25], 0.),
            ];
            if off_mesh {
                ops.push(BoundaryOp::Update(
                    mesh.reference(0, n * n),
                    [-1.15, 0.7, 2.25],
                    0.7,
                ));
            }
            let mut seed = 0x49275ae1u32;
            for i in 0..48 {
                seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                let px = (seed >> 16) as usize % n;
                let pz = (seed >> 24) as usize % n;
                let point = [
                    -1.3 + (px as f32 + 0.3) * 0.3,
                    0.13 + i as f32 * 0.1,
                    2.1 + (pz as f32 + 0.7) * 0.3,
                ];
                ops.push(BoundaryOp::Filter(
                    [0xffff, 1, 3, 7][i % 4],
                    if i % 9 == 0 { 4 } else { 0 },
                ));
                ops.push(BoundaryOp::Update(
                    mesh.reference(0, px + pz * n),
                    point,
                    [0., 0.3, 0.73, 10.][i % 4],
                ));
                if i % 7 == 0 {
                    ops.push(BoundaryOp::Reset);
                }
            }
            ops.extend([
                BoundaryOp::Filter(0xffff, 0),
                BoundaryOp::Update(start, [-1.15, 0.7, 2.25], 1.),
                BoundaryOp::Flags(start, 0),
                BoundaryOp::Flags(start, 1),
                BoundaryOp::Update(start, [f32::NAN, 0., 0.], 1.),
                BoundaryOp::Update(start, [-1.15, 0.7, 2.25], -1.),
                BoundaryOp::Update(0, [f32::NAN; 3], f32::NAN),
                BoundaryOp::Update(other, [-1.3 + tile_width + 0.15, 0.7, 2.25], 0.7),
                BoundaryOp::Remove(mesh.reference(1, 0)),
                BoundaryOp::Update(other, [0.; 3], 1.),
                BoundaryOp::Update(start, [-1.15, 0.7, 2.25], 10.),
                BoundaryOp::Reset,
            ]);
            let mut query = NavMeshQuery::new(128).unwrap();
            let mut filter = Filter::default();
            let mut boundary = LocalBoundary::default();
            let mut actual = vec![json!([
                null,
                boundary_snapshot(&boundary, &mesh, &query, &filter)
            ])];
            write!(input, "{} ", ops.len()).unwrap();
            for op in ops {
                let value = match op {
                    BoundaryOp::Update(r, p, range) => {
                        write!(input, "U {r} ").unwrap();
                        for f in p.into_iter().chain([range]) {
                            write!(input, "{} ", f.to_bits()).unwrap();
                        }
                        boundary.update(r, p, range, &mesh, &mut query, &filter);
                        Value::Null
                    }
                    BoundaryOp::Reset => {
                        input.push_str("R ");
                        boundary.reset();
                        Value::Null
                    }
                    BoundaryOp::Filter(inc, exc) => {
                        write!(input, "F {inc} {exc} ").unwrap();
                        filter.include = inc;
                        filter.exclude = exc;
                        Value::Null
                    }
                    BoundaryOp::Remove(r) => {
                        write!(input, "D {r} ").unwrap();
                        json!(match mesh.remove_tile_data(r) {
                            Ok(_) => SUCCESS,
                            Err(e) => e,
                        })
                    }
                    BoundaryOp::Flags(r, flags) => {
                        write!(input, "G {r} {flags} ").unwrap();
                        json!(mesh.set_poly_flags(r, flags))
                    }
                };
                actual.push(json!([
                    value,
                    boundary_snapshot(&boundary, &mesh, &query, &filter)
                ]));
            }
            assert_eq!(
                actual[1][1][1], 8,
                "fixture must fill the eight-segment boundary"
            );
            let expected = oracle(input);
            for (i, a) in actual.into_iter().enumerate() {
                assert_eq!(
                    a, expected[i],
                    "local boundary operation {i}, n={n}, offmesh={off_mesh}"
                );
            }
        }
    }
}

use recast_nav::crowd::PathQueue;
#[path = "support/node_snapshot.rs"]
mod node_snapshot;
enum QueueOp {
    Request(PolyRef, PolyRef, Point, Point, Option<usize>),
    Update(i32),
    Status(u32),
    Result(u32, usize),
    Filter(usize, u16, u16, usize, f32),
    Remove(TileRef),
    Init(usize, usize),
}
struct QueueFixture {
    mesh: NavMesh,
    queue: PathQueue<usize>,
    filters: [Filter; 3],
    issued: Vec<u32>,
    input: String,
    commands: Vec<String>,
    actual: Vec<Value>,
}
impl QueueFixture {
    fn new(path_size: usize, nodes: usize, offmesh: bool) -> Self {
        let n = 8;
        let tiles = [mesh_tile(n, 0, offmesh), mesh_tile(n, 1, false)];
        let origin = [-1.3, 0.13, 2.1];
        let width = n as f32 * 0.3;
        let mut input = "queue ".to_owned();
        for f in origin.into_iter().chain([width, width]) {
            write!(input, "{} ", f.to_bits()).unwrap();
        }
        write!(input, "4 128 {nodes} 2 ").unwrap();
        let mut mesh = NavMesh::new(origin, width, width, 4, 128).unwrap();
        for data in tiles {
            write!(input, "{} ", data.len()).unwrap();
            for b in &data {
                write!(input, "{b} ").unwrap();
            }
            mesh.add_tile_data(&data, 0).unwrap();
        }
        write!(input, "{path_size} ").unwrap();
        let mut f = Self {
            mesh,
            queue: PathQueue::new(path_size, nodes).ok().unwrap(),
            filters: std::array::from_fn(|_| Filter::default()),
            issued: vec![],
            input,
            commands: vec![],
            actual: vec![],
        };
        f.actual.push(json!([null, f.snapshot()]));
        f
    }
    fn snapshot(&self) -> Value {
        json!([
            self.issued
                .iter()
                .map(|&r| self.queue.request_status(r))
                .collect::<Vec<_>>(),
            node_snapshot::snapshot(self.queue.query().node_pool())
        ])
    }
    fn op(&mut self, op: QueueOp) -> Value {
        let mut s = String::new();
        let result = match op {
            QueueOp::Request(a, b, p, q, filter) => {
                write!(s, "Q {a} {b} ").unwrap();
                for f in p.into_iter().chain(q) {
                    write!(s, "{} ", f.to_bits()).unwrap();
                }
                write!(s, "{} ", filter.map_or(-1, |i| i as i32)).unwrap();
                let r = self.queue.request(a, b, p, q, filter);
                if r != 0 {
                    self.issued.push(r);
                }
                json!(r)
            }
            QueueOp::Update(n) => {
                write!(s, "U {n} ").unwrap();
                self.queue.update(&self.mesh, n, |&i| &self.filters[i]);
                Value::Null
            }
            QueueOp::Status(r) => {
                write!(s, "S {r} ").unwrap();
                json!(self.queue.request_status(r))
            }
            QueueOp::Result(r, cap) => {
                write!(s, "P {r} {cap} ").unwrap();
                let mut path = vec![0xface; cap + 2];
                let status = self.queue.path_result_into(r, cap, &mut path);
                json!([status, path.len(), path])
            }
            QueueOp::Filter(i, inc, exc, area, cost) => {
                write!(s, "F {i} {inc} {exc} {area} {} ", cost.to_bits()).unwrap();
                self.filters[i].include = inc;
                self.filters[i].exclude = exc;
                self.filters[i].area_costs[area] = cost;
                Value::Null
            }
            QueueOp::Remove(r) => {
                write!(s, "D {r} ").unwrap();
                json!(match self.mesh.remove_tile_data(r) {
                    Ok(_) => SUCCESS,
                    Err(s) => s,
                })
            }
            QueueOp::Init(size, nodes) => {
                write!(s, "I {size} {nodes} ").unwrap();
                self.queue.reinitialize(size, nodes).unwrap();
                json!(1)
            }
        };
        self.commands.push(s);
        self.actual.push(json!([result, self.snapshot()]));
        result
    }
    fn check(mut self) {
        write!(self.input, "{} ", self.commands.len()).unwrap();
        for s in &self.commands {
            self.input.push_str(s);
        }
        let expected = oracle(self.input);
        assert_eq!(self.actual.len(), expected.as_array().unwrap().len());
        for (i, a) in self.actual.into_iter().enumerate() {
            assert_eq!(
                a,
                expected[i],
                "path queue operation {i}: {}",
                i.checked_sub(1).map_or("init", |j| self.commands[j]
                    .split_whitespace()
                    .next()
                    .unwrap())
            );
        }
    }
}
#[test]
fn native_crowd_path_queue_budgets_filter_identity_expiry_and_result_lifecycle() {
    for path_size in [0, 1, 5, 128] {
        for nodes in [4, 128] {
            for offmesh in [false, true] {
                let mut f = QueueFixture::new(path_size, nodes, offmesh);
                let start = f.mesh.reference(0, 0);
                let end = f.mesh.reference(1, 63);
                let p = [-1.15, 0.7, 2.25];
                let q = [3.35, 0.7, 4.35];
                for (a, b, pa, pb, filter) in [
                    (start, end, p, q, Some(0)),
                    (start, start, p, p, Some(1)),
                    (start, end, p, q, None),
                    (0, end, p, q, Some(0)),
                    (start, end, [f32::NAN; 3], q, Some(2)),
                    (end, start, q, p, Some(2)),
                    (start, f.mesh.reference(0, 63), p, q, Some(0)),
                    (
                        if offmesh {
                            f.mesh.reference(0, 64)
                        } else {
                            end
                        },
                        start,
                        p,
                        p,
                        Some(1),
                    ),
                ] {
                    f.op(QueueOp::Request(a, b, pa, pb, filter));
                }
                assert_eq!(f.op(QueueOp::Request(start, end, p, q, Some(0))), 0);
                f.op(QueueOp::Update(0));
                f.op(QueueOp::Update(-1));
                f.op(QueueOp::Filter(0, 3, 0, 2, 3.75));
                f.op(QueueOp::Update(1));
                f.op(QueueOp::Update(2));
                f.op(QueueOp::Result(1, 2));
                f.op(QueueOp::Update(3));
                let new = f
                    .op(QueueOp::Request(start, end, p, q, Some(0)))
                    .as_u64()
                    .unwrap() as u32;
                assert_eq!(new, 9);
                f.op(QueueOp::Filter(2, 7, 0, 1, 0.125));
                f.op(QueueOp::Update(1000));
                f.op(QueueOp::Result(2, 0));
                f.op(QueueOp::Result(2, 4));
                f.op(QueueOp::Result(3, 3));
                f.op(QueueOp::Result(new, 128));
                // First free slot is initialized; native permits a zero-handle read of
                // its retained result after release. Fresh uninitialized slots excluded.
                f.op(QueueOp::Status(0));
                f.op(QueueOp::Result(0, 128));
                for _ in 0..3 {
                    f.op(QueueOp::Update(0));
                }
                f.op(QueueOp::Status(6));
                f.op(QueueOp::Result(6, 128));
                f.op(QueueOp::Request(start, end, p, q, None));
                f.op(QueueOp::Update(1));
                f.op(QueueOp::Init(5, 17));
                f.op(QueueOp::Status(0));
                let r = f
                    .op(QueueOp::Request(start, end, p, q, Some(0)))
                    .as_u64()
                    .unwrap() as u32;
                assert_eq!(r, 11);
                f.op(QueueOp::Update(1));
                f.op(QueueOp::Remove(f.mesh.reference(1, 0)));
                f.op(QueueOp::Update(3));
                f.op(QueueOp::Result(r, 128));
                let r = f
                    .op(QueueOp::Request(start, start, p, p, Some(0)))
                    .as_u64()
                    .unwrap() as u32;
                f.op(QueueOp::Update(0));
                f.op(QueueOp::Result(r, 1));
                f.check();
            }
        }
    }
}

use recast_nav::crowd::{
    OffMeshMove, PathCorridor, merge_corridor_end_moved, merge_corridor_start_moved,
    merge_corridor_start_shortcut,
};
#[test]
fn native_crowd_corridor_merges_preserve_duplicate_selection_capacity_and_buffer_tails() {
    let mut cases = vec![];
    let mut seed = 0x57ab8219u32;
    for _ in 0..160 {
        let mut next = || {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            seed
        };
        let count = (next() % 9) as usize;
        let vn = (next() % 9) as usize;
        let max = count.max(vn) + (next() % 3) as usize;
        let path = (0..32).map(|_| (next() % 7) as PolyRef).collect::<Vec<_>>();
        let visited = (0..vn).map(|_| (next() % 7) as PolyRef).collect::<Vec<_>>();
        for mode in 0..3 {
            cases.push((mode, path.clone(), count, max, visited.clone()));
        }
    }
    for (mode, max, visited) in [
        (0, 2, vec![1]),
        (2, 2, vec![99, 3]),
        (1, 2, vec![1, 99]),
        (1, 0, vec![99]),
        (0, 100, vec![99, 3]),
    ] {
        cases.push((mode, vec![1, 2, 3, 4, 5, 6, 77, 88], 6, max, visited));
    }
    let mut input = format!("merges {} ", cases.len());
    let mut actual = vec![];
    for (mode, mut path, count, max, visited) in cases {
        write!(
            input,
            "{mode} {} {count} {max} {} ",
            path.len(),
            visited.len()
        )
        .unwrap();
        for p in path.iter().chain(&visited) {
            write!(input, "{p} ").unwrap();
        }
        let count = match mode {
            0 => merge_corridor_start_moved(&mut path, count, max, &visited),
            1 => merge_corridor_end_moved(&mut path, count, max, &visited),
            _ => merge_corridor_start_shortcut(&mut path, count, max, &visited),
        }
        .unwrap();
        actual.push(json!([count, path]));
    }
    let expected = oracle(input);
    for (i, a) in actual.into_iter().enumerate() {
        assert_eq!(a, expected[i], "corridor merge fixture {i}");
    }
}

enum CorridorOp {
    Reset(PolyRef, Point),
    Set(Point, Vec<PolyRef>),
    Corners(usize),
    Visibility(Point, f32),
    Topology,
    Move(Point),
    Target(Point),
    Filter(u16, u16),
    Flags(PolyRef, u16),
    Remove(TileRef),
    Valid(i32),
    Fix(PolyRef, Point),
    Trim(PolyRef, Point),
    Offmesh(PolyRef),
}
struct CorridorFixture {
    mesh: NavMesh,
    query: NavMeshQuery,
    corridor: PathCorridor,
    filter: Filter,
    input: String,
    commands: Vec<String>,
    actual: Vec<Value>,
}
fn append_points(s: &mut String, points: impl IntoIterator<Item = f32>) {
    for f in points {
        write!(s, "{} ", f.to_bits()).unwrap();
    }
}
impl CorridorFixture {
    fn new(capacity: usize, nodes: usize, offmesh: bool) -> Self {
        Self::with_size(capacity, nodes, offmesh, 8)
    }
    fn with_size(capacity: usize, nodes: usize, offmesh: bool, n: usize) -> Self {
        let tiles = [mesh_tile(n, 0, offmesh), mesh_tile(n, 1, false)];
        let origin = [-1.3, 0.13, 2.1];
        let width = n as f32 * 0.3;
        let mut input = "corridor ".to_owned();
        append_points(&mut input, origin.into_iter().chain([width, width]));
        let max_polys = (n * n + 1).next_power_of_two();
        write!(input, "4 {max_polys} {nodes} 2 ").unwrap();
        let mut mesh = NavMesh::new(origin, width, width, 4, max_polys).unwrap();
        for data in tiles {
            write!(input, "{} ", data.len()).unwrap();
            for b in &data {
                write!(input, "{b} ").unwrap();
            }
            mesh.add_tile_data(&data, 0).unwrap();
        }
        let start = mesh.reference(0, 0);
        let pos = [-1.15, 0.7, 2.25];
        write!(input, "{capacity} {start} ").unwrap();
        append_points(&mut input, pos);
        let mut corridor = PathCorridor::new(capacity).unwrap();
        assert!(corridor.reset(start, pos));
        let mut f = Self {
            mesh,
            query: NavMeshQuery::new(nodes).unwrap(),
            corridor,
            filter: Filter::default(),
            input,
            commands: vec![],
            actual: vec![],
        };
        f.actual.push(json!([null, f.snapshot()]));
        f
    }
    fn snapshot(&self) -> Value {
        json!([
            self.corridor.position().map(f32::to_bits),
            self.corridor.target().map(f32::to_bits),
            self.corridor.path(),
            self.corridor.first_polygon(),
            self.corridor.last_polygon(),
            node_snapshot::snapshot(self.query.node_pool())
        ])
    }
    fn op(&mut self, op: CorridorOp) -> Value {
        let mut s = String::new();
        let value = match op {
            CorridorOp::Reset(r, p) => {
                write!(s, "R {r} ").unwrap();
                append_points(&mut s, p);
                assert!(self.corridor.reset(r, p));
                Value::Null
            }
            CorridorOp::Set(p, path) => {
                s.push_str("S ");
                append_points(&mut s, p);
                write!(s, "{} ", path.len()).unwrap();
                for r in &path {
                    write!(s, "{r} ").unwrap();
                }
                assert!(self.corridor.set_corridor(p, &path));
                Value::Null
            }
            CorridorOp::Corners(n) => {
                write!(s, "C {n} ").unwrap();
                let mut corners = vec![recast_nav::crowd::Corner::default(); n];
                let count = self.corridor.find_corners_into(
                    &self.mesh,
                    &self.query,
                    &mut corners,
                    &mut StraightPath::default(),
                );
                json!([
                    count,
                    corners
                        .iter()
                        .map(|c| json!([c.position.map(f32::to_bits), c.flags, c.reference]))
                        .collect::<Vec<_>>()
                ])
            }
            CorridorOp::Visibility(p, r) => {
                s.push_str("V ");
                append_points(&mut s, p.into_iter().chain([r]));
                self.corridor
                    .optimize_visibility(p, r, &self.mesh, &self.query, &self.filter);
                Value::Null
            }
            CorridorOp::Topology => {
                s.push_str("T ");
                json!(u8::from(self.corridor.optimize_topology(
                    &self.mesh,
                    &mut self.query,
                    &self.filter
                )))
            }
            CorridorOp::Move(p) => {
                s.push_str("M ");
                append_points(&mut s, p);
                json!(u8::from(self.corridor.move_position(
                    p,
                    &self.mesh,
                    &mut self.query,
                    &self.filter
                )))
            }
            CorridorOp::Target(p) => {
                s.push_str("E ");
                append_points(&mut s, p);
                json!(u8::from(self.corridor.move_target_position(
                    p,
                    &self.mesh,
                    &mut self.query,
                    &self.filter
                )))
            }
            CorridorOp::Filter(a, b) => {
                write!(s, "F {a} {b} ").unwrap();
                self.filter.include = a;
                self.filter.exclude = b;
                Value::Null
            }
            CorridorOp::Flags(r, v) => {
                write!(s, "G {r} {v} ").unwrap();
                json!(self.mesh.set_poly_flags(r, v))
            }
            CorridorOp::Remove(r) => {
                write!(s, "D {r} ").unwrap();
                json!(match self.mesh.remove_tile_data(r) {
                    Ok(_) => SUCCESS,
                    Err(s) => s,
                })
            }
            CorridorOp::Valid(n) => {
                write!(s, "H {n} ").unwrap();
                json!(u8::from(self.corridor.is_valid(
                    n,
                    &self.mesh,
                    &self.query,
                    &self.filter
                )))
            }
            CorridorOp::Fix(r, p) => {
                write!(s, "X {r} ").unwrap();
                append_points(&mut s, p);
                json!(u8::from(self.corridor.fix_path_start(r, p)))
            }
            CorridorOp::Trim(r, p) => {
                write!(s, "I {r} ").unwrap();
                append_points(&mut s, p);
                json!(u8::from(self.corridor.trim_invalid_path(
                    r,
                    p,
                    &self.mesh,
                    &self.query,
                    &self.filter
                )))
            }
            CorridorOp::Offmesh(r) => {
                write!(s, "O {r} ").unwrap();
                let mut output = OffMeshMove {
                    references: [0xface, 0xfade],
                    positions: [[-17., -19., -23.], [29., 31., 37.]],
                };
                let ok =
                    self.corridor
                        .move_over_off_mesh_connection_into(r, &self.mesh, &mut output);
                json!([
                    u8::from(ok),
                    output.references,
                    output.positions[0].map(f32::to_bits),
                    output.positions[1].map(f32::to_bits)
                ])
            }
        };
        self.commands.push(s);
        self.actual.push(json!([value, self.snapshot()]));
        value
    }
    fn check(mut self) {
        write!(self.input, "{} ", self.commands.len()).unwrap();
        for s in &self.commands {
            self.input.push_str(s);
        }
        let expected = oracle(self.input);
        assert_eq!(self.actual.len(), expected.as_array().unwrap().len());
        for (i, a) in self.actual.into_iter().enumerate() {
            assert_eq!(
                a,
                expected[i],
                "corridor operation {i}: {}",
                i.checked_sub(1).map_or("init", |j| self.commands[j]
                    .split_whitespace()
                    .next()
                    .unwrap())
            );
        }
    }
}
#[test]
fn native_crowd_maintained_corridors_movement_optimization_and_offmesh_pruning() {
    for capacity in [32, 64] {
        for nodes in [4, 128] {
            for offmesh in [false, true] {
                let mut f = CorridorFixture::new(capacity, nodes, offmesh);
                let start = f.mesh.reference(0, 0);
                let end = f.mesh.reference(1, 63);
                let p = [-1.15, 0.7, 2.25];
                let q = [3.35, 0.7, 4.35];
                let winding: Vec<_> = (0..8)
                    .map(|i| f.mesh.reference(0, i))
                    .chain((1..8).map(|z| f.mesh.reference(0, 7 + z * 8)))
                    .chain((0..8).map(|x| f.mesh.reference(1, x + 56)))
                    .collect();
                f.op(CorridorOp::Set(q, winding.clone()));
                for n in [-1, 0, 1, 100] {
                    f.op(CorridorOp::Valid(n));
                }
                for cap in [0, 1, 2, 3, 8, 64] {
                    f.op(CorridorOp::Corners(cap));
                }
                for range in [0., 0.73, 10., -1., f32::NAN] {
                    f.op(CorridorOp::Set(q, winding.clone()));
                    f.op(CorridorOp::Visibility(p, range));
                    f.op(CorridorOp::Visibility(q, range));
                    f.op(CorridorOp::Corners(8));
                }
                f.op(CorridorOp::Set(q, winding.clone()));
                f.op(CorridorOp::Topology);
                f.op(CorridorOp::Corners(8));
                for pos in [
                    [-0.72, 13., 2.7],
                    [0.85, 29., 3.2],
                    [2.7, -7., 4.1],
                    [f32::NAN, 0., 0.],
                ] {
                    f.op(CorridorOp::Move(pos));
                    f.op(CorridorOp::Corners(3));
                }
                for pos in [[3.2, 37., 4.3], [2.2, -13., 3.7], [f32::NAN, 0., 0.]] {
                    f.op(CorridorOp::Target(pos));
                    f.op(CorridorOp::Corners(8));
                }
                f.op(CorridorOp::Filter(1, 0));
                f.op(CorridorOp::Valid(100));
                f.op(CorridorOp::Trim(start, p));
                f.op(CorridorOp::Filter(0xffff, 0));
                f.op(CorridorOp::Reset(start, p));
                f.op(CorridorOp::Set(q, winding));
                f.op(CorridorOp::Flags(start, 0));
                f.op(CorridorOp::Trim(start, p));
                f.op(CorridorOp::Flags(start, 1));
                f.op(CorridorOp::Fix(end, q));
                f.op(CorridorOp::Valid(1));
                f.op(CorridorOp::Valid(2));
                f.op(CorridorOp::Trim(start, p));
                f.op(CorridorOp::Topology);
                let middle = f.mesh.reference(0, 1);
                for path in [vec![start, middle, end], vec![start, end]] {
                    f.op(CorridorOp::Reset(start, p));
                    f.op(CorridorOp::Set(q, path));
                    f.op(CorridorOp::Offmesh(middle));
                    f.op(CorridorOp::Offmesh(0xfeed));
                }
                if offmesh {
                    let off = f.mesh.reference(0, 64);
                    let destination = f.mesh.reference(0, 63);
                    for path in [
                        vec![start, off, destination],
                        vec![start, off],
                        vec![off, destination],
                    ] {
                        f.op(CorridorOp::Reset(start, p));
                        f.op(CorridorOp::Set(q, path));
                        f.op(CorridorOp::Corners(8));
                        f.op(CorridorOp::Offmesh(off));
                        f.op(CorridorOp::Corners(8));
                    }
                }
                f.op(CorridorOp::Reset(0, p));
                f.op(CorridorOp::Move(p));
                f.op(CorridorOp::Offmesh(0));
                f.op(CorridorOp::Trim(start, p));
                f.op(CorridorOp::Set(q, vec![start, end]));
                f.op(CorridorOp::Remove(f.mesh.reference(1, 0)));
                f.op(CorridorOp::Valid(100));
                f.op(CorridorOp::Corners(8));
                f.op(CorridorOp::Trim(start, p));
                f.op(CorridorOp::Fix(start, p));
                f.op(CorridorOp::Trim(0, p));
                f.check();
            }
        }
    }
    // A filtered 64-polygon strip fills the native 32-reference visibility/
    // topology outputs and truncates each surface move to its 16-reference budget.
    let n = 32;
    let mut f = CorridorFixture::with_size(128, 2048, false, n);
    for tile in 0..2 {
        for poly in n..n * n {
            f.op(CorridorOp::Flags(f.mesh.reference(tile, poly), 0));
        }
    }
    let start = [-1.15, 0.7, 2.25];
    let end = [-1.3 + (2 * n) as f32 * 0.3 - 0.15, 0.7, 2.25];
    let path: Vec<_> = (0..2)
        .flat_map(|tile| (0..n).map(move |poly| (tile, poly)))
        .map(|(tile, poly)| f.mesh.reference(tile, poly))
        .collect();
    f.op(CorridorOp::Set(end, path.clone()));
    f.op(CorridorOp::Visibility(end, 19.1));
    f.op(CorridorOp::Topology);
    f.op(CorridorOp::Move(end));
    assert_eq!(f.corridor.first_polygon(), f.mesh.reference(0, 15));
    f.op(CorridorOp::Reset(f.mesh.reference(0, 0), start));
    f.op(CorridorOp::Set(end, path));
    f.op(CorridorOp::Target(start));
    assert_eq!(f.corridor.last_polygon(), f.mesh.reference(1, 16));
    f.check();
}

use recast_nav::crowd::{
    AvoidanceDebugData, AvoidanceDebugSample, AvoidanceParams, ObstacleAvoidanceQuery,
    VelocityInput,
};
enum AvoidOp {
    Circle(Point, f32, Point, Point),
    Segment([Point; 2]),
    Reset,
    Init(usize, usize),
    DebugReset,
    DebugInit(usize),
    DebugAdd(AvoidanceDebugSample),
    Normalize,
    Sample(bool, VelocityInput, AvoidanceParams, bool),
}
struct AvoidFixture {
    query: ObstacleAvoidanceQuery,
    debug: AvoidanceDebugData,
    circles: usize,
    segments: usize,
    debug_capacity: usize,
    input: String,
    commands: Vec<String>,
    actual: Vec<Value>,
    // Per-slot provenance keeps directly supplied debug values bit-exact.
    calculated_debug: Vec<[bool; 3]>,
    debug_history: Vec<Vec<[bool; 3]>>,
}
fn avoidance_params() -> AvoidanceParams {
    AvoidanceParams {
        velocity_bias: 0.4,
        weight_desired_velocity: 2.,
        weight_current_velocity: 0.75,
        weight_side: 0.75,
        weight_time_of_impact: 2.5,
        horizon_time: 2.5,
        grid_size: 11,
        adaptive_divisions: 7,
        adaptive_rings: 2,
        adaptive_depth: 3,
    }
}
fn velocity_input() -> VelocityInput {
    VelocityInput {
        position: [-0.13, 0.17, 0.11],
        radius: 0.31,
        maximum_speed: 1.2,
        velocity: [0.37, -0.29, -0.21],
        desired_velocity: [0.73, 0.19, 0.27],
    }
}
fn first_difference(a: &Value, b: &Value, path: String) -> Option<String> {
    if a == b {
        return None;
    }
    if let (Some(a), Some(b)) = (a.as_array(), b.as_array()) {
        if a.len() != b.len() {
            return Some(format!("{path}: array lengths {} / {}", a.len(), b.len()));
        }
        for (i, (a, b)) in a.iter().zip(b).enumerate() {
            if let Some(d) = first_difference(a, b, format!("{path}/{i}")) {
                return Some(d);
            }
        }
    }
    Some(format!("{path}: Rust {a}, native {b}"))
}
impl AvoidFixture {
    fn new(circles: usize, segments: usize, debug_capacity: usize) -> Self {
        let mut f = Self {
            query: ObstacleAvoidanceQuery::new(circles, segments).unwrap(),
            debug: if debug_capacity == 0 {
                AvoidanceDebugData::default()
            } else {
                AvoidanceDebugData::new(debug_capacity).unwrap()
            },
            circles,
            segments,
            debug_capacity,
            input: format!("avoidance {circles} {segments} {debug_capacity} "),
            commands: vec![],
            actual: vec![],
            calculated_debug: vec![[false; 3]; debug_capacity],
            debug_history: vec![],
        };
        f.actual.push(json!([null, f.snapshot()]));
        f.debug_history.push(f.calculated_debug.clone());
        f
    }
    fn snapshot(&self) -> Value {
        json!([
            self.query.circles().len(),
            self.query.segments().len(),
            (0..self.circles)
                .map(|i| {
                    let c = self.query.circle(i).unwrap();
                    json!([
                        c.position.map(f32::to_bits),
                        c.velocity.map(f32::to_bits),
                        c.desired_velocity.map(f32::to_bits),
                        [c.radius.to_bits()],
                        c.direction.map(f32::to_bits),
                        c.preferred_side.map(f32::to_bits)
                    ])
                })
                .collect::<Vec<_>>(),
            (0..self.segments)
                .map(|i| {
                    let s = self.query.segment(i).unwrap();
                    json!([
                        s.positions[0].map(f32::to_bits),
                        s.positions[1].map(f32::to_bits),
                        u8::from(s.touch)
                    ])
                })
                .collect::<Vec<_>>(),
            self.debug.sample_count(),
            (0..self.debug_capacity)
                .map(|i| self.debug.sample(i).map(|s| json!([
                    s.velocity.map(f32::to_bits),
                    [s.size.to_bits()],
                    s.penalties.map(f32::to_bits)
                ])))
                .collect::<Vec<_>>()
        ])
    }
    fn op(&mut self, op: AvoidOp) {
        let mut s = String::new();
        let result = match op {
            AvoidOp::Circle(p, r, v, d) => {
                s.push_str("C ");
                append_points(&mut s, p.into_iter().chain([r]).chain(v).chain(d));
                self.query.add_circle(p, r, v, d);
                Value::Null
            }
            AvoidOp::Segment(p) => {
                s.push_str("S ");
                append_points(&mut s, p.into_iter().flatten());
                self.query.add_segment(p);
                Value::Null
            }
            AvoidOp::Reset => {
                s.push_str("R ");
                self.query.reset();
                Value::Null
            }
            AvoidOp::Init(c, n) => {
                write!(s, "I {c} {n} ").unwrap();
                self.query.reinitialize(c, n).unwrap();
                self.circles = c;
                self.segments = n;
                json!(1)
            }
            AvoidOp::DebugReset => {
                s.push_str("D ");
                self.debug.reset();
                Value::Null
            }
            AvoidOp::DebugInit(n) => {
                write!(s, "J {n} ").unwrap();
                self.debug.reinitialize(n).unwrap();
                self.debug_capacity = n;
                self.calculated_debug = vec![[false; 3]; n];
                json!(1)
            }
            AvoidOp::DebugAdd(sample) => {
                s.push_str("B ");
                append_points(
                    &mut s,
                    sample
                        .velocity
                        .into_iter()
                        .chain([sample.size])
                        .chain(sample.penalties),
                );
                let index = self.debug.sample_count();
                self.debug.add_sample(sample);
                if self.debug.sample_count() > index {
                    self.calculated_debug[index] = [false; 3];
                }
                Value::Null
            }
            AvoidOp::Normalize => {
                s.push_str("N ");
                let status = self.debug.normalize_samples();
                if status & FAILURE == 0 {
                    for entry in self
                        .calculated_debug
                        .iter_mut()
                        .take(self.debug.sample_count())
                    {
                        entry[2] = true;
                    }
                }
                json!(status)
            }
            AvoidOp::Sample(adaptive, input, p, record) => {
                s.push_str(if adaptive { "A " } else { "G " });
                append_points(
                    &mut s,
                    input
                        .position
                        .into_iter()
                        .chain([input.radius, input.maximum_speed])
                        .chain(input.velocity)
                        .chain(input.desired_velocity),
                );
                append_points(
                    &mut s,
                    [
                        p.velocity_bias,
                        p.weight_desired_velocity,
                        p.weight_current_velocity,
                        p.weight_side,
                        p.weight_time_of_impact,
                        p.horizon_time,
                    ],
                );
                write!(
                    s,
                    "{} {} {} {} {} ",
                    p.grid_size,
                    p.adaptive_divisions,
                    p.adaptive_rings,
                    p.adaptive_depth,
                    u8::from(record)
                )
                .unwrap();
                let debug = record.then_some(&mut self.debug);
                let result = if adaptive {
                    self.query
                        .sample_velocity_adaptive(input, &p, debug)
                        .unwrap()
                } else {
                    self.query.sample_velocity_grid(input, &p, debug)
                };
                if record {
                    for entry in self
                        .calculated_debug
                        .iter_mut()
                        .take(self.debug.sample_count())
                    {
                        *entry = [true; 3];
                    }
                }
                json!([result.sample_count, result.velocity.map(f32::to_bits)])
            }
        };
        self.commands.push(s);
        self.actual.push(json!([result, self.snapshot()]));
        self.debug_history.push(self.calculated_debug.clone());
    }
    fn check(mut self) {
        write!(self.input, "{} ", self.commands.len()).unwrap();
        for s in &self.commands {
            self.input.push_str(s);
        }
        let expected = oracle(self.input.clone());
        parity_trace::write(
            "avoidance",
            &self.input,
            &self.actual,
            expected.as_array().unwrap(),
        );
        assert_eq!(self.actual.len(), expected.as_array().unwrap().len());
        for (i, a) in self.actual.into_iter().enumerate() {
            if let Some(diff) = float_comparison::difference(&a, &expected[i], |path| match path {
                [0, 1, _] | [1, 2, _, 4 | 5, _] => true,
                [1, 5, slot, field, _] => self.debug_history[i]
                    .get(*slot)
                    .is_some_and(|fields| fields.get(*field) == Some(&true)),
                _ => false,
            }) {
                std::fs::create_dir_all("target/parity-failures").unwrap();
                std::fs::write("target/parity-failures/avoidance.txt", &self.input).unwrap();
                std::fs::write(
                    "target/parity-failures/avoidance-actual.json",
                    a.to_string(),
                )
                .unwrap();
                std::fs::write(
                    "target/parity-failures/avoidance-expected.json",
                    expected[i].to_string(),
                )
                .unwrap();
                panic!(
                    "avoidance operation {i} {} {diff}",
                    i.checked_sub(1).map_or("init", |j| self.commands[j]
                        .split_whitespace()
                        .next()
                        .unwrap())
                );
            }
        }
    }
}
#[test]
fn native_crowd_adaptive_pattern_preserves_float_operand_bits() {
    let values = [
        0.,
        -0.,
        1.,
        -1.,
        f32::from_bits(0x7fc1_2345),
        f32::from_bits(0xffc5_4321),
        f32::from_bits(0x7f81_2345),
        f32::from_bits(0xff85_4321),
        f32::INFINITY,
        f32::NEG_INFINITY,
    ];
    for x in values {
        for z in values {
            let mut f = AvoidFixture::new(0, 0, 128);
            for adaptive_divisions in [7, 31] {
                f.op(AvoidOp::Sample(
                    true,
                    VelocityInput {
                        desired_velocity: [x, 0., z],
                        ..velocity_input()
                    },
                    AvoidanceParams {
                        adaptive_divisions,
                        adaptive_depth: 2,
                        ..avoidance_params()
                    },
                    true,
                ));
            }
            f.check();
        }
    }
}
#[test]
fn native_crowd_avoidance_samples_prepared_obstacles_debug_costs_and_normalization() {
    let mixed = [
        1.,
        f32::from_bits(0x7fc1_2345),
        f32::from_bits(0xffc5_4321),
        f32::from_bits(0x7f81_2345),
        f32::from_bits(0xff85_4321),
        f32::INFINITY,
    ];
    for (circles, segments, debug) in [(0, 0, 0), (3, 2, 4), (6, 2, 128), (7, 2, 128), (8, 8, 128)]
    {
        let mut f = AvoidFixture::new(circles, segments, debug);
        let input = velocity_input();
        let p = avoidance_params();
        f.op(AvoidOp::Sample(false, input, p, false));
        f.op(AvoidOp::Sample(true, input, p, true));
        for i in 0..9 {
            let v = i as f32;
            f.op(AvoidOp::Circle(
                [
                    -1.13 + v * 0.41,
                    0.17 + (i % 3) as f32 * 0.2,
                    0.43 - (i % 4) as f32 * 0.37,
                ],
                0.2 + v * 0.03,
                [0.37 - v * 0.03, v * 0.01, -0.21 + v * 0.07],
                [0.2 + v * 0.01, -0.3, 0.1 + v * 0.09],
            ));
        }
        for segment in [
            [[-0.13, -13., 0.11], [0.87, 29., 0.11]],
            [[1.13, 0.7, -0.43], [-0.37, -0.8, 0.97]],
            [[0., 0., 1.], [1., 0., 1.]],
            [[0.51, 0., -0.27], [0.51, 4., -0.27]],
            [[-1., 0., -1.], [-1., 0., 1.]],
            [[1., 0., 1.], [-1., 0., 1.]],
            [[0., 0., -1.], [0., 0., 1.]],
            [[1., 0., 0.], [-1., 0., 0.]],
            [[0.; 3], [0.; 3]],
        ] {
            f.op(AvoidOp::Segment(segment));
        }
        f.op(AvoidOp::Sample(false, input, p, true));
        f.op(AvoidOp::Normalize);
        f.op(AvoidOp::Sample(true, input, p, true));
        if debug == 128 {
            for x in mixed {
                for y in mixed {
                    let params = AvoidanceParams {
                        weight_desired_velocity: x,
                        weight_current_velocity: y,
                        weight_side: x,
                        weight_time_of_impact: y,
                        grid_size: 3,
                        adaptive_divisions: 3,
                        adaptive_rings: 1,
                        adaptive_depth: 2,
                        ..p
                    };
                    for adaptive in [false, true] {
                        f.op(AvoidOp::Sample(adaptive, input, params, true));
                        f.op(AvoidOp::Normalize);
                    }
                }
            }
        }
        for bias in [0., 1., -0.2, 1.4] {
            f.op(AvoidOp::Sample(
                false,
                input,
                AvoidanceParams {
                    velocity_bias: bias,
                    grid_size: 5,
                    ..p
                },
                true,
            ));
        }
        for maximum_speed in [0., -0.1, 1.2] {
            f.op(AvoidOp::Sample(
                true,
                VelocityInput {
                    maximum_speed,
                    ..input
                },
                p,
                true,
            ));
        }
        for grid_size in [0, 1] {
            f.op(AvoidOp::Sample(
                false,
                input,
                AvoidanceParams { grid_size, ..p },
                true,
            ));
        }
        for (adaptive_divisions, adaptive_rings, adaptive_depth) in
            [(0, 0, 3), (1, 1, 0), (2, 1, 0), (255, 1, 0), (31, 255, 1)]
        {
            f.op(AvoidOp::Sample(
                true,
                input,
                AvoidanceParams {
                    adaptive_divisions,
                    adaptive_rings,
                    adaptive_depth,
                    ..p
                },
                true,
            ));
        }
        f.op(AvoidOp::Sample(
            true,
            VelocityInput {
                desired_velocity: [0.; 3],
                ..input
            },
            p,
            true,
        ));
        for horizon_time in [0., -1., f32::INFINITY] {
            f.op(AvoidOp::Sample(
                false,
                input,
                AvoidanceParams {
                    horizon_time,
                    grid_size: 5,
                    ..p
                },
                true,
            ));
            f.op(AvoidOp::Sample(
                true,
                input,
                AvoidanceParams { horizon_time, ..p },
                true,
            ));
        }
        for altered in [
            // Arithmetic fast paths must preserve underflow, overflow and signed
            // zero as well as the nonfinite operand corpus below.
            VelocityInput {
                maximum_speed: f32::MIN_POSITIVE,
                velocity: [-0., 0., 0.],
                desired_velocity: [0., 0., -0.],
                ..input
            },
            VelocityInput {
                maximum_speed: f32::from_bits(1),
                ..input
            },
            VelocityInput {
                maximum_speed: f32::MAX,
                ..input
            },
            VelocityInput {
                position: [1e20, 0., -1e20],
                velocity: [-1e20, 0., 1e20],
                desired_velocity: [1e20, 0., -1e20],
                ..input
            },
            VelocityInput {
                maximum_speed: f32::INFINITY,
                ..input
            },
            VelocityInput {
                desired_velocity: [f32::NAN, 0., -0.],
                ..input
            },
            VelocityInput {
                position: [f32::INFINITY, 0., 0.],
                ..input
            },
        ] {
            f.op(AvoidOp::Sample(
                false,
                altered,
                AvoidanceParams { grid_size: 3, ..p },
                true,
            ));
            f.op(AvoidOp::Sample(true, altered, p, true));
        }
        f.op(AvoidOp::Reset);
        f.op(AvoidOp::Circle(
            input.position,
            input.radius,
            [0.; 3],
            [0.; 3],
        ));
        f.op(AvoidOp::Sample(
            false,
            input,
            AvoidanceParams { grid_size: 3, ..p },
            true,
        ));
        f.op(AvoidOp::Sample(true, input, p, true));
        f.op(AvoidOp::DebugReset);
        for penalties in [
            [f32::NAN, f32::NEG_INFINITY, 0., -0., f32::MAX],
            [0.2, 0.2001, 0.3, 0.30001, 0.4],
        ] {
            f.op(AvoidOp::DebugAdd(AvoidanceDebugSample {
                velocity: [f32::NAN, -0., f32::INFINITY],
                size: 0.,
                penalties,
            }));
        }
        f.op(AvoidOp::Normalize);
        f.op(AvoidOp::DebugInit(2));
        f.op(AvoidOp::DebugReset);
        f.op(AvoidOp::DebugAdd(AvoidanceDebugSample {
            velocity: [0.; 3],
            size: 1.,
            penalties: [0.5; 5],
        }));
        f.op(AvoidOp::Normalize);
        if debug == 128 {
            for x in mixed {
                for y in mixed {
                    f.op(AvoidOp::Reset);
                    f.op(AvoidOp::Circle(
                        [input.position[0] + 0.17, 0., input.position[2] - 0.23],
                        0.2,
                        [x, 0., y],
                        [y, 0., x],
                    ));
                    for adaptive in [false, true] {
                        f.op(AvoidOp::Sample(
                            adaptive,
                            input,
                            AvoidanceParams {
                                grid_size: 3,
                                adaptive_divisions: 3,
                                adaptive_rings: 1,
                                adaptive_depth: 2,
                                ..p
                            },
                            true,
                        ));
                    }
                }
            }
            // Prepared geometry also combines distinct NaN operands before
            // the sampling helpers; retain those intermediate payloads too.
            for &x in &mixed[1..5] {
                for &z in &mixed[1..5] {
                    f.op(AvoidOp::Reset);
                    f.op(AvoidOp::Circle([x, 0., z], 0.2, [z, 0., x], [x, 0., z]));
                    f.op(AvoidOp::Segment([[x, 0., z], [z, 0., x]]));
                    for (adaptive, record) in
                        [(false, false), (false, true), (true, false), (true, true)]
                    {
                        f.op(AvoidOp::Sample(
                            adaptive,
                            input,
                            AvoidanceParams {
                                grid_size: 3,
                                adaptive_divisions: 3,
                                adaptive_rings: 1,
                                adaptive_depth: 2,
                                ..p
                            },
                            record,
                        ));
                    }
                }
            }
        }
        f.op(AvoidOp::Init(2, 1));
        f.op(AvoidOp::Sample(false, input, p, false));
        f.check();
    }
}
#[test]
fn avoidance_checks_native_uninitialized_pattern_and_debug_storage_domains() {
    let mut query = ObstacleAvoidanceQuery::new(0, 0).unwrap();
    let input = velocity_input();
    let p = avoidance_params();
    let mut pattern_debug = AvoidanceDebugData::new(8).unwrap();
    assert!(
        query
            .sample_velocity_adaptive(
                input,
                &AvoidanceParams {
                    adaptive_divisions: 2,
                    ..p
                },
                Some(&mut pattern_debug)
            )
            .is_err()
    );
    // Sampling the initialized prefix remains observable before the checked
    // rejection of the first native uninitialized pattern slot.
    assert!(pattern_debug.sample_count() > 0);
    assert!(
        query
            .sample_velocity_adaptive(
                input,
                &AvoidanceParams {
                    adaptive_divisions: 32,
                    adaptive_rings: 4,
                    adaptive_depth: 0,
                    ..p
                },
                None
            )
            .is_err()
    );
    let mut debug = AvoidanceDebugData::new(2).unwrap();
    debug.add_sample(AvoidanceDebugSample {
        velocity: [0.; 3],
        size: 1.,
        penalties: [0.; 5],
    });
    debug.reinitialize(2).unwrap();
    assert_eq!(debug.sample_count(), 1);
    assert!(debug.sample(0).is_none());
    assert_eq!(debug.normalize_samples(), FAILURE | INVALID_PARAM);
    debug.reset();
    assert_eq!(debug.normalize_samples(), SUCCESS);

    let sample = AvoidanceDebugSample {
        size: 7.,
        ..Default::default()
    };
    let mut debug = AvoidanceDebugData::new(6).unwrap();
    debug.add_sample(sample);
    debug.add_sample(sample);
    debug.reinitialize(6).unwrap(); // Retain count2, but all replacement slots are uninitialized.
    debug.add_sample(sample); // Only slot2 is initialized.
    assert!(debug.sample(0).is_none() && debug.sample(1).is_none());
    assert_eq!(
        debug.sample(2).unwrap().size.to_bits(),
        sample.size.to_bits()
    );
    assert_eq!(debug.normalize_samples(), FAILURE | INVALID_PARAM);
    debug.reset();
    debug.add_sample(sample); // Initialized prefix0 and retained slot2, with a gap at1.
    assert!(debug.sample(1).is_none());
    assert!(debug.sample(2).is_some());
    assert_eq!(debug.samples().count(), 1);
    assert_eq!(debug.normalize_samples(), SUCCESS);
    let copy = debug.clone();
    debug.add_sample(sample); // Close the gap without erasing the retained sample.
    assert!(debug.sample(1).is_some() && debug.sample(2).is_some());
    assert!(copy.sample(1).is_none() && copy.sample(2).is_some());
    assert_eq!(debug.normalize_samples(), SUCCESS);
    debug.reinitialize(1).unwrap(); // Retained count exceeds the new capacity.
    debug.add_sample(sample);
    assert_eq!(debug.sample_count(), 2);
    assert_eq!(debug.normalize_samples(), FAILURE | INVALID_PARAM);
    debug.reset();
    debug.add_sample(sample);
    assert_eq!(debug.normalize_samples(), SUCCESS);
}

use recast_nav::crowd::{AgentParams, Crowd, CrowdDebugInfo};
fn agent_params(flags: u8) -> AgentParams {
    AgentParams {
        radius: 0.2,
        height: 1.8,
        max_acceleration: 3.5,
        max_speed: 1.2,
        collision_query_range: 1.3,
        path_optimization_range: 3.7,
        separation_weight: 1.1,
        update_flags: flags,
        obstacle_avoidance_type: 0,
        query_filter_type: 0,
        user_data: 0x12345678,
    }
}
fn agent_param_json(p: &AgentParams) -> Value {
    let values = [
        p.radius,
        p.height,
        p.max_acceleration,
        p.max_speed,
        p.collision_query_range,
        p.path_optimization_range,
        p.separation_weight,
    ]
    .map(f32::to_bits);
    json!([
        values,
        p.update_flags,
        p.obstacle_avoidance_type,
        p.query_filter_type,
        p.user_data
    ])
}
fn avoidance_param_json(p: &AvoidanceParams) -> Value {
    let values = [
        p.velocity_bias,
        p.weight_desired_velocity,
        p.weight_current_velocity,
        p.weight_side,
        p.weight_time_of_impact,
        p.horizon_time,
    ]
    .map(f32::to_bits);
    json!([
        values,
        p.grid_size,
        p.adaptive_divisions,
        p.adaptive_rings,
        p.adaptive_depth
    ])
}
fn agent_param_input(s: &mut String, p: &AgentParams) {
    append_points(
        s,
        [
            p.radius,
            p.height,
            p.max_acceleration,
            p.max_speed,
            p.collision_query_range,
            p.path_optimization_range,
            p.separation_weight,
        ],
    );
    write!(
        s,
        "{} {} {} {} ",
        p.update_flags, p.obstacle_avoidance_type, p.query_filter_type, p.user_data
    )
    .unwrap();
}
fn avoidance_param_input(s: &mut String, p: &AvoidanceParams) {
    append_points(
        s,
        [
            p.velocity_bias,
            p.weight_desired_velocity,
            p.weight_current_velocity,
            p.weight_side,
            p.weight_time_of_impact,
            p.horizon_time,
        ],
    );
    write!(
        s,
        "{} {} {} {} ",
        p.grid_size, p.adaptive_divisions, p.adaptive_rings, p.adaptive_depth
    )
    .unwrap();
}
enum SimulationOp {
    Backing(i32),
    Add(Point, AgentParams),
    Params(i32, AgentParams),
    Remove(i32),
    Target(i32, PolyRef, Point),
    Velocity(i32, Point),
    Reset(i32),
    Update(f32, bool, i32, bool),
    Filter(i32, u16, u16, usize, f32),
    Avoidance(i32, AvoidanceParams),
    Handles(i32, i32, i32),
    Active(usize),
    Init(usize, f32),
    DeleteTile(TileRef),
    Tile(Vec<u8>, TileRef),
}
struct SimulationFixture {
    mesh: NavMesh,
    crowd: Crowd,
    data: AvoidanceDebugData,
    debug_index: i32,
    opt_start: Point,
    opt_end: Point,
    input: String,
    commands: Vec<String>,
    actual: Vec<Value>,
}
impl SimulationFixture {
    fn new(n: usize, maximum: usize, radius: f32, offmesh: bool) -> Self {
        let tiles = [mesh_tile(n, 0, offmesh), mesh_tile(n, 1, false)];
        let origin = [-1.3, 0.13, 2.1];
        let width = n as f32 * 0.3;
        let polygons = (n * n + 1).next_power_of_two().max(128);
        let mut input = "simulation ".to_owned();
        append_points(&mut input, origin.into_iter().chain([width, width]));
        write!(input, "4 {polygons} 128 2 ").unwrap();
        let mut mesh = NavMesh::new(origin, width, width, 4, polygons).unwrap();
        for data in tiles {
            write!(input, "{} ", data.len()).unwrap();
            for b in &data {
                write!(input, "{b} ").unwrap();
            }
            mesh.add_tile_data(&data, 0).unwrap();
        }
        write!(input, "{maximum} {} ", radius.to_bits()).unwrap();
        let mut f = Self {
            mesh,
            crowd: Crowd::new(maximum, radius).ok().unwrap(),
            data: AvoidanceDebugData::new(128).unwrap(),
            debug_index: -1,
            opt_start: [-17., -19., -23.],
            opt_end: [29., 31., 37.],
            input,
            commands: vec![],
            actual: vec![],
        };
        f.actual.push(json!([null, f.snapshot()]));
        f
    }
    fn snapshot(&self) -> Value {
        let c = &self.crowd;
        let grid = c.grid();
        let counts = (-2..=4)
            .flat_map(|z| (-2..=4).map(move |x| grid.item_count_at(x, z).unwrap()))
            .collect::<Vec<_>>();
        let agents = (0..c.agent_count())
            .map(|i| {
                let a = c.agent(i as i32).unwrap();
                let timers = [
                    a.topology_optimization_time,
                    a.desired_speed,
                    a.target_replan_time,
                ]
                .map(f32::to_bits);
                let vectors = [
                    a.position,
                    a.displacement,
                    a.desired_velocity,
                    a.new_velocity,
                    a.velocity,
                ]
                .map(|p| p.map(f32::to_bits));
                json!([
                    u8::from(a.active),
                    a.state,
                    u8::from(a.partial),
                    timers,
                    vectors,
                    agent_param_json(&a.params),
                    a.neighbors()
                        .iter()
                        .map(|n| json!([n.index, [n.distance_squared.to_bits()]]))
                        .collect::<Vec<_>>(),
                    a.corners()
                        .iter()
                        .map(|p| json!([p.position.map(f32::to_bits), p.flags, p.reference]))
                        .collect::<Vec<_>>(),
                    [
                        json!(a.target_state),
                        json!(a.target_reference),
                        json!(a.target_position.map(f32::to_bits)),
                        json!(a.target_path_queue_reference),
                        json!(u8::from(a.target_replan))
                    ],
                    [
                        json!(a.corridor.position().map(f32::to_bits)),
                        json!(a.corridor.target().map(f32::to_bits)),
                        json!(a.corridor.path())
                    ],
                    json!([
                        a.boundary.center().map(f32::to_bits),
                        a.boundary.segment_count(),
                        (0..8)
                            .map(|i| a
                                .boundary
                                .segment(i)
                                .unwrap()
                                .positions
                                .into_iter()
                                .flatten()
                                .map(f32::to_bits)
                                .collect::<Vec<_>>())
                            .collect::<Vec<_>>()
                    ]),
                    (a.target_path_queue_reference != 0)
                        .then(|| c.path_queue().request_status(a.target_path_queue_reference)),
                    json!([
                        a.neighbor_count,
                        a.neighbors
                            .iter()
                            .map(|n| json!([n.index, [n.distance_squared.to_bits()]]))
                            .collect::<Vec<_>>()
                    ]),
                    json!([
                        a.corner_count,
                        a.corners
                            .iter()
                            .map(|c| json!([c.position.map(f32::to_bits), c.flags, c.reference]))
                            .collect::<Vec<_>>()
                    ])
                ])
            })
            .collect::<Vec<_>>();
        let filters = (0..16)
            .map(|i| {
                let f = c.filter(i).unwrap();
                json!([
                    f.include,
                    f.exclude,
                    f.area_costs.iter().map(|v| v.to_bits()).collect::<Vec<_>>()
                ])
            })
            .collect::<Vec<_>>();
        json!([
            c.agent_count(),
            c.query_half_extents().map(f32::to_bits),
            c.velocity_sample_count(),
            [
                json!(grid.bounds()),
                json!([grid.cell_size().to_bits()]),
                json!(counts),
                json!(grid.query_items([[-2.; 2], [4.; 2]], 64).unwrap())
            ],
            c.active_agents(c.agent_count()),
            agents,
            filters,
            (0..8)
                .map(|i| avoidance_param_json(c.avoidance_params(i).unwrap()))
                .collect::<Vec<_>>(),
            node_snapshot::snapshot(c.query().node_pool()),
            node_snapshot::snapshot(c.path_queue().query().node_pool()),
            [
                json!(self.debug_index),
                json!(self.opt_start.map(f32::to_bits)),
                json!(self.opt_end.map(f32::to_bits)),
                json!(
                    self.data
                        .samples()
                        .map(|s| json!([
                            s.velocity.map(f32::to_bits),
                            [s.size.to_bits()],
                            s.penalties.map(f32::to_bits)
                        ]))
                        .collect::<Vec<_>>()
                )
            ]
        ])
    }
    fn op(&mut self, op: SimulationOp) -> Value {
        let mut s = String::new();
        let result = match op {
            SimulationOp::Add(p, params) => {
                s.push_str("A ");
                append_points(&mut s, p);
                agent_param_input(&mut s, &params);
                json!(self.crowd.add_agent(&self.mesh, p, &params).unwrap())
            }
            SimulationOp::Params(i, params) => {
                write!(s, "P {i} ").unwrap();
                agent_param_input(&mut s, &params);
                self.crowd.update_agent_parameters(i, &params);
                Value::Null
            }
            SimulationOp::Remove(i) => {
                write!(s, "R {i} ").unwrap();
                self.crowd.remove_agent(i);
                Value::Null
            }
            SimulationOp::Target(i, r, p) => {
                write!(s, "T {i} {r} ").unwrap();
                append_points(&mut s, p);
                json!(u8::from(self.crowd.request_move_target(i, r, p)))
            }
            SimulationOp::Velocity(i, p) => {
                write!(s, "V {i} ").unwrap();
                append_points(&mut s, p);
                json!(u8::from(self.crowd.request_move_velocity(i, p)))
            }
            SimulationOp::Reset(i) => {
                write!(s, "X {i} ").unwrap();
                json!(u8::from(self.crowd.reset_move_target(i)))
            }
            SimulationOp::Update(dt, enabled, index, record) => {
                write!(
                    s,
                    "U {} {} {index} {} ",
                    dt.to_bits(),
                    u8::from(enabled),
                    u8::from(record)
                )
                .unwrap();
                self.debug_index = index;
                let mut debug = CrowdDebugInfo {
                    index,
                    optimization_start: self.opt_start,
                    optimization_end: self.opt_end,
                    avoidance: record.then_some(&mut self.data),
                };
                self.crowd
                    .update(&self.mesh, dt, enabled.then_some(&mut debug))
                    .unwrap();
                self.opt_start = debug.optimization_start;
                self.opt_end = debug.optimization_end;
                Value::Null
            }
            SimulationOp::Filter(i, inc, exc, area, cost) => {
                write!(s, "F {i} {inc} {exc} {area} {} ", cost.to_bits()).unwrap();
                let f = self.crowd.editable_filter(i);
                let found = f.is_some();
                if let Some(f) = f {
                    f.include = inc;
                    f.exclude = exc;
                    f.area_costs[area] = cost;
                }
                json!(u8::from(found))
            }
            SimulationOp::Avoidance(i, p) => {
                write!(s, "O {i} ").unwrap();
                avoidance_param_input(&mut s, &p);
                self.crowd.set_avoidance_params(i, &p);
                Value::Null
            }
            SimulationOp::Backing(index) => {
                write!(s, "E {index} ").unwrap();
                let agent = self.crowd.editable_agent(index);
                let found = agent.is_some();
                if let Some(a) = agent {
                    a.neighbors[5] = recast_nav::crowd::Neighbor {
                        index: -17,
                        distance_squared: f32::from_bits(0xffc0_1871),
                    };
                    a.corners[3] = recast_nav::crowd::Corner {
                        position: [-0., f32::from_bits(0xffc0_1275), f32::INFINITY],
                        flags: 0xa5,
                        reference: PolyRef::MAX,
                    };
                }
                json!(u8::from(found))
            }
            SimulationOp::Handles(a, f, p) => {
                write!(s, "H {a} {f} {p} ").unwrap();
                json!([
                    u8::from(self.crowd.agent(a).is_some()),
                    u8::from(self.crowd.editable_agent(a).is_some()),
                    u8::from(self.crowd.filter(f).is_some()),
                    u8::from(self.crowd.editable_filter(f).is_some()),
                    u8::from(self.crowd.avoidance_params(p).is_some())
                ])
            }
            SimulationOp::Active(cap) => {
                write!(s, "L {cap} ").unwrap();
                json!(self.crowd.active_agents(cap))
            }
            SimulationOp::Init(n, r) => {
                write!(s, "I {n} {} ", r.to_bits()).unwrap();
                self.crowd.reinitialize(n, r).unwrap();
                json!(1)
            }
            SimulationOp::DeleteTile(r) => {
                write!(s, "D {r} ").unwrap();
                json!(match self.mesh.remove_tile_data(r) {
                    Ok(_) => SUCCESS,
                    Err(e) => e,
                })
            }
            SimulationOp::Tile(data, r) => {
                write!(s, "B {r} {} ", data.len()).unwrap();
                for b in &data {
                    write!(s, "{b} ").unwrap();
                }
                match self.mesh.add_tile_data(&data, r) {
                    Ok(r) => json!([SUCCESS, r]),
                    Err(e) => json!([e, 0]),
                }
            }
        };
        self.commands.push(s);
        self.actual.push(json!([result, self.snapshot()]));
        result
    }
    fn check(mut self) {
        write!(self.input, "{} ", self.commands.len()).unwrap();
        for s in &self.commands {
            self.input.push_str(s);
        }
        let expected = oracle(self.input.clone());
        assert_eq!(self.actual.len(), expected.as_array().unwrap().len());
        for (i, a) in self.actual.into_iter().enumerate() {
            if let Some(diff) = first_difference(&a, &expected[i], String::new()) {
                std::fs::create_dir_all("target/parity-failures").unwrap();
                std::fs::write("target/parity-failures/simulation.txt", self.input).unwrap();
                std::fs::write(
                    "target/parity-failures/simulation-actual.json",
                    a.to_string(),
                )
                .unwrap();
                std::fs::write(
                    "target/parity-failures/simulation-expected.json",
                    expected[i].to_string(),
                )
                .unwrap();
                panic!(
                    "simulation op {i} {} {diff}",
                    i.checked_sub(1).map_or("init", |j| self.commands[j]
                        .split_whitespace()
                        .next()
                        .unwrap())
                );
            }
        }
    }
}
#[test]
fn native_crowd_complete_agent_updates_requests_reuse_and_public_configuration() {
    for flags in [0, 1, 2, 4, 8, 16, 31] {
        let mut f = SimulationFixture::new(8, 3, 0.5, false);
        let params = agent_params(flags);
        let p = [-1.15, 0.7, 2.25];
        let q = [3.35, 0.7, 4.35];
        let end = f.mesh.reference(1, 63);
        f.op(SimulationOp::Handles(-1, 16, 8));
        f.op(SimulationOp::Handles(0, 15, 7));
        f.op(SimulationOp::Active(0));
        f.op(SimulationOp::Velocity(-1, [0.; 3]));
        f.op(SimulationOp::Reset(3));
        f.op(SimulationOp::Target(0, 0, p));
        f.op(SimulationOp::Velocity(0, [0.6, 0., 0.2]));
        f.op(SimulationOp::Add(p, params));
        f.op(SimulationOp::Update(0., true, 0, true));
        f.op(SimulationOp::Velocity(0, [0.37, 0.11, 0.23]));
        for dt in [0.05, 0.1, 0.5] {
            f.op(SimulationOp::Update(dt, true, 0, true));
        }
        f.op(SimulationOp::Reset(0));
        f.op(SimulationOp::Update(0.1, false, 0, false));
        f.op(SimulationOp::Target(0, end, q));
        for _ in 0..4 {
            f.op(SimulationOp::Update(0.1, true, 0, true));
        }
        f.op(SimulationOp::Add(
            [-0.85, 0.7, 2.55],
            AgentParams {
                user_data: 37,
                ..params
            },
        ));
        f.op(SimulationOp::Velocity(1, [-0.2, 0.3, -0.1]));
        f.op(SimulationOp::Update(0.1, true, 1, true));
        f.op(SimulationOp::Remove(0));
        f.op(SimulationOp::Active(1));
        f.op(SimulationOp::Update(0.1, true, 0, true));
        f.op(SimulationOp::Add([-0.55, 0.7, 2.85], params));
        f.op(SimulationOp::Filter(1, 3, 0, 2, 3.75));
        f.op(SimulationOp::Params(
            0,
            AgentParams {
                query_filter_type: 1,
                max_speed: 0.7,
                ..params
            },
        ));
        f.op(SimulationOp::Target(0, end, q));
        f.op(SimulationOp::Update(0.2, true, 0, true));
        f.op(SimulationOp::Avoidance(
            0,
            AvoidanceParams {
                adaptive_depth: 2,
                ..avoidance_params()
            },
        ));
        f.op(SimulationOp::Filter(-1, 0, 0, 0, 1.));
        f.op(SimulationOp::Avoidance(8, avoidance_params()));
        f.op(SimulationOp::DeleteTile(f.mesh.reference(1, 0)));
        f.op(SimulationOp::Update(0.1, true, 0, true));
        f.op(SimulationOp::Tile(mesh_tile(8, 1, false), 0));
        f.op(SimulationOp::Update(0.1, true, 0, true));
        f.op(SimulationOp::Init(2, 0.4));
        f.op(SimulationOp::Update(0.1, true, 0, true));
        f.op(SimulationOp::Add(p, params));
        f.op(SimulationOp::Add([8., 0.7, 2.5], params));
        assert_eq!(f.op(SimulationOp::Add(p, params)), -1);
        f.op(SimulationOp::Active(7));
        f.op(SimulationOp::Update(0.1, true, 0, true));
        f.check();
    }
}

#[test]
fn native_crowd_shared_path_budget_dense_neighbors_and_replanning() {
    for maximum in [12, 36] {
        let mut f = SimulationFixture::new(16, maximum, 0.5, false);
        let params = agent_params(31);
        let destination = f.mesh.reference(1, 255);
        let target = [8.15, 0.7, 6.75];
        for i in 0..maximum {
            let x = if i % 6 < 2 {
                0.
            } else {
                (i % 6) as f32 * 0.075
            };
            let z = (i / 6) as f32 * 0.075;
            f.op(SimulationOp::Add(
                [-1.15 + x, 0.7, 2.25 + z],
                AgentParams {
                    user_data: i,
                    ..params
                },
            ));
        }
        f.op(SimulationOp::Update(0.13, true, 0, true));
        assert_eq!(
            (0..maximum)
                .map(|i| f.crowd.agent(i as i32).unwrap().neighbors().len())
                .max(),
            Some(6)
        );
        for i in 0..maximum {
            f.op(SimulationOp::Target(i as i32, destination, target));
        }
        f.op(SimulationOp::Update(0.07, true, 0, true));
        assert!(
            (0..maximum).any(|i| matches!(
                f.crowd.agent(i as i32).unwrap().target_state,
                recast_nav::crowd::TARGET_WAITING_FOR_QUEUE
                    | recast_nav::crowd::TARGET_WAITING_FOR_PATH
            )),
            "fixture must exercise deferred requests"
        );
        for step in 0..12 {
            f.op(SimulationOp::Update(
                [0.03, 0.07, 0.13, 0.4][step % 4],
                true,
                (step % 3) as i32,
                step % 4 != 0,
            ));
        }
        f.op(SimulationOp::Remove(0));
        f.op(SimulationOp::Remove(5));
        f.op(SimulationOp::Active(maximum));
        f.op(SimulationOp::Update(0.1, true, 0, true));
        f.op(SimulationOp::Add(
            [-0.7, 0.7, 3.0],
            AgentParams {
                update_flags: 2,
                ..params
            },
        ));
        f.op(SimulationOp::Target(0, destination, target));
        f.op(SimulationOp::Update(0.1, true, 0, true));
        f.op(SimulationOp::Filter(1, 0, 0, 1, 3.));
        f.op(SimulationOp::Params(
            1,
            AgentParams {
                query_filter_type: 1,
                ..params
            },
        ));
        f.op(SimulationOp::Update(0.2, true, 1, true));
        f.op(SimulationOp::Filter(1, 7, 0, 1, 0.125));
        f.op(SimulationOp::Update(0.2, true, 1, true));
        f.op(SimulationOp::DeleteTile(f.mesh.reference(0, 0)));
        f.op(SimulationOp::Update(0.13, true, 0, true));
        f.op(SimulationOp::Tile(mesh_tile(16, 0, false), 0));
        f.op(SimulationOp::Update(0.13, true, 0, true));
        f.check();
    }
}
#[test]
fn native_crowd_offmesh_animation_completion_pause_and_slot_reuse() {
    let mut f = SimulationFixture::new(8, 2, 0.5, true);
    f.op(SimulationOp::Backing(0));
    let params = agent_params(31);
    let start = [-1.15, 0.7, 2.25];
    let destination = f.mesh.reference(0, 63);
    let end = [0.95, 0.7, 4.35];
    f.op(SimulationOp::Add(start, params));
    f.op(SimulationOp::Velocity(0, [0.1, 0., 0.]));
    f.op(SimulationOp::Update(0.1, true, 0, true));
    f.op(SimulationOp::Target(0, destination, end));
    f.op(SimulationOp::Update(0.05, true, 0, true));
    assert_eq!(
        f.crowd.agent(0).unwrap().state,
        recast_nav::crowd::STATE_OFFMESH,
        "fixture must trigger the native off-mesh animation"
    );
    for dt in [0.1, 0.2, 0.4, 1., 0.1] {
        f.op(SimulationOp::Update(dt, true, 0, true));
    }
    f.op(SimulationOp::Remove(0));
    f.op(SimulationOp::Add(start, params));
    f.op(SimulationOp::Velocity(0, [0.1, 0., 0.]));
    f.op(SimulationOp::Update(0.1, true, 0, true));
    f.op(SimulationOp::Target(0, destination, end));
    f.op(SimulationOp::Update(0.05, true, 0, true));
    assert_eq!(
        f.crowd.agent(0).unwrap().state,
        recast_nav::crowd::STATE_OFFMESH
    );
    f.op(SimulationOp::Remove(0));
    f.op(SimulationOp::Update(0.5, true, 0, true));
    f.op(SimulationOp::Add([0.65, 0.7, 2.55], params));
    f.op(SimulationOp::Update(0.1, true, 0, true));
    f.op(SimulationOp::Update(2., true, 0, true));
    f.op(SimulationOp::Remove(0));
    f.op(SimulationOp::Add(start, params));
    f.op(SimulationOp::Velocity(0, [0.1, 0., 0.]));
    f.op(SimulationOp::Update(0.1, true, 0, true));
    f.op(SimulationOp::Target(0, destination, end));
    f.op(SimulationOp::Update(3., true, 0, true));
    f.op(SimulationOp::Update(0.1, true, 0, true));
    f.check();
}
#[test]
fn native_crowd_defined_zero_numeric_parameters_and_time_steps() {
    let mut f = SimulationFixture::new(8, 2, 0.5, false);
    let params = agent_params(31);
    let start = [-1.15, 0.7, 2.25];
    let destination = f.mesh.reference(1, 63);
    let end = [3.35, 0.7, 4.35];
    f.op(SimulationOp::Add(
        start,
        AgentParams {
            radius: 0.,
            height: 0.,
            max_speed: 0.,
            max_acceleration: 0.,
            collision_query_range: 0.,
            path_optimization_range: 0.,
            separation_weight: 0.,
            ..params
        },
    ));
    f.op(SimulationOp::Target(0, destination, end));
    for dt in [0., 0.1, -0.1, f32::NAN] {
        f.op(SimulationOp::Update(dt, true, 0, true));
    }
    f.op(SimulationOp::Params(0, params));
    f.op(SimulationOp::Reset(0));
    f.op(SimulationOp::Velocity(0, [0.37, 0.11, 0.23]));
    for dt in [0., 0.1, -0.1, f32::NAN, 0.1] {
        f.op(SimulationOp::Update(dt, true, 0, true));
    }
    f.check();
}

#[test]
fn native_crowd_seeded_lifecycle_operation_streams() {
    fn word(seed: &mut u32) -> u32 {
        *seed ^= *seed << 13;
        *seed ^= *seed >> 17;
        *seed ^= *seed << 5;
        *seed
    }
    fn position(seed: &mut u32) -> Point {
        let x = word(seed) % 16;
        let z = word(seed) % 8;
        [-1.15 + x as f32 * 0.3, 0.7, 2.25 + z as f32 * 0.3]
    }
    fn parameters(seed: &mut u32) -> AgentParams {
        AgentParams {
            radius: 0.1 + (word(seed) % 5) as f32 * 0.035,
            max_speed: 0.4 + (word(seed) % 7) as f32 * 0.19,
            max_acceleration: 0.7 + (word(seed) % 7) as f32 * 0.37,
            collision_query_range: 0.6 + (word(seed) % 4) as f32 * 0.3,
            path_optimization_range: 1.1 + (word(seed) % 5) as f32 * 0.61,
            separation_weight: (word(seed) % 8) as f32 * 0.31,
            obstacle_avoidance_type: (word(seed) % 4) as u8,
            query_filter_type: (word(seed) % 4) as u8,
            user_data: word(seed) as usize,
            ..agent_params((word(seed) & 31) as u8)
        }
    }
    for initial in [
        0x8917_3a21,
        0x1d64_5987,
        0x3f2a_9181,
        0x9726_4ab5,
        0x762b_1c49,
        0x51d9_73a3,
        0x314b_8f65,
        0xc789_5a13,
    ]
    .into_iter()
    .chain((1..=56u32).map(|seed| seed.wrapping_mul(0x9e37_79b9)))
    {
        let mut seed = initial;
        let mut f = SimulationFixture::new(8, 12, 0.5, initial & 2 != 0);
        for _ in 0..8 {
            f.op(SimulationOp::Add(
                position(&mut seed),
                parameters(&mut seed),
            ));
        }
        for step in 0..160 {
            if step == 80 {
                f.op(SimulationOp::Init(12, 0.45));
                for _ in 0..8 {
                    f.op(SimulationOp::Add(
                        position(&mut seed),
                        parameters(&mut seed),
                    ));
                }
            }
            let choice = word(&mut seed) % 16;
            let index = (word(&mut seed) % 14) as i32 - 1;
            match choice {
                0 => {
                    f.op(SimulationOp::Add(
                        position(&mut seed),
                        parameters(&mut seed),
                    ));
                }
                1 => {
                    f.op(SimulationOp::Remove(index));
                }
                2 => {
                    f.op(SimulationOp::Params(index, parameters(&mut seed)));
                }
                3 => {
                    let tile = (word(&mut seed) % 2) as usize;
                    let poly = (word(&mut seed) % 64) as usize;
                    let r = f.mesh.reference(tile, poly);
                    let p = position(&mut seed);
                    f.op(SimulationOp::Target(index, r, p));
                }
                4 => {
                    let v = std::array::from_fn(|_| (word(&mut seed) % 17) as f32 * 0.11 - 0.8);
                    f.op(SimulationOp::Velocity(index, v));
                }
                5 => {
                    f.op(SimulationOp::Reset(index));
                }
                6 => {
                    let filter = (word(&mut seed) % 4) as i32;
                    let include = [0, 1, 3, 7, u16::MAX][(word(&mut seed) % 5) as usize];
                    let exclude = [0, 0, 2, 4][(word(&mut seed) % 4) as usize];
                    let area = (word(&mut seed) % 3) as usize;
                    let cost = 0.2 + (word(&mut seed) % 9) as f32 * 0.41;
                    f.op(SimulationOp::Filter(filter, include, exclude, area, cost));
                }
                7 => {
                    let slot = (word(&mut seed) % 4) as i32;
                    let mut p = avoidance_params();
                    p.adaptive_divisions = [3, 5, 7][(word(&mut seed) % 3) as usize];
                    p.adaptive_rings = 1 + (word(&mut seed) % 3) as u8;
                    p.adaptive_depth = 1 + (word(&mut seed) % 3) as u8;
                    p.horizon_time = 0.7 + (word(&mut seed) % 4) as f32 * 0.37;
                    p.weight_side = (word(&mut seed) % 5) as f32 * 0.29;
                    f.op(SimulationOp::Avoidance(slot, p));
                }
                8 => {
                    let r = f.mesh.tile_ref_at([1, 0, 0]);
                    if r != 0 {
                        f.op(SimulationOp::DeleteTile(r));
                    } else {
                        f.op(SimulationOp::Tile(mesh_tile(8, 1, false), 0));
                    }
                }
                9 => {
                    f.op(SimulationOp::Active((word(&mut seed) % 16) as usize));
                }
                _ => {
                    let dt = [0., 0.016, 0.033, 0.1, 0.25, 0.6][(word(&mut seed) % 6) as usize];
                    let enabled = word(&mut seed) & 1 != 0;
                    let record = word(&mut seed) & 1 != 0;
                    f.op(SimulationOp::Update(dt, enabled, index, record));
                }
            }
        }
        eprintln!("Crowd lifecycle seed {initial:#x}");
        f.check();
    }
}
