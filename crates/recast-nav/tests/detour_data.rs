#![cfg(feature = "cpp-oracle")]
#[path = "support/float_comparison.rs"]
mod float_comparison;
#[path = "support/parity_trace.rs"]
mod parity_trace;

use recast_nav::{
    nav::*,
    nav_data::*,
    query::{NavMeshQuery, RandomPoint, SurfaceMove, WallHit},
};
#[path = "support/node_snapshot.rs"]
mod node_snapshot;
use serde_json::{Value, json};
use std::{
    fmt::Write as _,
    io::Write as _,
    process::{Command, Stdio},
};

fn oracle(input: String) -> Value {
    let mut child = Command::new(env!("DETOUR_DATA_ORACLE"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    let mut stdin = child.stdin.take().unwrap();
    let output = std::thread::scope(|scope| {
        let writer = scope.spawn(move || stdin.write_all(input.as_bytes()).unwrap());
        let out = child.wait_with_output().unwrap();
        writer.join().unwrap();
        out
    });
    assert!(output.status.success());
    serde_json::from_slice(&output.stdout).unwrap()
}
fn bytes(value: &Value) -> Vec<u8> {
    value["bytes"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_u64().unwrap() as u8)
        .collect()
}
fn values<T: std::fmt::Display>(s: &mut String, iter: impl IntoIterator<Item = T>) {
    for v in iter {
        write!(s, "{v} ").unwrap();
    }
}
fn native(p: &CreateParams<'_>) -> Value {
    native_create(p, false)
}
fn native_create(p: &CreateParams<'_>, bits: bool) -> Value {
    fn float_values(s: &mut String, iter: impl IntoIterator<Item = f32>, bits: bool) {
        if bits {
            values(s, iter.into_iter().map(f32::to_bits));
        } else {
            values(s, iter);
        }
    }
    let mut s = format!(
        "{} {} {} {} {} {} {} {} {} {} {} {} {} ",
        if bits { "create_bits" } else { "create" },
        p.vertices.len(),
        p.polygons.len(),
        p.vertices_per_polygon,
        u8::from(p.detail_meshes.is_some()),
        p.detail_vertices.len(),
        p.detail_triangles.len(),
        p.off_mesh_connections.len(),
        u8::from(p.off_mesh_user_ids.is_some()),
        p.coordinate[0],
        p.coordinate[1],
        p.coordinate[2],
        p.user_id
    );
    float_values(
        &mut s,
        [
            p.walkable_height,
            p.walkable_radius,
            p.walkable_climb,
            p.cell_size,
            p.cell_height,
        ],
        bits,
    );
    write!(s, "{} ", u8::from(p.build_bv_tree)).unwrap();
    float_values(&mut s, p.bounds.iter().flatten().copied(), bits);
    values(&mut s, p.vertices.iter().flatten());
    for poly in p.polygons {
        values(&mut s, &poly.vertices[..p.vertices_per_polygon]);
        values(&mut s, &poly.neighbors[..p.vertices_per_polygon]);
    }
    values(&mut s, p.polygons.iter().map(|p| p.flags));
    values(&mut s, p.polygons.iter().map(|p| p.area));
    if let Some(d) = p.detail_meshes {
        values(&mut s, d.iter().flatten());
    }
    float_values(&mut s, p.detail_vertices.iter().flatten().copied(), bits);
    values(&mut s, p.detail_triangles.iter().flatten());
    float_values(
        &mut s,
        p.off_mesh_connections
            .iter()
            .flat_map(|c| c.positions.iter().flatten())
            .copied(),
        bits,
    );
    float_values(
        &mut s,
        p.off_mesh_connections.iter().map(|c| c.radius),
        bits,
    );
    values(&mut s, p.off_mesh_connections.iter().map(|c| c.flags));
    values(&mut s, p.off_mesh_connections.iter().map(|c| c.area));
    values(&mut s, p.off_mesh_connections.iter().map(|c| c.direction));
    if let Some(ids) = p.off_mesh_user_ids {
        values(&mut s, ids);
    }
    oracle(s)
}
fn operation(mode: &str, data: &[u8]) -> Value {
    let mut s = format!("{mode} {} ", data.len());
    values(&mut s, data);
    oracle(s)
}
fn compare(actual: &[u8], expected: &[u8], context: &str) {
    assert_eq!(actual.len(), expected.len(), "{context}: buffer size");
    if let Some(at) = actual.iter().zip(expected).position(|(a, b)| a != b) {
        panic!(
            "{context}: first byte mismatch at {at}: Rust={} C++={}",
            actual[at], expected[at]
        );
    }
}

fn polygon(vertices: &[u16], neighbors: &[u16], flags: u16, area: u8) -> CreatePolygon {
    let mut p = CreatePolygon {
        vertices: [u16::MAX; 6],
        neighbors: [u16::MAX; 6],
        flags,
        area,
    };
    p.vertices[..vertices.len()].copy_from_slice(vertices);
    p.neighbors[..neighbors.len()].copy_from_slice(neighbors);
    p
}
fn params<'a>(verts: &'a [[u16; 3]], polys: &'a [CreatePolygon]) -> CreateParams<'a> {
    CreateParams {
        vertices: verts,
        polygons: polys,
        vertices_per_polygon: 6,
        detail_meshes: None,
        detail_vertices: &[],
        detail_triangles: &[],
        off_mesh_connections: &[],
        off_mesh_user_ids: None,
        coordinate: [-2, 3, 1],
        user_id: 0xa1b2_c3d4,
        bounds: [[-1.3, -0.7, 2.1], [20., 15., 23.]],
        walkable_height: 1.8,
        walkable_radius: 0.3,
        walkable_climb: 0.9,
        cell_size: 0.3,
        cell_height: 0.2,
        build_bv_tree: true,
    }
}
fn check(p: &CreateParams<'_>, context: &str) -> Vec<u8> {
    let cpp = native(p);
    assert_eq!(cpp["ok"], true, "{context}");
    let cpp = bytes(&cpp);
    let rust = create(p).unwrap();
    compare(&rust, &cpp, context);
    // Explicit C++ -> Rust parsing and Rust -> C++ runtime loading, as well as
    // independent byte equality before the loader patches firstLink/link records.
    let parsed = TileData::parse(&cpp).unwrap();
    assert_eq!(parsed.header.coordinate, p.coordinate);
    assert_eq!(
        parsed.polygons().len(),
        parsed.header.polygon_count as usize
    );
    assert_eq!(parsed.vertices().len(), parsed.header.vertex_count as usize);
    assert_eq!(parsed.detail_meshes().len(), p.polygons.len());
    assert!(parsed.link_bytes().iter().all(|&b| b == 0));
    compare(parsed.bytes(), &cpp, context);
    let loaded = operation("load", &rust);
    assert_eq!(loaded["status"], SUCCESS, "C++ loading {context}");
    let loaded = bytes(&loaded);
    let mesh = NavMesh::from_tile_data(&cpp).unwrap();
    let reference = mesh.reference(0, 0);
    compare(
        &mesh.tile_data(reference).unwrap(),
        &loaded,
        &format!("Rust runtime load {context}"),
    );
    let loaded = TileData::parse(&loaded).unwrap();
    assert_eq!(loaded.header, parsed.header);
    assert_eq!(
        loaded.detail_meshes().collect::<Vec<_>>(),
        parsed.detail_meshes().collect::<Vec<_>>()
    );
    assert_eq!(
        loaded.detail_triangles().collect::<Vec<_>>(),
        parsed.detail_triangles().collect::<Vec<_>>()
    );
    assert_eq!(
        loaded.bv_nodes().collect::<Vec<_>>(),
        parsed.bv_nodes().collect::<Vec<_>>()
    );
    assert_eq!(
        loaded.off_mesh_connections().collect::<Vec<_>>(),
        parsed.off_mesh_connections().collect::<Vec<_>>()
    );
    rust
}

#[test]
fn native_tile_data_details_off_mesh_bv_and_cross_loading() {
    let verts = [
        [0, 2, 0],
        [0, 2, 10],
        [10, 3, 10],
        [10, 3, 0],
        [20, 4, 10],
        [20, 4, 0],
    ];
    let polys = [
        polygon(&[0, 1, 2, 3], &[0x8000, 0x8001, 1, 0x8003], 0x12ab, 255),
        polygon(&[3, 2, 4, 5], &[0, 0x8001, 0x8002, 0x8003], 0x3501, 3),
    ];
    let mut p = params(&verts, &polys);
    for bv in [false, true] {
        p.build_bv_tree = bv;
        check(&p, "ground fan");
    }
    let dv: Vec<_> = polys
        .iter()
        .flat_map(|poly| {
            poly.vertices[..4]
                .iter()
                .map(|&i| {
                    let v = verts[i as usize];
                    [
                        v[0] as f32 * 0.3 - 1.3,
                        v[1] as f32 * 0.2 - 0.7,
                        v[2] as f32 * 0.3 + 2.1,
                    ]
                })
                .chain([[0.2, 0.43, 3.6]])
        })
        .collect();
    let dm = [[0, 5, 0, 4], [5, 5, 4, 4]];
    let dt = [
        [0, 1, 4, 1],
        [1, 2, 4, 1],
        [2, 3, 4, 1],
        [3, 0, 4, 1],
        [0, 1, 4, 1],
        [1, 2, 4, 1],
        [2, 3, 4, 1],
        [3, 0, 4, 1],
        [1, 2, 3, 0x55],
    ];
    let mut off = Vec::new();
    // All eight neighbor sides, inside ends, exact border ties, rejected starts,
    // vertical culling, and endpoint links counted even for discarded records.
    let ends = [
        [21., 0., 8.],
        [21., 0., 24.],
        [0., 0., 24.],
        [-2., 0., 24.],
        [-2., 0., 8.],
        [-2., 0., 1.],
        [0., 0., 1.],
        [21., 0., 1.],
        [2., 0., 6.],
        [20., 0., 23.],
    ];
    for (i, end) in ends.into_iter().enumerate() {
        off.push(CreateOffMeshConnection {
            positions: [[0., 0., 3.], end],
            radius: 0.7,
            flags: 0xff00 | i as u16,
            area: 64 + i as u8,
            direction: i as u8,
        });
    }
    for start in [[0., 50., 3.], [-3., 0., 3.], [20., 0., 8.], [-1.3, 0., 2.1]] {
        off.push(CreateOffMeshConnection {
            positions: [start, [2., 90., 6.]],
            radius: 0.1,
            flags: 1,
            area: 5,
            direction: 255,
        });
    }
    let ids: Vec<_> = (0..off.len()).map(|i| 0xa1b2_0000 + i as u32).collect();
    p.off_mesh_connections = &off;
    for details in [false, true] {
        for bv in [false, true] {
            for user_ids in [false, true] {
                p.detail_meshes = details.then_some(&dm);
                // Intentionally supply detail vertices even without detail meshes: native
                // start-height culling still uses these independent input fields.
                p.detail_vertices = &dv;
                p.detail_triangles = &dt;
                p.build_bv_tree = bv;
                p.off_mesh_user_ids = user_ids.then_some(ids.as_slice());
                check(&p, &format!("details={details} bv={bv} ids={user_ids}"));
            }
        }
    }
}

#[test]
fn native_endian_fields_reserved_bytes_and_foreign_round_trip() {
    assert_eq!(
        oracle("status".into()),
        serde_json::json!([
            SUCCESS,
            FAILURE,
            IN_PROGRESS,
            STATUS_DETAIL_MASK,
            WRONG_MAGIC,
            WRONG_VERSION,
            OUT_OF_MEMORY,
            INVALID_PARAM,
            BUFFER_TOO_SMALL,
            OUT_OF_NODES,
            PARTIAL_RESULT,
            ALREADY_OCCUPIED
        ])
    );
    let verts = [[0, 0, 0], [0, 0, 5], [5, 0, 5], [5, 0, 0]];
    let polys = [polygon(&[0, 1, 2, 3], &[0xffff; 4], 0x3412, 17)];
    let off = [CreateOffMeshConnection {
        positions: [[0., -0.7, 3.], [1., -0.7, 4.]],
        radius: 0.2,
        flags: 0xabcd,
        area: 11,
        direction: 1,
    }];
    let mut p = params(&verts, &polys);
    p.off_mesh_connections = &off;
    p.off_mesh_user_ids = Some(&[0x1234_5678]);
    let data = check(&p, "endian base");
    // Include populated native link records and firstLink values from actual load.
    let loaded = bytes(&operation("load", &data));
    for original in [&data, &loaded] {
        let mut rust = original.clone();
        assert!(swap_data_endian(&mut rust));
        compare(&rust, &bytes(&operation("data", original)), "data swap");
        let data_swapped = rust.clone();
        assert!(swap_header_endian(&mut rust));
        compare(
            &rust,
            &bytes(&operation("header", &data_swapped)),
            "foreign export",
        );
        let foreign = rust.clone();
        assert!(!swap_data_endian(&mut rust));
        compare(&rust, &foreign, "foreign header rejection");
        assert!(swap_header_endian(&mut rust));
        assert!(swap_data_endian(&mut rust));
        compare(&rust, original, "foreign round trip");
    }
    let mut wrong = data.clone();
    wrong[0] ^= 1;
    assert_eq!(operation("load", &wrong)["status"], FAILURE | WRONG_MAGIC);
    assert_eq!(TileData::parse(&wrong).unwrap_err(), FAILURE | WRONG_MAGIC);
    assert!(!swap_header_endian(&mut wrong));
    assert!(!swap_data_endian(&mut wrong));
    let mut wrong = data.clone();
    wrong[4] ^= 1;
    assert_eq!(operation("load", &wrong)["status"], FAILURE | WRONG_VERSION);
    assert_eq!(
        TileData::parse(&wrong).unwrap_err(),
        FAILURE | WRONG_VERSION
    );
    // Native truncated buffers have undefined access. Safe Rust rejects every cut.
    for cut in 0..data.len() {
        assert!(TileData::parse(&data[..cut]).is_err(), "cut {cut}");
        assert!(
            NavMesh::from_tile_data(&data[..cut]).is_err(),
            "load cut {cut}"
        );
        let mut short = data[..cut].to_vec();
        assert!(!swap_data_endian(&mut short));
        compare(&short, &data[..cut], "truncation is atomic");
    }
}

#[test]
fn native_bv_tree_ordered_grid_and_seeded_ties() {
    // Minimized tie regression: seven records use insertion sort; eight enter
    // macOS qsort's partition path and move its equal-key pivot to the front.
    let verts = [[0, 0, 0], [0, 0, 1], [1, 0, 1], [1, 0, 0]];
    let polys = vec![polygon(&[0, 1, 2, 3], &[0xffff; 4], 1, 1); 8];
    check(&params(&verts, &polys), "eight equal BV bounds regression");
    let mut seed = 0x941a_23c1u32;
    for n in [1, 2, 3, 6, 7, 8, 13, 32, 97] {
        let mut verts = Vec::new();
        let mut polys = Vec::new();
        for i in 0..n {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let x = ((seed >> 16) % 9) as u16 * 2;
            let z = (i % 7) as u16 * 2;
            let y = ((seed >> 24) % 4) as u16;
            let base = verts.len() as u16;
            verts.extend([[x, y, z], [x, y, z + 1], [x + 1, y, z + 1], [x + 1, y, z]]);
            polys.push(polygon(
                &[base, base + 1, base + 2, base + 3],
                &[0xffff; 4],
                i as u16,
                i as u8,
            ));
        }
        check(&params(&verts, &polys), &format!("BV seed {n}"));
    }
}

#[test]
fn native_detail_storage_integer_widths_and_bv_clamping() {
    let verts = [[0, 2, 0], [0, 2, 10], [10, 3, 10], [10, 3, 0]];
    let polys = [
        polygon(&[0, 1, 2, 3], &[0xffff; 4], 1, 1),
        polygon(&[0, 1, 2, 3], &[0xffff; 4], 2, 2),
    ];
    let mut p = params(&verts, &polys);
    let mut dv = vec![[0., 0., 3.]; 65546];
    dv[4] = [30000., 40000., -30000.];
    dv[65545] = [1., 2., 3.];
    let dm = [[0, 65541, 0, 257], [65541, 5, 257, 1]];
    let tris = vec![[0, 1, 2, 21]; 259];
    p.detail_meshes = Some(&dm);
    p.detail_vertices = &dv;
    p.detail_triangles = &tris;
    let data = check(&p, "native detail vbase wrap and byte counts");
    let parsed = TileData::parse(&data).unwrap();
    let details: Vec<_> = parsed.detail_meshes().collect();
    assert_eq!(details[0].vertex_count, 1);
    assert_eq!(details[0].triangle_count, 1);
    assert_eq!(details[1].vertex_base, 1);
    assert_eq!(parsed.header.detail_vertex_count, 65538);
    assert_eq!(parsed.header.detail_triangle_count, 259);
    assert_eq!(parsed.detail_vertices().nth(1), Some([1., 2., 3.]));
    assert_eq!(parsed.bv_nodes().next().unwrap().max, [65535, 65535, 3]);
}

#[test]
fn native_polygon_widths_border_codes_and_creator_rejection() {
    let verts = [
        [0, 0, 0],
        [0, 1, 10],
        [5, 2, 13],
        [10, 2, 10],
        [10, 1, 0],
        [5, 0, 0],
    ];
    for width in 3..=6 {
        let polys = [polygon(
            &(0..width as u16).collect::<Vec<_>>(),
            &[0x8000, 0x8001, 0x8002, 0x8003, 0x8005, 0x800f][..width],
            65535,
            254,
        )];
        let mut p = params(&verts, &polys);
        p.vertices_per_polygon = width;
        check(&p, &format!("polygon width {width}"));
    }
    let polys = [polygon(&[0, 1, 2], &[0xffff; 3], 1, 1)];
    let many = vec![[0, 0, 0]; 65535];
    let mut p = params(&many, &polys);
    assert_eq!(native(&p)["ok"], false);
    assert!(create(&p).is_err());
    p.vertices = &[];
    assert_eq!(native(&p)["ok"], false);
    assert!(create(&p).is_err());
    p.vertices = &verts;
    p.polygons = &[];
    assert_eq!(native(&p)["ok"], false);
    assert!(create(&p).is_err());
}

struct Runtime {
    mesh: NavMesh,
    query: NavMeshQuery,
    sliced_filter: CallbackFilter,
    straight_scratch: StraightPath,
    path_scratch: Corridor,
    neighborhood_scratch: recast_nav::query::Neighborhood,
    dijkstra_scratch: recast_nav::query::DijkstraSearch,
    wall_scratch: recast_nav::query::WallSegments,
    tile_scratch: Vec<u8>,
    input: String,
    actual: Vec<Value>,
}
impl Runtime {
    fn random(
        &mut self,
        circle: Option<(PolyRef, Point, f32)>,
        values: &[f32],
        filter: CallbackFilter,
    ) -> RandomPoint {
        self.input
            .push_str(if circle.is_some() { "c " } else { "r " });
        self.filter(&filter);
        write!(self.input, "{} ", values.len()).unwrap();
        values
            .iter()
            .for_each(|v| write!(self.input, "{} ", v.to_bits()).unwrap());
        let mut random_calls = 0;
        let mut random = || {
            let value = values[random_calls % values.len()];
            random_calls += 1;
            filter
                .log
                .borrow_mut()
                .extend([3, i64::from(value.to_bits())]);
            value
        };
        let mut result = RandomPoint {
            reference: 123,
            position: [-31., 17., 3.],
        };
        let status = if let Some((start, center, radius)) = circle {
            write!(self.input, "{start} ").unwrap();
            self.point(center);
            writeln!(self.input, "{}", radius.to_bits()).unwrap();
            self.query.find_random_point_around_circle_into(
                &self.mesh,
                start,
                center,
                radius,
                &filter,
                &mut random,
                &mut result,
            )
        } else {
            self.query
                .find_random_point_into(&self.mesh, &filter, &mut random, &mut result)
        };
        self.actual.push(json!([
            status,
            result.reference,
            result.position[0].to_bits(),
            result.position[1].to_bits(),
            result.position[2].to_bits(),
            random_calls,
            *filter.log.borrow()
        ]));
        self.query_snapshot();
        result
    }
    fn local(
        &mut self,
        start: PolyRef,
        center: Point,
        radius: f32,
        capacity: usize,
        filter: CallbackFilter,
    ) {
        self.input.push_str("t ");
        self.filter(&filter);
        write!(self.input, "{start} {capacity} ").unwrap();
        self.point(center);
        writeln!(self.input, "{}", radius.to_bits()).unwrap();
        self.query.find_local_neighborhood_into(
            &self.mesh,
            start,
            center,
            radius,
            &filter,
            capacity,
            &mut self.neighborhood_scratch,
        );
        let result = &self.neighborhood_scratch;
        self.actual.push(json!([
            result.status,
            result
                .entries
                .iter()
                .map(|p| json!([p.reference, p.parent]))
                .collect::<Vec<_>>(),
            *filter.log.borrow()
        ]));
        self.query_snapshot();
    }
    fn walls(
        &mut self,
        reference: PolyRef,
        capacity: usize,
        portals: bool,
        filter: CallbackFilter,
    ) {
        self.input.push_str("w ");
        self.filter(&filter);
        writeln!(self.input, "{reference} {capacity} {}", u8::from(portals)).unwrap();
        self.query.poly_wall_segments_into(
            &self.mesh,
            reference,
            &filter,
            portals,
            capacity,
            &mut self.wall_scratch,
        );
        let result = &self.wall_scratch;
        self.actual.push(json!([
            result.status,
            result
                .segments
                .iter()
                .map(|p| json!([
                    p.reference,
                    p.positions[0][0].to_bits(),
                    p.positions[0][1].to_bits(),
                    p.positions[0][2].to_bits(),
                    p.positions[1][0].to_bits(),
                    p.positions[1][1].to_bits(),
                    p.positions[1][2].to_bits()
                ]))
                .collect::<Vec<_>>(),
            *filter.log.borrow()
        ]));
        self.query_snapshot();
    }
    fn distance_wall(
        &mut self,
        start: PolyRef,
        center: Point,
        radius: f32,
        filter: CallbackFilter,
    ) {
        self.input.push_str("v ");
        self.filter(&filter);
        write!(self.input, "{start} ").unwrap();
        self.point(center);
        writeln!(self.input, "{}", radius.to_bits()).unwrap();
        let mut hit = WallHit {
            distance: -19.,
            position: [-31., 17., 3.],
            normal: [2., 3., 5.],
        };
        let status = self
            .query
            .find_distance_to_wall_into(&self.mesh, start, center, radius, &filter, &mut hit);
        self.actual.push(json!([
            status,
            hit.distance.to_bits(),
            hit.position[0].to_bits(),
            hit.position[1].to_bits(),
            hit.position[2].to_bits(),
            hit.normal[0].to_bits(),
            hit.normal[1].to_bits(),
            hit.normal[2].to_bits(),
            *filter.log.borrow()
        ]));
        self.query_snapshot();
    }
    fn around(
        &mut self,
        start: PolyRef,
        center: Point,
        radius: f32,
        shape: Option<&[Point]>,
        capacity: usize,
        filter: CallbackFilter,
    ) {
        self.input
            .push_str(if shape.is_some() { "B " } else { "G " });
        self.filter(&filter);
        write!(self.input, "{start} {capacity} ").unwrap();
        if let Some(shape) = shape {
            write!(self.input, "{} ", shape.len()).unwrap();
            for &point in shape {
                self.point(point);
            }
            self.query.find_polygons_around_shape_into(
                &self.mesh,
                start,
                shape,
                &filter,
                capacity,
                &mut self.dijkstra_scratch,
            )
        } else {
            self.point(center);
            writeln!(self.input, "{}", radius.to_bits()).unwrap();
            self.query.find_polygons_around_circle_into(
                &self.mesh,
                start,
                center,
                radius,
                &filter,
                capacity,
                &mut self.dijkstra_scratch,
            )
        };
        let result = &self.dijkstra_scratch;
        self.actual.push(json!([
            result.status,
            result
                .entries
                .iter()
                .map(|p| json!([p.reference, p.parent, p.cost.to_bits()]))
                .collect::<Vec<_>>(),
            *filter.log.borrow()
        ]));
        self.query_snapshot();
    }
    fn dijkstra_path(&mut self, end: PolyRef, capacity: usize) {
        write!(
            self.input,
            "g {end} {capacity} {} ",
            self.path_scratch.polygons.len()
        )
        .unwrap();
        values(&mut self.input, &self.path_scratch.polygons);
        self.query.path_from_dijkstra_search_into(
            &self.mesh,
            end,
            capacity,
            &mut self.path_scratch,
        );
        self.actual.push(json!([
            self.path_scratch.status,
            self.path_scratch.polygons
        ]));
        self.query_snapshot();
    }
    fn new(tiles: usize, polygons: usize) -> Self {
        Self::with_nodes(tiles, polygons, 64)
    }
    fn with_nodes(tiles: usize, polygons: usize, nodes: usize) -> Self {
        Self {
            mesh: NavMesh::new([0.; 3], 4., 4., tiles, polygons).unwrap(),
            query: NavMeshQuery::new(nodes).unwrap(),
            sliced_filter: CallbackFilter::default(),
            straight_scratch: StraightPath::default(),
            path_scratch: Corridor::default(),
            neighborhood_scratch: recast_nav::query::Neighborhood::default(),
            dijkstra_scratch: recast_nav::query::DijkstraSearch::default(),
            wall_scratch: recast_nav::query::WallSegments::default(),
            tile_scratch: Vec::new(),
            input: format!("runtime 0 0 0 4 4 {tiles} {polygons} {nodes}\n"),
            actual: vec![serde_json::json!(SUCCESS)],
        }
    }
    fn query_init(&mut self, nodes: usize) {
        writeln!(self.input, "I {nodes}").unwrap();
        let s = self.query.reinitialize(nodes);
        let p = self.query.node_pool();
        self.actual
            .push(json!([s, p.max_nodes(), p.hash_size(), p.node_count()]));
    }
    fn query_snapshot(&mut self) {
        self.input.push_str("O\n");
        self.actual
            .push(node_snapshot::snapshot(self.query.node_pool()));
    }
    fn closed(&mut self, refs: &[PolyRef]) {
        write!(self.input, "L {} ", refs.len()).unwrap();
        values(&mut self.input, refs);
        self.actual.push(json!(
            refs.iter()
                .map(|&r| u8::from(self.query.is_in_closed_list(r)))
                .collect::<Vec<_>>()
        ));
    }
    fn persistent_path(
        &mut self,
        a: PolyRef,
        b: PolyRef,
        start: Point,
        end: Point,
        capacity: usize,
    ) {
        self.persistent_path_filtered(a, b, start, end, capacity, &Filter::default());
    }
    fn persistent_path_filtered(
        &mut self,
        a: PolyRef,
        b: PolyRef,
        start: Point,
        end: Point,
        capacity: usize,
        filter: &Filter,
    ) {
        write!(self.input, "d {} {} ", filter.include, filter.exclude).unwrap();
        values(&mut self.input, filter.area_costs.map(f32::to_bits));
        write!(self.input, "{a} {b} ").unwrap();
        self.point(start);
        self.point(end);
        writeln!(self.input, "{capacity}").unwrap();
        self.query.find_path_into(
            &self.mesh,
            a,
            b,
            start,
            end,
            filter,
            capacity,
            &mut self.path_scratch,
        );
        self.actual.push(json!([
            self.path_scratch.status,
            self.path_scratch.polygons
        ]));
        self.query_snapshot();
    }
    fn snapshot(&mut self) {
        self.input.push_str("S\n");
        let mesh = &self.mesh;
        let data = &mut self.tile_scratch;
        self.actual.push(Value::Array(
            mesh.tiles
                .iter()
                .enumerate()
                .map(|(i, t)| {
                    t.as_ref().map_or(Value::Null, |t| {
                        t.data_into(data).unwrap();
                        let (header, body) = t.data_parts();
                        assert!(header.iter().chain(body).eq(data.iter()));
                        serde_json::json!([mesh.reference(i, 0), t.free_link, data])
                    })
                })
                .collect(),
        ));
    }
    fn add(&mut self, data: &[u8], last: TileRef) -> TileRef {
        write!(self.input, "A {last} {} ", data.len()).unwrap();
        values(&mut self.input, data);
        let (status, reference) = match self.mesh.add_tile_data(data, last) {
            Ok(r) => (SUCCESS, r),
            Err(s) => (s, 0),
        };
        self.actual.push(serde_json::json!([status, reference]));
        self.snapshot();
        reference
    }
    fn remove(&mut self, reference: TileRef) -> Vec<u8> {
        writeln!(self.input, "R {reference}").unwrap();
        let (status, data) = match self.mesh.remove_tile_data(reference) {
            Ok(data) => (SUCCESS, data),
            Err(s) => (s, Vec::new()),
        };
        self.actual.push(serde_json::json!([status, data]));
        self.snapshot();
        data
    }
    fn point(&mut self, p: Point) {
        values(&mut self.input, p.map(f32::to_bits));
    }
    fn surface(
        &mut self,
        start: PolyRef,
        a: Point,
        b: Point,
        capacity: usize,
        filter: CallbackFilter,
    ) -> SurfaceMove {
        self.input.push_str("M ");
        self.filter(&filter);
        write!(self.input, "{start} {capacity} ").unwrap();
        self.point(a);
        self.point(b);
        let mut result = SurfaceMove {
            position: [-31., 17., 3.],
            visited: vec![123],
        };
        let status = self.query.move_along_surface_into(
            &self.mesh,
            start,
            a,
            b,
            &filter,
            capacity,
            &mut result,
        );
        self.actual.push(json!([
            status,
            result.position[0].to_bits(),
            result.position[1].to_bits(),
            result.position[2].to_bits(),
            result.visited,
            *filter.log.borrow()
        ]));
        self.query_snapshot();
        result
    }
    fn nearest(&mut self, p: Point, ext: Point, include: u16, exclude: u16) {
        self.input.push_str("N ");
        self.point(p);
        self.point(ext);
        writeln!(self.input, "{include} {exclude}").unwrap();
        let (status, found) = self.mesh.nearest(
            p,
            ext,
            &Filter {
                include,
                exclude,
                ..Filter::default()
            },
        );
        let (r, p) = found.unwrap_or((0, [0.; 3]));
        self.actual.push(serde_json::json!([
            status,
            r,
            p[0].to_bits(),
            p[1].to_bits(),
            p[2].to_bits()
        ]));
    }
    fn closest(&mut self, r: PolyRef, p: Point) {
        write!(self.input, "C {r} ").unwrap();
        self.point(p);
        let sentinel = [-0., 42., -3.];
        let (cs, cp, over) = self
            .mesh
            .closest_point(r, p)
            .map_or((FAILURE | INVALID_PARAM, sentinel, false), |(p, over)| {
                (SUCCESS, p, over)
            });
        let (bs, bp) = self
            .mesh
            .closest_boundary(r, p)
            .map_or((FAILURE | INVALID_PARAM, sentinel), |p| (SUCCESS, p));
        let (hs, h) = self
            .mesh
            .poly_height(r, p)
            .map_or_else(|s| (s, -123f32), |h| (SUCCESS, h));
        self.actual.push(serde_json::json!([
            cs,
            u8::from(over),
            cp[0].to_bits(),
            cp[1].to_bits(),
            cp[2].to_bits(),
            bs,
            bp[0].to_bits(),
            bp[1].to_bits(),
            bp[2].to_bits(),
            hs,
            h.to_bits()
        ]));
    }
    fn path(
        &mut self,
        a: PolyRef,
        b: PolyRef,
        start: Point,
        end: Point,
        capacity: usize,
        corners: usize,
    ) {
        write!(self.input, "P {a} {b} ").unwrap();
        self.point(start);
        self.point(end);
        writeln!(self.input, "64 {capacity} {corners}").unwrap();
        let path = self
            .mesh
            .find_path(a, b, start, end, &Filter::default(), 64, capacity);
        let straight = self.mesh.straight_path(start, end, &path.polygons, corners);
        let points: Vec<_> = straight
            .points
            .iter()
            .enumerate()
            .map(|(i, p)| {
                serde_json::json!([
                    straight.flags[i],
                    straight.references[i],
                    p[0].to_bits(),
                    p[1].to_bits(),
                    p[2].to_bits()
                ])
            })
            .collect();
        self.actual.push(serde_json::json!([
            SUCCESS,
            path.status,
            path.polygons,
            straight.status,
            points
        ]));
        for options in [STRAIGHTPATH_AREA_CROSSINGS, STRAIGHTPATH_ALL_CROSSINGS, 3] {
            self.straight(start, end, &path.polygons, corners, options);
        }
    }
    fn straight(
        &mut self,
        start: Point,
        end: Point,
        path: &[PolyRef],
        capacity: usize,
        options: u32,
    ) {
        self.input.push_str("H ");
        self.point(start);
        self.point(end);
        write!(self.input, "{capacity} {options} {} ", path.len()).unwrap();
        values(&mut self.input, path);
        self.query.straight_path_into(
            &self.mesh,
            start,
            end,
            path,
            capacity,
            options,
            &mut self.straight_scratch,
        );
        let result = &self.straight_scratch;
        let points: Vec<_> = result
            .points
            .iter()
            .enumerate()
            .map(|(i, p)| {
                json!([
                    result.flags[i],
                    result.references[i],
                    p[0].to_bits(),
                    p[1].to_bits(),
                    p[2].to_bits()
                ])
            })
            .collect();
        self.actual.push(json!([result.status, points]));
    }
    fn endpoints(&mut self, previous: PolyRef, r: PolyRef) {
        writeln!(self.input, "E {previous} {r}").unwrap();
        let (status, p) = self
            .mesh
            .off_mesh_endpoints(previous, r)
            .map_or_else(|s| (s, [[0.; 3]; 2]), |p| (SUCCESS, p));
        self.actual.push(serde_json::json!([
            status,
            p[0][0].to_bits(),
            p[0][1].to_bits(),
            p[0][2].to_bits(),
            p[1][0].to_bits(),
            p[1][1].to_bits(),
            p[1][2].to_bits(),
            self.mesh.off_mesh_connection(r).map(|c| [
                c.polygon as u32,
                c.flags as u32,
                c.side as u32,
                c.user_id
            ])
        ]));
    }
    fn flags(&mut self, r: PolyRef, flags: u16, area: u8) {
        writeln!(self.input, "F {r} {flags} {area}").unwrap();
        let fs = self.mesh.set_poly_flags(r, flags);
        let as_ = self.mesh.set_poly_area(r, area);
        let (fg, f) = self
            .mesh
            .poly_flags(r)
            .map_or_else(|s| (s, 0), |f| (SUCCESS, f));
        let (ag, a) = self
            .mesh
            .poly_area(r)
            .map_or_else(|s| (s, 0), |a| (SUCCESS, a));
        self.actual.push(serde_json::json!([fs, as_, fg, f, ag, a]));
    }
    fn store(&mut self, r: TileRef, size: usize) -> Vec<u8> {
        writeln!(self.input, "T {r} {size}").unwrap();
        let mut data = vec![0xa5; size];
        let status = self.mesh.store_tile_state(r, &mut data);
        self.actual.push(serde_json::json!([
            self.mesh.tile_state_size(r),
            status,
            data
        ]));
        data
    }
    fn restore(&mut self, r: TileRef, data: &[u8]) {
        write!(self.input, "U {r} {} ", data.len()).unwrap();
        values(&mut self.input, data);
        self.actual
            .push(serde_json::json!([self.mesh.restore_tile_state(r, data)]));
        self.snapshot();
    }
    fn finish(self, context: &str) {
        fn compare_json(a: &Value, b: &Value, path: String) {
            match (a, b) {
                (Value::Array(a), Value::Array(b)) => {
                    assert_eq!(a.len(), b.len(), "{path}: array size");
                    for (i, (a, b)) in a.iter().zip(b).enumerate() {
                        compare_json(a, b, format!("{path}[{i}]"));
                    }
                }
                _ => assert_eq!(a, b, "{path}"),
            }
        }
        compare_json(
            &Value::Array(self.actual),
            &oracle(self.input),
            context.into(),
        );
    }
}

#[test]
fn native_runtime_details_off_mesh_paths_and_reloading() {
    let verts = [[0, 0, 0], [0, 0, 4], [4, 0, 4], [4, 0, 0]];
    for portals in [false, true] {
        for bidirectional in [false, true] {
            for bv in [false, true] {
                let neighbors = if portals {
                    [0x8000, 0x8001, 0x8002, 0x8003]
                } else {
                    [0xffff; 4]
                };
                let polys = [polygon(&[0, 1, 2, 3], &neighbors, 0x8000, 3)];
                let off = [
                    CreateOffMeshConnection {
                        positions: [[1., 0.2, 1.], [6., 0.2, 1.]],
                        radius: 0.8,
                        flags: 2,
                        area: 5,
                        direction: u8::from(bidirectional),
                    },
                    CreateOffMeshConnection {
                        positions: [[0.5, 0.2, 2.], [3., 0.2, 3.]],
                        radius: 0.8,
                        flags: 4,
                        area: 6,
                        direction: 1,
                    },
                ];
                let dm = [[0, 5, 0, 4]];
                let dt = [[0, 1, 4, 1], [1, 2, 4, 1], [2, 3, 4, 1], [3, 0, 4, 1]];
                let dv = [
                    [0., 0., 0.],
                    [0., 0., 4.],
                    [4., 0., 4.],
                    [4., 0., 0.],
                    [2., 0.8, 2.],
                ];
                let target_dv = dv.map(|mut v| {
                    v[0] += 4.;
                    v[1] *= 1.5;
                    v
                });
                let mut p = params(&verts, &polys);
                p.coordinate = [0; 3];
                p.bounds = [[0.; 3], [4.; 3]];
                p.cell_size = 1.;
                p.cell_height = 0.5;
                p.walkable_climb = 1.;
                p.build_bv_tree = bv;
                p.detail_meshes = Some(&dm);
                p.detail_vertices = &dv;
                p.detail_triangles = &dt;
                p.off_mesh_connections = &off;
                let owner = create(&p).unwrap();
                p.coordinate = [1, 0, 0];
                p.user_id = 0x5566_7788;
                p.bounds = [[4., 0., 0.], [8., 4., 4.]];
                p.detail_vertices = &target_dv;
                p.off_mesh_connections = &[];
                let target = create(&p).unwrap();
                for reversed_load in [false, true] {
                    let mut r = Runtime::new(3, 16);
                    let (a, b) = if reversed_load {
                        let b = r.add(&target, 0);
                        (r.add(&owner, 0), b)
                    } else {
                        let a = r.add(&owner, 0);
                        (a, r.add(&target, 0))
                    };
                    let connection = a | 1;
                    if !portals {
                        let forward = r.mesh.find_path(
                            a,
                            b,
                            [1., 0., 1.],
                            [7., 0., 1.],
                            &Filter::default(),
                            64,
                            16,
                        );
                        assert_eq!(forward.polygons.last(), Some(&b));
                        assert!(forward.polygons.contains(&connection));
                        let reverse = r.mesh.find_path(
                            b,
                            a,
                            [7., 0., 1.],
                            [1., 0., 1.],
                            &Filter::default(),
                            64,
                            16,
                        );
                        assert_eq!(reverse.polygons.last() == Some(&a), bidirectional);
                    }
                    for (reference, point) in [
                        (a, [2., 9., 2.]),
                        (a, [-1., 9., 3.]),
                        (b, [6., -1., 2.]),
                        (connection, [3., 4., 1.]),
                        (connection, [9., 4., 1.]),
                        (a, [2., f32::NAN, 2.]),
                        (0, [0.; 3]),
                    ] {
                        r.closest(reference, point);
                    }
                    for (point, extents) in [
                        ([2., 1., 2.], [2.; 3]),
                        ([3.9, 0.4, 1.], [0.4; 3]),
                        ([9., 0., 2.], [2.; 3]),
                    ] {
                        r.nearest(point, extents, 0xffff, 0);
                        r.nearest(point, extents, 1, 0);
                    }
                    for (capacity, corners) in [(16, 16), (2, 2), (1, 1)] {
                        r.path(a, b, [1., 0., 1.], [7., 0., 1.], capacity, corners);
                        r.path(b, a, [7., 0., 1.], [1., 0., 1.], capacity, corners);
                    }
                    for reference in [a, connection, b] {
                        r.surface(
                            reference,
                            [1., 0.2, 1.],
                            [7., 9., 1.],
                            16,
                            CallbackFilter::default(),
                        );
                        r.local(reference, [1., 0.2, 1.], 10., 16, CallbackFilter::default());
                        r.walls(reference, 16, true, CallbackFilter::default());
                        r.distance_wall(reference, [1., 0.2, 1.], 10., CallbackFilter::default());
                        r.random(
                            Some((reference, [1., 0.2, 1.], 10.)),
                            &[0.5, 0.125, 0.875],
                            CallbackFilter::default(),
                        );
                    }
                    r.random(None, &[0.5, 0.125, 0.875], CallbackFilter::default());
                    r.around(a, [1., 0.2, 1.], 10., None, 16, CallbackFilter::default());
                    r.dijkstra_path(b, 16);
                    r.around(
                        a,
                        [0.; 3],
                        0.,
                        Some(&[[0.; 3], [0., 0., 4.], [8., 0., 4.], [8., 0., 0.]]),
                        16,
                        CallbackFilter::default(),
                    );
                    r.dijkstra_path(b, 16);
                    r.endpoints(a, connection);
                    r.endpoints(b, connection);
                    r.endpoints(0, a);
                    r.endpoints(a, 0);
                    r.add(&owner, 0); // occupied coordinate, before capacity/last-ref checks
                    r.remove(b);
                    r.closest(b, [6., 0., 2.]);
                    r.path(a, b, [1., 0., 1.], [7., 0., 1.], 16, 16);
                    let f = CallbackFilter {
                        base: Filter::default(),
                        blocked: 255,
                        ignore_flags: false,
                        log: std::cell::RefCell::new(Vec::new()),
                    };
                    r.custom_path_at(a, b, [1., 0., 1.], [7., 0., 1.], &f, 16);
                    r.custom_path_at(b, a, [7., 0., 1.], [1., 0., 1.], &f, 16);
                    assert_eq!(r.add(&target, b), b);
                    let removed = r.remove(a);
                    assert_eq!(r.add(&removed, a), a); // reuses stale native link bytes
                    r.path(a, b, [1., 0., 1.], [7., 0., 1.], 16, 16);
                    r.finish(&format!(
                        "portals={portals} bidir={bidirectional} bv={bv} reverse={reversed_load}"
                    ));
                }
            }
        }
    }
}

#[test]
fn native_tile_state_statuses_padding_and_reference_restoration() {
    let verts = [[0, 0, 0], [0, 0, 4], [4, 0, 0]];
    let polys = [polygon(&[0, 1, 2], &[0xffff; 3], 0x1234, 17)];
    let mut p = params(&verts, &polys);
    p.coordinate = [0; 3];
    p.bounds = [[0.; 3], [4.; 3]];
    p.cell_size = 1.;
    p.cell_height = 1.;
    let mut data = create(&p).unwrap();
    // Native ignores inactive geometry entries but retains their initialized
    // bytes through loading, state operations, and export. These indices would
    // be invalid if a runtime algorithm accidentally used the full array.
    let polygon_base = HEADER_SIZE + verts.len() * 12;
    for i in 3..6 {
        let vertex = polygon_base + 4 + i * 2;
        data[vertex..vertex + 2].copy_from_slice(&(0xffe0 + i as u16).to_ne_bytes());
        let neighbor = polygon_base + 16 + i * 2;
        data[neighbor..neighbor + 2].copy_from_slice(&(0x1234 + i as u16).to_ne_bytes());
    }
    let mut r = Runtime::new(3, 1);
    let a = r.add(&data, 0);
    let size = r.mesh.tile_state_size(a);
    let state = r.store(a, size + 5);
    r.store(a, size - 1);
    r.flags(a, 0xabcd, 255);
    r.snapshot();
    r.restore(a, &state);
    for (at, xor) in [(0, 1), (4, 1), (8, 0x80)] {
        let mut invalid = state.clone();
        invalid[at] ^= xor;
        r.restore(a, &invalid);
    }
    r.restore(a, &state[..size - 1]);
    r.flags(0, 1, 1);
    r.remove(a);
    r.remove(a);
    let new = r.add(&data, 0);
    assert_ne!(new, a);
    r.restore(new, &state);
    r.remove(new);
    assert_eq!(r.add(&data, a), a);
    r.restore(a, &state);
    p.coordinate = [1, 0, 0];
    p.bounds = [[4., 0., 0.], [8., 4., 4.]];
    let b_data = create(&p).unwrap();
    let b = r.add(&b_data, 0);
    p.coordinate = [2, 0, 0];
    p.bounds = [[8., 0., 0.], [12., 4., 4.]];
    let c_data = create(&p).unwrap();
    r.add(&c_data, b); // requested slot occupied
    let c = r.add(&c_data, 0);
    p.coordinate = [3, 0, 0];
    p.bounds = [[12., 0., 0.], [16., 4., 4.]];
    let d_data = create(&p).unwrap();
    r.add(&d_data, 0); // tile pool exhausted
    r.remove(c);
    let invalid = r.mesh.reference_layout().encode(1, 3, 0);
    r.add(&d_data, invalid); // representable tile index beyond maxTiles
    r.finish("native state and lastRef");
    let wider = vec![polys[0].clone(); 3];
    let mut p = params(&verts, &wider);
    p.coordinate = [0; 3];
    p.bounds = [[0.; 3], [4.; 3]];
    let data = create(&p).unwrap();
    let mut r = Runtime::new(1, 1);
    r.add(&data, 0);
    r.finish("native per-tile polygon capacity differs between reference modes");
}

struct CallbackFilter {
    base: Filter,
    blocked: u8,
    ignore_flags: bool,
    log: std::cell::RefCell<Vec<i64>>,
}
impl Default for CallbackFilter {
    fn default() -> Self {
        Self {
            base: Filter::default(),
            blocked: 255,
            ignore_flags: false,
            log: std::cell::RefCell::new(Vec::new()),
        }
    }
}
impl QueryFilter for CallbackFilter {
    fn pass_filter(&self, p: FilterPolygon<'_>) -> bool {
        // Fixture-only area sentinel: reject the first callback, then allow it.
        let reject_first = self.blocked == 254 && self.log.borrow().is_empty();
        self.log.borrow_mut().extend([
            0,
            p.reference as i64,
            p.polygon.flags().into(),
            p.polygon.area().into(),
            p.tile.coordinate[0].into(),
            p.tile.coordinate[1].into(),
            p.tile.coordinate[2].into(),
            p.tile.header.user_id.into(),
        ]);
        !reject_first
            && (self.ignore_flags || self.base.pass_filter(p))
            && p.polygon.area() != self.blocked
    }
    fn cost(
        &self,
        a: Point,
        b: Point,
        previous: Option<FilterPolygon<'_>>,
        current: FilterPolygon<'_>,
        next: Option<FilterPolygon<'_>>,
    ) -> f32 {
        let mut log = self.log.borrow_mut();
        log.extend([
            1,
            previous.map_or(0, |p| p.reference as i64),
            current.reference as i64,
            next.map_or(0, |p| p.reference as i64),
            previous.map_or(255, |p| p.polygon.area().into()),
            current.polygon.area().into(),
            next.map_or(255, |p| p.polygon.area().into()),
        ]);
        for p in [previous, Some(current), next] {
            let (coordinate, id) =
                p.map_or(([0; 3], 0), |p| (p.tile.coordinate, p.tile.header.user_id));
            log.extend(coordinate.map(i64::from));
            log.push(id.into());
        }
        log.extend(a.into_iter().chain(b).map(|v| i64::from(v.to_bits())));
        let mut cost = self.base.cost(a, b, previous, current, next);
        if let Some(p) = previous {
            cost += f32::from(p.polygon.area()) * 0.125;
        }
        if let Some(p) = next {
            cost += f32::from(p.polygon.area()) * 0.0625;
        }
        cost += current.tile.coordinate[0] as f32 * 0.25;
        cost
    }
}
impl Runtime {
    fn filter(&mut self, f: &CallbackFilter) {
        write!(
            self.input,
            "{} {} {} {} ",
            f.base.include,
            f.base.exclude,
            f.blocked,
            u8::from(f.ignore_flags)
        )
        .unwrap();
        values(&mut self.input, f.base.area_costs.map(f32::to_bits));
        f.log.borrow_mut().clear();
    }
    fn custom_nearest(&mut self, p: Point, ext: Point, f: &CallbackFilter) {
        self.input.push_str("J ");
        self.filter(f);
        self.point(p);
        self.point(ext);
        let filter: &dyn QueryFilter = f;
        let (s, found) = self.mesh.nearest_with_over(p, ext, filter);
        let NearestPoint {
            reference: r,
            position: p,
            over_polygon: over,
        } = found.unwrap_or(NearestPoint {
            reference: 0,
            position: [0.; 3],
            over_polygon: false,
        });
        self.actual.push(serde_json::json!([
            s,
            r,
            p[0].to_bits(),
            p[1].to_bits(),
            p[2].to_bits(),
            u8::from(over),
            *f.log.borrow()
        ]));
    }
    fn custom_path(
        &mut self,
        a: PolyRef,
        b: PolyRef,
        f: &CallbackFilter,
        capacity: usize,
    ) -> Vec<PolyRef> {
        self.custom_path_at(a, b, [0.5, 0., 0.5], [3.5, 0., 3.5], f, capacity)
    }
    fn custom_polygons(
        &mut self,
        p: Point,
        ext: Point,
        f: &CallbackFilter,
        capacity: Option<usize>,
    ) {
        self.input
            .push_str(if capacity.is_some() { "Q " } else { "V " });
        self.filter(f);
        self.point(p);
        self.point(ext);
        if let Some(capacity) = capacity {
            writeln!(self.input, "{capacity}").unwrap();
            let (status, refs) = self.mesh.query_polygons(p, ext, f, capacity);
            self.actual
                .push(serde_json::json!([status, refs, *f.log.borrow()]));
        } else {
            let status = self.mesh.query_polygons_with(p, ext, f, |batch| {
                let t = batch[0].tile;
                let mut log = f.log.borrow_mut();
                log.extend([
                    2,
                    t.coordinate[0].into(),
                    t.coordinate[1].into(),
                    t.coordinate[2].into(),
                    t.header.user_id.into(),
                    batch.len() as i64,
                ]);
                for p in batch {
                    log.extend([p.reference as i64, p.polygon.area().into()]);
                }
            });
            self.actual
                .push(serde_json::json!([status, *f.log.borrow()]));
        }
    }
    fn custom_path_at(
        &mut self,
        a: PolyRef,
        b: PolyRef,
        start: Point,
        end: Point,
        f: &CallbackFilter,
        capacity: usize,
    ) -> Vec<PolyRef> {
        self.input.push_str("K ");
        self.filter(f);
        write!(self.input, "{a} {b} ").unwrap();
        self.point(start);
        self.point(end);
        writeln!(self.input, "{capacity}").unwrap();
        let filter: &dyn QueryFilter = f;
        let mut query = NavMeshQuery::new(64).unwrap();
        let path = query.find_path(&self.mesh, a, b, start, end, filter, capacity);
        self.actual.push(serde_json::json!([
            path.status,
            path.polygons,
            *f.log.borrow(),
            node_snapshot::snapshot(query.node_pool())
        ]));
        path.polygons
    }
}

#[test]
fn native_polygon_query_batches_callbacks_and_buffer_capacities() {
    let verts: Vec<_> = (0..9)
        .flat_map(|z| (0..9).map(move |x| [x, 0, z]))
        .collect();
    let polys: Vec<_> = (0..8)
        .flat_map(|z| {
            (0..8).map(move |x| {
                let i = z * 8 + x;
                let a = z * 9 + x;
                polygon(
                    &[a, a + 9, a + 10, a + 1],
                    &[0xffff; 4],
                    if i % 5 == 4 { 0 } else { 1 << (i % 8) },
                    (i % 63) as u8,
                )
            })
        })
        .collect();
    for bv in [false, true] {
        let mut r = Runtime::new(2, 128);
        let mut p = params(&verts, &polys);
        p.bounds = [[0.; 3], [4.; 3]];
        p.coordinate = [0; 3];
        p.cell_size = 0.5;
        p.build_bv_tree = bv;
        r.add(&create(&p).unwrap(), 0);
        p.coordinate[2] = 1;
        p.user_id = 0x1122_3344;
        r.add(&create(&p).unwrap(), 0);
        for ignore_flags in [false, true] {
            let f = CallbackFilter {
                base: Filter::default(),
                blocked: 255,
                ignore_flags,
                log: std::cell::RefCell::new(Vec::new()),
            };
            for (point, ext) in [
                ([2., 1., 2.], [3.; 3]),
                ([0.25, 1., 0.25], [0.1, 2., 0.1]),
                ([20.; 3], [0.1; 3]),
                ([2.; 3], [-1.; 3]),
                ([f32::NAN, 0., 0.], [1.; 3]),
            ] {
                r.custom_polygons(point, ext, &f, None);
                for capacity in [0, 1, 31, 32, 33, 128, 129, 130] {
                    r.custom_polygons(point, ext, &f, Some(capacity));
                }
            }
        }
        r.finish(&format!("polygon callback batches bv={bv}"));
    }
}

#[test]
fn native_virtual_filter_contexts_costs_and_callback_order() {
    let verts: Vec<_> = (0..3)
        .flat_map(|z| (0..3).map(move |x| [x * 2, 0, z * 2]))
        .collect();
    let polys = [
        polygon(&[0, 3, 4, 1], &[0xffff, 2, 1, 0xffff], 1, 3),
        polygon(&[1, 4, 5, 2], &[0, 3, 0xffff, 0xffff], 2, 7),
        polygon(&[3, 6, 7, 4], &[0xffff, 0xffff, 3, 0], 4, 11),
        polygon(&[4, 7, 8, 5], &[2, 0xffff, 0xffff, 1], 8, 13),
    ];
    for bv in [false, true] {
        let mut p = params(&verts, &polys);
        p.bounds = [[0.; 3], [4.; 3]];
        p.coordinate = [0; 3];
        p.cell_size = 1.;
        p.cell_height = 1.;
        p.build_bv_tree = bv;
        let data = create(&p).unwrap();
        let mut r = Runtime::new(1, 4);
        let a = r.add(&data, 0);
        for (include, exclude, blocked, ignore_flags) in [
            (0xffff, 0, 255, false),
            (0xffff, 0, 7, false),
            (0, 0xffff, 255, true),
            (0xffff, 8, 255, false),
        ] {
            let mut f = CallbackFilter {
                base: Filter {
                    include,
                    exclude,
                    ..Filter::default()
                },
                blocked,
                ignore_flags,
                log: std::cell::RefCell::new(Vec::new()),
            };
            f.base.area_costs[7] = 4.25;
            for (p, e) in [
                ([0.5, 1., 0.5], [0.1, 2., 0.1]),
                ([2., 1., 2.], [3.; 3]),
                ([20., 0., 20.], [0.1; 3]),
            ] {
                r.custom_nearest(p, e, &f);
            }
            let path = r.custom_path(a, a | 3, &f, 16);
            if exclude == 0 || ignore_flags {
                assert_eq!(path.last(), Some(&(a | 3)));
                assert!(path.contains(&(a | 2))); // changed cost chooses the upper route
            }
            r.custom_path(a, a | 3, &f, 2);
        }
        r.finish(&format!("virtual callback bv={bv}"));
    }
}

impl Runtime {
    fn valid_ref(&mut self, reference: PolyRef, f: &CallbackFilter) {
        self.input.push_str("W ");
        self.filter(f);
        writeln!(self.input, "{reference}").unwrap();
        let valid = self.query.is_valid_poly_ref(&self.mesh, reference, f);
        self.actual.push(json!([u8::from(valid), *f.log.borrow()]));
    }
}
#[test]
fn persistent_query_nodes_closed_list_and_retained_capacity() {
    let verts: Vec<_> = (0..=8).flat_map(|x| [[x, 0, 0], [x, 0, 4]]).collect();
    let polys: Vec<_> = (0..8)
        .map(|x| {
            let a = x * 2;
            polygon(
                &[a, a + 1, a + 3, a + 2],
                &[
                    if x == 0 { 0xffff } else { x - 1 },
                    0xffff,
                    if x == 7 { 0xffff } else { x + 1 },
                    0xffff,
                ],
                1,
                63,
            )
        })
        .collect();
    let mut p = params(&verts, &polys);
    p.bounds = [[0.; 3], [4.; 3]];
    p.coordinate = [0; 3];
    p.cell_size = 0.5;
    let data = create(&p).unwrap();
    let mut r = Runtime::with_nodes(1, 8, 4);
    let a = r.add(&data, 0);
    let refs: Vec<_> = (0..8).map(|i| a | i).collect();
    r.query_snapshot();
    r.closed(&refs);
    r.persistent_path(a, a | 7, [0.25, 0., 1.], [3.75, 0., 1.], 16);
    r.closed(&refs);
    r.query_init(1);
    r.query_snapshot();
    r.query_init(0);
    r.query_init(3);
    r.query_snapshot();
    r.query_init(9);
    r.persistent_path(a, a | 7, [0.25, 0., 1.], [3.75, 0., 1.], 3);
    r.closed(&refs);
    r.persistent_path(a, a, [0.25, 0., 1.], [0.25, 0., 1.], 16); // retains the existing pool
    r.persistent_path(0, a, [0.; 3], [0.; 3], 16); // invalid input also retains it
    r.query_init(65536);
    r.query_snapshot();
    let f = CallbackFilter {
        base: Filter::default(),
        blocked: 255,
        ignore_flags: false,
        log: std::cell::RefCell::new(Vec::new()),
    };
    r.valid_ref(a, &f);
    r.valid_ref(0, &f);
    r.remove(a);
    r.closed(&refs);
    r.valid_ref(a, &f);
    r.add(&data, a);
    r.valid_ref(a, &f);
    r.query_init(4);
    r.query_snapshot();
    r.persistent_path(a, a | 7, [0.25, 0., 1.], [3.75, 0., 1.], 16);
    r.closed(&refs);
    r.finish("persistent query state");
}

impl Runtime {
    #[allow(clippy::too_many_arguments)]
    fn ray(
        &mut self,
        start: PolyRef,
        previous: PolyRef,
        a: Point,
        b: Point,
        options: u32,
        capacity: usize,
        f: &CallbackFilter,
        legacy: bool,
    ) {
        self.input.push_str(if legacy { "Z " } else { "Y " });
        self.filter(f);
        if legacy {
            write!(self.input, "{start} {capacity} ").unwrap();
        } else {
            write!(self.input, "{start} {previous} {options} {capacity} ").unwrap();
        }
        self.point(a);
        self.point(b);
        if legacy {
            let (s, hit) = self.query.raycast(&self.mesh, start, a, b, f, capacity);
            self.actual.push(json!([
                s,
                hit.t.to_bits(),
                hit.normal[0].to_bits(),
                hit.normal[1].to_bits(),
                hit.normal[2].to_bits(),
                hit.path,
                *f.log.borrow()
            ]));
        } else {
            let mut hit = RaycastHit {
                t: -9.,
                normal: [-0., 42., -3.],
                edge_index: -77,
                path: vec![0],
                path_cost: -13.,
            };
            let s = self.query.raycast_into(
                &self.mesh, start, a, b, f, options, previous, capacity, &mut hit,
            );
            self.actual.push(json!([
                s,
                hit.t.to_bits(),
                hit.normal[0].to_bits(),
                hit.normal[1].to_bits(),
                hit.normal[2].to_bits(),
                hit.edge_index,
                hit.path_cost.to_bits(),
                hit.path,
                *f.log.borrow()
            ]));
        }
    }
}

#[test]
fn native_raycast_hits_costs_contexts_and_output_capacity() {
    let verts = [
        [0, 0, 0],
        [0, 0, 4],
        [2, 1, 4],
        [2, 1, 0],
        [4, 2, 4],
        [4, 2, 0],
    ];
    let polys = [
        polygon(&[0, 1, 2, 3], &[0xffff, 0xffff, 1, 0xffff], 1, 3),
        polygon(&[3, 2, 4, 5], &[0, 0xffff, 0xffff, 0xffff], 2, 7),
    ];
    let mut p = params(&verts, &polys);
    p.bounds = [[0.; 3], [4.; 3]];
    p.coordinate = [0; 3];
    p.cell_size = 1.;
    p.cell_height = 0.3;
    let data = create(&p).unwrap();
    let mut r = Runtime::new(1, 2);
    let a = r.add(&data, 0);
    let b = a | 1;
    r.persistent_path(a, b, [1., 0., 2.], [3., 0., 2.], 16);
    let mut f = CallbackFilter {
        base: Filter::default(),
        blocked: 255,
        ignore_flags: false,
        log: std::cell::RefCell::new(Vec::new()),
    };
    f.base.area_costs[7] = 2.25;
    for blocked in [255, 7, 3] {
        f.blocked = blocked;
        for (start, end) in [
            ([1., 0.2, 2.], [3., 20., 2.]),
            ([1., 0.2, 2.], [7., -4., 2.]),
            ([1., 0., 1.], [1., 3., 1.]),
            ([-2., 0., 2.], [7., 0., 2.]),
            ([-2., 0., -1.], [-3., 0., -1.]),
            ([0., 0., 0.], [-1., 0., 0.]),
            ([1., 0., 1.], [f32::MAX, 0., f32::MAX]),
        ] {
            r.ray(a, 0, start, end, 0, 16, &f, true);
            for capacity in [0, 1, 2, 16] {
                for options in [0, 1, 3] {
                    r.ray(a, 0, start, end, options, capacity, &f, false);
                }
            }
        }
    }
    for previous in [0, b] {
        r.ray(a, previous, [1., 0.2, 2.], [3., 20., 2.], 1, 16, &f, false);
    }
    r.ray(0, 0, [1.; 3], [3.; 3], 1, 16, &f, false);
    r.ray(a, 0, [f32::NAN, 0., 0.], [3.; 3], 1, 16, &f, false);
    r.query_snapshot();
    r.remove(a);
    r.ray(a, 0, [1.; 3], [3.; 3], 1, 16, &f, false);
    r.finish("raycast costs and contexts");
    let mut r = Runtime::new(1, 2);
    assert_eq!(r.add(&data, 1), 0); // native accepts a caller-supplied zero salt
    r.ray(1, 0, [3., 0., 2.], [1., 0., 2.], 1, 16, &f, false);
    r.random(None, &[0.], CallbackFilter::default());
    r.random(None, &[1.], CallbackFilter::default());
    r.finish("raycast zero-ref internal candidate");
    let off = [CreateOffMeshConnection {
        positions: [[1., 0.1, 2.], [3., 0.3, 2.]],
        radius: 2.5,
        flags: 4,
        area: 5,
        direction: 1,
    }];
    p.off_mesh_connections = &off;
    let mut r = Runtime::new(1, 4);
    r.add(&create(&p).unwrap(), 1);
    r.finish("zero-ref nearest candidate during off-mesh basing");
}

#[test]
fn native_raycast_partial_tile_portals_and_slanted_normals() {
    for rotate in [false, true] {
        let transform = |p: Point| if rotate { [p[2], p[1], 8. - p[0]] } else { p };
        let rotate_vertex = |v: [u16; 3]| if rotate { [v[2], v[1], 4 - v[0]] } else { v };
        let a_verts = [[0, 0, 0], [0, 0, 4], [4, 0, 4], [4, 0, 0]].map(rotate_vertex);
        let b_verts = [[0, 0, 1], [0, 0, 3], [4, 0, 3], [4, 0, 1]].map(rotate_vertex);
        let a_nei = if rotate {
            [0x8001, 0x8002, 0x8003, 0x8000]
        } else {
            [0x8000, 0x8001, 0x8002, 0x8003]
        };
        let b_nei = if rotate {
            [0x8001, 0xffff, 0x8003, 0xffff]
        } else {
            [0x8000, 0xffff, 0x8002, 0xffff]
        };
        let a_poly = [polygon(&[0, 1, 2, 3], &a_nei, 1, 3)];
        let b_poly = [polygon(&[0, 1, 2, 3], &b_nei, 2, 11)];
        let off = [CreateOffMeshConnection {
            positions: [transform([1., 0., 0.5]), transform([7., 0., 0.5])],
            radius: 0.8,
            flags: 4,
            area: 5,
            direction: 1,
        }];
        let mut ap = params(&a_verts, &a_poly);
        ap.coordinate = if rotate { [0, 1, 0] } else { [0; 3] };
        ap.bounds = if rotate {
            [[0., 0., 4.], [4., 4., 8.]]
        } else {
            [[0.; 3], [4.; 3]]
        };
        ap.cell_size = 1.;
        ap.cell_height = 1.;
        ap.walkable_climb = 1.;
        ap.off_mesh_connections = &off;
        let mut bp = params(&b_verts, &b_poly);
        bp.coordinate = if rotate { [0; 3] } else { [1, 0, 0] };
        bp.bounds = if rotate {
            [[0.; 3], [4.; 3]]
        } else {
            [[4., 0., 0.], [8., 4., 4.]]
        };
        bp.cell_size = 1.;
        bp.cell_height = 1.;
        bp.walkable_climb = 1.;
        bp.user_id = 0x5566_7788;
        let mut r = Runtime::new(4, 4);
        let a = r.add(&create(&ap).unwrap(), 0);
        let b = r.add(&create(&bp).unwrap(), 0);
        let mut f = CallbackFilter {
            base: Filter::default(),
            blocked: 255,
            ignore_flags: false,
            log: std::cell::RefCell::new(Vec::new()),
        };
        for blocked in [255, 11] {
            f.blocked = blocked;
            for z in [0.5, 1., 1.01, 2., 2.99, 3., 3.5] {
                let start = transform([1., 0.2, z]);
                let end = transform([7., 10., z]);
                for options in [0, 1] {
                    r.ray(a, 0, start, end, options, 16, &f, false);
                    r.ray(b, a, end, start, options, 1, &f, false);
                }
            }
        }
        r.finish(&format!("partial ray portal rotate={rotate}"));
    }
    let verts = [[0, 0, 1], [1, 2, 6], [6, 3, 5], [5, 1, 0]];
    let polys = [polygon(&[0, 1, 2, 3], &[0xffff; 4], 1, 3)];
    let mut p = params(&verts, &polys);
    p.coordinate = [0; 3];
    p.bounds = [[0.17, -0.23, 0.31], [4., 3., 4.]];
    p.cell_size = 0.47;
    p.cell_height = 0.19;
    let mut r = Runtime::new(1, 1);
    let a = r.add(&create(&p).unwrap(), 0);
    let f = CallbackFilter {
        base: Filter::default(),
        blocked: 255,
        ignore_flags: false,
        log: std::cell::RefCell::new(Vec::new()),
    };
    let mut seed = 0x12e4_a53fu32;
    for _ in 0..32 {
        let end = std::array::from_fn(|_| {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            (seed >> 16) as f32 / 8192. - 2.
        });
        r.ray(a, 0, [1.5, 2., 1.6], end, 1, 16, &f, false);
    }
    r.finish("slanted wall normals and costs");
}

#[test]
fn native_bounded_layer_disconnection_keeps_defined_stale_link_queries() {
    let verts = [[0, 0, 0], [0, 0, 4], [4, 0, 4], [4, 0, 0]];
    let polys = [polygon(&[0, 1, 2, 3], &[0xffff; 4], 1, 3)];
    let connection = [CreateOffMeshConnection {
        positions: [[1., 0., 1.], [1., 10., 1.]],
        radius: 0.8,
        flags: 2,
        area: 5,
        direction: 1,
    }];
    let mut p = params(&verts, &polys);
    p.coordinate = [0; 3];
    p.bounds = [[0.; 3], [4., 500., 4.]];
    p.cell_size = 1.;
    p.cell_height = 1.;
    p.off_mesh_connections = &connection;
    let mut r = Runtime::new(34, 4);
    let owner = r.add(&create(&p).unwrap(), 0);
    p.off_mesh_connections = &[];
    p.coordinate[2] = 1;
    let target_verts = verts.map(|mut v| {
        v[1] = 10;
        v
    });
    p.vertices = &target_verts;
    let target_data = create(&p).unwrap();
    let old = r.add(&target_data, 0);
    for layer in 2..34 {
        let layer_verts = verts.map(|mut v| {
            v[1] = layer * 10;
            v
        });
        let lp = CreateParams {
            vertices: &layer_verts,
            coordinate: [0, 0, layer as i32],
            ..p.clone()
        };
        r.add(&create(&lp).unwrap(), 0);
    }
    r.remove(old);
    let replacement = r.add(&target_data, 0);
    assert_ne!(replacement, old);
    r.persistent_path(owner, replacement, [1., 0., 1.], [1., 10., 1.], 16);
    r.around(
        owner,
        [1., 0., 1.],
        20.,
        None,
        16,
        CallbackFilter::default(),
    );
    r.dijkstra_path(replacement, 16);
    r.dijkstra_path(old, 16);
    r.finish("native 32-layer disconnect lookup and stale slot contents");
}

impl Runtime {
    #[allow(clippy::too_many_arguments)]
    fn begin_slice(
        &mut self,
        start: PolyRef,
        end: PolyRef,
        a: Point,
        b: Point,
        options: u32,
        filter: CallbackFilter,
    ) -> u32 {
        self.input.push_str("b ");
        self.filter(&filter);
        write!(self.input, "{start} {end} {options} ").unwrap();
        self.point(a);
        self.point(b);
        self.sliced_filter = filter;
        let s = self.query.init_sliced_find_path(
            &self.mesh,
            start,
            end,
            a,
            b,
            &self.sliced_filter,
            options,
        );
        self.actual
            .push(json!([s, *self.sliced_filter.log.borrow()]));
        self.query_snapshot();
        s
    }
    fn update_slice(&mut self, iterations: i32, with_done: bool) -> u32 {
        writeln!(self.input, "u {iterations} {}", u8::from(with_done)).unwrap();
        self.sliced_filter.log.borrow_mut().clear();
        let mut done = -17;
        let s = self.query.update_sliced_find_path(
            &self.mesh,
            &self.sliced_filter,
            iterations,
            if with_done { Some(&mut done) } else { None },
        );
        self.actual
            .push(json!([s, done, *self.sliced_filter.log.borrow()]));
        self.query_snapshot();
        s
    }
    fn finish_slice(&mut self, capacity: usize, existing: Option<&[PolyRef]>) -> Vec<PolyRef> {
        if let Some(existing) = existing {
            write!(self.input, "p {capacity} {} ", existing.len()).unwrap();
            values(&mut self.input, existing);
        } else {
            writeln!(self.input, "f {capacity}").unwrap();
        }
        self.sliced_filter.log.borrow_mut().clear();
        if let Some(existing) = existing {
            self.query.finalize_sliced_find_path_partial_into(
                &self.mesh,
                existing,
                &self.sliced_filter,
                capacity,
                &mut self.path_scratch,
            )
        } else {
            self.query.finalize_sliced_find_path_into(
                &self.mesh,
                &self.sliced_filter,
                capacity,
                &mut self.path_scratch,
            )
        };
        let path = &self.path_scratch;
        self.actual.push(json!([
            path.status,
            path.polygons,
            *self.sliced_filter.log.borrow()
        ]));
        let polygons = path.polygons.clone();
        self.query_snapshot();
        polygons
    }
    fn edit_slice_filter(&mut self, filter: CallbackFilter) {
        self.input.push_str("e ");
        self.filter(&filter);
        self.sliced_filter = filter;
        self.actual.push(json!([]));
    }
    fn seed_positions(&mut self, point: Point) {
        self.input.push_str("n ");
        self.point(point);
        let pool = self.query.node_pool_mut();
        pool.clear();
        for i in 0..pool.max_nodes() {
            let node = pool.get_node((i + 1) as PolyRef, 0).unwrap();
            pool.node_mut(node).unwrap().position = point;
        }
        pool.clear();
        self.actual.push(json!([]));
    }
}

fn sliced_chain_data(radius: f32) -> Vec<u8> {
    let verts: Vec<_> = (0..=8).flat_map(|x| [[x, 0, 0], [x, 0, 4]]).collect();
    let polys: Vec<_> = (0..8)
        .map(|x| {
            let a = x * 2;
            polygon(
                &[a, a + 1, a + 3, a + 2],
                &[
                    if x == 0 { 0xffff } else { x - 1 },
                    0xffff,
                    if x == 7 { 0xffff } else { x + 1 },
                    0xffff,
                ],
                1,
                (x + 3) as u8,
            )
        })
        .collect();
    let mut p = params(&verts, &polys);
    p.bounds = [[0.; 3], [4.; 3]];
    p.coordinate = [0; 3];
    p.cell_size = 0.5;
    p.walkable_radius = radius;
    create(&p).unwrap()
}

#[test]
fn nonfinite_filter_costs_retain_query_node_bits() {
    let costs = [
        1.,
        f32::from_bits(0x7fc1_2345),
        f32::from_bits(0xffc5_4321),
        f32::from_bits(0x7f81_2345),
        f32::from_bits(0xff85_4321),
        f32::INFINITY,
    ];
    let data = sliced_chain_data(0.3);
    let mut comparisons = Vec::new();
    for a in costs {
        for b in costs {
            let filter = || {
                let mut filter = CallbackFilter::default();
                for (i, cost) in filter.base.area_costs.iter_mut().enumerate() {
                    *cost = if i % 2 == 0 { a } else { b };
                }
                filter
            };
            let mut runtime = Runtime::with_nodes(1, 8, 64);
            let start = runtime.add(&data, 0);
            let end = start | 7;
            runtime.persistent_path_filtered(
                start,
                end,
                [0.25, 0., 1.],
                [3.75, 0., 1.],
                16,
                &filter().base,
            );
            runtime.custom_path_at(start, end, [0.25, 0., 1.], [3.75, 0., 1.], &filter(), 16);
            runtime.ray(
                start,
                0,
                [0.25, 0., 1.],
                [1., 0., 1.],
                RAYCAST_USE_COSTS,
                0,
                &filter(),
                false,
            );
            runtime.around(start, [0.25, 0., 1.], 10., None, 16, filter());
            runtime.dijkstra_path(end, 16);
            for options in [0, FINDPATH_ANY_ANGLE] {
                runtime.begin_slice(
                    start,
                    end,
                    [0.25, 0., 1.],
                    [3.75, 0., 1.],
                    options,
                    filter(),
                );
                for _ in 0..8 {
                    if runtime.update_slice(1, true) & IN_PROGRESS == 0 {
                        break;
                    }
                }
                runtime.finish_slice(16, None);
            }
            let context = format!("query costs {:08x}/{:08x}", a.to_bits(), b.to_bits());
            let expected = oracle(runtime.input.clone());
            parity_trace::write(
                "query-costs",
                &runtime.input,
                &runtime.actual,
                expected.as_array().unwrap(),
            );
            comparisons.push((context, Value::Array(runtime.actual), expected));
        }
    }
    for (context, actual, expected) in comparisons {
        float_comparison::assert_eq(
            &actual,
            &expected,
            |path| match path {
                // Node-pool snapshots, including the custom-filter path result.
                [_, 5, _, 7 | 8] | [_, 3, 5, _, 7 | 8] => true,
                // Around-query entries are [reference, parent, calculated cost].
                [record, 1, entry, 2] => actual[*record].as_array().is_some_and(|row| {
                    row.len() == 3
                        && row[0].is_number()
                        && row[2].is_array()
                        && row[1][*entry]
                            .as_array()
                            .is_some_and(|entry| entry.len() == 3)
                }),
                // RaycastHit.path_cost; the last two fields are path and callbacks.
                [record, 6] => actual[*record]
                    .as_array()
                    .is_some_and(|row| row.len() == 9 && row[7].is_array() && row[8].is_array()),
                _ => false,
            },
            &context,
        );
    }
}

#[test]
fn native_sliced_search_steps_any_angle_finalize_and_partial_paths() {
    for nodes in [4, 64] {
        for options in [0, FINDPATH_ANY_ANGLE] {
            for radius in [0., 0.3] {
                let data = sliced_chain_data(radius);
                let mut r = Runtime::with_nodes(1, 8, nodes);
                let a = r.add(&data, 0);
                let b = a | 7;
                let refs: Vec<_> = (0..8).map(|i| a | i).collect();
                for capacity in [1, 2, 7, 8, 9, 16] {
                    assert_eq!(
                        r.begin_slice(
                            a,
                            b,
                            [0.25, 0., 1.],
                            [3.75, 0., 1.],
                            options,
                            CallbackFilter::default()
                        ),
                        IN_PROGRESS
                    );
                    r.update_slice(0, true);
                    r.update_slice(-3, true);
                    for iterations in [1, 2, 1, 20] {
                        r.update_slice(iterations, true);
                    }
                    r.closed(&refs);
                    let path = r.finish_slice(capacity, None);
                    if nodes == 64 && options == FINDPATH_ANY_ANGLE && radius > 0. && capacity == 7
                    {
                        assert_eq!(
                            path, refs,
                            "native detached ray fills budget before final append"
                        );
                    }
                    r.update_slice(1, true);
                }
                for iterations in [0, 1, 3, 20] {
                    for existing in [&refs[..], &refs[..4], &[b][..], &[0][..]] {
                        r.begin_slice(
                            a,
                            b,
                            [0.25, 0., 1.],
                            [3.75, 0., 1.],
                            options,
                            CallbackFilter::default(),
                        );
                        r.update_slice(iterations, false);
                        r.finish_slice(8, Some(existing));
                    }
                }
                r.begin_slice(
                    a,
                    b,
                    [0.25, 0., 1.],
                    [3.75, 0., 1.],
                    options,
                    CallbackFilter::default(),
                );
                r.update_slice(2, true);
                let changed = CallbackFilter {
                    blocked: 7,
                    ..CallbackFilter::default()
                };
                r.edit_slice_filter(changed);
                r.update_slice(20, true);
                r.finish_slice(16, None);
                r.finish(&format!(
                    "sliced nodes={nodes} options={options} radius={radius}"
                ));
            }
        }
    }
}

#[test]
fn native_sliced_validation_reset_and_mesh_mutation() {
    let data = sliced_chain_data(0.3);
    let mut r = Runtime::new(1, 8);
    let a = r.add(&data, 0);
    let b = a | 7;
    r.update_slice(1, true);
    r.finish_slice(16, None);
    r.finish_slice(16, None);
    r.begin_slice(
        a,
        a,
        [0.25, 0., 1.],
        [0.25, 0., 1.],
        FINDPATH_ANY_ANGLE,
        CallbackFilter::default(),
    );
    r.update_slice(1, true);
    r.finish_slice(0, None);
    r.finish_slice(1, Some(&[]));
    r.finish_slice(1, None);
    r.begin_slice(0, b, [0.; 3], [3.75, 0., 1.], 0, CallbackFilter::default());
    r.update_slice(4, true);
    r.finish_slice(0, None);
    r.finish_slice(16, None);
    r.update_slice(1, true);
    r.begin_slice(
        a,
        b,
        [f32::NAN, 0., 0.],
        [3.75, 0., 1.],
        0,
        CallbackFilter::default(),
    );
    r.query_init(128);
    r.update_slice(1, true);
    r.finish_slice(16, None);
    for options in [0, FINDPATH_ANY_ANGLE] {
        r.begin_slice(
            a,
            b,
            [0.25, 0., 1.],
            [3.75, 0., 1.],
            options,
            CallbackFilter::default(),
        );
        r.update_slice(1, true);
        let removed = r.remove(a);
        r.update_slice(20, true);
        r.finish_slice(16, None);
        r.add(&removed, a);
    }
    r.finish("sliced validation and reset");

    // Start/end remain live while the current node, its parent, or its
    // grandparent disappears. Each failure occurs after a different pop.
    let verts = [[0, 0, 0], [0, 0, 4], [4, 0, 4], [4, 0, 0]];
    let polys = [polygon(
        &[0, 1, 2, 3],
        &[0x8000, 0xffff, 0x8002, 0xffff],
        1,
        3,
    )];
    for options in [0, FINDPATH_ANY_ANGLE] {
        for removed_index in [1, 2, 3] {
            let mut r = Runtime::new(6, 1);
            let mut refs = Vec::new();
            for x in 0..6 {
                let mut p = params(&verts, &polys);
                p.coordinate = [x, 0, 0];
                p.bounds = [[x as f32 * 4., 0., 0.], [(x + 1) as f32 * 4., 4., 4.]];
                p.cell_size = 1.;
                // Disable shortcuts so the grandparent is retained, while
                // still exercising the any-angle branch's validity checks.
                p.walkable_radius = 0.;
                refs.push(r.add(&create(&p).unwrap(), 0));
            }
            r.begin_slice(
                refs[0],
                refs[5],
                [1., 0., 1.],
                [23., 0., 1.],
                options,
                CallbackFilter::default(),
            );
            r.update_slice(3, true);
            r.remove(refs[removed_index]);
            assert_eq!(r.update_slice(1, true), FAILURE);
            r.finish_slice(16, None);
            r.finish(&format!(
                "sliced disappeared ancestor {removed_index}, options={options}"
            ));
        }
    }
}

#[test]
fn native_failed_portal_midpoint_retains_initialized_node_positions() {
    let verts = [[0, 0, 0], [0, 0, 4], [4, 0, 4], [4, 0, 0]];
    let polys = [polygon(&[0, 1, 2, 3], &[0xffff; 4], 1, 3)];
    let off = [CreateOffMeshConnection {
        positions: [[1., 0., 1.], [1., 10., 1.]],
        radius: 0.8,
        flags: 2,
        area: 5,
        direction: 1,
    }];
    let mut p = params(&verts, &polys);
    p.coordinate = [0; 3];
    p.bounds = [[0.; 3], [4., 20., 4.]];
    p.cell_size = 1.;
    p.cell_height = 1.;
    p.off_mesh_connections = &off;
    let mut r = Runtime::new(8, 4);
    let owner = r.add(&create(&p).unwrap(), 0);
    let target_vertices = verts.map(|mut v| {
        v[1] = 10;
        v
    });
    p.vertices = &target_vertices;
    p.off_mesh_connections = &[];
    let mut target = 0;
    for layer in 1..=7 {
        p.coordinate[2] = layer;
        target = r.add(&create(&p).unwrap(), 0);
    }
    r.seed_positions([1., 5., 1.]);
    r.persistent_path(target, owner, [1., 10., 1.], [1., 0., 1.], 16);
    for options in [0, FINDPATH_ANY_ANGLE] {
        r.seed_positions([1., 5., 1.]);
        r.begin_slice(
            target,
            owner,
            [1., 10., 1.],
            [1., 0., 1.],
            options,
            CallbackFilter::default(),
        );
        r.update_slice(64, true);
        r.finish_slice(16, None);
    }
    r.finish("native ignored midpoint failure with initialized pool storage");
}

#[test]
fn native_straight_path_crossings_turns_and_broken_corridors() {
    let verts: Vec<_> = (0..=3)
        .flat_map(|z| (0..=3).map(move |x| [x * 4, x + z, z * 4]))
        .collect();
    let polys: Vec<_> = (0..3)
        .flat_map(|z| {
            (0..3).map(move |x| {
                let a = z * 4 + x;
                let poly = z * 3 + x;
                polygon(
                    &[a, a + 4, a + 5, a + 1],
                    &[
                        if x == 0 { 0xffff } else { poly - 1 },
                        if z == 2 { 0xffff } else { poly + 3 },
                        if x == 2 { 0xffff } else { poly + 1 },
                        if z == 0 { 0xffff } else { poly - 3 },
                    ],
                    1,
                    if x == 1 { 3 } else { 7 },
                )
            })
        })
        .collect();
    let mut p = params(&verts, &polys);
    p.coordinate = [0; 3];
    p.bounds = [[0.; 3], [12.; 3]];
    p.cell_size = 1.;
    p.cell_height = 0.3;
    let mut r = Runtime::new(1, 16);
    let a = r.add(&create(&p).unwrap(), 0);
    let corridors = [
        vec![a, a | 1, a | 2, a | 5, a | 8],
        vec![a, a | 3, a | 6, a | 7, a | 8],
        vec![a, a | 1, a | 4, a | 7, a | 8],
        vec![a, a | 1, a | 8], // valid endpoints, missing middle portal
        vec![a, a | 1, 0, a | 8],
        vec![a, a, a | 1, a | 8],
        vec![a, a | 1, a | 2],
        vec![a],
        vec![],
    ];
    for mut path in corridors {
        for reverse in [false, true] {
            let (start, end) = if reverse {
                ([11., 3., 11.], [1., 0., 1.])
            } else {
                ([1., 0., 1.], [11., 3., 11.])
            };
            if reverse {
                path.reverse();
            }
            for options in [0, 1, 2, 3, 0x80] {
                for capacity in [0, 1, 2, 3, 4, 5, 16] {
                    r.straight(start, end, &path, capacity, options);
                }
            }
        }
    }
    for point in [[4., 0., 1.], [4.00001, 0., 1.], [1., f32::NAN, 1.]] {
        r.straight(point, [11., 0., 1.], &[a, a | 1, a | 2], 16, 2);
    }
    // Recovery clamps the original endpoint to the last reachable polygon,
    // rather than re-clamping its projection onto the final polygon.
    for options in [0, 1, 2, 3] {
        r.straight(
            [1., 0., 1.],
            [20., 3., -5.],
            &[a, a | 1, a | 8],
            16,
            options,
        );
    }
    r.finish("straight path crossing policies, turns, partial recovery and numeric bits");
}

#[test]
fn native_surface_movement_walls_filters_heights_and_small_pool_budget() {
    for size in [3u16, 10] {
        let stride = size + 1;
        let verts: Vec<_> = (0..=size)
            .flat_map(|z| (0..=size).map(move |x| [x * 2, x + z, z * 2]))
            .collect();
        let polys: Vec<_> = (0..size)
            .flat_map(|z| {
                (0..size).map(move |x| {
                    let a = z * stride + x;
                    let poly = z * size + x;
                    polygon(
                        &[a, a + stride, a + stride + 1, a + 1],
                        &[
                            if x == 0 { 0xffff } else { poly - 1 },
                            if z + 1 == size { 0xffff } else { poly + size },
                            if x + 1 == size { 0xffff } else { poly + 1 },
                            if z == 0 { 0xffff } else { poly - size },
                        ],
                        1 << (poly % 4),
                        (poly % 7) as u8,
                    )
                })
            })
            .collect();
        let mut p = params(&verts, &polys);
        p.coordinate = [0; 3];
        p.bounds = [[0.; 3], [size as f32 * 2.; 3]];
        p.cell_size = 1.;
        p.cell_height = 0.3;
        let mut r = Runtime::with_nodes(1, polys.len(), 4);
        let a = r.add(&create(&p).unwrap(), 0);
        r.persistent_path(
            a,
            a | (polys.len() - 1) as PolyRef,
            [1., 0., 1.],
            [size as f32 * 2. - 1., 0., size as f32 * 2. - 1.],
            16,
        );
        let mut seed = 0x9753_bde1u32;
        for i in 0..64 {
            let mut point = || {
                std::array::from_fn(|_| {
                    seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                    ((seed >> 16) as f32 / 65535. * 3. - 0.5) * size as f32
                })
            };
            let b = match i {
                0 => [1., 99., 1.],
                1 => [100.; 3],
                2 => [1., f32::NAN, 1.],
                3 => [size as f32 * 2. - 0.5, 17., size as f32 * 2. - 0.5],
                _ => point(),
            };
            let filter = CallbackFilter {
                blocked: if i % 3 == 0 { 3 } else { 255 },
                base: Filter {
                    exclude: if i % 5 == 0 { 4 } else { 0 },
                    ..Filter::default()
                },
                ..CallbackFilter::default()
            };
            let capacity = [0, 1, 2, 4, 16, 64][i % 6];
            r.surface(
                if i == 4 { 0 } else { a },
                [0.75, 5., 0.75],
                b,
                capacity,
                filter,
            );
        }
        let distant = r.surface(
            a,
            [1., 0., 1.],
            [100., 99., 100.],
            64,
            CallbackFilter::default(),
        );
        if size == 10 {
            assert!(
                distant.position[0] < 20. || distant.position[2] < 20.,
                "the fixed small pool cannot reach the far corner"
            );
        }
        r.surface(
            a,
            [f32::INFINITY, 0., 0.],
            [1.; 3],
            4,
            CallbackFilter::default(),
        );
        r.query_init(128);
        r.surface(
            a,
            [1., 0., 1.],
            [100., 99., 100.],
            64,
            CallbackFilter::default(),
        );
        r.finish(&format!(
            "moveAlongSurface grid size={size} max64 nodes, filters and retained main pool"
        ));
    }

    // Twelve links share one border. Native calls every filter, then retains
    // only the first eight passing neighbors. Those eight have shorter floors.
    let owner_verts = [[0, 0, 0], [0, 0, 4], [4, 0, 4], [4, 0, 0]];
    let owner_polys = [polygon(
        &[0, 1, 2, 3],
        &[0x8000, 0x8001, 0x8002, 0x8003],
        1,
        15,
    )];
    let mut p = params(&owner_verts, &owner_polys);
    p.coordinate = [0; 3];
    p.bounds = [[0.; 3], [4.; 3]];
    p.cell_size = 1.;
    let mut r = Runtime::new(13, 1);
    let owner = r.add(&create(&p).unwrap(), 0);
    let target_polys = [polygon(
        &[0, 1, 2, 3],
        &[0x8000, 0xffff, 0xffff, 0xffff],
        1,
        3,
    )];
    for layer in 0..12 {
        let width = if layer < 4 { 4 } else { 1 };
        let verts = [[0, 0, 0], [0, 0, 4], [width, 0, 4], [width, 0, 0]];
        let p = CreateParams {
            vertices: &verts,
            polygons: &target_polys,
            coordinate: [1, 0, layer],
            bounds: [[4., 0., 0.], [8., 4., 4.]],
            ..p.clone()
        };
        r.add(&create(&p).unwrap(), 0);
    }
    let result = r.surface(
        owner,
        [1., 0., 1.],
        [7., 9., 1.],
        16,
        CallbackFilter::default(),
    );
    assert_eq!(result.position, [5., 0., 1.]);
    for include in [0, 1] {
        let filter = CallbackFilter {
            base: Filter {
                include,
                ..Filter::default()
            },
            ..CallbackFilter::default()
        };
        r.surface(owner, [1., 0., 1.], [4.5, 7., 1.], 2, filter);
    }
    r.finish("surface movement external eight-neighbor cap and complete filter callback order");
}

#[test]
fn native_circle_shape_dijkstra_costs_records_and_path_extraction() {
    let verts: Vec<_> = (0..=4)
        .flat_map(|z| (0..=4).map(move |x| [x * 4 + z, x + z, z * 4]))
        .collect();
    let polys: Vec<_> = (0..4)
        .flat_map(|z| {
            (0..4).map(move |x| {
                let a = z * 5 + x;
                let poly = z * 4 + x;
                polygon(
                    &[a, a + 5, a + 6, a + 1],
                    &[
                        if x == 0 { 0xffff } else { poly - 1 },
                        if z == 3 { 0xffff } else { poly + 4 },
                        if x == 3 { 0xffff } else { poly + 1 },
                        if z == 0 { 0xffff } else { poly - 4 },
                    ],
                    1 << (poly % 4),
                    (poly % 8) as u8,
                )
            })
        })
        .collect();
    let mut p = params(&verts, &polys);
    p.coordinate = [0; 3];
    p.bounds = [[0.; 3], [6.; 3]];
    p.cell_size = 0.3;
    p.cell_height = 0.2;
    let data = create(&p).unwrap();
    for nodes in [4, 17] {
        let mut r = Runtime::with_nodes(1, 16, nodes);
        let start = r.add(&data, 0);
        r.dijkstra_path(start, 16);
        for capacity in [0, 1, 4, 16, 17] {
            for radius in [0., 0.6, 1.5, 100., -1., f32::NAN] {
                for blocked in [255, 3] {
                    let mut filter = CallbackFilter {
                        blocked,
                        ..CallbackFilter::default()
                    };
                    filter.base.area_costs[5] = 3.75;
                    filter.base.area_costs[7] = 0.25;
                    r.around(start, [0.25, 3., 0.25], radius, None, capacity, filter);
                    r.closed(&(0..16).map(|i| start | i).collect::<Vec<_>>());
                    for end in [start, start | 3, start | 15, 0] {
                        for cap in [0, 1, 4, 16] {
                            r.dijkstra_path(end, cap);
                        }
                    }
                }
            }
        }
        let shapes = [
            vec![],
            vec![[0.; 3], [1.; 3]],
            vec![[0.; 3], [0., 1., 4.], [4., 2., 4.], [4., 3., 0.]],
            vec![
                [1.32, 2., 0.13],
                [0.32, 1., 4.43],
                [5.12, 2., 4.25],
                [4.13, 3., -0.34],
            ],
            vec![[-10., 0., -10.], [-10., 0., 20.], [20., 0., -10.]],
            vec![[0.; 3], [4., 3., 0.], [4., 2., 4.], [0., 1., 4.]],
            vec![[0.; 3], [0., f32::NAN, 4.], [4., 2., 4.], [4., 3., 0.]],
        ];
        for shape in shapes {
            for capacity in [0, 1, 4, 16, 17] {
                r.around(
                    start,
                    [0.; 3],
                    0.,
                    Some(&shape),
                    capacity,
                    CallbackFilter::default(),
                );
                r.dijkstra_path(start | 15, 16);
            }
        }
        r.around(0, [0.; 3], 2., None, 16, CallbackFilter::default());
        r.around(
            start,
            [f32::INFINITY, 0., 0.],
            2.,
            None,
            16,
            CallbackFilter::default(),
        );
        r.persistent_path(start, start | 15, [0.25, 0., 0.25], [4., 0., 4.], 16);
        r.dijkstra_path(start | 3, 16);
        r.finish(&format!(
            "Dijkstra circle/shape nodes={nodes}, costs, callback order and retained records"
        ));
    }
}

#[test]
fn native_local_neighborhood_and_wall_searches_match_ordered_native_state() {
    let verts: Vec<_> = (0..=10)
        .flat_map(|z| (0..=10).map(move |x| [x * 2 + z, x + z, z * 2]))
        .collect();
    let polys: Vec<_> = (0..10)
        .flat_map(|z| {
            (0..10).map(move |x| {
                let a = z * 11 + x;
                let poly = z * 10 + x;
                polygon(
                    &[a, a + 11, a + 12, a + 1],
                    &[
                        if x == 0 { 0xffff } else { poly - 1 },
                        if z == 9 { 0xffff } else { poly + 10 },
                        if x == 9 { 0xffff } else { poly + 1 },
                        if z == 0 { 0xffff } else { poly - 10 },
                    ],
                    1 << (poly % 4),
                    (poly % 7) as u8,
                )
            })
        })
        .collect();
    let mut p = params(&verts, &polys);
    p.coordinate = [0; 3];
    p.bounds = [[0.; 3], [10.; 3]];
    p.cell_size = 0.3;
    p.cell_height = 0.2;
    let data = create(&p).unwrap();
    for nodes in [4, 128] {
        let mut r = Runtime::with_nodes(1, 128, nodes);
        let start = r.add(&data, 0);
        for index in [0, 44, 99] {
            let reference = start | index;
            let center = [
                (index % 10) as f32 * 0.6 + (index / 10) as f32 * 0.3 + 0.35,
                3.,
                (index / 10) as f32 * 0.6 + 0.25,
            ];
            for exclude in [0, 4] {
                for radius in [0., 0.25, 1., 100., -1., f32::NAN] {
                    r.distance_wall(
                        reference,
                        center,
                        radius,
                        CallbackFilter {
                            base: Filter {
                                exclude,
                                ..Filter::default()
                            },
                            ..CallbackFilter::default()
                        },
                    );
                    for capacity in [0, 1, 4, 15, 16, 17, 64, 100] {
                        r.local(
                            reference,
                            center,
                            radius,
                            capacity,
                            CallbackFilter {
                                base: Filter {
                                    exclude,
                                    ..Filter::default()
                                },
                                ..CallbackFilter::default()
                            },
                        );
                        if exclude == 0 && radius == 100. && capacity >= 64 {
                            assert!(r.neighborhood_scratch.entries.len() >= 32);
                        }
                    }
                }
                for capacity in [0, 1, 3, 4, 8] {
                    for portals in [false, true] {
                        r.walls(
                            reference,
                            capacity,
                            portals,
                            CallbackFilter {
                                base: Filter {
                                    exclude,
                                    ..Filter::default()
                                },
                                ..CallbackFilter::default()
                            },
                        );
                    }
                }
            }
        }
        // Reused projection bounds must retain native decisions for initialized
        // exceptional vertex values as well as ordinary, thin geometry.
        let vertex_index = 60;
        let original = r.mesh.tiles[0]
            .as_ref()
            .unwrap()
            .vertex(vertex_index)
            .unwrap();
        for (x, z) in [
            (0x7fc1_2345, 0xffc5_4321),
            (0x7f81_2345, 0xff85_4321),
            (f32::INFINITY.to_bits(), f32::NEG_INFINITY.to_bits()),
            (f32::MAX.to_bits(), (-f32::MAX).to_bits()),
            (1, 0x8000_0001),
        ] {
            for point in [
                [f32::from_bits(x), original[1], f32::from_bits(z)],
                original,
            ] {
                writeln!(
                    r.input,
                    "vertex_record {start} {vertex_index} {} {} {}",
                    point[0].to_bits(),
                    point[1].to_bits(),
                    point[2].to_bits()
                )
                .unwrap();
                let tile = r.mesh.tiles[0].as_mut().unwrap();
                tile.set_vertex(vertex_index, point).unwrap();
                r.actual.push(json!(tile.data().unwrap()));
                for capacity in [16, 64] {
                    r.local(
                        start | 44,
                        [3.7, 3., 3.2],
                        100.,
                        capacity,
                        CallbackFilter::default(),
                    );
                }
            }
        }
        r.distance_wall(0, [0.; 3], 1., CallbackFilter::default());
        r.distance_wall(start, [f32::NAN, 0., 0.], 1., CallbackFilter::default());
        r.distance_wall(start, [0.; 3], 0., CallbackFilter::default());
        r.local(0, [0.; 3], 1., 4, CallbackFilter::default());
        r.local(
            start,
            [0., f32::INFINITY, 0.],
            1.,
            4,
            CallbackFilter::default(),
        );
        r.walls(0, 4, false, CallbackFilter::default());
        r.finish(&format!(
            "local/wall slantedgrid, node budget={nodes}, costs, normals and main pool retention"
        ));
    }
}

#[test]
fn native_layered_wall_intervals_first_link_and_overlapping_neighborhoods() {
    let verts = [[0, 0, 0], [0, 0, 4], [4, 0, 4], [4, 0, 0]];
    let mut polys = vec![polygon(
        &[0, 1, 2, 3],
        &[0x8000, 0x8001, 0x8002, 0x8003],
        1,
        25,
    )];
    polys.extend((0..3).map(|_| polygon(&[0, 1, 2, 3], &[0xffff; 4], 1, 26)));
    let mut p = params(&verts, &polys);
    p.coordinate = [0; 3];
    p.bounds = [[0.; 3], [4.; 3]];
    p.cell_size = 1.;
    let mut r = Runtime::new(21, 4);
    let owner = r.add(&create(&p).unwrap(), 0);
    for layer in 0..20 {
        let polys = [polygon(
            &[0, 1, 2, 3],
            &[0x8000, 0xffff, 0xffff, 0xffff],
            1,
            layer as u8,
        )];
        let start = if layer % 3 == 0 { 1 } else { 0 };
        let end = if layer % 3 == 0 { 3 } else { 4 };
        let vertices = [[0, 0, start], [0, 0, end], [4, 0, end], [4, 0, start]];
        let p = CreateParams {
            vertices: &vertices,
            polygons: &polys,
            coordinate: [1, 0, layer],
            bounds: [[4., 0., 0.], [8., 4., 4.]],
            ..p.clone()
        };
        r.add(&create(&p).unwrap(), 0);
    }
    for blocked in [255, 19, 7] {
        for capacity in [0, 1, 4, 16, 64] {
            for portals in [false, true] {
                r.walls(
                    owner,
                    capacity,
                    portals,
                    CallbackFilter {
                        blocked,
                        ..CallbackFilter::default()
                    },
                );
            }
            r.local(
                owner,
                [3.75, 0., 1.],
                10.,
                capacity,
                CallbackFilter {
                    blocked,
                    ..CallbackFilter::default()
                },
            );
        }
        for radius in [0., 0.3, 10.] {
            r.distance_wall(
                owner,
                [3.75, 0., 1.],
                radius,
                CallbackFilter {
                    blocked,
                    ..CallbackFilter::default()
                },
            );
        }
    }
    r.finish("native 16-interval cap, omitted sentinels, overlaps, first-link wall eligibility");

    // A folded authored corridor is defined even though its projected polygons
    // overlap. Only references actually stored in the result take part in tests.
    // References for polygons 0 and 66 share the connectivity-mask bit. The
    // fifth polygon overlaps the first and must still be rejected after a hit.
    let chain = [0, 1, 66, 2, 3];
    let vertices: Vec<_> = (0..chain.len())
        .flat_map(|i| {
            let x = if i % 2 == 0 { 0 } else { 2 };
            [[x, 0, 0], [x, 0, 2], [x + 2, 0, 2], [x + 2, 0, 0]]
        })
        .collect();
    let mut polys = vec![polygon(&[0, 1, 2, 3], &[0xffff; 4], 1, 0); 67];
    for (i, &index) in chain.iter().enumerate() {
        let a = (i * 4) as u16;
        polys[index as usize] = polygon(
            &[a, a + 1, a + 2, a + 3],
            &[
                if i == 0 { 0xffff } else { chain[i - 1] },
                0xffff,
                chain.get(i + 1).copied().unwrap_or(0xffff),
                0xffff,
            ],
            1,
            i as u8,
        );
    }
    let mut p = params(&vertices, &polys);
    p.coordinate = [0; 3];
    p.bounds = [[0.; 3], [4.; 3]];
    p.cell_size = 1.;
    let mut r = Runtime::new(1, 128);
    let start = r.add(&create(&p).unwrap(), 0);
    for capacity in [0, 1, 2, 3, 4, 16] {
        r.local(
            start,
            [1., 0., 1.],
            10.,
            capacity,
            CallbackFilter::default(),
        );
        if capacity >= 4 {
            assert_eq!(r.neighborhood_scratch.entries.len(), 4);
        }
    }
    r.finish("neighborhood overlap rejection uses stored references and native current-poly connectivity");
}

#[test]
fn native_random_queries_rng_filter_order_projection_and_retained_state() {
    let data = sliced_chain_data(0.3);
    for nodes in [4, 64] {
        let mut r = Runtime::with_nodes(4, 8, nodes);
        r.random(None, &[0.5], CallbackFilter::default());
        let start = r.add(&data, 0);
        let mut second = data.clone();
        // A separate layer participates in uniform tile reservoir sampling.
        second[16..20].copy_from_slice(&1i32.to_ne_bytes());
        let other = r.add(&second, 0);
        r.persistent_path(start, start | 7, [0.25, 0., 1.], [3.75, 0., 1.], 16);
        let mut seed = 0x51a9_d258u32;
        let values: Vec<_> = (0..53)
            .map(|_| {
                seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                (seed >> 8) as f32 / 16777216.
            })
            .collect();
        for values in [&[0.][..], &[1.][..], &[0.5][..], &values[..]] {
            for blocked in [255, 3, 7] {
                r.random(
                    None,
                    values,
                    CallbackFilter {
                        blocked,
                        ..CallbackFilter::default()
                    },
                );
                for radius in [0., 0.5, 2., 100., -1., f32::NAN] {
                    r.random(
                        Some((start, [0.25, 1.3, 1.], radius)),
                        values,
                        CallbackFilter {
                            blocked,
                            ..CallbackFilter::default()
                        },
                    );
                }
            }
        }
        r.random(
            None,
            &[0.5],
            CallbackFilter {
                base: Filter {
                    include: 0,
                    ..Filter::default()
                },
                ..CallbackFilter::default()
            },
        );
        r.random(Some((0, [0.; 3], 1.)), &[0.5], CallbackFilter::default());
        r.remove(other);
        r.random(None, &[0.5], CallbackFilter::default());
        r.finish(&format!(
            "random reservoir draw/filter order and retained nodes={nodes}"
        ));
    }
    let vertices = [
        [0, 0, 0],
        [0, 0, 4],
        [1, 0, 4],
        [1, 0, 0],
        [4, 0, 4],
        [4, 0, 0],
    ];
    let polygons = [
        polygon(&[0, 1, 2, 3], &[0xffff, 0xffff, 1, 0xffff], 1, 3),
        polygon(&[3, 2, 4, 5], &[0, 0xffff, 0xffff, 0xffff], 1, 7),
    ];
    let mut p = params(&vertices, &polygons);
    p.coordinate = [0; 3];
    p.bounds = [[0.; 3], [4.; 3]];
    p.cell_size = 1.;
    let mut r = Runtime::new(2, 2);
    let first = r.add(&create(&p).unwrap(), 0);
    assert_eq!(
        r.random(None, &[0.8], CallbackFilter::default()).reference,
        first
    );
    assert_eq!(
        r.random(None, &[0.7], CallbackFilter::default()).reference,
        first | 1
    );
    p.coordinate[2] = 1;
    let second = r.add(&create(&p).unwrap(), 0);
    r.flags(second, 0, 3);
    r.flags(second | 1, 0, 7);
    // Tile sampling is uniform before filtering. It can choose a tile with no
    // eligible polygons even while another tile has eligible ground.
    assert_eq!(
        r.random(None, &[0.], CallbackFilter::default()).reference,
        123
    );
    assert_eq!(
        r.random(None, &[1.], CallbackFilter::default()).reference,
        first
    );
    r.random(
        Some((second, [1., 0., 1.], 10.)),
        &[0.5],
        CallbackFilter::default(),
    );
    r.finish("native unequal-area polygon reservoir and tile selection before filtering");
}

impl Runtime {
    fn accessors(
        &mut self,
        coordinate: [i32; 3],
        capacity: usize,
        index: i32,
        reference: PolyRef,
        position: Point,
    ) {
        let [x, z, layer] = coordinate;
        write!(
            self.input,
            "a {x} {z} {layer} {capacity} {index} {reference} "
        )
        .unwrap();
        self.point(position);
        let params = self.mesh.params();
        let [tx, tz] = self.mesh.calc_tile_location(position).unwrap();
        let at = self.mesh.tile_at(coordinate);
        let slot = |slot: Option<TileSlot<'_>>| {
            slot.map_or_else(
                || json!([]),
                |s| {
                    json!([
                        s.index,
                        s.salt,
                        u8::from(s.tile.is_some()),
                        self.mesh.tile_reference(Some(s.index)),
                        self.mesh.tile_reference(Some(s.index))
                    ])
                },
            )
        };
        let valid = self.mesh.tile_and_polygon_by_ref(reference);
        let (status, ti, pi) = match valid {
            Ok(_) => {
                let (_, t, p) = self.mesh.reference_layout().decode(reference);
                (SUCCESS, i64::from(t), i64::from(p))
            }
            Err(s) => (s, -1, -1),
        };
        let decoded = self.mesh.polygon_from_bits(reference).map_or_else(
            || json!([]),
            |_| {
                let (_, t, p) = self.mesh.reference_layout().decode(reference);
                json!([t, p])
            },
        );
        self.actual.push(json!([
            self.mesh.max_tiles(),
            params.max_polygons,
            params.origin[0].to_bits(),
            params.origin[1].to_bits(),
            params.origin[2].to_bits(),
            params.tile_width.to_bits(),
            params.tile_height.to_bits(),
            tx,
            tz,
            self.mesh.tiles_at_bounded(x, z, capacity),
            at.map_or(-1, |s| s.index as i64),
            self.mesh.tile_ref_at(coordinate),
            self.mesh.tile_reference(at.map(|s| s.index)),
            self.mesh.tile_reference(at.map(|s| s.index)),
            slot(self.mesh.tile(index)),
            slot(self.mesh.tile_by_ref(reference)),
            [
                json!(status),
                json!(u8::from(self.mesh.is_valid_poly_ref(reference))),
                json!(ti),
                json!(pi)
            ],
            decoded
        ]));
    }
}
#[test]
fn native_mesh_public_bounded_accessors_free_slots_and_decoded_contexts() {
    let vertices = [[0, 0, 0], [0, 0, 4], [4, 0, 4], [4, 0, 0]];
    let polys = [polygon(&[0, 1, 2, 3], &[0xffff; 4], 1, 0)];
    let mut p = params(&vertices, &polys);
    p.coordinate = [0; 3];
    p.bounds = [[0.; 3], [4.; 3]];
    p.cell_size = 1.;
    let mut r = Runtime::new(40, 9);
    let free = r.mesh.reference(0, 0);
    for reference in [0, free, free | 1, PolyRef::MAX] {
        r.accessors([0; 3], 40, 0, reference, [-0.1, f32::NAN, 4.]);
    }
    let mut refs = vec![];
    for layer in 0..35 {
        p.coordinate = [0, 0, layer];
        refs.push(r.add(&create(&p).unwrap(), 0));
    }
    for cap in [0, 1, 31, 32, 33, 35, 40] {
        for (coordinate, index, reference) in [
            ([0, 0, 0], 0, refs[0]),
            ([0, 0, 34], 34, refs[34]),
            ([0, 0, 35], 35, r.mesh.reference(35, 0)),
            ([1, 0, 0], -1, refs[1] | 8),
        ] {
            r.accessors(
                coordinate,
                cap,
                index,
                reference,
                [4., f32::INFINITY, -4.000001],
            );
        }
    }
    let data = r.remove(refs[3]);
    for reference in [refs[3], r.mesh.reference(3, 0), r.mesh.reference(3, 8)] {
        r.accessors([0, 0, 3], 40, 3, reference, [0.; 3]);
    }
    r.add(&data, 0);
    for reference in [refs[3], r.mesh.reference(3, 0), r.mesh.reference(3, 1), 0] {
        r.accessors([0, 0, 3], 40, 40, reference, [15.999999, 0., -0.]);
    }
    assert_eq!(r.mesh.calc_tile_location([f32::NAN, 0., 0.]), None);
    assert_eq!(r.mesh.calc_tile_location([f32::MAX, 0., 0.]), None);
    r.finish("native public bounded accessor and free-slot semantics");
}

impl Runtime {
    fn add_owned(&mut self, data: &[u8], size: i32, flags: i32, last: TileRef) -> TileRef {
        write!(self.input, "k {flags} {size} {last} {} ", data.len()).unwrap();
        values(&mut self.input, data);
        let (status, reference) = self
            .mesh
            .add_tile_data_sized(data, size, flags, last)
            .map_or_else(|s| (s, 0), |r| (SUCCESS, r));
        self.actual.push(json!([status, reference]));
        self.ownership_snapshot();
        reference
    }
    fn ownership_snapshot(&mut self) {
        self.input.push_str("h\n");
        self.actual.push(json!(
            (0..self.mesh.max_tiles())
                .map(|i| {
                    let t = self.mesh.tile(i as i32).unwrap();
                    json!([
                        t.index,
                        t.salt,
                        u8::from(t.tile.is_some()),
                        t.flags,
                        t.data_size,
                        t.free_link,
                        t.data().unwrap()
                    ])
                })
                .collect::<Vec<_>>()
        ));
    }
    fn remove_owned(&mut self, reference: TileRef) -> RemovedTileData {
        writeln!(self.input, "o {reference}").unwrap();
        let mut output = RemovedTileData {
            data: Some(vec![0xa5, 0xa6, 0xa7]),
            data_size: -17,
        };
        let status = self.mesh.remove_tile_data_into(reference, &mut output);
        self.actual.push(json!([
            status,
            u8::from(output.data.is_some()),
            output.data_size,
            output.data.as_deref().unwrap_or(&[])
        ]));
        self.ownership_snapshot();
        output
    }
}
#[test]
fn native_tile_flags_declared_size_ownership_outputs_and_retained_vacant_data() {
    let data = sliced_chain_data(0.3);
    for flags in [0, TILE_FREE_DATA, 2, 3, -1, -2] {
        for size in [-1, 0, 17, data.len() as i32] {
            let mut r = Runtime::new(2, 8);
            r.ownership_snapshot();
            let a = r.add_owned(&data, size, flags, 0);
            assert_ne!(a, 0);
            // Occupied coordinates do not replace flags or change any free slot.
            assert_eq!(r.add_owned(&data, 23, flags ^ TILE_FREE_DATA, 0), 0);
            r.flags(a, 0xabcd, 15);
            r.ownership_snapshot();
            r.remove_owned(0);
            let removed = r.remove_owned(a);
            assert_eq!(removed.data.is_none(), flags & TILE_FREE_DATA != 0);
            assert_eq!(
                removed.data_size,
                if flags & TILE_FREE_DATA != 0 { 0 } else { size }
            );
            r.remove_owned(a); // stale salt: both caller outputs are retained.
            let b = r.add_owned(&data, size, flags ^ TILE_FREE_DATA, a);
            assert_eq!(a, b);
            r.remove_owned(b);
            r.finish("native tile ownership/size metadata and retained free-slot buffers");
            let mut input = format!("load_flags {} ", data.len());
            values(&mut input, &data);
            write!(input, "{flags} {size}").unwrap();
            let mesh = NavMesh::from_tile_data_sized(&data, size, flags).unwrap();
            let t = mesh.tile(0).unwrap();
            let p = mesh.params();
            assert_eq!(
                json!([
                    SUCCESS,
                    t.flags,
                    t.data_size,
                    mesh.tile_reference(Some(0)),
                    p.origin[0].to_bits(),
                    p.origin[1].to_bits(),
                    p.origin[2].to_bits(),
                    p.tile_width.to_bits(),
                    p.tile_height.to_bits(),
                    p.max_tiles,
                    p.max_polygons,
                    t.data().unwrap().unwrap()
                ]),
                oracle(input)
            );
        }
    }
}

#[test]
fn native_initialization_preserves_dimensions_zero_budgets_and_wrapped_bit_counts() {
    let dimensions = [
        ([0.; 3], 1., 1.),
        ([-0., 0., -0.], -0., 0.),
        ([1., -3., 2.], -2., -3.),
        ([0.; 3], f32::INFINITY, f32::NEG_INFINITY),
        ([0.; 3], f32::NAN, 1.),
        ([f32::INFINITY, 0., f32::NEG_INFINITY], 1., 1.),
        ([f32::from_bits(0xffc0_1234), f32::NAN, -0.], 1., 1.),
        ([0.; 3], f32::from_bits(1), f32::MIN_POSITIVE),
    ];
    let budgets = [
        (0, 0),
        (0, 1),
        (1, 0),
        (1, 1),
        (3, 5),
        (8, 1 << 19),
        (8, 1 << 20),
        (1, i32::MAX),
        (8, i32::MAX),
        (1, -1),
        (1, -65536),
        (1, i32::MIN),
        (8, i32::MIN),
    ];
    let mut input = format!("init {}\n", dimensions.len() * budgets.len());
    let mut actual = Vec::new();
    for (origin, width, height) in dimensions {
        for (tiles, polygons) in budgets {
            // maxPolys is reference-bit metadata; neither constructor allocates
            // polygons. Keep every actual tile allocation bounded to eight.
            assert!(tiles <= 8);
            values(
                &mut input,
                origin.into_iter().chain([width, height]).map(f32::to_bits),
            );
            writeln!(input, "{tiles} {polygons}").unwrap();
            match NavMesh::from_params(NavMeshParams {
                origin,
                tile_width: width,
                tile_height: height,
                max_tiles: tiles,
                max_polygons: polygons,
            }) {
                Err(_) => actual.push(json!([FAILURE | INVALID_PARAM])),
                Ok(mesh) => {
                    let p = mesh.params();
                    let layout = mesh.reference_layout();
                    let references = [0, 1, u32::MAX]
                        .into_iter()
                        .map(|word| {
                            let reference = layout.try_encode(word, word, word)?;
                            let (s, t, p) = layout.try_decode(reference).unwrap();
                            Some(json!([reference, s, t, p]))
                        })
                        .collect::<Option<Vec<_>>>();
                    if references.is_none() {
                        assert_eq!(layout.try_decode(0), None);
                    }
                    let slots: Vec<_> = (0..tiles)
                        .map(|i| {
                            let slot = mesh.tile(i).unwrap();
                            json!([
                                slot.salt,
                                slot.flags,
                                slot.data_size,
                                u8::from(slot.tile.is_some())
                            ])
                        })
                        .collect();
                    actual.push(json!([
                        SUCCESS,
                        p.origin[0].to_bits(),
                        p.origin[1].to_bits(),
                        p.origin[2].to_bits(),
                        p.tile_width.to_bits(),
                        p.tile_height.to_bits(),
                        p.max_tiles,
                        p.max_polygons,
                        slots,
                        references
                    ]));
                }
            }
        }
    }
    let expected = oracle(input);
    for (i, (a, b)) in actual.iter().zip(expected.as_array().unwrap()).enumerate() {
        assert_eq!(a, b, "init case={i}");
    }
    assert_eq!(actual.len(), expected.as_array().unwrap().len());
    for (width, height) in [(0., 1.), (1., 0.), (f32::NAN, 1.)] {
        let mesh = NavMesh::new([0.; 3], width, height, 1, 1).unwrap();
        assert_eq!(mesh.calc_tile_location([0.; 3]), None);
        assert_eq!(
            mesh.query_polygons([0.; 3], [1.; 3], &Filter::default(), 1),
            (FAILURE | INVALID_PARAM, vec![])
        );
    }
    let mesh = NavMesh::new([0.; 3], f32::INFINITY, f32::NEG_INFINITY, 0, 0).unwrap();
    assert_eq!(mesh.calc_tile_location([1.; 3]), Some([0, 0]));
    assert_eq!(
        mesh.query_polygons([0.; 3], [1.; 3], &Filter::default(), 1),
        (SUCCESS, vec![])
    );

    let vertices = [[0, 0, 0], [0, 0, 4], [4, 0, 0]];
    let polygons = [polygon(&[0, 1, 2], &[0xffff; 3], 1, 3)];
    let mut p = params(&vertices, &polygons);
    p.bounds = [[0.; 3], [4.; 3]];
    p.cell_size = 1.;
    p.cell_height = 1.;
    p.coordinate = [0; 3];
    let data = create(&p).unwrap();
    for tiles in [0, 1] {
        let mut r = Runtime::new(tiles, 0);
        r.add(&data, 0);
        r.snapshot();
        r.finish("zero native tile/polygon budgets");
    }
}

#[test]
fn native_tile_creation_preserves_float_payloads_and_checks_only_evaluated_conversions() {
    let vertices = [[0, 2, 0], [0, 2, 4], [4, 3, 0]];
    let polygons = [polygon(&[0, 1, 2], &[0xffff; 3], 17, 63)];
    let details = [[0, 5, 0, 1]];
    let triangles = [[0, 1, 3, 21]];
    let words = [
        0.,
        -0.,
        -1.,
        1.,
        65536.,
        f32::INFINITY,
        f32::NEG_INFINITY,
        f32::from_bits(0x7fc0_1234),
        f32::from_bits(0xffc0_5678),
    ];
    for with_details in [false, true] {
        for (case, value) in words.into_iter().enumerate() {
            for field in 0..17 {
                if !with_details && (11..=12).contains(&field) {
                    continue;
                }
                let mut dv = [
                    [0., 2., 0.],
                    [0., 2., 4.],
                    [4., 3., 0.],
                    [1., 2., 1.],
                    [2., 3., 1.],
                ];
                let mut connections = [CreateOffMeshConnection {
                    positions: [[1., 2., 1.], [12., 3., 2.]],
                    radius: 0.7,
                    flags: 5,
                    area: 12,
                    direction: 1,
                }];
                let mut p = params(&vertices, &polygons);
                p.bounds = [[0.; 3], [8.; 3]];
                p.build_bv_tree = false;
                match field {
                    0 => p.cell_size = value,
                    1 => p.cell_height = value,
                    2 => p.walkable_height = value,
                    3 => p.walkable_radius = value,
                    4 => p.walkable_climb = value,
                    5..=7 => p.bounds[0][field - 5] = value,
                    8..=10 => p.bounds[1][field - 8] = value,
                    11 => dv[0][0] = value,
                    12 => dv[4][1] = value,
                    13 => connections[0].positions[0][0] = value,
                    14 => connections[0].positions[0][1] = value,
                    15 => connections[0].positions[1][2] = value,
                    16 => connections[0].radius = value,
                    _ => unreachable!(),
                }
                if with_details {
                    p.detail_meshes = Some(&details);
                    p.detail_vertices = &dv;
                    p.detail_triangles = &triangles;
                }
                p.off_mesh_connections = &connections;
                let expected = native_create(&p, true);
                assert_eq!(expected["ok"], true);
                compare(
                    &create(&p).unwrap(),
                    &bytes(&expected),
                    &format!("float case={case} field={field} details={with_details}"),
                );
                if field <= 12 {
                    // With no BV tree, portals or off-mesh links, native loading
                    // performs no float-to-integer conversions on this payload.
                    p.off_mesh_connections = &[];
                    let data = create(&p).unwrap();
                    let mesh = NavMesh::from_tile_data(&data).unwrap();
                    let expected = operation("load", &data);
                    assert_eq!(expected["status"], SUCCESS);
                    compare(
                        &mesh.tile_data(mesh.tile_reference(Some(0))).unwrap(),
                        &bytes(&expected),
                        &format!("float load case={case} field={field} details={with_details}"),
                    );
                }
            }
        }
    }
    let polygons = [polygons[0].clone(), polygons[0].clone()];
    let mut p = params(&vertices, &polygons);
    p.bounds = [[10.; 3], [0.; 3]];
    p.cell_size = -1.;
    p.cell_height = -1.;
    let details = [[0, 3, 0, 1], [3, 3, 1, 1]];
    let dv = [
        [0., 1., 2.],
        [2., 3., 4.],
        [4., 5., 6.],
        [1., 2., 3.],
        [3., 4., 5.],
        [5., 6., 7.],
    ];
    let triangles = [[0, 1, 2, 21]; 2];
    p.detail_meshes = Some(&details);
    p.detail_vertices = &dv;
    p.detail_triangles = &triangles;
    compare(
        &create(&p).unwrap(),
        &bytes(&native_create(&p, true)),
        "negative BV quantization and reversed extents",
    );
    p.cell_size = 1.;
    p.bounds = [[0.; 3], [10.; 3]];
    let mut dv_with_nan = dv;
    dv_with_nan[0] = [f32::NAN; 3];
    p.detail_vertices = &dv_with_nan;
    compare(
        &create(&p).unwrap(),
        &bytes(&native_create(&p, true)),
        "BV bounds replace earlier NaNs with later finite vertices",
    );
    // Each of these reaches an undefined native float-to-integer conversion;
    // check the safe Rust result without executing that native operation.
    let mut dv_with_tail_nan = dv_with_nan;
    dv_with_tail_nan[2] = [f32::NAN; 3];
    p.detail_vertices = &dv_with_tail_nan;
    assert!(create(&p).is_err());
    p.detail_meshes = None;
    p.detail_vertices = &[];
    p.detail_triangles = &[];
    for (cs, ch) in [(0., 1.), (1., f32::INFINITY), (1., 65536.)] {
        p.cell_size = cs;
        p.cell_height = ch;
        assert!(create(&p).is_err());
    }
    for (cs, ch) in [(-1., -1.), (f32::INFINITY, 1.), (f32::NEG_INFINITY, 1.)] {
        p.cell_size = cs;
        p.cell_height = ch;
        compare(
            &create(&p).unwrap(),
            &bytes(&native_create(&p, true)),
            "defined BV scale arithmetic",
        );
    }
}

#[test]
fn native_status_predicates_and_promoted_detail_edge_shifts() {
    let mut input = String::from("helpers 4096\n");
    let mut actual = Vec::new();
    let mut seed = 3918_u32;
    for flags in 0..=255 {
        for edge in 0..16 {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let status = match flags % 5 {
                0 => 0,
                1 => SUCCESS,
                2 => FAILURE | IN_PROGRESS | SUCCESS,
                _ => seed,
            };
            let detail = match edge {
                0 => 0,
                1 => u32::MAX,
                _ => seed.rotate_left(edge as u32),
            };
            writeln!(input, "{flags} {edge} {status} {detail}").unwrap();
            actual.push(json!([
                detail_triangle_edge_flags(flags as u8, edge).unwrap(),
                u8::from(status_succeeded(status)),
                u8::from(status_failed(status)),
                u8::from(status_in_progress(status)),
                u8::from(status_has_detail(status, detail))
            ]));
        }
    }
    assert_eq!(json!(actual), oracle(input));
    for edge in [i32::MIN, -1, 16, i32::MAX] {
        assert_eq!(detail_triangle_edge_flags(255, edge), None);
    }
}

#[test]
fn native_thin_polygon_creation_loading_and_initialized_detail_queries() {
    let vertices = [[0, 0, 0], [0, 0, 4], [4, 0, 0]];
    for width in 0..=2 {
        for bv in [false, true] {
            let polys = [polygon(
                &(0..width as u16).collect::<Vec<_>>(),
                &vec![0xffff; width],
                1,
                7,
            )];
            let details = [[0, 3, 0, 1]];
            let dv = [[0., 0., 0.], [0., 0., 4.], [4., 0., 0.]];
            let triangles = [[0, 1, 2, 21]];
            let mut p = params(&vertices, &polys);
            p.coordinate = [0; 3];
            p.bounds = [[0.; 3], [4.; 3]];
            p.cell_size = 1.;
            p.cell_height = 1.;
            p.vertices_per_polygon = width;
            p.detail_meshes = Some(&details);
            p.detail_vertices = &dv;
            p.detail_triangles = &triangles;
            p.build_bv_tree = bv;
            let data = create(&p).unwrap();
            compare(&data, &bytes(&native(&p)), "thin creator bytes");
            if width == 0 {
                // Loading with zero links makes native initialize links[-1].
                assert_eq!(TileData::parse(&data).unwrap().header.max_link_count, 0);
                assert!(NavMesh::from_tile_data(&data).is_err());
                continue;
            }
            check(&p, "thin explicit-detail load/export");
            let mut runtime = Runtime::new(2, 8);
            let reference = runtime.add(&data, 0);
            runtime.snapshot();
            runtime.closest(reference, [1., 2., 1.]);
            runtime.nearest([1., 2., 1.], [4.; 3], 65535, 0);
            runtime.flags(reference, 0x8001, 255);
            runtime.finish(&format!("width={width} bv={bv}"));
            if width == 2 {
                p.detail_meshes = None;
                p.detail_vertices = &[];
                p.detail_triangles = &[];
                // Native creation/loading is defined; closest-point queries
                // without initialized detail triangles are excluded.
                check(&p, "two-vertex automatic zero-triangle details");
            }
        }
    }
    // The declared detail count can be zero while the BV builder reads one
    // initialized backing point. This case is creator-only (zero links).
    let polys = [polygon(&[], &[], 1, 0)];
    let details = [[0, 0, 0, 0]];
    let dv = [[1., 2., 3.]];
    let mut p = params(&vertices, &polys);
    p.vertices_per_polygon = 0;
    p.detail_meshes = Some(&details);
    p.detail_vertices = &dv;
    compare(
        &create(&p).unwrap(),
        &bytes(&native(&p)),
        "BV first backing point with zero declared detail count",
    );
    p.detail_vertices = &[];
    assert!(create(&p).is_err()); // Actual native backing read would be invalid.
}

#[test]
fn native_packed_polygon_types_two_and_three_load_and_query() {
    let vertices = [[0, 0, 0], [0, 0, 4], [4, 0, 4], [4, 0, 0]];
    let polygons = [polygon(&[0, 1, 2, 3], &[0xffff; 4], 1, 7)];
    for kind in [2, 3] {
        for bv in [false, true] {
            let mut p = params(&vertices, &polygons);
            p.coordinate = [0; 3];
            p.bounds = [[0.; 3], [4.; 3]];
            p.cell_size = 1.;
            p.cell_height = 1.;
            p.build_bv_tree = bv;
            let mut data = create(&p).unwrap();
            data[HEADER_SIZE + vertices.len() * 12 + 31] = (kind << 6) | 7;
            let native = operation("load", &data);
            assert_eq!(native["status"], SUCCESS);
            let mesh = NavMesh::from_tile_data(&data).unwrap();
            compare(
                &mesh.tile_data(mesh.reference(0, 0)).unwrap(),
                &bytes(&native),
                "packed type load",
            );
            let mut runtime = Runtime::new(2, 8);
            let reference = runtime.add(&data, 0);
            runtime.snapshot();
            runtime.closest(reference, [1., 2., 1.]);
            runtime.nearest([1., 2., 1.], [4.; 3], 65535, 0);
            runtime.random(None, &[0.3, 0.5, 0.7], CallbackFilter::default());
            runtime.random(
                Some((reference, [1., 0., 1.], 3.)),
                &[0.3, 0.5, 0.7],
                CallbackFilter::default(),
            );
            runtime.flags(reference, 0x8001, 255);
            runtime.finish(&format!("type={kind} bv={bv}"));
        }
    }
}

#[test]
fn native_unused_detail_metadata_and_signed_vertex_copy_bases() {
    let vertices = [[0, 0, 0], [0, 0, 4], [4, 0, 0]];
    let polygons = [polygon(&[0, 1, 2], &[0xffff; 3], 1, 7)];
    for base in [0, 8, i32::MAX as u32, i32::MIN as u32, u32::MAX] {
        for triangle_count in [0, 256, 65536] {
            let details = [[base, 3, u32::MAX, triangle_count]];
            let mut p = params(&vertices, &polygons);
            p.build_bv_tree = false;
            p.detail_meshes = Some(&details);
            let data = check(&p, "unused detail base/count metadata");
            let parsed = TileData::parse(&data).unwrap();
            assert_eq!(parsed.detail_meshes().next().unwrap().triangle_count, 0);
            let mut runtime = Runtime::new(2, 8);
            let r = runtime.add(&data, 0);
            runtime.snapshot();
            runtime.flags(r, 2, 255);
            runtime.finish("unused detail metadata load/mutation");
        }
    }
    let dv = [[1., 2., 3.], [4., 5., 6.], [7., 8., 9.]];
    let triangles = [[0, 1, 3, 21]];
    for signed_base in [-3_i32, -2, -1] {
        let details = [[signed_base as u32, 4, 0, 1]];
        let mut p = params(&vertices, &polygons);
        p.build_bv_tree = false;
        p.detail_meshes = Some(&details);
        p.detail_vertices = &dv;
        p.detail_triangles = &triangles;
        let data = check(&p, "signed source base after skipping polygon vertices");
        assert_eq!(
            TileData::parse(&data).unwrap().detail_vertices().next(),
            Some(dv[(signed_base + 3) as usize])
        );
    }
    // Creator serializes the byte count and base without dereferencing that
    // triangle range. Only initialized supplied global triangles are copied.
    let details = [[0, 3, u32::MAX, u32::MAX]];
    let mut p = params(&vertices, &polygons);
    p.build_bv_tree = false;
    p.detail_meshes = Some(&details);
    let data = check(&p, "creator and loader preserve unused triangle metadata");
    let mut runtime = Runtime::new(2, 8);
    let reference = runtime.add(&data, 0);
    runtime.snapshot();
    runtime.flags(reference, 2, 255);
    runtime.finish("unused triangle metadata load/mutation");
    // Loading and state changes never dereference these triangle indices.
    let details = [[0, 3, 0, 1]];
    let triangles = [[255, 254, 253, 255]];
    p.detail_meshes = Some(&details);
    p.detail_triangles = &triangles;
    let data = check(&p, "loader preserves unused detail vertex indices");
    let mut runtime = Runtime::new(2, 8);
    let reference = runtime.add(&data, 0);
    runtime.snapshot();
    runtime.flags(reference, 0x8001, 7);
    runtime.finish("unused detail indices load/mutation");
    p.coordinate = [0; 3];
    p.bounds = [[0.; 3], [4.; 3]];
    p.cell_size = 1.;
    p.cell_height = 1.;
    for (details, triangles, point) in [
        (
            [[0, 3, 0, 2]],
            vec![[255, 254, 253, 0], [0, 1, 2, 21]],
            [-1., 2., 1.],
        ),
        ([[0, 3, 0, 255]], vec![[0, 1, 2, 21]], [1., 2., 1.]),
        ([[0, 3, 1 << 30, 1]], vec![[0, 1, 2, 21]], [1., 2., 1.]),
    ] {
        let mut p = p.clone();
        p.detail_meshes = Some(&details);
        p.detail_triangles = &triangles;
        let data = check(&p, "detail records are accessed only when visited");
        let mut runtime = Runtime::new(2, 8);
        let reference = runtime.add(&data, 0);
        runtime.closest(reference, point);
        runtime.finish("skipped detail indices, early height hit and wrapped triangle base");
    }
    let details = [[0, 5, 0, 1]];
    let detail_vertices = [
        [0., 0., 0.],
        [0., 0., 4.],
        [4., 0., 0.],
        [1., 2., 3.],
        [4., 5., 6.],
    ];
    let triangles = [[0, 1, 3, 21]];
    p.detail_meshes = Some(&details);
    p.detail_vertices = &detail_vertices;
    p.detail_triangles = &triangles;
    let mut data = create(&p).unwrap();
    let view = TileData::parse(&data).unwrap();
    let detail_at = HEADER_SIZE
        + view.header.vertex_count as usize * 12
        + view.header.polygon_count as usize * 32
        + view.link_bytes().len();
    // Native unsigned multiplication by three wraps to float offset one,
    // giving the initialized point [2, 3, 4] across two detail records.
    data[detail_at..detail_at + 4].copy_from_slice(&0xaaaa_aaab_u32.to_ne_bytes());
    let mut runtime = Runtime::new(2, 8);
    let reference = runtime.add(&data, 0);
    runtime.closest(reference, [0.2, 2., 1.]);
    let state = runtime.store(reference, runtime.mesh.tile_state_size(reference));
    runtime.flags(reference, 0x8001, 7);
    runtime.restore(reference, &state);
    runtime.finish("wrapped detail vertex base and state storage");
}

#[test]
fn native_candidate_bounds_ordered_clamp_and_nan_comparisons() {
    let vertices = [[0, 0, 0], [0, 0, 1], [1, 0, 1], [1, 0, 0]];
    let polygons = [polygon(&[0, 1, 2, 3], &[0xffff; 4], 1, 7)];
    let mut p = params(&vertices, &polygons);
    p.coordinate = [0; 3];
    p.bounds = [[0.; 3], [4.; 3]];
    p.cell_size = 1.;
    p.cell_height = 1.;
    p.build_bv_tree = false;
    let original = create(&p).unwrap();
    for pattern in 0..3 {
        let mut data = original.clone();
        for vertex in 0..4 {
            if pattern == 0 || (pattern == 1 && vertex == 0) || (pattern == 2 && vertex == 3) {
                let at = HEADER_SIZE + vertex * 12;
                data[at..at + 4].copy_from_slice(&0x7fc0_1275_u32.to_ne_bytes());
            }
        }
        let mut runtime = Runtime::new(2, 8);
        runtime.add(&data, 0);
        runtime.snapshot();
        for capacity in [None, Some(0), Some(1), Some(8)] {
            runtime.custom_polygons(
                [2., 0., 0.5],
                [0.25, 1., 0.25],
                &CallbackFilter::default(),
                capacity,
            );
        }
        assert_eq!(
            runtime.actual.last().unwrap()[1].as_array().unwrap().len(),
            usize::from(pattern != 1)
        );
        runtime.finish(&format!("NaN candidate bounds pattern={pattern}"));
    }
    let details = [[0, 4, 0, 2]];
    let detail_vertices = [
        [10., 10., 10.],
        [10., 10., 6.],
        [6., 10., 6.],
        [6., 10., 10.],
    ];
    let triangles = [[0, 1, 2, 5], [0, 2, 3, 20]];
    p.bounds = [[10.; 3], [0.; 3]];
    p.cell_size = -1.;
    p.build_bv_tree = true;
    p.detail_meshes = Some(&details);
    p.detail_vertices = &detail_vertices;
    p.detail_triangles = &triangles;
    let data = create(&p).unwrap();
    compare(&data, &bytes(&native(&p)), "reversed bound BV creator");
    let mut runtime = Runtime::new(2, 8);
    runtime.mesh = NavMesh::new([10.; 3], -10., -10., 2, 8).unwrap();
    runtime.input = String::from("runtime 10 10 10 -10 -10 2 8 64\n");
    runtime.add(&data, 0);
    runtime.snapshot();
    for capacity in [None, Some(0), Some(1), Some(8)] {
        // Both bounds clamp to the native low endpoint. All quantized
        // conversions are zero/one and defined despite the reversed box.
        runtime.custom_polygons([9.; 3], [0.25; 3], &CallbackFilter::default(), capacity);
    }
    assert_eq!(
        runtime.actual.last().unwrap()[1].as_array().unwrap().len(),
        1
    );
    runtime.finish("reversed candidate bound clamp");
}

#[test]
fn native_bv_split_axis_narrows_promoted_extents_to_unsigned_short() {
    let vertices = [
        [0, 2, 0],
        [8, 2, 0],
        [0, 2, 1],
        [0, 0, 2],
        [9, 0, 2],
        [0, 0, 3],
    ];
    let polygons = [
        polygon(&[0, 1, 2], &[0xffff; 3], 1, 7),
        polygon(&[3, 4, 5], &[0xffff; 3], 1, 7),
    ];
    let details = [[0, 3, 0, 1], [3, 3, 1, 1]];
    let dv = vertices.map(|v| v.map(|c| 10. - f32::from(c)));
    let triangles = [[0, 1, 2, 21]; 2];
    let mut p = params(&vertices, &polygons);
    p.bounds = [[10.; 3], [0.; 3]];
    p.cell_size = -1.;
    p.cell_height = -1.;
    p.detail_meshes = Some(&details);
    p.detail_vertices = &dv;
    p.detail_triangles = &triangles;
    let data = check(&p, "unsigned-short BV split-axis extents");
    let nodes: Vec<_> = TileData::parse(&data).unwrap().bv_nodes().collect();
    assert_eq!(nodes[0].min, [8, 0, 1]);
    assert_eq!(nodes[0].max, [0, 2, 2]);
    // Native extents are [65528,2,1]. Distinct X keys (8,9) select [0,1],
    // while an incorrect signed-extent comparison selects Y and [1,0].
    // Avoid ties here so this axis regression is independent of CRT tie order.
    assert_eq!([nodes[1].index, nodes[2].index], [0, 1]);
}

#[test]
fn native_small_positive_query_budgets_keep_defined_independent_operations() {
    let vertices = [
        [0, 0, 0],
        [0, 0, 2],
        [2, 0, 2],
        [2, 0, 0],
        [4, 0, 2],
        [4, 0, 0],
    ];
    let polygons = [
        polygon(&[0, 1, 2, 3], &[0xffff, 0xffff, 1, 0xffff], 1, 7),
        polygon(&[3, 2, 4, 5], &[0, 0xffff, 0xffff, 0xffff], 1, 7),
    ];
    let mut p = params(&vertices, &polygons);
    p.coordinate = [0; 3];
    p.bounds = [[0.; 3], [4., 4., 2.]];
    p.cell_size = 1.;
    p.cell_height = 1.;
    let data = create(&p).unwrap();
    for budget in [1, 2, 3] {
        let mut runtime = Runtime::with_nodes(2, 8, budget);
        assert_eq!(runtime.query.node_pool().hash_size(), 0);
        runtime.query_snapshot();
        let reference = runtime.add(&data, 0);
        let standalone = runtime.mesh.find_path(
            reference,
            reference,
            [1., 0., 1.],
            [1.5, 0., 1.],
            &Filter::default(),
            budget,
            8,
        );
        assert_eq!(standalone.status, SUCCESS);
        assert_eq!(standalone.polygons, [reference]);
        runtime.persistent_path(reference, reference, [1., 0., 1.], [1.5, 0., 1.], 8);
        runtime.begin_slice(
            reference,
            reference,
            [1., 0., 1.],
            [1.5, 0., 1.],
            FINDPATH_ANY_ANGLE,
            CallbackFilter::default(),
        );
        runtime.update_slice(3, true);
        runtime.finish_slice(8, None);
        runtime.local(reference, [1., 0., 1.], 3., 8, CallbackFilter::default());
        let moved = runtime.surface(
            reference,
            [1., 0., 1.],
            [3., 0., 1.],
            8,
            CallbackFilter::default(),
        );
        assert_eq!(moved.visited.len(), 2);
        runtime.random(None, &[0.25, 0.5, 0.75], CallbackFilter::default());
        runtime.query_init(0);
        runtime.query_init(3);
        runtime.query_init(4);
        runtime.query_init(1);
        runtime.persistent_path(reference, reference + 1, [1., 0., 1.], [3., 0., 1.], 8);
        runtime.finish(&format!("small main node budget={budget}"));
    }
    // Do not send these main-hash operations to native: its zero-length
    // table access would be invalid. Safe Rust returns checked failures.
    let mesh = NavMesh::from_tile_data(&data).unwrap();
    let start = mesh.reference(0, 0);
    let end = mesh.reference(0, 1);
    let f = Filter::default();
    assert_eq!(
        mesh.find_path(start, end, [1., 0., 1.], [3., 0., 1.], &f, 1, 8)
            .status,
        FAILURE | INVALID_PARAM
    );
    let mut query = NavMeshQuery::new(1).unwrap();
    assert_eq!(
        query
            .find_path(&mesh, start, end, [1., 0., 1.], [3., 0., 1.], &f, 8)
            .status,
        FAILURE | INVALID_PARAM
    );
    assert_eq!(
        query
            .find_polygons_around_circle(&mesh, start, [1., 0., 1.], 3., &f, 8)
            .status,
        FAILURE | INVALID_PARAM
    );
    assert_eq!(
        query.init_sliced_find_path(&mesh, start, end, [1., 0., 1.], [3., 0., 1.], &f, 0),
        FAILURE | INVALID_PARAM
    );
    assert_eq!(
        query
            .find_random_point_around_circle(&mesh, start, [1., 0., 1.], 3., &f, &mut || 0.5)
            .0,
        FAILURE | INVALID_PARAM
    );
    assert_eq!(
        query
            .find_distance_to_wall(&mesh, start, [1., 0., 1.], 3., &f)
            .0,
        FAILURE | INVALID_PARAM
    );
    let mut pool = recast_nav::nodes::NodePool::new(4, 0).unwrap();
    assert!(pool.get_node(start, 0).is_none());
    assert!(pool.find_node(start, 0).is_none());
    assert!(pool.find_nodes(start, 8).is_empty());
    assert_eq!(pool.node_count(), 0);
}

#[test]
fn native_loading_preserves_metadata_unused_by_linking() {
    let vertices = [[0, 0, 0], [0, 0, 4], [4, 0, 4], [4, 0, 0]];
    let polygons = [polygon(&[0, 1, 2, 3], &[0xffff; 4], 1, 7)];
    let mut p = params(&vertices, &polygons);
    p.coordinate = [0; 3];
    p.bounds = [[0.; 3], [4.; 3]];
    p.cell_size = 1.;
    p.cell_height = 1.;
    let original = create(&p).unwrap();
    let view = TileData::parse(&original).unwrap();
    let polygon_at = HEADER_SIZE + view.header.vertex_count as usize * 12;
    let bv_at = polygon_at
        + view.header.polygon_count as usize * 32
        + view.link_bytes().len()
        + view.header.detail_mesh_count as usize * 12
        + view.header.detail_vertex_count as usize * 12
        + view.header.detail_triangle_count as usize * 4;
    assert!(view.header.bv_node_count > 0);
    for mutation in 0..12 {
        let mut data = original.clone();
        match mutation {
            0 => data[polygon_at + 4..polygon_at + 6].copy_from_slice(&u16::MAX.to_ne_bytes()),
            1 => data[polygon_at + 16..polygon_at + 18].copy_from_slice(&0x7fff_u16.to_ne_bytes()),
            5 => data[polygon_at + 30] = 0,
            2..=4 => data[bv_at + 12..bv_at + 16]
                .copy_from_slice(&[i32::MAX, i32::MIN, -1000][mutation - 2].to_ne_bytes()),
            _ => {
                data[polygon_at + 30] = [0, 1, 2, 4, 7, 255][mutation - 6];
                data[polygon_at + 31] = (data[polygon_at + 31] & 63) | 64;
            }
        }
        // No off-mesh connections or neighboring tiles exist, so native
        // linking never reads vertices/BV nodes or follows the created link.
        // A zero-edge ground polygon performs no internal-link iterations.
        // Off-mesh types skip that loop regardless of their stored count.
        let native = operation("load", &data);
        assert_eq!(native["status"], SUCCESS);
        let mesh = NavMesh::from_tile_data(&data).unwrap();
        compare(
            &mesh.tile_data(mesh.reference(0, 0)).unwrap(),
            &bytes(&native),
            "unused loading metadata",
        );
        let mut runtime = Runtime::new(2, 8);
        let reference = runtime.add(&data, 0);
        assert_ne!(reference, 0);
        let state = runtime.store(reference, runtime.mesh.tile_state_size(reference));
        runtime.flags(reference, 0x8001, 63);
        runtime.restore(reference, &state);
        runtime.remove(reference);
        runtime.finish(&format!("unused loading metadata mutation={mutation}"));
    }
}

#[test]
fn native_off_mesh_fixed_vertex_slots_and_loading_types() {
    let vertices = [[0, 0, 0], [0, 0, 4], [4, 0, 4], [4, 0, 0]];
    let polygons = [polygon(&[0, 1, 2, 3], &[0xffff; 4], 1, 7)];
    let connections = [CreateOffMeshConnection {
        positions: [[1., 0.2, 1.], [3., 0.2, 3.]],
        radius: 0.8,
        flags: 2,
        area: 5,
        direction: 1,
    }];
    let mut p = params(&vertices, &polygons);
    p.coordinate = [0; 3];
    p.bounds = [[0.; 3], [4.; 3]];
    p.cell_size = 1.;
    p.cell_height = 1.;
    p.off_mesh_connections = &connections;
    let original = create(&p).unwrap();
    let view = TileData::parse(&original).unwrap();
    assert_eq!(view.header.off_mesh_base, 1);
    assert_eq!(view.header.off_mesh_connection_count, 1);
    let polygon_at = HEADER_SIZE + view.header.vertex_count as usize * 12 + 32;
    for (kind, count) in [0, 1, 2, 3, 6, 7, 255]
        .into_iter()
        .map(|count| (1, count))
        .chain([0, 2, 3].map(|kind| (kind, 2)))
    {
        let mut data = original.clone();
        data[polygon_at + 30] = count;
        data[polygon_at + 31] = (data[polygon_at + 31] & 63) | (kind << 6);
        // The BV tree visits only the ground polygon. With no adjacent tiles,
        // linking reads the connection's two fixed vertex slots, not its count.
        // Endpoint queries also use those slots, or reject a non-off-mesh type.
        let native = operation("load", &data);
        assert_eq!(native["status"], SUCCESS);
        let mesh = NavMesh::from_tile_data(&data).unwrap();
        compare(
            &mesh.tile_data(mesh.reference(0, 0)).unwrap(),
            &bytes(&native),
            "off-mesh fixed slots",
        );
        let mut runtime = Runtime::new(2, 8);
        let base = runtime.add(&data, 0);
        assert_ne!(base, 0);
        runtime.endpoints(base, base | 1);
        runtime.endpoints(0, base | 1);
        let state = runtime.store(base, runtime.mesh.tile_state_size(base));
        runtime.flags(base | 1, 0x8001, 63);
        runtime.restore(base, &state);
        runtime.remove(base);
        runtime.finish(&format!("off-mesh fixed slots kind={kind} count={count}"));
    }
}

#[test]
fn native_record_views_preserve_backing_and_mutation() {
    fn exported(tile: &recast_nav::nav::Tile) -> Vec<u8> {
        let (header, body) = tile.data_parts();
        let bytes: Vec<_> = header.into_iter().chain(body.iter().copied()).collect();
        assert_eq!(bytes, tile.data().unwrap());
        bytes
    }
    let vertices = [
        [0, 0, 0],
        [0, 0, 4],
        [4, 0, 4],
        [4, 0, 0],
        [8, 0, 4],
        [8, 0, 0],
    ];
    let polygons = [
        polygon(&[0, 1, 2, 3], &[0xffff, 0xffff, 1, 0xffff], 1, 0),
        polygon(&[3, 2, 4, 5], &[0, 0xffff, 0xffff, 0xffff], 1, 1),
    ];
    let details = [[0, 5, 0, 4], [5, 5, 4, 4]];
    let detail_vertices = [
        [0., 0., 0.],
        [0., 0., 4.],
        [4., 0., 4.],
        [4., 0., 0.],
        [2., 0., 2.],
        [4., 0., 0.],
        [4., 0., 4.],
        [8., 0., 4.],
        [8., 0., 0.],
        [6., 0., 2.],
    ];
    let triangles = [
        [0, 1, 4, 1],
        [1, 2, 4, 1],
        [2, 3, 4, 1],
        [3, 0, 4, 1],
        [0, 1, 4, 1],
        [1, 2, 4, 1],
        [2, 3, 4, 1],
        [3, 0, 4, 1],
    ];
    let connections = [
        CreateOffMeshConnection {
            positions: [[1., 0., 1.], [7., 0., 1.]],
            radius: 0.5,
            flags: 1,
            area: 2,
            direction: 1,
        },
        CreateOffMeshConnection {
            positions: [[7., 0., 3.], [1., 0., 3.]],
            radius: 0.5,
            flags: 1,
            area: 3,
            direction: 1,
        },
    ];
    let mut params = params(&vertices, &polygons);
    params.bounds = [[0.; 3], [8., 4., 4.]];
    params.cell_size = 1.;
    params.cell_height = 1.;
    params.detail_meshes = Some(&details);
    params.detail_vertices = &detail_vertices;
    params.detail_triangles = &triangles;
    params.off_mesh_connections = &connections;
    let mut data = create(&params).unwrap();
    let view = TileData::parse(&data).unwrap();
    let detail_start = HEADER_SIZE
        + view.header.vertex_count as usize * 12
        + view.header.polygon_count as usize * 32
        + view.link_bytes().len();
    let detail_count = view.header.detail_mesh_count as usize;
    let link_start = HEADER_SIZE
        + view.header.vertex_count as usize * 12
        + view.header.polygon_count as usize * 32;
    let link_size = view.link_bytes().len() / view.header.max_link_count as usize;
    let link_end = link_start + view.link_bytes().len();
    for i in 0..detail_count {
        data[detail_start + i * 12 + 10..detail_start + i * 12 + 12].copy_from_slice(&[0xa5, 0x81]);
    }
    // Defined short link pools exhaust during internal or off-mesh attachment.
    // Keep all other sections intact, including their initialized spare bytes.
    for budget in [1usize, 2, 3, 4] {
        let mut limited = data.clone();
        limited.drain(link_start + budget * link_size..link_end);
        limited[32..36].copy_from_slice(&(budget as u32).to_ne_bytes());
        let mut runtime = Runtime::new(2, 8);
        runtime.add(&limited, 0);
        runtime.finish(&format!("link pool budget {budget}"));
    }
    let mut r = Runtime::new(2, 8);
    let reference = r.add(&data, 0);
    let slot = r.mesh.tile_by_ref(reference).unwrap().index;
    let tile = r.mesh.tiles[slot].as_mut().unwrap();
    assert_eq!(tile.polygons().len(), tile.polygon_count());
    for index in [0, tile.polygon_count() - 1] {
        let original = tile.polygon(index).unwrap().to_owned();
        let changed = Polygon {
            first_link: 0xfedc_ba98,
            vertices: [0xffff, 0x8001, 17, 0, 0xa581, 0xfe03],
            neighbors: [0x8000, 0xffff, 1, 0, 0xfe81, 0x1234],
            flags: 0xa581,
            vertex_count: 2,
            area_and_type: 0xfe,
        };
        // Raw export does not dereference the active count. Restore every
        // record before queries; native active-index access above six is invalid.
        for vertex_count in [0, 2, 6, 7, 255] {
            let changed = Polygon {
                vertex_count,
                ..changed.clone()
            };
            for polygon in [&changed, &original] {
                write!(
                    r.input,
                    "polygon_record {reference} {index} {} ",
                    polygon.first_link
                )
                .unwrap();
                values(&mut r.input, polygon.vertices);
                values(&mut r.input, polygon.neighbors);
                writeln!(
                    r.input,
                    "{} {} {}",
                    polygon.flags, polygon.vertex_count, polygon.area_and_type
                )
                .unwrap();
                tile.set_polygon(index, polygon).unwrap();
                let view = tile.polygon(index).unwrap();
                assert_eq!(&view.to_owned(), polygon);
                if polygon.vertex_count <= 6 {
                    assert!(view.vertices().eq(polygon.vertices().iter().copied()));
                    assert!(view.neighbors().eq(polygon.neighbors().iter().copied()));
                }
                r.actual.push(json!(exported(tile)));
            }
        }
    }
    let original_header = tile.header.clone();
    for value in [0, u32::MAX] {
        let mut changed = original_header.clone();
        for count in [
            &mut changed.polygon_count,
            &mut changed.vertex_count,
            &mut changed.max_link_count,
            &mut changed.detail_mesh_count,
            &mut changed.detail_vertex_count,
            &mut changed.detail_triangle_count,
            &mut changed.bv_node_count,
            &mut changed.off_mesh_connection_count,
        ] {
            *count = value;
        }
        // Native raw export copies the allocated buffer; it never follows
        // these counts. Restore the header before any structural operation.
        for header in [&changed, &original_header] {
            write!(r.input, "header_counts {reference} ").unwrap();
            values(
                &mut r.input,
                [
                    header.polygon_count,
                    header.vertex_count,
                    header.max_link_count,
                    header.detail_mesh_count,
                    header.detail_vertex_count,
                    header.detail_triangle_count,
                    header.bv_node_count,
                    header.off_mesh_connection_count,
                ],
            );
            tile.header = header.clone();
            r.actual.push(json!(exported(tile)));
        }
    }
    let before = exported(tile);
    let polygon = tile.polygon(0).unwrap().to_owned();
    for index in [tile.polygon_count(), usize::MAX] {
        assert!(tile.polygon(index).is_none());
        assert!(tile.polygon_mut(index).is_none());
        assert_eq!(
            tile.set_polygon(index, &polygon),
            Err(FAILURE | INVALID_PARAM)
        );
        assert_eq!(exported(tile), before);
    }
    let count = tile.link_count();
    for index in [0, tile.free_link as usize, count - 1] {
        let original = tile.link(index).unwrap();
        let changed = recast_nav::nav::Link {
            reference: PolyRef::MAX,
            next: original.next,
            edge: 0xa5,
            side: 0xfe,
            min: 0x81,
            max: 0xf3,
        };
        for link in [&changed, &original] {
            writeln!(
                r.input,
                "link_record {reference} {index} {} {} {} {} {} {}",
                link.reference, link.next, link.edge, link.side, link.min, link.max
            )
            .unwrap();
            tile.set_link(index, link).unwrap();
            r.actual.push(json!(exported(tile)));
        }
    }
    let before = exported(tile);
    let record = tile.link(0).unwrap();
    for index in [count, usize::MAX] {
        assert!(tile.link(index).is_none());
        assert_eq!(tile.set_link(index, &record), Err(FAILURE | INVALID_PARAM));
        assert_eq!(exported(tile), before);
    }
    for detail in [false, true] {
        let count = if detail {
            tile.detail_vertex_count()
        } else {
            tile.vertex_count()
        };
        assert!(count >= 2);
        for index in [0, count - 1] {
            let original = if detail {
                tile.detail_vertex(index)
            } else {
                tile.vertex(index)
            }
            .unwrap();
            for point in [
                [0x8000_0000, 0x7f81_2345, 0xffc5_4321].map(f32::from_bits),
                original,
            ] {
                let op = if detail {
                    "detail_vertex_record"
                } else {
                    "vertex_record"
                };
                writeln!(
                    r.input,
                    "{op} {reference} {index} {} {} {}",
                    point[0].to_bits(),
                    point[1].to_bits(),
                    point[2].to_bits()
                )
                .unwrap();
                let read = if detail {
                    tile.set_detail_vertex(index, point).unwrap();
                    tile.detail_vertex(index).unwrap()
                } else {
                    tile.set_vertex(index, point).unwrap();
                    tile.vertex(index).unwrap()
                };
                assert_eq!(read.map(f32::to_bits), point.map(f32::to_bits));
                r.actual.push(json!(exported(tile)));
            }
        }
        let before = exported(tile);
        for index in [count, usize::MAX] {
            let (read, write) = if detail {
                (
                    tile.detail_vertex(index),
                    tile.set_detail_vertex(index, [0.; 3]),
                )
            } else {
                (tile.vertex(index), tile.set_vertex(index, [0.; 3]))
            };
            assert!(read.is_none());
            assert_eq!(write, Err(FAILURE | INVALID_PARAM));
            assert_eq!(exported(tile), before);
        }
    }
    assert_eq!(tile.vertices().len(), tile.vertex_count());
    assert_eq!(tile.detail_vertices().len(), tile.detail_vertex_count());
    for index in [0, tile.detail_mesh_count() - 1] {
        let original = tile.detail_mesh(index).unwrap();
        let changed = DetailMesh {
            vertex_base: u32::MAX,
            triangle_base: 0x8000_0001,
            vertex_count: 255,
            triangle_count: 254,
        };
        for detail in [changed, original] {
            writeln!(
                r.input,
                "detail_record {reference} {index} {} {} {} {}",
                detail.vertex_base,
                detail.triangle_base,
                detail.vertex_count,
                detail.triangle_count
            )
            .unwrap();
            tile.set_detail_mesh(index, detail.clone()).unwrap();
            assert_eq!(tile.detail_mesh(index).unwrap(), detail);
            r.actual.push(json!(exported(tile)));
        }
    }
    assert_eq!(tile.detail_meshes().len(), tile.detail_mesh_count());
    for index in [0, tile.off_mesh_connection_count() - 1] {
        let original = tile.off_mesh_connection(index).unwrap();
        let changed = OffMeshConnection {
            positions: [
                [0x8000_0000, 0x7f81_2345, 0xffc5_4321].map(f32::from_bits),
                [1., 2., 3.],
            ],
            radius: f32::from_bits(0xff81_2345),
            polygon: u16::MAX,
            flags: 0xfe,
            side: 0x81,
            user_id: 0xfedc_ba98,
        };
        for connection in [changed, original] {
            write!(r.input, "off_mesh_record {reference} {index} ").unwrap();
            values(
                &mut r.input,
                connection.positions.into_iter().flatten().map(f32::to_bits),
            );
            writeln!(
                r.input,
                "{} {} {} {} {}",
                connection.radius.to_bits(),
                connection.polygon,
                connection.flags,
                connection.side,
                connection.user_id
            )
            .unwrap();
            tile.set_off_mesh_connection(index, connection.clone())
                .unwrap();
            let read = tile.off_mesh_connection(index).unwrap();
            assert_eq!(
                read.positions.map(|p| p.map(f32::to_bits)),
                connection.positions.map(|p| p.map(f32::to_bits))
            );
            assert_eq!(read.radius.to_bits(), connection.radius.to_bits());
            assert_eq!(
                (read.polygon, read.flags, read.side, read.user_id),
                (
                    connection.polygon,
                    connection.flags,
                    connection.side,
                    connection.user_id
                )
            );
            r.actual.push(json!(exported(tile)));
        }
    }
    assert_eq!(
        tile.off_mesh_connections().len(),
        tile.off_mesh_connection_count()
    );
    let before = exported(tile);
    for index in [tile.detail_mesh_count(), usize::MAX] {
        assert!(tile.detail_mesh(index).is_none());
        assert_eq!(
            tile.set_detail_mesh(index, tile.detail_mesh(0).unwrap()),
            Err(FAILURE | INVALID_PARAM)
        );
        assert_eq!(exported(tile), before);
    }
    for index in [tile.off_mesh_connection_count(), usize::MAX] {
        assert!(tile.off_mesh_connection(index).is_none());
        assert_eq!(
            tile.set_off_mesh_connection(index, tile.off_mesh_connection(0).unwrap()),
            Err(FAILURE | INVALID_PARAM)
        );
        assert_eq!(exported(tile), before);
    }
    for index in [0, tile.detail_triangles().len() - 1] {
        let original = tile.detail_triangles()[index];
        for triangle in [[0xff, 0xa5, 0x81, 0xfe], original] {
            writeln!(
                r.input,
                "triangle_record {reference} {index} {} {} {} {}",
                triangle[0], triangle[1], triangle[2], triangle[3]
            )
            .unwrap();
            tile.detail_triangles_mut()[index] = triangle;
            r.actual.push(json!(exported(tile)));
        }
    }
    for index in [0, tile.bv_node_count() - 1] {
        let original = tile.bv_node(index).unwrap();
        let changed = recast_nav::nav_data::BvNode {
            min: [u16::MAX, 0, 17],
            max: [1, u16::MAX - 1, 42],
            index: i32::MIN,
        };
        for node in [&changed, &original] {
            write!(r.input, "bv_record {reference} {index} ").unwrap();
            values(&mut r.input, node.min);
            values(&mut r.input, node.max);
            writeln!(r.input, "{}", node.index).unwrap();
            tile.set_bv_node(index, node).unwrap();
            assert_eq!(tile.bv_node(index).as_ref(), Some(node));
            r.actual.push(json!(exported(tile)));
        }
    }
    let before = exported(tile);
    let node = tile.bv_node(0).unwrap();
    for index in [tile.bv_node_count(), usize::MAX] {
        assert!(tile.bv_node(index).is_none());
        assert_eq!(tile.set_bv_node(index, &node), Err(FAILURE | INVALID_PARAM));
        assert_eq!(exported(tile), before);
    }
    assert_eq!(tile.bv_nodes().len(), tile.bv_node_count());
    r.finish("native tile record storage and exact byte mutation");
}

#[test]
fn native_queries_duplicate_portals_and_reference_hash_collisions() {
    let verts: Vec<_> = (0..=9)
        .flat_map(|z| (0..=9).map(move |x| [x, 0, z]))
        .collect();
    let polys: Vec<_> = (0..9)
        .flat_map(|z| {
            (0..9).map(move |x| {
                let a = z * 10 + x;
                let i = z * 9 + x;
                polygon(
                    &[a, a + 10, a + 11, a + 1],
                    &[
                        if x == 0 { 0xffff } else { i - 1 },
                        if z == 8 { 0xffff } else { i + 9 },
                        if x == 8 { 0xffff } else { i + 1 },
                        if z == 0 { 0xffff } else { i - 9 },
                    ],
                    1,
                    0,
                )
            })
        })
        .collect();
    let mut p = params(&verts, &polys);
    p.coordinate = [0; 3];
    p.bounds = [[0.; 3], [9.; 3]];
    p.cell_size = 1.;
    let data = create(&p).unwrap();
    let mut r = Runtime::with_nodes(1, 128, 128);
    let base = r.add(&data, 0);
    let start = base | 40;
    let first = base | 1;
    let hash = |reference: PolyRef| (reference ^ (reference >> 5)) & 63;
    let collision = (2..81)
        .map(|i| base | i)
        .find(|&reference| reference != start && hash(reference) == hash(first))
        .unwrap();
    assert_ne!(collision, first);
    let tile = r.mesh.tiles[0].as_mut().unwrap();
    let mut index = tile.polygon(40).unwrap().first_link() as usize;
    // The third link duplicates the first but has a different portal edge.
    // Rejecting the first callback must still use its portal on the third.
    for reference in [first, collision, first] {
        let mut link = tile.link(index).unwrap();
        link.reference = reference;
        writeln!(
            r.input,
            "link_record {base} {index} {} {} {} {} {} {}",
            link.reference, link.next, link.edge, link.side, link.min, link.max
        )
        .unwrap();
        tile.set_link(index, &link).unwrap();
        r.actual.push(json!(tile.data().unwrap()));
        index = link.next as usize;
    }
    let shape = [[3., 0., 3.], [3., 0., 6.], [6., 0., 6.], [6., 0., 3.]];
    for blocked in [255, 254] {
        let filter = || CallbackFilter {
            blocked,
            ..CallbackFilter::default()
        };
        r.custom_path_at(start, first, [4.5, 0., 4.5], [1.5, 0., 0.5], &filter(), 81);
        for options in [0, recast_nav::nav::FINDPATH_ANY_ANGLE] {
            r.begin_slice(
                start,
                first,
                [4.5, 0., 4.5],
                [1.5, 0., 0.5],
                options,
                filter(),
            );
            let status = r.update_slice(512, true);
            assert_eq!(status & recast_nav::nav::IN_PROGRESS, 0);
            r.finish_slice(81, None);
        }
        for radius in [0.25, 1., 100.] {
            let filter = || CallbackFilter {
                blocked,
                ..CallbackFilter::default()
            };
            r.local(start, [4.5, 0., 4.5], radius, 81, filter());
            r.distance_wall(start, [4.5, 0., 4.5], radius, filter());
            r.random(
                Some((start, [4.5, 0., 4.5], radius)),
                &[0.125, 0.625, 0.875],
                filter(),
            );
            for shape in [None, Some(shape.as_slice())] {
                r.around(
                    start,
                    [4.5, 0., 4.5],
                    radius,
                    shape,
                    81,
                    CallbackFilter {
                        blocked,
                        ..CallbackFilter::default()
                    },
                );
                r.dijkstra_path(first, 81);
                r.dijkstra_path(collision, 81);
            }
        }
    }
    r.finish("Native first portal across duplicate links, hash collisions and stateful filters");
}

#[test]
fn native_external_link_cap_exhaustion_and_reuse() {
    let vertices = [[0, 0, 0], [0, 0, 4], [4, 0, 4], [4, 0, 0]];
    let owners: Vec<_> = (0..2)
        .map(|_| polygon(&[0, 1, 2, 3], &[0xffff, 0xffff, 0x8002, 0xffff], 1, 0))
        .collect();
    // More than four matching polygons share the same external edge. Native
    // retains the first four in traversal order, then inserts them at the head.
    let targets: Vec<_> = (0..6)
        .map(|_| polygon(&[0, 1, 2, 3], &[0x8000, 0xffff, 0xffff, 0xffff], 1, 0))
        .collect();
    let owner_params = CreateParams {
        coordinate: [0; 3],
        bounds: [[0.; 3], [4.; 3]],
        cell_size: 1.,
        ..params(&vertices, &owners)
    };
    let target_params = CreateParams {
        coordinate: [1, 0, 0],
        bounds: [[4., 0., 0.], [8., 4., 4.]],
        polygons: &targets,
        ..owner_params.clone()
    };
    let data = create(&owner_params).unwrap();
    let target_data = create(&target_params).unwrap();
    // A tile can acquire portals through native-defined record replacement
    // after loading. A cached portal-free fact must not survive that mutation.
    let mut closed = data.clone();
    let polygon_start = HEADER_SIZE + vertices.len() * 12;
    for index in 0..owners.len() {
        closed[polygon_start + index * 32 + 20..polygon_start + index * 32 + 22]
            .copy_from_slice(&0_u16.to_ne_bytes());
    }
    let mut r = Runtime::new(2, 8);
    let owner = r.add(&closed, 0);
    let slot = r.mesh.tile_by_ref(owner).unwrap().index;
    for index in 0..owners.len() {
        let tile = r.mesh.tiles[slot].as_mut().unwrap();
        let mut changed = tile.polygon(index).unwrap().to_owned();
        changed.neighbors[2] = 0x8000;
        write!(
            r.input,
            "polygon_record {owner} {index} {} ",
            changed.first_link
        )
        .unwrap();
        values(&mut r.input, changed.vertices);
        values(&mut r.input, changed.neighbors);
        writeln!(
            r.input,
            "{} {} {}",
            changed.flags, changed.vertex_count, changed.area_and_type
        )
        .unwrap();
        tile.set_polygon(index, &changed).unwrap();
        r.actual.push(json!(tile.data().unwrap()));
    }
    let target = r.add(&target_data, 0);
    let tile = r.mesh.tile_by_ref(owner).unwrap().tile.unwrap();
    assert_ne!(tile.polygon(0).unwrap().first_link(), u32::MAX);
    r.remove(target);
    r.remove(owner);
    r.finish("external portals added by replacing loaded polygon records");
    let view = TileData::parse(&data).unwrap();
    let link_start = HEADER_SIZE
        + view.header.vertex_count as usize * 12
        + view.header.polygon_count as usize * 32;
    let link_size = view.link_bytes().len() / view.header.max_link_count as usize;
    let link_end = link_start + view.link_bytes().len();
    for budget in [1usize, 3, 4, 5, 8] {
        let mut limited = data.clone();
        limited.drain(link_start + budget * link_size..link_end);
        limited[32..36].copy_from_slice(&(budget as u32).to_ne_bytes());
        for target_first in [false, true] {
            let mut r = Runtime::new(2, 8);
            let (owner, target) = if target_first {
                let target = r.add(&target_data, 0);
                (r.add(&limited, 0), target)
            } else {
                let owner = r.add(&limited, 0);
                (owner, r.add(&target_data, 0))
            };
            let tile = r.mesh.tile_by_ref(owner).unwrap().tile.unwrap();
            for polygon in 0..2 {
                let mut references = Vec::new();
                let mut at = tile.polygon(polygon).unwrap().first_link();
                while at != u32::MAX {
                    let link = tile.link(at as usize).unwrap();
                    references.push(link.reference);
                    at = link.next;
                }
                let count = budget.saturating_sub(polygon * 4).min(4);
                assert_eq!(
                    references,
                    (0..count)
                        .rev()
                        .map(|p| target | p as PolyRef)
                        .collect::<Vec<_>>()
                );
            }
            let removed = r.remove(target);
            assert_eq!(r.add(&removed, target), target);
            let removed = r.remove(owner);
            assert_eq!(r.add(&removed, owner), owner);
            r.remove(target);
            r.remove(owner);
            r.finish(&format!(
                "external link cap, budget={budget}, target_first={target_first}"
            ));
        }
    }
}
