#![cfg(feature = "cpp-oracle")]
#[path = "support/float_comparison.rs"]
mod float_comparison;
#[path = "support/float_operands.rs"]
mod float_operands;
#[path = "support/parity_trace.rs"]
mod parity_trace;
use recast_nav::recast::*;
use std::{
    fmt::Write as _,
    io::Write,
    process::{Command, Stdio},
};

fn oracle(input: &str) -> serde_json::Value {
    let mut child = Command::new(env!("RECAST_UTILITIES_ORACLE"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    let output = std::thread::scope(|scope| {
        let mut input_pipe = child.stdin.take().unwrap();
        let writer = scope.spawn(move || input_pipe.write_all(input.as_bytes()).unwrap());
        let output = child.wait_with_output().unwrap();
        writer.join().unwrap();
        output
    });
    assert!(output.status.success(), "oracle failed: {}", output.status);
    serde_json::from_slice(&output.stdout).unwrap()
}

fn snapshot(hf: &CompactHeightfield) -> Vec<i64> {
    let mut out = vec![hf.border_size as i64, hf.max_region.bits() as i64];
    for c in &hf.cells {
        out.extend([c.index() as i64, c.count() as i64]);
    }
    for (span, area) in hf.spans.iter().zip(&hf.areas) {
        out.extend([
            span.y as i64,
            span.height() as i64,
            span.region.bits() as i64,
        ]);
        for d in 0..4 {
            out.push(span.con(d).unwrap_or(63) as i64);
        }
        out.push(area.0 as i64);
    }
    out
}

// Valid stacked surfaces, with disconnected boundaries and null spans.
fn field(seed: u32) -> CompactHeightfield {
    let mut hf = CompactHeightfield {
        width: 9,
        height: 7,
        cell_size: 0.3,
        cell_height: 0.2,
        aabb: recast_nav::Aabb3d {
            min: [-1.1, -0.7, -0.9].into(),
            max: [1.6, 5.7, 1.2].into(),
        },
        ..Default::default()
    };
    let mut random = seed;
    for z in 0..hf.height {
        for x in 0..hf.width {
            let mut c = CompactCell::default();
            c.set_index(hf.spans.len() as u32);
            c.set_count(2);
            hf.cells.push(c);
            for layer in 0..2 {
                random = random.wrapping_mul(1664525).wrapping_add(1013904223);
                let mut span = CompactSpan {
                    y: 2 + layer as u16 * 16,
                    region: (((x + z) % 4) as u16).into(),
                    ..Default::default()
                };
                span.set_height(12);
                for (d, valid) in [x > 0, z + 1 < hf.height, x + 1 < hf.width, z > 0]
                    .into_iter()
                    .enumerate()
                {
                    span.set_con(d as u8, valid.then_some(layer));
                }
                hf.spans.push(span);
                hf.areas.push(AreaType(if random.is_multiple_of(11) {
                    0
                } else {
                    1 + (random % 6) as u8
                }));
            }
        }
    }
    hf
}

fn field_input(hf: &CompactHeightfield) -> String {
    let mut input = format!(
        "areas {} {} {} {} ",
        hf.width, hf.height, hf.cell_size, hf.cell_height
    );
    for v in hf
        .aabb
        .min
        .to_array()
        .into_iter()
        .chain(hf.aabb.max.to_array())
    {
        write!(input, "{v} ").unwrap();
    }
    writeln!(input, "{}", hf.spans.len()).unwrap();
    for v in &snapshot(hf)[2..] {
        write!(input, "{v} ").unwrap();
    }
    input
}

#[test]
fn ordered_area_operations_match_recast() {
    for seed in [1, 19, 213, 0xabcdef01] {
        let mut hf = field(seed);
        let mut input = field_input(&hf);
        let operations = [
            "median",
            "box -2 -1 -2 0.2 0.2 0.1 9",
            "box -3 -1 -3 -1.5 7 -1.3 0",
            "box 0.1 2.9 -0.4 2 2.9 2 11",
            "cylinder -0.2 -1 0 0.6 7 17",
            "cylinder -0.95 -1 -0.75 0.3 1 0",
            "convex 4 -1 7 23 -1 13 -0.8 0.8 -13 -0.5 1 0 1 -0.7 4 0.6",
            "median",
            "median",
        ];
        writeln!(input, "{}", operations.len()).unwrap();
        for op in operations {
            writeln!(input, "{op}").unwrap();
        }
        let mut actual = vec![snapshot(&hf)];
        median_filter_walkable_area(&mut hf);
        actual.push(snapshot(&hf));
        mark_box_area(&mut hf, [-2., -1., -2.], [0.2, 0.2, 0.1], AreaType(9));
        actual.push(snapshot(&hf));
        mark_box_area(&mut hf, [-3., -1., -3.], [-1.5, 7., -1.3], AreaType(0));
        actual.push(snapshot(&hf));
        mark_box_area(&mut hf, [0.1, 2.9, -0.4], [2., 2.9, 2.], AreaType(11));
        actual.push(snapshot(&hf));
        mark_cylinder_area(&mut hf, [-0.2, -1., 0.], 0.6, 7., AreaType(17));
        actual.push(snapshot(&hf));
        mark_cylinder_area(&mut hf, [-0.95, -1., -0.75], 0.3, 1., AreaType(0));
        actual.push(snapshot(&hf));
        mark_convex_poly_area(
            &mut hf,
            &[
                [-1., 13., -0.8],
                [0.8, -13., -0.5],
                [1., 0., 1.],
                [-0.7, 4., 0.6],
            ],
            -1.,
            7.,
            AreaType(23),
        );
        actual.push(snapshot(&hf));
        median_filter_walkable_area(&mut hf);
        actual.push(snapshot(&hf));
        median_filter_walkable_area(&mut hf);
        actual.push(snapshot(&hf));
        let expected: Vec<Vec<i64>> = serde_json::from_value(oracle(&input)).unwrap();
        for (step, (a, e)) in actual.iter().zip(&expected).enumerate() {
            assert_eq!(a, e, "seed {seed}, operation {step}");
        }
        assert_eq!(actual.len(), expected.len());
    }
    // Five center/cardinal votes fix the median even when every diagonal is
    // extreme. A differing cardinal can instead require the full diagonal pass.
    for diagonal in [1, 255] {
        for cardinal in [0, 3, 17] {
            let mut hf = field(1);
            hf.areas.fill(AreaType(17));
            for layer in 0..2 {
                hf.areas[((3 + 3 * 9) * 2) + layer] = AreaType(cardinal);
                for (x, z) in [(3, 2), (3, 4), (5, 2), (5, 4)] {
                    hf.areas[((x + z * 9) * 2) + layer] = AreaType(diagonal);
                }
            }
            let mut input = field_input(&hf);
            input.push_str("1\nmedian\n");
            let before = snapshot(&hf);
            median_filter_walkable_area(&mut hf);
            let actual = vec![before, snapshot(&hf)];
            let expected: Vec<Vec<i64>> = serde_json::from_value(oracle(&input)).unwrap();
            assert_eq!(actual, expected, "diagonal={diagonal}, cardinal={cardinal}");
        }
    }
}

#[test]
fn polygon_offset_bits_and_capacity_match_recast() {
    let polygons: Vec<Vec<[f32; 3]>> = vec![
        vec![],
        vec![[-0., 1., 0.]],
        vec![[0., 1., 0.], [1., 2., 2.]],
        vec![[0., 1., 0.], [0., 2., 2.], [2., 3., 2.], [2., 4., 0.]],
        vec![
            [0.1, 1., 0.2],
            [2.9, 2., 0.3],
            [1.5, 3., 0.4],
            [1.4, 4., 3.7],
            [-0.7, 5., 1.9],
        ],
        vec![
            [0., 0., 0.],
            [0., 0., 0.],
            [0.0001, 0., 0.0001],
            [2., 0., 0.],
            [0., 0., 2.],
        ],
    ];
    for vertices in polygons {
        for offset in [-0.7, 0., 0.3] {
            for capacity in 0..=vertices.len() * 2 + 2 {
                let mut input = format!("offset {} {offset} {capacity} ", vertices.len());
                for v in vertices.iter().flatten() {
                    write!(input, "{v} ").unwrap();
                }
                let mut output = vec![[-123.25; 3]; capacity];
                let count = offset_poly(&vertices, offset, &mut output);
                let actual: Vec<i64> = std::iter::once(count as i64)
                    .chain(output.iter().flatten().map(|v| v.to_bits() as i64))
                    .collect();
                let expected: Vec<i64> = serde_json::from_value(oracle(&input)).unwrap();
                assert_eq!(
                    actual, expected,
                    "offset {offset}, capacity {capacity}, vertices {vertices:?}"
                );
            }
        }
    }
    let cases = float_operands::CASE_COUNT * 10;
    let mut input = format!("offset_bits {cases} ");
    let mut actual = Vec::new();
    for case in 0..cases {
        let [x, y, z] = float_operands::operands(case / 10);
        let offset = float_operands::operands(case % 10)[0];
        let vertices = [[x, y, z], [z, x, y], [y, z, x], [1., y, -1.]];
        let capacity = [0, 1, 3, 5, 10][(case + case / 10 + case / 100 + case / 1000) % 5];
        write!(input, "4 {capacity} {} ", offset.to_bits()).unwrap();
        for value in vertices.into_iter().flatten() {
            write!(input, "{} ", value.to_bits()).unwrap();
        }
        let mut output = vec![[-123.25; 3]; capacity];
        let count = offset_poly(&vertices, offset, &mut output);
        actual.push(
            std::iter::once(count as i64)
                .chain(output.iter().flatten().map(|v| i64::from(v.to_bits())))
                .collect::<Vec<_>>(),
        );
    }
    let expected: Vec<Vec<i64>> = serde_json::from_value(oracle(&input)).unwrap();
    let trace_actual = serde_json::to_value(&actual).unwrap();
    let trace_expected = serde_json::to_value(&expected).unwrap();
    parity_trace::write(
        "polygon-offset",
        &input,
        trace_actual.as_array().unwrap(),
        trace_expected.as_array().unwrap(),
    );
    assert_eq!(actual.len(), expected.len());
    for (case, (actual, expected)) in actual.iter().zip(expected).enumerate() {
        float_comparison::assert_eq(
            &serde_json::json!(actual),
            &serde_json::json!(expected),
            |path| matches!(path, [j] if *j > 0 && (*j-1)%3 != 1),
            &format!("mixed offset case {case}"),
        );
    }
}

fn poly_snapshot(mesh: &recast_nav::PolygonNavmesh) -> Vec<i64> {
    let nvp = mesh.max_vertices_per_polygon as usize;
    let count = if nvp == 0 { 0 } else { mesh.polygon_count() };
    let mut values = vec![
        nvp as i64,
        mesh.border_size as i64,
        mesh.max_edge_error.to_bits() as i64,
        mesh.vertices.len() as i64,
        count as i64,
    ];
    values.extend(
        mesh.aabb
            .min
            .to_array()
            .into_iter()
            .chain(mesh.aabb.max.to_array())
            .chain([mesh.cell_size, mesh.cell_height])
            .map(|v| v.to_bits() as i64),
    );
    values.extend(
        mesh.vertices
            .iter()
            .flat_map(|v| v.to_array())
            .map(i64::from),
    );
    if nvp > 0 {
        for (p, n) in mesh
            .polygons
            .chunks(nvp)
            .zip(mesh.polygon_neighbors.chunks(nvp))
        {
            values.extend(p.iter().chain(n).map(|&v| i64::from(v)));
        }
    }
    for i in 0..count {
        values.extend([
            mesh.regions[i].bits() as i64,
            mesh.areas[i].0 as i64,
            mesh.flags[i] as i64,
        ]);
    }
    values.push(mesh.max_polygons as i64);
    if nvp > 0 {
        for (p, n) in mesh
            .unused_polygons
            .chunks(nvp)
            .zip(mesh.unused_polygon_neighbors.chunks(nvp))
        {
            values.extend(p.iter().chain(n).map(|&v| i64::from(v)));
        }
    }
    for (region, area) in mesh.unused_regions.iter().zip(&mesh.unused_areas) {
        values.extend([i64::from(region.bits()), i64::from(area.0)]);
    }
    values
}

fn tile_mesh(nvp: u32, x: u16, z: u16, y: u16) -> recast_nav::PolygonNavmesh {
    use glam::U16Vec3;
    let mut mesh = recast_nav::PolygonNavmesh {
        vertices: vec![
            U16Vec3::new(0, y, 0),
            U16Vec3::new(0, y, 2),
            U16Vec3::new(2, y, 2),
            U16Vec3::new(2, y, 0),
        ],
        max_vertices_per_polygon: nvp,
        cell_size: 0.3,
        cell_height: 0.2,
        border_size: 3,
        max_edge_error: 1.3,
        aabb: recast_nav::Aabb3d {
            min: [-1.1 + x as f32 * 0.6, -0.7, -0.9 + z as f32 * 0.6].into(),
            max: [-0.5 + x as f32 * 0.6, 5.7, -0.3 + z as f32 * 0.6].into(),
        },
        ..Default::default()
    };
    if nvp == 3 {
        mesh.polygons = vec![0, 1, 2, 0, 2, 3];
        mesh.polygon_neighbors = vec![0x8000, 0x8001, 1, 0, 0x8002, 0x8003];
    } else {
        mesh.polygons = vec![u16::MAX; nvp as usize];
        mesh.polygons[..4].copy_from_slice(&[0, 1, 2, 3]);
        mesh.polygon_neighbors = vec![u16::MAX; nvp as usize];
        mesh.polygon_neighbors[..4].copy_from_slice(&[0x8000, 0x8001, 0x8002, 0x8003]);
    }
    let count = mesh.polygon_count();
    mesh.regions = (0..count)
        .map(|i| RegionId::from(11 + x + z + y + i as u16))
        .collect();
    mesh.areas = vec![AreaType(7 + x as u8); count];
    mesh.flags = vec![0x1234 + y; count];
    mesh
}

fn poly_merge_input(sources: &[&recast_nav::PolygonNavmesh]) -> String {
    let mut input = format!("{}\n", sources.len());
    for m in sources {
        writeln!(
            input,
            "{} {} {} {} {}",
            m.cell_size.to_bits(),
            m.cell_height.to_bits(),
            m.max_vertices_per_polygon,
            m.border_size,
            m.max_edge_error.to_bits()
        )
        .unwrap();
        for v in m
            .aabb
            .min
            .to_array()
            .into_iter()
            .chain(m.aabb.max.to_array())
        {
            write!(input, "{} ", v.to_bits()).unwrap();
        }
        writeln!(input, "{} {}", m.vertices.len(), m.polygon_count()).unwrap();
        for vertex in &m.vertices {
            for v in vertex.to_array() {
                write!(input, "{v} ").unwrap();
            }
        }
        for (poly, neighbors) in m.polygons.chunks(m.max_vertices_per_polygon as usize).zip(
            m.polygon_neighbors
                .chunks(m.max_vertices_per_polygon as usize),
        ) {
            for v in poly.iter().chain(neighbors) {
                write!(input, "{v} ").unwrap();
            }
        }
        for v in m
            .regions
            .iter()
            .map(|r| r.bits())
            .chain(m.areas.iter().map(|a| a.0 as u16))
            .chain(m.flags.iter().copied())
        {
            write!(input, "{v} ").unwrap();
        }
    }
    input
}

#[test]
fn mesh_copy_welding_portals_and_adjacency_match_recast() {
    for nvp in [3, 4, 6] {
        let meshes: Vec<_> = [
            (0, 0, 0),
            (1, 0, 2),
            (0, 1, 3),
            (1, 1, 1),
            (0, 0, 4),
            (0, 0, 2),
        ]
        .into_iter()
        .map(|(x, z, y)| tile_mesh(nvp, x, z, y))
        .collect();
        for reverse in [false, true] {
            let mut sources: Vec<_> = meshes.iter().collect();
            if reverse {
                sources.reverse();
            }
            let input = format!("merge_poly_bits {}", poly_merge_input(&sources));
            let mut actual: Vec<_> = sources
                .iter()
                .map(|m| poly_snapshot(&recast_nav::recast::copy_poly_mesh(m)))
                .collect();
            actual.push(poly_snapshot(&merge_poly_meshes(&sources)));
            let expected: Vec<Vec<i64>> = serde_json::from_value(oracle(&input)).unwrap();
            assert_eq!(actual, expected, "nvp {nvp}, reverse {reverse}");
        }
    }
    let mut input = format!("merge_poly_bits_batch {} ", float_operands::CASE_COUNT);
    let mut actual = Vec::new();
    for case in 0..float_operands::CASE_COUNT {
        let [x, y, z] = float_operands::operands(case);
        let mut meshes = [
            tile_mesh(6, 0, 0, 0),
            tile_mesh(6, 2, 0, 0),
            tile_mesh(6, 0, 2, 0),
        ];
        for (i, mesh) in meshes.iter_mut().enumerate() {
            mesh.aabb.min.y = [x, y, z][i];
            mesh.aabb.max.y = [z, x, y][i];
        }
        let sources: Vec<_> = meshes.iter().collect();
        input.push_str(&poly_merge_input(&sources));
        let mut snapshots: Vec<_> = sources
            .iter()
            .map(|m| poly_snapshot(&copy_poly_mesh(m)))
            .collect();
        snapshots.push(poly_snapshot(&merge_poly_meshes(&sources)));
        actual.push(snapshots);
    }
    let expected: Vec<Vec<Vec<i64>>> = serde_json::from_value(oracle(&input)).unwrap();
    assert_eq!(actual.len(), expected.len());
    for (case, (actual, expected)) in actual.iter().zip(expected).enumerate() {
        assert_eq!(actual, &expected, "merge Y bounds case {case}");
    }
    let expected: Vec<Vec<i64>> = serde_json::from_value(oracle("merge_poly 0\n")).unwrap();
    assert_eq!(vec![poly_snapshot(&merge_poly_meshes(&[]))], expected);
}

fn detail_snapshot(mesh: &recast_nav::DetailNavmesh) -> Vec<i64> {
    let mut values = vec![
        mesh.meshes.len() as i64,
        mesh.vertices.len() as i64,
        mesh.triangles.len() as i64,
    ];
    for m in &mesh.meshes {
        values.extend(
            [
                m.base_vertex_index,
                m.vertex_count,
                m.base_triangle_index,
                m.triangle_count,
            ]
            .map(i64::from),
        );
    }
    values.extend(
        mesh.vertices
            .iter()
            .flat_map(|v| v.to_array())
            .map(|v| v.to_bits() as i64),
    );
    for (t, f) in mesh.triangles.iter().zip(&mesh.triangle_flags) {
        values.extend([t[0], t[1], t[2], *f].map(i64::from));
    }
    values
}

#[test]
fn detail_mesh_merge_preserves_bits_local_indices_and_offsets() {
    use recast_nav::SubMesh;
    let a = recast_nav::DetailNavmesh {
        meshes: vec![
            SubMesh {
                base_vertex_index: 0,
                vertex_count: 4,
                base_triangle_index: 0,
                triangle_count: 2,
            },
            SubMesh {
                base_vertex_index: 4,
                vertex_count: 3,
                base_triangle_index: 2,
                triangle_count: 1,
            },
        ],
        vertices: vec![
            [0., -0., 0.],
            [0., 0.1, 1.],
            [1., 0.2, 1.],
            [1., 0.3, 0.],
            [-1., 2., 0.],
            [-1., 2., 1.],
            [0., 2., 0.],
        ]
        .into_iter()
        .map(Into::into)
        .collect(),
        triangles: vec![[0, 1, 2], [0, 2, 3], [0, 1, 2]],
        triangle_flags: vec![17, 42, 63],
    };
    let b = recast_nav::DetailNavmesh {
        meshes: vec![SubMesh {
            base_vertex_index: 0,
            vertex_count: 3,
            base_triangle_index: 0,
            triangle_count: 1,
        }],
        vertices: vec![[3., 0., 1.], [4., 0.9, 2.], [5., 2.3, 1.]]
            .into_iter()
            .map(Into::into)
            .collect(),
        triangles: vec![[0, 1, 2]],
        triangle_flags: vec![21],
    };
    for sources in [
        vec![None, Some(&a), None, Some(&b), None],
        vec![Some(&b), Some(&a), Some(&b)],
    ] {
        let mut input = format!("merge_detail {}\n", sources.len());
        for source in &sources {
            let Some(m) = source else {
                writeln!(input, "0").unwrap();
                continue;
            };
            writeln!(
                input,
                "1 {} {} {}",
                m.meshes.len(),
                m.vertices.len(),
                m.triangles.len()
            )
            .unwrap();
            for sub in &m.meshes {
                for v in [
                    sub.base_vertex_index,
                    sub.vertex_count,
                    sub.base_triangle_index,
                    sub.triangle_count,
                ] {
                    write!(input, "{v} ").unwrap();
                }
            }
            for v in m.vertices.iter().flat_map(|v| v.to_array()) {
                write!(input, "{v} ").unwrap();
            }
            for (t, f) in m.triangles.iter().zip(&m.triangle_flags) {
                for v in [t[0], t[1], t[2], *f] {
                    write!(input, "{v} ").unwrap();
                }
            }
        }
        let expected: Vec<i64> = serde_json::from_value(oracle(&input)).unwrap();
        assert_eq!(detail_snapshot(&merge_detail_meshes(&sources)), expected);
    }
}

fn region_field(seed: u32) -> CompactHeightfield {
    let mut hf = field(seed);
    // At most eight sweeps per row, within upstream's nine-slot buffer.
    // Two floors, area boundaries, holes and shifting run boundaries.
    for z in 0..hf.height {
        for x in 0..hf.width {
            for (floor, i) in hf.cell_at(x, z).index_range().enumerate() {
                let cut = 2 + (z + seed as i32) % 3;
                hf.areas[i] = AreaType(if floor == 0 && x == cut + 1 {
                    0
                } else if x < cut || x >= 7 {
                    1
                } else {
                    2
                });
            }
        }
    }
    hf
}

#[test]
fn monotone_and_layer_region_ordering_match_recast() {
    for seed in [1, 19] {
        for (border, min_area, merge_area) in [
            (0, 0, 0),
            (0, 100, 20),
            (1, 4, 20),
            (3, 100, 20),
            (0, i32::MIN, i32::MIN),
            (0, -1, 20),
            (1, 4, -1),
            (0, i32::MAX, i32::MAX),
            (0, 65536, 0),
            (0, 0, 65536),
            (10, 0, 0),
        ] {
            for mode in ["watershed", "monotone", "layer"] {
                let mut hf = region_field(seed);
                let mut input = field_input(&hf);
                writeln!(input, "1\n{mode} {border} {min_area} {merge_area}").unwrap();
                let mut actual = vec![snapshot(&hf)];
                if mode == "watershed" {
                    hf.build_distance_field();
                    hf.build_regions(border, min_area, merge_area).unwrap();
                } else if mode == "monotone" {
                    hf.build_regions_monotone(border, min_area, merge_area)
                        .unwrap();
                } else {
                    hf.build_layer_regions(border, min_area).unwrap();
                }
                actual.push(snapshot(&hf));
                let expected: Vec<Vec<i64>> = serde_json::from_value(oracle(&input)).unwrap();
                if actual != expected {
                    let directory = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                        .join("../../target/parity-failures");
                    std::fs::create_dir_all(&directory).unwrap();
                    std::fs::write(directory.join("regions-input.txt"), &input).unwrap();
                    std::fs::write(
                        directory.join("regions-rust.json"),
                        serde_json::to_vec(&actual).unwrap(),
                    )
                    .unwrap();
                    std::fs::write(
                        directory.join("regions-cpp.json"),
                        serde_json::to_vec(&expected).unwrap(),
                    )
                    .unwrap();
                    let differences = actual[1]
                        .iter()
                        .zip(&expected[1])
                        .enumerate()
                        .filter(|(_, (a, b))| a != b)
                        .take(10)
                        .collect::<Vec<_>>();
                    panic!(
                        "seed {seed}, {mode}, border {border}, min {min_area}, merge {merge_area}; first differences {differences:?}"
                    );
                }
            }
        }
    }

    // Exercise the complete byte-sized compact-column domain. The sweep
    // buffer needs one spare slot beyond the 255 disconnected floors.
    let mut hf = CompactHeightfield {
        width: 256,
        height: 1,
        cell_size: 1.,
        cell_height: 1.,
        aabb: recast_nav::Aabb3d {
            min: [0.; 3].into(),
            max: [256., 512., 1.].into(),
        },
        cells: vec![CompactCell::default(); 256],
        spans: (0..255)
            .map(|i| CompactSpan {
                y: i * 2,
                data: 0x01ff_ffff,
                ..Default::default()
            })
            .collect(),
        areas: vec![AreaType(63); 255],
        ..Default::default()
    };
    hf.cells[0].set_count(255);
    let mut input = field_input(&hf);
    writeln!(input, "1\nlayer 0 0 0").unwrap();
    let initial = snapshot(&hf);
    hf.build_layer_regions(0, 0).unwrap();
    let expected: Vec<Vec<i64>> = serde_json::from_value(oracle(&input)).unwrap();
    assert_eq!(vec![initial, snapshot(&hf)], expected, "255 stacked floors");
}

#[test]
fn native_sweep_capacity_is_reported_safely() {
    for layer in [false, true] {
        let mut hf = field(1);
        for (i, area) in hf.areas.iter_mut().enumerate() {
            area.0 = 1 + (i / 2 % 2) as u8;
        }
        let result = if layer {
            hf.build_layer_regions(0, 0)
        } else {
            hf.build_regions_monotone(0, 0, 0)
        };
        assert!(matches!(
            result,
            Err(BuildRegionsError::SweepCapacityExceeded)
        ));
    }
}

#[test]
fn triangle_marking_and_clearing_match_at_slope_boundaries() {
    use glam::Vec3;
    let mut vertices = vec![
        [0., 0., 0.],
        [0., 0., 0.],
        [1., 0., 1.],
        [0., 0., 0.],
        [0., 0., 1.],
        [1., 0., 0.],
    ];
    // Minimized classification regression at 33.23423 degrees.
    vertices.extend([
        [-1.954155, 1.1070938, 6.076432],
        [-6.3227806, -5.283146, -4.2008066],
        [-7.653673, 1.5148783, 6.0908804],
    ]);
    let mut random = 0x123abc_u32;
    for _ in 0..64 * 3 {
        let v = std::array::from_fn(|_| {
            random = random.wrapping_mul(1664525).wrapping_add(1013904223);
            (random >> 8) as f32 / 16777216. * 16. - 8.
        });
        vertices.push(v);
    }
    let triangles: Vec<_> = (0..vertices.len() / 3)
        .map(|i| [i as u32 * 3, i as u32 * 3 + 1, i as u32 * 3 + 2])
        .collect();
    let initial: Vec<_> = (0..triangles.len())
        .map(|i| AreaType(((i * 13 + 7) % 256) as u8))
        .collect();
    let mut angles = vec![
        0., 1.2345, 33.23423, 35.26439, 44.9, 45., 45.1, 60., 89.99, 89.99999,
    ];
    for t in &triangles[2..18] {
        let [a, b, c] = t.map(|i| Vec3::from(vertices[i as usize]));
        let normal = (b - a).cross(c - a).normalize();
        let degrees = normal.y.acos().to_degrees();
        if (0. ..90.).contains(&degrees) {
            angles.extend([degrees.next_down(), degrees, degrees.next_up()]);
        }
    }
    for angle in angles {
        let mut input = format!(
            "triangle_areas {} {} {angle}\n",
            vertices.len(),
            triangles.len()
        );
        for v in vertices.iter().flatten() {
            write!(input, "{v} ").unwrap();
        }
        for i in triangles.iter().flatten() {
            write!(input, "{i} ").unwrap();
        }
        for a in &initial {
            write!(input, "{} ", a.0).unwrap();
        }
        let mut marked = initial.clone();
        let mut cleared = initial.clone();
        mark_walkable_triangles(angle, &vertices, &triangles, &mut marked);
        clear_unwalkable_triangles(angle, &vertices, &triangles, &mut cleared);
        let actual: Vec<Vec<u8>> = [marked, cleared]
            .into_iter()
            .map(|a| a.into_iter().map(|a| a.0).collect())
            .collect();
        let expected: Vec<Vec<u8>> = serde_json::from_value(oracle(&input)).unwrap();
        for (stage, (a, e)) in actual.iter().zip(&expected).enumerate() {
            let difference = a.iter().zip(e).position(|(a, b)| a != b);
            assert!(
                a == e,
                "slope {angle}, stage {stage}, triangle {difference:?}, vertices {:?}",
                difference.map(|i| triangles[i].map(|j| vertices[j as usize]))
            );
        }
    }
}

#[test]
fn bounds_bits_and_grid_rounding_match_recast() {
    for vertices in [
        vec![[-0., 0., -0.], [0., -0., 0.]],
        vec![[-1.1, 2.9, -3.7], [2.3, -4.1, 1.9], [0., 0., 0.]],
    ] {
        for cell in [0.3, 0.5, 1.7] {
            let mut input = format!("bounds {} {cell}\n", vertices.len());
            for v in vertices.iter().flatten() {
                write!(input, "{v} ").unwrap();
            }
            let bounds = calc_bounds(&vertices).unwrap();
            let grid = calc_grid_size(bounds, cell);
            let actual: Vec<i64> = bounds
                .iter()
                .flatten()
                .map(|v| v.to_bits() as i64)
                .chain(grid.map(i64::from))
                .collect();
            let expected: Vec<i64> = serde_json::from_value(oracle(&input)).unwrap();
            assert_eq!(actual, expected);
        }
    }
}

fn solid_field() -> Heightfield {
    HeightfieldBuilder {
        aabb: recast_nav::Aabb3d {
            min: [-1.1, -0.7, -0.9].into(),
            max: [1.9, 5.3, 1.5].into(),
        },
        cell_size: 0.6,
        cell_height: 0.2,
    }
    .build()
    .unwrap()
}
fn solid_input(hf: &Heightfield, operations: usize) -> String {
    let mut input = format!("solid {} {} ", hf.width, hf.height);
    for v in hf
        .aabb
        .min
        .to_array()
        .into_iter()
        .chain(hf.aabb.max.to_array())
    {
        write!(input, "{v} ").unwrap();
    }
    writeln!(input, "{} {} {operations}", hf.cell_size, hf.cell_height).unwrap();
    input
}
fn solid_snapshot(hf: &Heightfield) -> Vec<i64> {
    let mut values = vec![
        heightfield_span_count(hf) as i64,
        hf.width as i64,
        hf.height as i64,
    ];
    values.extend(
        hf.aabb
            .min
            .to_array()
            .into_iter()
            .chain(hf.aabb.max.to_array())
            .chain([hf.cell_size, hf.cell_height])
            .map(|v| v.to_bits() as i64),
    );
    // Native observes width*height columns, independent of backing capacity.
    for &head in hf.spans.iter().take((hf.width * hf.height) as usize) {
        let mut column = Vec::new();
        let mut cursor = head;
        while let Some(key) = cursor {
            let s = hf.span(key);
            column.extend([s.min() as i64, s.max() as i64, s.area().0 as i64]);
            cursor = s.next;
        }
        values.push((column.len() / 3) as i64);
        values.extend(column);
    }
    values
}

#[test]
fn span_insertion_order_packed_fields_and_flag_thresholds_match_recast() {
    let mut hf = solid_field();
    let mut operations: Vec<(i32, i32, u16, u16, u8, i32)> = vec![
        (0, 0, 5, 8, 63, 1),
        (0, 0, 1, 3, 4, 1),
        (0, 0, 2, 6, 7, 1),
        (0, 0, 8, 8, 1, -1),
        (1, 0, 8193, 8195, 255, 1),
        (1, 0, 1, 3, 1, -1),
        (2, 0, 65535, 16384, 191, 100000),
    ];
    let mut random = 91u32;
    for i in 0..96 {
        random = random.wrapping_mul(1664525).wrapping_add(1013904223);
        let x = (random % hf.width as u32) as i32;
        let z = ((random >> 8) % hf.height as u32) as i32;
        let min = ((random >> 16) % 24) as u16;
        let max = min + ((random >> 24) % 7) as u16;
        operations.push((
            x,
            z,
            min,
            max,
            (random >> 24) as u8,
            [-1, 0, 1, 8191, 100000][i % 5],
        ));
    }
    let mut input = solid_input(&hf, operations.len());
    let mut actual = vec![solid_snapshot(&hf)];
    for &(x, z, min, max, area, threshold) in &operations {
        writeln!(input, "add {x} {z} {min} {max} {area} {threshold}").unwrap();
        if threshold == 100000 {
            let mut span = Span::new(0, 0, AreaType(0));
            span.set_min(min);
            span.set_max(max);
            span.set_area(AreaType(area));
            hf.add_span(SpanInsertion {
                x,
                z,
                span,
                flag_merge_threshold: threshold,
            })
            .unwrap();
        } else {
            add_span(&mut hf, x, z, min, max, AreaType(area), threshold).unwrap();
        }
        actual.push(solid_snapshot(&hf));
    }
    let expected: Vec<Vec<i64>> = serde_json::from_value(oracle(&input)).unwrap();
    assert_eq!(actual.len(), expected.len());
    for (step, (a, e)) in actual.iter().zip(&expected).enumerate() {
        let first = a.iter().zip(e).position(|(a, e)| a != e);
        assert!(
            a == e,
            "span step {step}, operation {:?}, field {first:?}",
            step.checked_sub(1).map(|i| operations[i])
        );
    }
}

fn raster_operation(
    input: &mut String,
    mode: &str,
    vertices: &[[f32; 3]],
    triangles: &[[u32; 3]],
    areas: &[AreaType],
    threshold: i32,
) {
    writeln!(
        input,
        "{mode} {} {} {threshold}",
        vertices.len(),
        triangles.len()
    )
    .unwrap();
    for v in vertices.iter().flatten() {
        write!(input, "{v} ").unwrap();
    }
    for i in triangles.iter().flatten() {
        write!(input, "{i} ").unwrap();
    }
    for a in areas {
        write!(input, "{} ", a.0).unwrap();
    }
}

#[test]
fn rasterization_overloads_preserve_native_span_order() {
    let mut vertices = vec![
        [0., 0., 0.],
        [0., 0., 0.],
        [0., 0., 0.],
        [-2., 0., -2.],
        [-2., 0., 3.],
        [3., 0., 3.],
        [-2., 0., -2.],
        [3., 0., 3.],
        [3., 0., -2.],
    ];
    // Exact clipping planes, including a triangle entirely on the X plane.
    // Rotate the starting vertex to exercise the closing edge in each position.
    let row_plane = f32::from_bits(0xbe99_9998); // -0.9f + 0.6f
    for triangle in [
        [
            [-0.5, 0.21, -1.5],
            [-1.7, 1.13, row_plane],
            [1.3, 3.07, 1.5],
        ],
        [[-0.5, -0.0, -1.5], [-0.5, 2.7, row_plane], [-0.5, 4.9, 1.5]],
    ] {
        for first in 0..3 {
            vertices.extend((0..3).map(|i| triangle[(first + i) % 3]));
        }
    }
    let mut random = 713u32;
    for _ in 0..24 * 3 {
        let v = std::array::from_fn(|_| {
            random = random.wrapping_mul(1664525).wrapping_add(1013904223);
            (random >> 8) as f32 / 16777216. * 5. - 2.
        });
        vertices.push(v);
    }
    let triangles: Vec<_> = (0..vertices.len() / 3)
        .map(|i| [i as u32 * 3, i as u32 * 3 + 1, i as u32 * 3 + 2])
        .collect();
    let areas: Vec<_> = (0..triangles.len())
        .map(|i| AreaType((i * 37 % 256) as u8))
        .collect();
    for threshold in [-1, 0, 3] {
        for mode in ["u32", "u16", "flat", "single"] {
            let mut hf = solid_field();
            let operations = if mode == "single" { triangles.len() } else { 1 };
            let mut input = solid_input(&hf, operations);
            let mut actual = vec![solid_snapshot(&hf)];
            match mode {
                "single" => {
                    for (&t, &area) in triangles.iter().zip(&areas) {
                        raster_operation(&mut input, mode, &vertices, &[t], &[area], threshold);
                        rasterize_triangle(
                            &mut hf,
                            t.map(|i| vertices[i as usize]),
                            area,
                            threshold,
                        )
                        .unwrap();
                        actual.push(solid_snapshot(&hf));
                    }
                }
                "u32" => {
                    rasterize_triangles_u32(&mut hf, &vertices, &triangles, &areas, threshold)
                        .unwrap();
                }
                "u16" => {
                    let t: Vec<_> = triangles.iter().map(|t| t.map(|i| i as u16)).collect();
                    rasterize_triangles_u16(&mut hf, &vertices, &t, &areas, threshold).unwrap();
                }
                "flat" => {
                    let t: Vec<_> = triangles
                        .iter()
                        .map(|t| t.map(|i| vertices[i as usize]))
                        .collect();
                    rasterize_triangles(&mut hf, &t, &areas, threshold).unwrap();
                }
                _ => unreachable!(),
            }
            if mode != "single" {
                raster_operation(&mut input, mode, &vertices, &triangles, &areas, threshold);
                actual.push(solid_snapshot(&hf));
            }
            let expected: Vec<Vec<i64>> = serde_json::from_value(oracle(&input)).unwrap();
            assert_eq!(actual.len(), expected.len());
            for (step, (a, e)) in actual.iter().zip(&expected).enumerate() {
                let first = a.iter().zip(e).position(|(a, e)| a != e);
                assert!(
                    a == e,
                    "{mode}, threshold {threshold}, step {step}, field {first:?}"
                );
            }
        }
    }
}

#[test]
fn polygon_mesh_vertex_capacity_matches_native_boundary() {
    use glam::IVec3;
    for (total, retained) in [
        (65533usize, false),
        (65534, false),
        (65535, false),
        (65534, true),
    ] {
        let first = match total % 3 {
            1 => 4,
            2 => 5,
            _ => 3,
        };
        let mut contours = ContourSet {
            width: 1024,
            height: 512,
            cell_size: 1.,
            cell_height: 1.,
            aabb: recast_nav::Aabb3d {
                min: [0.; 3].into(),
                max: [1024., 8., 512.].into(),
            },
            ..Default::default()
        };
        let points = [[0, 0], [0, 2], [1, 3], [2, 2], [2, 0]];
        for i in 0..(total - first) / 3 + 1 {
            let count = if i == 0 { first } else { 3 };
            contours.contours.push(Contour {
                region: RegionId::from(i as u16 + 1),
                area: AreaType(63),
                vertices: points[..count]
                    .iter()
                    .map(|&[x, z]| {
                        (
                            IVec3::new((i % 256 * 4) as i32 + x, 0, (i / 256 * 4) as i32 + z),
                            0,
                        )
                    })
                    .collect(),
                ..Default::default()
            });
        }
        let mode = if retained {
            "poly_capacity_retained_context"
        } else {
            "poly_capacity_context"
        };
        let expected = oracle(&format!("{mode} {total}\n"));
        let hooks = ContextRecorder::default();
        let context = Context::with_hooks(true, &hooks);
        let mut mesh = if retained {
            recast_nav::PolygonNavmesh {
                vertices: [[11, 13, 17], [19, 23, 29], [31, 37, 41]]
                    .map(glam::U16Vec3::from_array)
                    .to_vec(),
                polygons: vec![0, 1, 2],
                polygon_neighbors: vec![65535; 3],
                regions: vec![RegionId::from(31)],
                areas: vec![AreaType(7)],
                flags: vec![0xcafe],
                max_vertices_per_polygon: 3,
                max_polygons: 2,
                unused_polygons: vec![2, 1, 0],
                unused_polygon_neighbors: vec![3, 5, 7],
                unused_regions: vec![RegionId::from(51)],
                unused_areas: vec![AreaType(19)],
                ..Default::default()
            }
        } else {
            recast_nav::PolygonNavmesh::default()
        };
        let actual = match contours.build_polygon_mesh_into(&context, 3, &mut mesh) {
            Ok(()) => {
                let mut values = vec![1, mesh.vertices.len() as i64, mesh.polygon_count() as i64];
                values.extend(poly_snapshot(&mesh));
                values.extend(poly_snapshot(&recast_nav::recast::copy_poly_mesh(&mesh)));
                values
            }
            Err(error) => {
                assert!(
                    matches!(error,PolygonNavmeshError::TooManyVertices{actual,..} if actual==total)
                );
                let mut values = vec![0, mesh.vertices.len() as i64, mesh.polygon_count() as i64];
                values.extend(poly_snapshot(&mesh));
                values
            }
        };
        assert!(
            serde_json::json!([actual, *hooks.0.borrow()]) == expected,
            "contour vertices {total}: native output/context differ"
        );
    }
}

fn contour_snapshot(set: &ContourSet) -> Vec<i64> {
    let mut values = vec![
        set.width as i64,
        set.height as i64,
        set.border_size as i64,
        set.max_error.to_bits() as i64,
        set.contours.len() as i64,
    ];
    values.extend(
        set.aabb
            .min
            .to_array()
            .into_iter()
            .chain(set.aabb.max.to_array())
            .chain([set.cell_size, set.cell_height])
            .map(|v| v.to_bits() as i64),
    );
    for c in &set.contours {
        values.extend([
            c.region.bits() as i64,
            c.area.0 as i64,
            c.vertices.len() as i64,
            c.raw_vertices.len() as i64,
        ]);
        for (v, f) in &c.vertices {
            values.extend([v.x as i64, v.y as i64, v.z as i64, *f as i64]);
        }
        for (v, f) in &c.raw_vertices {
            values.extend([v.x as i64, v.y as i64, v.z as i64, f.bits() as i64]);
        }
    }
    values
}

#[test]
fn contour_options_preserve_raw_and_simplified_area_boundaries() {
    for border in [0, 1] {
        for bits in 0..=3 {
            for edge in [i32::MIN, -1, 0, 1, 2, 32768, 46340, i32::MAX] {
                if edge > 46340 && bits != 0 {
                    continue;
                }
                let mut hf = region_field(1);
                let mut input = field_input(&hf);
                writeln!(
                    input,
                    "2\nmonotone {border} 0 0\ncontours 1.3 {edge} {bits}"
                )
                .unwrap();
                let mut actual = vec![snapshot(&hf)];
                hf.build_regions_monotone(border, 0, 0).unwrap();
                actual.push(snapshot(&hf));
                let contours = hf
                    .build_contours(1.3, edge, BuildContoursFlags::from_bits_retain(bits))
                    .unwrap();
                actual.push(contour_snapshot(&contours));
                actual.push(snapshot(&hf));
                let expected: Vec<Vec<i64>> = serde_json::from_value(oracle(&input)).unwrap();
                assert_eq!(
                    actual, expected,
                    "contour flags {bits}, border {border}, edge {edge}"
                );
            }
        }
    }
    let mut hf = region_field(1);
    hf.build_regions_monotone(0, 0, 0).unwrap();
    assert!(matches!(
        hf.build_contours(
            1.3,
            i32::MAX,
            BuildContoursFlags::TESSELLATE_SOLID_WALL_EDGES
        ),
        Err(ContourError::EdgeLengthOverflow)
    ));
}

#[test]
fn native_polygon_border_vertex_zero_removal_and_partial_triangulation() {
    use glam::IVec3;
    let shapes = [
        vec![
            [1, 0, 0, 0x10000],
            [0, 0, 0, 0],
            [0, 0, 2, 0],
            [2, 0, 2, 0],
            [2, 0, 0, 0],
        ],
        // Warning recovery retains one triangle from this five-vertex input.
        vec![
            [5, 0, 4, 0],
            [1, 0, 4, 0],
            [6, 0, 1, 0],
            [6, 0, 7, 0],
            [2, 0, 5, 0],
        ],
        // The minimal four-vertex warning case retains vertices and no triangles.
        vec![[1, 0, 4, 0], [6, 0, 1, 0], [6, 0, 7, 0], [2, 0, 5, 0]],
    ];
    for (index, shape) in shapes.into_iter().enumerate() {
        for nvp in [3, 6] {
            let cset = ContourSet {
                width: 8,
                height: 8,
                cell_size: 0.3,
                cell_height: 0.2,
                max_error: 1.3,
                aabb: recast_nav::Aabb3d {
                    min: [0.; 3].into(),
                    max: [8.; 3].into(),
                },
                contours: vec![Contour {
                    region: RegionId::from(7),
                    area: AreaType(3),
                    vertices: shape
                        .iter()
                        .map(|v| (IVec3::new(v[0] as i32, v[1] as i32, v[2] as i32), v[3]))
                        .collect(),
                    ..Contour::default()
                }],
                ..ContourSet::default()
            };
            let mut input = format!(
                "poly_contours_context 0.3 0.2 {nvp} 8 8 0 1.3 0 0 0 8 8 8 1 {} 7 3 ",
                shape.len()
            );
            for v in shape.iter().flatten() {
                write!(input, "{v} ").unwrap();
            }
            let expected = oracle(&input);
            let hooks = ContextRecorder::default();
            let context = Context::with_hooks(true, &hooks);
            let actual = match cset.into_polygon_mesh_with_context(&context, nvp) {
                Ok(mesh) => {
                    assert_eq!(mesh.regions.len(), mesh.polygon_count());
                    if index == 0 && nvp == 6 {
                        assert_eq!(mesh.vertices.len(), 4);
                    }
                    let mut values =
                        vec![1, mesh.vertices.len() as i64, mesh.polygon_count() as i64];
                    values.extend(poly_snapshot(&mesh));
                    values.extend(poly_snapshot(&recast_nav::recast::copy_poly_mesh(&mesh)));
                    values
                }
                Err(_) => vec![0, 0, 0],
            };
            assert_eq!(
                serde_json::json!([actual, *hooks.0.borrow()]),
                expected,
                "contour case={index}, nvp={nvp}, shape={shape:?}"
            );
        }
    }
}

#[test]
fn native_erosion_signed_radius_byte_threshold_wrap_and_saturation() {
    for radius in [
        -1073741824,
        -65537,
        -32768,
        -129,
        -128,
        -1,
        0,
        1,
        2,
        3,
        126,
        127,
        128,
        129,
        255,
        256,
        32768,
        65535,
        1073741823,
    ] {
        for seed in [1, 213] {
            let mut hf = field(seed);
            let mut input = field_input(&hf);
            write!(input, "1 erode {radius}").unwrap();
            let before = snapshot(&hf);
            hf.erode_walkable_area(radius).unwrap();
            assert_eq!(
                serde_json::json!([before, snapshot(&hf)]),
                oracle(&input),
                "radius={radius} seed={seed}"
            );
        }
    }
    // Distances saturate at 255; a wide uninterrupted field retains its center
    // for the native 254 threshold, while radius 128 wraps the threshold to 0.
    for radius in [127, 128, 129, -1] {
        let mut hf = CompactHeightfield {
            width: 260,
            height: 260,
            ..Default::default()
        };
        for z in 0..hf.height {
            for x in 0..hf.width {
                let mut cell = CompactCell::default();
                cell.set_index(hf.spans.len() as u32);
                cell.set_count(1);
                hf.cells.push(cell);
                let mut span = CompactSpan::default();
                span.set_height(10);
                for (d, valid) in [x > 0, z + 1 < hf.height, x + 1 < hf.width, z > 0]
                    .into_iter()
                    .enumerate()
                {
                    span.set_con(d as u8, valid.then_some(0));
                }
                hf.spans.push(span);
                hf.areas.push(AreaType(63));
            }
        }
        let mut input = field_input(&hf);
        write!(input, "1 erode {radius}").unwrap();
        let before = snapshot(&hf);
        hf.erode_walkable_area(radius).unwrap();
        assert_eq!(
            serde_json::json!([before, snapshot(&hf)]),
            oracle(&input),
            "saturated radius={radius}"
        );
    }
    let mut hf = field(1);
    let before = snapshot(&hf);
    assert_eq!(
        hf.erode_walkable_area(i32::MAX),
        Err(ErosionError::RadiusOverflow)
    );
    assert_eq!(snapshot(&hf), before);
    assert_eq!(
        hf.erode_walkable_area(i32::MIN),
        Err(ErosionError::RadiusOverflow)
    );
    assert_eq!(snapshot(&hf), before);
}

fn compact_snapshot(hf: &CompactHeightfield) -> serde_json::Value {
    let mut metadata = vec![
        i64::from(hf.width),
        i64::from(hf.height),
        i64::from(hf.walkable_height),
        i64::from(hf.walkable_climb),
    ];
    metadata.extend(
        hf.aabb
            .min
            .to_array()
            .into_iter()
            .chain(hf.aabb.max.to_array())
            .chain([hf.cell_size, hf.cell_height])
            .map(|v| i64::from(v.to_bits())),
    );
    serde_json::json!([metadata, snapshot(hf)])
}
#[test]
fn native_solid_filters_signed_height_climb_exact_headroom_and_compact_metadata() {
    for height in [i32::MIN, -65536, -1, 0, 1, 3, 4, 255, 8192, 65536, i32::MAX] {
        for climb in [-65536, -1, 0, 1, 4, 8192, 65536] {
            let mut hf = solid_field();
            let mut additions = vec![];
            for z in 0..hf.height {
                for x in 0..hf.width {
                    let base = (x + z) % 3;
                    additions.push((x, z, base, base, 1));
                    additions.push((
                        x,
                        z,
                        base + 3,
                        base + 4,
                        if (x + z) % 2 == 0 { 0 } else { 2 },
                    ));
                    additions.push((x, z, base + 8, base + 9, 3));
                }
            }
            let mut input = solid_input(&hf, additions.len() + 4);
            let mut actual = vec![serde_json::json!(solid_snapshot(&hf))];
            for (x, z, min, max, area) in additions {
                writeln!(input, "add {x} {z} {min} {max} {area} -1").unwrap();
                add_span(&mut hf, x, z, min as u16, max as u16, AreaType(area), -1).unwrap();
                actual.push(serde_json::json!(solid_snapshot(&hf)));
            }
            for mode in ["low_hanging", "ledge", "low_height"] {
                writeln!(input, "{mode} {height} {climb}").unwrap();
                match mode {
                    "low_hanging" => hf.filter_low_hanging_walkable_obstacles(climb),
                    "ledge" => hf.filter_ledge_spans(height, climb).unwrap(),
                    _ => hf.filter_walkable_low_height_spans(height),
                }
                actual.push(serde_json::json!(solid_snapshot(&hf)));
            }
            writeln!(input, "compact {height} {climb}").unwrap();
            actual.push(compact_snapshot(
                &hf.clone().into_compact(height, climb).unwrap(),
            ));
            actual.push(serde_json::json!(solid_snapshot(&hf)));
            let expected = oracle(&input);
            assert_eq!(
                serde_json::json!(actual),
                expected,
                "height={height} climb={climb}"
            );
        }
    }
    let mut empty = solid_field();
    assert!(empty.filter_ledge_spans(0, i32::MIN).is_ok());
    add_span(&mut empty, 0, 0, 0, 1, AreaType(63), 0).unwrap();
    assert_eq!(
        empty.filter_ledge_spans(0, i32::MIN),
        Err(FilterError::HeightOverflow)
    );
}
#[test]
fn native_ledge_sparse_stacks_and_directional_boundaries() {
    for (width, depth) in [(1, 1), (1, 7), (7, 1), (7, 6)] {
        for (height, climb) in [(-1, -1), (0, 0), (3, 2), (8, 4)] {
            let mut hf = HeightfieldBuilder {
                aabb: recast_nav::Aabb3d {
                    min: [0.; 3].into(),
                    max: [width as f32, 100., depth as f32].into(),
                },
                cell_size: 1.,
                cell_height: 1.,
            }
            .build()
            .unwrap();
            let mut additions = vec![];
            for z in 0..depth {
                for x in 0..width {
                    if (x + 3 * z) % 5 == 2 {
                        continue;
                    }
                    for layer in 0..8 {
                        let floor = layer * 10 + (x + 2 * z) % 4;
                        let area = if (x + z + layer) % 3 == 0 { 0 } else { 63 };
                        additions.push((x, z, floor, area));
                    }
                }
            }
            let mut input = solid_input(&hf, additions.len() + 2);
            let mut actual = vec![serde_json::json!(solid_snapshot(&hf))];
            for (x, z, floor, area) in additions {
                writeln!(input, "add {x} {z} {floor} {} {area} -1", floor + 1).unwrap();
                add_span(
                    &mut hf,
                    x,
                    z,
                    floor as u16,
                    (floor + 1) as u16,
                    AreaType(area),
                    -1,
                )
                .unwrap();
                actual.push(serde_json::json!(solid_snapshot(&hf)));
            }
            // A second pass observes flags cleared by the first pass while
            // column geometry and span links remain unchanged.
            for _ in 0..2 {
                writeln!(input, "ledge {height} {climb}").unwrap();
                hf.filter_ledge_spans(height, climb).unwrap();
                actual.push(serde_json::json!(solid_snapshot(&hf)));
            }
            assert_eq!(
                serde_json::json!(actual),
                oracle(&input),
                "width={width} depth={depth} height={height} climb={climb}"
            );
        }
    }
}

#[test]
fn native_compact_neighbor_connection_accepts_last_valid_layer_62() {
    let mut hf = HeightfieldBuilder {
        aabb: recast_nav::Aabb3d {
            min: [0.; 3].into(),
            max: [2., 300., 1.].into(),
        },
        cell_size: 1.,
        cell_height: 1.,
    }
    .build()
    .unwrap();
    let mut input = solid_input(&hf, 65);
    let mut actual = vec![serde_json::json!(solid_snapshot(&hf))];
    for (x, min, max) in
        std::iter::once((0, 247, 248)).chain((0..63).map(|i| (1, i * 4, i * 4 + 1)))
    {
        writeln!(input, "add {x} 0 {min} {max} 63 -1").unwrap();
        add_span(&mut hf, x, 0, min, max, AreaType(63), -1).unwrap();
        actual.push(serde_json::json!(solid_snapshot(&hf)));
    }
    // Match the floor at neighbor index 62 (249); lower floors cannot connect.
    writeln!(input, "compact 2 1").unwrap();
    let compact = hf.clone().into_compact(2, 1).unwrap();
    assert_eq!(compact.spans[0].con(2), Some(62));
    actual.push(compact_snapshot(&compact));
    actual.push(serde_json::json!(solid_snapshot(&hf)));
    assert_eq!(serde_json::json!(actual), oracle(&input));
}

fn layer_snapshot(layers: &[recast_nav::layers::HeightfieldLayer]) -> Vec<i64> {
    let mut result = vec![layers.len() as i64];
    for l in layers {
        result.extend([
            l.width as i64,
            l.height as i64,
            l.min_x as i64,
            l.max_x as i64,
            l.min_z as i64,
            l.max_z as i64,
            i64::from(l.hmin),
            i64::from(l.hmax),
        ]);
        result.extend(
            l.bounds
                .into_iter()
                .flatten()
                .chain([l.cell_size, l.cell_height])
                .map(|v| i64::from(v.to_bits())),
        );
        result.extend(
            l.heights
                .iter()
                .chain(&l.areas)
                .chain(&l.connections)
                .map(|&v| i64::from(v)),
        );
    }
    result
}
fn height_islands() -> CompactHeightfield {
    let mut hf = CompactHeightfield {
        width: 3,
        height: 1,
        cell_size: 1.,
        cell_height: 1.,
        aabb: recast_nav::Aabb3d {
            min: [0.; 3].into(),
            max: [3., 60., 1.].into(),
        },
        cells: vec![CompactCell::default(); 3],
        spans: vec![
            CompactSpan {
                y: 0,
                data: u32::MAX,
                ..Default::default()
            },
            CompactSpan {
                y: 50,
                data: u32::MAX,
                ..Default::default()
            },
        ],
        areas: vec![AreaType(63); 2],
        ..Default::default()
    };
    hf.cells[0].set_count(1);
    hf.cells[2].set_index(1);
    hf.cells[2].set_count(1);
    hf
}
#[test]
fn native_layer_merge_threshold_wraps_before_promoted_range_comparisons() {
    let heights = [
        (i32::MIN, 2),
        (-16384, 2),
        (-1, 1),
        (0, 2),
        (1, 2),
        (12, 2),
        (13, 1),
        (16384, 2),
        (16385, 2),
        (16397, 1),
        (65536, 2),
        (536870000, 1),
        (i32::MAX, 1),
    ];
    for (islands, hf) in [(false, field(213)), (true, height_islands())] {
        for (height, expected_count) in heights {
            let mut input = field_input(&hf);
            write!(input, "1 layers {height}").unwrap();
            let layers = recast_nav::layers::build_heightfield_layers(&hf, 0, height).unwrap();
            if islands {
                assert_eq!(layers.len(), expected_count, "height={height}");
            }
            assert_eq!(
                serde_json::json!([snapshot(&hf), layer_snapshot(&layers), snapshot(&hf)]),
                oracle(&input),
                "height={height} islands={islands}"
            );
        }
    }
}

#[test]
fn native_rasterization_at_and_above_packed_span_ceiling() {
    for y in [8190., 8191., 8191.5, 8192., 9999.] {
        for mode in ["u32", "u16", "flat", "single"] {
            let mut hf = HeightfieldBuilder {
                aabb: recast_nav::Aabb3d {
                    min: [0.; 3].into(),
                    max: [2., 10000., 2.].into(),
                },
                cell_size: 1.,
                cell_height: 1.,
            }
            .build()
            .unwrap();
            let vertices = [[0., y, 0.], [0., y, 2.], [2., y, 2.]];
            let triangles = [[0, 1, 2]];
            let areas = [AreaType(63)];
            let mut input = solid_input(&hf, 1);
            let before = solid_snapshot(&hf);
            raster_operation(&mut input, mode, &vertices, &triangles, &areas, 0);
            rasterize_triangles_u32(&mut hf, &vertices, &triangles, &areas, 0).unwrap();
            assert_eq!(
                serde_json::json!([before, solid_snapshot(&hf)]),
                oracle(&input),
                "y={y} mode={mode}"
            );
        }
    }
    let mut hf = HeightfieldBuilder {
        aabb: recast_nav::Aabb3d {
            min: [0.; 3].into(),
            max: [2., 1e13, 2.].into(),
        },
        cell_size: 1.,
        cell_height: 1.,
    }
    .build()
    .unwrap();
    assert!(matches!(
        rasterize_triangle(
            &mut hf,
            [[0., 1e12, 0.], [0., 1e12, 2.], [2., 1e12, 2.]],
            AreaType(63),
            0
        ),
        Err(RasterizationError::IntegerConversion)
    ));
}

#[test]
fn native_grid_dimensions_and_span_coordinates_above_65535() {
    for (width, height) in [(65537, 1), (1, 65537)] {
        let mut hf = HeightfieldBuilder {
            aabb: recast_nav::Aabb3d {
                min: [0.; 3].into(),
                max: [width as f32, 4., height as f32].into(),
            },
            cell_size: 1.,
            cell_height: 1.,
        }
        .build()
        .unwrap();
        assert_eq!((hf.width, hf.height), (width, height));
        let mut input = solid_input(&hf, 4);
        let mut actual = vec![solid_snapshot(&hf)];
        for (x, z) in [(0, 0), (width - 1, height - 1), (width - 1, height - 1)] {
            writeln!(input, "add {x} {z} 1 2 63 0").unwrap();
            add_span(&mut hf, x, z, 1, 2, AreaType(63), 0).unwrap();
            actual.push(solid_snapshot(&hf));
        }
        writeln!(input, "low_height 3 0").unwrap();
        hf.filter_walkable_low_height_spans(3);
        actual.push(solid_snapshot(&hf));
        assert_eq!(
            serde_json::json!(actual),
            oracle(&input),
            "grid {width} x {height}"
        );
    }
    let mut hf = solid_field();
    assert!(add_span(&mut hf, -1, 0, 1, 2, AreaType(63), 0).is_err());
    // Native flat indexing permits a row crossing that stays in the allocation.
    let mut input = solid_input(&hf, 1);
    writeln!(input, "add -1 1 1 2 63 0").unwrap();
    let before = solid_snapshot(&hf);
    add_span(&mut hf, -1, 1, 1, 2, AreaType(63), 0).unwrap();
    assert_eq!(
        serde_json::json!([before, solid_snapshot(&hf)]),
        oracle(&input)
    );
    for maximum in [[0., 4., 2.], [2., 4., 0.], [-1.5, 4., -2.5]] {
        let hf = HeightfieldBuilder {
            aabb: recast_nav::Aabb3d {
                min: [0.; 3].into(),
                max: maximum.into(),
            },
            cell_size: 1.,
            cell_height: 1.,
        }
        .build()
        .unwrap();
        assert_eq!(
            serde_json::json!([solid_snapshot(&hf)]),
            oracle(&solid_input(&hf, 0))
        );
    }
    assert!(
        HeightfieldBuilder {
            aabb: recast_nav::Aabb3d {
                min: [0.; 3].into(),
                max: [f32::INFINITY, 1., 1.].into()
            },
            cell_size: 1.,
            cell_height: 1.
        }
        .build()
        .is_err()
    );
}
#[test]
fn native_signed_contour_coordinates_are_triangulated_before_u16_welding() {
    use glam::IVec3;
    for offset in [i32::MIN + 4, -65536, -1, 65535, 65536, 131072, i32::MAX - 4] {
        for nvp in [3, 6] {
            // Crossing the ushort boundary makes later merge scores overflow in
            // native code. Triangles still exercise its defined triangulation.
            if nvp == 6 && [-1, 65535].contains(&offset) {
                continue;
            }
            let points = [
                [offset, 65536, 0],
                [offset, 65536, 2],
                [offset + 2, 65536, 2],
                [offset + 2, 65536, 0],
            ];
            let cset = ContourSet {
                width: 100000,
                height: 8,
                cell_size: 0.3,
                cell_height: 0.2,
                aabb: recast_nav::Aabb3d {
                    min: [0.; 3].into(),
                    max: [8.; 3].into(),
                },
                contours: vec![Contour {
                    region: RegionId::from(7),
                    area: AreaType(3),
                    vertices: points.iter().map(|p| (IVec3::from_array(*p), 0)).collect(),
                    ..Default::default()
                }],
                ..Default::default()
            };
            let mut input =
                format!("poly_contours 0.3 0.2 {nvp} 100000 8 0 0 0 0 0 8 8 8 1 4 7 3 ");
            for p in points {
                for v in p {
                    write!(input, "{} ", v as u32).unwrap();
                }
                write!(input, "0 ").unwrap();
            }
            let mesh = cset.into_polygon_mesh(nvp).unwrap();
            let mut actual = vec![1, mesh.vertices.len() as i64, mesh.polygon_count() as i64];
            actual.extend(poly_snapshot(&mesh));
            actual.extend(poly_snapshot(&recast_nav::recast::copy_poly_mesh(&mesh)));
            assert_eq!(
                serde_json::json!(actual),
                oracle(&input),
                "offset={offset} nvp={nvp}"
            );
        }
    }
}

#[test]
fn native_maximum_compact_distance_promotes_additions_and_wraps_watershed_level() {
    let mut hf = CompactHeightfield {
        width: 3,
        height: 3,
        cell_size: 1.,
        cell_height: 1.,
        ..Default::default()
    };
    hf.cells = vec![CompactCell::default(); 9];
    hf.cells[4].set_count(1);
    let mut span = CompactSpan::default();
    span.set_height(10);
    for d in 0..4 {
        span.set_con(d, Some(0));
    }
    hf.spans.push(span);
    hf.areas.push(AreaType(63));
    // Neighbor cell indices alias the allocated center span. Every native read
    // stays in allocated cell/span storage, including its diagonal traversals.
    let mut input = field_input(&hf);
    write!(input, "2 distance watershed 0 -1 -1").unwrap();
    let mut actual = vec![snapshot(&hf)];
    hf.build_distance_field();
    assert_eq!(hf.max_distance, u16::MAX);
    assert_eq!(hf.dist, [u16::MAX]);
    actual.push(
        std::iter::once(i64::from(hf.max_distance))
            .chain(hf.dist.iter().copied().map(i64::from))
            .collect(),
    );
    actual.push(snapshot(&hf));
    hf.build_regions(0, -1, -1).unwrap();
    actual.push(snapshot(&hf));
    assert_eq!(serde_json::json!(actual), oracle(&input));
}

#[test]
fn native_contour_walking_retains_the_exact_39999_iteration_cutoff() {
    let mut hf = CompactHeightfield {
        width: 10003,
        height: 2,
        cell_size: 1.,
        cell_height: 1.,
        aabb: recast_nav::Aabb3d {
            min: [0.; 3].into(),
            max: [10003., 4., 2.].into(),
        },
        ..Default::default()
    };
    for z in 0..hf.height {
        for x in 0..hf.width {
            let mut cell = CompactCell::default();
            cell.set_index(hf.spans.len() as u32);
            cell.set_count(1);
            hf.cells.push(cell);
            let mut span = CompactSpan {
                region: RegionId::from(1),
                ..Default::default()
            };
            span.set_height(10);
            for (d, valid) in [x > 0, z + 1 < hf.height, x + 1 < hf.width, z > 0]
                .into_iter()
                .enumerate()
            {
                span.set_con(d as u8, valid.then_some(0));
            }
            hf.spans.push(span);
            hf.areas.push(AreaType(63));
        }
    }
    let mut input = field_input(&hf);
    write!(input, "1 contours 1.3 0 0").unwrap();
    let set = hf
        .build_contours(1.3, 0, BuildContoursFlags::empty())
        .unwrap();
    assert!(set.contours[0].raw_vertices.len() > 10000);
    assert!(
        set.contours[0].raw_vertices.len() < ((hf.width + hf.height) * 2) as usize,
        "fixture must reach the native traversal cutoff before closing the contour"
    );
    let actual = [snapshot(&hf), contour_snapshot(&set), snapshot(&hf)];
    let expected: Vec<Vec<i64>> = serde_json::from_value(oracle(&input)).unwrap();
    for (stage, (a, b)) in actual.iter().zip(&expected).enumerate() {
        let first = a.iter().zip(b).position(|(a, b)| a != b);
        assert!(
            a == b,
            "walk limit stage={stage}, first={first:?}, sizes={},{}",
            a.len(),
            b.len()
        );
    }
    assert_eq!(actual.len(), expected.len());
}

#[test]
fn native_compact_bitfields_connections_and_direction_helpers() {
    let mut cases = Vec::new();
    for value in [
        0,
        1,
        63,
        255,
        256,
        65535,
        0x00ff_ffff,
        0x0100_0000,
        u32::MAX,
    ] {
        for direction in [0, 1, 2, 3, 4, i32::MIN, i32::MIN + 4] {
            for (x, z) in [(0, -1), (-1, 0), (0, 0), (1, 0), (0, 1), (6, i32::MAX - 1)] {
                cases.push((value, direction, x, z));
            }
        }
    }
    let mut input = format!("packed {}\n", cases.len());
    let mut actual = Vec::new();
    for (value, direction, x, z) in cases {
        let connections = value.rotate_left(7);
        let height = value.rotate_right(5);
        let neighbor = value as i32;
        writeln!(
            input,
            "{value} {value} {connections} {height} {direction} {neighbor} {x} {z}"
        )
        .unwrap();
        let mut cell = CompactCell::default();
        // Native assignments commute: writing the index must retain the count.
        cell.set_count_raw(value);
        cell.set_index(value);
        let mut result = vec![i64::from(cell.index()), i64::from(cell.count())];
        cell.inc_count();
        result.push(i64::from(cell.count()));
        let mut span = CompactSpan::default();
        span.set_connections_raw(connections);
        span.set_height(height as u8);
        result.extend([
            i64::from(span.connections_raw()),
            i64::from(span.height()),
            i64::from(span.con_raw(direction).unwrap()),
        ]);
        assert!(span.set_con_raw(direction, neighbor));
        result.extend([
            i64::from(span.connections_raw()),
            i64::from(span.height()),
            i64::from(span.con_raw(direction).unwrap()),
            i64::from(direction_offset_x(direction)),
            i64::from(direction_offset_z(direction)),
            i64::from(direction_for_offset(x, z).unwrap()),
        ]);
        actual.push(result);
    }
    assert_eq!(serde_json::json!(actual), oracle(&input));
    let mut span = CompactSpan {
        data: u32::MAX,
        ..Default::default()
    };
    assert_eq!(span.con_raw(5), Some(0));
    for direction in [5, 6, -1, i32::MAX] {
        assert!(!span.set_con_raw(direction, 1));
        assert_eq!(span.data, u32::MAX);
    }
    assert_eq!(span.con_raw(6), None);
    for (x, z) in [(0, -2), (0, i32::MAX), (3, 1), (i32::MAX, 1)] {
        assert_eq!(direction_for_offset(x, z), None);
    }
}

#[test]
fn native_compact_column_counts_wrap_at_eight_bits() {
    for count in [255, 256, 257] {
        let mut hf = HeightfieldBuilder {
            aabb: recast_nav::Aabb3d {
                min: [0.; 3].into(),
                max: [1., 2048., 1.].into(),
            },
            cell_size: 1.,
            cell_height: 1.,
        }
        .build()
        .unwrap();
        let mut input = solid_input(&hf, count + 1);
        let mut actual = vec![serde_json::json!(solid_snapshot(&hf))];
        for i in 0..count {
            let min = (i * 4) as u16;
            writeln!(input, "add 0 0 {min} {} 63 -1", min + 1).unwrap();
            add_span(&mut hf, 0, 0, min, min + 1, AreaType(63), -1).unwrap();
            actual.push(serde_json::json!(solid_snapshot(&hf)));
        }
        writeln!(input, "compact 2 1").unwrap();
        let compact = hf.clone().into_compact(2, 1).unwrap();
        assert_eq!(compact.cells[0].count(), count as u8);
        assert_eq!(compact.spans.len(), count);
        actual.push(compact_snapshot(&compact));
        actual.push(serde_json::json!(solid_snapshot(&hf)));
        assert_eq!(serde_json::json!(actual), oracle(&input), "count={count}");
    }
}

#[test]
fn native_recast_common_vector_and_scalar_helpers() {
    let mut input = format!("common {}\n", 256 + float_operands::CASE_COUNT);
    let mut actual = Vec::new();
    let mut seed = 0xa536_429bu32;
    for i in 0..256 + float_operands::CASE_COUNT {
        let mut next = || {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            (seed as i32 as f32) / 12345.
        };
        let mut a = [next(), next(), next()];
        let mut b = [next(), next(), next()];
        let mut s = next();
        if i >= 256 {
            let [x, y, z] = float_operands::operands(i - 256);
            a = [x, y, z];
            b = [y, z, x];
            s = z;
        }
        match i {
            0 => {
                a = [0.; 3];
                b = a;
                s = -0.;
            }
            1 => {
                a = [-0., 0., -0.];
                b = [0., -0., 0.];
            }
            2 => {
                a = [f32::INFINITY, 1., -2.];
                s = f32::INFINITY;
            }
            3 => a = [1., f32::NAN, 2.],
            4 => {
                a = [f32::from_bits(0xffc0_1234), -1., 3.];
                s = f32::from_bits(0x7fc0_5678);
            }
            5 => {
                a = [f32::MIN_POSITIVE, f32::from_bits(1), -f32::from_bits(1)];
                s = -1.;
            }
            _ => {}
        }
        for value in a.into_iter().chain(b).chain([s]) {
            write!(input, "{} ", value.to_bits()).unwrap();
        }
        let mut result = Vec::new();
        for vector in [
            vector_cross(a, b),
            vector_multiply_add(a, b, s),
            vector_add(a, b),
            vector_subtract(a, b),
            vector_minimum(a, b),
            vector_maximum(a, b),
            vector_normalize(a),
        ] {
            result.extend(vector.map(|v| i64::from(v.to_bits())));
        }
        result.extend(
            [
                vector_dot(a, b),
                vector_distance(a, b),
                vector_distance_squared(a, b),
                minimum(a[0], b[0]),
                maximum(a[0], b[0]),
                absolute(a[0]),
                square(s),
                clamp(s, a[0], b[0]),
                square_root(s),
            ]
            .map(|v| i64::from(v.to_bits())),
        );
        actual.push(result);
    }
    let expected = oracle(&input);
    let trace_actual = serde_json::to_value(&actual).unwrap();
    parity_trace::write(
        "recast-math",
        &input,
        trace_actual.as_array().unwrap(),
        expected.as_array().unwrap(),
    );
    for (i, (a, b)) in actual.iter().zip(expected.as_array().unwrap()).enumerate() {
        float_comparison::assert_eq(
            &serde_json::json!(a),
            b,
            |path| matches!(path, [j] if *j < 12 || (18..24).contains(j) || *j == 27 || *j == 29),
            &format!("case={i}"),
        );
    }
    assert_eq!(actual.len(), expected.as_array().unwrap().len());
}

#[derive(Default)]
struct ContextRecorder(std::cell::RefCell<Vec<Vec<i64>>>);
impl ContextHooks for ContextRecorder {
    fn reset_log(&self) {
        self.0.borrow_mut().push(vec![0]);
    }
    fn log(&self, category: LogCategory, message: &[u8]) {
        let mut event = vec![1, i64::from(category.0)];
        event.extend(message.iter().map(|&b| i64::from(b)));
        self.0.borrow_mut().push(event);
    }
    fn reset_timers(&self) {
        self.0.borrow_mut().push(vec![2]);
    }
    fn start_timer(&self, label: TimerLabel) {
        self.0.borrow_mut().push(vec![3, i64::from(label.0)]);
    }
    fn stop_timer(&self, label: TimerLabel) {
        self.0.borrow_mut().push(vec![4, i64::from(label.0)]);
    }
    fn accumulated_time(&self, label: TimerLabel) -> i32 {
        self.0.borrow_mut().push(vec![5, i64::from(label.0)]);
        label.0 * 17 + 9
    }
}
#[test]
fn native_context_logging_timer_gates_scopes_and_byte_truncation() {
    for initial in [false, true] {
        let hooks = ContextRecorder::default();
        let context = Context::with_hooks(initial, &hooks);
        let base = Context::new(initial);
        let mut commands = Vec::new();
        let mut values = vec![TIMER_COUNT as i64];
        values.extend(
            [
                TimerLabel::TOTAL,
                TimerLabel::TEMP,
                TimerLabel::RASTERIZE_TRIANGLES,
                TimerLabel::BUILD_COMPACTHEIGHTFIELD,
                TimerLabel::BUILD_CONTOURS,
                TimerLabel::BUILD_CONTOURS_TRACE,
                TimerLabel::BUILD_CONTOURS_SIMPLIFY,
                TimerLabel::FILTER_BORDER,
                TimerLabel::FILTER_WALKABLE,
                TimerLabel::MEDIAN_AREA,
                TimerLabel::FILTER_LOW_OBSTACLES,
                TimerLabel::BUILD_POLYMESH,
                TimerLabel::MERGE_POLYMESH,
                TimerLabel::ERODE_AREA,
                TimerLabel::MARK_BOX_AREA,
                TimerLabel::MARK_CYLINDER_AREA,
                TimerLabel::MARK_CONVEXPOLY_AREA,
                TimerLabel::BUILD_DISTANCEFIELD,
                TimerLabel::BUILD_DISTANCEFIELD_DIST,
                TimerLabel::BUILD_DISTANCEFIELD_BLUR,
                TimerLabel::BUILD_REGIONS,
                TimerLabel::BUILD_REGIONS_WATERSHED,
                TimerLabel::BUILD_REGIONS_EXPAND,
                TimerLabel::BUILD_REGIONS_FLOOD,
                TimerLabel::BUILD_REGIONS_FILTER,
                TimerLabel::BUILD_LAYERS,
                TimerLabel::BUILD_POLYMESHDETAIL,
                TimerLabel::MERGE_POLYMESHDETAIL,
            ]
            .map(|label| i64::from(label.0)),
        );
        commands.push("R".to_owned());
        context.reset_log();
        commands.push("T".to_owned());
        context.reset_timers();
        commands.push("G 0".to_owned());
        values.extend([
            i64::from(context.accumulated_time(TimerLabel::TOTAL)),
            i64::from(base.accumulated_time(TimerLabel::TOTAL)),
        ]);
        for enabled in [false, true, false, true] {
            commands.push(format!("E {}", u8::from(enabled)));
            context.enable_log(enabled);
            commands.push("R".to_owned());
            context.reset_log();
            for length in [0, 1, 510, 511, 512, 513, 4096] {
                let message: Vec<_> = (0..length).map(|i| [b'x', 0xc3, 0xa9][i % 3]).collect();
                let mut command = format!("L 2 {length} ");
                for b in &message {
                    write!(command, "{b} ").unwrap();
                }
                commands.push(command);
                context.log_bytes(LogCategory::WARNING, &message);
            }
            commands.push("Z 1".to_owned());
            context.log_bytes(LogCategory::PROGRESS, &[0]);
            let text = "é".repeat(257);
            let mut command = format!("F 3 -17 {} ", text.len());
            for b in text.as_bytes() {
                write!(command, "{b} ").unwrap();
            }
            commands.push(command);
            context
                .log_format(
                    LogCategory::ERROR,
                    format_args!("value={}; text={}", -17, text),
                )
                .unwrap();
            commands.push(format!("e {}", u8::from(enabled)));
            context.enable_timer(enabled);
            base.enable_timer(enabled);
            commands.push("T".to_owned());
            context.reset_timers();
            for label in [0, 1, 27, 28, 31] {
                commands.push(format!("S {label}"));
                context.start_timer(TimerLabel(label));
                commands.push(format!("G {label}"));
                values.extend([
                    i64::from(context.accumulated_time(TimerLabel(label))),
                    i64::from(base.accumulated_time(TimerLabel(label))),
                ]);
                commands.push(format!("P {label}"));
                context.stop_timer(TimerLabel(label));
            }
        }
        for start in [false, true] {
            for end in [false, true] {
                commands.push(format!("C 1 {} {}", u8::from(start), u8::from(end)));
                context.enable_timer(start);
                base.enable_timer(end);
                {
                    let _timer = context.scoped_timer(TimerLabel::TEMP);
                    context.enable_timer(end);
                }
            }
        }
        commands.push("e 1".to_owned());
        context.enable_timer(true);
        commands.push("M".to_owned());
        merge_poly_meshes_with_context(&context, &[]);
        merge_detail_meshes_with_context(&context, &[]);
        let input = format!(
            "context {} {} {}",
            u8::from(initial),
            commands.len(),
            commands.join("\n")
        );
        assert_eq!(
            serde_json::json!([*hooks.0.borrow(), values]),
            oracle(&input),
            "initial={initial}"
        );
    }
}

#[test]
fn native_area_operations_emit_context_hooks_without_changing_field_outputs() {
    for seed in [1, 213] {
        let mut hf = field(seed);
        let hooks = ContextRecorder::default();
        let context = Context::with_hooks(true, &hooks);
        let mut input = field_input(&hf).replacen("areas ", "areas_context ", 1);
        let operations = [
            "median",
            "box -1 -1 -1 1 3 1 7",
            "cylinder 0 -1 0 0.7 6 13",
            "convex 3 -1 6 17 -1 0 -1 1 0 -1 0 0 1",
            "erode 1",
            "timer 0",
            "erode 0",
            "timer 1",
            "box 99 0 99 100 5 100 23",
        ];
        writeln!(input, "{}", operations.len()).unwrap();
        let mut actual = vec![serde_json::json!([snapshot(&hf), *hooks.0.borrow()])];
        for (i, operation) in operations.into_iter().enumerate() {
            writeln!(input, "{operation}").unwrap();
            match i {
                0 => median_filter_walkable_area_with_context(&context, &mut hf),
                1 => mark_box_area_with_context(
                    &context,
                    &mut hf,
                    [-1.; 3],
                    [1., 3., 1.],
                    AreaType(7),
                ),
                2 => mark_cylinder_area_with_context(
                    &context,
                    &mut hf,
                    [0., -1., 0.],
                    0.7,
                    6.,
                    AreaType(13),
                ),
                3 => mark_convex_poly_area_with_context(
                    &context,
                    &mut hf,
                    &[[-1., 0., -1.], [1., 0., -1.], [0., 0., 1.]],
                    -1.,
                    6.,
                    AreaType(17),
                ),
                4 => hf.erode_walkable_area_with_context(&context, 1).unwrap(),
                5 => context.enable_timer(false),
                6 => hf.erode_walkable_area_with_context(&context, 0).unwrap(),
                7 => context.enable_timer(true),
                8 => mark_box_area_with_context(
                    &context,
                    &mut hf,
                    [99., 0., 99.],
                    [100., 5., 100.],
                    AreaType(23),
                ),
                _ => unreachable!(),
            }
            actual.push(serde_json::json!([snapshot(&hf), *hooks.0.borrow()]));
        }
        assert_eq!(serde_json::json!(actual), oracle(&input), "seed={seed}");
    }
}

#[test]
fn native_compact_excess_connection_layers_log_error_and_still_succeed() {
    for layers in [64usize, 65, 200] {
        let mut hf = HeightfieldBuilder {
            aabb: recast_nav::Aabb3d {
                min: [0.; 3].into(),
                max: [2., 2048., 1.].into(),
            },
            cell_size: 1.,
            cell_height: 1.,
        }
        .build()
        .unwrap();
        let mut input = solid_input(&hf, layers + 2);
        let mut actual = vec![serde_json::json!(solid_snapshot(&hf))];
        let floor = (layers as u16 - 1) * 4;
        for (x, min, max) in std::iter::once((0, floor - 1, floor))
            .chain((0..layers as u16).map(|i| (1, i * 4, i * 4 + 1)))
        {
            writeln!(input, "add {x} 0 {min} {max} 63 -1").unwrap();
            add_span(&mut hf, x, 0, min, max, AreaType(63), -1).unwrap();
            actual.push(serde_json::json!(solid_snapshot(&hf)));
        }
        writeln!(input, "compact_context 2 1").unwrap();
        let hooks = ContextRecorder::default();
        let context = Context::with_hooks(true, &hooks);
        let compact = hf
            .clone()
            .into_compact_with_context(&context, 2, 1)
            .unwrap();
        assert_eq!(compact.spans[0].con(2), None);
        let result = compact_snapshot(&compact);
        actual.push(serde_json::json!([
            1,
            result[0],
            result[1],
            *hooks.0.borrow()
        ]));
        actual.push(serde_json::json!(solid_snapshot(&hf)));
        assert_eq!(serde_json::json!(actual), oracle(&input), "layers={layers}");
    }
}

#[test]
fn native_compact_uses_signed_linear_columns_and_only_reachable_spans() {
    for (width, height) in [(-3, -2), (3, 2), (0, 2)] {
        let mut hf = HeightfieldBuilder {
            aabb: recast_nav::Aabb3d {
                min: [0.; 3].into(),
                max: [3., 10., 2.].into(),
            },
            cell_size: 1.,
            cell_height: 1.,
        }
        .build()
        .unwrap();
        hf.width = width;
        hf.height = height;
        hf.allocated_spans.insert(Span::new(5, 6, AreaType(63)));
        let mut input = solid_input(&hf, if width == 0 { 1 } else { 2 });
        let mut actual = vec![serde_json::json!(solid_snapshot(&hf))];
        if width != 0 {
            writeln!(input, "add 0 0 0 1 63 -1").unwrap();
            add_span(&mut hf, 0, 0, 0, 1, AreaType(63), -1).unwrap();
            actual.push(serde_json::json!(solid_snapshot(&hf)));
        }
        writeln!(input, "compact_context 2 1").unwrap();
        let hooks = ContextRecorder::default();
        let context = Context::with_hooks(true, &hooks);
        let compact = hf
            .clone()
            .into_compact_with_context(&context, 2, 1)
            .unwrap();
        assert_eq!(compact.spans.len(), usize::from(width != 0));
        let result = compact_snapshot(&compact);
        actual.push(serde_json::json!([
            1,
            result[0],
            result[1],
            *hooks.0.borrow()
        ]));
        actual.push(serde_json::json!(solid_snapshot(&hf)));
        assert_eq!(
            serde_json::json!(actual),
            oracle(&input),
            "dimensions={width},{height}"
        );
    }
}

#[test]
fn native_distance_field_nested_context_scopes_and_repeated_storage() {
    for seed in [0, 1, 213, 2] {
        let mut hf = field(seed);
        if seed == 0 {
            // Keep one uniform layer and a second with an interior hole, so
            // both minimum-distance and longer diagonal paths are exercised.
            hf.areas.fill(AreaType(63));
            // Alias two interior cells while retaining coverage of every span.
            // Their connections stay in bounds and exercise deferred stores
            // when a cardinal or diagonal neighbor names the current span.
            let removed = (3 * hf.width as usize + 4) * 2;
            hf.spans.drain(removed..removed + 2);
            hf.areas.drain(removed..removed + 2);
            for cell in &mut hf.cells {
                if cell.index() as usize >= removed {
                    cell.set_index(cell.index() - 2);
                }
            }
            hf.areas[(2 * hf.width as usize + 3) * 2] = AreaType(0);
        }
        if seed == 2 {
            hf.areas.fill(AreaType(63));
            // Neighbors can reference a span outside every cell's traversal
            // range. Its distance stays MAX despite missing connections, so
            // the positive-distance invariant applies only to visited spans.
            let cell = &mut hf.cells[3 * hf.width as usize + 4];
            cell.set_count(1);
            let unvisited = cell.index() as usize + 1;
            for dir in 0..4 {
                hf.spans[unvisited].set_con(dir, None);
            }
        }
        let hooks = ContextRecorder::default();
        let context = Context::with_hooks(true, &hooks);
        let mut input = field_input(&hf).replacen("areas ", "areas_context ", 1);
        writeln!(input, "5 distance timer 0 distance timer 1 distance").unwrap();
        let mut actual = vec![serde_json::json!([snapshot(&hf), *hooks.0.borrow()])];
        for operation in 0..5 {
            if operation == 1 || operation == 3 {
                context.enable_timer(operation == 3);
            } else {
                hf.build_distance_field_with_context(&context);
                if seed == 0 {
                    assert!(hf.max_distance > 2 && hf.dist.contains(&2));
                }
                if seed == 2 {
                    assert_eq!(hf.max_distance, u16::MAX);
                    assert_eq!(
                        hf.dist[hf.cells[3 * hf.width as usize + 4].index() as usize + 1],
                        0
                    );
                }
                let values: Vec<_> = std::iter::once(i64::from(hf.max_distance))
                    .chain(hf.dist.iter().map(|&d| i64::from(d)))
                    .collect();
                actual.push(serde_json::json!(values));
            }
            actual.push(serde_json::json!([snapshot(&hf), *hooks.0.borrow()]));
        }
        assert_eq!(serde_json::json!(actual), oracle(&input), "seed={seed}");
    }
}

#[test]
fn native_layer_context_flat_connections_borders_and_region_overflow_diagnostics() {
    // The last and first cells of consecutive rows are flat-index neighbors
    // even though the connection crosses the logical x boundary.
    let mut crossed = field(213);
    crossed.areas.fill(AreaType(63));
    for z in 1..crossed.height as usize {
        let right = z * crossed.width as usize;
        for layer in 0..2 {
            let left_span = crossed.cells[right - 1].index() as usize + layer;
            let right_span = crossed.cells[right].index() as usize + layer;
            crossed.spans[left_span].set_con(2, Some(layer as u8));
            crossed.spans[right_span].set_con(0, Some(layer as u8));
        }
    }
    let layers = recast_nav::layers::build_heightfield_layers(&crossed, 0, 1).unwrap();
    let mut input = field_input(&crossed);
    writeln!(input, "1 layers 1").unwrap();
    assert_eq!(
        serde_json::json!([
            snapshot(&crossed),
            layer_snapshot(&layers),
            snapshot(&crossed)
        ]),
        oracle(&input),
        "native flat row-crossing layer connections"
    );
    let hf = height_islands();
    for border in [0, 1, 2, 3, i32::MAX] {
        let hooks = ContextRecorder::default();
        let context = Context::with_hooks(true, &hooks);
        let layers =
            recast_nav::layers::build_heightfield_layers_with_context(&context, &hf, border, 13)
                .unwrap();
        if border > 0 {
            assert!(layers.is_empty());
        }
        let mut input = field_input(&hf).replacen("areas ", "areas_context ", 1);
        writeln!(input, "1 layers_try {border} 13").unwrap();
        assert_eq!(
            serde_json::json!([
                [snapshot(&hf), []],
                [1, layer_snapshot(&layers)],
                [snapshot(&hf), *hooks.0.borrow()]
            ]),
            oracle(&input),
            "border={border}"
        );
    }
    let mut overflow = CompactHeightfield {
        width: 255,
        height: 3,
        cell_size: 1.,
        cell_height: 1.,
        aabb: recast_nav::Aabb3d {
            min: [0.; 3].into(),
            max: [255., 10., 3.].into(),
        },
        cells: vec![CompactCell::default(); 255 * 3],
        ..Default::default()
    };
    // Keep each native byte-sized sweep counter below its wrapping boundary.
    // Accumulate 256 disconnected regions across two rows instead.
    for z in [0, 2] {
        for x in (0..255).step_by(2) {
            let cell = &mut overflow.cells[x + z * 255];
            cell.set_index(overflow.spans.len() as u32);
            cell.set_count(1);
            overflow.spans.push(CompactSpan {
                data: u32::MAX,
                ..Default::default()
            });
            overflow.areas.push(AreaType(63));
        }
    }
    let hooks = ContextRecorder::default();
    let context = Context::with_hooks(true, &hooks);
    assert!(
        recast_nav::layers::build_heightfield_layers_with_context(&context, &overflow, 0, 1)
            .is_err()
    );
    let mut input = field_input(&overflow).replacen("areas ", "areas_context ", 1);
    writeln!(input, "1 layers_try 0 1").unwrap();
    let expected = oracle(&input);
    assert_eq!(expected[1], serde_json::json!([0, [0]]));
    assert_eq!(expected[2][1], serde_json::json!(*hooks.0.borrow()));
    assert!(
        serde_json::json!([
            [snapshot(&overflow), []],
            [0, [0]],
            [snapshot(&overflow), *hooks.0.borrow()]
        ]) == expected,
        "native layer-overflow field or frame mismatch"
    );
}

#[test]
fn native_rasterization_overloads_and_solid_filters_context_scopes() {
    let vertices = [
        [-0.9, 0., -0.7],
        [1.7, 0., -0.7],
        [1.7, 0., 1.3],
        [-0.9, 0., 1.3],
    ];
    let triangles = [[0u32, 2, 1], [0, 3, 2]];
    let areas = [AreaType(63), AreaType(17)];
    for mode in ["single", "u32", "u16", "flat", "packed", "aligned"] {
        let mut hf = solid_field();
        let hooks = ContextRecorder::default();
        let context = Context::with_hooks(true, &hooks);
        let mut input = solid_input(&hf, 6).replacen("solid ", "solid_context ", 1);
        let mut actual = vec![serde_json::json!([solid_snapshot(&hf), *hooks.0.borrow()])];
        let native_mode = match mode {
            "packed" | "aligned" => "u32",
            _ => mode,
        };
        write!(input, "{native_mode} 4 2 0 ").unwrap();
        for value in vertices.into_iter().flatten() {
            write!(input, "{value} ").unwrap();
        }
        for index in triangles.into_iter().flatten() {
            write!(input, "{index} ").unwrap();
        }
        writeln!(input, "63 17").unwrap();
        match mode {
            "packed" => hf
                .rasterize_indexed_triangles_with_context(
                    &context, &vertices, &triangles, &areas, 0,
                )
                .unwrap(),
            "aligned" => hf
                .rasterize_indexed_triangles_with_context(
                    &context,
                    &vertices.map(glam::Vec3A::from_array),
                    &triangles,
                    &areas,
                    0,
                )
                .unwrap(),
            "single" => {
                for (t, area) in triangles.into_iter().zip(areas) {
                    rasterize_triangle_with_context(
                        &context,
                        &mut hf,
                        t.map(|i| vertices[i as usize]),
                        area,
                        0,
                    )
                    .unwrap();
                }
            }
            "u32" => rasterize_triangles_u32_with_context(
                &context, &mut hf, &vertices, &triangles, &areas, 0,
            )
            .unwrap(),
            "u16" => rasterize_triangles_u16_with_context(
                &context,
                &mut hf,
                &vertices,
                &triangles.map(|t| t.map(|i| i as u16)),
                &areas,
                0,
            )
            .unwrap(),
            _ => rasterize_triangles_with_context(
                &context,
                &mut hf,
                &triangles.map(|t| t.map(|i| vertices[i as usize])),
                &areas,
                0,
            )
            .unwrap(),
        }
        actual.push(serde_json::json!([solid_snapshot(&hf), *hooks.0.borrow()]));
        writeln!(input, "low_hanging 2 1").unwrap();
        hf.filter_low_hanging_walkable_obstacles_with_context(&context, 1);
        actual.push(serde_json::json!([solid_snapshot(&hf), *hooks.0.borrow()]));
        writeln!(input, "ledge 2 1").unwrap();
        hf.filter_ledge_spans_with_context(&context, 2, 1).unwrap();
        actual.push(serde_json::json!([solid_snapshot(&hf), *hooks.0.borrow()]));
        writeln!(input, "low_height 2 1").unwrap();
        hf.filter_walkable_low_height_spans_with_context(&context, 2);
        actual.push(serde_json::json!([solid_snapshot(&hf), *hooks.0.borrow()]));
        writeln!(input, "timer 0").unwrap();
        context.enable_timer(false);
        actual.push(serde_json::json!([solid_snapshot(&hf), *hooks.0.borrow()]));
        writeln!(input, "low_height 2 1").unwrap();
        hf.filter_walkable_low_height_spans_with_context(&context, 2);
        actual.push(serde_json::json!([solid_snapshot(&hf), *hooks.0.borrow()]));
        assert_eq!(serde_json::json!(actual), oracle(&input), "mode={mode}");
    }
}

#[test]
fn native_layer_neighbor_queue_capacities_and_high_region_ids() {
    // A root region sees 16 or 17 branches. The last branch is high enough to
    // resist later merging: only the first 16 fit the native neighbor list.
    for branches in [16, 17] {
        let width = branches * 2 - 1;
        let mut hf = CompactHeightfield {
            width: width as i32,
            height: 2,
            cell_size: 1.,
            cell_height: 1.,
            aabb: recast_nav::Aabb3d {
                min: [0.; 3].into(),
                max: [width as f32, 255., 2.].into(),
            },
            cells: vec![CompactCell::default(); width * 2],
            ..Default::default()
        };
        for z in 0..2 {
            for x in 0..width {
                if z == 1 && x % 2 == 1 {
                    continue;
                }
                let cell = &mut hf.cells[x + z * width];
                cell.set_index(hf.spans.len() as u32);
                cell.set_count(1);
                hf.spans.push(CompactSpan {
                    y: if z == 1 && x == width - 1 { 200 } else { 0 },
                    data: u32::MAX,
                    ..Default::default()
                });
                hf.areas.push(AreaType(63));
            }
        }
        for z in 0..2 {
            for x in 0..width {
                for i in hf.cells[x + z * width].index_range() {
                    for dir in 0..4 {
                        let nx = x as i32 + direction_offset_x(dir);
                        let nz = z as i32 + direction_offset_z(dir);
                        if (0..width as i32).contains(&nx)
                            && (0..2).contains(&nz)
                            && hf.cells[nx as usize + nz as usize * width].count() != 0
                        {
                            hf.spans[i].set_con(dir as u8, Some(0));
                        }
                    }
                }
            }
        }
        let hooks = ContextRecorder::default();
        let context = Context::with_hooks(true, &hooks);
        let layers =
            recast_nav::layers::build_heightfield_layers_with_context(&context, &hf, 0, 1).unwrap();
        assert_eq!(layers.len(), if branches == 16 { 1 } else { 2 });
        let mut input = field_input(&hf).replacen("areas ", "areas_context ", 1);
        writeln!(input, "1 layers_try 0 1").unwrap();
        assert_eq!(
            serde_json::json!([
                [snapshot(&hf), []],
                [1, layer_snapshot(&layers)],
                [snapshot(&hf), *hooks.0.borrow()]
            ]),
            oracle(&input),
            "native neighbor cap with {branches} branches"
        );
    }
    // One continuous root row branches into twelve regions, each of which
    // branches into twelve more. The resulting 157 IDs include values > 127.
    // Nodes rejected by the native 64-entry FIFO are high enough to resist
    // later layer merging. An unbounded queue would produce one layer, not two.
    let width = 12 * 36;
    let mut high = [false; 144];
    let mut queued = 12;
    for group in high.chunks_mut(12) {
        queued -= 1;
        for high in group {
            if queued == 64 {
                *high = true;
            } else {
                queued += 1;
            }
        }
    }
    assert_eq!(high.iter().filter(|&&value| !value).count(), 64);
    let mut hf = CompactHeightfield {
        width: width as i32,
        height: 3,
        cell_size: 1.,
        cell_height: 1.,
        aabb: recast_nav::Aabb3d {
            min: [0.; 3].into(),
            max: [width as f32, 255., 3.].into(),
        },
        cells: vec![CompactCell::default(); width * 3],
        ..Default::default()
    };
    for z in 0..3 {
        for x in 0..width {
            if (z == 1 && x % 36 == 35) || (z == 2 && x % 3 == 2) {
                continue;
            }
            let cell = &mut hf.cells[x + z * width];
            cell.set_index(hf.spans.len() as u32);
            cell.set_count(1);
            let y = if z == 2 && high[x / 3] { 200 } else { 0 };
            hf.spans.push(CompactSpan {
                y,
                data: u32::MAX,
                ..Default::default()
            });
            hf.areas.push(AreaType(63));
        }
    }
    for z in 0..3 {
        for x in 0..width {
            for i in hf.cells[x + z * width].index_range() {
                for dir in 0..4 {
                    let nx = x as i32 + direction_offset_x(dir);
                    let nz = z as i32 + direction_offset_z(dir);
                    if (0..width as i32).contains(&nx)
                        && (0..3).contains(&nz)
                        && hf.cells[nx as usize + nz as usize * width].count() != 0
                    {
                        hf.spans[i].set_con(dir as u8, Some(0));
                    }
                }
            }
        }
    }
    let hooks = ContextRecorder::default();
    let context = Context::with_hooks(true, &hooks);
    let layers =
        recast_nav::layers::build_heightfield_layers_with_context(&context, &hf, 0, 1).unwrap();
    assert_eq!(layers.len(), 2);
    assert_eq!(
        layers.iter().map(|layer| layer.hmin).collect::<Vec<_>>(),
        [0, 200]
    );
    let mut input = field_input(&hf).replacen("areas ", "areas_context ", 1);
    writeln!(input, "1 layers_try 0 1").unwrap();
    assert_eq!(
        serde_json::json!([
            [snapshot(&hf), []],
            [1, layer_snapshot(&layers)],
            [snapshot(&hf), *hooks.0.borrow()]
        ]),
        oracle(&input),
        "native bounded FIFO, byte region IDs, layers and context"
    );

    // Exercise every live byte ID, with an unmapped empty column beside ID 254.
    let mut ceiling = CompactHeightfield {
        width: 256,
        height: 1,
        cell_size: 1.,
        cell_height: 1.,
        aabb: recast_nav::Aabb3d {
            min: [0.; 3].into(),
            max: [256., 65535., 1.].into(),
        },
        cells: vec![CompactCell::default(); 256],
        areas: vec![AreaType(63); 255],
        ..Default::default()
    };
    for (id, cell) in ceiling.cells[..255].iter_mut().enumerate() {
        cell.set_index(id as u32);
        cell.set_count(1);
        ceiling.spans.push(CompactSpan {
            y: (id * 256) as u16,
            data: u32::MAX,
            ..Default::default()
        });
    }
    ceiling.cells[255].set_index(0x00ff_ffff);
    let layers = recast_nav::layers::build_heightfield_layers(&ceiling, 0, 1).unwrap();
    assert_eq!(layers.len(), 255);
    let mut input = field_input(&ceiling);
    writeln!(input, "1 layers 1").unwrap();
    assert_eq!(
        serde_json::json!([
            snapshot(&ceiling),
            layer_snapshot(&layers),
            snapshot(&ceiling)
        ]),
        oracle(&input),
        "native final layer IDs 0 through 254 and sentinel 255"
    );
}

#[test]
fn native_region_context_strategies_nested_loops_filters_and_empty_fields() {
    // Shift reachable spans beyond ushort indices. Central high-distance seeds
    // leave lower-distance neighbors for deferred expansion writes; the unused
    // prefix must not receive their region IDs.
    let mut shifted = field(213);
    let prefix = 65536;
    shifted
        .spans
        .splice(..0, std::iter::repeat_n(CompactSpan::default(), prefix));
    shifted.areas.fill(AreaType(63));
    shifted
        .areas
        .splice(..0, std::iter::repeat_n(AreaType(0), prefix));
    shifted.dist = vec![0; shifted.spans.len()];
    shifted.max_distance = 12;
    for (i, cell) in shifted.cells.iter_mut().enumerate() {
        cell.set_index(cell.index() + prefix as u32);
        let x = i as i32 % shifted.width;
        let z = i as i32 / shifted.width;
        if (2..7).contains(&x) && (2..5).contains(&z) {
            shifted.dist[cell.index_range()].fill(12);
        }
    }
    let mut input = field_input(&shifted).replacen("areas ", "areas_context ", 1);
    write!(input, "2 set_distance {} ", shifted.max_distance).unwrap();
    for &distance in &shifted.dist {
        write!(input, "{distance} ").unwrap();
    }
    writeln!(input, "watershed_try 0 0 20").unwrap();
    let initial = snapshot(&shifted);
    let hooks = ContextRecorder::default();
    let context = Context::with_hooks(true, &hooks);
    shifted
        .build_regions_with_context(&context, 0, 0, 20)
        .unwrap();
    assert_ne!(shifted.spans[prefix].region, RegionId::NONE);
    assert_eq!(
        serde_json::json!([
            [initial, []],
            [initial, []],
            1,
            [snapshot(&shifted), *hooks.0.borrow()]
        ]),
        oracle(&input),
        "native watershed stacks and dirty writes above ushort span indices"
    );
    for partition in ["watershed", "monotone", "layer"] {
        for border in [0, 1, 3] {
            for enabled in [false, true] {
                let mut hf = field(213);
                hf.areas.fill(AreaType(63));
                let hooks = ContextRecorder::default();
                let context = Context::with_hooks(true, &hooks);
                let mut input = field_input(&hf).replacen("areas ", "areas_context ", 1);
                writeln!(
                    input,
                    "2 timer {} {partition} {border} 0 20",
                    u8::from(enabled)
                )
                .unwrap();
                let mut actual = vec![serde_json::json!([snapshot(&hf), []])];
                context.enable_timer(enabled);
                actual.push(serde_json::json!([snapshot(&hf), []]));
                match partition {
                    "watershed" => {
                        hf.build_distance_field_with_context(&context);
                        hf.build_regions_with_context(&context, border, 0, 20)
                            .unwrap();
                    }
                    "monotone" => hf
                        .build_regions_monotone_with_context(&context, border, 0, 20)
                        .unwrap(),
                    _ => hf
                        .build_layer_regions_with_context(&context, border, 0)
                        .unwrap(),
                }
                if partition == "watershed" && enabled {
                    let passes = hooks
                        .0
                        .borrow()
                        .iter()
                        .filter(|event| event.as_slice() == [3, 22])
                        .count();
                    assert_eq!(passes, 3);
                }
                actual.push(serde_json::json!([snapshot(&hf), *hooks.0.borrow()]));
                assert_eq!(
                    serde_json::json!(actual),
                    oracle(&input),
                    "partition={partition} border={border} enabled={enabled}"
                );
            }
        }
    }
    let mut hf = CompactHeightfield {
        width: 3,
        height: 1,
        cell_size: 1.,
        cell_height: 1.,
        cells: vec![CompactCell::default(); 3],
        ..Default::default()
    };
    let hooks = ContextRecorder::default();
    let context = Context::with_hooks(true, &hooks);
    let mut input = field_input(&hf).replacen("areas ", "areas_context ", 1);
    writeln!(input, "1 watershed 0 0 0").unwrap();
    let initial = snapshot(&hf);
    hf.build_distance_field_with_context(&context);
    hf.build_regions_with_context(&context, 0, 0, 0).unwrap();
    assert!(
        !hooks
            .0
            .borrow()
            .iter()
            .any(|event| event.as_slice() == [3, 22])
    );
    assert_eq!(
        serde_json::json!([[initial, []], [snapshot(&hf), *hooks.0.borrow()]]),
        oracle(&input)
    );
}

#[test]
fn native_watershed_overflow_retains_output_and_leaves_manual_timer_open() {
    let count = usize::from(u16::MAX);
    let mut hf = CompactHeightfield {
        width: count as i32,
        height: 1,
        cell_size: 1.,
        cell_height: 1.,
        aabb: recast_nav::Aabb3d {
            min: [0.; 3].into(),
            max: [count as f32, 255., 1.].into(),
        },
        cells: vec![CompactCell::default(); count],
        spans: vec![
            CompactSpan {
                data: u32::MAX,
                ..Default::default()
            };
            count
        ],
        areas: vec![AreaType(63); count],
        ..Default::default()
    };
    for (i, cell) in hf.cells.iter_mut().enumerate() {
        cell.set_index(i as u32);
        cell.set_count(1);
    }
    let mut input = field_input(&hf).replacen("areas ", "areas_context ", 1);
    write!(input, "2 set_distance 10 ").unwrap();
    for _ in 0..count {
        write!(input, "10 ").unwrap();
    }
    writeln!(input, "watershed_try 0 0 0").unwrap();
    hf.dist = vec![10; count];
    hf.max_distance = 10;
    let initial = snapshot(&hf);
    let hooks = ContextRecorder::default();
    let context = Context::with_hooks(true, &hooks);
    assert!(matches!(
        hf.build_regions_with_context(&context, 0, 0, 0),
        Err(BuildRegionsError::RegionIdOverflow)
    ));
    assert_eq!(snapshot(&hf), initial);
    assert!(
        hooks
            .0
            .borrow()
            .iter()
            .any(|event| event.as_slice() == [3, 21])
    );
    assert!(
        !hooks
            .0
            .borrow()
            .iter()
            .any(|event| event.as_slice() == [4, 21])
    );
    let expected = oracle(&input);
    assert_eq!(expected[2], 0);
    assert_eq!(expected[3][1], serde_json::json!(*hooks.0.borrow()));
    assert!(
        serde_json::json!([
            [initial, []],
            [snapshot(&hf), []],
            0,
            [snapshot(&hf), *hooks.0.borrow()]
        ]) == expected,
        "native watershed overflow field or frame mismatch"
    );
}

#[test]
fn native_watershed_overlap_diagnostic_preserves_success_and_region_output() {
    let mut hf = CompactHeightfield {
        width: 2,
        height: 1,
        cell_size: 1.,
        cell_height: 1.,
        aabb: recast_nav::Aabb3d {
            min: [0.; 3].into(),
            max: [2., 10., 1.].into(),
        },
        cells: vec![CompactCell::default(); 2],
        spans: [0, 4, 0, 4]
            .map(|y| CompactSpan {
                y,
                data: 0x02ff_ffff,
                ..Default::default()
            })
            .to_vec(),
        areas: vec![AreaType(63); 4],
        ..Default::default()
    };
    hf.cells[0].set_count(2);
    hf.cells[1].set_index(2);
    hf.cells[1].set_count(2);
    // All four cross-layer links stay in the two allocated columns. This cycle
    // joins both spans of each column into one overlapping region.
    hf.spans[0].set_con(2, Some(0));
    hf.spans[1].set_con(2, Some(1));
    hf.spans[2].set_con(0, Some(1));
    hf.spans[3].set_con(0, Some(0));
    let initial = snapshot(&hf);
    let mut input = field_input(&hf).replacen("areas ", "areas_context ", 1);
    writeln!(input, "2 set_distance 10 10 10 10 10 watershed_try 0 0 0").unwrap();
    hf.dist = vec![10; 4];
    hf.max_distance = 10;
    let hooks = ContextRecorder::default();
    let context = Context::with_hooks(true, &hooks);
    hf.build_regions_with_context(&context, 0, 0, 0).unwrap();
    // Native maxRegions also retains its unused trailing region ID.
    assert_eq!(hf.max_region.bits(), 2);
    assert!(hf.spans.iter().all(|span| span.region.bits() == 1));
    let message = b"rcBuildRegions: 1 overlapping regions.";
    assert!(hooks.0.borrow().iter().any(|event| {
        event.starts_with(&[1, 3])
            && event[2..]
                .iter()
                .copied()
                .eq(message.iter().map(|&b| i64::from(b)))
    }));
    assert_eq!(
        serde_json::json!([
            [initial, []],
            [initial, []],
            1,
            [snapshot(&hf), *hooks.0.borrow()]
        ]),
        oracle(&input)
    );
}

#[test]
fn native_sort_equal_key_record_order() {
    let mut cases = Vec::new();
    for n in [
        0, 1, 2, 3, 4, 7, 8, 9, 15, 16, 17, 40, 41, 64, 127, 256, 1024, 4097,
    ] {
        for pattern in 0..8 {
            let mut seed = 719_u32;
            let keys: Vec<i32> = (0..n)
                .map(|i| match pattern {
                    0 => 0,
                    1 => i,
                    2 => -i,
                    3 => i.min(n - 1 - i),
                    4 => i % 7,
                    5 => {
                        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                        (seed % 13) as i32
                    }
                    6 => {
                        if i % 2 == 0 {
                            i32::MIN
                        } else {
                            i32::MAX
                        }
                    }
                    _ => {
                        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                        seed as i32
                    }
                })
                .collect();
            cases.push(keys);
        }
    }
    let mut input = format!("sort_keys {}\n", cases.len());
    let mut actual = Vec::new();
    for keys in cases {
        writeln!(input, "{}", keys.len()).unwrap();
        for key in &keys {
            write!(input, "{key} ").unwrap();
        }
        let mut records: Vec<_> = keys.into_iter().enumerate().map(|(i, k)| (k, i)).collect();
        recast_nav::native_sort::sort_by_key(&mut records, |record| record.0);
        actual.push(records);
    }
    let expected = oracle(&input);
    for (i, records) in actual.into_iter().enumerate() {
        assert_eq!(serde_json::json!(records), expected[i], "case {i}");
    }
}

// Connected grids keep all native contour state initialized.
fn contour_grid(pattern: usize) -> CompactHeightfield {
    let mut hf = CompactHeightfield {
        width: 24,
        height: 24,
        cell_size: 1.,
        cell_height: 0.5,
        aabb: recast_nav::Aabb3d {
            min: [0.; 3].into(),
            max: [24., 4., 24.].into(),
        },
        ..Default::default()
    };
    let occupied = |x: i32, z: i32| -> bool {
        if !(0..24).contains(&x) || !(0..24).contains(&z) {
            return false;
        }
        match pattern {
            0 => false,
            1 => {
                (1..23).contains(&x)
                    && (1..23).contains(&z)
                    && !((6..10).contains(&x) && (6..10).contains(&z))
                    && !((14..18).contains(&x) && (14..18).contains(&z))
            }
            2 => x % 4 < 2 && z % 4 < 2,
            4 => (1..23).contains(&x) && (1..23).contains(&z),
            _ => {
                (x < 2 && z < 2)
                    || (x >= 8 && z >= 8 && !((12..16).contains(&x) && (12..16).contains(&z)))
            }
        }
    };
    for z in 0..hf.height {
        for x in 0..hf.width {
            let mut cell = CompactCell::default();
            cell.set_index(hf.spans.len() as u32);
            if occupied(x, z) {
                cell.set_count(1);
                let mut span = CompactSpan {
                    y: if pattern == 4 && (2..22).contains(&x) && (2..22).contains(&z) {
                        1 + ((x * 13 + z * 7) % 4) as u16 * 2
                    } else {
                        1
                    },
                    region: 1.into(),
                    ..Default::default()
                };
                span.set_height(7);
                for (dir, (dx, dz)) in [(-1, 0), (0, 1), (1, 0), (0, -1)].into_iter().enumerate() {
                    span.set_con(dir as u8, occupied(x + dx, z + dz).then_some(0));
                }
                hf.spans.push(span);
                hf.areas.push(AreaType(63));
            }
            hf.cells.push(cell);
        }
    }
    hf
}

#[test]
fn native_contour_context_trace_simplify_capacity_and_multiple_outline_logs() {
    for pattern in 0..4 {
        for bits in 0..4 {
            for enabled in [false, true] {
                let mut hf = contour_grid(pattern);
                let hooks = ContextRecorder::default();
                let context = Context::with_hooks(true, &hooks);
                let mut input = field_input(&hf).replacen("areas ", "areas_context ", 1);
                // The no-hole capacity case deliberately has zero initial capacity metadata.
                let max_region = if pattern == 2 { 0 } else { 1 };
                writeln!(
                    input,
                    "3 set_region_meta {max_region} 0 timer {} contours 0.5 2 {bits}",
                    u8::from(enabled)
                )
                .unwrap();
                let mut actual = vec![serde_json::json!([snapshot(&hf), []])];
                hf.max_region = max_region.into();
                actual.push(serde_json::json!([snapshot(&hf), []]));
                context.enable_timer(enabled);
                actual.push(serde_json::json!([snapshot(&hf), []]));
                let contours = hf
                    .build_contours_with_context(
                        &context,
                        0.5,
                        2,
                        BuildContoursFlags::from_bits_retain(bits),
                    )
                    .unwrap();
                actual.push(serde_json::json!(contour_snapshot(&contours)));
                actual.push(serde_json::json!([snapshot(&hf), *hooks.0.borrow()]));
                if pattern == 2 {
                    assert_eq!(contours.contours.len(), 36);
                    assert!(
                        hooks
                            .0
                            .borrow()
                            .iter()
                            .any(|event| event.starts_with(&[1, 2]))
                    );
                }
                if pattern == 3 {
                    assert!(
                        hooks
                            .0
                            .borrow()
                            .iter()
                            .any(|event| event.starts_with(&[1, 3]))
                    );
                }
                let expected = oracle(&input);
                assert!(
                    serde_json::json!(actual) == expected,
                    "pattern={pattern}, bits={bits}, enabled={enabled}; events={:?}, native events={}",
                    *hooks.0.borrow(),
                    expected[4][1]
                );
            }
        }
    }
}

#[test]
fn native_polygon_and_detail_context_full_stage_outputs() {
    for pattern in [0, 1, 3, 4] {
        for sample_distance in if pattern == 4 { [1., 3.] } else { [1.5, 3.] } {
            for enabled in [false, true] {
                let mut hf = contour_grid(pattern);
                let hooks = ContextRecorder::default();
                let context = Context::with_hooks(true, &hooks);
                let mut input = field_input(&hf).replacen("areas ", "areas_context ", 1);
                writeln!(
                    input,
                    "3 set_region_meta 1 0 timer {} meshes 0.5 2 3 6 {sample_distance} 0.1",
                    u8::from(enabled)
                )
                .unwrap();
                let mut actual = vec![serde_json::json!([snapshot(&hf), []])];
                hf.max_region = 1.into();
                actual.push(serde_json::json!([snapshot(&hf), []]));
                context.enable_timer(enabled);
                actual.push(serde_json::json!([snapshot(&hf), []]));
                let contours = hf
                    .build_contours_with_context(
                        &context,
                        0.5,
                        2,
                        BuildContoursFlags::from_bits_retain(3),
                    )
                    .unwrap();
                let csnapshot = contour_snapshot(&contours);
                let mesh = contours
                    .into_polygon_mesh_with_context(&context, 6)
                    .unwrap();
                let detail = recast_nav::DetailNavmesh::new_with_context(
                    &context,
                    &mesh,
                    &hf,
                    sample_distance,
                    0.1,
                )
                .unwrap();
                if pattern == 4 && sample_distance == 1. {
                    assert_eq!(
                        detail.meshes.iter().map(|mesh| mesh.vertex_count).max(),
                        Some(127),
                        "rough interior must exercise repeated refinement through the native vertex cap"
                    );
                }
                actual.push(serde_json::json!([
                    csnapshot,
                    poly_snapshot(&mesh),
                    detail_snapshot(&detail)
                ]));
                actual.push(serde_json::json!([snapshot(&hf), *hooks.0.borrow()]));
                let expected = oracle(&input);
                let actual = serde_json::json!(actual);
                if actual != expected {
                    let directory = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                        .join("../../target/parity-failures");
                    std::fs::create_dir_all(&directory).unwrap();
                    for (name, bytes) in [
                        ("detail-input.txt", input.as_bytes().to_vec()),
                        ("detail-rust.json", serde_json::to_vec(&actual).unwrap()),
                        ("detail-cpp.json", serde_json::to_vec(&expected).unwrap()),
                    ] {
                        std::fs::write(directory.join(name), bytes).unwrap();
                    }
                }
                assert!(
                    actual == expected,
                    "pattern={pattern}, sample={sample_distance}, enabled={enabled}; events={:?}, native events={}",
                    *hooks.0.borrow(),
                    expected[4][1]
                );
            }
        }
    }
}

#[test]
fn native_detail_fallback_uses_global_voxel_vertices() {
    for padding in [300, 0] {
        for region in [0, 2] {
            for pattern in [3, 4] {
                let mut hf = contour_grid(pattern);
                let bounded_seed = padding == 0 && region == 0 && pattern == 3;
                let hooks = ContextRecorder::default();
                let context = Context::with_hooks(true, &hooks);
                let mut input = field_input(&hf).replacen("areas ", "areas_context ", 1);
                writeln!(
                    input,
                    "2 set_region_meta 1 0 meshes_seed 0.5 2 3 6 1.5 0.1 {padding} {region} {}",
                    u8::from(bounded_seed)
                )
                .unwrap();
                // Native completes before Rust is called: every seed and index
                // is valid even when the mesh's vertex IDs exceed local scratch.
                let expected = oracle(&input);
                let mut actual = vec![serde_json::json!([snapshot(&hf), []])];
                hf.max_region = 1.into();
                actual.push(serde_json::json!([snapshot(&hf), []]));
                let contours = hf
                    .build_contours_with_context(
                        &context,
                        0.5,
                        2,
                        BuildContoursFlags::from_bits_retain(3),
                    )
                    .unwrap();
                let csnapshot = contour_snapshot(&contours);
                let mut mesh = contours
                    .into_polygon_mesh_with_context(&context, 6)
                    .unwrap();
                let mut vertices = vec![Default::default(); padding];
                vertices.extend_from_slice(&mesh.vertices);
                mesh.vertices = vertices;
                if bounded_seed {
                    // Native stops at the first exact-height seed. The declared
                    // spare spans must never be read by this operation.
                    for vertex in &mut mesh.vertices {
                        vertex.y = hf.spans[0].y;
                    }
                    for cell in &mut hf.cells {
                        if cell.count() != 0 {
                            cell.set_count(255);
                        }
                    }
                }
                mesh.regions.fill(region.into());
                for index in &mut mesh.polygons {
                    if *index != recast_nav::PolygonNavmesh::NO_INDEX {
                        *index += padding as u16;
                    }
                }
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    recast_nav::DetailNavmesh::new_with_context(&context, &mesh, &hf, 1.5, 0.1)
                        .unwrap()
                }));
                let detail = result.unwrap_or_else(|failure| {
                    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                        .join("../../target/parity-failures");
                    std::fs::create_dir_all(&dir).unwrap();
                    std::fs::write(dir.join("detail-seed-input.txt"), input.as_bytes()).unwrap();
                    std::fs::write(
                        dir.join("detail-seed-native.json"),
                        serde_json::to_vec(&expected).unwrap(),
                    )
                    .unwrap();
                    std::panic::resume_unwind(failure)
                });
                actual.push(serde_json::json!([
                    csnapshot,
                    poly_snapshot(&mesh),
                    detail_snapshot(&detail)
                ]));
                actual.push(serde_json::json!([snapshot(&hf), *hooks.0.borrow()]));
                assert_eq!(
                    serde_json::json!(actual),
                    expected,
                    "padding={padding}, region={region}, pattern={pattern}"
                );
            }
        }
    }
}

#[test]
fn detail_sampling_disabled_rejects_native_uninitialized_hull_domain() {
    let mut hf = contour_grid(1);
    hf.max_region = 1.into();
    let mesh = hf
        .build_contours(0.5, 2, BuildContoursFlags::empty())
        .unwrap()
        .into_polygon_mesh(6)
        .unwrap();
    for distance in [0., -1., f32::NAN] {
        assert!(matches!(
            recast_nav::DetailNavmesh::new(&mesh, &hf, distance, 0.1),
            Err(recast_nav::DetailNavmeshError::UninitializedHull)
        ));
        assert!(
            recast_nav::DetailNavmesh::new(
                &recast_nav::PolygonNavmesh::default(),
                &hf,
                distance,
                0.1
            )
            .is_ok()
        );
    }
}

#[test]
fn native_merge_polygon_context_reports_full_integer_vertex_count() {
    let sources: Vec<_> = (0..2)
        .map(|half| recast_nav::PolygonNavmesh {
            max_vertices_per_polygon: 3,
            cell_size: 1.,
            cell_height: 1.,
            aabb: recast_nav::Aabb3d {
                min: [0.; 3].into(),
                max: [65536., 8., 1.].into(),
            },
            vertices: (0..32768)
                .map(|i| glam::U16Vec3::new((half * 32768 + i) as u16, 0, 0))
                .collect(),
            ..Default::default()
        })
        .collect();
    let mut input = String::from("merge_poly_context 2\n");
    for mesh in &sources {
        writeln!(input, "1 1 3 0 0 0 0 0 65536 8 1 {} 0", mesh.vertices.len()).unwrap();
        for vertex in &mesh.vertices {
            for coordinate in vertex.to_array() {
                write!(input, "{coordinate} ").unwrap();
            }
        }
    }
    let hooks = ContextRecorder::default();
    let context = Context::with_hooks(true, &hooks);
    let mesh = merge_poly_meshes_with_context(&context, &[&sources[0], &sources[1]]);
    assert_eq!(mesh.vertices.len(), 65536);
    let mut snapshots: Vec<_> = sources.iter().map(poly_snapshot).collect();
    snapshots.push(poly_snapshot(&mesh));
    assert!(
        serde_json::json!([snapshots, *hooks.0.borrow()]) == oracle(&input),
        "native mesh overflow output/context differ"
    );
}

#[test]
fn native_empty_mesh_metadata_zero_width_counts_and_detail_early_return() {
    for nvp in [0, 1, 2, 6, 8, 65536] {
        for vertices in [0, 1] {
            let hooks = ContextRecorder::default();
            let context = Context::with_hooks(true, &hooks);
            let mut mesh = ContourSet::default()
                .into_polygon_mesh_with_context(&context, nvp)
                .unwrap();
            mesh.vertices = vec![glam::U16Vec3::ZERO; vertices];
            assert_eq!(mesh.polygon_count(), 0);
            assert_eq!(mesh.polygons().count(), 0);
            let detail = recast_nav::DetailNavmesh::new_with_context(
                &context,
                &mesh,
                &CompactHeightfield::default(),
                0.,
                0.,
            )
            .unwrap();
            assert_eq!(
                serde_json::json!([
                    1,
                    poly_snapshot(&mesh),
                    detail_snapshot(&detail),
                    *hooks.0.borrow()
                ]),
                oracle(&format!("empty_mesh_metadata {nvp} {vertices} 0"))
            );
        }
    }
    let mesh = PolygonNavmesh::default();
    let merged = merge_poly_meshes(&[&mesh]);
    assert_eq!(merged.polygon_count(), 0);
}

#[test]
fn solid_span_keys_survive_growth_clone_and_slot_reuse() {
    use std::mem::size_of;

    // Key replacement must preserve the packed pool allocation sizes.
    assert_eq!(size_of::<Option<SpanKey>>(), 8);
    assert_eq!(size_of::<Span>(), 16);
    let mut spans = Spans::default();
    let keys: Vec<_> = (0..4097)
        .map(|i| spans.insert(Span::new(i, i + 1, AreaType(63))))
        .collect();
    for (i, &key) in keys.iter().enumerate() {
        assert_eq!(spans[key].min(), i as u16);
    }
    assert!(spans.get(SpanKey::default()).is_none());
    let mut copy = spans.clone();
    for i in [0, 2048, 4096] {
        assert_eq!(spans.remove(keys[i]).unwrap().min(), i as u16);
        assert!(spans.get(keys[i]).is_none());
        assert_eq!(copy[keys[i]].min(), i as u16);
    }
    for i in 0..3 {
        let key = spans.insert(Span::new(7, 8, AreaType(i)));
        assert!(!keys.contains(&key));
        assert_eq!(spans[key].area(), AreaType(i));
    }
    assert_eq!(spans.len(), keys.len());
    assert_eq!(spans.iter().count(), keys.len());
    copy.clear();
    assert!(copy.is_empty());
    assert!(keys.iter().all(|&key| copy.get(key).is_none()));
    let key = copy.insert(Span::new(1, 2, AreaType(3)));
    assert!(!keys.contains(&key));
    assert_eq!(spans.len(), keys.len());
}

#[test]
fn native_layer_and_region_sweep_samples_wrap_at_sixteen_bits() {
    for width in [65535, 65536] {
        let mut hf = CompactHeightfield {
            width,
            height: 2,
            cell_size: 1.,
            cell_height: 1.,
            aabb: recast_nav::Aabb3d {
                min: [0.; 3].into(),
                max: [width as f32, 510., 2.].into(),
            },
            cells: vec![CompactCell::default(); width as usize * 2],
            spans: vec![
                CompactSpan {
                    data: u32::MAX,
                    ..Default::default()
                };
                width as usize * 2
            ],
            areas: vec![AreaType(63); width as usize * 2],
            ..Default::default()
        };
        for z in 0..2 {
            for x in 0..width as usize {
                let i = x + z * width as usize;
                hf.cells[i].set_index(i as u32);
                hf.cells[i].set_count(1);
                hf.spans[i].y = z as u16 * 255;
                for (dir, connected) in [
                    (0, x > 0),
                    (1, z == 0),
                    (2, x + 1 < width as usize),
                    (3, z == 1),
                ] {
                    if connected {
                        hf.spans[i].set_con(dir, Some(0));
                    }
                }
            }
        }
        let mut input = field_input(&hf);
        write!(input, "3 layers 1 monotone 0 0 0 layer 0 0 0").unwrap();
        let expected = oracle(&input);
        let layers = recast_nav::layers::build_heightfield_layers(&hf, 0, 1).unwrap();
        // Check counts first so a regression does not print million-entry grids.
        assert_eq!(
            layers.len() as u64,
            expected[1][0].as_u64().unwrap(),
            "width={width}"
        );
        let mut actual = vec![snapshot(&hf), layer_snapshot(&layers), snapshot(&hf)];
        // The partition's previous-row counter must retain 65,536 while its
        // sweep's ushort sample count wraps to zero. Reuse must clear rows.
        hf.build_regions_monotone(0, 0, 0).unwrap();
        actual.push(snapshot(&hf));
        hf.build_layer_regions(0, 0).unwrap();
        actual.push(snapshot(&hf));
        assert_eq!(serde_json::json!(actual), expected, "width={width}");
    }
}

#[test]
fn native_detail_extent_rounding_boundary() {
    let mut hf = contour_grid(4);
    for z in 0..hf.height {
        for x in 0..hf.width {
            let cell = hf.cells[(x + z * hf.width) as usize];
            if cell.count() != 0 {
                hf.spans[cell.index() as usize].y = if x == 3 && z == 3 { 7 } else { 1 };
            }
        }
    }
    // sqrt(40.5) rounds upward. Squaring that rounded threshold incorrectly
    // classifies this triangle as too narrow to receive its interior sample.
    let distance = 40.5_f32.sqrt() * 0.5;
    for sample_distance in [distance.next_down(), distance, distance.next_up()] {
        assert_native_detail_polygon(&hf, &[[1, 1, 1], [1, 1, 10], [10, 1, 1]], sample_distance);
    }
}

fn assert_native_detail_polygon(
    hf: &CompactHeightfield,
    vertices: &[[u16; 3]],
    sample_distance: f32,
) {
    let mesh = recast_nav::PolygonNavmesh {
        vertices: vertices.iter().copied().map(Into::into).collect(),
        polygons: (0..vertices.len() as u16).collect(),
        polygon_neighbors: vec![u16::MAX; vertices.len()],
        flags: vec![0],
        regions: vec![1.into()],
        areas: vec![AreaType(63)],
        max_polygons: 1,
        max_vertices_per_polygon: vertices.len() as u32,
        aabb: hf.aabb,
        cell_size: hf.cell_size,
        cell_height: hf.cell_height,
        max_edge_error: 1.,
        ..Default::default()
    };
    let hooks = ContextRecorder::default();
    let context = Context::with_hooks(true, &hooks);
    let mut input = field_input(hf).replacen("areas ", "areas_context ", 1);
    writeln!(
        input,
        "1 detail_polygon {} {sample_distance} 0.1",
        vertices.len()
    )
    .unwrap();
    for vertex in &mesh.vertices {
        writeln!(input, "{} {} {}", vertex.x, vertex.y, vertex.z).unwrap();
    }
    let expected = oracle(&input);
    let detail =
        recast_nav::DetailNavmesh::new_with_context(&context, &mesh, hf, sample_distance, 0.1)
            .unwrap();
    let actual = serde_json::json!([
        [snapshot(hf), []],
        detail_snapshot(&detail),
        [snapshot(hf), *hooks.0.borrow()]
    ]);
    assert!(
        actual == expected,
        "distance={sample_distance:?}, bits={:#x}",
        sample_distance.to_bits()
    );
}

#[test]
fn native_detail_edge_sample_rounding() {
    let mut hf = contour_grid(4);
    let mut random = 299_u32;
    for span in &mut hf.spans {
        random = random.wrapping_mul(1664525).wrapping_add(1013904223);
        span.y = 1 + ((random >> 24) % 8) as u16;
    }
    // At this distance, uncontracted edge deviations retain an extra boundary
    // sample in FMA mode, leaving less room for interior detail at the vertex cap.
    assert_native_detail_polygon(
        &hf,
        &[[1, 1, 1], [1, 1, 23], [23, 1, 23], [23, 1, 1]],
        f32::from_bits(0x3f8c_be30),
    );
}
