#![cfg(feature = "cpp-oracle")]
use recast_nav::{cache::*, layers::HeightfieldLayer, nav::*};
use serde_json::{Value, json};
use std::{
    fmt::Write as _,
    io::Write as _,
    process::{Command, Stdio},
};

fn oracle(input: String) -> Value {
    let mut child = Command::new(env!("TILE_CACHE_ORACLE"))
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
fn numbers(input: &mut String, values: impl IntoIterator<Item = f32>) {
    for value in values {
        write!(input, "{} ", value.to_bits()).unwrap();
    }
}
fn snapshot(layer: &HeightfieldLayer, status: u32) -> Value {
    json!([status, layer.heights, layer.areas, layer.connections])
}
fn field(w: usize, h: usize, cs: f32, ch: f32) -> HeightfieldLayer {
    let mut seed = 0x73a8_d671u32;
    let mut values = || {
        (0..w * h)
            .map(|_| {
                seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                (seed >> 16) as u8
            })
            .collect::<Vec<_>>()
    };
    HeightfieldLayer {
        bounds: [
            [-1.3, 0.13, 2.1],
            [w as f32 * cs - 1.3, 256. * ch + 0.13, h as f32 * cs + 2.1],
        ],
        cell_size: cs,
        cell_height: ch,
        width: w,
        height: h,
        min_x: 0,
        max_x: w.saturating_sub(1),
        min_z: 0,
        max_z: h.saturating_sub(1),
        hmin: 0,
        hmax: 255,
        heights: values(),
        areas: values(),
        connections: values(),
    }
}
#[test]
fn native_cache_shape_marking_preserves_voxel_rules_and_byte_areas() {
    for (w, h) in [(19, 13), (0, 13), (255, 1), (1, 255)] {
        for (cs, ch) in [(0.3, 0.2), (1., 0.5), (0.25, 0.125)] {
            let mut layer = field(w, h, cs, ch);
            if let Some(height) = layer.heights.first_mut() {
                *height = 255;
                layer.areas[0] = 63;
            }
            let mut input = format!("mark {w} {h} ");
            numbers(&mut input, layer.bounds[0]);
            numbers(&mut input, [cs, ch]);
            for value in layer
                .heights
                .iter()
                .chain(&layer.areas)
                .chain(&layer.connections)
            {
                write!(input, "{value} ").unwrap();
            }
            let mut actual = vec![snapshot(&layer, SUCCESS)];
            input.push_str("180 ");
            let mut seed = 0x19e7_52a3u32;
            for i in 0..180 {
                let mut value = || {
                    seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                    (seed >> 16) as f32 / 65535.
                };
                let mut center = [
                    layer.bounds[0][0] + (value() * 25. - 3.) * cs,
                    layer.bounds[0][1] + value() * 256. * ch,
                    layer.bounds[0][2] + (value() * 20. - 3.) * cs,
                ];
                let mut extents = [value() * 8. * cs, value() * 256. * ch, value() * 8. * cs];
                if i < 3 {
                    center = [
                        layer.bounds[0][0] + 0.5 * cs,
                        layer.bounds[0][1] + 255. * ch,
                        layer.bounds[0][2] + 0.5 * cs,
                    ];
                    extents = [0.; 3];
                }
                let area = [0, 1, 63, 127, 255][i % 5];
                let status = match i % 3 {
                    0 => {
                        let min = if i == 3 {
                            layer.bounds[0]
                        } else {
                            std::array::from_fn(|a| center[a] - extents[a])
                        };
                        let max = if i == 3 {
                            layer.bounds[1]
                        } else {
                            std::array::from_fn(|a| center[a] + extents[a])
                        };
                        input.push_str("B ");
                        numbers(&mut input, min);
                        numbers(&mut input, max);
                        write!(input, "{area} ").unwrap();
                        mark_box_area(&mut layer, min, max, area)
                    }
                    1 => {
                        let radius = if i == 1 {
                            0.
                        } else if i == 4 {
                            -extents[0]
                        } else {
                            extents[0]
                        };
                        input.push_str("C ");
                        numbers(&mut input, center);
                        let height = if i == 7 { -extents[1] } else { extents[1] };
                        numbers(&mut input, [radius, height]);
                        write!(input, "{area} ").unwrap();
                        mark_cylinder_area(&mut layer, center, radius, height, area)
                    }
                    _ => {
                        let angle = value() * std::f32::consts::TAU;
                        let rot = if i == 5 {
                            [f32::NAN, 0.5]
                        } else if i == 2 {
                            [0., 0.5]
                        } else {
                            [-0.5 * angle.sin(), 0.5 * angle.cos()]
                        };
                        input.push_str("O ");
                        numbers(&mut input, center);
                        numbers(&mut input, extents);
                        numbers(&mut input, rot);
                        write!(input, "{area} ").unwrap();
                        mark_oriented_box_area(&mut layer, center, extents, rot, area)
                    }
                };
                assert_eq!(status, SUCCESS);
                actual.push(snapshot(&layer, status));
            }
            let expected = oracle(input);
            for (i, actual) in actual.iter().enumerate() {
                assert_eq!(
                    *actual, expected[i],
                    "shape operation {i} grid={w}x{h} cs={cs} ch={ch}"
                );
            }
        }
    }
}

#[test]
fn cache_marking_rejects_invalid_storage_and_conversion_domains_atomically() {
    let mut layer = field(3, 2, 0.3, 0.2);
    let original = layer.areas.clone();
    assert_eq!(
        mark_box_area(&mut layer, [f32::NAN, 0., 0.], [1.; 3], 255),
        FAILURE | INVALID_PARAM
    );
    assert_eq!(
        mark_cylinder_area(&mut layer, [f32::MAX, 0., 0.], 1., 1., 255),
        FAILURE | INVALID_PARAM
    );
    assert_eq!(
        mark_oriented_box_area(&mut layer, [0.; 3], [f32::INFINITY; 3], [0., 0.5], 255),
        FAILURE | INVALID_PARAM
    );
    assert_eq!(layer.areas, original);
    layer.heights.clear();
    assert_eq!(
        mark_box_area(&mut layer, [0.; 3], [1.; 3], 255),
        FAILURE | INVALID_PARAM
    );
    assert_eq!(layer.areas, original);
}

use recast_nav::cache_data::{self, CompressionResult, Compressor};
#[derive(Clone)]
struct FixtureCompressor {
    mode: u8,
    extra: usize,
    limit: Option<usize>,
    report: Option<usize>,
    compress_status: u32,
    decompress_status: u32,
    log: Vec<i64>,
}
impl Default for FixtureCompressor {
    fn default() -> Self {
        Self {
            mode: 0,
            extra: 5,
            limit: None,
            report: None,
            compress_status: SUCCESS,
            decompress_status: SUCCESS,
            log: vec![],
        }
    }
}
impl FixtureCompressor {
    fn input(&self, input: &mut String) {
        write!(
            input,
            "{} {} {} {} {} {} ",
            self.mode,
            self.extra,
            self.limit.map_or(-1, |n| n as i64),
            self.report.map_or(-1, |n| n as i64),
            self.compress_status,
            self.decompress_status
        )
        .unwrap();
    }
    fn record(&mut self, kind: i64, input: &[u8], output: &[u8]) {
        self.log
            .extend([kind, input.len() as i64, output.len() as i64]);
        self.log
            .extend(input.iter().chain(output).map(|&v| i64::from(v)));
    }
}
impl Compressor for FixtureCompressor {
    fn max_compressed_size(&mut self, size: usize) -> usize {
        self.log
            .extend([0, size as i64, (size + self.extra) as i64]);
        size + self.extra
    }
    fn compress(&mut self, input: &[u8], output: &mut [u8]) -> CompressionResult {
        self.record(1, input, output);
        let n = input.len().min(output.len());
        for i in 0..n {
            output[i] = if self.mode == 0 {
                input[i]
            } else {
                input[input.len() - 1 - i] ^ 0xa5
            };
        }
        CompressionResult {
            status: self.compress_status,
            size: self.report.unwrap_or(n),
        }
    }
    fn decompress(&mut self, input: &[u8], output: &mut [u8]) -> CompressionResult {
        self.record(2, input, output);
        let n = self.limit.unwrap_or(output.len()).min(if self.mode == 2 {
            output.len()
        } else {
            input.len().min(output.len())
        });
        for i in 0..n {
            output[i] = match self.mode {
                0 => input[i],
                2 => (i * 17 + 3) as u8,
                _ => input[input.len() - 1 - i] ^ 0xa5,
            };
        }
        CompressionResult {
            status: self.decompress_status,
            size: self.report.unwrap_or(n),
        }
    }
}
fn input_bytes(input: &mut String, data: &[u8]) {
    write!(input, "{} ", data.len()).unwrap();
    for byte in data {
        write!(input, "{byte} ").unwrap();
    }
}
fn header_input(input: &mut String, header: &cache_data::Header) {
    write!(
        input,
        "{} {} {} {} {} {} {} ",
        header.padding[0],
        header.padding[1],
        header.magic,
        header.version,
        header.coordinate[0],
        header.coordinate[1],
        header.coordinate[2]
    )
    .unwrap();
    numbers(input, header.bounds.into_iter().flatten());
    write!(
        input,
        "{} {} {} {} {} {} {} {} ",
        header.height_range[0],
        header.height_range[1],
        header.width,
        header.height,
        header.usable_bounds[0][0],
        header.usable_bounds[1][0],
        header.usable_bounds[0][1],
        header.usable_bounds[1][1]
    )
    .unwrap();
}
fn check_build(
    header: &cache_data::Header,
    grids: &[Vec<u8>; 3],
    mut compressor: FixtureCompressor,
) -> Option<Vec<u8>> {
    compressor.log.clear();
    let mut input = String::from("build ");
    compressor.input(&mut input);
    header_input(&mut input, header);
    for grid in grids {
        input_bytes(&mut input, grid);
    }
    let result = cache_data::build_layer(&mut compressor, header, &grids[0], &grids[1], &grids[2]);
    let actual = match &result {
        Ok(data) => json!([SUCCESS, data, compressor.log]),
        Err(s) => json!([s, null, compressor.log]),
    };
    let expected = oracle(input);
    assert_eq!(
        actual, expected,
        "native compressed layer construction and compressor callbacks"
    );
    // Feed the C++ producer's bytes into the Rust decompressor below. Equality
    // also checks every reserved/header/payload byte from the Rust producer.
    expected[1]
        .as_array()
        .map(|data| data.iter().map(|v| v.as_u64().unwrap() as u8).collect())
}
fn check_decompress(data: &[u8], mut compressor: FixtureCompressor) {
    compressor.log.clear();
    let mut input = String::from("decompress ");
    compressor.input(&mut input);
    input_bytes(&mut input, data);
    let result = cache_data::decompress_layer(&mut compressor, data);
    let actual = match result {
        Ok(layer) => json!([
            SUCCESS,
            layer.region_count,
            layer.header.to_bytes().as_slice(),
            layer.grids(),
            compressor.log
        ]),
        Err(s) => json!([s, null, compressor.log]),
    };
    assert_eq!(
        actual,
        oracle(input),
        "native decompression, initialized capacity, grids and callbacks"
    );
}
fn check_endian(data: &[u8]) -> Vec<u8> {
    let mut input = String::from("endian ");
    input_bytes(&mut input, data);
    let mut data = data.to_vec();
    let ok = cache_data::swap_header_endian(&mut data);
    assert_eq!(json!([u8::from(ok), data]), oracle(input));
    data
}
#[test]
fn native_cache_layer_bytes_callbacks_decompression_and_endian_round_trips() {
    for (width, height) in [(0, 0), (1, 1), (7, 5), (255, 1)] {
        let header = cache_data::Header {
            coordinate: [-3, 7, -2],
            bounds: [
                [-1.3, -0., 2.1],
                [9.7, f32::from_bits(0x7fc0_1234), f32::NEG_INFINITY],
            ],
            height_range: [0x1234, 0xabcd],
            width,
            height,
            usable_bounds: [[3, 5], [17, 11]],
            padding: [0xa7, 0x39],
            ..cache_data::Header::default()
        };
        let grids = std::array::from_fn(|g| {
            (0..header.grid_size() + 3)
                .map(|i| (i * 37 + g * 11) as u8)
                .collect::<Vec<_>>()
        });
        for mode in [0, 1, 2] {
            let compressor = FixtureCompressor {
                mode,
                ..FixtureCompressor::default()
            };
            let data = check_build(&header, &grids, compressor.clone()).unwrap();
            assert_eq!(
                cache_data::Header::parse(&data).unwrap().to_bytes(),
                header.to_bytes()
            );
            for (limit, report, status) in [
                (None, None, SUCCESS),
                (Some(7), None, SUCCESS),
                (None, Some(999999), IN_PROGRESS | BUFFER_TOO_SMALL),
                (None, None, FAILURE | OUT_OF_MEMORY),
            ] {
                check_decompress(
                    &data,
                    FixtureCompressor {
                        limit,
                        report,
                        decompress_status: status,
                        ..compressor.clone()
                    },
                );
            }
            // The paired native reader consumes the bytes emitted by Rust, as
            // well as the identical C++ buffer already cross-loaded above.
            let mut rust_compressor = compressor.clone();
            let rust = cache_data::build_layer(
                &mut rust_compressor,
                &header,
                &grids[0],
                &grids[1],
                &grids[2],
            )
            .unwrap();
            check_decompress(&rust, compressor.clone());
            check_decompress(&data[..cache_data::HEADER_SIZE], compressor.clone());
            let foreign = check_endian(&data);
            assert_eq!(check_endian(&foreign), data);
            check_decompress(&foreign, compressor.clone());
            for (offset, value) in [(0, 0xdead_beefu32), (4, 27)] {
                let mut invalid = data.clone();
                invalid[offset..offset + 4].copy_from_slice(&value.to_ne_bytes());
                assert_eq!(check_endian(&invalid), invalid);
                check_decompress(&invalid, compressor.clone());
            }
        }
        for status in [
            0,
            IN_PROGRESS | OUT_OF_NODES,
            SUCCESS | BUFFER_TOO_SMALL,
            FAILURE | WRONG_VERSION,
        ] {
            check_build(
                &header,
                &grids,
                FixtureCompressor {
                    compress_status: status,
                    ..FixtureCompressor::default()
                },
            );
        }
        for report in [0, header.grid_size() * 3 + 5] {
            let compressor = FixtureCompressor {
                report: Some(report),
                ..FixtureCompressor::default()
            };
            let data = check_build(&header, &grids, compressor.clone()).unwrap();
            check_decompress(&data, compressor);
        }
    }
}

#[test]
fn cache_layer_byte_apis_check_truncation_storage_and_callback_sizes() {
    let header = cache_data::Header {
        width: 2,
        height: 2,
        ..cache_data::Header::default()
    };
    let mut compressor = FixtureCompressor::default();
    let mut short = header.to_bytes()[..55].to_vec();
    let original = short.clone();
    assert_eq!(
        cache_data::decompress_layer(&mut compressor, &short).unwrap_err(),
        FAILURE | INVALID_PARAM
    );
    assert!(!cache_data::swap_header_endian(&mut short));
    assert_eq!(short, original);
    assert!(compressor.log.is_empty());
    assert_eq!(
        cache_data::build_layer(&mut compressor, &header, &[0; 3], &[0; 4], &[0; 4]).unwrap_err(),
        FAILURE | INVALID_PARAM
    );
    assert!(compressor.log.is_empty());
    compressor.extra = i32::MAX as usize;
    assert_eq!(
        cache_data::build_layer(&mut compressor, &header, &[0; 4], &[0; 4], &[0; 4]).unwrap_err(),
        FAILURE | INVALID_PARAM
    );
    let mut compressor = FixtureCompressor {
        report: Some(usize::MAX),
        ..FixtureCompressor::default()
    };
    assert_eq!(
        cache_data::build_layer(&mut compressor, &header, &[0; 4], &[0; 4], &[0; 4]).unwrap_err(),
        FAILURE | INVALID_PARAM
    );
}

fn check_mesh(layer: &HeightfieldLayer, climb: i32, error: f32) {
    let mut input = format!("mesh {} {} ", layer.width, layer.height);
    numbers(&mut input, layer.bounds.into_iter().flatten());
    numbers(&mut input, [layer.cell_size, layer.cell_height]);
    write!(input, "{climb} {} ", error.to_bits()).unwrap();
    for v in layer
        .heights
        .iter()
        .chain(&layer.areas)
        .chain(&layer.connections)
    {
        write!(input, "{v} ").unwrap();
    }
    let result = build_cache_mesh(layer, climb, error);
    let actual = match result {
        Ok(b) => json!([
            SUCCESS,
            6,
            b.region_count,
            b.regions,
            b.contours
                .iter()
                .map(|c| json!([c.region, c.area, c.vertices]))
                .collect::<Vec<_>>(),
            b.mesh.vertices,
            b.mesh.polygons,
            b.mesh.areas,
            b.mesh.flags
        ]),
        Err(e) => json!([e.status, null]),
    };
    assert_eq!(
        actual,
        oracle(input),
        "native cache builder {}x{}, climb={climb}, error={error}",
        layer.width,
        layer.height
    );
}
#[test]
fn native_cache_builder_signed_climb_empty_layers_and_simplification() {
    // Narrow and asymmetric grids exercise contour scans in both directions
    // and cyclic output rotation through the first/last vertex.
    for (w, h) in [(0, 0), (0, 7), (1, 1), (3, 7), (7, 3), (11, 9), (17, 11)] {
        let mut layer = field(w, h, 0.3, 0.2);
        for z in 0..h {
            for x in 0..w {
                let i = x + z * w;
                layer.heights[i] = ((x / 3 + z / 3) % 3) as u8;
                layer.areas[i] = if x > 2 && z > 2 && (x * 3 + z * 5) % 7 == 0 {
                    0
                } else {
                    63
                };
                layer.connections[i] = 0;
                for (dir, dx, dz) in [(0, -1, 0), (1, 0, 1), (2, 1, 0), (3, 0, -1)] {
                    let nx = x as isize + dx;
                    let nz = z as isize + dz;
                    if nx >= 0 && nz >= 0 && nx < w as isize && nz < h as isize {
                        layer.connections[i] |= 1 << dir;
                    } else {
                        layer.connections[i] |= 1 << (dir + 4);
                    }
                }
            }
        }
        // Native's one-cell active contour reaches a modulo-zero path in its
        // simplifier. An empty one-cell layer has a defined empty mesh instead.
        if w * h == 1 {
            layer.areas.fill(0);
        }
        for climb in [-1, 0, 2, 65536, i32::MAX] {
            for error in [0., 0.5, 1.3, 4., f32::NAN] {
                check_mesh(&layer, climb, error);
            }
        }
    }
    let mut layer = field(33, 17, 1., 1.);
    layer.heights.fill(0);
    layer.connections.fill(0);
    for z in 0..layer.height {
        for x in 0..layer.width {
            layer.areas[x + z * layer.width] = if (x + z) % 2 == 0 { 63 } else { 0 };
        }
    }
    check_mesh(&layer, 0, 0.);
}

fn check_polygon_storage(contours: &[CacheContour]) {
    use recast_nav::{ContourSet, PolygonNavmesh, TileCacheContourView, TileCachePolygonMesh};
    let views = contours.iter().map(|c| TileCacheContourView {
        vertices: &c.vertices,
        region: c.region,
        area: c.area,
    });
    let mut owned = PolygonNavmesh::default();
    let owned_result =
        ContourSet::build_tile_cache_polygons_from_contours(views.clone(), &mut owned);
    let previous = Some(TileCachePolygonMesh {
        polygons: vec![[37; 12]],
        ..Default::default()
    });
    let mut rows = previous.clone();
    let rows_result = ContourSet::build_tile_cache_rows_from_contours(views, &mut rows);
    assert_eq!(
        rows_result.map_err(|e| e.to_string()),
        owned_result.map_err(|e| e.to_string())
    );
    if owned.max_vertices_per_polygon != 6 {
        assert_eq!(rows, previous, "early errors retain existing output");
        return;
    }
    let rows = rows.unwrap();
    assert_eq!(rows.vertices, owned.vertices);
    assert_eq!(rows.areas, owned.areas);
    assert_eq!(rows.flags, owned.flags);
    for (range, expected) in [(0..6, owned.polygons), (6..12, owned.polygon_neighbors)] {
        assert!(
            rows.polygons
                .iter()
                .flat_map(|p| p[range.clone()].iter().copied())
                .eq(expected),
            "native row storage retains all live and partial polygon words"
        );
    }
}

fn check_polygons(contours: &[CacheContour]) -> Result<CacheMesh, CacheBuildError> {
    check_polygon_storage(contours);
    let mut input = format!("poly {} ", contours.len());
    for c in contours {
        write!(input, "{} {} {} ", c.vertices.len(), c.region, c.area).unwrap();
        for v in c.vertices.iter().flatten() {
            write!(input, "{v} ").unwrap();
        }
    }
    let result = build_polygon_mesh(contours);
    let actual = match &result {
        Ok(m) => json!([SUCCESS, m.vertices, m.polygons, m.areas, m.flags]),
        Err(e) => json!([e.status, null]),
    };
    let partial_input = input.replacen("poly ", "poly_into ", 1);
    let expected = oracle(input);
    if result.is_err() {
        let mut partial = CacheMesh::default();
        let status =
            build_polygon_mesh_into(contours, &mut partial).map_or_else(|e| e.status, |()| SUCCESS);
        assert_eq!(
            json!([
                status,
                partial.vertices,
                partial.polygons,
                partial.areas,
                partial.flags
            ]),
            oracle(partial_input),
            "native initialized partial polygon mesh"
        );
    }
    assert_eq!(
        actual[0],
        expected[0],
        "native cache polygon status for {} contours",
        contours.len()
    );
    assert!(
        actual == expected,
        "native cache polygon builder mismatch for {} contours",
        contours.len()
    );
    result
}
#[test]
fn native_cache_polygon_builder_strict_triangulation_removal_and_vertex_domain() {
    // Convex rings start with 16/17 triangles, crossing the merge search's
    // cached-prefix boundary before its first mutation.
    for n in [18, 19] {
        let mesh = check_polygons(&[CacheContour {
            region: 7,
            area: 63,
            vertices: (0..n)
                .map(|i| {
                    let angle = -(i as f32) * std::f32::consts::TAU / n as f32;
                    [
                        (128. + 100. * angle.cos()).round() as u8,
                        0,
                        (128. + 100. * angle.sin()).round() as u8,
                        15,
                    ]
                })
                .collect(),
        }])
        .unwrap();
        assert_eq!(mesh.vertices.len(), n);
    }
    for shape in [
        vec![
            [1, 0, 0, 128],
            [0, 0, 0, 15],
            [0, 0, 2, 15],
            [2, 0, 2, 15],
            [2, 0, 0, 15],
        ],
        vec![[0, 0, 0, 15], [2, 0, 2, 15], [0, 0, 2, 15], [2, 0, 0, 15]],
        vec![
            [5, 0, 4, 15],
            [1, 0, 4, 15],
            [6, 0, 1, 15],
            [6, 0, 7, 15],
            [2, 0, 5, 15],
        ],
    ] {
        check_polygons(&[CacheContour {
            region: 7,
            area: 3,
            vertices: shape,
        }])
        .unwrap();
    }
    let mut fan: Vec<_> = (0..13)
        .map(|i| CacheContour {
            region: i,
            area: 3,
            vertices: std::iter::once([128, 0, 128, 15])
                .chain((0..=4).map(|j| {
                    let angle = -(i as f32 * 4. + j as f32) * std::f32::consts::TAU / 52.;
                    [
                        (128. + 120. * angle.cos()).round() as u8,
                        0,
                        (128. + 120. * angle.sin()).round() as u8,
                        15,
                    ]
                }))
                .collect(),
        })
        .collect();
    let original = check_polygons(&fan).unwrap();
    let mut incident = std::collections::BTreeMap::new();
    let mut touched = 0;
    let mut remaining = 0;
    for p in &original.polygons {
        let n = p[..6].iter().take_while(|&&v| v != u16::MAX).count();
        if p[..n].contains(&0) {
            touched += 1;
            remaining += n - 2;
        }
        for j in 0..n {
            let k = if j == 0 { n - 1 } else { j - 1 };
            if p[j] == 0 || p[k] == 0 {
                *incident
                    .entry(if p[j] == 0 { p[k] } else { p[j] })
                    .or_insert(0) += 1;
            }
        }
    }
    assert!(
        touched * 2 <= 48
            && incident.len() <= 16
            && incident.values().all(|&n| n == 2)
            && remaining > 48,
        "fixture must reach the native removal limit before any undefined scratch access"
    );
    for c in &mut fan {
        c.vertices[0][3] |= 128;
    }
    // Native rejects this valid fan at its fixed removal-edge limit.
    // The first call above verifies the exact pre-removal mesh and bounds.
    assert_eq!(
        check_polygons(&fan).unwrap_err().status,
        FAILURE | BUFFER_TOO_SMALL
    );
    // Cache has no Recast-style 0xfffe summed-contour cutoff. Every actual
    // mesh index remains representable, including the last valid u16 index.
    let mut contours: Vec<_> = (0..21845)
        .map(|i| CacheContour {
            region: (i % 255) as u8,
            area: 63,
            vertices: (0..3)
                .map(|j| {
                    let n = i * 3 + j;
                    [(n % 256) as u8, 0, (n / 256) as u8, 15]
                })
                .collect(),
        })
        .collect();
    let mesh = check_polygons(&contours).unwrap();
    assert_eq!(mesh.vertices.len(), 65535);
    assert_eq!(mesh.polygons.len(), 21845);
    // The next contour's first welded index becomes the ushort sentinel.
    // Both triangles then have empty live prefixes during merge scoring.
    // Distinct hash buckets avoid the independent cyclic-chain domain.
    let mut empty_prefixes = contours.clone();
    empty_prefixes.push(CacheContour {
        region: 7,
        area: 63,
        vertices: vec![
            [250, 3, 250, 15],
            [250, 3, 251, 15],
            [251, 3, 251, 15],
            [251, 3, 250, 15],
        ],
    });
    let mesh = check_polygons(&empty_prefixes).unwrap();
    assert_eq!(mesh.polygons.len(), 21847);
    assert!(mesh.polygons[21845..].iter().all(|p| p[0] == u16::MAX));
    contours.push(CacheContour {
        region: 7,
        area: 63,
        vertices: vec![[255, 0, 255, 15], [254, 0, 255, 15], [253, 0, 255, 15]],
    });
    let mesh = check_polygons(&contours).unwrap();
    assert_eq!(mesh.vertices.len(), 65536);
    assert_eq!(mesh.polygons.last().unwrap()[0], u16::MAX);
    // Inserting index 0xffff clears its bucket in the native 256-entry table.
    // (0,162) collides with (255,255) at eight hash bits, but not twelve.
    // Its next lookup therefore creates a wrapped vertex instead of welding.
    let mut cleared_bucket = contours.clone();
    cleared_bucket.push(CacheContour {
        region: 7,
        area: 63,
        vertices: vec![[0, 0, 162, 15], [1, 0, 0, 15], [2, 0, 0, 15]],
    });
    let mesh = check_polygons(&cleared_bucket).unwrap();
    assert_eq!(mesh.vertices.len(), 65537);
    assert_eq!(mesh.vertices[0], [0, 0, 162]);
    contours.push(CacheContour {
        region: 7,
        area: 63,
        vertices: vec![[0, 3, 0, 15], [1, 0, 0, 15], [2, 0, 0, 15]],
    });
    let mesh = check_polygons(&contours).unwrap();
    assert_eq!(mesh.vertices.len(), 65537);
    assert_eq!(mesh.vertices[0], [0, 3, 0]);
    assert_eq!(mesh.vertices[65536], [0, 0, 0]);
    // A repeated late triangle performs an adjacency lookup after the native
    // edge-bucket heads have crossed the ushort index/sentinel boundary.
    contours.push(CacheContour {
        region: 7,
        area: 63,
        vertices: vec![[252, 0, 255, 15], [253, 0, 255, 15], [254, 0, 255, 15]],
    });
    let mesh = check_polygons(&contours).unwrap();
    assert_eq!(mesh.polygons.len(), 21848);
    // A further lookup of the overwritten zero-height point would make the
    // native vertex chain cycle forever. Rust checks that nonterminating domain.
    contours.push(CacheContour {
        region: 7,
        area: 63,
        vertices: vec![[0, 0, 0, 15], [1, 0, 0, 15], [2, 0, 0, 15]],
    });
    assert!(build_polygon_mesh(&contours).is_err());
    check_polygon_storage(&contours);
    // Polygon counts remain int while adjacency records retain ushort polygon
    // indices. Repeated points avoid the independent vertex-chain overflow.
    let repeated = vec![
        CacheContour {
            region: 7,
            area: 63,
            vertices: vec![[0, 0, 0, 15], [1, 0, 0, 15], [2, 0, 0, 15]],
        };
        65537
    ];
    assert_eq!(check_polygons(&repeated).unwrap().polygons.len(), 65537);
    for n in [32768, 32769] {
        let mut points = vec![[0, 0, 0, 15]; n];
        points[n - 1] = [1, 0, 1, 15];
        let mesh = check_polygons(&[CacheContour {
            region: 7,
            area: 63,
            vertices: points,
        }])
        .unwrap();
        assert_eq!(mesh.vertices.len(), 2);
        assert!(mesh.polygons.is_empty());
    }
}

fn check_stage(
    layer: &HeightfieldLayer,
    mut regions: Vec<u8>,
    mut count: u8,
    climb: i32,
    error: f32,
    contour: bool,
) -> (u32, Vec<CacheContour>) {
    let mut input = format!(
        "stages {} {} {} {climb} ",
        if contour { "C" } else { "R" },
        layer.width,
        layer.height
    );
    numbers(&mut input, [error]);
    write!(input, "{count} ").unwrap();
    for a in [&layer.heights, &layer.areas, &layer.connections, &regions] {
        write!(input, "{} ", a.len()).unwrap();
        for b in a {
            write!(input, "{b} ").unwrap();
        }
    }
    let mut contours = vec![CacheContour {
        region: 17,
        area: 21,
        vertices: vec![[1, 2, 3, 4]],
    }];
    let result = if contour {
        build_contours_into(layer, &regions, count, climb, error, &mut contours)
    } else {
        build_regions_into(layer, climb, &mut regions, &mut count)
    };
    let status = result.map_or_else(|e| e.status, |()| SUCCESS);
    let actual = json!([
        status,
        count,
        regions,
        if contour {
            Some(
                contours
                    .iter()
                    .map(|c| json!([c.region, c.area, c.vertices]))
                    .collect::<Vec<_>>(),
            )
        } else {
            None
        }
    ]);
    assert_eq!(
        actual,
        oracle(input),
        "native cache stage contour={contour} grid={}x{}",
        layer.width,
        layer.height
    );
    (status, contours)
}
#[test]
fn native_cache_independent_regions_contours_and_partial_failures() {
    // Empty cells break westward sweeps, and no row may inherit its west
    // neighbor from the preceding row. Height extremes cover signed climb.
    for (w, h) in [(1, 7), (7, 1), (2, 6), (255, 2)] {
        let mut layer = field(w, h, 1., 1.);
        layer.connections.clear();
        for z in 0..h {
            for x in 0..w {
                let i = x + z * w;
                layer.areas[i] = [63, 63, 0, 1, 1, 0, 63][(x + z * 2) % 7];
                layer.heights[i] = [0, 255, 1, 254][(x + z) % 4];
            }
        }
        for climb in [i32::MIN, -1, 0, 1, 254, 255, i32::MAX] {
            check_stage(&layer, vec![47; w * h + 3], 73, climb, 0., false);
        }
    }
    for (w, h) in [(0, 7), (11, 9), (33, 17), (255, 2)] {
        let mut layer = field(w, h, 0.3, 0.2);
        layer.connections.clear();
        layer.heights.fill(0);
        for z in 0..h {
            for x in 0..w {
                layer.areas[x + z * w] = if (x + z) % 2 == 0 { 63 } else { 0 };
            }
        }
        // Connections are not a region-builder input. Overflow retains the
        // previous count and the partially assigned labels; tails stay intact.
        for climb in [-1, 0, 65536] {
            check_stage(&layer, vec![47; w * h + 3], 73, climb, 0., false);
        }
    }
    // The top/bottom bars each meet sixteen distinct monotone tooth regions:
    // this fills the native neighbor array exactly without entering its overflow.
    let mut comb = field(31, 3, 1., 1.);
    comb.heights.fill(0);
    comb.connections.clear();
    for z in 0..3 {
        for x in 0..31 {
            comb.areas[x + z * 31] = if z != 1 || x % 2 == 0 { 63 } else { 0 };
        }
    }
    check_stage(&comb, vec![47; 31 * 3 + 3], 73, 0, 0., false);
    // Exercise every scratch sweep/region slot, plus the maximum area of one
    // region. No neighboring region list approaches its native 16-entry cap.
    for (w, h) in [(255, 1), (255, 255)] {
        let mut layer = field(w, h, 1., 1.);
        layer.heights.fill(0);
        layer.connections.clear();
        for (i, area) in layer.areas.iter_mut().enumerate() {
            *area = if h == 1 { (i % 2 + 1) as u8 } else { 63 };
        }
        check_stage(&layer, vec![47; w * h + 3], 73, 0, 0., false);
    }
    for (w, h, complex) in [
        (11, 9, false),
        (37, 31, true),
        (64, 64, true),
        (65, 64, true),
        (255, 255, true),
    ] {
        let mut layer = field(w, h, 0.3, 0.2);
        let mut regions = vec![255; w * h + 3];
        layer.connections.fill(0);
        layer.heights.fill(0);
        for z in 0..h {
            for x in 0..w {
                let active = if complex {
                    x < 3 || (x >= 5 && (x == 5 || z % 3 == 0 || (z % 3 == 1 && x % 2 == 0)))
                } else {
                    x < 3 || (x > 4 && z > 2)
                };
                let i = x + z * w;
                layer.areas[i] = if active { 63 } else { 0 };
                regions[i] = if !active {
                    255
                } else if x < 3 {
                    0
                } else {
                    2
                };
                for (d, dx, dz) in [(0, -1, 0), (1, 0, 1), (2, 1, 0), (3, 0, -1)] {
                    if x as isize + dx >= 0
                        && (x as isize + dx) < w as isize
                        && z as isize + dz >= 0
                        && (z as isize + dz) < h as isize
                    {
                        layer.connections[i] |= 1 << d;
                    }
                }
            }
        }
        // The maximum grid reaches the 2040-vertex scratch limit. Existing
        // smaller cases cover signed climb and exceptional error arithmetic.
        let climbs: &[i32] = if w == 255 { &[0] } else { &[-1, 0, 65536] };
        let errors: &[f32] = if w == 255 {
            &[1.3]
        } else {
            &[0., 1.3, f32::NAN]
        };
        for &climb in climbs {
            for &error in errors {
                let (status, contours) =
                    check_stage(&layer, regions.clone(), 4, climb, error, true);
                if complex {
                    assert_eq!(status, FAILURE | BUFFER_TOO_SMALL);
                    assert!(!contours[0].vertices.is_empty());
                    assert_eq!(contours[2].area, 63);
                    assert!(contours[2].vertices.is_empty());
                }
            }
        }
    }
}

#[test]
fn native_cache_contours_retain_defined_flat_grid_row_crossings() {
    for (w, h, wrap_rows) in [(5, 5, &[1][..]), (5, 5, &[2, 4][..]), (7, 3, &[1, 2][..])] {
        let mut layer = field(w, h, 1., 1.);
        if wrap_rows == [1] {
            layer.heights.fill(0);
        }
        layer.areas.fill(63);
        layer.connections.fill(0);
        for z in 0..h as isize {
            for x in 0..w as isize {
                for (d, dx, dz) in [(0, -1, 0), (1, 0, 1), (2, 1, 0), (3, 0, -1)] {
                    if (0..w as isize).contains(&(x + dx)) && (0..h as isize).contains(&(z + dz)) {
                        layer.connections[x as usize + z as usize * w] |= 1 << d;
                    }
                }
            }
        }
        // West from the first cell of a row and east from its predecessor
        // stay inside the native flat buffer. Native contour coordinates can
        // become negative while indices stay valid, including with heights.
        for &row in wrap_rows {
            layer.connections[row * w] |= 1;
            layer.connections[row * w - 1] |= 4;
        }
        check_stage(&layer, vec![0; w * h], 1, 0, 0., true);
    }
    // Every directed connection stays in the flat buffer. The walk reaches
    // the iteration limit with repeated upper-right extrema; native keeps the
    // first extremum when seeding simplification.
    let mut layer = field(5, 5, 1., 1.);
    layer.areas.fill(63);
    layer.connections = vec![
        6, 4, 7, 0, 1, 5, 12, 12, 0, 8, 1, 13, 8, 6, 0, 15, 12, 0, 4, 1, 13, 5, 1, 13, 9,
    ];
    for error in [0., 1.3] {
        check_stage(&layer, vec![0; 25], 1, 0, error, true);
    }
}
