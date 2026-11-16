//! Live differential tests: identical geometry/configuration, independently executed C++ and Rust.
use recast_nav::{BuildConfig, BuildStage, TriangleMesh, build_tile_observed_with_context};
use std::{
    cell::RefCell,
    collections::BTreeMap,
    fmt::Write as _,
    io::Write,
    process::{Command, Stdio},
};

type Snapshots = BTreeMap<String, Vec<i64>>;

#[derive(Default)]
struct RecordingContext(RefCell<Vec<i64>>);
impl recast_nav::recast::ContextHooks for RecordingContext {
    fn log(&self, category: recast_nav::recast::LogCategory, message: &[u8]) {
        let mut events = self.0.borrow_mut();
        events.extend([1, category.0 as i64, message.len() as i64]);
        events.extend(message.iter().map(|&b| i64::from(b)));
    }
    fn start_timer(&self, label: recast_nav::recast::TimerLabel) {
        self.0.borrow_mut().extend([3, label.0 as i64]);
    }
    fn stop_timer(&self, label: recast_nav::recast::TimerLabel) {
        self.0.borrow_mut().extend([4, label.0 as i64]);
    }
}

fn bounds(out: &mut Vec<i64>, aabb: recast_nav::Aabb3d, cs: f32, ch: f32) {
    out.extend(
        aabb.min
            .to_array()
            .into_iter()
            .chain(aabb.max.to_array())
            .chain([cs, ch])
            .map(|f| i64::from(f.to_bits())),
    );
}

fn snapshot(stage: BuildStage<'_>) -> (&'static str, Vec<i64>) {
    let mut out = Vec::new();
    let name = match stage {
        BuildStage::Layers(layers) => {
            out.push(layers.len() as i64);
            for l in layers {
                out.extend([
                    l.width as i64,
                    l.height as i64,
                    l.min_x as i64,
                    l.max_x as i64,
                    l.min_z as i64,
                    l.max_z as i64,
                    l.hmin.into(),
                    l.hmax.into(),
                ]);
                let aabb = recast_nav::Aabb3d {
                    min: l.bounds[0].into(),
                    max: l.bounds[1].into(),
                };
                bounds(&mut out, aabb, l.cell_size, l.cell_height);
                out.extend(
                    l.heights
                        .iter()
                        .chain(&l.areas)
                        .chain(&l.connections)
                        .map(|&v| i64::from(v)),
                );
            }
            "layers"
        }
        BuildStage::TriangleAreas(areas) => {
            out.extend(areas.iter().map(|a| i64::from(a.0)));
            "triangle_areas"
        }
        BuildStage::Heightfield(name, hf) => {
            out.extend([i64::from(hf.width), hf.height.into()]);
            bounds(&mut out, hf.aabb, hf.cell_size, hf.cell_height);
            for &first in &hf.spans {
                let mut spans = Vec::new();
                let mut key = first;
                while let Some(k) = key {
                    let span = hf.span(k);
                    spans.extend([
                        i64::from(span.min()),
                        span.max().into(),
                        span.area().0.into(),
                    ]);
                    key = span.next;
                }
                out.push((spans.len() / 3) as i64);
                out.extend(spans);
            }
            name
        }
        BuildStage::Compact(name, hf) => {
            out.extend([
                i64::from(hf.width),
                hf.height.into(),
                hf.walkable_height.into(),
                hf.walkable_climb.into(),
                hf.border_size.into(),
                hf.max_distance.into(),
                hf.max_region.bits().into(),
                hf.spans.len() as i64,
            ]);
            bounds(&mut out, hf.aabb, hf.cell_size, hf.cell_height);
            for c in &hf.cells {
                out.extend([i64::from(c.index()), c.count().into()]);
            }
            for (s, a) in hf.spans.iter().zip(&hf.areas) {
                out.extend([i64::from(s.y), s.height().into(), s.region.bits().into()]);
                for dir in 0..4 {
                    out.push(s.con(dir).unwrap_or(63).into());
                }
                out.push(a.0.into());
            }
            out.push(hf.dist.len() as i64);
            out.extend(hf.dist.iter().map(|&d| i64::from(d)));
            name
        }
        BuildStage::Contours(set) => {
            out.extend([
                i64::from(set.width),
                set.height.into(),
                set.border_size.into(),
                set.max_error.to_bits().into(),
                set.contours.len() as i64,
            ]);
            bounds(&mut out, set.aabb, set.cell_size, set.cell_height);
            for c in &set.contours {
                out.extend([
                    c.region.bits().into(),
                    c.area.0.into(),
                    c.vertices.len() as i64,
                    c.raw_vertices.len() as i64,
                ]);
                for (v, flags) in &c.vertices {
                    out.extend([i64::from(v.x), v.y.into(), v.z.into(), (*flags).into()]);
                }
                for (v, flags) in &c.raw_vertices {
                    out.extend([i64::from(v.x), v.y.into(), v.z.into(), flags.bits().into()]);
                }
            }
            "contours"
        }
        BuildStage::Polygons(mesh) => {
            out.extend([
                mesh.vertices.len() as i64,
                mesh.polygon_count() as i64,
                mesh.max_vertices_per_polygon.into(),
                mesh.border_size.into(),
                mesh.max_edge_error.to_bits().into(),
            ]);
            bounds(&mut out, mesh.aabb, mesh.cell_size, mesh.cell_height);
            out.extend(
                mesh.vertices
                    .iter()
                    .flat_map(|v| v.to_array())
                    .map(i64::from),
            );
            out.extend(
                mesh.polygons
                    .iter()
                    .chain(&mesh.polygon_neighbors)
                    .map(|&v| i64::from(v)),
            );
            for i in 0..mesh.polygon_count() {
                out.extend([
                    i64::from(mesh.regions[i].bits()),
                    mesh.areas[i].0.into(),
                    mesh.flags[i].into(),
                ]);
            }
            out.push(mesh.max_polygons as i64);
            let nvp = mesh.max_vertices_per_polygon as usize;
            if nvp > 0 {
                for (p, n) in mesh
                    .unused_polygons
                    .chunks(nvp)
                    .zip(mesh.unused_polygon_neighbors.chunks(nvp))
                {
                    out.extend(p.iter().chain(n).map(|&v| i64::from(v)));
                }
            }
            for (r, a) in mesh.unused_regions.iter().zip(&mesh.unused_areas) {
                out.extend([i64::from(r.bits()), i64::from(a.0)]);
            }
            "polygons"
        }
        BuildStage::Detail(mesh) => {
            out.extend([
                mesh.meshes.len() as i64,
                mesh.vertices.len() as i64,
                mesh.triangles.len() as i64,
            ]);
            for m in &mesh.meshes {
                out.extend([
                    i64::from(m.base_vertex_index),
                    m.vertex_count.into(),
                    m.base_triangle_index.into(),
                    m.triangle_count.into(),
                ]);
            }
            out.extend(
                mesh.vertices
                    .iter()
                    .flat_map(|v| v.to_array())
                    .map(|f| i64::from(f.to_bits())),
            );
            for (t, flag) in mesh.triangles.iter().zip(&mesh.triangle_flags) {
                out.extend([i64::from(t[0]), t[1].into(), t[2].into(), (*flag).into()]);
            }
            "detail"
        }
    };
    (name, out)
}

fn reference(mesh: &TriangleMesh, c: &BuildConfig) -> Snapshots {
    let mut input = String::new();
    for v in c.bounds.iter().flatten() {
        write!(input, "{v} ").unwrap();
    }
    writeln!(
        input,
        "{} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
        c.cell_size,
        c.cell_height,
        c.slope_degrees,
        c.walkable_height,
        c.walkable_climb,
        c.walkable_radius,
        c.border_size,
        c.min_region_area,
        c.merge_region_area,
        c.max_edge_len,
        c.max_simplification_error,
        c.max_vertices_per_polygon,
        c.detail_sample_dist,
        c.detail_sample_max_error,
        c.partitioning as u8,
        c.contour_flags.bits(),
        mesh.vertices.len(),
        mesh.triangles.len()
    )
    .unwrap();
    for v in mesh.vertices.iter().flatten() {
        write!(input, "{v} ").unwrap();
    }
    for i in mesh.triangles.iter().flatten() {
        write!(input, "{i} ").unwrap();
    }
    let mut child = Command::new(env!("RECAST_CPP_ORACLE"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    child
        .stdin
        .take()
        .unwrap()
        .write_all(input.as_bytes())
        .unwrap();
    let output = child.wait_with_output().unwrap();
    assert!(
        output.status.success(),
        "C++ oracle: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    serde_json::from_slice(&output.stdout).unwrap()
}

fn compare(name: &str, mesh: TriangleMesh, config: BuildConfig) {
    use recast_nav::RegionPartitioning;
    for partitioning in [
        RegionPartitioning::Watershed,
        RegionPartitioning::Monotone,
        RegionPartitioning::Layers,
    ] {
        let mut config = config.clone();
        config.partitioning = partitioning;
        compare_partition(&format!("{name}-{partitioning:?}"), mesh.clone(), config);
    }
}
fn compare_partition(name: &str, mesh: TriangleMesh, config: BuildConfig) {
    let mut cpp = reference(&mesh, &config);
    let mut stages = 0;
    let hooks = RecordingContext::default();
    let context = recast_nav::recast::Context::with_hooks(true, &hooks);
    let tile = build_tile_observed_with_context(&context, &mesh, &config, |stage| {
        let (stage, rust) = snapshot(stage);
        let expected = cpp.remove(stage).expect("stage present in C++ oracle");
        let first_difference = rust.iter().zip(&expected).position(|(a, b)| a != b);
        if rust != expected {
            let directory=std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/parity-failures");
            std::fs::create_dir_all(&directory).unwrap();
            let fixture=name.replace(' ',"-");
            std::fs::write(directory.join(format!("{fixture}-{stage}-rust.json")),serde_json::to_vec(&rust).unwrap()).unwrap();
            std::fs::write(directory.join(format!("{fixture}-{stage}-cpp.json")),serde_json::to_vec(&expected).unwrap()).unwrap();
        }
        assert!(rust == expected, "{name}: {stage} differs at {first_difference:?}; lengths Rust={} C++={}; values Rust={:?} C++={:?}", rust.len(), expected.len(), first_difference.and_then(|i| rust.get(i)), first_difference.and_then(|i| expected.get(i)));
        stages += 1;
    }).unwrap_or_else(|e| panic!("{name}: Rust pipeline failed: {e}"));
    assert_eq!(
        *hooks.0.borrow(),
        cpp.remove("context").unwrap(),
        "{name}: native context callback order"
    );
    for (index, layer) in tile.layers.iter().enumerate() {
        if layer.width > 255 || layer.height > 255 {
            continue;
        }
        for blocked in [false, true] {
            let mut layer = layer.clone();
            if blocked {
                recast_nav::cache::mark_box(&mut layer, [6., -1., 6.], [10., 8., 10.]);
            }
            let build =
                recast_nav::cache::build_cache_mesh(&layer, config.walkable_climb, 0.).unwrap();
            let prefix = format!("cache/{index}/{}", if blocked { "box" } else { "base" });
            let mut rs = vec![i64::from(build.region_count)];
            rs.extend(
                layer
                    .areas
                    .iter()
                    .chain(&build.regions)
                    .map(|&v| i64::from(v)),
            );
            let mut ct = vec![build.contours.len() as i64];
            for c in &build.contours {
                ct.extend([i64::from(c.region), c.area.into(), c.vertices.len() as i64]);
                ct.extend(c.vertices.iter().flatten().map(|&v| i64::from(v)));
            }
            let mut pm = vec![
                build.mesh.vertices.len() as i64,
                build.mesh.polygons.len() as i64,
                6,
            ];
            pm.extend(
                build
                    .mesh
                    .vertices
                    .iter()
                    .flatten()
                    .chain(build.mesh.polygons.iter().flatten())
                    .map(|&v| i64::from(v)),
            );
            pm.extend(build.mesh.areas.iter().map(|&v| i64::from(v)));
            for (stage, rust) in [("regions", rs), ("contours", ct), ("polygons", pm)] {
                let stage = format!("{prefix}/{stage}");
                let expected = cpp.remove(&stage).unwrap();
                let first = rust.iter().zip(&expected).position(|(a, b)| a != b);
                assert!(
                    rust == expected,
                    "{name}: {stage} differs at {first:?}: Rust={:?} C++={:?}, lengths {} vs {}",
                    first.and_then(|i| rust.get(i)),
                    first.and_then(|i| expected.get(i)),
                    rust.len(),
                    expected.len()
                );
            }
        }
    }
    assert!(cpp.is_empty(), "uncompared C++ stages: {:?}", cpp.keys());
    assert_eq!(stages, 13, "every expected stage must execute");
}

fn empty_mesh() -> TriangleMesh {
    TriangleMesh {
        vertices: vec![],
        triangles: vec![],
    }
}

fn quad(mesh: &mut TriangleMesh, corners: [[f32; 3]; 4]) {
    let n = mesh.vertices.len() as u32;
    mesh.vertices.extend(corners);
    mesh.triangles
        .extend([[n, n + 1, n + 2], [n, n + 2, n + 3]]);
}

fn floor(mesh: &mut TriangleMesh, x: [f32; 2], z: [f32; 2], y: f32) {
    quad(
        mesh,
        [
            [x[0], y, z[0]],
            [x[0], y, z[1]],
            [x[1], y, z[1]],
            [x[1], y, z[0]],
        ],
    );
}

#[test]
fn geometry_corpus_matches_cpp_at_every_stage() {
    let c = BuildConfig::default();
    let mut mesh = empty_mesh();
    floor(&mut mesh, [0., 16.], [0., 16.], 0.);
    compare("flat", mesh.clone(), c.clone());

    let mut multiple_holes = mesh.clone();
    for (x, z) in [(3., 3.), (10., 10.)] {
        quad(
            &mut multiple_holes,
            [
                [x, 1.5, z],
                [x + 3., 1.5, z],
                [x + 3., 1.5, z + 3.],
                [x, 1.5, z + 3.],
            ],
        );
    }
    compare("multiple holes", multiple_holes, c.clone());

    let mut obstacle = mesh.clone();
    floor(&mut obstacle, [6., 10.], [6., 10.], 3.);
    quad(
        &mut obstacle,
        [[6., 0., 6.], [10., 0., 6.], [10., 3., 6.], [6., 3., 6.]],
    );
    quad(
        &mut obstacle,
        [[10., 0., 10.], [6., 0., 10.], [6., 3., 10.], [10., 3., 10.]],
    );
    quad(
        &mut obstacle,
        [[6., 0., 10.], [6., 0., 6.], [6., 3., 6.], [6., 3., 10.]],
    );
    quad(
        &mut obstacle,
        [[10., 0., 6.], [10., 0., 10.], [10., 3., 10.], [10., 3., 6.]],
    );
    compare("solid box", obstacle.clone(), c.clone());
    compare(
        "region pruning",
        obstacle,
        BuildConfig {
            min_region_area: 64,
            merge_region_area: 400,
            ..c.clone()
        },
    );

    for degrees in [44.9_f32, 45., 45.1] {
        let rise = 4. * degrees.to_radians().tan();
        let mut slope = empty_mesh();
        quad(
            &mut slope,
            [[0., 0., 0.], [0., rise, 4.], [16., rise, 4.], [16., 0., 0.]],
        );
        compare(&format!("slope {degrees}"), slope, c.clone());
    }

    let mut large = empty_mesh();
    quad(
        &mut large,
        [
            [0., 0., 0.],
            [5., 0., 160.],
            [180., 0., 150.],
            [160., 0., 5.],
        ],
    );
    compare(
        "large contour metrics",
        large,
        BuildConfig {
            bounds: [[0., -1., 0.], [180., 8., 160.]],
            max_edge_len: 512,
            ..c.clone()
        },
    );

    let mut fractional_ramp = empty_mesh();
    quad(
        &mut fractional_ramp,
        [
            [-10., 0., -10.],
            [-10., 0., 10.],
            [10., 4., 10.],
            [10., 4., -10.],
        ],
    );
    compare(
        "fractional ramp",
        fractional_ramp,
        BuildConfig {
            bounds: [[-13.25, -0.25, -13.25], [10.75, 6.95, 10.75]],
            border_size: 4,
            walkable_climb: 4,
            ..c.clone()
        },
    );

    let mut stacked = mesh.clone();
    floor(&mut stacked, [2., 14.], [2., 14.], 4.);
    compare("stacked floors", stacked, c.clone());

    let mut low_ceiling = mesh.clone();
    // Downward-facing ceiling must remain a solid obstruction with a null area.
    quad(
        &mut low_ceiling,
        [
            [4., 1.5, 4.],
            [12., 1.5, 4.],
            [12., 1.5, 12.],
            [4., 1.5, 12.],
        ],
    );
    compare("low ceiling", low_ceiling, c.clone());

    let mut ramp = empty_mesh();
    floor(&mut ramp, [0., 16.], [0., 4.], 0.);
    quad(
        &mut ramp,
        [[0., 0., 4.], [0., 3., 12.], [16., 3., 12.], [16., 0., 4.]],
    );
    floor(&mut ramp, [0., 16.], [12., 16.], 3.);
    compare("ramp", ramp, c.clone());

    let mut cliff = empty_mesh();
    floor(&mut cliff, [0., 8.], [0., 16.], 0.);
    floor(&mut cliff, [8., 16.], [0., 16.], 3.);
    compare("ledge", cliff, c.clone());

    let mut isolated = empty_mesh();
    floor(&mut isolated, [0., 6.], [0., 16.], 0.);
    floor(&mut isolated, [10., 16.], [0., 16.], 0.);
    compare("disconnected islands", isolated, c.clone());

    let mut upside_down = mesh.clone();
    for triangle in &mut upside_down.triangles {
        triangle.swap(1, 2);
    }
    compare("no walkable surfaces", upside_down, c.clone());

    let mut across_tiles = empty_mesh();
    floor(&mut across_tiles, [-8., 40.], [-8., 24.], 0.);
    for (name, x) in [("left tile", -2.), ("right tile", 14.)] {
        compare(
            name,
            across_tiles.clone(),
            BuildConfig {
                bounds: [[x, -1., -2.], [x + 20., 8., 18.]],
                border_size: 4,
                ..c.clone()
            },
        );
    }

    let mut fractional = mesh;
    for v in &mut fractional.vertices {
        v[0] -= 9.125;
        v[2] -= 3.75;
        v[1] += 0.125;
    }
    compare(
        "negative fractional origin",
        fractional,
        BuildConfig {
            bounds: [[-9.125, -0.875, -3.75], [6.875, 8.125, 12.25]],
            ..c
        },
    );
    compare(
        "triangle slope rounding",
        TriangleMesh {
            vertices: vec![
                [-1.954155, 1.1070938, 6.076432],
                [-6.3227806, -5.283146, -4.2008066],
                [-7.653673, 1.5148783, 6.0908804],
            ],
            triangles: vec![[0, 1, 2]],
        },
        BuildConfig {
            bounds: [[-8., -6., -5.], [0., 3., 7.]],
            slope_degrees: 33.23423,
            ..Default::default()
        },
    );
}

#[test]
fn seeded_terrain_matches_cpp_at_every_stage() {
    for seed in 1..=8_u32 {
        let mut random = seed;
        let mut heights = [[0.; 9]; 9];
        for row in &mut heights {
            for h in row {
                random = random.wrapping_mul(1664525).wrapping_add(1013904223);
                *h = ((random >> 24) % 5) as f32 * 0.25;
            }
        }
        let mut mesh = empty_mesh();
        for z in 0..8 {
            for x in 0..8 {
                let xx = x as f32 * 2.;
                let zz = z as f32 * 2.;
                quad(
                    &mut mesh,
                    [
                        [xx, heights[z][x], zz],
                        [xx, heights[z + 1][x], zz + 2.],
                        [xx + 2., heights[z + 1][x + 1], zz + 2.],
                        [xx + 2., heights[z][x + 1], zz],
                    ],
                );
            }
        }
        compare(&format!("seed {seed}"), mesh, BuildConfig::default());
    }
}

#[test]
fn native_generation_configuration_budgets_and_contour_flags() {
    use recast_nav::BuildContoursFlags;
    let mut flat = empty_mesh();
    floor(&mut flat, [0., 16.], [0., 16.], 0.);
    for (minimum, merge) in [(i32::MIN, i32::MIN), (-1, 0), (0, -1), (i32::MAX, i32::MAX)] {
        compare(
            &format!("signed region thresholds {minimum} {merge}"),
            flat.clone(),
            BuildConfig {
                min_region_area: minimum,
                merge_region_area: merge,
                ..Default::default()
            },
        );
    }
    let mut distant_hump = empty_mesh();
    for z in 0..4 {
        for x in 0..4 {
            let point = |x: usize, z: usize| {
                [
                    60000. + x as f32 * 2.,
                    if x == 2 && z == 2 { 1. } else { 0. },
                    z as f32 * 2.,
                ]
            };
            quad(
                &mut distant_hump,
                [
                    point(x, z),
                    point(x, z + 1),
                    point(x + 1, z + 1),
                    point(x + 1, z),
                ],
            );
        }
    }
    let detail_config = BuildConfig {
        bounds: [[0., -1., 0.], [60008., 8., 8.]],
        cell_size: 1.,
        walkable_radius: 0,
        detail_sample_dist: 0.9,
        detail_sample_max_error: 0.1,
        ..Default::default()
    };
    let built = recast_nav::build_tile(&distant_hump, &detail_config).unwrap();
    assert!(
        built.detail.vertices.len() > built.polygons.vertices.len(),
        "fixture must refine the interior high-coordinate samples"
    );
    compare(
        "signed detail sample coordinates beyond 65535",
        distant_hump,
        detail_config,
    );
    for width in [16384., 65537.] {
        let mut small = empty_mesh();
        floor(&mut small, [0., 4.], [0., 4.], 0.);
        compare(
            &format!("wide native grid {width}"),
            small,
            BuildConfig {
                bounds: [[0., -1., 0.], [width, 8., 4.]],
                cell_size: 1.,
                walkable_radius: 0,
                ..Default::default()
            },
        );
    }
    for top in [3000., 30000.] {
        compare(
            &format!("vertical bounds above packed spans {top}"),
            flat.clone(),
            BuildConfig {
                bounds: [[0., -1., 0.], [16., top, 16.]],
                ..Default::default()
            },
        );
    }
    for edge in [i32::MIN, -1, 32768, 46340] {
        compare(
            &format!("signed native max edge {edge}"),
            flat.clone(),
            BuildConfig {
                max_edge_len: edge,
                ..Default::default()
            },
        );
    }
    for (height, climb) in [(-1, 0), (0, -1), (1, 0), (8192, 8192), (65536, 65536)] {
        compare(
            &format!("signed height {height} climb {climb}"),
            flat.clone(),
            BuildConfig {
                walkable_height: height,
                walkable_climb: climb,
                ..Default::default()
            },
        );
    }
    for radius in [-128, -1, 128, 129, 65535] {
        compare(
            &format!("signed native erosion radius {radius}"),
            flat.clone(),
            BuildConfig {
                walkable_radius: radius,
                ..Default::default()
            },
        );
    }
    compare(
        "large minimum region budget",
        flat.clone(),
        BuildConfig {
            min_region_area: 65536,
            ..Default::default()
        },
    );
    compare_partition(
        "contours without tessellation",
        flat.clone(),
        BuildConfig {
            max_edge_len: 2,
            contour_flags: BuildContoursFlags::empty(),
            ..Default::default()
        },
    );
    let points = [
        [4., 0., 0.],
        [0., 0., 4.],
        [0., 0., 12.],
        [4., 0., 16.],
        [12., 0., 16.],
        [16., 0., 12.],
        [16., 0., 4.],
        [12., 0., 0.],
    ];
    let octagon = TriangleMesh {
        vertices: points.to_vec(),
        triangles: (1..7).map(|i| [0, i, i + 1]).collect(),
    };
    let octagon_config = BuildConfig {
        max_vertices_per_polygon: 8,
        max_edge_len: 0,
        walkable_radius: 0,
        max_simplification_error: 0.1,
        ..Default::default()
    };
    let built = recast_nav::build_tile(&octagon, &octagon_config).unwrap();
    assert!(built.polygons.polygons().any(|p| p.count() > 6));
    compare("eight-vertex polygon budget", octagon, octagon_config);
    compare_partition(
        "wide polygon storage budget",
        flat,
        BuildConfig {
            max_vertices_per_polygon: 65536,
            ..Default::default()
        },
    );
}
