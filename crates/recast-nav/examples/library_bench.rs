//! Opt-in, matched native/Rust workloads. See scripts/bench_libraries.py.
use recast_nav::{
    BuildConfig, RegionPartitioning, TriangleMesh, build_tile,
    cache_data::{self, CompressionResult, Compressor},
    crowd::{AgentParams, Crowd},
    nav::{self, Corridor, Filter, NavMesh, Point, StraightPath},
    nav_data::{self, CreatePolygon, OwnedCreateParams},
    query::NavMeshQuery,
    tile_cache::{self, MeshProcess, TileCache},
};
use std::{fs, hint::black_box, path::Path, time::Instant};
#[path = "library_bench/crowd_cases.rs"]
mod crowd_cases;
#[path = "library_bench/extended.rs"]
mod extended;

#[derive(Clone, Copy)]
struct Hash(u64);
impl Hash {
    fn new() -> Self {
        Self(0xcbf29ce484222325)
    }
    fn word(&mut self, value: impl Into<u64>) {
        for b in value.into().to_le_bytes() {
            self.0 = (self.0 ^ u64::from(b)).wrapping_mul(0x100000001b3);
        }
    }
    fn bytes(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 = (self.0 ^ u64::from(b)).wrapping_mul(0x100000001b3);
        }
    }
    fn point(&mut self, p: Point) {
        for v in p {
            self.word(v.to_bits());
        }
    }
}
struct Identity;
impl Compressor for Identity {
    fn max_compressed_size(&mut self, n: usize) -> usize {
        n
    }
    fn compress(&mut self, input: &[u8], output: &mut [u8]) -> CompressionResult {
        output[..input.len()].copy_from_slice(input);
        CompressionResult {
            status: nav::SUCCESS,
            size: input.len(),
        }
    }
    fn decompress(&mut self, input: &[u8], output: &mut [u8]) -> CompressionResult {
        self.compress(input, output)
    }
}
struct Flags;
impl MeshProcess for Flags {
    fn process(&mut self, p: &mut OwnedCreateParams) {
        for poly in &mut p.polygons {
            poly.flags = 1;
        }
    }
}
fn config() -> BuildConfig {
    BuildConfig {
        bounds: [[0., -2., 0.], [24., 8., 24.]],
        cell_size: 0.25,
        cell_height: 0.125,
        slope_degrees: 45.,
        walkable_height: 12,
        walkable_climb: 4,
        walkable_radius: 2,
        min_region_area: 8,
        merge_region_area: 20,
        max_edge_len: 48,
        detail_sample_dist: 1.5,
        detail_sample_max_error: 0.125,
        ..BuildConfig::default()
    }
}
fn prepare(dir: &Path) {
    fs::create_dir_all(dir).unwrap();
    let mut vertices = vec![];
    let mut triangles = vec![];
    for z in 0..=48 {
        for x in 0..=48 {
            vertices.push([
                x as f32 * 0.5,
                ((x / 8 + z / 8) % 3) as f32 * 0.125,
                z as f32 * 0.5,
            ]);
        }
    }
    for z in 0..48 {
        for x in 0..48 {
            if (10..14).contains(&x) && (12..18).contains(&z) {
                continue;
            }
            let a = z * 49 + x;
            triangles.extend([[a, a + 49, a + 50], [a, a + 50, a + 1]]);
        }
    }
    let mut bytes = vec![];
    for n in [vertices.len() as u32, triangles.len() as u32] {
        bytes.extend(n.to_le_bytes());
    }
    for v in vertices.iter().flatten() {
        bytes.extend(v.to_le_bytes());
    }
    for i in triangles.iter().flatten() {
        bytes.extend(u32::to_le_bytes(*i));
    }
    fs::write(dir.join("geometry.bin"), bytes).unwrap();
    for capacity_case in [true, false] {
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        for floor in 0..2_u32 {
            let base = vertices.len() as u32;
            for z in 0..=128_u32 {
                for x in 0..=128_u32 {
                    vertices.push([
                        x as f32 * 0.5,
                        floor as f32 * 4. + ((x / 16 + z / 16) % 3) as f32 * 0.125,
                        z as f32 * 0.5,
                    ]);
                }
            }
            for z in 0..128_u32 {
                for x in 0..128_u32 {
                    let hole = if capacity_case {
                        (x / 8 + z / 8 + floor) % 7 == 0
                            && (3..5).contains(&(x % 8))
                            && (2..6).contains(&(z % 8))
                    } else {
                        ((24..32).contains(&x) && (40..56).contains(&z))
                            || ((80..88).contains(&x) && (88..104).contains(&z))
                    };
                    if hole {
                        continue;
                    }
                    let a = base + z * 129 + x;
                    triangles.extend([[a, a + 129, a + 130], [a, a + 130, a + 1]]);
                }
            }
        }
        let mut bytes = Vec::new();
        for n in [vertices.len() as u32, triangles.len() as u32] {
            bytes.extend(n.to_le_bytes());
        }
        for v in vertices.iter().flatten() {
            bytes.extend(v.to_le_bytes());
        }
        for i in triangles.iter().flatten() {
            bytes.extend(i.to_le_bytes());
        }
        fs::write(
            dir.join(if capacity_case {
                "geometry-layer-capacity.bin"
            } else {
                "geometry-stacked.bin"
            }),
            bytes,
        )
        .unwrap();
    }
    let mut p = OwnedCreateParams {
        vertices_per_polygon: 4,
        bounds: [[0.; 3], [32., 2., 32.]],
        walkable_height: 2.,
        walkable_radius: 0.25,
        walkable_climb: 0.5,
        cell_size: 1.,
        cell_height: 0.125,
        build_bv_tree: true,
        ..Default::default()
    };
    for z in 0..=32 {
        for x in 0..=32 {
            p.vertices.push([x, 0, z]);
        }
    }
    for z in 0..32 {
        for x in 0..32 {
            let a = z * 33 + x;
            p.polygons.push(CreatePolygon {
                vertices: [a, a + 33, a + 34, a + 1, u16::MAX, u16::MAX],
                neighbors: [
                    if x > 0 { z * 32 + x - 1 } else { u16::MAX },
                    if z < 31 { (z + 1) * 32 + x } else { u16::MAX },
                    if x < 31 { z * 32 + x + 1 } else { u16::MAX },
                    if z > 0 { (z - 1) * 32 + x } else { u16::MAX },
                    u16::MAX,
                    u16::MAX,
                ],
                flags: 1,
                area: 0,
            });
        }
    }
    fs::write(
        dir.join("nav.bin"),
        nav_data::create(&p.as_params()).unwrap(),
    )
    .unwrap();
    // Two full tiles with a bidirectional off-mesh seam and solid ground edges.
    p.off_mesh_connections
        .push(nav_data::CreateOffMeshConnection {
            positions: [[31.5, 0., 16.5], [32.5, 0., 16.5]],
            radius: 0.25,
            flags: 1,
            area: 0,
            direction: 1,
        });
    p.off_mesh_user_ids = Some(vec![0x193751b1]);
    fs::write(
        dir.join("nav-left.bin"),
        nav_data::create(&p.as_params()).unwrap(),
    )
    .unwrap();
    p.off_mesh_connections.clear();
    p.off_mesh_user_ids = None;
    p.coordinate[0] = 1;
    p.bounds[0][0] = 32.;
    p.bounds[1][0] = 64.;
    fs::write(
        dir.join("nav-right.bin"),
        nav_data::create(&p.as_params()).unwrap(),
    )
    .unwrap();
    let header = cache_data::Header {
        bounds: [[0.; 3], [16., 2., 16.]],
        width: 32,
        height: 32,
        usable_bounds: [[0, 0], [31, 31]],
        height_range: [0, 0],
        ..Default::default()
    };
    let mut cons = vec![];
    for z in 0..32 {
        for x in 0..32 {
            cons.push(
                u8::from(x > 0)
                    | (u8::from(z < 31) << 1)
                    | (u8::from(x < 31) << 2)
                    | (u8::from(z > 0) << 3),
            );
        }
    }
    fs::write(
        dir.join("layer.bin"),
        cache_data::build_layer(&mut Identity, &header, &[0; 1024], &[63; 1024], &cons).unwrap(),
    )
    .unwrap();
    crowd_cases::prepare(dir);
}
fn geometry(dir: &Path, name: &str) -> TriangleMesh {
    let data = fs::read(dir.join(name)).unwrap();
    let words: Vec<_> = data
        .chunks_exact(4)
        .map(|s| u32::from_le_bytes(s.try_into().unwrap()))
        .collect();
    let nv = words[0] as usize;
    let nt = words[1] as usize;
    assert_eq!(words.len(), 2 + nv * 3 + nt * 3);
    TriangleMesh {
        vertices: words[2..2 + nv * 3]
            .chunks_exact(3)
            .map(|v| {
                [
                    f32::from_bits(v[0]),
                    f32::from_bits(v[1]),
                    f32::from_bits(v[2]),
                ]
            })
            .collect(),
        triangles: words[2 + nv * 3..]
            .chunks_exact(3)
            .map(|v| [v[0], v[1], v[2]])
            .collect(),
    }
}
fn recast(mesh: &TriangleMesh, partitioning: RegionPartitioning) -> u64 {
    recast_with_width(mesh, partitioning, false, 6)
}
fn recast_with_width(
    mesh: &TriangleMesh,
    partitioning: RegionPartitioning,
    stacked: bool,
    nvp: u32,
) -> u64 {
    let mut config = config();
    config.partitioning = partitioning;
    config.max_vertices_per_polygon = nvp;
    if stacked {
        config.bounds[1] = [64., 10., 64.];
    }
    let tile = build_tile(mesh, &config).unwrap();
    let mut h = Hash::new();
    let p = &tile.polygons;
    for v in [
        p.vertices.len(),
        p.polygon_count(),
        p.max_vertices_per_polygon as usize,
    ] {
        h.word(v as u64);
    }
    for v in &p.vertices {
        for n in v.to_array() {
            h.word(n);
        }
    }
    for &n in p.polygons.iter().chain(&p.polygon_neighbors) {
        h.word(n);
    }
    for i in 0..p.polygon_count() {
        h.word(p.regions[i].bits());
        h.word(p.areas[i].0);
        h.word(p.flags[i]);
    }
    let d = &tile.detail;
    for n in [d.meshes.len(), d.vertices.len(), d.triangles.len()] {
        h.word(n as u64);
    }
    for m in &d.meshes {
        h.word(m.base_vertex_index);
        h.word(m.vertex_count);
        h.word(m.base_triangle_index);
        h.word(m.triangle_count);
    }
    for v in &d.vertices {
        h.point(v.to_array());
    }
    for (t, &flags) in d.triangles.iter().zip(&d.triangle_flags) {
        for &v in t {
            h.word(v);
        }
        h.word(flags);
    }
    h.word(tile.layers.len() as u64);
    for l in &tile.layers {
        for n in [l.width, l.height] {
            h.word(n as u64);
        }
        for &v in l.heights.iter().chain(&l.areas).chain(&l.connections) {
            h.word(v);
        }
    }
    h.0
}
fn point(i: u32) -> Point {
    [(i % 32) as f32 + 0.5, 0., (i / 32) as f32 + 0.5]
}
fn detour(
    mesh: &NavMesh,
    q: &mut NavMeshQuery,
    path: &mut Corridor,
    straight: &mut StraightPath,
) -> u64 {
    let mut h = Hash::new();
    let filter = Filter::default();
    let mut seed = 0x187531b1_u32;
    for _ in 0..128 {
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        let a = (seed >> 16) % 1024;
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        let b = (seed >> 16) % 1024;
        q.find_path_into(
            mesh,
            mesh.reference(0, a as usize),
            mesh.reference(0, b as usize),
            point(a),
            point(b),
            &filter,
            256,
            path,
        );
        h.word(path.status);
        h.word(path.polygons.len() as u64);
        for &r in &path.polygons {
            h.word(r as u64);
        }
        q.straight_path_into(mesh, point(a), point(b), &path.polygons, 256, 0, straight);
        h.word(straight.status);
        h.word(straight.points.len() as u64);
        for i in 0..straight.points.len() {
            h.point(straight.points[i]);
            h.word(straight.flags[i]);
            h.word(straight.references[i] as u64);
        }
    }
    h.0
}
fn crowd(mesh: &NavMesh) -> u64 {
    let mut c = Crowd::new(64, 0.25).unwrap();
    let params = AgentParams {
        radius: 0.25,
        height: 2.,
        max_acceleration: 8.,
        max_speed: 3.5,
        collision_query_range: 3.,
        path_optimization_range: 7.5,
        separation_weight: 2.,
        update_flags: 31,
        ..Default::default()
    };
    for i in 0..64 {
        let p = [0.5 + (i % 8) as f32 * 4., 0., 0.5 + (i / 8) as f32 * 4.];
        let index = c.add_agent(mesh, p, &params).unwrap();
        assert_eq!(index, i);
        let end = [32. - p[0], 0., 32. - p[2]];
        let poly = end[2] as usize * 32 + end[0] as usize;
        assert!(c.request_move_target(index, mesh.reference(0, poly), end));
    }
    let mut h = Hash::new();
    for _ in 0..120 {
        c.update(mesh, 0.05, None).unwrap();
        h.word(c.velocity_sample_count() as u64);
    }
    for i in 0..64 {
        let a = c.agent(i).unwrap();
        h.word(a.state);
        h.word(a.target_state);
        for p in [
            a.position,
            a.displacement,
            a.desired_velocity,
            a.new_velocity,
            a.velocity,
        ] {
            h.point(p);
        }
        h.word(a.desired_speed.to_bits());
        h.word(a.corridor.path().len() as u64);
        for &r in a.corridor.path() {
            h.word(r as u64);
        }
    }
    h.0
}
fn tilecache(data: &[u8], all_shapes: bool) -> u64 {
    let params = tile_cache::Params {
        origin: [0.; 3],
        cell_size: 0.5,
        cell_height: 0.125,
        width: 32,
        height: 32,
        walkable_height: 2.,
        walkable_radius: 0.25,
        walkable_climb: 0.5,
        max_simplification_error: 1.3,
        max_tiles: 4,
        max_obstacles: 32,
    };
    let mut c = TileCache::new(params, Identity, Some(Box::new(Flags))).unwrap();
    let mut mesh = NavMesh::new([0.; 3], 16., 16., 4, 2048).unwrap();
    let reference = c.add_tile(data.to_vec(), tile_cache::FREE_DATA).unwrap();
    assert_eq!(c.build_tile(reference, &mut mesh), nav::SUCCESS);
    let mut h = Hash::new();
    for episode in 0..4 {
        let mut obstacles = [0; 8];
        for (i, obstacle) in obstacles.iter_mut().enumerate() {
            let position = [
                2. + (i % 4) as f32 * 3.,
                0.,
                3. + (i / 4) as f32 * 6. + episode as f32 * 0.125,
            ];
            *obstacle = match if all_shapes { i % 3 } else { 0 } {
                0 => c.add_cylinder(position, 0.75, 2.),
                1 => c.add_box([
                    [position[0] - 0.75, -0.5, position[2] - 0.5],
                    [position[0] + 0.75, 2., position[2] + 0.5],
                ]),
                _ => c.add_oriented_box(position, [0.75, 2., 0.5], 0.375),
            }
            .unwrap();
        }
        for remove in [false, true] {
            if remove {
                for &r in &obstacles {
                    h.word(c.remove_obstacle(r));
                }
            }
            loop {
                let (status, done) = c.update(0.05, &mut mesh);
                h.word(status);
                h.word(u32::from(done));
                if done {
                    break;
                }
            }
            let r = mesh.tile_ref_at([0; 3]);
            h.word(r as u64);
            let slot = mesh.tile_by_ref(r).unwrap();
            let (header, body) = slot.tile.unwrap().data_parts();
            h.word((header.len() + body.len()) as u64);
            for &b in header.iter().chain(body) {
                h.word(b);
            }
        }
    }
    h.0
}
fn main() {
    if cfg!(feature = "fma") || cfg!(feature = "polyref64") {
        eprintln!("benchmark policy requires separate arithmetic and 32-bit references");
        std::process::exit(2);
    }
    let args: Vec<_> = std::env::args().collect();
    let dir = Path::new(&args[2]);
    if args[1] == "prepare" {
        prepare(dir);
        return;
    }
    if args[1] == "build-info" {
        println!(
            "{}",
            serde_json::json!({"rustc": env!("RECAST_BENCH_RUSTC"), "cxx": env!("RECAST_BENCH_CXX"), "tilecache_snapshot_policy": "borrowed-tile-bytes-v1"})
        );
        return;
    }
    if args[1] == "native-path" {
        println!("{}", env!("RECAST_NATIVE_BENCH"));
        return;
    }
    if args[1] == "measure" {
        let mut command = std::process::Command::new("python3");
        command
            .arg(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../../scripts/bench_libraries.py"
            ))
            .arg("--rust")
            .arg(std::env::current_exe().unwrap())
            .arg("--native")
            .arg(env!("RECAST_NATIVE_BENCH"))
            .arg("--fixtures")
            .arg(dir)
            .arg("--output")
            .arg(&args[3]);
        if let (Some(profiler), Some(calibration)) = (
            option_env!("RECAST_ALLOC_PROFILER"),
            option_env!("RECAST_ALLOC_CALIBRATION"),
        ) {
            command
                .arg("--profiler")
                .arg(profiler)
                .arg("--calibration")
                .arg(calibration);
        }
        std::process::exit(
            command
                .status()
                .expect("start measurement driver")
                .code()
                .unwrap_or(1),
        );
    }
    let iterations: u32 = args[3].parse().unwrap();
    let geometry = (args[1].starts_with("recast")).then(|| {
        geometry(
            dir,
            if args[1] == "recast-layer-capacity" {
                "geometry-layer-capacity.bin"
            } else if args[1].starts_with("recast-stacked") {
                "geometry-stacked.bin"
            } else {
                "geometry.bin"
            },
        )
    });
    let mesh = matches!(
        args[1].as_str(),
        "detour"
            | "detour-spatial"
            | "detour-search"
            | "detour-sliced"
            | "crowd"
            | "crowd-small"
            | "crowd-dense"
            | "crowd-offmesh"
            | "crowd-corridor"
            | "crowd-path-queue"
    )
    .then(|| {
        let file = if args[1] == "crowd-offmesh" {
            "nav-crowd-link.bin"
        } else {
            "nav.bin"
        };
        NavMesh::from_tile_data(&fs::read(dir.join(file)).unwrap()).unwrap()
    });
    let layer =
        (args[1].starts_with("tilecache")).then(|| fs::read(dir.join("layer.bin")).unwrap());
    let mut query = (matches!(
        args[1].as_str(),
        "detour" | "detour-spatial" | "detour-search" | "detour-sliced"
    ))
    .then(|| NavMeshQuery::new(2048).unwrap());
    let mut path = (args[1] == "detour").then(|| Corridor {
        polygons: Vec::with_capacity(256),
        ..Default::default()
    });
    let mut straight = (args[1] == "detour").then(|| StraightPath {
        points: Vec::with_capacity(256),
        flags: Vec::with_capacity(256),
        references: Vec::with_capacity(256),
        ..Default::default()
    });
    let mut spatial = (args[1] == "detour-spatial").then(extended::SpatialBuffers::new);
    let mut search = matches!(args[1].as_str(), "detour-search" | "detour-sliced")
        .then(extended::SearchBuffers::new);
    let mut cache_stream = (args[1] == "tilecache-stream").then(extended::CacheStreamBuffers::new);
    let mut recast_utilities = (args[1] == "recast-utilities")
        .then(|| extended::RecastUtilities::new(geometry.as_ref().unwrap()));
    let lifecycle_data = (args[1] == "detour-lifecycle").then(|| {
        [
            fs::read(dir.join("nav-left.bin")).unwrap(),
            fs::read(dir.join("nav-right.bin")).unwrap(),
        ]
    });
    let mut lifecycle = lifecycle_data
        .as_ref()
        .map(|data| extended::LifecycleBuffers::new(data.iter().map(Vec::len).max().unwrap()));
    let mut components = matches!(args[1].as_str(), "crowd-corridor" | "crowd-path-queue")
        .then(crowd_cases::ComponentBuffers::new);
    let mut hash = Hash::new();
    let start = Instant::now();
    for _ in 0..iterations {
        let value = match args[1].as_str() {
            "recast-layer-capacity" => extended::recast_layer_capacity(geometry.as_ref().unwrap()),
            "recast-utilities" => recast_utilities
                .as_mut()
                .unwrap()
                .run(geometry.as_ref().unwrap()),
            "recast-stacked" => recast_with_width(
                geometry.as_ref().unwrap(),
                RegionPartitioning::Watershed,
                true,
                6,
            ),
            "recast-stacked-monotone" => recast_with_width(
                geometry.as_ref().unwrap(),
                RegionPartitioning::Monotone,
                true,
                3,
            ),
            "recast-stacked-layers" => recast_with_width(
                geometry.as_ref().unwrap(),
                RegionPartitioning::Layers,
                true,
                12,
            ),
            "recast" => recast(geometry.as_ref().unwrap(), RegionPartitioning::Watershed),
            "recast-monotone" => recast(geometry.as_ref().unwrap(), RegionPartitioning::Monotone),
            "recast-layers" => recast(geometry.as_ref().unwrap(), RegionPartitioning::Layers),
            "detour-spatial" => extended::spatial(
                mesh.as_ref().unwrap(),
                query.as_mut().unwrap(),
                spatial.as_mut().unwrap(),
            ),
            "detour-search" => extended::search(
                mesh.as_ref().unwrap(),
                query.as_mut().unwrap(),
                search.as_mut().unwrap(),
            ),
            "detour-sliced" => extended::sliced(
                mesh.as_ref().unwrap(),
                query.as_mut().unwrap(),
                search.as_mut().unwrap(),
            ),
            "detour-lifecycle" => extended::lifecycle(
                lifecycle_data.as_ref().unwrap(),
                lifecycle.as_mut().unwrap(),
            ),
            "proximity-grid" => extended::proximity(),
            "avoidance-grid" => extended::avoidance(false),
            "avoidance-adaptive" => extended::avoidance(true),
            "detour" => detour(
                mesh.as_ref().unwrap(),
                query.as_mut().unwrap(),
                path.as_mut().unwrap(),
                straight.as_mut().unwrap(),
            ),
            "crowd" => crowd(mesh.as_ref().unwrap()),
            "crowd-corridor" => {
                crowd_cases::corridor(mesh.as_ref().unwrap(), components.as_mut().unwrap())
            }
            "crowd-path-queue" => {
                crowd_cases::queue(mesh.as_ref().unwrap(), components.as_mut().unwrap())
            }
            "crowd-small" | "crowd-dense" | "crowd-offmesh" => crowd_cases::run(
                mesh.as_ref().unwrap(),
                args[1] == "crowd-dense",
                args[1] == "crowd-offmesh",
            ),
            "tilecache-codec" => extended::cache_codec(),
            "tilecache-builder" => extended::cache_builder(),
            "tilecache-stream" => extended::cache_stream(cache_stream.as_mut().unwrap()),
            "tilecache" => tilecache(layer.as_ref().unwrap(), false),
            "tilecache-shapes" => tilecache(layer.as_ref().unwrap(), true),
            _ => panic!("unknown workload"),
        };
        hash.word(black_box(value));
    }
    let elapsed = start.elapsed().as_nanos();
    println!(
        "{{\"case\":\"{}\",\"iterations\":{iterations},\"checksum\":\"{:016x}\",\"elapsed_ns\":{elapsed}}}",
        args[1], hash.0
    );
}
