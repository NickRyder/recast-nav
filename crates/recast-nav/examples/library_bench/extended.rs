//! Additional matched workloads; construction and output ownership match C++.
use super::{Hash, point};
use recast_nav::{
    crowd::{
        AvoidanceDebugData, AvoidanceParams, ObstacleAvoidanceQuery, ProximityGrid, VelocityInput,
    },
    nav::{self, Filter, NavMesh, RaycastHit},
    query::{NavMeshQuery, Neighborhood, RandomPoint, SurfaceMove, WallHit, WallSegments},
};

pub struct SpatialBuffers {
    ray: RaycastHit,
    surface: SurfaceMove,
    neighborhood: Neighborhood,
    walls: WallSegments,
}
impl SpatialBuffers {
    pub fn new() -> Self {
        Self {
            ray: RaycastHit {
                path: Vec::with_capacity(256),
                ..Default::default()
            },
            surface: SurfaceMove {
                visited: Vec::with_capacity(256),
                ..Default::default()
            },
            neighborhood: Neighborhood {
                entries: Vec::with_capacity(64),
                ..Default::default()
            },
            walls: WallSegments {
                segments: Vec::with_capacity(18),
                ..Default::default()
            },
        }
    }
}

pub fn spatial(mesh: &NavMesh, q: &mut NavMeshQuery, output: &mut SpatialBuffers) -> u64 {
    let filter = Filter::default();
    let mut h = Hash::new();
    let mut seed = 0x187531b1_u32;
    for i in 0..32 {
        let a = (i * 37) % 1024;
        let b = (i * 113 + 17) % 1024;
        let start = point(a);
        let end = point(b);
        let reference = mesh.reference(0, a as usize);
        h.word(q.raycast_into(
            mesh,
            reference,
            start,
            end,
            &filter,
            nav::RAYCAST_USE_COSTS,
            0,
            256,
            &mut output.ray,
        ));
        h.word(output.ray.t.to_bits());
        h.point(output.ray.normal);
        h.word(output.ray.edge_index as u32);
        h.word(output.ray.path_cost.to_bits());
        h.word(output.ray.path.len() as u64);
        for &r in &output.ray.path {
            h.word(r as u64);
        }
        h.word(q.move_along_surface_into(
            mesh,
            reference,
            start,
            end,
            &filter,
            256,
            &mut output.surface,
        ));
        h.point(output.surface.position);
        h.word(output.surface.visited.len() as u64);
        for &r in &output.surface.visited {
            h.word(r as u64);
        }
        q.find_local_neighborhood_into(
            mesh,
            reference,
            start,
            4.,
            &filter,
            64,
            &mut output.neighborhood,
        );
        h.word(output.neighborhood.status);
        h.word(output.neighborhood.entries.len() as u64);
        for p in &output.neighborhood.entries {
            h.word(p.reference as u64);
            h.word(p.parent as u64);
        }
        q.poly_wall_segments_into(mesh, reference, &filter, true, 18, &mut output.walls);
        h.word(output.walls.status);
        h.word(output.walls.segments.len() as u64);
        for s in &output.walls.segments {
            h.point(s.positions[0]);
            h.point(s.positions[1]);
            h.word(s.reference as u64);
        }
        let mut wall = WallHit::default();
        h.word(q.find_distance_to_wall_into(mesh, reference, start, 6., &filter, &mut wall));
        h.word(wall.distance.to_bits());
        h.point(wall.position);
        h.point(wall.normal);
        let (status, nearest) = mesh.nearest_with_over(start, [2., 4., 2.], &filter);
        h.word(status);
        let nearest = nearest.unwrap();
        h.word(nearest.reference as u64);
        h.point(nearest.position);
        h.word(u32::from(nearest.over_polygon));
        let mut random = || {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            (seed >> 8) as f32 * (1. / 16777216.)
        };
        let mut selected = RandomPoint::default();
        h.word(q.find_random_point_into(mesh, &filter, &mut random, &mut selected));
        h.word(selected.reference as u64);
        h.point(selected.position);
        h.word(q.find_random_point_around_circle_into(
            mesh,
            reference,
            start,
            5.,
            &filter,
            &mut random,
            &mut selected,
        ));
        h.word(selected.reference as u64);
        h.point(selected.position);
        h.word(seed);
    }
    h.0
}

pub fn proximity() -> u64 {
    let mut grid = ProximityGrid::new(4096, 1.).unwrap();
    let mut ids = [0; 128];
    let mut h = Hash::new();
    for batch in 0..4 {
        grid.clear();
        for i in 0..128_u16 {
            let x = f32::from(i % 16) * 0.375 - 3.;
            let z = f32::from(i / 16) * 0.375 - 1.5 + batch as f32 * 0.125;
            let id = if i % 5 == 0 { 60000 + i } else { i % 64 };
            assert_eq!(
                grid.add_item(id, [[x - 0.35, z - 0.35], [x + 0.35, z + 0.35]]),
                nav::SUCCESS
            );
        }
        for n in grid.bounds() {
            h.word(n as u32);
        }
        for i in 0..256 {
            let x = (i % 16) as f32 * 0.5 - 4.;
            let z = (i / 16) as f32 * 0.25 - 2.;
            let cap = [0, 8, 32, 128][i % 4];
            let count = grid
                .query_items_slice([[x - 1., z - 1.], [x + 1., z + 1.]], &mut ids[..cap])
                .unwrap();
            h.word(count as u64);
            for &id in &ids[..count] {
                h.word(id);
            }
            h.word(
                grid.item_count_at(i as i32 % 8 - 4, i as i32 / 32 - 4)
                    .unwrap() as u64,
            );
        }
    }
    h.0
}

pub fn avoidance(adaptive: bool) -> u64 {
    let mut query = ObstacleAvoidanceQuery::new(16, 8).unwrap();
    let mut debug = AvoidanceDebugData::new(2048).unwrap();
    let params = AvoidanceParams {
        velocity_bias: 0.4,
        weight_desired_velocity: 2.,
        weight_current_velocity: 0.75,
        weight_side: 0.75,
        weight_time_of_impact: 2.5,
        horizon_time: 2.5,
        grid_size: 23,
        adaptive_divisions: 7,
        adaptive_rings: 2,
        adaptive_depth: 3,
    };
    let mut h = Hash::new();
    for i in 0..32 {
        query.reset();
        for j in 0..8 {
            query.add_circle(
                [(j % 4) as f32 * 1.25 - 1.5, 0., (j / 4) as f32 * 1.5 + 1.],
                0.3,
                [j as f32 * 0.125 - 0.5, 0., 0.25],
                [0.5, 0., -0.25],
            );
        }
        let corners = [[-4., 0., -4.], [-4., 0., 4.], [4., 0., 4.], [4., 0., -4.]];
        for j in 0..4 {
            query.add_segment([corners[j], corners[(j + 1) % 4]]);
        }
        let input = VelocityInput {
            position: [0.125, 0., 0.25],
            radius: 0.3,
            maximum_speed: 3.,
            velocity: [i as f32 * 0.03125 - 0.5, 0., 0.25],
            desired_velocity: [1.5 - (i % 4) as f32 * 0.5, 0., -0.75 + (i % 3) as f32 * 0.5],
        };
        let record = i % 8 == 0;
        let recorder = record.then_some(&mut debug);
        let result = if adaptive {
            query
                .sample_velocity_adaptive(input, &params, recorder)
                .unwrap()
        } else {
            query.sample_velocity_grid(input, &params, recorder)
        };
        h.word(result.sample_count as u64);
        h.point(result.velocity);
        if record {
            assert_eq!(debug.normalize_samples(), nav::SUCCESS);
            h.word(debug.sample_count() as u64);
            for s in debug.samples() {
                h.point(s.velocity);
                h.word(s.size.to_bits());
                for v in s.penalties {
                    h.word(v.to_bits());
                }
            }
        }
        for c in query.circles() {
            h.point(c.direction);
            h.word(c.preferred_side[0].to_bits());
            h.word(c.preferred_side[2].to_bits());
        }
        for s in query.segments() {
            h.word(u32::from(s.touch));
        }
    }
    h.0
}

pub struct LifecycleBuffers {
    path: recast_nav::nav::Corridor,
    straight: recast_nav::nav::StraightPath,
    exported: Vec<u8>,
    state: [u8; 8192],
}
impl LifecycleBuffers {
    pub fn new(data_size: usize) -> Self {
        Self {
            path: recast_nav::nav::Corridor {
                polygons: Vec::with_capacity(256),
                ..Default::default()
            },
            straight: recast_nav::nav::StraightPath {
                points: Vec::with_capacity(256),
                flags: Vec::with_capacity(256),
                references: Vec::with_capacity(256),
                ..Default::default()
            },
            exported: Vec::with_capacity(data_size),
            state: [0; 8192],
        }
    }
}
fn export(mesh: &NavMesh, reference: nav::TileRef, buffer: &mut Vec<u8>, h: &mut Hash) {
    mesh.tile_data_into(reference, buffer).unwrap();
    h.word(buffer.len() as u64);
    h.bytes(buffer);
}
pub fn lifecycle(data: &[Vec<u8>; 2], output: &mut LifecycleBuffers) -> u64 {
    let mut mesh = NavMesh::new([0.; 3], 32., 32., 4, 2048).unwrap();
    let mut query = NavMeshQuery::new(4096).unwrap();
    let filter = Filter::default();
    let mut references = [0; 2];
    let mut h = Hash::new();
    for cycle in 0..4 {
        for step in 0..2 {
            let tile = (cycle + step) % 2;
            let reference = mesh
                .add_tile_data_owned(
                    data[tile].clone(),
                    nav::TILE_FREE_DATA,
                    if cycle % 2 == 0 { 0 } else { references[tile] },
                )
                .unwrap();
            h.word(nav::SUCCESS);
            h.word(reference as u64);
            references[tile] = reference;
        }
        for &reference in &references {
            export(&mesh, reference, &mut output.exported, &mut h);
        }
        let state_size = mesh.tile_state_size(references[0]);
        let state = &mut output.state[..state_size];
        state.fill(0xa5);
        h.word(mesh.store_tile_state(references[0], state));
        h.word(state_size as u64);
        h.bytes(state);
        for j in 0..32 {
            let reference = references[0] | (j * 13) as nav::PolyRef;
            h.word(mesh.set_poly_flags(reference, 2 + (j % 3) as u16));
            h.word(mesh.set_poly_area(reference, ((cycle + j) % 64) as u8));
            h.word(mesh.poly_flags(reference).unwrap());
            h.word(mesh.poly_area(reference).unwrap());
        }
        export(&mesh, references[0], &mut output.exported, &mut h);
        h.word(mesh.restore_tile_state(references[0], state));
        let start = [30.5, 0., 16.5];
        let end = [33.5, 0., 16.5];
        query.find_path_into(
            &mesh,
            references[0] | 542,
            references[1] | 513,
            start,
            end,
            &filter,
            256,
            &mut output.path,
        );
        assert_eq!(output.path.status, nav::SUCCESS);
        assert!(output.path.polygons.contains(&(references[0] | 1024)));
        h.word(output.path.status);
        h.word(output.path.polygons.len() as u64);
        for &reference in &output.path.polygons {
            h.word(reference as u64);
        }
        query.straight_path_into(
            &mesh,
            start,
            end,
            &output.path.polygons,
            256,
            nav::STRAIGHTPATH_ALL_CROSSINGS,
            &mut output.straight,
        );
        h.word(output.straight.status);
        h.word(output.straight.points.len() as u64);
        for i in 0..output.straight.points.len() {
            h.point(output.straight.points[i]);
            h.word(output.straight.flags[i]);
            h.word(output.straight.references[i] as u64);
        }
        let mut removed = nav::RemovedTileData::default();
        h.word(mesh.remove_tile_data_into(references[1], &mut removed));
        h.word(u32::from(removed.data.is_some()));
        h.word(removed.data_size as u32);
        h.word(u32::from(mesh.is_valid_poly_ref(references[1] | 1)));
        export(&mesh, references[0], &mut output.exported, &mut h);
        h.word(mesh.remove_tile_data_into(references[0], &mut removed));
        h.word(u32::from(removed.data.is_some()));
        h.word(removed.data_size as u32);
    }
    h.0
}

pub struct SearchBuffers {
    entries: recast_nav::query::DijkstraSearch,
    path: nav::Corridor,
    existing: nav::Corridor,
}
impl SearchBuffers {
    pub fn new() -> Self {
        Self {
            entries: recast_nav::query::DijkstraSearch {
                entries: Vec::with_capacity(1024),
                ..Default::default()
            },
            path: nav::Corridor {
                polygons: Vec::with_capacity(1025),
                ..Default::default()
            },
            existing: nav::Corridor {
                polygons: Vec::with_capacity(1025),
                ..Default::default()
            },
        }
    }
}
pub fn search(mesh: &NavMesh, query: &mut NavMeshQuery, out: &mut SearchBuffers) -> u64 {
    let filter = Filter::default();
    let base = mesh.reference(0, 0);
    let mut h = Hash::new();
    for i in 0..32_u32 {
        let a = i * 37 % 1024;
        let center = point(a);
        let start = base | a as nav::PolyRef;
        let cap = [0, 8, 128, 1024][i as usize % 4];
        for shape in [false, true] {
            if shape {
                let [x, y, z] = center;
                let vertices = [
                    [x - 3.25, y, z - 3.25],
                    [x - 3.25, y, z + 3.25],
                    [x + 3.25, y, z + 3.25],
                    [x + 3.25, y, z - 3.25],
                ];
                query.find_polygons_around_shape_into(
                    mesh,
                    start,
                    &vertices,
                    &filter,
                    cap,
                    &mut out.entries,
                );
            } else {
                query.find_polygons_around_circle_into(
                    mesh,
                    start,
                    center,
                    4.25,
                    &filter,
                    cap,
                    &mut out.entries,
                );
            }
            h.word(out.entries.status);
            h.word(out.entries.entries.len() as u64);
            for entry in &out.entries.entries {
                h.word(entry.reference as u64);
                h.word(entry.parent as u64);
                h.word(entry.cost.to_bits());
            }
            for index in [a, (a + 33) % 1024, (i * 113 + 17) % 1024] {
                let end = base | index as nav::PolyRef;
                h.word(query.is_in_closed_list(end));
                h.word(query.path_from_dijkstra_search_into(mesh, end, 16, &mut out.path));
                h.word(out.path.polygons.len() as u64);
                for &r in &out.path.polygons {
                    h.word(r as u64);
                }
            }
        }
        let status = mesh.query_polygons_with(center, [4., 2., 4.], &filter, |batch| {
            h.word(batch.len() as u64);
            for p in batch {
                h.word(p.reference as u64);
                h.word(p.polygon.flags());
                h.word(p.polygon.area());
            }
        });
        h.word(status);
    }
    h.0
}
pub fn sliced(mesh: &NavMesh, query: &mut NavMeshQuery, out: &mut SearchBuffers) -> u64 {
    let filter = Filter::default();
    let base = mesh.reference(0, 0);
    let mut h = Hash::new();
    for i in 0..32_u32 {
        let a = i * 37 % 1024;
        let b = (i * 113 + 17) % 1024;
        let start = point(a);
        let end = point(b);
        let (a, b) = (base | a as nav::PolyRef, base | b as nav::PolyRef);
        let partial = i % 4 >= 2;
        if partial {
            h.word(query.find_path_into(mesh, a, b, start, end, &filter, 256, &mut out.existing));
        }
        let options = if i % 2 == 0 {
            0
        } else {
            nav::FINDPATH_ANY_ANGLE
        };
        let mut status = query.init_sliced_find_path(mesh, a, b, start, end, &filter, options);
        h.word(status);
        while status & nav::IN_PROGRESS != 0 {
            let mut done = 0;
            status = query.update_sliced_find_path(
                mesh,
                &filter,
                if partial { (i % 7 + 1) as i32 } else { 8 },
                Some(&mut done),
            );
            h.word(status);
            h.word(done as u32);
            if partial {
                break;
            }
        }
        let cap = [1, 3, 16, 256][(i / 4) as usize % 4];
        if partial {
            query.finalize_sliced_find_path_partial_into(
                mesh,
                &out.existing.polygons,
                &filter,
                cap,
                &mut out.path,
            );
        } else {
            query.finalize_sliced_find_path_into(mesh, &filter, cap, &mut out.path);
        }
        h.word(out.path.status);
        h.word(out.path.polygons.len() as u64);
        for &r in &out.path.polygons {
            h.word(r as u64);
        }
    }
    h.0
}

use recast_nav::{
    cache,
    cache_data::{self, CompressionResult, Compressor},
    layers::HeightfieldLayer,
    nav_data::OwnedCreateParams,
    tile_cache::{self, MeshProcess, TileCache},
};
use std::{cell::Cell, rc::Rc};

struct BenchCodec {
    mask: u8,
    fail: u8,
    log: Hash,
}
impl BenchCodec {
    fn new(mask: u8) -> Self {
        Self {
            mask,
            fail: 0,
            log: Hash::new(),
        }
    }
    fn transfer(&mut self, tag: u32, input: &[u8], output: &mut [u8]) -> CompressionResult {
        self.log.word(tag);
        self.log.word(input.len() as u64);
        self.log.word(output.len() as u64);
        let failed = self.fail == tag as u8;
        let status = if failed {
            nav::FAILURE | nav::INVALID_PARAM
        } else {
            nav::SUCCESS | nav::BUFFER_TOO_SMALL
        };
        self.log.word(status);
        let count = if failed { input.len() / 2 } else { input.len() };
        assert!(count <= output.len());
        for (dst, &src) in output.iter_mut().zip(&input[..count]) {
            *dst = src ^ self.mask;
        }
        CompressionResult {
            status,
            size: count,
        }
    }
}
impl Compressor for BenchCodec {
    fn max_compressed_size(&mut self, n: usize) -> usize {
        self.log.word(0_u32);
        self.log.word(n as u64);
        n + 8
    }
    fn compress(&mut self, input: &[u8], output: &mut [u8]) -> CompressionResult {
        self.transfer(1, input, output)
    }
    fn decompress(&mut self, input: &[u8], output: &mut [u8]) -> CompressionResult {
        self.transfer(2, input, output)
    }
}
fn cache_grids(width: usize, batch: usize, varied: bool) -> [Vec<u8>; 3] {
    let mut heights = vec![0; width * width];
    let mut areas = vec![63; width * width];
    let mut cons = vec![0; width * width];
    for z in 0..width {
        for x in 0..width {
            let i = x + z * width;
            if varied {
                heights[i] = ((x / 8 + z / 8) % 3) as u8;
                areas[i] = if (8..12).contains(&x) && (10..18).contains(&z) {
                    0
                } else {
                    1 + ((x / 8 + z / 8 + batch) % 3) as u8
                };
            }
            cons[i] = u8::from(x > 0)
                | (u8::from(z + 1 < width) << 1)
                | (u8::from(x + 1 < width) << 2)
                | (u8::from(z > 0) << 3);
        }
    }
    [heights, areas, cons]
}
fn cache_header(width: usize, coordinate: [i32; 3]) -> cache_data::Header {
    let [x, z, level] = coordinate;
    cache_data::Header {
        coordinate,
        bounds: [
            [x as f32 * 16., level as f32 * 4., z as f32 * 16.],
            [
                x as f32 * 16. + width as f32 * 0.5,
                level as f32 * 4. + 2.,
                z as f32 * 16. + width as f32 * 0.5,
            ],
        ],
        width: width as u8,
        height: width as u8,
        usable_bounds: [[0, 0], [width as u8 - 1, width as u8 - 1]],
        ..Default::default()
    }
}
pub fn cache_codec() -> u64 {
    let mut h = Hash::new();
    for batch in 0..16 {
        let width = 16 + batch % 3 * 8;
        let header = cache_header(width, [batch as i32 % 2, 0, 0]);
        let [heights, areas, cons] = cache_grids(width, batch, true);
        let mut codec = BenchCodec::new((batch * 17) as u8);
        let mut data =
            cache_data::build_layer(&mut codec, &header, &heights, &areas, &cons).unwrap();
        h.word(nav::SUCCESS);
        h.word(data.len() as u64);
        h.bytes(&data);
        h.word(cache_data::swap_header_endian(&mut data));
        h.bytes(&data);
        h.word(cache_data::swap_header_endian(&mut data));
        h.bytes(&data);
        let layer = cache_data::decompress_layer(&mut codec, &data).unwrap();
        h.word(nav::SUCCESS);
        for grid in layer.grids() {
            h.bytes(grid);
        }
        codec.fail = 1;
        h.word(cache_data::build_layer(&mut codec, &header, &heights, &areas, &cons).unwrap_err());
        codec.fail = 2;
        h.word(cache_data::decompress_layer(&mut codec, &data).unwrap_err());
        codec.fail = 0;
        for byte in [0, 4] {
            data[byte] ^= 1;
            h.word(cache_data::decompress_layer(&mut codec, &data).unwrap_err());
            data[byte] ^= 1;
        }
        h.word(codec.log.0);
    }
    h.0
}
pub fn cache_builder() -> u64 {
    let mut h = Hash::new();
    for batch in 0..8 {
        let [heights, areas, connections] = cache_grids(32, batch, true);
        let mut layer = HeightfieldLayer {
            bounds: [[0., 0., 0.], [16., 2., 16.]],
            cell_size: 0.5,
            cell_height: 0.125,
            width: 32,
            height: 32,
            min_x: 0,
            max_x: 31,
            min_z: 0,
            max_z: 31,
            hmin: 0,
            hmax: 0,
            heights,
            areas,
            connections,
        };
        let x = 2. + batch as f32 * 0.25;
        h.word(cache::mark_cylinder_area(
            &mut layer,
            [x, 0., 4.],
            0.75,
            2.,
            0,
        ));
        h.word(cache::mark_box_area(
            &mut layer,
            [8., 0., 7.],
            [9.5, 2., 9.],
            0,
        ));
        h.word(cache::mark_oriented_box_area(
            &mut layer,
            [12., 0., 12.],
            [1., 2., 0.5],
            [0.5, 0.5],
            0,
        ));
        let built = cache::build_cache_mesh(&layer, 4, 1.3).unwrap();
        h.word(nav::SUCCESS);
        h.word(6_u32);
        h.word(built.region_count);
        h.bytes(&built.regions);
        h.word(built.contours.len() as u64);
        for c in &built.contours {
            h.word(c.region);
            h.word(c.area);
            h.word(c.vertices.len() as u64);
            for v in &c.vertices {
                h.bytes(v);
            }
        }
        h.word(built.mesh.vertices.len() as u64);
        h.word(built.mesh.polygons.len() as u64);
        for v in &built.mesh.vertices {
            for &n in v {
                h.word(n);
            }
        }
        for p in &built.mesh.polygons {
            for &n in p {
                h.word(n);
            }
        }
        for &n in &built.mesh.areas {
            h.word(n);
        }
        for &n in &built.mesh.flags {
            h.word(n);
        }
    }
    h.0
}
struct CacheProcess(Rc<Cell<Hash>>);
impl MeshProcess for CacheProcess {
    fn process(&mut self, p: &mut OwnedCreateParams) {
        let mut log = self.0.get();
        log.word(p.vertices.len() as u64);
        log.word(p.polygons.len() as u64);
        for c in p.coordinate {
            log.word(c as u32);
        }
        for (i, poly) in p.polygons.iter_mut().enumerate() {
            log.word(poly.area);
            log.word(poly.flags);
            poly.area = ((usize::from(poly.area) + i % 4) % 7) as u8;
            poly.flags = 1 + poly.area as u16 % 3;
        }
        p.user_id = 0x17600000
            | ((p.coordinate[0] as u32) << 12)
            | ((p.coordinate[1] as u32) << 4)
            | p.coordinate[2] as u32;
        self.0.set(log);
    }
}
pub struct CacheStreamBuffers {
    exported: Vec<u8>,
}
impl CacheStreamBuffers {
    pub fn new() -> Self {
        Self {
            exported: Vec::with_capacity(65536),
        }
    }
}
fn cache_snapshot(
    cache: &TileCache<BenchCodec>,
    mesh: &NavMesh,
    out: &mut CacheStreamBuffers,
    h: &mut Hash,
) {
    for z in 0..2 {
        for x in 0..2 {
            for level in 0..2 {
                let coordinate = [x, z, level];
                let slot = mesh.tile_at(coordinate).unwrap();
                let r = mesh.tile_reference(Some(slot.index));
                h.word(r as u64);
                mesh.tile_data_into(r, &mut out.exported).unwrap();
                h.word(out.exported.len() as u64);
                h.bytes(&out.exported);
            }
        }
    }
    for ob in cache.obstacles() {
        h.word(ob.state);
        h.word(ob.kind);
        h.word(ob.salt);
        h.word(ob.touched_count);
        h.word(ob.pending_count);
        for &r in &ob.touched[..usize::from(ob.touched_count)] {
            h.word(r);
        }
        for &r in &ob.pending[..usize::from(ob.pending_count)] {
            h.word(r);
        }
    }
}
pub fn cache_stream(out: &mut CacheStreamBuffers) -> u64 {
    let log = Rc::new(Cell::new(Hash::new()));
    let mut cache = TileCache::new(
        tile_cache::Params {
            cell_size: 0.5,
            cell_height: 0.125,
            width: 32,
            height: 32,
            walkable_height: 2.,
            walkable_radius: 0.25,
            walkable_climb: 0.5,
            max_simplification_error: 1.3,
            max_tiles: 16,
            max_obstacles: 32,
            ..Default::default()
        },
        BenchCodec::new(0x5a),
        Some(Box::new(CacheProcess(log.clone()))),
    )
    .unwrap();
    let mut mesh = NavMesh::new([0.; 3], 16., 16., 16, 2048).unwrap();
    let mut references = [0; 8];
    let mut h = Hash::new();
    for z in 0..2 {
        for x in 0..2 {
            for level in 0..2 {
                let coordinate = [x, z, level];
                let header = cache_header(32, coordinate);
                let [heights, areas, cons] = cache_grids(32, 0, false);
                let data =
                    cache_data::build_layer(cache.compressor(), &header, &heights, &areas, &cons)
                        .unwrap();
                let r = cache.add_tile(data, tile_cache::FREE_DATA).unwrap();
                references[(z * 4 + x * 2 + level) as usize] = r;
                h.word(nav::SUCCESS);
                h.word(r);
                h.word(cache.build_tile(r, &mut mesh));
            }
        }
    }
    cache_snapshot(&cache, &mesh, out, &mut h);
    for batch in 0..2 {
        let mut obstacles = [0; 8];
        for (i, obstacle) in obstacles.iter_mut().enumerate() {
            let pos = [
                14. + (i % 4) as f32 * 1.25,
                -1.,
                14. + (i / 4) as f32 * 3.25 + batch as f32 * 0.125,
            ];
            *obstacle = match i % 3 {
                0 => cache.add_cylinder(pos, 1.25, 9.),
                1 => cache.add_box([
                    [pos[0] - 1.25, -1., pos[2] - 1.],
                    [pos[0] + 1.25, 8., pos[2] + 1.],
                ]),
                _ => cache.add_oriented_box([pos[0], 3., pos[2]], [1.25, 4., 1.], 0.375),
            }
            .unwrap();
            h.word(nav::SUCCESS);
            h.word(*obstacle);
        }
        for remove in [false, true] {
            if remove {
                for r in obstacles {
                    h.word(cache.remove_obstacle(r));
                }
            }
            loop {
                let (s, done) = cache.update(0.05, &mut mesh);
                h.word(s);
                h.word(done);
                assert!(s & nav::FAILURE == 0);
                if done {
                    break;
                }
            }
            cache_snapshot(&cache, &mesh, out, &mut h);
        }
    }
    cache.compressor().fail = 2;
    h.word(cache.build_tile(references[0], &mut mesh));
    cache_snapshot(&cache, &mesh, out, &mut h);
    cache.compressor().fail = 0;
    h.word(cache.build_tile(references[0], &mut mesh));
    let removed = cache.remove_tile(references[3]).unwrap();
    h.word(nav::SUCCESS);
    h.word(removed.is_some());
    h.word(cache.build_tile(references[3], &mut mesh));
    let header = cache_header(32, [1, 0, 1]);
    let [heights, areas, cons] = cache_grids(32, 0, false);
    let data =
        cache_data::build_layer(cache.compressor(), &header, &heights, &areas, &cons).unwrap();
    let r = cache.add_tile(data, tile_cache::FREE_DATA).unwrap();
    h.word(nav::SUCCESS);
    h.word(r);
    h.word(cache.build_tile(r, &mut mesh));
    h.word(cache.build_tiles_at(2, 2, &mut mesh));
    cache_snapshot(&cache, &mesh, out, &mut h);
    h.word(cache.compressor().log.0);
    h.word(log.get().0);
    h.0
}

use recast_nav::{AreaType, CompactHeightfield};
use recast_nav::{BuildStage, BuiltTile, TriangleMesh, recast};

fn hash_recast_polygons(h: &mut Hash, p: &recast_nav::PolygonNavmesh) {
    h.word(p.vertices.len() as u64);
    h.word(p.polygon_count() as u64);
    h.word(p.max_vertices_per_polygon);
    h.word(p.max_polygons as u64);
    h.point(p.aabb.min.to_array());
    h.point(p.aabb.max.to_array());
    h.word(p.cell_size.to_bits());
    h.word(p.cell_height.to_bits());
    h.word(p.border_size as u32);
    h.word(p.max_edge_error.to_bits());
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
}
fn hash_recast_detail(h: &mut Hash, d: &recast_nav::DetailNavmesh) {
    h.word(d.meshes.len() as u64);
    h.word(d.vertices.len() as u64);
    h.word(d.triangles.len() as u64);
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
        for &n in t {
            h.word(n);
        }
        h.word(flags);
    }
}
pub struct RecastUtilities {
    compact: CompactHeightfield,
    initial_areas: Vec<AreaType>,
    tile: BuiltTile,
    triangle_areas: Vec<AreaType>,
    offset: [[f32; 3]; 32],
}
impl RecastUtilities {
    pub fn new(mesh: &TriangleMesh) -> Self {
        let mut compact = None;
        let mut initial_areas = Vec::new();
        let mut config = super::config();
        config.max_vertices_per_polygon = 12;
        let mut tile = recast_nav::build_tile_observed(mesh, &config, |stage| match stage {
            BuildStage::Compact("eroded", hf) => initial_areas = hf.areas.clone(),
            BuildStage::Compact("regions", hf) => compact = Some(hf.clone()),
            _ => {}
        })
        .unwrap();
        for (i, flag) in tile.polygons.flags.iter_mut().enumerate() {
            *flag = (i * 13 + 7) as u16;
        }
        Self {
            compact: compact.unwrap(),
            initial_areas,
            tile,
            triangle_areas: vec![AreaType(17); mesh.triangles.len()],
            offset: [[0.; 3]; 32],
        }
    }
    pub fn run(&mut self, mesh: &TriangleMesh) -> u64 {
        let mut h = Hash::new();
        h.word(self.compact.width as u32);
        h.word(self.compact.height as u32);
        h.word(self.compact.spans.len() as u64);
        for batch in 0..16 {
            self.compact.areas.copy_from_slice(&self.initial_areas);
            let x = 2. + batch as f32 * 0.125;
            recast::mark_box_area(
                &mut self.compact,
                [x, -1., 2.],
                [x + 4., 4., 6.],
                AreaType(21),
            );
            recast::mark_cylinder_area(&mut self.compact, [12., 0., 12.], 2.25, 4., AreaType(7));
            let polygon = [[5., 0., 7.], [5., 0., 11.], [9., 0., 11.], [9., 0., 7.]];
            recast::mark_convex_poly_area(&mut self.compact, &polygon, -1., 4., AreaType(3));
            recast::median_filter_walkable_area(&mut self.compact);
            for area in &self.compact.areas {
                h.word(area.0);
            }
            let bounds = recast::calc_bounds(&mesh.vertices).unwrap();
            for p in bounds {
                h.point(p);
            }
            for n in recast::calc_grid_size(bounds, 0.3) {
                h.word(n as u32);
            }
            self.triangle_areas.fill(AreaType(17));
            recast::mark_walkable_triangles(
                30. + batch as f32 * 0.25,
                &mesh.vertices,
                &mesh.triangles,
                &mut self.triangle_areas,
            );
            recast::clear_unwalkable_triangles(
                15.,
                &mesh.vertices,
                &mesh.triangles,
                &mut self.triangle_areas,
            );
            for area in &self.triangle_areas {
                h.word(area.0);
            }
            self.offset.fill([-17., 0.25, 31.]);
            let cap = [0, 4, 16, 32][batch % 4];
            let count = recast::offset_poly(
                &polygon,
                [-0.5, 0., 0.75, 1.25][batch % 4],
                &mut self.offset[..cap],
            );
            h.word(count as u64);
            for p in self.offset {
                h.point(p);
            }
            let copy = recast::copy_poly_mesh(&self.tile.polygons);
            hash_recast_polygons(&mut h, &copy);
            let merged = recast::merge_poly_meshes(&[&self.tile.polygons, &copy]);
            hash_recast_polygons(&mut h, &merged);
            let details = recast::merge_detail_meshes(&[
                Some(&self.tile.detail),
                None,
                Some(&self.tile.detail),
            ]);
            hash_recast_detail(&mut h, &details);
        }
        h.0
    }
}

pub fn recast_layer_capacity(mesh: &TriangleMesh) -> u64 {
    let mut config = super::config();
    config.bounds[1] = [64., 10., 64.];
    let mut h = Hash::new();
    let result = recast_nav::build_tile_observed(mesh, &config, |stage| {
        if let BuildStage::Compact("eroded", hf) = stage {
            h.word(hf.width as u32);
            h.word(hf.height as u32);
            h.word(hf.spans.len() as u64);
            for a in &hf.areas {
                h.word(a.0);
            }
        }
    });
    assert!(
        result
            .unwrap_err()
            .0
            .contains("heightfield layer overlap limit exceeded")
    );
    h.word(1_u32);
    h.0
}
