//! Convenience runtime for tiled generation, box obstacles, and path queries.
//!
//! Layer construction and Detour algorithms follow the pinned upstream reference.
//! This facade adds keyed obstacle tracking, update revisions, and query policies;
//! it is not a one-to-one translation of the upstream public API.
// Tile-cache lifecycle is an altered translation of DetourTileCache.cpp (v1.6.0).
// Copyright (c) 2009-2010 Mikko Mononen. Zlib: vendor/recastnavigation/License.txt.
use crate::{
    BuildConfig, BuildError, TriangleMesh, build_layers,
    cache::{build_cache_mesh, mark_box},
    layers::HeightfieldLayer,
    nav::{self, Filter, NavMesh, Point},
    validate_mesh,
};
use std::{
    collections::{BTreeMap, HashSet, VecDeque},
    sync::{
        Condvar, Mutex, OnceLock,
        atomic::{AtomicUsize, Ordering},
    },
};

fn navigation_filter() -> Filter {
    // This facade assigns flag 1 to generated walkable ground polygons.
    Filter {
        include: 1,
        ..Filter::default()
    }
}

// Share the same four-worker ceiling across simultaneous zone allocations.
static ACTIVE_BUILDS: Mutex<usize> = Mutex::new(0);
static BUILD_AVAILABLE: Condvar = Condvar::new();
static BUILD_LIMIT: OnceLock<usize> = OnceLock::new();
struct BuildPermit;
impl BuildPermit {
    fn acquire() -> Self {
        let limit = *BUILD_LIMIT.get_or_init(|| {
            std::thread::available_parallelism()
                .map_or(1, usize::from)
                .min(4)
        });
        let mut active = ACTIVE_BUILDS.lock().unwrap();
        while *active >= limit {
            active = BUILD_AVAILABLE.wait(active).unwrap();
        }
        *active += 1;
        Self
    }
}
impl Drop for BuildPermit {
    fn drop(&mut self) {
        *ACTIVE_BUILDS.lock().unwrap() -= 1;
        BUILD_AVAILABLE.notify_one();
    }
}
#[derive(Clone, Debug)]
pub struct NavigationSettings {
    pub origin: Point,
    pub maximum_y: f32,
    pub tiles: [u16; 2],
    pub tile_voxels: u16,
    pub cell_size: f32,
    pub cell_height: f32,
    pub agent_height: f32,
    pub agent_radius: f32,
    pub agent_climb: f32,
    pub slope_degrees: f32,
    pub max_tiles: usize,
    pub max_polygons_per_tile: usize,
    pub max_obstacles: usize,
    pub workers: usize,
}
impl Default for NavigationSettings {
    fn default() -> Self {
        Self {
            origin: [0., -1., 0.],
            maximum_y: 8.,
            tiles: [2, 2],
            tile_voxels: 16,
            cell_size: 0.5,
            cell_height: 0.25,
            agent_height: 1.8,
            agent_radius: 0.4,
            agent_climb: 0.9,
            slope_degrees: 45.,
            max_tiles: 128,
            max_polygons_per_tile: 4096,
            max_obstacles: 4096,
            workers: 4,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoxObstacle {
    pub min: Point,
    pub max: Point,
}
impl BoxObstacle {
    fn valid(self) -> bool {
        self.min.iter().chain(&self.max).all(|v| v.is_finite())
            && (0..3).all(|i| self.min[i] <= self.max[i])
    }
}
#[derive(Clone, Debug)]
pub struct CachedLayer {
    pub coordinate: [i32; 3],
    pub layer: HeightfieldLayer,
}
#[derive(Clone, Debug)]
pub struct Obstacle {
    pub salt: u16,
    pub state: u8,
    pub bounds: BoxObstacle,
    pub touched: Vec<usize>,
    pub pending: Vec<usize>,
}
impl Default for Obstacle {
    fn default() -> Self {
        Self {
            salt: 1,
            state: 0,
            bounds: BoxObstacle {
                min: [0.; 3],
                max: [0.; 3],
            },
            touched: vec![],
            pending: vec![],
        }
    }
}
#[derive(Clone, Debug)]
pub struct UpdateProgress {
    pub up_to_date: bool,
    pub rebuilt_layer: Option<usize>,
}
#[derive(Clone, Debug, PartialEq)]
pub struct Path {
    pub points: Vec<Point>,
    pub reaches_goal: bool,
    pub corridor_status: u32,
    pub straight_status: u32,
}
#[derive(Clone, Debug)]
pub struct Navigation {
    nav: NavMesh,
    settings: NavigationSettings,
    layers: Vec<CachedLayer>,
    obstacles: Vec<Obstacle>,
    free_obstacles: VecDeque<usize>,
    requests: Vec<(bool, u32)>,
    updates: VecDeque<usize>,
    keys: BTreeMap<u64, (u32, BoxObstacle)>,
    revision: u64,
    requested_revision: u64,
}
impl NavigationSettings {
    pub fn build_config(&self, x: u16, z: u16) -> Result<BuildConfig, BuildError> {
        if self.tiles.contains(&0)
            || self.tile_voxels == 0
            || self.tile_voxels > 255
            || self.max_obstacles == 0
            || self.max_obstacles > 65536
            || self.workers == 0
            || ![self.agent_height, self.agent_radius, self.agent_climb]
                .iter()
                .all(|v| v.is_finite() && *v >= 0.)
        {
            return Err(BuildError("invalid tiled navigation settings".into()));
        }
        let radius = (self.agent_radius / self.cell_size).ceil();
        if radius > 127. {
            return Err(BuildError(
                "agent radius exceeds layer builder limit".into(),
            ));
        }
        let border = radius as u16 + 3;
        let tile = self.tile_voxels as f32 * self.cell_size;
        let pad = border as f32 * self.cell_size;
        let lo = [
            self.origin[0] + x as f32 * tile,
            self.origin[1],
            self.origin[2] + z as f32 * tile,
        ];
        let config = BuildConfig {
            bounds: [
                [lo[0] - pad, lo[1], lo[2] - pad],
                [lo[0] + tile + pad, self.maximum_y, lo[2] + tile + pad],
            ],
            cell_size: self.cell_size,
            cell_height: self.cell_height,
            slope_degrees: self.slope_degrees,
            walkable_height: (self.agent_height / self.cell_height).ceil() as i32,
            walkable_climb: (self.agent_climb / self.cell_height).ceil() as i32,
            walkable_radius: radius as i32,
            border_size: i32::from(border),
            ..Default::default()
        };
        config.validate()?;
        Ok(config)
    }
}
// Preserve source triangle order and include the closed padded raster bounds.
// Coarse tile ranges avoid scanning the entire zone once for every tile; the
// exact overlap check handles triangles on a boundary and non-binary cell sizes.
fn tile_triangles(mesh: &TriangleMesh, settings: &NavigationSettings) -> Vec<Vec<usize>> {
    let width = i32::from(settings.tiles[0]);
    let height = i32::from(settings.tiles[1]);
    let tile = settings.tile_voxels as f32 * settings.cell_size;
    let border = ((settings.agent_radius / settings.cell_size).ceil() + 3.) * settings.cell_size;
    let mut output = vec![Vec::new(); width as usize * height as usize];
    let origin = [settings.origin[0], settings.origin[2]];
    for (index, triangle) in mesh.triangles.iter().enumerate() {
        let vertices = triangle.map(|i| mesh.vertices[i as usize]);
        let min = [0, 2].map(|axis| {
            vertices[0][axis]
                .min(vertices[1][axis])
                .min(vertices[2][axis])
        });
        let max = [0, 2].map(|axis| {
            vertices[0][axis]
                .max(vertices[1][axis])
                .max(vertices[2][axis])
        });
        let lo: [i32; 2] = std::array::from_fn(|axis| {
            (((min[axis] - origin[axis] - border) / tile).floor() as i32)
                .saturating_sub(1)
                .max(0)
        });
        let hi: [i32; 2] = std::array::from_fn(|axis| {
            (((max[axis] - origin[axis] + border) / tile).floor() as i32)
                .saturating_add(1)
                .min([width, height][axis] - 1)
        });
        for z in lo[1]..=hi[1] {
            for x in lo[0]..=hi[0] {
                let base = [origin[0] + x as f32 * tile, origin[1] + z as f32 * tile];
                if (0..2).all(|axis| {
                    min[axis] <= base[axis] + tile + border && max[axis] >= base[axis] - border
                }) {
                    output[(z * width + x) as usize].push(index);
                }
            }
        }
    }
    output
}

fn selected_mesh(mesh: &TriangleMesh, triangles: &[usize]) -> TriangleMesh {
    let mut output = TriangleMesh {
        vertices: Vec::with_capacity(triangles.len() * 3),
        triangles: Vec::with_capacity(triangles.len()),
    };
    for &index in triangles {
        let base = output.vertices.len() as u32;
        output
            .vertices
            .extend(mesh.triangles[index].map(|i| mesh.vertices[i as usize]));
        output.triangles.push([base, base + 1, base + 2]);
    }
    output
}

impl Navigation {
    /// Build independent padded tiles in bounded parallel jobs and assemble in row-major order.
    pub fn build(mesh: &TriangleMesh, settings: NavigationSettings) -> Result<Self, BuildError> {
        settings.build_config(0, 0)?;
        validate_mesh(mesh)?;
        let tile_world = settings.tile_voxels as f32 * settings.cell_size;
        let nav = NavMesh::new(
            settings.origin,
            tile_world,
            tile_world,
            settings.max_tiles,
            settings.max_polygons_per_tile,
        )?;
        let triangles = tile_triangles(mesh, &settings);
        let count = settings.tiles[0] as usize * settings.tiles[1] as usize;
        let next = AtomicUsize::new(0);
        type TileResult = Option<Result<Vec<HeightfieldLayer>, BuildError>>;
        let results: Mutex<Vec<TileResult>> = Mutex::new((0..count).map(|_| None).collect());
        std::thread::scope(|scope| {
            for _ in 0..settings.workers.min(count).min(4) {
                let permit = BuildPermit::acquire();
                let settings = &settings;
                let triangles = &triangles;
                let next = &next;
                let results = &results;
                scope.spawn(move || {
                    let _permit = permit;
                    loop {
                        let i = next.fetch_add(1, Ordering::Relaxed);
                        if i >= count {
                            break;
                        }
                        let result = settings
                            .build_config(
                                (i % settings.tiles[0] as usize) as u16,
                                (i / settings.tiles[0] as usize) as u16,
                            )
                            .and_then(|config| {
                                if triangles[i].is_empty() {
                                    Ok(Vec::new())
                                } else {
                                    build_layers(&selected_mesh(mesh, &triangles[i]), &config)
                                }
                            });
                        results.lock().unwrap()[i] = Some(result);
                    }
                });
            }
        });
        let mut output = Self {
            nav,
            obstacles: vec![Obstacle::default(); settings.max_obstacles],
            free_obstacles: (0..settings.max_obstacles).collect(),
            settings,
            layers: vec![],
            requests: vec![],
            updates: VecDeque::new(),
            keys: BTreeMap::new(),
            revision: 0,
            requested_revision: 0,
        };
        for (index, result) in results.into_inner().unwrap().into_iter().enumerate() {
            let x = (index % output.settings.tiles[0] as usize) as i32;
            let z = (index / output.settings.tiles[0] as usize) as i32;
            for (id, layer) in result
                .expect("each tile job produced a result")?
                .into_iter()
                .enumerate()
            {
                if output.layers.len() == output.settings.max_tiles {
                    return Err(BuildError("compressed tile capacity exhausted".into()));
                }
                output.layers.push(CachedLayer {
                    coordinate: [x, z, id as i32],
                    layer,
                });
                // Match addTile followed by buildNavMeshTilesAt, including layer rebuild order.
                let at: Vec<_> = (0..output.layers.len())
                    .rev()
                    .filter(|&i| output.layers[i].coordinate[..2] == [x, z])
                    .take(32)
                    .collect();
                for i in at {
                    output.rebuild(i)?;
                }
            }
        }
        Ok(output)
    }
    pub fn nav_mesh(&self) -> &NavMesh {
        &self.nav
    }
    pub fn settings(&self) -> &NavigationSettings {
        &self.settings
    }
    pub fn layers(&self) -> &[CachedLayer] {
        &self.layers
    }
    pub fn obstacles(&self) -> &[Obstacle] {
        &self.obstacles
    }
    fn rebuild(&mut self, index: usize) -> Result<(), BuildError> {
        let cached = &self.layers[index];
        let mut layer = cached.layer.clone();
        for obstacle in &self.obstacles {
            if obstacle.state != 0 && obstacle.state != 3 && obstacle.touched.contains(&index) {
                mark_box(&mut layer, obstacle.bounds.min, obstacle.bounds.max);
            }
        }
        let climb = (self.settings.agent_climb / self.settings.cell_height) as i32;
        let build = build_cache_mesh(&layer, climb, 0.)?;
        self.nav.remove_tile(cached.coordinate);
        self.nav.add_tile(
            cached.coordinate,
            &layer,
            &build.mesh,
            self.settings.agent_height,
            self.settings.agent_radius,
            self.settings.agent_climb,
        )
    }
    /// Low-level Detour box request. Returns a salted obstacle reference or Detour status.
    pub fn add_box(&mut self, bounds: BoxObstacle) -> Result<u32, u32> {
        if !bounds.valid() {
            return Err(nav::FAILURE | nav::INVALID_PARAM);
        }
        if self.requests.len() >= 64 {
            return Err(nav::FAILURE | nav::BUFFER_TOO_SMALL);
        }
        let index = self.free_obstacles.pop_front().ok_or(nav::FAILURE | 4)?;
        let salt = self.obstacles[index].salt;
        self.obstacles[index] = Obstacle {
            salt,
            state: 1,
            bounds,
            touched: vec![],
            pending: vec![],
        };
        let reference = ((salt as u32) << 16) | index as u32;
        self.requests.push((true, reference));
        Ok(reference)
    }
    /// Queue removal; stale handles are ignored when requests are processed, as in Detour.
    pub fn remove_obstacle(&mut self, reference: u32) -> u32 {
        if reference == 0 {
            return nav::SUCCESS;
        }
        if self.requests.len() >= 64 {
            return nav::FAILURE | nav::BUFFER_TOO_SMALL;
        }
        self.requests.push((false, reference));
        nav::SUCCESS
    }
    pub fn obstacle(&self, reference: u32) -> Option<&Obstacle> {
        let obstacle = self.obstacles.get((reference & 65535) as usize)?;
        if obstacle.salt as u32 == reference >> 16 {
            Some(obstacle)
        } else {
            None
        }
    }
    fn touched(&self, bounds: BoxObstacle) -> Vec<usize> {
        let world = self.settings.tile_voxels as f32 * self.settings.cell_size;
        let lo = [
            ((bounds.min[0] - self.settings.origin[0]) / world).floor() as i32,
            ((bounds.min[2] - self.settings.origin[2]) / world).floor() as i32,
        ];
        let hi = [
            ((bounds.max[0] - self.settings.origin[0]) / world).floor() as i32,
            ((bounds.max[2] - self.settings.origin[2]) / world).floor() as i32,
        ];
        let mut output = Vec::new();
        for z in lo[1]..=hi[1] {
            for x in lo[0]..=hi[0] {
                for i in (0..self.layers.len())
                    .rev()
                    .filter(|&i| self.layers[i].coordinate[..2] == [x, z])
                    .take(32)
                {
                    let l = &self.layers[i].layer;
                    let cs = l.cell_size;
                    let bmin = [
                        l.bounds[0][0] + l.min_x as f32 * cs,
                        l.bounds[0][1],
                        l.bounds[0][2] + l.min_z as f32 * cs,
                    ];
                    let bmax = [
                        l.bounds[0][0] + (l.max_x + 1) as f32 * cs,
                        l.bounds[1][1],
                        l.bounds[0][2] + (l.max_z + 1) as f32 * cs,
                    ];
                    if (0..3).all(|a| bounds.min[a] <= bmax[a] && bounds.max[a] >= bmin[a])
                        && output.len() < 8
                    {
                        output.push(i);
                    }
                }
            }
        }
        output
    }
    /// Advance exactly one tile-cache update, preserving request and pending-state semantics.
    pub fn update(&mut self) -> Result<UpdateProgress, BuildError> {
        if self.updates.is_empty() {
            for &(add, reference) in &self.requests {
                let index = (reference & 65535) as usize;
                if self
                    .obstacles
                    .get(index)
                    .is_none_or(|o| o.salt as u32 != reference >> 16)
                {
                    continue;
                }
                if add {
                    self.obstacles[index].touched = self.touched(self.obstacles[index].bounds);
                } else {
                    self.obstacles[index].state = 3;
                }
                let obstacle = &mut self.obstacles[index];
                obstacle.pending.clear();
                for &tile in &obstacle.touched {
                    if self.updates.len() < 64 {
                        if !self.updates.contains(&tile) {
                            self.updates.push_back(tile);
                        }
                        obstacle.pending.push(tile);
                    }
                }
            }
            self.requests.clear();
        }
        let rebuilt = self.updates.front().copied();
        if let Some(index) = rebuilt {
            self.rebuild(index)?;
            self.updates.pop_front();
            self.revision = self.revision.wrapping_add(1);
            for (i, obstacle) in self.obstacles.iter_mut().enumerate() {
                if obstacle.state != 1 && obstacle.state != 3 {
                    continue;
                }
                if let Some(j) = obstacle.pending.iter().position(|&t| t == index) {
                    obstacle.pending.swap_remove(j);
                }
                if obstacle.pending.is_empty() {
                    if obstacle.state == 1 {
                        obstacle.state = 2;
                    } else {
                        obstacle.state = 0;
                        obstacle.salt = obstacle.salt.wrapping_add(1).max(1);
                        self.free_obstacles.push_front(i);
                    }
                }
            }
        }
        Ok(UpdateProgress {
            up_to_date: self.updates.is_empty() && self.requests.is_empty(),
            rebuilt_layer: rebuilt,
        })
    }
    pub fn update_until_up_to_date(&mut self) -> Result<usize, BuildError> {
        let mut calls = 0;
        loop {
            calls += 1;
            if self.update()?.up_to_date {
                return Ok(calls);
            }
        }
    }
    pub fn add_or_replace_box(
        &mut self,
        key: u64,
        bounds: BoxObstacle,
    ) -> Result<bool, BuildError> {
        let previous = self.keys.get(&key).copied();
        if previous.is_some_and(|(_, b)| b == bounds) {
            return Ok(false);
        }
        let requests = if previous.is_some() { 2 } else { 1 };
        if !bounds.valid() || self.requests.len() + requests > 64 || self.free_obstacles.is_empty()
        {
            return Err(BuildError(
                "box request exceeds capacity or contains invalid bounds".into(),
            ));
        }
        if let Some((reference, _)) = previous {
            self.remove_obstacle(reference);
        }
        let reference = self
            .add_box(bounds)
            .map_err(|s| BuildError(format!("add box failed: {s:#x}")))?;
        self.keys.insert(key, (reference, bounds));
        self.requested_revision = self.requested_revision.wrapping_add(1);
        Ok(true)
    }
    pub fn remove_key(&mut self, key: u64) -> Result<bool, BuildError> {
        let Some(&(reference, _)) = self.keys.get(&key) else {
            return Ok(false);
        };
        let status = self.remove_obstacle(reference);
        if status & nav::FAILURE != 0 {
            return Err(BuildError(format!("remove obstacle failed: {status:#x}")));
        }
        self.keys.remove(&key);
        self.requested_revision = self.requested_revision.wrapping_add(1);
        Ok(true)
    }
    pub fn retain_only(&mut self, keep: &HashSet<u64>) -> Result<(), BuildError> {
        let remove: Vec<_> = self
            .keys
            .keys()
            .copied()
            .filter(|k| !keep.contains(k))
            .collect();
        if self.requests.len() + remove.len() > 64 {
            return Err(BuildError(
                "obstacle removal request capacity exceeded".into(),
            ));
        }
        for key in remove {
            self.remove_key(key)?;
        }
        Ok(())
    }
    pub fn revision(&self) -> u64 {
        self.revision
    }
    pub fn requested_revision(&self) -> u64 {
        self.requested_revision
    }
    pub fn key_reference(&self, key: u64) -> Option<u32> {
        self.keys.get(&key).map(|&(reference, _)| reference)
    }
    pub fn obstacle_count(&self) -> usize {
        self.keys.len()
    }
    pub fn debug_obstacles(&self) -> impl Iterator<Item = (u64, BoxObstacle)> + '_ {
        self.keys.iter().map(|(&key, &(_, b))| (key, b))
    }
    pub fn project_point(
        &self,
        point: Point,
        half_extents: Point,
    ) -> Result<Option<Point>, BuildError> {
        let (status, nearest) = self.nav.nearest(point, half_extents, &navigation_filter());
        if status & nav::FAILURE != 0 {
            return Err(BuildError(format!("nearest polygon failed: {status:#x}")));
        }
        Ok(nearest.map(|(_, p)| p))
    }
    pub fn path(
        &self,
        start: Point,
        end: Point,
        allow_partial: bool,
    ) -> Result<Option<Path>, BuildError> {
        let filter = navigation_filter();
        let (ss, start) = self.nav.nearest(start, [1., 100., 1.], &filter);
        let (es, mut goal) = self.nav.nearest(end, [1., 100., 1.], &filter);
        if (ss | es) & nav::FAILURE != 0 {
            return Err(BuildError("invalid path query coordinates".into()));
        }
        let proxy = goal.is_none() && allow_partial;
        if proxy {
            goal = self.nav.nearest(end, [4., 100., 4.], &filter).1;
        }
        let (Some((sr, start)), Some((er, end))) = (start, goal) else {
            return Ok(None);
        };
        let corridor = self.nav.find_path(sr, er, start, end, &filter, 4096, 256);
        if corridor.status & nav::FAILURE != 0 {
            return Err(BuildError(format!(
                "corridor query failed: {:#x}",
                corridor.status
            )));
        }
        let reaches_goal = corridor.polygons.last() == Some(&er)
            && !proxy
            && corridor.status & (nav::BUFFER_TOO_SMALL | nav::OUT_OF_NODES | nav::PARTIAL_RESULT)
                == 0;
        if !reaches_goal && !allow_partial {
            return Ok(None);
        }
        let straight = self.nav.straight_path(start, end, &corridor.polygons, 256);
        if straight.status & nav::FAILURE != 0 {
            return Err(BuildError(format!(
                "straight path query failed: {:#x}",
                straight.status
            )));
        }
        Ok(Some(Path {
            points: straight.points,
            reaches_goal: reaches_goal
                && straight.status & (nav::BUFFER_TOO_SMALL | nav::PARTIAL_RESULT) == 0,
            corridor_status: corridor.status,
            straight_status: straight.status,
        }))
    }
    pub fn path_for_agent(
        &self,
        key: u64,
        start: Point,
        end: Point,
        allow_partial: bool,
    ) -> Result<Option<Path>, BuildError> {
        let start = if let Some((_, b)) = self.keys.get(&key) {
            let extents = [
                ((b.max[0] - b.min[0]) * 0.5 + 0.05).max(1.),
                100.,
                ((b.max[2] - b.min[2]) * 0.5 + 0.05).max(1.),
            ];
            let Some(start) = self.project_point(start, extents)? else {
                return Ok(None);
            };
            start
        } else {
            start
        };
        self.path(start, end, allow_partial)
    }
    pub fn debug_polygons(&self) -> Vec<Vec<Point>> {
        self.nav
            .tiles
            .iter()
            .flatten()
            .flat_map(|tile| {
                tile.polygons().map(|poly| {
                    poly.vertices()
                        .map(|i| tile.vertex_at(usize::from(i)))
                        .collect()
                })
            })
            .collect()
    }
}
