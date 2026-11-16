// Altered Rust translation of DetourTileCache.cpp, RecastNavigation v1.6.0.
// Copyright (c) 2009-2010 Mikko Mononen. Zlib: vendor/recastnavigation/License.txt.
//! Streaming compressed tiles and deferred native obstacle updates. Cache and
//! obstacle references remain 32-bit under `polyref64`. Rust owns tile bytes;
//! `FREE_DATA` controls whether removal returns them to the caller.
use crate::{
    cache,
    cache_data::{self, Compressor, Header},
    nav::*,
    nav_data,
};

pub type CompressedTileRef = u32;
pub type ObstacleRef = u32;
pub const FREE_DATA: u8 = 1;
pub const EMPTY: u8 = 0;
pub const PROCESSING: u8 = 1;
pub const PROCESSED: u8 = 2;
pub const REMOVING: u8 = 3;
pub const CYLINDER: u8 = 0;
pub const BOX: u8 = 1;
pub const ORIENTED_BOX: u8 = 2;
pub const MAX_TOUCHED_TILES: usize = 8;
const MAX_REQUESTS: usize = 64;
const MAX_UPDATE: usize = 64;

#[derive(Clone, Debug, Default)]
pub struct Params {
    pub origin: Point,
    pub cell_size: f32,
    pub cell_height: f32,
    pub width: i32,
    pub height: i32,
    pub walkable_height: f32,
    pub walkable_radius: f32,
    pub walkable_climb: f32,
    pub max_simplification_error: f32,
    pub max_tiles: i32,
    pub max_obstacles: i32,
}
pub trait MeshProcess {
    fn process(&mut self, params: &mut nav_data::OwnedCreateParams);
}
#[derive(Clone, Debug)]
pub struct CompressedTile {
    pub salt: u32,
    pub header: Option<Header>,
    pub data: Vec<u8>,
    pub flags: u8,
    /// Index in the occupied hash chain or the free list.
    pub next: Option<usize>,
}
impl CompressedTile {
    pub fn compressed(&self) -> &[u8] {
        self.data.get(cache_data::HEADER_SIZE..).unwrap_or(&[])
    }
}
#[derive(Clone, Debug)]
pub struct Obstacle {
    /// Native union storage: cylinder `[position, radius, height]`; box `[min,max]`;
    /// oriented box [center,half extents,rotation auxiliary]. Unused values and
    /// touched/pending tails are retained after processing and removal.
    pub shape: [f32; 8],
    pub touched: [CompressedTileRef; MAX_TOUCHED_TILES],
    pub pending: [CompressedTileRef; MAX_TOUCHED_TILES],
    pub salt: u16,
    pub kind: u8,
    pub state: u8,
    pub touched_count: u8,
    pub pending_count: u8,
    pub next: Option<usize>,
}
impl Default for Obstacle {
    fn default() -> Self {
        Self {
            shape: [0.; 8],
            touched: [0; 8],
            pending: [0; 8],
            salt: 1,
            kind: CYLINDER,
            state: EMPTY,
            touched_count: 0,
            pending_count: 0,
            next: None,
        }
    }
}
impl Obstacle {
    pub fn bounds(&self) -> [Point; 2] {
        let a = &self.shape;
        match self.kind {
            CYLINDER => [
                [a[0] - a[3], a[1], a[2] - a[3]],
                [a[0] + a[3], a[1] + a[4], a[2] + a[3]],
            ],
            BOX => [[a[0], a[1], a[2]], [a[3], a[4], a[5]]],
            ORIENTED_BOX => {
                let r = 1.41 * if a[3] > a[5] { a[3] } else { a[5] };
                [
                    [a[0] - r, a[1] - a[4], a[2] - r],
                    [a[0] + r, a[1] + a[4], a[2] + r],
                ]
            }
            _ => [[0.; 3]; 2],
        }
    }
}
#[derive(Clone, Copy, Debug)]
struct Request {
    remove: bool,
    reference: ObstacleRef,
}

pub struct TileCache<C> {
    params: Params,
    compressor: C,
    mesh_process: Option<Box<dyn MeshProcess>>,
    tiles: Vec<CompressedTile>,
    obstacles: Vec<Obstacle>,
    lookup: Vec<Option<usize>>,
    free_tile: Option<usize>,
    free_obstacle: Option<usize>,
    tile_bits: u32,
    salt_bits: u32,
    requests: Vec<Request>,
    updates: Vec<CompressedTileRef>,
}
fn allocated<T>(count: usize, mut value: impl FnMut(usize) -> T) -> Result<Vec<T>, u32> {
    let mut out = Vec::new();
    out.try_reserve_exact(count)
        .map_err(|_| FAILURE | OUT_OF_MEMORY)?;
    out.extend((0..count).map(&mut value));
    Ok(out)
}
fn integer(value: f32) -> Option<i32> {
    (value.is_finite() && (-2147483648.0..2147483648.0).contains(&value)).then_some(value as i32)
}
impl<C: Compressor> TileCache<C> {
    pub fn new(
        params: Params,
        compressor: C,
        mesh_process: Option<Box<dyn MeshProcess>>,
    ) -> Result<Self, u32> {
        let mut cache = Self {
            params: Params::default(),
            compressor,
            mesh_process,
            tiles: vec![],
            obstacles: vec![],
            lookup: vec![],
            free_tile: None,
            free_obstacle: None,
            tile_bits: 0,
            salt_bits: 0,
            requests: vec![],
            updates: vec![],
        };
        cache.reinitialize(params)?;
        Ok(cache)
    }
    /// Replaces the pools and clears requests. Native retains already queued
    /// tile rebuilds across reinitialization, including their old references.
    pub fn reinitialize(&mut self, params: Params) -> Result<(), u32> {
        if params.max_tiles < 0 || params.max_obstacles < 0 {
            return Err(FAILURE | INVALID_PARAM);
        }
        let n = params.max_tiles as usize;
        let tile_bits = if n == 0 {
            0
        } else {
            n.next_power_of_two().ilog2()
        };
        let salt_bits = 31.min(32 - tile_bits);
        if salt_bits < 10 {
            return Err(FAILURE | INVALID_PARAM);
        }
        let obstacles = allocated(params.max_obstacles as usize, |i| Obstacle {
            next: (i + 1 < params.max_obstacles as usize).then_some(i + 1),
            ..Obstacle::default()
        })?;
        let tiles = allocated(n, |i| CompressedTile {
            salt: 1,
            header: None,
            data: vec![],
            flags: 0,
            next: (i + 1 < n).then_some(i + 1),
        })?;
        let lookup = allocated((n / 4).next_power_of_two().max(1), |_| None)?;
        self.free_tile = (!tiles.is_empty()).then_some(0);
        self.free_obstacle = (!obstacles.is_empty()).then_some(0);
        self.params = params;
        self.tiles = tiles;
        self.obstacles = obstacles;
        self.lookup = lookup;
        self.tile_bits = tile_bits;
        self.salt_bits = salt_bits;
        self.requests.clear();
        Ok(())
    }
    pub fn params(&self) -> &Params {
        &self.params
    }
    pub fn compressor(&mut self) -> &mut C {
        &mut self.compressor
    }
    pub fn set_mesh_process(&mut self, process: Option<Box<dyn MeshProcess>>) {
        self.mesh_process = process;
    }
    pub fn tiles(&self) -> &[CompressedTile] {
        &self.tiles
    }
    pub fn obstacles(&self) -> &[Obstacle] {
        &self.obstacles
    }
    pub fn tile_bits(&self) -> u32 {
        self.tile_bits
    }
    pub fn salt_bits(&self) -> u32 {
        self.salt_bits
    }
    pub fn encode_tile_id(&self, salt: u32, tile: u32) -> CompressedTileRef {
        (salt << self.tile_bits) | tile
    }
    pub fn decode_tile_id(&self, reference: CompressedTileRef) -> (u32, u32) {
        (
            (reference >> self.tile_bits) & ((1u32 << self.salt_bits) - 1),
            reference & ((1u32 << self.tile_bits) - 1),
        )
    }
    pub fn encode_obstacle_id(salt: u32, index: u32) -> ObstacleRef {
        (salt << 16) | index
    }
    pub fn decode_obstacle_id(reference: ObstacleRef) -> (u32, u32) {
        (reference >> 16, reference & 65535)
    }
    pub fn tile_ref(&self, index: usize) -> CompressedTileRef {
        self.tiles
            .get(index)
            .map_or(0, |t| self.encode_tile_id(t.salt, index as u32))
    }
    pub fn obstacle_ref(&self, index: usize) -> ObstacleRef {
        self.obstacles
            .get(index)
            .map_or(0, |o| Self::encode_obstacle_id(o.salt.into(), index as u32))
    }
    pub fn tile_by_ref(&self, reference: CompressedTileRef) -> Option<&CompressedTile> {
        let (salt, index) = self.decode_tile_id(reference);
        self.tiles
            .get(index as usize)
            .filter(|t| reference != 0 && t.salt == salt)
    }
    pub fn obstacle_by_ref(&self, reference: ObstacleRef) -> Option<&Obstacle> {
        let (salt, index) = Self::decode_obstacle_id(reference);
        self.obstacles
            .get(index as usize)
            .filter(|o| reference != 0 && u32::from(o.salt) == salt)
    }
    fn hash(&self, x: i32, y: i32) -> usize {
        ((x as u32)
            .wrapping_mul(0x8da6b343)
            .wrapping_add((y as u32).wrapping_mul(0xd8163841)) as usize)
            & (self.lookup.len() - 1)
    }
    pub fn tiles_at(&self, x: i32, y: i32, capacity: usize) -> Vec<CompressedTileRef> {
        let mut result = vec![];
        let mut at = self.lookup[self.hash(x, y)];
        while let Some(i) = at {
            let tile = &self.tiles[i];
            at = tile.next;
            if result.len() < capacity
                && tile
                    .header
                    .as_ref()
                    .is_some_and(|h| h.coordinate[..2] == [x, y])
            {
                result.push(self.tile_ref(i));
            }
        }
        result
    }
    pub fn tile_at(&self, coordinate: [i32; 3]) -> Option<usize> {
        let mut at = self.lookup[self.hash(coordinate[0], coordinate[1])];
        while let Some(i) = at {
            let tile = &self.tiles[i];
            if tile
                .header
                .as_ref()
                .is_some_and(|h| h.coordinate == coordinate)
            {
                return Some(i);
            }
            at = tile.next;
        }
        None
    }
    pub fn add_tile(&mut self, data: Vec<u8>, flags: u8) -> Result<CompressedTileRef, u32> {
        let header = Header::parse(&data)?;
        if data.len() > i32::MAX as usize {
            return Err(FAILURE | INVALID_PARAM);
        }
        if self.tile_at(header.coordinate).is_some() {
            return Err(FAILURE);
        }
        let i = self.free_tile.ok_or(FAILURE | OUT_OF_MEMORY)?;
        self.free_tile = self.tiles[i].next;
        let hash = self.hash(header.coordinate[0], header.coordinate[1]);
        let tile = &mut self.tiles[i];
        tile.next = self.lookup[hash];
        self.lookup[hash] = Some(i);
        tile.header = Some(header);
        tile.data = data;
        tile.flags = flags;
        Ok(self.tile_ref(i))
    }
    pub fn remove_tile(&mut self, reference: CompressedTileRef) -> Result<Option<Vec<u8>>, u32> {
        let tile = self.tile_by_ref(reference).ok_or(FAILURE | INVALID_PARAM)?;
        let coordinate = tile
            .header
            .as_ref()
            .ok_or(FAILURE | INVALID_PARAM)?
            .coordinate;
        let index = self.decode_tile_id(reference).1 as usize;
        let hash = self.hash(coordinate[0], coordinate[1]);
        let mut at = self.lookup[hash];
        let mut previous: Option<usize> = None;
        while let Some(i) = at {
            if i == index {
                if let Some(p) = previous {
                    self.tiles[p].next = self.tiles[i].next;
                } else {
                    self.lookup[hash] = self.tiles[i].next;
                }
                break;
            }
            previous = at;
            at = self.tiles[i].next;
        }
        let tile = &mut self.tiles[index];
        let data = std::mem::take(&mut tile.data);
        let result = (tile.flags & FREE_DATA == 0).then_some(data);
        tile.header = None;
        tile.flags = 0;
        tile.salt = (tile.salt + 1) & ((1u32 << self.salt_bits) - 1);
        if tile.salt == 0 {
            tile.salt = 1;
        }
        tile.next = self.free_tile;
        self.free_tile = Some(index);
        Ok(result)
    }
    pub fn tight_tile_bounds(&self, header: &Header) -> [Point; 2] {
        let cs = self.params.cell_size;
        [
            [
                madd(
                    f32::from(header.usable_bounds[0][0]),
                    cs,
                    header.bounds[0][0],
                ),
                header.bounds[0][1],
                madd(
                    f32::from(header.usable_bounds[0][1]),
                    cs,
                    header.bounds[0][2],
                ),
            ],
            [
                madd(
                    f32::from(header.usable_bounds[1][0]) + 1.,
                    cs,
                    header.bounds[0][0],
                ),
                header.bounds[1][1],
                madd(
                    f32::from(header.usable_bounds[1][1]) + 1.,
                    cs,
                    header.bounds[0][2],
                ),
            ],
        ]
    }
    pub fn query_tiles(
        &self,
        bounds: [Point; 2],
        capacity: usize,
    ) -> Result<Vec<CompressedTileRef>, u32> {
        let tw = self.params.width as f32 * self.params.cell_size;
        let th = self.params.height as f32 * self.params.cell_size;
        let mut range = [[0; 2]; 2];
        for side in 0..2 {
            range[side][0] = integer(((bounds[side][0] - self.params.origin[0]) / tw).floor())
                .ok_or(FAILURE | INVALID_PARAM)?;
            range[side][1] = integer(((bounds[side][2] - self.params.origin[2]) / th).floor())
                .ok_or(FAILURE | INVALID_PARAM)?;
        }
        let mut result = vec![];
        for y in range[0][1]..=range[1][1] {
            for x in range[0][0]..=range[1][0] {
                for r in self.tiles_at(x, y, 32) {
                    let tile = self.tile_by_ref(r).unwrap();
                    let tb = self.tight_tile_bounds(tile.header.as_ref().unwrap());
                    if !(0..3).any(|i| bounds[0][i] > tb[1][i] || bounds[1][i] < tb[0][i])
                        && result.len() < capacity
                    {
                        result.push(r);
                    }
                }
            }
        }
        Ok(result)
    }
    fn add_shape(&mut self, kind: u8, shape: [f32; 8]) -> Result<ObstacleRef, u32> {
        if self.requests.len() >= MAX_REQUESTS {
            return Err(FAILURE | BUFFER_TOO_SMALL);
        }
        let i = self.free_obstacle.ok_or(FAILURE | OUT_OF_MEMORY)?;
        let ob = &mut self.obstacles[i];
        self.free_obstacle = ob.next;
        *ob = Obstacle {
            salt: ob.salt,
            state: PROCESSING,
            kind,
            shape,
            ..Obstacle::default()
        };
        let reference = self.obstacle_ref(i);
        self.requests.push(Request {
            remove: false,
            reference,
        });
        Ok(reference)
    }
    pub fn add_cylinder(
        &mut self,
        position: Point,
        radius: f32,
        height: f32,
    ) -> Result<ObstacleRef, u32> {
        self.add_shape(
            CYLINDER,
            [
                position[0],
                position[1],
                position[2],
                radius,
                height,
                0.,
                0.,
                0.,
            ],
        )
    }
    pub fn add_box(&mut self, bounds: [Point; 2]) -> Result<ObstacleRef, u32> {
        self.add_shape(
            BOX,
            [
                bounds[0][0],
                bounds[0][1],
                bounds[0][2],
                bounds[1][0],
                bounds[1][1],
                bounds[1][2],
                0.,
                0.,
            ],
        )
    }
    pub fn add_oriented_box(
        &mut self,
        center: Point,
        half_extents: Point,
        angle: f32,
    ) -> Result<ObstacleRef, u32> {
        let cos = crate::common::cosine(0.5 * angle);
        let sin = crate::common::sine(-0.5 * angle);
        self.add_shape(
            ORIENTED_BOX,
            [
                center[0],
                center[1],
                center[2],
                half_extents[0],
                half_extents[1],
                half_extents[2],
                cos * sin,
                madd(cos, cos, -0.5),
            ],
        )
    }
    pub fn remove_obstacle(&mut self, reference: ObstacleRef) -> u32 {
        if reference == 0 {
            return SUCCESS;
        }
        if self.requests.len() >= MAX_REQUESTS {
            return FAILURE | BUFFER_TOO_SMALL;
        }
        self.requests.push(Request {
            remove: true,
            reference,
        });
        SUCCESS
    }
    pub fn build_tiles_at(&mut self, x: i32, y: i32, mesh: &mut NavMesh) -> u32 {
        for r in self.tiles_at(x, y, 32) {
            let status = self.build_tile(r, mesh);
            if status & FAILURE != 0 {
                return status;
            }
        }
        SUCCESS
    }
    pub fn build_tile(&mut self, reference: CompressedTileRef, mesh: &mut NavMesh) -> u32 {
        if self.tile_by_ref(reference).is_none() {
            return FAILURE | INVALID_PARAM;
        }
        let tile = &self.tiles[self.decode_tile_id(reference).1 as usize];
        let Some(header) = tile.header.as_ref() else {
            return FAILURE | INVALID_PARAM;
        };
        let coordinate = header.coordinate;
        let Some(climb) = integer(self.params.walkable_climb / self.params.cell_height) else {
            return FAILURE | INVALID_PARAM;
        };
        let mut decompressed = match cache_data::decompress_layer(&mut self.compressor, &tile.data)
        {
            Ok(l) => l,
            Err(s) => return s,
        };
        let (mut layer, regions) =
            decompressed.heightfield_mut(self.params.cell_size, self.params.cell_height);
        for ob in &self.obstacles {
            if ob.state == EMPTY
                || ob.state == REMOVING
                || !ob.touched[..ob.touched_count as usize].contains(&reference)
            {
                continue;
            }
            let a = ob.shape;
            let status = match ob.kind {
                CYLINDER => {
                    cache::mark_cylinder_area(&mut layer, [a[0], a[1], a[2]], a[3], a[4], 0)
                }
                BOX => cache::mark_box_area(&mut layer, [a[0], a[1], a[2]], [a[3], a[4], a[5]], 0),
                ORIENTED_BOX => cache::mark_oriented_box_area(
                    &mut layer,
                    [a[0], a[1], a[2]],
                    [a[3], a[4], a[5]],
                    [a[6], a[7]],
                    0,
                ),
                _ => SUCCESS,
            };
            if status & FAILURE != 0 {
                return status;
            }
        }
        let built = match cache::build_cache_mesh_in(
            &layer,
            climb,
            self.params.max_simplification_error,
            regions,
        ) {
            Ok(mesh) => mesh,
            Err(e) => return e.status,
        };
        if built.polygons.is_empty() {
            mesh.remove_tile(coordinate);
            return SUCCESS;
        }
        let mut params = nav_data::OwnedCreateParams {
            vertices: built.vertices,
            polygons: built
                .polygons
                .into_iter()
                .enumerate()
                .map(|(i, p)| nav_data::CreatePolygon {
                    vertices: p[..6].try_into().unwrap(),
                    neighbors: p[6..].try_into().unwrap(),
                    flags: built.flags[i],
                    area: built.areas[i],
                })
                .collect(),
            vertices_per_polygon: 6,
            coordinate,
            bounds: header.bounds,
            walkable_height: self.params.walkable_height,
            walkable_radius: self.params.walkable_radius,
            walkable_climb: self.params.walkable_climb,
            cell_size: self.params.cell_size,
            cell_height: self.params.cell_height,
            ..nav_data::OwnedCreateParams::default()
        };
        if let Some(process) = &mut self.mesh_process {
            process.process(&mut params);
        }
        let data = match nav_data::create(&params.as_params()) {
            Ok(d) => d,
            Err(_) => return FAILURE,
        };
        mesh.remove_tile(coordinate);
        match mesh.add_tile_data_owned(data, crate::nav::TILE_FREE_DATA, 0) {
            Ok(_) => SUCCESS,
            Err(s) => s,
        }
    }
    pub fn update(&mut self, _dt: f32, mesh: &mut NavMesh) -> (u32, bool) {
        if self.updates.is_empty() {
            for request in &self.requests {
                let (salt, index) = Self::decode_obstacle_id(request.reference);
                let index = index as usize;
                let Some(ob) = self.obstacles.get(index) else {
                    continue;
                };
                if u32::from(ob.salt) != salt {
                    continue;
                }
                if request.remove {
                    self.obstacles[index].state = REMOVING;
                } else {
                    let touched = match self.query_tiles(ob.bounds(), MAX_TOUCHED_TILES) {
                        Ok(t) => t,
                        Err(s) => {
                            self.requests.clear();
                            return (s, false);
                        }
                    };
                    let ob = &mut self.obstacles[index];
                    ob.touched[..touched.len()].copy_from_slice(&touched);
                    ob.touched_count = touched.len() as u8;
                }
                let ob = &mut self.obstacles[index];
                ob.pending_count = 0;
                for &r in &ob.touched[..ob.touched_count as usize] {
                    if self.updates.len() < MAX_UPDATE {
                        if !self.updates.contains(&r) {
                            self.updates.push(r);
                        }
                        ob.pending[ob.pending_count as usize] = r;
                        ob.pending_count += 1;
                    }
                }
            }
            self.requests.clear();
        }
        let mut status = SUCCESS;
        if let Some(&r) = self.updates.first() {
            status = self.build_tile(r, mesh);
            // Native consumes a rebuild and advances states even when it fails.
            self.updates.remove(0);
            for (i, ob) in self.obstacles.iter_mut().enumerate() {
                if ob.state != PROCESSING && ob.state != REMOVING {
                    continue;
                }
                if let Some(j) = ob.pending[..ob.pending_count as usize]
                    .iter()
                    .position(|&p| p == r)
                {
                    ob.pending[j] = ob.pending[ob.pending_count as usize - 1];
                    ob.pending_count -= 1;
                }
                if ob.pending_count == 0 {
                    if ob.state == PROCESSING {
                        ob.state = PROCESSED;
                    } else {
                        ob.state = EMPTY;
                        ob.salt = ob.salt.wrapping_add(1);
                        if ob.salt == 0 {
                            ob.salt = 1;
                        }
                        ob.next = self.free_obstacle;
                        self.free_obstacle = Some(i);
                    }
                }
            }
        }
        (status, self.updates.is_empty() && self.requests.is_empty())
    }
}
