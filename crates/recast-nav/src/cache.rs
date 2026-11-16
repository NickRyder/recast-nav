// Altered Rust translation of DetourTileCacheBuilder.cpp, RecastNavigation v1.6.0.
// Copyright (c) 2009-2010 Mikko Mononen. Zlib: vendor/recastnavigation/License.txt.
use crate::{
    BuildError,
    layers::HeightfieldLayer,
    nav::{BUFFER_TOO_SMALL, FAILURE, INVALID_PARAM},
};
use crate::{ContourSet, TileCacheContourView};
use std::ops::{Deref, DerefMut};

pub const NULL_AREA: u8 = 0;
pub const WALKABLE_AREA: u8 = 63;
pub const NULL_INDEX: u16 = u16::MAX;
pub const VERTICES_PER_POLYGON: usize = 6;

#[derive(Clone, Debug)]
pub struct CacheBuildError {
    pub status: u32,
    pub message: String,
}
impl std::fmt::Display for CacheBuildError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
    }
}
impl std::error::Error for CacheBuildError {}
impl From<CacheBuildError> for BuildError {
    fn from(error: CacheBuildError) -> Self {
        Self(error.message)
    }
}
fn build_error(detail: u32, message: impl Into<String>) -> CacheBuildError {
    CacheBuildError {
        status: FAILURE | detail,
        message: message.into(),
    }
}

#[derive(Clone, Debug)]
pub struct CacheContour {
    pub region: u8,
    pub area: u8,
    pub vertices: Vec<[u8; 4]>,
}
#[derive(Clone, Debug, Default)]
pub struct CacheMesh {
    pub vertices: Vec<[u16; 3]>,
    pub polygons: Vec<[u16; 12]>,
    pub areas: Vec<u8>,
    pub flags: Vec<u16>,
}
#[derive(Clone, Debug)]
pub struct CacheBuild {
    pub regions: Vec<u8>,
    pub region_count: u8,
    pub contours: Vec<CacheContour>,
    pub mesh: CacheMesh,
}
#[derive(Clone, Copy, Default)]
struct Sweep {
    samples: u16,
    neighbor: u8,
    id: u8,
}
// Validated byte dimensions bound area by 65,025 and region IDs by 254.
#[derive(Clone, Copy, Default)]
struct Region {
    area: u32,
    neighbors: [u8; 16],
    neighbor_count: u8,
    id: u8,
    area_id: u8,
}
impl Region {
    fn neighbors(&self) -> &[u8] {
        &self.neighbors[..usize::from(self.neighbor_count)]
    }
}
const DX: [isize; 4] = [-1, 0, 1, 0];
const DZ: [isize; 4] = [0, 1, 0, -1];

pub fn build_cache_mesh(
    layer: &HeightfieldLayer,
    climb: i32,
    max_error: f32,
) -> Result<CacheBuild, CacheBuildError> {
    let mut regions = vec![0; checked_layer_size(layer, true)?];
    let mut region_count = 0;
    build_regions_into(layer, climb, &mut regions, &mut region_count)?;
    let mut contours = vec![];
    build_contours_into(
        layer,
        &regions,
        region_count,
        climb,
        max_error,
        &mut contours,
    )?;
    let mesh = build_polygon_mesh(&contours)?;
    Ok(CacheBuild {
        regions,
        region_count,
        contours,
        mesh,
    })
}
/// Uses caller-owned region storage and returns only the runtime mesh. The
/// owned API also exposes regions/contours and keeps its large passes out of
/// the pipeline's code generation; both entry points share all build kernels.
pub(crate) fn build_cache_mesh_in<T: Deref<Target = [u8]>>(
    layer: &HeightfieldLayer<T>,
    climb: i32,
    max_error: f32,
    regions: &mut [u8],
) -> Result<CacheMesh, CacheBuildError> {
    checked_layer_size(layer, true)?;
    let mut region_count = 0;
    build_regions(layer, climb, regions, &mut region_count)?;
    let mut contours = vec![];
    build_contours(
        layer,
        regions,
        region_count,
        climb,
        max_error,
        &mut contours,
    )?;
    build_polygon_mesh(&contours)
}
#[inline]
fn checked_layer_size<T: Deref<Target = [u8]>>(
    layer: &HeightfieldLayer<T>,
    connections: bool,
) -> Result<usize, CacheBuildError> {
    let (w, h) = (layer.width, layer.height);
    if w > 255
        || h > 255
        || layer.heights.len() < w * h
        || layer.areas.len() < w * h
        || (connections && layer.connections.len() < w * h)
    {
        return Err(build_error(
            INVALID_PARAM,
            "invalid tile-cache layer dimensions or grid storage",
        ));
    }
    Ok(w * h)
}
/// Writes the native region grid, retaining extra buffer entries. A region-ID
/// overflow leaves the partially written grid and the previous region count.
// Preserve the owned API's separate large pass while borrowed rebuilds can
// specialize the same kernel inside their pipeline.
#[inline(never)]
pub fn build_regions_into(
    layer: &HeightfieldLayer,
    climb: i32,
    regions: &mut [u8],
    region_count: &mut u8,
) -> Result<(), CacheBuildError> {
    build_regions(layer, climb, regions, region_count)
}

fn build_regions<T: Deref<Target = [u8]>>(
    layer: &HeightfieldLayer<T>,
    climb: i32,
    regions: &mut [u8],
    region_count: &mut u8,
) -> Result<(), CacheBuildError> {
    let n = checked_layer_size(layer, false)?;
    if regions.len() < n {
        return Err(build_error(INVALID_PARAM, "region grid is too short"));
    }
    let (w, h) = (layer.width, layer.height);
    let heights = &layer.heights[..n];
    let areas = &layer.areas[..n];
    let labels = &mut regions[..n];
    labels.fill(255);
    let mut count = 0usize;
    // A byte-wide row has at most 255 sweeps; initialized scratch stays local.
    let mut sweeps = [Sweep::default(); 255];
    let mut previous = [0u8; 256];
    // The first row has no northern regions, regardless of area or height.
    let empty_labels = [255; 255];
    for z in 0..h {
        // Previous-row labels reference only already assigned region IDs.
        previous[..count].fill(0);
        let start = z * w;
        let row_areas = &areas[start..start + w];
        let row_heights = &heights[start..start + w];
        let (before, after) = labels.split_at_mut(start);
        let row = &mut after[..w];
        let previous_start = start.saturating_sub(w);
        let previous_areas = &areas[previous_start..previous_start + w];
        let previous_heights = &heights[previous_start..previous_start + w];
        let previous_labels = if z == 0 {
            &empty_labels[..w]
        } else {
            &before[previous_start..]
        };
        let mut ns = 0;
        let mut west = (0, 0, 255);
        let current = row.iter_mut().zip(row_areas.iter().zip(row_heights));
        let north = previous_labels
            .iter()
            .zip(previous_areas.iter().zip(previous_heights));
        for ((label, (&area, &height)), (&nr, (&north_area, &north_height))) in current.zip(north) {
            if area == 0 {
                west = (0, height, 255);
                continue;
            }
            let mut sid = 255;
            if area == west.0 && height.abs_diff(west.1) as i32 <= climb {
                sid = west.2;
            }
            if sid == 255 {
                sid = ns as u8;
                ns += 1;
                sweeps[sid as usize] = Sweep {
                    neighbor: 255,
                    ..Default::default()
                };
            }
            let s = &mut sweeps[sid as usize];
            if nr != 255 && area == north_area && height.abs_diff(north_height) as i32 <= climb {
                if s.samples == 0 {
                    s.neighbor = nr;
                }
                if s.neighbor == nr {
                    s.samples += 1;
                    previous[nr as usize] = previous[nr as usize].wrapping_add(1);
                } else {
                    s.neighbor = 255;
                }
            }
            *label = sid;
            west = (area, height, sid);
        }
        for s in &mut sweeps[..ns] {
            if s.neighbor != 255 && previous[s.neighbor as usize] as u16 == s.samples {
                s.id = s.neighbor;
            } else {
                if count == 255 {
                    return Err(build_error(
                        BUFFER_TOO_SMALL,
                        "tile-cache region ID overflow",
                    ));
                }
                s.id = count as u8;
                count += 1;
            }
        }
        for label in row {
            if *label != 255 {
                *label = sweeps[*label as usize].id;
            }
        }
    }
    // Successful partitioning assigns only IDs 0..254. Use the fixed extent
    // while accumulating neighbors, then restrict merging to the live prefix.
    let mut regs = [Region::default(); 255];
    for (i, (&ri, &area)) in labels.iter().zip(areas).enumerate() {
        let ri = ri as usize;
        if ri == 255 {
            continue;
        }
        regs[ri].area += 1;
        regs[ri].area_id = area;
        if i >= w {
            let nr = labels[i - w] as usize;
            // Only distinct regions can add an edge. Avoid reading heights
            // and checking connectivity for cells already in the same region.
            if nr != 255
                && nr != ri
                && area == areas[i - w]
                && heights[i].abs_diff(heights[i - w]) as i32 <= climb
            {
                for (a, b) in [(ri, nr), (nr, ri)] {
                    let region = &mut regs[a];
                    let count = usize::from(region.neighbor_count);
                    if region.neighbors().last() != Some(&(b as u8)) {
                        if count >= region.neighbors.len() {
                            return Err(build_error(
                                INVALID_PARAM,
                                "tile-cache neighbor limit exceeded",
                            ));
                        }
                        region.neighbors[count] = b as u8;
                        region.neighbor_count += 1;
                    }
                }
            }
        }
    }
    let regs = &mut regs[..count];
    for (i, r) in regs.iter_mut().enumerate() {
        r.id = i as u8;
    }
    for i in 0..count {
        let mut best = None;
        let mut area = 0;
        for &n in regs[i].neighbors() {
            let n = usize::from(n);
            if regs[i].id == regs[n].id
                || regs[i].area_id != regs[n].area_id
                || regs[n].area <= area
            {
                continue;
            }
            let connections: usize = regs
                .iter()
                .filter(|r| r.id == regs[i].id)
                .map(|r| {
                    r.neighbors()
                        .iter()
                        .filter(|&&k| regs[usize::from(k)].id == regs[n].id)
                        .count()
                })
                .sum();
            if connections == 1 {
                area = regs[n].area;
                best = Some(n);
            }
        }
        if let Some(n) = best {
            let old = regs[i].id;
            let new = regs[n].id;
            for r in regs.iter_mut() {
                if r.id == old {
                    r.id = new;
                }
            }
        }
    }
    let mut remap = [0_u8; 256];
    for r in regs.iter() {
        remap[usize::from(r.id)] = 1;
    }
    let mut nreg = 0_usize;
    for id in &mut remap {
        if *id != 0 {
            *id = nreg as u8;
            nreg += 1;
        }
    }
    for r in regs.iter_mut() {
        r.id = remap[usize::from(r.id)];
    }
    for l in labels.iter_mut() {
        if *l != 255 {
            *l = regs[*l as usize].id;
        }
    }
    *region_count = nreg as u8;
    Ok(())
}
/// Replaces the contour set, retaining initialized partial contours on failure.
/// Regions and connections are independent inputs, as in the native API.
#[inline(never)]
pub fn build_contours_into(
    layer: &HeightfieldLayer,
    labels: &[u8],
    region_count: u8,
    climb: i32,
    max_error: f32,
    contours: &mut Vec<CacheContour>,
) -> Result<(), CacheBuildError> {
    build_contours(layer, labels, region_count, climb, max_error, contours)
}

fn build_contours<T: Deref<Target = [u8]>>(
    layer: &HeightfieldLayer<T>,
    labels: &[u8],
    region_count: u8,
    climb: i32,
    max_error: f32,
    contours: &mut Vec<CacheContour>,
) -> Result<(), CacheBuildError> {
    let n = checked_layer_size(layer, true)?;
    if labels.len() < n || labels[..n].iter().any(|&r| r != 255 && r >= region_count) {
        return Err(build_error(INVALID_PARAM, "invalid region grid or count"));
    }
    *contours = vec![
        CacheContour {
            region: 0,
            area: 0,
            vertices: vec![]
        };
        region_count as usize
    ];
    // Smaller layers need less initialized scratch. Both paths retain the
    // dimension-derived native overflow limit inside the contour walk.
    if (layer.width + layer.height) * 4 <= 512 {
        build_contours_with_scratch::<T, 513>(layer, labels, climb, max_error, contours)
    } else {
        build_contours_with_scratch::<T, { MAX_CONTOUR_VERTICES + 1 }>(
            layer, labels, climb, max_error, contours,
        )
    }
}

fn build_contours_with_scratch<T: Deref<Target = [u8]>, const N: usize>(
    layer: &HeightfieldLayer<T>,
    labels: &[u8],
    climb: i32,
    max_error: f32,
    contours: &mut [CacheContour],
) -> Result<(), CacheBuildError> {
    let n = layer.width * layer.height;
    let w = layer.width;
    // One extra raw slot permits the checked overflow path. The simplifier
    // uses no more than the raw count, or two seeds for a single raw vertex.
    let mut raw = [[0; 4]; N];
    let mut indices = [0; N];
    let mut last_completed = 255;
    let mut cursor = 0;
    while cursor < n {
        let i = cursor;
        let region = labels[i];
        cursor += 1;
        let ri = region as usize;
        if region == last_completed || ri == 255 {
            cursor += labels[cursor..n]
                .iter()
                .position(|&next| next != region)
                .unwrap_or(n - cursor);
            continue;
        }
        if !contours[ri].vertices.is_empty() {
            last_completed = region;
            continue;
        }
        let (x, z) = (i % w, i / w);
        contours[ri].region = ri as u8;
        contours[ri].area = layer.areas[i];
        let raw_count = walk_into(layer, &labels[..n], x, z, &mut raw)?;
        let raw = &raw[..raw_count];
        let (start, count) = simplify_into(raw, max_error, &mut indices);
        let mut result = Vec::with_capacity(count);
        let mut at = start;
        for _ in 0..count {
            let next_at = next_index(at, count);
            let v = raw[usize::from(indices[at])];
            let next = raw[usize::from(indices[next_at])];
            at = next_at;
            let (height, remove) = corner(layer, labels, v, climb);
            let nei = next[3];
            let mut flags = 15;
            if nei != 255 && nei >= 248 {
                flags = nei - 248;
            }
            if remove {
                flags |= 128;
            }
            result.push([v[0], height, v[2], flags]);
        }
        contours[ri] = CacheContour {
            region: ri as u8,
            area: layer.areas[i],
            vertices: result,
        };
        if count != 0 {
            last_completed = region;
        }
    }
    Ok(())
}

#[inline(always)]
fn neighbor_reg(
    connections: &[u8],
    regs: &[u8],
    index: isize,
    dir: usize,
    offset: isize,
) -> Result<u8, CacheBuildError> {
    let i = index as usize;
    if i >= regs.len() {
        return Err(build_error(INVALID_PARAM, "contour leaves the region grid"));
    }
    let mask = 1 << dir;
    if connections[i] & mask == 0 {
        return Ok(if (connections[i] >> 4) & mask != 0 {
            248 + dir as u8
        } else {
            255
        });
    }
    regs.get((index + offset) as usize)
        .copied()
        .ok_or_else(|| build_error(INVALID_PARAM, "connection leaves the region grid"))
}

const MAX_CONTOUR_VERTICES: usize = (255 + 255) * 4;

fn append(vertices: &mut [[u8; 4]], count: &mut usize, x: isize, y: u8, z: isize, region: u8) {
    let n = *count;
    if n > 1 && vertices[n - 1][3] == region {
        let a = vertices[n - 2];
        let b = vertices[n - 1];
        // Native compares the full coordinates before narrowing for storage.
        if a[0] == b[0] && b[0] as isize == x {
            vertices[n - 1][1] = y;
            vertices[n - 1][2] = z as u8;
            return;
        }
        if a[2] == b[2] && b[2] as isize == z {
            vertices[n - 1][0] = x as u8;
            vertices[n - 1][1] = y;
            return;
        }
    }
    vertices[n] = [x as u8, y, z as u8, region];
    *count += 1;
}
fn walk_into<T: Deref<Target = [u8]>, const N: usize>(
    layer: &HeightfieldLayer<T>,
    regs: &[u8],
    sx: usize,
    sz: usize,
    vertices: &mut [[u8; 4]; N],
) -> Result<usize, CacheBuildError> {
    let mut count = 0;
    let width = layer.width as isize;
    let offsets = [-1, width, 1, -width];
    let connections = &layer.connections[..regs.len()];
    let heights = &layer.heights[..regs.len()];
    // Keep the native coordinates as well as their flat index: connections can
    // cross a row boundary while still addressing valid backing storage.
    let mut index = (sx + sz * layer.width) as isize;
    let mut start_dir = None;
    for i in 0..4 {
        let d = (i + 3) & 3;
        if neighbor_reg(connections, regs, index, d, offsets[d])? != regs[index as usize] {
            start_dir = Some(d);
            break;
        }
    }
    let Some(start_dir) = start_dir else {
        return Ok(0);
    };
    let (mut x, mut z, mut dir) = (sx as isize, sz as isize, start_dir);
    for iteration in 0..layer.width * layer.height {
        let nr = neighbor_reg(connections, regs, index, dir, offsets[dir])?;
        let (mut nx, mut nz, mut next_index) = (x, z, index);
        let next_dir;
        if nr != regs[index as usize] {
            // Directions 0..3 select corner offsets (0,1), (1,1), (1,0), (0,0).
            let px = x + (((dir + 1) & 2) >> 1) as isize;
            let pz = z + ((dir >> 1) ^ 1) as isize;
            append(vertices, &mut count, px, heights[index as usize], pz, nr);
            if count > (layer.width + layer.height) * 4 {
                return Err(build_error(
                    BUFFER_TOO_SMALL,
                    "tile-cache contour buffer exhausted",
                ));
            }
            next_dir = (dir + 1) & 3;
        } else {
            nx = x + DX[dir];
            nz = z + DZ[dir];
            next_index = index + offsets[dir];
            next_dir = (dir + 3) & 3;
        }
        if iteration > 0 && x == sx as isize && z == sz as isize && dir == start_dir {
            break;
        }
        (x, z, dir, index) = (nx, nz, next_dir, next_index);
    }
    if count > 0
        && vertices[0][0] == vertices[count - 1][0]
        && vertices[0][2] == vertices[count - 1][2]
    {
        count -= 1;
    }
    Ok(count)
}

fn distance(p: [u8; 4], a: [u8; 4], b: [u8; 4]) -> f32 {
    let pqx = b[0] as f32 - a[0] as f32;
    let pqz = b[2] as f32 - a[2] as f32;
    let dx = p[0] as f32 - a[0] as f32;
    let dz = p[2] as f32 - a[2] as f32;
    let d = crate::nav::madd(pqx, pqx, pqz * pqz);
    let mut t = crate::nav::madd(pqx, dx, pqz * dz);
    if d > 0. {
        t /= d;
    }
    t = t.clamp(0., 1.);
    let dx = crate::nav::madd(t, pqx, a[0] as f32) - p[0] as f32;
    let dz = crate::nav::madd(t, pqz, a[2] as f32) - p[2] as f32;
    crate::nav::madd(dx, dx, dz * dz)
}
// Callers advance a valid position within a nonempty ring.
#[inline]
fn next_index(index: usize, len: usize) -> usize {
    if index + 1 == len { 0 } else { index + 1 }
}

#[inline]
fn previous_index(index: usize, len: usize) -> usize {
    if index == 0 { len - 1 } else { index - 1 }
}

// walk_into bounds raw vertices by (width + height) * 4 <= 2040, so native
// u16 scratch indices suffice. Only the final contour owns a vertex copy.
fn simplify_into<const N: usize>(
    raw: &[[u8; 4]],
    error: f32,
    poly: &mut [u16; N],
) -> (usize, usize) {
    let mut count = 0;
    if raw.is_empty() {
        return (0, 0);
    }
    let n = raw.len();
    for i in 0..n {
        if raw[next_index(i, n)][3] != raw[i][3] {
            poly[count] = i as u16;
            count += 1;
        }
    }
    if count < 2 {
        let (mut lo, mut hi) = (0, 0);
        for (i, v) in raw.iter().enumerate().skip(1) {
            let key = (v[0], v[2]);
            if key < (raw[lo][0], raw[lo][2]) {
                lo = i;
            }
            if key > (raw[hi][0], raw[hi][2]) {
                hi = i;
            }
        }
        poly[0] = lo as u16;
        poly[1] = hi as u16;
        count = 2;
    }
    let mut i = 0;
    while i < count {
        let ai = usize::from(poly[i]);
        let bi = usize::from(poly[next_index(i, count)]);
        let a = raw[ai];
        let b = raw[bi];
        let forward = b[0] > a[0] || (b[0] == a[0] && b[2] > a[2]);
        let (mut ci, end) = if forward {
            (next_index(ai, n), bi)
        } else {
            (previous_index(bi, n), ai)
        };
        let mut best = None;
        let mut maxd = 0.;
        while ci != end {
            let d = distance(raw[ci], a, b);
            if d > maxd {
                maxd = d;
                best = Some(ci);
            }
            ci = if forward {
                next_index(ci, n)
            } else {
                previous_index(ci, n)
            };
        }
        if let Some(best) = best.filter(|_| maxd > error * error) {
            poly.copy_within(i + 1..count, i + 2);
            poly[i + 1] = best as u16;
            count += 1;
        } else {
            i += 1;
        }
    }
    ((0..count).min_by_key(|&i| poly[i]).unwrap(), count)
}

fn corner<T: Deref<Target = [u8]>>(
    layer: &HeightfieldLayer<T>,
    regs: &[u8],
    v: [u8; 4],
    climb: i32,
) -> (u8, bool) {
    let mut count = 0;
    let mut portal = 15u8;
    let mut height = 0;
    let mut previous = 255;
    let mut same = true;
    for dz in -1..=0 {
        for dx in -1..=0 {
            let x = v[0] as isize + dx;
            let z = v[2] as isize + dz;
            if x < 0 || z < 0 || x >= layer.width as isize || z >= layer.height as isize {
                continue;
            }
            let i = x as usize + z as usize * layer.width;
            if layer.heights[i].abs_diff(v[1]) as i32 <= climb && layer.areas[i] != 0 {
                height = height.max(layer.heights[i]);
                portal &= layer.connections[i] >> 4;
                if previous != 255 && previous != regs[i] {
                    same = false;
                }
                previous = regs[i];
                count += 1;
            }
        }
    }
    (height, count > 1 && portal.count_ones() == 1 && same)
}
pub fn build_polygon_mesh(contours: &[CacheContour]) -> Result<CacheMesh, CacheBuildError> {
    let mut output = CacheMesh::default();
    build_polygon_mesh_into(contours, &mut output)?;
    Ok(output)
}
/// Retains initialized partial mesh data after a native removal-buffer failure.
pub fn build_polygon_mesh_into(
    contours: &[CacheContour],
    output: &mut CacheMesh,
) -> Result<(), CacheBuildError> {
    let mut mesh = None;
    let result = ContourSet::build_tile_cache_rows_from_contours(
        contours.iter().map(|c| TileCacheContourView {
            vertices: &c.vertices,
            region: c.region,
            area: c.area,
        }),
        &mut mesh,
    )
    .map_err(|e| {
        let detail = if matches!(e, crate::PolygonNavmeshError::VertexChainCycle) {
            crate::nav::INVALID_PARAM
        } else {
            BUFFER_TOO_SMALL
        };
        build_error(detail, e.to_string())
    });
    // Checked nonterminating vertex chains leave output untouched. Native
    // removal failures have their six-vertex partial mesh layout initialized.
    let Some(mesh) = mesh else {
        return result;
    };
    let vertices: Vec<_> = mesh.vertices.into_iter().map(|v| v.to_array()).collect();
    let mut polygons = mesh.polygons;
    if result.is_ok() {
        build_adjacency(&vertices, &mut polygons, contours);
    }
    *output = CacheMesh {
        vertices,
        polygons,
        areas: mesh.areas.into_iter().map(|a| a.0).collect(),
        flags: mesh.flags,
    };
    result
}

// The reference's two-pass edge buckets preserve pairing order without a quadratic polygon scan.
fn build_adjacency(vertices: &[[u16; 3]], polygons: &mut [[u16; 12]], contours: &[CacheContour]) {
    struct Edge {
        vertices: [u16; 2],
        polygons: [u16; 2],
        // Polygon slots 0..5, portal directions 0..14, or the native 0xff.
        edges: [u8; 2],
        next: u16,
    }
    // Native keeps the allocation/count as int but truncates bucket links and
    // polygon indices to ushort, including the 0xffff sentinel at wraparound.
    let mut first = vec![u16::MAX; vertices.len()];
    // Each directed polygon edge can append at most one record across the two
    // passes. Reserve that bound once, avoiding growth without padding short
    // polygons to the native six-slot maximum.
    let capacity = polygons
        .iter()
        .map(|p| {
            p[..VERTICES_PER_POLYGON]
                .iter()
                .take_while(|&&v| v != NULL_INDEX)
                .count()
        })
        .sum();
    let mut edges: Vec<Edge> = Vec::with_capacity(capacity);
    for (i, p) in polygons.iter().enumerate() {
        let n = p[..6].iter().take_while(|&&v| v != 65535).count();
        for j in 0..n {
            let a = p[j];
            let b = p[next_index(j, n)];
            if a < b {
                let next = first[a as usize];
                first[a as usize] = edges.len() as u16;
                edges.push(Edge {
                    vertices: [a, b],
                    polygons: [i as u16; 2],
                    edges: [j as u8, 255],
                    next,
                });
            }
        }
    }
    for (i, p) in polygons.iter().enumerate() {
        let n = p[..6].iter().take_while(|&&v| v != 65535).count();
        for j in 0..n {
            let a = p[j];
            let b = p[next_index(j, n)];
            if a <= b {
                continue;
            }
            let mut found = false;
            let mut at = first[b as usize];
            while at != u16::MAX {
                let edge = &mut edges[usize::from(at)];
                at = edge.next;
                if edge.vertices[1] == a && edge.polygons[0] == edge.polygons[1] {
                    edge.polygons[1] = i as u16;
                    edge.edges[1] = j as u8;
                    found = true;
                    break;
                }
            }
            if !found {
                let next = first[b as usize];
                first[b as usize] = edges.len() as u16;
                edges.push(Edge {
                    vertices: [b, a],
                    polygons: [i as u16; 2],
                    edges: [j as u8, 255],
                    next,
                });
            }
        }
    }
    for c in contours {
        if c.vertices.len() < 3 {
            continue;
        }
        for j in 0..c.vertices.len() {
            let k = previous_index(j, c.vertices.len());
            let a = c.vertices[k];
            let b = c.vertices[j];
            let dir = a[3] & 15;
            if dir == 15 {
                continue;
            }
            let (fixed, moving) = if dir == 0 || dir == 2 { (0, 2) } else { (2, 0) };
            for edge in &mut edges {
                if edge.polygons[0] != edge.polygons[1] {
                    continue;
                }
                let va = vertices[edge.vertices[0] as usize];
                let vb = vertices[edge.vertices[1] as usize];
                if va[fixed] == a[fixed] as u16
                    && vb[fixed] == a[fixed] as u16
                    && va[moving].min(vb[moving]) < (a[moving].max(b[moving]) as u16)
                    && va[moving].max(vb[moving]) > (a[moving].min(b[moving]) as u16)
                {
                    edge.edges[1] = dir;
                }
            }
        }
    }
    for e in edges {
        if e.polygons[0] != e.polygons[1] {
            polygons[usize::from(e.polygons[0])][6 + usize::from(e.edges[0])] = e.polygons[1];
            polygons[usize::from(e.polygons[1])][6 + usize::from(e.edges[1])] = e.polygons[0];
        } else if e.edges[1] != 255 {
            polygons[usize::from(e.polygons[0])][6 + usize::from(e.edges[0])] =
                0x8000 | u16::from(e.edges[1]);
        }
    }
}

/// Mark axis-aligned box obstacles with area zero.
pub fn mark_box<T: DerefMut<Target = [u8]>>(
    layer: &mut HeightfieldLayer<T>,
    min: [f32; 3],
    max: [f32; 3],
) {
    mark_box_area(layer, min, max, 0);
}

fn valid_mark_layer<T: Deref<Target = [u8]>>(layer: &HeightfieldLayer<T>) -> bool {
    layer.width <= 255
        && layer.height <= 255
        && layer.heights.len() >= layer.width * layer.height
        && layer.areas.len() >= layer.width * layer.height
}
fn voxel_floor(value: f32) -> Option<i32> {
    let value = value.floor();
    (value.is_finite() && (-2147483648.0..2147483648.0).contains(&value)).then_some(value as i32)
}
fn voxel_bounds<T: Deref<Target = [u8]>>(
    layer: &HeightfieldLayer<T>,
    bounds: [[f32; 3]; 2],
) -> Option<[[i32; 3]; 2]> {
    let scales = [
        1. / layer.cell_size,
        1. / layer.cell_height,
        1. / layer.cell_size,
    ];
    let mut out = [[0; 3]; 2];
    for side in 0..2 {
        for axis in 0..3 {
            out[side][axis] =
                voxel_floor((bounds[side][axis] - layer.bounds[0][axis]) * scales[axis])?;
        }
    }
    Some(out)
}
fn mark_area<T: DerefMut<Target = [u8]>>(
    layer: &mut HeightfieldLayer<T>,
    bounds: [[i32; 3]; 2],
    area: u8,
    contains: impl Fn(i32, i32) -> bool,
) -> u32 {
    let [lo, hi] = bounds;
    for z in lo[2].max(0)..=hi[2].min(layer.height as i32 - 1) {
        for x in lo[0].max(0)..=hi[0].min(layer.width as i32 - 1) {
            if !contains(x, z) {
                continue;
            }
            let i = x as usize + z as usize * layer.width;
            let y = i32::from(layer.heights[i]);
            if y >= lo[1] && y <= hi[1] {
                layer.areas[i] = area;
            }
        }
    }
    crate::nav::SUCCESS
}

/// Native inclusive voxel bounds, with an arbitrary byte area ID. Invalid
/// storage or an undefined native float-to-int conversion returns a failure.
pub fn mark_box_area<T: DerefMut<Target = [u8]>>(
    layer: &mut HeightfieldLayer<T>,
    min: [f32; 3],
    max: [f32; 3],
    area: u8,
) -> u32 {
    if !valid_mark_layer(layer) {
        return crate::nav::FAILURE | crate::nav::INVALID_PARAM;
    }
    let Some(bounds) = voxel_bounds(layer, [min, max]) else {
        return crate::nav::FAILURE | crate::nav::INVALID_PARAM;
    };
    mark_area(layer, bounds, area, |_, _| true)
}
/// Native cylinder marking expands the voxel radius by half a cell.
pub fn mark_cylinder_area<T: DerefMut<Target = [u8]>>(
    layer: &mut HeightfieldLayer<T>,
    position: [f32; 3],
    radius: f32,
    height: f32,
    area: u8,
) -> u32 {
    if !valid_mark_layer(layer) {
        return crate::nav::FAILURE | crate::nav::INVALID_PARAM;
    }
    let min = [position[0] - radius, position[1], position[2] - radius];
    let max = [
        position[0] + radius,
        position[1] + height,
        position[2] + radius,
    ];
    let Some(bounds) = voxel_bounds(layer, [min, max]) else {
        return crate::nav::FAILURE | crate::nav::INVALID_PARAM;
    };
    let r = radius / layer.cell_size + 0.5;
    let r2 = r * r;
    let ics = 1. / layer.cell_size;
    let px = (position[0] - layer.bounds[0][0]) * ics;
    let pz = (position[2] - layer.bounds[0][2]) * ics;
    mark_area(layer, bounds, area, |x, z| {
        let dx = x as f32 + 0.5 - px;
        let dz = z as f32 + 0.5 - pz;
        crate::nav::madd(dx, dx, dz * dz).partial_cmp(&r2) != Some(std::cmp::Ordering::Greater)
    })
}
/// Native oriented-box auxiliary rotation values are supplied directly; the
/// identity is `[0, 0.5]`. Bounds retain the native `1.41` extent approximation.
pub fn mark_oriented_box_area<T: DerefMut<Target = [u8]>>(
    layer: &mut HeightfieldLayer<T>,
    center: [f32; 3],
    half_extents: [f32; 3],
    rotation: [f32; 2],
    area: u8,
) -> u32 {
    use crate::nav::{FAILURE, INVALID_PARAM, madd};
    if !valid_mark_layer(layer) {
        return FAILURE | INVALID_PARAM;
    }
    let ics = 1. / layer.cell_size;
    let ich = 1. / layer.cell_height;
    let cx = (center[0] - layer.bounds[0][0]) * ics;
    let cz = (center[2] - layer.bounds[0][2]) * ics;
    let maxr = 1.41
        * if half_extents[0] > half_extents[2] {
            half_extents[0]
        } else {
            half_extents[2]
        };
    let limits = [
        madd(-maxr, ics, cx),
        (center[1] - half_extents[1] - layer.bounds[0][1]) * ich,
        madd(-maxr, ics, cz),
        madd(maxr, ics, cx),
        (center[1] + half_extents[1] - layer.bounds[0][1]) * ich,
        madd(maxr, ics, cz),
    ];
    let mut bounds = [[0; 3]; 2];
    for i in 0..6 {
        let Some(value) = voxel_floor(limits[i]) else {
            return FAILURE | INVALID_PARAM;
        };
        bounds[i / 3][i % 3] = value;
    }
    let xhalf = madd(half_extents[0], ics, 0.5);
    let zhalf = madd(half_extents[2], ics, 0.5);
    mark_area(layer, bounds, area, |x, z| {
        let x2 = 2. * (x as f32 - cx);
        let z2 = 2. * (z as f32 - cz);
        let xrot = madd(rotation[1], x2, rotation[0] * z2);
        if xrot > xhalf || xrot < -xhalf {
            return false;
        }
        let zrot = madd(rotation[1], z2, -rotation[0] * x2);
        !(zrot > zhalf || zrot < -zhalf)
    })
}

#[cfg(all(test, feature = "cpp-oracle"))]
mod tests {
    use super::*;
    use std::{
        io::Write as _,
        process::{Command, Stdio},
    };
    #[test]
    fn native_cache_simplification_distance_rounding_regression() {
        let input = "distance 1 59 222 2 11 223 240";
        let mut child = Command::new(env!("TILE_CACHE_ORACLE"))
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()
            .unwrap();
        child
            .stdin
            .take()
            .unwrap()
            .write_all(input.as_bytes())
            .unwrap();
        let out = child.wait_with_output().unwrap();
        assert!(out.status.success());
        let expected: Vec<u32> = serde_json::from_slice(&out.stdout).unwrap();
        assert_eq!(
            vec![distance([59, 0, 222, 0], [2, 0, 11, 0], [223, 0, 240, 0]).to_bits()],
            expected
        );
    }
}
