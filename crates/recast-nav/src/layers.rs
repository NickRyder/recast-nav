// Altered Rust translation of RecastNavigation v1.6.0 RecastLayers.cpp.
// Copyright (c) 2009-2010 Mikko Mononen. Zlib license: vendor/recastnavigation/License.txt.
use crate::BuildError;
use crate::CompactHeightfield;
use crate::context::{Context, LogCategory, TimerLabel};

/// Layer metadata with owned grids by default. Borrowed slice grids use the
/// same marking, region and contour kernels without converting their storage.
#[derive(Clone, Debug, PartialEq)]
pub struct HeightfieldLayer<T = Vec<u8>> {
    pub bounds: [[f32; 3]; 2],
    pub cell_size: f32,
    pub cell_height: f32,
    pub width: usize,
    pub height: usize,
    pub min_x: usize,
    pub max_x: usize,
    pub min_z: usize,
    pub max_z: usize,
    pub hmin: u16,
    pub hmax: u16,
    pub heights: T,
    pub areas: T,
    pub connections: T,
}

#[derive(Clone)]
struct Region {
    overlaps: Vec<u8>,
    neighbors: [u8; 16],
    neighbor_count: u8,
    ymin: u16,
    ymax: u16,
    layer: u8,
    base: bool,
}
impl Default for Region {
    fn default() -> Self {
        Self {
            overlaps: vec![],
            neighbors: [0; 16],
            neighbor_count: 0,
            ymin: u16::MAX,
            ymax: 0,
            layer: 255,
            base: false,
        }
    }
}
impl Region {
    fn add_neighbor(&mut self, value: u8) {
        let count = usize::from(self.neighbor_count);
        // Native silently ignores new neighbors after its fixed list is full.
        if count < self.neighbors.len() && !self.neighbors[..count].contains(&value) {
            self.neighbors[count] = value;
            self.neighbor_count += 1;
        }
    }
}
#[derive(Clone, Default)]
struct Sweep {
    samples: u16,
    neighbor: u8,
    id: u8,
}

fn unique(items: &mut Vec<u8>, value: usize, limit: usize) -> Result<(), BuildError> {
    // Every region reference comes from a byte ID; 255 is excluded before
    // insertion. Keep the native representation instead of machine-word IDs.
    let value = value as u8;
    if !items.contains(&value) {
        if items.len() == limit {
            return Err(BuildError(
                "heightfield layer overlap limit exceeded".into(),
            ));
        }
        items.push(value);
    }
    Ok(())
}

fn add_overlap(context: &Context<'_>, items: &mut Vec<u8>, value: usize) -> Result<(), BuildError> {
    unique(items, value, 63).inspect_err(|_| {
        context.log(LogCategory::ERROR, "rcBuildHeightfieldLayers: layer overflow (too many overlapping walkable platforms). Try increasing RC_MAX_LAYERS.");
    })
}

static DIRECTION_X: [isize; 4] = [-1, 0, 1, 0];
static DIRECTION_Z: [isize; 4] = [0, 1, 0, -1];

fn neighbor(hf: &CompactHeightfield, x: usize, z: usize, i: usize, dir: u8) -> Option<usize> {
    let con = hf.spans[i].con(dir)? as usize;
    let dx = DIRECTION_X[dir as usize];
    let dz = DIRECTION_Z[dir as usize];
    let index = (x as isize + dx) + (z as isize + dz) * hf.width as isize;
    Some(hf.cells[index as usize].index() as usize + con)
}

/// Build layered heightfields with the reference's monotone partition and bounded queues.
pub fn build_heightfield_layers(
    hf: &CompactHeightfield,
    border: i32,
    walkable_height: i32,
) -> Result<Vec<HeightfieldLayer>, BuildError> {
    build_heightfield_layers_with_context(&Context::new(false), hf, border, walkable_height)
}

pub fn build_heightfield_layers_with_context(
    context: &Context<'_>,
    hf: &CompactHeightfield,
    border: i32,
    walkable_height: i32,
) -> Result<Vec<HeightfieldLayer>, BuildError> {
    let _timer = context.scoped_timer(TimerLabel::BUILD_LAYERS);
    if hf.width < 0 || hf.height < 0 || border < 0 {
        return Err(BuildError("invalid native layer grid storage".into()));
    }
    let w = hf.width as usize;
    let h = hf.height as usize;
    let b = border as usize;
    if 2 * b >= w || 2 * b >= h {
        return Ok(Vec::new());
    }
    let mut src = vec![255_u8; hf.spans.len()];
    let mut sweeps = vec![Sweep::default(); w];
    let mut regions_count = 0;
    for z in b..h - b {
        let mut previous = [0_i32; 256];
        let mut sweep_count = 0;
        for x in b..w - b {
            for i in hf.cells[x + z * w].index_range() {
                if hf.areas[i].0 == 0 {
                    continue;
                }
                let mut sid = 255;
                if let Some(ni) = neighbor(hf, x, z, i, 0)
                    && hf.areas[ni].0 != 0
                    && src[ni] != 255
                {
                    sid = usize::from(src[ni]);
                }
                if sid == 255 {
                    if sweep_count >= sweeps.len() || sweep_count == 255 {
                        return Err(BuildError("layer sweep limit exceeded".into()));
                    }
                    sid = sweep_count;
                    sweep_count += 1;
                    sweeps[sid].neighbor = 255;
                    sweeps[sid].samples = 0;
                }
                if let Some(ni) = neighbor(hf, x, z, i, 3) {
                    let nr = src[ni];
                    if nr != 255 {
                        if sweeps[sid].samples == 0 {
                            sweeps[sid].neighbor = nr;
                        }
                        if sweeps[sid].neighbor == nr {
                            sweeps[sid].samples = sweeps[sid].samples.wrapping_add(1);
                            previous[usize::from(nr)] += 1;
                        } else {
                            sweeps[sid].neighbor = 255;
                        }
                    }
                }
                src[i] = sid as u8;
            }
        }
        for sweep in &mut sweeps[..sweep_count] {
            if sweep.neighbor != 255
                && previous[usize::from(sweep.neighbor)] == i32::from(sweep.samples)
            {
                sweep.id = sweep.neighbor;
            } else {
                if regions_count == 255 {
                    context.log(
                        LogCategory::ERROR,
                        "rcBuildHeightfieldLayers: Region ID overflow.",
                    );
                    return Err(BuildError("heightfield layer region ID overflow".into()));
                }
                sweep.id = regions_count as u8;
                regions_count += 1;
            }
        }
        for x in b..w - b {
            for i in hf.cells[x + z * w].index_range() {
                if src[i] != 255 {
                    src[i] = sweeps[usize::from(src[i])].id;
                }
            }
        }
    }
    let mut regions = vec![Region::default(); regions_count];
    let mut column = [0; 63];
    for z in 0..h {
        for x in 0..w {
            let mut column_count = 0;
            for i in hf.cells[x + z * w].index_range() {
                let ri = usize::from(src[i]);
                if ri == 255 {
                    continue;
                }
                regions[ri].ymin = regions[ri].ymin.min(hf.spans[i].y);
                regions[ri].ymax = regions[ri].ymax.max(hf.spans[i].y);
                if column_count < column.len() {
                    column[column_count] = ri;
                    column_count += 1;
                }
                for dir in 0..4 {
                    if let Some(ni) = neighbor(hf, x, z, i, dir) {
                        let nr = usize::from(src[ni]);
                        if nr != 255 && nr != ri {
                            regions[ri].add_neighbor(nr as u8);
                        }
                    }
                }
            }
            for i in 0..column_count {
                for j in i + 1..column_count {
                    if column[i] != column[j] {
                        add_overlap(context, &mut regions[column[i]].overlaps, column[j])?;
                        add_overlap(context, &mut regions[column[j]].overlaps, column[i])?;
                    }
                }
            }
        }
    }
    // The region overflow guard limits both region and layer counts to 255.
    let mut layer_count = 0_u8;
    for root in 0..regions_count {
        if regions[root].layer != 255 {
            continue;
        }
        regions[root].layer = layer_count;
        regions[root].base = true;
        // Native retains at most 64 queued regions. A fixed ring preserves its
        // FIFO traversal and full-queue rejection without allocating per layer.
        let mut queue = [0usize; 64];
        queue[0] = root;
        let mut head = 0;
        let mut queued = 1;
        while queued != 0 {
            let current = queue[head];
            head = (head + 1) % queue.len();
            queued -= 1;
            for neighbor_index in 0..usize::from(regions[current].neighbor_count) {
                let nei = usize::from(regions[current].neighbors[neighbor_index]);
                if regions[nei].layer != 255 || regions[root].overlaps.contains(&(nei as u8)) {
                    continue;
                }
                let lo = regions[root].ymin.min(regions[nei].ymin);
                let hi = regions[root].ymax.max(regions[nei].ymax);
                if hi - lo >= 255 || queued == queue.len() {
                    continue;
                }
                queue[(head + queued) % queue.len()] = nei;
                queued += 1;
                regions[nei].layer = layer_count;
                for overlap_index in 0..regions[nei].overlaps.len() {
                    let overlap = usize::from(regions[nei].overlaps[overlap_index]);
                    add_overlap(context, &mut regions[root].overlaps, overlap)?;
                }
                regions[root].ymin = lo;
                regions[root].ymax = hi;
            }
        }
        layer_count += 1;
    }
    let merge_height = i32::from((walkable_height as u16).wrapping_mul(4));
    for root in 0..regions_count {
        if !regions[root].base {
            continue;
        }
        let new = regions[root].layer;
        loop {
            let mut old = None;
            for j in 0..regions_count {
                if root == j || !regions[j].base {
                    continue;
                }
                if i32::from(regions[root].ymin) > i32::from(regions[j].ymax) + merge_height
                    || i32::from(regions[root].ymax) + merge_height < i32::from(regions[j].ymin)
                {
                    continue;
                }
                if regions[root].ymax.max(regions[j].ymax) - regions[root].ymin.min(regions[j].ymin)
                    >= 255
                {
                    continue;
                }
                if (0..regions_count).any(|k| {
                    regions[k].layer == regions[j].layer
                        && regions[root].overlaps.contains(&(k as u8))
                }) {
                    continue;
                }
                old = Some(regions[j].layer);
                break;
            }
            let Some(old) = old else {
                break;
            };
            for j in 0..regions_count {
                if regions[j].layer != old {
                    continue;
                }
                regions[j].base = false;
                regions[j].layer = new;
                for overlap_index in 0..regions[j].overlaps.len() {
                    let overlap = usize::from(regions[j].overlaps[overlap_index]);
                    add_overlap(context, &mut regions[root].overlaps, overlap)?;
                }
                regions[root].ymin = regions[root].ymin.min(regions[j].ymin);
                regions[root].ymax = regions[root].ymax.max(regions[j].ymax);
            }
        }
    }
    let mut remap = [255_u8; 256];
    for r in &regions {
        remap[usize::from(r.layer)] = 1;
    }
    layer_count = 0;
    for id in &mut remap {
        if *id != 255 {
            *id = layer_count;
            layer_count += 1;
        }
    }
    for r in &mut regions {
        r.layer = remap[usize::from(r.layer)];
    }
    // Reuse the layer-ID scratch table for region-to-layer lookups. Region
    // IDs and final layer IDs are bytes; 255 remains the unmapped sentinel.
    remap.fill(255);
    for (id, region) in regions.iter().enumerate() {
        remap[id] = region.layer;
    }
    let lw = w - 2 * b;
    let lh = h - 2 * b;
    let mut output = Vec::with_capacity(usize::from(layer_count));
    for id in 0..layer_count {
        let base = regions
            .iter()
            .rev()
            .find(|r| r.base && r.layer == id)
            .expect("layer has a base region");
        let mut bounds = [hf.aabb.min.to_array(), hf.aabb.max.to_array()];
        for axis in [0, 2] {
            bounds[0][axis] += border as f32 * hf.cell_size;
            bounds[1][axis] -= border as f32 * hf.cell_size;
        }
        bounds[1][1] = crate::nav::madd(base.ymax as f32, hf.cell_height, bounds[0][1]);
        bounds[0][1] = crate::nav::madd(base.ymin as f32, hf.cell_height, bounds[0][1]);
        let mut layer = HeightfieldLayer {
            bounds,
            cell_size: hf.cell_size,
            cell_height: hf.cell_height,
            width: lw,
            height: lh,
            min_x: lw,
            max_x: 0,
            min_z: lh,
            max_z: 0,
            hmin: base.ymin,
            hmax: base.ymax,
            heights: vec![255; lw * lh],
            areas: vec![0; lw * lh],
            connections: vec![0; lw * lh],
        };
        for z in 0..lh {
            for x in 0..lw {
                let (cx, cz) = (x + b, z + b);
                for i in hf.cells[cx + cz * w].index_range() {
                    if remap[usize::from(src[i])] != id {
                        continue;
                    }
                    layer.min_x = layer.min_x.min(x);
                    layer.max_x = layer.max_x.max(x);
                    layer.min_z = layer.min_z.min(z);
                    layer.max_z = layer.max_z.max(z);
                    let index = x + z * lw;
                    layer.heights[index] = (hf.spans[i].y - base.ymin) as u8;
                    layer.areas[index] = hf.areas[i].0;
                    let mut portal = 0;
                    let mut connections = 0;
                    // Keep native direction order, but specialize each visit so the
                    // hot output loop uses constant shifts and neighbor offsets.
                    macro_rules! visit_connection {
                        ($dir:expr) => {{
                            if let Some(ni) = neighbor(hf, cx, cz, i, $dir) {
                                let lid = remap[usize::from(src[ni])];
                                if hf.areas[ni].0 != 0 {
                                    if id != lid {
                                        portal |= 1 << $dir;
                                        if hf.spans[ni].y > base.ymin {
                                            layer.heights[index] = layer.heights[index]
                                                .max((hf.spans[ni].y - base.ymin) as u8);
                                        }
                                    } else {
                                        let nx = x as isize + DIRECTION_X[$dir as usize];
                                        let nz = z as isize + DIRECTION_Z[$dir as usize];
                                        if nx >= 0
                                            && nz >= 0
                                            && nx < lw as isize
                                            && nz < lh as isize
                                        {
                                            connections |= 1 << $dir;
                                        }
                                    }
                                }
                            }
                        }};
                    }
                    visit_connection!(0);
                    visit_connection!(1);
                    visit_connection!(2);
                    visit_connection!(3);
                    layer.connections[index] = (portal << 4) | connections;
                }
            }
        }
        if layer.min_x > layer.max_x {
            layer.min_x = 0;
            layer.max_x = 0;
        }
        if layer.min_z > layer.max_z {
            layer.min_z = 0;
            layer.max_z = 0;
        }
        output.push(layer);
    }
    Ok(output)
}
