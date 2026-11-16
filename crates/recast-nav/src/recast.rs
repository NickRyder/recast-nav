//! Low-level Recast generation data and operational utilities.
//!
//! Area marking uses world units and preserves null areas. Voxel conversions
//! require finite, representable inputs; compact cells, spans and neighbor
//! connections must reference valid storage. Polygon offsets preserve native
//! floating-point operations, including nonfinite values.
//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

// Altered Rust translations of RecastArea.cpp, RecastMesh.cpp,
// RecastMeshDetail.cpp and Recast.cpp, pinned RecastNavigation 1.6.0.
use crate::common::{add_scaled, subtract_scaled};
pub use crate::context::{
    self, Context, ContextHooks, LogCategory, ScopedTimer, TIMER_COUNT, TimerLabel,
};
use crate::nav::madd;
// Recast and Detour use identical expressions and call boundaries for these
// helpers. Keep one translation, verified against both libraries directly.
pub use crate::common::{
    Scalar, absolute, clamp, maximum, minimum, square, vector_add, vector_cross, vector_distance,
    vector_distance_squared, vector_dot, vector_maximum, vector_minimum, vector_multiply_add,
    vector_normalize, vector_subtract,
};
pub use crate::{
    AreaType, BuildContoursFlags, BuildRegionsError, CompactCell, CompactHeightfield,
    CompactHeightfieldError, CompactSpan, Contour, ContourError, ContourSet, ConvexVolume,
    DetailNavmesh, DetailNavmeshError, ErosionError, FilterError, Heightfield, HeightfieldBuilder,
    PolygonNavmesh, PolygonNavmeshError, RasterizationError, RegionId, RegionVertexId, Span,
    SpanInsertion, SpanInsertionError, SpanKey, Spans, SubMesh, TriMesh,
};

pub const PI: f32 = std::f32::consts::PI;
pub const SPAN_HEIGHT_BITS: u32 = 13;
pub const SPAN_MAX_HEIGHT: u16 = (1 << SPAN_HEIGHT_BITS) - 1;
pub const MESH_NULL_INDEX: u16 = u16::MAX;
pub const NOT_CONNECTED: u8 = 0x3f;

/// Native `rcConfig` parameter aggregation, using degrees and signed integer
/// fields. Zero initialization matches `rcConfig{}`; use `BuildConfig::default`
/// for the high-level pipeline's configured defaults.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Config {
    pub width: i32,
    pub height: i32,
    pub tile_size: i32,
    pub border_size: i32,
    pub cell_size: f32,
    pub cell_height: f32,
    pub bounds: [[f32; 3]; 2],
    pub slope_degrees: f32,
    pub walkable_height: i32,
    pub walkable_climb: i32,
    pub walkable_radius: i32,
    pub max_edge_len: i32,
    pub max_simplification_error: f32,
    pub min_region_area: i32,
    pub merge_region_area: i32,
    pub max_vertices_per_polygon: i32,
    pub detail_sample_dist: f32,
    pub detail_sample_max_error: f32,
}

/// Native `rcSqrt` (`sqrtf`).
pub fn square_root(value: f32) -> f32 {
    value.sqrt()
}

/// Native `rcGetDirOffsetX`; all signed directions use their low two bits.
pub fn direction_offset_x(direction: i32) -> i32 {
    [-1, 0, 1, 0][(direction & 3) as usize]
}

/// Native `rcGetDirOffsetY` (the second grid axis is world Z).
pub fn direction_offset_z(direction: i32) -> i32 {
    [0, 1, 0, -1][(direction & 3) as usize]
}

/// Native `rcGetDirForOffset`, or `None` for undefined arithmetic or an
/// out-of-bounds native lookup. The center offset returns -1.
pub fn direction_for_offset(x: i32, z: i32) -> Option<i32> {
    let z = z.checked_add(1)?;
    if z < 0 {
        return None;
    }
    // A nonnegative signed left operand may shift into the sign bit in C++.
    let index = ((z as u32) << 1) as i32;
    let index = index.checked_add(x)?;
    [3, 0, -1, 2, 1].get(usize::try_from(index).ok()?).copied()
}

// C++ casts truncate toward zero, including just-outside negative coordinates.
fn footprint(hf: &CompactHeightfield, min: [f32; 3], max: [f32; 3]) -> ([i32; 3], [i32; 3]) {
    let origin = hf.aabb.min.to_array();
    let scale = [hf.cell_size, hf.cell_height, hf.cell_size];
    let mut lo = std::array::from_fn(|i| ((min[i] - origin[i]) / scale[i]) as i32);
    let mut hi = std::array::from_fn(|i| ((max[i] - origin[i]) / scale[i]) as i32);
    lo[0] = lo[0].max(0);
    lo[2] = lo[2].max(0);
    hi[0] = hi[0].min(hf.width - 1);
    hi[2] = hi[2].min(hf.height - 1);
    (lo, hi)
}

/// Mark non-null spans in the inclusive voxel footprint of an axis-aligned box.
/// Corresponds to `rcMarkBoxArea`; Y bounds include both endpoint voxel heights.
pub fn mark_box_area(hf: &mut CompactHeightfield, min: [f32; 3], max: [f32; 3], area: AreaType) {
    let (lo, hi) = footprint(hf, min, max);
    for z in lo[2]..=hi[2] {
        for x in lo[0]..=hi[0] {
            for i in hf.cells[(x + z * hf.width) as usize].index_range() {
                let y = hf.spans[i].y as i32;
                if y >= lo[1] && y <= hi[1] && hf.areas[i] != AreaType::NOT_WALKABLE {
                    hf.areas[i] = area;
                }
            }
        }
    }
}

/// Mark a vertical cylinder (`rcMarkCylinderArea`). Cell centers exactly on the
/// radius are excluded. Null areas remain null, even when marking a walkable ID.
pub fn mark_cylinder_area(
    hf: &mut CompactHeightfield,
    pos: [f32; 3],
    radius: f32,
    height: f32,
    area: AreaType,
) {
    let (lo, hi) = footprint(
        hf,
        [pos[0] - radius, pos[1], pos[2] - radius],
        [pos[0] + radius, pos[1] + height, pos[2] + radius],
    );
    let radius_squared = radius * radius;
    for z in lo[2]..=hi[2] {
        for x in lo[0]..=hi[0] {
            for i in hf.cells[(x + z * hf.width) as usize].index_range() {
                let y = hf.spans[i].y as i32;
                if hf.areas[i] == AreaType::NOT_WALKABLE || y < lo[1] || y > hi[1] {
                    continue;
                }
                let dx = madd(x as f32 + 0.5, hf.cell_size, hf.aabb.min.x) - pos[0];
                let dz = madd(z as f32 + 0.5, hf.cell_size, hf.aabb.min.z) - pos[2];
                if madd(dx, dx, dz * dz) < radius_squared {
                    hf.areas[i] = area;
                }
            }
        }
    }
}

/// Mark the XZ projection of a convex polygon extruded between two Y bounds
/// (`rcMarkConvexPolyArea`). Vertex Y coordinates are ignored. Empty input is a no-op.
pub fn mark_convex_poly_area(
    hf: &mut CompactHeightfield,
    vertices: &[[f32; 3]],
    min_y: f32,
    max_y: f32,
    area: AreaType,
) {
    let Some(&first) = vertices.first() else {
        return;
    };
    let mut min = first;
    let mut max = first;
    for v in &vertices[1..] {
        for axis in [0, 2] {
            min[axis] = min[axis].min(v[axis]);
            max[axis] = max[axis].max(v[axis]);
        }
    }
    min[1] = min_y;
    max[1] = max_y;
    let (lo, hi) = footprint(hf, min, max);
    for z in lo[2]..=hi[2] {
        for x in lo[0]..=hi[0] {
            for i in hf.cells[(x + z * hf.width) as usize].index_range() {
                let y = hf.spans[i].y as i32;
                if hf.areas[i] == AreaType::NOT_WALKABLE || y < lo[1] || y > hi[1] {
                    continue;
                }
                let px = madd(x as f32 + 0.5, hf.cell_size, hf.aabb.min.x);
                let pz = madd(z as f32 + 0.5, hf.cell_size, hf.aabb.min.z);
                let mut inside = false;
                let mut j = vertices.len() - 1;
                for (i, vi) in vertices.iter().enumerate() {
                    let vj = vertices[j];
                    if (vi[2] > pz) != (vj[2] > pz)
                        && px < (vj[0] - vi[0]) * (pz - vi[2]) / (vj[2] - vi[2]) + vi[0]
                    {
                        inside = !inside;
                    }
                    j = i;
                }
                if inside {
                    hf.areas[i] = area;
                }
            }
        }
    }
}

/// Apply the nine-sample median area filter (`rcMedianFilterWalkableArea`).
/// Samples use the original field for the entire pass. Missing and null neighbors
/// contribute the current span's ID; null current spans are preserved.
pub fn median_filter_walkable_area(hf: &mut CompactHeightfield) {
    let mut areas = vec![AreaType(255); hf.spans.len()];
    for z in 0..hf.height {
        for x in 0..hf.width {
            for i in hf.cells[x as usize + z as usize * hf.width as usize].index_range() {
                if hf.areas[i] == AreaType::NOT_WALKABLE {
                    areas[i] = hf.areas[i];
                    continue;
                }
                let mut neighbors = [hf.areas[i].0; 9];
                let mut cardinal_indices = [usize::MAX; 4];
                let mut same_cardinals = true;
                for dir in 0..4 {
                    if let Some(con) = hf.spans[i].con(dir) {
                        let (_, _, ai) = hf.con_indices(x, z, dir, con);
                        cardinal_indices[dir as usize] = ai;
                        if hf.areas[ai] != AreaType::NOT_WALKABLE {
                            neighbors[dir as usize * 2] = hf.areas[ai].0;
                        }
                    }
                    same_cardinals &= neighbors[dir as usize * 2] == hf.areas[i].0;
                }
                // The center and four effective cardinal samples give five
                // identical votes. Diagonals cannot change the nine-way median.
                if same_cardinals {
                    areas[i] = hf.areas[i];
                    continue;
                }
                for dir in 0..4_u8 {
                    let ai = cardinal_indices[usize::from(dir)];
                    if ai != usize::MAX {
                        let dir2 = (dir + 1) & 3;
                        if let Some(con2) = hf.spans[ai].con(dir2) {
                            let ax = x + direction_offset_x(i32::from(dir));
                            let az = z + direction_offset_z(i32::from(dir));
                            let (_, _, ai2) = hf.con_indices(ax, az, dir2, con2);
                            if hf.areas[ai2] != AreaType::NOT_WALKABLE {
                                neighbors[usize::from(dir) * 2 + 1] = hf.areas[ai2].0;
                            }
                        }
                    }
                }
                // Upstream's nine-element insertion sort.
                for j in 1..9 {
                    let value = neighbors[j];
                    let mut k = j;
                    while k > 0 && neighbors[k - 1] > value {
                        neighbors[k] = neighbors[k - 1];
                        k -= 1;
                    }
                    neighbors[k] = value;
                }
                areas[i] = AreaType(neighbors[4]);
            }
        }
    }
    hf.areas = areas;
}

/// Offset an XZ polygon (`rcOffsetPoly`), preserving each corner's Y coordinate.
/// Returns the used vertex count, or zero if the output buffer is too small.
/// Upstream requires one spare slot: a result of N vertices needs capacity N+1.
/// On exhaustion, previously written vertices remain in the buffer.
pub fn offset_poly(vertices: &[[f32; 3]], offset: f32, output: &mut [[f32; 3]]) -> usize {
    // Keep the already-negated normal components as the addition's operands.
    #[inline(never)]
    fn average(a: f32, b: f32) -> f32 {
        (a + b) * 0.5
    }
    let mut n = 0;
    for (i, &vb) in vertices.iter().enumerate() {
        let va = vertices[(i + vertices.len() - 1) % vertices.len()];
        let vc = vertices[(i + 1) % vertices.len()];
        let mut dx0 = vb[0] - va[0];
        let mut dz0 = vb[2] - va[2];
        let d0 = madd(dx0, dx0, dz0 * dz0);
        if d0 > 1e-6 {
            let scale = 1. / d0.sqrt();
            dx0 *= scale;
            dz0 *= scale;
        }
        let mut dx1 = vc[0] - vb[0];
        let mut dz1 = vc[2] - vb[2];
        let d1 = madd(dx1, dx1, dz1 * dz1);
        if d1 > 1e-6 {
            let scale = 1. / d1.sqrt();
            dx1 *= scale;
            dz1 *= scale;
        }
        let dlx0 = -dz0;
        let dlz0 = dx0;
        let dlx1 = -dz1;
        let dlz1 = dx1;
        let cross = crate::common::multiply_subtract(dx1, dz0, dx0 * dz1);
        let mut dmx = average(dlx0, dlx1);
        let mut dmz = average(dlz0, dlz1);
        let dmr2 = madd(dmx, dmx, dmz * dmz);
        let bevel = dmr2 * 1.20 * 1.20 < 1.;
        if dmr2 > 1e-6 {
            let scale = 1. / dmr2;
            dmx *= scale;
            dmz *= scale;
        }
        if bevel && cross < 0. {
            if n + 2 >= output.len() {
                return 0;
            }
            let d = (1. - madd(dx0, dx1, dz0 * dz1)) * 0.5;
            output[n] = [
                add_scaled(vb[0], add_scaled(-dlx0, dx0, d), offset),
                vb[1],
                add_scaled(vb[2], add_scaled(-dlz0, dz0, d), offset),
            ];
            n += 1;
            output[n] = [
                add_scaled(vb[0], subtract_scaled(-dlx1, dx1, d), offset),
                vb[1],
                add_scaled(vb[2], subtract_scaled(-dlz1, dz1, d), offset),
            ];
            n += 1;
        } else {
            if n + 1 >= output.len() {
                return 0;
            }
            output[n] = [
                subtract_scaled(vb[0], dmx, offset),
                vb[1],
                subtract_scaled(vb[2], dmz, offset),
            ];
            n += 1;
        }
    }
    n
}

/// Native `rcCopyPolyMesh`: copy live data and reset capacity to the live count.
/// Initialized spare slots are not copied. Rust `Clone` retains all Rust fields.
pub fn copy_poly_mesh(source: &PolygonNavmesh) -> PolygonNavmesh {
    let count = source.polygon_count();
    let words = count * source.max_vertices_per_polygon as usize;
    PolygonNavmesh {
        vertices: source.vertices.clone(),
        polygons: source.polygons[..words].to_vec(),
        polygon_neighbors: source.polygon_neighbors[..words].to_vec(),
        flags: source.flags[..count].to_vec(),
        regions: source.regions[..count].to_vec(),
        areas: source.areas[..count].to_vec(),
        max_polygons: count,
        max_vertices_per_polygon: source.max_vertices_per_polygon,
        aabb: source.aabb,
        cell_size: source.cell_size,
        cell_height: source.cell_height,
        border_size: source.border_size,
        max_edge_error: source.max_edge_error,
        ..Default::default()
    }
}

/// Merge polygon meshes (`rcMergePolyMeshes`). Inputs must have matching polygon
/// sizes and cell scales, valid indices and complete per-polygon attribute arrays.
/// Vertex welding uses the upstream two-voxel height threshold and hash-chain
/// order. Only outer portals survive; adjacency is rebuilt after welding.
/// Empty input produces an empty mesh. Native merging retains zero `maxpolys`.
pub fn merge_poly_meshes(meshes: &[&PolygonNavmesh]) -> PolygonNavmesh {
    use glam::U16Vec3;
    let Some(first) = meshes.first() else {
        return Default::default();
    };
    let mut output = PolygonNavmesh {
        max_vertices_per_polygon: first.max_vertices_per_polygon,
        cell_size: first.cell_size,
        cell_height: first.cell_height,
        aabb: first.aabb,
        ..Default::default()
    };
    for mesh in meshes {
        output.aabb.min = output.aabb.min.min(mesh.aabb.min);
        output.aabb.max = output.aabb.max.max(mesh.aabb.max);
    }
    let nvp = output.max_vertices_per_polygon as usize;
    let max_vertices: usize = meshes.iter().map(|m| m.vertices.len()).sum();
    let max_polygons: usize = meshes.iter().map(|m| m.polygon_count()).sum();
    output.vertices.reserve_exact(max_vertices);
    output.polygons.reserve_exact(max_polygons * nvp);
    output.polygon_neighbors.reserve_exact(max_polygons * nvp);
    output.regions.reserve_exact(max_polygons);
    output.areas.reserve_exact(max_polygons);
    output.flags.reserve_exact(max_polygons);
    // Native chains use signed 32-bit indices with -1 as the empty sentinel.
    // Their defined index domain fits this unsigned representation exactly.
    const EMPTY: u32 = u32::MAX;
    let mut first_vertex = vec![EMPTY; 4096];
    let mut next_vertex = Vec::with_capacity(max_vertices);
    let mut remap = Vec::with_capacity(meshes.iter().map(|m| m.vertices.len()).max().unwrap_or(0));
    for mesh in meshes {
        let ox = ((mesh.aabb.min.x - output.aabb.min.x) / output.cell_size + 0.5).floor() as u16;
        let oz = ((mesh.aabb.min.z - output.aabb.min.z) / output.cell_size + 0.5).floor() as u16;
        let max_x =
            ((output.aabb.max.x - mesh.aabb.max.x) / output.cell_size + 0.5).floor() as u16 == 0;
        let max_z =
            ((output.aabb.max.z - mesh.aabb.max.z) / output.cell_size + 0.5).floor() as u16 == 0;
        let border = [ox == 0, max_z, max_x, oz == 0];
        remap.clear();
        for v in &mesh.vertices {
            let vertex = U16Vec3::new(v.x.wrapping_add(ox), v.y, v.z.wrapping_add(oz));
            let bucket = ((0x8da6b343u32.wrapping_mul(vertex.x as u32))
                .wrapping_add(0xcb1ab31fu32.wrapping_mul(vertex.z as u32))
                & 4095) as usize;
            let mut candidate = first_vertex[bucket];
            let mut found = None;
            while candidate != EMPTY {
                let i = candidate as usize;
                let other: U16Vec3 = output.vertices[i];
                if other.x == vertex.x && other.z == vertex.z && other.y.abs_diff(vertex.y) <= 2 {
                    found = Some(i);
                    break;
                }
                candidate = next_vertex[i];
            }
            let index = found.unwrap_or_else(|| {
                let i = output.vertices.len();
                output.vertices.push(vertex);
                next_vertex.push(first_vertex[bucket]);
                first_vertex[bucket] = i as u32;
                i
            });
            remap.push(index as u16);
        }
        for p in 0..mesh.polygon_count() {
            let start = output.polygons.len();
            output.polygons.resize(start + nvp, u16::MAX);
            output.polygon_neighbors.resize(start + nvp, u16::MAX);
            for (target, &v) in output.polygons[start..]
                .iter_mut()
                .zip(&mesh.polygons[p * nvp..(p + 1) * nvp])
            {
                if v == u16::MAX {
                    break;
                }
                *target = remap[v as usize];
            }
            for (k, target) in output.polygon_neighbors[start..].iter_mut().enumerate() {
                let source = mesh.polygon_neighbors[p * nvp + k];
                let direction = (source & 0xf) as usize;
                if source & 0x8000 != 0 && source != u16::MAX && direction < 4 && border[direction]
                {
                    *target = source;
                }
            }
            output.regions.push(mesh.regions[p]);
            output.areas.push(mesh.areas[p]);
            output.flags.push(mesh.flags[p]);
        }
    }
    rebuild_mesh_adjacency(&mut output);
    output
}

struct MeshEdge {
    vertices: [u16; 2],
    polygons: [u16; 2],
    polygon_edges: [u16; 2],
}

// Upstream buildMeshAdjacency, including its 16-bit edge-chain indices.
// Based on Eric Lengyel's edge algorithm referenced by RecastMesh.cpp.
fn rebuild_mesh_adjacency(mesh: &mut PolygonNavmesh) {
    if mesh.polygon_count() == 0 {
        return;
    }
    let nvp = mesh.max_vertices_per_polygon as usize;
    let mut first_edge = vec![u16::MAX; mesh.vertices.len()];
    let max_edges = mesh.polygon_count() * nvp;
    let mut next_edge = Vec::with_capacity(max_edges);
    let mut edges = Vec::<MeshEdge>::with_capacity(max_edges);
    for (p, polygon) in mesh.polygons.chunks(nvp).enumerate() {
        let count = polygon.iter().position(|&v| v == u16::MAX).unwrap_or(nvp);
        for edge in 0..count {
            let v0 = polygon[edge];
            let v1 = polygon[(edge + 1) % count];
            if v0 < v1 {
                next_edge.push(first_edge[v0 as usize]);
                first_edge[v0 as usize] = edges.len() as u16;
                edges.push(MeshEdge {
                    vertices: [v0, v1],
                    polygons: [p as u16; 2],
                    polygon_edges: [edge as u16, 0],
                });
            }
        }
    }
    for (p, polygon) in mesh.polygons.chunks(nvp).enumerate() {
        let count = polygon.iter().position(|&v| v == u16::MAX).unwrap_or(nvp);
        for edge in 0..count {
            let v0 = polygon[edge];
            let v1 = polygon[(edge + 1) % count];
            if v0 > v1 {
                let mut candidate = first_edge[v1 as usize];
                while candidate != u16::MAX {
                    let e = &mut edges[candidate as usize];
                    if e.vertices[1] == v0 && e.polygons[0] == e.polygons[1] {
                        e.polygons[1] = p as u16;
                        e.polygon_edges[1] = edge as u16;
                        break;
                    }
                    candidate = next_edge[candidate as usize];
                }
            }
        }
    }
    for e in edges {
        if e.polygons[0] != e.polygons[1] {
            mesh.polygon_neighbors[e.polygons[0] as usize * nvp + e.polygon_edges[0] as usize] =
                e.polygons[1];
            mesh.polygon_neighbors[e.polygons[1] as usize * nvp + e.polygon_edges[1] as usize] =
                e.polygons[0];
        }
    }
}

/// Merge detail submeshes (`rcMergePolyMeshDetails`), skipping null source entries.
/// Preserves local triangle indices, flags and vertex bits; rebases submesh offsets.
pub fn merge_detail_meshes(meshes: &[Option<&DetailNavmesh>]) -> DetailNavmesh {
    let mut output = DetailNavmesh::default();
    output
        .meshes
        .reserve_exact(meshes.iter().flatten().map(|m| m.meshes.len()).sum());
    output
        .vertices
        .reserve_exact(meshes.iter().flatten().map(|m| m.vertices.len()).sum());
    let max_triangles = meshes.iter().flatten().map(|m| m.triangles.len()).sum();
    output.triangles.reserve_exact(max_triangles);
    output.triangle_flags.reserve_exact(max_triangles);
    for mesh in meshes.iter().flatten() {
        let vertex_base = output.vertices.len() as u32;
        let triangle_base = output.triangles.len() as u32;
        for sub in &mesh.meshes {
            let mut sub = sub.clone();
            sub.base_vertex_index = sub.base_vertex_index.wrapping_add(vertex_base);
            sub.base_triangle_index = sub.base_triangle_index.wrapping_add(triangle_base);
            output.meshes.push(sub);
        }
        output.vertices.extend_from_slice(&mesh.vertices);
        output.triangles.extend_from_slice(&mesh.triangles);
        output
            .triangle_flags
            .extend_from_slice(&mesh.triangle_flags);
    }
    output
}

/// Mark walkable triangles, preserving other area IDs (`rcMarkWalkableTriangles`).
pub fn mark_walkable_triangles(
    slope_degrees: f32,
    vertices: &[[f32; 3]],
    triangles: &[[u32; 3]],
    areas: &mut [AreaType],
) {
    let limit = crate::common::cosine(slope_degrees / 180. * std::f32::consts::PI);
    for (&t, a) in triangles.iter().zip(areas) {
        if triangle_normal(vertices, t)[1] > limit {
            *a = AreaType::DEFAULT_WALKABLE;
        }
    }
}
fn triangle_normal(vertices: &[[f32; 3]], triangle: [u32; 3]) -> [f32; 3] {
    let [a, b, c] = triangle.map(|i| vertices[i as usize]);
    let ab: [f32; 3] = std::array::from_fn(|i| b[i] - a[i]);
    let ac: [f32; 3] = std::array::from_fn(|i| c[i] - a[i]);
    let n = [
        madd(ab[1], ac[2], -ab[2] * ac[1]),
        madd(ab[2], ac[0], -ab[0] * ac[2]),
        madd(ab[0], ac[1], -ab[1] * ac[0]),
    ];
    // Native rcSqr calls keep these products separate at the oracle's -O0.
    let length = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    let inverse = 1. / length;
    n.map(|v| v * inverse)
}
/// Clear unwalkable triangles (`rcClearUnwalkableTriangles`), preserving custom
/// area IDs on other triangles. Degenerate normals compare as NaN, as upstream.
pub fn clear_unwalkable_triangles(
    slope_degrees: f32,
    vertices: &[[f32; 3]],
    triangles: &[[u32; 3]],
    areas: &mut [AreaType],
) {
    let limit = crate::common::cosine(slope_degrees / 180. * std::f32::consts::PI);
    for (&t, a) in triangles.iter().zip(areas) {
        if triangle_normal(vertices, t)[1] <= limit {
            *a = AreaType::NOT_WALKABLE;
        }
    }
}
/// Bounds of a nonempty finite vertex array (`rcCalcBounds`); empty input returns None.
pub fn calc_bounds(vertices: &[[f32; 3]]) -> Option<[[f32; 3]; 2]> {
    let &first = vertices.first()?;
    let mut lo = first;
    let mut hi = first;
    for v in &vertices[1..] {
        for i in 0..3 {
            // Preserve upstream tie behavior, including signed zero bits.
            lo[i] = if lo[i] < v[i] { lo[i] } else { v[i] };
            hi[i] = if hi[i] > v[i] { hi[i] } else { v[i] };
        }
    }
    Some([lo, hi])
}
/// Rounded X/Z grid dimensions (`rcCalcGridSize`). Cell size must be positive;
/// dimensions must be finite and representable as signed 32-bit integers.
pub fn calc_grid_size(bounds: [[f32; 3]; 2], cell_size: f32) -> [i32; 2] {
    [0, 2].map(|i| ((bounds[1][i] - bounds[0][i]) / cell_size + 0.5) as i32)
}

/// Insert/merge a solid span (`rcAddSpan`). Coordinates must identify a valid cell.
pub fn add_span(
    hf: &mut Heightfield,
    x: i32,
    z: i32,
    min: u16,
    max: u16,
    area: AreaType,
    threshold: i32,
) -> Result<(), SpanInsertionError> {
    hf.add_span(SpanInsertion {
        x,
        z,
        span: Span::new(min, max, area),
        flag_merge_threshold: threshold,
    })
}
/// Count non-null linked spans (`rcGetHeightFieldSpanCount`).
pub fn heightfield_span_count(hf: &Heightfield) -> usize {
    let mut count = 0;
    let columns = hf
        .width
        .checked_mul(hf.height)
        .and_then(|n| usize::try_from(n).ok())
        .unwrap_or(0);
    for &head in hf.spans.iter().take(columns) {
        let mut cursor = head;
        while let Some(key) = cursor {
            let span = hf.span(key);
            if span.area() != AreaType::NOT_WALKABLE {
                count += 1;
            }
            cursor = span.next;
        }
    }
    count
}
/// Rasterize one triangle (`rcRasterizeTriangle`).
pub fn rasterize_triangle(
    hf: &mut Heightfield,
    vertices: [[f32; 3]; 3],
    area: AreaType,
    threshold: i32,
) -> Result<(), RasterizationError> {
    hf.rasterize_triangle(vertices.map(Into::into), area, threshold)
}
/// Indexed 32-bit triangle rasterization (`rcRasterizeTriangles`).
pub fn rasterize_triangles_u32(
    hf: &mut Heightfield,
    vertices: &[[f32; 3]],
    triangles: &[[u32; 3]],
    areas: &[AreaType],
    threshold: i32,
) -> Result<(), RasterizationError> {
    for (&t, &area) in triangles.iter().zip(areas) {
        rasterize_triangle(hf, t.map(|i| vertices[i as usize]), area, threshold)?;
    }
    Ok(())
}
/// Indexed 16-bit triangle rasterization (`rcRasterizeTriangles`).
pub fn rasterize_triangles_u16(
    hf: &mut Heightfield,
    vertices: &[[f32; 3]],
    triangles: &[[u16; 3]],
    areas: &[AreaType],
    threshold: i32,
) -> Result<(), RasterizationError> {
    for (&t, &area) in triangles.iter().zip(areas) {
        rasterize_triangle(hf, t.map(|i| vertices[i as usize]), area, threshold)?;
    }
    Ok(())
}
/// Nonindexed triangle rasterization (`rcRasterizeTriangles`).
pub fn rasterize_triangles(
    hf: &mut Heightfield,
    triangles: &[[[f32; 3]; 3]],
    areas: &[AreaType],
    threshold: i32,
) -> Result<(), RasterizationError> {
    for (&t, &area) in triangles.iter().zip(areas) {
        rasterize_triangle(hf, t, area, threshold)?;
    }
    Ok(())
}

/// `rcMarkBoxArea` with native context hooks.
pub fn mark_box_area_with_context(
    context: &Context<'_>,
    hf: &mut CompactHeightfield,
    min: [f32; 3],
    max: [f32; 3],
    area: AreaType,
) {
    let _timer = context.scoped_timer(TimerLabel::MARK_BOX_AREA);
    mark_box_area(hf, min, max, area);
}

/// `rcMarkCylinderArea` with native context hooks.
pub fn mark_cylinder_area_with_context(
    context: &Context<'_>,
    hf: &mut CompactHeightfield,
    position: [f32; 3],
    radius: f32,
    height: f32,
    area: AreaType,
) {
    let _timer = context.scoped_timer(TimerLabel::MARK_CYLINDER_AREA);
    mark_cylinder_area(hf, position, radius, height, area);
}

/// `rcMarkConvexPolyArea` with native context hooks.
pub fn mark_convex_poly_area_with_context(
    context: &Context<'_>,
    hf: &mut CompactHeightfield,
    vertices: &[[f32; 3]],
    min_y: f32,
    max_y: f32,
    area: AreaType,
) {
    let _timer = context.scoped_timer(TimerLabel::MARK_CONVEXPOLY_AREA);
    mark_convex_poly_area(hf, vertices, min_y, max_y, area);
}

/// `rcMedianFilterWalkableArea` with native context hooks.
pub fn median_filter_walkable_area_with_context(
    context: &Context<'_>,
    hf: &mut CompactHeightfield,
) {
    let _timer = context.scoped_timer(TimerLabel::MEDIAN_AREA);
    median_filter_walkable_area(hf);
}

/// `rcMergePolyMeshes` with native context hooks. Empty input starts no timer.
pub fn merge_poly_meshes_with_context(
    context: &Context<'_>,
    meshes: &[&PolygonNavmesh],
) -> PolygonNavmesh {
    let _timer = (!meshes.is_empty()).then(|| context.scoped_timer(TimerLabel::MERGE_POLYMESH));
    let output = merge_poly_meshes(meshes);
    for (kind, count) in [
        ("vertices", output.vertices.len()),
        ("polygons", output.polygon_count()),
    ] {
        if count > 0xffff {
            let _ = context.log_format(LogCategory::ERROR, format_args!(
                "rcMergePolyMeshes: The resulting mesh has too many {kind} {count} (max 65535). Data can be corrupted."
            ));
        }
    }
    output
}

/// `rcMergePolyMeshDetails` with native context hooks, including empty input.
pub fn merge_detail_meshes_with_context(
    context: &Context<'_>,
    meshes: &[Option<&DetailNavmesh>],
) -> DetailNavmesh {
    let _timer = context.scoped_timer(TimerLabel::MERGE_POLYMESHDETAIL);
    merge_detail_meshes(meshes)
}

/// Single-triangle rasterization with its native scoped timer.
pub fn rasterize_triangle_with_context(
    context: &Context<'_>,
    hf: &mut Heightfield,
    vertices: [[f32; 3]; 3],
    area: AreaType,
    threshold: i32,
) -> Result<(), RasterizationError> {
    hf.rasterize_triangle_with_context(context, vertices.map(Into::into), area, threshold)
}

/// Indexed 32-bit rasterization with one timer for the entire batch.
pub fn rasterize_triangles_u32_with_context(
    context: &Context<'_>,
    hf: &mut Heightfield,
    vertices: &[[f32; 3]],
    triangles: &[[u32; 3]],
    areas: &[AreaType],
    threshold: i32,
) -> Result<(), RasterizationError> {
    let _timer = context.scoped_timer(TimerLabel::RASTERIZE_TRIANGLES);
    rasterize_triangles_u32(hf, vertices, triangles, areas, threshold)
}

/// Indexed 16-bit rasterization with one timer for the entire batch.
pub fn rasterize_triangles_u16_with_context(
    context: &Context<'_>,
    hf: &mut Heightfield,
    vertices: &[[f32; 3]],
    triangles: &[[u16; 3]],
    areas: &[AreaType],
    threshold: i32,
) -> Result<(), RasterizationError> {
    let _timer = context.scoped_timer(TimerLabel::RASTERIZE_TRIANGLES);
    rasterize_triangles_u16(hf, vertices, triangles, areas, threshold)
}

/// Nonindexed rasterization with one timer for the entire batch.
pub fn rasterize_triangles_with_context(
    context: &Context<'_>,
    hf: &mut Heightfield,
    triangles: &[[[f32; 3]; 3]],
    areas: &[AreaType],
    threshold: i32,
) -> Result<(), RasterizationError> {
    let _timer = context.scoped_timer(TimerLabel::RASTERIZE_TRIANGLES);
    rasterize_triangles(hf, triangles, areas, threshold)
}
