// Rust translation of RecastNavigation v1.6.0 RecastContour.cpp hole merging.
// Original: Copyright (c) 2009-2010 Mikko Mononen, licensed under zlib.
// This is an altered implementation; see vendor/recastnavigation/License.txt.
use crate::{
    ContourSet,
    context::{Context, LogCategory},
};
use alloc::vec::Vec;
use glam::IVec3;

type Vertex = (IVec3, u32);

fn area2(a: IVec3, b: IVec3, c: IVec3) -> i64 {
    let [ax, _, az] = a.to_array().map(i64::from);
    let [bx, _, bz] = b.to_array().map(i64::from);
    let [cx, _, cz] = c.to_array().map(i64::from);
    (bx - ax) * (cz - az) - (cx - ax) * (bz - az)
}

fn same(a: IVec3, b: IVec3) -> bool {
    a.x == b.x && a.z == b.z
}

fn between(a: IVec3, b: IVec3, c: IVec3) -> bool {
    if area2(a, b, c) != 0 {
        return false;
    }
    if a.x != b.x {
        (a.x.min(b.x)..=a.x.max(b.x)).contains(&c.x)
    } else {
        (a.z.min(b.z)..=a.z.max(b.z)).contains(&c.z)
    }
}

fn intersects(a: IVec3, b: IVec3, c: IVec3, d: IVec3) -> bool {
    let abc = area2(a, b, c);
    let abd = area2(a, b, d);
    let cda = area2(c, d, a);
    let cdb = area2(c, d, b);
    (abc != 0
        && abd != 0
        && cda != 0
        && cdb != 0
        && (abc < 0) != (abd < 0)
        && (cda < 0) != (cdb < 0))
        || between(a, b, c)
        || between(a, b, d)
        || between(c, d, a)
        || between(c, d, b)
}

fn intersects_contour(a: IVec3, b: IVec3, vertices: &[Vertex]) -> bool {
    (0..vertices.len()).any(|i| {
        let c = vertices[i].0;
        let d = vertices[(i + 1) % vertices.len()].0;
        // This also excludes all edges incident to the candidate diagonal's endpoint.
        !same(a, c) && !same(a, d) && !same(b, c) && !same(b, d) && intersects(a, b, c, d)
    })
}

fn in_cone(i: usize, vertices: &[Vertex], corner: IVec3) -> bool {
    let p = vertices[i].0;
    let next = vertices[(i + 1) % vertices.len()].0;
    let prev = vertices[(i + vertices.len() - 1) % vertices.len()].0;
    if area2(prev, p, next) <= 0 {
        area2(p, corner, prev) < 0 && area2(corner, p, next) < 0
    } else {
        !(area2(p, corner, next) <= 0 && area2(corner, p, prev) <= 0)
    }
}

fn winding(vertices: &[Vertex]) -> i64 {
    let mut area = 0_i64;
    for i in 0..vertices.len() {
        let a = vertices[i].0;
        let b = vertices[(i + vertices.len() - 1) % vertices.len()].0;
        area += i64::from(a.x) * i64::from(b.z) - i64::from(b.x) * i64::from(a.z);
    }
    (area + 1) / 2
}

pub(crate) fn merge_holes(context: &Context<'_>, set: &mut ContourSet) {
    let windings: Vec<_> = set.contours.iter().map(|c| winding(&c.vertices)).collect();
    if windings.iter().all(|&w| w >= 0) {
        return;
    }
    let max_region = set
        .contours
        .iter()
        .map(|c| c.region.bits())
        .max()
        .unwrap_or(0);
    let mut outlines = vec![None; usize::from(max_region) + 1];
    for (i, contour) in set.contours.iter().enumerate() {
        if windings[i] >= 0 {
            let region = usize::from(contour.region.bits());
            if outlines[region].is_some() {
                let _ = context.log_format(
                    LogCategory::ERROR,
                    format_args!("rcBuildContours: Multiple outlines for region {region}."),
                );
            }
            outlines[region] = Some(i);
        }
    }
    for region in 0..=max_region {
        let mut holes: Vec<_> = set
            .contours
            .iter()
            .enumerate()
            .filter(|(i, c)| c.region.bits() == region && windings[*i] < 0)
            .map(|(i, c)| {
                let leftmost = (0..c.vertices.len())
                    .min_by_key(|&j| {
                        let p = c.vertices[j].0;
                        (p.x, p.z)
                    })
                    .unwrap();
                let p = c.vertices[leftmost].0;
                (p.x, p.z, i, leftmost)
            })
            .collect();
        if holes.is_empty() {
            continue;
        }
        let Some(outline) = outlines[usize::from(region)] else {
            let _ = context.log_format(LogCategory::ERROR, format_args!(
                "rcBuildContours: Bad outline for region {region}, contour simplification is likely too aggressive."
            ));
            continue;
        };
        crate::native_sort::sort_by_key(&mut holes, |h| (h.0, h.1));
        for h in 0..holes.len() {
            let (_, _, hole, leftmost) = holes[h];
            let n = set.contours[hole].vertices.len();
            let mut connection = None;
            for iteration in 0..n {
                let best = (leftmost + iteration) % n;
                let corner = set.contours[hole].vertices[best].0;
                let vertices = &set.contours[outline].vertices;
                let mut candidates: Vec<_> = (0..vertices.len())
                    .filter(|&j| in_cone(j, vertices, corner))
                    .map(|j| {
                        let dx = i64::from(vertices[j].0.x) - i64::from(corner.x);
                        let dz = i64::from(vertices[j].0.z) - i64::from(corner.z);
                        (dx * dx + dz * dz, j)
                    })
                    .collect();
                crate::native_sort::sort_by_key(&mut candidates, |c| c.0);
                for (_, j) in candidates {
                    let point = vertices[j].0;
                    if !intersects_contour(point, corner, vertices)
                        && holes[h..].iter().all(|hole| {
                            !intersects_contour(point, corner, &set.contours[hole.2].vertices)
                        })
                    {
                        connection = Some((j, best));
                        break;
                    }
                }
                if connection.is_some() {
                    break;
                }
            }
            let Some((a, b)) = connection else {
                let _ = context.log_format(
                    LogCategory::WARNING,
                    format_args!(
                        "mergeHoles: Failed to find merge points for {:p} and {:p}.",
                        core::ptr::from_ref(&set.contours[outline]),
                        core::ptr::from_ref(&set.contours[hole])
                    ),
                );
                continue;
            };
            let outline_vertices = &set.contours[outline].vertices;
            let hole_vertices = &set.contours[hole].vertices;
            let mut merged = Vec::with_capacity(outline_vertices.len() + hole_vertices.len() + 2);
            for i in 0..=outline_vertices.len() {
                merged.push(outline_vertices[(a + i) % outline_vertices.len()]);
            }
            for i in 0..=hole_vertices.len() {
                merged.push(hole_vertices[(b + i) % hole_vertices.len()]);
            }
            set.contours[outline].vertices = merged;
            set.contours[hole].vertices.clear();
        }
    }
}
