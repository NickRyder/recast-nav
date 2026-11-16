// Altered Rust translation of wall queries in DetourNavMeshQuery.cpp.
// Copyright (c) 2009-2010 Mikko Mononen. Zlib: vendor/recastnavigation/License.txt.
use super::NavMeshQuery;
use crate::{nav::*, nodes};

#[derive(Clone, Copy, Debug)]
pub struct WallSegment {
    pub positions: [Point; 2],
    pub reference: PolyRef,
}
#[derive(Clone, Debug, Default)]
pub struct WallSegments {
    pub status: u32,
    pub segments: Vec<WallSegment>,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct WallHit {
    pub distance: f32,
    pub position: Point,
    pub normal: Point,
}
#[derive(Clone, Copy, Default)]
struct Interval {
    reference: PolyRef,
    min: i16,
    max: i16,
}
fn insert_interval(
    intervals: &mut [Interval; 16],
    count: &mut usize,
    min: i16,
    max: i16,
    reference: PolyRef,
) {
    if *count == intervals.len() {
        return;
    }
    let index = intervals[..*count]
        .iter()
        .position(|i| max <= i.min)
        .unwrap_or(*count);
    intervals.copy_within(index..*count, index + 1);
    intervals[index] = Interval {
        reference,
        min,
        max,
    };
    *count += 1;
}
impl NavMesh {
    pub fn poly_wall_segments<F: QueryFilter + ?Sized>(
        &self,
        reference: PolyRef,
        filter: &F,
        store_portals: bool,
        capacity: usize,
    ) -> WallSegments {
        let mut result = WallSegments::default();
        self.poly_wall_segments_into(reference, filter, store_portals, capacity, &mut result);
        result
    }
    /// Reuses caller-owned wall segment storage, retaining allocation capacity.
    pub fn poly_wall_segments_into<F: QueryFilter + ?Sized>(
        &self,
        reference: PolyRef,
        filter: &F,
        store_portals: bool,
        capacity: usize,
        result: &mut WallSegments,
    ) -> u32 {
        result.segments.clear();
        self.poly_wall_segments_with_output(
            reference,
            filter,
            store_portals,
            capacity,
            &mut result.status,
            |segment| result.segments.push(segment),
        )
    }
    /// Emit the same ordered segments into owned or fixed caller storage.
    pub(crate) fn poly_wall_segments_with_output<F: QueryFilter + ?Sized>(
        &self,
        reference: PolyRef,
        filter: &F,
        store_portals: bool,
        capacity: usize,
        status: &mut u32,
        mut emit: impl FnMut(WallSegment),
    ) -> u32 {
        *status = SUCCESS;
        let Some(context) = self.filter_polygon(reference) else {
            *status = FAILURE | INVALID_PARAM;
            return *status;
        };
        if capacity > i32::MAX as usize {
            *status = FAILURE | INVALID_PARAM;
            return *status;
        }
        let mut count = 0;
        let mut append = |a, b, reference| {
            if count < capacity {
                emit(WallSegment {
                    positions: [a, b],
                    reference,
                });
                count += 1;
            } else {
                *status |= BUFFER_TOO_SMALL;
            }
        };
        for i in 0..context.polygon.vertices().len() {
            let j = if i == 0 {
                context.polygon.vertices().len() - 1
            } else {
                i - 1
            };
            let a = context.tile.vertex_at(context.polygon.vertex(j) as usize);
            let b = context.tile.vertex_at(context.polygon.vertex(i) as usize);
            let edge = context.polygon.neighbor(j);
            if edge & 0x8000 == 0 {
                let mut neighbor = 0;
                if edge != 0 {
                    let (_, tile, _) = self.reference_layout().decode(reference);
                    neighbor = self.reference(tile as usize, usize::from(edge - 1));
                    let Some(context) = self.polygon_from_bits(neighbor) else {
                        *status = FAILURE | INVALID_PARAM;
                        return *status;
                    };
                    if !filter.pass_filter(context) {
                        neighbor = 0;
                    }
                }
                if neighbor == 0 || store_portals {
                    append(a, b, neighbor);
                }
                continue;
            }
            let mut intervals = [Interval::default(); 16];
            let mut count = 0;
            for link in context.tile.links(context.polygon) {
                if usize::from(link.edge()) != j || link.reference() == 0 {
                    continue;
                }
                let Some(neighbor) = self.polygon_from_bits(link.reference()) else {
                    *status = FAILURE | INVALID_PARAM;
                    return *status;
                };
                if filter.pass_filter(neighbor) {
                    insert_interval(
                        &mut intervals,
                        &mut count,
                        link.min().into(),
                        link.max().into(),
                        link.reference(),
                    );
                }
            }
            insert_interval(&mut intervals, &mut count, -1, 0, 0);
            insert_interval(&mut intervals, &mut count, 255, 256, 0);
            for k in 1..count {
                let previous = intervals[k - 1];
                let interval = intervals[k];
                if store_portals && interval.reference != 0 {
                    append(
                        lerp(a, b, f32::from(interval.min) / 255.),
                        lerp(a, b, f32::from(interval.max) / 255.),
                        interval.reference,
                    );
                }
                if previous.max != interval.min {
                    append(
                        lerp(a, b, f32::from(previous.max) / 255.),
                        lerp(a, b, f32::from(interval.min) / 255.),
                        0,
                    );
                }
            }
        }
        *status
    }
}
impl NavMeshQuery {
    pub fn poly_wall_segments<F: QueryFilter + ?Sized>(
        &self,
        mesh: &NavMesh,
        reference: PolyRef,
        filter: &F,
        store_portals: bool,
        capacity: usize,
    ) -> WallSegments {
        mesh.poly_wall_segments(reference, filter, store_portals, capacity)
    }
    pub fn poly_wall_segments_into<F: QueryFilter + ?Sized>(
        &self,
        mesh: &NavMesh,
        reference: PolyRef,
        filter: &F,
        store_portals: bool,
        capacity: usize,
        result: &mut WallSegments,
    ) -> u32 {
        mesh.poly_wall_segments_into(reference, filter, store_portals, capacity, result)
    }
    pub fn find_distance_to_wall<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        center: Point,
        radius: f32,
        filter: &F,
    ) -> (u32, WallHit) {
        let mut hit = WallHit::default();
        let status = self.find_distance_to_wall_into(mesh, start, center, radius, filter, &mut hit);
        (status, hit)
    }
    pub fn find_distance_to_wall_into<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        center: Point,
        radius: f32,
        filter: &F,
        hit: &mut WallHit,
    ) -> u32 {
        if mesh.resolve(start).is_none()
            || !center.iter().all(|v| v.is_finite())
            || radius < 0.
            || !radius.is_finite()
        {
            return FAILURE | INVALID_PARAM;
        }
        self.pool.clear();
        self.open.clear();
        let Some(first) = self.pool.get_node(start, 0) else {
            return FAILURE | INVALID_PARAM;
        };
        let node = self.pool.node_mut(first).unwrap();
        node.position = center;
        node.set_flags(nodes::OPEN);
        self.open.push(first, &self.pool).unwrap();
        let mut radius_squared = radius * radius;
        let mut status = SUCCESS;
        while let Some(current) = self.open.pop(&self.pool) {
            let node = self.pool.node_mut(current).unwrap();
            node.set_flags((node.flags() & !nodes::OPEN) | nodes::CLOSED);
            let best = *node;
            let Some(context) = mesh.polygon_from_bits(best.reference) else {
                return FAILURE | INVALID_PARAM;
            };
            let parent = self
                .pool
                .node(best.parent_index())
                .map_or(0, |p| p.reference);
            if parent != 0 && mesh.polygon_from_bits(parent).is_none() {
                return FAILURE | INVALID_PARAM;
            }
            for i in 0..context.polygon.vertices().len() {
                let j = if i == 0 {
                    context.polygon.vertices().len() - 1
                } else {
                    i - 1
                };
                let edge = context.polygon.neighbor(j);
                if edge & 0x8000 != 0 {
                    let mut solid = true;
                    if let Some(link) = context
                        .tile
                        .links(context.polygon)
                        .find(|l| usize::from(l.edge()) == j)
                        && link.reference() != 0
                    {
                        let Some(neighbor) = mesh.polygon_from_bits(link.reference()) else {
                            return FAILURE | INVALID_PARAM;
                        };
                        if filter.pass_filter(neighbor) {
                            solid = false;
                        }
                    }
                    if !solid {
                        continue;
                    }
                } else if edge != 0 {
                    let (_, tile, _) = mesh.reference_layout().decode(best.reference);
                    let reference = mesh.reference(tile as usize, usize::from(edge - 1));
                    let Some(neighbor) = mesh.polygon_from_bits(reference) else {
                        return FAILURE | INVALID_PARAM;
                    };
                    if filter.pass_filter(neighbor) {
                        continue;
                    }
                }
                let a = context.tile.vertex_at(context.polygon.vertex(j) as usize);
                let b = context.tile.vertex_at(context.polygon.vertex(i) as usize);
                let (distance, t) = segment(center, a, b);
                if distance > radius_squared {
                    continue;
                }
                radius_squared = distance;
                hit.position = lerp(a, b, t);
            }
            for (link, first_link) in context.tile.links_with_first(context.polygon) {
                let reference = link.reference();
                if reference == 0 || reference == parent {
                    continue;
                }
                let Some(neighbor) = mesh.polygon_from_bits(reference) else {
                    return FAILURE | INVALID_PARAM;
                };
                if neighbor.polygon.kind() == 1 {
                    continue;
                }
                let a = context
                    .tile
                    .vertex_at(context.polygon.vertex(usize::from(link.edge())) as usize);
                let b = context.tile.vertex_at(
                    context
                        .polygon
                        .vertex((usize::from(link.edge()) + 1) % context.polygon.vertices().len())
                        as usize,
                );
                if segment(center, a, b).0 > radius_squared {
                    continue;
                }
                if !filter.pass_filter(neighbor) {
                    continue;
                }
                let Some(index) = self.pool.get_node(reference, 0) else {
                    status |= OUT_OF_NODES;
                    continue;
                };
                let node = self.pool.node_mut(index).unwrap();
                if node.flags() & nodes::CLOSED != 0 {
                    continue;
                }
                if node.flags() == 0
                    && let Some((a, b)) =
                        NavMesh::portal_with_link(context, neighbor, link, first_link)
                {
                    node.position = std::array::from_fn(|i| (a[i] + b[i]) * 0.5);
                }
                let total = best.total + dist(best.position, node.position);
                if node.flags() & nodes::OPEN != 0 && total >= node.total {
                    continue;
                }
                node.set_flags(node.flags() & !nodes::CLOSED);
                node.set_parent_index(current);
                node.total = total;
                if node.flags() & nodes::OPEN != 0 {
                    self.open.modify(index, &self.pool);
                } else {
                    node.set_flags(node.flags() | nodes::OPEN);
                    self.open.push(index, &self.pool).unwrap();
                }
            }
        }
        hit.normal = sub(center, hit.position);
        let n = hit.normal;
        // Native dtSqr calls keep these products separate at -O0.
        let inverse = 1. / (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        for value in &mut hit.normal {
            *value *= inverse;
        }
        hit.distance = radius_squared.sqrt();
        status
    }
}
