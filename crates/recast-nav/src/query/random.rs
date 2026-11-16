// Altered Rust translation of random queries in DetourNavMeshQuery.cpp.
// Copyright (c) 2009-2010 Mikko Mononen. Zlib: vendor/recastnavigation/License.txt.
use super::{NavMeshQuery, polygon_vertices};
use crate::{nav::*, nodes};

#[derive(Clone, Copy, Debug, Default)]
pub struct RandomPoint {
    pub reference: PolyRef,
    pub position: Point,
}
fn polygon_area(context: FilterPolygon<'_>) -> f32 {
    let indices = context.polygon.vertex_words();
    let mut sum = 0.;
    if indices.len() > 2 {
        let records = context.tile.vertex_records();
        let first = crate::nav_data::read_point(&records[u16::from_ne_bytes(indices[0]) as usize]);
        for j in 2..indices.len() {
            sum += area(
                first,
                crate::nav_data::read_point(&records[u16::from_ne_bytes(indices[j - 1]) as usize]),
                crate::nav_data::read_point(&records[u16::from_ne_bytes(indices[j]) as usize]),
            );
        }
    }
    sum
}

fn sample(
    mesh: &NavMesh,
    context: FilterPolygon<'_>,
    random: &mut (impl FnMut() -> f32 + ?Sized),
    result: &mut RandomPoint,
) -> u32 {
    let mut vertices = [[0.; 3]; 6];
    let Some(vertices) = polygon_vertices(context, &mut vertices) else {
        return FAILURE | INVALID_PARAM;
    };
    let s = random();
    let t = random();
    let Some(point) = crate::common::random_point_in_convex_polygon(vertices, &mut [0.; 6], s, t)
    else {
        return FAILURE | INVALID_PARAM;
    };
    // Native ignores closestPointOnPoly's failure. Its sampled local point is
    // already initialized, including for zero/stale refs and non-finite draws.
    result.position = mesh
        .closest_point(context.reference, point)
        .map_or(point, |p| p.0);
    result.reference = context.reference;
    SUCCESS
}
impl NavMeshQuery {
    pub fn find_random_point<F: QueryFilter + ?Sized>(
        &self,
        mesh: &NavMesh,
        filter: &F,
        random: &mut (impl FnMut() -> f32 + ?Sized),
    ) -> (u32, RandomPoint) {
        let mut result = RandomPoint::default();
        let status = self.find_random_point_into(mesh, filter, random, &mut result);
        (status, result)
    }
    pub fn find_random_point_into<F: QueryFilter + ?Sized>(
        &self,
        mesh: &NavMesh,
        filter: &F,
        random: &mut (impl FnMut() -> f32 + ?Sized),
        result: &mut RandomPoint,
    ) -> u32 {
        let mut selected_tile = None;
        let mut sum = 0.;
        for (index, tile) in mesh.tiles.iter().enumerate() {
            if tile.is_none() {
                continue;
            }
            sum += 1.;
            if random() * sum <= 1. {
                selected_tile = Some(index);
            }
        }
        let Some(index) = selected_tile else {
            return FAILURE;
        };
        let tile = mesh.tiles[index].as_ref().unwrap();
        let mut chosen = None;
        let mut sum = 0.;
        for (polygon_index, polygon) in tile.polygons().enumerate() {
            if polygon.kind() != 0 {
                continue;
            }
            let reference = mesh.reference(index, polygon_index);
            let context = FilterPolygon {
                reference,
                tile,
                polygon,
            };
            if !filter.pass_filter(context) {
                continue;
            }
            let area = polygon_area(context);
            sum += area;
            if random() * sum <= area {
                chosen = Some(context);
            }
        }
        let Some(context) = chosen else {
            return FAILURE;
        };
        sample(mesh, context, random, result)
    }
    #[allow(clippy::too_many_arguments)]
    pub fn find_random_point_around_circle<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        center: Point,
        radius: f32,
        filter: &F,
        random: &mut (impl FnMut() -> f32 + ?Sized),
    ) -> (u32, RandomPoint) {
        let mut result = RandomPoint::default();
        let status = self.find_random_point_around_circle_into(
            mesh,
            start,
            center,
            radius,
            filter,
            random,
            &mut result,
        );
        (status, result)
    }
    #[allow(clippy::too_many_arguments)]
    pub fn find_random_point_around_circle_into<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        center: Point,
        radius: f32,
        filter: &F,
        random: &mut (impl FnMut() -> f32 + ?Sized),
        result: &mut RandomPoint,
    ) -> u32 {
        let Some(start_context) = mesh.filter_polygon(start) else {
            return FAILURE | INVALID_PARAM;
        };
        if !center.iter().all(|v| v.is_finite())
            || radius < 0.
            || !radius.is_finite()
            || !filter.pass_filter(start_context)
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
        let radius_squared = radius * radius;
        let mut sum = 0.;
        let mut chosen = None;
        let mut status = SUCCESS;
        while let Some(current) = self.open.pop(&self.pool) {
            let node = self.pool.node_mut(current).unwrap();
            node.set_flags((node.flags() & !nodes::OPEN) | nodes::CLOSED);
            let best = *node;
            let Some(context) = mesh.polygon_from_bits(best.reference) else {
                return FAILURE | INVALID_PARAM;
            };
            if context.polygon.kind() == 0 {
                let area = polygon_area(context);
                sum += area;
                if random() * sum <= area {
                    chosen = Some(context);
                }
            }
            let parent = self
                .pool
                .node(best.parent_index())
                .map_or(0, |p| p.reference);
            if parent != 0 && mesh.polygon_from_bits(parent).is_none() {
                return FAILURE | INVALID_PARAM;
            }
            for (link, first_link) in context.tile.links_with_first(context.polygon) {
                let reference = link.reference();
                if reference == 0 || reference == parent {
                    continue;
                }
                let Some(neighbor) = mesh.polygon_from_bits(reference) else {
                    return FAILURE | INVALID_PARAM;
                };
                if !filter.pass_filter(neighbor) {
                    continue;
                }
                let Some((a, b)) = NavMesh::portal_with_link(context, neighbor, link, first_link)
                else {
                    return FAILURE | INVALID_PARAM;
                };
                if segment(center, a, b).0 > radius_squared {
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
                if node.flags() == 0 {
                    node.position = lerp(a, b, 0.5);
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
                    node.set_flags(nodes::OPEN);
                    self.open.push(index, &self.pool).unwrap();
                }
            }
        }
        let Some(context) = chosen else {
            return FAILURE;
        };
        let sampled = sample(mesh, context, random, result);
        if sampled & FAILURE != 0 {
            return sampled;
        }
        status
    }
}
