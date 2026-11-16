// Altered Rust translation of the persistent state in DetourNavMeshQuery.cpp.
// Copyright (c) 2009-2010 Mikko Mononen. Zlib: vendor/recastnavigation/License.txt.
//! Persistent Detour query storage. Supply the mesh for each operation; this
//! replaces an attached raw pointer and permits tile mutation between operations.
//! Keep the same mesh and filter throughout a sliced search. Operations that
//! alter query storage (including direct pool mutation) invalidate that search.

use crate::{
    nav::*,
    nodes::{self, NodePool, NodeQueue},
    poly_buffer::PolyBuffer,
};

mod walls;
pub use walls::{WallHit, WallSegment, WallSegments};
mod random;
pub use random::RandomPoint;

// Native neighborhood searches use a bounded 48-entry FIFO.
struct TinyQueue {
    items: [u32; 48],
    head: usize,
    len: usize,
}
impl TinyQueue {
    fn new() -> Self {
        Self {
            items: [0; 48],
            head: 0,
            len: 0,
        }
    }
    fn len(&self) -> usize {
        self.len
    }
    fn push_back(&mut self, value: u32) {
        assert!(self.len < self.items.len());
        self.items[(self.head + self.len) % self.items.len()] = value;
        self.len += 1;
    }
    fn pop_front(&mut self) -> Option<u32> {
        if self.len == 0 {
            return None;
        }
        let value = self.items[self.head];
        self.head = (self.head + 1) % self.items.len();
        self.len -= 1;
        Some(value)
    }
}
#[derive(Debug)]
pub struct NavMeshQuery {
    pool: NodePool,
    tiny_pool: NodePool,
    open: NodeQueue,
    sliced: SlicedQuery,
}
#[derive(Clone, Debug, Default)]
pub struct SurfaceMove {
    pub position: Point,
    pub visited: Vec<PolyRef>,
}
#[derive(Clone, Copy, Debug)]
pub struct DijkstraEntry {
    pub reference: PolyRef,
    pub parent: PolyRef,
    pub cost: f32,
}
#[derive(Clone, Debug, Default)]
pub struct DijkstraSearch {
    pub status: u32,
    pub entries: Vec<DijkstraEntry>,
}
#[derive(Clone, Copy, Debug)]
pub struct NeighborhoodEntry {
    pub reference: PolyRef,
    pub parent: PolyRef,
}
#[derive(Clone, Debug, Default)]
pub struct Neighborhood {
    pub status: u32,
    pub entries: Vec<NeighborhoodEntry>,
}
// Boundary queries need only references; public owned results also retain parents.
trait NeighborhoodStorage {
    fn clear(&mut self);
    fn len(&self) -> usize;
    fn reference(&self, index: usize) -> PolyRef;
    fn push(&mut self, entry: NeighborhoodEntry);
}
impl NeighborhoodStorage for Vec<NeighborhoodEntry> {
    #[inline]
    fn clear(&mut self) {
        Vec::clear(self);
    }
    #[inline]
    fn len(&self) -> usize {
        Vec::len(self)
    }
    #[inline]
    fn reference(&self, index: usize) -> PolyRef {
        self[index].reference
    }
    #[inline]
    fn push(&mut self, entry: NeighborhoodEntry) {
        Vec::push(self, entry);
    }
}
struct NeighborhoodRefs<'a> {
    values: &'a mut [PolyRef],
    count: &'a mut u8,
}
impl NeighborhoodStorage for NeighborhoodRefs<'_> {
    #[inline]
    fn clear(&mut self) {
        *self.count = 0;
    }
    #[inline]
    fn len(&self) -> usize {
        usize::from(*self.count)
    }
    #[inline]
    fn reference(&self, index: usize) -> PolyRef {
        self.values[index]
    }
    #[inline]
    fn push(&mut self, entry: NeighborhoodEntry) {
        self.values[usize::from(*self.count)] = entry.reference;
        *self.count += 1;
    }
}
#[derive(Clone, Copy, Debug, Default)]
struct SlicedQuery {
    status: u32,
    start: PolyRef,
    end: PolyRef,
    end_position: Point,
    options: u32,
    raycast_limit_squared: f32,
    last_best: u32,
    last_best_cost: f32,
}
impl NavMeshQuery {
    pub fn find_local_neighborhood<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        center: Point,
        radius: f32,
        filter: &F,
        capacity: usize,
    ) -> Neighborhood {
        let mut result = Neighborhood::default();
        self.find_local_neighborhood_into(
            mesh,
            start,
            center,
            radius,
            filter,
            capacity,
            &mut result,
        );
        result
    }
    /// Reuses caller-owned neighborhood entries, retaining allocation capacity.
    #[allow(clippy::too_many_arguments)]
    pub fn find_local_neighborhood_into<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        center: Point,
        radius: f32,
        filter: &F,
        capacity: usize,
        result: &mut Neighborhood,
    ) -> u32 {
        self.find_local_neighborhood_with_storage(
            mesh,
            start,
            center,
            radius,
            filter,
            capacity,
            &mut result.status,
            &mut result.entries,
        )
    }
    /// Native boundary output omits parents. The private 64-node pool bounds
    /// the count even when the supplied reference slice is larger.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn find_local_neighborhood_refs_into<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        center: Point,
        radius: f32,
        filter: &F,
        references: &mut [PolyRef],
        count: &mut u8,
    ) -> u32 {
        let capacity = references.len();
        let mut status = SUCCESS;
        self.find_local_neighborhood_with_storage(
            mesh,
            start,
            center,
            radius,
            filter,
            capacity,
            &mut status,
            &mut NeighborhoodRefs {
                values: references,
                count,
            },
        )
    }
    #[allow(clippy::too_many_arguments)]
    fn find_local_neighborhood_with_storage<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        center: Point,
        radius: f32,
        filter: &F,
        capacity: usize,
        status: &mut u32,
        result: &mut impl NeighborhoodStorage,
    ) -> u32 {
        *status = SUCCESS;
        result.clear();
        if mesh.resolve(start).is_none()
            || !center.iter().all(|v| v.is_finite())
            || radius < 0.
            || !radius.is_finite()
            || capacity > i32::MAX as usize
        {
            *status = FAILURE | INVALID_PARAM;
            return *status;
        }
        self.tiny_pool.clear();
        let first = self.tiny_pool.get_node(start, 0).unwrap();
        self.tiny_pool
            .node_mut(first)
            .unwrap()
            .set_flags(nodes::CLOSED);
        let mut queue = TinyQueue::new();
        queue.push_back(first);
        let radius_squared = radius * radius;
        if capacity > 0 {
            result.push(NeighborhoodEntry {
                reference: start,
                parent: 0,
            });
        } else {
            *status |= BUFFER_TOO_SMALL;
        }
        // The mesh is immutable for the query. Reuse the first sixteen stored
        // polygons' vertices across overlap tests; larger outputs fall back to
        // the same extraction. Fill lazily at the original validation point.
        let mut past_vertices = [None; 16];
        while let Some(current) = queue.pop_front() {
            let reference = self.tiny_pool.node(current).unwrap().reference;
            let Some(context) = mesh.polygon_from_bits(reference) else {
                *status = FAILURE | INVALID_PARAM;
                return *status;
            };
            let current_links = context.tile.links(context.polygon);
            let connected_bits = current_links
                .clone()
                .fold(0, |bits, link| bits | link_reference_bit(link.reference()));
            for (link, first_link) in context.tile.links_with_first(context.polygon) {
                if link.reference() == 0 {
                    continue;
                }
                let Some(index) = self.tiny_pool.get_node(link.reference(), 0) else {
                    continue;
                };
                if self.tiny_pool.node(index).unwrap().flags() & nodes::CLOSED != 0 {
                    continue;
                }
                let Some(neighbor) = mesh.polygon_from_bits(link.reference()) else {
                    *status = FAILURE | INVALID_PARAM;
                    return *status;
                };
                if neighbor.polygon.kind() == 1 || !filter.pass_filter(neighbor) {
                    continue;
                }
                let Some((a, b)) = NavMesh::portal_with_link(context, neighbor, link, first_link)
                else {
                    *status = FAILURE | INVALID_PARAM;
                    return *status;
                };
                if segment(center, a, b).0 > radius_squared {
                    continue;
                }
                let node = self.tiny_pool.node_mut(index).unwrap();
                node.set_flags(node.flags() | nodes::CLOSED);
                node.set_parent_index(current);
                let mut a = [[0.; 3]; 6];
                let Some(a) = polygon_vertices(neighbor, &mut a) else {
                    *status = FAILURE | INVALID_PARAM;
                    return *status;
                };
                let mut projections = crate::common::PolygonOverlap::new(a);
                let mut overlap = false;
                for j in 0..result.len() {
                    let past = result.reference(j);
                    // A missing bit proves non-membership. Collisions retain
                    // the exact native reference comparison and link order.
                    if connected_bits & link_reference_bit(past) != 0
                        && current_links.clone().any(|l| l.reference() == past)
                    {
                        continue;
                    }
                    let mut uncached = None;
                    let cached = past_vertices.get_mut(j).unwrap_or(&mut uncached);
                    if cached.is_none() {
                        let Some(past) = mesh.polygon_from_bits(past) else {
                            *status = FAILURE | INVALID_PARAM;
                            return *status;
                        };
                        let mut vertices = [[0.; 3]; 6];
                        let Some(points) = polygon_vertices(past, &mut vertices) else {
                            *status = FAILURE | INVALID_PARAM;
                            return *status;
                        };
                        let count = points.len() as u8;
                        *cached = Some((vertices, count));
                    }
                    let (vertices, count) = cached.as_ref().unwrap();
                    let Some(hit) = projections.overlaps(&vertices[..usize::from(*count)]) else {
                        *status = FAILURE | INVALID_PARAM;
                        return *status;
                    };
                    if hit {
                        overlap = true;
                        break;
                    }
                }
                if overlap {
                    continue;
                }
                if result.len() < capacity {
                    result.push(NeighborhoodEntry {
                        reference: link.reference(),
                        parent: reference,
                    });
                } else {
                    *status |= BUFFER_TOO_SMALL;
                }
                if queue.len() < 48 {
                    queue.push_back(index);
                }
            }
        }
        *status
    }
    pub fn find_polygons_around_circle<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        center: Point,
        radius: f32,
        filter: &F,
        capacity: usize,
    ) -> DijkstraSearch {
        let mut result = DijkstraSearch::default();
        self.find_polygons_around_circle_into(
            mesh,
            start,
            center,
            radius,
            filter,
            capacity,
            &mut result,
        );
        result
    }
    /// Reuses caller-owned Dijkstra entries, retaining their allocation capacity.
    #[allow(clippy::too_many_arguments)]
    pub fn find_polygons_around_circle_into<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        center: Point,
        radius: f32,
        filter: &F,
        capacity: usize,
        result: &mut DijkstraSearch,
    ) -> u32 {
        if !center.iter().all(|v| v.is_finite()) || radius < 0. || !radius.is_finite() {
            result.entries.clear();
            result.status = FAILURE | INVALID_PARAM;
            return result.status;
        }
        let radius_squared = radius * radius;
        self.find_polygons_around(
            mesh,
            start,
            center,
            filter,
            capacity,
            |a, b| {
                segment(center, a, b).0.partial_cmp(&radius_squared)
                    != Some(std::cmp::Ordering::Greater)
            },
            result,
        )
    }
    pub fn find_polygons_around_shape<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        vertices: &[Point],
        filter: &F,
        capacity: usize,
    ) -> DijkstraSearch {
        let mut result = DijkstraSearch::default();
        self.find_polygons_around_shape_into(mesh, start, vertices, filter, capacity, &mut result);
        result
    }
    /// Reuses caller-owned Dijkstra entries, preserving native cost/traversal order.
    #[allow(clippy::too_many_arguments)]
    pub fn find_polygons_around_shape_into<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        vertices: &[Point],
        filter: &F,
        capacity: usize,
        result: &mut DijkstraSearch,
    ) -> u32 {
        if vertices.len() < 3 || vertices.len() > i32::MAX as usize {
            result.entries.clear();
            result.status = FAILURE | INVALID_PARAM;
            return result.status;
        }
        let mut center = [0.; 3];
        for vertex in vertices {
            for i in 0..3 {
                center[i] += vertex[i];
            }
        }
        let scale = 1. / vertices.len() as f32;
        for value in &mut center {
            *value *= scale;
        }
        self.find_polygons_around(
            mesh,
            start,
            center,
            filter,
            capacity,
            |a, b| {
                let hit = crate::common::intersect_segment_polygon_2d(a, b, vertices);
                hit.intersects && !(hit.t_min > 1. || hit.t_max < 0.)
            },
            result,
        )
    }
    #[allow(clippy::too_many_arguments)]
    fn find_polygons_around<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        center: Point,
        filter: &F,
        capacity: usize,
        overlaps: impl Fn(Point, Point) -> bool,
        result: &mut DijkstraSearch,
    ) -> u32 {
        result.status = SUCCESS;
        result.entries.clear();
        if mesh.resolve(start).is_none() || capacity > i32::MAX as usize {
            result.status = FAILURE | INVALID_PARAM;
            return result.status;
        }
        self.pool.clear();
        self.open.clear();
        let Some(first) = self.pool.get_node(start, 0) else {
            result.status = FAILURE | INVALID_PARAM;
            return result.status;
        };
        let node = self.pool.node_mut(first).unwrap();
        node.position = center;
        node.set_flags(nodes::OPEN);
        self.open.push(first, &self.pool).unwrap();
        while let Some(current) = self.open.pop(&self.pool) {
            let node = self.pool.node_mut(current).unwrap();
            node.set_flags((node.flags() & !nodes::OPEN) | nodes::CLOSED);
            let best = *node;
            let Some(context) = mesh.polygon_from_bits(best.reference) else {
                result.status = FAILURE | INVALID_PARAM;
                return result.status;
            };
            let parent = self
                .pool
                .node(best.parent_index())
                .map_or(0, |p| p.reference);
            let parent_context = if parent == 0 {
                None
            } else {
                let Some(context) = mesh.polygon_from_bits(parent) else {
                    result.status = FAILURE | INVALID_PARAM;
                    return result.status;
                };
                Some(context)
            };
            if result.entries.len() < capacity {
                result.entries.push(DijkstraEntry {
                    reference: best.reference,
                    parent,
                    cost: best.total,
                });
            } else {
                result.status |= BUFFER_TOO_SMALL;
            }
            for (link, first_link) in context.tile.links_with_first(context.polygon) {
                let reference = link.reference();
                if reference == 0 || reference == parent {
                    continue;
                }
                let Some(neighbor) = mesh.polygon_from_bits(reference) else {
                    result.status = FAILURE | INVALID_PARAM;
                    return result.status;
                };
                if !filter.pass_filter(neighbor) {
                    continue;
                }
                let portal = NavMesh::portal_points(context, neighbor, link, first_link);
                let Some((a, b)) = portal else {
                    // Native proceeds with uninitialized local portal points
                    // after this failure. Reject that undefined memory domain.
                    result.status = FAILURE | INVALID_PARAM;
                    return result.status;
                };
                if !overlaps(a, b) {
                    continue;
                }
                let Some(index) = self.pool.get_node(reference, 0) else {
                    result.status |= OUT_OF_NODES;
                    continue;
                };
                let node = self.pool.node_mut(index).unwrap();
                if node.flags() & nodes::CLOSED != 0 {
                    continue;
                }
                if node.flags() == 0 {
                    node.position = lerp(a, b, 0.5);
                }
                let total = best.total
                    + filter.cost(
                        best.position,
                        node.position,
                        parent_context,
                        context,
                        Some(neighbor),
                    );
                if node.flags() & nodes::OPEN != 0 && total >= node.total {
                    continue;
                }
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
        result.status
    }
    pub fn path_from_dijkstra_search(
        &self,
        mesh: &NavMesh,
        end: PolyRef,
        capacity: usize,
    ) -> Corridor {
        let mut result = Corridor::default();
        self.path_from_dijkstra_search_into(mesh, end, capacity, &mut result);
        result
    }
    /// Reuses caller-owned corridor storage for a completed Dijkstra search.
    /// An invalid reference or output budget retains the previous path; a valid
    /// reference without a closed search node clears it, matching native order.
    pub fn path_from_dijkstra_search_into(
        &self,
        mesh: &NavMesh,
        end: PolyRef,
        capacity: usize,
        result: &mut Corridor,
    ) -> u32 {
        result.status = FAILURE | INVALID_PARAM;
        if mesh.resolve(end).is_none() || capacity > i32::MAX as usize {
            return result.status;
        }
        result.polygons.clear();
        let Some(end_node) = self.pool.matching_nodes(end).next() else {
            return result.status;
        };
        if self.pool.node(end_node).unwrap().flags() & nodes::CLOSED == 0 {
            return result.status;
        }
        // Native counts the complete chain, skips the truncated tail, then
        // fills only the advertised prefix backwards. Retain that bounded work
        // buffer rather than growing a full reverse path and truncating it.
        let mut length = 0;
        let mut at = end_node;
        while let Some(node) = self.pool.node(at) {
            length += 1;
            at = node.parent_index();
        }
        let write_count = length.min(capacity);
        at = end_node;
        for _ in write_count..length {
            at = self.pool.node(at).unwrap().parent_index();
        }
        result.polygons.resize(write_count, 0);
        for reference in result.polygons.iter_mut().rev() {
            let node = self.pool.node(at).unwrap();
            *reference = node.reference;
            at = node.parent_index();
        }
        let status = SUCCESS
            | if length > capacity {
                BUFFER_TOO_SMALL
            } else {
                0
            };
        result.status = status;
        result.status
    }
    #[allow(clippy::too_many_arguments)]
    pub fn move_along_surface<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        start_position: Point,
        end_position: Point,
        filter: &F,
        capacity: usize,
    ) -> (u32, SurfaceMove) {
        let mut result = SurfaceMove::default();
        let status = self.move_along_surface_into(
            mesh,
            start,
            start_position,
            end_position,
            filter,
            capacity,
            &mut result,
        );
        (status, result)
    }
    /// Searches nearby polygons in XZ. A reached endpoint retains its requested
    /// Y; a wall result interpolates its edge height. On invalid input only the
    /// visited list is cleared, preserving the supplied position.
    #[allow(clippy::too_many_arguments)]
    pub fn move_along_surface_into<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        start_position: Point,
        end_position: Point,
        filter: &F,
        capacity: usize,
        result: &mut SurfaceMove,
    ) -> u32 {
        self.move_along_surface_to_buffer(
            mesh,
            start,
            start_position,
            end_position,
            filter,
            capacity,
            &mut result.position,
            &mut result.visited,
        )
    }
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn move_along_surface_to_buffer<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        start_position: Point,
        end_position: Point,
        filter: &F,
        capacity: usize,
        position: &mut Point,
        visited: &mut impl PolyBuffer,
    ) -> u32 {
        visited.clear();
        if mesh.resolve(start).is_none()
            || !start_position
                .iter()
                .chain(&end_position)
                .all(|v| v.is_finite())
            || capacity == 0
            || capacity > i32::MAX as usize
        {
            return FAILURE | INVALID_PARAM;
        }
        self.tiny_pool.clear();
        let first = self.tiny_pool.get_node(start, 0).unwrap();
        self.tiny_pool
            .node_mut(first)
            .unwrap()
            .set_flags(nodes::CLOSED);
        let mut queue = TinyQueue::new();
        queue.push_back(first);
        let mut best_position = start_position;
        let mut best_distance = f32::MAX;
        let mut best_node = 0;
        let search_position = lerp(start_position, end_position, 0.5);
        let radius = dist(start_position, end_position) / 2. + 0.001;
        let search_radius_squared = radius * radius;
        while let Some(current) = queue.pop_front() {
            let reference = self.tiny_pool.node(current).unwrap().reference;
            let Some(context) = mesh.polygon_from_bits(reference) else {
                return FAILURE | INVALID_PARAM;
            };
            let mut vertices = [[0.; 3]; 6];
            let Some(vertices) = polygon_vertices(context, &mut vertices) else {
                return FAILURE | INVALID_PARAM;
            };
            if inside(end_position, vertices) {
                best_node = current;
                best_position = end_position;
                break;
            }
            for i in 0..vertices.len() {
                let j = if i == 0 { vertices.len() - 1 } else { i - 1 };
                let mut neighbors = [0; 8];
                let mut neighbor_count = 0;
                let edge = context.polygon.neighbor(j);
                if edge & 0x8000 != 0 {
                    for link in context.tile.links(context.polygon) {
                        if usize::from(link.edge()) != j || link.reference() == 0 {
                            continue;
                        }
                        let Some(neighbor) = mesh.polygon_from_bits(link.reference()) else {
                            return FAILURE | INVALID_PARAM;
                        };
                        if filter.pass_filter(neighbor) && neighbor_count < neighbors.len() {
                            neighbors[neighbor_count] = link.reference();
                            neighbor_count += 1;
                        }
                    }
                } else if edge != 0 {
                    let (_, tile, _) = mesh.reference_layout().decode(reference);
                    let reference = mesh.reference(tile as usize, usize::from(edge - 1));
                    let Some(neighbor) = mesh.polygon_from_bits(reference) else {
                        return FAILURE | INVALID_PARAM;
                    };
                    if filter.pass_filter(neighbor) {
                        neighbors[0] = reference;
                        neighbor_count = 1;
                    }
                }
                if neighbor_count == 0 {
                    let (distance, t) = segment(end_position, vertices[j], vertices[i]);
                    if distance < best_distance {
                        best_position = lerp(vertices[j], vertices[i], t);
                        best_distance = distance;
                        best_node = current;
                    }
                } else {
                    for &reference in &neighbors[..neighbor_count] {
                        let Some(index) = self.tiny_pool.get_node(reference, 0) else {
                            continue;
                        };
                        if self.tiny_pool.node(index).unwrap().flags() & nodes::CLOSED != 0 {
                            continue;
                        }
                        if segment(search_position, vertices[j], vertices[i]).0
                            > search_radius_squared
                        {
                            continue;
                        }
                        if queue.len() < 48 {
                            let node = self.tiny_pool.node_mut(index).unwrap();
                            node.set_parent_index(current);
                            node.set_flags(node.flags() | nodes::CLOSED);
                            queue.push_back(index);
                        }
                    }
                }
            }
        }
        let mut previous = 0;
        let mut at = best_node;
        while at != 0 {
            let node = self.tiny_pool.node_mut(at).unwrap();
            let next = node.parent_index();
            node.set_parent_index(previous);
            previous = at;
            at = next;
        }
        let mut status = SUCCESS;
        at = previous;
        while at != 0 {
            let node = self.tiny_pool.node(at).unwrap();
            visited.push(node.reference);
            if visited.len() >= capacity {
                status |= BUFFER_TOO_SMALL;
                break;
            }
            at = node.parent_index();
        }
        *position = best_position;
        status
    }
    pub fn straight_path(
        &self,
        mesh: &NavMesh,
        start: Point,
        end: Point,
        path: &[PolyRef],
        capacity: usize,
        options: u32,
    ) -> StraightPath {
        mesh.straight_path_with_options(start, end, path, capacity, options)
    }
    /// Reuses the three straight-path result buffers.
    #[allow(clippy::too_many_arguments)]
    pub fn straight_path_into(
        &self,
        mesh: &NavMesh,
        start: Point,
        end: Point,
        path: &[PolyRef],
        capacity: usize,
        options: u32,
        output: &mut StraightPath,
    ) -> u32 {
        mesh.straight_path_into(start, end, path, capacity, options, output)
    }
    #[allow(clippy::too_many_arguments)]
    pub fn raycast_into<F: QueryFilter + ?Sized>(
        &self,
        mesh: &NavMesh,
        start: PolyRef,
        start_pos: Point,
        end_pos: Point,
        filter: &F,
        options: u32,
        previous: PolyRef,
        capacity: usize,
        hit: &mut RaycastHit,
    ) -> u32 {
        mesh.raycast_into(
            start, start_pos, end_pos, filter, options, previous, capacity, hit,
        )
    }
    #[allow(clippy::too_many_arguments)]
    pub fn raycast_with_options<F: QueryFilter + ?Sized>(
        &self,
        mesh: &NavMesh,
        start: PolyRef,
        start_pos: Point,
        end_pos: Point,
        filter: &F,
        options: u32,
        previous: PolyRef,
        capacity: usize,
    ) -> (u32, RaycastHit) {
        mesh.raycast_with_options(
            start, start_pos, end_pos, filter, options, previous, capacity,
        )
    }
    pub fn raycast<F: QueryFilter + ?Sized>(
        &self,
        mesh: &NavMesh,
        start: PolyRef,
        start_pos: Point,
        end_pos: Point,
        filter: &F,
        capacity: usize,
    ) -> (u32, RaycastHit) {
        mesh.raycast(start, start_pos, end_pos, filter, capacity)
    }
    /// Native positive budgets below four create a zero-sized main hash table.
    /// Initialization and independent/tiny-pool operations remain supported;
    /// main hash access returns a checked failure instead of native invalid access.
    pub fn new(max_nodes: usize) -> Result<Self, u32> {
        if !(1..=65535).contains(&max_nodes) {
            return Err(FAILURE | INVALID_PARAM);
        }
        Ok(Self {
            pool: NodePool::new(
                max_nodes,
                crate::common::next_power_of_two((max_nodes / 4) as u32) as usize,
            )?,
            tiny_pool: NodePool::new(64, 32)?,
            open: NodeQueue::new(max_nodes)?,
            sliced: SlicedQuery::default(),
        })
    }
    /// Grows storage when necessary, otherwise clears and retains the larger
    /// native capacity/hash size. A zero request can reuse existing storage.
    pub fn reinitialize(&mut self, max_nodes: usize) -> u32 {
        if max_nodes > 65535 {
            return FAILURE | INVALID_PARAM;
        }
        if self.pool.max_nodes() < max_nodes {
            let pool = match NodePool::new(
                max_nodes,
                crate::common::next_power_of_two((max_nodes / 4) as u32) as usize,
            ) {
                Ok(pool) => pool,
                Err(s) => return s,
            };
            let open = match NodeQueue::new(max_nodes) {
                Ok(open) => open,
                Err(s) => return s,
            };
            self.pool = pool;
            self.open = open;
        } else {
            self.pool.clear();
            self.open.clear();
        }
        self.tiny_pool.clear();
        SUCCESS
    }
    pub fn node_pool(&self) -> &NodePool {
        &self.pool
    }
    pub fn node_pool_mut(&mut self) -> &mut NodePool {
        &mut self.pool
    }
    #[allow(clippy::too_many_arguments)]
    pub fn init_sliced_find_path<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        end: PolyRef,
        start_pos: Point,
        end_pos: Point,
        filter: &F,
        options: u32,
    ) -> u32 {
        self.init_sliced_find_path_optional(
            mesh,
            start,
            end,
            start_pos,
            end_pos,
            Some(filter),
            options,
        )
    }
    /// The optional form preserves native null-filter validation and retained
    /// search storage. A missing filter returns FAILURE | INVALID_PARAM.
    #[allow(clippy::too_many_arguments)]
    pub fn init_sliced_find_path_optional<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        end: PolyRef,
        start_pos: Point,
        end_pos: Point,
        filter: Option<&F>,
        options: u32,
    ) -> u32 {
        self.sliced = SlicedQuery {
            status: FAILURE,
            start,
            end,
            end_position: end_pos,
            options,
            raycast_limit_squared: f32::MAX,
            ..SlicedQuery::default()
        };
        if mesh.resolve(start).is_none()
            || mesh.resolve(end).is_none()
            || !start_pos.iter().chain(&end_pos).all(|v| v.is_finite())
            || filter.is_none()
        {
            return FAILURE | INVALID_PARAM;
        }
        if options & FINDPATH_ANY_ANGLE != 0 {
            let limit = mesh
                .filter_polygon(start)
                .unwrap()
                .tile
                .header
                .walkable_radius
                * RAY_CAST_LIMIT_PROPORTIONS;
            self.sliced.raycast_limit_squared = limit * limit;
        }
        if start == end {
            self.sliced.status = SUCCESS;
            return SUCCESS;
        }
        self.pool.clear();
        self.open.clear();
        let Some(first) = self.pool.get_node(start, 0) else {
            return FAILURE | INVALID_PARAM;
        };
        let node = self.pool.node_mut(first).unwrap();
        node.position = start_pos;
        node.total = dist(start_pos, end_pos) * 0.999;
        node.set_flags(nodes::OPEN);
        self.sliced.last_best = first;
        self.sliced.last_best_cost = node.total;
        self.open.push(first, &self.pool).unwrap();
        self.sliced.status = IN_PROGRESS;
        IN_PROGRESS
    }
    pub fn update_sliced_find_path<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        filter: &F,
        max_iterations: i32,
        done_iterations: Option<&mut i32>,
    ) -> u32 {
        if self.sliced.status & IN_PROGRESS == 0 {
            return self.sliced.status;
        }
        if mesh.resolve(self.sliced.start).is_none() || mesh.resolve(self.sliced.end).is_none() {
            self.sliced.status = FAILURE;
            return FAILURE;
        }
        let mut iterations = 0;
        while iterations < max_iterations && !self.open.is_empty() {
            iterations += 1;
            let current = self.open.pop(&self.pool).unwrap();
            let node = self.pool.node_mut(current).unwrap();
            node.set_flags((node.flags() & !nodes::OPEN) | nodes::CLOSED);
            let best = *node;
            if best.reference == self.sliced.end {
                self.sliced.last_best = current;
                self.sliced.status = SUCCESS | (self.sliced.status & STATUS_DETAIL_MASK);
                if let Some(done) = done_iterations {
                    *done = iterations;
                }
                return self.sliced.status;
            }
            let Some(context) = mesh.filter_polygon(best.reference) else {
                self.sliced.status = FAILURE;
                if let Some(done) = done_iterations {
                    *done = iterations;
                }
                return FAILURE;
            };
            let parent_node = self.pool.node(best.parent_index()).copied();
            let parent = parent_node.map_or(0, |p| p.reference);
            let grandparent = parent_node
                .and_then(|p| self.pool.node(p.parent_index()))
                .map_or(0, |p| p.reference);
            let parent_context = if parent == 0 {
                None
            } else {
                mesh.filter_polygon(parent)
            };
            if parent != 0
                && (parent_context.is_none()
                    || (grandparent != 0 && mesh.resolve(grandparent).is_none()))
            {
                self.sliced.status = FAILURE;
                if let Some(done) = done_iterations {
                    *done = iterations;
                }
                return FAILURE;
            }
            let try_line_of_sight = self.sliced.options & FINDPATH_ANY_ANGLE != 0
                && parent != 0
                && length_squared(sub(parent_node.unwrap().position, best.position))
                    < self.sliced.raycast_limit_squared;
            for (link, first_link) in context.tile.links_with_first(context.polygon) {
                let reference = link.reference();
                if reference == 0 || reference == parent {
                    continue;
                }
                let Some(neighbor) = mesh.polygon_from_bits(reference) else {
                    self.sliced.status = FAILURE;
                    if let Some(done) = done_iterations {
                        *done = iterations;
                    }
                    return FAILURE;
                };
                if !filter.pass_filter(neighbor) {
                    continue;
                }
                let Some(index) = self.pool.get_node(reference, 0) else {
                    self.sliced.status |= OUT_OF_NODES;
                    continue;
                };
                let n = *self.pool.node(index).unwrap();
                if n.parent_index() != 0 && n.parent_index() == best.parent_index() {
                    continue;
                }
                // Native ignores a failed midpoint lookup and retains the
                // position storage, which can be initialized by an old query.
                if n.flags() == 0
                    && let Some((a, b)) =
                        NavMesh::portal_with_link(context, neighbor, link, first_link)
                {
                    self.pool.node_mut(index).unwrap().position =
                        std::array::from_fn(|i| (a[i] + b[i]) * 0.5);
                }
                let n = *self.pool.node(index).unwrap();
                let mut shortcut = false;
                let mut ray_cost = 0.;
                if try_line_of_sight {
                    let (_, hit) = mesh.raycast_with_options(
                        parent,
                        parent_node.unwrap().position,
                        n.position,
                        filter,
                        RAYCAST_USE_COSTS,
                        grandparent,
                        0,
                    );
                    shortcut = hit.t >= 1.;
                    ray_cost = hit.path_cost;
                }
                let mut cost = if shortcut {
                    parent_node.unwrap().cost + ray_cost
                } else {
                    best.cost
                        + filter.cost(
                            best.position,
                            n.position,
                            parent_context,
                            context,
                            Some(neighbor),
                        )
                };
                let heuristic = if reference == self.sliced.end {
                    cost += filter.cost(
                        n.position,
                        self.sliced.end_position,
                        Some(context),
                        neighbor,
                        None,
                    );
                    0.
                } else {
                    dist(n.position, self.sliced.end_position) * 0.999
                };
                let total = cost + heuristic;
                if n.flags() & (nodes::OPEN | nodes::CLOSED) != 0 && total >= n.total {
                    continue;
                }
                let node = self.pool.node_mut(index).unwrap();
                node.set_parent_index(if shortcut {
                    best.parent_index()
                } else {
                    current
                });
                node.set_flags(node.flags() & !(nodes::CLOSED | nodes::PARENT_DETACHED));
                node.cost = cost;
                node.total = total;
                if shortcut {
                    node.set_flags(node.flags() | nodes::PARENT_DETACHED);
                }
                if node.flags() & nodes::OPEN != 0 {
                    self.open.modify(index, &self.pool);
                } else {
                    node.set_flags(node.flags() | nodes::OPEN);
                    self.open.push(index, &self.pool).unwrap();
                }
                if heuristic < self.sliced.last_best_cost {
                    self.sliced.last_best_cost = heuristic;
                    self.sliced.last_best = index;
                }
            }
        }
        if self.open.is_empty() {
            self.sliced.status = SUCCESS | (self.sliced.status & STATUS_DETAIL_MASK);
        }
        if let Some(done) = done_iterations {
            *done = iterations;
        }
        self.sliced.status
    }
    pub fn finalize_sliced_find_path<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        filter: &F,
        capacity: usize,
    ) -> Corridor {
        let mut result = Corridor::default();
        self.finalize_sliced_find_path_into(mesh, filter, capacity, &mut result);
        result
    }
    /// Reuses caller-owned path storage. Invalid output budgets retain sliced state.
    pub fn finalize_sliced_find_path_into<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        filter: &F,
        capacity: usize,
        result: &mut Corridor,
    ) -> u32 {
        result.status = self.finalize_sliced(mesh, filter, None, capacity, &mut result.polygons);
        result.status
    }
    pub fn finalize_sliced_find_path_partial<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        existing: &[PolyRef],
        filter: &F,
        capacity: usize,
    ) -> Corridor {
        let mut result = Corridor::default();
        self.finalize_sliced_find_path_partial_into(mesh, existing, filter, capacity, &mut result);
        result
    }
    /// Reuses caller-owned path storage, preserving partial-finalization precedence.
    pub fn finalize_sliced_find_path_partial_into<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        existing: &[PolyRef],
        filter: &F,
        capacity: usize,
        result: &mut Corridor,
    ) -> u32 {
        result.status = self.finalize_sliced_partial_to_buffer(
            mesh,
            existing,
            filter,
            capacity,
            &mut result.polygons,
        );
        result.status
    }
    pub(crate) fn finalize_sliced_partial_to_buffer<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        existing: &[PolyRef],
        filter: &F,
        capacity: usize,
        path: &mut impl PolyBuffer,
    ) -> u32 {
        if existing.is_empty() || existing.len() > i32::MAX as usize {
            path.clear();
            return FAILURE | INVALID_PARAM;
        }
        self.finalize_sliced(mesh, filter, Some(existing), capacity, path)
    }
    fn finalize_sliced<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        filter: &F,
        existing: Option<&[PolyRef]>,
        capacity: usize,
        path: &mut impl PolyBuffer,
    ) -> u32 {
        path.clear();
        if capacity == 0 || capacity > i32::MAX as usize {
            return FAILURE | INVALID_PARAM;
        }
        if self.sliced.status & FAILURE != 0 {
            self.sliced = SlicedQuery::default();
            return FAILURE;
        }
        if self.sliced.start == self.sliced.end {
            path.push(self.sliced.start);
        } else {
            let mut at = self.sliced.last_best;
            if let Some(existing) = existing {
                if let Some(found) = existing
                    .iter()
                    .rev()
                    .find_map(|&r| self.pool.matching_nodes(r).next())
                {
                    at = found;
                } else {
                    self.sliced.status |= PARTIAL_RESULT;
                }
            } else if self.pool.node(at).unwrap().reference != self.sliced.end {
                self.sliced.status |= PARTIAL_RESULT;
            }
            let mut previous = 0;
            let mut previous_ray = 0;
            while at != 0 {
                let node = self.pool.node_mut(at).unwrap();
                let next = node.parent_index();
                node.set_parent_index(previous);
                previous = at;
                let next_ray = node.flags() & nodes::PARENT_DETACHED;
                node.set_flags((node.flags() & !nodes::PARENT_DETACHED) | previous_ray);
                previous_ray = next_ray;
                at = next;
            }
            at = previous;
            while at != 0 {
                let node = *self.pool.node(at).unwrap();
                let next = node.parent_index();
                let mut status = 0;
                if node.flags() & nodes::PARENT_DETACHED != 0 {
                    let target = *self.pool.node(next).unwrap();
                    let remaining = capacity - path.len();
                    let mut hit = RaycastHit::default();
                    let s = mesh.raycast_append_to_buffer(
                        node.reference,
                        node.position,
                        target.position,
                        filter,
                        0,
                        0,
                        remaining,
                        hit.split_output().0,
                        path,
                    );
                    if path.as_slice().last() == Some(&target.reference) {
                        path.pop();
                    }
                    status = s;
                } else {
                    // Native may append one final node after a detached ray
                    // exactly filled the advertised budget. Owned output can grow;
                    // bounded reconstruction supplies the corresponding spare slot.
                    path.push(node.reference);
                    if path.len() >= capacity {
                        status = BUFFER_TOO_SMALL;
                    }
                }
                if status & STATUS_DETAIL_MASK != 0 {
                    self.sliced.status |= status & STATUS_DETAIL_MASK;
                    break;
                }
                at = next;
            }
        }
        let status = SUCCESS | (self.sliced.status & STATUS_DETAIL_MASK);
        self.sliced = SlicedQuery::default();
        status
    }
    pub fn is_in_closed_list(&self, reference: PolyRef) -> bool {
        self.pool
            .matching_nodes(reference)
            .take(nodes::MAX_STATES)
            .any(|i| self.pool.node(i).unwrap().flags() & nodes::CLOSED != 0)
    }
    pub fn is_valid_poly_ref<F: QueryFilter + ?Sized>(
        &self,
        mesh: &NavMesh,
        reference: PolyRef,
        filter: &F,
    ) -> bool {
        mesh.filter_polygon(reference)
            .is_some_and(|p| filter.pass_filter(p))
    }
    #[allow(clippy::too_many_arguments)]
    pub fn find_path<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        end: PolyRef,
        start_pos: Point,
        end_pos: Point,
        filter: &F,
        max_path: usize,
    ) -> Corridor {
        mesh.find_path_with_nodes(
            start,
            end,
            start_pos,
            end_pos,
            filter,
            max_path,
            &mut self.pool,
            &mut self.open,
        )
    }
    /// Reuses caller-owned corridor storage; invalid input clears logical output.
    #[allow(clippy::too_many_arguments)]
    pub fn find_path_into<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        start: PolyRef,
        end: PolyRef,
        start_pos: Point,
        end_pos: Point,
        filter: &F,
        max_path: usize,
        output: &mut Corridor,
    ) -> u32 {
        mesh.find_path_with_nodes_into(
            start,
            end,
            start_pos,
            end_pos,
            filter,
            max_path,
            &mut self.pool,
            &mut self.open,
            output,
        )
    }
}

// Keep this bounded copy in the caller; outlining it regresses neighborhood queries.
#[inline(always)]
fn polygon_vertices<'a>(
    context: FilterPolygon<'_>,
    buffer: &'a mut [Point; 6],
) -> Option<&'a [Point]> {
    let vertices = buffer.get_mut(..context.polygon.vertices().len())?;
    let records = context.tile.vertex_records();
    for (point, index) in vertices.iter_mut().zip(context.polygon.vertices()) {
        *point = crate::nav_data::read_point(records.get(usize::from(index))?);
    }
    Some(vertices)
}
