// Altered Rust translation of DetourPathCorridor.cpp, RecastNavigation v1.6.0.
// Copyright (c) 2009-2010 Mikko Mononen. Zlib: vendor/recastnavigation/License.txt.
use crate::{
    nav::*,
    poly_buffer::{PolyBuffer, SlicePolys},
    query::{NavMeshQuery, SurfaceMove},
};
fn common(path: &[PolyRef], visited: &[PolyRef], backwards: bool) -> Option<(usize, usize)> {
    if backwards {
        path.iter()
            .enumerate()
            .rev()
            .find_map(|(i, p)| visited.iter().position(|v| v == p).map(|j| (i, j)))
    } else {
        path.iter()
            .enumerate()
            .find_map(|(i, p)| visited.iter().position(|v| v == p).map(|j| (i, j)))
    }
}
/// Native merges mutate a fixed buffer. The count selects its used prefix;
/// untouched tail entries remain intact. `max_path` is the native logical
/// capacity; the slice separately bounds actual memory. Invalid accesses fail.
pub fn merge_corridor_start_moved(
    path: &mut [PolyRef],
    count: usize,
    max_path: usize,
    visited: &[PolyRef],
) -> Result<usize, u32> {
    if count > path.len() {
        return Err(FAILURE | INVALID_PARAM);
    }
    let Some((p, v)) = common(&path[..count], visited, true) else {
        return Ok(count);
    };
    let required = visited.len() - v;
    if required > max_path {
        return Err(FAILURE | INVALID_PARAM);
    }
    let original = p + 1;
    let size = (count - original).min(max_path - required);
    if required + size > path.len() {
        return Err(FAILURE | INVALID_PARAM);
    }
    path.copy_within(original..original + size, required);
    for i in 0..required {
        path[i] = visited[visited.len() - 1 - i];
    }
    Ok(required + size)
}
pub fn merge_corridor_end_moved(
    path: &mut [PolyRef],
    count: usize,
    max_path: usize,
    visited: &[PolyRef],
) -> Result<usize, u32> {
    if count > path.len() {
        return Err(FAILURE | INVALID_PARAM);
    }
    let Some((p, v)) = common(&path[..count], visited, false) else {
        return Ok(count);
    };
    let original = p + 1;
    let from = v + 1;
    if original > max_path {
        return Err(FAILURE | INVALID_PARAM);
    }
    let count = (visited.len() - from).min(max_path - original);
    if original + count > path.len() {
        return Err(FAILURE | INVALID_PARAM);
    }
    path[original..original + count].copy_from_slice(&visited[from..from + count]);
    Ok(original + count)
}
pub fn merge_corridor_start_shortcut(
    path: &mut [PolyRef],
    count: usize,
    max_path: usize,
    visited: &[PolyRef],
) -> Result<usize, u32> {
    if count > path.len() {
        return Err(FAILURE | INVALID_PARAM);
    }
    let Some((p, required)) = common(&path[..count], visited, true) else {
        return Ok(count);
    };
    if required == 0 {
        return Ok(count);
    }
    if required > max_path {
        return Err(FAILURE | INVALID_PARAM);
    }
    let size = (count - p).min(max_path - required);
    if required + size > path.len() {
        return Err(FAILURE | INVALID_PARAM);
    }
    path.copy_within(p..p + size, required);
    path[..required].copy_from_slice(&visited[..required]);
    Ok(required + size)
}
#[derive(Clone, Copy, Debug, Default)]
pub struct Corner {
    pub position: Point,
    pub flags: u8,
    pub reference: PolyRef,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct OffMeshMove {
    pub references: [PolyRef; 2],
    pub positions: [Point; 2],
}
#[derive(Clone, Debug)]
pub struct PathCorridor {
    position: Point,
    target: Point,
    path: Vec<PolyRef>,
    count: usize,
}
fn distance_squared(a: Point, b: Point) -> f32 {
    let dx = b[0] - a[0];
    let dz = b[2] - a[2];
    madd(dx, dx, dz * dz)
}
impl PathCorridor {
    pub fn new(capacity: usize) -> Result<Self, u32> {
        if capacity > i32::MAX as usize {
            return Err(FAILURE | INVALID_PARAM);
        }
        let mut path = Vec::new();
        path.try_reserve_exact(capacity)
            .map_err(|_| FAILURE | OUT_OF_MEMORY)?;
        path.resize(capacity, 0);
        Ok(Self {
            position: [0.; 3],
            target: [0.; 3],
            path,
            count: 0,
        })
    }
    pub fn position(&self) -> Point {
        self.position
    }
    pub fn target(&self) -> Point {
        self.target
    }
    pub fn path(&self) -> &[PolyRef] {
        &self.path[..self.count]
    }
    pub fn first_polygon(&self) -> PolyRef {
        self.path().first().copied().unwrap_or(0)
    }
    pub fn last_polygon(&self) -> PolyRef {
        self.path().last().copied().unwrap_or(0)
    }
    pub fn reset(&mut self, reference: PolyRef, position: Point) -> bool {
        if self.path.is_empty() {
            return false;
        }
        self.position = position;
        self.target = position;
        self.path[0] = reference;
        self.count = 1;
        true
    }
    /// Native's asserted input contract requires a nonempty path strictly
    /// smaller than the buffer capacity. Position is retained.
    pub fn set_corridor(&mut self, target: Point, path: &[PolyRef]) -> bool {
        if path.is_empty() || path.len() >= self.path.len() {
            return false;
        }
        self.target = target;
        self.path[..path.len()].copy_from_slice(path);
        self.count = path.len();
        true
    }
    pub fn find_corners(
        &self,
        mesh: &NavMesh,
        query: &NavMeshQuery,
        capacity: usize,
    ) -> Vec<Corner> {
        let result =
            query.straight_path(mesh, self.position, self.target, self.path(), capacity, 0);
        let mut corners: Vec<_> = result
            .points
            .into_iter()
            .zip(result.flags)
            .zip(result.references)
            .map(|((position, flags), reference)| Corner {
                position,
                flags,
                reference,
            })
            .collect();
        let count = corners.len();
        let count = self.prune_corners(&mut corners, count);
        corners.truncate(count);
        corners
    }
    /// Writes caller-owned corner slots, retaining initialized entries after
    /// the returned count. Scratch straight-path buffers can be reused too.
    pub fn find_corners_into(
        &self,
        mesh: &NavMesh,
        query: &NavMeshQuery,
        corners: &mut [Corner],
        scratch: &mut StraightPath,
    ) -> usize {
        query.straight_path_into(
            mesh,
            self.position,
            self.target,
            self.path(),
            corners.len(),
            0,
            scratch,
        );
        for (index, ((&position, &flags), &reference)) in scratch
            .points
            .iter()
            .zip(&scratch.flags)
            .zip(&scratch.references)
            .enumerate()
        {
            corners[index] = Corner {
                position,
                flags,
                reference,
            };
        }
        self.prune_corners(corners, scratch.points.len())
    }
    fn prune_corners(&self, corners: &mut [Corner], mut count: usize) -> usize {
        // Native removes one leading point at a time. Repeating the move also
        // preserves its retained tail slots when several points are pruned.
        while count > 0 {
            if corners[0].flags & 4 != 0
                || distance_squared(corners[0].position, self.position) > 0.01 * 0.01
            {
                break;
            }
            count -= 1;
            corners.copy_within(1..count + 1, 0);
        }
        if let Some(index) = corners[..count].iter().position(|c| c.flags & 4 != 0) {
            count = index + 1;
        }
        count
    }
    pub fn optimize_visibility<F: QueryFilter + ?Sized>(
        &mut self,
        next: Point,
        range: f32,
        mesh: &NavMesh,
        _query: &NavMeshQuery,
        filter: &F,
    ) {
        let mut values = [0; 32];
        let mut path = SlicePolys::new(&mut values);
        let mut hit = RaycastHit::default();
        self.optimize_visibility_to_buffer(
            next,
            range,
            mesh,
            filter,
            &mut hit.split_output().0,
            &mut path,
        );
    }
    pub(crate) fn optimize_visibility_with_storage<F: QueryFilter + ?Sized>(
        &mut self,
        next: Point,
        range: f32,
        mesh: &NavMesh,
        _query: &NavMeshQuery,
        filter: &F,
        hit: &mut RaycastHit,
    ) {
        let (mut output, path) = hit.split_output();
        self.optimize_visibility_to_buffer(next, range, mesh, filter, &mut output, path);
    }
    fn optimize_visibility_to_buffer<F: QueryFilter + ?Sized>(
        &mut self,
        next: Point,
        range: f32,
        mesh: &NavMesh,
        filter: &F,
        hit: &mut RaycastOutput<'_>,
        path: &mut impl PolyBuffer,
    ) {
        if self.count == 0 {
            return;
        }
        let distance = distance_squared(self.position, next).sqrt();
        if distance < 0.01 {
            return;
        }
        let distance = if distance + 0.01 < range {
            distance + 0.01
        } else {
            range
        };
        let scale = range / distance;
        let goal =
            std::array::from_fn(|i| madd(next[i] - self.position[i], scale, self.position[i]));
        let capacity = self.path.len();
        path.clear();
        mesh.raycast_append_to_buffer(
            self.path[0],
            self.position,
            goal,
            filter,
            0,
            0,
            32,
            hit.reborrow(),
            path,
        );
        if path.len() > 1
            && hit.fraction() > 0.99
            && let Ok(count) =
                merge_corridor_start_shortcut(&mut self.path, self.count, capacity, path.as_slice())
        {
            self.count = count;
        }
    }
    pub fn optimize_topology<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        query: &mut NavMeshQuery,
        filter: &F,
    ) -> bool {
        // Reconstruction permits a defined spare slot for a detached parent.
        let mut values = [0; 33];
        self.optimize_topology_to_buffer(
            mesh,
            query,
            filter,
            &mut 0,
            &mut SlicePolys::new(&mut values),
        )
    }
    pub(crate) fn optimize_topology_with_storage<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        query: &mut NavMeshQuery,
        filter: &F,
        result: &mut Corridor,
    ) -> bool {
        self.optimize_topology_to_buffer(
            mesh,
            query,
            filter,
            &mut result.status,
            &mut result.polygons,
        )
    }
    fn optimize_topology_to_buffer<F: QueryFilter + ?Sized>(
        &mut self,
        mesh: &NavMesh,
        query: &mut NavMeshQuery,
        filter: &F,
        status: &mut u32,
        path: &mut impl PolyBuffer,
    ) -> bool {
        if self.count < 3 {
            return false;
        }
        query.init_sliced_find_path(
            mesh,
            self.path[0],
            self.path[self.count - 1],
            self.position,
            self.target,
            filter,
            0,
        );
        query.update_sliced_find_path(mesh, filter, 32, None);
        *status = query.finalize_sliced_partial_to_buffer(mesh, self.path(), filter, 32, path);
        let capacity = self.path.len();
        if *status & SUCCESS != 0
            && path.len() != 0
            && let Ok(count) =
                merge_corridor_start_shortcut(&mut self.path, self.count, capacity, path.as_slice())
        {
            self.count = count;
            return true;
        }
        false
    }
    /// Native may prune the corridor and write references before endpoint lookup
    /// fails. A connection at the final path index is rejected before pruning.
    pub fn move_over_off_mesh_connection_into(
        &mut self,
        reference: PolyRef,
        mesh: &NavMesh,
        output: &mut OffMeshMove,
    ) -> bool {
        if self.count == 0 {
            return false;
        }
        let mut previous = 0;
        let mut current = self.path[0];
        let mut count = 0;
        while count < self.count && current != reference {
            previous = current;
            current = self.path[count];
            count += 1;
        }
        if count == self.count {
            return false;
        }
        self.path.copy_within(count..self.count, 0);
        self.count -= count;
        output.references = [previous, current];
        if let Ok(positions) = mesh.off_mesh_endpoints(previous, current) {
            output.positions = positions;
            self.position = positions[1];
            true
        } else {
            false
        }
    }
    pub fn move_over_off_mesh_connection(
        &mut self,
        reference: PolyRef,
        mesh: &NavMesh,
    ) -> Option<OffMeshMove> {
        let mut output = OffMeshMove::default();
        self.move_over_off_mesh_connection_into(reference, mesh, &mut output)
            .then_some(output)
    }
    pub fn move_position<F: QueryFilter + ?Sized>(
        &mut self,
        next: Point,
        mesh: &NavMesh,
        query: &mut NavMeshQuery,
        filter: &F,
    ) -> bool {
        let mut values = [0; 16];
        self.move_position_to_buffer(
            next,
            mesh,
            query,
            filter,
            &mut [0.; 3],
            &mut SlicePolys::new(&mut values),
        )
    }
    pub(crate) fn move_position_with_storage<F: QueryFilter + ?Sized>(
        &mut self,
        next: Point,
        mesh: &NavMesh,
        query: &mut NavMeshQuery,
        filter: &F,
        result: &mut SurfaceMove,
    ) -> bool {
        self.move_position_to_buffer(
            next,
            mesh,
            query,
            filter,
            &mut result.position,
            &mut result.visited,
        )
    }
    fn move_position_to_buffer<F: QueryFilter + ?Sized>(
        &mut self,
        next: Point,
        mesh: &NavMesh,
        query: &mut NavMeshQuery,
        filter: &F,
        position: &mut Point,
        visited: &mut impl PolyBuffer,
    ) -> bool {
        if self.count == 0 {
            return false;
        }
        let status = query.move_along_surface_to_buffer(
            mesh,
            self.path[0],
            self.position,
            next,
            filter,
            16,
            position,
            visited,
        );
        if status & SUCCESS == 0 {
            return false;
        }
        let capacity = self.path.len();
        let Ok(count) =
            merge_corridor_start_moved(&mut self.path, self.count, capacity, visited.as_slice())
        else {
            return false;
        };
        self.count = count;
        let height = mesh
            .poly_height(self.path[0], *position)
            .unwrap_or(self.position[1]);
        self.position = *position;
        self.position[1] = height;
        true
    }
    pub fn move_target_position<F: QueryFilter + ?Sized>(
        &mut self,
        next: Point,
        mesh: &NavMesh,
        query: &mut NavMeshQuery,
        filter: &F,
    ) -> bool {
        if self.count == 0 {
            return false;
        }
        let mut values = [0; 16];
        let mut visited = SlicePolys::new(&mut values);
        let mut position = [0.; 3];
        let status = query.move_along_surface_to_buffer(
            mesh,
            self.path[self.count - 1],
            self.target,
            next,
            filter,
            16,
            &mut position,
            &mut visited,
        );
        if status & SUCCESS == 0 {
            return false;
        }
        let capacity = self.path.len();
        let Ok(count) =
            merge_corridor_end_moved(&mut self.path, self.count, capacity, visited.as_slice())
        else {
            return false;
        };
        self.count = count;
        self.target = position;
        true
    }
    pub fn fix_path_start(&mut self, safe: PolyRef, position: Point) -> bool {
        let needed = if self.count > 0 && self.count < 3 {
            3
        } else {
            2
        };
        if self.path.len() < needed {
            return false;
        }
        self.position = position;
        if self.count > 0 && self.count < 3 {
            self.path[2] = self.path[self.count - 1];
            self.path[0] = safe;
            self.path[1] = 0;
            self.count = 3;
        } else {
            self.path[0] = safe;
            self.path[1] = 0;
        }
        true
    }
    pub fn trim_invalid_path<F: QueryFilter + ?Sized>(
        &mut self,
        safe: PolyRef,
        position: Point,
        mesh: &NavMesh,
        query: &NavMeshQuery,
        filter: &F,
    ) -> bool {
        let count = self
            .path()
            .iter()
            .take_while(|&&p| query.is_valid_poly_ref(mesh, p, filter))
            .count();
        if count == self.count {
            return true;
        }
        if count == 0 {
            self.position = position;
            self.path[0] = safe;
            self.count = 1;
        } else {
            self.count = count;
        }
        if let Some(target) = mesh.closest_boundary(self.path[self.count - 1], self.target) {
            self.target = target;
        }
        true
    }
    pub fn is_valid<F: QueryFilter + ?Sized>(
        &self,
        look_ahead: i32,
        mesh: &NavMesh,
        query: &NavMeshQuery,
        filter: &F,
    ) -> bool {
        self.path()
            .iter()
            .take(look_ahead.max(0) as usize)
            .all(|&p| query.is_valid_poly_ref(mesh, p, filter))
    }
}
