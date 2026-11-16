// Altered Rust translation of DetourPathQueue.cpp, RecastNavigation v1.6.0.
// Copyright (c) 2009-2010 Mikko Mononen. Zlib: vendor/recastnavigation/License.txt.
use crate::{nav::*, query::NavMeshQuery};
pub type PathQueueRef = u32;
struct Request<K> {
    reference: PathQueueRef,
    start: PolyRef,
    end: PolyRef,
    start_position: Point,
    end_position: Point,
    path: Vec<PolyRef>,
    status: u32,
    keep_alive: u32,
    filter: Option<K>,
}
impl<K> Default for Request<K> {
    fn default() -> Self {
        Self {
            reference: 0,
            start: 0,
            end: 0,
            start_position: [0.; 3],
            end_position: [0.; 3],
            path: vec![],
            status: 0,
            keep_alive: 0,
            filter: None,
        }
    }
}
/// Eight deferred requests sharing a sliced query. Requests keep caller-defined
/// filter keys; each update resolves them to the caller's current filters. This
/// preserves mutable native filter identity without retaining unsafe pointers.
pub struct PathQueue<K> {
    queue: [Request<K>; 8],
    next_handle: PathQueueRef,
    max_path_size: usize,
    head: u32,
    query: NavMeshQuery,
}
impl<K> PathQueue<K> {
    pub fn new(max_path_size: usize, max_search_nodes: usize) -> Result<Self, u32> {
        if max_path_size > i32::MAX as usize {
            return Err(FAILURE | INVALID_PARAM);
        }
        let query = NavMeshQuery::new(max_search_nodes)?;
        let mut queue: [Request<K>; 8] = std::array::from_fn(|_| Request::default());
        for q in &mut queue {
            q.path
                .try_reserve_exact(max_path_size)
                .map_err(|_| FAILURE | OUT_OF_MEMORY)?;
        }
        Ok(Self {
            queue,
            next_handle: 1,
            max_path_size,
            head: 0,
            query,
        })
    }
    /// Retains the handle counter and initialized slot metadata. Fresh Rust
    /// path buffers replace native uninitialized allocation storage.
    pub fn reinitialize(
        &mut self,
        max_path_size: usize,
        max_search_nodes: usize,
    ) -> Result<(), u32> {
        let fresh = Self::new(max_path_size, max_search_nodes)?;
        self.replace_storage(fresh);
        Ok(())
    }
    pub(crate) fn replace_storage(&mut self, fresh: Self) {
        self.query = fresh.query;
        self.max_path_size = fresh.max_path_size;
        self.head = 0;
        for (q, path) in self.queue.iter_mut().zip(fresh.queue) {
            q.reference = 0;
            q.path = path.path;
        }
    }
    pub fn query(&self) -> &NavMeshQuery {
        &self.query
    }
    pub fn request(
        &mut self,
        start: PolyRef,
        end: PolyRef,
        start_position: Point,
        end_position: Point,
        filter: Option<K>,
    ) -> PathQueueRef {
        let Some(q) = self.queue.iter_mut().find(|q| q.reference == 0) else {
            return 0;
        };
        let reference = self.next_handle;
        self.next_handle = self.next_handle.wrapping_add(1);
        if self.next_handle == 0 {
            self.next_handle = 1;
        }
        q.reference = reference;
        q.start = start;
        q.end = end;
        q.start_position = start_position;
        q.end_position = end_position;
        q.status = 0;
        q.path.clear();
        q.filter = filter;
        q.keep_alive = 0;
        reference
    }
    pub fn request_status(&self, reference: PathQueueRef) -> u32 {
        self.queue
            .iter()
            .find(|q| q.reference == reference)
            .map_or(FAILURE, |q| q.status)
    }
    /// Unknown requests leave the supplied output unchanged. Reading a pending
    /// request is allowed and releases it, matching native behavior.
    pub fn path_result_into(
        &mut self,
        reference: PathQueueRef,
        capacity: usize,
        output: &mut Vec<PolyRef>,
    ) -> u32 {
        let Some(q) = self.queue.iter_mut().find(|q| q.reference == reference) else {
            return FAILURE;
        };
        let details = q.status & STATUS_DETAIL_MASK;
        q.reference = 0;
        q.status = 0;
        output.clear();
        output.extend(q.path.iter().take(capacity).copied());
        SUCCESS | details
    }
    pub fn path_result(&mut self, reference: PathQueueRef, capacity: usize) -> Corridor {
        let mut polygons = vec![];
        let status = self.path_result_into(reference, capacity, &mut polygons);
        Corridor { status, polygons }
    }
    pub fn update<'f, F: QueryFilter + ?Sized + 'f>(
        &mut self,
        mesh: &NavMesh,
        max_iterations: i32,
        mut filters: impl FnMut(&K) -> &'f F,
    ) {
        let mut remaining = max_iterations;
        for _ in 0..8 {
            let q = &mut self.queue[(self.head % 8) as usize];
            if q.reference == 0 {
                self.head = self.head.wrapping_add(1);
                continue;
            }
            if q.status & (SUCCESS | FAILURE) != 0 {
                q.keep_alive += 1;
                if q.keep_alive > 2 {
                    q.reference = 0;
                    q.status = 0;
                }
                self.head = self.head.wrapping_add(1);
                continue;
            }
            let filter = q.filter.as_ref().map(&mut filters);
            if q.status == 0 {
                q.status = self.query.init_sliced_find_path_optional(
                    mesh,
                    q.start,
                    q.end,
                    q.start_position,
                    q.end_position,
                    filter,
                    0,
                );
            }
            if q.status & IN_PROGRESS != 0 {
                let mut iterations = 0;
                q.status = self.query.update_sliced_find_path(
                    mesh,
                    filter.unwrap(),
                    remaining,
                    Some(&mut iterations),
                );
                remaining -= iterations;
            }
            if q.status & SUCCESS != 0 {
                let mut result = Corridor {
                    polygons: std::mem::take(&mut q.path),
                    ..Default::default()
                };
                q.status = self.query.finalize_sliced_find_path_into(
                    mesh,
                    filter.unwrap(),
                    self.max_path_size,
                    &mut result,
                );
                q.path = result.polygons;
            }
            if remaining <= 0 {
                break;
            }
            self.head = self.head.wrapping_add(1);
        }
    }
}
