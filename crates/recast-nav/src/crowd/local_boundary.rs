// Altered Rust translation of DetourLocalBoundary.cpp, RecastNavigation v1.6.0.
// Copyright (c) 2009-2010 Mikko Mononen. Zlib: vendor/recastnavigation/License.txt.
use crate::{nav::*, query::NavMeshQuery};
#[derive(Clone, Copy, Debug, Default)]
pub struct BoundarySegment {
    pub positions: [Point; 2],
    distance_squared: f32,
}
#[derive(Clone, Debug)]
pub struct LocalBoundary {
    center: Point,
    segments: [BoundarySegment; 8],
    segment_count: u8,
    // Writes always extend an initialized prefix; reset preserves its backing.
    initialized_segments: u8,
    polygons: [PolyRef; 16],
    polygon_count: u8,
}
impl Default for LocalBoundary {
    fn default() -> Self {
        Self {
            center: [f32::MAX; 3],
            segments: [BoundarySegment::default(); 8],
            segment_count: 0,
            initialized_segments: 0,
            polygons: [0; 16],
            polygon_count: 0,
        }
    }
}
impl LocalBoundary {
    // dtCrowd value-initializes its aggregate agents before constructing their
    // boundary members, so these backing records are initialized to zero.
    pub(crate) fn zero_initialized() -> Self {
        Self {
            initialized_segments: 8,
            ..Self::default()
        }
    }

    pub fn reset(&mut self) {
        self.center = [f32::MAX; 3];
        self.segment_count = 0;
        self.polygon_count = 0;
    }
    pub fn center(&self) -> Point {
        self.center
    }
    pub fn segment_count(&self) -> usize {
        usize::from(self.segment_count)
    }
    /// Initialized backing entries remain readable after reset or a shorter
    /// update. Slots never initialized by an update return None.
    pub fn segment(&self, index: usize) -> Option<&BoundarySegment> {
        (index < usize::from(self.initialized_segments)).then(|| &self.segments[index])
    }
    pub fn segments(&self) -> impl Iterator<Item = &BoundarySegment> {
        self.segments[..self.segment_count()].iter()
    }
    fn add_segment(&mut self, distance_squared: f32, positions: [Point; 2]) {
        let n = self.segment_count();
        let index = if n == 0 {
            0
        } else if distance_squared >= self.segments[n - 1].distance_squared {
            if n == 8 {
                return;
            }
            n
        } else {
            let index = (0..n)
                .find(|&i| distance_squared <= self.segments[i].distance_squared)
                .unwrap_or(n);
            // Native's NaN/full-array path may form an out-of-bounds pointer.
            if index >= 8 {
                return;
            }
            let count = (n - index).min(8 - index - 1);
            self.segments.copy_within(index..index + count, index + 1);
            index
        };
        self.segments[index] = BoundarySegment {
            positions,
            distance_squared,
        };
        self.segment_count = (n + 1).min(8) as u8;
        self.initialized_segments = self.initialized_segments.max(self.segment_count);
    }
    pub fn update<F: QueryFilter + ?Sized>(
        &mut self,
        reference: PolyRef,
        position: Point,
        range: f32,
        mesh: &NavMesh,
        query: &mut NavMeshQuery,
        filter: &F,
    ) {
        if reference == 0 {
            self.reset();
            return;
        }
        self.center = position;
        query.find_local_neighborhood_refs_into(
            mesh,
            reference,
            position,
            range,
            filter,
            &mut self.polygons,
            &mut self.polygon_count,
        );
        self.segment_count = 0;
        // Native reuses eighteen temporary wall segments, without portal refs.
        let mut walls = [[[0.; 3]; 2]; 18];
        for j in 0..usize::from(self.polygon_count) {
            let mut count = 0;
            mesh.poly_wall_segments_with_output(
                self.polygons[j],
                filter,
                false,
                walls.len(),
                &mut 0,
                |segment| {
                    walls[count] = segment.positions;
                    count += 1;
                },
            );
            for &positions in &walls[..count] {
                let distance_squared = crate::nav::segment(position, positions[0], positions[1]).0;
                if distance_squared > range * range {
                    continue;
                }
                self.add_segment(distance_squared, positions);
            }
        }
    }
    pub fn is_valid<F: QueryFilter + ?Sized>(
        &self,
        mesh: &NavMesh,
        query: &NavMeshQuery,
        filter: &F,
    ) -> bool {
        self.polygon_count != 0
            && self.polygons[..usize::from(self.polygon_count)]
                .iter()
                .all(|&reference| query.is_valid_poly_ref(mesh, reference, filter))
    }
}
