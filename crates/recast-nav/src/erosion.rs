// Altered Rust translation: signed native radius and byte threshold conversion.
use crate::{
    AreaType, CompactHeightfield,
    math::{dir_offset_x, dir_offset_z},
};
use alloc::vec::Vec;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ErosionError {
    RadiusOverflow,
    Allocation,
}

impl core::fmt::Display for ErosionError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Self::RadiusOverflow => {
                "twice the erosion radius overflows the native signed integer domain"
            }
            Self::Allocation => "erosion distance allocation failed",
        })
    }
}
impl core::error::Error for ErosionError {}

impl CompactHeightfield {
    /// Native context timer and diagnostics around erosion.
    pub fn erode_walkable_area_with_context(
        &mut self,
        context: &crate::context::Context<'_>,
        erosion_radius: i32,
    ) -> Result<(), ErosionError> {
        use crate::context::{LogCategory, TimerLabel};
        let _timer = context.scoped_timer(TimerLabel::ERODE_AREA);
        let result = self.erode_walkable_area(erosion_radius);
        if result == Err(ErosionError::Allocation) {
            let _ = context.log_format(
                LogCategory::ERROR,
                format_args!(
                    "erodeWalkableArea: Out of memory 'dist' ({}).",
                    self.spans.len()
                ),
            );
        }
        result
    }

    /// Erode the walkable area by agent radius.
    pub fn erode_walkable_area(&mut self, erosion_radius: i32) -> Result<(), ErosionError> {
        let min_boundary_distance = erosion_radius
            .checked_mul(2)
            .ok_or(ErosionError::RadiusOverflow)? as u8;
        let mut distance_to_boundary = Vec::new();
        distance_to_boundary
            .try_reserve_exact(self.spans.len())
            .map_err(|_| ErosionError::Allocation)?;
        distance_to_boundary.resize(self.spans.len(), u8::MAX);

        // Mark boundary cells.
        for z in 0..self.height {
            for x in 0..self.width {
                let cell = self.cell_at(x, z);
                let max_span_index = cell.index() as usize + cell.count() as usize;
                #[expect(
                    clippy::needless_range_loop,
                    reason = "lol the alternative suggestion is really unreadable"
                )]
                for span_index in cell.index() as usize..max_span_index {
                    if !self.areas[span_index].is_walkable() {
                        distance_to_boundary[span_index] = 0;
                        continue;
                    }
                    let span = &self.spans[span_index];
                    // Check that there is a non-null adjacent span in each of the 4 cardinal directions.
                    let mut neighbor_count = 0;
                    for direction in 0..4 {
                        let Some(neighbor_connection) = span.con(direction) else {
                            break;
                        };
                        let neighbor_x = x + dir_offset_x(direction) as i32;
                        let neighbor_z = z + dir_offset_z(direction) as i32;
                        let neighbor_span_index = self.cell_at(neighbor_x, neighbor_z).index()
                            as usize
                            + neighbor_connection as usize;

                        if !self.areas[neighbor_span_index].is_walkable() {
                            break;
                        }
                        neighbor_count += 1;
                    }

                    // At least one missing neighbour, so this is a boundary cell.
                    if neighbor_count != 4 {
                        distance_to_boundary[span_index] = 0;
                    }
                }
            }
        }

        let mut new_distance: u8;

        // Pass 1
        for z in 0..self.height {
            for x in 0..self.width {
                let cell = self.cell_at(x, z);
                let max_span_index = cell.index() as usize + cell.count() as usize;
                for span_index in cell.index() as usize..max_span_index {
                    let span = &self.spans[span_index];

                    if let Some(con) = span.con(0) {
                        // (-1,0)
                        let a_x = x + dir_offset_x(0) as i32;
                        let a_z = z + dir_offset_z(0) as i32;
                        let a_index = self.cell_at(a_x, a_z).index() as usize + con as usize;
                        let a_span = &self.spans[a_index];
                        new_distance = distance_to_boundary[a_index].saturating_add(2);
                        if new_distance < distance_to_boundary[span_index] {
                            distance_to_boundary[span_index] = new_distance;
                        };

                        // (-1,-1)
                        if let Some(con) = a_span.con(3) {
                            let b_x = a_x + dir_offset_x(3) as i32;
                            let b_z = a_z + dir_offset_z(3) as i32;
                            let b_index = self.cell_at(b_x, b_z).index() as usize + con as usize;
                            new_distance = distance_to_boundary[b_index].saturating_add(3);
                            if new_distance < distance_to_boundary[span_index] {
                                distance_to_boundary[span_index] = new_distance;
                            };
                        }
                    }
                    if let Some(con) = span.con(3) {
                        // (0,-1)
                        let a_x = x + dir_offset_x(3) as i32;
                        let a_z = z + dir_offset_z(3) as i32;
                        let a_index = self.cell_at(a_x, a_z).index() as usize + con as usize;
                        let a_span = &self.spans[a_index];
                        new_distance = distance_to_boundary[a_index].saturating_add(2);
                        if new_distance < distance_to_boundary[span_index] {
                            distance_to_boundary[span_index] = new_distance;
                        };

                        // (1,-1)
                        if let Some(con) = a_span.con(2) {
                            let b_x = a_x + dir_offset_x(2) as i32;
                            let b_z = a_z + dir_offset_z(2) as i32;
                            let b_index = self.cell_at(b_x, b_z).index() as usize + con as usize;
                            new_distance = distance_to_boundary[b_index].saturating_add(3);
                            if new_distance < distance_to_boundary[span_index] {
                                distance_to_boundary[span_index] = new_distance;
                            };
                        }
                    }
                }
            }
        }

        // Pass 2
        for z in (0..self.height).rev() {
            for x in (0..self.width).rev() {
                let cell = self.cell_at(x, z);
                let max_span_index = cell.index() as usize + cell.count() as usize;
                for span_index in cell.index() as usize..max_span_index {
                    let span = &self.spans[span_index];

                    if let Some(con) = span.con(2) {
                        // (1,0)
                        let a_x = x + dir_offset_x(2) as i32;
                        let a_z = z + dir_offset_z(2) as i32;
                        let a_index = self.cell_at(a_x, a_z).index() as usize + con as usize;
                        let a_span = &self.spans[a_index];
                        new_distance = distance_to_boundary[a_index].saturating_add(2);
                        if new_distance < distance_to_boundary[span_index] {
                            distance_to_boundary[span_index] = new_distance;
                        };

                        // (1,1)
                        if let Some(con) = a_span.con(1) {
                            let b_x = a_x + dir_offset_x(1) as i32;
                            let b_z = a_z + dir_offset_z(1) as i32;
                            let b_index = self.cell_at(b_x, b_z).index() as usize + con as usize;
                            new_distance = distance_to_boundary[b_index].saturating_add(3);
                            if new_distance < distance_to_boundary[span_index] {
                                distance_to_boundary[span_index] = new_distance;
                            };
                        }
                    }
                    if let Some(con) = span.con(1) {
                        // (0,1)
                        let a_x = x + dir_offset_x(1) as i32;
                        let a_z = z + dir_offset_z(1) as i32;
                        let a_index = self.cell_at(a_x, a_z).index() as usize + con as usize;
                        let a_span = &self.spans[a_index];
                        new_distance = distance_to_boundary[a_index].saturating_add(2);
                        if new_distance < distance_to_boundary[span_index] {
                            distance_to_boundary[span_index] = new_distance;
                        };

                        // (1,1)
                        if let Some(con) = a_span.con(0) {
                            let b_x = a_x + dir_offset_x(0) as i32;
                            let b_z = a_z + dir_offset_z(0) as i32;
                            let b_index = self.cell_at(b_x, b_z).index() as usize + con as usize;
                            new_distance = distance_to_boundary[b_index].saturating_add(3);
                            if new_distance < distance_to_boundary[span_index] {
                                distance_to_boundary[span_index] = new_distance;
                            };
                        }
                    }
                }
            }
        }

        #[expect(
            clippy::needless_range_loop,
            reason = "lol the alternative suggestion is really unreadable"
        )]
        for span_index in 0..self.spans.len() {
            if distance_to_boundary[span_index] < min_boundary_distance {
                self.areas[span_index] = AreaType::NOT_WALKABLE;
            }
        }
        Ok(())
    }
}
