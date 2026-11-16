//! Watershed partitioning
//!   - the classic Recast partitioning
//!   - creates the nicest tessellation
//!   - usually slowest
//!   - partitions the heightfield into nice regions without holes or overlaps
//!   - the are some corner cases where this method creates produces holes and overlaps
//!      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
//!      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
//!   * generally the best choice if you precompute the navmesh, use this if you have large open areas

use alloc::vec::Vec;

use crate::{
    CompactHeightfield,
    math::{dir_offset_x, dir_offset_z},
};

impl CompactHeightfield {
    /// Prepare for region partitioning, by calculating distance field along the walkable surface.
    pub fn build_distance_field(&mut self) {
        self.build_distance_field_with_context(&crate::context::Context::new(false));
    }

    pub fn build_distance_field_with_context(&mut self, context: &crate::context::Context<'_>) {
        use crate::context::TimerLabel;
        let _timer = context.scoped_timer(TimerLabel::BUILD_DISTANCEFIELD);
        self.dist = Vec::new();
        let mut distance_field = vec![0; self.spans.len()];
        let mut blurred = vec![0; self.spans.len()];
        {
            let _distance = context.scoped_timer(TimerLabel::BUILD_DISTANCEFIELD_DIST);
            self.calculate_distance_field(&mut distance_field);
            self.max_distance = distance_field.iter().max().copied().unwrap_or_default();
        }
        {
            let _blur = context.scoped_timer(TimerLabel::BUILD_DISTANCEFIELD_BLUR);
            self.box_blur(1, &distance_field, &mut blurred);
            self.dist = blurred;
        }
    }

    fn calculate_distance_field(&self, distance_field: &mut [u16]) {
        // Init distance and points.
        distance_field.fill(u16::MAX);

        // Mark boundary cells.
        for z in 0..self.height {
            for x in 0..self.width {
                let cell = self.cell_at(x, z);
                let max_index = cell.index() as usize + cell.count() as usize;
                #[expect(
                    clippy::needless_range_loop,
                    reason = "lol the alternative suggestion is really unreadable"
                )]
                for i in cell.index() as usize..max_index {
                    let span = &self.spans[i];
                    let area = self.areas[i];

                    let mut connection_count = 0;
                    for dir in 0..4 {
                        if let Some(con) = span.con(dir) {
                            let a_x = x + dir_offset_x(dir) as i32;
                            let a_z = z + dir_offset_z(dir) as i32;
                            let a_index = self.cell_at(a_x, a_z).index() as usize + con as usize;
                            if area == self.areas[a_index] {
                                connection_count += 1;
                            }
                        }
                    }
                    if connection_count < 4 {
                        distance_field[i] = 0;
                    }
                }
            }
        }

        // Defer each span's store until its updates finish. A neighbor aliasing
        // this span cannot lower it: its old value plus a positive cost is at
        // least the decreasing local value. Boundary marking also guarantees
        // that a span above zero has all four cardinal connections, so only
        // diagonal connections need optional checks below.
        // Pass 1
        for z in 0..self.height {
            for x in 0..self.width {
                let cell = self.cell_at(x, z);
                let max_index = cell.index() as usize + cell.count() as usize;
                for i in cell.index() as usize..max_index {
                    // Every incoming step costs at least two, so values at or
                    // below two cannot improve during this relaxation pass.
                    let mut distance = distance_field[i];
                    if distance <= 2 {
                        continue;
                    }
                    let span = &self.spans[i];
                    {
                        let con = span.con_raw(0).unwrap();
                        // (-1,0)
                        let a_x = x + dir_offset_x(0) as i32;
                        let a_z = z + dir_offset_z(0) as i32;
                        let a_index = self.cell_at(a_x, a_z).index() as usize + con as usize;
                        let a_span = &self.spans[a_index];
                        distance = distance.min(distance_field[a_index].saturating_add(2));

                        // (-1,-1)
                        if let Some(con) = a_span.con(3) {
                            let b_x = a_x + dir_offset_x(3) as i32;
                            let b_z = a_z + dir_offset_z(3) as i32;
                            let b_index = self.cell_at(b_x, b_z).index() as usize + con as usize;
                            distance = distance.min(distance_field[b_index].saturating_add(3));
                        }
                    }

                    {
                        let con = span.con_raw(3).unwrap();
                        // (0, -1)
                        let a_x = x + dir_offset_x(3) as i32;
                        let a_z = z + dir_offset_z(3) as i32;
                        let a_index = self.cell_at(a_x, a_z).index() as usize + con as usize;
                        let a_span = &self.spans[a_index];
                        distance = distance.min(distance_field[a_index].saturating_add(2));

                        // (1, -1)
                        if let Some(con) = a_span.con(2) {
                            let b_x = a_x + dir_offset_x(2) as i32;
                            let b_z = a_z + dir_offset_z(2) as i32;
                            let b_index = self.cell_at(b_x, b_z).index() as usize + con as usize;
                            distance = distance.min(distance_field[b_index].saturating_add(3));
                        }
                    }
                    distance_field[i] = distance;
                }
            }
        }

        // Pass 2
        for z in (0..self.height).rev() {
            for x in (0..self.width).rev() {
                let cell = self.cell_at(x, z);
                let max_index = cell.index() as usize + cell.count() as usize;
                for i in cell.index() as usize..max_index {
                    // Every incoming step costs at least two, so values at or
                    // below two cannot improve during this relaxation pass.
                    let mut distance = distance_field[i];
                    if distance <= 2 {
                        continue;
                    }
                    let span = &self.spans[i];
                    {
                        let con = span.con_raw(2).unwrap();
                        // (1, 0)
                        let a_x = x + dir_offset_x(2) as i32;
                        let a_z = z + dir_offset_z(2) as i32;
                        let a_index = self.cell_at(a_x, a_z).index() as usize + con as usize;
                        let a_span = &self.spans[a_index];
                        distance = distance.min(distance_field[a_index].saturating_add(2));

                        // (1, 1)
                        if let Some(con) = a_span.con(1) {
                            let b_x = a_x + dir_offset_x(1) as i32;
                            let b_z = a_z + dir_offset_z(1) as i32;
                            let b_index = self.cell_at(b_x, b_z).index() as usize + con as usize;
                            distance = distance.min(distance_field[b_index].saturating_add(3));
                        }
                    }

                    {
                        let con = span.con_raw(1).unwrap();
                        // (0, 1)
                        let a_x = x + dir_offset_x(1) as i32;
                        let a_z = z + dir_offset_z(1) as i32;
                        let a_index = self.cell_at(a_x, a_z).index() as usize + con as usize;
                        let a_span = &self.spans[a_index];
                        distance = distance.min(distance_field[a_index].saturating_add(2));

                        // (-1, 1)
                        if let Some(con) = a_span.con(0) {
                            let b_x = a_x + dir_offset_x(0) as i32;
                            let b_z = a_z + dir_offset_z(0) as i32;
                            let b_index = self.cell_at(b_x, b_z).index() as usize + con as usize;
                            distance = distance.min(distance_field[b_index].saturating_add(3));
                        }
                    }
                    distance_field[i] = distance;
                }
            }
        }
    }

    fn box_blur(&self, threshold: u16, distance_field: &[u16], result: &mut [u16]) {
        let threshold = threshold.saturating_mul(2);

        for z in 0..self.height {
            for x in 0..self.width {
                let cell = self.cell_at(x, z);
                let max_index = cell.index() as usize + cell.count() as usize;
                for i in cell.index() as usize..max_index {
                    let span = &self.spans[i];
                    let cd = distance_field[i];
                    if cd <= threshold {
                        result[i] = cd;
                        continue;
                    }
                    let mut d = cd as u32;
                    for dir in 0..4 {
                        // Boundary marking sets any span with a missing
                        // connection to zero, and relaxation only lowers it.
                        // A span reaching this branch therefore has all four
                        // cardinal connections. Keep the diagonal check: its
                        // neighbor need not have been visited by either pass.
                        let con = span.con_raw(i32::from(dir)).unwrap();
                        let a_x = x + dir_offset_x(dir) as i32;
                        let a_z = z + dir_offset_z(dir) as i32;
                        let a_index = self.cell_at(a_x, a_z).index() as usize + con as usize;
                        d += distance_field[a_index] as u32;

                        let a_span = &self.spans[a_index];
                        let dir2 = (dir + 1) & 0x3;
                        if let Some(con) = a_span.con(dir2) {
                            let b_x = a_x + dir_offset_x(dir2) as i32;
                            let b_z = a_z + dir_offset_z(dir2) as i32;
                            let b_index = self.cell_at(b_x, b_z).index() as usize + con as usize;
                            d += distance_field[b_index] as u32;
                        } else {
                            d += cd as u32;
                        }
                    }
                    result[i] = ((d + 5) / 9) as u16;
                }
            }
        }
    }
}
