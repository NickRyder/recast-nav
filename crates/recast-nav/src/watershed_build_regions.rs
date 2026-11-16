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

// Additional monotone/layer APIs are altered Rust translations of RecastRegion.cpp.
use alloc::vec::Vec;

use crate::{
    AreaType, CompactHeightfield, RegionId,
    context::{Context, LogCategory, TimerLabel},
    math::{dir_offset_x, dir_offset_z},
};

impl CompactHeightfield {
    /// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
    /// Contours will form simple polygons.
    ///
    /// If multiple regions form an area that is smaller than `min_region_area`, then all spans will be
    /// re-assigned to [`AreaType::NOT_WALKABLE`].
    ///
    /// Watershed partitioning can result in smaller than necessary regions, especially in diagonal corridors.
    /// `merge_region_area` helps reduce unnecessarily small regions.
    ///
    /// See the #rcConfig documentation for more information on the configuration parameters.
    ///
    /// The region data will be available via the [`CompactHeightfield::max_region`]
    /// and [`CompactSpan::region`](crate::CompactSpan::region) fields.
    ///
    /// Warning: The distance field must be created using [`CompactHeightfield::build_distance_field`] before attempting to build regions.
    pub fn build_regions(
        &mut self,
        border_size: i32,
        min_region_area: i32,
        merge_region_area: i32,
    ) -> Result<(), BuildRegionsError> {
        self.build_regions_with_context(
            &Context::new(false),
            border_size,
            min_region_area,
            merge_region_area,
        )
    }

    pub fn build_regions_with_context(
        &mut self,
        context: &Context<'_>,
        border_size: i32,
        min_region_area: i32,
        merge_region_area: i32,
    ) -> Result<(), BuildRegionsError> {
        let _timer = context.scoped_timer(TimerLabel::BUILD_REGIONS);
        let mut src_reg = Vec::with_capacity(self.spans.len());
        let mut src_dist = Vec::with_capacity(self.spans.len());
        context.start_timer(TimerLabel::BUILD_REGIONS_WATERSHED);
        const LOG_NB_STACKS: usize = 3;
        const NB_STACKS: usize = 1 << LOG_NB_STACKS;
        let mut level_stacks: [Vec<LevelStackEntry>; NB_STACKS] = [const { Vec::new() }; NB_STACKS];
        for stack in &mut level_stacks {
            stack.reserve(256);
        }

        let mut stack: Vec<LevelStackEntry> = Vec::with_capacity(256);
        let mut dirty_entries = Vec::new();

        src_reg.resize(self.spans.len(), RegionId::NONE);
        src_dist.resize(self.spans.len(), 0_u16);

        let mut region_id = RegionId::from(1);
        let mut level = self.max_distance.wrapping_add(1) & !1;

        // Jan: The following comment is taken from the original implementation.
        // TODO: Figure better formula, expandIters defines how much the
        // watershed "overflows" and simplifies the regions. Tying it to
        // agent radius was usually good indication how greedy it could be.
        //	const int expandIters = 4 + walkableRadius * 2;
        let expand_iters = 8;

        if border_size > 0 {
            // Make sure border will not overflow.
            let border_width = border_size.min(self.width);
            let border_height = border_size.min(self.height);

            // Paint regions
            self.paint_rect_region(
                0,
                border_width,
                0,
                self.height,
                region_id | RegionId::BORDER_REGION,
                &mut src_reg,
            );
            region_id += 1;
            self.paint_rect_region(
                self.width - border_width,
                self.width,
                0,
                self.height,
                region_id | RegionId::BORDER_REGION,
                &mut src_reg,
            );
            region_id += 1;
            self.paint_rect_region(
                0,
                self.width,
                0,
                border_height,
                region_id | RegionId::BORDER_REGION,
                &mut src_reg,
            );
            region_id += 1;
            self.paint_rect_region(
                0,
                self.width,
                self.height - border_height,
                self.height,
                region_id | RegionId::BORDER_REGION,
                &mut src_reg,
            );
            region_id += 1;
        }
        self.border_size = border_size;

        let mut s_id = -1_i32;
        while level > 0 {
            level = level.saturating_sub(2);
            s_id = (s_id + 1) & (NB_STACKS as i32 - 1);

            if s_id == 0 {
                self.sort_cells_by_level(level, &mut src_reg, NB_STACKS, &mut level_stacks, 1);
            } else {
                // copy left overs from last level
                let (src, dst) = level_stacks.split_at_mut(s_id as usize);
                append_stacks(&src[s_id as usize - 1], &mut dst[0], &src_reg);
            }

            {
                let _expand = context.scoped_timer(TimerLabel::BUILD_REGIONS_EXPAND);
                self.expand_regions(
                    expand_iters,
                    level,
                    &mut src_reg,
                    &mut src_dist,
                    (&mut level_stacks[s_id as usize], &mut dirty_entries),
                    false,
                );
            }

            // Mark new regions with IDs.
            {
                let _flood = context.scoped_timer(TimerLabel::BUILD_REGIONS_FLOOD);
                for current in level_stacks[s_id as usize].iter() {
                    let Some(i) = current.span_index() else {
                        continue;
                    };
                    if src_reg[i] == RegionId::NONE
                        && self.flood_region(
                            current,
                            level,
                            region_id,
                            &mut src_reg,
                            &mut src_dist,
                            &mut stack,
                        )
                    {
                        if region_id == RegionId::MAX {
                            context.log(LogCategory::ERROR, "rcBuildRegions: Region ID overflow");
                            return Err(BuildRegionsError::RegionIdOverflow);
                        }
                        region_id += 1;
                    }
                }
            }
        }

        // Expand current regions until no empty connected cells found.
        self.expand_regions(
            expand_iters * 8,
            0,
            &mut src_reg,
            &mut src_dist,
            (&mut stack, &mut dirty_entries),
            true,
        );
        // Expansion scratch is no longer needed during region filtering.
        drop(dirty_entries);
        context.stop_timer(TimerLabel::BUILD_REGIONS_WATERSHED);

        // Merge regions and filter out small regions.
        {
            let _filter = context.scoped_timer(TimerLabel::BUILD_REGIONS_FILTER);
            self.max_region = region_id;
            let overlaps =
                self.merge_and_filter_regions(min_region_area, merge_region_area, &mut src_reg);

            // Native reports overlaps while retaining the merged output.
            if !overlaps.is_empty() {
                let _ = context.log_format(
                    LogCategory::ERROR,
                    format_args!("rcBuildRegions: {} overlapping regions.", overlaps.len()),
                );
                #[cfg(feature = "tracing")]
                tracing::error!(
                    "{len} overlapping regions found during merging.",
                    len = overlaps.len()
                );
            }
        }

        // Write the result out
        #[expect(clippy::needless_range_loop)]
        for i in 0..self.spans.len() {
            self.spans[i].region = src_reg[i];
        }

        Ok(())
    }

    fn merge_and_filter_regions(
        &mut self,
        min_region_area: i32,
        merge_region_size: i32,
        src_reg: &mut [RegionId],
    ) -> Vec<RegionId> {
        let mut overlaps = Vec::new();
        let w = self.width;
        let h = self.height;

        let nreg = self.max_region.bits() as usize + 1;

        // Construct regions
        let mut regions = (0..nreg)
            .map(|i| Region::new(RegionId::from(i as u16)))
            .collect::<Vec<_>>();

        // Find edge of a region and find connections around the contour.
        for z in 0..h {
            for x in 0..w {
                let cell = self.cell_at(x, z);
                let max_index = cell.index() as usize + cell.count() as usize;
                #[expect(clippy::needless_range_loop)]
                for i in cell.index() as usize..max_index {
                    let r = src_reg[i];
                    if r == RegionId::NONE || r.bits() as usize >= nreg {
                        continue;
                    }
                    let reg = &mut regions[r.bits() as usize];
                    reg.span_count += 1;

                    // Update floors
                    for j in cell.index() as usize..max_index {
                        if i == j {
                            continue;
                        }
                        let floor_id = src_reg[j];
                        if floor_id == RegionId::NONE || floor_id.bits() as usize >= nreg {
                            continue;
                        }
                        if floor_id == r {
                            reg.overlap = true;
                        }
                        reg.add_unique_floor_region(floor_id);
                    }

                    // Have found contour
                    if !reg.connections.is_empty() {
                        continue;
                    }

                    reg.area = self.areas[i];

                    // Check if this cell is next to a border
                    let ndir = (0..4).find(|&dir| self.is_solid_edge(src_reg, x, z, i, dir));
                    if let Some(ndir) = ndir {
                        // The cell is at border.
                        // Walk around the contour to find all the neighbours.
                        self.walk_contour(x, z, i, ndir, src_reg, &mut reg.connections);
                    }
                }
            }
        }

        // Remove too small regions.
        let mut stack = Vec::with_capacity(32);
        let mut trace = Vec::with_capacity(32);
        for i in 0..nreg {
            let reg: &mut Region = &mut regions[i];
            if reg.id == RegionId::NONE || reg.id.intersects(RegionId::BORDER_REGION) {
                continue;
            }
            if reg.span_count == 0 {
                continue;
            }
            if reg.visited {
                continue;
            }

            // Count the total size of all the connected regions.
            // Also keep track of the regions connects to a tile border.
            let mut connects_to_border = false;
            let mut span_count = 0;
            stack.clear();
            trace.clear();

            reg.visited = true;
            stack.push(i);

            while let Some(ri) = stack.pop() {
                let c_reg = regions[ri].clone();
                span_count += c_reg.span_count;
                trace.push(ri);

                for connection in c_reg.connections.iter() {
                    if connection.intersects(RegionId::BORDER_REGION) {
                        connects_to_border = true;
                        continue;
                    }
                    let nei_reg = &mut regions[connection.bits() as usize];
                    if nei_reg.visited {
                        continue;
                    }
                    if nei_reg.id == RegionId::NONE
                        || nei_reg.id.intersects(RegionId::BORDER_REGION)
                    {
                        continue;
                    }
                    // Visit
                    stack.push(nei_reg.id.bits() as usize);
                    nei_reg.visited = true;
                }
            }

            // If the accumulated regions size is too small, remove it.
            // Do not remove areas which connect to tile borders
            // as their size cannot be estimated correctly and removing them
            // can potentially remove necessary areas.
            if (span_count as i64) < i64::from(min_region_area) && !connects_to_border {
                // Kill all visited regions.
                for &trace in trace.iter() {
                    regions[trace].span_count = 0;
                    regions[trace].id = RegionId::NONE;
                }
            }
        }

        // Merge too small regions to neighbour regions.
        loop {
            let mut merge_count = 0;
            for i in 0..nreg {
                let reg = regions[i].clone();
                if reg.id == RegionId::NONE || reg.id.intersects(RegionId::BORDER_REGION) {
                    continue;
                }
                if reg.overlap {
                    continue;
                }
                if reg.span_count == 0 {
                    continue;
                }

                // Check to see if the region should be merged
                if (reg.span_count as i64) > i64::from(merge_region_size)
                    && reg.is_region_connected_to_border()
                {
                    continue;
                }
                // Small region with more than 1 connection.
                // Or region which is not connected to a border at all.
                // Find smallest neighbour region that connects to this one.
                let mut smallest = usize::MAX;
                let mut merge_id = reg.id;
                for connection in reg.connections.iter() {
                    if connection.intersects(RegionId::BORDER_REGION) {
                        continue;
                    }
                    let mreg = regions[connection.bits() as usize].clone();
                    if mreg.id == RegionId::NONE
                        || mreg.id.intersects(RegionId::BORDER_REGION)
                        || mreg.overlap
                    {
                        continue;
                    }
                    if mreg.span_count < smallest
                        && reg.can_merge_with(&mreg)
                        && mreg.can_merge_with(&reg)
                    {
                        smallest = mreg.span_count;
                        merge_id = mreg.id;
                    }
                }
                // Found new id.
                if merge_id != reg.id {
                    let old_id = reg.id;
                    // Merge neighbours.
                    if regions[merge_id.bits() as usize].merge_regions(&reg) {
                        regions[i].span_count = 0;
                        regions[i].connections.clear();
                        // Fixup regions pointing to current region.
                        #[expect(clippy::needless_range_loop)]
                        for j in 0..nreg {
                            let reg = &mut regions[j];
                            if reg.id == RegionId::NONE
                                || reg.id.intersects(RegionId::BORDER_REGION)
                            {
                                continue;
                            }
                            // If another region was already merged into current region
                            // change the nid of the previous region too.
                            if reg.id == old_id {
                                reg.id = merge_id;
                            }
                            // Replace the current region with the new one if the
                            // current regions is neighbour.
                            reg.replace_neighbour(old_id, merge_id);
                        }
                        merge_count += 1;
                    }
                }
            }

            if merge_count == 0 {
                break;
            }
        }

        // Compress region IDs
        #[expect(clippy::needless_range_loop)]
        for i in 0..nreg {
            let reg = &mut regions[i];
            // Skip nil regions and external regions.
            reg.remap = !(reg.id == RegionId::NONE || reg.id.intersects(RegionId::BORDER_REGION));
        }

        let mut reg_id_gen = 0;
        for i in 0..nreg {
            if !regions[i].remap {
                continue;
            }
            let old_id = regions[i].id;
            reg_id_gen += 1;
            let new_id = RegionId::from(reg_id_gen);
            #[expect(clippy::needless_range_loop)]
            for j in i..nreg {
                if regions[j].id == old_id {
                    regions[j].id = new_id;
                    regions[j].remap = false;
                }
            }
        }
        self.max_region = RegionId::from(reg_id_gen);

        // Remap regions
        for reg in src_reg.iter_mut() {
            if !reg.intersects(RegionId::BORDER_REGION) {
                *reg = regions[reg.bits() as usize].id;
            }
        }

        // Return regions that we found to be overlapping.
        #[expect(clippy::needless_range_loop)]
        for i in 0..nreg {
            if regions[i].overlap {
                overlaps.push(regions[i].id);
            }
        }

        overlaps
    }

    fn walk_contour(
        &self,
        mut x: i32,
        mut z: i32,
        mut i: usize,
        mut dir: u8,
        src_reg: &[RegionId],
        connections: &mut Vec<RegionId>,
    ) {
        let start_dir = dir;
        let start_i = i;

        let ss = &self.spans[i];
        let mut current_region = RegionId::NONE;
        if let Some(con) = ss.con(dir) {
            let a_x = x + dir_offset_x(dir) as i32;
            let a_z = z + dir_offset_z(dir) as i32;
            let a_index = self.cell_at(a_x, a_z).index() as usize + con as usize;
            current_region = src_reg[a_index];
        }
        connections.push(current_region);

        // Jan: cool magic number lol
        for _ in 1..40_000 {
            let s = &self.spans[i];
            if self.is_solid_edge(src_reg, x, z, i, dir) {
                // Choose the edge corner
                let mut r = RegionId::NONE;
                if let Some(con) = s.con(dir) {
                    let a_x = x + dir_offset_x(dir) as i32;
                    let a_z = z + dir_offset_z(dir) as i32;
                    let a_index = self.cell_at(a_x, a_z).index() as usize + con as usize;
                    r = src_reg[a_index];
                }
                if r != current_region {
                    current_region = r;
                    connections.push(current_region);
                }
                // Rotate clockwise
                dir = (dir + 1) & 0x3;
            } else {
                let mut ni = None;
                let n_x = x + dir_offset_x(dir) as i32;
                let n_z = z + dir_offset_z(dir) as i32;
                if let Some(con) = s.con(dir) {
                    let n_c = self.cell_at(n_x, n_z);
                    let n_index = n_c.index() as usize + con as usize;
                    ni = Some(n_index);
                }
                if let Some(ni) = ni {
                    x = n_x;
                    z = n_z;
                    i = ni;
                    // Rotate counter-clockwise
                    dir = (dir + 3) & 0x3;
                } else {
                    // Should not happen
                    // Jan: Why not an error then??
                    return;
                }
            }
            if start_i == i && start_dir == dir {
                break;
            }
        }

        // Remove adjacent duplicates.
        if connections.len() > 1 {
            let mut j = 0;
            while j < connections.len() {
                let nj = (j + 1) % connections.len();
                if connections[j] == connections[nj] {
                    for k in j..connections.len() - 1 {
                        connections[k] = connections[k + 1];
                    }
                    connections.pop();
                } else {
                    j += 1;
                }
            }
        }
    }

    fn is_solid_edge(&self, src_reg: &[RegionId], x: i32, z: i32, i: usize, dir: u8) -> bool {
        let span = &self.spans[i];
        let mut r = RegionId::NONE;
        if let Some(con) = span.con(dir) {
            let a_x = x + dir_offset_x(dir) as i32;
            let a_z = z + dir_offset_z(dir) as i32;
            let a_index = self.cell_at(a_x, a_z).index() as usize + con as usize;
            r = src_reg[a_index];
        }
        r != src_reg[i]
    }

    fn flood_region(
        &self,
        entry: &LevelStackEntry,
        level: u16,
        region: RegionId,
        src_reg: &mut [RegionId],
        src_dist: &mut [u16],
        stack: &mut Vec<LevelStackEntry>,
    ) -> bool {
        // The caller checks that the entry has a span index.
        let i = entry.span_index().unwrap();
        let area = self.areas[i];

        // Flood fill mark region
        stack.clear();
        stack.push(entry.clone());
        src_reg[i] = region;
        src_dist[i] = 0;

        let level = level.saturating_sub(2);
        let mut count = 0;

        while let Some(back) = stack.pop() {
            let cx = back.x;
            let cz = back.z;
            let Some(ci) = back.span_index() else {
                // Jan: The original just accesses invalid memory here lol.
                continue;
            };

            let cs = &self.spans[ci];

            // Check if any of the neighbours already have a valid region set.
            let mut ar = RegionId::NONE;
            for dir in 0..4 {
                // 8 connected
                if let Some(con) = cs.con(dir) {
                    let a_x = cx + dir_offset_x(dir) as i32;
                    let a_z = cz + dir_offset_z(dir) as i32;
                    let a_index = self.cell_at(a_x, a_z).index() as usize + con as usize;
                    if self.areas[a_index] != area {
                        continue;
                    }
                    let nr = src_reg[a_index];
                    if nr.intersects(RegionId::BORDER_REGION) {
                        // Do not take borders into account.
                        continue;
                    }
                    if nr != RegionId::NONE && nr != region {
                        ar = nr;
                        break;
                    }

                    let a_span = &self.spans[a_index];

                    let dir2 = (dir + 1) & 0x3;
                    if let Some(con) = a_span.con(dir2) {
                        let a_x = a_x + dir_offset_x(dir2) as i32;
                        let a_z = a_z + dir_offset_z(dir2) as i32;
                        let a_index = self.cell_at(a_x, a_z).index() as usize + con as usize;
                        if self.areas[a_index] != area {
                            continue;
                        }
                        let nr = src_reg[a_index];
                        if nr != RegionId::NONE && nr != region {
                            ar = nr;
                            break;
                        }
                    }
                }
            }

            if ar != RegionId::NONE {
                src_reg[ci] = RegionId::NONE;
                continue;
            }

            count += 1;

            // Expand neighbours.
            for dir in 0..4 {
                let Some(con) = cs.con(dir) else {
                    continue;
                };
                let a_x = cx + dir_offset_x(dir) as i32;
                let a_z = cz + dir_offset_z(dir) as i32;
                let a_index = self.cell_at(a_x, a_z).index() as usize + con as usize;
                if self.areas[a_index] != area {
                    continue;
                }
                if self.dist[a_index] >= level && src_reg[a_index] == RegionId::NONE {
                    src_reg[a_index] = region;
                    src_dist[a_index] = 0;
                    stack.push(LevelStackEntry {
                        x: a_x,
                        z: a_z,
                        index: a_index as i32,
                    });
                }
            }
        }
        count > 0
    }

    fn paint_rect_region(
        &self,
        min_x: i32,
        max_x: i32,
        min_z: i32,
        max_z: i32,
        region: RegionId,
        src_reg: &mut [RegionId],
    ) {
        for z in min_z..max_z {
            for x in min_x..max_x {
                let cell = self.cell_at(x, z);
                let max_index = cell.index() as usize + cell.count() as usize;
                #[expect(clippy::needless_range_loop)]
                for i in cell.index() as usize..max_index {
                    if self.areas[i].is_walkable() {
                        src_reg[i] = region;
                    }
                }
            }
        }
    }

    fn sort_cells_by_level(
        &self,
        start_level: u16,
        src_reg: &mut [RegionId],
        nb_stacks: usize,
        stacks: &mut [Vec<LevelStackEntry>],
        log_levels_per_stack: u16,
    ) {
        let start_level = start_level >> log_levels_per_stack;
        for stack in stacks.iter_mut().take(nb_stacks) {
            stack.clear();
        }

        // put all cells in the level range into the appropriate stacks
        for z in 0..self.height {
            for x in 0..self.width {
                let cell = self.cell_at(x, z);
                let max_index = cell.index() as usize + cell.count() as usize;
                #[expect(clippy::needless_range_loop)]
                for i in cell.index() as usize..max_index {
                    if !self.areas[i].is_walkable() || src_reg[i] != RegionId::NONE {
                        continue;
                    }
                    let level = self.dist[i] >> log_levels_per_stack;
                    // Jan: The original can underflow here FYI
                    let s_id = start_level.saturating_sub(level);
                    if s_id >= nb_stacks as u16 {
                        continue;
                    }
                    stacks[s_id as usize].push(LevelStackEntry {
                        x,
                        z,
                        index: i as i32,
                    });
                }
            }
        }
    }

    fn expand_regions(
        &self,
        max_iter: u16,
        level: u16,
        src_reg: &mut [RegionId],
        src_dist: &mut [u16],
        (stack, dirty_entries): (&mut Vec<LevelStackEntry>, &mut Vec<DirtyEntry>),
        fill_stack: bool,
    ) {
        if fill_stack {
            // Find cells revealed by the raised level.
            stack.clear();
            for z in 0..self.height {
                for x in 0..self.width {
                    let cell = self.cell_at(x, z);
                    let max_index = cell.index() as usize + cell.count() as usize;
                    #[expect(clippy::needless_range_loop)]
                    for i in cell.index() as usize..max_index {
                        if self.dist[i] >= level
                            && src_reg[i] == RegionId::NONE
                            && self.areas[i].is_walkable()
                        {
                            stack.push(LevelStackEntry {
                                x,
                                z,
                                index: i as i32,
                            });
                        }
                    }
                }
            }
        } else {
            // use cells in the input stack
            // mark all cells which already have a region
            for entry in stack.iter_mut() {
                let Some(i) = entry.span_index() else {
                    continue;
                };
                if src_reg[i] != RegionId::NONE {
                    entry.index = -1;
                }
            }
        }

        // Retain capacity only within this region build. Each expansion starts
        // with the same empty logical buffer and applies writes after its scan.
        dirty_entries.clear();
        let mut iter = 0;
        // Jan: I don't think stack is ever made smaller? Is this just an `if` in disguise?
        while !stack.is_empty() {
            let mut failed = 0;
            dirty_entries.clear();

            for entry in stack.iter_mut() {
                let x = entry.x;
                let z = entry.z;
                let Some(i) = entry.span_index() else {
                    failed += 1;
                    continue;
                };

                let mut r = src_reg[i];
                let mut d2 = u16::MAX;
                let area = self.areas[i];
                let span = &self.spans[i];
                for dir in 0..4 {
                    let Some(con) = span.con(dir) else {
                        continue;
                    };
                    let a_x = x + dir_offset_x(dir) as i32;
                    let a_z = z + dir_offset_z(dir) as i32;
                    let a_index = self.cell_at(a_x, a_z).index() as usize + con as usize;
                    if self.areas[a_index] != area {
                        continue;
                    }
                    let a_region = src_reg[a_index];
                    let a_dist = src_dist[a_index].saturating_add(2);
                    if a_region != RegionId::NONE
                        && !a_region.intersects(RegionId::BORDER_REGION)
                        && a_dist < d2
                    {
                        r = a_region;
                        d2 = a_dist;
                    }
                }
                if r != RegionId::NONE {
                    // Mark as used
                    entry.index = -1;
                    dirty_entries.push(DirtyEntry {
                        index: i as i32,
                        region: r,
                        distance2: d2,
                    });
                } else {
                    failed += 1;
                }
            }
            // Copy entries that differ between src and dst to keep them in sync.
            for dirty_entry in dirty_entries.iter() {
                let index = dirty_entry.index as usize;
                src_reg[index] = dirty_entry.region;
                src_dist[index] = dirty_entry.distance2;
            }

            if failed == stack.len() {
                break;
            }

            if level > 0 {
                iter += 1;
                if iter >= max_iter {
                    break;
                }
            }
        }
    }
}

fn append_stacks(
    src_stack: &[LevelStackEntry],
    dst_stack: &mut Vec<LevelStackEntry>,
    src_region: &[RegionId],
) {
    for stack in src_stack.iter() {
        let Some(i) = stack.span_index() else {
            continue;
        };
        if src_region[i] != RegionId::NONE {
            continue;
        }
        dst_stack.push(stack.clone());
    }
}

// Every producer starts at a compact cell's 24-bit index and adds either a
// column offset below its 8-bit count or a connection offset at most 62.
// Copies preserve that bound (< 2^25). Native int indices therefore retain
// every reachable span, with -1 reserved for invalidated level-stack entries.
#[derive(Clone, Debug)]
struct LevelStackEntry {
    x: i32,
    z: i32,
    index: i32,
}

impl LevelStackEntry {
    #[inline]
    fn span_index(&self) -> Option<usize> {
        (self.index >= 0).then_some(self.index as usize)
    }
}

#[derive(Clone, Debug)]
struct DirtyEntry {
    index: i32,
    region: RegionId,
    distance2: u16,
}

#[derive(Debug, Clone)]
struct Region {
    span_count: usize,
    id: RegionId,
    area: AreaType,
    remap: bool,
    visited: bool,
    overlap: bool,
    connects_to_border: bool,
    y_min: u16,
    y_max: u16,
    connections: Vec<RegionId>,
    floors: Vec<RegionId>,
}
impl Region {
    fn new(id: RegionId) -> Self {
        Self {
            id,
            span_count: 0,
            area: AreaType::NOT_WALKABLE,
            remap: false,
            visited: false,
            overlap: false,
            connects_to_border: false,
            y_min: u16::MAX,
            y_max: u16::MIN,
            connections: Vec::new(),
            floors: Vec::new(),
        }
    }

    fn add_unique_floor_region(&mut self, floor_id: RegionId) {
        if self.floors.contains(&floor_id) {
            return;
        }
        self.floors.push(floor_id);
    }

    fn is_region_connected_to_border(&self) -> bool {
        // Region is connected to border if
        // one of the neighbours is null id.
        self.connections.contains(&RegionId::NONE)
    }

    fn can_merge_with(&self, other: &Self) -> bool {
        if self.area != other.area {
            return false;
        }
        let mut n = 0;
        for connection in self.connections.iter() {
            if *connection == other.id {
                n += 1;
            }
        }
        if n > 1 {
            return false;
        }
        for floor in self.floors.iter() {
            if floor == &other.id {
                return false;
            }
        }
        true
    }

    fn replace_neighbour(&mut self, old_id: RegionId, new_id: RegionId) {
        let mut nei_changed = false;
        for connection in self.connections.iter_mut() {
            if *connection == old_id {
                *connection = new_id;
                nei_changed = true;
            }
        }
        for floor in self.floors.iter_mut() {
            if *floor == old_id {
                *floor = new_id;
            }
        }
        if nei_changed {
            self.remove_adjacent_neighbours();
        }
    }

    fn remove_adjacent_neighbours(&mut self) {
        // Remove adjacent duplicates.
        let mut i = 0;
        while i < self.connections.len() && self.connections.len() > 1 {
            let ni = (i + 1) % self.connections.len();
            if self.connections[i] == self.connections[ni] {
                // Remove duplicate
                for j in i..self.connections.len() - 1 {
                    self.connections[j] = self.connections[j + 1];
                }
                self.connections.pop();
            } else {
                i += 1;
            }
        }
    }

    /// Implementation note: due to mutability rules, do the following outside the function when it returns `true`:
    /// ```ignore
    /// other.span_count = 0;
    /// other.connections.clear();
    /// ```
    fn merge_regions(&mut self, other: &Self) -> bool {
        let a_id = self.id;
        let b_id = other.id;

        // Duplicate current neighbourhood.
        let a_con = self.connections.clone();
        let b_con = &other.connections;

        // Find insertion point on A.
        let ins_a = a_con.iter().position(|con| *con == b_id);

        let Some(ins_a) = ins_a else {
            return false;
        };

        // Find insertion point on B.
        let ins_b = b_con.iter().position(|con| *con == a_id);

        let Some(ins_b) = ins_b else {
            return false;
        };

        // Merge neighbours.
        self.connections.clear();
        let ni = a_con.len();
        for i in 0..ni - 1 {
            let connection = a_con[(ins_a + 1 + i) % ni];
            self.connections.push(connection);
        }

        let ni = b_con.len();
        for i in 0..ni - 1 {
            let connection = b_con[(ins_b + 1 + i) % ni];
            self.connections.push(connection);
        }

        self.remove_adjacent_neighbours();
        for floor in other.floors.iter() {
            self.add_unique_floor_region(*floor);
        }
        self.span_count += other.span_count;

        true
    }
}
/// Error type for [`CompactHeightfield::build_regions`].
#[derive(Debug)]
pub enum BuildRegionsError {
    /// The region ID overflowed.
    RegionIdOverflow,
    /// A row needs more sweep entries than the native grid-sized buffer.
    SweepCapacityExceeded,
}

impl core::fmt::Display for BuildRegionsError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Self::RegionIdOverflow => "Region ID overflow",
            Self::SweepCapacityExceeded => "Partition row exceeds the native sweep capacity",
        })
    }
}
impl core::error::Error for BuildRegionsError {}

#[derive(Clone, Default)]
struct PartitionSweep {
    id: u16,
    samples: u16,
    neighbor: u16,
}

impl CompactHeightfield {
    /// Monotone partitioning (`rcBuildRegionsMonotone`). No distance field is needed.
    pub fn build_regions_monotone(
        &mut self,
        border_size: i32,
        min_region_area: i32,
        merge_region_area: i32,
    ) -> Result<(), BuildRegionsError> {
        self.build_regions_monotone_with_context(
            &Context::new(false),
            border_size,
            min_region_area,
            merge_region_area,
        )
    }

    pub fn build_regions_monotone_with_context(
        &mut self,
        context: &Context<'_>,
        border_size: i32,
        min_region_area: i32,
        merge_region_area: i32,
    ) -> Result<(), BuildRegionsError> {
        let _timer = context.scoped_timer(TimerLabel::BUILD_REGIONS);
        let (mut regions, id) = self.partition_sweeps(border_size)?;
        {
            let _filter = context.scoped_timer(TimerLabel::BUILD_REGIONS_FILTER);
            self.max_region = id;
            self.merge_and_filter_regions(min_region_area, merge_region_area, &mut regions);
        }
        for (span, region) in self.spans.iter_mut().zip(regions) {
            span.region = region;
        }
        Ok(())
    }

    /// Non-overlapping layer partitioning (`rcBuildLayerRegions`). Area boundaries
    /// divide initial sweeps, then connected regions on non-overlapping floors merge.
    pub fn build_layer_regions(
        &mut self,
        border_size: i32,
        min_region_area: i32,
    ) -> Result<(), BuildRegionsError> {
        self.build_layer_regions_with_context(&Context::new(false), border_size, min_region_area)
    }

    pub fn build_layer_regions_with_context(
        &mut self,
        context: &Context<'_>,
        border_size: i32,
        min_region_area: i32,
    ) -> Result<(), BuildRegionsError> {
        let _timer = context.scoped_timer(TimerLabel::BUILD_REGIONS);
        let (mut regions, id) = self.partition_sweeps(border_size)?;
        {
            let _filter = context.scoped_timer(TimerLabel::BUILD_REGIONS_FILTER);
            self.max_region = id;
            self.merge_and_filter_layer_regions(min_region_area, &mut regions);
        }
        for (span, region) in self.spans.iter_mut().zip(regions) {
            span.region = region;
        }
        Ok(())
    }

    // Shared sweep algorithm of the two upstream partitioning functions.
    fn partition_sweeps(
        &mut self,
        border: i32,
    ) -> Result<(Vec<RegionId>, RegionId), BuildRegionsError> {
        let mut regions = vec![RegionId::NONE; self.spans.len()];
        let mut id = 1u16;
        if border > 0 {
            let bw = self.width.min(border);
            let bh = self.height.min(border);
            for (minx, maxx, minz, maxz) in [
                (0, bw, 0, self.height),
                (self.width - bw, self.width, 0, self.height),
                (0, self.width, 0, bh),
                (0, self.width, self.height - bh, self.height),
            ] {
                self.paint_rect_region(
                    minx,
                    maxx,
                    minz,
                    maxz,
                    RegionId::from(id) | RegionId::BORDER_REGION,
                    &mut regions,
                );
                id += 1;
            }
        }
        self.border_size = border;
        let mut sweeps = vec![PartitionSweep::default(); self.width.max(self.height) as usize];
        // Native prev[] holds signed int counts; overflowing an increment is
        // outside its defined domain. Retain capacity only within this build.
        let mut previous = Vec::<i32>::new();
        for z in border..self.height.saturating_sub(border) {
            previous.clear();
            previous.resize(id as usize + 1, 0);
            let mut rid = 1u16;
            for x in border..self.width.saturating_sub(border) {
                for i in self.cell_at(x, z).index_range() {
                    if self.areas[i] == AreaType::NOT_WALKABLE {
                        continue;
                    }
                    let span = &self.spans[i];
                    let mut previd = 0;
                    if let Some(con) = span.con(0) {
                        let (_, _, ai) = self.con_indices(x, z, 0, con);
                        if !regions[ai].intersects(RegionId::BORDER_REGION)
                            && self.areas[i] == self.areas[ai]
                        {
                            previd = regions[ai].bits();
                        }
                    }
                    if previd == 0 {
                        previd = rid;
                        rid = rid
                            .checked_add(1)
                            .ok_or(BuildRegionsError::RegionIdOverflow)?;
                        let Some(sweep) = sweeps.get_mut(previd as usize) else {
                            return Err(BuildRegionsError::SweepCapacityExceeded);
                        };
                        *sweep = PartitionSweep::default();
                    }
                    if let Some(con) = span.con(3) {
                        let (_, _, ai) = self.con_indices(x, z, 3, con);
                        let nr = regions[ai];
                        if nr != RegionId::NONE
                            && !nr.intersects(RegionId::BORDER_REGION)
                            && self.areas[i] == self.areas[ai]
                        {
                            let sweep = &mut sweeps[previd as usize];
                            if sweep.neighbor == 0 || sweep.neighbor == nr.bits() {
                                sweep.neighbor = nr.bits();
                                sweep.samples = sweep.samples.wrapping_add(1);
                                previous[nr.bits() as usize] += 1;
                            } else {
                                sweep.neighbor = u16::MAX;
                            }
                        }
                    }
                    regions[i] = RegionId::from(previd);
                }
            }
            for sweep in &mut sweeps[1..rid as usize] {
                if sweep.neighbor != 0
                    && sweep.neighbor != u16::MAX
                    && previous[sweep.neighbor as usize] == i32::from(sweep.samples)
                {
                    sweep.id = sweep.neighbor;
                } else {
                    sweep.id = id;
                    id = id
                        .checked_add(1)
                        .ok_or(BuildRegionsError::RegionIdOverflow)?;
                }
            }
            for x in border..self.width.saturating_sub(border) {
                for i in self.cell_at(x, z).index_range() {
                    let r = regions[i].bits();
                    if r > 0 && r < rid {
                        regions[i] = RegionId::from(sweeps[r as usize].id);
                    }
                }
            }
        }
        Ok((regions, RegionId::from(id)))
    }

    fn merge_and_filter_layer_regions(&mut self, min_area: i32, src: &mut [RegionId]) {
        use alloc::collections::VecDeque;
        let count = self.max_region.bits() as usize + 1;
        let mut regions: Vec<_> = (0..count)
            .map(|id| Region::new(RegionId::from(id as u16)))
            .collect();
        // A compact cell's count is an eight-bit field. Reuse bounded column
        // scratch while preserving the native span and overlap-pair order.
        let mut layers = [0; u8::MAX as usize];
        for z in 0..self.height {
            for x in 0..self.width {
                let mut layer_count = 0;
                for i in self.cell_at(x, z).index_range() {
                    let ri = src[i].bits() as usize;
                    if ri == 0 || ri >= count {
                        continue;
                    }
                    let reg = &mut regions[ri];
                    let span = &self.spans[i];
                    reg.span_count += 1;
                    reg.y_min = reg.y_min.min(span.y);
                    reg.y_max = reg.y_max.max(span.y);
                    layers[layer_count] = ri;
                    layer_count += 1;
                    for dir in 0..4 {
                        if let Some(con) = span.con(dir) {
                            let (_, _, ai) = self.con_indices(x, z, dir, con);
                            let neighbor = src[ai];
                            let ni = neighbor.bits() as usize;
                            if ni > 0
                                && ni < count
                                && ni != ri
                                && !reg.connections.contains(&neighbor)
                            {
                                reg.connections.push(neighbor);
                            }
                            if neighbor.intersects(RegionId::BORDER_REGION) {
                                reg.connects_to_border = true;
                            }
                        }
                    }
                }
                let layers = &layers[..layer_count];
                for (i, &ri) in layers.iter().enumerate() {
                    for &rj in &layers[i + 1..] {
                        if ri != rj {
                            regions[ri].add_unique_floor_region(RegionId::from(rj as u16));
                            regions[rj].add_unique_floor_region(RegionId::from(ri as u16));
                        }
                    }
                }
            }
        }
        for reg in &mut regions {
            reg.id = RegionId::NONE;
        }
        let mut layer_id = 1u16;
        for root in 1..count {
            if regions[root].id != RegionId::NONE {
                continue;
            }
            regions[root].id = RegionId::from(layer_id);
            let mut queue = VecDeque::from([root]);
            while let Some(current) = queue.pop_front() {
                for j in 0..regions[current].connections.len() {
                    let neighbor = regions[current].connections[j];
                    let ni = neighbor.bits() as usize;
                    if regions[ni].id != RegionId::NONE || regions[root].floors.contains(&neighbor)
                    {
                        continue;
                    }
                    queue.push_back(ni);
                    regions[ni].id = RegionId::from(layer_id);
                    for floor_index in 0..regions[ni].floors.len() {
                        let floor = regions[ni].floors[floor_index];
                        regions[root].add_unique_floor_region(floor);
                    }
                    regions[root].y_min = regions[root].y_min.min(regions[ni].y_min);
                    regions[root].y_max = regions[root].y_max.max(regions[ni].y_max);
                    regions[root].span_count += regions[ni].span_count;
                    regions[ni].span_count = 0;
                    regions[root].connects_to_border |= regions[ni].connects_to_border;
                }
            }
            layer_id = layer_id.wrapping_add(1);
        }
        for i in 0..count {
            if regions[i].span_count > 0
                && (regions[i].span_count as i64) < i64::from(min_area)
                && !regions[i].connects_to_border
            {
                let id = regions[i].id;
                for reg in &mut regions {
                    if reg.id == id {
                        reg.id = RegionId::NONE;
                    }
                }
            }
        }
        for reg in &mut regions {
            reg.remap = reg.id != RegionId::NONE && !reg.id.intersects(RegionId::BORDER_REGION);
        }
        let mut id = 0u16;
        for i in 0..count {
            if !regions[i].remap {
                continue;
            }
            let old = regions[i].id;
            id = id.wrapping_add(1);
            for reg in &mut regions[i..] {
                if reg.id == old {
                    reg.id = RegionId::from(id);
                    reg.remap = false;
                }
            }
        }
        self.max_region = RegionId::from(id);
        for r in src {
            if !r.intersects(RegionId::BORDER_REGION) {
                *r = regions[r.bits() as usize].id;
            }
        }
    }
}
