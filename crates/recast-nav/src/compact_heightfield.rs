use crate::{
    Aabb3d,
    compact_cell::CompactCell,
    compact_span::CompactSpan,
    context::{Context, LogCategory, TimerLabel},
    heightfield::Heightfield,
    math::{dir_offset_x, dir_offset_z},
    region::RegionId,
    span::AreaType,
};
use alloc::vec::Vec;

/// A compact, static heightfield representing unobstructed space.
///
/// For this type of heightfield, the spans represent the open (unobstructed) space above the solid surfaces of a voxel field.
/// It is usually created from a [`Heightfield`] object. Data is stored in a compact, efficient manner,
/// but the structure is not condusive to adding and removing spans.
///
/// The standard process for buidling a compact heightfield is to build it using [`Heightfield::into_compact`], then run it through the various helper functions to generate neighbor and region data.
///
/// Connected neighbor spans form non-overlapping surfaces. When neighbor information is generated, spans will include data that can be used to locate axis-neighbors. Axis-neighbors are connected spans that are offset from the current cell column as follows:
///
/// ```txt
/// Direction 0 = (-1, 0)
/// Direction 1 = (0, 1)
/// Direction 2 = (1, 0)
/// Direction 3 = (0, -1)
/// ```
/// Example of iterating and inspecting spans, including connected neighbors:
///
/// ```rust
/// # use recast_nav::*;
/// # let chf = CompactHeightfield::default();
/// // Where chf is an instance of a CompactHeightfield.
///
/// let cs = chf.cell_size;
/// let ch = chf.cell_height;
///
/// for z in 0..chf.height {
///     for x in 0..chf.width {
///         // Deriving the minimum corner of the grid location.
///         let fx = chf.aabb.min.x + x as f32 * cs;
///         let fz = chf.aabb.min.z + z as f32 * cs;
///         println!("Corners: {fx},  {fz}");
///
///         // Get the cell for the grid location then iterate
///         // up the column.
///         let c = chf.cell_at(x, z);
///         for i in c.index_range() {
///             let s = &chf.spans[i];
///
///             // Deriving the minimum (floor) of the span.
///             let fy = chf.aabb.min.y + (s.y + 1) as f32 * ch;
///             println!("Span floor: {fy}");
///
///             // Testing the area assignment of the span.
///             if chf.areas[i] == AreaType::DEFAULT_WALKABLE {
///                 // The span is in the default 'walkable area'.
///             } else if chf.areas[i] == AreaType::NOT_WALKABLE {
///                 // The surface is not considered walkable.
///                 // E.g. It was filtered out during the build processes.
///             } else {
///                 // Do something. (Only applicable for custom build
///                 // build processes.)
///             }
///
///             // Iterating the connected axis-neighbor spans.
///             for dir in 0..4 {
///                 if let Some(con) = s.con(dir) {
///                     // There is a neighbor in this direction.
///                     let (_nx, _ny, ni) = chf.con_indices(x as i32, z as i32, dir, con);
///                     let ns = &chf.spans[ni];
///                     // Do something with the neighbor span.
///                     println!("Neighbor span: {ns:?}");
///                 }
///             }
///         }
///     }
/// }
/// ```
#[derive(Debug, Default, Clone)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct CompactHeightfield {
    /// The width of the heightfield along the x-axis in cell units
    pub width: i32,
    /// The height of the heightfield along the z-axis in cell units
    pub height: i32,
    /// The walkable height used during the build of the field
    /// (See: [`Config::walkable_height`](crate::Config::walkable_height))
    pub walkable_height: i32,
    /// The walkable climb used during the build of the field.
    /// (See: [`Config::walkable_climb`](crate::Config::walkable_climb))
    pub walkable_climb: i32,
    /// The AABB border size used during the build of the field.
    /// (See: [`Config::border_size`](crate::Config::border_size))
    pub border_size: i32,
    /// The maximum distance value of any span within the field.
    pub max_distance: u16,
    /// The maximum region id of any span within the field.
    pub max_region: RegionId,
    /// The AABB of the heightfield
    pub aabb: Aabb3d,
    /// The size of each cell on the xz-plane
    pub cell_size: f32,
    /// The size of each cell along the y-axis
    pub cell_height: f32,
    /// The cells in the heightfield [Size: `width * height`]
    pub cells: Vec<CompactCell>,
    /// All walkable spans in the heightfield
    pub spans: Vec<CompactSpan>,
    /// Vector containing border distance data. [Size: `spans.len()`]
    pub dist: Vec<u16>,
    /// Vector containing area type data. [Size: `spans.len()`]
    pub areas: Vec<AreaType>,
}

impl Heightfield {
    const MAX_HEIGHT: u16 = u16::MAX;

    /// Builds a compact heightfield from a heightfield.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid native storage or failed allocation.
    /// Excess connection layers retain their output and succeed, as in Recast.
    pub fn into_compact(
        self,
        walkable_height: i32,
        walkable_climb: i32,
    ) -> Result<CompactHeightfield, CompactHeightfieldError> {
        self.into_compact_with_context(&Context::new(false), walkable_height, walkable_climb)
    }

    pub fn into_compact_with_context(
        self,
        context: &Context<'_>,
        walkable_height: i32,
        walkable_climb: i32,
    ) -> Result<CompactHeightfield, CompactHeightfieldError> {
        let _timer = context.scoped_timer(TimerLabel::BUILD_COMPACTHEIGHTFIELD);
        let column_count = self
            .width
            .checked_mul(self.height)
            .and_then(|count| usize::try_from(count).ok())
            .ok_or(CompactHeightfieldError::InvalidStorage)?;
        let columns = self
            .spans
            .get(..column_count)
            .ok_or(CompactHeightfieldError::InvalidStorage)?;
        let mut walkable_span_count = 0usize;
        for &head in columns {
            let mut cursor = head;
            while let Some(key) = cursor {
                let span = self
                    .allocated_spans
                    .get(key)
                    .ok_or(CompactHeightfieldError::InvalidStorage)?;
                if span.area().is_walkable() {
                    walkable_span_count += 1;
                    if walkable_span_count > i32::MAX as usize {
                        return Err(CompactHeightfieldError::InvalidStorage);
                    }
                }
                cursor = span.next;
            }
        }

        let mut compact_heightfield = CompactHeightfield {
            width: self.width,
            height: self.height,
            walkable_height,
            walkable_climb,
            border_size: 0,
            aabb: self.aabb,
            max_distance: 0,
            max_region: RegionId::NONE,
            cell_size: self.cell_size,
            cell_height: self.cell_height,
            cells: compact_buffer(context, column_count, CompactCell::default(), "chf.cells")?,
            spans: compact_buffer(
                context,
                walkable_span_count,
                CompactSpan::default(),
                "chf.spans",
            )?,
            dist: vec![],
            areas: compact_buffer(
                context,
                walkable_span_count,
                AreaType::NOT_WALKABLE,
                "chf.areas",
            )?,
        };
        #[cfg(feature = "fma")]
        {
            compact_heightfield.aabb.max.y = (walkable_height as f32).mul_add(
                compact_heightfield.cell_height,
                compact_heightfield.aabb.max.y,
            );
        }
        #[cfg(not(feature = "fma"))]
        {
            compact_heightfield.aabb.max.y +=
                walkable_height as f32 * compact_heightfield.cell_height;
        }

        let mut cell_index = 0_usize;
        // Fill in cells and spans
        for (column_index, &head) in columns.iter().enumerate() {
            let Some(span_key) = head else {
                // If there are no spans at this cell, just leave the data to index=0, count=0.
                continue;
            };
            let mut span_key_iter = Some(span_key);

            let cell = &mut compact_heightfield.cells[column_index];
            cell.set_index(cell_index as u32);
            cell.set_count(0);

            while let Some(span_key) = span_key_iter {
                let span = self.span(span_key);
                span_key_iter = span.next;
                if !span.area().is_walkable() {
                    continue;
                }
                let bot = span.max();
                let top = span
                    .next
                    .map(|span| self.span(span).min())
                    .unwrap_or(Self::MAX_HEIGHT);
                compact_heightfield.spans[cell_index].y = bot.clamp(0, Self::MAX_HEIGHT);
                let height = (top.saturating_sub(bot)).min(u8::MAX.into()) as u8;
                compact_heightfield.spans[cell_index].set_height(height);
                compact_heightfield.areas[cell_index] = span.area();
                cell_index += 1;
                cell.inc_count();
            }
        }

        // Find neighbour connections
        const MAX_LAYERS: u8 = CompactSpan::NOT_CONNECTED - 1;
        let mut max_layer_index = 0_u32;
        for z in 0..self.height {
            for x in 0..self.width {
                let column_index =
                    usize::try_from(i64::from(x) + i64::from(z) * i64::from(self.width))
                        .unwrap_or(usize::MAX);
                let cell = compact_heightfield.cells[column_index];
                if cell.count() == 0 {
                    continue;
                }
                // Linking changes only connection bits, so column ranges and
                // span floor/ceiling values remain fixed throughout this pass.
                let neighbors = core::array::from_fn::<_, 4, _>(|dir| {
                    let nx = x + i32::from(dir_offset_x(dir as u8));
                    let nz = z + i32::from(dir_offset_z(dir as u8));
                    if self.contains(nx, nz) {
                        compact_heightfield.cells[self.column_index(nx, nz)].index_range()
                    } else {
                        0..0
                    }
                });
                for i in cell.index_range() {
                    let floor = compact_heightfield.spans[i].y;
                    let ceiling =
                        i32::from(floor) + i32::from(compact_heightfield.spans[i].height());
                    let mut connections = 0x00ff_ffff;
                    for (dir, range) in neighbors.iter().enumerate() {
                        for (layer_index, neighbor) in
                            compact_heightfield.spans[range.clone()].iter().enumerate()
                        {
                            let bot = floor.max(neighbor.y);
                            let top =
                                ceiling.min(i32::from(neighbor.y) + i32::from(neighbor.height()));
                            if top - i32::from(bot) < walkable_height
                                || (i32::from(neighbor.y) - i32::from(floor)).abs() > walkable_climb
                            {
                                continue;
                            }
                            if layer_index > usize::from(MAX_LAYERS) {
                                max_layer_index = max_layer_index.max(layer_index as u32);
                                continue;
                            }
                            connections = (connections & !(0x3f << (dir * 6)))
                                | ((layer_index as u32) << (dir * 6));
                            break;
                        }
                    }
                    compact_heightfield.spans[i].set_connections_raw(connections);
                }
            }
        }
        if max_layer_index > MAX_LAYERS as u32 {
            let _ = context.log_format(
                LogCategory::ERROR,
                format_args!(
                    "rcBuildCompactHeightfield: Heightfield has too many layers {} (max: {})",
                    max_layer_index, MAX_LAYERS,
                ),
            );
        }
        Ok(compact_heightfield)
    }
}

impl CompactHeightfield {
    #[inline]
    pub(crate) fn column_index(&self, x: i32, z: i32) -> usize {
        usize::try_from(i64::from(x) + i64::from(z) * i64::from(self.width)).unwrap_or(usize::MAX)
    }

    /// Returns the cell at the given coordinates. Returns `None` if the coordinates are invalid.
    #[inline]
    pub fn get_cell_at(&self, x: i32, z: i32) -> Option<&CompactCell> {
        let Some(cell) = self.cells.get(self.column_index(x, z)) else {
            // Invalid coordinates
            return None;
        };
        Some(cell)
    }

    /// Returns the cell at the given coordinates. Panics if the coordinates are invalid.
    #[inline]
    pub fn cell_at(&self, x: i32, z: i32) -> &CompactCell {
        &self.cells[self.column_index(x, z)]
    }

    /// Returns the cell mutably at the given coordinates. Returns `None` if the coordinates are invalid.
    #[inline]
    pub fn get_cell_at_mut(&mut self, x: i32, z: i32) -> Option<&mut CompactCell> {
        let index = self.column_index(x, z);
        let Some(cell) = self.cells.get_mut(index) else {
            // Invalid coordinates
            return None;
        };
        Some(cell)
    }

    /// Returns the cell mutably at the given coordinates. Panics if the coordinates are invalid.
    #[inline]
    pub fn cell_at_mut(&mut self, x: i32, z: i32) -> &mut CompactCell {
        let index = self.column_index(x, z);
        &mut self.cells[index]
    }

    /// Given a span at the indices `(x, z)`, a direction `dir`, and a connection `con`, returns:
    /// - The x index of the neighbor span
    /// - The z index of the neighbor span
    /// - The index of the neighbor span in [`Self::spans`]
    #[inline]
    pub fn con_indices(&self, x: i32, z: i32, dir: u8, con: u8) -> (i32, i32, usize) {
        let a_x = x + dir_offset_x(dir) as i32;
        let a_z = z + dir_offset_z(dir) as i32;
        let cell_index = (a_x + a_z * self.width) as usize;
        let a_i = self.cells[cell_index].index() as usize + con as usize;
        (a_x, a_z, a_i)
    }
}

/// Errors that can occur when building a compact heightfield.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CompactHeightfieldError {
    InvalidStorage,
    Allocation { array: &'static str, count: usize },
}

impl core::fmt::Display for CompactHeightfieldError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidStorage => {
                f.write_str("invalid compact heightfield storage or signed index domain")
            }
            Self::Allocation { array, count } => write!(
                f,
                "compact heightfield allocation failed for {array} ({count})"
            ),
        }
    }
}
impl core::error::Error for CompactHeightfieldError {}

fn compact_buffer<T: Clone>(
    context: &Context<'_>,
    count: usize,
    value: T,
    array: &'static str,
) -> Result<Vec<T>, CompactHeightfieldError> {
    let mut buffer = Vec::new();
    buffer.try_reserve_exact(count).map_err(|_| {
        let _ = context.log_format(
            LogCategory::ERROR,
            format_args!(
                "rcBuildCompactHeightfield: Out of memory '{}' ({})",
                array, count
            ),
        );
        CompactHeightfieldError::Allocation { array, count }
    })?;
    buffer.resize(count, value);
    Ok(buffer)
}
