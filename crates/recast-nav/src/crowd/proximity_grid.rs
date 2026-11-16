// Altered Rust translation of DetourProximityGrid.cpp, RecastNavigation v1.6.0.
// Copyright (c) 2009-2010 Mikko Mononen. Zlib: vendor/recastnavigation/License.txt.
use crate::nav::{FAILURE, INVALID_PARAM, OUT_OF_MEMORY, SUCCESS};
#[derive(Clone, Copy, Debug, Default)]
struct Item {
    id: u16,
    x: i16,
    y: i16,
    next: u16,
}
#[derive(Clone, Debug)]
pub struct ProximityGrid {
    cell_size: f32,
    inv_cell_size: f32,
    pool: Vec<Item>,
    pool_head: usize,
    buckets: Vec<u16>,
    bounds: [i32; 4],
}
// Both public output forms share one traversal without forcing a heap buffer.
trait IdBuffer {
    fn clear(&mut self);
    fn len(&self) -> usize;
    fn contains(&self, id: u16) -> bool;
    fn push(&mut self, id: u16);
}
impl IdBuffer for Vec<u16> {
    #[inline]
    fn clear(&mut self) {
        Vec::clear(self);
    }
    #[inline]
    fn len(&self) -> usize {
        Vec::len(self)
    }
    #[inline]
    fn contains(&self, id: u16) -> bool {
        self.as_slice().contains(&id)
    }
    #[inline]
    fn push(&mut self, id: u16) {
        Vec::push(self, id);
    }
}
struct SliceIds<'a> {
    values: &'a mut [u16],
    count: usize,
}
impl IdBuffer for SliceIds<'_> {
    #[inline]
    fn clear(&mut self) {
        self.count = 0;
    }
    #[inline]
    fn len(&self) -> usize {
        self.count
    }
    #[inline]
    fn contains(&self, id: u16) -> bool {
        self.values[..self.count].contains(&id)
    }
    #[inline]
    fn push(&mut self, id: u16) {
        self.values[self.count] = id;
        self.count += 1;
    }
}
fn coordinate(value: f32) -> Result<i32, u32> {
    let value = value.floor();
    if value.is_finite() && (-2147483648.0..2147483648.0).contains(&value) {
        Ok(value as i32)
    } else {
        Err(FAILURE | INVALID_PARAM)
    }
}
impl ProximityGrid {
    pub fn new(pool_size: usize, cell_size: f32) -> Result<Self, u32> {
        if pool_size == 0
            || pool_size > i32::MAX as usize
            || cell_size.partial_cmp(&0.) != Some(std::cmp::Ordering::Greater)
        {
            return Err(FAILURE | INVALID_PARAM);
        }
        // Native stores the rounded bucket count in a signed int. Its first
        // allocation cannot represent a positive size beyond this boundary.
        if pool_size > (1 << 30) {
            return Err(FAILURE | OUT_OF_MEMORY);
        }
        let mut pool = Vec::new();
        pool.try_reserve_exact(pool_size)
            .map_err(|_| FAILURE | OUT_OF_MEMORY)?;
        pool.resize(pool_size, Item::default());
        let bucket_count = pool_size.next_power_of_two();
        let mut buckets = Vec::new();
        buckets
            .try_reserve_exact(bucket_count)
            .map_err(|_| FAILURE | OUT_OF_MEMORY)?;
        buckets.resize(bucket_count, u16::MAX);
        Ok(Self {
            cell_size,
            inv_cell_size: 1. / cell_size,
            pool,
            pool_head: 0,
            buckets,
            bounds: [65535, 65535, -65535, -65535],
        })
    }
    /// Successful reinitialization replaces storage; allocation failure retains
    /// the previous Rust grid rather than a partially initialized native object.
    pub fn reinitialize(&mut self, pool_size: usize, cell_size: f32) -> Result<(), u32> {
        *self = Self::new(pool_size, cell_size)?;
        Ok(())
    }
    pub fn clear(&mut self) {
        self.buckets.fill(u16::MAX);
        self.pool_head = 0;
        self.bounds = [65535, 65535, -65535, -65535];
    }
    pub fn bounds(&self) -> [i32; 4] {
        self.bounds
    }
    pub fn cell_size(&self) -> f32 {
        self.cell_size
    }
    fn hash(&self, x: i32, y: i32) -> usize {
        ((x.wrapping_mul(73856093) ^ y.wrapping_mul(19349663)) as u32 as usize)
            & (self.buckets.len() - 1)
    }
    fn cells(&self, bounds: [[f32; 2]; 2]) -> Result<[[i32; 2]; 2], u32> {
        Ok([
            [
                coordinate(bounds[0][0] * self.inv_cell_size)?,
                coordinate(bounds[0][1] * self.inv_cell_size)?,
            ],
            [
                coordinate(bounds[1][0] * self.inv_cell_size)?,
                coordinate(bounds[1][1] * self.inv_cell_size)?,
            ],
        ])
    }
    /// Pool exhaustion silently drops entries, but the declared bounds still
    /// expand. IDs and stored cell coordinates retain native 16-bit conversion.
    pub fn add_item(&mut self, id: u16, bounds: [[f32; 2]; 2]) -> u32 {
        let [[minx, miny], [maxx, maxy]] = match self.cells(bounds) {
            Ok(c) => c,
            Err(e) => return e,
        };
        self.bounds = [
            self.bounds[0].min(minx),
            self.bounds[1].min(miny),
            self.bounds[2].max(maxx),
            self.bounds[3].max(maxy),
        ];
        for y in miny..=maxy {
            for x in minx..=maxx {
                if self.pool_head < self.pool.len() {
                    let h = self.hash(x, y);
                    let index = self.pool_head as u16;
                    self.pool_head += 1;
                    self.pool[index as usize] = Item {
                        id,
                        x: x as i16,
                        y: y as i16,
                        next: self.buckets[h],
                    };
                    self.buckets[h] = index;
                }
            }
        }
        SUCCESS
    }
    /// Returns unique IDs in cell and hash-chain order. Invalid conversions or
    /// a cyclic chain from overflowing native 16-bit pool indices are checked.
    pub fn query_items(&self, bounds: [[f32; 2]; 2], capacity: usize) -> Result<Vec<u16>, u32> {
        let mut ids = vec![];
        self.query_items_into(bounds, capacity, &mut ids)?;
        Ok(ids)
    }
    /// Reuses caller-owned ID storage, preserving native traversal and uniqueness.
    pub fn query_items_into(
        &self,
        bounds: [[f32; 2]; 2],
        capacity: usize,
        ids: &mut Vec<u16>,
    ) -> Result<(), u32> {
        self.query_items_buffer(bounds, capacity, ids)
    }
    /// Write ordered unique IDs into a native-style caller buffer. Returns the
    /// used prefix length; all entries after it retain their previous values.
    pub fn query_items_slice(&self, bounds: [[f32; 2]; 2], ids: &mut [u16]) -> Result<usize, u32> {
        let capacity = ids.len();
        let mut output = SliceIds {
            values: ids,
            count: 0,
        };
        self.query_items_buffer(bounds, capacity, &mut output)?;
        Ok(output.count)
    }
    fn query_items_buffer(
        &self,
        bounds: [[f32; 2]; 2],
        capacity: usize,
        ids: &mut impl IdBuffer,
    ) -> Result<(), u32> {
        let [[minx, miny], [maxx, maxy]] = self.cells(bounds)?;
        ids.clear();
        let mut seen_low_ids = 0_u64;
        for y in miny..=maxy {
            for x in minx..=maxx {
                let mut at = self.buckets[self.hash(x, y)];
                let mut steps = 0;
                while at != u16::MAX {
                    if steps >= self.pool.len() {
                        return Err(FAILURE | INVALID_PARAM);
                    }
                    steps += 1;
                    let item = &self.pool[at as usize];
                    if i32::from(item.x) == x && i32::from(item.y) == y {
                        // Crowd agent indices commonly fit one word. Larger IDs
                        // retain the same ordered result-vector lookup.
                        let duplicate = if item.id < 64 {
                            let bit = 1_u64 << item.id;
                            let duplicate = seen_low_ids & bit != 0;
                            seen_low_ids |= bit;
                            duplicate
                        } else {
                            ids.contains(item.id)
                        };
                        if !duplicate {
                            if ids.len() >= capacity {
                                return Ok(());
                            }
                            ids.push(item.id);
                        }
                    }
                    at = item.next;
                }
            }
        }
        Ok(())
    }
    pub fn item_count_at(&self, x: i32, y: i32) -> Result<usize, u32> {
        let mut at = self.buckets[self.hash(x, y)];
        let mut count = 0;
        let mut steps = 0;
        while at != u16::MAX {
            if steps >= self.pool.len() {
                return Err(FAILURE | INVALID_PARAM);
            }
            steps += 1;
            let item = &self.pool[at as usize];
            if i32::from(item.x) == x && i32::from(item.y) == y {
                count += 1;
            }
            at = item.next;
        }
        Ok(count)
    }
}
