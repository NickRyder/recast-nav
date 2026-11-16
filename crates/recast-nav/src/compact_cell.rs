use core::ops::Range;

/// Provides information on the content of a cell column in a [`CompactHeightfield`].
///
/// See the rcCompactHeightfield documentation for an example of how compact cells are used to iterate the heightfield.
///
/// Useful instances of this type can only by obtained from a [`CompactHeightfield`].
///
/// [`CompactHeightfield`]: crate::compact_heightfield::CompactHeightfield
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[repr(transparent)]
pub struct CompactCell {
    // Native 24-bit index followed by its 8-bit count.
    data: u32,
}

impl CompactCell {
    /// Returns the index of the first span in the column.
    #[inline]
    pub fn index(&self) -> u32 {
        self.data & 0x00ff_ffff
    }

    /// Returns the number of spans in the column.
    #[inline]
    pub fn count(&self) -> u8 {
        (self.data >> 24) as u8
    }

    /// Sets the index of the first span in the column.
    #[inline]
    pub fn set_index(&mut self, index: u32) {
        self.data = (self.data & 0xff00_0000) | (index & 0x00ff_ffff);
    }

    /// Sets the number of spans in the column.
    #[inline]
    pub fn set_count(&mut self, count: u8) {
        self.set_count_raw(u32::from(count));
    }

    /// Assigns the native 8-bit count field, truncating higher bits.
    #[inline]
    pub fn set_count_raw(&mut self, count: u32) {
        self.data = (self.data & 0x00ff_ffff) | (count << 24);
    }

    /// Increments the number of spans in the column by 1.
    #[inline]
    pub fn inc_count(&mut self) {
        self.data = self.data.wrapping_add(1 << 24);
    }

    /// Returns a range over the indices of spans in the cell column.
    #[inline]
    pub fn index_range(&self) -> Range<usize> {
        let start = self.index() as usize;
        start..start + usize::from(self.count())
    }
}
