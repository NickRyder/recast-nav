use crate::region::RegionId;

/// Represents a span of unobstructed space within a compact heightfield.
///
/// The span represents open, unobstructed space within a compact heightfield column.
/// See the [`CompactHeightfield`] documentation for an example of iterating spans and searching span connections.
///
/// Useful instances of this type can only by obtained from a [`CompactHeightfield`] object.
///
/// [`CompactHeightfield`]: crate::CompactHeightfield
#[derive(Debug, Clone, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct CompactSpan {
    /// The lower extent of the span. (Measured from the heightfield's base.)
    pub y: u16,
    /// The id of the region the span belongs to. (Or [`RegionId::NONE`] if not in a region.)
    pub region: RegionId,
    /// 24 bits: packed neighbor connection data
    /// 8 bits: the height of the span, measured from [`Self::y`]
    pub data: u32,
}

impl CompactSpan {
    pub(crate) const NOT_CONNECTED: u8 = 0x3f;

    /// Sets the neighbor connection data for the given direction.
    /// `None` if the neighbor is not connected.
    pub fn set_con(&mut self, direction: u8, neighbor: impl Into<Option<u8>>) {
        self.set_con_raw(
            i32::from(direction),
            i32::from(neighbor.into().unwrap_or(Self::NOT_CONNECTED)),
        );
    }

    /// Returns the neighbor connection data for the given direction.
    /// `None` if the neighbor is not connected.
    pub fn con(&self, direction: u8) -> Option<u8> {
        let value = self.con_raw(i32::from(direction))?;
        if value == Self::NOT_CONNECTED {
            None
        } else {
            Some(value)
        }
    }

    /// The native 24-bit connection field, excluding the height byte.
    pub fn connections_raw(&self) -> u32 {
        self.data & 0x00ff_ffff
    }

    /// Assigns the native connection bitfield, preserving the height byte.
    pub fn set_connections_raw(&mut self, connections: u32) {
        self.data = (self.data & 0xff00_0000) | (connections & 0x00ff_ffff);
    }

    /// Native `rcSetCon`, including unsigned direction multiplication and
    /// truncation back to the 24-bit field. Returns false without mutation when
    /// the native signed mask shift would be undefined.
    pub fn set_con_raw(&mut self, direction: i32, neighbor: i32) -> bool {
        let shift = (direction as u32).wrapping_mul(6);
        if shift > 26 {
            return false;
        }
        let value = (neighbor as u32) & 0x3f;
        self.set_connections_raw((self.connections_raw() & !(0x3f << shift)) | (value << shift));
        true
    }

    /// Native `rcGetCon`. Returns the raw value 63 for a missing connection,
    /// or `None` when the native unsigned right shift would be undefined.
    pub fn con_raw(&self, direction: i32) -> Option<u8> {
        let shift = (direction as u32).wrapping_mul(6);
        self.connections_raw()
            .checked_shr(shift)
            .map(|value| (value & 0x3f) as u8)
    }

    /// Returns the height of the span.
    pub fn height(&self) -> u8 {
        (self.data >> 24) as u8
    }

    /// Sets the height of the span.
    pub fn set_height(&mut self, height: u8) {
        self.data = (self.data & 0x00FF_FFFF) | ((height as u32) << 24);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compact_span() {
        let mut span = CompactSpan::default();
        span.set_height(10);
        assert_eq!(span.height(), 10);
    }

    #[test]
    fn test_compact_span_con() {
        let mut span = CompactSpan::default();
        span.set_con(0, Some(1));
        assert_eq!(span.con(0), Some(1));

        span.set_con(1, Some(3));
        assert_eq!(span.con(1), Some(3));

        span.set_con(2, Some(5));
        assert_eq!(span.con(2), Some(5));

        span.set_con(0, Some(2));
        assert_eq!(span.con(0), Some(2));

        span.set_con(1, None);
        assert_eq!(span.con(1), None);

        span.set_con(2, None);
        assert_eq!(span.con(2), None);
    }
}
