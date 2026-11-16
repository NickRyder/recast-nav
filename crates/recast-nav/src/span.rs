//! The span module contains the types and functions for working with spans.
//!
//! A span is a voxel column with a floor and a ceiling.
//! It is used to represent the height of a non-occupied voxel column.
//!
//! The spans are stored in a [`Spans`](crate::span::Spans) collection.
use alloc::{boxed::Box, vec, vec::Vec};

use core::num::NonZeroU32;
use core::ops::{Deref, DerefMut, Index, IndexMut};

/// A generational key for a span in [`Spans`].
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "serialize",
    serde(from = "SpanKeyWire", into = "SpanKeyWire")
)]
#[repr(C)]
pub struct SpanKey {
    index: u32,
    version: NonZeroU32,
}

impl SpanKey {
    #[inline]
    fn new(index: u32, version: u32) -> Self {
        Self {
            index,
            version: NonZeroU32::new(version | 1).unwrap(),
        }
    }
}

impl Default for SpanKey {
    fn default() -> Self {
        Self::new(u32::MAX, 1)
    }
}

impl core::hash::Hash for SpanKey {
    fn hash<H: core::hash::Hasher>(&self, state: &mut H) {
        state.write_u64((u64::from(self.version.get()) << 32) | u64::from(self.index));
    }
}

impl core::fmt::Debug for SpanKey {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if self.index == u32::MAX {
            f.write_str("SpanKey(null)")
        } else {
            write!(f, "SpanKey({}v{})", self.index, self.version)
        }
    }
}

#[cfg(feature = "serialize")]
#[derive(serde::Serialize, serde::Deserialize)]
struct SpanKeyWire {
    idx: u32,
    version: u32,
}

#[cfg(feature = "serialize")]
impl From<SpanKeyWire> for SpanKey {
    fn from(value: SpanKeyWire) -> Self {
        Self::new(
            value.idx,
            if value.idx == u32::MAX {
                1
            } else {
                value.version
            },
        )
    }
}

#[cfg(feature = "serialize")]
impl From<SpanKey> for SpanKeyWire {
    fn from(value: SpanKey) -> Self {
        Self {
            idx: value.index,
            version: value.version.get(),
        }
    }
}

/// A collection of spans with stable generational keys.
/// Storage grows in fixed-size pools, without moving or copying existing spans.
#[derive(Debug, Default, Clone)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", serde(try_from = "SpanStorage"))]
pub struct Spans {
    pools: Vec<SpanPool>,
    high_water: u32,
    free: u32,
    len: usize,
}

#[derive(Debug, Clone)]
struct SpanPool(Box<[Span; Spans::POOL_SIZE]>);

#[cfg(feature = "serialize")]
impl serde::Serialize for SpanPool {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serde::Serialize::serialize(self.0.as_slice(), serializer)
    }
}
#[cfg(feature = "serialize")]
impl<'de> serde::Deserialize<'de> for SpanPool {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let spans: Vec<Span> = serde::Deserialize::deserialize(deserializer)?;
        let values = spans
            .into_boxed_slice()
            .try_into()
            .map_err(|_| serde::de::Error::custom("span pool must contain exactly 2048 records"))?;
        Ok(Self(values))
    }
}

// Deserialize metadata only after validating the private pool/free-list invariants.
#[cfg(feature = "serialize")]
#[derive(serde::Deserialize)]
struct SpanStorage {
    pools: Vec<SpanPool>,
    high_water: u32,
    free: u32,
    len: usize,
}

#[cfg(feature = "serialize")]
impl TryFrom<SpanStorage> for Spans {
    type Error = &'static str;
    fn try_from(storage: SpanStorage) -> Result<Self, Self::Error> {
        const INVALID: &str = "invalid span pool storage";
        let result = Self {
            pools: storage.pools,
            high_water: storage.high_water,
            free: storage.free,
            len: storage.len,
        };
        if result.high_water == u32::MAX
            || result.pools.len() != (result.high_water as usize).div_ceil(Self::POOL_SIZE)
            || result.len > result.high_water as usize
            || result.iter().count() != result.len
            || result
                .pools
                .iter()
                .flat_map(|p| p.0.iter())
                .skip(result.high_water as usize)
                .any(|s| s.version != 0)
        {
            return Err(INVALID);
        }
        let expected_free = result.high_water as usize - result.len;
        let mut cursor = result.free;
        let mut count = 0;
        while cursor != 0 {
            if cursor > result.high_water || count >= expected_free {
                return Err(INVALID);
            }
            let offset = cursor as usize - 1;
            let span = &result.pools[offset / Self::POOL_SIZE].0[offset % Self::POOL_SIZE];
            if span.version & 1 != 0 {
                return Err(INVALID);
            }
            cursor = span.next.map_or(0, |key| key.index);
            count += 1;
        }
        if count != expected_free {
            return Err(INVALID);
        }
        Ok(result)
    }
}

impl Spans {
    const POOL_SIZE: usize = 2048;

    #[inline]
    fn key(index: u32, version: u32) -> SpanKey {
        SpanKey::new(index, version)
    }

    #[inline]
    fn parts(key: SpanKey) -> (usize, u32) {
        // Index zero wraps to an unallocated slot; checked pool lookup and the
        // generation test reject it, without a branch on every valid access.
        (key.index.wrapping_sub(1) as usize, key.version.get())
    }

    /// Insert a span, reusing the most recently removed slot first.
    #[inline]
    pub fn insert(&mut self, mut value: Span) -> SpanKey {
        let reused = self.free != 0;
        let index = if reused {
            self.free
        } else {
            assert!(self.high_water < u32::MAX - 1, "span storage is full");
            self.high_water + 1
        };
        let offset = index as usize - 1;
        if offset / Self::POOL_SIZE == self.pools.len() {
            self.pools.push(SpanPool(
                vec![Span::new(0, 0, AreaType(0)); Self::POOL_SIZE]
                    .into_boxed_slice()
                    .try_into()
                    .unwrap(),
            ));
        }
        let pool = &mut self.pools[offset / Self::POOL_SIZE].0;
        let slot = offset % Self::POOL_SIZE;
        let version = pool[slot].version | 1;
        if reused {
            self.free = pool[slot].next.map_or(0, |key| key.index);
        } else {
            self.high_water = index;
        }
        value.version = version;
        pool[slot] = value;
        self.len += 1;
        Self::key(index, version)
    }

    /// Remove a live span. Removed or unallocated keys return `None`.
    #[inline]
    pub fn remove(&mut self, key: SpanKey) -> Option<Span> {
        let (offset, version) = Self::parts(key);
        let pool = &mut self.pools.get_mut(offset / Self::POOL_SIZE)?.0;
        let slot = offset % Self::POOL_SIZE;
        if pool[slot].version != version {
            return None;
        }
        let value = pool[slot].clone();
        pool[slot].next = (self.free != 0).then(|| Self::key(self.free, 1));
        pool[slot].version = version.wrapping_add(1);
        self.free = offset as u32 + 1;
        self.len -= 1;
        Some(value)
    }

    /// Look up a live span without accepting a removed generation.
    #[inline]
    pub fn get(&self, key: SpanKey) -> Option<&Span> {
        let (offset, version) = Self::parts(key);
        let pool = &self.pools.get(offset / Self::POOL_SIZE)?.0;
        let slot = offset % Self::POOL_SIZE;
        (pool[slot].version == version).then(|| &pool[slot])
    }

    /// Mutably look up a live span without accepting a removed generation.
    #[inline]
    pub fn get_mut(&mut self, key: SpanKey) -> Option<&mut Span> {
        let (offset, version) = Self::parts(key);
        let pool = &mut self.pools.get_mut(offset / Self::POOL_SIZE)?.0;
        let slot = offset % Self::POOL_SIZE;
        if pool[slot].version != version {
            return None;
        }
        Some(&mut pool[slot])
    }

    /// Number of live spans.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether no live spans remain.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Whether a key still identifies a live span.
    pub fn contains_key(&self, key: SpanKey) -> bool {
        self.get(key).is_some()
    }

    /// Iterate live spans in ascending slot order.
    pub fn iter(&self) -> impl Iterator<Item = (SpanKey, &Span)> {
        self.pools
            .iter()
            .flat_map(|pool| pool.0.iter())
            .take(self.high_water as usize)
            .enumerate()
            .filter(|(_, value)| value.version & 1 != 0)
            .map(|(offset, value)| (Self::key(offset as u32 + 1, value.version), value))
    }

    /// Remove all spans while retaining allocated pools and rejecting old keys.
    pub fn clear(&mut self) {
        for index in 1..=self.high_water {
            let offset = index as usize - 1;
            let version = self.pools[offset / Self::POOL_SIZE].0[offset % Self::POOL_SIZE].version;
            if version & 1 != 0 {
                self.remove(Self::key(index, version));
            }
        }
    }
}

impl Index<SpanKey> for Spans {
    type Output = Span;
    #[inline]
    fn index(&self, key: SpanKey) -> &Span {
        self.get(key).expect("invalid span key")
    }
}

impl IndexMut<SpanKey> for Spans {
    #[inline]
    fn index_mut(&mut self, key: SpanKey) -> &mut Span {
        self.get_mut(key).expect("invalid span key")
    }
}

pub(crate) struct SpanBuilder {
    pub(crate) min: u16,
    pub(crate) max: u16,
    pub(crate) area: AreaType,
    pub(crate) next: Option<SpanKey>,
}

impl SpanBuilder {
    pub(crate) fn build(self) -> Span {
        let mut span = Span::new(self.min, self.max, self.area);
        span.next = self.next;
        span
    }
}

impl From<SpanBuilder> for Span {
    fn from(builder: SpanBuilder) -> Self {
        builder.build()
    }
}

/// A solid span with native 13-bit heights and a six-bit area ID.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[repr(C)]
pub struct Span {
    /// The key of the next-higher span in the column.
    pub next: Option<SpanKey>,
    data: u32,
    // Generation metadata shares the record's otherwise unused alignment space.
    // It is not part of the logical span value.
    version: u32,
}

impl PartialEq for Span {
    fn eq(&self, other: &Self) -> bool {
        self.data == other.data && self.next == other.next
    }
}
impl Eq for Span {}

impl Span {
    pub(crate) const MAX_HEIGHT: u16 = (1 << 13) - 1;

    /// Construct a span, truncating each field to its native bit width.
    #[inline]
    pub const fn new(min: u16, max: u16, area: AreaType) -> Self {
        Self {
            next: None,
            data: (min as u32 & 8191) | ((max as u32 & 8191) << 13) | ((area.0 as u32 & 63) << 26),
            version: 0,
        }
    }
    /// Lower solid extent, in heightfield height units.
    #[inline]
    pub const fn min(&self) -> u16 {
        (self.data & 8191) as u16
    }
    /// Upper solid extent, in heightfield height units.
    #[inline]
    pub const fn max(&self) -> u16 {
        ((self.data >> 13) & 8191) as u16
    }
    /// Native six-bit area ID.
    #[inline]
    pub const fn area(&self) -> AreaType {
        AreaType((self.data >> 26) as u8)
    }
    /// Assign the lower extent, truncating to thirteen bits.
    #[inline]
    pub fn set_min(&mut self, value: u16) {
        self.data = (self.data & !8191) | (u32::from(value) & 8191);
    }
    /// Assign the upper extent, truncating to thirteen bits.
    #[inline]
    pub fn set_max(&mut self, value: u16) {
        self.data = (self.data & !(8191 << 13)) | ((u32::from(value) & 8191) << 13);
    }
    /// Assign the area ID, truncating to six bits.
    #[inline]
    pub fn set_area(&mut self, area: AreaType) {
        self.data = (self.data & !(63 << 26)) | ((u32::from(area.0) & 63) << 26);
    }
}

/// An identifier for the area type of a span.
/// The values 0 ([`AreaType::NOT_WALKABLE`]) and 63 ([`AreaType::DEFAULT_WALKABLE`]) are reserved.
/// The rest can be used for custom area types to e.g. assign different costs to different areas.
/// When two spans are merged, the area type of the merged span is the maximum of the two area types.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[repr(transparent)]
pub struct AreaType(pub u8);

impl Deref for AreaType {
    type Target = u8;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for AreaType {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl Default for AreaType {
    fn default() -> Self {
        Self::NOT_WALKABLE
    }
}

impl From<u8> for AreaType {
    fn from(value: u8) -> Self {
        AreaType(value)
    }
}

impl AreaType {
    /// The area type 0. Triangles with this area type are not walkable.
    /// All other area types are walkable.
    pub const NOT_WALKABLE: Self = Self(0);
    /// Default area type for walkable triangles. The highest C++ Recast area ID.
    /// Other area types that are not [`AreaType::NOT_WALKABLE`] are also walkable.
    pub const DEFAULT_WALKABLE: Self = Self(63);

    /// Returns whether the area type is walkable.
    #[inline]
    pub fn is_walkable(&self) -> bool {
        self != &Self::NOT_WALKABLE
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn span() -> Span {
        SpanBuilder {
            min: 2,
            max: 10,
            area: AreaType(4),
            next: None,
        }
        .build()
    }

    #[test]
    fn can_retrieve_span_data_after_building() {
        let span = span();
        assert_eq!(span.min(), 2);
        assert_eq!(span.max(), 10);
        assert_eq!(span.area(), AreaType(4));
        assert_eq!(span.next, None);
    }

    #[test]
    fn can_retrieve_span_data_after_setting() {
        let mut span = span();
        let mut spans = Spans::default();
        let span_key = spans.insert(span.clone());

        span.set_min(1);
        span.set_max(4);
        span.set_area(AreaType(3));
        span.next = Some(span_key);

        assert_eq!(span.min(), 1);
        assert_eq!(span.max(), 4);
        assert_eq!(span.area(), AreaType(3));
        assert_eq!(span.next, Some(span_key));
    }

    #[cfg(feature = "serialize")]
    #[test]
    fn serialized_span_storage_preserves_keys_and_rejects_invalid_free_lists() {
        let mut spans = Spans::default();
        let removed = spans.insert(span());
        let live = spans.insert(span());
        spans.remove(removed).unwrap();
        let encoded = serde_json::to_value(&spans).unwrap();
        let mut restored: Spans = serde_json::from_value(encoded.clone()).unwrap();
        assert_eq!(restored[live], spans[live]);
        assert!(restored.get(removed).is_none());
        let replacement = restored.insert(span());
        assert_ne!(replacement, removed);
        assert_eq!(replacement.index, removed.index);
        assert_eq!(
            serde_json::to_value(live).unwrap(),
            serde_json::json!({"idx": 2, "version": 1})
        );
        let null: SpanKey =
            serde_json::from_value(serde_json::json!({"idx": u32::MAX, "version": 0})).unwrap();
        assert_eq!(null, SpanKey::default());
        let mut invalid = encoded;
        invalid["free"] = serde_json::json!(u32::MAX);
        assert!(serde_json::from_value::<Spans>(invalid).is_err());
    }
}
