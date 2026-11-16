//! Shared ordered output for owned queries and native bounded stack scratch.
use crate::nav::PolyRef;

pub(crate) trait PolyBuffer {
    fn clear(&mut self);
    fn len(&self) -> usize;
    fn push(&mut self, value: PolyRef);
    fn pop(&mut self) -> Option<PolyRef>;
    fn as_slice(&self) -> &[PolyRef];
}
impl PolyBuffer for Vec<PolyRef> {
    #[inline]
    fn clear(&mut self) {
        Vec::clear(self);
    }
    #[inline]
    fn len(&self) -> usize {
        Vec::len(self)
    }
    #[inline]
    fn push(&mut self, value: PolyRef) {
        Vec::push(self, value);
    }
    #[inline]
    fn pop(&mut self) -> Option<PolyRef> {
        Vec::pop(self)
    }
    #[inline]
    fn as_slice(&self) -> &[PolyRef] {
        Vec::as_slice(self)
    }
}

/// The caller supplies enough backing for the kernel's advertised budget,
/// including a spare slot when sliced any-angle reconstruction requires it.
pub(crate) struct SlicePolys<'a> {
    values: &'a mut [PolyRef],
    count: usize,
}
impl<'a> SlicePolys<'a> {
    pub(crate) fn new(values: &'a mut [PolyRef]) -> Self {
        Self { values, count: 0 }
    }
}
impl PolyBuffer for SlicePolys<'_> {
    #[inline]
    fn clear(&mut self) {
        self.count = 0;
    }
    #[inline]
    fn len(&self) -> usize {
        self.count
    }
    #[inline]
    fn push(&mut self, value: PolyRef) {
        self.values[self.count] = value;
        self.count += 1;
    }
    #[inline]
    fn pop(&mut self) -> Option<PolyRef> {
        if self.count == 0 {
            return None;
        }
        self.count -= 1;
        Some(self.values[self.count])
    }
    #[inline]
    fn as_slice(&self) -> &[PolyRef] {
        &self.values[..self.count]
    }
}
