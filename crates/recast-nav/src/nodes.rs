// Altered Rust translation of DetourNode.h/.cpp.
// Copyright (c) 2009-2010 Mikko Mononen. Zlib: vendor/recastnavigation/License.txt.
//! Native node identity, hash-chain and priority ordering with checked Rust handles.
//! Node handles are one-based; hash-chain slots are zero-based `u16` indices.

use crate::nav::{FAILURE, INVALID_PARAM, OUT_OF_MEMORY, Point, PolyRef};
use std::mem::size_of;

pub const OPEN: u8 = 1;
pub const CLOSED: u8 = 2;
pub const PARENT_DETACHED: u8 = 4;
pub const NULL_INDEX: u16 = u16::MAX;
pub const PARENT_BITS: u32 = 24;
pub const STATE_BITS: u32 = 2;
pub const MAX_STATES: usize = 1 << STATE_BITS;

#[derive(Clone, Copy, Debug, Default)]
pub struct Node {
    pub position: Point,
    pub cost: f32,
    pub total: f32,
    pub reference: PolyRef,
    packed: u32,
}
impl Node {
    pub fn parent_index(&self) -> u32 {
        self.packed & 0x00ff_ffff
    }
    pub fn set_parent_index(&mut self, index: u32) {
        self.packed = (self.packed & !0x00ff_ffff) | (index & 0x00ff_ffff);
    }
    pub fn state(&self) -> u8 {
        ((self.packed >> 24) & 3) as u8
    }
    pub fn set_state(&mut self, state: u8) {
        self.packed = (self.packed & !(3 << 24)) | (u32::from(state & 3) << 24);
    }
    pub fn flags(&self) -> u8 {
        ((self.packed >> 26) & 7) as u8
    }
    pub fn set_flags(&mut self, flags: u8) {
        self.packed = (self.packed & !(7 << 26)) | (u32::from(flags & 7) << 26);
    }
}

fn hash_ref(mut a: PolyRef) -> u32 {
    #[cfg(feature = "polyref64")]
    {
        a = (!a).wrapping_add(a << 18);
        a ^= a >> 31;
        a = a.wrapping_mul(21);
        a ^= a >> 11;
        a = a.wrapping_add(a << 6);
        a ^= a >> 22;
        a as u32
    }
    #[cfg(not(feature = "polyref64"))]
    {
        a = a.wrapping_add(!(a << 15));
        a ^= a >> 10;
        a = a.wrapping_add(a << 3);
        a ^= a >> 6;
        a = a.wrapping_add(!(a << 11));
        a ^= a >> 16;
        a
    }
}

#[derive(Debug)]
pub struct NodePool {
    nodes: Vec<Node>,
    first: Vec<u16>,
    next: Vec<u16>,
    count: usize,
}
impl NodePool {
    pub fn new(max_nodes: usize, hash_size: usize) -> Result<Self, u32> {
        if !(1..=NULL_INDEX as usize).contains(&max_nodes)
            || (hash_size != 0 && !hash_size.is_power_of_two())
            || hash_size > i32::MAX as usize
        {
            return Err(FAILURE | INVALID_PARAM);
        }
        let mut nodes = Vec::new();
        let mut next = Vec::new();
        let mut first = Vec::new();
        nodes
            .try_reserve_exact(max_nodes)
            .map_err(|_| FAILURE | OUT_OF_MEMORY)?;
        next.try_reserve_exact(max_nodes)
            .map_err(|_| FAILURE | OUT_OF_MEMORY)?;
        first
            .try_reserve_exact(hash_size)
            .map_err(|_| FAILURE | OUT_OF_MEMORY)?;
        nodes.resize(max_nodes, Node::default());
        next.resize(max_nodes, NULL_INDEX);
        first.resize(hash_size, NULL_INDEX);
        Ok(Self {
            nodes,
            next,
            first,
            count: 0,
        })
    }
    pub fn clear(&mut self) {
        self.first.fill(NULL_INDEX);
        self.count = 0;
    }
    pub fn max_nodes(&self) -> usize {
        self.nodes.len()
    }
    #[inline]
    pub fn hash_size(&self) -> usize {
        self.first.len()
    }
    pub fn node_count(&self) -> usize {
        self.count
    }
    pub fn first(&self, bucket: usize) -> Option<u16> {
        self.first.get(bucket).copied()
    }
    pub fn next(&self, slot: usize) -> Option<u16> {
        self.next.get(slot).copied()
    }
    #[inline]
    pub fn nodes(&self) -> &[Node] {
        &self.nodes[..self.count]
    }
    #[inline(always)]
    pub fn node(&self, index: u32) -> Option<&Node> {
        self.nodes().get(index.checked_sub(1)? as usize)
    }
    #[inline]
    fn storage_node(&self, index: u32) -> Option<&Node> {
        self.nodes.get(index.checked_sub(1)? as usize)
    }
    /// Native one-based getNodeAtIdx, including retained records after clear.
    /// Fresh unused backing has safe Rust defaults; native fields are unspecified
    /// until initialized. Zero and out-of-storage indices return None.
    pub fn node_at_index(&self, index: u32) -> Option<&Node> {
        self.storage_node(index)
    }
    /// Mutable native backing access, independent of the logical node count.
    pub fn node_at_index_mut(&mut self, index: u32) -> Option<&mut Node> {
        self.nodes.get_mut(index.checked_sub(1)? as usize)
    }
    #[inline]
    pub fn node_mut(&mut self, index: u32) -> Option<&mut Node> {
        self.nodes[..self.count].get_mut(index.checked_sub(1)? as usize)
    }
    fn bucket(&self, reference: PolyRef) -> Option<usize> {
        self.hash_size()
            .checked_sub(1)
            .map(|mask| hash_ref(reference) as usize & mask)
    }
    #[inline]
    pub fn find_node(&self, reference: PolyRef, state: u8) -> Option<u32> {
        self.find_node_in_bucket(reference, state, self.bucket(reference)?)
    }
    #[inline]
    fn find_node_in_bucket(&self, reference: PolyRef, state: u8, bucket: usize) -> Option<u32> {
        let mut at = self.first[bucket];
        while at != NULL_INDEX {
            let n = &self.nodes[at as usize];
            if n.reference == reference && n.state() == state {
                return Some(u32::from(at) + 1);
            }
            at = self.next[at as usize];
        }
        None
    }
    pub fn find_nodes(&self, reference: PolyRef, capacity: usize) -> Vec<u32> {
        self.matching_nodes(reference).take(capacity).collect()
    }
    /// Native hash-chain order without allocating a temporary result vector.
    pub(crate) fn matching_nodes(&self, reference: PolyRef) -> impl Iterator<Item = u32> + '_ {
        let mut at = self.bucket(reference).map_or(NULL_INDEX, |b| self.first[b]);
        std::iter::from_fn(move || {
            while at != NULL_INDEX {
                let index = at;
                at = self.next[index as usize];
                if self.nodes[index as usize].reference == reference {
                    return Some(u32::from(index) + 1);
                }
            }
            None
        })
    }
    #[inline(always)]
    pub fn get_node(&mut self, reference: PolyRef, state: u8) -> Option<u32> {
        let bucket = self.bucket(reference)?;
        if let Some(index) = self.find_node_in_bucket(reference, state, bucket) {
            return Some(index);
        }
        if self.count == self.nodes.len() {
            return None;
        }
        let slot = self.count;
        self.count += 1;
        let n = &mut self.nodes[slot];
        n.set_parent_index(0);
        n.cost = 0.;
        n.total = 0.;
        n.reference = reference;
        n.set_state(state);
        n.set_flags(0);
        // Native allocation/clear retains position storage. Initial Rust storage
        // is zeroed; uninitialized native positions are outside comparisons.
        self.next[slot] = self.first[bucket];
        self.first[bucket] = slot as u16;
        Some(slot as u32 + 1)
    }
    /// Rust struct/vector storage bytes, excluding allocator overhead; this is
    /// independent of the native allocator/ABI.
    pub fn memory_usage(&self) -> usize {
        size_of::<Self>()
            + self.nodes.capacity() * size_of::<Node>()
            + (self.next.capacity() + self.first.capacity()) * size_of::<u16>()
    }
}

/// Queue handles refer to one pool's storage. Keep that pool alive and use the
/// same pool for every operation. Clearing a pool retains its backing records.
#[derive(Debug)]
pub struct NodeQueue {
    // Zero-based pool slots internally; u32::MAX represents the native null pointer.
    heap: Vec<u32>,
    capacity: usize,
    retained_top: Option<u32>,
}
impl NodeQueue {
    pub fn new(capacity: usize) -> Result<Self, u32> {
        if capacity == 0 || capacity >= i32::MAX as usize {
            return Err(FAILURE | INVALID_PARAM);
        }
        let mut heap = Vec::new();
        heap.try_reserve_exact(capacity + 1)
            .map_err(|_| FAILURE | OUT_OF_MEMORY)?;
        Ok(Self {
            heap,
            capacity,
            retained_top: None,
        })
    }
    pub fn clear(&mut self) {
        if let Some(&head) = self.heap.first() {
            self.retained_top = Some(head);
        }
        self.heap.clear();
    }
    pub fn is_empty(&self) -> bool {
        self.heap.is_empty()
    }
    pub fn len(&self) -> usize {
        self.heap.len()
    }
    pub fn capacity(&self) -> usize {
        self.capacity
    }
    /// Native initialized `heap[0]`, retained after clear or the final pop.
    /// Some(0) is an initialized native null entry. A fresh queue returns None.
    /// Use is_empty for the logical queue state.
    pub fn top(&self) -> Option<u32> {
        self.heap
            .first()
            .copied()
            .or(self.retained_top)
            .map(|slot| slot.wrapping_add(1))
    }
    #[inline(always)]
    fn bubble(&mut self, mut at: usize, slot: u32, pool: &NodePool) {
        // An initialized null entry is valid only as the sole heap element.
        if at > 0 {
            let total = pool.nodes[slot as usize].total;
            while at > 0 {
                let parent = (at - 1) / 2;
                if pool.nodes[self.heap[parent] as usize].total > total {
                    self.heap[at] = self.heap[parent];
                    at = parent;
                } else {
                    break;
                }
            }
        }
        self.heap[at] = slot;
    }
    #[inline(always)]
    pub fn push(&mut self, node: u32, pool: &NodePool) -> Result<(), u32> {
        // Native allocates capacity+1 pointers. The spare slot is observable
        // through this public queue, including duplicate node pointers.
        if self.heap.len() > self.capacity
            || (node == 0 && !self.heap.is_empty())
            || self.heap.first() == Some(&u32::MAX)
            || (node != 0 && pool.storage_node(node).is_none())
        {
            return Err(FAILURE | INVALID_PARAM);
        }
        let slot = node.wrapping_sub(1);
        self.heap.push(slot);
        self.bubble(self.heap.len() - 1, slot, pool);
        Ok(())
    }
    #[inline(always)]
    pub fn pop(&mut self, pool: &NodePool) -> Option<u32> {
        let result = self.heap.first().copied()?;
        let node = self.heap.pop().unwrap();
        if !self.heap.is_empty() {
            let mut at = 0;
            let mut child = 1;
            while child < self.heap.len() {
                let mut slot = self.heap[child];
                if let Some(&right) = self.heap.get(child + 1)
                    && pool.nodes[slot as usize].total > pool.nodes[right as usize].total
                {
                    child += 1;
                    slot = right;
                }
                self.heap[at] = slot;
                at = child;
                child = at * 2 + 1;
            }
            self.bubble(at, node, pool);
        } else {
            self.retained_top = Some(node);
        }
        Some(result.wrapping_add(1))
    }
    pub fn modify(&mut self, node: u32, pool: &NodePool) {
        let slot = node.wrapping_sub(1);
        if let Some(at) = self.heap.iter().position(|&i| i == slot) {
            self.bubble(at, slot, pool);
        }
    }
    pub fn memory_usage(&self) -> usize {
        size_of::<Self>() + self.heap.capacity() * size_of::<u32>()
    }
}
