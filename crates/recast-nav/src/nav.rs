// Altered Rust translation of DetourNavMesh.cpp and portions of DetourNavMeshQuery.cpp.
// Copyright (c) 2009-2010 Mikko Mononen. Zlib: vendor/recastnavigation/License.txt.
use crate::{
    BuildError, cache::CacheMesh, layers::HeightfieldLayer, nav_data, nodes,
    poly_buffer::PolyBuffer,
};
use std::{borrow::Cow, collections::VecDeque};
pub const SUCCESS: u32 = 1 << 30;
pub const FAILURE: u32 = 1 << 31;
pub const IN_PROGRESS: u32 = 1 << 29;
pub const STATUS_DETAIL_MASK: u32 = 0x00ff_ffff;
pub const WRONG_MAGIC: u32 = 1;
pub const WRONG_VERSION: u32 = 1 << 1;
pub const OUT_OF_MEMORY: u32 = 1 << 2;
pub const INVALID_PARAM: u32 = 1 << 3;
pub const BUFFER_TOO_SMALL: u32 = 1 << 4;
pub const OUT_OF_NODES: u32 = 1 << 5;
pub const PARTIAL_RESULT: u32 = 1 << 6;
pub const ALREADY_OCCUPIED: u32 = 1 << 7;
/// Native status predicates test individual bits, including combined high bits.
pub const fn status_succeeded(status: u32) -> bool {
    status & SUCCESS != 0
}
pub const fn status_failed(status: u32) -> bool {
    status & FAILURE != 0
}
pub const fn status_in_progress(status: u32) -> bool {
    status & IN_PROGRESS != 0
}
/// A combined detail mask succeeds when any requested bit is present.
pub const fn status_has_detail(status: u32, detail: u32) -> bool {
    status & detail != 0
}

/// Native `dtGetDetailTriEdgeFlags`. Indices through 15 have defined promoted
/// integer shifts; other indices return `None` rather than invoking native UB.
pub fn detail_triangle_edge_flags(flags: u8, edge_index: i32) -> Option<u8> {
    let shift = edge_index.checked_mul(2)?;
    (0..32)
        .contains(&shift)
        .then(|| ((u32::from(flags) >> shift) & 3) as u8)
}

pub const NULL_LINK: u32 = u32::MAX;
pub const MAX_VERTICES_PER_POLYGON: usize = 6;
pub const EXT_LINK: u16 = 0x8000;
pub const OFFMESH_CON_BIDIR: u32 = 1;
pub const MAX_AREAS: usize = 64;
pub const STRAIGHTPATH_START: u8 = 1;
pub const STRAIGHTPATH_END: u8 = 2;
pub const STRAIGHTPATH_OFFMESH_CONNECTION: u8 = 4;
pub const DETAIL_EDGE_BOUNDARY: u8 = 1;
pub const POLYTYPE_GROUND: u8 = 0;
pub const POLYTYPE_OFFMESH_CONNECTION: u8 = 1;
pub const RAYCAST_USE_COSTS: u32 = 1;
pub const FINDPATH_ANY_ANGLE: u32 = 2;
pub const RAY_CAST_LIMIT_PROPORTIONS: f32 = 50.;
pub const STRAIGHTPATH_AREA_CROSSINGS: u32 = 1;
pub const STRAIGHTPATH_ALL_CROSSINGS: u32 = 2;
/// Native Detour polygon/tile reference width. Cache and obstacle handles stay 32-bit.
#[cfg(not(feature = "polyref64"))]
pub type PolyRef = u32;
#[cfg(feature = "polyref64")]
pub type PolyRef = u64;
pub type TileRef = PolyRef;

/// Fixed native reference fields, available only with `polyref64`.
#[cfg(feature = "polyref64")]
pub const SALT_BITS: u32 = 16;
#[cfg(feature = "polyref64")]
pub const TILE_BITS: u32 = 28;
#[cfg(feature = "polyref64")]
pub const POLY_BITS: u32 = 20;
pub const TILE_FREE_DATA: i32 = 1;
pub type Point = [f32; 3];

#[inline]
pub(crate) fn link_reference_bit(reference: PolyRef) -> u64 {
    1u64 << ((reference ^ (reference >> 5)) & 63)
}

/// Upstream reference bit allocation for the selected build mode.
/// In 32-bit mode native unsigned arithmetic determines the salt budget. In 64-bit mode
/// allocation is fixed at 16/28/20 bits, independently of the requested budgets.
/// As upstream, the 64-bit constructor does not check whether budgets fit those
/// fields; callers must keep actual tile/polygon indices representable.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReferenceLayout {
    salt_bits: u32,
    tile_bits: u32,
    poly_bits: u32,
}
impl ReferenceLayout {
    pub fn new(max_tiles: usize, max_polygons: usize) -> Result<Self, BuildError> {
        if max_tiles > i32::MAX as usize || max_polygons > i32::MAX as usize {
            return Err(BuildError("invalid navigation mesh capacity".into()));
        }
        Self::from_native_capacities(max_tiles as i32, max_polygons as i32)
    }
    /// Signed native parameters. Negative polygon budgets affect only native
    /// unsigned bit calculations; negative tile counts request invalid storage.
    pub fn from_native_capacities(max_tiles: i32, max_polygons: i32) -> Result<Self, BuildError> {
        if max_tiles < 0 {
            return Err(BuildError("invalid navigation mesh capacity".into()));
        }
        #[cfg(feature = "polyref64")]
        {
            let _ = max_polygons;
            Ok(Self {
                salt_bits: SALT_BITS,
                tile_bits: TILE_BITS,
                poly_bits: POLY_BITS,
            })
        }
        #[cfg(not(feature = "polyref64"))]
        {
            let bits =
                |n: i32| crate::common::integer_log2(crate::common::next_power_of_two(n as u32));
            let tile_bits = bits(max_tiles);
            let poly_bits = bits(max_polygons);
            let salt_bits = 32u32.wrapping_sub(tile_bits + poly_bits).min(31);
            if salt_bits < 10 {
                return Err(BuildError(
                    "polygon reference leaves fewer than ten salt bits".into(),
                ));
            }
            Ok(Self {
                salt_bits,
                tile_bits,
                poly_bits,
            })
        }
    }
    pub fn salt_bits(self) -> u32 {
        self.salt_bits
    }
    pub fn tile_bits(self) -> u32 {
        self.tile_bits
    }
    pub fn polygon_bits(self) -> u32 {
        self.poly_bits
    }
    #[inline]
    pub fn encode(self, salt: u32, tile: u32, polygon: u32) -> PolyRef {
        self.try_encode(salt, tile, polygon)
            .expect("undefined native reference shift")
    }
    /// Native encoding, or `None` when initialization accepted capacities whose
    /// combined bit counts make the native reference shift undefined.
    #[inline]
    pub fn try_encode(self, salt: u32, tile: u32, polygon: u32) -> Option<PolyRef> {
        Some(
            (salt as PolyRef).checked_shl(self.tile_bits + self.poly_bits)?
                | (tile as PolyRef).checked_shl(self.poly_bits)?
                | polygon as PolyRef,
        )
    }
    #[inline]
    pub fn decode(self, reference: PolyRef) -> (u32, u32, u32) {
        self.try_decode(reference)
            .expect("undefined native reference shift")
    }
    /// Native decoding with the same checked shift domain as `try_encode`.
    #[inline]
    pub fn try_decode(self, reference: PolyRef) -> Option<(u32, u32, u32)> {
        let polygon = reference & ((1 << self.poly_bits) - 1);
        let tile = (reference >> self.poly_bits) & ((1 << self.tile_bits) - 1);
        let salt =
            reference.checked_shr(self.tile_bits + self.poly_bits)? & self.salt_mask() as PolyRef;
        #[cfg(feature = "polyref64")]
        {
            Some((salt as u32, tile as u32, polygon as u32))
        }
        #[cfg(not(feature = "polyref64"))]
        {
            Some((salt, tile, polygon))
        }
    }
    #[inline]
    fn salt_mask(self) -> u32 {
        (1u32 << self.salt_bits) - 1
    }
}
#[derive(Clone, Debug)]
pub struct Link {
    pub reference: PolyRef,
    pub next: u32,
    pub edge: u8,
    pub side: u8,
    pub min: u8,
    pub max: u8,
}
/// Native fixed-width polygon storage, including initialized spare entries.
/// `vertices()` and `neighbors()` expose the active prefix. Serialization is
/// explicit; this alias does not promise a native pointer ABI.
pub type Poly = nav_data::Polygon;
#[derive(Clone, Debug)]
pub struct Tile {
    pub flags: i32,
    pub data_size: i32,
    pub coordinate: [i32; 3],
    vertex_range: std::ops::Range<usize>,
    polygon_start: usize,
    polygon_count: u32,
    pub climb: f32,
    pub header: nav_data::Header,
    detail_range: std::ops::Range<usize>,
    detail_vertex_range: std::ops::Range<usize>,
    triangle_range: std::ops::Range<usize>,
    bv_range: std::ops::Range<usize>,
    off_mesh_range: std::ops::Range<usize>,
    link_start: usize,
    link_count: u32,
    pub free_link: u32,
    source_data: Vec<u8>,
}

impl Tile {
    pub fn data(&self) -> Result<Vec<u8>, u32> {
        let mut data = Vec::new();
        self.data_into(&mut data)?;
        Ok(data)
    }
    /// Copies the complete native buffer into reusable caller storage, retaining
    /// padding and initialized spare records before applying current tile fields.
    /// Raw metadata is retained without traversing its counts or topology.
    pub fn data_into(&self, data: &mut Vec<u8>) -> Result<(), u32> {
        data.clear();
        data.extend_from_slice(&self.source_data);
        nav_data::write_tile_header(data, self);
        Ok(())
    }
    /// Returns a serialized header and the borrowed native buffer body without
    /// allocating or copying the body. Concatenating the parts yields `data()`.
    /// The header captures current metadata; the borrow prevents body mutation.
    pub fn data_parts(&self) -> ([u8; nav_data::HEADER_SIZE], &[u8]) {
        let mut header = [0; nav_data::HEADER_SIZE];
        nav_data::write_tile_header(&mut header, self);
        (header, &self.source_data[nav_data::HEADER_SIZE..])
    }
    fn from_data(view: &nav_data::TileData<'_>) -> Result<Self, u32> {
        let h = &view.header;
        let details = view.detail_records();
        let off_mesh_connections = view.off_mesh_connections();
        let polygons = view.polygon_records();
        let ground = h.off_mesh_base as usize;
        if polygons.is_empty()
            || ground > polygons.len()
            || ground != details.len()
            || off_mesh_connections.len() != polygons.len() - ground
            || h.max_link_count == 0
        {
            return Err(FAILURE | INVALID_PARAM);
        }
        // Native loading interprets polygon types and counts only as linking
        // needs them; it does not scan all polygon, BV or detail records for
        // validity. Geometry operations interpret the records they visit.
        for (i, con) in off_mesh_connections.enumerate() {
            if con.polygon as usize != ground + i {
                return Err(FAILURE | INVALID_PARAM);
            }
        }
        let [
            vertex_range,
            detail_range,
            detail_vertex_range,
            off_mesh_range,
        ] = view.geometry_ranges();
        Ok(Self {
            flags: 0,
            data_size: 0,
            coordinate: h.coordinate,
            climb: h.walkable_climb,
            header: h.clone(),
            vertex_range,
            polygon_start: view.polygon_range().start,
            polygon_count: h.polygon_count,
            detail_range,
            detail_vertex_range,
            triangle_range: view.triangle_range(),
            bv_range: view.bv_range(),
            off_mesh_range,
            link_start: view.link_range().start,
            link_count: h.max_link_count,
            free_link: 0,
            source_data: Vec::new(),
        })
    }
    #[inline]
    pub fn polygon_count(&self) -> usize {
        self.polygon_count as usize
    }
    #[inline(always)]
    fn polygon_records(&self) -> &[[u8; 32]] {
        self.source_data[self.polygon_start..self.polygon_start + self.polygon_count as usize * 32]
            .as_chunks::<32>()
            .0
    }
    pub fn polygons(&self) -> impl ExactSizeIterator<Item = nav_data::PolygonView<'_>> {
        self.polygon_records().iter().map(nav_data::PolygonView)
    }
    fn polygon_records_mut(&mut self) -> &mut [[u8; 32]] {
        self.source_data[self.polygon_start..self.polygon_start + self.polygon_count as usize * 32]
            .as_chunks_mut::<32>()
            .0
    }
    pub fn polygons_mut(&mut self) -> impl ExactSizeIterator<Item = nav_data::PolygonMut<'_>> {
        self.polygon_records_mut()
            .iter_mut()
            .map(nav_data::PolygonMut)
    }
    #[inline]
    pub fn polygon(&self, index: usize) -> Option<nav_data::PolygonView<'_>> {
        self.polygon_records().get(index).map(nav_data::PolygonView)
    }
    #[inline]
    fn polygon_at(&self, index: usize) -> nav_data::PolygonView<'_> {
        nav_data::PolygonView(&self.polygon_records()[index])
    }
    pub fn polygon_mut(&mut self, index: usize) -> Option<nav_data::PolygonMut<'_>> {
        self.polygon_records_mut()
            .get_mut(index)
            .map(nav_data::PolygonMut)
    }
    /// Replaces all fields, including initialized spare vertex and neighbor words.
    /// Every count byte is retained by raw export. Keep the active vertex count
    /// at most six before accessing active index slices, and restore valid topology
    /// before queries.
    pub fn set_polygon(&mut self, index: usize, value: &Poly) -> Result<(), u32> {
        nav_data::write_polygon(
            self.polygon_records_mut()
                .get_mut(index)
                .ok_or(FAILURE | INVALID_PARAM)?,
            value,
        );
        Ok(())
    }
    pub fn vertex_count(&self) -> usize {
        self.vertex_range.len() / 12
    }
    /// Reads fixed native records without allocating a decoded array.
    pub fn vertices(&self) -> impl ExactSizeIterator<Item = Point> + '_ {
        self.vertex_records().iter().map(nav_data::read_point)
    }
    #[inline]
    pub fn vertex(&self, index: usize) -> Option<Point> {
        self.vertex_records().get(index).map(nav_data::read_point)
    }
    /// Replaces one record; invalid indices leave bytes intact.
    pub fn set_vertex(&mut self, index: usize, value: Point) -> Result<(), u32> {
        let records = self.source_data[self.vertex_range.clone()]
            .as_chunks_mut::<12>()
            .0;
        nav_data::write_point(
            records.get_mut(index).ok_or(FAILURE | INVALID_PARAM)?,
            value,
        );
        Ok(())
    }
    pub fn detail_mesh_count(&self) -> usize {
        self.detail_range.len() / 12
    }
    /// Reads fixed native records without allocating a decoded array.
    pub fn detail_meshes(&self) -> impl ExactSizeIterator<Item = nav_data::DetailMesh> + '_ {
        self.source_data[self.detail_range.clone()]
            .as_chunks::<12>()
            .0
            .iter()
            .map(nav_data::read_detail)
    }
    #[inline]
    pub fn detail_mesh(&self, index: usize) -> Option<nav_data::DetailMesh> {
        self.source_data[self.detail_range.clone()]
            .as_chunks::<12>()
            .0
            .get(index)
            .map(nav_data::read_detail)
    }
    /// Replaces one record; invalid indices leave bytes intact.
    pub fn set_detail_mesh(
        &mut self,
        index: usize,
        value: nav_data::DetailMesh,
    ) -> Result<(), u32> {
        let records = self.source_data[self.detail_range.clone()]
            .as_chunks_mut::<12>()
            .0;
        nav_data::write_detail(
            records.get_mut(index).ok_or(FAILURE | INVALID_PARAM)?,
            &value,
        );
        Ok(())
    }
    pub fn detail_vertex_count(&self) -> usize {
        self.detail_vertex_range.len() / 12
    }
    /// Reads fixed native records without allocating a decoded array.
    pub fn detail_vertices(&self) -> impl ExactSizeIterator<Item = Point> + '_ {
        self.source_data[self.detail_vertex_range.clone()]
            .as_chunks::<12>()
            .0
            .iter()
            .map(nav_data::read_point)
    }
    #[inline]
    pub fn detail_vertex(&self, index: usize) -> Option<Point> {
        self.source_data[self.detail_vertex_range.clone()]
            .as_chunks::<12>()
            .0
            .get(index)
            .map(nav_data::read_point)
    }
    /// Replaces one record; invalid indices leave bytes intact.
    pub fn set_detail_vertex(&mut self, index: usize, value: Point) -> Result<(), u32> {
        let records = self.source_data[self.detail_vertex_range.clone()]
            .as_chunks_mut::<12>()
            .0;
        nav_data::write_point(
            records.get_mut(index).ok_or(FAILURE | INVALID_PARAM)?,
            value,
        );
        Ok(())
    }
    pub fn off_mesh_connection_count(&self) -> usize {
        self.off_mesh_range.len() / 36
    }
    /// Reads fixed native records without allocating a decoded array.
    pub fn off_mesh_connections(
        &self,
    ) -> impl ExactSizeIterator<Item = nav_data::OffMeshConnection> + '_ {
        self.source_data[self.off_mesh_range.clone()]
            .as_chunks::<36>()
            .0
            .iter()
            .map(nav_data::read_off_mesh)
    }
    #[inline]
    pub fn off_mesh_connection(&self, index: usize) -> Option<nav_data::OffMeshConnection> {
        self.source_data[self.off_mesh_range.clone()]
            .as_chunks::<36>()
            .0
            .get(index)
            .map(nav_data::read_off_mesh)
    }
    /// Replaces one record; invalid indices leave bytes intact.
    pub fn set_off_mesh_connection(
        &mut self,
        index: usize,
        value: nav_data::OffMeshConnection,
    ) -> Result<(), u32> {
        let records = self.source_data[self.off_mesh_range.clone()]
            .as_chunks_mut::<36>()
            .0;
        nav_data::write_off_mesh(
            records.get_mut(index).ok_or(FAILURE | INVALID_PARAM)?,
            &value,
        );
        Ok(())
    }
    #[inline]
    pub(crate) fn vertex_records(&self) -> &[[u8; 12]] {
        self.source_data[self.vertex_range.clone()]
            .as_chunks::<12>()
            .0
    }
    #[inline]
    pub(crate) fn vertex_at(&self, index: usize) -> Point {
        nav_data::read_point(&self.vertex_records()[index])
    }
    /// Native detail-triangle records, including edge flags, without a copy.
    #[inline]
    pub fn detail_triangles(&self) -> &[[u8; 4]] {
        self.source_data[self.triangle_range.clone()]
            .as_chunks::<4>()
            .0
    }
    /// Mutates existing native records without changing their count.
    pub fn detail_triangles_mut(&mut self) -> &mut [[u8; 4]] {
        self.source_data[self.triangle_range.clone()]
            .as_chunks_mut::<4>()
            .0
    }
    #[inline]
    fn bv_records(&self) -> &[[u8; 16]] {
        self.source_data[self.bv_range.clone()].as_chunks::<16>().0
    }
    pub fn bv_node_count(&self) -> usize {
        self.bv_range.len() / 16
    }
    /// Reads native BV-tree records without allocating a decoded array.
    pub fn bv_nodes(&self) -> impl ExactSizeIterator<Item = nav_data::BvNode> + '_ {
        self.bv_records().iter().map(nav_data::read_bv_node)
    }
    pub fn bv_node(&self, index: usize) -> Option<nav_data::BvNode> {
        self.bv_records().get(index).map(nav_data::read_bv_node)
    }
    /// Replaces one native BV-tree record; invalid indices leave bytes intact.
    pub fn set_bv_node(&mut self, index: usize, node: &nav_data::BvNode) -> Result<(), u32> {
        let records = self.source_data[self.bv_range.clone()]
            .as_chunks_mut::<16>()
            .0;
        let record = records.get_mut(index).ok_or(FAILURE | INVALID_PARAM)?;
        nav_data::write_bv_node(record, node);
        Ok(())
    }
    /// Number of native link records, including initialized free records.
    pub fn link_count(&self) -> usize {
        self.link_count as usize
    }
    /// Reads a link directly from the owned native buffer without allocating.
    #[inline]
    pub fn link(&self, index: usize) -> Option<Link> {
        let start = index.checked_mul(nav_data::LINK_SIZE)?;
        let end = start.checked_add(nav_data::LINK_SIZE)?;
        self.source_data
            [self.link_start..self.link_start + self.link_count as usize * nav_data::LINK_SIZE]
            .get(start..end)
            .map(nav_data::read_link)
    }
    /// Replaces one link record; invalid indices leave the backing unchanged.
    pub fn set_link(&mut self, index: usize, link: &Link) -> Result<(), u32> {
        let start = index
            .checked_mul(nav_data::LINK_SIZE)
            .ok_or(FAILURE | INVALID_PARAM)?;
        let end = start
            .checked_add(nav_data::LINK_SIZE)
            .ok_or(FAILURE | INVALID_PARAM)?;
        let bytes = self.source_data
            [self.link_start..self.link_start + self.link_count as usize * nav_data::LINK_SIZE]
            .get_mut(start..end)
            .ok_or(FAILURE | INVALID_PARAM)?;
        nav_data::write_link(bytes, link);
        Ok(())
    }
    /// Native linked-list order, independently of the pool's storage order.
    #[inline]
    pub fn links<'a>(
        &'a self,
        poly: nav_data::PolygonView<'_>,
    ) -> impl Iterator<Item = nav_data::LinkView<'a>> + Clone + 'a {
        let mut at = poly.first_link();
        let (links, _) = self.source_data
            [self.link_start..self.link_start + self.link_count as usize * nav_data::LINK_SIZE]
            .as_chunks::<{ nav_data::LINK_SIZE }>();
        std::iter::from_fn(move || {
            if at == NULL_LINK {
                return None;
            }
            let link = nav_data::LinkView(&links[at as usize]);
            at = link.next();
            Some(link)
        })
    }
    /// Marks links proved to be the first occurrence of their reference.
    /// Hash collisions fall back to lookup; rejected links still set their bit.
    #[inline]
    pub(crate) fn links_with_first<'a>(
        &'a self,
        poly: nav_data::PolygonView<'_>,
    ) -> impl Iterator<Item = (nav_data::LinkView<'a>, bool)> + 'a {
        let mut seen = 0u64;
        self.links(poly).map(move |link| {
            let reference = link.reference();
            let bit = link_reference_bit(reference);
            let first = seen & bit == 0;
            seen |= bit;
            (link, first)
        })
    }
    fn connect_internal_links(&mut self, base: PolyRef) {
        // The polygon and link sections are disjoint fixed records. Borrow each
        // once so internal links do not repeatedly resolve the same tile ranges.
        let (before_links, after_links) = self.source_data.split_at_mut(self.link_start);
        let polygons = before_links
            [self.polygon_start..self.polygon_start + self.polygon_count as usize * 32]
            .as_chunks_mut::<32>()
            .0;
        let links = after_links[..self.link_count as usize * nav_data::LINK_SIZE]
            .as_chunks_mut::<{ nav_data::LINK_SIZE }>()
            .0;
        let mut free_link = self.free_link;
        for record in polygons {
            let mut polygon = nav_data::PolygonMut(record);
            polygon.set_first_link(NULL_LINK);
            if polygon.view().kind() == POLYTYPE_OFFMESH_CONNECTION {
                continue;
            }
            // Reverse insertion retains the native ascending-edge linked list.
            for edge in (0..polygon.view().vertex_count() as usize).rev() {
                let neighbor = polygon.view().neighbor(edge);
                if neighbor == 0 || neighbor & EXT_LINK != 0 || free_link == NULL_LINK {
                    continue;
                }
                let index = free_link;
                let link = &mut links[index as usize];
                free_link = nav_data::LinkView(link).next();
                nav_data::write_link(
                    link,
                    &Link {
                        reference: base | (neighbor - 1) as PolyRef,
                        next: polygon.view().first_link(),
                        edge: edge as u8,
                        side: 255,
                        min: 0,
                        max: 0,
                    },
                );
                polygon.set_first_link(index);
            }
        }
        self.free_link = free_link;
    }
    fn add_link(&mut self, polygon: usize, mut link: Link) -> bool {
        if self.free_link == NULL_LINK {
            return false;
        }
        let at = self.free_link as usize;
        self.free_link = self.link(at).unwrap().next;
        link.next = self.polygon_at(polygon).first_link();
        self.polygon_mut(polygon).unwrap().set_first_link(at as u32);
        self.set_link(at, &link).unwrap();
        true
    }
    fn disconnect(&mut self, target: usize, layout: ReferenceLayout) {
        for polygon in 0..self.polygon_count() {
            let mut prev: Option<usize> = None;
            let mut at = self.polygon_at(polygon).first_link();
            while at != NULL_LINK {
                let i = at as usize;
                let base = self.link_start + i * nav_data::LINK_SIZE;
                let link = nav_data::read_link(&self.source_data[base..base + nav_data::LINK_SIZE]);
                let next = link.next;
                if layout.decode(link.reference).1 as usize == target {
                    if let Some(prev) = prev {
                        let base = self.link_start + prev * nav_data::LINK_SIZE;
                        self.source_data
                            [base + nav_data::LINK_SIZE - 8..base + nav_data::LINK_SIZE - 4]
                            .copy_from_slice(&next.to_ne_bytes());
                    } else {
                        self.polygon_mut(polygon).unwrap().set_first_link(next);
                    }
                    self.source_data
                        [base + nav_data::LINK_SIZE - 8..base + nav_data::LINK_SIZE - 4]
                        .copy_from_slice(&self.free_link.to_ne_bytes());
                    self.free_link = at;
                } else {
                    prev = Some(i);
                }
                at = next;
            }
        }
    }
    fn triangles(
        &self,
        polygon: usize,
        boundary_only: bool,
    ) -> impl Iterator<Item = ([u8; 4], [Point; 3])> + '_ {
        let poly = self.polygon_at(polygon);
        let d = self.detail_mesh(polygon).unwrap();
        let triangles = self.source_data[self.triangle_range.start..]
            .as_chunks::<4>()
            .0;
        let vertices = self.vertex_records();
        let detail_vertices = &self.source_data[self.detail_vertex_range.start..];
        let indices = poly.vertex_words();
        (0..u32::from(d.triangle_count)).filter_map(move |i| {
            // Native unsigned offsets wrap before pointer arithmetic. Read
            // each visited record, leaving skipped and early-exit tails alone.
            let offset = d.triangle_base.wrapping_add(i).wrapping_mul(4) as usize;
            let tri = triangles[offset / 4];
            if boundary_only && tri[3] & 0x15 == 0 {
                return None;
            }
            let point = |id: u8| {
                let id = id as usize;
                if id < indices.len() {
                    nav_data::read_point(&vertices[u16::from_ne_bytes(indices[id]) as usize])
                } else {
                    let offset = d
                        .vertex_base
                        .wrapping_add((id - indices.len()) as u32)
                        .wrapping_mul(3) as usize
                        * 4;
                    nav_data::read_point(detail_vertices[offset..offset + 12].try_into().unwrap())
                }
            };
            Some((tri, [point(tri[0]), point(tri[1]), point(tri[2])]))
        })
    }
    fn detail_edge(&self, polygon: usize, point: Point, boundary_only: bool) -> Option<Point> {
        let mut best = f32::MAX;
        let mut edge = None;
        for (tri, v) in self.triangles(polygon, boundary_only) {
            for k in 0..3 {
                let j = (k + 2) % 3;
                if (tri[3] >> (j * 2)) & 1 == 0 && (boundary_only || tri[j] < tri[k]) {
                    continue;
                }
                let (d, t) = segment(point, v[j], v[k]);
                if d < best {
                    best = d;
                    edge = Some((v[j], v[k], t));
                }
            }
        }
        edge.map(|(a, b, t)| lerp(a, b, t))
    }
    fn ground_height(&self, polygon: usize, point: Point) -> Option<f32> {
        let p = self.polygon_at(polygon);
        if p.kind() == 1 {
            return None;
        }
        let mut vertices = [[0.; 3]; 6];
        for (dst, id) in vertices.iter_mut().zip(p.vertices()) {
            *dst = self.vertex_at(usize::from(id));
        }
        if !inside(point, &vertices[..p.vertices().len()]) {
            return None;
        }
        for (_, tri) in self.triangles(polygon, false) {
            if let Some(y) = height(point, tri[0], tri[1], tri[2]) {
                return Some(y);
            }
        }
        self.detail_edge(polygon, point, false).map(|p| p[1])
    }
    fn closest_point(&self, polygon: usize, point: Point) -> Option<(Point, bool)> {
        if let Some(y) = self.ground_height(polygon, point) {
            return Some(([point[0], y, point[2]], true));
        }
        let poly = self.polygon_at(polygon);
        if poly.kind() == 1 {
            let (a, b) = (
                self.vertex_at(poly.vertex(0) as usize),
                self.vertex_at(poly.vertex(1) as usize),
            );
            return Some((lerp(a, b, segment(point, a, b).1), false));
        }
        self.detail_edge(polygon, point, true).map(|p| (p, false))
    }
    fn candidates<'a>(
        &'a self,
        lo: Point,
        hi: Point,
        mut accept: impl FnMut(usize) -> bool + 'a,
    ) -> impl Iterator<Item = usize> + 'a {
        let nodes = self.bv_records();
        let quantized = (!nodes.is_empty()).then(|| {
            let mut min = [0; 3];
            let mut max = [0; 3];
            for axis in 0..3 {
                let a = crate::common::clamp(
                    lo[axis],
                    self.header.bounds[0][axis],
                    self.header.bounds[1][axis],
                ) - self.header.bounds[0][axis];
                let b = crate::common::clamp(
                    hi[axis],
                    self.header.bounds[0][axis],
                    self.header.bounds[1][axis],
                ) - self.header.bounds[0][axis];
                min[axis] = (self.header.bv_quantization_factor * a) as u16 & 0xfffe;
                max[axis] = madd(self.header.bv_quantization_factor, b, 1.) as u16 | 1;
            }
            (min, max)
        });
        let mut i = 0;
        std::iter::from_fn(move || {
            if let Some((min, max)) = quantized {
                while i < nodes.len() {
                    let node = nav_data::read_bv_node(&nodes[i]);
                    let overlap = (0..3).all(|a| min[a] <= node.max[a] && max[a] >= node.min[a]);
                    let leaf = node.index >= 0;
                    i += if overlap || leaf {
                        1
                    } else {
                        -node.index as usize
                    };
                    if leaf && overlap && accept(node.index as usize) {
                        return Some(node.index as usize);
                    }
                }
            } else {
                while i < self.polygon_count() {
                    let at = i;
                    i += 1;
                    let p = self.polygon_at(at);
                    if p.kind() == 1 || !accept(at) {
                        continue;
                    }
                    let mut min = self.vertex_at(p.vertex(0) as usize);
                    let mut max = min;
                    for id in p.vertices().skip(1) {
                        for axis in 0..3 {
                            min[axis] = crate::common::minimum(
                                min[axis],
                                self.vertex_at(usize::from(id))[axis],
                            );
                            max[axis] = crate::common::maximum(
                                max[axis],
                                self.vertex_at(usize::from(id))[axis],
                            );
                        }
                    }
                    if crate::common::overlap_bounds([lo, hi], [min, max]) {
                        return Some(at);
                    }
                }
            }
            None
        })
    }
}

fn neighbor_offset(side: usize) -> (i32, i32) {
    (
        [1, 1, 0, -1, -1, -1, 0, 1][side],
        [0, 1, 1, 1, 0, -1, -1, -1][side],
    )
}
/// Native mesh initialization parameters, retaining requested capacities.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NavMeshParams {
    pub origin: Point,
    pub tile_width: f32,
    pub tile_height: f32,
    pub max_tiles: i32,
    pub max_polygons: i32,
}
/// A native allocated tile slot can exist without a loaded tile/header.
#[derive(Clone, Copy, Debug)]
pub struct TileSlot<'a> {
    pub index: usize,
    pub salt: u32,
    pub tile: Option<&'a Tile>,
    pub flags: i32,
    pub data_size: i32,
    pub free_link: u32,
    retained_data: Option<&'a [u8]>,
}
impl TileSlot<'_> {
    pub fn data(&self) -> Result<Option<Vec<u8>>, u32> {
        self.tile.map_or_else(
            || Ok(self.retained_data.map(<[u8]>::to_vec)),
            |tile| tile.data().map(Some),
        )
    }
}
#[derive(Clone, Debug)]
struct VacantTileData {
    data: Vec<u8>,
    size: i32,
}
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RemovedTileData {
    pub data: Option<Vec<u8>>,
    pub data_size: i32,
}
#[derive(Clone, Debug)]
pub struct NavMesh {
    pub origin: Point,
    pub tile_width: f32,
    pub tile_height: f32,
    pub tiles: Vec<Option<Tile>>,
    salts: Vec<u32>,
    vacant_data: Vec<Option<VacantTileData>>,
    free: VecDeque<usize>,
    order: Vec<usize>,
    layout: ReferenceLayout,
    max_polygons: i32,
}
#[derive(Clone, Debug)]
pub struct Filter {
    pub include: u16,
    pub exclude: u16,
    pub area_costs: [f32; MAX_AREAS],
}
impl Default for Filter {
    fn default() -> Self {
        Self {
            include: 0xffff,
            exclude: 0,
            area_costs: [1.; MAX_AREAS],
        }
    }
}

/// A polygon and its owning tile at a native filter callback boundary.
#[derive(Clone, Copy, Debug)]
pub struct FilterPolygon<'a> {
    pub reference: PolyRef,
    pub tile: &'a Tile,
    pub polygon: nav_data::PolygonView<'a>,
}

/// Rust counterpart of Detour's virtual query-filter extension. Implementations
/// may replace both eligibility and cost logic, independently of default flags.
pub trait QueryFilter {
    fn pass_filter(&self, polygon: FilterPolygon<'_>) -> bool;
    fn cost(
        &self,
        a: Point,
        b: Point,
        previous: Option<FilterPolygon<'_>>,
        current: FilterPolygon<'_>,
        next: Option<FilterPolygon<'_>>,
    ) -> f32;
}

impl QueryFilter for Filter {
    #[inline]
    fn pass_filter(&self, p: FilterPolygon<'_>) -> bool {
        p.polygon.flags() & self.include != 0 && p.polygon.flags() & self.exclude == 0
    }
    #[inline]
    fn cost(
        &self,
        a: Point,
        b: Point,
        _previous: Option<FilterPolygon<'_>>,
        current: FilterPolygon<'_>,
        _next: Option<FilterPolygon<'_>>,
    ) -> f32 {
        dist(a, b) * self.area_costs[current.polygon.area() as usize]
    }
}
#[derive(Clone, Debug, Default)]
pub struct Corridor {
    pub status: u32,
    pub polygons: Vec<PolyRef>,
}
#[derive(Clone, Debug, Default)]
pub struct StraightPath {
    pub status: u32,
    pub points: Vec<Point>,
    pub flags: Vec<u8>,
    pub references: Vec<PolyRef>,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NearestPoint {
    pub reference: PolyRef,
    pub position: Point,
    pub over_polygon: bool,
}
#[derive(Clone, Debug, Default)]
pub struct RaycastHit {
    pub t: f32,
    pub normal: Point,
    pub edge_index: i32,
    pub path: Vec<PolyRef>,
    pub path_cost: f32,
}
/// Split mutable fields let either path backing share the kernel while keeping
/// partial writes visible even if a caller's filter unwinds.
pub(crate) struct RaycastOutput<'a> {
    t: &'a mut f32,
    normal: &'a mut Point,
    edge_index: &'a mut i32,
    path_cost: &'a mut f32,
}
impl RaycastOutput<'_> {
    pub(crate) fn fraction(&self) -> f32 {
        *self.t
    }
    pub(crate) fn reborrow(&mut self) -> RaycastOutput<'_> {
        RaycastOutput {
            t: self.t,
            normal: self.normal,
            edge_index: self.edge_index,
            path_cost: self.path_cost,
        }
    }
}
impl RaycastHit {
    pub(crate) fn split_output(&mut self) -> (RaycastOutput<'_>, &mut Vec<PolyRef>) {
        (
            RaycastOutput {
                t: &mut self.t,
                normal: &mut self.normal,
                edge_index: &mut self.edge_index,
                path_cost: &mut self.path_cost,
            },
            &mut self.path,
        )
    }
}
pub(crate) fn madd(a: f32, b: f32, c: f32) -> f32 {
    #[cfg(feature = "fma")]
    {
        a.mul_add(b, c)
    }
    #[cfg(not(feature = "fma"))]
    {
        a * b + c
    }
}
// Native accumulators keep the existing cost as the left addend, including
// when both it and a callback result carry different NaN payloads.
#[inline(never)]
fn add_cost(current: f32, next: f32) -> f32 {
    current + next
}
pub(crate) fn length_squared(d: Point) -> f32 {
    madd(d[2], d[2], madd(d[0], d[0], d[1] * d[1]))
}
pub(crate) fn sub(a: Point, b: Point) -> Point {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub(crate) fn dist(a: Point, b: Point) -> f32 {
    let d = sub(a, b);
    length_squared(d).sqrt()
}
pub(crate) fn lerp(a: Point, b: Point, t: f32) -> Point {
    let d = sub(b, a);
    #[cfg(feature = "fma")]
    {
        [
            d[0].mul_add(t, a[0]),
            d[1].mul_add(t, a[1]),
            d[2].mul_add(t, a[2]),
        ]
    }
    #[cfg(not(feature = "fma"))]
    {
        [a[0] + d[0] * t, a[1] + d[1] * t, a[2] + d[2] * t]
    }
}
fn equal(a: Point, b: Point) -> bool {
    let d = sub(a, b);
    length_squared(d) < (1. / 16384.) * (1. / 16384.)
}
pub(crate) fn area(a: Point, b: Point, c: Point) -> f32 {
    crate::common::triangle_area_2d(a, b, c)
}
pub(crate) fn segment(p: Point, a: Point, b: Point) -> (f32, f32) {
    crate::common::distance_point_segment_squared_2d(p, a, b)
}
pub(crate) fn inside(p: Point, verts: &[Point]) -> bool {
    let mut result = false;
    for i in 0..verts.len() {
        let a = verts[i];
        let b = verts[(i + verts.len() - 1) % verts.len()];
        if (a[2] > p[2]) != (b[2] > p[2])
            && p[0] < (b[0] - a[0]) * (p[2] - a[2]) / (b[2] - a[2]) + a[0]
        {
            result = !result;
        }
    }
    result
}
fn height(p: Point, a: Point, b: Point, c: Point) -> Option<f32> {
    let mut height = 0.;
    crate::common::closest_height_point_triangle_into(p, a, b, c, &mut height).then_some(height)
}
impl NavMesh {
    #[allow(clippy::too_many_arguments)]
    pub fn raycast_into<F: QueryFilter + ?Sized>(
        &self,
        start: PolyRef,
        start_pos: Point,
        end_pos: Point,
        filter: &F,
        options: u32,
        previous: PolyRef,
        capacity: usize,
        hit: &mut RaycastHit,
    ) -> u32 {
        hit.path.clear();
        self.raycast_append(
            start, start_pos, end_pos, filter, options, previous, capacity, hit,
        )
    }
    /// Shares the ray kernel while appending to an existing reconstruction prefix.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn raycast_append<F: QueryFilter + ?Sized>(
        &self,
        start: PolyRef,
        start_pos: Point,
        end_pos: Point,
        filter: &F,
        options: u32,
        previous: PolyRef,
        capacity: usize,
        hit: &mut RaycastHit,
    ) -> u32 {
        let (output, path) = hit.split_output();
        self.raycast_append_to_buffer(
            start, start_pos, end_pos, filter, options, previous, capacity, output, path,
        )
    }
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn raycast_append_to_buffer<F: QueryFilter + ?Sized>(
        &self,
        start: PolyRef,
        start_pos: Point,
        end_pos: Point,
        filter: &F,
        options: u32,
        previous: PolyRef,
        capacity: usize,
        hit: RaycastOutput<'_>,
        path: &mut impl PolyBuffer,
    ) -> u32 {
        *hit.t = 0.;
        let path_start = path.len();
        *hit.path_cost = 0.;
        let Some(mut current) = self.filter_polygon(start) else {
            return FAILURE | INVALID_PARAM;
        };
        if !start_pos.iter().chain(&end_pos).all(|v| v.is_finite())
            || capacity > i32::MAX as usize
            || (previous != 0 && self.resolve(previous).is_none())
        {
            return FAILURE | INVALID_PARAM;
        }
        let mut previous_context = if previous == 0 {
            FilterPolygon {
                reference: 0,
                ..current
            }
        } else {
            self.filter_polygon(previous).unwrap()
        };
        let mut next_context = current;
        let mut current_position = start_pos;
        let direction = sub(end_pos, start_pos);
        *hit.normal = [0.; 3];
        let mut status = SUCCESS;
        loop {
            let mut vertices = [[0.; 3]; 6];
            for (dst, id) in vertices.iter_mut().zip(current.polygon.vertices()) {
                *dst = current.tile.vertex_at(usize::from(id));
            }
            let vertices = &vertices[..current.polygon.vertices().len()];
            let clip = crate::common::intersect_segment_polygon_2d(start_pos, end_pos, vertices);
            if !clip.intersects {
                return status;
            }
            *hit.edge_index = clip.leaving_edge;
            if clip.t_max > *hit.t {
                *hit.t = clip.t_max;
            }
            if path.len() - path_start < capacity {
                path.push(current.reference);
            } else {
                status |= BUFFER_TOO_SMALL;
            }
            if clip.leaving_edge == -1 {
                *hit.t = f32::MAX;
                if options & RAYCAST_USE_COSTS != 0 {
                    *hit.path_cost = add_cost(
                        *hit.path_cost,
                        filter.cost(
                            current_position,
                            end_pos,
                            Some(previous_context),
                            current,
                            Some(current),
                        ),
                    );
                }
                return status;
            }
            let edge = clip.leaving_edge as usize;
            let mut next_ref = 0;
            for link in current.tile.links(current.polygon) {
                if usize::from(link.edge()) != edge {
                    continue;
                }
                let Some(candidate) = self.polygon_from_bits(link.reference()) else {
                    return FAILURE | INVALID_PARAM;
                };
                next_context = candidate;
                if next_context.polygon.kind() == 1 || !filter.pass_filter(next_context) {
                    continue;
                }
                if link.side() == 255 || (link.min() == 0 && link.max() == 255) {
                    next_ref = link.reference();
                    break;
                }
                let axis = match link.side() {
                    0 | 4 => 2,
                    2 | 6 => 0,
                    _ => continue,
                };
                let (a, b) = (
                    vertices[edge][axis],
                    vertices[(edge + 1) % vertices.len()][axis],
                );
                let scale = 1. / 255.;
                let mut lo = madd(b - a, link.min() as f32 * scale, a);
                let mut hi = madd(b - a, link.max() as f32 * scale, a);
                if lo > hi {
                    std::mem::swap(&mut lo, &mut hi);
                }
                let p = madd(end_pos[axis] - start_pos[axis], clip.t_max, start_pos[axis]);
                if p >= lo && p <= hi {
                    next_ref = link.reference();
                    break;
                }
            }
            if options & RAYCAST_USE_COSTS != 0 {
                let last = current_position;
                current_position =
                    std::array::from_fn(|i| madd(direction[i], *hit.t, start_pos[i]));
                let (a, b) = (vertices[edge], vertices[(edge + 1) % vertices.len()]);
                let e = sub(b, a);
                let diff = sub(current_position, a);
                let s = if e[0] * e[0] > e[2] * e[2] {
                    diff[0] / e[0]
                } else {
                    diff[2] / e[2]
                };
                current_position[1] = madd(e[1], s, a[1]);
                // Native retains candidate pointers even when its next ref is
                // zero (wall/filter/portal rejection). Preserve that context.
                *hit.path_cost = add_cost(
                    *hit.path_cost,
                    filter.cost(
                        last,
                        current_position,
                        Some(previous_context),
                        current,
                        Some(FilterPolygon {
                            reference: next_ref,
                            ..next_context
                        }),
                    ),
                );
            }
            if next_ref == 0 {
                let (a, b) = (vertices[edge], vertices[(edge + 1) % vertices.len()]);
                *hit.normal = [b[2] - a[2], 0., -(b[0] - a[0])];
                // dtVnormalize's dtSqr calls remain separate at native -O0.
                let n = *hit.normal;
                let inverse = 1. / (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                *hit.normal = n.map(|v| v * inverse);
                return status;
            }
            previous_context = current;
            current = next_context;
        }
    }
    #[allow(clippy::too_many_arguments)]
    pub fn raycast_with_options<F: QueryFilter + ?Sized>(
        &self,
        start: PolyRef,
        start_pos: Point,
        end_pos: Point,
        filter: &F,
        options: u32,
        previous: PolyRef,
        capacity: usize,
    ) -> (u32, RaycastHit) {
        let mut hit = RaycastHit::default();
        let status = self.raycast_into(
            start, start_pos, end_pos, filter, options, previous, capacity, &mut hit,
        );
        (status, hit)
    }
    pub fn raycast<F: QueryFilter + ?Sized>(
        &self,
        start: PolyRef,
        start_pos: Point,
        end_pos: Point,
        filter: &F,
        capacity: usize,
    ) -> (u32, RaycastHit) {
        self.raycast_with_options(start, start_pos, end_pos, filter, 0, 0, capacity)
    }
    pub fn from_tile_data(data: &[u8]) -> Result<Self, u32> {
        Self::from_tile_data_with_flags(data, 0)
    }
    pub fn from_tile_data_with_flags(data: &[u8], flags: i32) -> Result<Self, u32> {
        let size = i32::try_from(data.len()).map_err(|_| FAILURE | INVALID_PARAM)?;
        Self::from_tile_data_sized(data, size, flags)
    }
    pub fn from_tile_data_sized(data: &[u8], data_size: i32, flags: i32) -> Result<Self, u32> {
        let h = nav_data::Header::read(data)?;
        let mut mesh = Self::new(
            h.bounds[0],
            h.bounds[1][0] - h.bounds[0][0],
            h.bounds[1][2] - h.bounds[0][2],
            1,
            h.polygon_count as usize,
        )
        .map_err(|_| FAILURE | INVALID_PARAM)?;
        mesh.add_tile_data_sized(data, data_size, flags, 0)?;
        Ok(mesh)
    }
    pub fn new(
        origin: Point,
        tile_width: f32,
        tile_height: f32,
        max_tiles: usize,
        max_polygons: usize,
    ) -> Result<Self, BuildError> {
        if max_tiles > i32::MAX as usize || max_polygons > i32::MAX as usize {
            return Err(BuildError("invalid navigation mesh capacity".into()));
        }
        Self::from_params(NavMeshParams {
            origin,
            tile_width,
            tile_height,
            max_tiles: max_tiles as i32,
            max_polygons: max_polygons as i32,
        })
    }
    /// Native initialization, retaining signed budgets and every float bit.
    /// Coordinate conversion and reference helpers check their own defined domains.
    pub fn from_params(params: NavMeshParams) -> Result<Self, BuildError> {
        let layout =
            ReferenceLayout::from_native_capacities(params.max_tiles, params.max_polygons)?;
        let max_tiles = params.max_tiles as usize;
        Ok(Self {
            origin: params.origin,
            tile_width: params.tile_width,
            tile_height: params.tile_height,
            tiles: vec![None; max_tiles],
            salts: vec![1; max_tiles],
            vacant_data: vec![None; max_tiles],
            free: (0..max_tiles).collect(),
            order: vec![],
            layout,
            max_polygons: params.max_polygons,
        })
    }
    #[inline]
    pub fn reference_layout(&self) -> ReferenceLayout {
        self.layout
    }
    #[inline]
    pub fn reference(&self, tile: usize, poly: usize) -> PolyRef {
        self.layout
            .encode(self.salts[tile], tile as u32, poly as u32)
    }
    #[inline]
    pub fn resolve(&self, reference: PolyRef) -> Option<(usize, usize)> {
        if reference == 0 {
            return None;
        }
        let (salt, t, p) = self.layout.decode(reference);
        let t = t as usize;
        let p = p as usize;
        let tile = self.tiles.get(t)?.as_ref()?;
        if self.salts[t] != salt || p >= tile.polygon_count() {
            return None;
        }
        Some((t, p))
    }
    pub fn is_valid_poly_ref(&self, reference: PolyRef) -> bool {
        self.resolve(reference).is_some()
    }
    pub fn tile_and_polygon_by_ref(&self, reference: PolyRef) -> Result<FilterPolygon<'_>, u32> {
        if reference == 0 {
            return Err(FAILURE);
        }
        self.filter_polygon(reference)
            .ok_or(FAILURE | INVALID_PARAM)
    }
    #[inline]
    pub fn filter_polygon(&self, reference: PolyRef) -> Option<FilterPolygon<'_>> {
        let (t, p) = self.resolve(reference)?;
        let tile = self.tiles[t].as_ref()?;
        Some(FilterPolygon {
            reference,
            tile,
            polygon: tile.polygon_at(p),
        })
    }
    /// Counterpart of native getTileAndPolyByRefUnsafe: decodes without checking
    /// zero/salt, while keeping memory bounds checked. Stale refs may resolve.
    #[inline]
    pub fn polygon_from_bits(&self, reference: PolyRef) -> Option<FilterPolygon<'_>> {
        let (_, t, p) = self.layout.decode(reference);
        let tile = self.tiles.get(t as usize)?.as_ref()?;
        Some(FilterPolygon {
            reference,
            tile,
            polygon: tile.polygon(p as usize)?,
        })
    }
    pub fn params(&self) -> NavMeshParams {
        NavMeshParams {
            origin: self.origin,
            tile_width: self.tile_width,
            tile_height: self.tile_height,
            max_tiles: self.tiles.len() as i32,
            max_polygons: self.max_polygons,
        }
    }
    /// Native floor conversion with checked non-finite/out-of-range results.
    pub fn calc_tile_location(&self, position: Point) -> Option<[i32; 2]> {
        let coordinates = [
            ((position[0] - self.origin[0]) / self.tile_width).floor(),
            ((position[2] - self.origin[2]) / self.tile_height).floor(),
        ];
        coordinates
            .iter()
            .all(|&v| {
                v.is_finite()
                    && f64::from(v) >= f64::from(i32::MIN)
                    && f64::from(v) < f64::from(i32::MAX) + 1.
            })
            .then(|| coordinates.map(|v| v as i32))
    }
    pub fn max_tiles(&self) -> usize {
        self.tiles.len()
    }
    pub fn tile(&self, index: i32) -> Option<TileSlot<'_>> {
        let index = usize::try_from(index).ok()?;
        Some(TileSlot {
            index,
            salt: *self.salts.get(index)?,
            tile: self.tiles.get(index)?.as_ref(),
            flags: self.tiles[index].as_ref().map_or(0, |t| t.flags),
            data_size: self.tiles[index].as_ref().map_or_else(
                || self.vacant_data[index].as_ref().map_or(0, |v| v.size),
                |t| t.data_size,
            ),
            free_link: self.tiles[index].as_ref().map_or(0, |t| t.free_link),
            retained_data: self.vacant_data[index].as_ref().map(|v| v.data.as_slice()),
        })
    }
    /// A reference with the current salt may address an unloaded slot. Polygon
    /// bits are ignored, as in native getTileByRef.
    pub fn tile_by_ref(&self, reference: TileRef) -> Option<TileSlot<'_>> {
        if reference == 0 {
            return None;
        }
        let (salt, index, _) = self.layout.decode(reference);
        let slot = self.tile(i32::try_from(index).ok()?)?;
        (slot.salt == salt).then_some(slot)
    }
    /// Index-based counterpart of getTileRef/getPolyRefBase; None is native null.
    pub fn tile_reference(&self, index: Option<usize>) -> TileRef {
        index
            .filter(|&i| i < self.tiles.len())
            .map_or(0, |i| self.reference(i, 0))
    }
    pub fn tile_at(&self, coordinate: [i32; 3]) -> Option<TileSlot<'_>> {
        self.order.iter().find_map(|&i| {
            self.tiles[i]
                .as_ref()
                .filter(|t| t.coordinate == coordinate)
                .and_then(|_| self.tile(i as i32))
        })
    }
    pub fn tile_ref_at(&self, coordinate: [i32; 3]) -> TileRef {
        self.tile_reference(self.tile_at(coordinate).map(|s| s.index))
    }
    fn tile_indices_at(&self, x: i32, z: i32) -> impl Iterator<Item = usize> + '_ {
        self.order.iter().copied().filter(move |&i| {
            self.tiles[i]
                .as_ref()
                .is_some_and(|t| t.coordinate[..2] == [x, z])
        })
    }
    // Native internal lookups snapshot at most 32 layers before mutating links.
    fn tile_indices_at_into(&self, x: i32, z: i32, output: &mut [usize; 32]) -> usize {
        let mut count = 0;
        for (target, index) in output.iter_mut().zip(self.tile_indices_at(x, z)) {
            *target = index;
            count += 1;
        }
        count
    }
    pub fn tiles_at_bounded(&self, x: i32, z: i32, capacity: usize) -> Vec<usize> {
        self.tile_indices_at(x, z).take(capacity).collect()
    }
    /// Convenience lookup with the native internal neighbor limit. Use
    /// tiles_at_bounded for the public native overload's caller-chosen capacity.
    pub fn tiles_at(&self, x: i32, z: i32) -> Vec<usize> {
        self.tiles_at_bounded(x, z, 32)
    }
    pub fn remove_tile(&mut self, coordinate: [i32; 3]) -> bool {
        let Some(slot) = self
            .order
            .iter()
            .copied()
            .find(|&i| self.tiles[i].as_ref().unwrap().coordinate == coordinate)
        else {
            return false;
        };
        self.remove_slot(slot);
        true
    }
    fn remove_slot(&mut self, slot: usize) {
        let tile = self.tiles[slot].as_ref().unwrap();
        let coordinate = tile.coordinate;
        self.order.retain(|&t| t != slot);
        let mut targets = [0; 32];
        let count = self.tile_indices_at_into(coordinate[0], coordinate[1], &mut targets);
        for &target in &targets[..count] {
            self.tiles[target]
                .as_mut()
                .unwrap()
                .disconnect(slot, self.layout);
        }
        for side in 0..8 {
            let (dx, dz) = neighbor_offset(side);
            let count =
                self.tile_indices_at_into(coordinate[0] + dx, coordinate[1] + dz, &mut targets);
            for &target in &targets[..count] {
                self.tiles[target]
                    .as_mut()
                    .unwrap()
                    .disconnect(slot, self.layout);
            }
        }
        let mut tile = self.tiles[slot].take().unwrap();
        self.vacant_data[slot] = if tile.flags & TILE_FREE_DATA == 0 {
            // The vacant slot retains this same backing buffer. Refresh public
            // header fields before transferring it, as a full export would.
            let (header, _) = tile.data_parts();
            tile.source_data[..nav_data::HEADER_SIZE].copy_from_slice(&header);
            Some(VacantTileData {
                data: tile.source_data,
                size: tile.data_size,
            })
        } else {
            None
        };
        self.salts[slot] = (self.salts[slot] + 1) & self.layout.salt_mask();
        if self.salts[slot] == 0 {
            self.salts[slot] = 1;
        }
        self.free.push_front(slot);
    }
    pub fn add_tile(
        &mut self,
        coordinate: [i32; 3],
        layer: &HeightfieldLayer,
        mesh: &CacheMesh,
        height: f32,
        radius: f32,
        climb: f32,
    ) -> Result<(), BuildError> {
        if mesh.polygons.is_empty() {
            return Ok(());
        }
        let polygons: Vec<_> = mesh
            .polygons
            .iter()
            .enumerate()
            .map(|(i, p)| nav_data::CreatePolygon {
                vertices: p[..6].try_into().unwrap(),
                neighbors: p[6..].try_into().unwrap(),
                flags: 1,
                area: mesh.areas[i],
            })
            .collect();
        let data = nav_data::create(&nav_data::CreateParams {
            vertices: &mesh.vertices,
            polygons: &polygons,
            vertices_per_polygon: 6,
            detail_meshes: None,
            detail_vertices: &[],
            detail_triangles: &[],
            off_mesh_connections: &[],
            off_mesh_user_ids: None,
            coordinate,
            user_id: 0,
            bounds: layer.bounds,
            walkable_height: height,
            walkable_radius: radius,
            walkable_climb: climb,
            cell_size: layer.cell_size,
            cell_height: layer.cell_height,
            build_bv_tree: false,
        })?;
        self.add_tile_data_with_flags(&data, TILE_FREE_DATA, 0)
            .map(|_| ())
            .map_err(|s| BuildError(format!("add navigation tile failed: {s:#x}")))
    }
    /// Loads native v7 tile data using the selected reference width. Rust owns a
    /// copy. `last_ref` restores a free slot and its salt, as native addTile does.
    pub fn add_tile_data(&mut self, data: &[u8], last_ref: TileRef) -> Result<TileRef, u32> {
        self.add_tile_data_with_flags(data, 0, last_ref)
    }
    pub fn add_tile_data_with_flags(
        &mut self,
        data: &[u8],
        flags: i32,
        last_ref: TileRef,
    ) -> Result<TileRef, u32> {
        let size = i32::try_from(data.len()).map_err(|_| FAILURE | INVALID_PARAM)?;
        self.add_tile_data_sized(data, size, flags, last_ref)
    }
    /// Takes ownership of the backing buffer, avoiding a second tile-byte copy.
    /// Native flags and removal behavior remain independent of Rust ownership.
    pub fn add_tile_data_owned(
        &mut self,
        data: Vec<u8>,
        flags: i32,
        last_ref: TileRef,
    ) -> Result<TileRef, u32> {
        let size = i32::try_from(data.len()).map_err(|_| FAILURE | INVALID_PARAM)?;
        self.add_tile_data_storage(Cow::Owned(data), size, flags, last_ref)
    }
    /// Keeps the declared native size independently of the safe backing slice.
    /// Native addTile records this value without using it for geometry access.
    pub fn add_tile_data_sized(
        &mut self,
        data: &[u8],
        data_size: i32,
        flags: i32,
        last_ref: TileRef,
    ) -> Result<TileRef, u32> {
        self.add_tile_data_storage(Cow::Borrowed(data), data_size, flags, last_ref)
    }
    fn add_tile_data_storage(
        &mut self,
        data: Cow<'_, [u8]>,
        data_size: i32,
        flags: i32,
        last_ref: TileRef,
    ) -> Result<TileRef, u32> {
        let view = nav_data::TileData::parse(&data)?;
        let h = &view.header;
        #[cfg(not(feature = "polyref64"))]
        if h.polygon_count as usize > (1usize << self.layout.poly_bits) {
            return Err(FAILURE | INVALID_PARAM);
        }
        if self
            .order
            .iter()
            .any(|&i| self.tiles[i].as_ref().unwrap().coordinate == h.coordinate)
        {
            return Err(FAILURE | ALREADY_OCCUPIED);
        }
        let mut tile = Tile::from_data(&view)?;
        tile.source_data = data.into_owned();
        let count = tile.link_count();
        let links = tile.source_data
            [tile.link_start..tile.link_start + count * nav_data::LINK_SIZE]
            .as_chunks_mut::<{ nav_data::LINK_SIZE }>()
            .0;
        // Loading requires at least one link record. Keep the native final
        // sentinel separate from the contiguous free-list initialization loop.
        let (last, links) = links.split_last_mut().unwrap();
        for (i, bytes) in links.iter_mut().enumerate() {
            bytes[nav_data::LINK_SIZE - 8..nav_data::LINK_SIZE - 4]
                .copy_from_slice(&((i + 1) as u32).to_ne_bytes());
        }
        last[nav_data::LINK_SIZE - 8..nav_data::LINK_SIZE - 4]
            .copy_from_slice(&NULL_LINK.to_ne_bytes());
        tile.flags = flags;
        tile.data_size = data_size;
        let slot = if last_ref == 0 {
            self.free.pop_front().ok_or(FAILURE | OUT_OF_MEMORY)?
        } else {
            let (salt, index, _) = self.layout.decode(last_ref);
            let index = index as usize;
            let at = self
                .free
                .iter()
                .position(|&i| i == index)
                .ok_or(FAILURE | OUT_OF_MEMORY)?;
            self.free.remove(at);
            self.salts[index] = salt;
            index
        };
        tile.connect_internal_links(self.reference(slot, 0));
        let coordinate = tile.coordinate;
        self.tiles[slot] = Some(tile);
        self.vacant_data[slot] = None;
        self.order.insert(0, slot);
        self.base_off_mesh(slot);
        self.connect_off_mesh(slot, slot, None);
        let mut targets = [0; 32];
        let count = self.tile_indices_at_into(coordinate[0], coordinate[1], &mut targets);
        for &target in &targets[..count] {
            if target != slot {
                self.connect(slot, target, None);
                self.connect(target, slot, None);
                self.connect_off_mesh(slot, target, None);
                self.connect_off_mesh(target, slot, None);
            }
        }
        for side in 0..8 {
            let (dx, dz) = neighbor_offset(side);
            let count =
                self.tile_indices_at_into(coordinate[0] + dx, coordinate[1] + dz, &mut targets);
            for &target in &targets[..count] {
                self.connect(slot, target, Some(side as u8));
                self.connect(target, slot, Some((side as u8 + 4) & 7));
                self.connect_off_mesh(slot, target, Some(side as u8));
                self.connect_off_mesh(target, slot, Some((side as u8 + 4) & 7));
            }
        }
        Ok(self.reference(slot, 0))
    }
    fn tile_slot(&self, reference: TileRef) -> Option<usize> {
        if reference == 0 {
            return None;
        }
        let (salt, slot, _) = self.layout.decode(reference);
        let slot = slot as usize;
        (self.tiles.get(slot)?.is_some() && self.salts[slot] == salt).then_some(slot)
    }
    /// Exports the complete native buffer, retaining padding and unused link data.
    pub fn tile_data(&self, reference: TileRef) -> Result<Vec<u8>, u32> {
        let mut data = Vec::new();
        self.tile_data_into(reference, &mut data)?;
        Ok(data)
    }
    /// Reuses export storage. An invalid reference retains the caller's buffer.
    pub fn tile_data_into(&self, reference: TileRef, data: &mut Vec<u8>) -> Result<(), u32> {
        let slot = self.tile_slot(reference).ok_or(FAILURE | INVALID_PARAM)?;
        self.tiles[slot].as_ref().unwrap().data_into(data)
    }
    /// Returns a byte backup regardless of flags. Use ownership-aware removal
    /// for native free-data/null output semantics.
    pub fn remove_tile_data(&mut self, reference: TileRef) -> Result<Vec<u8>, u32> {
        let slot = self.tile_slot(reference).ok_or(FAILURE | INVALID_PARAM)?;
        let data = self.tile_data(reference)?;
        self.remove_slot(slot);
        Ok(data)
    }
    /// Native ownership-aware removal; failure retains both caller outputs.
    pub fn remove_tile_data_into(
        &mut self,
        reference: TileRef,
        output: &mut RemovedTileData,
    ) -> u32 {
        let Some(slot) = self.tile_slot(reference) else {
            return FAILURE | INVALID_PARAM;
        };
        self.remove_slot(slot);
        match &self.vacant_data[slot] {
            Some(retained) => {
                output.data = Some(retained.data.clone());
                output.data_size = retained.size;
            }
            None => {
                output.data = None;
                output.data_size = 0;
            }
        }
        SUCCESS
    }
    pub fn remove_tile_data_with_ownership(
        &mut self,
        reference: TileRef,
    ) -> Result<RemovedTileData, u32> {
        let mut output = RemovedTileData::default();
        let status = self.remove_tile_data_into(reference, &mut output);
        if status & FAILURE != 0 {
            Err(status)
        } else {
            Ok(output)
        }
    }
    pub fn tile_state_size(&self, reference: TileRef) -> usize {
        self.tile_slot(reference).map_or(0, |slot| {
            8 + std::mem::size_of::<TileRef>()
                + self.tiles[slot].as_ref().unwrap().polygon_count() * 4
        })
    }
    pub fn store_tile_state(&self, reference: TileRef, data: &mut [u8]) -> u32 {
        let Some(slot) = self.tile_slot(reference) else {
            return FAILURE | INVALID_PARAM;
        };
        if data.len() < self.tile_state_size(reference) {
            return FAILURE | BUFFER_TOO_SMALL;
        }
        data[..4].copy_from_slice(&nav_data::STATE_MAGIC.to_ne_bytes());
        data[4..8].copy_from_slice(&nav_data::STATE_VERSION.to_ne_bytes());
        let base = 8 + std::mem::size_of::<TileRef>();
        data[8..base].copy_from_slice(&self.reference(slot, 0).to_ne_bytes());
        for (poly, record) in self.tiles[slot]
            .as_ref()
            .unwrap()
            .polygons()
            .zip(data[base..].chunks_exact_mut(4))
        {
            record[..2].copy_from_slice(&poly.flags().to_ne_bytes());
            record[2] = poly.area();
        }
        SUCCESS
    }
    pub fn restore_tile_state(&mut self, reference: TileRef, data: &[u8]) -> u32 {
        let Some(slot) = self.tile_slot(reference) else {
            return FAILURE | INVALID_PARAM;
        };
        if data.len() < self.tile_state_size(reference) {
            return FAILURE | INVALID_PARAM;
        }
        if u32::from_ne_bytes(data[..4].try_into().unwrap()) != nav_data::STATE_MAGIC {
            return FAILURE | WRONG_MAGIC;
        }
        if u32::from_ne_bytes(data[4..8].try_into().unwrap()) != nav_data::STATE_VERSION {
            return FAILURE | WRONG_VERSION;
        }
        let base = 8 + std::mem::size_of::<TileRef>();
        if TileRef::from_ne_bytes(data[8..base].try_into().unwrap()) != self.reference(slot, 0) {
            return FAILURE | INVALID_PARAM;
        }
        for (mut poly, record) in self.tiles[slot]
            .as_mut()
            .unwrap()
            .polygons_mut()
            .zip(data[base..].chunks_exact(4))
        {
            poly.set_flags(u16::from_ne_bytes(record[..2].try_into().unwrap()));
            poly.set_area(record[2]);
        }
        SUCCESS
    }
    pub fn set_poly_flags(&mut self, reference: PolyRef, flags: u16) -> u32 {
        if reference == 0 {
            return FAILURE;
        }
        let Some((t, p)) = self.resolve(reference) else {
            return FAILURE | INVALID_PARAM;
        };
        self.tiles[t]
            .as_mut()
            .unwrap()
            .polygon_mut(p)
            .unwrap()
            .set_flags(flags);
        SUCCESS
    }
    pub fn poly_flags(&self, reference: PolyRef) -> Result<u16, u32> {
        if reference == 0 {
            return Err(FAILURE);
        }
        let (t, p) = self.resolve(reference).ok_or(FAILURE | INVALID_PARAM)?;
        Ok(self.tiles[t].as_ref().unwrap().polygon_at(p).flags())
    }
    pub fn set_poly_area(&mut self, reference: PolyRef, area: u8) -> u32 {
        if reference == 0 {
            return FAILURE;
        }
        let Some((t, p)) = self.resolve(reference) else {
            return FAILURE | INVALID_PARAM;
        };
        self.tiles[t]
            .as_mut()
            .unwrap()
            .polygon_mut(p)
            .unwrap()
            .set_area(area);
        SUCCESS
    }
    pub fn poly_area(&self, reference: PolyRef) -> Result<u8, u32> {
        if reference == 0 {
            return Err(FAILURE);
        }
        let (t, p) = self.resolve(reference).ok_or(FAILURE | INVALID_PARAM)?;
        Ok(self.tiles[t].as_ref().unwrap().polygon_at(p).area())
    }
    pub fn off_mesh_connection(&self, reference: PolyRef) -> Option<nav_data::OffMeshConnection> {
        let (t, p) = self.resolve(reference)?;
        let tile = self.tiles[t].as_ref()?;
        if tile.polygon_at(p).kind() != 1 {
            return None;
        }
        tile.off_mesh_connection(p - tile.header.off_mesh_base as usize)
    }
    pub fn off_mesh_endpoints(
        &self,
        previous: PolyRef,
        reference: PolyRef,
    ) -> Result<[Point; 2], u32> {
        if reference == 0 {
            return Err(FAILURE);
        }
        let (t, p) = self.resolve(reference).ok_or(FAILURE | INVALID_PARAM)?;
        let tile = self.tiles[t].as_ref().unwrap();
        let poly = tile.polygon_at(p);
        if poly.kind() != 1 {
            return Err(FAILURE);
        }
        let reverse = tile
            .links(poly)
            .find(|l| l.edge() == 0)
            .is_some_and(|l| l.reference() != previous);
        let (a, b) = if reverse { (1, 0) } else { (0, 1) };
        Ok([
            tile.vertex_at(poly.vertex(a) as usize),
            tile.vertex_at(poly.vertex(b) as usize),
        ])
    }
    fn nearest_in_tile(
        &self,
        slot: usize,
        point: Point,
        extents: Point,
    ) -> Option<(PolyRef, Point)> {
        let tile = self.tiles[slot].as_ref().unwrap();
        let lo = std::array::from_fn(|i| point[i] - extents[i]);
        let hi = std::array::from_fn(|i| point[i] + extents[i]);
        let mut best = f32::MAX;
        let mut result = None;
        for p in tile.candidates(lo, hi, |_| true).take(128) {
            let reference = self.reference(slot, p);
            let Some((position, over)) = tile.closest_point(p, point) else {
                continue;
            };
            let diff = sub(point, position);
            let d = if over {
                let d = diff[1].abs() - tile.climb;
                if d > 0. { d * d } else { 0. }
            } else {
                length_squared(diff)
            };
            if d < best {
                best = d;
                result = Some((reference, position));
            }
        }
        result
    }
    fn base_off_mesh(&mut self, slot: usize) {
        for index in 0..self.tiles[slot]
            .as_ref()
            .unwrap()
            .off_mesh_connection_count()
        {
            // Linking mutates vertices and links, never the connection records.
            let con = self.tiles[slot]
                .as_ref()
                .unwrap()
                .off_mesh_connection(index)
                .unwrap();
            let climb = self.tiles[slot].as_ref().unwrap().climb;
            let p = con.positions[0];
            let Some((reference, nearest)) =
                self.nearest_in_tile(slot, p, [con.radius, climb, con.radius])
            else {
                continue;
            };
            if reference == 0 {
                continue;
            }
            // Native dtSqr calls are separate function boundaries in the oracle.
            let (dx, dz) = (nearest[0] - p[0], nearest[2] - p[2]);
            if dx * dx + dz * dz > con.radius * con.radius {
                continue;
            }
            let off_reference = self.reference(slot, con.polygon as usize);
            let landing = self.layout.decode(reference).2 as u16 as usize;
            let tile = self.tiles[slot].as_mut().unwrap();
            let vertex = tile.polygon_at(con.polygon as usize).vertex(0);
            tile.set_vertex(usize::from(vertex), nearest).unwrap();
            tile.add_link(
                con.polygon as usize,
                Link {
                    reference,
                    next: NULL_LINK,
                    edge: 0,
                    side: 255,
                    min: 0,
                    max: 0,
                },
            );
            tile.add_link(
                landing,
                Link {
                    reference: off_reference,
                    next: NULL_LINK,
                    edge: 255,
                    side: 255,
                    min: 0,
                    max: 0,
                },
            );
        }
    }
    fn connect_off_mesh(&mut self, landing_tile: usize, owner: usize, side: Option<u8>) {
        let opposite = side.map_or(255, |s| (s + 4) & 7);
        for index in 0..self.tiles[owner]
            .as_ref()
            .unwrap()
            .off_mesh_connection_count()
        {
            // Linking mutates vertices and links, never the connection records.
            let con = self.tiles[owner]
                .as_ref()
                .unwrap()
                .off_mesh_connection(index)
                .unwrap();
            let target = self.tiles[owner].as_ref().unwrap();
            if con.side != opposite
                || target.polygon_at(con.polygon as usize).first_link() == NULL_LINK
            {
                continue;
            }
            let p = con.positions[1];
            let Some((reference, nearest)) =
                self.nearest_in_tile(landing_tile, p, [con.radius, target.climb, con.radius])
            else {
                continue;
            };
            if reference == 0 {
                continue;
            }
            let (dx, dz) = (nearest[0] - p[0], nearest[2] - p[2]);
            if dx * dx + dz * dz > con.radius * con.radius {
                continue;
            }
            let off_reference = self.reference(owner, con.polygon as usize);
            let target = self.tiles[owner].as_mut().unwrap();
            let vertex = target.polygon_at(con.polygon as usize).vertex(1);
            target.set_vertex(usize::from(vertex), nearest).unwrap();
            target.add_link(
                con.polygon as usize,
                Link {
                    reference,
                    next: NULL_LINK,
                    edge: 1,
                    side: opposite,
                    min: 0,
                    max: 0,
                },
            );
            if con.flags & 1 != 0 {
                let landing = self.layout.decode(reference).2 as u16 as usize;
                self.tiles[landing_tile].as_mut().unwrap().add_link(
                    landing,
                    Link {
                        reference: off_reference,
                        next: NULL_LINK,
                        edge: 255,
                        side: side.unwrap_or(255),
                        min: 0,
                        max: 0,
                    },
                );
            }
        }
    }
    fn connect(&mut self, from: usize, to: usize, side: Option<u8>) {
        let target_base = self.reference(to, 0);
        // Callers connect distinct loaded tiles. Attach each matching link
        // directly; link insertion cannot change either tile's portal geometry.
        let [tile, target] = self.tiles.get_disjoint_mut([from, to]).unwrap();
        let tile = tile.as_mut().unwrap();
        let target = target.as_ref().unwrap();
        // A conservative whole-tile scan avoids visiting every active edge
        // when none of the fixed neighbor records can contain a portal.
        if !tile.polygons().any(|poly| poly.may_have_portal()) {
            return;
        }
        for pi in 0..tile.polygon_count() {
            let vertex_count = tile.polygon_at(pi).vertices().len();
            for edge in 0..vertex_count {
                let poly = tile.polygon_at(pi);
                let n = poly.neighbor(edge);
                if n & 0x8000 == 0 {
                    continue;
                }
                let dir = (n & 255) as u8;
                if side.is_some_and(|side| side != dir) {
                    continue;
                }
                let a = tile.vertex_at(poly.vertex(edge) as usize);
                let b = tile.vertex_at(poly.vertex((edge + 1) % vertex_count) as usize);
                let axis = if dir == 0 || dir == 4 { 2 } else { 0 };
                let fixed = if axis == 2 { 0 } else { 2 };
                let mut found = 0;
                for (pj, other) in target.polygons().enumerate() {
                    for e in 0..other.vertices().len() {
                        if other.neighbor(e) != (0x8000 | ((dir as u16 + 4) & 7)) {
                            continue;
                        }
                        let c = target.vertex_at(other.vertex(e) as usize);
                        let d = target
                            .vertex_at(other.vertex((e + 1) % other.vertices().len()) as usize);
                        if (a[fixed] - c[fixed]).abs() > 0.01 {
                            continue;
                        }
                        let (amin, amax) = if a[axis] < b[axis] { (a, b) } else { (b, a) };
                        let (bmin, bmax) = if c[axis] < d[axis] { (c, d) } else { (d, c) };
                        let lo = (amin[axis] + 0.01).max(bmin[axis] + 0.01);
                        let hi = (amax[axis] - 0.01).min(bmax[axis] - 0.01);
                        if lo > hi {
                            continue;
                        }
                        let ad = (amax[1] - amin[1]) / (amax[axis] - amin[axis]);
                        let ak = madd(-ad, amin[axis], amin[1]);
                        let bd = (bmax[1] - bmin[1]) / (bmax[axis] - bmin[axis]);
                        let bk = madd(-bd, bmin[axis], bmin[1]);
                        let dmin = madd(bd, lo, bk) - madd(ad, lo, ak);
                        let dmax = madd(bd, hi, bk) - madd(ad, hi, ak);
                        let threshold = (target.climb * 2.) * (target.climb * 2.);
                        if dmin * dmax >= 0. && dmin * dmin > threshold && dmax * dmax > threshold {
                            continue;
                        }
                        if found < 4 {
                            let ta = (amin[axis].max(bmin[axis]) - a[axis]) / (b[axis] - a[axis]);
                            let tb = (amax[axis].min(bmax[axis]) - a[axis]) / (b[axis] - a[axis]);
                            tile.add_link(
                                pi,
                                Link {
                                    reference: target_base | pj as PolyRef,
                                    next: NULL_LINK,
                                    edge: edge as u8,
                                    side: dir,
                                    min: (ta.min(tb).clamp(0., 1.) * 255.).round() as u8,
                                    max: (ta.max(tb).clamp(0., 1.) * 255.).round() as u8,
                                },
                            );
                            found += 1;
                        }
                        break;
                    }
                }
            }
        }
    }
    pub fn closest_point(&self, reference: PolyRef, point: Point) -> Option<(Point, bool)> {
        if !point.iter().all(|v| v.is_finite()) {
            return None;
        }
        let (t, p) = self.resolve(reference)?;
        let tile = self.tiles[t].as_ref()?;
        tile.closest_point(p, point)
    }

    /// Native getPolyHeight accepts a non-finite Y coordinate; only XZ is used.
    pub fn poly_height(&self, reference: PolyRef, point: Point) -> Result<f32, u32> {
        let (t, p) = self.resolve(reference).ok_or(FAILURE | INVALID_PARAM)?;
        if !point[0].is_finite() || !point[2].is_finite() {
            return Err(FAILURE | INVALID_PARAM);
        }
        let tile = self.tiles[t].as_ref().unwrap();
        let poly = tile.polygon_at(p);
        if poly.kind() == 1 {
            let (a, b) = (
                tile.vertex_at(poly.vertex(0) as usize),
                tile.vertex_at(poly.vertex(1) as usize),
            );
            return Ok(madd(b[1] - a[1], segment(point, a, b).1, a[1]));
        }
        tile.ground_height(p, point).ok_or(FAILURE | INVALID_PARAM)
    }
    /// Calls `process` with native batches of at most 32 polygons, flushing at
    /// each tile boundary. Filter callbacks retain upstream traversal order.
    pub fn query_polygons_with<'a, F: QueryFilter + ?Sized>(
        &'a self,
        point: Point,
        extents: Point,
        filter: &F,
        mut process: impl FnMut(&[FilterPolygon<'a>]),
    ) -> u32 {
        if !point.iter().chain(&extents).all(|v| v.is_finite()) {
            return FAILURE | INVALID_PARAM;
        }
        let lo = [
            point[0] - extents[0],
            point[1] - extents[1],
            point[2] - extents[2],
        ];
        let hi = [
            point[0] + extents[0],
            point[1] + extents[1],
            point[2] + extents[2],
        ];
        let Some([minx, minz]) = self.calc_tile_location(lo) else {
            return FAILURE | INVALID_PARAM;
        };
        let Some([maxx, maxz]) = self.calc_tile_location(hi) else {
            return FAILURE | INVALID_PARAM;
        };
        for z in minz..=maxz {
            for x in minx..=maxx {
                for t in self.tile_indices_at(x, z).take(32) {
                    let tile = self.tiles[t].as_ref().unwrap();
                    let polygons = tile.polygon_records();
                    let base = self.reference(t, 0);
                    let mut batch: Option<([FilterPolygon<'a>; 32], usize)> = None;
                    for p in tile.candidates(lo, hi, |p| {
                        filter.pass_filter(FilterPolygon {
                            reference: base | p as PolyRef,
                            tile,
                            polygon: nav_data::PolygonView(&polygons[p]),
                        })
                    }) {
                        let context = FilterPolygon {
                            reference: base | p as PolyRef,
                            tile,
                            polygon: nav_data::PolygonView(&polygons[p]),
                        };
                        // Initialize the full batch only once per tile, not once
                        // for every candidate passed to this callback batch.
                        #[allow(clippy::unnecessary_lazy_evaluations)]
                        let (items, n) = batch.get_or_insert_with(|| ([context; 32], 0));
                        items[*n] = context;
                        *n += 1;
                        if *n == 32 {
                            process(items);
                            *n = 0;
                        }
                    }
                    if let Some((items, n)) = batch
                        && n != 0
                    {
                        process(&items[..n]);
                    }
                }
            }
        }
        SUCCESS
    }
    pub fn query_polygons<F: QueryFilter + ?Sized>(
        &self,
        point: Point,
        extents: Point,
        filter: &F,
        capacity: usize,
    ) -> (u32, Vec<PolyRef>) {
        if capacity > i32::MAX as usize {
            return (FAILURE | INVALID_PARAM, Vec::new());
        }
        let mut result = Vec::new();
        let mut overflow = false;
        let status = self.query_polygons_with(point, extents, filter, |batch| {
            let n = batch.len().min(capacity - result.len());
            if n < batch.len() {
                overflow = true;
            }
            result.extend(batch[..n].iter().map(|p| p.reference));
        });
        (status | if overflow { BUFFER_TOO_SMALL } else { 0 }, result)
    }
    pub fn nearest<F: QueryFilter + ?Sized>(
        &self,
        point: Point,
        extents: Point,
        filter: &F,
    ) -> (u32, Option<(PolyRef, Point)>) {
        let (status, found) = self.nearest_with_over(point, extents, filter);
        (status, found.map(|p| (p.reference, p.position)))
    }
    pub fn nearest_with_over<F: QueryFilter + ?Sized>(
        &self,
        point: Point,
        extents: Point,
        filter: &F,
    ) -> (u32, Option<NearestPoint>) {
        let mut nearest = None;
        let mut distance = f32::MAX;
        let mut invalid = false;
        let status = self.query_polygons_with(point, extents, filter, |batch| {
            for context in batch {
                let Some((position, over)) = self.closest_point(context.reference, point) else {
                    invalid = true;
                    continue;
                };
                let diff = sub(point, position);
                let d = if over {
                    let d = diff[1].abs() - context.tile.climb;
                    if d > 0. { d * d } else { 0. }
                } else {
                    length_squared(diff)
                };
                if d < distance {
                    distance = d;
                    nearest = Some(NearestPoint {
                        reference: context.reference,
                        position,
                        over_polygon: over,
                    });
                }
            }
        });
        if invalid {
            (FAILURE | INVALID_PARAM, None)
        } else {
            (status, nearest)
        }
    }
    pub fn portal(&self, from: PolyRef, to: PolyRef) -> Option<(Point, Point)> {
        Self::portal_context(self.filter_polygon(from)?, self.filter_polygon(to)?)
    }
    pub(crate) fn portal_context(
        from: FilterPolygon<'_>,
        to: FilterPolygon<'_>,
    ) -> Option<(Point, Point)> {
        let link = from
            .tile
            .links(from.polygon)
            .find(|l| l.reference() == to.reference)?;
        Self::portal_with_link(from, to, link, true)
    }
    #[inline]
    pub(crate) fn portal_with_link(
        from: FilterPolygon<'_>,
        to: FilterPolygon<'_>,
        link: nav_data::LinkView<'_>,
        first: bool,
    ) -> Option<(Point, Point)> {
        let link = if first {
            link
        } else {
            from.tile
                .links(from.polygon)
                .find(|l| l.reference() == to.reference)?
        };
        let tile = from.tile;
        let poly = from.polygon;
        if poly.kind() == 1 {
            let v = tile.vertex_at(poly.vertex(usize::from(link.edge())) as usize);
            return Some((v, v));
        }
        if to.polygon.kind() == 1 {
            let back = to
                .tile
                .links(to.polygon)
                .find(|l| l.reference() == from.reference)?;
            let v = to
                .tile
                .vertex_at(to.polygon.vertex(usize::from(back.edge())) as usize);
            return Some((v, v));
        }
        Some(Self::ground_portal(tile, poly, link))
    }
    // Keep the common ground-to-ground Dijkstra edge small enough to inline;
    // duplicate references and off-mesh connections retain the full traversal.
    #[inline(always)]
    pub(crate) fn portal_points(
        from: FilterPolygon<'_>,
        to: FilterPolygon<'_>,
        link: nav_data::LinkView<'_>,
        first: bool,
    ) -> Option<(Point, Point)> {
        if first && from.polygon.kind() != 1 && to.polygon.kind() != 1 {
            Some(Self::ground_portal(from.tile, from.polygon, link))
        } else {
            Self::portal_with_link(from, to, link, first)
        }
    }
    #[inline(always)]
    fn ground_portal(
        tile: &Tile,
        poly: nav_data::PolygonView<'_>,
        link: nav_data::LinkView<'_>,
    ) -> (Point, Point) {
        let a = tile.vertex_at(poly.vertex(usize::from(link.edge())) as usize);
        let b = tile.vertex_at(
            poly.vertex((usize::from(link.edge()) + 1) % poly.vertices().len()) as usize,
        );
        if link.side() != 255 && (link.min() != 0 || link.max() != 255) {
            let scale = 1. / 255.;
            (
                lerp(a, b, link.min() as f32 * scale),
                lerp(a, b, link.max() as f32 * scale),
            )
        } else {
            (a, b)
        }
    }
    pub fn closest_boundary(&self, reference: PolyRef, point: Point) -> Option<Point> {
        if !point.iter().all(|v| v.is_finite()) {
            return None;
        }
        let (tile, polygon) = self.resolve(reference)?;
        let tile = self.tiles[tile].as_ref()?;
        let polygon = tile.polygon_at(polygon);
        let mut vertices = [[0.; 3]; MAX_VERTICES_PER_POLYGON];
        let v = vertices.get_mut(..polygon.vertices().len())?;
        for (point, index) in v.iter_mut().zip(polygon.vertices()) {
            *point = tile.vertex(usize::from(index))?;
        }
        if inside(point, v) {
            return Some(point);
        }
        let mut best = f32::MAX;
        let mut closest = point;
        for i in 0..v.len() {
            let (d, t) = segment(point, v[i], v[(i + 1) % v.len()]);
            if d < best {
                best = d;
                closest = lerp(v[i], v[(i + 1) % v.len()], t);
            }
        }
        Some(closest)
    }
    fn straight_flag(&self, reference: PolyRef) -> u8 {
        if reference == 0 {
            return 2;
        }
        if self
            .resolve(reference)
            .is_some_and(|(t, p)| self.tiles[t].as_ref().unwrap().polygon_at(p).kind() == 1)
        {
            4
        } else {
            0
        }
    }
}

impl NavMesh {
    #[allow(clippy::too_many_arguments)]
    pub fn find_path<F: QueryFilter + ?Sized>(
        &self,
        start: PolyRef,
        end: PolyRef,
        start_pos: Point,
        end_pos: Point,
        filter: &F,
        max_nodes: usize,
        max_path: usize,
    ) -> Corridor {
        if !(1..=65535).contains(&max_nodes) {
            return Corridor {
                status: FAILURE | INVALID_PARAM,
                polygons: vec![],
            };
        }
        let mut pool = match nodes::NodePool::new(
            max_nodes,
            crate::common::next_power_of_two((max_nodes / 4) as u32) as usize,
        ) {
            Ok(pool) => pool,
            Err(status) => {
                return Corridor {
                    status,
                    polygons: vec![],
                };
            }
        };
        let mut queue = match nodes::NodeQueue::new(max_nodes) {
            Ok(queue) => queue,
            Err(status) => {
                return Corridor {
                    status,
                    polygons: vec![],
                };
            }
        };
        self.find_path_with_nodes(
            start, end, start_pos, end_pos, filter, max_path, &mut pool, &mut queue,
        )
    }
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn find_path_with_nodes<F: QueryFilter + ?Sized>(
        &self,
        start: PolyRef,
        end: PolyRef,
        start_pos: Point,
        end_pos: Point,
        filter: &F,
        max_path: usize,
        pool: &mut nodes::NodePool,
        queue: &mut nodes::NodeQueue,
    ) -> Corridor {
        let mut output = Corridor::default();
        self.find_path_with_nodes_into(
            start,
            end,
            start_pos,
            end_pos,
            filter,
            max_path,
            pool,
            queue,
            &mut output,
        );
        output
    }
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn find_path_with_nodes_into<F: QueryFilter + ?Sized>(
        &self,
        start: PolyRef,
        end: PolyRef,
        start_pos: Point,
        end_pos: Point,
        filter: &F,
        max_path: usize,
        pool: &mut nodes::NodePool,
        queue: &mut nodes::NodeQueue,
        output: &mut Corridor,
    ) -> u32 {
        output.polygons.clear();
        if self.resolve(start).is_none()
            || self.resolve(end).is_none()
            || !start_pos.iter().chain(&end_pos).all(|v| v.is_finite())
            || max_path == 0
            || max_path > i32::MAX as usize
        {
            output.status = FAILURE | INVALID_PARAM;
            return output.status;
        }
        if start == end {
            output.polygons.push(start);
            output.status = SUCCESS;
            return output.status;
        }
        pool.clear();
        queue.clear();
        let Some(first) = pool.get_node(start, 0) else {
            output.status = FAILURE | INVALID_PARAM;
            return output.status;
        };
        let start_node = pool.node_mut(first).unwrap();
        start_node.position = start_pos;
        start_node.total = dist(start_pos, end_pos) * 0.999;
        start_node.set_flags(nodes::OPEN);
        let mut last_cost = start_node.total;
        let mut last = first;
        queue.push(first, pool).unwrap();
        let mut out = false;
        while let Some(current) = queue.pop(pool) {
            let node = pool.node_mut(current).unwrap();
            node.set_flags((node.flags() & !nodes::OPEN) | nodes::CLOSED);
            let best = *node;
            if best.reference == end {
                last = current;
                break;
            }
            let Some(context) = self.polygon_from_bits(best.reference) else {
                output.status = FAILURE | INVALID_PARAM;
                return output.status;
            };
            let tile = context.tile;
            let poly = context.polygon;
            let parent = pool.node(best.parent_index()).map_or(0, |n| n.reference);
            let parent_context = if parent == 0 {
                None
            } else {
                let Some(context) = self.polygon_from_bits(parent) else {
                    output.status = FAILURE | INVALID_PARAM;
                    return output.status;
                };
                Some(context)
            };
            for (link, first_link) in tile.links_with_first(poly) {
                let reference = link.reference();
                if reference == 0 || reference == parent {
                    continue;
                }
                let Some(neighbor) = self.polygon_from_bits(reference) else {
                    output.status = FAILURE | INVALID_PARAM;
                    return output.status;
                };
                if !filter.pass_filter(neighbor) {
                    continue;
                }
                let state = if link.side() != 255 {
                    link.side() >> 1
                } else {
                    0
                };
                let Some(index) = pool.get_node(reference, state) else {
                    out = true;
                    continue;
                };
                if pool.node(index).unwrap().flags() == 0
                    && let Some((a, b)) =
                        Self::portal_with_link(context, neighbor, link, first_link)
                {
                    pool.node_mut(index).unwrap().position =
                        std::array::from_fn(|i| (a[i] + b[i]) * 0.5);
                }
                let n = *pool.node(index).unwrap();
                let current_cost = filter.cost(
                    best.position,
                    n.position,
                    parent_context,
                    context,
                    Some(neighbor),
                );
                let (cost, heuristic) = if reference == end {
                    (
                        best.cost
                            + current_cost
                            + filter.cost(n.position, end_pos, Some(context), neighbor, None),
                        0.,
                    )
                } else {
                    (best.cost + current_cost, dist(n.position, end_pos) * 0.999)
                };
                let total = cost + heuristic;
                if n.flags() != 0 && total >= n.total {
                    continue;
                }
                let was_open = n.flags() & nodes::OPEN != 0;
                let node = pool.node_mut(index).unwrap();
                node.set_parent_index(current);
                node.set_flags((node.flags() & !nodes::CLOSED) | nodes::OPEN);
                node.cost = cost;
                node.total = total;
                if was_open {
                    queue.modify(index, pool);
                } else {
                    queue.push(index, pool).unwrap();
                }
                if heuristic < last_cost {
                    last_cost = heuristic;
                    last = index;
                }
            }
        }
        let path = &mut output.polygons;
        let mut at = last;
        while let Some(node) = pool.node(at) {
            path.push(node.reference);
            at = node.parent_index();
        }
        path.reverse();
        let mut status = SUCCESS;
        if pool.node(last).unwrap().reference != end {
            status |= PARTIAL_RESULT;
        }
        if out {
            status |= OUT_OF_NODES;
        }
        if path.len() > max_path {
            path.truncate(max_path);
            status |= BUFFER_TOO_SMALL;
        }
        output.status = status;
        status
    }
    pub fn straight_path(
        &self,
        start: Point,
        end: Point,
        path: &[PolyRef],
        max_points: usize,
    ) -> StraightPath {
        self.straight_path_with_options(start, end, path, max_points, 0)
    }
    #[allow(clippy::too_many_arguments)]
    fn append_portals(
        &self,
        output: &mut StraightPath,
        path: &[PolyRef],
        start_index: usize,
        end_index: usize,
        end: Point,
        capacity: usize,
        options: u32,
    ) -> bool {
        let start = *output.points.last().unwrap();
        for i in start_index..end_index {
            let Some(from) = self.filter_polygon(path[i]) else {
                output.status = FAILURE | INVALID_PARAM;
                return false;
            };
            let Some(to) = self.filter_polygon(path[i + 1]) else {
                output.status = FAILURE | INVALID_PARAM;
                return false;
            };
            let Some((left, right)) = Self::portal_context(from, to) else {
                break;
            };
            if options & STRAIGHTPATH_AREA_CROSSINGS != 0
                && from.polygon.area() == to.polygon.area()
            {
                continue;
            }
            if let Some((_, t)) = crate::common::intersect_segments_2d(start, end, left, right)
                && !append(output, lerp(left, right, t), 0, path[i + 1], capacity)
            {
                return false;
            }
        }
        true
    }
    pub fn straight_path_with_options(
        &self,
        start: Point,
        end: Point,
        path: &[PolyRef],
        max_points: usize,
        options: u32,
    ) -> StraightPath {
        let mut output = StraightPath::default();
        self.straight_path_into(start, end, path, max_points, options, &mut output);
        output
    }
    /// Reuses caller-owned output storage; invalid input clears logical output.
    #[allow(clippy::too_many_arguments)]
    pub fn straight_path_into(
        &self,
        start: Point,
        end: Point,
        path: &[PolyRef],
        max_points: usize,
        options: u32,
        output: &mut StraightPath,
    ) -> u32 {
        output.status = SUCCESS;
        output.points.clear();
        output.flags.clear();
        output.references.clear();
        if path.is_empty()
            || path.len() > i32::MAX as usize
            || max_points == 0
            || max_points > i32::MAX as usize
            || !start.iter().chain(&end).all(|v| v.is_finite())
        {
            output.status = FAILURE | INVALID_PARAM;
            return output.status;
        }
        let Some(start) = self.closest_boundary(path[0], start) else {
            output.status = FAILURE | INVALID_PARAM;
            return output.status;
        };
        let end_position = end;
        let Some(end) = self.closest_boundary(*path.last().unwrap(), end_position) else {
            output.status = FAILURE | INVALID_PARAM;
            return output.status;
        };
        if !append(output, start, 1, path[0], max_points) {
            return output.status;
        }
        let crossings = options & (STRAIGHTPATH_AREA_CROSSINGS | STRAIGHTPATH_ALL_CROSSINGS) != 0;
        if path.len() > 1 {
            let (mut apex, mut left, mut right) = (start, start, start);
            let mut apex_index = 0;
            let (mut li, mut ri) = (0, 0);
            let (mut lr, mut rr) = (path[0], path[0]);
            let mut i = 0;
            while i < path.len() {
                let (l, r) = if i + 1 < path.len() {
                    let Some(portal) = self.portal(path[i], path[i + 1]) else {
                        if let Some(end) = self.closest_boundary(path[i], end_position) {
                            if crossings {
                                self.append_portals(
                                    output, path, apex_index, i, end, max_points, options,
                                );
                            }
                            append(output, end, 0, path[i], max_points);
                            output.status = SUCCESS
                                | PARTIAL_RESULT
                                | if output.points.len() >= max_points {
                                    BUFFER_TOO_SMALL
                                } else {
                                    0
                                };
                        } else {
                            output.status = FAILURE | INVALID_PARAM;
                        }
                        return output.status;
                    };
                    if i == 0 && segment(apex, portal.0, portal.1).0 < 0.001 * 0.001 {
                        i += 1;
                        continue;
                    }
                    portal
                } else {
                    (end, end)
                };
                if area(apex, right, r) <= 0. {
                    if equal(apex, right) || area(apex, left, r) > 0. {
                        right = r;
                        rr = path.get(i + 1).copied().unwrap_or(0);
                        ri = i;
                    } else {
                        if crossings
                            && !self.append_portals(
                                output, path, apex_index, li, left, max_points, options,
                            )
                        {
                            return output.status;
                        }
                        apex = left;
                        apex_index = li;
                        if !append(output, apex, self.straight_flag(lr), lr, max_points) {
                            return output.status;
                        }
                        left = apex;
                        right = apex;
                        ri = li;
                        i = li + 1;
                        continue;
                    }
                }
                if area(apex, left, l) >= 0. {
                    if equal(apex, left) || area(apex, right, l) < 0. {
                        left = l;
                        lr = path.get(i + 1).copied().unwrap_or(0);
                        li = i;
                    } else {
                        if crossings
                            && !self.append_portals(
                                output, path, apex_index, ri, right, max_points, options,
                            )
                        {
                            return output.status;
                        }
                        apex = right;
                        apex_index = ri;
                        if !append(output, apex, self.straight_flag(rr), rr, max_points) {
                            return output.status;
                        }
                        left = apex;
                        right = apex;
                        li = ri;
                        i = ri + 1;
                        continue;
                    }
                }
                i += 1;
            }
            if crossings
                && !self.append_portals(
                    output,
                    path,
                    apex_index,
                    path.len() - 1,
                    end,
                    max_points,
                    options,
                )
            {
                return output.status;
            }
        }
        append(output, end, 2, 0, max_points);
        output.status = SUCCESS
            | if output.points.len() >= max_points {
                BUFFER_TOO_SMALL
            } else {
                0
            };
        output.status
    }
}
fn append(
    out: &mut StraightPath,
    point: Point,
    flag: u8,
    reference: PolyRef,
    limit: usize,
) -> bool {
    if out.points.last().is_some_and(|&last| equal(last, point)) {
        let n = out.points.len() - 1;
        out.flags[n] = flag;
        out.references[n] = reference;
    } else {
        out.points.push(point);
        out.flags.push(flag);
        out.references.push(reference);
        if out.points.len() >= limit {
            out.status = SUCCESS | BUFFER_TOO_SMALL;
            return false;
        }
        if flag == 2 {
            out.status = SUCCESS;
            return false;
        }
    }
    true
}
