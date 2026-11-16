// Altered Rust translation of DetourNavMeshBuilder.cpp.
// Copyright (c) 2009-2010 Mikko Mononen. Zlib: vendor/recastnavigation/License.txt.
//! Native Detour v7 tile data. The selected `polyref64` feature must match the
//! producer: the native format does not identify its link-reference width.
//! Integers and floats use host byte order; use the explicit swap functions for
//! foreign-endian data. All access is checked and independent of Rust struct ABI.

use crate::common::{add_scaled, maximum, minimum};
use crate::{BuildError, nav::*};
use std::ops::Range;

pub const MAGIC: u32 = 0x444e_4156;
pub const VERSION: u32 = 7;
pub const STATE_MAGIC: u32 = 0x444e_4d53;
pub const STATE_VERSION: u32 = 1;
pub const HEADER_SIZE: usize = 100;
pub(crate) const LINK_SIZE: usize = if cfg!(feature = "polyref64") { 16 } else { 12 };

#[derive(Clone, Debug)]
pub struct CreatePolygon {
    /// Unused entries are `u16::MAX`; at most `vertices_per_polygon` are read.
    pub vertices: [u16; 6],
    /// Recast neighbor indices/border direction codes, before Detour conversion.
    pub neighbors: [u16; 6],
    pub flags: u16,
    pub area: u8,
}

#[derive(Clone, Debug)]
pub struct CreateOffMeshConnection {
    pub positions: [Point; 2],
    pub radius: f32,
    pub flags: u16,
    pub area: u8,
    /// Any nonzero value selects the native bidirectional flag.
    pub direction: u8,
}

#[derive(Clone, Debug)]
pub struct CreateParams<'a> {
    pub vertices: &'a [[u16; 3]],
    pub polygons: &'a [CreatePolygon],
    pub vertices_per_polygon: usize,
    /// Each record is [vertex base, vertex count, triangle base, triangle count].
    /// Native signed vertex-base conversion applies only to actual copies/BV
    /// reads. Triangle metadata is serialized; its count wraps to one byte.
    pub detail_meshes: Option<&'a [[u32; 4]]>,
    pub detail_vertices: &'a [Point],
    pub detail_triangles: &'a [[u8; 4]],
    pub off_mesh_connections: &'a [CreateOffMeshConnection],
    pub off_mesh_user_ids: Option<&'a [u32]>,
    pub coordinate: [i32; 3],
    pub user_id: u32,
    pub bounds: [Point; 2],
    pub walkable_height: f32,
    pub walkable_radius: f32,
    pub walkable_climb: f32,
    pub cell_size: f32,
    pub cell_height: f32,
    pub build_bv_tree: bool,
}

/// Owned creation inputs, including arrays that a tile-cache mesh processor can
/// replace or extend before native tile serialization.
#[derive(Clone, Debug, Default)]
pub struct OwnedCreateParams {
    pub vertices: Vec<[u16; 3]>,
    pub polygons: Vec<CreatePolygon>,
    pub vertices_per_polygon: usize,
    pub detail_meshes: Option<Vec<[u32; 4]>>,
    pub detail_vertices: Vec<Point>,
    pub detail_triangles: Vec<[u8; 4]>,
    pub off_mesh_connections: Vec<CreateOffMeshConnection>,
    pub off_mesh_user_ids: Option<Vec<u32>>,
    pub coordinate: [i32; 3],
    pub user_id: u32,
    pub bounds: [Point; 2],
    pub walkable_height: f32,
    pub walkable_radius: f32,
    pub walkable_climb: f32,
    pub cell_size: f32,
    pub cell_height: f32,
    pub build_bv_tree: bool,
}
impl OwnedCreateParams {
    pub fn as_params(&self) -> CreateParams<'_> {
        CreateParams {
            vertices: &self.vertices,
            polygons: &self.polygons,
            vertices_per_polygon: self.vertices_per_polygon,
            detail_meshes: self.detail_meshes.as_deref(),
            detail_vertices: &self.detail_vertices,
            detail_triangles: &self.detail_triangles,
            off_mesh_connections: &self.off_mesh_connections,
            off_mesh_user_ids: self.off_mesh_user_ids.as_deref(),
            coordinate: self.coordinate,
            user_id: self.user_id,
            bounds: self.bounds,
            walkable_height: self.walkable_height,
            walkable_radius: self.walkable_radius,
            walkable_climb: self.walkable_climb,
            cell_size: self.cell_size,
            cell_height: self.cell_height,
            build_bv_tree: self.build_bv_tree,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Header {
    pub coordinate: [i32; 3],
    pub user_id: u32,
    pub polygon_count: u32,
    pub vertex_count: u32,
    pub max_link_count: u32,
    pub detail_mesh_count: u32,
    pub detail_vertex_count: u32,
    pub detail_triangle_count: u32,
    pub bv_node_count: u32,
    pub off_mesh_connection_count: u32,
    pub off_mesh_base: u32,
    pub walkable_height: f32,
    pub walkable_radius: f32,
    pub walkable_climb: f32,
    pub bounds: [Point; 2],
    pub bv_quantization_factor: f32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Polygon {
    pub first_link: u32,
    pub vertices: [u16; 6],
    pub neighbors: [u16; 6],
    pub flags: u16,
    pub vertex_count: u8,
    pub area_and_type: u8,
}
impl Polygon {
    /// Active native vertex indices. Raw `vertex_count` must remain at most six.
    #[inline]
    pub fn vertices(&self) -> &[u16] {
        &self.vertices[..self.vertex_count as usize]
    }
    /// Active native neighbor records, in edge order.
    #[inline]
    pub fn neighbors(&self) -> &[u16] {
        &self.neighbors[..self.vertex_count as usize]
    }
    #[inline]
    pub const fn area(&self) -> u8 {
        self.area_and_type & 63
    }
    #[inline]
    pub const fn kind(&self) -> u8 {
        self.area_and_type >> 6
    }
    #[inline]
    pub const fn set_area(&mut self, area: u8) {
        self.area_and_type = (self.area_and_type & 0xc0) | (area & 63);
    }
    #[inline]
    pub const fn set_kind(&mut self, kind: u8) {
        self.area_and_type = (self.area_and_type & 63) | (kind << 6);
    }
}

/// Borrowed native polygon record, including its initialized spare words.
#[derive(Clone, Copy, Debug)]
pub struct PolygonView<'a>(pub(crate) &'a [u8; 32]);
impl<'a> PolygonView<'a> {
    #[inline]
    pub fn first_link(self) -> u32 {
        u32::from_ne_bytes(self.0[..4].try_into().unwrap())
    }
    #[inline]
    pub fn flags(self) -> u16 {
        u16::from_ne_bytes(self.0[28..30].try_into().unwrap())
    }
    #[inline]
    pub fn vertex_count(self) -> u8 {
        self.0[30]
    }
    #[inline]
    pub fn area_and_type(self) -> u8 {
        self.0[31]
    }
    #[inline]
    pub fn area(self) -> u8 {
        self.area_and_type() & 63
    }
    #[inline]
    pub fn kind(self) -> u8 {
        self.area_and_type() >> 6
    }
    #[inline]
    pub(crate) fn vertex_words(self) -> &'a [[u8; 2]] {
        &self.0[4..16].as_chunks::<2>().0[..self.vertex_count() as usize]
    }
    #[inline]
    fn neighbor_words(self) -> &'a [[u8; 2]] {
        &self.0[16..28].as_chunks::<2>().0[..self.vertex_count() as usize]
    }
    /// Stored vertex index, including inactive slots in the six-word record.
    /// An index beyond that record panics as slice indexing does.
    #[inline]
    pub fn vertex(self, index: usize) -> u16 {
        u16::from_ne_bytes(self.0[4..16].as_chunks::<2>().0[index])
    }
    /// Active neighbor record, in edge order.
    #[inline]
    pub fn neighbor(self, index: usize) -> u16 {
        u16::from_ne_bytes(self.neighbor_words()[index])
    }
    /// Includes initialized inactive words conservatively. A false result
    /// rules out active portals; true still requires the usual edge checks.
    #[inline]
    pub(crate) fn may_have_portal(self) -> bool {
        let first = u64::from_ne_bytes(self.0[16..24].try_into().unwrap());
        let last = u32::from_ne_bytes(self.0[24..28].try_into().unwrap());
        (first | u64::from(last)) & 0x8000_8000_8000_8000 != 0
    }
    #[inline]
    pub fn vertices(self) -> impl ExactSizeIterator<Item = u16> + DoubleEndedIterator + 'a {
        self.vertex_words().iter().map(|&b| u16::from_ne_bytes(b))
    }
    #[inline]
    pub fn neighbors(self) -> impl ExactSizeIterator<Item = u16> + DoubleEndedIterator + 'a {
        self.neighbor_words().iter().map(|&b| u16::from_ne_bytes(b))
    }
    pub fn to_owned(self) -> Polygon {
        read_polygon(self.0)
    }
}
/// A mutable fixed-size record; writes retain every unmodified native field.
#[derive(Debug)]
pub struct PolygonMut<'a>(pub(crate) &'a mut [u8; 32]);
impl PolygonMut<'_> {
    pub fn view(&self) -> PolygonView<'_> {
        PolygonView(self.0)
    }
    pub fn set_first_link(&mut self, value: u32) {
        self.0[..4].copy_from_slice(&value.to_ne_bytes());
    }
    pub fn set_flags(&mut self, value: u16) {
        self.0[28..30].copy_from_slice(&value.to_ne_bytes());
    }
    pub fn set_area_and_type(&mut self, value: u8) {
        self.0[31] = value;
    }
    pub fn set_area(&mut self, value: u8) {
        self.0[31] = (self.0[31] & 0xc0) | (value & 63);
    }
    pub fn set_kind(&mut self, value: u8) {
        self.0[31] = (self.0[31] & 63) | (value << 6);
    }
}
#[inline]
fn read_polygon(b: &[u8; 32]) -> Polygon {
    let view = PolygonView(b);
    Polygon {
        first_link: view.first_link(),
        vertices: std::array::from_fn(|i| {
            u16::from_ne_bytes(b[4 + i * 2..6 + i * 2].try_into().unwrap())
        }),
        neighbors: std::array::from_fn(|i| {
            u16::from_ne_bytes(b[16 + i * 2..18 + i * 2].try_into().unwrap())
        }),
        flags: view.flags(),
        vertex_count: view.vertex_count(),
        area_and_type: view.area_and_type(),
    }
}
pub(crate) fn write_polygon(b: &mut [u8; 32], polygon: &Polygon) {
    b[..4].copy_from_slice(&polygon.first_link.to_ne_bytes());
    for i in 0..6 {
        b[4 + i * 2..6 + i * 2].copy_from_slice(&polygon.vertices[i].to_ne_bytes());
        b[16 + i * 2..18 + i * 2].copy_from_slice(&polygon.neighbors[i].to_ne_bytes());
    }
    b[28..30].copy_from_slice(&polygon.flags.to_ne_bytes());
    b[30] = polygon.vertex_count;
    b[31] = polygon.area_and_type;
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DetailMesh {
    pub vertex_base: u32,
    pub triangle_base: u32,
    pub vertex_count: u8,
    pub triangle_count: u8,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct BvNode {
    pub min: [u16; 3],
    pub max: [u16; 3],
    /// Polygon index for a leaf, negative subtree size for an internal node.
    pub index: i32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct OffMeshConnection {
    pub positions: [Point; 2],
    pub radius: f32,
    pub polygon: u16,
    pub flags: u8,
    pub side: u8,
    pub user_id: u32,
}

fn u32_at(b: &[u8], p: usize) -> u32 {
    u32::from_ne_bytes(b[p..p + 4].try_into().unwrap())
}
fn float_at(b: &[u8], p: usize) -> f32 {
    f32::from_bits(u32_at(b, p))
}
fn point_at(b: &[u8], p: usize) -> Point {
    std::array::from_fn(|i| float_at(b, p + i * 4))
}
fn put16(b: &mut [u8], p: usize, n: u16) {
    b[p..p + 2].copy_from_slice(&n.to_ne_bytes());
}
fn put32(b: &mut [u8], p: usize, n: u32) {
    b[p..p + 4].copy_from_slice(&n.to_ne_bytes());
}
fn put_float(b: &mut [u8], p: usize, n: f32) {
    put32(b, p, n.to_bits());
}
fn put_point(b: &mut [u8], p: usize, v: Point) {
    for (i, n) in v.into_iter().enumerate() {
        put_float(b, p + i * 4, n);
    }
}

impl Header {
    pub fn read(data: &[u8]) -> Result<Self, u32> {
        if data.len() < HEADER_SIZE {
            return Err(FAILURE | INVALID_PARAM);
        }
        if u32_at(data, 0) != MAGIC {
            return Err(FAILURE | WRONG_MAGIC);
        }
        if u32_at(data, 4) != VERSION {
            return Err(FAILURE | WRONG_VERSION);
        }
        Ok(Self {
            coordinate: std::array::from_fn(|i| u32_at(data, 8 + i * 4) as i32),
            user_id: u32_at(data, 20),
            polygon_count: u32_at(data, 24),
            vertex_count: u32_at(data, 28),
            max_link_count: u32_at(data, 32),
            detail_mesh_count: u32_at(data, 36),
            detail_vertex_count: u32_at(data, 40),
            detail_triangle_count: u32_at(data, 44),
            bv_node_count: u32_at(data, 48),
            off_mesh_connection_count: u32_at(data, 52),
            off_mesh_base: u32_at(data, 56),
            walkable_height: float_at(data, 60),
            walkable_radius: float_at(data, 64),
            walkable_climb: float_at(data, 68),
            bounds: [point_at(data, 72), point_at(data, 84)],
            bv_quantization_factor: float_at(data, 96),
        })
    }
    fn write(&self, data: &mut [u8]) {
        for (i, n) in [
            MAGIC,
            VERSION,
            self.coordinate[0] as u32,
            self.coordinate[1] as u32,
            self.coordinate[2] as u32,
            self.user_id,
            self.polygon_count,
            self.vertex_count,
            self.max_link_count,
            self.detail_mesh_count,
            self.detail_vertex_count,
            self.detail_triangle_count,
            self.bv_node_count,
            self.off_mesh_connection_count,
            self.off_mesh_base,
        ]
        .into_iter()
        .enumerate()
        {
            put32(data, i * 4, n);
        }
        put_float(data, 60, self.walkable_height);
        put_float(data, 64, self.walkable_radius);
        put_float(data, 68, self.walkable_climb);
        put_point(data, 72, self.bounds[0]);
        put_point(data, 84, self.bounds[1]);
        put_float(data, 96, self.bv_quantization_factor);
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Layout {
    vertices: Range<usize>,
    polygons: Range<usize>,
    links: Range<usize>,
    details: Range<usize>,
    detail_vertices: Range<usize>,
    triangles: Range<usize>,
    bv: Range<usize>,
    off_mesh: Range<usize>,
}
impl Layout {
    fn new(h: &Header) -> Result<Self, u32> {
        let mut end = HEADER_SIZE;
        let mut section = |count: u32, stride: usize| {
            // Native counts and the complete native buffer size are signed int.
            let start = end;
            end = end.checked_add((count as usize).checked_mul(stride)?)?;
            (count <= i32::MAX as u32 && end <= i32::MAX as usize).then_some(start..end)
        };
        let result = (|| {
            Some(Self {
                vertices: section(h.vertex_count, 12)?,
                polygons: section(h.polygon_count, 32)?,
                links: section(h.max_link_count, LINK_SIZE)?,
                details: section(h.detail_mesh_count, 12)?,
                detail_vertices: section(h.detail_vertex_count, 12)?,
                triangles: section(h.detail_triangle_count, 4)?,
                bv: section(h.bv_node_count, 16)?,
                off_mesh: section(h.off_mesh_connection_count, 36)?,
            })
        })();
        result.ok_or(FAILURE | INVALID_PARAM)
    }
}

/// Checked, borrowed view of native tile sections. Record indices and topology
/// are not validated here; this also supports inspecting pre-load native data.
#[derive(Clone, Debug)]
pub struct TileData<'a> {
    data: &'a [u8],
    pub header: Header,
    layout: Layout,
}
impl<'a> TileData<'a> {
    pub fn parse(data: &'a [u8]) -> Result<Self, u32> {
        let header = Header::read(data)?;
        let layout = Layout::new(&header)?;
        if layout.off_mesh.end > data.len() {
            return Err(FAILURE | INVALID_PARAM);
        }
        Ok(Self {
            data,
            header,
            layout,
        })
    }
    /// Includes any trailing bytes supplied by the caller, without normalization.
    pub fn bytes(&self) -> &'a [u8] {
        self.data
    }
    pub fn vertices(&self) -> impl ExactSizeIterator<Item = Point> + '_ {
        self.vertex_records().iter().map(read_point)
    }
    pub fn polygons(&self) -> impl ExactSizeIterator<Item = Polygon> + '_ {
        self.polygon_records().iter().map(read_polygon)
    }
    pub(crate) fn polygon_records(&self) -> &[[u8; 32]] {
        self.data[self.layout.polygons.clone()].as_chunks::<32>().0
    }
    pub(crate) fn polygon_range(&self) -> Range<usize> {
        self.layout.polygons.clone()
    }
    /// Link records are reserved during creation and rebuilt by Detour on load.
    pub fn link_bytes(&self) -> &'a [u8] {
        &self.data[self.layout.links.clone()]
    }
    pub fn links(&self) -> impl ExactSizeIterator<Item = Link> + '_ {
        self.link_bytes().chunks_exact(LINK_SIZE).map(read_link)
    }
    pub(crate) fn link_range(&self) -> Range<usize> {
        self.layout.links.clone()
    }

    pub fn detail_meshes(&self) -> impl ExactSizeIterator<Item = DetailMesh> + '_ {
        self.detail_records().iter().map(read_detail)
    }
    pub(crate) fn vertex_records(&self) -> &[[u8; 12]] {
        self.data[self.layout.vertices.clone()].as_chunks::<12>().0
    }
    pub(crate) fn detail_records(&self) -> &[[u8; 12]] {
        self.data[self.layout.details.clone()].as_chunks::<12>().0
    }
    pub(crate) fn geometry_ranges(&self) -> [Range<usize>; 4] {
        [
            self.layout.vertices.clone(),
            self.layout.details.clone(),
            self.layout.detail_vertices.clone(),
            self.layout.off_mesh.clone(),
        ]
    }
    pub fn detail_vertices(&self) -> impl ExactSizeIterator<Item = Point> + '_ {
        self.data[self.layout.detail_vertices.clone()]
            .as_chunks::<12>()
            .0
            .iter()
            .map(read_point)
    }
    pub fn detail_triangles(&self) -> impl ExactSizeIterator<Item = [u8; 4]> + '_ {
        self.triangle_records().iter().copied()
    }
    pub(crate) fn triangle_records(&self) -> &[[u8; 4]] {
        self.data[self.layout.triangles.clone()].as_chunks::<4>().0
    }
    pub(crate) fn triangle_range(&self) -> Range<usize> {
        self.layout.triangles.clone()
    }
    pub(crate) fn bv_range(&self) -> Range<usize> {
        self.layout.bv.clone()
    }
    pub fn bv_nodes(&self) -> impl ExactSizeIterator<Item = BvNode> + '_ {
        self.data[self.layout.bv.clone()]
            .as_chunks::<16>()
            .0
            .iter()
            .map(read_bv_node)
    }
    pub fn off_mesh_connections(&self) -> impl ExactSizeIterator<Item = OffMeshConnection> + '_ {
        self.data[self.layout.off_mesh.clone()]
            .as_chunks::<36>()
            .0
            .iter()
            .map(read_off_mesh)
    }
}

#[inline]
pub(crate) fn read_point(b: &[u8; 12]) -> Point {
    std::array::from_fn(|i| f32::from_ne_bytes(b[i * 4..i * 4 + 4].try_into().unwrap()))
}
#[inline]
pub(crate) fn write_point(b: &mut [u8; 12], point: Point) {
    for (dst, value) in b.as_chunks_mut::<4>().0.iter_mut().zip(point) {
        *dst = value.to_ne_bytes();
    }
}
#[inline]
pub(crate) fn read_detail(b: &[u8; 12]) -> DetailMesh {
    DetailMesh {
        vertex_base: u32::from_ne_bytes(b[..4].try_into().unwrap()),
        triangle_base: u32::from_ne_bytes(b[4..8].try_into().unwrap()),
        vertex_count: b[8],
        triangle_count: b[9],
    }
}
#[inline]
pub(crate) fn write_detail(b: &mut [u8; 12], detail: &DetailMesh) {
    b[..4].copy_from_slice(&detail.vertex_base.to_ne_bytes());
    b[4..8].copy_from_slice(&detail.triangle_base.to_ne_bytes());
    b[8] = detail.vertex_count;
    b[9] = detail.triangle_count;
    // Preserve the native padding bytes.
}
#[inline]
pub(crate) fn read_off_mesh(b: &[u8; 36]) -> OffMeshConnection {
    OffMeshConnection {
        positions: [
            read_point(b[..12].try_into().unwrap()),
            read_point(b[12..24].try_into().unwrap()),
        ],
        radius: f32::from_ne_bytes(b[24..28].try_into().unwrap()),
        polygon: u16::from_ne_bytes(b[28..30].try_into().unwrap()),
        flags: b[30],
        side: b[31],
        user_id: u32::from_ne_bytes(b[32..36].try_into().unwrap()),
    }
}
#[inline]
pub(crate) fn write_off_mesh(b: &mut [u8; 36], connection: &OffMeshConnection) {
    write_point((&mut b[..12]).try_into().unwrap(), connection.positions[0]);
    write_point(
        (&mut b[12..24]).try_into().unwrap(),
        connection.positions[1],
    );
    b[24..28].copy_from_slice(&connection.radius.to_ne_bytes());
    b[28..30].copy_from_slice(&connection.polygon.to_ne_bytes());
    b[30] = connection.flags;
    b[31] = connection.side;
    b[32..36].copy_from_slice(&connection.user_id.to_ne_bytes());
}

#[inline]
pub(crate) fn read_bv_node(b: &[u8; 16]) -> BvNode {
    BvNode {
        min: std::array::from_fn(|i| u16::from_ne_bytes(b[i * 2..i * 2 + 2].try_into().unwrap())),
        max: std::array::from_fn(|i| {
            u16::from_ne_bytes(b[6 + i * 2..8 + i * 2].try_into().unwrap())
        }),
        index: i32::from_ne_bytes(b[12..16].try_into().unwrap()),
    }
}
#[inline]
pub(crate) fn write_bv_node(b: &mut [u8; 16], node: &BvNode) {
    for i in 0..3 {
        b[i * 2..i * 2 + 2].copy_from_slice(&node.min[i].to_ne_bytes());
        b[6 + i * 2..8 + i * 2].copy_from_slice(&node.max[i].to_ne_bytes());
    }
    b[12..16].copy_from_slice(&node.index.to_ne_bytes());
}

/// Borrowed native link record. Accessors preserve native field widths and
/// byte order without allocating or copying a decoded record.
#[derive(Clone, Copy, Debug)]
pub struct LinkView<'a>(pub(crate) &'a [u8; LINK_SIZE]);
impl LinkView<'_> {
    #[inline]
    pub fn reference(self) -> PolyRef {
        #[cfg(feature = "polyref64")]
        {
            u64::from_ne_bytes(self.0[..8].try_into().unwrap())
        }
        #[cfg(not(feature = "polyref64"))]
        {
            u32::from_ne_bytes(self.0[..4].try_into().unwrap())
        }
    }
    #[inline]
    pub fn next(self) -> u32 {
        u32::from_ne_bytes(self.0[LINK_SIZE - 8..LINK_SIZE - 4].try_into().unwrap())
    }
    #[inline]
    pub fn edge(self) -> u8 {
        self.0[LINK_SIZE - 4]
    }
    #[inline]
    pub fn side(self) -> u8 {
        self.0[LINK_SIZE - 3]
    }
    #[inline]
    pub fn min(self) -> u8 {
        self.0[LINK_SIZE - 2]
    }
    #[inline]
    pub fn max(self) -> u8 {
        self.0[LINK_SIZE - 1]
    }
}

#[inline]
pub(crate) fn read_link(b: &[u8]) -> Link {
    #[cfg(feature = "polyref64")]
    let reference = u64::from_ne_bytes(b[..8].try_into().unwrap());
    #[cfg(not(feature = "polyref64"))]
    let reference = u32_at(b, 0);
    let p = LINK_SIZE - 8;
    Link {
        reference,
        next: u32_at(b, p),
        edge: b[p + 4],
        side: b[p + 5],
        min: b[p + 6],
        max: b[p + 7],
    }
}

#[inline]
pub(crate) fn write_link(b: &mut [u8], link: &Link) {
    #[cfg(feature = "polyref64")]
    b[..8].copy_from_slice(&link.reference.to_ne_bytes());
    #[cfg(not(feature = "polyref64"))]
    put32(b, 0, link.reference);
    let p = LINK_SIZE - 8;
    put32(b, p, link.next);
    b[p + 4..p + 8].copy_from_slice(&[link.edge, link.side, link.min, link.max]);
}

/// Swaps a native or foreign header. A failed operation leaves the buffer intact.
pub fn swap_header_endian(data: &mut [u8]) -> bool {
    if data.len() < HEADER_SIZE {
        return false;
    }
    let (magic, version) = (u32_at(data, 0), u32_at(data, 4));
    if (magic != MAGIC || version != VERSION)
        && (magic != MAGIC.swap_bytes() || version != VERSION.swap_bytes())
    {
        return false;
    }
    for word in data[..HEADER_SIZE].chunks_exact_mut(4) {
        word.reverse();
    }
    true
}

/// Requires a native-endian header. Export: swap data, then header. Import: swap
/// header, then data. Matches upstream's omission of link/first-link fields and
/// off-mesh user IDs, including when those reserved fields contain nonzero data.
pub fn swap_data_endian(data: &mut [u8]) -> bool {
    let Ok(view) = TileData::parse(data) else {
        return false;
    };
    let l = view.layout;
    for range in [l.vertices, l.detail_vertices] {
        for word in data[range].chunks_exact_mut(4) {
            word.reverse();
        }
    }
    for p in data[l.polygons].chunks_exact_mut(32) {
        for word in p[4..30].chunks_exact_mut(2) {
            word.reverse();
        }
    }
    for p in data[l.details].chunks_exact_mut(12) {
        for word in p[..8].chunks_exact_mut(4) {
            word.reverse();
        }
    }
    for p in data[l.bv].chunks_exact_mut(16) {
        for word in p[..12].chunks_exact_mut(2) {
            word.reverse();
        }
        p[12..16].reverse();
    }
    for p in data[l.off_mesh].chunks_exact_mut(36) {
        for word in p[..28].chunks_exact_mut(4) {
            word.reverse();
        }
        p[28..30].reverse();
    }
    true
}

// The destination contains at least the fixed native header.
// Export writes metadata only: raw counts and topology are not dereferenced.
pub(crate) fn write_tile_header(data: &mut [u8], tile: &Tile) {
    let mut header = tile.header.clone();
    header.coordinate = tile.coordinate;
    header.walkable_climb = tile.climb;
    header.write(data);
}

fn classify(p: Point, bounds: [Point; 2]) -> u8 {
    let code = u8::from(p[0] >= bounds[1][0])
        | (u8::from(p[2] >= bounds[1][2]) << 1)
        | (u8::from(p[0] < bounds[0][0]) << 2)
        | (u8::from(p[2] < bounds[0][2]) << 3);
    match code {
        1 => 0,
        3 => 1,
        2 => 2,
        6 => 3,
        4 => 4,
        12 => 5,
        8 => 6,
        9 => 7,
        _ => 255,
    }
}

fn polygon_len(p: &CreatePolygon, nvp: usize) -> usize {
    p.vertices[..nvp]
        .iter()
        .position(|&v| v == u16::MAX)
        .unwrap_or(nvp)
}

// Native casts the unsigned source base to int, skips the polygon's own
// vertices, and only evaluates the resulting pointer when a copy is needed.
fn detail_vertex_copy_range(
    mesh: [u32; 4],
    nv: usize,
    available: usize,
) -> Result<Range<usize>, BuildError> {
    let count = (mesh[1] as i32)
        .checked_sub(nv as i32)
        .filter(|&n| n >= 0)
        .ok_or_else(invalid)? as usize;
    if count == 0 {
        return Ok(0..0);
    }
    let start = (mesh[0] as i32)
        .checked_add(nv as i32)
        .ok_or_else(invalid)?;
    start.checked_mul(3).ok_or_else(invalid)?;
    let start = usize::try_from(start).map_err(|_| invalid())?;
    let end = start
        .checked_add(count)
        .filter(|&n| n <= available)
        .ok_or_else(invalid)?;
    Ok(start..end)
}

fn detail_vertex_bounds_range(
    mesh: [u32; 4],
    available: usize,
) -> Result<Range<usize>, BuildError> {
    let base = mesh[0] as i32;
    base.checked_mul(3).ok_or_else(invalid)?;
    let start = usize::try_from(base).map_err(|_| invalid())?;
    // createBVTree reads dv[0] even for a declared count of zero.
    let count = usize::try_from(mesh[1] as i32)
        .map_err(|_| invalid())?
        .max(1);
    let end = start
        .checked_add(count)
        .filter(|&n| n <= available)
        .ok_or_else(invalid)?;
    Ok(start..end)
}

fn invalid() -> BuildError {
    BuildError("invalid native tile creation parameters".into())
}

/// Builds the native Detour tile buffer, including zero-filled reserved data.
/// Safe array/domain checks reject inputs that would read invalid memory or
/// overflow native signed size arithmetic. No allocator/pointer ABI is exposed.
pub fn create(params: &CreateParams<'_>) -> Result<Vec<u8>, BuildError> {
    let p = params;
    if p.vertices_per_polygon > 6
        || p.vertices.is_empty()
        || p.vertices.len() >= 0xffff
        || p.polygons.is_empty()
        || p.polygons.len() > i32::MAX as usize
        || p.off_mesh_user_ids
            .is_some_and(|ids| ids.len() < p.off_mesh_connections.len())
        || p.detail_meshes.is_some_and(|d| d.len() < p.polygons.len())
    {
        return Err(invalid());
    }
    let counts: Vec<_> = p
        .polygons
        .iter()
        .map(|poly| polygon_len(poly, p.vertices_per_polygon))
        .collect();
    let mut edges = 0usize;
    let mut portals = 0usize;
    let mut unique = 0usize;
    let mut triangle_count = 0usize;
    for (i, poly) in p.polygons.iter().enumerate() {
        let nv = counts[i];
        if (nv < 2 && p.detail_meshes.is_none())
            || poly.vertices[..nv]
                .iter()
                .any(|&v| v as usize >= p.vertices.len())
        {
            return Err(invalid());
        }
        edges += nv;
        portals += poly.neighbors[..nv]
            .iter()
            .filter(|&&n| n & 0x8000 != 0 && n & 15 != 15)
            .count();
        if let Some(meshes) = p.detail_meshes {
            let copy = detail_vertex_copy_range(meshes[i], nv, p.detail_vertices.len())?;
            if p.build_bv_tree {
                detail_vertex_bounds_range(meshes[i], p.detail_vertices.len())?;
            }
            unique = unique.checked_add(copy.len()).ok_or_else(invalid)?;
        } else {
            triangle_count += nv - 2;
        }
    }
    if p.detail_meshes.is_some() {
        triangle_count = p.detail_triangles.len();
    }
    let mut heights = [f32::MAX, -f32::MAX];
    if !p.detail_vertices.is_empty() {
        for v in p.detail_vertices {
            heights[0] = minimum(heights[0], v[1]);
            heights[1] = maximum(heights[1], v[1]);
        }
    } else {
        for v in p.vertices {
            let y = add_scaled(p.bounds[0][1], v[1] as f32, p.cell_height);
            heights[0] = minimum(heights[0], y);
            heights[1] = maximum(heights[1], y);
        }
    }
    heights[0] -= p.walkable_climb;
    heights[1] += p.walkable_climb;
    let classes: Vec<_> = p
        .off_mesh_connections
        .iter()
        .map(|c| {
            let mut a = classify(c.positions[0], p.bounds);
            if a == 255 && (c.positions[0][1] < heights[0] || c.positions[0][1] > heights[1]) {
                a = 0;
            }
            [a, classify(c.positions[1], p.bounds)]
        })
        .collect();
    let stored = classes.iter().filter(|c| c[0] == 255).count();
    let off_mesh_links = classes.iter().flatten().filter(|&&c| c == 255).count();
    let checked_count = |n: usize| {
        u32::try_from(n)
            .ok()
            .filter(|&v| v <= i32::MAX as u32)
            .ok_or_else(invalid)
    };
    let header = Header {
        coordinate: p.coordinate,
        user_id: p.user_id,
        polygon_count: checked_count(p.polygons.len() + stored)?,
        vertex_count: checked_count(p.vertices.len() + stored * 2)?,
        max_link_count: checked_count(edges + portals * 2 + off_mesh_links * 2)?,
        detail_mesh_count: checked_count(p.polygons.len())?,
        detail_vertex_count: checked_count(unique)?,
        detail_triangle_count: checked_count(triangle_count)?,
        bv_node_count: checked_count(if p.build_bv_tree {
            p.polygons.len() * 2
        } else {
            0
        })?,
        off_mesh_connection_count: checked_count(stored)?,
        off_mesh_base: checked_count(p.polygons.len())?,
        walkable_height: p.walkable_height,
        walkable_radius: p.walkable_radius,
        walkable_climb: p.walkable_climb,
        bounds: p.bounds,
        bv_quantization_factor: 1. / p.cell_size,
    };
    let l = Layout::new(&header).map_err(|_| invalid())?;
    let mut data = Vec::new();
    data.try_reserve_exact(l.off_mesh.end)
        .map_err(|_| BuildError("native tile allocation failed".into()))?;
    data.resize(l.off_mesh.end, 0);
    header.write(&mut data);
    for (i, v) in p.vertices.iter().enumerate() {
        put_point(
            &mut data,
            l.vertices.start + i * 12,
            [
                add_scaled(p.bounds[0][0], v[0] as f32, p.cell_size),
                add_scaled(p.bounds[0][1], v[1] as f32, p.cell_height),
                add_scaled(p.bounds[0][2], v[2] as f32, p.cell_size),
            ],
        );
    }
    for (i, poly) in p.polygons.iter().enumerate() {
        let base = l.polygons.start + i * 32;
        put16(&mut data, base + 28, poly.flags);
        data[base + 30] = counts[i] as u8;
        data[base + 31] = poly.area & 63;
        for j in 0..counts[i] {
            put16(&mut data, base + 4 + j * 2, poly.vertices[j]);
            let n = poly.neighbors[j];
            let neighbor = if n & 0x8000 == 0 {
                n.wrapping_add(1)
            } else {
                match n & 15 {
                    0 => 0x8004,
                    1 => 0x8002,
                    2 => 0x8000,
                    3 => 0x8006,
                    _ => 0,
                }
            };
            put16(&mut data, base + 16 + j * 2, neighbor);
        }
    }
    let mut vbase = 0u16;
    let mut tbase = 0usize;
    for (i, &nv) in counts.iter().enumerate() {
        let base = l.details.start + i * 12;
        if let Some(meshes) = p.detail_meshes {
            let [_, ndv, tb, nt] = meshes[i].map(|v| v as usize);
            put32(&mut data, base, vbase as u32);
            put32(&mut data, base + 4, tb as u32);
            data[base + 8] = (ndv - nv) as u8;
            data[base + 9] = nt as u8;
            let copy = detail_vertex_copy_range(meshes[i], nv, p.detail_vertices.len())?;
            for (j, &v) in p.detail_vertices[copy].iter().enumerate() {
                put_point(
                    &mut data,
                    l.detail_vertices.start + (vbase as usize + j) * 12,
                    v,
                );
            }
            vbase = vbase.wrapping_add((ndv - nv) as u16);
        } else {
            put32(&mut data, base + 4, tbase as u32);
            data[base + 9] = (nv - 2) as u8;
            for j in 2..nv {
                let at = l.triangles.start + tbase * 4;
                data[at..at + 4].copy_from_slice(&[
                    0,
                    (j - 1) as u8,
                    j as u8,
                    4 | if j == 2 { 1 } else { 0 } | if j == nv - 1 { 16 } else { 0 },
                ]);
                tbase += 1;
            }
        }
    }
    if p.detail_meshes.is_some() {
        for (b, tri) in data[l.triangles.clone()]
            .chunks_exact_mut(4)
            .zip(p.detail_triangles)
        {
            b.copy_from_slice(tri);
        }
    }
    if p.build_bv_tree {
        let mut items: Vec<_> = p
            .polygons
            .iter()
            .enumerate()
            .map(|(i, poly)| {
                let mut node = BvNode {
                    index: i as i32,
                    ..BvNode::default()
                };
                if let Some(meshes) = p.detail_meshes {
                    let range = detail_vertex_bounds_range(meshes[i], p.detail_vertices.len())?;
                    let verts = &p.detail_vertices[range];
                    let mut bounds = [verts[0]; 2];
                    for v in &verts[1..] {
                        for (axis, &value) in v.iter().enumerate() {
                            bounds[0][axis] = minimum(bounds[0][axis], value);
                            bounds[1][axis] = maximum(bounds[1][axis], value);
                        }
                    }
                    for (axis, (&min, &max)) in bounds[0].iter().zip(&bounds[1]).enumerate() {
                        node.min[axis] =
                            native_i32((min - p.bounds[0][axis]) * header.bv_quantization_factor)?
                                .clamp(0, 65535) as u16;
                        node.max[axis] =
                            native_i32((max - p.bounds[0][axis]) * header.bv_quantization_factor)?
                                .clamp(0, 65535) as u16;
                    }
                } else {
                    node.min = p.vertices[poly.vertices[0] as usize];
                    node.max = node.min;
                    for &id in &poly.vertices[1..counts[i]] {
                        for axis in 0..3 {
                            node.min[axis] = node.min[axis].min(p.vertices[id as usize][axis]);
                            node.max[axis] = node.max[axis].max(p.vertices[id as usize][axis]);
                        }
                    }
                    node.min[1] =
                        native_u16((node.min[1] as f32 * p.cell_height / p.cell_size).floor())?;
                    node.max[1] =
                        native_u16((node.max[1] as f32 * p.cell_height / p.cell_size).ceil())?;
                }
                Ok(node)
            })
            .collect::<Result<_, BuildError>>()?;
        let mut nodes = Vec::with_capacity(items.len() * 2);
        subdivide(&mut items, &mut nodes);
        for (i, node) in nodes.into_iter().enumerate() {
            let base = l.bv.start + i * 16;
            for axis in 0..3 {
                put16(&mut data, base + axis * 2, node.min[axis]);
                put16(&mut data, base + 6 + axis * 2, node.max[axis]);
            }
            put32(&mut data, base + 12, node.index as u32);
        }
    }
    let mut n = 0;
    for (i, con) in p.off_mesh_connections.iter().enumerate() {
        if classes[i][0] != 255 {
            continue;
        }
        let vi = p.vertices.len() + n * 2;
        let pi = p.polygons.len() + n;
        for j in 0..2 {
            put_point(
                &mut data,
                l.vertices.start + (vi + j) * 12,
                con.positions[j],
            );
        }
        let poly = l.polygons.start + pi * 32;
        put16(&mut data, poly + 4, vi as u16);
        put16(&mut data, poly + 6, (vi + 1) as u16);
        put16(&mut data, poly + 28, con.flags);
        data[poly + 30] = 2;
        data[poly + 31] = (con.area & 63) | 64;
        let base = l.off_mesh.start + n * 36;
        put_point(&mut data, base, con.positions[0]);
        put_point(&mut data, base + 12, con.positions[1]);
        put_float(&mut data, base + 24, con.radius);
        put16(&mut data, base + 28, pi as u16);
        data[base + 30] = u8::from(con.direction != 0);
        data[base + 31] = classes[i][1];
        if let Some(ids) = p.off_mesh_user_ids {
            put32(&mut data, base + 32, ids[i]);
        }
        n += 1;
    }
    Ok(data)
}

fn native_i32(value: f32) -> Result<i32, BuildError> {
    if value.is_finite() && value >= i32::MIN as f32 && value < -(i32::MIN as f32) {
        Ok(value as i32)
    } else {
        Err(invalid())
    }
}

fn native_u16(value: f32) -> Result<u16, BuildError> {
    if value.is_finite() && value > -1. && value < 65536. {
        Ok(value as u16)
    } else {
        Err(invalid())
    }
}

fn subdivide(items: &mut [BvNode], nodes: &mut Vec<BvNode>) {
    if items.len() == 1 {
        nodes.push(items[0].clone());
        return;
    }
    let at = nodes.len();
    let mut node = items[0].clone();
    for item in &items[1..] {
        for axis in 0..3 {
            node.min[axis] = node.min[axis].min(item.min[axis]);
            node.max[axis] = node.max[axis].max(item.max[axis]);
        }
    }
    let mut axis = 0;
    for candidate in 1..3 {
        // longestAxis takes unsigned short arguments: the promoted signed
        // differences narrow back to 16 bits before the comparisons.
        if node.max[candidate].wrapping_sub(node.min[candidate])
            > node.max[axis].wrapping_sub(node.min[axis])
        {
            axis = candidate;
        }
    }
    // Equal-key order is observable in the serialized BV tree. Apple's qsort
    // partitions ties; glibc's normal (non-allocation-failure) path is stable.
    crate::native_sort::sort_by_key(items, |item| item.min[axis]);
    nodes.push(node);
    let mid = items.len() / 2;
    let (left, right) = items.split_at_mut(mid);
    subdivide(left, nodes);
    subdivide(right, nodes);
    nodes[at].index = -((nodes.len() - at) as i32);
}
