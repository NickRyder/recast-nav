use crate::{
    Aabb3d, AreaType, RegionId,
    context::{Context, LogCategory, TimerLabel},
    contours::{ContourSet, RegionVertexId},
    math::{next, prev},
};
use alloc::vec::Vec;

use core::ops::{Deref, DerefMut};
use glam::{IVec3, U16Vec2, U16Vec3, Vec3Swizzles as _, u16vec3, uvec3};

#[derive(Debug, Default, Clone, PartialEq)]
struct InternalPolygonNavmesh<P: PolygonStorage = Vec<u16>> {
    vertices: Vec<U16Vec3>,
    nvertices: usize,
    polygons: P,
    npolys: usize,
    regions: Vec<P::Region>,
    flags: Vec<u16>,
    areas: Vec<AreaType>,
    max_polygons: usize,
    max_vertices_per_polygon: u32,
    tile_cache: bool,
    aabb: Aabb3d,
    cell_size: f32,
    cell_height: f32,
    border_size: i32,
    max_edge_error: f32,
}

#[derive(Clone, Copy)]
enum PolygonMeshMode {
    Recast,
    TileCache,
    TileCacheDeferredAdjacency,
}

trait PolygonStorage: Default + DerefMut<Target = [u16]> {
    type Region: Copy + Default + PartialEq + core::fmt::Debug;

    fn store_region(region: RegionId) -> Self::Region;
    fn filled(rows: usize, nvp: usize) -> Self;
}

impl PolygonStorage for Vec<u16> {
    type Region = RegionId;

    fn store_region(region: RegionId) -> Self::Region {
        region
    }

    fn filled(rows: usize, nvp: usize) -> Self {
        vec![u16::MAX; rows * nvp * 2]
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
struct TileCachePolygons(Vec<[u16; 12]>);

impl Deref for TileCachePolygons {
    type Target = [u16];

    fn deref(&self) -> &Self::Target {
        self.0.as_flattened()
    }
}

impl DerefMut for TileCachePolygons {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0.as_flattened_mut()
    }
}

impl PolygonStorage for TileCachePolygons {
    // Native TileCache rows expose areas but no region metadata. Zero-sized
    // regions preserve the shared builder without allocating region buffers.
    // General PolygonNavmesh output retains RegionId, including cache mode.
    type Region = ();

    fn store_region(_: RegionId) {}

    fn filled(rows: usize, nvp: usize) -> Self {
        debug_assert_eq!(nvp, 6);
        Self(vec![[u16::MAX; 12]; rows])
    }
}

trait PolygonMeshOutput: Sized {
    type Polygons: PolygonStorage;

    fn from_internal(mesh: InternalPolygonNavmesh<Self::Polygons>) -> Self;
}

impl PolygonMeshOutput for PolygonNavmesh {
    type Polygons = Vec<u16>;

    fn from_internal(mesh: InternalPolygonNavmesh) -> Self {
        mesh.into()
    }
}

/// Unlinked TileCache output, retaining native six-vertex/six-neighbor rows.
/// Storage transfers directly from the shared builder without splitting and
/// rejoining polygon buffers. Adjacency is left for the cache-specific caller.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct TileCachePolygonMesh {
    pub vertices: Vec<U16Vec3>,
    pub polygons: Vec<[u16; 12]>,
    pub flags: Vec<u16>,
    pub areas: Vec<AreaType>,
}

// The outer Option distinguishes an untouched output from an initialized empty
// mesh, including on the same partial-error paths as PolygonNavmesh output.
impl PolygonMeshOutput for Option<TileCachePolygonMesh> {
    type Polygons = TileCachePolygons;

    fn from_internal(mut mesh: InternalPolygonNavmesh<TileCachePolygons>) -> Self {
        mesh.vertices.truncate(mesh.nvertices);
        mesh.polygons.0.truncate(mesh.npolys);
        mesh.areas.truncate(mesh.npolys);
        Some(TileCachePolygonMesh {
            vertices: mesh.vertices,
            polygons: mesh.polygons.0,
            flags: mesh.flags,
            areas: mesh.areas,
        })
    }
}

/// Borrowed native byte coordinates and metadata for a TileCache contour.
#[derive(Clone, Copy, Debug)]
pub struct TileCacheContourView<'a> {
    /// X, Y, Z and cache corner flags, without a converted vertex allocation.
    pub vertices: &'a [[u8; 4]],
    /// Native region identifier.
    pub region: u8,
    /// Native area identifier.
    pub area: u8,
}

trait ContourVertices {
    const BYTE_COORDINATES: bool = false;
    fn len(&self) -> usize;
    fn position(&self, index: usize) -> IVec3;
    fn border_vertex(&self, index: usize) -> bool;
}

impl ContourVertices for [(IVec3, u32)] {
    #[inline]
    fn len(&self) -> usize {
        <[_]>::len(self)
    }
    #[inline]
    fn position(&self, index: usize) -> IVec3 {
        self[index].0
    }
    #[inline]
    fn border_vertex(&self, index: usize) -> bool {
        self[index].1 & RegionVertexId::BORDER_VERTEX.bits() != 0
    }
}

impl ContourVertices for [[u8; 4]] {
    const BYTE_COORDINATES: bool = true;
    #[inline]
    fn len(&self) -> usize {
        <[_]>::len(self)
    }
    #[inline]
    fn position(&self, index: usize) -> IVec3 {
        let v = self[index];
        IVec3::new(i32::from(v[0]), i32::from(v[1]), i32::from(v[2]))
    }
    #[inline]
    fn border_vertex(&self, index: usize) -> bool {
        self[index][3] & 128 != 0
    }
}

struct PolygonContourRef<'a, V: ?Sized> {
    vertices: &'a V,
    region: RegionId,
    area: AreaType,
}

/// Represents a polygon mesh suitable for use in building a navigation mesh.
///
/// A mesh of potentially overlapping convex polygons of between three and nvp vertices. The mesh exists within the context of an axis-aligned bounding box (AABB) with vertices laid out in an evenly spaced grid, based on the values of cs and ch.
///
/// The standard process for building a contour set is to allocate it using rcAllocPolyMesh, the initialize it using rcBuildPolyMesh
///
/// Example of iterating the polygons:
/// ```rust
/// # use recast_nav::*;
/// # use glam::Vec3;
/// # let mut mesh = PolygonNavmesh::default();
/// # mesh.max_vertices_per_polygon = 1;
/// // Where mesh is a reference to a PolygonNavmesh.
/// let nvp = mesh.max_vertices_per_polygon as usize;
/// let cs = mesh.cell_size;
/// let ch = mesh.cell_height;
/// let orig = mesh.aabb.min;
///
/// for i in 0..mesh.polygon_count() {
///     let p = &mesh.polygons[i * nvp..];
///
///     // Iterate the vertices.
///     for j in 0..nvp {
///         if p[j] == PolygonNavmesh::NO_INDEX {
///             // End of vertices.
///             break;
///         }
///
///         if p[j + nvp] == PolygonNavmesh::NO_CONNECTION {
///             // The edge beginning with this vertex is a solid border.
///         } else {
///             // The edge beginning with this vertex connects to
///             // polygon p[j + nvp].
///         }
///
///         // Convert to world space.
///         let v = &mesh.vertices[p[j] as usize];
///         let world_vertex = Vec3 {
///             x: orig.x + v.x as f32 * cs,
///             y: orig.y + v.y as f32 * ch,
///             z: orig.z + v.z as f32 * cs,
///         };
///         // Do something with the vertices.
///         println!("Vertex: {world_vertex}");
///     }
/// }
/// ```
#[derive(Debug, Default, Clone, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct PolygonNavmesh {
    /// The mesh vertices.
    ///
    /// The values of [`Aabb3d::min`], [`Self::cell_size`], and [`Self::cell_height`] are used to convert vertex coordinates to world space as follows:
    /// ```rust
    /// # use recast_nav::*;
    /// # use glam::{Vec3, U16Vec3};
    /// # let mut mesh = PolygonNavmesh::default();
    /// # mesh.vertices = vec![U16Vec3::ZERO; 1];
    /// # let i = 0;
    /// let world_vertex = Vec3 {
    ///     x: mesh.aabb.min.x + mesh.vertices[i].x as f32 * mesh.cell_size,
    ///     y: mesh.aabb.min.y + mesh.vertices[i].y as f32 * mesh.cell_height,
    ///     z: mesh.aabb.min.z + mesh.vertices[i].z as f32 * mesh.cell_size,
    /// };
    /// ```
    pub vertices: Vec<U16Vec3>,
    /// Polygons. [Length: [Self::polygon_count]].
    ///
    /// Each entry is [`Self::max_vertices_per_polygon`] in length.
    /// The polygons are in counter-clockwise order on the XZ plane. They are guaranteed to be convex. They are not guaranteed to be planar.
    ///
    /// The first instance of [`Self::NO_INDEX`] indicates the end of the indices for the entry.
    ///
    /// For example:
    /// ```txt
    /// vertices_per_polygon = 6
    /// For the entry:
    /// - polygons: (1, 3, 4, 8, NO_INDEX, NO_INDEX)
    /// - polygon_neighbors: (18, NO_CONNECTION , 21, NO_CONNECTION, NO_CONNECTION, NO_CONNECTION)
    ///
    /// (1, 3, 4, 8) defines a polygon with 4 vertices.
    /// Edge 1->3 is shared with polygon 18.
    /// Edge 4->8 is shared with polygon 21.
    /// Edges 3->4 and 4->8 are border edges not shared with any other polygon.
    /// ```
    pub polygons: Vec<u16>,
    /// Contains indices to each edge's connected polygons.
    /// A value of [`Self::NO_CONNECTION`] indicates no connection for the associated edge.
    /// (i.e. The edge is a solid border.)
    pub polygon_neighbors: Vec<u16>,
    /// The user-defined flags assigned to each polygon.
    pub flags: Vec<u16>,
    /// The region id assigned to each polygon.
    pub regions: Vec<RegionId>,
    /// The area id assigned to each polygon.
    ///
    /// The standard build process assigns the value of [`AreaType::DEFAULT_WALKABLE`] to all walkable polygons.
    /// This value can then be changed to meet user requirements.
    pub areas: Vec<AreaType>,
    /// Native `rcPolyMesh::maxpolys` metadata. Native merge leaves this at zero.
    pub max_polygons: usize,
    /// Initialized polygon slots beyond `polygon_count()`, in native slot order.
    pub unused_polygons: Vec<u16>,
    /// Initialized neighbor slots paired with `unused_polygons`.
    pub unused_polygon_neighbors: Vec<u16>,
    /// Initialized region slots beyond the live polygon count.
    pub unused_regions: Vec<RegionId>,
    /// Initialized area slots beyond the live polygon count.
    pub unused_areas: Vec<AreaType>,
    /// The maximum number of vertices per polygon
    pub max_vertices_per_polygon: u32,
    /// The bounding box of the mesh in world space.
    pub aabb: Aabb3d,
    /// The size of each cell. (On the xz-plane.)
    pub cell_size: f32,
    /// The height of each cell. (The minimum increment along the y-axis.)
    pub cell_height: f32,
    /// The AABB border size used to generate the source data from which the mesh was derived.
    pub border_size: i32,
    /// The max error of the polygon edges in the mesh.
    pub max_edge_error: f32,
}

impl PolygonNavmesh {
    /// The number of polygons in the mesh. Note that this is different from `polygons.len()`,
    /// as each polygon in that collection is represented by [`Self::max_vertices_per_polygon`] vertices.
    /// A zero-width mesh has no decoded polygons, including its default empty state.
    #[inline]
    pub fn polygon_count(&self) -> usize {
        self.polygons
            .len()
            .checked_div(self.max_vertices_per_polygon as usize)
            .unwrap_or(0)
    }

    /// A value which indicates that a polygon in [`Self::polygons`] has no more vertices starting from this vertex with this value.
    pub const NO_INDEX: u16 = 0xffff;

    /// A value which indicates that an edge of a polygon in [`Self::polygon_neighbors`] has no connection.
    pub const NO_CONNECTION: u16 = 0xffff;

    /// Iterates over all polygons in the mesh.
    pub fn polygons(&self) -> impl Iterator<Item = impl Iterator<Item = u16>> {
        self.polygons
            .chunks_exact((self.max_vertices_per_polygon as usize).max(1))
            .take(self.polygon_count())
            .map(|chunk| chunk.iter().take_while(|i| **i != Self::NO_INDEX).copied())
    }
}

impl From<InternalPolygonNavmesh> for PolygonNavmesh {
    fn from(mut value: InternalPolygonNavmesh) -> Self {
        let nvp = value.max_vertices_per_polygon as usize;
        let mut polygons = Vec::with_capacity(value.npolys * nvp);
        let mut polygon_neighbors = Vec::with_capacity(value.npolys * nvp);
        let unused_capacity = if value.tile_cache {
            0
        } else {
            value.polygons.len() / 2 - value.npolys * nvp
        };
        let mut unused_polygons = Vec::with_capacity(unused_capacity);
        let mut unused_polygon_neighbors = Vec::with_capacity(unused_capacity);
        if nvp != 0 {
            let words = if value.tile_cache {
                value.npolys * nvp * 2
            } else {
                value.polygons.len()
            };
            for (index, poly) in value.polygons[..words].chunks_exact(nvp * 2).enumerate() {
                let (vertices, neighbors) = poly.split_at(nvp);
                if index < value.npolys {
                    polygons.extend_from_slice(vertices);
                    polygon_neighbors.extend_from_slice(neighbors);
                } else {
                    unused_polygons.extend_from_slice(vertices);
                    unused_polygon_neighbors.extend_from_slice(neighbors);
                }
            }
        }
        value.vertices.truncate(value.nvertices);
        let (unused_areas, unused_regions) = if value.tile_cache {
            value.areas.truncate(value.npolys);
            value.regions.truncate(value.npolys);
            (Vec::new(), Vec::new())
        } else {
            (
                value.areas.split_off(value.npolys),
                value.regions.split_off(value.npolys),
            )
        };
        PolygonNavmesh {
            vertices: value.vertices,
            polygons,
            polygon_neighbors,
            regions: value.regions,
            flags: value.flags,
            areas: value.areas,
            max_polygons: value.max_polygons,
            unused_polygons,
            unused_polygon_neighbors,
            unused_regions,
            unused_areas,
            max_vertices_per_polygon: value.max_vertices_per_polygon,
            aabb: value.aabb,
            cell_size: value.cell_size,
            cell_height: value.cell_height,
            border_size: value.border_size,
            max_edge_error: value.max_edge_error,
        }
    }
}

impl ContourSet {
    /// Builds a polygon mesh from the provided contours.
    pub fn into_polygon_mesh(
        self,
        max_vertices_per_polygon: u32,
    ) -> Result<PolygonNavmesh, PolygonNavmeshError> {
        self.into_polygon_mesh_with_context(&Context::default(), max_vertices_per_polygon)
    }

    /// Build polygons with native diagnostics and the mesh-build timer.
    pub fn into_polygon_mesh_with_context(
        self,
        context: &Context<'_>,
        max_vertices_per_polygon: u32,
    ) -> Result<PolygonNavmesh, PolygonNavmeshError> {
        let mut output = PolygonNavmesh::default();
        self.build_polygon_mesh_into(context, max_vertices_per_polygon, &mut output)?;
        Ok(output)
    }

    /// Native output-parameter form. The vertex-capacity rejection retains old
    /// buffers/counts while writing bounds, scales, border size and edge error.
    pub fn build_polygon_mesh_into(
        self,
        context: &Context<'_>,
        max_vertices_per_polygon: u32,
        output: &mut PolygonNavmesh,
    ) -> Result<(), PolygonNavmeshError> {
        let _timer = context.scoped_timer(TimerLabel::BUILD_POLYMESH);
        output.aabb = self.aabb;
        output.cell_size = self.cell_size;
        output.cell_height = self.cell_height;
        output.border_size = self.border_size;
        output.max_edge_error = self.max_error;
        let mesh = self.build_polygon_mesh(
            context,
            max_vertices_per_polygon,
            PolygonMeshMode::Recast,
            None,
        )?;
        *output = mesh;
        Ok(())
    }

    /// Uses DetourTileCache's six-vertex polygons and strict ear clipping.
    /// Preserves native ushort mesh-index wrapping and 15-bit triangulation
    /// flags separately from the integer vertex count.
    pub fn into_tile_cache_polygon_mesh(self) -> Result<PolygonNavmesh, PolygonNavmeshError> {
        self.build_polygon_mesh(&Context::new(false), 6, PolygonMeshMode::TileCache, None)
    }

    /// Replaces output with the completed mesh or the initialized partial mesh
    /// when native TileCache vertex removal reaches its fixed buffer limit.
    pub fn build_tile_cache_polygon_mesh_into(
        self,
        output: &mut PolygonNavmesh,
    ) -> Result<(), PolygonNavmeshError> {
        let mesh = self.build_polygon_mesh(
            &Context::new(false),
            6,
            PolygonMeshMode::TileCache,
            Some(&mut *output),
        )?;
        *output = mesh;
        Ok(())
    }

    /// Builds TileCache polygons without adjacency or portal generation, for
    /// callers that perform their own cache-contour adjacency pass. Retains the
    /// same initialized partial output on a vertex-removal buffer failure.
    pub fn build_tile_cache_polygons_into(
        self,
        output: &mut PolygonNavmesh,
    ) -> Result<(), PolygonNavmeshError> {
        let mesh = self.build_polygon_mesh(
            &Context::new(false),
            6,
            PolygonMeshMode::TileCacheDeferredAdjacency,
            Some(&mut *output),
        )?;
        *output = mesh;
        Ok(())
    }

    /// Builds the same unlinked TileCache polygons directly from byte contours,
    /// without materializing owned Recast contours. Partial-error output follows
    /// [`Self::build_tile_cache_polygons_into`].
    /// Cloned iterators must yield the same ordered contours in both passes.
    pub fn build_tile_cache_polygons_from_contours<'a>(
        contours: impl Clone + Iterator<Item = TileCacheContourView<'a>>,
        output: &mut PolygonNavmesh,
    ) -> Result<(), PolygonNavmeshError> {
        Self::build_tile_cache_output_from_contours(contours, output)
    }

    /// Builds the same unlinked cache mesh directly into native polygon rows.
    /// Replaces output on success and initialized vertex-removal failures;
    /// earlier errors leave it untouched. Clones must yield the same ordered
    /// byte contours in both passes, as for the owned-output entry point.
    pub fn build_tile_cache_rows_from_contours<'a>(
        contours: impl Clone + Iterator<Item = TileCacheContourView<'a>>,
        output: &mut Option<TileCachePolygonMesh>,
    ) -> Result<(), PolygonNavmeshError> {
        Self::build_tile_cache_output_from_contours(contours, output)
    }

    fn build_tile_cache_output_from_contours<'a, M: PolygonMeshOutput>(
        contours: impl Clone + Iterator<Item = TileCacheContourView<'a>>,
        output: &mut M,
    ) -> Result<(), PolygonNavmeshError> {
        let settings = Self::default();
        let mesh = settings.build_polygon_mesh_from_contours(
            contours.map(|c| PolygonContourRef {
                vertices: c.vertices,
                region: RegionId::from(u16::from(c.region)),
                area: AreaType(c.area),
            }),
            &Context::new(false),
            6,
            PolygonMeshMode::TileCacheDeferredAdjacency,
            Some(&mut *output),
        )?;
        *output = mesh;
        Ok(())
    }

    fn build_polygon_mesh(
        self,
        context: &Context<'_>,
        max_vertices_per_polygon: u32,
        mode: PolygonMeshMode,
        partial: Option<&mut PolygonNavmesh>,
    ) -> Result<PolygonNavmesh, PolygonNavmeshError> {
        self.build_polygon_mesh_from_contours(
            self.contours.iter().map(|c| PolygonContourRef {
                vertices: c.vertices.as_slice(),
                region: c.region,
                area: c.area,
            }),
            context,
            max_vertices_per_polygon,
            mode,
            partial,
        )
    }

    fn build_polygon_mesh_from_contours<
        'a,
        V: ContourVertices + ?Sized + 'a,
        M: PolygonMeshOutput,
    >(
        &self,
        contours: impl Clone + Iterator<Item = PolygonContourRef<'a, V>>,
        context: &Context<'_>,
        max_vertices_per_polygon: u32,
        mode: PolygonMeshMode,
        partial: Option<&mut M>,
    ) -> Result<M, PolygonNavmeshError> {
        let tile_cache = !matches!(mode, PolygonMeshMode::Recast);
        let mut mesh = InternalPolygonNavmesh {
            tile_cache,
            aabb: self.aabb,
            cell_size: self.cell_size,
            cell_height: self.cell_height,
            border_size: self.border_size,
            max_edge_error: self.max_error,
            max_vertices_per_polygon,
            ..Default::default()
        };
        let nvp = max_vertices_per_polygon as usize;

        let mut max_vertices = 0;
        let mut max_tris = 0;
        let mut max_verts_per_cont = 0;
        for contour in contours.clone() {
            // Skip null contours.
            if contour.vertices.len() < 3 {
                continue;
            }
            max_vertices += contour.vertices.len();
            max_tris += contour.vertices.len() - 2;
            max_verts_per_cont = max_verts_per_cont.max(contour.vertices.len());
        }

        if !tile_cache && max_vertices >= 0xfffe {
            let _ = context.log_format(
                LogCategory::ERROR,
                format_args!("rcBuildPolyMesh: Too many vertices {max_vertices}."),
            );
            // Match the upstream conservative contour-vertex count cutoff.
            return Err(PolygonNavmeshError::TooManyVertices {
                actual: max_vertices,
                max: 0xfffd,
            });
        }

        let mut vflags = vec![false; max_vertices];
        mesh.vertices = vec![U16Vec3::ZERO; max_vertices];
        mesh.polygons = M::Polygons::filled(max_tris, nvp);
        mesh.max_polygons = if tile_cache { 0 } else { max_tris };
        mesh.regions = vec![M::Polygons::store_region(RegionId::default()); max_tris];
        mesh.areas = vec![AreaType::default(); max_tris];

        // Native ushort links use 0xffff as their sole sentinel. Keep the
        // allocation/count wide even when inserted indices wrap to that value.
        let mut next_vert = vec![0; max_vertices];
        // A sentinel-valued vertex insertion clears its bucket chain, so
        // TileCache must use the native eight-bit hash, not Recast's twelve.
        // Initialize only the table selected by this builder mode.
        let mut cache_buckets;
        let mut recast_buckets;
        let first_vert: &mut [u16] = if tile_cache {
            cache_buckets = [u16::MAX; 1 << 8];
            &mut cache_buckets
        } else {
            recast_buckets = [u16::MAX; VERTEX_BUCKET_COUNT];
            &mut recast_buckets
        };
        let mut indices = vec![0; max_verts_per_cont];
        let mut tris = vec![U16Vec3::ZERO; max_verts_per_cont];
        let mut polys = vec![u16::MAX; (max_verts_per_cont + 1) * nvp];

        let temp_poly_index = max_verts_per_cont * nvp;

        for (contour_index, cont) in contours.enumerate() {
            // Skip null contours.
            if cont.vertices.len() < 3 {
                continue;
            }

            // Triangulate contour
            #[expect(clippy::needless_range_loop)]
            for j in 0..cont.vertices.len() {
                indices[j] = if tile_cache {
                    // Translate the native ushort index/ear bit into the shared
                    // triangulator's flag representation before any predicates.
                    (j & 0x7fff) | if j & 0x8000 != 0 { CAN_REMOVE } else { 0 }
                } else {
                    j
                };
            }

            // Native callers retain partial triangles after a triangulation warning.
            let ntris = triangulate(
                cont.vertices.len(),
                cont.vertices,
                &mut indices,
                &mut tris,
                !tile_cache,
            );
            if ntris <= 0 && !tile_cache {
                let _ = context.log_format(
                    LogCategory::WARNING,
                    format_args!("rcBuildPolyMesh: Bad triangulation Contour {contour_index}."),
                );
            }
            let ntris = ntris.unsigned_abs();
            // Add and merge vertices.
            for j in 0..cont.vertices.len() {
                let v = cont.vertices.position(j);
                indices[j] = add_vertex(
                    U16Vec3::new(v.x as u16, v.y as u16, v.z as u16),
                    &mut mesh.vertices,
                    first_vert,
                    &mut next_vert,
                    &mut mesh.nvertices,
                )? as usize;
                if cont.vertices.border_vertex(j) {
                    // This vertex should be removed.
                    vflags[indices[j]] = true;
                }
            }
            // Build initial polygons.
            let mut npolys = 0;
            polys.fill(u16::MAX);
            for t in tris.iter().take(ntris) {
                if t.x != t.y && t.x != t.z && t.y != t.z {
                    polys[npolys * nvp] = indices[t.x as usize] as u16;
                    polys[npolys * nvp + 1] = indices[t.y as usize] as u16;
                    polys[npolys * nvp + 2] = indices[t.z as usize] as u16;
                    npolys += 1;
                }
            }

            if npolys == 0 {
                continue;
            }

            // Merge polygons.
            if nvp > 3 {
                while let Some(best) = find_best_poly_merge(&polys, npolys, &mesh.vertices, nvp) {
                    let pa_index = best.polygon_a * nvp;
                    let pb_index = best.polygon_b * nvp;
                    merge_poly_verts(
                        &mut polys,
                        pa_index,
                        pb_index,
                        best.value.edge_a,
                        best.value.edge_b,
                        temp_poly_index,
                        nvp,
                    );
                    let last_poly = (npolys - 1) * nvp;
                    if pb_index != last_poly {
                        polys.copy_within(last_poly..last_poly + nvp, pb_index);
                    }
                    npolys -= 1;
                }
            }

            // Store polygons.
            for j in 0..npolys {
                let p = &mut mesh.polygons[mesh.npolys * nvp * 2..];
                let q = &polys[j * nvp..];
                p[..nvp].copy_from_slice(&q[..nvp]);
                mesh.regions[mesh.npolys] = M::Polygons::store_region(cont.region);
                mesh.areas[mesh.npolys] = cont.area;
                mesh.npolys += 1;
                if mesh.npolys > max_tris {
                    let _ = context.log_format(
                        LogCategory::ERROR,
                        format_args!(
                            "rcBuildPolyMesh: Too many polygons {} (max:{max_tris}).",
                            mesh.npolys
                        ),
                    );
                    return Err(PolygonNavmeshError::TooManyPolygons {
                        actual: mesh.npolys,
                        max: max_tris,
                    });
                }
            }
        }

        // Remove edge vertices.
        let mut i = 0;
        while i < mesh.nvertices {
            if vflags[i] && mesh.can_remove_vertex(i as u16) {
                if let Err(error) = mesh.remove_vertex(context, i as u16, max_tris) {
                    let _ = context.log_format(
                        LogCategory::ERROR,
                        format_args!("rcBuildPolyMesh: Failed to remove edge vertex {i}."),
                    );
                    if let Some(output) = partial {
                        mesh.flags = vec![0; mesh.npolys];
                        *output = M::from_internal(mesh);
                    }
                    return Err(error);
                }
                // Remove vertex
                // Note: nverts is already decremented inside removeVertex()!
                // Fixup vertex flags
                vflags.copy_within((i + 1)..=mesh.nvertices, i);
            } else {
                i += 1;
            }
        }
        // Calculate adjacency.
        if !matches!(mode, PolygonMeshMode::TileCacheDeferredAdjacency)
            && let Err(error) = mesh.build_mesh_adjacency()
        {
            context.log(LogCategory::ERROR, "rcBuildPolyMesh: Adjacency failed.");
            return Err(error);
        }

        // Find portal edges
        if self.border_size > 0 && !matches!(mode, PolygonMeshMode::TileCacheDeferredAdjacency) {
            let w = self.width;
            let h = self.height;
            for i in 0..mesh.npolys {
                let p = &mut mesh.polygons[i * 2 * nvp..];
                for j in 0..nvp {
                    if p[j] == PolygonNavmesh::NO_INDEX {
                        break;
                    }
                    // Skip connected edges.
                    if p[nvp + j] != PolygonNavmesh::NO_CONNECTION {
                        continue;
                    }
                    let nj = j + 1;
                    let nj = if nj >= nvp || p[nj] == PolygonNavmesh::NO_INDEX {
                        0
                    } else {
                        nj
                    };
                    let va = mesh.vertices[p[j] as usize];
                    let vb = mesh.vertices[p[nj] as usize];
                    if va.x == 0 && vb.x == 0 {
                        p[nvp + j] = RegionId::BORDER_REGION.bits();
                    } else if i32::from(va.z) == h && i32::from(vb.z) == h {
                        p[nvp + j] = RegionId::BORDER_REGION.bits() | 1;
                    } else if i32::from(va.x) == w && i32::from(vb.x) == w {
                        p[nvp + j] = RegionId::BORDER_REGION.bits() | 2;
                    } else if va.z == 0 && vb.z == 0 {
                        p[nvp + j] = RegionId::BORDER_REGION.bits() | 3;
                    }
                }
            }
        }
        // Just allocate the mesh flags array. The user is resposible to fill it.
        mesh.flags = vec![0; mesh.npolys];
        if mesh.npolys > 0xffff {
            let _ = context.log_format(LogCategory::ERROR, format_args!(
                "rcBuildPolyMesh: The resulting mesh has too many polygons {} (max 65535). Data can be corrupted.", mesh.npolys
            ));
        }

        Ok(M::from_internal(mesh))
    }
}

#[derive(Debug, Default, Clone)]
struct Edge {
    vert: U16Vec2,
    poly_edge: U16Vec2,
    poly: U16Vec2,
}

impl<P: PolygonStorage> InternalPolygonNavmesh<P> {
    fn build_mesh_adjacency(&mut self) -> Result<(), PolygonNavmeshError> {
        let nvp = self.max_vertices_per_polygon as usize;
        // Based on code by Eric Lengyel from:
        // https://web.archive.org/web/20080704083314/http://www.terathon.com/code/edges.php
        let max_edge_count = self.npolys * nvp;
        let mut first_edge = vec![PolygonNavmesh::NO_CONNECTION; self.nvertices + max_edge_count];
        let next_edge_index = self.nvertices;
        let mut edge_count = 0;
        let mut edges = vec![Edge::default(); max_edge_count];
        for i in 0..self.npolys {
            let t = &self.polygons[i * nvp * 2..];
            for j in 0..nvp {
                if t[j] == PolygonNavmesh::NO_INDEX {
                    break;
                }
                let v0 = t[j];
                let v1 = if j + 1 >= nvp || t[j + 1] == PolygonNavmesh::NO_INDEX {
                    t[0]
                } else {
                    t[j + 1]
                };
                if v0 < v1 {
                    let edge = &mut edges[edge_count];
                    edge.vert.x = v0;
                    edge.vert.y = v1;
                    edge.poly.x = i as u16;
                    edge.poly_edge.x = j as u16;
                    edge.poly.y = i as u16;
                    edge.poly_edge.y = 0;
                    // Insert edge
                    first_edge[next_edge_index + edge_count] = first_edge[v0 as usize];
                    first_edge[v0 as usize] = edge_count as u16;
                    edge_count += 1;
                }
            }
        }
        for i in 0..self.npolys {
            let t = &self.polygons[i * nvp * 2..];
            let nv = count_poly_verts(t, nvp);
            for j in 0..nv {
                if t[j] == PolygonNavmesh::NO_INDEX {
                    break;
                }
                let v0 = t[j];
                let v1 = if j + 1 >= nvp || t[j + 1] == PolygonNavmesh::NO_INDEX {
                    t[0]
                } else {
                    t[j + 1]
                };
                if v0 > v1 {
                    let mut e = first_edge[v1 as usize];
                    while e != PolygonNavmesh::NO_CONNECTION {
                        let edge = &mut edges[e as usize];
                        if edge.vert.y == v0 && edge.poly.x == edge.poly.y {
                            edge.poly.y = i as u16;
                            edge.poly_edge.y = j as u16;
                            break;
                        }
                        e = first_edge[next_edge_index + e as usize];
                    }
                }
            }
        }

        // Store adjacency
        for e in edges.iter().take(edge_count) {
            if e.poly.x != e.poly.y {
                {
                    let p0 = &mut self.polygons[e.poly.x as usize * nvp * 2..];
                    p0[nvp + e.poly_edge.x as usize] = e.poly.y;
                }
                let p1 = &mut self.polygons[e.poly.y as usize * nvp * 2..];
                p1[nvp + e.poly_edge.y as usize] = e.poly.x;
            }
        }
        Ok(())
    }

    fn remove_vertex(
        &mut self,
        context: &Context<'_>,
        rem: u16,
        max_tris: usize,
    ) -> Result<(), PolygonNavmeshError> {
        let nvp = self.max_vertices_per_polygon as usize;

        // Count number of polygons to remove.
        let mut num_removed_verts = 0;
        for i in 0..self.npolys {
            let p = &self.polygons[i * nvp * 2..];
            let nv = count_poly_verts(p, nvp);
            for pj in p.iter().take(nv) {
                if *pj == rem {
                    num_removed_verts += 1;
                }
            }
        }

        let mut nedges = 0;
        // Format: [polygon1, polygon2, region, area]
        #[derive(Debug, Clone, Default)]
        struct Edge<R> {
            polygon1: u16,
            polygon2: u16,
            region: R,
            area: AreaType,
        }
        let mut edges = vec![Edge::<P::Region>::default(); num_removed_verts * nvp];
        let mut nhole = 0;
        let mut hole = vec![0; num_removed_verts * nvp];
        let mut nhreg = 0;
        let mut hreg = vec![P::Region::default(); num_removed_verts * nvp];
        let mut nharea = 0;
        let mut harea = vec![AreaType::default(); num_removed_verts * nvp];

        let mut i = 0;
        while i < self.npolys {
            let i1 = i * nvp * 2;
            {
                // Scope p immutable in this scope
                let p = &self.polygons[i1..];
                let nv = count_poly_verts(p, nvp);
                let has_rem = (0..nv).any(|j| p[j] == rem);
                if !has_rem {
                    i += 1;
                    continue;
                }
                // Collect edges which does not touch the removed vertex.
                for j in 0..nv {
                    let k = if j == 0 { nv - 1 } else { j - 1 };
                    if p[j] != rem && p[k] != rem {
                        if self.tile_cache && nedges >= 48 {
                            return Err(PolygonNavmeshError::VertexRemovalBufferTooSmall);
                        }
                        let e = &mut edges[nedges];
                        e.polygon1 = p[k];
                        e.polygon2 = p[j];
                        e.region = self.regions[i];
                        e.area = self.areas[i];
                        nedges += 1;
                    }
                }
            }
            // Remove the polygon.
            let i2 = (self.npolys - 1) * nvp * 2;
            if i1 != i2 {
                self.polygons.copy_within(i2..(i2 + nvp), i1);
            }
            self.polygons[i1 + nvp..(i1 + 2 * nvp)].fill(u16::MAX);
            self.regions[i] = self.regions[self.npolys - 1];
            self.areas[i] = self.areas[self.npolys - 1];
            self.npolys -= 1;
        }

        // Remove vertex.
        for i in usize::from(rem)..self.nvertices - 1 {
            self.vertices[i] = self.vertices[i + 1];
        }
        self.nvertices -= 1;

        // Adjust indices to match the removed vertex layout.
        for i in 0..self.npolys {
            let p = &mut self.polygons[i * nvp * 2..];
            let nv = count_poly_verts(p, nvp);
            for pj in p.iter_mut().take(nv) {
                if *pj > rem {
                    *pj -= 1;
                }
            }
        }
        for edge in edges.iter_mut().take(nedges) {
            if edge.polygon1 > rem {
                edge.polygon1 -= 1;
            }
            if edge.polygon2 > rem {
                edge.polygon2 -= 1;
            }
        }

        if nedges == 0 {
            return Ok(());
        }

        // Start with one vertex, keep appending connected
        // segments to the start and end of the hole.
        push_back(edges[0].polygon1, &mut hole, &mut nhole);
        push_back(edges[0].region, &mut hreg, &mut nhreg);
        push_back(edges[0].area, &mut harea, &mut nharea);

        while nedges != 0 {
            let mut match_ = false;
            let mut i = 0;
            while i < nedges {
                let edge = edges[i].clone();
                let ea = edge.polygon1;
                let eb = edge.polygon2;
                let r = edge.region;
                let a = edge.area;
                let mut add = false;
                if hole[0] == eb {
                    // The segment matches the beginning of the hole boundary.
                    if self.tile_cache && nhole >= 48 {
                        return Err(PolygonNavmeshError::VertexRemovalBufferTooSmall);
                    }
                    push_front(ea, &mut hole, &mut nhole);
                    push_front(r, &mut hreg, &mut nhreg);
                    push_front(a, &mut harea, &mut nharea);
                    add = true;
                } else if hole[nhole - 1] == ea {
                    // The segment matches the end of the hole boundary.
                    if self.tile_cache && nhole >= 48 {
                        return Err(PolygonNavmeshError::VertexRemovalBufferTooSmall);
                    }
                    push_back(eb, &mut hole, &mut nhole);
                    push_back(r, &mut hreg, &mut nhreg);
                    push_back(a, &mut harea, &mut nharea);
                    add = true;
                }
                if add {
                    // The edge segment was added, remove it.
                    edges[i] = edges[nedges - 1].clone();
                    nedges -= 1;
                    match_ = true;
                } else {
                    i += 1;
                }
            }
            if !match_ {
                break;
            }
        }
        let mut tris = vec![U16Vec3::default(); nhole];
        let mut tverts = vec![(IVec3::default(), 0); nhole];
        let mut thole = vec![0; nhole];

        // Generate temp vertex array for triangulation.
        for i in 0..nhole {
            let pi = hole[i] as usize;
            tverts[i].0 = self.vertices[pi].as_ivec3();
            thole[i] = i;
        }

        // Triangulate the hole.
        // Native callers retain partial triangles after a triangulation warning.
        let ntris = triangulate(
            nhole,
            tverts.as_slice(),
            &mut thole,
            &mut tris,
            !self.tile_cache,
        );
        if ntris < 0 && !self.tile_cache {
            context.log(
                LogCategory::WARNING,
                "removeVertex: triangulate() returned bad results.",
            );
        }
        let ntris = ntris.unsigned_abs();

        // Merge the hole triangles back to polygons.
        let mut polys = vec![0; (ntris + 1) * nvp];
        let mut pregs = vec![P::Region::default(); ntris];
        let mut pareas = vec![AreaType::default(); ntris];
        let tmp_poly_index = ntris * nvp;

        // Build initial polygons.
        let mut npolys = 0;
        polys[..ntris * nvp].fill(u16::MAX);
        for t in tris.iter().take(ntris) {
            if t.x != t.y && t.x != t.z && t.y != t.z {
                let t_x = t.x as usize;
                let t_y = t.y as usize;
                let t_z = t.z as usize;
                polys[npolys * nvp] = hole[t_x];
                polys[npolys * nvp + 1] = hole[t_y];
                polys[npolys * nvp + 2] = hole[t_z];

                // If this polygon covers multiple region types then
                // mark it as such
                if hreg[t_x] != hreg[t_y] || hreg[t_y] != hreg[t_z] {
                    pregs[npolys] = P::store_region(RegionId::NONE);
                } else {
                    pregs[npolys] = hreg[t_x];
                }
                pareas[npolys] = harea[t_x];
                npolys += 1;
            }
        }
        if npolys == 0 {
            return Ok(());
        }

        // Merge polygons.
        if nvp > 3 {
            while let Some(best) = find_best_poly_merge(&polys, npolys, &self.vertices, nvp) {
                let pa_index = best.polygon_a * nvp;
                let pb_index = best.polygon_b * nvp;
                merge_poly_verts(
                    &mut polys,
                    pa_index,
                    pb_index,
                    best.value.edge_a,
                    best.value.edge_b,
                    tmp_poly_index,
                    nvp,
                );
                if pregs[best.polygon_a] != pregs[best.polygon_b] {
                    pregs[best.polygon_a] = P::store_region(RegionId::NONE);
                }
                let last_index = (npolys - 1) * nvp;
                if pb_index != last_index {
                    polys.copy_within(last_index..last_index + nvp, pb_index);
                }
                pregs[best.polygon_b] = pregs[npolys - 1];
                pareas[best.polygon_b] = pareas[npolys - 1];
                npolys -= 1;
            }
        }

        // Store polygons.
        for i in 0..npolys {
            if self.npolys >= max_tris {
                break;
            }
            let p = &mut self.polygons[self.npolys * nvp * 2..self.npolys * nvp * 2 + nvp * 2];
            p[..nvp * 2].fill(u16::MAX);
            for j in 0..nvp {
                p[j] = polys[i * nvp + j];
            }
            self.regions[self.npolys] = pregs[i];
            self.areas[self.npolys] = pareas[i];
            self.npolys += 1;
            if self.npolys > max_tris {
                let _ = context.log_format(
                    LogCategory::ERROR,
                    format_args!(
                        "removeVertex: Too many polygons {} (max:{max_tris}).",
                        self.npolys
                    ),
                );
                return Err(PolygonNavmeshError::TooManyPolygons {
                    actual: self.npolys,
                    max: max_tris,
                });
            }
        }

        Ok(())
    }

    fn can_remove_vertex(&self, rem: u16) -> bool {
        let nvp = self.max_vertices_per_polygon as usize;

        // Count number of polygons to remove.
        let mut num_touched_verts = 0;
        let mut num_remaining_edges = 0;
        for i in 0..self.npolys {
            let p = &self.polygons[i * nvp * 2..];
            let nv = count_poly_verts(p, nvp);
            let mut num_removed = 0;
            let mut num_verts = 0;
            for pj in p.iter().take(nv) {
                if *pj == rem {
                    num_touched_verts += 1;
                    num_removed += 1;
                }
                num_verts += 1;
            }
            if num_removed != 0 {
                num_remaining_edges += num_verts - (num_removed + 1);
            }
        }

        // There would be too few edges remaining to create a polygon.
        // This can happen for example when a tip of a triangle is marked
        // as deletion, but there are no other polys that share the vertex.
        // In this case, the vertex should not be removed.
        if num_remaining_edges <= 2 {
            return false;
        }
        // Find edges which share the removed vertex.
        let max_edges = num_touched_verts * 2;
        if self.tile_cache && max_edges > 48 {
            return false;
        }
        let mut nedges = 0;
        // Format: [poly1, poly2, vertex share count]
        let mut edges = vec![U16Vec3::ZERO; max_edges];
        for i in 0..self.npolys {
            let p = &self.polygons[i * nvp * 2..];
            let nv = count_poly_verts(p, nvp);

            // Collect edges which touches the removed vertex.
            for j in 0..nv {
                let k = if j == 0 { nv - 1 } else { j - 1 };
                if !(p[j] == rem || p[k] == rem) {
                    continue;
                }
                // Arrange edge so that a=rem.
                let a = p[j];
                let b = p[k];
                let (a, b) = if b != rem { (a, b) } else { (b, a) };

                // Check if the edge exists
                let mut exists = false;
                for e in edges.iter_mut().take(nedges) {
                    if e[1] == b {
                        // Exists, increment vertex share count.
                        e[2] += 1;
                        exists = true;
                    }
                }
                // Add new edge
                if !exists {
                    let e = &mut edges[nedges];
                    e[0] = a;
                    e[1] = b;
                    e[2] = 1;
                    nedges += 1;
                }
            }
        }

        // There should be no more than 2 open edges.
        // This catches the case that two non-adjacent polygons
        // share the removed vertex. In that case, do not remove the vertex.
        let num_open_edges = edges.iter().take(nedges).filter(|e| e[2] < 2).count();
        num_open_edges <= 2
    }
}

fn push_back<T>(value: T, vec: &mut [T], index: &mut usize) {
    vec[*index] = value;
    *index += 1;
}

fn push_front<T: Clone>(value: T, vec: &mut [T], index: &mut usize) {
    *index += 1;
    for i in (1..*index).rev() {
        vec[i] = vec[i - 1].clone();
    }
    vec[0] = value;
}

// Jan: signature changed to align with the borrow checker :)
fn merge_poly_verts(
    polys: &mut [u16],
    pa_index: usize,
    pb_index: usize,
    ea: usize,
    eb: usize,
    tmp_index: usize,
    nvp: usize,
) {
    let na = count_poly_verts(&polys[pa_index..], nvp);
    let nb = count_poly_verts(&polys[pb_index..], nvp);

    // Merge polygons.
    polys[tmp_index..tmp_index + nvp].fill(u16::MAX);
    let mut n = 0;
    // Add pa
    let mut at = next(ea, na);
    for _ in 0..na - 1 {
        polys[tmp_index + n] = polys[pa_index + at];
        n += 1;
        at = next(at, na);
    }
    // Add pb
    let mut at = next(eb, nb);
    for _ in 0..nb - 1 {
        polys[tmp_index + n] = polys[pb_index + at];
        n += 1;
        at = next(at, nb);
    }

    // Implicit assumption of the original code
    polys.copy_within(tmp_index..tmp_index + nvp, pa_index);
}

struct PolyMergePair {
    polygon_a: usize,
    polygon_b: usize,
    value: PolyMergeValue,
}

fn find_best_poly_merge(
    polys: &[u16],
    npolys: usize,
    verts: &[U16Vec3],
    nvp: usize,
) -> Option<PolyMergePair> {
    if npolys < 2 {
        return None;
    }
    let mut best_value = 0;
    let mut best = None;
    // One immutable search round only. The caller applies the chosen merge
    // after returning, so no cached key survives a mutation or swapped row.
    if nvp == 6 {
        let mut second_keys = [None; 16];
        for j in 0..npolys - 1 {
            let pa = &polys[j * 6..j * 6 + 6];
            let na = count_poly_verts(pa, 6);
            let pa = &pa[..na];
            let first_keys = packed_edge_keys(pa);
            for k in j + 1..npolys {
                let pb = &polys[k * 6..k * 6 + 6];
                let nb = count_poly_verts(pb, 6);
                if na + nb > 8 {
                    continue;
                }
                let pb = &pb[..nb];
                let mut uncached = None;
                let keys = second_keys.get_mut(k).unwrap_or(&mut uncached);
                let keys = keys.get_or_insert_with(|| packed_edge_keys(pb));
                let value = shared_packed_edge(&first_keys[..na], keys, nb)
                    .and_then(|(ea, eb)| poly_merge_edge_value(pa, pb, verts, ea, eb));
                if let Some(value) = value
                    && value.length_squared > best_value
                {
                    best_value = value.length_squared;
                    best = Some(PolyMergePair {
                        polygon_a: j,
                        polygon_b: k,
                        value,
                    });
                }
            }
        }
    } else {
        for j in 0..npolys - 1 {
            let pa = &polys[j * nvp..];
            for k in j + 1..npolys {
                let pb = &polys[k * nvp..];
                if let Some(value) = get_poly_merge_value(pa, pb, verts, nvp)
                    && value.length_squared > best_value
                {
                    best_value = value.length_squared;
                    best = Some(PolyMergePair {
                        polygon_a: j,
                        polygon_b: k,
                        value,
                    });
                }
            }
        }
    }
    best
}

fn packed_edge_keys(poly: &[u16]) -> [u32; 6] {
    let mut keys = [0; 6];
    for (i, key) in keys[..poly.len()].iter_mut().enumerate() {
        let a = poly[i];
        let b = poly[next(i, poly.len())];
        *key = u32::from(a.min(b)) | (u32::from(a.max(b)) << 16);
    }
    keys
}

fn shared_packed_edge(first: &[u32], second: &[u32; 6], count: usize) -> Option<(usize, usize)> {
    // Native selects the last matching first edge and the first matching
    // second edge. Search that first ordering backwards to stop at the winner.
    let active = (1u8 << count) - 1;
    for (i, &key) in first.iter().enumerate().rev() {
        let mut matches = 0u8;
        for (j, &other) in second.iter().enumerate() {
            matches |= u8::from(key == other) << j;
        }
        let matches = matches & active;
        if matches != 0 {
            return Some((i, matches.trailing_zeros() as usize));
        }
    }
    None
}

fn get_poly_merge_value(
    pa: &[u16],
    pb: &[u16],
    verts: &[U16Vec3],
    nvp: usize,
) -> Option<PolyMergeValue> {
    let na = count_poly_verts(pa, nvp);
    let nb = count_poly_verts(pb, nvp);
    if na + nb - 2 > nvp {
        return None;
    }
    let pa = &pa[..na];
    let pb = &pb[..nb];
    let mut shared = None;
    for i in 0..na {
        let va0 = pa[i];
        let va1 = pa[next(i, na)];
        let (va0, va1) = if va0 <= va1 { (va0, va1) } else { (va1, va0) };
        for j in 0..nb {
            let vb0 = pb[j];
            let vb1 = pb[next(j, nb)];
            let (vb0, vb1) = if vb0 <= vb1 { (vb0, vb1) } else { (vb1, vb0) };
            if va0 == vb0 && va1 == vb1 {
                shared = Some((i, j));
                break;
            }
        }
    }
    let (ea, eb) = shared?;
    poly_merge_edge_value(pa, pb, verts, ea, eb)
}

fn poly_merge_edge_value(
    pa: &[u16],
    pb: &[u16],
    verts: &[U16Vec3],
    ea: usize,
    eb: usize,
) -> Option<PolyMergeValue> {
    let na = pa.len();
    let nb = pb.len();
    // Check to see if the merged polygon would be convex.
    let mut va = pa[prev(ea, na)] as usize;
    let mut vb = pa[ea] as usize;
    let mut vc = pb[next(next(eb, nb), nb)] as usize;
    if !uleft(verts[va], verts[vb], verts[vc]) {
        return None;
    }

    va = pb[prev(eb, nb)] as usize;
    vb = pb[eb] as usize;
    vc = pa[next(next(ea, na), na)] as usize;
    if !uleft(verts[va], verts[vb], verts[vc]) {
        return None;
    };

    va = pa[ea] as usize;
    vb = pa[next(ea, na)] as usize;

    let d = verts[va].as_ivec3() - verts[vb].as_ivec3();
    let length_squared = d.xz().length_squared() as u32;
    Some(PolyMergeValue {
        length_squared,
        edge_a: ea,
        edge_b: eb,
    })
}

#[inline]
fn uleft(a: U16Vec3, b: U16Vec3, c: U16Vec3) -> bool {
    let cross = (b.x as i32 - a.x as i32) * (c.z as i32 - a.z as i32)
        - (c.x as i32 - a.x as i32) * (b.z as i32 - a.z as i32);
    cross < 0
}

fn count_poly_verts(p: &[u16], nvp: usize) -> usize {
    p.iter()
        .take(nvp)
        .position(|p| *p == PolygonNavmesh::NO_INDEX)
        .unwrap_or(nvp)
}

#[derive(Debug)]
struct PolyMergeValue {
    length_squared: u32,
    edge_a: usize,
    edge_b: usize,
}

fn add_vertex(
    vertex: U16Vec3,
    verts: &mut [U16Vec3],
    first_vert: &mut [u16],
    next_vert: &mut [u16],
    nverts: &mut usize,
) -> Result<u16, PolygonNavmeshError> {
    let bucket = compute_vertex_hash(u16vec3(vertex.x, 0, vertex.z), first_vert.len());
    let mut i_iter = first_vert[bucket];

    let mut traversed = 0;
    while i_iter != u16::MAX {
        if traversed >= usize::from(u16::MAX) {
            return Err(PolygonNavmeshError::VertexChainCycle);
        }
        traversed += 1;
        let i = i_iter;
        let v = verts[i as usize];
        if v.x == vertex.x && (v.y as i32 - vertex.y as i32).abs() <= 2 && v.z == vertex.z {
            return Ok(i);
        }
        i_iter = next_vert[i as usize];
    }

    // TileCache truncates the index before writing, while retaining its int
    // vertex count and zero-initialized unused rows. Recast's earlier summed
    // contour cutoff keeps its indices below this boundary.
    let i = *nverts as u16;
    *nverts += 1;
    verts[i as usize] = vertex;
    next_vert[i as usize] = first_vert[bucket];
    first_vert[bucket] = i;

    Ok(i)
}

fn compute_vertex_hash(vertex: U16Vec3, bucket_count: usize) -> usize {
    let h = uvec3(
        0x8da6b343, // Large multiplicative constants;
        0xd8163841, // here arbitrarily chosen primes
        0xcb1ab31f,
    );
    let v = vertex.as_uvec3();
    let n = h[0]
        .wrapping_mul(v[0])
        .wrapping_add(h[1].wrapping_mul(v[1]))
        .wrapping_add(h[2].wrapping_mul(v[2]));
    n as usize & (bucket_count - 1)
}

const VERTEX_BUCKET_COUNT: usize = 1 << 12;

fn triangulate<V: ContourVertices + ?Sized>(
    mut n: usize,
    verts: &V,
    indices: &mut [usize],
    tris: &mut [U16Vec3],
    allow_loose: bool,
) -> isize {
    if n < 3 {
        return 0;
    }
    if n == 3 {
        // No ear will be selected. The private flags would only be masked
        // away by triangle output and overwritten/discarded by both callers.
        tris[0] = U16Vec3::new(
            (indices[0] & INDEX_MASK) as u16,
            (indices[1] & INDEX_MASK) as u16,
            (indices[2] & INDEX_MASK) as u16,
        );
        return 1;
    }
    if n == 4 && !allow_loose && V::BYTE_COORDINATES {
        // Byte contours bound squared XZ lengths within i32. Only one ear
        // is selected, so longer candidates' flags are never used. Owned
        // contours and vertex-removal holes retain the general path.
        let mut best = None;
        let mut minimum = 0;
        for i in 0..4 {
            let i1 = next(i, 4);
            let i2 = next(i1, 4);
            let p0 = verts.position(indices[i] & INDEX_MASK);
            let p2 = verts.position(indices[i2] & INDEX_MASK);
            let d = p2 - p0;
            let length = d.xz().length_squared();
            if (best.is_none() || length < minimum) && is_diagonal(i, i2, 4, verts, indices) {
                minimum = length;
                best = Some(i);
            }
        }
        let Some(i) = best else {
            return 0;
        };
        let i1 = next(i, 4);
        let i2 = next(i1, 4);
        tris[0] = U16Vec3::new(
            (indices[i] & INDEX_MASK) as u16,
            (indices[i1] & INDEX_MASK) as u16,
            (indices[i2] & INDEX_MASK) as u16,
        );
        indices.copy_within(i1 + 1..4, i1);
        tris[1] = U16Vec3::new(
            (indices[0] & INDEX_MASK) as u16,
            (indices[1] & INDEX_MASK) as u16,
            (indices[2] & INDEX_MASK) as u16,
        );
        return 2;
    }
    let mut ntris = 0;

    // The last bit of the index is used to indicate if the vertex can be removed.
    for i in 0..n {
        let i1 = next(i, n);
        let i2 = next(i1, n);
        if is_diagonal(i, i2, n, verts, indices) {
            indices[i1] |= CAN_REMOVE;
        }
    }
    while n > 3 {
        let mut min_len = None;
        let mut mini = None;
        for i in 0..n {
            let i1 = next(i, n);
            if (indices[i1] & CAN_REMOVE) != 0 {
                let p0 = verts.position(indices[i] & INDEX_MASK);
                let p2 = verts.position(indices[next(i1, n)] & INDEX_MASK);

                let d = p2 - p0;
                let len = d.xz().length_squared();
                if min_len.is_none_or(|min| len < min) {
                    min_len = Some(len);
                    mini = Some(i);
                }
            }
        }
        if mini.is_none() && allow_loose {
            // We might get here because the contour has overlapping segments, like this:
            //
            //  A o-o=====o---o B
            //   /  |C   D|    \.
            //  o   o     o     o
            //  :   :     :     :
            // We'll try to recover by loosing up the inCone test a bit so that a diagonal
            // like A-B or C-D can be found and we can continue.
            min_len = None;
            for i in 0..n {
                let i1 = next(i, n);
                let i2 = next(i1, n);
                if is_diagonal_loose(i, i2, n, verts, indices) {
                    let p0 = verts.position(indices[i] & INDEX_MASK);
                    let p2 = verts.position(indices[next(i2, n)] & INDEX_MASK);
                    let d = p2 - p0;
                    let len = d.xz().length_squared();
                    if min_len.is_none_or(|min| len < min) {
                        min_len = Some(len);
                        mini = Some(i);
                    }
                }
            }
        }

        let Some(mini) = mini else {
            // The contour is messed up. This sometimes happens
            // if the contour simplification is too aggressive.
            return -(ntris as isize);
        };

        let mut i = mini;
        let mut i1 = next(i, n);
        let i2 = next(i1, n);

        tris[ntris].x = (indices[i] & INDEX_MASK) as u16;
        tris[ntris].y = (indices[i1] & INDEX_MASK) as u16;
        tris[ntris].z = (indices[i2] & INDEX_MASK) as u16;
        ntris += 1;

        // Removes P[i1] by copying P[i+1]...P[n-1] left one index.
        n -= 1;
        for k in i1..n {
            indices[k] = indices[k + 1];
        }
        if n == 3 {
            break;
        }

        if i1 >= n {
            i1 = 0;
        }
        i = prev(i1, n);
        // Update diagonal flags.
        if is_diagonal(prev(i, n), i1, n, verts, indices) {
            indices[i] |= CAN_REMOVE;
        } else {
            indices[i] &= INDEX_MASK;
        }

        if is_diagonal(i, next(i1, n), n, verts, indices) {
            indices[i1] |= CAN_REMOVE;
        } else {
            indices[i1] &= INDEX_MASK;
        }
    }
    // Append the remaining triangle.
    tris[ntris].x = (indices[0] & INDEX_MASK) as u16;
    tris[ntris].y = (indices[1] & INDEX_MASK) as u16;
    tris[ntris].z = (indices[2] & INDEX_MASK) as u16;
    ntris += 1;

    ntris as isize
}

const CAN_REMOVE: usize = 0x80000000;

/// Returns true iff (v_i, v_j) is a proper internal diagonal of P.
fn is_diagonal<V: ContourVertices + ?Sized>(
    i: usize,
    j: usize,
    n: usize,
    verts: &V,
    indices: &[usize],
) -> bool {
    in_cone(i, j, n, verts, indices) && is_diagonal_internal_or_external(i, j, n, verts, indices)
}

/// Returns true iff the diagonal (i,j) is strictly internal to the
/// polygon P in the neighborhood of the i endpoint.
fn in_cone<V: ContourVertices + ?Sized>(
    i: usize,
    j: usize,
    n: usize,
    verts: &V,
    indices: &[usize],
) -> bool {
    let pi = verts.position(indices[i] & INDEX_MASK);
    let pj = verts.position(indices[j] & INDEX_MASK);
    let pi1 = verts.position(indices[next(i, n)] & INDEX_MASK);
    let pin1 = verts.position(indices[prev(i, n)] & INDEX_MASK);

    // If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
    if is_left_on(pin1, pi, pi1) {
        is_left(pi, pj, pin1) && is_left(pj, pi, pi1)
    } else {
        // Assume (i-1,i,i+1) not collinear.
        // else P[i] is reflex.
        !(is_left_on(pi, pj, pi1) && is_left_on(pj, pi, pin1))
    }
}

#[inline]
fn is_left_on(a: IVec3, b: IVec3, c: IVec3) -> bool {
    area2(a, b, c) <= 0
}

/// Returns true iff c is strictly to the left of the directed line through a to b.
#[inline]
fn is_left(a: IVec3, b: IVec3, c: IVec3) -> bool {
    area2(a, b, c) < 0
}

#[inline]
fn area2(a: IVec3, b: IVec3, c: IVec3) -> i32 {
    (b.x - a.x) * (c.z - a.z) - (c.x - a.x) * (b.z - a.z)
}

// Returns T iff (v_i, v_j) is a proper internal *or* external
// diagonal of P, *ignoring edges incident to v_i and v_j*.
fn is_diagonal_internal_or_external<V: ContourVertices + ?Sized>(
    i: usize,
    j: usize,
    n: usize,
    verts: &V,
    indices: &[usize],
) -> bool {
    let d0 = verts.position(indices[i] & INDEX_MASK);
    let d1 = verts.position(indices[j] & INDEX_MASK);
    let minimum = d0.xz().min(d1.xz());
    let maximum = d0.xz().max(d1.xz());

    // For each edge (k,k+1) of P
    for k in 0..n {
        let k1 = next(k, n);
        // Skip edges incident to i or j
        if !((k == i) || (k1 == i) || (k == j) || (k1 == j)) {
            let p0 = verts.position(indices[k] & INDEX_MASK);
            let p1 = verts.position(indices[k1] & INDEX_MASK);
            // Closed boxes must overlap for the segments to intersect. Keep
            // touching boxes on the original collinearity/betweenness path.
            if p0.x.min(p1.x) > maximum.x
                || p0.x.max(p1.x) < minimum.x
                || p0.z.min(p1.z) > maximum.y
                || p0.z.max(p1.z) < minimum.y
            {
                continue;
            }
            if vequal(d0, p0) || vequal(d1, p0) || vequal(d0, p1) || vequal(d1, p1) {
                continue;
            }
            if intersect(d0, d1, p0, p1) {
                return false;
            }
        }
    }
    true
}

const INDEX_MASK: usize = 0x0fffffff;

#[inline]
fn vequal(a: IVec3, b: IVec3) -> bool {
    a.xz() == b.xz()
}

/// Returns true iff segments ab and cd intersect, properly or improperly.
#[inline]
fn intersect(a: IVec3, b: IVec3, c: IVec3, d: IVec3) -> bool {
    if intersect_prop(a, b, c, d) {
        return true;
    }
    between(a, b, c) || between(a, b, d) || between(c, d, a) || between(c, d, b)
}

/// Returns true iff ab properly intersects cd: they share
/// a point interior to both segments.  The properness of the
/// intersection is ensured by using strict leftness.
#[inline]
fn intersect_prop(a: IVec3, b: IVec3, c: IVec3, d: IVec3) -> bool {
    // Eliminate improper cases.
    if collinear(a, b, c) || collinear(a, b, d) || collinear(c, d, a) || collinear(c, d, b) {
        return false;
    }
    (left(a, b, c) ^ left(a, b, d)) && (left(c, d, a) ^ left(c, d, b))
}

#[inline]
fn collinear(a: IVec3, b: IVec3, c: IVec3) -> bool {
    area2(a, b, c) == 0
}

/// Returns true iff c is strictly to the left of the directed
/// line through a to b.
#[inline]
fn left(a: IVec3, b: IVec3, c: IVec3) -> bool {
    area2(a, b, c) < 0
}

#[inline]
fn left_on(a: IVec3, b: IVec3, c: IVec3) -> bool {
    area2(a, b, c) <= 0
}

/// Returns T iff (a,b,c) are collinear and point c lies
/// on the closed segment ab.
#[inline]
fn between(a: IVec3, b: IVec3, c: IVec3) -> bool {
    if !collinear(a, b, c) {
        return false;
    }
    // If ab not vertical, check betweenness on x; else on z.
    if a.x != b.x {
        (a.x <= c.x && c.x <= b.x) || (a.x >= c.x && c.x >= b.x)
    } else {
        (a.z <= c.z && c.z <= b.z) || (a.z >= c.z && c.z >= b.z)
    }
}

fn is_diagonal_loose<V: ContourVertices + ?Sized>(
    i: usize,
    j: usize,
    n: usize,
    verts: &V,
    indices: &[usize],
) -> bool {
    in_cone_loose(i, j, n, verts, indices)
        && is_diagonal_internal_or_external_loose(i, j, n, verts, indices)
}

fn in_cone_loose<V: ContourVertices + ?Sized>(
    i: usize,
    j: usize,
    n: usize,
    verts: &V,
    indices: &[usize],
) -> bool {
    let pi = verts.position(indices[i] & INDEX_MASK);
    let pj = verts.position(indices[j] & INDEX_MASK);
    let pi1 = verts.position(indices[next(i, n)] & INDEX_MASK);
    let pin1 = verts.position(indices[prev(i, n)] & INDEX_MASK);

    // If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
    if left_on(pin1, pi, pi1) {
        left_on(pi, pj, pin1) && left_on(pj, pi, pi1)
    } else {
        !(left_on(pi, pj, pi1) && left_on(pj, pi, pin1))
    }
}

fn is_diagonal_internal_or_external_loose<V: ContourVertices + ?Sized>(
    i: usize,
    j: usize,
    n: usize,
    verts: &V,
    indices: &[usize],
) -> bool {
    let d0 = verts.position(indices[i] & INDEX_MASK);
    let d1 = verts.position(indices[j] & INDEX_MASK);

    // For each edge (k,k+1) of P
    for k in 0..n {
        let k1 = next(k, n);
        // Skip edges incident to i or j
        if !(k == i || k1 == i || k == j || k1 == j) {
            let p0 = verts.position(indices[k] & INDEX_MASK);
            let p1 = verts.position(indices[k1] & INDEX_MASK);
            if vequal(d0, p0) || vequal(d1, p0) || vequal(d0, p1) || vequal(d1, p1) {
                continue;
            }
            if intersect_prop(d0, d1, p0, p1) {
                return false;
            }
        }
    }
    true
}

#[derive(Debug)]
pub enum PolygonNavmeshError {
    VertexChainCycle,
    VertexRemovalBufferTooSmall,
    TooManyVertices { actual: usize, max: usize },
    TooManyPolygons { actual: usize, max: usize },
}

impl core::fmt::Display for PolygonNavmeshError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::VertexChainCycle => {
                f.write_str("native wrapped vertex hash chain does not terminate")
            }
            Self::VertexRemovalBufferTooSmall => {
                f.write_str("Tile-cache vertex-removal buffer exhausted")
            }
            Self::TooManyVertices { actual, max } => {
                write!(f, "Too many vertices: {actual} > {max}")
            }
            Self::TooManyPolygons { actual, max } => {
                write!(f, "Too many polygons: {actual} > {max}")
            }
        }
    }
}
impl core::error::Error for PolygonNavmeshError {}

#[cfg(test)]
mod tests {
    #[test]
    fn shared_edge_uses_native_duplicate_order() {
        assert_eq!(
            super::shared_packed_edge(&[7, 3, 7, 9], &[7, 9, 7, 9, 3, 9], 4),
            Some((3, 1))
        );
        assert_eq!(
            super::shared_packed_edge(&[3], &[7, 9, 7, 9, 3, 9], 4),
            None
        );
    }
}
