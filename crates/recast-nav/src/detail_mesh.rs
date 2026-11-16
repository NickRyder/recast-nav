use crate::ops::*;
use alloc::vec::Vec;

use core::ops::{Deref, DerefMut};
use glam::{IVec3, U16Vec3, Vec2, Vec3, Vec3A, Vec3Swizzles as _, ivec3};

#[cfg(not(feature = "fma"))]
use crate::math::distance_squared_between_point_and_line_vec2;
use crate::{
    Aabb3d, CompactHeightfield, PolygonNavmesh, RegionId,
    context::{Context, LogCategory, TimerLabel},
    math::{
        dir_offset, dir_offset_x, dir_offset_z, distance_squared_between_point_and_line_vec3, next,
        prev,
    },
};

/// Contains triangle meshes that represent detailed height data associated with the polygons in its associated polygon mesh object.
///
/// The detail mesh is made up of triangle sub-meshes that provide extra height detail for each polygon in its assoicated polygon mesh.
///
/// The standard process for building a detail mesh is to build it using [`DetailNavmesh::new`].
///
/// See the individual field definitions for details related to the structure the mesh.
#[derive(Debug, Default, Clone, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct DetailNavmesh {
    /// The sub-mesh data.
    ///
    /// Maximum number of vertices per sub-mesh: [`Self::MAX_VERTICES_PER_SUBMESH`]
    /// Maximum number of triangles per sub-mesh: [`Self::MAX_TRIANGLES_PER_SUBMESH`]
    ///
    /// The sub-meshes are stored in the same order as the polygons from the [`PolygonNavmesh`] they represent.
    /// E.g. [`DetailNavmesh`] sub-mesh 5 is associated with [`PolygonNavmesh`] polygon 5.
    ///
    /// Example of iterating the triangles in a sub-mesh.
    /// ```rust
    /// # use recast_nav::*;
    /// # let dmesh = DetailNavmesh::default();
    /// // Where dmesh is a DetailNavmesh
    ///
    /// // Iterate the sub-meshes. (One for each source polygon.)
    /// for mesh in &dmesh.meshes {
    ///     let verts =
    ///         &dmesh.vertices[mesh.base_vertex_index as usize..][..mesh.vertex_count as usize];
    ///     let tris =
    ///         &dmesh.triangles[mesh.base_triangle_index as usize..][..mesh.triangle_count as usize];
    ///
    ///     // Iterate the sub-meshes. (One for each source polygon.)
    ///     for mesh in &dmesh.meshes {
    ///         let verts =
    ///             &dmesh.vertices[mesh.base_vertex_index as usize..][..mesh.vertex_count as usize];
    ///         let tris =
    ///             &dmesh.triangles[mesh.base_triangle_index as usize..][..mesh.triangle_count as usize];
    ///
    ///         // Iterate the sub-mesh's triangles.
    ///         for tri in tris {
    ///             let a = verts[tri[0] as usize];
    ///             let b = verts[tri[1] as usize];
    ///             let c = verts[tri[2] as usize];
    ///
    ///             // Do something with the vertex.
    ///             println!("Vertex A: {a}");
    ///             println!("Vertex B: {b}");
    ///             println!("Vertex C: {c}");
    ///         }
    ///     }
    /// }
    /// ```
    pub meshes: Vec<SubMesh>,

    /// The mesh vertices.
    ///
    /// The vertices are grouped by sub-mesh and will contain duplicates since each sub-mesh is independently defined.
    ///
    /// The first group of vertices for each sub-mesh are in the same order as the vertices for the sub-mesh's associated [`PolygonNavmesh`] polygon.
    /// These vertices are followed by any additional detail vertices.
    /// So if the associated polygon has 5 vertices, the sub-mesh will have a minimum of 5 vertices and the first 5 vertices will be equivalent to the 5 polygon vertices.
    pub vertices: Vec<Vec3>,
    /// The mesh triangles.
    ///
    /// The triangles are grouped by sub-mesh. Their winding order is clockwise on the XZ plane.
    ///
    /// ## Vertex Indices
    ///
    /// The vertex indices in the triangle array are local to the sub-mesh, not global.
    /// To translate into an global index in the vertices array, the values must be offset by the sub-mesh's base vertex index.
    ///
    /// Example: If the [`SubMesh::base_vertex_index`] is 5 and the triangle entry is (4, 8, 7), then the actual indices for the vertices are (4 + 5, 8 + 5, 7 + 5).
    pub triangles: Vec<[u8; 3]>,
    /// Flags corresponding to [`DetailNavmesh::triangles`].
    /// Indicates which edges are internal and which are external to the sub-mesh.
    /// Internal edges connect to other triangles within the same sub-mesh.
    /// External edges represent portals to other sub-meshes or the null region.
    ///
    /// Each flag is stored in a 2-bit position. Where position 0 is the lowest 2-bits and position 4 is the highest 2-bits:
    ///
    /// Position 0: Edge AB (>> 0)
    /// Position 1: Edge BC (>> 2)
    /// Position 2: Edge CA (>> 4)
    /// Position 4: Unused
    ///
    /// Testing can be performed as follows:
    /// if (((flags >> 2) & 0x3) != 0)
    /// {
    ///     // Edge BC is an external edge.
    /// }
    pub triangle_flags: Vec<u8>,
}

/// A sub-mesh in [`DetailNavmesh::meshes`]
#[derive(Debug, Default, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct SubMesh {
    /// The index in [`DetailNavmesh::vertices`] that begins this sub-mesh.
    pub base_vertex_index: u32,
    /// Length of the sub-mesh in [`DetailNavmesh::vertices`]
    pub vertex_count: u32,
    /// The index in [`DetailNavmesh::triangles`] that begins this sub-mesh.
    pub base_triangle_index: u32,
    /// Length of the sub-mesh in [`DetailNavmesh::triangles`]
    pub triangle_count: u32,
}

impl DetailNavmesh {
    /// The maximum number of vertices per entry in [`DetailNavmesh::meshes`]
    pub const MAX_VERTICES_PER_SUBMESH: usize = 127;
    // Max tris for delaunay is 2n-2-k (n=num verts, k=num hull verts).
    /// The maximum number of triangles per entry in [`DetailNavmesh::meshes`]
    pub const MAX_TRIANGLES_PER_SUBMESH: usize = u8::MAX as usize;
    const MAX_VERTS_PER_EDGE: usize = 32;

    /// Builds a detail mesh from the provided polygon mesh.
    pub fn new(
        mesh: &PolygonNavmesh,
        heightfield: &CompactHeightfield,
        sample_distance: f32,
        sample_max_error: f32,
    ) -> Result<Self, DetailNavmeshError> {
        Self::new_with_context(
            &Context::default(),
            mesh,
            heightfield,
            sample_distance,
            sample_max_error,
        )
    }

    /// Build a detail mesh with native timer and diagnostic callbacks.
    pub fn new_with_context(
        context: &Context<'_>,
        mesh: &PolygonNavmesh,
        heightfield: &CompactHeightfield,
        sample_distance: f32,
        sample_max_error: f32,
    ) -> Result<Self, DetailNavmeshError> {
        let _timer = context.scoped_timer(TimerLabel::BUILD_POLYMESHDETAIL);
        let mut dmesh = DetailNavmesh::default();
        if mesh.vertices.is_empty() || mesh.polygon_count() == 0 {
            return Ok(dmesh);
        }
        if sample_distance <= 0. || sample_distance.is_nan() {
            // Native leaves hull[] uninitialized when edge sampling is disabled,
            // then reads hull[nhull-1] while triangulating. Do not run that domain.
            return Err(DetailNavmeshError::UninitializedHull);
        }
        let chf = heightfield;
        let nvp = mesh.max_vertices_per_polygon as usize;
        let cs = mesh.cell_size;
        let ch = mesh.cell_height;
        let orig = Vec3A::from(mesh.aabb.min);
        let border_size = mesh.border_size;
        let height_search_radius = 1.max(ceil(mesh.max_edge_error) as u32);

        let mut triangulation = TriangulationScratch {
            edges: Vec::with_capacity(64 / 4),
            faces: Vec::new(),
        };
        let mut tris = Vec::with_capacity((512 / 4) * 3);
        let mut flags = Vec::with_capacity(512 / 4);
        let mut arr = Vec::with_capacity(512 / 3);
        let mut samples = Vec::with_capacity(512 / 4);
        let mut verts = [Vec3A::default(); 256];
        let mut hp = HeightPatch::default();
        let mut poly_vert_count = 0;
        let mut maxhw = 0;
        let mut maxhh = 0;

        let mut bounds = vec![Bounds::default(); mesh.polygon_count()];
        let mut poly = vec![Vec3A::default(); nvp];

        // Find max size for a polygon area.
        for (i, b) in bounds.iter_mut().enumerate() {
            let p = &mesh.polygons[i * nvp..];
            let Bounds {
                xmin,
                xmax,
                zmin,
                zmax,
            } = b;
            *xmin = chf.width;
            *xmax = 0;
            *zmin = chf.height;
            *zmax = 0;
            for pj in &p[..nvp] {
                if *pj == PolygonNavmesh::NO_INDEX {
                    break;
                }
                let v = &mesh.vertices[*pj as usize];
                *xmin = (*xmin).min(i32::from(v.x));
                *xmax = (*xmax).max(i32::from(v.x));
                *zmin = (*zmin).min(i32::from(v.z));
                *zmax = (*zmax).max(i32::from(v.z));
                poly_vert_count += 1;
            }
            *xmin = (*xmin - 1).max(0);
            *xmax = chf.width.min(*xmax + 1);
            *zmin = (*zmin - 1).max(0);
            *zmax = chf.height.min(*zmax + 1);
            if xmin >= xmax || zmin >= zmax {
                continue;
            }
            maxhw = maxhw.max(*xmax - *xmin);
            maxhh = maxhh.max(*zmax - *zmin);
        }
        hp.data = vec![0; maxhw as usize * maxhh as usize];
        dmesh.meshes = vec![SubMesh::default(); mesh.polygon_count()];

        let mut vcap = poly_vert_count + poly_vert_count / 2;
        let mut tcap = vcap * 2;

        dmesh.vertices = Vec::with_capacity(vcap);
        dmesh.triangles = Vec::with_capacity(tcap);

        for (i, bounds_i) in bounds.iter().enumerate().take(mesh.polygon_count()) {
            let p = &mesh.polygons[i * nvp..];

            // Store polygon vertices for processing.
            let mut npoly = 0;
            for j in 0..nvp {
                if p[j] == PolygonNavmesh::NO_INDEX {
                    break;
                }
                let v = mesh.vertices[p[j] as usize].as_vec3();
                poly[j].x = v.x * cs;
                poly[j].y = v.y * ch;
                poly[j].z = v.z * cs;
                npoly += 1;
            }

            // Get the height data from the area of the polygon.
            hp.xmin = bounds_i.xmin;
            hp.zmin = bounds_i.zmin;
            hp.width = bounds_i.width();
            hp.height = bounds_i.height();
            // Fallback seeds use global polygon indices and voxel coordinates.
            hp.get_height_data(
                context,
                chf,
                p,
                npoly,
                &mesh.vertices,
                border_size,
                &mut arr,
                mesh.regions[i],
            );

            // Build detail mesh.
            let mut nverts = 0;
            build_poly_detail(
                context,
                &poly,
                npoly,
                sample_distance,
                sample_max_error,
                height_search_radius,
                chf,
                &hp,
                &mut verts,
                &mut nverts,
                &mut tris,
                &mut flags,
                &mut triangulation,
                &mut samples,
            )?;

            // Move detail verts to world space.
            for vert in &mut verts[..nverts] {
                *vert += orig;
                // [sic] Is this offset necessary?
                vert.y += chf.cell_height;
            }
            // Offset poly too, will be used to flag checking.
            for poly in &mut poly[..npoly] {
                *poly += orig;
            }

            // Store detail submesh
            let submesh = &mut dmesh.meshes[i];
            submesh.base_vertex_index = dmesh.vertices.len() as u32;
            submesh.vertex_count = nverts as u32;
            submesh.base_triangle_index = dmesh.triangles.len() as u32;
            submesh.triangle_count = tris.len() as u32;

            // Store vertices, allocate more memory if necessary.
            if dmesh.vertices.len() + nverts > vcap {
                while dmesh.vertices.len() + nverts > vcap {
                    vcap += 256;
                }
                dmesh.vertices.reserve(vcap - dmesh.vertices.len());
            }
            for vert in &verts[..nverts] {
                dmesh.vertices.push(Vec3::from(*vert));
            }

            // Store triangles, allocate more memory if necessary.
            if dmesh.triangles.len() + tris.len() > tcap {
                while dmesh.triangles.len() + tris.len() > tcap {
                    tcap += 256;
                }
                dmesh.triangles.reserve(tcap - dmesh.triangles.len());
            }
            for tri in &tris {
                dmesh.triangles.push([tri[0], tri[1], tri[2]]);
            }
            for flag in &flags {
                dmesh.triangle_flags.push(*flag);
            }
        }

        Ok(dmesh)
    }
}
#[expect(
    clippy::too_many_arguments,
    reason = "Keeps native inputs and caller-owned scratch explicit"
)]
fn build_poly_detail(
    context: &Context<'_>,
    in_: &[Vec3A],
    nin: usize,
    sample_dist: f32,
    sample_max_error: f32,
    height_search_radius: u32,
    chf: &CompactHeightfield,
    hp: &HeightPatch,
    verts: &mut [Vec3A],
    nverts: &mut usize,
    tris: &mut Vec<[u8; 3]>,
    flags: &mut Vec<u8>,
    triangulation: &mut TriangulationScratch,
    samples: &mut Vec<(IVec3, bool)>,
) -> Result<(), DetailNavmeshError> {
    let mut edge = [Vec3A::default(); DetailNavmesh::MAX_VERTS_PER_EDGE + 1];
    let mut hull = [0; DetailNavmesh::MAX_VERTICES_PER_SUBMESH];
    let mut nhull = 0;

    *nverts = nin;

    verts[..nin].clone_from_slice(&in_[..nin]);
    triangulation.edges.clear();
    tris.clear();
    flags.clear();

    let cs = chf.cell_size;
    let ics = 1.0 / cs;

    // Calculate minimum extents of the polygon based on input data.
    let min_extent = poly_min_extent(verts, *nverts);

    // Tessellate outlines.
    // This is done in separate pass in order to ensure
    // seamless height values across the ply boundaries.
    if sample_dist > 0.0 {
        let mut j = nin - 1;
        for i in 0..nin {
            let mut vj = in_[j];
            let mut vi = in_[i];
            let mut swapped = false;
            // Make sure the segments are always handled in same order
            // using lexological sort or else there will be seams.
            if abs(vj.x - vi.x) < 1.0e-6 {
                if vj.z > vi.z {
                    core::mem::swap(&mut vj, &mut vi);
                    swapped = true;
                }
            } else if vj.x > vi.x {
                core::mem::swap(&mut vj, &mut vi);
                swapped = true;
            }
            // Create samples along the edge.
            let dij = vi - vj;
            let d = dij.xz().length();
            let mut nn = 1 + floor(d / sample_dist) as usize;
            if nn >= DetailNavmesh::MAX_VERTS_PER_EDGE {
                nn = DetailNavmesh::MAX_VERTS_PER_EDGE - 1;
            }
            if *nverts + nn >= DetailNavmesh::MAX_VERTICES_PER_SUBMESH {
                nn = DetailNavmesh::MAX_VERTICES_PER_SUBMESH - 1 - *nverts;
            }
            for (k, pos) in edge.iter_mut().enumerate().take(nn + 1) {
                let u = k as f32 / nn as f32;
                #[cfg(feature = "fma")]
                {
                    *pos = Vec3A::new(
                        dij.x.mul_add(u, vj.x),
                        dij.y.mul_add(u, vj.y),
                        dij.z.mul_add(u, vj.z),
                    );
                }
                #[cfg(not(feature = "fma"))]
                {
                    *pos = vj + dij * u;
                }
                pos.y = get_height(*pos, ics, chf.cell_height, height_search_radius, hp) as f32
                    * chf.cell_height;
            }
            // Simplify samples.
            let mut idx = [0; DetailNavmesh::MAX_VERTS_PER_EDGE];
            idx[1] = nn;
            let mut nidx = 2;
            let mut k = 0;
            while k < nidx - 1 {
                let a = idx[k];
                let b = idx[k + 1];
                let va = edge[a];
                let vb = edge[b];
                // Find maximum deviation along the segment.
                let mut maxd = 0.0;
                let mut maxi = None;
                let mut m = a + 1;
                while m < b {
                    let dev = distance_squared_between_point_and_line_vec3(edge[m], (va, vb));
                    if dev > maxd {
                        maxd = dev;
                        maxi = Some(m);
                    }
                    m += 1;
                }
                // If the max deviation is larger than accepted error,
                // add new point, else continue to next segment.
                if let Some(maxi) = maxi
                    && maxd > sample_max_error * sample_max_error
                {
                    for m in ((k + 1)..=nidx).rev() {
                        idx[m] = idx[m - 1];
                    }
                    idx[k + 1] = maxi;
                    nidx += 1;
                } else {
                    k += 1;
                }
            }

            hull[nhull] = j;
            nhull += 1;
            // Add new vertices.
            if swapped {
                for k in (1..nidx - 1).rev() {
                    verts[*nverts] = edge[idx[k]];
                    hull[nhull] = *nverts;
                    nhull += 1;
                    *nverts += 1;
                }
            } else {
                for k in 1..nidx - 1 {
                    verts[*nverts] = edge[idx[k]];
                    hull[nhull] = *nverts;
                    nhull += 1;
                    *nverts += 1;
                }
            }
            j = i;
        }
    }

    // If the polygon minimum extent is small (sliver or small triangle), do not try to add internal points.
    if min_extent < sample_dist * 2.0 {
        triangulate_hull(verts, nhull, &hull, nin, tris, flags);
        set_tri_flags(tris, flags, nhull, &hull);
        return Ok(());
    }

    // Tessellate the base mesh.
    // We're using the triangulateHull instead of delaunayHull as it tends to
    // create a bit better triangulation for long thin triangles when there
    // are no internal points.
    triangulate_hull(verts, nhull, &hull, nin, tris, flags);

    if tris.is_empty() {
        let _ = context.log_format(
            LogCategory::WARNING,
            format_args!(
                "buildPolyDetail: Could not triangulate polygon ({} verts).",
                *nverts
            ),
        );
        // Could not triangulate the poly, make sure there is some valid data there.
        #[cfg(feature = "tracing")]
        tracing::warn!("Could not triangulate polygon ({nverts} verts)");
        // Jan: how is this not an Err?
        return Ok(());
    }

    if sample_dist > 0.0 {
        // Create sample locations in a grid.
        let mut aabb = Aabb3d {
            min: in_[0].into(),
            max: in_[0].into(),
        };
        for in_ in in_[..nin].iter().copied() {
            aabb.min = aabb.min.min(in_.into());
            aabb.max = aabb.max.max(in_.into());
        }
        let x0 = floor(aabb.min.x / sample_dist) as i32;
        let x1 = ceil(aabb.max.x / sample_dist) as i32;
        let z0 = floor(aabb.min.z / sample_dist) as i32;
        let z1 = ceil(aabb.max.z / sample_dist) as i32;
        samples.clear();
        for z in z0..z1 {
            for x in x0..x1 {
                let mut pt = Vec3A::default();
                pt.x = x as f32 * sample_dist;
                pt.y = (aabb.max.y + aabb.min.y) * 0.5;
                pt.z = z as f32 * sample_dist;
                // Make sure the samples are not too close to the edges.
                // Jan: I believe this check is bugged, see https://github.com/recastnavigation/recastnavigation/issues/788
                if dist_to_poly(nin, in_, pt) > -sample_dist / 2.0 {
                    continue;
                }
                let y = get_height(pt, ics, chf.cell_height, height_search_radius, hp);
                samples.push((ivec3(x, i32::from(y), z), false));
            }
        }

        // Add the samples starting from the one that has the most
        // error. The procedure stops when all samples are added
        // or when the max error is within treshold.
        for _iter in 0..samples.len() {
            if *nverts >= DetailNavmesh::MAX_VERTICES_PER_SUBMESH {
                break;
            }

            // Find sample with most error.
            let mut bestpt = Vec3A::default();
            let mut bestd = 0.0;
            let mut besti = None;
            for (i, (s, added)) in samples.iter().enumerate() {
                if *added {
                    continue;
                }
                let mut pt = Vec3A::default();
                // The sample location is jittered to get rid of some bad triangulations
                // which are cause by symmetrical data from the grid structure.
                pt.x = native_madd(s.x as f32, sample_dist, get_jitter_x(i) * cs * 0.1);
                pt.y = s.y as f32 * chf.cell_height;
                pt.z = native_madd(s.z as f32, sample_dist, get_jitter_y(i) * cs * 0.1);
                let d = dist_to_tri_mesh(pt, verts, tris);
                let Some(d) = d else {
                    // did not hit the mesh.
                    continue;
                };
                if d > bestd {
                    bestd = d;
                    besti = Some(i);
                    bestpt = pt;
                }
            }
            // If the max error is within accepted threshold, stop tesselating.
            if bestd <= sample_max_error {
                break;
            }
            let Some(besti) = besti else {
                break;
            };
            // Mark sample as added.
            samples[besti].1 = true;
            // Add the new sample point.
            verts[*nverts] = bestpt;
            *nverts += 1;

            // Create new triangulation.
            // [sic] TODO: Incremental add instead of full rebuild.
            triangulation.edges.clear();
            tris.clear();
            flags.clear();
            delaunay_hull(
                context,
                *nverts,
                verts,
                nhull,
                &mut hull,
                tris,
                flags,
                triangulation,
            );
        }
    }
    if tris.len() > DetailNavmesh::MAX_TRIANGLES_PER_SUBMESH {
        let _ = context.log_format(
            LogCategory::ERROR,
            format_args!(
                "rcBuildPolyMeshDetail: Shrinking triangle count from {} to max {}.",
                tris.len(),
                DetailNavmesh::MAX_TRIANGLES_PER_SUBMESH
            ),
        );
        tris.truncate(DetailNavmesh::MAX_TRIANGLES_PER_SUBMESH);
        flags.truncate(DetailNavmesh::MAX_TRIANGLES_PER_SUBMESH);
        #[cfg(feature = "tracing")]
        tracing::error!(
            "Too many triangles! Shringking triangle count from {} to {}",
            tris.len(),
            DetailNavmesh::MAX_TRIANGLES_PER_SUBMESH
        );
    }
    set_tri_flags(tris, flags, nhull, &hull);
    Ok(())
}
#[expect(
    clippy::too_many_arguments,
    reason = "Keeps native inputs and caller-owned scratch explicit"
)]
fn delaunay_hull(
    context: &Context<'_>,
    npts: usize,
    pts: &[Vec3A],
    nhull: usize,
    hull: &mut [usize],
    tris: &mut Vec<[u8; 3]>,
    flags: &mut Vec<u8>,
    triangulation: &mut TriangulationScratch,
) {
    let TriangulationScratch { edges, faces } = triangulation;
    let mut nfaces = 0;
    let mut nedges = 0;
    let max_edges = npts * 10;
    edges.resize(max_edges, Edges::default());

    let mut j = nhull - 1;
    for i in 0..nhull {
        add_edge(
            context,
            edges,
            &mut nedges,
            max_edges,
            hull[j],
            hull[i],
            Edge::HULL,
            Edge::UNDEFINED,
        );
        j = i;
    }

    let mut current_edge = 0;
    while current_edge < nedges {
        if edges[current_edge][2].is_undefined() {
            complete_facet(
                context,
                pts,
                npts,
                edges,
                &mut nedges,
                max_edges,
                &mut nfaces,
                current_edge,
            );
        }
        if edges[current_edge][3].is_undefined() {
            complete_facet(
                context,
                pts,
                npts,
                edges,
                &mut nedges,
                max_edges,
                &mut nfaces,
                current_edge,
            );
        }
        current_edge += 1;
    }

    // Create tris
    faces.clear();
    faces.resize(nfaces, [Edge::UNDEFINED; 3]);

    for e in edges[..nedges].iter() {
        if let Some(e_3) = e[3].index() {
            // Left face
            let t = &mut faces[e_3];
            if t[0].is_undefined() {
                t[0] = e[0];
                t[1] = e[1];
            } else if t[0] == e[1] {
                t[2] = e[0];
            } else if t[1] == e[0] {
                t[2] = e[1];
            }
        }
        if let Some(e_2) = e[2].index() {
            // Right
            let t = &mut faces[e_2];
            if t[0].is_undefined() {
                t[0] = e[1];
                t[1] = e[0];
            } else if t[0] == e[0] {
                t[2] = e[1];
            } else if t[1] == e[1] {
                t[2] = e[0];
            }
        }
    }
    let mut i = 0;
    while i < faces.len() {
        let t = faces[i];
        if t[0].is_undefined() || t[1].is_undefined() || t[2].is_undefined() {
            let _ = context.log_format(
                LogCategory::WARNING,
                format_args!(
                    "delaunayHull: Removing dangling face {i} [{},{},{}].",
                    t[0].raw(),
                    t[1].raw(),
                    t[2].raw()
                ),
            );
            #[cfg(feature = "tracing")]
            tracing::warn!(
                "Removing dangling face {i} [{:?}, {:?}, {:?}]",
                t[0],
                t[1],
                t[2]
            );
            faces.swap_remove(i);
            continue;
        }
        i += 1;
    }
    tris.resize(faces.len(), Default::default());
    flags.resize(faces.len(), Default::default());
    for ((p, d), edge) in (tris.iter_mut().zip(flags.iter_mut())).zip(faces.iter()) {
        p[0] = edge[0].unwrap() as u8;
        p[1] = edge[1].unwrap() as u8;
        p[2] = edge[2].unwrap() as u8;
        // Native initializes the unused fourth face word to -1; the previous
        // typed staging mapped it to zero before set_tri_flags overwrites it.
        *d = 0;
    }
}
#[expect(
    clippy::too_many_arguments,
    reason = "Keeps native inputs and caller-owned scratch explicit"
)]
fn complete_facet(
    context: &Context<'_>,
    pts: &[Vec3A],
    npts: usize,
    edges: &mut [Edges],
    nedges: &mut usize,
    max_edges: usize,
    nfaces: &mut usize,
    e: usize,
) {
    const EPS: f32 = 1.0e-5;

    let edge = edges[e];
    let mut e = Edge::from(e);

    // Cache s and t.
    let (s, t) = if edge[2].is_undefined() {
        (edge[0], edge[1])
    } else if edge[3].is_undefined() {
        (edge[1], edge[0])
    } else {
        // Edge already completed.
        return;
    };

    // Find best point on left of edge.
    let mut pt = npts;
    let mut c = Vec3A::default();
    let mut radius = None;

    // Jan: original implies this:
    let s = s.unwrap();
    let t = t.unwrap();

    for u in 0..npts {
        if u == s || u == t {
            continue;
        }
        if cross2(pts[s].xz(), pts[t].xz(), pts[u].xz()) > EPS {
            let Some(radius) = radius.as_mut() else {
                // The circle is not updated yet, do it now.
                pt = u;
                radius = Some(circum_circle(pts[s], pts[t], pts[u], &mut c));
                continue;
            };
            // Native rounds both square roots before applying its linear
            // tolerance. Squaring the thresholds can select another diagonal.
            let distance = native_distance2(c.xz(), pts[u].xz());
            let tol = 1.0e-3;
            let threshold_out = *radius * (1.0 + tol);
            let threshold_in = *radius * (1.0 - tol);
            if distance > threshold_out {
                // Outside current circumcircle, skip.
                continue;
            } else if distance < threshold_in {
                // Inside safe circumcircle, update circle.
                pt = u;
                *radius = circum_circle(pts[s], pts[t], pts[u], &mut c);
            } else {
                // Inside epsilon circum circle, do extra tests to make sure the edge is valid.
                // s-u and t-u cannot overlap with s-pt nor t-pt if they exists.
                if overlap_edges(pts, edges, *nedges, s, u)
                    || overlap_edges(pts, edges, *nedges, t, u)
                {
                    continue;
                }
                // Edge is valid.
                pt = u;
                *radius = circum_circle(pts[s], pts[t], pts[u], &mut c);
            }
        }
    }

    // Add new triangle or update edge info if s-t is on hull.
    if pt < npts {
        // Update face information of edge being completed.
        update_left_face(&mut edges[e.unwrap()], s, t, *nfaces);

        // Add new edge or update face info of old edge.
        e = find_edge(edges, *nedges, pt, s);
        if e.is_undefined() {
            add_edge(
                context,
                edges,
                nedges,
                max_edges,
                pt,
                s,
                *nfaces,
                Edge::UNDEFINED,
            );
        } else {
            update_left_face(&mut edges[e.unwrap()], pt, s, *nfaces);
        }

        // Add new edge or update face info of old edge.
        e = find_edge(edges, *nedges, t, pt);
        if e.is_undefined() {
            add_edge(
                context,
                edges,
                nedges,
                max_edges,
                t,
                pt,
                *nfaces,
                Edge::UNDEFINED,
            );
        } else {
            update_left_face(&mut edges[e.unwrap()], t, pt, *nfaces);
        }
        *nfaces += 1;
    } else {
        update_left_face(&mut edges[e.unwrap()], s, t, Edge::HULL);
    }
}

fn update_left_face(e: &mut Edges, s: impl Into<Edge>, t: impl Into<Edge>, f: impl Into<Edge>) {
    let s = s.into();
    let t = t.into();
    let f = f.into();
    if e[0] == s && e[1] == t && e[2].is_undefined() {
        e[2] = f;
    } else if e[1] == s && e[0] == t && e[3].is_undefined() {
        e[3] = f;
    }
}

fn overlap_edges(pts: &[Vec3A], edges: &[Edges], nedges: usize, s1: usize, t1: usize) -> bool {
    for edges in edges[..nedges].iter() {
        let s0 = edges[0];
        let t0 = edges[1];
        // Jan: original implies this
        let s0 = s0.unwrap();
        let t0 = t0.unwrap();
        // Same or connected edges do not overlap.
        if s0 == s1 || s0 == t1 || t0 == s1 || t0 == t1 {
            continue;
        }
        if overlap_seg_seg2(pts[s0], pts[t0], pts[s1], pts[t1]) {
            return true;
        }
    }
    false
}

#[inline]
fn overlap_seg_seg2(a: Vec3A, b: Vec3A, c: Vec3A, d: Vec3A) -> bool {
    let a1 = cross2(a.xz(), b.xz(), d.xz());
    let a2 = cross2(a.xz(), b.xz(), c.xz());
    if a1 * a2 < 0.0 {
        let a3 = cross2(c.xz(), d.xz(), a.xz());
        let a4 = a3 + a2 - a1;
        if a3 * a4 < 0.0 {
            return true;
        }
    }
    false
}

fn circum_circle(p1: Vec3A, p2: Vec3A, p3: Vec3A, c: &mut Vec3A) -> f32 {
    const EPS: f32 = 1e-6;
    // Calculate the circle relative to p1, to avoid some precision issues.
    let v1 = Vec2::ZERO;
    let v2 = (p2 - p1).xz();
    let v3 = (p3 - p1).xz();

    let cp = cross2(v1, v2, v3);
    if abs(cp) > EPS {
        let v1_sq = native_dot2(v1, v1);
        let v2_sq = native_dot2(v2, v2);
        let v3_sq = native_dot2(v3, v3);
        // Retain native expression grouping and zero-origin terms: contraction
        // changes the rounded circle center when the products nearly cancel.
        c.x = native_madd(
            v3_sq,
            v1.y - v2.y,
            native_madd(v2_sq, v3.y - v1.y, v1_sq * (v2.y - v3.y)),
        ) / (2.0 * cp);
        c.y = 0.0;
        c.z = native_madd(
            v3_sq,
            v2.x - v1.x,
            native_madd(v2_sq, v1.x - v3.x, v1_sq * (v3.x - v2.x)),
        ) / (2.0 * cp);

        let r = native_distance2(c.xz(), v1);
        *c += p1;
        r
    } else {
        *c = p1;
        0.0
    }
}

#[inline]
fn cross2(p1: Vec2, p2: Vec2, p3: Vec2) -> f32 {
    let a = p2 - p1;
    let b = p3 - p1;
    native_msub(a.x, b.y, a.y * b.x)
}
#[expect(
    clippy::too_many_arguments,
    reason = "Keeps native inputs and caller-owned scratch explicit"
)]
fn add_edge(
    context: &Context<'_>,
    edges: &mut [Edges],
    nedges: &mut usize,
    max_edges: usize,
    s: impl Into<Edge>,
    t: impl Into<Edge>,
    l: impl Into<Edge>,
    r: impl Into<Edge>,
) -> Edge {
    let s = s.into();
    let t = t.into();
    let l = l.into();
    let r = r.into();
    if *nedges >= max_edges {
        let _ = context.log_format(
            LogCategory::ERROR,
            format_args!("addEdge: Too many edges ({nedges}/{max_edges})."),
        );
        #[cfg(feature = "tracing")]
        tracing::error!("Too many edges ({nedges}/{max_edges})");
        return Edge::UNDEFINED;
    }

    // Add edge if not already in the triangulation.
    let e = find_edge(edges, *nedges, s, t);
    if e == Edge::UNDEFINED {
        let edge = &mut edges[*nedges];
        edge[0] = s;
        edge[1] = t;
        edge[2] = l;
        edge[3] = r;
        *nedges += 1;
        Edge::from(*nedges - 1)
    } else {
        Edge::UNDEFINED
    }
}

fn find_edge(edges: &[Edges], nedges: usize, s: impl Into<Edge>, t: impl Into<Edge>) -> Edge {
    let s = s.into();
    let t = t.into();
    for (i, e) in edges.iter().enumerate().take(nedges) {
        if (e[0] == s && e[1] == t) || (e[0] == t && e[1] == s) {
            return Edge::from(i);
        }
    }
    Edge::UNDEFINED
}

struct TriangulationScratch {
    edges: Vec<Edges>,
    faces: Vec<[Edge; 3]>,
}

#[derive(Debug, Default, Copy, Clone, Eq, PartialEq)]
struct Edges([Edge; 4]);
impl Deref for Edges {
    type Target = [Edge; 4];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for Edges {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Default, Copy, Clone, Eq, PartialEq)]
struct Edge(u16);

impl From<usize> for Edge {
    fn from(value: usize) -> Self {
        // Delaunay runs only after adding a point below the 127-vertex cap.
        // At most 1,270 edges and two facet completions per edge keep every
        // point/edge/face index below 2,540, disjoint from both sentinels.
        Self(value as u16)
    }
}

impl Edge {
    const UNDEFINED: Self = Self(u16::MAX);
    const HULL: Self = Self(u16::MAX - 1);

    #[inline]
    fn index(self) -> Option<usize> {
        (self.0 < Self::HULL.0).then_some(usize::from(self.0))
    }

    #[inline]
    fn unwrap(self) -> usize {
        self.index().expect("unwrap called on non-regular edge")
    }

    #[inline]
    fn is_undefined(self) -> bool {
        self == Self::UNDEFINED
    }

    fn raw(self) -> i32 {
        match self {
            Self::UNDEFINED => -1,
            Self::HULL => -2,
            _ => i32::from(self.0),
        }
    }
}

impl core::fmt::Debug for Edge {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.index() {
            Some(index) => f.debug_tuple("Regular").field(&index).finish(),
            None if self.is_undefined() => f.write_str("Undefined"),
            None => f.write_str("Hull"),
        }
    }
}

fn dist_to_tri_mesh(p: Vec3A, verts: &[Vec3A], tris: &[[u8; 3]]) -> Option<f32> {
    let mut dmin = f32::MAX;
    for tri in tris {
        let va = verts[tri[0] as usize];
        let vb = verts[tri[1] as usize];
        let vc = verts[tri[2] as usize];
        let d = dist_pt_tri(p, va, vb, vc);
        if let Some(d) = d
            && d < dmin
        {
            dmin = d;
        }
    }
    if dmin == f32::MAX { None } else { Some(dmin) }
}

/// Distance from point p to triangle defined by vertices a, b, and c.
/// Returns None if the point is outside the triangle.
fn dist_pt_tri(p: Vec3A, a: Vec3A, b: Vec3A, c: Vec3A) -> Option<f32> {
    let v0 = c - a;
    let v1 = b - a;
    let v2 = p - a;

    let dot00 = native_dot2(v0.xz(), v0.xz());
    let dot01 = native_dot2(v0.xz(), v1.xz());
    let dot02 = native_dot2(v0.xz(), v2.xz());
    let dot11 = native_dot2(v1.xz(), v1.xz());
    let dot12 = native_dot2(v1.xz(), v2.xz());

    // Compute barycentric coordinates
    let inv_denom = 1.0 / native_msub(dot00, dot11, dot01 * dot01);
    let u = native_msub(dot11, dot02, dot01 * dot12) * inv_denom;
    let v = native_msub(dot00, dot12, dot01 * dot02) * inv_denom;

    // If point lies inside the triangle, return interpolated y-coord.
    const EPS: f32 = 1.0e-4;
    if u >= -EPS && v >= -EPS && (u + v) <= 1.0 + EPS {
        #[cfg(feature = "fma")]
        let y = v1.y.mul_add(v, v0.y.mul_add(u, a.y));
        #[cfg(not(feature = "fma"))]
        let y = a.y + v0.y * u + v1.y * v;
        Some(abs(y - p.y))
    } else {
        None
    }
}

fn native_distance2(a: Vec2, b: Vec2) -> f32 {
    let d = b - a;
    sqrt(native_dot2(d, d))
}

fn native_dot2(a: Vec2, b: Vec2) -> f32 {
    native_madd(a.x, b.x, a.y * b.y)
}

fn native_msub(a: f32, b: f32, c: f32) -> f32 {
    #[cfg(feature = "fma")]
    {
        a.mul_add(b, -c)
    }
    #[cfg(not(feature = "fma"))]
    {
        a * b - c
    }
}

fn native_madd(a: f32, b: f32, c: f32) -> f32 {
    #[cfg(feature = "fma")]
    {
        a.mul_add(b, c)
    }
    #[cfg(not(feature = "fma"))]
    {
        a * b + c
    }
}
fn get_jitter_x(i: usize) -> f32 {
    native_madd(
        (i.wrapping_mul(0x8da6b343) & 0xffff) as f32 / 65535.0,
        2.,
        -1.,
    )
}

fn get_jitter_y(i: usize) -> f32 {
    native_madd(
        (i.wrapping_mul(0xd8163841) & 0xffff) as f32 / 65535.0,
        2.,
        -1.,
    )
}

#[cfg(feature = "fma")]
#[expect(clippy::manual_clamp, reason = "Match the native branch sequence")]
fn distance_squared_between_point_and_line_vec2(pt: Vec2, (p, q): (Vec2, Vec2)) -> f32 {
    let pq = q - p;
    let dt = pt - p;
    let d = native_madd(pq.x, pq.x, pq.y * pq.y);
    let mut t = native_madd(pq.x, dt.x, pq.y * dt.y);
    if d > 0. {
        t /= d;
    }
    if t < 0. {
        t = 0.;
    } else if t > 1. {
        t = 1.;
    }
    let dx = native_madd(t, pq.x, p.x) - pt.x;
    let dz = native_madd(t, pq.y, p.y) - pt.y;
    native_madd(dx, dx, dz * dz)
}

fn dist_to_poly(nvert: usize, verts: &[Vec3A], p: Vec3A) -> f32 {
    let mut dmin = f32::MAX;
    let mut c = false;
    let mut j = nvert - 1;
    for i in 0..nvert {
        let vi = verts[i];
        let vj = verts[j];
        if (vi.z > p.z) != (vj.z > p.z) && p.x < (vj.x - vi.x) * (p.z - vi.z) / (vj.z - vi.z) + vi.x
        {
            c = !c;
        }
        dmin = dmin.min(distance_squared_between_point_and_line_vec2(
            p.xz(),
            (vj.xz(), vi.xz()),
        ));
        j = i;
    }
    if c { -dmin } else { dmin }
}

/// Find edges that lie on hull and mark them as such.
fn set_tri_flags(tris: &[[u8; 3]], flags: &mut [u8], nhull: usize, hull: &[usize]) {
    // Matches DT_DETAIL_EDGE_BOUNDARY
    const DETAIL_EDGE_BOUNDARY: u8 = 0x1;

    for (tri, tri_flags) in tris.iter().zip(flags.iter_mut()) {
        let mut flags = 0;
        flags |= if on_hull(tri[0] as usize, tri[1] as usize, nhull, hull) {
            DETAIL_EDGE_BOUNDARY
        } else {
            0
        };
        flags |= if on_hull(tri[1] as usize, tri[2] as usize, nhull, hull) {
            DETAIL_EDGE_BOUNDARY
        } else {
            0
        } << 2;
        flags |= if on_hull(tri[2] as usize, tri[0] as usize, nhull, hull) {
            DETAIL_EDGE_BOUNDARY
        } else {
            0
        } << 4;
        *tri_flags = flags;
    }
}

fn on_hull(a: usize, b: usize, nhull: usize, hull: &[usize]) -> bool {
    // All internal sampled points come after the hull so we can early out for those.
    if a >= nhull || b >= nhull {
        return false;
    }
    let mut j = nhull - 1;
    for i in 0..nhull {
        if a == hull[j] && b == hull[i] {
            return true;
        }
        j = i;
    }
    false
}

fn triangulate_hull(
    verts: &[Vec3A],
    nhull: usize,
    hull: &[usize],
    nin: usize,
    tris: &mut Vec<[u8; 3]>,
    flags: &mut Vec<u8>,
) {
    let mut start = 0;
    let mut left = 1;
    let mut right = nhull - 1;

    // Start from an ear with shortest perimeter.
    // This tends to favor well formed triangles as starting point.
    let mut dmin = f32::MAX;
    for i in 0..nhull {
        if hull[i] >= nin {
            // Ears are triangles with original vertices as middle vertex while others are actually line segments on edges
            continue;
        }
        let pi = prev(i, nhull);
        let ni = next(i, nhull);
        let pv = verts[hull[pi]].xz();
        let cv = verts[hull[i]].xz();
        let nv = verts[hull[ni]].xz();
        let d = pv.distance(cv) + cv.distance(nv) + nv.distance(pv);
        if d < dmin {
            start = i;
            left = ni;
            right = pi;
            dmin = d;
        }
    }

    // Add first triangle
    tris.push([hull[start] as u8, hull[left] as u8, hull[right] as u8]);
    flags.push(0);

    // Triangulate the polygon by moving left or right,
    // depending on which triangle has shorter perimeter.
    // This heuristic was chose empirically, since it seems
    // handle tessellated straight edges well.
    while next(left, nhull) != right {
        // Check to see if se should advance left or right.
        let nleft = next(left, nhull);
        let nright = prev(right, nhull);

        let cvleft = verts[hull[left]].xz();
        let nvleft = verts[hull[nleft]].xz();
        let cvright = verts[hull[right]].xz();
        let nvright = verts[hull[nright]].xz();
        let dleft = cvleft.distance(nvleft) + nvleft.distance(cvright);
        let dright = cvright.distance(nvright) + cvleft.distance(nvright);
        if dleft < dright {
            tris.push([hull[left] as u8, hull[nleft] as u8, hull[right] as u8]);
            flags.push(0);
            left = nleft;
        } else {
            tris.push([hull[left] as u8, hull[nright] as u8, hull[right] as u8]);
            flags.push(0);
            right = nright;
        }
    }
}

fn get_height(f: Vec3A, ics: f32, ch: f32, radius: u32, hp: &HeightPatch) -> u16 {
    let mut ix = floor(f.x * ics + 0.01) as i32;
    let mut iz = floor(f.z * ics + 0.01) as i32;
    ix = (ix - hp.xmin).clamp(0, hp.width - 1);
    iz = (iz - hp.zmin).clamp(0, hp.height - 1);
    let mut h = hp.data[(ix + iz * hp.width) as usize];
    if h == RC_UNSET_HEIGHT {
        // Special case when data might be bad.
        // Walk adjacent cells in a spiral up to 'radius', and look
        // for a pixel which has a valid height.
        let mut x = 1;
        let mut z = 0;
        let mut dx = 1;
        let mut dz = 0;
        let max_size = radius * 2 + 1;
        let max_iter = max_size * max_size - 1;

        let mut next_ring_iter_start = 8;
        let mut next_ring_iters = 16;

        let mut dmin = f32::MAX;
        for i in 0..max_iter {
            let nx = ix + x;
            let nz = iz + z;
            if nx >= 0 && nz >= 0 && nx < hp.width && nz < hp.height {
                let nh = hp.data[(nx + nz * hp.width) as usize];
                if nh != RC_UNSET_HEIGHT {
                    let d = abs(nh as f32 * ch - f.y);
                    if d < dmin {
                        h = nh;
                        dmin = d;
                    }
                }
            }
            // We are searching in a grid which looks approximately like this:
            //  __________
            // |2 ______ 2|
            // | |1 __ 1| |
            // | | |__| | |
            // | |______| |
            // |__________|
            // We want to find the best height as close to the center cell as possible. This means that
            // if we find a height in one of the neighbor cells to the center, we don't want to
            // expand further out than the 8 neighbors - we want to limit our search to the closest
            // of these "rings", but the best height in the ring.
            // For example, the center is just 1 cell. We checked that at the entrance to the function.
            // The next "ring" contains 8 cells (marked 1 above). Those are all the neighbors to the center cell.
            // The next one again contains 16 cells (marked 2). In general each ring has 8 additional cells, which
            // can be thought of as adding 2 cells around the "center" of each side when we expand the ring.
            // Here we detect if we are about to enter the next ring, and if we are and we have found
            // a height, we abort the search.
            if i + 1 == next_ring_iter_start {
                if h != RC_UNSET_HEIGHT {
                    break;
                }
                next_ring_iter_start += next_ring_iters;
                next_ring_iters += 8;
            }

            if x == z || (x < 0 && x == -z) || (x > 0 && x == 1 - z) {
                let tmp = dx;
                dx = -dz;
                dz = tmp;
            }
            x += dx;
            z += dz;
        }
    }
    h
}

/// Calculate minimum extent of the polygon.
fn poly_min_extent(verts: &[Vec3A], nverts: usize) -> f32 {
    let mut min_dist = f32::MAX;
    for i in 0..nverts {
        let ni = next(i, nverts);
        let p1 = verts[i];
        let p2 = verts[ni];
        let mut max_edge_dist = 0.0_f32;
        for (j, vert) in verts.iter().enumerate().take(nverts) {
            if j == i || j == ni {
                continue;
            }
            let d = distance_squared_between_point_and_line_vec2(vert.xz(), (p1.xz(), p2.xz()));
            max_edge_dist = max_edge_dist.max(d);
        }
        min_dist = min_dist.min(max_edge_dist);
    }
    // Keep the native square-root rounding before the strict sampling threshold.
    sqrt(min_dist)
}

#[derive(Debug)]
pub enum DetailNavmeshError {
    UninitializedHull,
}

impl core::fmt::Display for DetailNavmeshError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::UninitializedHull => f.write_str("nonempty detail meshes require positive sampling distance; native hull storage is uninitialized otherwise"),
        }
    }
}
impl core::error::Error for DetailNavmeshError {}

#[derive(Debug, Default, Clone, PartialEq, Eq)]
struct HeightPatch {
    data: Vec<u16>,
    xmin: i32,
    zmin: i32,
    width: i32,
    height: i32,
}

impl HeightPatch {
    #[expect(
        clippy::too_many_arguments,
        reason = "Keeps native inputs and caller-owned scratch explicit"
    )]
    fn get_height_data(
        &mut self,
        context: &Context<'_>,
        chf: &CompactHeightfield,
        poly: &[u16],
        npoly: usize,
        verts: &[U16Vec3],
        bs: i32,
        queue: &mut Vec<(i32, i32, usize)>,
        region: RegionId,
    ) {
        // Note: Reads to the compact heightfield are offset by border size (bs)
        // since border size offset is already removed from the polymesh vertices.
        queue.clear();
        let data_len = self.data_len();
        // Set all heights to RC_UNSET_HEIGHT.
        self.data[..data_len].fill(0xffff);

        let mut empty = true;

        // We cannot sample from this poly if it was created from polys
        // of different regions. If it was then it could potentially be overlapping
        // with polys of that region and the heights sampled here could be wrong.
        if region != RegionId::NONE {
            // Copy the height from the same region, and mark region borders
            // as seed points to fill the rest.
            for hz in 0..self.height {
                let z = self.zmin + hz + bs;
                for hx in 0..self.width {
                    let x = self.xmin + hx + bs;
                    let c = &chf.cells[x as usize + z as usize * chf.width as usize];
                    for i in c.index_range() {
                        let s = &chf.spans[i];
                        if s.region == region {
                            // Store height
                            *self.data_at_mut(hx, hz) = s.y;
                            empty = false;

                            // If any of the neighbours is not in same region,
                            // add the current location as flood fill start
                            let mut border = false;
                            for dir in 0..4 {
                                if let Some(con) = s.con(dir) {
                                    let (_ax, _az, ai) = chf.con_indices(x, z, dir, con);
                                    let as_ = &chf.spans[ai];
                                    if as_.region != region {
                                        border = true;
                                        break;
                                    }
                                }
                            }
                            if border {
                                queue.push((x, z, i));
                            }
                            break;
                        }
                    }
                }
            }
        }
        // if the polygon does not contain any points from the current region (rare, but happens)
        // or if it could potentially be overlapping polygons of the same region,
        // then use the center as the seed point.
        if empty {
            self.seed_array_with_poly_center(context, chf, poly, npoly, verts, bs, queue);
        }
        const RETRACT_SIZE: usize = 256;
        let mut head = 0;

        // We assume the seed is centered in the polygon, so a BFS to collect
        // height data will ensure we do not move onto overlapping polygons and
        // sample wrong heights.
        while head < queue.len() {
            let (cx, cz, ci) = queue[head];
            head += 1;
            if head >= RETRACT_SIZE {
                head = 0;
                if queue.len() > RETRACT_SIZE {
                    queue.copy_within(RETRACT_SIZE.., 0);
                }
                queue.truncate(queue.len() - RETRACT_SIZE);
            }
            let cs = &chf.spans[ci];
            for dir in 0..4 {
                let Some(con) = cs.con(dir) else {
                    continue;
                };
                let ax = cx + dir_offset_x(dir) as i32;
                let az = cz + dir_offset_z(dir) as i32;
                let hx = ax - self.xmin - bs;
                let hz = az - self.zmin - bs;

                if hx < 0 || hz < 0 || hx >= self.width || hz >= self.height {
                    continue;
                }

                if *self.data_at(hx, hz) != RC_UNSET_HEIGHT {
                    continue;
                }
                let ai = chf.cells[(ax + az * chf.width) as usize].index() as usize + con as usize;
                let as_ = &chf.spans[ai];

                *self.data_at_mut(hx, hz) = as_.y;
                queue.push((ax, az, ai));
            }
        }
    }
    #[expect(
        clippy::too_many_arguments,
        reason = "Keeps native inputs and caller-owned scratch explicit"
    )]
    fn seed_array_with_poly_center(
        &mut self,
        context: &Context<'_>,
        chf: &CompactHeightfield,
        poly: &[u16],
        npoly: usize,
        verts: &[U16Vec3],
        bs: i32,
        array: &mut Vec<(i32, i32, usize)>,
    ) {
        // Note: Reads to the compact heightfield are offset by border size (bs)
        // since border size offset is already removed from the polymesh vertices.
        const OFFSET: [i32; 9 * 2] = [0, 0, -1, -1, 0, -1, 1, -1, 1, 0, 1, 1, 0, 1, -1, 1, -1, 0];

        // Find cell closest to a poly vertex
        let mut start_cell_x = 0;
        let mut start_cell_z = 0;
        let mut start_span_index = None;
        let mut dmin = RC_UNSET_HEIGHT as i32;
        for poly_j in poly[..npoly].iter().map(|p| *p as usize) {
            if dmin <= 0 {
                break;
            }
            for k in 0..9 {
                if dmin <= 0 {
                    break;
                }
                let ax = verts[poly_j].x as i32 + OFFSET[k * 2];
                let ay = verts[poly_j].y as i32;
                let az = verts[poly_j].z as i32 + OFFSET[k * 2 + 1];
                if ax < self.xmin
                    || ax >= self.xmin + self.width
                    || az < self.zmin
                    || az >= self.zmin + self.height
                {
                    continue;
                };
                let c = &chf.cells[((ax + bs) + (az + bs) * chf.width) as usize];
                for i in c.index_range() {
                    if dmin == 0 {
                        break;
                    }
                    let s = &chf.spans[i];
                    let d = (ay - s.y as i32).abs();
                    if d < dmin {
                        start_cell_x = ax;
                        start_cell_z = az;
                        start_span_index = Some(i);
                        dmin = d;
                    }
                }
            }
        }

        // Jan: Original code also asserts this.
        let start_span_index = start_span_index.expect("Internal error: found no start span");
        // Find center of the polygon
        let mut pcx = 0;
        let mut pcz = 0;
        for poly_j in poly[..npoly].iter().map(|p| *p as usize) {
            // Jan: shouldn't the type conversion happen only at the final value?
            pcx += verts[poly_j].x as i32;
            pcz += verts[poly_j].z as i32;
        }
        pcx /= npoly as i32;
        pcz /= npoly as i32;

        // Use seeds array as a stack for DFS
        array.clear();
        array.push((start_cell_x, start_cell_z, start_span_index));

        let mut dirs = [0, 1, 2, 3];
        let data_len = self.data_len();
        self.data[..data_len].fill(0);
        // DFS to move to the center. Note that we need a DFS here and can not just move
        // directly towards the center without recording intermediate nodes, even though the polygons
        // are convex. In very rare we can get stuck due to contour simplification if we do not
        // record nodes.
        let mut cx = None;
        let mut cz = None;
        let mut ci = None;
        loop {
            if array.is_empty() {
                context.log(
                    LogCategory::WARNING,
                    "Walk towards polygon center failed to reach center",
                );
                #[cfg(feature = "tracing")]
                tracing::warn!("Walk towards polygon center failed to reach center");
                break;
            }

            let (cx_raw, cz_raw, ci_raw) = array.pop().unwrap();
            cx = Some(cx_raw);
            cz = Some(cz_raw);
            ci = Some(ci_raw);
            let cx = cx.unwrap();
            let cz = cz.unwrap();
            let ci = ci.unwrap();

            if cx == pcx && cz == pcz {
                break;
            }

            // If we are already at the correct X-position, prefer direction
            // directly towards the center in the Y-axis; otherwise prefer
            // direction in the X-axis
            let direct_dir = if cx == pcx {
                dir_offset(0, if pcz > cz { 1 } else { -1 })
            } else {
                dir_offset(if pcx > cx { 1 } else { -1 }, 0)
            } as usize;

            // Push the direct dir last so we start with this on next iteration
            dirs.swap(direct_dir, 3);

            let cs = &chf.spans[ci];
            for dir in dirs {
                let Some(con) = cs.con(dir) else {
                    continue;
                };

                let new_x = cx + dir_offset_x(dir) as i32;
                let new_z = cz + dir_offset_z(dir) as i32;

                let hpx = new_x - self.xmin;
                let hpz = new_z - self.zmin;
                if hpx < 0 || hpx >= self.width || hpz < 0 || hpz >= self.height {
                    continue;
                }
                if *self.data_at(hpx, hpz) != 0 {
                    continue;
                }
                *self.data_at_mut(hpx, hpz) = 1;
                let new_index = chf.cells[((new_x + bs) + (new_z + bs) * chf.width) as usize]
                    .index() as i32
                    + con as i32;
                array.push((new_x, new_z, new_index as usize));
            }
            dirs.swap(direct_dir, 3);
        }

        array.clear();
        // getHeightData seeds are given in coordinates with borders
        let (Some(cx), Some(cz), Some(ci)) = (cx, cz, ci) else {
            // Jan: We panic earlier in the loop before this could even happen.
            unreachable!()
        };
        array.push((cx + bs, cz + bs, ci));
        self.data[..data_len].fill(0xffff);
        let cs = &chf.spans[ci];
        self.data[(cx - self.xmin + (cz - self.zmin) * self.width) as usize] = cs.y;
    }

    #[inline]
    fn data_len(&self) -> usize {
        self.width as usize * self.height as usize
    }

    #[inline]
    fn data_at(&self, x: i32, z: i32) -> &u16 {
        &self.data[(x + z * self.width) as usize]
    }

    #[inline]
    fn data_at_mut(&mut self, x: i32, z: i32) -> &mut u16 {
        &mut self.data[(x + z * self.width) as usize]
    }
}

const RC_UNSET_HEIGHT: u16 = 0xffff;

#[derive(Debug, Default, Clone, PartialEq, Eq)]
struct Bounds {
    xmin: i32,
    xmax: i32,
    zmin: i32,
    zmax: i32,
}
impl Bounds {
    #[inline]
    fn width(&self) -> i32 {
        self.xmax - self.xmin
    }

    #[inline]
    fn height(&self) -> i32 {
        self.zmax - self.zmin
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn native_circumcircle_tolerance_boundary_changes_diagonal() {
        // Exact native circle center/radius bits also exercise translated
        // coordinates and contraction-sensitive scalar expressions.
        let inputs = [
            [[0.1, 0.2, 0.3], [2.3, -1.1, 0.7], [-0.4, 2.6, 3.9]],
            [[-2.71, 1.3, 4.8], [0.38, 0.2, -3.1], [5.7, 1.9, 2.6]],
        ];
        #[cfg(not(feature = "fma"))]
        let expected = [
            [1063390363, 1045220557, 1074763023, 1074141442],
            [1064810844, 1067869798, 1071092472, 1083850369],
        ];
        #[cfg(feature = "fma")]
        let expected = [
            [1063390363, 1045220557, 1074763023, 1074141442],
            [1064810848, 1067869798, 1071092472, 1083850370],
        ];
        for (points, expected) in inputs.into_iter().zip(expected) {
            let points = points.map(Vec3A::from_array);
            let mut center = Vec3A::ZERO;
            let radius = circum_circle(points[0], points[1], points[2], &mut center);
            assert_eq!(
                [
                    center.x.to_bits(),
                    center.y.to_bits(),
                    center.z.to_bits(),
                    radius.to_bits()
                ],
                expected,
                "circle points={points:?}"
            );
        }
        // Pinned Recast delaunayHull results, identical with contraction on/off.
        // The middle point is exactly on the rounded radius * 1.001 boundary;
        // comparing squared distances instead incorrectly chooses the diagonal
        // from vertex 1 to vertex 2. Adjacent floats bracket the transition.
        for (z_bits, expected) in [
            (0x4000_3116, [[1, 0, 3], [0, 2, 3]]),
            (0x4000_3117, [[1, 0, 3], [0, 2, 3]]),
            (0x4000_3118, [[1, 0, 2], [2, 3, 1]]),
        ] {
            let points = [
                Vec3A::ZERO,
                Vec3A::new(2., 0., 0.),
                Vec3A::new(0., 0., 2.),
                Vec3A::new(f32::from_bits(0x3a83_126f), 0., f32::from_bits(z_bits)),
            ];
            let mut triangles = Vec::new();
            let mut flags = Vec::new();
            delaunay_hull(
                &Context::new(false),
                points.len(),
                &points,
                4,
                &mut [0, 2, 3, 1],
                &mut triangles,
                &mut flags,
                &mut TriangulationScratch {
                    edges: Vec::new(),
                    faces: Vec::new(),
                },
            );
            assert_eq!(triangles, expected, "z bits={z_bits:#x}");
        }
    }
}
