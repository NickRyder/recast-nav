#![doc = include_str!("../../../README.md")]
#![forbid(unsafe_code)]

use context::Context;
use glam::Vec3;

extern crate alloc;

mod compact_cell;
mod compact_heightfield;
mod compact_span;
mod config;
pub mod context;
mod contour_holes;
mod contours;
mod detail_mesh;
mod erosion;
mod flags;
mod heightfield;
mod mark_convex_poly_area;
pub(crate) mod math;
#[doc(hidden)]
pub mod native_sort;
pub(crate) mod ops;
mod poly_mesh;
mod pre_filter;
mod rasterize;
mod region;
mod span;
mod trimesh;
mod watershed_build_regions;
mod watershed_distance_field;

pub use compact_cell::CompactCell;
pub use compact_heightfield::{CompactHeightfield, CompactHeightfieldError};
pub use compact_span::CompactSpan;
pub use config::{Config, ConfigBuilder};
pub use contours::{BuildContoursFlags, Contour, ContourError, ContourSet, RegionVertexId};
pub use detail_mesh::{DetailNavmesh, DetailNavmeshError, SubMesh};
pub use erosion::ErosionError;
pub use heightfield::{
    Heightfield, HeightfieldBuilder, HeightfieldBuilderError, SpanInsertion, SpanInsertionError,
};
pub use mark_convex_poly_area::ConvexVolume;
pub use math::{Aabb2d, Aabb3d};
pub use poly_mesh::{
    PolygonNavmesh, PolygonNavmeshError, TileCacheContourView, TileCachePolygonMesh,
};
pub use pre_filter::FilterError;
pub use rasterize::RasterizationError;
pub use region::RegionId;
pub use span::{AreaType, Span, SpanKey, Spans};
pub use trimesh::TriMesh;
pub use watershed_build_regions::BuildRegionsError;

pub mod cache;
pub mod cache_data;
pub mod common;
pub mod crowd;
pub mod layers;
pub mod nav;
pub mod nav_data;
pub mod nodes;
mod poly_buffer;
pub mod query;
pub mod recast;
pub mod runtime;
pub mod tile_cache;

/// Indexed triangle geometry in a right-handed, Y-up coordinate system.
#[derive(Clone, Debug)]
pub struct TriangleMesh {
    pub vertices: Vec<[f32; 3]>,
    pub triangles: Vec<[u32; 3]>,
}

/// Region partitioning strategy used by the standard generation pipeline.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum RegionPartitioning {
    #[default]
    Watershed,
    Monotone,
    Layers,
}

/// Pipeline parameters in explicit voxel units. See [`recast::Config`] for the
/// complete native parameter aggregation, including explicit grid/tile sizes.
/// `bounds` includes padding; `border_size` removes that padding during region building.
#[derive(Clone, Debug)]
pub struct BuildConfig {
    pub bounds: [[f32; 3]; 2],
    pub cell_size: f32,
    pub cell_height: f32,
    pub slope_degrees: f32,
    pub walkable_height: i32,
    pub walkable_climb: i32,
    pub walkable_radius: i32,
    pub border_size: i32,
    pub partitioning: RegionPartitioning,
    pub min_region_area: i32,
    pub merge_region_area: i32,
    pub max_edge_len: i32,
    pub contour_flags: BuildContoursFlags,
    pub max_simplification_error: f32,
    pub max_vertices_per_polygon: u32,
    pub detail_sample_dist: f32,
    pub detail_sample_max_error: f32,
    /// Allocation limit for the padded grid. Does not change the build algorithm.
    pub max_grid_cells: usize,
}

impl Default for BuildConfig {
    fn default() -> Self {
        Self {
            bounds: [[0., -1., 0.], [16., 8., 16.]],
            cell_size: 0.5,
            cell_height: 0.25,
            slope_degrees: 45.,
            walkable_height: 8,
            walkable_climb: 3,
            walkable_radius: 1,
            border_size: 0,
            partitioning: RegionPartitioning::Watershed,
            min_region_area: 0,
            merge_region_area: 0,
            max_edge_len: 12,
            contour_flags: BuildContoursFlags::default(),
            max_simplification_error: 1.3,
            max_vertices_per_polygon: 6,
            detail_sample_dist: 3.,
            detail_sample_max_error: 0.25,
            max_grid_cells: 4_194_304,
        }
    }
}

#[derive(Debug)]
pub struct BuildError(pub String);

impl std::fmt::Display for BuildError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.0.fmt(f)
    }
}
impl std::error::Error for BuildError {}

/// Polygon, detail, and layered output for one generation tile.
/// Use [`runtime::Navigation`] to assemble tiled runtime navigation.
#[derive(Debug)]
pub struct BuiltTile {
    pub polygons: PolygonNavmesh,
    pub detail: DetailNavmesh,
    pub layers: Vec<layers::HeightfieldLayer>,
}

/// Borrowed stage data for diagnostics and differential testing.
pub enum BuildStage<'a> {
    TriangleAreas(&'a [AreaType]),
    Heightfield(&'static str, &'a crate::Heightfield),
    Compact(&'static str, &'a crate::CompactHeightfield),
    Contours(&'a crate::ContourSet),
    Polygons(&'a PolygonNavmesh),
    Detail(&'a DetailNavmesh),
    Layers(&'a [layers::HeightfieldLayer]),
}

impl BuildConfig {
    /// Validate before allocating or entering the low-level port.
    pub fn validate(&self) -> Result<(), BuildError> {
        let bad = |message: &str| Err(BuildError(message.into()));
        if !self.bounds.iter().flatten().all(|v| v.is_finite())
            || (0..3).any(|i| self.bounds[0][i] >= self.bounds[1][i])
        {
            return bad("bounds must be finite with positive extent on each axis");
        }
        if !self.cell_size.is_finite()
            || self.cell_size <= 0.
            || !self.cell_height.is_finite()
            || self.cell_height <= 0.
        {
            return bad("cell dimensions must be finite and positive");
        }
        let dimensions = [0, 2]
            .map(|i| ((self.bounds[1][i] - self.bounds[0][i]) / self.cell_size + 0.5).floor());
        if self.border_size < 0
            || dimensions.iter().any(|&v| {
                !v.is_finite()
                    || v <= self.border_size as f32 * 2.
                    || f64::from(v) >= f64::from(i32::MAX) + 1.
            })
            || dimensions[0] as u64 * dimensions[1] as u64
                > self.max_grid_cells.min(i32::MAX as usize) as u64
        {
            return bad(
                "grid dimensions exceed the signed integer/allocation domain or leave no interior after padding",
            );
        }
        if !(0. ..90.).contains(&self.slope_degrees)
            || self.walkable_radius.checked_mul(2).is_none()
            || !(3..=i32::MAX as u32).contains(&self.max_vertices_per_polygon)
        {
            return bad("agent or polygon parameters are outside the supported Recast range");
        }
        if [
            self.max_simplification_error,
            self.detail_sample_dist,
            self.detail_sample_max_error,
        ]
        .iter()
        .any(|v| !v.is_finite() || *v < 0.)
            || (self.detail_sample_dist > 0. && self.detail_sample_dist < 0.9)
        {
            return bad("invalid contour or detail sampling parameters");
        }
        Ok(())
    }
}

/// Build one tile with the selected Recast partitioning strategy (watershed by default).
pub fn build_tile(mesh: &TriangleMesh, config: &BuildConfig) -> Result<BuiltTile, BuildError> {
    build_tile_observed(mesh, config, |_| {})
}

/// Like [`build_tile`], with observations of every implemented pipeline stage.
pub fn build_tile_observed(
    mesh: &TriangleMesh,
    config: &BuildConfig,
    observe: impl FnMut(BuildStage<'_>),
) -> Result<BuiltTile, BuildError> {
    build_tile_observed_with_context(&Context::new(false), mesh, config, observe)
}

/// Build a tile using a caller-owned Recast diagnostic context.
pub fn build_tile_with_context(
    context: &Context<'_>,
    mesh: &TriangleMesh,
    config: &BuildConfig,
) -> Result<BuiltTile, BuildError> {
    build_tile_observed_with_context(context, mesh, config, |_| {})
}

/// Observe every stage while preserving each underlying native context scope.
pub fn build_tile_observed_with_context(
    context: &Context<'_>,
    mesh: &TriangleMesh,
    config: &BuildConfig,
    mut observe: impl FnMut(BuildStage<'_>),
) -> Result<BuiltTile, BuildError> {
    let mut compact = build_compact_observed(context, mesh, config, &mut observe)?;
    let layers = layers::build_heightfield_layers_with_context(
        context,
        &compact,
        config.border_size,
        config.walkable_height,
    )?;
    observe(BuildStage::Layers(&layers));
    compact.build_distance_field_with_context(context);
    observe(BuildStage::Compact("distance", &compact));
    match config.partitioning {
        RegionPartitioning::Watershed => compact.build_regions_with_context(
            context,
            config.border_size,
            config.min_region_area,
            config.merge_region_area,
        ),
        RegionPartitioning::Monotone => compact.build_regions_monotone_with_context(
            context,
            config.border_size,
            config.min_region_area,
            config.merge_region_area,
        ),
        RegionPartitioning::Layers => compact.build_layer_regions_with_context(
            context,
            config.border_size,
            config.min_region_area,
        ),
    }
    .map_err(error)?;
    observe(BuildStage::Compact("regions", &compact));
    let contours = compact
        .build_contours_with_context(
            context,
            config.max_simplification_error,
            config.max_edge_len,
            config.contour_flags,
        )
        .map_err(error)?;
    observe(BuildStage::Contours(&contours));
    let polygons = contours
        .into_polygon_mesh_with_context(context, config.max_vertices_per_polygon)
        .map_err(error)?;
    observe(BuildStage::Polygons(&polygons));
    let detail = DetailNavmesh::new_with_context(
        context,
        &polygons,
        &compact,
        config.detail_sample_dist,
        config.detail_sample_max_error,
    )
    .map_err(error)?;
    observe(BuildStage::Detail(&detail));
    Ok(BuiltTile {
        polygons,
        detail,
        layers,
    })
}

fn error(error: impl std::fmt::Display) -> BuildError {
    BuildError(error.to_string())
}

fn build_compact_observed(
    context: &Context<'_>,
    mesh: &TriangleMesh,
    config: &BuildConfig,
    observe: &mut impl FnMut(BuildStage<'_>),
) -> Result<crate::CompactHeightfield, BuildError> {
    config.validate()?;
    validate_mesh(mesh)?;
    let mut areas = vec![AreaType::NOT_WALKABLE; mesh.triangles.len()];
    recast::mark_walkable_triangles(
        config.slope_degrees,
        &mesh.vertices,
        &mesh.triangles,
        &mut areas,
    );
    observe(BuildStage::TriangleAreas(&areas));
    let mut solid = HeightfieldBuilder {
        aabb: Aabb3d {
            min: Vec3::from_array(config.bounds[0]),
            max: Vec3::from_array(config.bounds[1]),
        },
        cell_size: config.cell_size,
        cell_height: config.cell_height,
    }
    .build()
    .map_err(error)?;
    solid
        .rasterize_indexed_triangles_with_context(
            context,
            &mesh.vertices,
            &mesh.triangles,
            &areas,
            config.walkable_climb,
        )
        .map_err(error)?;
    observe(BuildStage::Heightfield("rasterized", &solid));
    solid.filter_low_hanging_walkable_obstacles_with_context(context, config.walkable_climb);
    observe(BuildStage::Heightfield("low_hanging", &solid));
    solid
        .filter_ledge_spans_with_context(context, config.walkable_height, config.walkable_climb)
        .map_err(error)?;
    observe(BuildStage::Heightfield("ledges", &solid));
    solid.filter_walkable_low_height_spans_with_context(context, config.walkable_height);
    observe(BuildStage::Heightfield("low_height", &solid));
    let mut compact = solid
        .into_compact_with_context(context, config.walkable_height, config.walkable_climb)
        .map_err(error)?;
    observe(BuildStage::Compact("compact", &compact));
    compact
        .erode_walkable_area_with_context(context, config.walkable_radius)
        .map_err(error)?;
    observe(BuildStage::Compact("eroded", &compact));
    Ok(compact)
}

// Shared validation runs before tiled selection, so invalid geometry outside the
// navigation bounds is rejected just like geometry inside them.
fn validate_mesh(mesh: &TriangleMesh) -> Result<(), BuildError> {
    if mesh.vertices.is_empty()
        || mesh.triangles.is_empty()
        || !mesh.vertices.iter().flatten().all(|v| v.is_finite())
        || mesh
            .triangles
            .iter()
            .flatten()
            .any(|&i| i as usize >= mesh.vertices.len())
        || mesh.vertices.len() > i32::MAX as usize
        || mesh.triangles.len() > i32::MAX as usize
    {
        return Err(BuildError(
            "mesh must contain finite vertices and valid triangle indices".into(),
        ));
    }
    Ok(())
}

pub fn build_layers(
    mesh: &TriangleMesh,
    config: &BuildConfig,
) -> Result<Vec<layers::HeightfieldLayer>, BuildError> {
    build_layers_with_context(&Context::new(false), mesh, config)
}

/// Build heightfield layers using a caller-owned Recast diagnostic context.
pub fn build_layers_with_context(
    context: &Context<'_>,
    mesh: &TriangleMesh,
    config: &BuildConfig,
) -> Result<Vec<layers::HeightfieldLayer>, BuildError> {
    let compact = build_compact_observed(context, mesh, config, &mut |_| {})?;
    layers::build_heightfield_layers_with_context(
        context,
        &compact,
        config.border_size,
        config.walkable_height,
    )
}
