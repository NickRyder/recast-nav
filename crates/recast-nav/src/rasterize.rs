//! Contains methods for rasterizing triangles of a [`TrimeshedCollider`] into a [`Heightfield`].

use crate::ops::*;
use glam::Vec3A;

use crate::{
    TriMesh,
    heightfield::{Heightfield, SpanInsertion, SpanInsertionError},
    math::TriangleVertices as _,
    span::{AreaType, Span, SpanBuilder},
};

// Native rcClamp permits low > high; its ordered branches are observable at
// the packed solid-span ceiling and in empty grids.
fn native_clamp(value: i32, low: i32, high: i32) -> i32 {
    if value < low {
        low
    } else if value > high {
        high
    } else {
        value
    }
}
fn native_int(value: f32) -> Result<i32, RasterizationError> {
    if value.is_finite()
        && f64::from(value) >= f64::from(i32::MIN)
        && f64::from(value) < f64::from(i32::MAX) + 1.
    {
        Ok(value as i32)
    } else {
        Err(RasterizationError::IntegerConversion)
    }
}

impl Heightfield {
    /// Indexed mesh rasterization with one native timer for the complete batch.
    pub fn rasterize_triangles_with_context(
        &mut self,
        context: &crate::context::Context<'_>,
        trimesh: &TriMesh,
        walkable_climb: i32,
    ) -> Result<(), RasterizationError> {
        let _timer = context.scoped_timer(crate::context::TimerLabel::RASTERIZE_TRIANGLES);
        self.rasterize_triangles(trimesh, walkable_climb)
    }

    pub fn rasterize_triangle_with_context(
        &mut self,
        context: &crate::context::Context<'_>,
        triangle: [Vec3A; 3],
        area_type: AreaType,
        walkable_climb: i32,
    ) -> Result<(), RasterizationError> {
        let _timer = context.scoped_timer(crate::context::TimerLabel::RASTERIZE_TRIANGLES);
        self.rasterize_triangle(triangle, area_type, walkable_climb)
    }

    /// Rasterizes the triangles of a [`TriMesh`] into a [`Heightfield`].
    pub fn rasterize_triangles(
        &mut self,
        trimesh: &TriMesh,
        walkable_climb: i32,
    ) -> Result<(), RasterizationError> {
        self.rasterize_indexed_triangles(
            &trimesh.vertices,
            &trimesh.indices,
            &trimesh.area_types,
            walkable_climb,
        )
    }

    /// Rasterizes borrowed indexed geometry, loading aligned or packed vertices
    /// only for the current triangle.
    pub fn rasterize_indexed_triangles<I, V>(
        &mut self,
        vertices: &[V],
        indices: &[I],
        areas: &[AreaType],
        walkable_climb: i32,
    ) -> Result<(), RasterizationError>
    where
        I: core::ops::Index<usize, Output = u32>,
        V: Copy + Into<Vec3A>,
    {
        for (i, triangle) in indices.iter().enumerate() {
            let triangle = [
                vertices[triangle[0] as usize].into(),
                vertices[triangle[1] as usize].into(),
                vertices[triangle[2] as usize].into(),
            ];
            self.rasterize_triangle(triangle, areas[i], walkable_climb)?;
        }
        Ok(())
    }

    /// One native batch timer around borrowed indexed geometry.
    pub fn rasterize_indexed_triangles_with_context<I, V>(
        &mut self,
        context: &crate::context::Context<'_>,
        vertices: &[V],
        indices: &[I],
        areas: &[AreaType],
        walkable_climb: i32,
    ) -> Result<(), RasterizationError>
    where
        I: core::ops::Index<usize, Output = u32>,
        V: Copy + Into<Vec3A>,
    {
        let _timer = context.scoped_timer(crate::context::TimerLabel::RASTERIZE_TRIANGLES);
        self.rasterize_indexed_triangles(vertices, indices, areas, walkable_climb)
    }

    /// Rasterizes a triangle into a [`Heightfield`].
    #[inline(always)]
    pub fn rasterize_triangle(
        &mut self,
        triangle: [Vec3A; 3],
        area_type: AreaType,
        flag_merge_threshold: i32,
    ) -> Result<(), RasterizationError> {
        let aabb = triangle.aabb();
        // If the triangle does not touch the bounding box of the heightfield, skip the triangle.
        if !self.aabb.intersects(&aabb) {
            return Ok(());
        }

        let inverse_cell_size = 1.0 / self.cell_size;
        let inverse_cell_height = 1.0 / self.cell_height;

        let w = self.width;
        let h = self.height;
        // The height of the heightfield AABB
        let by = self.aabb.max[1] - self.aabb.min[1];

        // Calculate the footprint of the triangle on the grid's z-axis
        // z0 is the first z cell that the triangle touches
        // z1 is the last z cell that the triangle touches
        let z0 = native_int((aabb.min[2] - self.aabb.min[2]) * inverse_cell_size)?;
        let z1 = native_int((aabb.max[2] - self.aabb.min[2]) * inverse_cell_size)?;

        // use -1 rather than 0 to cut the polygon properly at the start of the tile
        let z0 = native_clamp(z0, -1, h - 1);
        let z1 = native_clamp(z1, 0, h - 1);

        // Clip the triangle into all grid cells it touches.
        const MAX_VERTICES_AFTER_CLIPPING: usize = 7;
        // x-min, x-max, z-min, z-max
        const CLIP_DIRS: usize = 4;
        const BUF_LEN: usize = MAX_VERTICES_AFTER_CLIPPING * CLIP_DIRS;

        let mut buf = [Vec3A::ZERO; BUF_LEN];

        let (mut in_tri, rest) = buf.split_at_mut(MAX_VERTICES_AFTER_CLIPPING);
        let (mut in_row, rest) = rest.split_at_mut(MAX_VERTICES_AFTER_CLIPPING);
        let (mut p1, mut p2) = rest.split_at_mut(MAX_VERTICES_AFTER_CLIPPING);

        in_tri[0] = triangle[0];
        in_tri[1] = triangle[1];
        in_tri[2] = triangle[2];

        let mut nv_row = 0_u8;
        let mut nv_in = 3_u8;

        for z in z0..=z1 {
            // Clip polygon to row. Store the remaining polygon as well
            #[cfg(feature = "fma")]
            let cell_z = (z as f32).mul_add(self.cell_size, self.aabb.min[2]);
            #[cfg(not(feature = "fma"))]
            let cell_z = self.aabb.min[2] + z as f32 * self.cell_size;
            divide_poly::<2>(
                in_tri,
                nv_in,
                in_row,
                &mut nv_row,
                p1,
                &mut nv_in,
                cell_z + self.cell_size,
            )?;
            core::mem::swap(&mut in_tri, &mut p1);

            if nv_row < 3 || z < 0 {
                continue;
            }

            // find X-axis bounds of the row
            let mut min_x = in_row[0].x;
            let mut max_x = in_row[0].x;
            for i in 1..nv_row {
                min_x = min_x.min(in_row[i as usize].x);
                max_x = max_x.max(in_row[i as usize].x);
            }
            let x0 = native_int((min_x - self.aabb.min[0]) * inverse_cell_size)?;
            let x1 = native_int((max_x - self.aabb.min[0]) * inverse_cell_size)?;
            if x1 < 0 || x0 >= w {
                continue;
            }
            let x0 = native_clamp(x0, -1, w - 1);
            let x1 = native_clamp(x1, 0, w - 1);

            let mut nv = 0_u8;
            let mut nv2 = nv_row;
            for x in x0..=x1 {
                // Clip polygon to column. store the remaining polygon as well
                #[cfg(feature = "fma")]
                let cx = (x as f32).mul_add(self.cell_size, self.aabb.min[0]);
                #[cfg(not(feature = "fma"))]
                let cx = self.aabb.min[0] + x as f32 * self.cell_size;
                divide_poly::<0>(in_row, nv2, p1, &mut nv, p2, &mut nv2, cx + self.cell_size)?;
                core::mem::swap(&mut in_row, &mut p2);

                if nv < 3 || x < 0 {
                    continue;
                }

                // Calculate min and max of the span.
                let mut span_min = p1[0].y;
                let mut span_max = span_min;
                for i in 1..nv {
                    let y = p1[i as usize].y;
                    span_min = span_min.min(y);
                    span_max = span_max.max(y);
                }
                span_min -= self.aabb.min[1];
                span_max -= self.aabb.min[1];
                // Skip the span if it's completely outside the heightfield bounding box
                if span_max < 0.0 || span_min > by {
                    continue;
                }

                // Clamp the span to the heightfield bounding box.
                span_min = span_min.max(0.0);
                span_max = span_max.min(by);

                // Snap the span to the heightfield height grid.
                let span_min_cell_index = native_clamp(
                    native_int(floor(span_min * inverse_cell_height))?,
                    0,
                    i32::from(Span::MAX_HEIGHT),
                ) as u16;
                let span_max_cell_index = native_clamp(
                    native_int(ceil(span_max * inverse_cell_height))?,
                    i32::from(span_min_cell_index) + 1,
                    i32::from(Span::MAX_HEIGHT),
                ) as u16;

                self.add_span(SpanInsertion {
                    x,
                    z,
                    span: SpanBuilder {
                        min: span_min_cell_index,
                        max: span_max_cell_index,
                        area: area_type,
                        next: None,
                    }
                    .build(),
                    flag_merge_threshold,
                })?;
            }
        }
        Ok(())
    }
}

/// Errors that can occur when rasterizing a triangle into a heightfield with [`Heightfield::populate_from_trimesh`].
#[derive(Debug)]
pub enum RasterizationError {
    IntegerConversion,
    FilterError(crate::FilterError),
    /// Happens when the polygon division fails.
    PolygonDivisionError(PolygonDivisionError),
    /// Happens when the span insertion fails.
    SpanInsertionError(SpanInsertionError),
}

impl core::fmt::Display for RasterizationError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::IntegerConversion => f.write_str(
                "native raster coordinate conversion is outside the signed integer domain",
            ),
            Self::FilterError(error) => core::fmt::Display::fmt(error, f),
            Self::PolygonDivisionError(error) => write!(f, "Failed to rasterize triangle: {error}"),
            Self::SpanInsertionError(error) => write!(f, "Failed to add span: {error}"),
        }
    }
}
impl core::error::Error for RasterizationError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::IntegerConversion => None,
            Self::FilterError(error) => core::error::Error::source(error),
            Self::PolygonDivisionError(error) => Some(error),
            Self::SpanInsertionError(error) => Some(error),
        }
    }
}
impl From<crate::FilterError> for RasterizationError {
    fn from(error: crate::FilterError) -> Self {
        Self::FilterError(error)
    }
}
impl From<PolygonDivisionError> for RasterizationError {
    fn from(error: PolygonDivisionError) -> Self {
        Self::PolygonDivisionError(error)
    }
}
impl From<SpanInsertionError> for RasterizationError {
    fn from(error: SpanInsertionError) -> Self {
        Self::SpanInsertionError(error)
    }
}

/// Divides a convex polygon of max 12 vertices into two convex polygons
/// across a separating axis.
#[inline]
fn divide_poly<const AXIS: usize>(
    in_verts: &[Vec3A],
    in_vert_count: u8,
    out_verts_1: &mut [Vec3A],
    out_vert_count_1: &mut u8,
    out_verts_2: &mut [Vec3A],
    out_vert_count_2: &mut u8,
    axis_offset: f32,
) -> Result<(), PolygonDivisionError> {
    if in_vert_count > 12 {
        return Err(PolygonDivisionError::TooManyVertices(in_vert_count));
    } else if in_vert_count == 0 {
        *out_vert_count_1 = 0;
        *out_vert_count_2 = 0;
        return Ok(());
    }
    let in_vert_count = in_vert_count as usize;

    let in_verts = &in_verts[..in_vert_count];
    let mut previous = in_verts[in_vert_count - 1];
    let mut previous_delta = axis_offset - previous[AXIS];
    let mut poly_1_vert = 0;
    let mut poly_2_vert = 0;
    for &current in in_verts {
        let current_delta = axis_offset - current[AXIS];
        let same_side = (current_delta >= 0.0) == (previous_delta >= 0.0);
        if !same_side {
            let s = previous_delta / (previous_delta - current_delta);
            let delta = current - previous;
            #[cfg(feature = "fma")]
            let intersection = Vec3A::new(
                delta.x.mul_add(s, previous.x),
                delta.y.mul_add(s, previous.y),
                delta.z.mul_add(s, previous.z),
            );
            #[cfg(not(feature = "fma"))]
            let intersection = previous + delta * s;
            out_verts_1[poly_1_vert] = intersection;
            out_verts_2[poly_2_vert] = intersection;
            poly_1_vert += 1;
            poly_2_vert += 1;

            // A point on the dividing line was already emitted above.
            if current_delta > 0.0 {
                out_verts_1[poly_1_vert] = current;
                poly_1_vert += 1;
            } else if current_delta < 0.0 {
                out_verts_2[poly_2_vert] = current;
                poly_2_vert += 1;
            }
        } else if current_delta >= 0.0 {
            out_verts_1[poly_1_vert] = current;
            poly_1_vert += 1;
            if current_delta == 0.0 {
                out_verts_2[poly_2_vert] = current;
                poly_2_vert += 1;
            }
        } else {
            out_verts_2[poly_2_vert] = current;
            poly_2_vert += 1;
        }
        previous = current;
        previous_delta = current_delta;
    }

    *out_vert_count_1 = poly_1_vert as u8;
    *out_vert_count_2 = poly_2_vert as u8;

    Ok(())
}

/// Errors that can occur when dividing a polygon into two convex polygons across a separating axis.
#[derive(Debug)]
pub enum PolygonDivisionError {
    /// Happens when the polygon has more than 12 vertices.
    TooManyVertices(u8),
    /// Happens when the polygon has no vertices.
    NoVertices,
}

impl core::fmt::Display for PolygonDivisionError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::TooManyVertices(count) => write!(
                f,
                "Failed to divide polygon: too many vertices. Expected at most 12, got {count}."
            ),
            Self::NoVertices => f.write_str("Failed to divide polygon: no vertices."),
        }
    }
}
impl core::error::Error for PolygonDivisionError {}
