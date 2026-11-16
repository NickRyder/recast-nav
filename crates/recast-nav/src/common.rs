// Altered Rust translation of DetourCommon.cpp.
// Copyright (c) 2009-2010 Mikko Mononen. Zlib: vendor/recastnavigation/License.txt.
use crate::nav::{Point, area, madd, sub};

/// Samples native triangle-fan weights. Scratch entries 0/1 and any tail are
/// retained. Two vertices are a defined native line case; fewer return `None`.
pub fn random_point_in_convex_polygon(
    vertices: &[Point],
    areas: &mut [f32],
    s: f32,
    t: f32,
) -> Option<Point> {
    if vertices.len() < 2
        || vertices.len() > i32::MAX as usize
        || (vertices.len() > 2 && areas.len() < vertices.len())
    {
        return None;
    }
    let mut sum = 0.;
    for i in 2..vertices.len() {
        areas[i] = area(vertices[0], vertices[i - 1], vertices[i]);
        sum += if 0.001 > areas[i] { 0.001 } else { areas[i] };
    }
    let threshold = s * sum;
    let mut acc = 0.;
    let mut u = 1.;
    let mut tri = vertices.len() - 1;
    for (i, &weight) in areas.iter().enumerate().take(vertices.len()).skip(2) {
        if threshold >= acc && threshold < acc + weight {
            u = (threshold - acc) / weight;
            tri = i;
            break;
        }
        acc += weight;
    }
    let v = t.sqrt();
    let a = 1. - v;
    let b = (1. - u) * v;
    let c = u * v;
    Some(std::array::from_fn(|i| {
        add_scaled(
            madd(a, vertices[0][i], b * vertices[tri - 1][i]),
            c,
            vertices[tri][i],
        )
    }))
}

/// Native separating-axis polygon overlap in XZ. A single empty polygon would
/// make native projection read outside its vertices and returns `None` here.
pub fn overlap_polygons_2d(a: &[Point], b: &[Point]) -> Option<bool> {
    PolygonOverlap::new(a).overlaps(b)
}
#[derive(Clone, Copy, Default)]
struct Projection {
    axis: [f32; 2],
    min: f32,
    max: f32,
}
fn project_polygon(axis: [f32; 2], polygon: &[Point]) -> (f32, f32) {
    let dot = |p: Point| madd(axis[0], p[0], axis[1] * p[2]);
    let mut low = dot(polygon[0]);
    let mut high = low;
    for &point in &polygon[1..] {
        let d = dot(point);
        low = if low < d { low } else { d };
        high = if high > d { high } else { d };
    }
    (low, high)
}
/// Reuses a polygon's own projections across neighborhood overlap comparisons.
/// Only the first six axes are cached; larger public polygons retain the same
/// ordered traversal without allocating additional storage.
pub(crate) struct PolygonOverlap<'a> {
    polygon: &'a [Point],
    projections: [Projection; 6],
    initialized: u8,
}
impl<'a> PolygonOverlap<'a> {
    pub(crate) fn new(polygon: &'a [Point]) -> Self {
        Self {
            polygon,
            projections: [Projection::default(); 6],
            initialized: 0,
        }
    }
    pub(crate) fn overlaps(&mut self, b: &[Point]) -> Option<bool> {
        let a = self.polygon;
        if a.is_empty() && b.is_empty() {
            return Some(true);
        }
        if a.is_empty() || b.is_empty() {
            return None;
        }
        let edge_axis = |polygon: &[Point], i: usize| {
            let j = if i == 0 { polygon.len() - 1 } else { i - 1 };
            [
                polygon[i][2] - polygon[j][2],
                -(polygon[i][0] - polygon[j][0]),
            ]
        };
        // Only the first polygon's axes can reuse cached projections.
        for i in 0..a.len() {
            let (axis, amin, amax) =
                if i < self.projections.len() && self.initialized & (1 << i) != 0 {
                    let cached = self.projections[i];
                    (cached.axis, cached.min, cached.max)
                } else {
                    let axis = edge_axis(a, i);
                    let (min, max) = project_polygon(axis, a);
                    if i < self.projections.len() {
                        self.projections[i] = Projection { axis, min, max };
                        self.initialized |= 1 << i;
                    }
                    (axis, min, max)
                };
            let (bmin, bmax) = project_polygon(axis, b);
            if amin + 0.0001 > bmax || amax - 0.0001 < bmin {
                return Some(false);
            }
        }
        for i in 0..b.len() {
            let axis = edge_axis(b, i);
            let (amin, amax) = project_polygon(axis, a);
            let (bmin, bmax) = project_polygon(axis, b);
            if amin + 0.0001 > bmax || amax - 0.0001 < bmin {
                return Some(false);
            }
        }
        Some(true)
    }
}

/// Native XZ line-intersection parameters. Parameters are not clamped to the
/// segments; near-parallel lines return `None`.
pub fn intersect_segments_2d(a: Point, b: Point, c: Point, d: Point) -> Option<(f32, f32)> {
    let u = sub(b, a);
    let v = sub(d, c);
    let w = sub(a, c);
    let perp = |a: Point, b: Point| product_difference(a[0], b[2], a[2], b[0]);
    let divisor = perp(u, v);
    if divisor.abs() < 0.000001 {
        return None;
    }
    Some((perp(v, w) / divisor, perp(u, w) / divisor))
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SegmentPolygonIntersection {
    pub intersects: bool,
    pub t_min: f32,
    pub t_max: f32,
    pub entering_edge: i32,
    pub leaving_edge: i32,
}

/// Clips an XZ segment against the polygon's ordered half planes. Output fields
/// retain the native partial results even when `intersects` is false.
pub fn intersect_segment_polygon_2d(
    start: Point,
    end: Point,
    vertices: &[Point],
) -> SegmentPolygonIntersection {
    let mut result = SegmentPolygonIntersection {
        intersects: true,
        t_min: 0.,
        t_max: 1.,
        entering_edge: -1,
        leaving_edge: -1,
    };
    let dir: Point = std::array::from_fn(|i| end[i] - start[i]);
    for i in 0..vertices.len() {
        let j = if i == 0 { vertices.len() - 1 } else { i - 1 };
        let edge: Point = std::array::from_fn(|a| vertices[i][a] - vertices[j][a]);
        let diff: Point = std::array::from_fn(|a| start[a] - vertices[j][a]);
        let n = vector_perpendicular_2d(edge, diff);
        let d = vector_perpendicular_2d(dir, edge);
        if d.abs() < 0.000001 {
            if n < 0. {
                result.intersects = false;
                return result;
            }
            continue;
        }
        let t = n / d;
        if d < 0. {
            if t > result.t_min {
                result.t_min = t;
                result.entering_edge = j as i32;
                if result.t_min > result.t_max {
                    result.intersects = false;
                    return result;
                }
            }
        } else if t < result.t_max {
            result.t_max = t;
            result.leaving_edge = j as i32;
            if result.t_max < result.t_min {
                result.intersects = false;
                return result;
            }
        }
    }
    result
}

/// Native comparison order, including unordered floating-point values.
pub fn minimum<T: PartialOrd>(a: T, b: T) -> T {
    if a < b { a } else { b }
}
pub fn maximum<T: PartialOrd>(a: T, b: T) -> T {
    if a > b { a } else { b }
}
pub fn clamp<T: PartialOrd>(value: T, low: T, high: T) -> T {
    if value < low {
        low
    } else if value > high {
        high
    } else {
        value
    }
}
/// Numeric operations used by the native `rcAbs`/`dtAbs` and `rcSqr`/`dtSqr`
/// templates. Primitive integers, `bool`, and `f32`/`f64` are supported. Custom scalar
/// types can implement the same operations without a C++ operator interface.
///
/// Integer results wrap to their output width. This preserves native promoted
/// arithmetic and narrowing wherever the native expression is defined. Native
/// signed-overflow inputs have no equivalence guarantee.
pub trait Scalar: Copy {
    fn absolute(self) -> Self;
    fn square(self) -> Self;
}

macro_rules! float_scalar {
    ($($type:ty),+ $(,)?) => {$(
        impl Scalar for $type {
            fn absolute(self) -> Self {
                if self < 0. { -self } else { self }
            }
            fn square(self) -> Self {
                self * self
            }
        }
    )+};
}
macro_rules! signed_scalar {
    ($($type:ty),+ $(,)?) => {$(
        impl Scalar for $type {
            fn absolute(self) -> Self {
                if self < 0 { self.wrapping_neg() } else { self }
            }
            fn square(self) -> Self {
                self.wrapping_mul(self)
            }
        }
    )+};
}
macro_rules! unsigned_scalar {
    ($($type:ty),+ $(,)?) => {$(
        impl Scalar for $type {
            fn absolute(self) -> Self {
                self
            }
            fn square(self) -> Self {
                self.wrapping_mul(self)
            }
        }
    )+};
}
float_scalar!(f32, f64);
signed_scalar!(i8, i16, i32, i64, i128, isize);
unsigned_scalar!(u8, u16, u32, u64, u128, usize);
impl Scalar for bool {
    fn absolute(self) -> Self {
        self
    }
    fn square(self) -> Self {
        self
    }
}

/// Unlike floating-point `abs`, the native template retains negative zero and
/// NaN sign bits. Integer narrowing follows [`Scalar`].
pub fn absolute<T: Scalar>(value: T) -> T {
    Scalar::absolute(value)
}
/// Native `dtMathFloorf`, including quieting a signaling NaN without losing its payload.
#[inline]
pub fn floor(value: f32) -> f32 {
    if value.is_nan() {
        f32::from_bits(value.to_bits() | 0x0040_0000)
    } else {
        value.floor()
    }
}
/// Native `dtMathCeilf`, including quieting a signaling NaN without losing its payload.
#[inline]
pub fn ceil(value: f32) -> f32 {
    if value.is_nan() {
        f32::from_bits(value.to_bits() | 0x0040_0000)
    } else {
        value.ceil()
    }
}
/// Calls the native cosine backend independently of sine.
/// Keeping the call boundary prevents LLVM from substituting a combined sin/cos call.
#[inline(never)]
pub fn cosine(value: f32) -> f32 {
    value.cos()
}
/// Calls the native sine backend independently of cosine.
#[inline(never)]
pub fn sine(value: f32) -> f32 {
    value.sin()
}
pub fn square<T: Scalar>(value: T) -> T {
    Scalar::square(value)
}
// Retain the native operand order when the result carries a NaN payload.
#[inline(never)]
fn ordered_multiply_subtract(a: f32, b: f32, c: f32) -> f32 {
    #[cfg(feature = "fma")]
    {
        a.mul_add(b, -c)
    }
    #[cfg(not(feature = "fma"))]
    {
        a * b - c
    }
}
// Ordinary finite/infinite results do not need a call boundary. A NaN result
// is recomputed through the ordered helper using the original operand bits.
#[inline]
pub(crate) fn multiply_subtract(a: f32, b: f32, c: f32) -> f32 {
    #[cfg(feature = "fma")]
    let result = a.mul_add(b, -c);
    #[cfg(not(feature = "fma"))]
    let result = a * b - c;
    if result.is_nan() {
        ordered_multiply_subtract(a, b, c)
    } else {
        result
    }
}
// Passing the second product's operands separately retains their native order
// even when both carry different NaN payloads.
#[inline(never)]
fn ordered_product_difference(a: f32, b: f32, c: f32, d: f32) -> f32 {
    ordered_multiply_subtract(a, b, c * d)
}
#[inline]
fn product_difference(a: f32, b: f32, c: f32, d: f32) -> f32 {
    #[cfg(feature = "fma")]
    let result = a.mul_add(b, -(c * d));
    #[cfg(not(feature = "fma"))]
    let result = a * b - c * d;
    if result.is_nan() {
        ordered_product_difference(a, b, c, d)
    } else {
        result
    }
}

// Keep the native left operand first when separate operations propagate NaNs.
#[inline(never)]
fn ordered_add_scaled(a: f32, b: f32, scale: f32) -> f32 {
    #[cfg(feature = "fma")]
    {
        b.mul_add(scale, a)
    }
    #[cfg(not(feature = "fma"))]
    {
        a + b * scale
    }
}
#[inline]
pub(crate) fn add_scaled(a: f32, b: f32, scale: f32) -> f32 {
    #[cfg(feature = "fma")]
    let result = b.mul_add(scale, a);
    #[cfg(not(feature = "fma"))]
    let result = a + b * scale;
    if result.is_nan() {
        ordered_add_scaled(a, b, scale)
    } else {
        result
    }
}

// Retain the native multiply/subtract boundary and operand order.
#[inline(never)]
fn ordered_subtract_scaled(a: f32, b: f32, scale: f32) -> f32 {
    #[cfg(feature = "fma")]
    {
        (-b).mul_add(scale, a)
    }
    #[cfg(not(feature = "fma"))]
    {
        a - b * scale
    }
}
#[inline]
pub(crate) fn subtract_scaled(a: f32, b: f32, scale: f32) -> f32 {
    #[cfg(feature = "fma")]
    let result = (-b).mul_add(scale, a);
    #[cfg(not(feature = "fma"))]
    let result = a - b * scale;
    if result.is_nan() {
        ordered_subtract_scaled(a, b, scale)
    } else {
        result
    }
}

pub fn vector_cross(a: Point, b: Point) -> Point {
    [
        multiply_subtract(a[1], b[2], a[2] * b[1]),
        multiply_subtract(a[2], b[0], a[0] * b[2]),
        multiply_subtract(a[0], b[1], a[1] * b[0]),
    ]
}
// Retain the native helper boundary, including operand selection when more
// than one component carries a NaN payload.
#[inline(never)]
fn ordered_vector_dot(a: Point, b: Point) -> f32 {
    ordered_add_scaled(madd(a[0], b[0], a[1] * b[1]), a[2], b[2])
}
#[inline]
pub fn vector_dot(a: Point, b: Point) -> f32 {
    let first = madd(a[0], b[0], a[1] * b[1]);
    #[cfg(feature = "fma")]
    let result = a[2].mul_add(b[2], first);
    #[cfg(not(feature = "fma"))]
    let result = first + a[2] * b[2];
    if result.is_nan() {
        ordered_vector_dot(a, b)
    } else {
        result
    }
}

pub fn vector_multiply_add(a: Point, b: Point, scale: f32) -> Point {
    std::array::from_fn(|i| add_scaled(a[i], b[i], scale))
}
pub fn vector_lerp(a: Point, b: Point, t: f32) -> Point {
    vector_multiply_add(a, sub(b, a), t)
}
pub fn vector_add(a: Point, b: Point) -> Point {
    std::array::from_fn(|i| a[i] + b[i])
}
pub fn vector_subtract(a: Point, b: Point) -> Point {
    sub(a, b)
}
pub fn vector_scale(v: Point, scale: f32) -> Point {
    v.map(|x| x * scale)
}
pub fn vector_minimum(a: Point, b: Point) -> Point {
    std::array::from_fn(|i| minimum(a[i], b[i]))
}
pub fn vector_maximum(a: Point, b: Point) -> Point {
    std::array::from_fn(|i| maximum(a[i], b[i]))
}
pub fn vector_length_squared(v: Point) -> f32 {
    vector_dot(v, v)
}
pub fn vector_length(v: Point) -> f32 {
    vector_length_squared(v).sqrt()
}
pub fn vector_distance_squared(a: Point, b: Point) -> f32 {
    vector_length_squared(sub(b, a))
}
pub fn vector_distance(a: Point, b: Point) -> f32 {
    vector_distance_squared(a, b).sqrt()
}
#[inline(never)]
fn ordered_vector_dot_2d(a: Point, b: Point) -> f32 {
    madd(a[0], b[0], a[2] * b[2])
}
#[inline]
pub fn vector_dot_2d(a: Point, b: Point) -> f32 {
    let result = madd(a[0], b[0], a[2] * b[2]);
    if result.is_nan() {
        ordered_vector_dot_2d(a, b)
    } else {
        result
    }
}

pub fn vector_perpendicular_2d(a: Point, b: Point) -> f32 {
    multiply_subtract(a[2], b[0], a[0] * b[2])
}
pub fn vector_distance_squared_2d(a: Point, b: Point) -> f32 {
    let d = sub(b, a);
    vector_dot_2d(d, d)
}
pub fn vector_distance_2d(a: Point, b: Point) -> f32 {
    vector_distance_squared_2d(a, b).sqrt()
}
/// Native normalization does not guard zero length. Its dtSqr calls keep the
/// three products separate even under the FMA reference policy.
pub fn vector_normalize(v: Point) -> Point {
    let inverse = 1. / (square(v[0]) + square(v[1]) + square(v[2])).sqrt();
    vector_scale(v, inverse)
}
pub fn vector_equal(a: Point, b: Point) -> bool {
    vector_distance_squared(a, b) < square(1. / 16384.)
}
pub fn vector_is_finite(v: Point) -> bool {
    v.into_iter().all(f32::is_finite)
}
pub fn vector_is_finite_2d(v: Point) -> bool {
    v[0].is_finite() && v[2].is_finite()
}
pub fn triangle_area_2d(a: Point, b: Point, c: Point) -> f32 {
    let ab = sub(b, a);
    let ac = sub(c, a);
    multiply_subtract(ac[0], ab[2], ab[0] * ac[2])
}
pub fn overlap_bounds(a: [Point; 2], b: [Point; 2]) -> bool {
    !(0..3).any(|i| a[0][i] > b[1][i] || a[1][i] < b[0][i])
}
pub fn overlap_quantized_bounds(a: [[u16; 3]; 2], b: [[u16; 3]; 2]) -> bool {
    !(0..3).any(|i| a[0][i] > b[1][i] || a[1][i] < b[0][i])
}
pub fn next_power_of_two(value: u32) -> u32 {
    if value == 0 {
        0
    } else {
        value.checked_next_power_of_two().unwrap_or(0)
    }
}
pub fn integer_log2(value: u32) -> u32 {
    if value == 0 {
        0
    } else {
        31 - value.leading_zeros()
    }
}
/// Rejects the native signed-overflow domain.
pub fn align4(value: i32) -> Option<i32> {
    value.checked_add(3).map(|v| v & !3)
}
pub fn opposite_tile(side: i32) -> Option<i32> {
    side.checked_add(4).map(|v| v & 7)
}
pub fn swap_endian_u16(value: u16) -> u16 {
    value.swap_bytes()
}
pub fn swap_endian_i16(value: i16) -> i16 {
    value.swap_bytes()
}
pub fn swap_endian_u32(value: u32) -> u32 {
    value.swap_bytes()
}
pub fn swap_endian_i32(value: i32) -> i32 {
    value.swap_bytes()
}
pub fn swap_endian_f32(value: f32) -> f32 {
    f32::from_bits(value.to_bits().swap_bytes())
}

pub fn closest_point_on_triangle(p: Point, a: Point, b: Point, c: Point) -> Point {
    let project = |origin: Point, direction: Point, scale: f32| {
        std::array::from_fn(|i| add_scaled(origin[i], scale, direction[i]))
    };
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ap = sub(p, a);
    let d1 = vector_dot(ab, ap);
    let d2 = vector_dot(ac, ap);
    if d1 <= 0. && d2 <= 0. {
        return a;
    }
    let bp = sub(p, b);
    let d3 = vector_dot(ab, bp);
    let d4 = vector_dot(ac, bp);
    if d3 >= 0. && d4 <= d3 {
        return b;
    }
    let vc = multiply_subtract(d1, d4, d3 * d2);
    if vc <= 0. && d1 >= 0. && d3 <= 0. {
        return project(a, ab, d1 / (d1 - d3));
    }
    let cp = sub(p, c);
    let d5 = vector_dot(ab, cp);
    let d6 = vector_dot(ac, cp);
    if d6 >= 0. && d5 <= d6 {
        return c;
    }
    let vb = multiply_subtract(d5, d2, d1 * d6);
    if vb <= 0. && d2 >= 0. && d6 <= 0. {
        return project(a, ac, d2 / (d2 - d6));
    }
    let va = multiply_subtract(d3, d6, d5 * d4);
    if va <= 0. && d4 - d3 >= 0. && d5 - d6 >= 0. {
        return project(b, sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    let denominator = 1. / (va + vb + vc);
    let v = vb * denominator;
    let w = vc * denominator;
    vector_multiply_add(vector_multiply_add(a, ab, v), ac, w)
}
/// A failed height lookup leaves the supplied height unchanged.
pub fn closest_height_point_triangle_into(
    p: Point,
    a: Point,
    b: Point,
    c: Point,
    height: &mut f32,
) -> bool {
    let v0 = sub(c, a);
    let v1 = sub(b, a);
    let v2 = sub(p, a);
    let mut denominator = multiply_subtract(v0[0], v1[2], v0[2] * v1[0]);
    if denominator.abs() < 1e-6 {
        return false;
    }
    let mut u = multiply_subtract(v1[2], v2[0], v1[0] * v2[2]);
    let mut v = multiply_subtract(v0[0], v2[2], v0[2] * v2[0]);
    if denominator < 0. {
        denominator = -denominator;
        u = -u;
        v = -v;
    }
    if u >= 0. && v >= 0. && u + v <= denominator {
        *height = a[1] + madd(v0[1], u, v1[1] * v) / denominator;
        true
    } else {
        false
    }
}
pub fn point_in_polygon(p: Point, vertices: &[Point]) -> bool {
    crate::nav::inside(p, vertices)
}
pub fn distance_point_segment_squared_2d(p: Point, a: Point, b: Point) -> (f32, f32) {
    let delta = sub(b, a);
    let relative = sub(p, a);
    let d = vector_dot_2d(delta, delta);
    let mut t = vector_dot_2d(delta, relative);
    if d > 0. {
        t /= d;
    }
    t = clamp(t, 0., 1.);
    let x = add_scaled(a[0], t, delta[0]) - p[0];
    let z = add_scaled(a[2], t, delta[2]) - p[2];
    (madd(x, x, z * z), t)
}
/// Writes one distance/parameter per edge, retaining any caller buffer tail.
/// Returns `None` if either output slice is too small.
pub fn distance_point_polygon_edges_squared(
    p: Point,
    vertices: &[Point],
    distances: &mut [f32],
    parameters: &mut [f32],
) -> Option<bool> {
    if distances.len() < vertices.len() || parameters.len() < vertices.len() {
        return None;
    }
    for i in 0..vertices.len() {
        let j = if i == 0 { vertices.len() - 1 } else { i - 1 };
        (distances[j], parameters[j]) =
            distance_point_segment_squared_2d(p, vertices[j], vertices[i]);
    }
    Some(point_in_polygon(p, vertices))
}
/// Zero indices retain native 0*infinity results. Invalid vertex indices are checked.
pub fn polygon_center(indices: &[u16], vertices: &[Point]) -> Option<Point> {
    if indices.len() > i32::MAX as usize {
        return None;
    }
    let mut center = [0.; 3];
    for &i in indices {
        center = vector_add(center, *vertices.get(i as usize)?);
    }
    Some(vector_scale(center, 1. / indices.len() as f32))
}
