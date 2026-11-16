// Altered Rust translation of DetourObstacleAvoidance.cpp, RecastNavigation v1.6.0.
// Copyright (c) 2009-2010 Mikko Mononen. Zlib: vendor/recastnavigation/License.txt.
use crate::common::{add_scaled as strict_offset, multiply_subtract as strict_msub};
use crate::nav::*;
// Non-NaN results have deterministic bits for these unchanged expressions.
// Recompute NaN results through the strict call boundaries, with the original
// operands, to retain native payload priority without guarding each input.
#[inline]
fn msub(a: f32, b: f32, c: f32) -> f32 {
    #[cfg(feature = "fma")]
    let value = a.mul_add(b, -c);
    #[cfg(not(feature = "fma"))]
    let value = a * b - c;
    if value.is_nan() {
        strict_msub(a, b, c)
    } else {
        value
    }
}
#[inline]
fn offset(a: f32, b: f32, scale: f32) -> f32 {
    #[cfg(feature = "fma")]
    let value = b.mul_add(scale, a);
    #[cfg(not(feature = "fma"))]
    let value = a + b * scale;
    if value.is_nan() {
        strict_offset(a, b, scale)
    } else {
        value
    }
}
// Preserve native pattern arithmetic operand order, including NaNs.
#[inline(never)]
fn strict_pattern_add(a: f32, b: f32, c: f32) -> f32 {
    madd(a, b, c)
}
#[inline]
fn pattern_add(a: f32, b: f32, c: f32) -> f32 {
    let value = madd(a, b, c);
    if value.is_nan() {
        strict_pattern_add(a, b, c)
    } else {
        value
    }
}
fn min(a: f32, b: f32) -> f32 {
    if a < b { a } else { b }
}
fn max(a: f32, b: f32) -> f32 {
    if a > b { a } else { b }
}
fn clamp(a: f32) -> f32 {
    a.clamp(0., 1.)
}

#[inline(never)]
fn strict_product(a: f32, b: f32) -> f32 {
    a * b
}
#[inline(never)]
fn strict_dot(a: Point, b: Point) -> f32 {
    // Keep both the second product's operand order and the addition/FMA
    // boundary when multiple prepared geometry values carry distinct NaNs.
    strict_pattern_add(a[0], b[0], strict_product(a[2], b[2]))
}
fn dot(a: Point, b: Point) -> f32 {
    let value = madd(a[0], b[0], a[2] * b[2]);
    if value.is_nan() {
        strict_dot(a, b)
    } else {
        value
    }
}
// Unobserved penalty NaNs only reach unordered comparisons and cannot select
// a candidate. Recorded costs use strict payloads unless their dot operands
// are proven finite and small enough to keep both products and their sum finite.
#[inline]
fn penalty_dot<const PRESERVE: bool>(a: Point, b: Point) -> f32 {
    if PRESERVE {
        dot(a, b)
    } else {
        madd(a[0], b[0], a[2] * b[2])
    }
}
fn perp(a: Point, b: Point) -> f32 {
    msub(a[2], b[0], a[0] * b[2])
}
fn distance<const PRESERVE: bool>(a: Point, b: Point) -> f32 {
    let d = sub(b, a);
    penalty_dot::<PRESERVE>(d, d).sqrt()
}
fn zeroed<T: Default>(n: usize) -> Result<Vec<T>, u32> {
    let mut v = Vec::new();
    v.try_reserve_exact(n)
        .map_err(|_| FAILURE | OUT_OF_MEMORY)?;
    v.resize_with(n, T::default);
    Ok(v)
}
#[derive(Clone, Copy, Debug, Default)]
pub struct ObstacleCircle {
    pub position: Point,
    pub velocity: Point,
    pub desired_velocity: Point,
    pub radius: f32,
    pub direction: Point,
    pub preferred_side: Point,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct ObstacleSegment {
    pub positions: [Point; 2],
    pub touch: bool,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct AvoidanceParams {
    pub velocity_bias: f32,
    pub weight_desired_velocity: f32,
    pub weight_current_velocity: f32,
    pub weight_side: f32,
    pub weight_time_of_impact: f32,
    pub horizon_time: f32,
    pub grid_size: u8,
    pub adaptive_divisions: u8,
    pub adaptive_rings: u8,
    pub adaptive_depth: u8,
}
#[derive(Clone, Copy, Debug)]
pub struct VelocityInput {
    pub position: Point,
    pub radius: f32,
    pub maximum_speed: f32,
    pub velocity: Point,
    pub desired_velocity: Point,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct VelocitySample {
    pub velocity: Point,
    pub sample_count: usize,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct AvoidanceDebugSample {
    pub velocity: Point,
    pub size: f32,
    /// Total, desired-velocity, current-velocity, preferred-side, collision-time.
    pub penalties: [f32; 5],
}
#[derive(Clone, Debug, Default)]
pub struct AvoidanceDebugData {
    samples: Vec<AvoidanceDebugSample>,
    count: usize,
    initialized_end: usize,
    gap_start: usize,
    gap_end: usize,
}
impl AvoidanceDebugData {
    pub fn new(capacity: usize) -> Result<Self, u32> {
        if capacity == 0 || capacity > i32::MAX as usize {
            return Err(FAILURE | INVALID_PARAM);
        }
        Ok(Self {
            samples: zeroed(capacity)?,
            ..Default::default()
        })
    }
    /// Native retains the sample count while replacing storage with uninitialized
    /// arrays. Rust tracks which replacement entries are initialized.
    pub fn reinitialize(&mut self, capacity: usize) -> Result<(), u32> {
        let fresh = Self::new(capacity)?;
        self.samples = fresh.samples;
        self.initialized_end = 0;
        self.gap_start = 0;
        self.gap_end = 0;
        Ok(())
    }
    pub fn reset(&mut self) {
        self.count = 0;
    }
    pub fn sample_count(&self) -> usize {
        self.count
    }
    pub fn sample(&self, index: usize) -> Option<&AvoidanceDebugSample> {
        self.samples.get(index).filter(|_| self.initialized(index))
    }
    pub fn samples(&self) -> impl Iterator<Item = &AvoidanceDebugSample> {
        self.samples
            .iter()
            .enumerate()
            .take(self.count)
            .filter_map(move |(i, sample)| self.initialized(i).then_some(sample))
    }
    pub fn add_sample(&mut self, sample: AvoidanceDebugSample) {
        if self.count >= self.samples.len() {
            return;
        }
        let index = self.count;
        self.samples[index] = sample;
        // Writes are sequential from either the retained reinit count or zero
        // after reset. Their union has at most one uninitialized gap.
        if index >= self.initialized_end {
            if index > self.initialized_end {
                self.gap_start = self.initialized_end;
                self.gap_end = index;
            }
            self.initialized_end = index + 1;
        } else if index == self.gap_start && self.gap_start < self.gap_end {
            self.gap_start += 1;
        }
        self.count += 1;
    }
    fn initialized(&self, index: usize) -> bool {
        index < self.initialized_end && !(self.gap_start <= index && index < self.gap_end)
    }
    pub fn normalize_samples(&mut self) -> u32 {
        if self.count > self.samples.len()
            || self.count > self.initialized_end
            || (self.gap_start < self.count && self.gap_start < self.gap_end)
        {
            return FAILURE | INVALID_PARAM;
        }
        for col in 0..5 {
            let mut low = f32::MAX;
            let mut high = -f32::MAX;
            for sample in &self.samples[..self.count] {
                let p = sample.penalties[col];
                low = min(low, p);
                high = max(high, p);
            }
            let range = high - low;
            let scale = if range > 0.001 { 1. / range } else { 1. };
            for sample in &mut self.samples[..self.count] {
                let p = &mut sample.penalties[col];
                *p = clamp((*p - low) * scale);
            }
        }
        SUCCESS
    }
}
#[derive(Clone, Debug)]
pub struct ObstacleAvoidanceQuery {
    circles: Vec<ObstacleCircle>,
    circle_count: usize,
    segments: Vec<ObstacleSegment>,
    segment_count: usize,
    rotation_divisions: u8,
    rotation: [f32; 4],
}
struct Sampling {
    input: VelocityInput,
    params: AvoidanceParams,
    inverse_horizon: f32,
    inverse_maximum_speed: f32,
    preserve_nan_costs: bool,
    sweep_geometry: [SweepGeometry; super::MAX_NEIGHBORS],
}
impl Sampling {
    fn new(
        input: VelocityInput,
        params: AvoidanceParams,
        preserve_nan_costs: bool,
        circles: &[ObstacleCircle],
    ) -> Self {
        // Crowd supplies at most six neighbors. Cache only candidate-independent
        // geometry on the stack; larger public queries retain the same fallback.
        let mut sweep_geometry = [SweepGeometry::default(); super::MAX_NEIGHBORS];
        for (geometry, circle) in sweep_geometry.iter_mut().zip(circles) {
            *geometry = SweepGeometry::new(input, circle);
        }
        Self {
            sweep_geometry,
            input,
            params,
            preserve_nan_costs,
            inverse_horizon: 1. / params.horizon_time,
            inverse_maximum_speed: if input.maximum_speed > 0. {
                1. / input.maximum_speed
            } else {
                f32::MAX
            },
        }
    }
}
#[derive(Clone, Copy, Default)]
struct SweepGeometry {
    separation: [f32; 2],
    c: f32,
}
impl SweepGeometry {
    fn new(input: VelocityInput, circle: &ObstacleCircle) -> Self {
        let s = sub(circle.position, input.position);
        let r = input.radius + circle.radius;
        Self {
            separation: [s[0], s[2]],
            c: madd(-r, r, madd(s[0], s[0], s[2] * s[2])),
        }
    }
}
fn sweep_circle(velocity: Point, geometry: SweepGeometry) -> Option<(f32, f32)> {
    let [sx, sz] = geometry.separation;
    let c = geometry.c;
    let mut a = madd(velocity[0], velocity[0], velocity[2] * velocity[2]);
    if a < 0.0001 {
        return None;
    }
    let b = madd(velocity[0], sx, velocity[2] * sz);
    let d = madd(b, b, -a * c);
    if d < 0. {
        return None;
    }
    a = 1. / a;
    let rd = d.sqrt();
    Some(((b - rd) * a, (b + rd) * a))
}
#[allow(clippy::manual_range_contains)] // Native accepts NaN parameters; contains rejects them.
fn intersect_ray(position: Point, velocity: Point, a: Point, b: Point) -> Option<f32> {
    let v = sub(b, a);
    let w = sub(position, a);
    let d = perp(velocity, v);
    if d.abs() < 1e-6 {
        return None;
    }
    let d = 1. / d;
    let t = perp(v, w) * d;
    if t < 0. || t > 1. {
        return None;
    }
    let s = perp(velocity, w) * d;
    if s < 0. || s > 1. {
        return None;
    }
    Some(t)
}
impl ObstacleAvoidanceQuery {
    pub fn new(max_circles: usize, max_segments: usize) -> Result<Self, u32> {
        if max_circles > i32::MAX as usize || max_segments > i32::MAX as usize {
            return Err(FAILURE | INVALID_PARAM);
        }
        Ok(Self {
            circles: zeroed(max_circles)?,
            circle_count: 0,
            segments: zeroed(max_segments)?,
            segment_count: 0,
            rotation_divisions: 0,
            rotation: [0.; 4],
        })
    }
    pub fn reinitialize(&mut self, max_circles: usize, max_segments: usize) -> Result<(), u32> {
        *self = Self::new(max_circles, max_segments)?;
        Ok(())
    }
    pub fn reset(&mut self) {
        self.circle_count = 0;
        self.segment_count = 0;
    }
    pub fn circles(&self) -> &[ObstacleCircle] {
        &self.circles[..self.circle_count]
    }
    pub fn segments(&self) -> &[ObstacleSegment] {
        &self.segments[..self.segment_count]
    }
    /// Allocated records are initialized and remain readable after reset.
    pub fn circle(&self, index: usize) -> Option<&ObstacleCircle> {
        self.circles.get(index)
    }
    pub fn segment(&self, index: usize) -> Option<&ObstacleSegment> {
        self.segments.get(index)
    }
    pub fn add_circle(
        &mut self,
        position: Point,
        radius: f32,
        velocity: Point,
        desired_velocity: Point,
    ) {
        let Some(c) = self.circles.get_mut(self.circle_count) else {
            return;
        };
        self.circle_count += 1;
        c.position = position;
        c.radius = radius;
        c.velocity = velocity;
        c.desired_velocity = desired_velocity;
    }
    pub fn add_segment(&mut self, positions: [Point; 2]) {
        let Some(s) = self.segments.get_mut(self.segment_count) else {
            return;
        };
        self.segment_count += 1;
        s.positions = positions;
    }
    fn prepare(&mut self, position: Point, desired: Point) {
        for c in &mut self.circles[..self.circle_count] {
            c.direction = sub(c.position, position);
            // Native dtSqr calls keep these products separate under contraction.
            let d = c.direction;
            let inverse = 1. / (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            for v in &mut c.direction {
                *v *= inverse;
            }
            let dv = sub(c.desired_velocity, desired);
            let area = madd(dv[0], c.direction[2], -c.direction[0] * dv[2]);
            if area < 0.01 {
                c.preferred_side[0] = -c.direction[2];
                c.preferred_side[2] = c.direction[0];
            } else {
                c.preferred_side[0] = c.direction[2];
                c.preferred_side[2] = -c.direction[0];
            }
        }
        for s in &mut self.segments[..self.segment_count] {
            s.touch = crate::nav::segment(position, s.positions[0], s.positions[1]).0 < 0.01 * 0.01;
        }
    }
    fn pattern_rotation(&mut self, divisions: i32) -> [f32; 4] {
        // The clamped division count alone determines these finite angles.
        // Keep the original expressions and independent libm calls, retaining
        // their exact results for repeated samples on this query.
        if self.rotation_divisions != divisions as u8 {
            let angle = (1. / divisions as f32) * std::f32::consts::PI * 2.;
            let ca = crate::common::cosine(angle);
            let sa = crate::common::sine(angle);
            let half = angle * 0.5;
            self.rotation = [
                ca,
                sa,
                crate::common::cosine(half),
                crate::common::sine(half),
            ];
            self.rotation_divisions = divisions as u8;
        }
        self.rotation
    }
    fn finite_cost_vectors(
        &self,
        input: VelocityInput,
        p: &AvoidanceParams,
        adaptive: bool,
    ) -> bool {
        // With the clamped 32-division/four-ring pattern and eight-bit grid,
        // these conservative bounds keep candidates below 1e17 and every
        // observed dot product and sum below f32::MAX. Other scenes retain
        // strict NaN operand ordering.
        let bounded = |v: f32| v.abs() <= 1e6;
        let point = |v: Point| bounded(v[0]) && bounded(v[2]);
        (adaptive || p.grid_size >= 2)
            && (0. ..=1.).contains(&p.velocity_bias)
            && (0. ..=1e6).contains(&input.maximum_speed)
            && point(input.velocity)
            && point(input.desired_velocity)
            && [
                p.weight_desired_velocity,
                p.weight_current_velocity,
                p.weight_side,
                p.weight_time_of_impact,
                p.horizon_time,
            ]
            .iter()
            .all(|v| v.is_finite())
            && self
                .circles()
                .iter()
                .all(|c| point(c.direction) && point(c.preferred_side) && point(c.velocity))
            && self
                .segments()
                .iter()
                .all(|s| !s.touch || (point(s.positions[0]) && point(s.positions[1])))
    }
    #[inline]
    fn process_sample(
        &self,
        candidate: Point,
        size: f32,
        context: &Sampling,
        minimum: f32,
        debug: &mut Option<&mut AvoidanceDebugData>,
    ) -> f32 {
        if context.preserve_nan_costs {
            self.process_sample_inner::<true>(candidate, size, context, minimum, debug)
        } else {
            self.process_sample_inner::<false>(candidate, size, context, minimum, debug)
        }
    }
    fn process_sample_inner<const PRESERVE: bool>(
        &self,
        candidate: Point,
        size: f32,
        context: &Sampling,
        minimum: f32,
        debug: &mut Option<&mut AvoidanceDebugData>,
    ) -> f32 {
        let p = context.params;
        let input = context.input;
        let vpen = p.weight_desired_velocity
            * (distance::<PRESERVE>(candidate, input.desired_velocity)
                * context.inverse_maximum_speed);
        let vcpen = p.weight_current_velocity
            * (distance::<PRESERVE>(candidate, input.velocity) * context.inverse_maximum_speed);
        let min_penalty = minimum - vpen - vcpen;
        let threshold = (p.weight_time_of_impact / min_penalty - 0.1) * p.horizon_time;
        if threshold - p.horizon_time > -f32::EPSILON {
            return minimum;
        }
        let mut tmin = p.horizon_time;
        let mut side = 0.;
        let mut count = 0;
        for (index, c) in self.circles().iter().enumerate() {
            let relative =
                std::array::from_fn(|i| candidate[i] * 2. - input.velocity[i] - c.velocity[i]);
            side += clamp(min(
                madd(penalty_dot::<PRESERVE>(c.direction, relative), 0.5, 0.5),
                penalty_dot::<PRESERVE>(c.preferred_side, relative) * 2.,
            ));
            count += 1;
            let geometry = context
                .sweep_geometry
                .get(index)
                .copied()
                .unwrap_or_else(|| SweepGeometry::new(input, c));
            let Some((mut first, last)) = sweep_circle(relative, geometry) else {
                continue;
            };
            if first < 0. && last > 0. {
                first = -first * 0.5;
            }
            if first >= 0. && first < tmin {
                tmin = first;
                if tmin < threshold {
                    return minimum;
                }
            }
        }
        for s in self.segments() {
            let mut time = if s.touch {
                let d = sub(s.positions[1], s.positions[0]);
                if penalty_dot::<PRESERVE>([-d[2], 0., d[0]], candidate) < 0. {
                    continue;
                }
                0.
            } else {
                let Some(time) =
                    intersect_ray(input.position, candidate, s.positions[0], s.positions[1])
                else {
                    continue;
                };
                time
            };
            time *= 2.;
            if time < tmin {
                tmin = time;
                if tmin < threshold {
                    return minimum;
                }
            }
        }
        if count != 0 {
            side /= count as f32;
        }
        let spen = p.weight_side * side;
        let tpen = p.weight_time_of_impact * (1. / madd(tmin, context.inverse_horizon, 0.1));
        let penalty = vpen + vcpen + spen + tpen;
        if let Some(debug) = debug {
            debug.add_sample(AvoidanceDebugSample {
                velocity: candidate,
                size,
                penalties: [penalty, vpen, vcpen, spen, tpen],
            });
        }
        penalty
    }
    pub fn sample_velocity_grid(
        &mut self,
        input: VelocityInput,
        params: &AvoidanceParams,
        mut debug: Option<&mut AvoidanceDebugData>,
    ) -> VelocitySample {
        self.prepare(input.position, input.desired_velocity);
        let context = Sampling::new(
            input,
            *params,
            debug.is_some() && !self.finite_cost_vectors(input, params, false),
            self.circles(),
        );
        if let Some(debug) = &mut debug {
            debug.reset();
        }
        let mut result = VelocitySample::default();
        let p = *params;
        let cx = input.desired_velocity[0] * p.velocity_bias;
        let cz = input.desired_velocity[2] * p.velocity_bias;
        let step =
            input.maximum_speed * 2. * (1. - p.velocity_bias) / (i32::from(p.grid_size) - 1) as f32;
        let half = (i32::from(p.grid_size) - 1) as f32 * step * 0.5;
        let mut minimum = f32::MAX;
        for z in 0..p.grid_size {
            for x in 0..p.grid_size {
                let candidate = [
                    madd(x as f32, step, cx) - half,
                    0.,
                    madd(z as f32, step, cz) - half,
                ];
                let limit = input.maximum_speed + step / 2.;
                if candidate[0] * candidate[0] + candidate[2] * candidate[2] > limit * limit {
                    continue;
                }
                let penalty = self.process_sample(candidate, step, &context, minimum, &mut debug);
                result.sample_count += 1;
                if penalty < minimum {
                    minimum = penalty;
                    result.velocity = candidate;
                }
            }
        }
        result
    }
    /// Checks native uninitialized-pattern reads and pattern-array overflow.
    /// In particular, even clamped division counts with positive depth leave
    /// an uninitialized slot in the pinned native implementation.
    pub fn sample_velocity_adaptive(
        &mut self,
        input: VelocityInput,
        params: &AvoidanceParams,
        mut debug: Option<&mut AvoidanceDebugData>,
    ) -> Result<VelocitySample, u32> {
        self.prepare(input.position, input.desired_velocity);
        let context = Sampling::new(
            input,
            *params,
            debug.is_some() && !self.finite_cost_vectors(input, params, true),
            self.circles(),
        );
        if let Some(debug) = &mut debug {
            debug.reset();
        }
        let p = *params;
        let nd = i32::from(p.adaptive_divisions).clamp(1, 32);
        let nr = i32::from(p.adaptive_rings).clamp(1, 4);
        let [ca, sa, c, s] = self.pattern_rotation(nd);
        let mut direction = input.desired_velocity;
        let length = dot(direction, direction).sqrt();
        if length != 0. {
            let inverse = 1. / length;
            direction[0] *= inverse;
            direction[2] *= inverse;
        }
        let rotated = [
            msub(direction[0], c, direction[2] * s),
            direction[1],
            pattern_add(direction[0], s, direction[2] * c),
        ];
        let directions = [direction, rotated];
        let mut pattern = [[0.; 2]; 129];
        let mut count = 1;
        for j in 0..nr {
            let radius = (nr - j) as f32 / nr as f32;
            let dir = directions[(j % 2) as usize];
            pattern[count] = [dir[0] * radius, dir[2] * radius];
            let mut last1 = count;
            let mut last2 = count;
            count += 1;
            for _ in (1..nd - 1).step_by(2) {
                let a = pattern[last1];
                let b = pattern[last2];
                pattern[count] = [
                    pattern_add(a[0], ca, a[1] * sa),
                    pattern_add(-a[0], sa, a[1] * ca),
                ];
                pattern[count + 1] = [msub(b[0], ca, b[1] * sa), pattern_add(b[0], sa, b[1] * ca)];
                last1 = count;
                last2 = count + 1;
                count += 2;
            }
            if nd & 1 == 0 {
                let b = pattern[last2];
                let Some(slot) = pattern.get_mut(count + 1) else {
                    return Err(FAILURE | INVALID_PARAM);
                };
                *slot = [msub(b[0], ca, b[1] * sa), pattern_add(b[0], sa, b[1] * ca)];
                count += 1;
            }
        }
        // Odd division counts initialize the whole pattern. The native even
        // branch skips slot nd in the first ring; later rings never fill it.
        // Process exactly the initialized prefix before reporting that read,
        // retaining partial debug samples and depth-zero behavior.
        let initialized_count = if nd & 1 == 0 { nd as usize } else { count };
        let mut radius = input.maximum_speed * (1. - p.velocity_bias);
        let mut result = VelocitySample {
            velocity: [
                input.desired_velocity[0] * p.velocity_bias,
                0.,
                input.desired_velocity[2] * p.velocity_bias,
            ],
            sample_count: 0,
        };
        for _ in 0..p.adaptive_depth {
            let mut minimum = f32::MAX;
            let mut best = [0.; 3];
            for point in &pattern[..initialized_count] {
                let candidate = [
                    offset(result.velocity[0], point[0], radius),
                    0.,
                    offset(result.velocity[2], point[1], radius),
                ];
                let limit = input.maximum_speed + 0.001;
                if candidate[0] * candidate[0] + candidate[2] * candidate[2] > limit * limit {
                    continue;
                }
                let penalty =
                    self.process_sample(candidate, radius / 10., &context, minimum, &mut debug);
                result.sample_count += 1;
                if penalty < minimum {
                    minimum = penalty;
                    best = candidate;
                }
            }
            if initialized_count < count {
                return Err(FAILURE | INVALID_PARAM);
            }
            result.velocity = best;
            radius *= 0.5;
        }
        Ok(result)
    }
}
