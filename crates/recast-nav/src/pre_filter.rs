// Altered Rust translation: native signed filter parameters and ledge comparisons.
use crate::{
    heightfield::Heightfield,
    math::{dir_offset_x, dir_offset_z},
    span::{AreaType, Span},
};

impl Heightfield {
    pub fn filter_low_hanging_walkable_obstacles_with_context(
        &mut self,
        context: &crate::context::Context<'_>,
        walkable_climb: i32,
    ) {
        let _timer = context.scoped_timer(crate::context::TimerLabel::FILTER_LOW_OBSTACLES);
        self.filter_low_hanging_walkable_obstacles(walkable_climb);
    }

    pub fn filter_ledge_spans_with_context(
        &mut self,
        context: &crate::context::Context<'_>,
        walkable_height: i32,
        walkable_climb: i32,
    ) -> Result<(), FilterError> {
        let _timer = context.scoped_timer(crate::context::TimerLabel::FILTER_BORDER);
        self.filter_ledge_spans(walkable_height, walkable_climb)
    }

    pub fn filter_walkable_low_height_spans_with_context(
        &mut self,
        context: &crate::context::Context<'_>,
        walkable_height: i32,
    ) {
        let _timer = context.scoped_timer(crate::context::TimerLabel::FILTER_WALKABLE);
        self.filter_walkable_low_height_spans(walkable_height);
    }

    /// Adds the walkable flag to spans which are adjacent to a walkable span and the height difference is small enough for the agent to walk over.
    pub fn filter_low_hanging_walkable_obstacles(&mut self, walkable_climb: i32) {
        for z in 0..self.height {
            for x in 0..self.width {
                let mut previous_span: Option<Span> = None;
                let mut previous_was_walkable = false;
                let mut previous_area_id = AreaType::NOT_WALKABLE;
                let mut span = self.span_at_mut(x, z);

                // For each span in the column...
                while let Some(current_span) = span {
                    let walkable = current_span.area().is_walkable();

                    // If current span is not walkable, but there is walkable span just below it and the height difference
                    // is small enough for the agent to walk over, mark the current span as walkable too.
                    if let Some(previous_span) = previous_span.as_ref()
                        && !walkable
                        && previous_was_walkable
                        && (current_span.max() as i32 - previous_span.max() as i32).abs()
                            <= walkable_climb
                    {
                        current_span.set_area(previous_area_id);
                    }

                    // Copy the original walkable value regardless of whether we changed it.
                    // This prevents multiple consecutive non-walkable spans from being erroneously marked as walkable.
                    previous_was_walkable = walkable;
                    previous_area_id = current_span.area();
                    previous_span.replace(current_span.clone());
                    span = current_span.next.map(|key| self.span_mut(key));
                }
            }
        }
    }

    /// Removes spans adjacent to drops or steep accessible neighbor floors.
    /// Returns a checked error where native signed height subtraction overflows.
    pub fn filter_ledge_spans(
        &mut self,
        walkable_height: i32,
        walkable_climb: i32,
    ) -> Result<(), FilterError> {
        for z in 0..self.height {
            for x in 0..self.width {
                let mut key = self.span_key_at(x, z);
                if key.is_none() {
                    continue;
                }
                // Only area flags change during filtering. Resolve column heads
                // once and retain the native direction order for every span.
                let neighbors = core::array::from_fn::<_, 4, _>(|direction| {
                    let nx = x + i32::from(dir_offset_x(direction as u8));
                    let nz = z + i32::from(dir_offset_z(direction as u8));
                    self.contains(nx, nz).then(|| self.span_key_at(nx, nz))
                });
                while let Some(current) = key {
                    let span = self.span(current);
                    key = span.next;
                    if !span.area().is_walkable() {
                        continue;
                    }
                    let bottom = i32::from(span.max());
                    let top = span.next.map_or(65535, |k| i32::from(self.span(k).min()));
                    let negative_climb = walkable_climb
                        .checked_neg()
                        .ok_or(FilterError::HeightOverflow)?;
                    let mut minimum = 65535;
                    let mut accessible_min = bottom;
                    let mut accessible_max = bottom;
                    for neighbor_key in neighbors {
                        let Some(neighbor_key) = neighbor_key else {
                            minimum = minimum.min(
                                negative_climb
                                    .checked_sub(bottom)
                                    .ok_or(FilterError::HeightOverflow)?,
                            );
                            continue;
                        };
                        let mut neighbor = neighbor_key.map(|k| self.span(k));
                        let mut neighbor_bottom = negative_climb;
                        let mut neighbor_top = neighbor.map_or(65535, |s| i32::from(s.min()));
                        if top
                            .min(neighbor_top)
                            .checked_sub(bottom.max(neighbor_bottom))
                            .ok_or(FilterError::HeightOverflow)?
                            > walkable_height
                        {
                            minimum = minimum.min(
                                neighbor_bottom
                                    .checked_sub(bottom)
                                    .ok_or(FilterError::HeightOverflow)?,
                            );
                        }
                        while let Some(other) = neighbor {
                            neighbor_bottom = i32::from(other.max());
                            let next = other.next.map(|k| self.span(k));
                            neighbor_top = next.map_or(65535, |s| i32::from(s.min()));
                            if top.min(neighbor_top) - bottom.max(neighbor_bottom) > walkable_height
                            {
                                minimum = minimum.min(neighbor_bottom - bottom);
                                if (neighbor_bottom - bottom).abs() <= walkable_climb {
                                    accessible_min = accessible_min.min(neighbor_bottom);
                                    accessible_max = accessible_max.max(neighbor_bottom);
                                }
                            }
                            neighbor = next;
                        }
                    }
                    if minimum < negative_climb || accessible_max - accessible_min > walkable_climb
                    {
                        self.span_mut(current).set_area(AreaType::NOT_WALKABLE);
                    }
                }
            }
        }
        Ok(())
    }

    /// Taken 1:1 from the original implementation.
    const MAX_HEIGHTFIELD_HEIGHT: u16 = u16::MAX;

    /// Removes the walkable flag from spans which do not have enough space above them for the agent to stand there.
    pub fn filter_walkable_low_height_spans(&mut self, walkable_height: i32) {
        // Remove walkable flag from spans which do not have enough
        // space above them for the agent to stand there.
        for z in 0..self.height {
            for x in 0..self.width {
                let mut span_key = self.span_key_at(x, z);
                while let Some(current_span_key) = span_key {
                    let filtered: bool = {
                        let span = self.span(current_span_key);
                        let floor = span.max() as i32;
                        let ceiling = span
                            .next
                            .map(|key| self.span(key).min() as i32)
                            .unwrap_or(Self::MAX_HEIGHTFIELD_HEIGHT as i32);
                        ceiling - floor < walkable_height
                    };

                    let span = self.span_mut(current_span_key);
                    if filtered {
                        span.set_area(AreaType::NOT_WALKABLE);
                    }

                    span_key = span.next;
                }
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FilterError {
    HeightOverflow,
}

impl core::fmt::Display for FilterError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::HeightOverflow => f.write_str("native ledge height arithmetic overflows"),
        }
    }
}
impl core::error::Error for FilterError {}
