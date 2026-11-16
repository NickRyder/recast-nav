// Altered Rust translation of rcContext and rcScopedTimer, Recast 1.6.0.
// Copyright (c) 2009-2010 Mikko Mononen. Zlib: vendor/recastnavigation/License.txt.
//! Caller-owned diagnostic hooks. Timer values come from the caller, as in Recast.
use core::{cell::Cell, fmt};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogCategory(pub i32);
impl LogCategory {
    pub const PROGRESS: Self = Self(1);
    pub const WARNING: Self = Self(2);
    pub const ERROR: Self = Self(3);
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TimerLabel(pub i32);
impl TimerLabel {
    pub const TOTAL: Self = Self(0);
    pub const TEMP: Self = Self(1);
    pub const RASTERIZE_TRIANGLES: Self = Self(2);
    pub const BUILD_COMPACTHEIGHTFIELD: Self = Self(3);
    pub const BUILD_CONTOURS: Self = Self(4);
    pub const BUILD_CONTOURS_TRACE: Self = Self(5);
    pub const BUILD_CONTOURS_SIMPLIFY: Self = Self(6);
    pub const FILTER_BORDER: Self = Self(7);
    pub const FILTER_WALKABLE: Self = Self(8);
    pub const MEDIAN_AREA: Self = Self(9);
    pub const FILTER_LOW_OBSTACLES: Self = Self(10);
    pub const BUILD_POLYMESH: Self = Self(11);
    pub const MERGE_POLYMESH: Self = Self(12);
    pub const ERODE_AREA: Self = Self(13);
    pub const MARK_BOX_AREA: Self = Self(14);
    pub const MARK_CYLINDER_AREA: Self = Self(15);
    pub const MARK_CONVEXPOLY_AREA: Self = Self(16);
    pub const BUILD_DISTANCEFIELD: Self = Self(17);
    pub const BUILD_DISTANCEFIELD_DIST: Self = Self(18);
    pub const BUILD_DISTANCEFIELD_BLUR: Self = Self(19);
    pub const BUILD_REGIONS: Self = Self(20);
    pub const BUILD_REGIONS_WATERSHED: Self = Self(21);
    pub const BUILD_REGIONS_EXPAND: Self = Self(22);
    pub const BUILD_REGIONS_FLOOD: Self = Self(23);
    pub const BUILD_REGIONS_FILTER: Self = Self(24);
    pub const BUILD_LAYERS: Self = Self(25);
    pub const BUILD_POLYMESHDETAIL: Self = Self(26);
    pub const MERGE_POLYMESHDETAIL: Self = Self(27);
}
pub const TIMER_COUNT: usize = 28;

/// Equivalent to the protected native context callbacks. Use interior
/// mutability in the sink when recording state; callbacks can be reentrant.
pub trait ContextHooks {
    fn reset_log(&self) {}
    fn log(&self, _category: LogCategory, _message: &[u8]) {}
    fn reset_timers(&self) {}
    fn start_timer(&self, _label: TimerLabel) {}
    fn stop_timer(&self, _label: TimerLabel) {}
    fn accumulated_time(&self, _label: TimerLabel) -> i32 {
        -1
    }
}
impl ContextHooks for () {}

/// Native enable/disable gates around a borrowed caller-owned hook sink.
pub struct Context<'a> {
    log_enabled: Cell<bool>,
    timer_enabled: Cell<bool>,
    hooks: &'a dyn ContextHooks,
}
impl Default for Context<'static> {
    fn default() -> Self {
        Self::new(true)
    }
}
impl Context<'static> {
    pub const fn new(enabled: bool) -> Self {
        Self::with_hooks(enabled, &())
    }
}
impl<'a> Context<'a> {
    pub const fn with_hooks(enabled: bool, hooks: &'a dyn ContextHooks) -> Self {
        Self {
            log_enabled: Cell::new(enabled),
            timer_enabled: Cell::new(enabled),
            hooks,
        }
    }
    pub fn enable_log(&self, enabled: bool) {
        self.log_enabled.set(enabled);
    }
    pub fn reset_log(&self) {
        if self.log_enabled.get() {
            self.hooks.reset_log();
        }
    }
    /// Log an already formatted byte message. Native truncation is byte-based,
    /// and emits an error before the truncated message when it exceeds 511 bytes.
    pub fn log_bytes(&self, category: LogCategory, message: &[u8]) {
        if self.log_enabled.get() {
            self.emit(
                category,
                &message[..message.len().min(511)],
                message.len() > 511,
            );
        }
    }
    pub fn log(&self, category: LogCategory, message: &str) {
        self.log_bytes(category, message.as_bytes());
    }
    /// Rust formatting replaces native C varargs. Disabled logs do not format
    /// their arguments. The formatter uses the same bounded stack buffer.
    pub fn log_format(&self, category: LogCategory, arguments: fmt::Arguments<'_>) -> fmt::Result {
        if !self.log_enabled.get() {
            return Ok(());
        }
        let mut buffer = MessageBuffer {
            bytes: [0; 511],
            length: 0,
            truncated: false,
        };
        fmt::write(&mut buffer, arguments)?;
        self.emit(category, &buffer.bytes[..buffer.length], buffer.truncated);
        Ok(())
    }
    fn emit(&self, category: LogCategory, message: &[u8], truncated: bool) {
        if truncated {
            self.hooks
                .log(LogCategory::ERROR, b"Log message was truncated");
        }
        self.hooks.log(category, message);
    }
    pub fn enable_timer(&self, enabled: bool) {
        self.timer_enabled.set(enabled);
    }
    pub fn reset_timers(&self) {
        if self.timer_enabled.get() {
            self.hooks.reset_timers();
        }
    }
    pub fn start_timer(&self, label: TimerLabel) {
        if self.timer_enabled.get() {
            self.hooks.start_timer(label);
        }
    }
    pub fn stop_timer(&self, label: TimerLabel) {
        if self.timer_enabled.get() {
            self.hooks.stop_timer(label);
        }
    }
    pub fn accumulated_time(&self, label: TimerLabel) -> i32 {
        if self.timer_enabled.get() {
            self.hooks.accumulated_time(label)
        } else {
            -1
        }
    }
    pub fn scoped_timer(&self, label: TimerLabel) -> ScopedTimer<'_, 'a> {
        self.start_timer(label);
        ScopedTimer {
            context: self,
            label,
        }
    }
}

struct MessageBuffer {
    bytes: [u8; 511],
    length: usize,
    truncated: bool,
}
impl fmt::Write for MessageBuffer {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        let length = text.len().min(self.bytes.len() - self.length);
        self.bytes[self.length..self.length + length].copy_from_slice(&text.as_bytes()[..length]);
        self.length += length;
        self.truncated |= length < text.len();
        Ok(())
    }
}

#[must_use]
pub struct ScopedTimer<'a, 'hooks> {
    context: &'a Context<'hooks>,
    label: TimerLabel,
}
impl Drop for ScopedTimer<'_, '_> {
    fn drop(&mut self) {
        self.context.stop_timer(self.label);
    }
}
