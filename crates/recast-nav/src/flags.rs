// Shared operations for the three native integer flag types.
macro_rules! impl_flag_ops {
    ($name:ident, $bits:ty) => {
        impl $name {
            /// No bits set.
            #[inline]
            pub const fn empty() -> Self {
                Self(0)
            }

            /// Preserve all bits, including flags unknown to this version.
            #[inline]
            pub const fn from_bits_retain(bits: $bits) -> Self {
                Self(bits)
            }

            /// The underlying native bit pattern.
            #[inline]
            pub const fn bits(self) -> $bits {
                self.0
            }

            /// Whether every bit in `other` is set.
            #[inline]
            pub const fn contains(self, other: Self) -> bool {
                self.0 & other.0 == other.0
            }

            /// Whether any bit in `other` is set.
            #[inline]
            pub const fn intersects(self, other: Self) -> bool {
                self.0 & other.0 != 0
            }
        }

        impl core::ops::BitAnd for $name {
            type Output = Self;
            #[inline]
            fn bitand(self, other: Self) -> Self {
                Self(self.0 & other.0)
            }
        }

        impl core::ops::BitOr for $name {
            type Output = Self;
            #[inline]
            fn bitor(self, other: Self) -> Self {
                Self(self.0 | other.0)
            }
        }

        impl core::ops::BitOrAssign for $name {
            #[inline]
            fn bitor_assign(&mut self, other: Self) {
                self.0 |= other.0;
            }
        }
    };
}

pub(crate) use impl_flag_ops;
