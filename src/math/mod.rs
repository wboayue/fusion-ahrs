//! Math types and utilities for the Fusion AHRS library
//!
//! [`Vector`], [`Quaternion`], [`Matrix`], and [`Euler`] mirror the C
//! library's `FusionMath.h` types and operations.

mod euler;
mod matrix;
mod quaternion;
mod vector;

pub use euler::Euler;
pub use matrix::Matrix;
pub use quaternion::Quaternion;
pub use vector::Vector;

// Mathematical constants for angle conversion

/// Conversion factor from degrees to radians
///
/// # Example
/// ```
/// use fusion_ahrs::DEG_TO_RAD;
///
/// let angle_deg = 45.0;
/// let angle_rad = angle_deg * DEG_TO_RAD;
/// assert!((angle_rad - std::f32::consts::FRAC_PI_4).abs() < 1e-6);
/// ```
pub const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;

/// Conversion factor from radians to degrees
///
/// # Example
/// ```
/// use fusion_ahrs::RAD_TO_DEG;
///
/// let angle_rad = std::f32::consts::FRAC_PI_2;
/// let angle_deg = angle_rad * RAD_TO_DEG;
/// assert!((angle_deg - 90.0).abs() < 1e-6);
/// ```
pub const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;

/// Arc sine that clamps its input to [-1, 1], as C's `FusionArcSin`
#[inline]
pub(crate) fn arc_sin(value: f32) -> f32 {
    if value <= -1.0 {
        return core::f32::consts::PI / -2.0;
    }
    if value >= 1.0 {
        return core::f32::consts::PI / 2.0;
    }
    libm::asinf(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_arc_sin_clamps() {
        assert_eq!(arc_sin(1.5), core::f32::consts::FRAC_PI_2);
        assert_eq!(arc_sin(-1.5), -core::f32::consts::FRAC_PI_2);
        assert_eq!(arc_sin(0.0), 0.0);
    }
}
