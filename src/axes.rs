//! Sensor axes alignment for different mounting orientations
//!
//! This module provides functionality to remap sensor axes to body axes when
//! the sensor is mounted in a different orientation than the default.
//!
//! # Example
//! ```
//! use fusion_ahrs::Vector;
//! use fusion_ahrs::{AxesAlignment, axes_swap};
//!
//! // Sensor reading in sensor frame
//! let sensor = Vector::new(1.0, 2.0, 3.0);
//!
//! // Remap axes for a sensor mounted with Y pointing forward, X pointing right
//! let body = axes_swap(sensor, AxesAlignment::PyNxPz);
//!
//! assert_eq!(body.x, 2.0);   // Body X = Sensor Y
//! assert_eq!(body.y, -1.0);  // Body Y = -Sensor X
//! assert_eq!(body.z, 3.0);   // Body Z = Sensor Z
//! ```

use core::fmt;

use crate::math::Vector;

/// Axes alignment describing the sensor axes relative to the body axes.
///
/// Each variant name describes where each body axis comes from in sensor
/// coordinates. The three letter-pairs specify the source for body X, Y, Z
/// respectively.
///
/// For example, `PyNxPz` means:
/// - Body X = +Sensor Y (first pair: Py)
/// - Body Y = -Sensor X (second pair: Nx)
/// - Body Z = +Sensor Z (third pair: Pz)
///
/// The naming convention uses:
/// - `P` = Positive (same direction)
/// - `N` = Negative (inverted direction)
/// - `x`, `y`, `z` = which sensor axis to use
///
/// # Example
/// ```
/// use fusion_ahrs::Vector;
/// use fusion_ahrs::{AxesAlignment, axes_swap};
///
/// let sensor = Vector::new(1.0, 0.0, 0.0);
///
/// // Identity alignment - no change
/// let result = axes_swap(sensor, AxesAlignment::PxPyPz);
/// assert_eq!(result, sensor);
///
/// // Swap X and Y, negate X
/// let result = axes_swap(sensor, AxesAlignment::PyNxPz);
/// assert_eq!(result, Vector::new(0.0, -1.0, 0.0));
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(non_camel_case_types)]
pub enum AxesAlignment {
    /// +X+Y+Z (identity - no remapping)
    #[default]
    PxPyPz,
    /// +X-Z+Y
    PxNzPy,
    /// +X-Y-Z
    PxNyNz,
    /// +X+Z-Y
    PxPzNy,
    /// -X+Y-Z
    NxPyNz,
    /// -X+Z+Y
    NxPzPy,
    /// -X-Y+Z
    NxNyPz,
    /// -X-Z-Y
    NxNzNy,
    /// +Y-X+Z
    PyNxPz,
    /// +Y-Z-X
    PyNzNx,
    /// +Y+X-Z
    PyPxNz,
    /// +Y+Z+X
    PyPzPx,
    /// -Y+X+Z
    NyPxPz,
    /// -Y-Z+X
    NyNzPx,
    /// -Y-X-Z
    NyNxNz,
    /// -Y+Z-X
    NyPzNx,
    /// +Z+Y-X
    PzPyNx,
    /// +Z+X+Y
    PzPxPy,
    /// +Z-Y+X
    PzNyPx,
    /// +Z-X-Y
    PzNxNy,
    /// -Z+Y+X
    NzPyPx,
    /// -Z-X+Y
    NzNxPy,
    /// -Z-Y-X
    NzNyNx,
    /// -Z+X-Y
    NzPxNy,
}

impl AxesAlignment {
    /// All 24 alignments, in declaration order.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    /// use fusion_ahrs::{AxesAlignment, axes_swap};
    ///
    /// // Find the alignment that maps sensor (1, 2, 3) onto body (3, 1, 2)
    /// let sensor = Vector::new(1.0, 2.0, 3.0);
    /// let alignment = AxesAlignment::ALL
    ///     .into_iter()
    ///     .find(|&a| axes_swap(sensor, a) == Vector::new(3.0, 1.0, 2.0))
    ///     .unwrap();
    /// assert_eq!(alignment.as_str(), "+Z+X+Y");
    /// ```
    pub const ALL: [AxesAlignment; 24] = [
        AxesAlignment::PxPyPz,
        AxesAlignment::PxNzPy,
        AxesAlignment::PxNyNz,
        AxesAlignment::PxPzNy,
        AxesAlignment::NxPyNz,
        AxesAlignment::NxPzPy,
        AxesAlignment::NxNyPz,
        AxesAlignment::NxNzNy,
        AxesAlignment::PyNxPz,
        AxesAlignment::PyNzNx,
        AxesAlignment::PyPxNz,
        AxesAlignment::PyPzPx,
        AxesAlignment::NyPxPz,
        AxesAlignment::NyNzPx,
        AxesAlignment::NyNxNz,
        AxesAlignment::NyPzNx,
        AxesAlignment::PzPyNx,
        AxesAlignment::PzPxPy,
        AxesAlignment::PzNyPx,
        AxesAlignment::PzNxNy,
        AxesAlignment::NzPyPx,
        AxesAlignment::NzNxPy,
        AxesAlignment::NzNyNx,
        AxesAlignment::NzPxNy,
    ];

    /// Returns the alignment as a string, e.g. `"+Y-X+Z"`, matching the C
    /// library's `FusionRemapAlignmentToString`.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::AxesAlignment;
    ///
    /// assert_eq!(AxesAlignment::PyNxPz.as_str(), "+Y-X+Z");
    /// assert_eq!(AxesAlignment::PyNxPz.to_string(), "+Y-X+Z");
    /// ```
    pub const fn as_str(self) -> &'static str {
        match self {
            AxesAlignment::PxPyPz => "+X+Y+Z",
            AxesAlignment::PxNzPy => "+X-Z+Y",
            AxesAlignment::PxNyNz => "+X-Y-Z",
            AxesAlignment::PxPzNy => "+X+Z-Y",
            AxesAlignment::NxPyNz => "-X+Y-Z",
            AxesAlignment::NxPzPy => "-X+Z+Y",
            AxesAlignment::NxNyPz => "-X-Y+Z",
            AxesAlignment::NxNzNy => "-X-Z-Y",
            AxesAlignment::PyNxPz => "+Y-X+Z",
            AxesAlignment::PyNzNx => "+Y-Z-X",
            AxesAlignment::PyPxNz => "+Y+X-Z",
            AxesAlignment::PyPzPx => "+Y+Z+X",
            AxesAlignment::NyPxPz => "-Y+X+Z",
            AxesAlignment::NyNzPx => "-Y-Z+X",
            AxesAlignment::NyNxNz => "-Y-X-Z",
            AxesAlignment::NyPzNx => "-Y+Z-X",
            AxesAlignment::PzPyNx => "+Z+Y-X",
            AxesAlignment::PzPxPy => "+Z+X+Y",
            AxesAlignment::PzNyPx => "+Z-Y+X",
            AxesAlignment::PzNxNy => "+Z-X-Y",
            AxesAlignment::NzPyPx => "-Z+Y+X",
            AxesAlignment::NzNxPy => "-Z-X+Y",
            AxesAlignment::NzNyNx => "-Z-Y-X",
            AxesAlignment::NzPxNy => "-Z+X-Y",
        }
    }
}

impl fmt::Display for AxesAlignment {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Swaps sensor axes for alignment with the body axes.
///
/// Use this function to remap sensor readings when the sensor is mounted
/// in a different orientation than the default body frame.
///
/// # Arguments
/// * `sensor` - Sensor measurement in sensor frame
/// * `alignment` - Axes alignment describing sensor orientation
///
/// # Returns
/// Sensor measurement remapped to body frame
///
/// # Example
/// ```
/// use fusion_ahrs::Vector;
/// use fusion_ahrs::{AxesAlignment, axes_swap};
///
/// // Gyroscope reading from a rotated sensor
/// let gyro_sensor = Vector::new(10.0, 20.0, 30.0);
///
/// // Sensor is mounted with Z pointing forward, X pointing up
/// let gyro_body = axes_swap(gyro_sensor, AxesAlignment::PzPyNx);
///
/// // Now gyro_body is in the correct body frame orientation
/// ```
#[inline]
pub fn axes_swap(sensor: impl Into<Vector>, alignment: AxesAlignment) -> Vector {
    let sensor = sensor.into();
    match alignment {
        AxesAlignment::PxPyPz => sensor,
        AxesAlignment::PxNzPy => Vector::new(sensor.x, -sensor.z, sensor.y),
        AxesAlignment::PxNyNz => Vector::new(sensor.x, -sensor.y, -sensor.z),
        AxesAlignment::PxPzNy => Vector::new(sensor.x, sensor.z, -sensor.y),
        AxesAlignment::NxPyNz => Vector::new(-sensor.x, sensor.y, -sensor.z),
        AxesAlignment::NxPzPy => Vector::new(-sensor.x, sensor.z, sensor.y),
        AxesAlignment::NxNyPz => Vector::new(-sensor.x, -sensor.y, sensor.z),
        AxesAlignment::NxNzNy => Vector::new(-sensor.x, -sensor.z, -sensor.y),
        AxesAlignment::PyNxPz => Vector::new(sensor.y, -sensor.x, sensor.z),
        AxesAlignment::PyNzNx => Vector::new(sensor.y, -sensor.z, -sensor.x),
        AxesAlignment::PyPxNz => Vector::new(sensor.y, sensor.x, -sensor.z),
        AxesAlignment::PyPzPx => Vector::new(sensor.y, sensor.z, sensor.x),
        AxesAlignment::NyPxPz => Vector::new(-sensor.y, sensor.x, sensor.z),
        AxesAlignment::NyNzPx => Vector::new(-sensor.y, -sensor.z, sensor.x),
        AxesAlignment::NyNxNz => Vector::new(-sensor.y, -sensor.x, -sensor.z),
        AxesAlignment::NyPzNx => Vector::new(-sensor.y, sensor.z, -sensor.x),
        AxesAlignment::PzPyNx => Vector::new(sensor.z, sensor.y, -sensor.x),
        AxesAlignment::PzPxPy => Vector::new(sensor.z, sensor.x, sensor.y),
        AxesAlignment::PzNyPx => Vector::new(sensor.z, -sensor.y, sensor.x),
        AxesAlignment::PzNxNy => Vector::new(sensor.z, -sensor.x, -sensor.y),
        AxesAlignment::NzPyPx => Vector::new(-sensor.z, sensor.y, sensor.x),
        AxesAlignment::NzNxPy => Vector::new(-sensor.z, -sensor.x, sensor.y),
        AxesAlignment::NzNyNx => Vector::new(-sensor.z, -sensor.y, -sensor.x),
        AxesAlignment::NzPxNy => Vector::new(-sensor.z, sensor.x, -sensor.y),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity_alignment() {
        let sensor = Vector::new(1.0, 2.0, 3.0);
        let result = axes_swap(sensor, AxesAlignment::PxPyPz);
        assert_eq!(result, sensor);
    }

    #[test]
    fn test_all_alignments_preserve_magnitude() {
        let sensor = Vector::new(1.0, 2.0, 3.0);
        let original_magnitude = sensor.norm();

        let alignments = [
            AxesAlignment::PxPyPz,
            AxesAlignment::PxNzPy,
            AxesAlignment::PxNyNz,
            AxesAlignment::PxPzNy,
            AxesAlignment::NxPyNz,
            AxesAlignment::NxPzPy,
            AxesAlignment::NxNyPz,
            AxesAlignment::NxNzNy,
            AxesAlignment::PyNxPz,
            AxesAlignment::PyNzNx,
            AxesAlignment::PyPxNz,
            AxesAlignment::PyPzPx,
            AxesAlignment::NyPxPz,
            AxesAlignment::NyNzPx,
            AxesAlignment::NyNxNz,
            AxesAlignment::NyPzNx,
            AxesAlignment::PzPyNx,
            AxesAlignment::PzPxPy,
            AxesAlignment::PzNyPx,
            AxesAlignment::PzNxNy,
            AxesAlignment::NzPyPx,
            AxesAlignment::NzNxPy,
            AxesAlignment::NzNyNx,
            AxesAlignment::NzPxNy,
        ];

        for alignment in alignments {
            let result = axes_swap(sensor, alignment);
            let result_magnitude = result.norm();
            assert!(
                (result_magnitude - original_magnitude).abs() < 1e-6,
                "Alignment {:?} changed magnitude from {} to {}",
                alignment,
                original_magnitude,
                result_magnitude
            );
        }
    }

    #[test]
    fn test_specific_alignments() {
        let sensor = Vector::new(1.0, 2.0, 3.0);

        // +X-Z+Y: x'=x, y'=-z, z'=y
        let result = axes_swap(sensor, AxesAlignment::PxNzPy);
        assert_eq!(result, Vector::new(1.0, -3.0, 2.0));

        // +Y-X+Z: x'=y, y'=-x, z'=z
        let result = axes_swap(sensor, AxesAlignment::PyNxPz);
        assert_eq!(result, Vector::new(2.0, -1.0, 3.0));

        // -X-Y+Z: x'=-x, y'=-y, z'=z
        let result = axes_swap(sensor, AxesAlignment::NxNyPz);
        assert_eq!(result, Vector::new(-1.0, -2.0, 3.0));

        // +Z+X+Y: x'=z, y'=x, z'=y
        let result = axes_swap(sensor, AxesAlignment::PzPxPy);
        assert_eq!(result, Vector::new(3.0, 1.0, 2.0));
    }

    #[test]
    fn test_zero_vector() {
        let sensor = Vector::ZERO;
        for alignment in [
            AxesAlignment::PxPyPz,
            AxesAlignment::PyNxPz,
            AxesAlignment::NzNyNx,
        ] {
            let result = axes_swap(sensor, alignment);
            assert_eq!(result, Vector::ZERO);
        }
    }

    #[test]
    fn test_unit_vectors() {
        let x = Vector::new(1.0, 0.0, 0.0);
        let y = Vector::new(0.0, 1.0, 0.0);
        let z = Vector::new(0.0, 0.0, 1.0);

        // PyNxPz: x'=y, y'=-x, z'=z
        assert_eq!(
            axes_swap(x, AxesAlignment::PyNxPz),
            Vector::new(0.0, -1.0, 0.0)
        );
        assert_eq!(
            axes_swap(y, AxesAlignment::PyNxPz),
            Vector::new(1.0, 0.0, 0.0)
        );
        assert_eq!(axes_swap(z, AxesAlignment::PyNxPz), z);
    }

    #[test]
    fn test_inverse_round_trip() {
        // Verify that applying an alignment and its inverse returns original
        // These are known inverse pairs from the rotation group
        let inverse_pairs = [
            (AxesAlignment::PxPyPz, AxesAlignment::PxPyPz), // identity
            (AxesAlignment::PyNxPz, AxesAlignment::NyPxPz), // 90° about Z
            (AxesAlignment::NxNyPz, AxesAlignment::NxNyPz), // 180° about Z (self-inverse)
            (AxesAlignment::PxNzPy, AxesAlignment::PxPzNy), // 90° about X
            (AxesAlignment::PzPyNx, AxesAlignment::NzPyPx), // 90° about Y
        ];

        let test_vectors = [
            Vector::new(1.0, 2.0, 3.0),
            Vector::new(-5.0, 0.0, 7.0),
            Vector::new(0.1, -0.2, 0.3),
        ];

        for (forward, inverse) in inverse_pairs {
            for &v in &test_vectors {
                let transformed = axes_swap(v, forward);
                let recovered = axes_swap(transformed, inverse);
                assert!(
                    (recovered - v).norm() < 1e-6,
                    "Round-trip failed for {:?}: {:?} -> {:?} -> {:?}",
                    forward,
                    v,
                    transformed,
                    recovered
                );
            }
        }
    }

    #[test]
    fn test_display() {
        assert_eq!(AxesAlignment::PxPyPz.as_str(), "+X+Y+Z");
        assert_eq!(AxesAlignment::NzPxNy.as_str(), "-Z+X-Y");
    }
}
