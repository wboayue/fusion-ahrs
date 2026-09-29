//! Sensor axes remapping, mirroring the C library's `FusionRemap`
//!
//! This module provides functionality to remap sensor axes to body axes when
//! the sensor is mounted in a different orientation than the default.
//!
//! # Example
//! ```
//! use fusion_ahrs::Vector;
//! use fusion_ahrs::{RemapAlignment, remap};
//!
//! // Sensor reading in sensor frame
//! let sensor = Vector::new(1.0, 2.0, 3.0);
//!
//! // Remap axes for a sensor mounted with Y pointing forward, X pointing right
//! let body = remap(sensor, RemapAlignment::PyNxPz);
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
/// use fusion_ahrs::{RemapAlignment, remap};
///
/// let sensor = Vector::new(1.0, 0.0, 0.0);
///
/// // Identity alignment - no change
/// let result = remap(sensor, RemapAlignment::PxPyPz);
/// assert_eq!(result, sensor);
///
/// // Swap X and Y, negate X
/// let result = remap(sensor, RemapAlignment::PyNxPz);
/// assert_eq!(result, Vector::new(0.0, -1.0, 0.0));
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(non_camel_case_types)]
pub enum RemapAlignment {
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

impl RemapAlignment {
    /// All 24 alignments, in declaration order.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    /// use fusion_ahrs::{RemapAlignment, remap};
    ///
    /// // Find the alignment that maps sensor (1, 2, 3) onto body (3, 1, 2)
    /// let sensor = Vector::new(1.0, 2.0, 3.0);
    /// let alignment = RemapAlignment::ALL
    ///     .into_iter()
    ///     .find(|&a| remap(sensor, a) == Vector::new(3.0, 1.0, 2.0))
    ///     .unwrap();
    /// assert_eq!(alignment.as_str(), "+Z+X+Y");
    /// ```
    pub const ALL: [RemapAlignment; 24] = [
        RemapAlignment::PxPyPz,
        RemapAlignment::PxNzPy,
        RemapAlignment::PxNyNz,
        RemapAlignment::PxPzNy,
        RemapAlignment::NxPyNz,
        RemapAlignment::NxPzPy,
        RemapAlignment::NxNyPz,
        RemapAlignment::NxNzNy,
        RemapAlignment::PyNxPz,
        RemapAlignment::PyNzNx,
        RemapAlignment::PyPxNz,
        RemapAlignment::PyPzPx,
        RemapAlignment::NyPxPz,
        RemapAlignment::NyNzPx,
        RemapAlignment::NyNxNz,
        RemapAlignment::NyPzNx,
        RemapAlignment::PzPyNx,
        RemapAlignment::PzPxPy,
        RemapAlignment::PzNyPx,
        RemapAlignment::PzNxNy,
        RemapAlignment::NzPyPx,
        RemapAlignment::NzNxPy,
        RemapAlignment::NzNyNx,
        RemapAlignment::NzPxNy,
    ];

    /// Returns the alignment as a string, e.g. `"+Y-X+Z"`, matching the C
    /// library's `FusionRemapAlignmentToString`.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::RemapAlignment;
    ///
    /// assert_eq!(RemapAlignment::PyNxPz.as_str(), "+Y-X+Z");
    /// assert_eq!(RemapAlignment::PyNxPz.to_string(), "+Y-X+Z");
    /// ```
    pub const fn as_str(self) -> &'static str {
        match self {
            RemapAlignment::PxPyPz => "+X+Y+Z",
            RemapAlignment::PxNzPy => "+X-Z+Y",
            RemapAlignment::PxNyNz => "+X-Y-Z",
            RemapAlignment::PxPzNy => "+X+Z-Y",
            RemapAlignment::NxPyNz => "-X+Y-Z",
            RemapAlignment::NxPzPy => "-X+Z+Y",
            RemapAlignment::NxNyPz => "-X-Y+Z",
            RemapAlignment::NxNzNy => "-X-Z-Y",
            RemapAlignment::PyNxPz => "+Y-X+Z",
            RemapAlignment::PyNzNx => "+Y-Z-X",
            RemapAlignment::PyPxNz => "+Y+X-Z",
            RemapAlignment::PyPzPx => "+Y+Z+X",
            RemapAlignment::NyPxPz => "-Y+X+Z",
            RemapAlignment::NyNzPx => "-Y-Z+X",
            RemapAlignment::NyNxNz => "-Y-X-Z",
            RemapAlignment::NyPzNx => "-Y+Z-X",
            RemapAlignment::PzPyNx => "+Z+Y-X",
            RemapAlignment::PzPxPy => "+Z+X+Y",
            RemapAlignment::PzNyPx => "+Z-Y+X",
            RemapAlignment::PzNxNy => "+Z-X-Y",
            RemapAlignment::NzPyPx => "-Z+Y+X",
            RemapAlignment::NzNxPy => "-Z-X+Y",
            RemapAlignment::NzNyNx => "-Z-Y-X",
            RemapAlignment::NzPxNy => "-Z+X-Y",
        }
    }
}

impl fmt::Display for RemapAlignment {
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
/// use fusion_ahrs::{RemapAlignment, remap};
///
/// // Gyroscope reading from a rotated sensor
/// let gyro_sensor = Vector::new(10.0, 20.0, 30.0);
///
/// // Sensor is mounted with Z pointing forward, X pointing up
/// let gyro_body = remap(gyro_sensor, RemapAlignment::PzPyNx);
///
/// // Now gyro_body is in the correct body frame orientation
/// ```
#[inline]
pub fn remap(sensor: impl Into<Vector>, alignment: RemapAlignment) -> Vector {
    let sensor = sensor.into();
    match alignment {
        RemapAlignment::PxPyPz => sensor,
        RemapAlignment::PxNzPy => Vector::new(sensor.x, -sensor.z, sensor.y),
        RemapAlignment::PxNyNz => Vector::new(sensor.x, -sensor.y, -sensor.z),
        RemapAlignment::PxPzNy => Vector::new(sensor.x, sensor.z, -sensor.y),
        RemapAlignment::NxPyNz => Vector::new(-sensor.x, sensor.y, -sensor.z),
        RemapAlignment::NxPzPy => Vector::new(-sensor.x, sensor.z, sensor.y),
        RemapAlignment::NxNyPz => Vector::new(-sensor.x, -sensor.y, sensor.z),
        RemapAlignment::NxNzNy => Vector::new(-sensor.x, -sensor.z, -sensor.y),
        RemapAlignment::PyNxPz => Vector::new(sensor.y, -sensor.x, sensor.z),
        RemapAlignment::PyNzNx => Vector::new(sensor.y, -sensor.z, -sensor.x),
        RemapAlignment::PyPxNz => Vector::new(sensor.y, sensor.x, -sensor.z),
        RemapAlignment::PyPzPx => Vector::new(sensor.y, sensor.z, sensor.x),
        RemapAlignment::NyPxPz => Vector::new(-sensor.y, sensor.x, sensor.z),
        RemapAlignment::NyNzPx => Vector::new(-sensor.y, -sensor.z, sensor.x),
        RemapAlignment::NyNxNz => Vector::new(-sensor.y, -sensor.x, -sensor.z),
        RemapAlignment::NyPzNx => Vector::new(-sensor.y, sensor.z, -sensor.x),
        RemapAlignment::PzPyNx => Vector::new(sensor.z, sensor.y, -sensor.x),
        RemapAlignment::PzPxPy => Vector::new(sensor.z, sensor.x, sensor.y),
        RemapAlignment::PzNyPx => Vector::new(sensor.z, -sensor.y, sensor.x),
        RemapAlignment::PzNxNy => Vector::new(sensor.z, -sensor.x, -sensor.y),
        RemapAlignment::NzPyPx => Vector::new(-sensor.z, sensor.y, sensor.x),
        RemapAlignment::NzNxPy => Vector::new(-sensor.z, -sensor.x, sensor.y),
        RemapAlignment::NzNyNx => Vector::new(-sensor.z, -sensor.y, -sensor.x),
        RemapAlignment::NzPxNy => Vector::new(-sensor.z, sensor.x, -sensor.y),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity_alignment() {
        let sensor = Vector::new(1.0, 2.0, 3.0);
        let result = remap(sensor, RemapAlignment::PxPyPz);
        assert_eq!(result, sensor);
    }

    #[test]
    fn test_all_alignments_preserve_magnitude() {
        let sensor = Vector::new(1.0, 2.0, 3.0);
        let original_magnitude = sensor.norm();

        let alignments = [
            RemapAlignment::PxPyPz,
            RemapAlignment::PxNzPy,
            RemapAlignment::PxNyNz,
            RemapAlignment::PxPzNy,
            RemapAlignment::NxPyNz,
            RemapAlignment::NxPzPy,
            RemapAlignment::NxNyPz,
            RemapAlignment::NxNzNy,
            RemapAlignment::PyNxPz,
            RemapAlignment::PyNzNx,
            RemapAlignment::PyPxNz,
            RemapAlignment::PyPzPx,
            RemapAlignment::NyPxPz,
            RemapAlignment::NyNzPx,
            RemapAlignment::NyNxNz,
            RemapAlignment::NyPzNx,
            RemapAlignment::PzPyNx,
            RemapAlignment::PzPxPy,
            RemapAlignment::PzNyPx,
            RemapAlignment::PzNxNy,
            RemapAlignment::NzPyPx,
            RemapAlignment::NzNxPy,
            RemapAlignment::NzNyNx,
            RemapAlignment::NzPxNy,
        ];

        for alignment in alignments {
            let result = remap(sensor, alignment);
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
        let result = remap(sensor, RemapAlignment::PxNzPy);
        assert_eq!(result, Vector::new(1.0, -3.0, 2.0));

        // +Y-X+Z: x'=y, y'=-x, z'=z
        let result = remap(sensor, RemapAlignment::PyNxPz);
        assert_eq!(result, Vector::new(2.0, -1.0, 3.0));

        // -X-Y+Z: x'=-x, y'=-y, z'=z
        let result = remap(sensor, RemapAlignment::NxNyPz);
        assert_eq!(result, Vector::new(-1.0, -2.0, 3.0));

        // +Z+X+Y: x'=z, y'=x, z'=y
        let result = remap(sensor, RemapAlignment::PzPxPy);
        assert_eq!(result, Vector::new(3.0, 1.0, 2.0));
    }

    #[test]
    fn test_zero_vector() {
        let sensor = Vector::ZERO;
        for alignment in [
            RemapAlignment::PxPyPz,
            RemapAlignment::PyNxPz,
            RemapAlignment::NzNyNx,
        ] {
            let result = remap(sensor, alignment);
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
            remap(x, RemapAlignment::PyNxPz),
            Vector::new(0.0, -1.0, 0.0)
        );
        assert_eq!(remap(y, RemapAlignment::PyNxPz), Vector::new(1.0, 0.0, 0.0));
        assert_eq!(remap(z, RemapAlignment::PyNxPz), z);
    }

    #[test]
    fn test_inverse_round_trip() {
        // Verify that applying an alignment and its inverse returns original
        // These are known inverse pairs from the rotation group
        let inverse_pairs = [
            (RemapAlignment::PxPyPz, RemapAlignment::PxPyPz), // identity
            (RemapAlignment::PyNxPz, RemapAlignment::NyPxPz), // 90° about Z
            (RemapAlignment::NxNyPz, RemapAlignment::NxNyPz), // 180° about Z (self-inverse)
            (RemapAlignment::PxNzPy, RemapAlignment::PxPzNy), // 90° about X
            (RemapAlignment::PzPyNx, RemapAlignment::NzPyPx), // 90° about Y
        ];

        let test_vectors = [
            Vector::new(1.0, 2.0, 3.0),
            Vector::new(-5.0, 0.0, 7.0),
            Vector::new(0.1, -0.2, 0.3),
        ];

        for (forward, inverse) in inverse_pairs {
            for &v in &test_vectors {
                let transformed = remap(v, forward);
                let recovered = remap(transformed, inverse);
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
        assert_eq!(RemapAlignment::PxPyPz.as_str(), "+X+Y+Z");
        assert_eq!(RemapAlignment::NzPxNy.as_str(), "-Z+X-Y");
    }
}
