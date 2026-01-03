//! Sensor calibration functions for the Fusion AHRS library

use nalgebra::{Matrix3, Vector3};

/// Applies inertial sensor calibration (gyroscope and accelerometer)
///
/// Matches C implementation order: `misalignment * ((uncalibrated - offset) * sensitivity)`
///
/// # Arguments
/// * `uncalibrated` - Raw sensor reading
/// * `misalignment` - 3x3 misalignment correction matrix
/// * `sensitivity` - Sensitivity scaling factors for each axis
/// * `offset` - Bias offset to subtract from raw reading
///
/// # Returns
/// Calibrated sensor reading
///
/// # Example
/// ```
/// use nalgebra::{Matrix3, Vector3};
/// use fusion_ahrs::calibration::calibrate_inertial;
///
/// let raw = Vector3::new(1.0, 2.0, 3.0);
/// let misalignment = Matrix3::identity();
/// let sensitivity = Vector3::new(1.0, 1.0, 1.0);
/// let offset = Vector3::new(0.1, 0.2, 0.3);
///
/// let calibrated = calibrate_inertial(raw, misalignment, sensitivity, offset);
/// ```
pub fn calibrate_inertial(
    uncalibrated: Vector3<f32>,
    misalignment: Matrix3<f32>,
    sensitivity: Vector3<f32>,
    offset: Vector3<f32>,
) -> Vector3<f32> {
    // C order: (uncalibrated - offset) * sensitivity, then apply misalignment
    misalignment * (uncalibrated - offset).component_mul(&sensitivity)
}

/// Applies magnetometer calibration (hard and soft iron correction)
///
/// # Arguments
/// * `uncalibrated` - Raw magnetometer reading
/// * `soft_iron_matrix` - 3x3 soft iron correction matrix
/// * `hard_iron_offset` - Hard iron offset vector
///
/// # Returns
/// Calibrated magnetometer reading
///
/// # Example
/// ```
/// use nalgebra::{Matrix3, Vector3};
/// use fusion_ahrs::calibration::calibrate_magnetic;
///
/// let raw = Vector3::new(100.0, 200.0, 300.0);
/// let soft_iron = Matrix3::identity();
/// let hard_iron = Vector3::new(10.0, 20.0, 30.0);
///
/// let calibrated = calibrate_magnetic(raw, soft_iron, hard_iron);
/// ```
pub fn calibrate_magnetic(
    uncalibrated: Vector3<f32>,
    soft_iron_matrix: Matrix3<f32>,
    hard_iron_offset: Vector3<f32>,
) -> Vector3<f32> {
    soft_iron_matrix * (uncalibrated - hard_iron_offset)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_inertial_calibration() {
        let raw = Vector3::new(1.0, 2.0, 3.0);
        let misalignment = Matrix3::identity();
        let sensitivity = Vector3::new(0.5, 0.5, 0.5);
        let offset = Vector3::new(0.1, 0.2, 0.3);

        let calibrated = calibrate_inertial(raw, misalignment, sensitivity, offset);
        // C order: (raw - offset) * sensitivity
        // (1.0-0.1, 2.0-0.2, 3.0-0.3) * (0.5, 0.5, 0.5) = (0.9, 1.8, 2.7) * 0.5 = (0.45, 0.9, 1.35)
        let expected = Vector3::new(0.45, 0.9, 1.35);

        assert!((calibrated - expected).magnitude() < 1e-6);
    }

    #[test]
    fn test_magnetic_calibration() {
        let raw = Vector3::new(100.0, 200.0, 300.0);
        let soft_iron = Matrix3::identity();
        let hard_iron = Vector3::new(10.0, 20.0, 30.0);

        let calibrated = calibrate_magnetic(raw, soft_iron, hard_iron);
        let expected = Vector3::new(90.0, 180.0, 270.0); // raw - hard_iron

        assert!((calibrated - expected).magnitude() < 1e-6);
    }
}
