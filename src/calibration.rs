//! Sensor calibration functions for the Fusion AHRS library

use crate::math::{Matrix, Vector};

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
/// use fusion_ahrs::{Matrix, Vector};
/// use fusion_ahrs::calibration::calibrate_inertial;
///
/// let raw = Vector::new(1.0, 2.0, 3.0);
/// let misalignment = Matrix::IDENTITY;
/// let sensitivity = Vector::new(1.0, 1.0, 1.0);
/// let offset = Vector::new(0.1, 0.2, 0.3);
///
/// let calibrated = calibrate_inertial(raw, misalignment, sensitivity, offset);
/// ```
pub fn calibrate_inertial(
    uncalibrated: impl Into<Vector>,
    misalignment: impl Into<Matrix>,
    sensitivity: impl Into<Vector>,
    offset: impl Into<Vector>,
) -> Vector {
    // C order: (uncalibrated - offset) * sensitivity, then apply misalignment
    misalignment.into()
        * sensitivity
            .into()
            .hadamard(uncalibrated.into() - offset.into())
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
/// use fusion_ahrs::{Matrix, Vector};
/// use fusion_ahrs::calibration::calibrate_magnetic;
///
/// let raw = Vector::new(100.0, 200.0, 300.0);
/// let soft_iron = Matrix::IDENTITY;
/// let hard_iron = Vector::new(10.0, 20.0, 30.0);
///
/// let calibrated = calibrate_magnetic(raw, soft_iron, hard_iron);
/// ```
pub fn calibrate_magnetic(
    uncalibrated: impl Into<Vector>,
    soft_iron_matrix: impl Into<Matrix>,
    hard_iron_offset: impl Into<Vector>,
) -> Vector {
    soft_iron_matrix.into() * (uncalibrated.into() - hard_iron_offset.into())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_inertial_calibration() {
        let raw = Vector::new(1.0, 2.0, 3.0);
        let misalignment = Matrix::IDENTITY;
        let sensitivity = Vector::new(0.5, 0.5, 0.5);
        let offset = Vector::new(0.1, 0.2, 0.3);

        let calibrated = calibrate_inertial(raw, misalignment, sensitivity, offset);
        // C order: (raw - offset) * sensitivity
        // (1.0-0.1, 2.0-0.2, 3.0-0.3) * (0.5, 0.5, 0.5) = (0.9, 1.8, 2.7) * 0.5 = (0.45, 0.9, 1.35)
        let expected = Vector::new(0.45, 0.9, 1.35);

        assert!((calibrated - expected).norm() < 1e-6);
    }

    #[test]
    fn test_magnetic_calibration() {
        let raw = Vector::new(100.0, 200.0, 300.0);
        let soft_iron = Matrix::IDENTITY;
        let hard_iron = Vector::new(10.0, 20.0, 30.0);

        let calibrated = calibrate_magnetic(raw, soft_iron, hard_iron);
        let expected = Vector::new(90.0, 180.0, 270.0); // raw - hard_iron

        assert!((calibrated - expected).norm() < 1e-6);
    }
}
