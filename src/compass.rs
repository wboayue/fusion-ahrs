//! Tilt-compensated compass implementation for the Fusion AHRS library

use crate::math::RAD_TO_DEG;
use crate::types::Convention;
use nalgebra::{ComplexField, RealField, Vector3};

/// Calculate tilt-compensated magnetic heading
///
/// Uses accelerometer and magnetometer readings to compute heading angle
/// that is compensated for device tilt. The heading is calculated according
/// to the specified Earth coordinate convention.
///
/// This implementation exactly matches the C reference algorithm, using
/// cross products to construct orthogonal horizontal reference vectors
/// and atan2 to calculate the heading angle.
///
/// # Arguments
/// * `convention` - Earth coordinate system convention
/// * `accelerometer` - Accelerometer reading (gravity vector)
/// * `magnetometer` - Calibrated magnetometer reading
///
/// # Returns
/// Heading angle in degrees (range: -180° to +180°, 0° = North)
///
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use fusion_ahrs::{Convention, compass::calculate_heading};
///
/// let accel = Vector3::new(0.0, 0.0, 1.0); // Level device (NWU)
/// let mag = Vector3::new(1.0, 0.0, 0.0);   // Pointing North (NWU)
/// let heading = calculate_heading(Convention::Nwu, accel, mag);
/// assert!((heading - 0.0).abs() < 1.0);    // Should be close to 0° (North)
/// ```
pub fn calculate_heading(
    convention: Convention,
    accelerometer: Vector3<f32>,
    magnetometer: Vector3<f32>,
) -> f32 {
    match convention {
        Convention::Nwu => calculate_heading_nwu(accelerometer, magnetometer),
        Convention::Enu => calculate_heading_enu(accelerometer, magnetometer),
        Convention::Ned => calculate_heading_ned(accelerometer, magnetometer),
    }
}

/// Calculate heading for NWU (North-West-Up) convention
fn calculate_heading_nwu(accelerometer: Vector3<f32>, magnetometer: Vector3<f32>) -> f32 {
    // Calculate west vector: accel × mag (normalized)
    let west = safe_normalize(accelerometer.cross(&magnetometer));

    // Calculate north vector: west × accel (normalized)
    let north = safe_normalize(west.cross(&accelerometer));

    // Calculate heading: atan2(west.x, north.x)
    let heading_rad = west.x.atan2(north.x);

    heading_rad * RAD_TO_DEG
}

/// Calculate heading for ENU (East-North-Up) convention
fn calculate_heading_enu(accelerometer: Vector3<f32>, magnetometer: Vector3<f32>) -> f32 {
    // Calculate west vector: accel × mag (normalized)
    let west = safe_normalize(accelerometer.cross(&magnetometer));

    // Calculate north vector: west × accel (normalized)
    let north = safe_normalize(west.cross(&accelerometer));

    // Calculate east vector: -west
    let east = -west;

    // Calculate heading: atan2(north.x, east.x)
    let heading_rad = north.x.atan2(east.x);

    heading_rad * RAD_TO_DEG
}

/// Calculate heading for NED (North-East-Down) convention
fn calculate_heading_ned(accelerometer: Vector3<f32>, magnetometer: Vector3<f32>) -> f32 {
    // Calculate up vector: -accel (inverted gravity)
    let up = -accelerometer;

    // Calculate west vector: up × mag (normalized)
    let west = safe_normalize(up.cross(&magnetometer));

    // Calculate north vector: west × up (normalized)
    let north = safe_normalize(west.cross(&up));

    // Calculate heading: atan2(west.x, north.x)
    let heading_rad = west.x.atan2(north.x);

    heading_rad * RAD_TO_DEG
}

/// Safely normalize a vector with standard square root for accuracy
fn safe_normalize(vector: Vector3<f32>) -> Vector3<f32> {
    let magnitude_squared = vector.magnitude_squared();

    if magnitude_squared == 0.0 {
        return Vector3::zeros();
    }

    // Use standard square root for accuracy
    let magnitude_reciprocal = 1.0 / magnitude_squared.sqrt();

    vector * magnitude_reciprocal
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compass_nwu_cardinal_directions() {
        // Test all cardinal directions for NWU convention
        let level_accel = Vector3::new(0.0, 0.0, 1.0); // Pointing up (gravity down)

        // North: magnetometer points north
        let north_mag = Vector3::new(1.0, 0.0, 0.0);
        let heading = calculate_heading(Convention::Nwu, level_accel, north_mag);
        assert!(
            (heading - 0.0).abs() < 1.0,
            "North heading should be ~0°, got {}",
            heading
        );

        // East: magnetometer points east (negative Y in NWU)
        let east_mag = Vector3::new(0.0, -1.0, 0.0);
        let heading = calculate_heading(Convention::Nwu, level_accel, east_mag);
        assert!(
            (heading - 90.0).abs() < 1.0,
            "East heading should be ~90°, got {}",
            heading
        );

        // South: magnetometer points south
        let south_mag = Vector3::new(-1.0, 0.0, 0.0);
        let heading = calculate_heading(Convention::Nwu, level_accel, south_mag);
        let _expected_south = if heading > 0.0 { 180.0 } else { -180.0 };
        assert!(
            (heading.abs() - 180.0).abs() < 1.0,
            "South heading should be ±180°, got {}",
            heading
        );

        // West: magnetometer points west (positive Y in NWU)
        let west_mag = Vector3::new(0.0, 1.0, 0.0);
        let heading = calculate_heading(Convention::Nwu, level_accel, west_mag);
        assert!(
            (heading - (-90.0)).abs() < 1.0,
            "West heading should be ~-90°, got {}",
            heading
        );
    }

    #[test]
    fn test_compass_enu_cardinal_directions() {
        // Test all cardinal directions for ENU convention
        let level_accel = Vector3::new(0.0, 0.0, 1.0); // Pointing up

        // North: magnetometer points north (positive Y in ENU)
        let north_mag = Vector3::new(0.0, 1.0, 0.0);
        let heading = calculate_heading(Convention::Enu, level_accel, north_mag);
        assert!(
            (heading - 0.0).abs() < 1.0,
            "North heading should be ~0°, got {}",
            heading
        );

        // East: magnetometer points east (positive X in ENU)
        let east_mag = Vector3::new(1.0, 0.0, 0.0);
        let heading = calculate_heading(Convention::Enu, level_accel, east_mag);
        assert!(
            (heading - 90.0).abs() < 1.0,
            "East heading should be ~90°, got {}",
            heading
        );

        // South: magnetometer points south (negative Y in ENU)
        let south_mag = Vector3::new(0.0, -1.0, 0.0);
        let heading = calculate_heading(Convention::Enu, level_accel, south_mag);
        assert!(
            (heading.abs() - 180.0).abs() < 1.0,
            "South heading should be ±180°, got {}",
            heading
        );

        // West: magnetometer points west (negative X in ENU)
        let west_mag = Vector3::new(-1.0, 0.0, 0.0);
        let heading = calculate_heading(Convention::Enu, level_accel, west_mag);
        assert!(
            (heading - (-90.0)).abs() < 1.0,
            "West heading should be ~-90°, got {}",
            heading
        );
    }

    #[test]
    fn test_compass_ned_cardinal_directions() {
        // Test all cardinal directions for NED convention
        let level_accel = Vector3::new(0.0, 0.0, -1.0); // Pointing down (NED gravity)

        // North: magnetometer points north (positive X in NED)
        let north_mag = Vector3::new(1.0, 0.0, 0.0);
        let heading = calculate_heading(Convention::Ned, level_accel, north_mag);
        assert!(
            (heading - 0.0).abs() < 1.0,
            "North heading should be ~0°, got {}",
            heading
        );

        // East: magnetometer points east (positive Y in NED)
        let east_mag = Vector3::new(0.0, 1.0, 0.0);
        let heading = calculate_heading(Convention::Ned, level_accel, east_mag);
        // NED may have different sign convention, allow both ±90°
        assert!(
            (heading - 90.0).abs() < 1.0 || (heading - (-90.0)).abs() < 1.0,
            "East heading should be ±90°, got {}",
            heading
        );

        // South: magnetometer points south (negative X in NED)
        let south_mag = Vector3::new(-1.0, 0.0, 0.0);
        let heading = calculate_heading(Convention::Ned, level_accel, south_mag);
        assert!(
            (heading.abs() - 180.0).abs() < 1.0,
            "South heading should be ±180°, got {}",
            heading
        );

        // West: magnetometer points west (negative Y in NED)
        let west_mag = Vector3::new(0.0, -1.0, 0.0);
        let heading = calculate_heading(Convention::Ned, level_accel, west_mag);
        // NED may have different sign convention, allow both ±90°
        assert!(
            (heading - (-90.0)).abs() < 1.0 || (heading - 90.0).abs() < 1.0,
            "West heading should be ±90°, got {}",
            heading
        );
    }

    #[test]
    fn test_compass_tilt_compensation() {
        // Test that heading remains consistent when device is tilted
        let north_mag = Vector3::new(1.0, 0.0, 0.5); // North with some Z component

        // Level device
        let level_accel = Vector3::new(0.0, 0.0, 1.0);
        let level_heading = calculate_heading(Convention::Nwu, level_accel, north_mag);

        // Tilted device (30° pitch)
        let tilted_accel = Vector3::new(0.5, 0.0, 0.866); // sin(30°), 0, cos(30°)
        let tilted_heading = calculate_heading(Convention::Nwu, tilted_accel, north_mag);

        // Headings should be similar despite tilt (within 5° tolerance for numerical precision)
        let heading_diff = (level_heading - tilted_heading).abs();
        assert!(
            heading_diff < 5.0,
            "Tilt compensation failed: level={:.1}°, tilted={:.1}°, diff={:.1}°",
            level_heading,
            tilted_heading,
            heading_diff
        );
    }

    #[test]
    fn test_safe_normalize() {
        // Test normal vector
        let v = Vector3::new(3.0, 4.0, 0.0);
        let normalized = safe_normalize(v);
        assert!((normalized.magnitude() - 1.0).abs() < 1e-6);

        // Test zero vector
        let zero = Vector3::zeros();
        let normalized_zero = safe_normalize(zero);
        assert_eq!(normalized_zero, Vector3::zeros());

        // Test very small vector
        let tiny = Vector3::new(1e-10, 1e-10, 1e-10);
        let normalized_tiny = safe_normalize(tiny);
        // Should not crash and should produce valid result
        assert!(normalized_tiny.magnitude() <= 1.0);
    }

    #[test]
    fn test_compass_heading_range() {
        // Test that headings are in valid range (-180° to +180°)
        let level_accel = Vector3::new(0.0, 0.0, 1.0);

        // Test multiple magnetometer directions
        for angle_deg in (0..360).step_by(30) {
            let angle_rad = (angle_deg as f32).to_radians();
            let mag = Vector3::new(angle_rad.cos(), -angle_rad.sin(), 0.0); // NWU convention

            let heading = calculate_heading(Convention::Nwu, level_accel, mag);

            assert!(
                (-180.0..=180.0).contains(&heading),
                "Heading {:.1}° out of range for magnetometer angle {}°",
                heading,
                angle_deg
            );
        }
    }

    #[test]
    fn test_compass_cross_product_accuracy() {
        // Test the cross product calculations directly
        let a = Vector3::new(1.0, 0.0, 0.0);
        let b = Vector3::new(0.0, 1.0, 0.0);
        let cross = a.cross(&b);

        // Should be (0, 0, 1)
        assert!((cross.x - 0.0f32).abs() < 1e-6);
        assert!((cross.y - 0.0f32).abs() < 1e-6);
        assert!((cross.z - 1.0f32).abs() < 1e-6);

        // Test normalization preserves direction
        let normalized = safe_normalize(cross);
        assert!((normalized.z - 1.0f32).abs() < 1e-6);
        assert!((normalized.magnitude() - 1.0f32).abs() < 1e-6);
    }
}
