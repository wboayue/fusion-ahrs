//! Mathematical utilities and nalgebra extensions for the Fusion AHRS library

use nalgebra::{UnitQuaternion, Vector3};

/// Mathematical constants
pub const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
pub const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;

/// Fast inverse square root implementation
/// Based on the famous "Quake" algorithm with Newton-Raphson iteration
#[allow(dead_code)]
pub fn fast_inverse_sqrt(x: f32) -> f32 {
    if x <= 0.0 {
        return 0.0;
    }

    let half_x = 0.5 * x;
    let mut i = x.to_bits();
    i = 0x5f3759df - (i >> 1); // Magic number from Quake
    let mut y = f32::from_bits(i);

    // Newton-Raphson iteration for improved accuracy
    y = y * (1.5 - (half_x * y * y));
    y = y * (1.5 - (half_x * y * y)); // Second iteration for higher precision

    y
}

/// Extension trait for Vector3 operations
pub trait Vector3Ext {
    /// Calculate the magnitude of the vector
    fn magnitude(&self) -> f32;

    /// Normalize the vector, returning zero vector if magnitude is zero
    fn safe_normalize(&self) -> Vector3<f32>;

    /// Convert degrees to radians
    fn deg_to_rad(&self) -> Vector3<f32>;

    /// Convert radians to degrees  
    fn rad_to_deg(&self) -> Vector3<f32>;
}

impl Vector3Ext for Vector3<f32> {
    fn magnitude(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    fn safe_normalize(&self) -> Vector3<f32> {
        let mag = self.magnitude();
        if mag > 0.0 {
            *self / mag
        } else {
            Vector3::zeros()
        }
    }

    fn deg_to_rad(&self) -> Vector3<f32> {
        *self * DEG_TO_RAD
    }

    fn rad_to_deg(&self) -> Vector3<f32> {
        *self * RAD_TO_DEG
    }
}

/// Extension trait for UnitQuaternion operations
pub trait QuaternionExt {
    /// Convert quaternion to Euler angles (roll, pitch, yaw) in radians
    fn to_euler(&self) -> Vector3<f32>;

    /// Convert quaternion to Euler angles in degrees
    fn to_euler_degrees(&self) -> Vector3<f32>;

    /// Create quaternion from Euler angles in radians
    fn from_euler(roll: f32, pitch: f32, yaw: f32) -> UnitQuaternion<f32>;

    /// Create quaternion from Euler angles in degrees
    fn from_euler_degrees(roll: f32, pitch: f32, yaw: f32) -> UnitQuaternion<f32>;
}

impl QuaternionExt for UnitQuaternion<f32> {
    fn to_euler(&self) -> Vector3<f32> {
        let (roll, pitch, yaw) = self.euler_angles();
        Vector3::new(roll, pitch, yaw)
    }

    fn to_euler_degrees(&self) -> Vector3<f32> {
        self.to_euler().rad_to_deg()
    }

    fn from_euler(roll: f32, pitch: f32, yaw: f32) -> UnitQuaternion<f32> {
        UnitQuaternion::from_euler_angles(roll, pitch, yaw)
    }

    fn from_euler_degrees(roll: f32, pitch: f32, yaw: f32) -> UnitQuaternion<f32> {
        let euler_rad = Vector3::new(roll, pitch, yaw).deg_to_rad();
        Self::from_euler(euler_rad.x, euler_rad.y, euler_rad.z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fast_inverse_sqrt() {
        let x = 4.0;
        let result = fast_inverse_sqrt(x);
        let expected = 1.0 / x.sqrt();

        // Should be accurate to within 1%
        assert!((result - expected).abs() / expected < 0.01);
    }

    #[test]
    fn test_vector_extensions() {
        let v = Vector3::new(3.0f32, 4.0, 0.0);
        assert!((v.magnitude() - 5.0).abs() < 1e-6);

        let normalized = v.safe_normalize();
        assert!((normalized.magnitude() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_quaternion_euler_conversion() {
        let euler = Vector3::new(30.0, 45.0, 60.0);
        let quat = UnitQuaternion::from_euler_degrees(euler.x, euler.y, euler.z);
        let recovered = quat.to_euler_degrees();

        // Allow for some numerical precision loss
        assert!((euler - recovered).magnitude() < 1e-5);
    }
}
