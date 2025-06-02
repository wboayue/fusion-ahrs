//! Mathematical utilities and nalgebra extensions for the Fusion AHRS library

use nalgebra::{UnitQuaternion, Vector3};

/// Mathematical constants for angle conversion

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


/// Extension trait for Vector3 operations
///
/// Provides additional utility methods for `nalgebra::Vector3<f32>`
/// including safe normalization and angle conversions.
///
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use fusion_ahrs::Vector3Ext;
/// 
/// let v = Vector3::new(3.0, 4.0, 0.0);
/// let magnitude = v.magnitude();
/// let normalized = v.safe_normalize();
/// ```
pub trait Vector3Ext {
    /// Calculate the magnitude of the vector
    /// 
    /// Returns the Euclidean length of the vector.
    /// 
    /// # Returns
    /// Vector magnitude as f32
    fn magnitude(&self) -> f32;

    /// Normalize the vector, returning zero vector if magnitude is zero
    /// 
    /// Provides safe normalization that handles zero-length vectors
    /// without causing division by zero errors.
    /// 
    /// # Returns
    /// Unit vector in same direction, or zero vector if input is zero
    fn safe_normalize(&self) -> Vector3<f32>;

    /// Convert degrees to radians
    /// 
    /// Applies degree-to-radian conversion to each component.
    /// 
    /// # Returns
    /// Vector with components converted from degrees to radians
    fn deg_to_rad(&self) -> Vector3<f32>;

    /// Convert radians to degrees
    /// 
    /// Applies radian-to-degree conversion to each component.
    /// 
    /// # Returns
    /// Vector with components converted from radians to degrees
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
///
/// Provides convenient methods for working with quaternions,
/// including Euler angle conversions with degree support.
///
/// # Example
/// ```
/// use nalgebra::UnitQuaternion;
/// use fusion_ahrs::QuaternionExt;
/// 
/// let quat = UnitQuaternion::from_euler_degrees(30.0, 45.0, 60.0);
/// let euler_deg = quat.to_euler_degrees();
/// ```
pub trait QuaternionExt {
    /// Convert quaternion to Euler angles (roll, pitch, yaw) in radians
    /// 
    /// Returns Euler angles in the order: roll (X), pitch (Y), yaw (Z).
    /// 
    /// # Returns
    /// Vector containing [roll, pitch, yaw] in radians
    fn to_euler(&self) -> Vector3<f32>;

    /// Convert quaternion to Euler angles in degrees
    /// 
    /// Returns Euler angles in the order: roll (X), pitch (Y), yaw (Z).
    /// 
    /// # Returns
    /// Vector containing [roll, pitch, yaw] in degrees
    fn to_euler_degrees(&self) -> Vector3<f32>;

    /// Create quaternion from Euler angles in radians
    /// 
    /// # Arguments
    /// * `roll` - Rotation around X-axis in radians
    /// * `pitch` - Rotation around Y-axis in radians
    /// * `yaw` - Rotation around Z-axis in radians
    /// 
    /// # Returns
    /// Unit quaternion representing the rotation
    fn from_euler(roll: f32, pitch: f32, yaw: f32) -> UnitQuaternion<f32>;

    /// Create quaternion from Euler angles in degrees
    /// 
    /// # Arguments
    /// * `roll` - Rotation around X-axis in degrees
    /// * `pitch` - Rotation around Y-axis in degrees
    /// * `yaw` - Rotation around Z-axis in degrees
    /// 
    /// # Returns
    /// Unit quaternion representing the rotation
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
