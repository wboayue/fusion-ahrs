//! Euler angles, mirroring the C library's `FusionEuler`

/// Euler angles in degrees, ZYX order (yaw, then pitch, then roll).
///
/// # Example
/// ```
/// use fusion_ahrs::{Euler, Quaternion};
///
/// let euler = Quaternion::IDENTITY.to_euler();
/// assert_eq!(euler, Euler::new(0.0, 0.0, 0.0));
/// ```
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Euler {
    /// Rotation about the X axis in degrees
    pub roll: f32,
    /// Rotation about the Y axis in degrees
    pub pitch: f32,
    /// Rotation about the Z axis in degrees
    pub yaw: f32,
}

impl Euler {
    /// Creates Euler angles from roll, pitch, and yaw in degrees.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Euler;
    ///
    /// let euler = Euler::new(10.0, 20.0, 30.0);
    /// assert_eq!(euler.yaw, 30.0);
    /// ```
    #[inline]
    pub const fn new(roll: f32, pitch: f32, yaw: f32) -> Self {
        Self { roll, pitch, yaw }
    }
}
