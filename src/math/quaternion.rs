//! Quaternion, mirroring the C library's `FusionQuaternion`

use core::ops::{Add, Mul};

use super::{DEG_TO_RAD, Euler, Matrix, RAD_TO_DEG, Vector, arc_sin};

/// Quaternion with `w` as the scalar part.
///
/// Orientation outputs from [`Ahrs`](crate::Ahrs) are unit quaternions
/// describing the sensor frame relative to the Earth frame. Unit length is
/// not enforced by the type, matching the C library.
///
/// # Example
/// ```
/// use fusion_ahrs::{Euler, Quaternion, Vector};
///
/// let q = Quaternion::from_euler(Euler::new(0.0, 0.0, 90.0));
/// let v = q.rotate(Vector::new(1.0, 0.0, 0.0));
/// assert!((v.y - 1.0).abs() < 1e-6);
/// ```
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    /// Scalar part
    pub w: f32,
    /// X component of the vector part
    pub x: f32,
    /// Y component of the vector part
    pub y: f32,
    /// Z component of the vector part
    pub z: f32,
}

impl Quaternion {
    /// Identity quaternion (no rotation).
    pub const IDENTITY: Quaternion = Quaternion::new(1.0, 0.0, 0.0, 0.0);

    /// Creates a quaternion from its components, scalar first.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Quaternion;
    ///
    /// let q = Quaternion::new(1.0, 0.0, 0.0, 0.0);
    /// assert_eq!(q, Quaternion::IDENTITY);
    /// ```
    #[inline]
    pub const fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    /// Creates a quaternion from Euler angles in degrees (ZYX order: yaw,
    /// then pitch, then roll). Inverse of [`Quaternion::to_euler`].
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::{Euler, Quaternion};
    ///
    /// let euler = Euler::new(10.0, 20.0, 30.0);
    /// let back = Quaternion::from_euler(euler).to_euler();
    /// assert!((back.roll - 10.0).abs() < 1e-3);
    /// assert!((back.pitch - 20.0).abs() < 1e-3);
    /// assert!((back.yaw - 30.0).abs() < 1e-3);
    /// ```
    #[inline]
    pub fn from_euler(euler: Euler) -> Self {
        let half_roll = 0.5 * euler.roll * DEG_TO_RAD;
        let half_pitch = 0.5 * euler.pitch * DEG_TO_RAD;
        let half_yaw = 0.5 * euler.yaw * DEG_TO_RAD;

        let (sr, cr) = (libm::sinf(half_roll), libm::cosf(half_roll));
        let (sp, cp) = (libm::sinf(half_pitch), libm::cosf(half_pitch));
        let (sy, cy) = (libm::sinf(half_yaw), libm::cosf(half_yaw));

        Quaternion::new(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
    }

    /// Returns the sum of the components.
    #[inline]
    pub fn sum(self) -> f32 {
        self.w + self.x + self.y + self.z
    }

    /// Returns the Hadamard (element-wise) product.
    #[inline]
    pub fn hadamard(self, rhs: Quaternion) -> Quaternion {
        Quaternion::new(
            self.w * rhs.w,
            self.x * rhs.x,
            self.y * rhs.y,
            self.z * rhs.z,
        )
    }

    /// Returns the product of the quaternion and a vector treated as a pure
    /// quaternion (zero scalar part): `q ⊗ [0, v]`.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::{Quaternion, Vector};
    ///
    /// let v = Vector::new(1.0, 2.0, 3.0);
    /// assert_eq!(Quaternion::IDENTITY.vector_product(v), Quaternion::new(0.0, 1.0, 2.0, 3.0));
    /// ```
    #[inline]
    pub fn vector_product(self, v: Vector) -> Quaternion {
        let q = self;
        Quaternion::new(
            -q.x * v.x - q.y * v.y - q.z * v.z,
            q.w * v.x + q.y * v.z - q.z * v.y,
            q.w * v.y - q.x * v.z + q.z * v.x,
            q.w * v.z + q.x * v.y - q.y * v.x,
        )
    }

    /// Returns the conjugate, which is the inverse for a unit quaternion.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Quaternion;
    ///
    /// let q = Quaternion::new(0.5, 0.5, 0.5, 0.5);
    /// assert_eq!(q * q.conjugate(), Quaternion::IDENTITY);
    /// ```
    #[inline]
    pub fn conjugate(self) -> Quaternion {
        Quaternion::new(self.w, -self.x, -self.y, -self.z)
    }

    /// Returns the squared norm.
    #[inline]
    pub fn norm_squared(self) -> f32 {
        self.hadamard(self).sum()
    }

    /// Returns the norm.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Quaternion;
    ///
    /// assert_eq!(Quaternion::new(1.0, 1.0, 1.0, 1.0).norm(), 2.0);
    /// ```
    #[inline]
    pub fn norm(self) -> f32 {
        libm::sqrtf(self.norm_squared())
    }

    /// Returns the unit quaternion, multiplying by the reciprocal of the
    /// norm. Matches C's `FusionQuaternionNormalise` built with
    /// `FUSION_USE_NORMAL_SQRT` (the default C build uses a fast approximate
    /// inverse square root instead).
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Quaternion;
    ///
    /// let q = Quaternion::new(2.0, 0.0, 0.0, 0.0).normalize();
    /// assert_eq!(q, Quaternion::IDENTITY);
    /// ```
    #[inline]
    pub fn normalize(self) -> Quaternion {
        self * (1.0 / self.norm())
    }

    /// Returns the rotation matrix. Multiplying it by a vector rotates the
    /// vector from the sensor frame to the Earth frame.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::{Matrix, Quaternion};
    ///
    /// assert_eq!(Quaternion::IDENTITY.to_matrix(), Matrix::IDENTITY);
    /// ```
    #[inline]
    pub fn to_matrix(self) -> Matrix {
        let q = self;
        let two_w = 2.0 * q.w;
        let two_x = 2.0 * q.x;
        let two_y = 2.0 * q.y;
        let two_z = 2.0 * q.z;
        Matrix {
            xx: two_w * q.w - 1.0 + two_x * q.x,
            xy: two_x * q.y - two_w * q.z,
            xz: two_x * q.z + two_w * q.y,
            yx: two_x * q.y + two_w * q.z,
            yy: two_w * q.w - 1.0 + two_y * q.y,
            yz: two_y * q.z - two_w * q.x,
            zx: two_x * q.z - two_w * q.y,
            zy: two_y * q.z + two_w * q.x,
            zz: two_w * q.w - 1.0 + two_z * q.z,
        }
    }

    /// Returns the Euler angles in degrees (ZYX order).
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::{Euler, Quaternion};
    ///
    /// assert_eq!(Quaternion::IDENTITY.to_euler(), Euler::new(0.0, 0.0, 0.0));
    /// ```
    #[inline]
    pub fn to_euler(self) -> Euler {
        let q = self;
        Euler::new(
            libm::atan2f(q.y * q.z + q.w * q.x, q.w * q.w + q.z * q.z - 0.5) * RAD_TO_DEG,
            arc_sin(2.0 * (q.w * q.y - q.x * q.z)) * RAD_TO_DEG,
            libm::atan2f(q.x * q.y + q.w * q.z, q.w * q.w + q.x * q.x - 0.5) * RAD_TO_DEG,
        )
    }

    /// Rotates a vector from the sensor frame to the Earth frame.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::{Euler, Quaternion, Vector};
    ///
    /// let q = Quaternion::from_euler(Euler::new(90.0, 0.0, 0.0));
    /// let v = q.rotate(Vector::new(0.0, 1.0, 0.0));
    /// assert!((v.z - 1.0).abs() < 1e-6);
    /// ```
    #[inline]
    pub fn rotate(self, v: Vector) -> Vector {
        self.to_matrix() * v
    }
}

impl Default for Quaternion {
    #[inline]
    fn default() -> Self {
        Quaternion::IDENTITY
    }
}

impl Add for Quaternion {
    type Output = Quaternion;

    #[inline]
    fn add(self, rhs: Quaternion) -> Quaternion {
        Quaternion::new(
            self.w + rhs.w,
            self.x + rhs.x,
            self.y + rhs.y,
            self.z + rhs.z,
        )
    }
}

impl Mul<f32> for Quaternion {
    type Output = Quaternion;

    #[inline]
    fn mul(self, rhs: f32) -> Quaternion {
        Quaternion::new(self.w * rhs, self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

/// Hamilton product.
impl Mul for Quaternion {
    type Output = Quaternion;

    #[inline]
    fn mul(self, rhs: Quaternion) -> Quaternion {
        let (a, b) = (self, rhs);
        Quaternion::new(
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        )
    }
}

/// Scalar first: `[w, x, y, z]`.
impl From<[f32; 4]> for Quaternion {
    #[inline]
    fn from([w, x, y, z]: [f32; 4]) -> Self {
        Quaternion::new(w, x, y, z)
    }
}

/// Scalar first: `[w, x, y, z]`.
impl From<Quaternion> for [f32; 4] {
    #[inline]
    fn from(q: Quaternion) -> Self {
        [q.w, q.x, q.y, q.z]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(a: Quaternion, b: Quaternion) {
        let d: [f32; 4] = (a + b * -1.0).into();
        assert!(d.iter().all(|x| x.abs() < 1e-6), "{a:?} != {b:?}");
    }

    #[test]
    fn test_product_matches_rotation_composition() {
        let a = Quaternion::from_euler(Euler::new(10.0, 0.0, 0.0));
        let b = Quaternion::from_euler(Euler::new(0.0, 20.0, 0.0));
        let v = Vector::new(0.3, -0.4, 0.5);
        let composed = (a * b).rotate(v);
        let sequential = a.rotate(b.rotate(v));
        assert!((composed - sequential).norm() < 1e-6);
    }

    #[test]
    fn test_vector_product_is_product_with_pure_quaternion() {
        let q = Quaternion::new(0.1, 0.2, 0.3, 0.4);
        let v = Vector::new(1.0, -2.0, 3.0);
        assert_close(q.vector_product(v), q * Quaternion::new(0.0, v.x, v.y, v.z));
    }

    #[test]
    fn test_rotate_matches_sandwich_product() {
        let q = Quaternion::from_euler(Euler::new(15.0, -30.0, 45.0));
        let v = Vector::new(1.0, 2.0, 3.0);
        let p = q.vector_product(v) * q.conjugate();
        let r = q.rotate(v);
        assert!((Vector::new(p.x, p.y, p.z) - r).norm() < 1e-5);
    }

    #[test]
    fn test_euler_round_trip() {
        for (roll, pitch, yaw) in [(0.0, 0.0, 0.0), (45.0, 30.0, -60.0), (-170.0, 80.0, 170.0)] {
            let e = Quaternion::from_euler(Euler::new(roll, pitch, yaw)).to_euler();
            assert!((e.roll - roll).abs() < 1e-3, "{e:?}");
            assert!((e.pitch - pitch).abs() < 1e-3, "{e:?}");
            assert!((e.yaw - yaw).abs() < 1e-3, "{e:?}");
        }
    }

    #[test]
    fn test_gimbal_lock_pitch_is_clamped() {
        // Adjacent f32 values around 1/√2: 2 * (wy - xz) rounds above 1,
        // which must not produce NaN
        let (w, y) = (f32::from_bits(0x3f35_04f3), f32::from_bits(0x3f35_04f4));
        let e = Quaternion::new(w, 0.0, y, 0.0).to_euler();
        assert!(e.pitch.is_finite());
        assert!((e.pitch - 90.0).abs() < 1e-3);
    }
}
