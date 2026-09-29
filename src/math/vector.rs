//! Three-dimensional vector, mirroring the C library's `FusionVector`

use core::ops::{Add, AddAssign, Mul, MulAssign, Neg, Sub, SubAssign};

use super::{DEG_TO_RAD, RAD_TO_DEG};

/// Three-dimensional vector of `f32`.
///
/// Used for sensor inputs (gyroscope in degrees per second, accelerometer
/// in g, magnetometer in any calibrated units) and algorithm outputs.
///
/// # Example
/// ```
/// use fusion_ahrs::Vector;
///
/// let a = Vector::new(1.0, 2.0, 3.0);
/// let b: Vector = [4.0, 5.0, 6.0].into();
///
/// assert_eq!(a + b, Vector::new(5.0, 7.0, 9.0));
/// assert_eq!(a.dot(b), 32.0);
/// assert_eq!(a.cross(b), Vector::new(-3.0, 6.0, -3.0));
/// ```
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vector {
    /// X component
    pub x: f32,
    /// Y component
    pub y: f32,
    /// Z component
    pub z: f32,
}

impl Vector {
    /// Vector of zeros.
    pub const ZERO: Vector = Vector::new(0.0, 0.0, 0.0);

    /// Vector of ones.
    pub const ONES: Vector = Vector::new(1.0, 1.0, 1.0);

    /// Creates a vector from its components.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    ///
    /// let v = Vector::new(1.0, 2.0, 3.0);
    /// assert_eq!(v.y, 2.0);
    /// ```
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Returns true if all components are zero.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    ///
    /// assert!(Vector::ZERO.is_zero());
    /// assert!(!Vector::new(0.0, 0.0, 1.0).is_zero());
    /// ```
    pub fn is_zero(self) -> bool {
        self.x == 0.0 && self.y == 0.0 && self.z == 0.0
    }

    /// Returns the sum of the components.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    ///
    /// assert_eq!(Vector::new(1.0, 2.0, 3.0).sum(), 6.0);
    /// ```
    pub fn sum(self) -> f32 {
        self.x + self.y + self.z
    }

    /// Returns the Hadamard (element-wise) product.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    ///
    /// let v = Vector::new(1.0, 2.0, 3.0).hadamard(Vector::new(4.0, 5.0, 6.0));
    /// assert_eq!(v, Vector::new(4.0, 10.0, 18.0));
    /// ```
    pub fn hadamard(self, rhs: Vector) -> Vector {
        Vector::new(self.x * rhs.x, self.y * rhs.y, self.z * rhs.z)
    }

    /// Returns the dot product.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    ///
    /// assert_eq!(Vector::new(1.0, 2.0, 3.0).dot(Vector::new(4.0, 5.0, 6.0)), 32.0);
    /// ```
    pub fn dot(self, rhs: Vector) -> f32 {
        self.hadamard(rhs).sum()
    }

    /// Returns the cross product.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    ///
    /// let z = Vector::new(1.0, 0.0, 0.0).cross(Vector::new(0.0, 1.0, 0.0));
    /// assert_eq!(z, Vector::new(0.0, 0.0, 1.0));
    /// ```
    pub fn cross(self, rhs: Vector) -> Vector {
        Vector::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
    }

    /// Returns the squared norm (magnitude).
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    ///
    /// assert_eq!(Vector::new(1.0, 2.0, 2.0).norm_squared(), 9.0);
    /// ```
    pub fn norm_squared(self) -> f32 {
        self.hadamard(self).sum()
    }

    /// Returns the norm (magnitude).
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    ///
    /// assert_eq!(Vector::new(3.0, 4.0, 0.0).norm(), 5.0);
    /// ```
    pub fn norm(self) -> f32 {
        libm::sqrtf(self.norm_squared())
    }

    /// Returns the unit vector in the same direction, or zero for a zero
    /// vector.
    ///
    /// Multiplies by the reciprocal of the norm, as the C library does.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    ///
    /// assert_eq!(Vector::new(3.0, 0.0, 4.0).normalize(), Vector::new(0.6, 0.0, 0.8));
    /// assert_eq!(Vector::ZERO.normalize(), Vector::ZERO);
    /// ```
    pub fn normalize(self) -> Vector {
        let norm = self.norm();
        if norm == 0.0 {
            return Vector::ZERO;
        }
        self * (1.0 / norm)
    }

    /// Converts each component from degrees to radians.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    ///
    /// let v = Vector::new(180.0, 0.0, 0.0).to_radians();
    /// assert!((v.x - core::f32::consts::PI).abs() < 1e-6);
    /// ```
    pub fn to_radians(self) -> Vector {
        self * DEG_TO_RAD
    }

    /// Converts each component from radians to degrees.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    ///
    /// let v = Vector::new(core::f32::consts::PI, 0.0, 0.0).to_degrees();
    /// assert!((v.x - 180.0).abs() < 1e-4);
    /// ```
    pub fn to_degrees(self) -> Vector {
        self * RAD_TO_DEG
    }
}

impl Add for Vector {
    type Output = Vector;

    fn add(self, rhs: Vector) -> Vector {
        Vector::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub for Vector {
    type Output = Vector;

    fn sub(self, rhs: Vector) -> Vector {
        Vector::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Neg for Vector {
    type Output = Vector;

    fn neg(self) -> Vector {
        Vector::new(-self.x, -self.y, -self.z)
    }
}

impl Mul<f32> for Vector {
    type Output = Vector;

    fn mul(self, rhs: f32) -> Vector {
        Vector::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl Mul<Vector> for f32 {
    type Output = Vector;

    fn mul(self, rhs: Vector) -> Vector {
        rhs * self
    }
}

impl AddAssign for Vector {
    fn add_assign(&mut self, rhs: Vector) {
        *self = *self + rhs;
    }
}

impl SubAssign for Vector {
    fn sub_assign(&mut self, rhs: Vector) {
        *self = *self - rhs;
    }
}

impl MulAssign<f32> for Vector {
    fn mul_assign(&mut self, rhs: f32) {
        *self = *self * rhs;
    }
}

impl From<[f32; 3]> for Vector {
    fn from([x, y, z]: [f32; 3]) -> Self {
        Vector::new(x, y, z)
    }
}

impl From<Vector> for [f32; 3] {
    fn from(v: Vector) -> Self {
        [v.x, v.y, v.z]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_operators() {
        let a = Vector::new(1.0, 2.0, 3.0);
        let b = Vector::new(4.0, 5.0, 6.0);

        assert_eq!(a + b, Vector::new(5.0, 7.0, 9.0));
        assert_eq!(b - a, Vector::new(3.0, 3.0, 3.0));
        assert_eq!(-a, Vector::new(-1.0, -2.0, -3.0));
        assert_eq!(a * 2.0, Vector::new(2.0, 4.0, 6.0));
        assert_eq!(2.0 * a, a * 2.0);

        let mut c = a;
        c += b;
        c -= a;
        c *= 0.5;
        assert_eq!(c, b * 0.5);
    }

    #[test]
    fn test_cross_is_orthogonal() {
        let a = Vector::new(1.0, -2.0, 0.5);
        let b = Vector::new(-3.0, 0.25, 2.0);
        let c = a.cross(b);
        assert!(c.dot(a).abs() < 1e-5);
        assert!(c.dot(b).abs() < 1e-5);
        assert_eq!(b.cross(a), -c);
    }

    #[test]
    fn test_normalize() {
        let v = Vector::new(1.0, 2.0, 3.0).normalize();
        assert!((v.norm() - 1.0).abs() < 1e-6);
        assert_eq!(Vector::ZERO.normalize(), Vector::ZERO);
    }

    #[test]
    fn test_array_conversion() {
        let v: Vector = [1.0, 2.0, 3.0].into();
        let a: [f32; 3] = v.into();
        assert_eq!(a, [1.0, 2.0, 3.0]);
    }
}
