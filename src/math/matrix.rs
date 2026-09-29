//! 3x3 matrix, mirroring the C library's `FusionMatrix`

use core::ops::Mul;

use super::Vector;

/// 3x3 matrix of `f32`, row-major.
///
/// Field names give the row then the column: `xy` is row x, column y.
/// Used for calibration misalignment and soft-iron matrices.
///
/// # Example
/// ```
/// use fusion_ahrs::{Matrix, Vector};
///
/// let m = Matrix::from_rows([
///     [0.0, -1.0, 0.0],
///     [1.0, 0.0, 0.0],
///     [0.0, 0.0, 1.0],
/// ]);
/// assert_eq!(m * Vector::new(1.0, 0.0, 0.0), Vector::new(0.0, 1.0, 0.0));
/// ```
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Matrix {
    /// Row x, column x
    pub xx: f32,
    /// Row x, column y
    pub xy: f32,
    /// Row x, column z
    pub xz: f32,
    /// Row y, column x
    pub yx: f32,
    /// Row y, column y
    pub yy: f32,
    /// Row y, column z
    pub yz: f32,
    /// Row z, column x
    pub zx: f32,
    /// Row z, column y
    pub zy: f32,
    /// Row z, column z
    pub zz: f32,
}

impl Matrix {
    /// Identity matrix.
    pub const IDENTITY: Matrix =
        Matrix::from_rows([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);

    /// Creates a matrix from rows.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Matrix;
    ///
    /// let m = Matrix::from_rows([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
    /// assert_eq!(m.xy, 2.0);
    /// assert_eq!(m.yx, 4.0);
    /// ```
    pub const fn from_rows(rows: [[f32; 3]; 3]) -> Self {
        let [[xx, xy, xz], [yx, yy, yz], [zx, zy, zz]] = rows;
        Self {
            xx,
            xy,
            xz,
            yx,
            yy,
            yz,
            zx,
            zy,
            zz,
        }
    }

    /// Returns the rows.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Matrix;
    ///
    /// let rows = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
    /// assert_eq!(Matrix::from_rows(rows).to_rows(), rows);
    /// ```
    pub const fn to_rows(self) -> [[f32; 3]; 3] {
        [
            [self.xx, self.xy, self.xz],
            [self.yx, self.yy, self.yz],
            [self.zx, self.zy, self.zz],
        ]
    }

    /// Returns the transpose.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Matrix;
    ///
    /// let m = Matrix::from_rows([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
    /// assert_eq!(m.transpose().xy, 4.0);
    /// ```
    pub fn transpose(self) -> Matrix {
        Matrix::from_rows([
            [self.xx, self.yx, self.zx],
            [self.xy, self.yy, self.zy],
            [self.xz, self.yz, self.zz],
        ])
    }
}

impl Default for Matrix {
    fn default() -> Self {
        Matrix::IDENTITY
    }
}

/// Scales every element.
impl Mul<f32> for Matrix {
    type Output = Matrix;

    fn mul(self, s: f32) -> Matrix {
        let m = self;
        Matrix {
            xx: m.xx * s,
            xy: m.xy * s,
            xz: m.xz * s,
            yx: m.yx * s,
            yy: m.yy * s,
            yz: m.yz * s,
            zx: m.zx * s,
            zy: m.zy * s,
            zz: m.zz * s,
        }
    }
}

/// Matrix-vector product.
impl Mul<Vector> for Matrix {
    type Output = Vector;

    fn mul(self, v: Vector) -> Vector {
        let m = self;
        Vector::new(
            m.xx * v.x + m.xy * v.y + m.xz * v.z,
            m.yx * v.x + m.yy * v.y + m.yz * v.z,
            m.zx * v.x + m.zy * v.y + m.zz * v.z,
        )
    }
}

impl From<[[f32; 3]; 3]> for Matrix {
    fn from(rows: [[f32; 3]; 3]) -> Self {
        Matrix::from_rows(rows)
    }
}

impl From<Matrix> for [[f32; 3]; 3] {
    fn from(m: Matrix) -> Self {
        m.to_rows()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity_multiply() {
        let v = Vector::new(1.0, 2.0, 3.0);
        assert_eq!(Matrix::IDENTITY * v, v);
    }

    #[test]
    fn test_scale() {
        let m = Matrix::IDENTITY * 2.0;
        assert_eq!(m * Vector::ONES, Vector::new(2.0, 2.0, 2.0));
    }

    #[test]
    fn test_transpose_twice() {
        let m = Matrix::from_rows([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
        assert_eq!(m.transpose().transpose(), m);
    }
}
