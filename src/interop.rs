//! Conversions to and from third-party math types, behind versioned
//! features.
//!
//! Each supported `nalgebra` version has its own feature (for example
//! `nalgebra-0_35`), so support for a new `nalgebra` release can be added
//! alongside the old one without a breaking change. Enable the feature that
//! matches the `nalgebra` version in your project.
//!
//! With `nalgebra-0_35` enabled, `nalgebra` types can be passed straight to
//! functions that accept `impl Into<Vector>` / `impl Into<Quaternion>` /
//! `impl Into<Matrix>`, and outputs convert back with `.into()`:
//!
//! | fusion-ahrs | nalgebra |
//! |---|---|
//! | [`Vector`](crate::Vector) | `Vector3<f32>` |
//! | [`Quaternion`](crate::Quaternion) | `Quaternion<f32>`, `UnitQuaternion<f32>` |
//! | [`Matrix`](crate::Matrix) | `Matrix3<f32>` |
//!
//! Converting a [`Quaternion`](crate::Quaternion) into `UnitQuaternion<f32>`
//! normalises it (a zero quaternion gives NaN components); the other
//! conversions copy components unchanged.
//!
//! ```
//! # #[cfg(feature = "nalgebra-0_35")] {
//! # use nalgebra_0_35 as nalgebra;
//! use fusion_ahrs::Ahrs;
//! use nalgebra::{UnitQuaternion, Vector3};
//!
//! let mut ahrs = Ahrs::new();
//! ahrs.update(
//!     Vector3::new(0.0, 0.0, 0.0),
//!     Vector3::new(0.0, 0.0, 1.0),
//!     Vector3::new(1.0, 0.0, 0.0),
//! );
//!
//! let orientation: UnitQuaternion<f32> = ahrs.quaternion().into();
//! let gravity: Vector3<f32> = ahrs.gravity().into();
//! # }
//! ```

/// Implements conversions between the crate math types and one `nalgebra`
/// version, given the dependency's crate name and its feature name.
#[allow(unused_macros)] // unused when no nalgebra feature is enabled
macro_rules! nalgebra_conversions {
    ($na:ident, $feature:literal) => {
        #[cfg_attr(docsrs, doc(cfg(feature = $feature)))]
        impl From<$crate::Vector> for $na::Vector3<f32> {
            #[inline]
            fn from(v: $crate::Vector) -> Self {
                $na::Vector3::new(v.x, v.y, v.z)
            }
        }

        #[cfg_attr(docsrs, doc(cfg(feature = $feature)))]
        impl From<$na::Vector3<f32>> for $crate::Vector {
            #[inline]
            fn from(v: $na::Vector3<f32>) -> Self {
                $crate::Vector::new(v.x, v.y, v.z)
            }
        }

        #[cfg_attr(docsrs, doc(cfg(feature = $feature)))]
        impl From<$crate::Quaternion> for $na::Quaternion<f32> {
            #[inline]
            fn from(q: $crate::Quaternion) -> Self {
                $na::Quaternion::new(q.w, q.x, q.y, q.z)
            }
        }

        #[cfg_attr(docsrs, doc(cfg(feature = $feature)))]
        impl From<$na::Quaternion<f32>> for $crate::Quaternion {
            #[inline]
            fn from(q: $na::Quaternion<f32>) -> Self {
                $crate::Quaternion::new(q.w, q.i, q.j, q.k)
            }
        }

        /// Normalises the quaternion (reciprocal of the norm, as
        /// [`Quaternion::normalize`](crate::Quaternion::normalize)), so a
        /// round trip back to [`Quaternion`](crate::Quaternion) may change
        /// the last bits of a nearly-unit input. A zero quaternion produces
        /// NaN components, as nalgebra's own normalisation does.
        #[cfg_attr(docsrs, doc(cfg(feature = $feature)))]
        impl From<$crate::Quaternion> for $na::UnitQuaternion<f32> {
            #[inline]
            fn from(q: $crate::Quaternion) -> Self {
                // Normalised here, so nalgebra needs no float-math feature
                $na::UnitQuaternion::new_unchecked(q.normalize().into())
            }
        }

        #[cfg_attr(docsrs, doc(cfg(feature = $feature)))]
        impl From<$na::UnitQuaternion<f32>> for $crate::Quaternion {
            #[inline]
            fn from(q: $na::UnitQuaternion<f32>) -> Self {
                q.into_inner().into()
            }
        }

        #[cfg_attr(docsrs, doc(cfg(feature = $feature)))]
        impl From<$crate::Matrix> for $na::Matrix3<f32> {
            #[inline]
            fn from(m: $crate::Matrix) -> Self {
                // Matrix3::new takes elements in row-major order
                $na::Matrix3::new(m.xx, m.xy, m.xz, m.yx, m.yy, m.yz, m.zx, m.zy, m.zz)
            }
        }

        #[cfg_attr(docsrs, doc(cfg(feature = $feature)))]
        impl From<$na::Matrix3<f32>> for $crate::Matrix {
            #[inline]
            fn from(m: $na::Matrix3<f32>) -> Self {
                // Index is (row, column)
                $crate::Matrix::from_rows([
                    [m[(0, 0)], m[(0, 1)], m[(0, 2)]],
                    [m[(1, 0)], m[(1, 1)], m[(1, 2)]],
                    [m[(2, 0)], m[(2, 1)], m[(2, 2)]],
                ])
            }
        }
    };
}

#[cfg(feature = "nalgebra-0_35")]
nalgebra_conversions!(nalgebra_0_35, "nalgebra-0_35");
