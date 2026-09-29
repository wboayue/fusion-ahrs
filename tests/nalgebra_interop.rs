//! Conversions between the crate math types and nalgebra 0.35.
//!
//! Run with `cargo test --features nalgebra-0_35`.

#![cfg(feature = "nalgebra-0_35")]

use fusion_ahrs::{Ahrs, Euler, Matrix, Quaternion, Vector, calibrate_inertial};
use nalgebra_0_35::{Matrix3, Quaternion as NaQuaternion, UnitQuaternion, Vector3};

#[test]
fn vector_round_trip() {
    let v = Vector::new(1.0, -2.0, 3.5);
    let na: Vector3<f32> = v.into();
    assert_eq!(na, Vector3::new(1.0, -2.0, 3.5));
    assert_eq!(Vector::from(na), v);
}

#[test]
fn quaternion_round_trip_keeps_scalar_first() {
    let q = Quaternion::new(0.1, 0.2, 0.3, 0.4);
    let na: NaQuaternion<f32> = q.into();
    assert_eq!((na.w, na.i, na.j, na.k), (0.1, 0.2, 0.3, 0.4));
    assert_eq!(Quaternion::from(na), q);
}

#[test]
fn unit_quaternion_is_normalised() {
    let q = Quaternion::new(2.0, 0.0, 0.0, 0.0);
    let unit: UnitQuaternion<f32> = q.into();
    assert_eq!(unit.into_inner(), NaQuaternion::new(1.0, 0.0, 0.0, 0.0));
    assert_eq!(Quaternion::from(unit), Quaternion::IDENTITY);
}

/// Re-normalising a nearly-unit quaternion may change its last bits
#[test]
fn unit_quaternion_round_trip_within_tolerance() {
    for euler in [
        Euler::new(10.0, 20.0, 30.0),
        Euler::new(-170.0, 80.0, 170.0),
        Euler::new(0.5, -45.0, 90.0),
    ] {
        let q = Quaternion::from_euler(euler);
        let back = Quaternion::from(UnitQuaternion::from(q));
        let diff: [f32; 4] = (back + q * -1.0).into();
        assert!(
            diff.iter().all(|d| d.abs() < 1e-6),
            "{euler:?}: {q:?} -> {back:?}"
        );
    }
}

/// Zero has no direction; matches nalgebra's own normalisation
#[test]
fn zero_quaternion_to_unit_is_nan() {
    let unit: UnitQuaternion<f32> = Quaternion::new(0.0, 0.0, 0.0, 0.0).into();
    assert!(unit.coords.iter().all(|c| c.is_nan()));
    let theirs = UnitQuaternion::from_quaternion(NaQuaternion::new(0.0_f32, 0.0, 0.0, 0.0));
    assert!(theirs.coords.iter().all(|c| c.is_nan()));
}

#[test]
fn unit_quaternion_rotation_agrees() {
    let q = Quaternion::from_euler(Euler::new(15.0, -30.0, 45.0));
    let unit: UnitQuaternion<f32> = q.into();
    let v = Vector::new(1.0, 2.0, 3.0);

    let ours = q.rotate(v);
    let theirs = unit * Vector3::from(v);
    assert!((ours - Vector::from(theirs)).norm() < 1e-5);
}

#[test]
fn matrix_round_trip_keeps_row_major_order() {
    let m = Matrix::from_rows([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
    let na: Matrix3<f32> = m.into();
    assert_eq!(na[(0, 1)], 2.0); // row 0, column 1
    assert_eq!(na[(1, 0)], 4.0);
    assert_eq!(Matrix::from(na), m);

    let v = Vector::new(1.0, -1.0, 0.5);
    assert_eq!(Vector::from(na * Vector3::from(v)), m * v);
}

/// nalgebra values go straight into `impl Into<...>` parameters
#[test]
fn nalgebra_inputs_match_native_inputs() {
    let (g, a, m) = (
        Vector::new(1.0, -2.0, 0.5),
        Vector::new(0.1, 0.0, 1.0),
        Vector::new(0.4, 0.1, -0.3),
    );

    let mut native = Ahrs::new();
    let mut na = Ahrs::new();
    for _ in 0..100 {
        native.update(g, a, m);
        na.update(Vector3::from(g), Vector3::from(a), Vector3::from(m));
    }
    assert_eq!(native.quaternion(), na.quaternion());

    let q = UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);
    na.set_quaternion(q);
    assert_eq!(na.quaternion(), Quaternion::from(q));

    let calibrated = calibrate_inertial(
        Vector3::new(1.0, 2.0, 3.0),
        Matrix3::identity(),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::zeros(),
    );
    assert_eq!(calibrated, Vector::new(1.0, 2.0, 3.0));
}
