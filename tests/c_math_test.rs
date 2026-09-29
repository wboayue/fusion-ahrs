//! Parity tests for the math types against the C library's `FusionMath.h`.
//!
//! Each operation runs on the same pseudo-random inputs in Rust and C.
//! Arithmetic must match bit for bit; functions that call into `libm`
//! (`asinf`, `atan2f`) allow a small tolerance, since the C side links the
//! platform `libm` and Rust uses the `libm` crate.

use fusion_ahrs::{DEG_TO_RAD, Matrix, Quaternion, RAD_TO_DEG, Vector};
use fusion_c_sys::math as c;

const SAMPLES: usize = 10_000;

/// Deterministic xorshift generator
struct Rng(u32);

impl Rng {
    /// Uniform value in [-10, 10)
    fn next(&mut self) -> f32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        (self.0 as f32 / u32::MAX as f32) * 20.0 - 10.0
    }

    fn array<const N: usize>(&mut self) -> [f32; N] {
        core::array::from_fn(|_| self.next())
    }
}

/// Runs `check` on `SAMPLES` random inputs
fn for_samples(mut check: impl FnMut(&mut Rng)) {
    let mut rng = Rng(0x2545_f491);
    for _ in 0..SAMPLES {
        check(&mut rng);
    }
}

fn bits<const N: usize>(a: [f32; N]) -> [u32; N] {
    a.map(f32::to_bits)
}

fn assert_bits<const N: usize>(name: &str, rust: [f32; N], c: [f32; N]) {
    assert_eq!(bits(rust), bits(c), "{name}: rust {rust:?}, c {c:?}");
}

fn assert_scalar_bits(name: &str, rust: f32, c: f32) {
    assert_eq!(rust.to_bits(), c.to_bits(), "{name}: rust {rust}, c {c}");
}

fn matrix(m: [f32; 9]) -> Matrix {
    Matrix::from_rows([[m[0], m[1], m[2]], [m[3], m[4], m[5]], [m[6], m[7], m[8]]])
}

fn matrix_array(m: Matrix) -> [f32; 9] {
    let [a, b, c] = m.to_rows();
    [a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2]]
}

#[test]
fn angle_conversion() {
    for_samples(|rng| {
        let x = rng.next() * 36.0;
        assert_scalar_bits(
            "degrees_to_radians",
            x * DEG_TO_RAD,
            c::degrees_to_radians(x),
        );
        assert_scalar_bits(
            "radians_to_degrees",
            x * RAD_TO_DEG,
            c::radians_to_degrees(x),
        );

        let v: [f32; 3] = rng.array();
        assert_bits(
            "to_radians",
            Vector::from(v).to_radians().into(),
            v.map(c::degrees_to_radians),
        );
    });
}

#[test]
fn vector_arithmetic() {
    for_samples(|rng| {
        let (a, b, s): ([f32; 3], [f32; 3], f32) = (rng.array(), rng.array(), rng.next());
        let (va, vb) = (Vector::from(a), Vector::from(b));

        assert_bits("add", (va + vb).into(), c::vector_add(a, b));
        assert_bits("subtract", (va - vb).into(), c::vector_subtract(a, b));
        assert_bits("scale", (va * s).into(), c::vector_scale(a, s));
        assert_bits("hadamard", va.hadamard(vb).into(), c::vector_hadamard(a, b));
        assert_bits("cross", va.cross(vb).into(), c::vector_cross(a, b));
        assert_scalar_bits("sum", va.sum(), c::vector_sum(a));
        assert_scalar_bits("dot", va.dot(vb), c::vector_dot(a, b));
        assert_scalar_bits("norm_squared", va.norm_squared(), c::vector_norm_squared(a));
        assert_scalar_bits("norm", va.norm(), c::vector_norm(a));
        assert_bits("normalise", va.normalize().into(), c::vector_normalise(a));
    });
}

#[test]
fn vector_is_zero() {
    for v in [
        [0.0, 0.0, 0.0],
        [-0.0, 0.0, 0.0],
        [0.0, 0.0, 1e-30],
        [1.0, 2.0, 3.0],
    ] {
        assert_eq!(Vector::from(v).is_zero(), c::vector_is_zero(v), "{v:?}");
    }
}

/// C divides by zero and returns NaN; Rust deliberately returns zero
#[test]
fn vector_normalise_zero_diverges_from_c() {
    assert!(c::vector_normalise([0.0; 3]).iter().all(|x| x.is_nan()));
    assert_eq!(Vector::ZERO.normalize(), Vector::ZERO);
}

#[test]
fn quaternion_arithmetic() {
    for_samples(|rng| {
        let (a, b, v, s): ([f32; 4], [f32; 4], [f32; 3], f32) =
            (rng.array(), rng.array(), rng.array(), rng.next());
        let (qa, qb) = (Quaternion::from(a), Quaternion::from(b));

        assert_bits("add", (qa + qb).into(), c::quaternion_add(a, b));
        assert_bits("scale", (qa * s).into(), c::quaternion_scale(a, s));
        assert_bits(
            "hadamard",
            qa.hadamard(qb).into(),
            c::quaternion_hadamard(a, b),
        );
        assert_bits("product", (qa * qb).into(), c::quaternion_product(a, b));
        assert_bits(
            "vector_product",
            qa.vector_product(Vector::from(v)).into(),
            c::quaternion_vector_product(a, v),
        );
        assert_scalar_bits("sum", qa.sum(), c::quaternion_sum(a));
        assert_scalar_bits(
            "norm_squared",
            qa.norm_squared(),
            c::quaternion_norm_squared(a),
        );
        assert_scalar_bits("norm", qa.norm(), c::quaternion_norm(a));
        assert_bits(
            "normalise",
            qa.normalize().into(),
            c::quaternion_normalise(a),
        );
    });
}

#[test]
fn quaternion_to_matrix() {
    for_samples(|rng| {
        let q = Quaternion::from(rng.array::<4>()).normalize();
        assert_bits(
            "to_matrix",
            matrix_array(q.to_matrix()),
            c::quaternion_to_matrix(q.into()),
        );
    });
}

#[test]
fn quaternion_to_euler() {
    for_samples(|rng| {
        let q = Quaternion::from(rng.array::<4>()).normalize();
        let rust = q.to_euler();
        let [roll, pitch, yaw] = c::quaternion_to_euler(q.into());

        // Wrap to (-180, 180] so ±180° results compare equal
        let diff = |a: f32, b: f32| {
            let d = (a - b).rem_euclid(360.0);
            d.min(360.0 - d)
        };
        let max = diff(rust.roll, roll)
            .max(diff(rust.pitch, pitch))
            .max(diff(rust.yaw, yaw));
        assert!(
            max < 1e-3,
            "to_euler differs by {max}°: rust {rust:?}, c {roll} {pitch} {yaw}"
        );
    });
}

/// `arc_sin` is internal; it's exercised through `to_euler` pitch, where
/// rounding can push `2 * (wy - xz)` just past ±1
#[test]
fn arc_sin_clamps_like_c() {
    // Adjacent f32 values around 1/√2
    let (w, y) = (f32::from_bits(0x3f35_04f3), f32::from_bits(0x3f35_04f4));
    for q in [
        Quaternion::new(w, 0.0, y, 0.0),
        Quaternion::new(w, 0.0, -y, 0.0),
    ] {
        let [_, c_pitch, _] = c::quaternion_to_euler(q.into());
        assert!(c_pitch.is_finite());
        assert_eq!(q.to_euler().pitch, c_pitch, "{q:?}");
    }
}

#[test]
fn matrix_operations() {
    for_samples(|rng| {
        let (m, v, s): ([f32; 9], [f32; 3], f32) = (rng.array(), rng.array(), rng.next());
        assert_bits(
            "multiply",
            (matrix(m) * Vector::from(v)).into(),
            c::matrix_multiply(m, v),
        );
        assert_bits("scale", matrix_array(matrix(m) * s), c::matrix_scale(m, s));
    });
}

#[test]
fn quaternion_rotate_matches_c_matrix() {
    for_samples(|rng| {
        let q = Quaternion::from(rng.array::<4>()).normalize();
        let v: [f32; 3] = rng.array();
        let c_rotated = c::matrix_multiply(c::quaternion_to_matrix(q.into()), v);
        assert_bits("rotate", q.rotate(Vector::from(v)).into(), c_rotated);
    });
}
