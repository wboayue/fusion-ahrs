//! Parity tests against the upstream C library.
//!
//! The `c_*` tests run the C reference implementation (compiled from the
//! `fusion-c/` submodule by the `fusion-c-sys` crate) side by side with the
//! Rust port on `testdata/sensor_data.csv` and compare every output on every
//! sample. The remaining tests check Rust-only behavior on the same data.
//!
//! The C library is built with `FUSION_USE_NORMAL_SQRT` and without FMA
//! contraction; remaining differences are float rounding.

use fusion_ahrs::{
    Ahrs, AhrsSettings, Bias, BiasSettings, Convention, Euler, Matrix, Quaternion, RemapAlignment,
    Vector, calculate_heading, calibrate_inertial, calibrate_magnetic, remap,
};
use fusion_c_sys as c;
use serde::Deserialize;
use std::sync::LazyLock;

#[derive(Debug, Deserialize)]
struct SensorData {
    #[serde(rename = "Time (s)")]
    time: f32,
    #[serde(rename = "Gyroscope X (deg/s)")]
    gyro_x: f32,
    #[serde(rename = "Gyroscope Y (deg/s)")]
    gyro_y: f32,
    #[serde(rename = "Gyroscope Z (deg/s)")]
    gyro_z: f32,
    #[serde(rename = "Accelerometer X (g)")]
    accel_x: f32,
    #[serde(rename = "Accelerometer Y (g)")]
    accel_y: f32,
    #[serde(rename = "Accelerometer Z (g)")]
    accel_z: f32,
    #[serde(rename = "Magnetometer X (uT)")]
    mag_x: f32,
    #[serde(rename = "Magnetometer Y (uT)")]
    mag_y: f32,
    #[serde(rename = "Magnetometer Z (uT)")]
    mag_z: f32,
}

const SAMPLE_RATE: f32 = 100.0; // 100 Hz

impl SensorData {
    fn gyroscope(&self) -> Vector {
        Vector::new(self.gyro_x, self.gyro_y, self.gyro_z)
    }

    fn accelerometer(&self) -> Vector {
        Vector::new(self.accel_x, self.accel_y, self.accel_z)
    }

    fn magnetometer(&self) -> Vector {
        Vector::new(self.mag_x, self.mag_y, self.mag_z)
    }
}

/// Sensor data parsed once and shared by all tests
static SENSOR_DATA: LazyLock<Vec<SensorData>> = LazyLock::new(|| {
    csv::Reader::from_path("testdata/sensor_data.csv")
        .unwrap()
        .deserialize()
        .map(|r| r.unwrap())
        .collect()
});

// Tolerances for Rust vs C comparisons
const VECTOR_TOLERANCE: f32 = 1e-4;
// Error angles go through asin, which amplifies rounding near 90°
const ANGLE_TOLERANCE: f32 = 0.05;
const TRIGGER_TOLERANCE: f32 = 1e-5;

fn convention_index(convention: Convention) -> u32 {
    match convention {
        Convention::Nwu => 0,
        Convention::Enu => 1,
        Convention::Ned => 2,
    }
}

fn c_settings(settings: &AhrsSettings) -> c::AhrsSettings {
    c::AhrsSettings {
        sample_rate: settings.sample_rate,
        convention: convention_index(settings.convention),
        gain: settings.gain,
        gyroscope_range: settings.gyroscope_range,
        acceleration_rejection: settings.acceleration_rejection,
        magnetic_rejection: settings.magnetic_rejection,
        rejection_timeout: settings.rejection_timeout,
    }
}

fn arr(v: Vector) -> [f32; 3] {
    [v.x, v.y, v.z]
}

fn assert_vector(context: &str, name: &str, rust: Vector, c: [f32; 3], tolerance: f32) {
    let d = rust - Vector::from(c);
    let diff = d.x.abs().max(d.y.abs()).max(d.z.abs());
    assert!(
        diff <= tolerance,
        "{context}: {name} differs by {diff:e} (rust {rust:?}, c {c:?})"
    );
}

fn assert_scalar(context: &str, name: &str, rust: f32, c: f32, tolerance: f32) {
    let diff = (rust - c).abs();
    assert!(
        diff <= tolerance,
        "{context}: {name} differs by {diff:e} (rust {rust}, c {c})"
    );
}

/// Compare every AHRS output of the Rust and C instances
fn assert_ahrs_matches(context: &str, rust: &Ahrs, c: &c::Ahrs) {
    let out = c.outputs();

    let q = rust.quaternion();
    let q = [q.w, q.x, q.y, q.z];
    let diff = q
        .iter()
        .zip(out.quaternion)
        .map(|(r, c)| (r - c).abs())
        .fold(0.0, f32::max);
    assert!(
        diff <= VECTOR_TOLERANCE,
        "{context}: quaternion differs by {diff:e} (rust {q:?}, c {:?})",
        out.quaternion
    );

    assert_vector(
        context,
        "gravity",
        rust.gravity(),
        out.gravity,
        VECTOR_TOLERANCE,
    );
    assert_vector(
        context,
        "linear_acceleration",
        rust.linear_acceleration(),
        out.linear_acceleration,
        VECTOR_TOLERANCE,
    );
    assert_vector(
        context,
        "earth_acceleration",
        rust.earth_acceleration(),
        out.earth_acceleration,
        VECTOR_TOLERANCE,
    );

    let rs = rust.internal_states();
    let cs = out.internal_states;
    assert_scalar(
        context,
        "acceleration_error",
        rs.acceleration_error,
        cs.acceleration_error,
        ANGLE_TOLERANCE,
    );
    assert_scalar(
        context,
        "magnetic_error",
        rs.magnetic_error,
        cs.magnetic_error,
        ANGLE_TOLERANCE,
    );
    assert_scalar(
        context,
        "acceleration_recovery_trigger",
        rs.acceleration_recovery_trigger,
        cs.acceleration_recovery_trigger,
        TRIGGER_TOLERANCE,
    );
    assert_scalar(
        context,
        "magnetic_recovery_trigger",
        rs.magnetic_recovery_trigger,
        cs.magnetic_recovery_trigger,
        TRIGGER_TOLERANCE,
    );
    assert_eq!(
        rs.accelerometer_ignored, cs.accelerometer_ignored,
        "{context}: accelerometer_ignored"
    );
    assert_eq!(
        rs.magnetometer_ignored, cs.magnetometer_ignored,
        "{context}: magnetometer_ignored"
    );

    let rf = rust.flags();
    let cf = out.flags;
    assert_eq!(rf.startup, cf.startup, "{context}: startup");
    assert_eq!(
        rf.overrange_recovery, cf.overrange_recovery,
        "{context}: overrange_recovery"
    );
    assert_eq!(
        rf.acceleration_recovery, cf.acceleration_recovery,
        "{context}: acceleration_recovery"
    );
    assert_eq!(
        rf.magnetic_recovery, cf.magnetic_recovery,
        "{context}: magnetic_recovery"
    );
}

#[derive(Debug, Clone, Copy)]
enum Mode {
    Full,
    NoMagnetometer,
    ExternalHeading,
}

#[derive(Debug, Clone, Copy)]
struct Case {
    settings: AhrsSettings,
    mode: Mode,
    variable_sample_period: bool,
    skip_startup: bool,
}

impl Case {
    /// Full 9-axis update at the nominal sample rate
    fn new(settings: AhrsSettings) -> Self {
        Self {
            settings,
            mode: Mode::Full,
            variable_sample_period: false,
            skip_startup: false,
        }
    }
}

/// Run a case through both implementations, comparing after every sample
fn run_case(case: Case) {
    run_case_with(case, |_, _, _| {});
}

/// Like [`run_case`], calling `before_sample` on both instances before each update
fn run_case_with(case: Case, mut before_sample: impl FnMut(usize, &mut Ahrs, &mut c::Ahrs)) {
    let mut rust = Ahrs::with_settings(case.settings);
    let mut c = c::Ahrs::new(&c_settings(&case.settings));

    if case.skip_startup {
        rust.skip_startup();
        c.skip_startup();
    }

    let mut previous_time = 0.0;
    for (i, d) in SENSOR_DATA.iter().enumerate() {
        before_sample(i, &mut rust, &mut c);

        if case.variable_sample_period {
            let period = if i == 0 {
                d.time
            } else {
                d.time - previous_time
            };
            rust.set_sample_period(period);
            c.set_sample_period(period);
        }
        previous_time = d.time;

        let (g, a, m) = (d.gyroscope(), d.accelerometer(), d.magnetometer());
        match case.mode {
            Mode::Full => {
                rust.update(g, a, m);
                c.update(arr(g), arr(a), arr(m));
            }
            Mode::NoMagnetometer => {
                rust.update_no_magnetometer(g, a);
                c.update_no_magnetometer(arr(g), arr(a));
            }
            Mode::ExternalHeading => {
                let heading = d.time * 10.0;
                rust.update_external_heading(g, a, heading);
                c.update_external_heading(arr(g), arr(a), heading);
            }
        }

        assert_ahrs_matches(&format!("{case:?} sample {i}"), &rust, &c);
    }
}

fn advanced_settings(convention: Convention) -> AhrsSettings {
    AhrsSettings {
        sample_rate: SAMPLE_RATE,
        convention,
        gain: 0.5,
        gyroscope_range: 2000.0,
        acceleration_rejection: 10.0,
        magnetic_rejection: 10.0,
        rejection_timeout: 5.0,
    }
}

#[test]
fn c_ahrs_all_conventions_and_modes() {
    for convention in Convention::ALL {
        for mode in [Mode::Full, Mode::NoMagnetometer, Mode::ExternalHeading] {
            run_case(Case {
                mode,
                ..Case::new(advanced_settings(convention))
            });
        }
    }
}

#[test]
fn c_ahrs_variable_sample_period() {
    for convention in [Convention::Nwu, Convention::Ned] {
        run_case(Case {
            variable_sample_period: true,
            ..Case::new(advanced_settings(convention))
        });
    }
}

#[test]
fn c_ahrs_skip_startup() {
    run_case(Case {
        skip_startup: true,
        ..Case::new(advanced_settings(Convention::Nwu))
    });
}

/// A low gyroscope range forces frequent overrange recovery
#[test]
fn c_ahrs_overrange_recovery() {
    for convention in [Convention::Nwu, Convention::Ned] {
        run_case(Case::new(AhrsSettings {
            gyroscope_range: 200.0,
            ..advanced_settings(convention)
        }));
    }
}

/// Default settings, zero gain, and a short timeout exercise the
/// rejection-disabled and fast-recovery paths
#[test]
fn c_ahrs_settings_variants() {
    let variants = [
        AhrsSettings::default(),
        AhrsSettings {
            gain: 0.0,
            ..advanced_settings(Convention::Nwu)
        },
        AhrsSettings {
            rejection_timeout: 0.1,
            acceleration_rejection: 2.0,
            magnetic_rejection: 2.0,
            ..advanced_settings(Convention::Enu)
        },
        AhrsSettings {
            sample_rate: 50.0,
            ..advanced_settings(Convention::Nwu)
        },
    ];
    for settings in variants {
        run_case(Case::new(settings));
    }
}

/// Mid-run set_heading, set_quaternion, set_settings, and restart
#[test]
fn c_ahrs_state_changes() {
    let settings = advanced_settings(Convention::Nwu);
    run_case_with(Case::new(settings), |i, rust, c| match i {
        2000 => {
            rust.set_heading(45.0);
            c.set_heading(45.0);
        }
        4000 => {
            let q = Quaternion::from_euler(Euler::new(17.2, -11.5, 57.3));
            rust.set_quaternion(q);
            c.set_quaternion([q.w, q.x, q.y, q.z]);
        }
        6000 => {
            let new = AhrsSettings {
                convention: Convention::Ned,
                gain: 1.0,
                ..settings
            };
            rust.set_settings(new);
            c.set_settings(&c_settings(&new));
        }
        8000 => {
            rust.restart();
            c.restart();
        }
        _ => {}
    });
}

/// Bias ports C's arithmetic exactly, so outputs must be bit-identical
#[test]
fn c_bias_matches() {
    let settings = BiasSettings {
        sample_rate: SAMPLE_RATE,
        ..Default::default()
    };
    let mut rust = Bias::with_settings(settings);
    let mut c = c::Bias::new(&c::BiasSettings {
        sample_rate: settings.sample_rate,
        stationary_threshold: settings.stationary_threshold,
        stationary_period: settings.stationary_period,
    });

    for (i, d) in SENSOR_DATA.iter().enumerate() {
        let g = d.gyroscope();
        let context = format!("bias sample {i}");
        assert_vector(&context, "corrected", rust.update(g), c.update(arr(g)), 0.0);
        assert_vector(&context, "offset", rust.offset(), c.offset(), 0.0);
    }
}

#[test]
fn c_compass_heading() {
    for convention in Convention::ALL {
        for (i, d) in SENSOR_DATA.iter().enumerate() {
            let (a, m) = (d.accelerometer(), d.magnetometer());
            let rust = calculate_heading(convention, a, m);
            let c = c::compass(arr(a), arr(m), convention_index(convention));
            assert_scalar(
                &format!("{convention:?} sample {i}"),
                "heading",
                rust,
                c,
                1e-3,
            );
        }
    }
}

/// C enum order differs from Rust; alignments are matched by name
#[test]
fn c_remap_all_alignments() {
    let sensor = Vector::new(1.0, 2.0, 3.0);

    for index in 0..24 {
        let name = c::remap_alignment_to_string(index);
        let alignment = RemapAlignment::ALL
            .into_iter()
            .find(|a| a.to_string() == name)
            .unwrap_or_else(|| panic!("no Rust alignment named {name}"));

        assert_eq!(
            arr(remap(sensor, alignment)),
            c::remap(arr(sensor), index),
            "{name}"
        );
    }
}

#[test]
fn c_convention_strings() {
    for convention in Convention::ALL {
        assert_eq!(
            convention.to_string(),
            c::convention_to_string(convention_index(convention))
        );
    }
}

#[test]
fn c_calibration_models() {
    // Deterministic pseudo-random values in [-2, 2)
    let mut state = 0x2545_f491_u32;
    let mut next = || {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        (state as f32 / u32::MAX as f32) * 4.0 - 2.0
    };

    for i in 0..1000 {
        let uncalibrated = [next(), next(), next()];
        let matrix: [f32; 9] = core::array::from_fn(|_| next());
        let sensitivity = [next(), next(), next()];
        let offset = [next(), next(), next()];

        let rust = calibrate_inertial(
            Vector::from(uncalibrated),
            Matrix::from_rows([
                [matrix[0], matrix[1], matrix[2]],
                [matrix[3], matrix[4], matrix[5]],
                [matrix[6], matrix[7], matrix[8]],
            ]),
            Vector::from(sensitivity),
            Vector::from(offset),
        );
        let c = c::model_inertial(uncalibrated, matrix, sensitivity, offset);
        assert_vector(&format!("sample {i}"), "inertial", rust, c, 1e-5);

        let rust = calibrate_magnetic(
            Vector::from(uncalibrated),
            Matrix::from_rows([
                [matrix[0], matrix[1], matrix[2]],
                [matrix[3], matrix[4], matrix[5]],
                [matrix[6], matrix[7], matrix[8]],
            ]),
            Vector::from(offset),
        );
        let c = c::model_magnetic(uncalibrated, matrix, offset);
        assert_vector(&format!("sample {i}"), "magnetic", rust, c, 1e-5);
    }
}

/// Roll and pitch agree with and without the magnetometer; heading is
/// zeroed during startup without it
#[test]
fn test_update_method_consistency() {
    let mut ahrs_full = Ahrs::new();
    let mut ahrs_no_mag = Ahrs::new();

    // First 4 seconds
    for d in &SENSOR_DATA[..400] {
        ahrs_full.update(d.gyroscope(), d.accelerometer(), d.magnetometer());
        ahrs_no_mag.update_no_magnetometer(d.gyroscope(), d.accelerometer());
    }

    assert!(!ahrs_full.flags().startup);
    assert!(!ahrs_no_mag.flags().startup);

    let Euler {
        roll: roll_full,
        pitch: pitch_full,
        yaw: _,
    } = ahrs_full.quaternion().to_euler();
    let Euler {
        roll: roll_no_mag,
        pitch: pitch_no_mag,
        yaw: yaw_no_mag,
    } = ahrs_no_mag.quaternion().to_euler();

    let roll_diff = (roll_full - roll_no_mag).abs();
    let pitch_diff = (pitch_full - pitch_no_mag).abs();
    assert!(
        roll_diff < 10.0,
        "Roll difference too large: {roll_diff} deg"
    );
    assert!(
        pitch_diff < 10.0,
        "Pitch difference too large: {pitch_diff} deg"
    );
    assert!(
        yaw_no_mag.abs() < 5.0,
        "No-mag heading should be near zero: {} deg",
        yaw_no_mag
    );
}

/// Quaternion norm stays at 1 over the full recording
#[test]
fn test_numerical_stability() {
    let mut ahrs = Ahrs::new();

    for (i, d) in SENSOR_DATA.iter().enumerate() {
        ahrs.update(d.gyroscope(), d.accelerometer(), d.magnetometer());

        let norm = ahrs.quaternion().norm();
        assert!(
            (norm - 1.0).abs() < 1e-5,
            "Quaternion not normalized at sample {i}: norm = {norm}"
        );
    }
}
