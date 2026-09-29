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
    Ahrs, AhrsSettings, AxesAlignment, Convention, Offset, OffsetSettings, axes_swap,
    calculate_heading, calibrate_inertial, calibrate_magnetic,
};
use fusion_c_sys as c;
use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use serde::Deserialize;
use std::error::Error;

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

// Tolerances for Rust vs C comparisons
const VECTOR_TOLERANCE: f32 = 1e-4;
// Error angles go through asin, which amplifies rounding near 90°
const ANGLE_TOLERANCE: f32 = 0.05;
const TRIGGER_TOLERANCE: f32 = 1e-5;

fn load_sensor_data() -> Vec<SensorData> {
    csv::Reader::from_path("testdata/sensor_data.csv")
        .unwrap()
        .deserialize()
        .map(|r| r.unwrap())
        .collect()
}

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

fn arr(v: Vector3<f32>) -> [f32; 3] {
    [v.x, v.y, v.z]
}

fn assert_vector(context: &str, name: &str, rust: Vector3<f32>, c: [f32; 3], tolerance: f32) {
    let diff = (rust - Vector3::from(c)).amax();
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
    let q = [q.w, q.i, q.j, q.k];
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

/// Run a case through both implementations, comparing after every sample
fn run_case(data: &[SensorData], case: Case) {
    let mut rust = Ahrs::with_settings(case.settings);
    let mut c = c::Ahrs::new(&c_settings(&case.settings));

    if case.skip_startup {
        rust.skip_startup();
        c.skip_startup();
    }

    let mut previous_time = 0.0;
    for (i, d) in data.iter().enumerate() {
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

        let g = Vector3::new(d.gyro_x, d.gyro_y, d.gyro_z);
        let a = Vector3::new(d.accel_x, d.accel_y, d.accel_z);
        let m = Vector3::new(d.mag_x, d.mag_y, d.mag_z);

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
    let data = load_sensor_data();
    for convention in [Convention::Nwu, Convention::Enu, Convention::Ned] {
        for mode in [Mode::Full, Mode::NoMagnetometer, Mode::ExternalHeading] {
            run_case(
                &data,
                Case {
                    settings: advanced_settings(convention),
                    mode,
                    variable_sample_period: false,
                    skip_startup: false,
                },
            );
        }
    }
}

#[test]
fn c_ahrs_variable_sample_period() {
    let data = load_sensor_data();
    for convention in [Convention::Nwu, Convention::Ned] {
        run_case(
            &data,
            Case {
                settings: advanced_settings(convention),
                mode: Mode::Full,
                variable_sample_period: true,
                skip_startup: false,
            },
        );
    }
}

#[test]
fn c_ahrs_skip_startup() {
    run_case(
        &load_sensor_data(),
        Case {
            settings: advanced_settings(Convention::Nwu),
            mode: Mode::Full,
            variable_sample_period: false,
            skip_startup: true,
        },
    );
}

/// A low gyroscope range forces frequent overrange recovery
#[test]
fn c_ahrs_overrange_recovery() {
    let data = load_sensor_data();
    for convention in [Convention::Nwu, Convention::Ned] {
        run_case(
            &data,
            Case {
                settings: AhrsSettings {
                    gyroscope_range: 200.0,
                    ..advanced_settings(convention)
                },
                mode: Mode::Full,
                variable_sample_period: false,
                skip_startup: false,
            },
        );
    }
}

/// Default settings, zero gain, and a short timeout exercise the
/// rejection-disabled and fast-recovery paths
#[test]
fn c_ahrs_settings_variants() {
    let data = load_sensor_data();
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
        run_case(
            &data,
            Case {
                settings,
                mode: Mode::Full,
                variable_sample_period: false,
                skip_startup: false,
            },
        );
    }
}

/// Mid-run set_heading, set_quaternion, set_settings, and restart
#[test]
fn c_ahrs_state_changes() {
    let data = load_sensor_data();
    let settings = advanced_settings(Convention::Nwu);
    let mut rust = Ahrs::with_settings(settings);
    let mut c = c::Ahrs::new(&c_settings(&settings));

    for (i, d) in data.iter().enumerate() {
        match i {
            2000 => {
                rust.set_heading(45.0);
                c.set_heading(45.0);
            }
            4000 => {
                let q = UnitQuaternion::from_euler_angles(0.3_f32, -0.2, 1.0);
                rust.set_quaternion(q);
                c.set_quaternion([q.w, q.i, q.j, q.k]);
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
        }

        let g = Vector3::new(d.gyro_x, d.gyro_y, d.gyro_z);
        let a = Vector3::new(d.accel_x, d.accel_y, d.accel_z);
        let m = Vector3::new(d.mag_x, d.mag_y, d.mag_z);
        rust.update(g, a, m);
        c.update(arr(g), arr(a), arr(m));

        assert_ahrs_matches(&format!("state changes sample {i}"), &rust, &c);
    }
}

#[test]
fn c_offset_matches_bias() {
    let data = load_sensor_data();
    let settings = OffsetSettings::default();
    let mut rust = Offset::new(settings, SAMPLE_RATE);
    let mut c = c::Bias::new(&c::BiasSettings {
        sample_rate: SAMPLE_RATE,
        stationary_threshold: settings.threshold,
        stationary_period: settings.timeout,
    });

    for (i, d) in data.iter().enumerate() {
        let g = Vector3::new(d.gyro_x, d.gyro_y, d.gyro_z);
        let context = format!("offset sample {i}");
        assert_vector(
            &context,
            "corrected",
            rust.update(g),
            c.update(arr(g)),
            1e-6,
        );
        assert_vector(&context, "offset", rust.offset(), c.offset(), 1e-6);
    }
}

#[test]
fn c_compass_heading() {
    let data = load_sensor_data();
    for convention in [Convention::Nwu, Convention::Enu, Convention::Ned] {
        for (i, d) in data.iter().enumerate() {
            let a = Vector3::new(d.accel_x, d.accel_y, d.accel_z);
            let m = Vector3::new(d.mag_x, d.mag_y, d.mag_z);
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
    let rust_alignments = [
        AxesAlignment::PxPyPz,
        AxesAlignment::PxNzPy,
        AxesAlignment::PxNyNz,
        AxesAlignment::PxPzNy,
        AxesAlignment::NxPyNz,
        AxesAlignment::NxPzPy,
        AxesAlignment::NxNyPz,
        AxesAlignment::NxNzNy,
        AxesAlignment::PyNxPz,
        AxesAlignment::PyNzNx,
        AxesAlignment::PyPxNz,
        AxesAlignment::PyPzPx,
        AxesAlignment::NyPxPz,
        AxesAlignment::NyNzPx,
        AxesAlignment::NyNxNz,
        AxesAlignment::NyPzNx,
        AxesAlignment::PzPyNx,
        AxesAlignment::PzPxPy,
        AxesAlignment::PzNyPx,
        AxesAlignment::PzNxNy,
        AxesAlignment::NzPyPx,
        AxesAlignment::NzNxPy,
        AxesAlignment::NzNyNx,
        AxesAlignment::NzPxNy,
    ];
    let sensor = Vector3::new(1.0, 2.0, 3.0);

    for index in 0..24 {
        let name = c::remap_alignment_to_string(index);
        let alignment = rust_alignments
            .iter()
            .copied()
            .find(|a| a.to_string() == name)
            .unwrap_or_else(|| panic!("no Rust alignment named {name}"));

        assert_eq!(
            arr(axes_swap(sensor, alignment)),
            c::remap(arr(sensor), index),
            "{name}"
        );
    }
}

#[test]
fn c_convention_strings() {
    for convention in [Convention::Nwu, Convention::Enu, Convention::Ned] {
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
            Vector3::from(uncalibrated),
            Matrix3::from_row_slice(&matrix),
            Vector3::from(sensitivity),
            Vector3::from(offset),
        );
        let c = c::model_inertial(uncalibrated, matrix, sensitivity, offset);
        assert_vector(&format!("sample {i}"), "inertial", rust, c, 1e-5);

        let rust = calibrate_magnetic(
            Vector3::from(uncalibrated),
            Matrix3::from_row_slice(&matrix),
            Vector3::from(offset),
        );
        let c = c::model_magnetic(uncalibrated, matrix, offset);
        assert_vector(&format!("sample {i}"), "magnetic", rust, c, 1e-5);
    }
}

/// Rust-only sanity checks on the sensor data
#[test]
fn test_sensor_data_processing() -> Result<(), Box<dyn Error>> {
    // Load sensor data
    let mut reader = csv::Reader::from_path("testdata/sensor_data.csv")?;
    let mut sensor_data = Vec::new();

    for result in reader.deserialize() {
        let record: SensorData = result?;
        sensor_data.push(record);
    }

    // Process with different settings to test various behaviors
    let test_cases = [
        AhrsSettings {
            sample_rate: SAMPLE_RATE,
            convention: Convention::Nwu,
            gain: 0.5,
            gyroscope_range: 2000.0,
            acceleration_rejection: 10.0,
            magnetic_rejection: 10.0,
            rejection_timeout: 5.0,
        },
        AhrsSettings {
            sample_rate: SAMPLE_RATE,
            convention: Convention::Enu,
            gain: 0.5,
            gyroscope_range: 2000.0,
            acceleration_rejection: 10.0,
            magnetic_rejection: 10.0,
            rejection_timeout: 5.0,
        },
        AhrsSettings {
            sample_rate: SAMPLE_RATE,
            convention: Convention::Ned,
            gain: 0.5,
            gyroscope_range: 2000.0,
            acceleration_rejection: 10.0,
            magnetic_rejection: 10.0,
            rejection_timeout: 5.0,
        },
    ];

    for (i, settings) in test_cases.iter().enumerate() {
        let mut ahrs = Ahrs::with_settings(*settings);
        let mut euler_angles = Vec::new();
        let mut delta_times = Vec::new();

        // Calculate delta times
        for j in 0..sensor_data.len() {
            let delta_time = if j == 0 {
                sensor_data[0].time
            } else {
                sensor_data[j].time - sensor_data[j - 1].time
            };
            delta_times.push(delta_time);
        }

        // Process all sensor data
        for (j, data) in sensor_data.iter().enumerate() {
            let gyroscope = Vector3::new(data.gyro_x, data.gyro_y, data.gyro_z);
            let accelerometer = Vector3::new(data.accel_x, data.accel_y, data.accel_z);
            let magnetometer = Vector3::new(data.mag_x, data.mag_y, data.mag_z);

            ahrs.set_sample_period(delta_times[j]);
            ahrs.update(gyroscope, accelerometer, magnetometer);

            let quaternion = ahrs.quaternion();
            let (roll, pitch, yaw) = quaternion.euler_angles();

            euler_angles.push((roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees()));
        }

        // Validate results make sense
        assert!(
            euler_angles.len() == sensor_data.len(),
            "Should have processed all data points for case {}",
            i
        );

        // Check that quaternions are normalized
        let final_quat = ahrs.quaternion();
        let norm = (final_quat.w * final_quat.w
            + final_quat.i * final_quat.i
            + final_quat.j * final_quat.j
            + final_quat.k * final_quat.k)
            .sqrt();
        assert!(
            (norm - 1.0).abs() < 1e-6,
            "Final quaternion should be normalized for case {}",
            i
        );

        // Check that angles are within reasonable bounds
        let (final_roll, final_pitch, final_yaw) = euler_angles.last().unwrap();
        assert!(
            final_roll.abs() < 360.0 && final_pitch.abs() < 360.0 && final_yaw.abs() < 360.0,
            "Euler angles should be bounded for case {}",
            i
        );

        // Check algorithm completed initialization
        assert!(
            !ahrs.flags().startup,
            "Algorithm should have completed initialization for case {}",
            i
        );

        // Validate internal states make sense
        let states = ahrs.internal_states();
        assert!(
            states.acceleration_error >= 0.0 && states.acceleration_error <= 180.0,
            "Acceleration error should be bounded for case {}",
            i
        );
        assert!(
            states.magnetic_error >= 0.0 && states.magnetic_error <= 180.0,
            "Magnetic error should be bounded for case {}",
            i
        );
    }

    Ok(())
}

/// Test consistency between update methods
#[test]
fn test_update_method_consistency() -> Result<(), Box<dyn Error>> {
    // Load a subset of sensor data
    let mut reader = csv::Reader::from_path("testdata/sensor_data.csv")?;
    let mut sensor_data = Vec::new();

    for (i, result) in reader.deserialize().enumerate() {
        if i >= 400 {
            break;
        } // Test first 400 samples (4 seconds)
        let record: SensorData = result?;
        sensor_data.push(record);
    }

    // Test update vs update_no_magnetometer consistency
    let mut ahrs_full = Ahrs::new();
    let mut ahrs_no_mag = Ahrs::new();

    for data in sensor_data.iter() {
        let gyroscope = Vector3::new(data.gyro_x, data.gyro_y, data.gyro_z);
        let accelerometer = Vector3::new(data.accel_x, data.accel_y, data.accel_z);
        let magnetometer = Vector3::new(data.mag_x, data.mag_y, data.mag_z);

        // Update with magnetometer
        ahrs_full.update(gyroscope, accelerometer, magnetometer);

        // Update without magnetometer
        ahrs_no_mag.update_no_magnetometer(gyroscope, accelerometer);
    }

    // Both should have completed initialization
    assert!(!ahrs_full.flags().startup);
    assert!(!ahrs_no_mag.flags().startup);

    // Roll and pitch should be similar (heading will differ)
    let (roll_full, pitch_full, _) = ahrs_full.quaternion().euler_angles();
    let (roll_no_mag, pitch_no_mag, yaw_no_mag) = ahrs_no_mag.quaternion().euler_angles();

    let roll_diff = (roll_full - roll_no_mag).abs().to_degrees();
    let pitch_diff = (pitch_full - pitch_no_mag).abs().to_degrees();

    // Allow some difference due to magnetometer influence
    assert!(
        roll_diff < 10.0,
        "Roll difference too large: {} deg",
        roll_diff
    );
    assert!(
        pitch_diff < 10.0,
        "Pitch difference too large: {} deg",
        pitch_diff
    );

    // No-mag version should have near-zero heading due to heading zeroing
    assert!(
        yaw_no_mag.abs().to_degrees() < 5.0,
        "No-mag heading should be near zero: {} deg",
        yaw_no_mag.to_degrees()
    );

    Ok(())
}

/// Test numerical stability over long sequences
#[test]
fn test_numerical_stability() -> Result<(), Box<dyn Error>> {
    let mut reader = csv::Reader::from_path("testdata/sensor_data.csv")?;
    let mut sensor_data = Vec::new();

    for result in reader.deserialize() {
        let record: SensorData = result?;
        sensor_data.push(record);
    }

    let mut ahrs = Ahrs::new();
    let mut previous_norm = 1.0;

    // Process all data and check quaternion stays normalized
    for (i, data) in sensor_data.iter().enumerate() {
        let gyroscope = Vector3::new(data.gyro_x, data.gyro_y, data.gyro_z);
        let accelerometer = Vector3::new(data.accel_x, data.accel_y, data.accel_z);
        let magnetometer = Vector3::new(data.mag_x, data.mag_y, data.mag_z);

        ahrs.update(gyroscope, accelerometer, magnetometer);

        // Check quaternion normalization every 100 samples
        if i % 100 == 0 {
            let quat = ahrs.quaternion();
            let norm =
                (quat.w * quat.w + quat.i * quat.i + quat.j * quat.j + quat.k * quat.k).sqrt();

            assert!(
                (norm - 1.0).abs() < 1e-5,
                "Quaternion not normalized at sample {}: norm = {}",
                i,
                norm
            );

            // Check norm doesn't drift significantly
            assert!(
                (norm - previous_norm).abs() < 1e-5,
                "Quaternion norm drifting at sample {}: {} -> {}",
                i,
                previous_norm,
                norm
            );

            previous_norm = norm;
        }
    }

    Ok(())
}

/// Test that gravity calculation is consistent with quaternion
#[test]
fn test_gravity_quaternion_consistency() -> Result<(), Box<dyn Error>> {
    let mut reader = csv::Reader::from_path("testdata/sensor_data.csv")?;
    let mut sensor_data = Vec::new();

    // Just use first 200 samples for this test
    for (i, result) in reader.deserialize().enumerate() {
        if i >= 200 {
            break;
        }
        let record: SensorData = result?;
        sensor_data.push(record);
    }

    let mut ahrs = Ahrs::new();

    for (i, data) in sensor_data.iter().enumerate() {
        let gyroscope = Vector3::new(data.gyro_x, data.gyro_y, data.gyro_z);
        let accelerometer = Vector3::new(data.accel_x, data.accel_y, data.accel_z);
        let magnetometer = Vector3::new(data.mag_x, data.mag_y, data.mag_z);

        ahrs.update(gyroscope, accelerometer, magnetometer);

        // Check gravity calculation consistency
        let gravity = ahrs.gravity();
        let quat = ahrs.quaternion();

        // Gravity should be unit length
        assert!(
            (gravity.magnitude() - 1.0).abs() < 1e-5,
            "Gravity magnitude should be 1.0 at sample {}: {}",
            i,
            gravity.magnitude()
        );

        // Manual gravity calculation from quaternion (NWU convention)
        let q = quat.as_ref();
        let expected_gravity = Vector3::new(
            2.0 * (q.i * q.k - q.w * q.j),
            2.0 * (q.j * q.k + q.w * q.i),
            2.0 * (q.w * q.w - 0.5 + q.k * q.k),
        );

        let diff = (gravity - expected_gravity).magnitude();
        assert!(
            diff < 1e-5,
            "Gravity calculation inconsistent at sample {}: diff = {}",
            i,
            diff
        );
    }

    Ok(())
}
