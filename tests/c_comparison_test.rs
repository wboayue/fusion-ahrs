use fusion_ahrs::{Ahrs, AhrsSettings, Convention};
use nalgebra::Vector3;
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

/// This test validates that our Rust implementation produces reasonable results
/// with the actual sensor data. While we can't directly compare with C output
/// without building the C library, we can validate behavioral consistency.
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
            convention: Convention::Nwu,
            gain: 0.5,
            gyroscope_range: 2000.0,
            acceleration_rejection: 10.0,
            magnetic_rejection: 10.0,
            recovery_trigger_period: (5.0 * SAMPLE_RATE) as u32,
        },
        AhrsSettings {
            convention: Convention::Enu,
            gain: 0.5,
            gyroscope_range: 2000.0,
            acceleration_rejection: 10.0,
            magnetic_rejection: 10.0,
            recovery_trigger_period: (5.0 * SAMPLE_RATE) as u32,
        },
        AhrsSettings {
            convention: Convention::Ned,
            gain: 0.5,
            gyroscope_range: 2000.0,
            acceleration_rejection: 10.0,
            magnetic_rejection: 10.0,
            recovery_trigger_period: (5.0 * SAMPLE_RATE) as u32,
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

            ahrs.update(gyroscope, accelerometer, magnetometer, delta_times[j]);

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
            !ahrs.flags().initialising,
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
        let delta_time = 0.01;
        let gyroscope = Vector3::new(data.gyro_x, data.gyro_y, data.gyro_z);
        let accelerometer = Vector3::new(data.accel_x, data.accel_y, data.accel_z);
        let magnetometer = Vector3::new(data.mag_x, data.mag_y, data.mag_z);

        // Update with magnetometer
        ahrs_full.update(gyroscope, accelerometer, magnetometer, delta_time);

        // Update without magnetometer
        ahrs_no_mag.update_no_magnetometer(gyroscope, accelerometer, delta_time);
    }

    // Both should have completed initialization
    assert!(!ahrs_full.flags().initialising);
    assert!(!ahrs_no_mag.flags().initialising);

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
        let delta_time = 0.01;
        let gyroscope = Vector3::new(data.gyro_x, data.gyro_y, data.gyro_z);
        let accelerometer = Vector3::new(data.accel_x, data.accel_y, data.accel_z);
        let magnetometer = Vector3::new(data.mag_x, data.mag_y, data.mag_z);

        ahrs.update(gyroscope, accelerometer, magnetometer, delta_time);

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
        let delta_time = 0.01;
        let gyroscope = Vector3::new(data.gyro_x, data.gyro_y, data.gyro_z);
        let accelerometer = Vector3::new(data.accel_x, data.accel_y, data.accel_z);
        let magnetometer = Vector3::new(data.mag_x, data.mag_y, data.mag_z);

        ahrs.update(gyroscope, accelerometer, magnetometer, delta_time);

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
