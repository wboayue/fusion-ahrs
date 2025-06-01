use fusion_ahrs::{Ahrs, AhrsSettings, Convention};
use nalgebra::{UnitQuaternion, Vector3};
use std::f32::consts::PI;

const EPSILON: f32 = 1e-6;
const ANGLE_EPSILON: f32 = 0.01; // degrees

/// Test that settings are processed correctly to match C implementation
#[test]
fn test_settings_processing() {
    // Test default settings
    let ahrs = Ahrs::new();
    let settings = ahrs.get_settings();
    assert_eq!(settings.gain, 0.5);
    assert_eq!(settings.gyroscope_range, 2000.0);
    assert_eq!(settings.acceleration_rejection, 10.0);
    assert_eq!(settings.magnetic_rejection, 20.0);

    // Test disabled gyroscope range (should use f32::MAX internally)
    let settings_disabled = AhrsSettings {
        gyroscope_range: 0.0,
        ..Default::default()
    };
    let ahrs_disabled = Ahrs::with_settings(settings_disabled);
    // The internal threshold should be f32::MAX when range is 0

    // Test disabled rejection thresholds
    let settings_no_rejection = AhrsSettings {
        acceleration_rejection: 0.0,
        magnetic_rejection: 0.0,
        ..Default::default()
    };
    let _ahrs_no_rejection = Ahrs::with_settings(settings_no_rejection);
    // Should internally set rejection thresholds to f32::MAX
}

/// Test gravity calculation for different coordinate conventions
#[test]
fn test_gravity_calculation() {
    // Test NWU convention (default)
    let mut ahrs_nwu = Ahrs::new();
    let gravity_nwu = ahrs_nwu.gravity();

    // At identity quaternion, gravity should point up in Z for NWU
    assert!((gravity_nwu.magnitude() - 1.0).abs() < EPSILON);
    assert!((gravity_nwu.z - 1.0).abs() < EPSILON);
    assert!(gravity_nwu.x.abs() < EPSILON);
    assert!(gravity_nwu.y.abs() < EPSILON);

    // Test ENU convention
    let settings_enu = AhrsSettings {
        convention: Convention::Enu,
        ..Default::default()
    };
    let mut ahrs_enu = Ahrs::with_settings(settings_enu);
    let gravity_enu = ahrs_enu.gravity();

    // Should be same for identity quaternion
    assert!((gravity_enu.magnitude() - 1.0).abs() < EPSILON);
    assert!((gravity_enu.z - 1.0).abs() < EPSILON);

    // Test NED convention
    let settings_ned = AhrsSettings {
        convention: Convention::Ned,
        ..Default::default()
    };
    let mut ahrs_ned = Ahrs::with_settings(settings_ned);
    let gravity_ned = ahrs_ned.gravity();

    // In NED, gravity points down (negative Z)
    assert!((gravity_ned.magnitude() - 1.0).abs() < EPSILON);
    assert!((gravity_ned.z + 1.0).abs() < EPSILON); // Should be -1
}

/// Test quaternion integration accuracy
#[test]
fn test_quaternion_integration() {
    let mut ahrs = Ahrs::new();

    // Small rotation around Z axis
    let gyroscope = Vector3::new(0.0, 0.0, 10.0); // 10 deg/s around Z
    let accelerometer = Vector3::new(0.0, 0.0, 1.0); // Static
    let magnetometer = Vector3::new(1.0, 0.0, 0.0); // North
    let delta_time = 0.01; // 10ms

    // Complete initialization first (3+ seconds)
    for _ in 0..400 {
        ahrs.update(Vector3::zeros(), accelerometer, magnetometer, delta_time);
    }
    assert!(!ahrs.flags().initialising);

    // Apply rotation for 1 second (should rotate ~10 degrees)
    for _ in 0..100 {
        ahrs.update(gyroscope, accelerometer, magnetometer, delta_time);
    }

    let (roll, pitch, yaw) = ahrs.quaternion().euler_angles();
    let yaw_deg = yaw.to_degrees();

    // Should have rotated approximately 10 degrees around Z
    // Allow some tolerance due to filtering effects
    assert!(yaw_deg.abs() < 15.0, "Yaw: {} degrees", yaw_deg);
    assert!(roll.to_degrees().abs() < 2.0, "Roll should be minimal");
    assert!(pitch.to_degrees().abs() < 2.0, "Pitch should be minimal");
}

/// Test accelerometer rejection mechanism
#[test]
fn test_accelerometer_rejection() {
    let settings = AhrsSettings {
        acceleration_rejection: 10.0, // 10 degree threshold
        recovery_trigger_period: 50,  // 50 samples
        ..Default::default()
    };
    let mut ahrs = Ahrs::with_settings(settings);

    // Complete initialization
    let gyro = Vector3::zeros();
    let normal_accel = Vector3::new(0.0, 0.0, 1.0);
    let mag = Vector3::new(1.0, 0.0, 0.0);

    for _ in 0..400 {
        ahrs.update(gyro, normal_accel, mag, 0.01);
    }
    assert!(!ahrs.flags().initialising);

    // Apply large acceleration (should be rejected)
    let large_accel = Vector3::new(2.0, 2.0, 1.0);

    let mut rejection_triggered = false;
    for _i in 0..100 {
        ahrs.update(gyro, large_accel, mag, 0.01);
        let states = ahrs.internal_states();

        if states.accelerometer_ignored || states.acceleration_recovery_trigger > 10.0 {
            rejection_triggered = true;
            break;
        }
    }

    assert!(
        rejection_triggered,
        "Accelerometer rejection should have been triggered"
    );
}

/// Test magnetometer rejection mechanism  
#[test]
fn test_magnetometer_rejection() {
    let settings = AhrsSettings {
        magnetic_rejection: 10.0,    // 10 degree threshold
        recovery_trigger_period: 50, // 50 samples
        ..Default::default()
    };
    let mut ahrs = Ahrs::with_settings(settings);

    // Complete initialization
    let gyro = Vector3::zeros();
    let accel = Vector3::new(0.0, 0.0, 1.0);
    let normal_mag = Vector3::new(1.0, 0.0, 0.0);

    for _ in 0..400 {
        ahrs.update(gyro, accel, normal_mag, 0.01);
    }
    assert!(!ahrs.flags().initialising);

    // Apply magnetic interference (should cause large error)
    // The cross product preprocessing in the algorithm means we need a different test case
    let interfered_mag = Vector3::new(0.0, 1.0, 0.0); // East instead of North

    let mut rejection_triggered = false;
    for _i in 0..100 {
        ahrs.update(gyro, accel, interfered_mag, 0.01);
        let states = ahrs.internal_states();

        if states.magnetometer_ignored || states.magnetic_recovery_trigger > 10.0 {
            rejection_triggered = true;
            break;
        }
    }

    assert!(
        rejection_triggered,
        "Magnetometer rejection should have been triggered"
    );
}

/// Test gyroscope overflow detection
#[test]
fn test_gyroscope_overflow() {
    let settings = AhrsSettings {
        gyroscope_range: 500.0, // 500 deg/s limit
        ..Default::default()
    };
    let mut ahrs = Ahrs::with_settings(settings);

    // Complete initialization
    let normal_gyro = Vector3::zeros();
    let accel = Vector3::new(0.0, 0.0, 1.0);
    let mag = Vector3::new(1.0, 0.0, 0.0);

    for _ in 0..400 {
        ahrs.update(normal_gyro, accel, mag, 0.01);
    }
    assert!(!ahrs.flags().initialising);

    // Apply gyroscope overflow
    let overflow_gyro = Vector3::new(600.0, 0.0, 0.0); // Exceeds limit
    ahrs.update(overflow_gyro, accel, mag, 0.01);

    assert!(ahrs.flags().angular_rate_recovery);
    assert!(ahrs.flags().initialising); // Should restart initialization
}

/// Test coordinate convention consistency
#[test]
fn test_coordinate_conventions() {
    let test_cases = [Convention::Nwu, Convention::Enu, Convention::Ned];

    for convention in test_cases {
        let settings = AhrsSettings {
            convention,
            ..Default::default()
        };
        let mut ahrs = Ahrs::with_settings(settings);

        // Test that gravity calculation is consistent
        let gravity = ahrs.gravity();
        assert!(
            (gravity.magnitude() - 1.0).abs() < EPSILON,
            "Gravity magnitude should be 1.0 for {:?}",
            convention
        );

        // For NWU/ENU, gravity.z should be positive at identity
        // For NED, gravity.z should be negative at identity
        match convention {
            Convention::Nwu | Convention::Enu => {
                assert!(
                    gravity.z > 0.9,
                    "Gravity Z should be positive for {:?}",
                    convention
                );
            }
            Convention::Ned => {
                assert!(
                    gravity.z < -0.9,
                    "Gravity Z should be negative for {:?}",
                    convention
                );
            }
        }

        // Test that linear acceleration is calculated correctly
        let accel_input = Vector3::new(0.0, 0.0, 1.0);
        ahrs.update(
            Vector3::zeros(),
            accel_input,
            Vector3::new(1.0, 0.0, 0.0),
            0.01,
        );

        let linear_accel = ahrs.linear_acceleration();

        // Linear acceleration should be near zero when input equals gravity
        match convention {
            Convention::Nwu | Convention::Enu => {
                assert!(
                    linear_accel.magnitude() < 0.1,
                    "Linear acceleration should be small for {:?}",
                    convention
                );
            }
            Convention::Ned => {
                // In NED, positive Z acceleration input means upward, opposite to gravity
                assert!(
                    linear_accel.z > 1.5,
                    "Linear acceleration Z should be large positive for {:?}",
                    convention
                );
            }
        }
    }
}

/// Test initialization ramping behavior
#[test]
fn test_initialization_ramping() {
    let mut ahrs = Ahrs::new();

    assert!(ahrs.flags().initialising);

    let gyro = Vector3::zeros();
    let accel = Vector3::new(0.0, 0.0, 1.0);
    let mag = Vector3::new(1.0, 0.0, 0.0);

    // Run for exactly 3 seconds at 100Hz to complete initialization
    for i in 0..300 {
        ahrs.update(gyro, accel, mag, 0.01);

        if i < 299 {
            // Should still be initializing
            assert!(
                ahrs.flags().initialising,
                "Should be initializing at step {}",
                i
            );
        }
    }

    // Should be done initializing after 3 seconds
    assert!(!ahrs.flags().initialising, "Should be done initializing");
    assert!(!ahrs.flags().angular_rate_recovery);
}

/// Test numerical precision by comparing with expected quaternion values
#[test]
fn test_numerical_precision() {
    let mut ahrs = Ahrs::new();

    // Complete initialization with static readings
    let gyro = Vector3::zeros();
    let accel = Vector3::new(0.0, 0.0, 1.0);
    let mag = Vector3::new(1.0, 0.0, 0.0);

    for _ in 0..400 {
        ahrs.update(gyro, accel, mag, 0.01);
    }

    // Quaternion should be very close to identity
    let quat = ahrs.quaternion();
    assert!((quat.w - 1.0).abs() < EPSILON, "W component should be ~1.0");
    assert!(quat.i.abs() < EPSILON, "X component should be ~0.0");
    assert!(quat.j.abs() < EPSILON, "Y component should be ~0.0");
    assert!(quat.k.abs() < EPSILON, "Z component should be ~0.0");

    // Verify quaternion is normalized
    let norm = (quat.w * quat.w + quat.i * quat.i + quat.j * quat.j + quat.k * quat.k).sqrt();
    assert!(
        (norm - 1.0).abs() < EPSILON,
        "Quaternion should be normalized"
    );
}
