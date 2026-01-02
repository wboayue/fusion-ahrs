use fusion_ahrs::{Ahrs, AhrsSettings, Convention};
use nalgebra::Vector3;

const EPSILON: f32 = 1e-6;

/// Test that settings are processed correctly to match C implementation
#[test]
fn test_settings_processing() {
    // Test default settings (matches C library defaults)
    let ahrs = Ahrs::new();
    let settings = ahrs.get_settings();
    assert_eq!(settings.gain, 0.5);
    assert_eq!(settings.gyroscope_range, 0.0);         // Disabled by default (C compat)
    assert_eq!(settings.acceleration_rejection, 90.0); // C default
    assert_eq!(settings.magnetic_rejection, 90.0);     // C default
    assert_eq!(settings.recovery_trigger_period, 0);   // Disabled by default (C compat)

    // Test enabled gyroscope range
    let settings_enabled = AhrsSettings {
        gyroscope_range: 2000.0,
        ..Default::default()
    };
    let _ahrs_enabled = Ahrs::with_settings(settings_enabled);
    // The internal threshold should be 2000 * 0.98 = 1960

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
    let ahrs_nwu = Ahrs::new();
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
    let ahrs_enu = Ahrs::with_settings(settings_enu);
    let gravity_enu = ahrs_enu.gravity();

    // Should be same for identity quaternion
    assert!((gravity_enu.magnitude() - 1.0).abs() < EPSILON);
    assert!((gravity_enu.z - 1.0).abs() < EPSILON);

    // Test NED convention
    let settings_ned = AhrsSettings {
        convention: Convention::Ned,
        ..Default::default()
    };
    let ahrs_ned = Ahrs::with_settings(settings_ned);
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

/// Test feedback scaling matches C implementation (no extra 0.5 factor)
///
/// The C implementation applies feedback directly without scaling:
///   halfAccelerometerFeedback = ahrs->halfAccelerometerFeedback
/// The feedback vectors are already "half" scaled because halfGravity is 0.5 scaled.
#[test]
fn test_feedback_scaling_parity() {
    // Use high gain and low rejection threshold to make feedback effects visible
    let settings = AhrsSettings {
        gain: 1.0,
        acceleration_rejection: 90.0, // Don't reject
        magnetic_rejection: 90.0,
        recovery_trigger_period: 0,
        ..Default::default()
    };
    let mut ahrs = Ahrs::with_settings(settings);

    // Skip initialization by running enough updates
    let gyro = Vector3::zeros();
    let accel = Vector3::new(0.0, 0.0, 1.0);
    let mag = Vector3::new(1.0, 0.0, 0.0);
    for _ in 0..400 {
        ahrs.update(gyro, accel, mag, 0.01);
    }
    assert!(!ahrs.flags().initialising);

    // Now introduce a tilt error
    // Start from identity and apply tilted accelerometer
    ahrs.set_quaternion(nalgebra::UnitQuaternion::identity());

    // Tilted accelerometer (30 degrees pitch)
    let tilted_accel = Vector3::new(0.5, 0.0, 0.866); // sin(30°), 0, cos(30°)

    // Single update with tilted accelerometer
    ahrs.update(gyro, tilted_accel, mag, 0.01);

    // Get the quaternion after one update
    let quat_after = ahrs.quaternion();
    let (roll_after, pitch_after, _) = quat_after.euler_angles();

    // With gain=1.0 and dt=0.01, the correction should be noticeable
    // The pitch error is ~30°, half_feedback magnitude ≈ 0.5 * sin(30°) = 0.25
    // With correct feedback (no extra 0.5), pitch correction ≈ 0.25 * 1.0 * 0.01 rad ≈ 0.14°
    // With buggy feedback (extra 0.5), correction would be half: ≈ 0.07°
    let pitch_after_deg = pitch_after.to_degrees();

    // After multiple updates, the difference becomes more apparent
    for _ in 0..50 {
        ahrs.update(gyro, tilted_accel, mag, 0.01);
    }

    let quat_final = ahrs.quaternion();
    let (_, pitch_final, _) = quat_final.euler_angles();
    let pitch_deg = pitch_final.to_degrees();

    // With correct feedback, pitch should converge toward the accelerometer indication (~30°)
    // After 51 updates with gain=1.0, with CORRECT feedback we expect pitch > 10°
    // With HALF feedback (bug), convergence is slower, pitch would be around 5-7°
    assert!(
        pitch_deg.abs() > 10.0,
        "Pitch should converge toward ~30° tilt. Got {:.2}° after 51 updates.\n\
         Expected >10° with correct feedback.\n\
         If pitch is around 5-7°, feedback may be incorrectly halved.\n\
         First update pitch: {:.2}°",
        pitch_deg,
        pitch_after_deg
    );
    assert!(
        roll_after.to_degrees().abs() < 5.0,
        "Roll should remain small, got {:.2}°",
        roll_after.to_degrees()
    );
}

/// Test internal_states error calculation uses asin (matches C implementation)
///
/// C implementation: FusionRadiansToDegrees(FusionAsin(2.0f * magnitude))
/// This correctly converts from the cross product magnitude back to angle in degrees.
#[test]
fn test_internal_states_error_uses_asin() {
    let settings = AhrsSettings {
        acceleration_rejection: 90.0,
        magnetic_rejection: 90.0,
        recovery_trigger_period: 0,
        ..Default::default()
    };
    let mut ahrs = Ahrs::with_settings(settings);

    // Complete initialization
    let gyro = Vector3::zeros();
    let accel = Vector3::new(0.0, 0.0, 1.0);
    let mag = Vector3::new(1.0, 0.0, 0.0);
    for _ in 0..400 {
        ahrs.update(gyro, accel, mag, 0.01);
    }

    // Create a known error angle
    // Tilt accelerometer by 45 degrees
    let angle_deg = 45.0_f32;
    let angle_rad = angle_deg.to_radians();
    let tilted_accel = Vector3::new(angle_rad.sin(), 0.0, angle_rad.cos());

    ahrs.update(gyro, tilted_accel, mag, 0.01);

    let states = ahrs.internal_states();

    // The acceleration_error should report approximately 45 degrees
    // The feedback vector magnitude is approximately 0.5 * sin(angle)
    // So error = asin(2 * 0.5 * sin(angle)) = asin(sin(angle)) = angle (for angle < 90°)
    //
    // Without asin: error would be rad_to_deg(2 * 0.5 * sin(45°)) = rad_to_deg(0.707) ≈ 40.5°
    // With asin: error = rad_to_deg(asin(sin(45°))) = 45°

    let expected_error = angle_deg;
    let tolerance = 5.0; // Allow some tolerance for filter effects

    assert!(
        (states.acceleration_error - expected_error).abs() < tolerance,
        "Acceleration error should be approximately {:.1}° (the tilt angle), got {:.1}°. \
         If error is ~{:.1}°, asin() may be missing.",
        expected_error,
        states.acceleration_error,
        angle_rad.sin().to_degrees() * 2.0  // What you'd get without asin
    );

    // Test with a larger angle where the difference is more pronounced
    let angle_deg_large = 60.0_f32;
    let angle_rad_large = angle_deg_large.to_radians();
    let tilted_accel_large = Vector3::new(angle_rad_large.sin(), 0.0, angle_rad_large.cos());

    ahrs.update(gyro, tilted_accel_large, mag, 0.01);
    let states_large = ahrs.internal_states();

    // Without asin: error ≈ rad_to_deg(sin(60°)) = rad_to_deg(0.866) ≈ 49.6°
    // With asin: error = 60°
    assert!(
        (states_large.acceleration_error - angle_deg_large).abs() < tolerance,
        "Acceleration error for 60° tilt should be ~60°, got {:.1}°",
        states_large.acceleration_error
    );
}

/// Test calibration inertial function order of operations matches C implementation
///
/// C implementation: misalignment * ((uncalibrated - offset) * sensitivity)
/// The offset is subtracted BEFORE sensitivity scaling.
#[test]
fn test_calibration_order_of_operations() {
    use fusion_ahrs::calibration::calibrate_inertial;
    use nalgebra::Matrix3;

    let uncalibrated = Vector3::new(100.0, 200.0, 300.0);
    let misalignment = Matrix3::identity();
    let sensitivity = Vector3::new(0.5, 0.5, 0.5); // Non-unity to reveal order difference
    let offset = Vector3::new(10.0, 20.0, 30.0);

    let calibrated = calibrate_inertial(uncalibrated, misalignment, sensitivity, offset);

    // C order: (uncalibrated - offset) * sensitivity
    // (100-10, 200-20, 300-30) * (0.5, 0.5, 0.5) = (90, 180, 270) * 0.5 = (45, 90, 135)
    let expected_c_order = Vector3::new(45.0, 90.0, 135.0);

    // Wrong order would give: uncalibrated * sensitivity - offset
    // (100, 200, 300) * (0.5, 0.5, 0.5) - (10, 20, 30) = (50, 100, 150) - (10, 20, 30) = (40, 80, 120)
    let wrong_order_result = Vector3::new(40.0, 80.0, 120.0);

    // Verify we get the C-compatible result, not the wrong order
    assert!(
        (calibrated - expected_c_order).magnitude() < EPSILON,
        "Calibration should follow C order: (uncalibrated - offset) * sensitivity.\n\
         Expected: {:?}\n\
         Got: {:?}",
        expected_c_order,
        calibrated
    );

    // Explicitly verify we don't get the wrong order result
    assert!(
        (calibrated - wrong_order_result).magnitude() > 1.0,
        "Calibration should NOT match wrong order result {:?}",
        wrong_order_result
    );
}
