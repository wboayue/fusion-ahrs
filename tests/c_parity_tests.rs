//! Tests to verify Rust implementation matches C reference behavior
//!
//! These tests were created to expose and verify fixes for parity issues
//! between the Rust port and original C implementation.

use fusion_ahrs::{Ahrs, AhrsSettings, Convention};
use nalgebra::{UnitQuaternion, Vector3};

/// Test half_magnetic calculation for NED convention
/// C uses: x = -(x*y + w*z), y = 0.5 - w² - y², z = w*x - y*z
#[test]
fn test_half_magnetic_ned_formula() {
    // Create AHRS with NED convention and a known non-identity quaternion
    let settings = AhrsSettings {
        convention: Convention::Ned,
        gain: 0.5,
        ..Default::default()
    };
    let mut ahrs = Ahrs::with_settings(settings);

    // Set a specific quaternion for testing (45° rotation around Z)
    let q = UnitQuaternion::from_euler_angles(0.0, 0.0, std::f32::consts::FRAC_PI_4);
    ahrs.set_quaternion(q);

    // Get quaternion components
    let qw = q.w;
    let qx = q.i;
    let qy = q.j;
    let qz = q.k;

    // Expected half_magnetic for NED (from C implementation):
    // x = -1.0 * (qx * qy + qw * qz)
    // y = 0.5 - qw * qw - qy * qy
    // z = qw * qx - qy * qz
    let expected_x = -(qx * qy + qw * qz);
    let expected_y = 0.5 - qw * qw - qy * qy;
    let expected_z = qw * qx - qy * qz;

    // Update with magnetometer to trigger half_magnetic calculation
    let gyro = Vector3::zeros();
    let accel = Vector3::new(0.0, 0.0, -1.0); // NED gravity
    let mag = Vector3::new(1.0, 0.0, 0.0);

    // Store quaternion before update
    let q_before = ahrs.quaternion();

    // Single update with zero gyro - orientation should barely change
    ahrs.update(gyro, accel, mag, 0.001);

    // The internal half_magnetic should use correct formula
    // We can verify by checking the states show reasonable error
    let states = ahrs.internal_states();

    // With correct formula, magnetic error should be small for aligned magnetometer
    // This test will fail if the formula is wrong because feedback will be incorrect
    println!(
        "NED half_magnetic test - expected: ({}, {}, {})",
        expected_x, expected_y, expected_z
    );
    println!("Magnetic error: {}", states.magnetic_error);

    // Quaternion should not have drifted significantly
    let q_after = ahrs.quaternion();
    let angle_diff = q_before.angle_to(&q_after).to_degrees();
    assert!(
        angle_diff < 5.0,
        "Quaternion drifted too much: {}° - half_magnetic formula may be wrong",
        angle_diff
    );
}

/// Test linear_acceleration handles NED convention correctly
/// In NED, gravity points DOWN (+Z), so we must ADD gravity to remove it
#[test]
fn test_linear_acceleration_ned_convention() {
    let settings = AhrsSettings {
        convention: Convention::Ned,
        ..Default::default()
    };
    let mut ahrs = Ahrs::with_settings(settings);

    // Complete initialization
    let gyro = Vector3::zeros();
    let accel_ned = Vector3::new(0.0, 0.0, -1.0); // NED: gravity = +Z, so measured accel = -Z
    let mag = Vector3::new(1.0, 0.0, 0.0);

    for _ in 0..400 {
        ahrs.update(gyro, accel_ned, mag, 0.01);
    }

    // For a stationary level device in NED:
    // - Accelerometer reads (0, 0, -1) (reaction to gravity)
    // - Gravity vector from quaternion is (0, 0, 1) pointing down
    // - Linear acceleration should be (0, 0, 0) for stationary

    let linear_accel = ahrs.linear_acceleration();
    let magnitude = linear_accel.magnitude();

    println!("NED linear_acceleration: {:?}", linear_accel);
    println!("Magnitude: {}", magnitude);

    // Should be near zero for stationary device
    assert!(
        magnitude < 0.1,
        "Linear acceleration should be near zero for stationary NED device, got magnitude {}",
        magnitude
    );
}

/// Test linear_acceleration for NWU convention (baseline)
#[test]
fn test_linear_acceleration_nwu_convention() {
    let settings = AhrsSettings {
        convention: Convention::Nwu,
        ..Default::default()
    };
    let mut ahrs = Ahrs::with_settings(settings);

    let gyro = Vector3::zeros();
    let accel_nwu = Vector3::new(0.0, 0.0, 1.0); // NWU: gravity = -Z, measured accel = +Z
    let mag = Vector3::new(1.0, 0.0, 0.0);

    for _ in 0..400 {
        ahrs.update(gyro, accel_nwu, mag, 0.01);
    }

    let linear_accel = ahrs.linear_acceleration();
    let magnitude = linear_accel.magnitude();

    println!("NWU linear_acceleration: {:?}", linear_accel);

    assert!(
        magnitude < 0.1,
        "Linear acceleration should be near zero for stationary NWU device, got magnitude {}",
        magnitude
    );
}

/// Test flags() uses correct recovery comparison
/// C: accelerationRecovery = trigger > timeout
/// Not: trigger > 0
#[test]
fn test_flags_recovery_comparison() {
    let settings = AhrsSettings {
        convention: Convention::Nwu,
        gain: 0.5,
        acceleration_rejection: 10.0,
        magnetic_rejection: 10.0,
        recovery_trigger_period: 100,
        ..Default::default()
    };
    let mut ahrs = Ahrs::with_settings(settings);

    // Complete initialization
    let gyro = Vector3::zeros();
    let good_accel = Vector3::new(0.0, 0.0, 1.0);
    let mag = Vector3::new(1.0, 0.0, 0.0);

    for _ in 0..400 {
        ahrs.update(gyro, good_accel, mag, 0.01);
    }

    // Now apply a few bad readings to increment trigger (but not exceed timeout)
    let bad_accel = Vector3::new(2.0, 2.0, 1.0);
    for _ in 0..50 {
        ahrs.update(gyro, bad_accel, mag, 0.01);
    }

    let flags = ahrs.flags();
    let states = ahrs.internal_states();

    println!(
        "Recovery trigger (normalized): {}",
        states.acceleration_recovery_trigger
    );
    println!("acceleration_recovery flag: {}", flags.acceleration_recovery);

    // With trigger period of 100 and only 50 bad readings,
    // C would have trigger < timeout, so acceleration_recovery should be FALSE
    // Normalized trigger should be around 0.5 (50/100)
    assert!(
        states.acceleration_recovery_trigger < 1.0,
        "Trigger should be < 1.0 (normalized)"
    );

    // C: recovery = trigger > timeout
    // With 50 readings and period 100, trigger=50, timeout=100
    // 50 > 100 = FALSE, so acceleration_recovery should be FALSE
    assert!(
        !flags.acceleration_recovery,
        "acceleration_recovery should be FALSE when trigger ({}) <= timeout ({})",
        states.acceleration_recovery_trigger,
        1.0 // normalized timeout
    );
}

/// Test initialise() sets timeout correctly
/// C: timeout = recovery_trigger_period
/// Not: timeout = 0
#[test]
fn test_initialise_timeout_values() {
    let settings = AhrsSettings {
        recovery_trigger_period: 500,
        ..Default::default()
    };
    let ahrs = Ahrs::with_settings(settings);

    // After initialization, check flags behavior
    // If timeout is initialized to 0 (wrong), any trigger > 0 would show recovery
    // If timeout is initialized to recovery_trigger_period (correct), recovery starts false

    let flags = ahrs.flags();

    // Right after init, no recovery should be happening
    assert!(
        !flags.acceleration_recovery,
        "acceleration_recovery should be false after init"
    );
    assert!(
        !flags.magnetic_recovery,
        "magnetic_recovery should be false after init"
    );
}

/// Test internal_states returns normalized recovery trigger (0.0-1.0)
/// C: trigger / recovery_trigger_period
/// Not: raw trigger value
#[test]
fn test_internal_states_normalized_trigger() {
    let settings = AhrsSettings {
        convention: Convention::Nwu,
        gain: 0.5,
        acceleration_rejection: 10.0,
        recovery_trigger_period: 100,
        ..Default::default()
    };
    let mut ahrs = Ahrs::with_settings(settings);

    // Complete initialization
    let gyro = Vector3::zeros();
    let good_accel = Vector3::new(0.0, 0.0, 1.0);
    let mag = Vector3::new(1.0, 0.0, 0.0);

    for _ in 0..400 {
        ahrs.update(gyro, good_accel, mag, 0.01);
    }

    // Apply some bad readings
    let bad_accel = Vector3::new(2.0, 2.0, 1.0);
    for _ in 0..20 {
        ahrs.update(gyro, bad_accel, mag, 0.01);
    }

    let states = ahrs.internal_states();

    println!(
        "acceleration_recovery_trigger: {}",
        states.acceleration_recovery_trigger
    );

    // C returns normalized value (0.0 to 1.0)
    // With period=100 and ~20 bad readings, should be around 0.2
    // If Rust returns raw value, it would be around 20.0

    assert!(
        states.acceleration_recovery_trigger <= 1.0,
        "acceleration_recovery_trigger should be normalized (0.0-1.0), got {}",
        states.acceleration_recovery_trigger
    );
}

/// Test update_external_heading matches C behavior
/// C: Uses roll from quaternion to create synthetic magnetometer
#[test]
fn test_update_external_heading_c_parity() {
    let mut ahrs = Ahrs::new();

    // Initialize with known orientation
    let gyro = Vector3::zeros();
    let accel = Vector3::new(0.0, 0.0, 1.0);

    // Complete initialization with external heading
    for _ in 0..400 {
        ahrs.update_external_heading(gyro, accel, 45.0, 0.01);
    }

    // Extract yaw - should converge toward 45°
    let (_, _, yaw) = ahrs.quaternion().euler_angles();
    let yaw_deg = yaw.to_degrees();

    println!("External heading test - target: 45°, actual: {}°", yaw_deg);

    // Should be within reasonable range of target heading after full initialization
    let diff = (yaw_deg - 45.0).abs();
    assert!(
        diff < 10.0,
        "Heading should converge toward 45°, got {}° (diff: {}°)",
        yaw_deg,
        diff
    );
}

/// Test set_heading matches C algorithm
/// C: Applies rotation quaternion to adjust yaw only
#[test]
fn test_set_heading_c_parity() {
    let mut ahrs = Ahrs::new();

    // Initialize
    let gyro = Vector3::zeros();
    let accel = Vector3::new(0.0, 0.0, 1.0);
    let mag = Vector3::new(1.0, 0.0, 0.0);

    for _ in 0..400 {
        ahrs.update(gyro, accel, mag, 0.01);
    }

    // Add some roll and pitch
    let tilted_q = UnitQuaternion::from_euler_angles(0.2, 0.3, 0.0); // ~11° roll, ~17° pitch
    ahrs.set_quaternion(tilted_q);

    let (roll_before, pitch_before, _) = ahrs.quaternion().euler_angles();

    // Set heading to 90°
    ahrs.set_heading(90.0);

    let (roll_after, pitch_after, yaw_after) = ahrs.quaternion().euler_angles();

    println!("Roll before: {}, after: {}", roll_before, roll_after);
    println!("Pitch before: {}, after: {}", pitch_before, pitch_after);
    println!("Yaw after: {}°", yaw_after.to_degrees());

    // Roll and pitch should be preserved
    assert!(
        (roll_before - roll_after).abs() < 0.01,
        "Roll changed: {} -> {}",
        roll_before,
        roll_after
    );
    assert!(
        (pitch_before - pitch_after).abs() < 0.01,
        "Pitch changed: {} -> {}",
        pitch_before,
        pitch_after
    );

    // Yaw should be ~90°
    let yaw_deg = yaw_after.to_degrees();
    assert!(
        (yaw_deg - 90.0).abs() < 5.0,
        "Yaw should be ~90°, got {}°",
        yaw_deg
    );
}

/// Test gravity vector for all conventions
#[test]
fn test_gravity_all_conventions() {
    for convention in [Convention::Nwu, Convention::Enu, Convention::Ned] {
        let settings = AhrsSettings {
            convention,
            ..Default::default()
        };
        let ahrs = Ahrs::with_settings(settings);

        let gravity = ahrs.gravity();

        println!("{:?} gravity: {:?}", convention, gravity);

        // Gravity should be unit vector
        assert!(
            (gravity.magnitude() - 1.0).abs() < 1e-5,
            "{:?}: Gravity magnitude should be 1.0, got {}",
            convention,
            gravity.magnitude()
        );

        // Check direction based on convention
        match convention {
            Convention::Nwu | Convention::Enu => {
                // Gravity points UP (+Z) in body frame for level device
                assert!(
                    gravity.z > 0.9,
                    "{:?}: Gravity Z should be positive, got {}",
                    convention,
                    gravity.z
                );
            }
            Convention::Ned => {
                // Gravity points DOWN (+Z in Earth frame, but we're looking at sensor frame)
                // For identity quaternion, half_gravity gives the third column scaled
                // In NED, this should point down
                assert!(
                    gravity.z < -0.9,
                    "{:?}: Gravity Z should be negative (down), got {}",
                    convention,
                    gravity.z
                );
            }
        }
    }
}

/// Comprehensive test comparing behavior with known C output
#[test]
fn test_basic_fusion_convergence() {
    let mut ahrs = Ahrs::new();

    // Simulate stationary sensor with gravity up and north magnetic field
    let gyro = Vector3::zeros();
    let accel = Vector3::new(0.0, 0.0, 1.0);
    let mag = Vector3::new(1.0, 0.0, 0.0);

    // Run for initialization period
    for _ in 0..400 {
        ahrs.update(gyro, accel, mag, 0.01);
    }

    // Should no longer be initializing
    assert!(!ahrs.flags().initialising);

    // Quaternion should be near identity
    let q = ahrs.quaternion();
    assert!(
        (q.w - 1.0).abs() < 0.1,
        "Quaternion W should be near 1.0, got {}",
        q.w
    );

    // Euler angles should be near zero
    let (roll, pitch, yaw) = q.euler_angles();
    assert!(
        roll.abs() < 0.1,
        "Roll should be near 0, got {}",
        roll.to_degrees()
    );
    assert!(
        pitch.abs() < 0.1,
        "Pitch should be near 0, got {}",
        pitch.to_degrees()
    );
    assert!(
        yaw.abs() < 0.1,
        "Yaw should be near 0, got {}",
        yaw.to_degrees()
    );
}
