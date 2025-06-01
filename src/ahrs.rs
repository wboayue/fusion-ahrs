//! Main AHRS algorithm implementation for the Fusion AHRS library

use nalgebra::{UnitQuaternion, Vector3, Quaternion};
use crate::types::{AhrsSettings, AhrsInternalStates, AhrsFlags, Convention};
use crate::math::{Vector3Ext, DEG_TO_RAD};

/// AHRS algorithm constants
const INITIAL_GAIN: f32 = 10.0;
const INITIALISATION_PERIOD: f32 = 3.0; // seconds
const GYROSCOPE_RANGE_FACTOR: f32 = 0.98;
const RECOVERY_DECREMENT: i32 = 9;

/// Main AHRS algorithm structure
/// 
/// Implements a complementary filter that fuses gyroscope, accelerometer, 
/// and magnetometer data to estimate orientation. Features automatic sensor
/// rejection during motion/interference and recovery mechanisms.
pub struct Ahrs {
    /// Algorithm settings
    settings: AhrsSettings,
    /// Current orientation quaternion (WXYZ format)
    quaternion: UnitQuaternion<f32>,
    /// Last accelerometer reading for linear acceleration calculation
    accelerometer: Vector3<f32>,
    /// Whether algorithm is initializing
    initialising: bool,
    /// Ramped gain value during initialization
    ramped_gain: f32,
    /// Gain ramping step size per update
    ramped_gain_step: f32,
    /// Angular rate recovery flag
    angular_rate_recovery: bool,
    /// Half accelerometer feedback vector (cached for efficiency)
    half_accelerometer_feedback: Vector3<f32>,
    /// Half magnetometer feedback vector (cached for efficiency)
    half_magnetometer_feedback: Vector3<f32>,
    /// Accelerometer ignored flag
    accelerometer_ignored: bool,
    /// Acceleration recovery trigger countdown
    acceleration_recovery_trigger: i32,
    /// Acceleration recovery timeout
    acceleration_recovery_timeout: u32,
    /// Magnetometer ignored flag
    magnetometer_ignored: bool,
    /// Magnetic recovery trigger countdown
    magnetic_recovery_trigger: i32,
    /// Magnetic recovery timeout
    magnetic_recovery_timeout: u32,
    /// Processed rejection thresholds (squared for efficiency)
    acceleration_rejection_squared: f32,
    magnetic_rejection_squared: f32,
    /// Processed gyroscope range threshold
    gyroscope_range_threshold: f32,
}

impl Ahrs {
    /// Create a new AHRS instance with default settings
    pub fn new() -> Self {
        Self::with_settings(AhrsSettings::default())
    }

    /// Create a new AHRS instance with specified settings
    pub fn with_settings(settings: AhrsSettings) -> Self {
        let mut ahrs = Ahrs {
            settings,
            quaternion: UnitQuaternion::identity(),
            accelerometer: Vector3::zeros(),
            initialising: true,
            ramped_gain: 0.0,
            ramped_gain_step: 0.0,
            angular_rate_recovery: false,
            half_accelerometer_feedback: Vector3::zeros(),
            half_magnetometer_feedback: Vector3::zeros(),
            accelerometer_ignored: false,
            acceleration_recovery_trigger: 0,
            acceleration_recovery_timeout: 0,
            magnetometer_ignored: false,
            magnetic_recovery_trigger: 0,
            magnetic_recovery_timeout: 0,
            acceleration_rejection_squared: 0.0,
            magnetic_rejection_squared: 0.0,
            gyroscope_range_threshold: 0.0,
        };
        
        ahrs.process_settings();
        ahrs.initialise();
        ahrs
    }

    /// Initialize/reset the AHRS algorithm
    pub fn initialise(&mut self) {
        self.quaternion = UnitQuaternion::identity();
        self.accelerometer = Vector3::zeros();
        self.initialising = true;
        self.ramped_gain = INITIAL_GAIN;
        self.ramped_gain_step = 0.0; // Will be set when sample rate is known
        self.angular_rate_recovery = false;
        self.half_accelerometer_feedback = Vector3::zeros();
        self.half_magnetometer_feedback = Vector3::zeros();
        self.accelerometer_ignored = false;
        self.acceleration_recovery_trigger = 0;
        self.acceleration_recovery_timeout = 0;
        self.magnetometer_ignored = false;
        self.magnetic_recovery_trigger = 0;
        self.magnetic_recovery_timeout = 0;
    }

    /// Reset the algorithm (alias for initialise)
    pub fn reset(&mut self) {
        self.initialise();
    }

    /// Update algorithm settings
    pub fn set_settings(&mut self, settings: AhrsSettings) {
        self.settings = settings;
        self.process_settings();
    }

    /// Get current algorithm settings
    pub fn get_settings(&self) -> AhrsSettings {
        self.settings
    }

    /// Update AHRS with gyroscope, accelerometer, and magnetometer data
    /// 
    /// # Arguments
    /// * `gyroscope` - Gyroscope reading in degrees per second
    /// * `accelerometer` - Accelerometer reading in g
    /// * `magnetometer` - Magnetometer reading in µT
    /// * `delta_time` - Time step in seconds
    pub fn update(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        magnetometer: Vector3<f32>,
        delta_time: f32,
    ) {
        // Store accelerometer for linear acceleration calculation
        self.accelerometer = accelerometer;
        
        // Set up gain ramping step if not initialized
        if self.ramped_gain_step == 0.0 && delta_time > 0.0 {
            self.ramped_gain_step = (INITIAL_GAIN - self.settings.gain) / INITIALISATION_PERIOD * delta_time;
        }
        
        // Check for gyroscope overflow
        if self.gyroscope_range_threshold > 0.0 {
            let gyroscope_magnitude = gyroscope.magnitude();
            if gyroscope_magnitude > self.gyroscope_range_threshold {
                self.angular_rate_recovery = true;
                self.initialising = true;
                self.ramped_gain = INITIAL_GAIN;
            }
        }
        
        // Ramp down gain during initialization
        if self.initialising {
            if self.ramped_gain > self.settings.gain {
                self.ramped_gain -= self.ramped_gain_step;
                if self.ramped_gain < self.settings.gain {
                    self.ramped_gain = self.settings.gain;
                }
            } else {
                self.initialising = false;
                self.angular_rate_recovery = false;
            }
        }
        
        // Calculate gravity direction in sensor frame
        let half_gravity = self.calculate_half_gravity();
        
        // Calculate accelerometer feedback
        self.half_accelerometer_feedback = Vector3::zeros();
        if !self.accelerometer_ignored {
            let accelerometer_normalized = accelerometer.safe_normalize();
            if accelerometer_normalized.magnitude() > 0.0 {
                let feedback = self.calculate_feedback(accelerometer_normalized, half_gravity);
                let error_squared = feedback.magnitude_squared();
                
                if error_squared <= self.acceleration_rejection_squared {
                    // Accept accelerometer reading
                    self.half_accelerometer_feedback = feedback * 0.5;
                    if self.acceleration_recovery_trigger > 0 {
                        self.acceleration_recovery_trigger -= RECOVERY_DECREMENT;
                        if self.acceleration_recovery_trigger < 0 {
                            self.acceleration_recovery_trigger = 0;
                        }
                    }
                } else if !self.initialising {
                    // Reject accelerometer reading
                    self.acceleration_recovery_trigger += 1;
                    if self.acceleration_recovery_trigger as u32 > self.acceleration_recovery_timeout {
                        self.acceleration_recovery_trigger = 0;
                        self.accelerometer_ignored = true;
                    }
                }
            }
        } else {
            // Handle acceleration recovery
            if self.acceleration_recovery_trigger > 0 {
                self.acceleration_recovery_trigger -= 1;
            } else {
                self.accelerometer_ignored = false;
            }
        }
        
        // Calculate magnetic field direction in sensor frame
        let half_magnetic = self.calculate_half_magnetic();
        
        // Calculate magnetometer feedback
        self.half_magnetometer_feedback = Vector3::zeros();
        if !self.magnetometer_ignored && magnetometer.magnitude() > 0.0 {
            let magnetometer_normalized = magnetometer.safe_normalize();
            let feedback = self.calculate_feedback(magnetometer_normalized, half_magnetic);
            let error_squared = feedback.magnitude_squared();
            
            if error_squared <= self.magnetic_rejection_squared {
                // Accept magnetometer reading
                self.half_magnetometer_feedback = feedback * 0.5;
                if self.magnetic_recovery_trigger > 0 {
                    self.magnetic_recovery_trigger -= RECOVERY_DECREMENT;
                    if self.magnetic_recovery_trigger < 0 {
                        self.magnetic_recovery_trigger = 0;
                    }
                }
            } else if !self.initialising {
                // Reject magnetometer reading
                self.magnetic_recovery_trigger += 1;
                if self.magnetic_recovery_trigger as u32 > self.magnetic_recovery_timeout {
                    self.magnetic_recovery_trigger = 0;
                    self.magnetometer_ignored = true;
                }
            }
        } else if self.magnetometer_ignored {
            // Handle magnetic recovery
            if self.magnetic_recovery_trigger > 0 {
                self.magnetic_recovery_trigger -= 1;
            } else {
                self.magnetometer_ignored = false;
            }
        }
        
        // Convert gyroscope to half-radians per second
        let half_gyroscope = gyroscope * (DEG_TO_RAD * 0.5);
        
        // Apply feedback to gyroscope
        let adjusted_half_gyroscope = half_gyroscope + 
            (self.half_accelerometer_feedback + self.half_magnetometer_feedback) * self.ramped_gain;
        
        // Integrate quaternion
        self.integrate_quaternion(adjusted_half_gyroscope, delta_time);
    }

    /// Update AHRS without magnetometer (gyroscope and accelerometer only)
    pub fn update_no_magnetometer(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        delta_time: f32,
    ) {
        // Update with zero magnetometer
        self.update(gyroscope, accelerometer, Vector3::zeros(), delta_time);
        
        // Force magnetometer to be ignored
        self.magnetometer_ignored = true;
        self.half_magnetometer_feedback = Vector3::zeros();
        
        // Zero heading during initialization to prevent drift
        if self.initialising {
            self.zero_heading();
        }
    }

    /// Update AHRS with external heading source
    pub fn update_external_heading(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        heading: f32,
        delta_time: f32,
    ) {
        // Calculate synthetic magnetometer from heading
        let heading_rad = heading * DEG_TO_RAD;
        let cos_heading = heading_rad.cos();
        let sin_heading = heading_rad.sin();
        
        // Use current roll and pitch to calculate expected magnetometer
        let gravity = self.gravity();
        let east = Vector3::new(-gravity.y, gravity.x, 0.0).safe_normalize();
        let north = east.cross(&gravity).safe_normalize();
        
        let synthetic_magnetometer = north * cos_heading + east * sin_heading;
        
        // Update with synthetic magnetometer
        self.update(gyroscope, accelerometer, synthetic_magnetometer, delta_time);
    }

    /// Get current orientation quaternion
    pub fn quaternion(&self) -> UnitQuaternion<f32> {
        self.quaternion
    }

    /// Set orientation quaternion directly
    pub fn set_quaternion(&mut self, quaternion: UnitQuaternion<f32>) {
        self.quaternion = quaternion;
    }

    /// Get gravity vector in sensor frame
    pub fn gravity(&self) -> Vector3<f32> {
        self.calculate_half_gravity() * 2.0
    }

    /// Get linear acceleration (acceleration minus gravity)
    pub fn linear_acceleration(&self) -> Vector3<f32> {
        self.accelerometer - self.gravity()
    }

    /// Get earth-frame acceleration
    pub fn earth_acceleration(&self) -> Vector3<f32> {
        self.quaternion * self.linear_acceleration()
    }

    /// Get internal algorithm states
    pub fn internal_states(&self) -> AhrsInternalStates {
        AhrsInternalStates {
            acceleration_error: (self.half_accelerometer_feedback.magnitude() * 2.0).to_degrees(),
            accelerometer_ignored: self.accelerometer_ignored,
            acceleration_recovery_trigger: self.acceleration_recovery_trigger as f32,
            magnetic_error: (self.half_magnetometer_feedback.magnitude() * 2.0).to_degrees(),
            magnetometer_ignored: self.magnetometer_ignored,
            magnetic_recovery_trigger: self.magnetic_recovery_trigger as f32,
        }
    }

    /// Get algorithm flags
    pub fn flags(&self) -> AhrsFlags {
        AhrsFlags {
            initialising: self.initialising,
            angular_rate_recovery: self.angular_rate_recovery,
            acceleration_recovery: self.acceleration_recovery_trigger > 0,
            magnetic_recovery: self.magnetic_recovery_trigger > 0,
        }
    }

    /// Set heading angle directly
    pub fn set_heading(&mut self, heading: f32) {
        let heading_rad = heading * DEG_TO_RAD;
        
        // Extract current roll and pitch
        let (roll, pitch, _) = self.quaternion.euler_angles();
        
        // Create new quaternion with same roll/pitch but new heading
        self.quaternion = UnitQuaternion::from_euler_angles(roll, pitch, heading_rad);
    }
    
    /// Private helper methods
    
    /// Process settings and calculate derived values
    fn process_settings(&mut self) {
        // Process gyroscope range
        self.gyroscope_range_threshold = if self.settings.gyroscope_range > 0.0 {
            self.settings.gyroscope_range * GYROSCOPE_RANGE_FACTOR
        } else {
            0.0
        };
        
        // Process rejection thresholds (convert to squared for efficiency)
        let accel_rad = self.settings.acceleration_rejection * DEG_TO_RAD;
        self.acceleration_rejection_squared = (0.5 * accel_rad.sin()).powi(2);
        
        let mag_rad = self.settings.magnetic_rejection * DEG_TO_RAD;
        self.magnetic_rejection_squared = (0.5 * mag_rad.sin()).powi(2);
        
        // Set recovery timeouts
        self.acceleration_recovery_timeout = self.settings.recovery_trigger_period;
        self.magnetic_recovery_timeout = self.settings.recovery_trigger_period;
    }
    
    /// Calculate half gravity vector in sensor frame based on current quaternion
    fn calculate_half_gravity(&self) -> Vector3<f32> {
        let q = self.quaternion.as_ref();
        let qw = q.w;
        let qx = q.i;
        let qy = q.j; 
        let qz = q.k;
        
        match self.settings.convention {
            Convention::Nwu | Convention::Enu => {
                Vector3::new(
                    qx * qz - qw * qy,
                    qy * qz + qw * qx,
                    qw * qw - 0.5 + qz * qz
                )
            },
            Convention::Ned => {
                Vector3::new(
                    qw * qy - qx * qz,
                    -(qy * qz + qw * qx),
                    0.5 - qw * qw - qz * qz
                )
            }
        }
    }
    
    /// Calculate half magnetic field vector in sensor frame
    fn calculate_half_magnetic(&self) -> Vector3<f32> {
        let q = self.quaternion.as_ref();
        let qw = q.w;
        let qx = q.i;
        let qy = q.j;
        let qz = q.k;
        
        match self.settings.convention {
            Convention::Nwu => {
                // Second column of rotation matrix * 0.5
                Vector3::new(
                    qw * qy + qx * qz,
                    0.5 - qx * qx - qz * qz,
                    qy * qz - qw * qx
                )
            },
            Convention::Enu => {
                // First column of rotation matrix * -0.5
                Vector3::new(
                    -(0.5 - qy * qy - qz * qz),
                    -(qx * qy - qw * qz),
                    -(qx * qz + qw * qy)
                )
            },
            Convention::Ned => {
                // Second column of rotation matrix * -0.5
                Vector3::new(
                    -(qw * qy + qx * qz),
                    -(0.5 - qx * qx - qz * qz),
                    -(qy * qz - qw * qx)
                )
            }
        }
    }
    
    /// Calculate feedback vector between sensor reading and reference
    fn calculate_feedback(&self, sensor: Vector3<f32>, reference: Vector3<f32>) -> Vector3<f32> {
        let cross = sensor.cross(&reference);
        
        // Check if vectors are opposing (dot product < 0)
        if sensor.dot(&reference) < 0.0 {
            // Normalize cross product for opposing vectors
            cross.safe_normalize()
        } else {
            cross
        }
    }
    
    /// Integrate quaternion using gyroscope reading
    fn integrate_quaternion(&mut self, half_gyroscope: Vector3<f32>, delta_time: f32) {
        // Create quaternion from gyroscope reading
        let gyro_quat = Quaternion::from_parts(0.0, half_gyroscope.into());
        
        // Quaternion derivative: dq/dt = 0.5 * q * ω
        let quaternion_derivative = self.quaternion.as_ref() * gyro_quat;
        
        // Integrate using first-order approximation
        let new_quaternion = self.quaternion.as_ref() + quaternion_derivative * delta_time;
        
        // Normalize to maintain unit quaternion
        self.quaternion = UnitQuaternion::from_quaternion(new_quaternion);
    }
    
    /// Zero the heading while preserving roll and pitch
    fn zero_heading(&mut self) {
        let (roll, pitch, _) = self.quaternion.euler_angles();
        self.quaternion = UnitQuaternion::from_euler_angles(roll, pitch, 0.0);
    }
}

impl Default for Ahrs {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_new_ahrs() {
        let ahrs = Ahrs::new();
        assert_eq!(ahrs.quaternion(), UnitQuaternion::identity());
        assert!(ahrs.flags().initialising);
    }
    
    #[test]
    fn test_ahrs_initialization() {
        let mut ahrs = Ahrs::new();
        
        // Should start in initializing state
        assert!(ahrs.flags().initialising);
        
        // Update for initialization period to complete ramping
        let gyro = Vector3::zeros();
        let accel = Vector3::new(0.0, 0.0, 1.0);
        let mag = Vector3::new(1.0, 0.0, 0.0);
        let delta_time = 0.01; // 10ms
        
        // Simulate 4 seconds at 100Hz to complete initialization
        for _ in 0..400 {
            ahrs.update(gyro, accel, mag, delta_time);
        }
        
        // Should no longer be initializing
        assert!(!ahrs.flags().initialising);
    }
    
    #[test]
    fn test_gravity_calculation() {
        let ahrs = Ahrs::new();
        let gravity = ahrs.gravity();
        
        // Should be unit vector pointing up in NWU convention
        assert!((gravity.magnitude() - 1.0).abs() < 1e-6);
        assert!((gravity.z - 1.0).abs() < 1e-6);
    }
    
    #[test]
    fn test_gyroscope_overflow_detection() {
        let mut settings = AhrsSettings::default();
        settings.gyroscope_range = 500.0; // 500 deg/s range
        let mut ahrs = Ahrs::with_settings(settings);
        
        // Complete initialization first
        let normal_gyro = Vector3::zeros();
        let accel = Vector3::new(0.0, 0.0, 1.0);
        let mag = Vector3::new(1.0, 0.0, 0.0);
        
        for _ in 0..400 {
            ahrs.update(normal_gyro, accel, mag, 0.01);
        }
        assert!(!ahrs.flags().initialising);
        
        // Now test overflow
        let overflow_gyro = Vector3::new(600.0, 0.0, 0.0); // Exceeds 500 deg/s
        ahrs.update(overflow_gyro, accel, mag, 0.01);
        
        assert!(ahrs.flags().angular_rate_recovery);
        assert!(ahrs.flags().initialising); // Should restart initialization
    }
    
    #[test]
    fn test_accelerometer_rejection() {
        let mut settings = AhrsSettings::default();
        settings.acceleration_rejection = 10.0; // 10 degree threshold
        settings.recovery_trigger_period = 100;
        
        let mut ahrs = Ahrs::with_settings(settings);
        
        // Complete initialization first
        let gyro = Vector3::zeros();
        let normal_accel = Vector3::new(0.0, 0.0, 1.0); // Normal gravity
        let mag = Vector3::new(1.0, 0.0, 0.0);
        
        for _ in 0..400 {
            ahrs.update(gyro, normal_accel, mag, 0.01);
        }
        
        // Test that normal acceleration is accepted
        let states = ahrs.internal_states();
        assert!(!states.accelerometer_ignored);
        
        // Apply large acceleration (should be rejected after enough samples)
        let large_accel = Vector3::new(2.0, 2.0, 1.0); // Large acceleration indicating motion
        
        // Apply bad readings repeatedly to eventually trigger rejection
        let mut rejected = false;
        for i in 0..150 {
            ahrs.update(gyro, large_accel, mag, 0.01);
            let states = ahrs.internal_states();
            
            if states.accelerometer_ignored || states.acceleration_recovery_trigger > 50.0 {
                rejected = true;
                break;
            }
        }
        
        // Should eventually trigger rejection mechanism
        assert!(rejected, "Accelerometer should be rejected for large accelerations");
    }
}