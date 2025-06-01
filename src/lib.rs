#![no_std]

use nalgebra::{UnitQuaternion, Vector3};

/// Earth axes convention
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Convention {
    /// North-West-Up
    Nwu,
    /// East-North-Up
    Enu,
    /// North-East-Down
    Ned,
}

impl Default for Convention {
    fn default() -> Self {
        Self::Nwu
    }
}

/// AHRS algorithm settings
#[derive(Debug, Clone, Copy)]
pub struct AhrsSettings {
    /// Earth axes convention
    pub convention: Convention,
    /// Algorithm gain (typically 0.5)
    pub gain: f32,
    /// Gyroscope range in degrees per second
    pub gyroscope_range: f32,
    /// Acceleration rejection threshold
    pub acceleration_rejection: f32,
    /// Magnetic rejection threshold
    pub magnetic_rejection: f32,
    /// Recovery trigger period in samples
    pub recovery_trigger_period: u32,
}

impl Default for AhrsSettings {
    fn default() -> Self {
        Self {
            convention: Convention::default(),
            gain: 0.5,
            gyroscope_range: 2000.0,
            acceleration_rejection: 10.0,
            magnetic_rejection: 20.0,
            recovery_trigger_period: 5 * 512, // 5 seconds at 512 Hz
        }
    }
}

/// AHRS algorithm internal states
#[derive(Debug, Clone, Copy)]
pub struct AhrsInternalStates {
    /// Acceleration error magnitude
    pub acceleration_error: f32,
    /// Whether accelerometer is being ignored
    pub accelerometer_ignored: bool,
    /// Acceleration recovery trigger countdown
    pub acceleration_recovery_trigger: f32,
    /// Magnetic error magnitude
    pub magnetic_error: f32,
    /// Whether magnetometer is being ignored
    pub magnetometer_ignored: bool,
    /// Magnetic recovery trigger countdown
    pub magnetic_recovery_trigger: f32,
}

impl Default for AhrsInternalStates {
    fn default() -> Self {
        Self {
            acceleration_error: 0.0,
            accelerometer_ignored: false,
            acceleration_recovery_trigger: 0.0,
            magnetic_error: 0.0,
            magnetometer_ignored: false,
            magnetic_recovery_trigger: 0.0,
        }
    }
}

/// AHRS algorithm flags
#[derive(Debug, Clone, Copy, Default)]
pub struct AhrsFlags {
    /// Whether algorithm is initializing
    pub initialising: bool,
    /// Whether angular rate recovery is active
    pub angular_rate_recovery: bool,
    /// Whether acceleration recovery is active
    pub acceleration_recovery: bool,
    /// Whether magnetic recovery is active
    pub magnetic_recovery: bool,
}

/// Gyroscope offset correction structure
#[derive(Debug, Clone, Copy)]
pub struct Offset {
    /// Filter coefficient for offset estimation
    filter_coefficient: f32,
    /// Timeout period in samples
    timeout: u32,
    /// Current timer value
    timer: u32,
    /// Estimated gyroscope offset
    gyroscope_offset: Vector3<f32>,
}

impl Offset {
    /// Initialize offset correction with the given sample rate
    pub fn new(sample_rate: u32) -> Self {
        let timeout = sample_rate * 5; // 5 second timeout
        Self {
            filter_coefficient: 0.01, // 1% filter coefficient
            timeout,
            timer: timeout,
            gyroscope_offset: Vector3::zeros(),
        }
    }

    /// Update offset estimation and return corrected gyroscope reading
    pub fn update(&mut self, gyroscope: Vector3<f32>) -> Vector3<f32> {
        // Placeholder implementation - will be replaced with actual algorithm
        gyroscope - self.gyroscope_offset
    }

    /// Get current offset estimate
    pub fn get_offset(&self) -> Vector3<f32> {
        self.gyroscope_offset
    }
}

/// Main AHRS algorithm structure
pub struct FusionAhrs {
    /// Algorithm settings
    settings: AhrsSettings,
    /// Current orientation quaternion
    quaternion: UnitQuaternion<f32>,
    /// Last accelerometer reading
    accelerometer: Vector3<f32>,
    /// Whether algorithm is initializing
    initialising: bool,
    /// Ramped gain value
    ramped_gain: f32,
    /// Gain ramping step size
    ramped_gain_step: f32,
    /// Angular rate recovery flag
    angular_rate_recovery: bool,
    /// Half accelerometer feedback vector
    half_accelerometer_feedback: Vector3<f32>,
    /// Half magnetometer feedback vector
    half_magnetometer_feedback: Vector3<f32>,
    /// Accelerometer ignored flag
    accelerometer_ignored: bool,
    /// Acceleration recovery trigger countdown
    acceleration_recovery_trigger: i32,
    /// Acceleration recovery timeout
    acceleration_recovery_timeout: i32,
    /// Magnetometer ignored flag
    magnetometer_ignored: bool,
    /// Magnetic recovery trigger countdown
    magnetic_recovery_trigger: i32,
    /// Magnetic recovery timeout
    magnetic_recovery_timeout: i32,
}

impl FusionAhrs {
    /// Create a new AHRS instance with default settings
    pub fn new() -> Self {
        Self::with_settings(AhrsSettings::default())
    }

    /// Create a new AHRS instance with specified settings
    pub fn with_settings(settings: AhrsSettings) -> Self {
        FusionAhrs {
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
        }
    }

    /// Initialize/reset the AHRS algorithm
    pub fn initialise(&mut self) {
        self.quaternion = UnitQuaternion::identity();
        self.accelerometer = Vector3::zeros();
        self.initialising = true;
        self.ramped_gain = 0.0;
        self.ramped_gain_step = 0.002; // 2 seconds @ 1000 Hz
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
    }

    /// Get current algorithm settings
    pub fn get_settings(&self) -> AhrsSettings {
        self.settings
    }

    /// Update AHRS with gyroscope, accelerometer, and magnetometer data
    pub fn update(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        magnetometer: Vector3<f32>,
        delta_time: f32,
    ) {
        // Placeholder - will implement full algorithm
        self.accelerometer = accelerometer;
    }

    /// Update AHRS without magnetometer (gyroscope and accelerometer only)
    pub fn update_no_magnetometer(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        delta_time: f32,
    ) {
        // Placeholder - will implement algorithm without magnetometer
        self.accelerometer = accelerometer;
    }

    /// Update AHRS with external heading source
    pub fn update_external_heading(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        heading: f32,
        delta_time: f32,
    ) {
        // Placeholder - will implement algorithm with external heading
        self.accelerometer = accelerometer;
    }

    /// Get current orientation quaternion
    pub fn get_quaternion(&self) -> UnitQuaternion<f32> {
        self.quaternion
    }

    /// Set orientation quaternion directly
    pub fn set_quaternion(&mut self, quaternion: UnitQuaternion<f32>) {
        self.quaternion = quaternion;
    }

    /// Get gravity vector in sensor frame
    pub fn get_gravity(&self) -> Vector3<f32> {
        // Placeholder - will implement gravity calculation
        Vector3::new(0.0, 0.0, 1.0)
    }

    /// Get linear acceleration (acceleration minus gravity)
    pub fn get_linear_acceleration(&self) -> Vector3<f32> {
        // Placeholder - will implement linear acceleration calculation
        self.accelerometer - self.get_gravity()
    }

    /// Get earth-frame acceleration
    pub fn get_earth_acceleration(&self) -> Vector3<f32> {
        // Placeholder - will implement earth acceleration calculation
        self.quaternion * self.get_linear_acceleration()
    }

    /// Get internal algorithm states
    pub fn get_internal_states(&self) -> AhrsInternalStates {
        // Placeholder - will implement based on internal state
        AhrsInternalStates::default()
    }

    /// Get algorithm flags
    pub fn get_flags(&self) -> AhrsFlags {
        AhrsFlags {
            initialising: self.initialising,
            angular_rate_recovery: self.angular_rate_recovery,
            acceleration_recovery: self.acceleration_recovery_trigger > 0,
            magnetic_recovery: self.magnetic_recovery_trigger > 0,
        }
    }

    /// Set heading angle directly
    pub fn set_heading(&mut self, heading: f32) {
        // Placeholder - will implement heading adjustment
    }
}

impl Default for FusionAhrs {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_ahrs() {
        let ahrs = FusionAhrs::new();
        // Add assertions to verify the initial state of the AHRS
        assert_eq!(ahrs.get_quaternion(), UnitQuaternion::identity());
    }
}
