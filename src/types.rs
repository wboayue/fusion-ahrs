//! Core types and conventions for the Fusion AHRS library

use nalgebra::Vector3;

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

/// Gyroscope offset correction settings
#[derive(Debug, Clone, Copy)]
pub struct OffsetSettings {
    /// Filter coefficient for offset estimation (typically 0.01)
    pub filter_coefficient: f32,
    /// Timeout period in seconds (typically 5.0)
    pub timeout: f32,
}

impl Default for OffsetSettings {
    fn default() -> Self {
        Self {
            filter_coefficient: 0.01,
            timeout: 5.0,
        }
    }
}