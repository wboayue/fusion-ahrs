//! Core types and conventions for the Fusion AHRS library

use core::fmt;

/// Earth axes convention
///
/// Defines the coordinate system used for Earth-relative calculations.
/// Each convention defines different orientations for the X, Y, and Z axes.
///
/// # Conventions
/// - **NWU**: North-West-Up (X=North, Y=West, Z=Up)
/// - **ENU**: East-North-Up (X=East, Y=North, Z=Up)
/// - **NED**: North-East-Down (X=North, Y=East, Z=Down)
///
/// # Example
/// ```
/// use fusion_ahrs::{Convention, Ahrs, AhrsSettings};
///
/// let settings = AhrsSettings {
///     convention: Convention::Enu,
///     ..Default::default()
/// };
/// let ahrs = Ahrs::with_settings(settings);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Convention {
    /// North-West-Up coordinate system
    ///
    /// - X axis points North
    /// - Y axis points West
    /// - Z axis points Up
    #[default]
    Nwu,
    /// East-North-Up coordinate system
    ///
    /// - X axis points East
    /// - Y axis points North
    /// - Z axis points Up
    Enu,
    /// North-East-Down coordinate system
    ///
    /// - X axis points North
    /// - Y axis points East
    /// - Z axis points Down
    Ned,
}

impl Convention {
    /// All conventions, in declaration order.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Convention;
    ///
    /// for convention in Convention::ALL {
    ///     println!("{convention}");
    /// }
    /// ```
    pub const ALL: [Convention; 3] = [Convention::Nwu, Convention::Enu, Convention::Ned];

    /// Returns the convention as a string, matching the C library's
    /// `FusionConventionToString`.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Convention;
    ///
    /// assert_eq!(Convention::Ned.as_str(), "North, East, Down (NED)");
    /// assert_eq!(Convention::Enu.to_string(), "East, North, Up (ENU)");
    /// ```
    pub const fn as_str(self) -> &'static str {
        match self {
            Convention::Nwu => "North, West, Up (NWU)",
            Convention::Enu => "East, North, Up (ENU)",
            Convention::Ned => "North, East, Down (NED)",
        }
    }
}

impl fmt::Display for Convention {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// AHRS algorithm settings
///
/// Configuration parameters for the AHRS algorithm. These settings control
/// the algorithm's behavior including coordinate convention, filter gain,
/// sensor range limits, and rejection thresholds.
///
/// # Example
/// ```
/// use fusion_ahrs::{AhrsSettings, Convention};
///
/// let settings = AhrsSettings {
///     sample_rate: 512.0,            // 512 Hz
///     convention: Convention::Enu,
///     gain: 0.25,                    // Lower gain for more stability
///     gyroscope_range: 1000.0,       // 1000 deg/s range
///     acceleration_rejection: 15.0,  // 15° threshold
///     magnetic_rejection: 30.0,      // 30° threshold
///     rejection_timeout: 2.0,        // 2 seconds
/// };
/// ```
#[derive(Debug, Clone, Copy)]
pub struct AhrsSettings {
    /// Sample rate in Hz (default 100). Must be positive.
    ///
    /// Determines the nominal sample period used to integrate the gyroscope.
    /// Use [`Ahrs::set_sample_period`](crate::Ahrs::set_sample_period) to
    /// compensate for per-sample timing jitter.
    pub sample_rate: f32,
    /// Earth axes convention (NWU, ENU, or NED)
    pub convention: Convention,
    /// Algorithm gain controlling fusion rate (typically 0.5)
    ///
    /// Higher values make the algorithm more responsive to accelerometer/magnetometer
    /// but less stable. Lower values provide more stability but slower convergence.
    pub gain: f32,
    /// Gyroscope range limit in degrees per second
    ///
    /// When gyroscope readings exceed 98% of this range, the algorithm
    /// restarts while preserving its outputs. Set to 0 to disable.
    pub gyroscope_range: f32,
    /// Acceleration rejection threshold in degrees
    ///
    /// When the angle between measured and expected acceleration exceeds
    /// this threshold, the accelerometer will be ignored. Set to 0 to disable.
    pub acceleration_rejection: f32,
    /// Magnetic rejection threshold in degrees
    ///
    /// When the angle between measured and expected magnetic field exceeds
    /// this threshold, the magnetometer will be ignored. Set to 0 to disable.
    pub magnetic_rejection: f32,
    /// Rejection timeout in seconds
    ///
    /// Maximum duration a sensor may be rejected before recovery is triggered.
    /// Set to 0 to disable acceleration and magnetic rejection.
    pub rejection_timeout: f32,
}

impl Default for AhrsSettings {
    fn default() -> Self {
        Self {
            sample_rate: 100.0,
            convention: Convention::default(),
            gain: 0.5,
            gyroscope_range: 0.0,
            acceleration_rejection: 0.0,
            magnetic_rejection: 0.0,
            rejection_timeout: 0.0,
        }
    }
}

/// AHRS algorithm internal states
///
/// Diagnostic information about the algorithm's internal state,
/// including error measurements and sensor status. Useful for
/// monitoring algorithm performance and debugging issues.
///
/// # Example
/// ```
/// use fusion_ahrs::Ahrs;
///
/// let ahrs = Ahrs::new();
/// let states = ahrs.internal_states();
///
/// // Check if sensors are being rejected
/// if states.accelerometer_ignored {
///     println!("Motion detected, accelerometer rejected");
/// }
/// if states.magnetometer_ignored {
///     println!("Magnetic interference detected");
/// }
/// ```
#[derive(Debug, Clone, Copy)]
pub struct AhrsInternalStates {
    /// Acceleration error magnitude in degrees
    ///
    /// Angle between measured and expected acceleration direction.
    /// Large values indicate motion or accelerometer errors.
    pub acceleration_error: f32,
    /// Whether accelerometer is currently being ignored
    ///
    /// True when acceleration error exceeds rejection threshold,
    /// indicating device motion or sensor errors.
    pub accelerometer_ignored: bool,
    /// Acceleration recovery trigger as a fraction of the rejection timeout
    ///
    /// Ranges from 0.0 to 1.0; recovery is triggered when it reaches 1.0.
    pub acceleration_recovery_trigger: f32,
    /// Magnetic error magnitude in degrees
    ///
    /// Angle between measured and expected magnetic field direction.
    /// Large values indicate magnetic interference.
    pub magnetic_error: f32,
    /// Whether magnetometer is currently being ignored
    ///
    /// True when magnetic error exceeds rejection threshold,
    /// indicating magnetic interference or sensor errors.
    pub magnetometer_ignored: bool,
    /// Magnetic recovery trigger as a fraction of the rejection timeout
    ///
    /// Ranges from 0.0 to 1.0; recovery is triggered when it reaches 1.0.
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
///
/// Status flags indicating the current operating mode and state
/// of the AHRS algorithm. These flags help monitor algorithm
/// behavior and detect special conditions.
///
/// # Example
/// ```
/// use fusion_ahrs::Ahrs;
///
/// let ahrs = Ahrs::new();
/// let flags = ahrs.flags();
///
/// if flags.startup {
///     println!("Algorithm still converging...");
/// }
/// if flags.overrange_recovery {
///     println!("Recovering from gyroscope overrange");
/// }
/// ```
#[derive(Debug, Clone, Copy, Default)]
pub struct AhrsFlags {
    /// Whether the algorithm is in startup
    ///
    /// True during the first few seconds of operation when
    /// the algorithm uses higher gain for faster convergence.
    pub startup: bool,
    /// Whether gyroscope overrange recovery is active
    ///
    /// True when recovering from gyroscope overrange.
    /// The algorithm preserves its outputs but restarts internal state.
    pub overrange_recovery: bool,
    /// Whether acceleration recovery is active
    ///
    /// True when the acceleration recovery mechanism is engaged
    /// due to persistent accelerometer rejection.
    pub acceleration_recovery: bool,
    /// Whether magnetic recovery is active
    ///
    /// True when the magnetic recovery mechanism is engaged
    /// due to persistent magnetometer rejection.
    pub magnetic_recovery: bool,
}

/// Gyroscope offset correction settings
///
/// Configuration for the gyroscope offset correction algorithm.
/// This algorithm estimates and corrects for gyroscope bias drift
/// that can occur due to temperature changes or sensor aging.
///
/// # Example
/// ```
/// use fusion_ahrs::OffsetSettings;
///
/// let settings = OffsetSettings {
///     cutoff_frequency: 0.01,  // Slower convergence
///     timeout: 10.0,           // 10 seconds to detect stationary
///     threshold: 3.0,          // 3 deg/s motion threshold
/// };
/// ```
#[derive(Debug, Clone, Copy)]
pub struct OffsetSettings {
    /// Low-pass filter cutoff frequency in Hz (default 0.02)
    ///
    /// Used to compute the filter coefficient as `2π × cutoff_frequency / sample_rate`.
    /// Lower values provide more stability but slower convergence.
    pub cutoff_frequency: f32,
    /// Timeout period in seconds before offset estimation begins (default 3.0)
    ///
    /// Duration the sensor must remain stationary before offset
    /// correction begins. Longer timeouts reduce false corrections.
    pub timeout: f32,
    /// Gyroscope threshold in degrees per second (default 3.0)
    ///
    /// If any gyroscope axis exceeds this value, the sensor is
    /// considered in motion and the stationary timer resets.
    pub threshold: f32,
}

impl Default for OffsetSettings {
    fn default() -> Self {
        Self {
            cutoff_frequency: 0.02,
            timeout: 3.0,
            threshold: 3.0,
        }
    }
}
