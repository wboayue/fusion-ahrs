//! Core types and conventions for the Fusion AHRS library

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
///     convention: Convention::Enu,
///     gain: 0.25,                    // Lower gain for more stability
///     gyroscope_range: 1000.0,       // 1000 deg/s range
///     acceleration_rejection: 15.0,   // 15° threshold
///     magnetic_rejection: 30.0,       // 30° threshold
///     recovery_trigger_period: 1024,  // 2s at 512Hz
/// };
/// ```
#[derive(Debug, Clone, Copy)]
pub struct AhrsSettings {
    /// Earth axes convention (NWU, ENU, or NED)
    pub convention: Convention,
    /// Algorithm gain controlling fusion rate (typically 0.5)
    ///
    /// Higher values make the algorithm more responsive to accelerometer/magnetometer
    /// but less stable. Lower values provide more stability but slower convergence.
    pub gain: f32,
    /// Gyroscope range limit in degrees per second
    ///
    /// When gyroscope readings exceed this threshold, the algorithm will
    /// reinitialize to prevent integration windup. Set to 0 to disable.
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
    /// Recovery trigger period in samples
    ///
    /// Number of consecutive samples before triggering recovery from
    /// sensor rejection. Higher values provide more stability but slower recovery.
    pub recovery_trigger_period: u32,
}

impl Default for AhrsSettings {
    fn default() -> Self {
        Self {
            convention: Convention::default(),
            gain: 0.5,
            gyroscope_range: 0.0,
            acceleration_rejection: 90.0,
            magnetic_rejection: 90.0,
            recovery_trigger_period: 0,
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
    /// Acceleration recovery trigger countdown
    ///
    /// Counts samples until acceleration recovery is triggered.
    /// Resets when good accelerometer readings are detected.
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
    /// Magnetic recovery trigger countdown
    ///
    /// Counts samples until magnetic recovery is triggered.
    /// Resets when good magnetometer readings are detected.
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
/// if flags.initialising {
///     println!("Algorithm still converging...");
/// }
/// if flags.angular_rate_recovery {
///     println!("Recovering from gyroscope overflow");
/// }
/// ```
#[derive(Debug, Clone, Copy, Default)]
pub struct AhrsFlags {
    /// Whether algorithm is in initialization mode
    ///
    /// True during the first few seconds of operation when
    /// the algorithm uses higher gain for faster convergence.
    pub initialising: bool,
    /// Whether angular rate recovery is active
    ///
    /// True when recovering from gyroscope range overflow.
    /// The algorithm preserves orientation but reinitializes internal state.
    pub angular_rate_recovery: bool,
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
///     filter_coefficient: 0.005,  // Slower convergence
///     timeout: 10.0,              // 10 seconds to detect stationary
/// };
/// ```
#[derive(Debug, Clone, Copy)]
pub struct OffsetSettings {
    /// Filter coefficient for offset estimation (typically 0.01)
    ///
    /// Controls how quickly the offset estimate converges.
    /// Lower values provide more stability but slower response.
    pub filter_coefficient: f32,
    /// Timeout period in seconds before offset estimation begins (typically 5.0)
    ///
    /// Duration the sensor must remain stationary before offset
    /// correction begins. Longer timeouts reduce false corrections.
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
