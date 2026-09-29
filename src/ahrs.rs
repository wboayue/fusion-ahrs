//! Main AHRS algorithm implementation for the Fusion AHRS library

use crate::math::{DEG_TO_RAD, Quaternion, RAD_TO_DEG, Vector, arc_sin};
use crate::types::{AhrsFlags, AhrsInternalStates, AhrsSettings, Convention};

/// Initial gain used at the start of startup
const INITIAL_STARTUP_GAIN: f32 = 10.0;
/// Startup period in seconds
const STARTUP_PERIOD: f32 = 3.0;
/// Fraction of the gyroscope range above which overrange is detected
const OVERRANGE_FACTOR: f32 = 0.98;
/// Recovery trigger decrement applied for each accepted sample
const RECOVERY_DECREMENT: i32 = 9;

/// Main AHRS algorithm structure
///
/// Implements a complementary filter that fuses gyroscope, accelerometer,
/// and magnetometer data to estimate orientation. Features automatic sensor
/// rejection during motion/interference and recovery mechanisms.
#[derive(Debug, Clone)]
pub struct Ahrs {
    /// Algorithm settings as provided by the user
    settings: AhrsSettings,
    /// Values derived from the settings
    config: Config,
    /// Current orientation quaternion (WXYZ format)
    quaternion: Quaternion,
    /// Last accelerometer reading for linear acceleration calculation
    accelerometer: Vector,
    /// Whether the algorithm is in startup
    startup: bool,
    /// Gain ramped down during startup
    startup_gain: f32,
    /// Gyroscope overrange recovery flag
    overrange_recovery: bool,
    /// Accelerometer rejection state
    acceleration: Rejection,
    /// Magnetometer rejection state
    magnetic: Rejection,
}

/// Values derived from [`AhrsSettings`]
#[derive(Debug, Clone, Copy)]
struct Config {
    /// Sample period in seconds
    sample_period: f32,
    /// Algorithm gain
    gain: f32,
    /// Startup gain decrement per update
    startup_gain_rate: f32,
    /// Whether gyroscope overrange detection is enabled
    overrange_enabled: bool,
    /// Gyroscope overrange threshold in degrees per second
    overrange_threshold: f32,
    /// Acceleration rejection threshold (squared half-residual)
    acceleration_rejection: f32,
    /// Magnetic rejection threshold (squared half-residual)
    magnetic_rejection: f32,
    /// Rejection timeout in samples
    rejection_timeout: i32,
}

impl Config {
    fn new(settings: &AhrsSettings) -> Self {
        debug_assert!(settings.sample_rate > 0.0, "sample_rate must be positive");

        let sample_period = 1.0 / settings.sample_rate;

        // Disable acceleration and magnetic rejection if gain or timeout is zero
        let rejection_enabled = settings.gain != 0.0 && settings.rejection_timeout != 0.0;
        let rejection = |degrees: f32| {
            if rejection_enabled {
                rejection_threshold(degrees)
            } else {
                f32::MAX
            }
        };

        Self {
            sample_period,
            gain: settings.gain,
            startup_gain_rate: ((INITIAL_STARTUP_GAIN - settings.gain) / STARTUP_PERIOD)
                * sample_period,
            overrange_enabled: settings.gyroscope_range > 0.0,
            overrange_threshold: OVERRANGE_FACTOR * settings.gyroscope_range,
            acceleration_rejection: rejection(settings.acceleration_rejection),
            magnetic_rejection: rejection(settings.magnetic_rejection),
            rejection_timeout: (settings.sample_rate * settings.rejection_timeout) as i32,
        }
    }
}

/// Rejection and recovery state for one sensor (accelerometer or magnetometer)
#[derive(Debug, Clone, Copy)]
struct Rejection {
    /// Residual between sensor and algorithm reference, scaled by 0.5
    half_residual: Vector,
    /// Recovery trigger in samples
    recovery_trigger: i32,
    /// Recovery threshold in samples
    recovery_threshold: i32,
    /// Whether the sensor was ignored by the last update
    ignored: bool,
}

impl Rejection {
    fn new(timeout: i32) -> Self {
        Self {
            half_residual: Vector::ZERO,
            recovery_trigger: 0,
            recovery_threshold: timeout,
            ignored: false,
        }
    }

    /// Mark the sensor ignored for an update with no measurement
    fn skip(&mut self) {
        self.ignored = true;
    }

    /// Update with a new residual and return the feedback to apply, scaled by 0.5
    fn update(
        &mut self,
        half_residual: Vector,
        startup: bool,
        rejection: f32,
        timeout: i32,
    ) -> Vector {
        self.half_residual = half_residual;

        // Don't ignore sensor if error below threshold
        if startup || half_residual.norm_squared() <= rejection {
            self.ignored = false;
            self.recovery_trigger = self.recovery_trigger.saturating_sub(RECOVERY_DECREMENT);
        } else {
            self.ignored = true;
            self.recovery_trigger = self.recovery_trigger.saturating_add(1);
        }

        // Don't ignore sensor during recovery
        if self.recovery() {
            self.recovery_threshold = 0;
            self.ignored = false;
        } else {
            self.recovery_threshold = timeout;
        }
        self.recovery_trigger = clamp(self.recovery_trigger, 0, timeout);

        if self.ignored {
            Vector::ZERO
        } else {
            self.half_residual
        }
    }

    /// Whether recovery is active
    fn recovery(&self) -> bool {
        self.recovery_trigger > self.recovery_threshold
    }

    /// Error between sensor and algorithm reference in degrees
    fn error(&self) -> f32 {
        arc_sin(2.0 * self.half_residual.norm()) * RAD_TO_DEG
    }

    /// Recovery trigger as a fraction of the timeout
    fn recovery_trigger(&self, timeout: i32) -> f32 {
        if timeout == 0 {
            0.0
        } else {
            self.recovery_trigger as f32 / timeout as f32
        }
    }
}

impl Ahrs {
    /// Create a new AHRS instance with default settings
    ///
    /// This creates an AHRS algorithm with the default settings:
    /// - Sample rate: 100 Hz
    /// - Convention: NWU (North-West-Up)
    /// - Gain: 0.5
    /// - Gyroscope range: 0 (disabled)
    /// - Acceleration rejection: 0 (disabled)
    /// - Magnetic rejection: 0 (disabled)
    /// - Rejection timeout: 0 (disabled)
    ///
    /// **Note:** These defaults match the C library and disable sensor
    /// rejection and gyroscope overrange recovery. For applications with
    /// motion or magnetic interference, configure rejection thresholds
    /// (e.g., 10° acceleration, 10° magnetic, 5 s timeout) via
    /// [`Ahrs::with_settings`].
    ///
    /// The algorithm will start in startup mode with ramped gain.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new();
    /// assert!(ahrs.flags().startup);
    /// ```
    pub fn new() -> Self {
        Self::with_settings(AhrsSettings::default())
    }

    /// Create a new AHRS instance with specified settings
    ///
    /// This allows customization of all algorithm parameters including
    /// sample rate, coordinate convention, gain, gyroscope range, and
    /// rejection thresholds.
    ///
    /// # Arguments
    /// * `settings` - Configuration for the AHRS algorithm
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::{Ahrs, AhrsSettings, Convention};
    ///
    /// let settings = AhrsSettings {
    ///     sample_rate: 512.0,
    ///     convention: Convention::Enu,
    ///     gain: 0.75,
    ///     gyroscope_range: 1000.0,
    ///     acceleration_rejection: 15.0,
    ///     magnetic_rejection: 25.0,
    ///     rejection_timeout: 2.0,
    /// };
    ///
    /// let mut ahrs = Ahrs::with_settings(settings);
    /// assert_eq!(ahrs.get_settings().gain, 0.75);
    /// ```
    pub fn with_settings(settings: AhrsSettings) -> Self {
        let config = Config::new(&settings);
        Ahrs {
            settings,
            config,
            quaternion: Quaternion::IDENTITY,
            accelerometer: Vector::ZERO,
            startup: true,
            startup_gain: INITIAL_STARTUP_GAIN,
            overrange_recovery: false,
            acceleration: Rejection::new(config.rejection_timeout),
            magnetic: Rejection::new(config.rejection_timeout),
        }
    }

    /// Restart the AHRS algorithm
    ///
    /// Resets the algorithm to its initial state while keeping the settings:
    /// - Sets quaternion to identity (no rotation)
    /// - Clears all internal state variables
    /// - Enters startup mode with ramped gain
    /// - Resets all recovery mechanisms
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new();
    /// ahrs.skip_startup();
    /// // ... use the AHRS ...
    /// ahrs.restart(); // Back to initial state
    /// assert!(ahrs.flags().startup);
    /// ```
    pub fn restart(&mut self) {
        self.quaternion = Quaternion::IDENTITY;
        self.accelerometer = Vector::ZERO;

        self.startup = true;
        self.startup_gain = INITIAL_STARTUP_GAIN;

        self.overrange_recovery = false;

        self.acceleration = Rejection::new(self.config.rejection_timeout);
        self.magnetic = Rejection::new(self.config.rejection_timeout);
    }

    /// Restart the AHRS algorithm
    #[deprecated(since = "0.8.0", note = "use `restart` instead")]
    pub fn initialise(&mut self) {
        self.restart();
    }

    /// Restart the AHRS algorithm
    #[deprecated(since = "0.8.0", note = "use `restart` instead")]
    pub fn reset(&mut self) {
        self.restart();
    }

    /// Skip startup
    ///
    /// Intended to be called before the first update when the initial
    /// orientation is already known (e.g. after [`Ahrs::set_quaternion`]),
    /// so the algorithm starts with the configured gain instead of ramping
    /// down from a high startup gain.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::{Ahrs, Euler, Quaternion};
    ///
    /// let mut ahrs = Ahrs::new();
    /// ahrs.set_quaternion(Quaternion::from_euler(Euler::new(0.0, 0.0, 45.0)));
    /// ahrs.skip_startup();
    /// assert!(!ahrs.flags().startup);
    /// ```
    pub fn skip_startup(&mut self) {
        self.startup = false;
        self.overrange_recovery = false;
    }

    /// Update algorithm settings
    ///
    /// Changes the algorithm configuration and recalculates derived values.
    /// The sample period is reset to `1 / settings.sample_rate`.
    ///
    /// # Arguments
    /// * `settings` - New configuration to apply
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::{Ahrs, AhrsSettings};
    ///
    /// let mut ahrs = Ahrs::new();
    /// let mut settings = ahrs.get_settings();
    /// settings.gain = 0.25; // Lower gain for more stable operation
    /// ahrs.set_settings(settings);
    /// assert_eq!(ahrs.get_settings().gain, 0.25);
    /// ```
    pub fn set_settings(&mut self, settings: AhrsSettings) {
        self.settings = settings;
        self.config = Config::new(&settings);

        self.acceleration.recovery_threshold = self.config.rejection_timeout;
        self.magnetic.recovery_threshold = self.config.rejection_timeout;
    }

    /// Get current algorithm settings
    ///
    /// Returns a copy of the current algorithm configuration.
    ///
    /// # Returns
    /// Current AHRS settings
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::{Ahrs, Convention};
    ///
    /// let ahrs = Ahrs::new();
    /// let settings = ahrs.get_settings();
    /// assert_eq!(settings.convention, Convention::Nwu);
    /// assert_eq!(settings.gain, 0.5);
    /// ```
    pub fn get_settings(&self) -> AhrsSettings {
        self.settings
    }

    /// Set the sample period
    ///
    /// The sample period must be approximately equal to `1 / sample_rate`
    /// from the settings. Intended to be called before each update to
    /// compensate for gyroscope sample clock errors. The value persists until
    /// changed again or until [`Ahrs::set_settings`] is called.
    ///
    /// # Arguments
    /// * `sample_period` - Sample period in seconds
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new(); // 100 Hz nominal
    /// ahrs.set_sample_period(0.0101); // measured timestamp delta
    /// ahrs.update_no_magnetometer(Vector::ZERO, Vector::new(0.0, 0.0, 1.0));
    /// ```
    pub fn set_sample_period(&mut self, sample_period: f32) {
        self.config.sample_period = sample_period;
    }

    /// Update AHRS with gyroscope, accelerometer, and magnetometer data
    ///
    /// This is the main algorithm function that fuses all sensor readings
    /// to estimate orientation. The algorithm automatically:
    /// - Detects and rejects accelerometer readings during motion
    /// - Detects and rejects magnetometer readings during magnetic interference
    /// - Manages startup with ramped gain
    /// - Handles gyroscope overrange detection and recovery
    ///
    /// The gyroscope is integrated over the sample period derived from
    /// [`AhrsSettings::sample_rate`], or the value last passed to
    /// [`Ahrs::set_sample_period`].
    ///
    /// # Arguments
    /// * `gyroscope` - Gyroscope reading in degrees per second
    /// * `accelerometer` - Accelerometer reading in g
    /// * `magnetometer` - Magnetometer reading in any calibrated units
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new(); // 100 Hz
    ///
    /// // Typical sensor readings
    /// let gyro = Vector::new(0.1, -0.2, 0.05);     // Small rotation rates
    /// let accel = Vector::new(0.0, 0.0, 1.0);      // Gravity pointing up (NWU)
    /// let mag = Vector::new(25.0, 2.0, -15.0);     // Earth's magnetic field
    ///
    /// ahrs.update(gyro, accel, mag);
    ///
    /// let orientation = ahrs.quaternion();
    /// let gravity = ahrs.gravity();
    /// ```
    pub fn update(
        &mut self,
        gyroscope: impl Into<Vector>,
        accelerometer: impl Into<Vector>,
        magnetometer: impl Into<Vector>,
    ) {
        // Generic shell over a non-generic body, so the algorithm is compiled
        // (and inlined) in this crate rather than in each caller
        self.update_vectors(gyroscope.into(), accelerometer.into(), magnetometer.into());
    }

    /// Update AHRS without magnetometer (gyroscope and accelerometer only)
    ///
    /// Use this when magnetometer data is unavailable or unreliable.
    /// The algorithm will still estimate roll and pitch from the accelerometer
    /// but heading will drift over time. During startup, heading is
    /// automatically zeroed.
    ///
    /// # Arguments
    /// * `gyroscope` - Gyroscope reading in degrees per second
    /// * `accelerometer` - Accelerometer reading in g
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new();
    ///
    /// let gyro = Vector::new(0.1, -0.2, 0.05);
    /// let accel = Vector::new(0.0, 0.0, 1.0);
    ///
    /// ahrs.update_no_magnetometer(gyro, accel);
    ///
    /// // Roll and pitch will be accurate, heading may drift
    /// let euler = ahrs.quaternion().to_euler();
    /// ```
    pub fn update_no_magnetometer(
        &mut self,
        gyroscope: impl Into<Vector>,
        accelerometer: impl Into<Vector>,
    ) {
        self.update_no_magnetometer_vectors(gyroscope.into(), accelerometer.into());
    }

    /// Update AHRS with external heading source
    ///
    /// Use this when you have an external heading reference (GPS, compass,
    /// etc.) instead of a magnetometer. The function synthesizes a virtual
    /// magnetometer reading from the provided heading angle.
    ///
    /// # Arguments
    /// * `gyroscope` - Gyroscope reading in degrees per second
    /// * `accelerometer` - Accelerometer reading in g
    /// * `heading` - Heading angle in degrees
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new();
    ///
    /// let gyro = Vector::new(0.1, -0.2, 0.05);
    /// let accel = Vector::new(0.0, 0.0, 1.0);
    /// let heading_from_gps = 45.0; // 45° (northeast)
    ///
    /// ahrs.update_external_heading(gyro, accel, heading_from_gps);
    /// ```
    pub fn update_external_heading(
        &mut self,
        gyroscope: impl Into<Vector>,
        accelerometer: impl Into<Vector>,
        heading: f32,
    ) {
        let magnetometer = self.heading_magnetometer(heading);
        self.update_vectors(gyroscope.into(), accelerometer.into(), magnetometer);
    }

    /// Get current orientation quaternion
    ///
    /// Returns the estimated orientation of the sensor relative to the Earth
    /// frame (per the configured convention) as a unit quaternion.
    ///
    /// # Returns
    /// Unit quaternion representing device orientation (WXYZ format)
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::{Ahrs, Vector};
    ///
    /// let ahrs = Ahrs::new();
    /// let quaternion = ahrs.quaternion();
    ///
    /// // Convert to Euler angles in degrees
    /// let euler = quaternion.to_euler();
    ///
    /// // Or rotate a sensor-frame vector into the Earth frame
    /// let earth_vector = quaternion.rotate(Vector::new(1.0, 0.0, 0.0));
    /// assert_eq!(earth_vector, Vector::new(1.0, 0.0, 0.0));
    /// ```
    pub fn quaternion(&self) -> Quaternion {
        self.quaternion
    }

    /// Set orientation quaternion directly
    ///
    /// Allows direct setting of the device orientation. Useful for
    /// initialization with a known orientation or for external corrections.
    ///
    /// # Arguments
    /// * `quaternion` - New orientation quaternion
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::{Ahrs, Euler, Quaternion};
    ///
    /// let mut ahrs = Ahrs::new();
    ///
    /// // Set to 45° rotation around Z-axis
    /// let rotation = Quaternion::from_euler(Euler::new(0.0, 0.0, 45.0));
    /// ahrs.set_quaternion(rotation);
    ///
    /// assert_eq!(ahrs.quaternion(), rotation);
    /// ```
    pub fn set_quaternion(&mut self, quaternion: impl Into<Quaternion>) {
        self.quaternion = quaternion.into();
    }

    /// Get gravity vector in sensor frame
    ///
    /// Returns the direction of gravity as measured in the sensor coordinate frame.
    /// This is the negative of the accelerometer reading when the device is
    /// stationary (assuming proper calibration).
    ///
    /// # Returns
    /// Gravity vector in sensor frame (unit vector)
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Ahrs;
    ///
    /// let ahrs = Ahrs::new();
    /// let gravity = ahrs.gravity();
    ///
    /// // For a level device in NWU convention, gravity points down (-Z)
    /// // When tilted, gravity will point in different directions
    /// println!("Gravity: {:?}", gravity);
    /// ```
    pub fn gravity(&self) -> Vector {
        self.calculate_half_gravity() * 2.0
    }

    /// Get linear acceleration (acceleration minus gravity)
    ///
    /// Calculates the linear acceleration by subtracting the estimated
    /// gravity vector from the accelerometer reading. This represents
    /// the motion-induced acceleration in the sensor frame.
    ///
    /// # Returns
    /// Linear acceleration vector in sensor frame (units of g)
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new();
    ///
    /// // Simulate accelerometer reading with motion
    /// let accel_with_motion = Vector::new(0.5, 0.0, 1.0); // 0.5g lateral + gravity
    /// ahrs.update_no_magnetometer(Vector::ZERO, accel_with_motion);
    ///
    /// let linear_accel = ahrs.linear_acceleration();
    /// // Should show the 0.5g lateral acceleration
    /// ```
    pub fn linear_acceleration(&self) -> Vector {
        self.accelerometer - self.gravity()
    }

    /// Get earth-frame acceleration
    ///
    /// Transforms the linear acceleration from sensor frame to Earth frame
    /// using the current orientation estimate. This provides acceleration
    /// in the global coordinate system.
    ///
    /// # Returns
    /// Linear acceleration vector in Earth frame (units of g)
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Vector;
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new();
    ///
    /// // Update with some motion
    /// ahrs.update_no_magnetometer(Vector::ZERO, Vector::new(0.5, 0.0, 1.0));
    ///
    /// let earth_accel = ahrs.earth_acceleration();
    /// // Acceleration now expressed in Earth coordinates
    /// ```
    pub fn earth_acceleration(&self) -> Vector {
        let q = self.quaternion;
        let (qw, qx, qy, qz) = (q.w, q.x, q.y, q.z);
        let a = self.accelerometer;

        // Rotation matrix multiplied with the accelerometer
        let mut acceleration = Vector::new(
            2.0 * ((qw * qw - 0.5 + qx * qx) * a.x
                + (qx * qy - qw * qz) * a.y
                + (qx * qz + qw * qy) * a.z),
            2.0 * ((qx * qy + qw * qz) * a.x
                + (qw * qw - 0.5 + qy * qy) * a.y
                + (qy * qz - qw * qx) * a.z),
            2.0 * ((qx * qz - qw * qy) * a.x
                + (qy * qz + qw * qx) * a.y
                + (qw * qw - 0.5 + qz * qz) * a.z),
        );

        match self.settings.convention {
            Convention::Nwu | Convention::Enu => acceleration.z -= 1.0,
            Convention::Ned => acceleration.z += 1.0,
        }
        acceleration
    }

    /// Get internal algorithm states
    ///
    /// Provides diagnostic information about the algorithm's internal state,
    /// including error measurements and sensor rejection status. Useful
    /// for monitoring algorithm performance and debugging.
    ///
    /// # Returns
    /// Structure containing internal algorithm diagnostics
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Ahrs;
    ///
    /// let ahrs = Ahrs::new();
    /// let states = ahrs.internal_states();
    ///
    /// println!("Acceleration error: {:.2}°", states.acceleration_error);
    /// println!("Accelerometer ignored: {}", states.accelerometer_ignored);
    /// println!("Magnetic error: {:.2}°", states.magnetic_error);
    /// println!("Magnetometer ignored: {}", states.magnetometer_ignored);
    /// ```
    pub fn internal_states(&self) -> AhrsInternalStates {
        let timeout = self.config.rejection_timeout;
        AhrsInternalStates {
            acceleration_error: self.acceleration.error(),
            accelerometer_ignored: self.acceleration.ignored,
            acceleration_recovery_trigger: self.acceleration.recovery_trigger(timeout),
            magnetic_error: self.magnetic.error(),
            magnetometer_ignored: self.magnetic.ignored,
            magnetic_recovery_trigger: self.magnetic.recovery_trigger(timeout),
        }
    }

    /// Get algorithm flags
    ///
    /// Returns status flags indicating the current operating mode of the
    /// algorithm, including startup and recovery states.
    ///
    /// # Returns
    /// Structure containing algorithm status flags
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Ahrs;
    ///
    /// let ahrs = Ahrs::new();
    /// let flags = ahrs.flags();
    ///
    /// if flags.startup {
    ///     println!("Algorithm is still starting up");
    /// }
    /// if flags.overrange_recovery {
    ///     println!("Recovering from gyroscope overrange");
    /// }
    /// ```
    pub fn flags(&self) -> AhrsFlags {
        AhrsFlags {
            startup: self.startup,
            overrange_recovery: self.overrange_recovery,
            acceleration_recovery: self.acceleration.recovery(),
            magnetic_recovery: self.magnetic.recovery(),
        }
    }

    /// Set heading angle directly
    ///
    /// Sets the heading (yaw) angle while preserving the current roll and pitch.
    /// Useful for compass calibration or initialization with a known heading.
    ///
    /// # Arguments
    /// * `heading` - New heading angle in degrees (0° = North, positive = clockwise)
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new();
    ///
    /// // Set heading to face East (90°)
    /// ahrs.set_heading(90.0);
    ///
    /// let yaw = ahrs.quaternion().to_euler().yaw;
    /// assert!((yaw - 90.0).abs() < 1.0);
    /// ```
    pub fn set_heading(&mut self, heading: f32) {
        let q = self.quaternion;
        let yaw = libm::atan2f(q.w * q.z + q.x * q.y, 0.5 - q.y * q.y - q.z * q.z);
        let half_yaw_minus_heading = 0.5 * (yaw - heading * DEG_TO_RAD);
        let rotation = Quaternion::new(
            libm::cosf(half_yaw_minus_heading),
            0.0,
            0.0,
            -libm::sinf(half_yaw_minus_heading),
        );
        self.quaternion = rotation * self.quaternion;
    }

    // Private helper methods

    /// Non-generic body of [`Ahrs::update`]
    fn update_vectors(&mut self, gyroscope: Vector, accelerometer: Vector, magnetometer: Vector) {
        self.accelerometer = accelerometer;

        self.overrange(gyroscope);

        let gain = self.startup_gain();

        let half_gyroscope = gyroscope * (DEG_TO_RAD * 0.5);

        let half_gravity = self.calculate_half_gravity();

        let half_feedback = self.half_inclination_feedback(half_gravity, accelerometer)
            + self.half_heading_feedback(half_gravity, magnetometer);

        let half_angular_rate = half_gyroscope + half_feedback * gain;

        self.integrate_quaternion(half_angular_rate * self.config.sample_period);
    }

    /// Non-generic body of [`Ahrs::update_no_magnetometer`]
    fn update_no_magnetometer_vectors(&mut self, gyroscope: Vector, accelerometer: Vector) {
        self.update_vectors(gyroscope, accelerometer, Vector::ZERO);

        // Zero heading during startup
        if self.startup {
            self.set_heading(0.0);
        }
    }

    /// Magnetometer equivalent to an external heading in degrees
    fn heading_magnetometer(&self, heading: f32) -> Vector {
        let q = self.quaternion;
        let roll = libm::atan2f(q.w * q.x + q.y * q.z, 0.5 - q.y * q.y - q.x * q.x);

        let heading_radians = heading * DEG_TO_RAD;
        let sin_heading = libm::sinf(heading_radians);
        Vector::new(
            libm::cosf(heading_radians),
            -libm::cosf(roll) * sin_heading,
            sin_heading * libm::sinf(roll),
        )
    }

    /// Trigger a soft restart if gyroscope overrange is detected
    fn overrange(&mut self, gyroscope: Vector) {
        if !self.config.overrange_enabled {
            return;
        }

        let threshold = self.config.overrange_threshold;
        if libm::fabsf(gyroscope.x) <= threshold
            && libm::fabsf(gyroscope.y) <= threshold
            && libm::fabsf(gyroscope.z) <= threshold
        {
            return;
        }

        self.soft_restart();
        self.overrange_recovery = true;
    }

    /// Restart the algorithm while preserving outputs
    fn soft_restart(&mut self) {
        let quaternion = self.quaternion;
        let accelerometer = self.accelerometer;

        self.restart();

        self.quaternion = quaternion;
        self.accelerometer = accelerometer;
    }

    /// Ramp down the gain during startup and return the gain to apply
    fn startup_gain(&mut self) -> f32 {
        if !self.startup {
            return self.config.gain;
        }

        self.startup_gain -= self.config.startup_gain_rate;

        if self.startup_gain > self.config.gain {
            return self.startup_gain;
        }

        self.startup = false;
        self.overrange_recovery = false;

        self.config.gain
    }

    /// Return inclination feedback scaled by 0.5
    fn half_inclination_feedback(&mut self, half_gravity: Vector, accelerometer: Vector) -> Vector {
        if accelerometer.is_zero() {
            self.acceleration.skip();
            return Vector::ZERO;
        }

        let half_residual = residual(accelerometer.normalize(), half_gravity);
        self.acceleration.update(
            half_residual,
            self.startup,
            self.config.acceleration_rejection,
            self.config.rejection_timeout,
        )
    }

    /// Return heading feedback scaled by 0.5
    fn half_heading_feedback(&mut self, half_gravity: Vector, magnetometer: Vector) -> Vector {
        if magnetometer.is_zero() {
            self.magnetic.skip();
            return Vector::ZERO;
        }

        // Direction of magnetic field indicated by algorithm
        let half_west = self.calculate_half_west();

        let half_residual = residual(half_gravity.cross(magnetometer).normalize(), half_west);
        self.magnetic.update(
            half_residual,
            self.startup,
            self.config.magnetic_rejection,
            self.config.rejection_timeout,
        )
    }

    /// Calculate half gravity vector in sensor frame based on current quaternion
    fn calculate_half_gravity(&self) -> Vector {
        let Quaternion {
            w: qw,
            x: qx,
            y: qy,
            z: qz,
        } = self.quaternion;

        match self.settings.convention {
            Convention::Nwu | Convention::Enu => Vector::new(
                qx * qz - qw * qy,
                qy * qz + qw * qx,
                qw * qw - 0.5 + qz * qz,
            ),
            Convention::Ned => Vector::new(
                qw * qy - qx * qz,
                -(qy * qz + qw * qx),
                0.5 - qw * qw - qz * qz,
            ),
        }
    }

    /// Calculate direction of west in sensor frame scaled by 0.5. The cross
    /// product of gravity and the magnetometer is west.
    fn calculate_half_west(&self) -> Vector {
        let Quaternion {
            w: qw,
            x: qx,
            y: qy,
            z: qz,
        } = self.quaternion;

        match self.settings.convention {
            // C: second column of transposed rotation matrix scaled by 0.5
            Convention::Nwu => Vector::new(
                qx * qy + qw * qz,
                qw * qw - 0.5 + qy * qy,
                qy * qz - qw * qx,
            ),
            // C: first column of transposed rotation matrix scaled by -0.5
            Convention::Enu => Vector::new(
                0.5 - qw * qw - qx * qx,
                qw * qz - qx * qy,
                -(qx * qz + qw * qy),
            ),
            // C: second column of transposed rotation matrix scaled by -0.5
            Convention::Ned => Vector::new(
                -(qx * qy + qw * qz),
                0.5 - qw * qw - qy * qy,
                qw * qx - qy * qz,
            ),
        }
    }

    /// Integrate the quaternion by the half angular displacement
    fn integrate_quaternion(&mut self, half_angular_displacement: Vector) {
        let q = self.quaternion;
        self.quaternion = (q + q.vector_product(half_angular_displacement)).normalize();
    }
}

/// Convert a rejection angle in degrees to a squared half-residual threshold
fn rejection_threshold(degrees: f32) -> f32 {
    if degrees == 0.0 {
        f32::MAX
    } else {
        let half_sin = 0.5 * libm::sinf(degrees * DEG_TO_RAD);
        half_sin * half_sin
    }
}

/// Clamp matching C's `Clamp`: never panics when `min > max`
fn clamp(value: i32, min: i32, max: i32) -> i32 {
    if value < min {
        return min;
    }
    if value > max {
        return max;
    }
    value
}

/// Residual between the sensor and reference vectors
fn residual(sensor: Vector, reference: Vector) -> Vector {
    let cross = sensor.cross(reference);

    // Error is <90 degrees
    if sensor.dot(reference) > 0.0 {
        return cross;
    }

    // normalize returns zero when sensor and reference are exactly opposite
    cross.normalize()
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
        assert_eq!(ahrs.quaternion(), Quaternion::IDENTITY);
        assert!(ahrs.flags().startup);
    }

    #[test]
    fn test_ahrs_startup() {
        let mut ahrs = Ahrs::new();

        // Should start in startup
        assert!(ahrs.flags().startup);

        // Update for startup period to complete ramping
        let gyro = Vector::ZERO;
        let accel = Vector::new(0.0, 0.0, 1.0);
        let mag = Vector::new(1.0, 0.0, 0.0);

        // Simulate 4 seconds at 100Hz to complete startup
        for _ in 0..400 {
            ahrs.update(gyro, accel, mag);
        }

        // Should no longer be in startup
        assert!(!ahrs.flags().startup);
    }

    #[test]
    fn test_gravity_calculation() {
        let ahrs = Ahrs::new();
        let gravity = ahrs.gravity();

        // Should be unit vector pointing up in NWU convention
        assert!((gravity.norm() - 1.0).abs() < 1e-6);
        assert!((gravity.z - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_gyroscope_overrange_detection() {
        let settings = AhrsSettings {
            gyroscope_range: 500.0, // 500 deg/s range
            ..Default::default()
        };
        let mut ahrs = Ahrs::with_settings(settings);

        // Complete initialization first
        let normal_gyro = Vector::ZERO;
        let accel = Vector::new(0.0, 0.0, 1.0);
        let mag = Vector::new(1.0, 0.0, 0.0);

        for _ in 0..400 {
            ahrs.update(normal_gyro, accel, mag);
        }
        assert!(!ahrs.flags().startup);

        // Now test overrange
        let overflow_gyro = Vector::new(600.0, 0.0, 0.0); // Exceeds 500 deg/s
        ahrs.update(overflow_gyro, accel, mag);

        assert!(ahrs.flags().overrange_recovery);
        assert!(ahrs.flags().startup); // Should restart startup
    }

    #[test]
    fn test_accelerometer_rejection() {
        let settings = AhrsSettings {
            acceleration_rejection: 10.0, // 10 degree threshold
            rejection_timeout: 1.0,
            ..Default::default()
        };

        let mut ahrs = Ahrs::with_settings(settings);

        // Complete initialization first
        let gyro = Vector::ZERO;
        let normal_accel = Vector::new(0.0, 0.0, 1.0); // Normal gravity
        let mag = Vector::new(1.0, 0.0, 0.0);

        for _ in 0..400 {
            ahrs.update(gyro, normal_accel, mag);
        }

        // Test that normal acceleration is accepted
        let states = ahrs.internal_states();
        assert!(!states.accelerometer_ignored);

        // Apply large acceleration (should be rejected after enough samples)
        let large_accel = Vector::new(2.0, 2.0, 1.0); // Large acceleration indicating motion

        // Apply bad readings repeatedly to eventually trigger rejection
        let mut rejected = false;
        for _i in 0..150 {
            ahrs.update(gyro, large_accel, mag);
            let states = ahrs.internal_states();

            if states.accelerometer_ignored || states.acceleration_recovery_trigger > 50.0 {
                rejected = true;
                break;
            }
        }

        // Should eventually trigger rejection mechanism
        assert!(
            rejected,
            "Accelerometer should be rejected for large accelerations"
        );
    }

    #[test]
    fn test_skip_startup() {
        let mut ahrs = Ahrs::new();
        ahrs.skip_startup();
        assert!(!ahrs.flags().startup);

        // Configured gain applies immediately: 1 s of tilted accel converges slowly
        let tilted = Vector::new(0.0, 1.0, 0.0);
        for _ in 0..100 {
            ahrs.update_no_magnetometer(Vector::ZERO, tilted);
        }
        let skipped = ahrs.gravity();

        let mut ahrs = Ahrs::new();
        for _ in 0..100 {
            ahrs.update_no_magnetometer(Vector::ZERO, tilted);
        }
        // Startup gain converges faster than the configured gain
        assert!(ahrs.gravity().y > skipped.y);
    }

    #[test]
    fn test_sample_period() {
        let gyro = Vector::new(0.0, 0.0, 90.0);

        let mut ahrs = Ahrs::new(); // 100 Hz
        ahrs.skip_startup();
        ahrs.update(gyro, Vector::ZERO, Vector::ZERO);
        let yaw_default = ahrs.quaternion().to_euler().yaw;

        let mut ahrs = Ahrs::new();
        ahrs.skip_startup();
        ahrs.set_sample_period(0.02);
        ahrs.update(gyro, Vector::ZERO, Vector::ZERO);
        let yaw_doubled = ahrs.quaternion().to_euler().yaw;

        assert!((yaw_default - 0.9).abs() < 1e-3);
        assert!((yaw_doubled - 1.8).abs() < 1e-3);

        // set_settings resets the sample period
        ahrs.set_settings(AhrsSettings {
            sample_rate: 50.0,
            ..Default::default()
        });
        assert_eq!(ahrs.config.sample_period, 0.02);
    }

    #[test]
    fn test_overrange_preserves_outputs() {
        let settings = AhrsSettings {
            gyroscope_range: 500.0,
            ..Default::default()
        };
        let mut ahrs = Ahrs::with_settings(settings);
        let accel = Vector::new(0.0, 0.5, 1.0);
        ahrs.update_no_magnetometer(Vector::ZERO, accel);

        // Soft restart keeps the quaternion rather than resetting to identity,
        // and linear acceleration still reflects the latest accelerometer
        ahrs.update_no_magnetometer(Vector::new(600.0, 0.0, 0.0), accel);
        assert!(ahrs.flags().overrange_recovery);
        assert!(ahrs.flags().startup);
        assert_ne!(ahrs.quaternion(), Quaternion::IDENTITY);
        assert_eq!(ahrs.linear_acceleration(), accel - ahrs.gravity());
    }

    #[test]
    fn test_residual_opposite_vectors() {
        let sensor = Vector::new(0.0, 0.0, 1.0);
        let reference = Vector::new(0.0, 0.0, -0.5);
        let r = residual(sensor, reference);
        assert_eq!(r, Vector::ZERO);

        // Orthogonal vectors are normalised
        let r = residual(Vector::new(1.0, 0.0, 0.0), Vector::new(0.0, 0.5, 0.0));
        assert!((r.norm() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_upside_down_start_no_nan() {
        let mut ahrs = Ahrs::new();
        ahrs.update_no_magnetometer(Vector::ZERO, Vector::new(0.0, 0.0, -1.0));
        let q = ahrs.quaternion();
        assert!(q.w.is_finite() && q.x.is_finite() && q.y.is_finite() && q.z.is_finite());
    }

    #[test]
    fn test_default_rejection_disabled() {
        let mut ahrs = Ahrs::new();
        ahrs.skip_startup();
        for _ in 0..1000 {
            ahrs.update_no_magnetometer(Vector::ZERO, Vector::new(1.0, 1.0, 0.0));
            assert!(!ahrs.internal_states().accelerometer_ignored);
        }
        assert_eq!(ahrs.internal_states().acceleration_recovery_trigger, 0.0);
    }

    #[test]
    fn test_negative_rejection_timeout_does_not_panic() {
        let settings = AhrsSettings {
            acceleration_rejection: 10.0,
            magnetic_rejection: 10.0,
            rejection_timeout: -1.0,
            ..Default::default()
        };
        let mut ahrs = Ahrs::with_settings(settings);
        for _ in 0..10 {
            ahrs.update(
                Vector::ZERO,
                Vector::new(1.0, 0.0, 0.0),
                Vector::new(0.0, 1.0, 0.0),
            );
        }
    }

    #[test]
    fn test_clamp_matches_c() {
        assert_eq!(clamp(-5, 0, 10), 0);
        assert_eq!(clamp(15, 0, 10), 10);
        assert_eq!(clamp(5, 0, 10), 5);
        // min > max: C checks min first, then max
        assert_eq!(clamp(-5, 0, -100), 0);
        assert_eq!(clamp(5, 0, -100), -100);
    }
}
