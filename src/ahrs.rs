//! Main AHRS algorithm implementation for the Fusion AHRS library

use crate::math::{DEG_TO_RAD, Vector3Ext};
use crate::types::{AhrsFlags, AhrsInternalStates, AhrsSettings, Convention};
#[allow(unused_imports)]
use nalgebra::{ComplexField, RealField}; // Required for no_std float methods
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

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
pub struct Ahrs {
    /// Algorithm settings
    settings: AhrsSettings,
    /// Sample period in seconds
    sample_period: f32,
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
    /// Current orientation quaternion (WXYZ format)
    quaternion: UnitQuaternion<f32>,
    /// Last accelerometer reading for linear acceleration calculation
    accelerometer: Vector3<f32>,
    /// Whether the algorithm is in startup
    startup: bool,
    /// Gain ramped down during startup
    startup_gain: f32,
    /// Gyroscope overrange recovery flag
    overrange_recovery: bool,
    /// Accelerometer residual scaled by 0.5
    half_accelerometer_residual: Vector3<f32>,
    /// Acceleration recovery trigger in samples
    acceleration_recovery_trigger: i32,
    /// Acceleration recovery threshold in samples
    acceleration_recovery_threshold: i32,
    /// Accelerometer ignored flag
    accelerometer_ignored: bool,
    /// Magnetometer residual scaled by 0.5
    half_magnetometer_residual: Vector3<f32>,
    /// Magnetic recovery trigger in samples
    magnetic_recovery_trigger: i32,
    /// Magnetic recovery threshold in samples
    magnetic_recovery_threshold: i32,
    /// Magnetometer ignored flag
    magnetometer_ignored: bool,
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
        let mut ahrs = Ahrs {
            settings,
            sample_period: 0.0,
            startup_gain_rate: 0.0,
            overrange_enabled: false,
            overrange_threshold: 0.0,
            acceleration_rejection: 0.0,
            magnetic_rejection: 0.0,
            rejection_timeout: 0,
            quaternion: UnitQuaternion::identity(),
            accelerometer: Vector3::zeros(),
            startup: true,
            startup_gain: INITIAL_STARTUP_GAIN,
            overrange_recovery: false,
            half_accelerometer_residual: Vector3::zeros(),
            acceleration_recovery_trigger: 0,
            acceleration_recovery_threshold: 0,
            accelerometer_ignored: false,
            half_magnetometer_residual: Vector3::zeros(),
            magnetic_recovery_trigger: 0,
            magnetic_recovery_threshold: 0,
            magnetometer_ignored: false,
        };

        ahrs.set_settings(settings);
        ahrs.restart();
        ahrs
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
        self.quaternion = UnitQuaternion::identity();
        self.accelerometer = Vector3::zeros();

        self.startup = true;
        self.startup_gain = INITIAL_STARTUP_GAIN;

        self.overrange_recovery = false;

        self.half_accelerometer_residual = Vector3::zeros();
        self.acceleration_recovery_trigger = 0;
        self.acceleration_recovery_threshold = self.rejection_timeout;
        self.accelerometer_ignored = false;

        self.half_magnetometer_residual = Vector3::zeros();
        self.magnetic_recovery_trigger = 0;
        self.magnetic_recovery_threshold = self.rejection_timeout;
        self.magnetometer_ignored = false;
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
    /// use nalgebra::UnitQuaternion;
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new();
    /// ahrs.set_quaternion(UnitQuaternion::from_euler_angles(0.0, 0.0, 1.0));
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
        self.sample_period = 1.0 / settings.sample_rate;

        self.startup_gain_rate =
            ((INITIAL_STARTUP_GAIN - settings.gain) / STARTUP_PERIOD) * self.sample_period;

        self.overrange_enabled = settings.gyroscope_range > 0.0;
        self.overrange_threshold = OVERRANGE_FACTOR * settings.gyroscope_range;

        self.acceleration_rejection = rejection_threshold(settings.acceleration_rejection);
        self.magnetic_rejection = rejection_threshold(settings.magnetic_rejection);
        self.rejection_timeout = (settings.sample_rate * settings.rejection_timeout) as i32;

        self.acceleration_recovery_threshold = self.rejection_timeout;
        self.magnetic_recovery_threshold = self.rejection_timeout;

        // Disable acceleration and magnetic rejection if gain or timeout is zero
        if settings.gain == 0.0 || settings.rejection_timeout == 0.0 {
            self.acceleration_rejection = f32::MAX;
            self.magnetic_rejection = f32::MAX;
        }
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
    /// use nalgebra::Vector3;
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new(); // 100 Hz nominal
    /// ahrs.set_sample_period(0.0101); // measured timestamp delta
    /// ahrs.update_no_magnetometer(Vector3::zeros(), Vector3::new(0.0, 0.0, 1.0));
    /// ```
    pub fn set_sample_period(&mut self, sample_period: f32) {
        self.sample_period = sample_period;
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
    /// use nalgebra::Vector3;
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new(); // 100 Hz
    ///
    /// // Typical sensor readings
    /// let gyro = Vector3::new(0.1, -0.2, 0.05);     // Small rotation rates
    /// let accel = Vector3::new(0.0, 0.0, 1.0);      // Gravity pointing up (NWU)
    /// let mag = Vector3::new(25.0, 2.0, -15.0);     // Earth's magnetic field
    ///
    /// ahrs.update(gyro, accel, mag);
    ///
    /// let orientation = ahrs.quaternion();
    /// let gravity = ahrs.gravity();
    /// ```
    pub fn update(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        magnetometer: Vector3<f32>,
    ) {
        self.accelerometer = accelerometer;

        self.overrange(gyroscope);

        let gain = self.startup_gain();

        let half_gyroscope = gyroscope * (DEG_TO_RAD * 0.5);

        let half_gravity = self.calculate_half_gravity();

        let half_feedback = self.half_inclination_feedback(half_gravity, accelerometer)
            + self.half_heading_feedback(half_gravity, magnetometer);

        let half_angular_rate = half_gyroscope + half_feedback * gain;

        self.integrate_quaternion(half_angular_rate * self.sample_period);
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
    /// use nalgebra::Vector3;
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new();
    ///
    /// let gyro = Vector3::new(0.1, -0.2, 0.05);
    /// let accel = Vector3::new(0.0, 0.0, 1.0);
    ///
    /// ahrs.update_no_magnetometer(gyro, accel);
    ///
    /// // Roll and pitch will be accurate, heading may drift
    /// let euler = ahrs.quaternion().euler_angles();
    /// ```
    pub fn update_no_magnetometer(&mut self, gyroscope: Vector3<f32>, accelerometer: Vector3<f32>) {
        self.update(gyroscope, accelerometer, Vector3::zeros());

        // Zero heading during startup
        if self.startup {
            self.set_heading(0.0);
        }
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
    /// use nalgebra::Vector3;
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new();
    ///
    /// let gyro = Vector3::new(0.1, -0.2, 0.05);
    /// let accel = Vector3::new(0.0, 0.0, 1.0);
    /// let heading_from_gps = 45.0; // 45° (northeast)
    ///
    /// ahrs.update_external_heading(gyro, accel, heading_from_gps);
    /// ```
    pub fn update_external_heading(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        heading: f32,
    ) {
        // Calculate roll from quaternion
        let q = self.quaternion.as_ref();
        let qw = q.w;
        let qx = q.i;
        let qy = q.j;
        let qz = q.k;

        let roll = (qw * qx + qy * qz).atan2(0.5 - qy * qy - qx * qx);

        // Calculate synthetic magnetometer from heading and roll
        let heading_rad = heading * DEG_TO_RAD;
        let sin_heading = heading_rad.sin();
        let cos_heading = heading_rad.cos();
        let sin_roll = roll.sin();
        let cos_roll = roll.cos();

        let magnetometer =
            Vector3::new(cos_heading, -cos_roll * sin_heading, sin_heading * sin_roll);

        // Update with synthetic magnetometer
        self.update(gyroscope, accelerometer, magnetometer);
    }

    /// Get current orientation quaternion
    ///
    /// Returns the estimated device orientation as a unit quaternion.
    /// The quaternion represents the rotation from the Earth frame
    /// to the sensor frame according to the configured convention.
    ///
    /// # Returns
    /// Unit quaternion representing device orientation (WXYZ format)
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Ahrs;
    ///
    /// let ahrs = Ahrs::new();
    /// let quaternion = ahrs.quaternion();
    ///
    /// // Convert to Euler angles if needed
    /// let (roll, pitch, yaw) = quaternion.euler_angles();
    ///
    /// // Or use for transformations
    /// let sensor_vector = nalgebra::Vector3::new(1.0, 0.0, 0.0);
    /// let earth_vector = quaternion * sensor_vector;
    /// ```
    pub fn quaternion(&self) -> UnitQuaternion<f32> {
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
    /// use nalgebra::UnitQuaternion;
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new();
    ///
    /// // Set to 45° rotation around Z-axis
    /// let rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, 45.0_f32.to_radians());
    /// ahrs.set_quaternion(rotation);
    ///
    /// assert_eq!(ahrs.quaternion(), rotation);
    /// ```
    pub fn set_quaternion(&mut self, quaternion: UnitQuaternion<f32>) {
        self.quaternion = quaternion;
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
    pub fn gravity(&self) -> Vector3<f32> {
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
    /// use nalgebra::Vector3;
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new();
    ///
    /// // Simulate accelerometer reading with motion
    /// let accel_with_motion = Vector3::new(0.5, 0.0, 1.0); // 0.5g lateral + gravity
    /// ahrs.update_no_magnetometer(Vector3::zeros(), accel_with_motion);
    ///
    /// let linear_accel = ahrs.linear_acceleration();
    /// // Should show the 0.5g lateral acceleration
    /// ```
    pub fn linear_acceleration(&self) -> Vector3<f32> {
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
    /// use nalgebra::Vector3;
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new();
    ///
    /// // Update with some motion
    /// ahrs.update_no_magnetometer(Vector3::zeros(), Vector3::new(0.5, 0.0, 1.0));
    ///
    /// let earth_accel = ahrs.earth_acceleration();
    /// // Acceleration now expressed in Earth coordinates
    /// ```
    pub fn earth_acceleration(&self) -> Vector3<f32> {
        let q = self.quaternion.as_ref();
        let (qw, qx, qy, qz) = (q.w, q.i, q.j, q.k);
        let a = self.accelerometer;

        // Rotation matrix multiplied with the accelerometer
        let mut acceleration = Vector3::new(
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
        // Clamp to valid asin range to match FusionArcSin
        let error = |half_residual: Vector3<f32>| {
            (2.0 * half_residual.magnitude())
                .clamp(-1.0, 1.0)
                .asin()
                .to_degrees()
        };
        let trigger = |trigger: i32| {
            if self.rejection_timeout == 0 {
                0.0
            } else {
                trigger as f32 / self.rejection_timeout as f32
            }
        };

        AhrsInternalStates {
            acceleration_error: error(self.half_accelerometer_residual),
            accelerometer_ignored: self.accelerometer_ignored,
            acceleration_recovery_trigger: trigger(self.acceleration_recovery_trigger),
            magnetic_error: error(self.half_magnetometer_residual),
            magnetometer_ignored: self.magnetometer_ignored,
            magnetic_recovery_trigger: trigger(self.magnetic_recovery_trigger),
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
            acceleration_recovery: self.acceleration_recovery_trigger
                > self.acceleration_recovery_threshold,
            magnetic_recovery: self.magnetic_recovery_trigger > self.magnetic_recovery_threshold,
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
    /// let (_, _, yaw) = ahrs.quaternion().euler_angles();
    /// assert!((yaw.to_degrees() - 90.0).abs() < 1.0);
    /// ```
    pub fn set_heading(&mut self, heading: f32) {
        let q = self.quaternion.as_ref();
        let yaw = (q.w * q.k + q.i * q.j).atan2(0.5 - q.j * q.j - q.k * q.k);
        let half_yaw_minus_heading = 0.5 * (yaw - heading * DEG_TO_RAD);
        let rotation = UnitQuaternion::from_quaternion(Quaternion::new(
            half_yaw_minus_heading.cos(),
            0.0,
            0.0,
            -half_yaw_minus_heading.sin(),
        ));
        self.quaternion = rotation * self.quaternion;
    }

    // Private helper methods

    /// Trigger a soft restart if gyroscope overrange is detected
    fn overrange(&mut self, gyroscope: Vector3<f32>) {
        if !self.overrange_enabled {
            return;
        }

        if gyroscope.x.abs() <= self.overrange_threshold
            && gyroscope.y.abs() <= self.overrange_threshold
            && gyroscope.z.abs() <= self.overrange_threshold
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
            return self.settings.gain;
        }

        self.startup_gain -= self.startup_gain_rate;

        if self.startup_gain > self.settings.gain {
            return self.startup_gain;
        }

        self.startup = false;
        self.overrange_recovery = false;

        self.settings.gain
    }

    /// Return inclination feedback scaled by 0.5
    fn half_inclination_feedback(
        &mut self,
        half_gravity: Vector3<f32>,
        accelerometer: Vector3<f32>,
    ) -> Vector3<f32> {
        let mut half_inclination_feedback = Vector3::zeros();
        self.accelerometer_ignored = true;
        if accelerometer != Vector3::zeros() {
            // Calculate accelerometer residual scaled by 0.5
            self.half_accelerometer_residual =
                residual(accelerometer.safe_normalize(), half_gravity);

            // Don't ignore accelerometer if acceleration error below threshold
            if self.startup
                || self.half_accelerometer_residual.magnitude_squared()
                    <= self.acceleration_rejection
            {
                self.accelerometer_ignored = false;
                self.acceleration_recovery_trigger -= RECOVERY_DECREMENT;
            } else {
                self.acceleration_recovery_trigger += 1;
            }

            // Don't ignore accelerometer during acceleration recovery
            if self.acceleration_recovery_trigger > self.acceleration_recovery_threshold {
                self.acceleration_recovery_threshold = 0;
                self.accelerometer_ignored = false;
            } else {
                self.acceleration_recovery_threshold = self.rejection_timeout;
            }
            self.acceleration_recovery_trigger = self
                .acceleration_recovery_trigger
                .clamp(0, self.rejection_timeout);

            // Apply accelerometer feedback
            if !self.accelerometer_ignored {
                half_inclination_feedback = self.half_accelerometer_residual;
            }
        }
        half_inclination_feedback
    }

    /// Return heading feedback scaled by 0.5
    fn half_heading_feedback(
        &mut self,
        half_gravity: Vector3<f32>,
        magnetometer: Vector3<f32>,
    ) -> Vector3<f32> {
        let mut half_heading_feedback = Vector3::zeros();
        self.magnetometer_ignored = true;
        if magnetometer != Vector3::zeros() {
            // Calculate direction of magnetic field indicated by algorithm
            let half_west = self.calculate_half_west();

            // Calculate magnetometer residual scaled by 0.5
            self.half_magnetometer_residual = residual(
                half_gravity.cross(&magnetometer).safe_normalize(),
                half_west,
            );

            // Don't ignore magnetometer if magnetic error below threshold
            if self.startup
                || self.half_magnetometer_residual.magnitude_squared() <= self.magnetic_rejection
            {
                self.magnetometer_ignored = false;
                self.magnetic_recovery_trigger -= RECOVERY_DECREMENT;
            } else {
                self.magnetic_recovery_trigger += 1;
            }

            // Don't ignore magnetometer during magnetic recovery
            if self.magnetic_recovery_trigger > self.magnetic_recovery_threshold {
                self.magnetic_recovery_threshold = 0;
                self.magnetometer_ignored = false;
            } else {
                self.magnetic_recovery_threshold = self.rejection_timeout;
            }
            self.magnetic_recovery_trigger = self
                .magnetic_recovery_trigger
                .clamp(0, self.rejection_timeout);

            // Apply magnetometer feedback
            if !self.magnetometer_ignored {
                half_heading_feedback = self.half_magnetometer_residual;
            }
        }
        half_heading_feedback
    }

    /// Calculate half gravity vector in sensor frame based on current quaternion
    fn calculate_half_gravity(&self) -> Vector3<f32> {
        let q = self.quaternion.as_ref();
        let qw = q.w;
        let qx = q.i;
        let qy = q.j;
        let qz = q.k;

        match self.settings.convention {
            Convention::Nwu | Convention::Enu => Vector3::new(
                qx * qz - qw * qy,
                qy * qz + qw * qx,
                qw * qw - 0.5 + qz * qz,
            ),
            Convention::Ned => Vector3::new(
                qw * qy - qx * qz,
                -(qy * qz + qw * qx),
                0.5 - qw * qw - qz * qz,
            ),
        }
    }

    /// Calculate direction of west in sensor frame scaled by 0.5. The cross
    /// product of gravity and the magnetometer is west.
    fn calculate_half_west(&self) -> Vector3<f32> {
        let q = self.quaternion.as_ref();
        let qw = q.w;
        let qx = q.i;
        let qy = q.j;
        let qz = q.k;

        match self.settings.convention {
            // C: second column of transposed rotation matrix scaled by 0.5
            Convention::Nwu => Vector3::new(
                qx * qy + qw * qz,
                qw * qw - 0.5 + qy * qy,
                qy * qz - qw * qx,
            ),
            // C: first column of transposed rotation matrix scaled by -0.5
            Convention::Enu => Vector3::new(
                0.5 - qw * qw - qx * qx,
                qw * qz - qx * qy,
                -(qx * qz + qw * qy),
            ),
            // C: second column of transposed rotation matrix scaled by -0.5
            Convention::Ned => Vector3::new(
                -(qx * qy + qw * qz),
                0.5 - qw * qw - qy * qy,
                qw * qx - qy * qz,
            ),
        }
    }

    /// Integrate the quaternion by the half angular displacement
    fn integrate_quaternion(&mut self, half_angular_displacement: Vector3<f32>) {
        let q = self.quaternion.as_ref();
        let new_quaternion = q + q * Quaternion::from_parts(0.0, half_angular_displacement);
        self.quaternion = UnitQuaternion::from_quaternion(new_quaternion);
    }
}

/// Convert a rejection angle in degrees to a squared half-residual threshold
fn rejection_threshold(degrees: f32) -> f32 {
    if degrees == 0.0 {
        f32::MAX
    } else {
        (0.5 * (degrees * DEG_TO_RAD).sin()).powi(2)
    }
}

/// Residual between the sensor and reference vectors
fn residual(sensor: Vector3<f32>, reference: Vector3<f32>) -> Vector3<f32> {
    let cross = sensor.cross(&reference);

    // Error is <90 degrees
    if sensor.dot(&reference) > 0.0 {
        return cross;
    }

    // safe_normalize returns zero when sensor and reference are exactly opposite
    cross.safe_normalize()
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
        assert!(ahrs.flags().startup);
    }

    #[test]
    fn test_ahrs_startup() {
        let mut ahrs = Ahrs::new();

        // Should start in startup
        assert!(ahrs.flags().startup);

        // Update for startup period to complete ramping
        let gyro = Vector3::zeros();
        let accel = Vector3::new(0.0, 0.0, 1.0);
        let mag = Vector3::new(1.0, 0.0, 0.0);

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
        assert!((gravity.magnitude() - 1.0).abs() < 1e-6);
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
        let normal_gyro = Vector3::zeros();
        let accel = Vector3::new(0.0, 0.0, 1.0);
        let mag = Vector3::new(1.0, 0.0, 0.0);

        for _ in 0..400 {
            ahrs.update(normal_gyro, accel, mag);
        }
        assert!(!ahrs.flags().startup);

        // Now test overrange
        let overflow_gyro = Vector3::new(600.0, 0.0, 0.0); // Exceeds 500 deg/s
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
        let gyro = Vector3::zeros();
        let normal_accel = Vector3::new(0.0, 0.0, 1.0); // Normal gravity
        let mag = Vector3::new(1.0, 0.0, 0.0);

        for _ in 0..400 {
            ahrs.update(gyro, normal_accel, mag);
        }

        // Test that normal acceleration is accepted
        let states = ahrs.internal_states();
        assert!(!states.accelerometer_ignored);

        // Apply large acceleration (should be rejected after enough samples)
        let large_accel = Vector3::new(2.0, 2.0, 1.0); // Large acceleration indicating motion

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
        let tilted = Vector3::new(0.0, 1.0, 0.0);
        for _ in 0..100 {
            ahrs.update_no_magnetometer(Vector3::zeros(), tilted);
        }
        let skipped = ahrs.gravity();

        let mut ahrs = Ahrs::new();
        for _ in 0..100 {
            ahrs.update_no_magnetometer(Vector3::zeros(), tilted);
        }
        // Startup gain converges faster than the configured gain
        assert!(ahrs.gravity().y > skipped.y);
    }

    #[test]
    fn test_sample_period() {
        let gyro = Vector3::new(0.0, 0.0, 90.0);

        let mut ahrs = Ahrs::new(); // 100 Hz
        ahrs.skip_startup();
        ahrs.update(gyro, Vector3::zeros(), Vector3::zeros());
        let (_, _, yaw_default) = ahrs.quaternion().euler_angles();

        let mut ahrs = Ahrs::new();
        ahrs.skip_startup();
        ahrs.set_sample_period(0.02);
        ahrs.update(gyro, Vector3::zeros(), Vector3::zeros());
        let (_, _, yaw_doubled) = ahrs.quaternion().euler_angles();

        assert!((yaw_default.to_degrees() - 0.9).abs() < 1e-3);
        assert!((yaw_doubled.to_degrees() - 1.8).abs() < 1e-3);

        // set_settings resets the sample period
        ahrs.set_settings(AhrsSettings {
            sample_rate: 50.0,
            ..Default::default()
        });
        assert_eq!(ahrs.sample_period, 0.02);
    }

    #[test]
    fn test_overrange_preserves_outputs() {
        let settings = AhrsSettings {
            gyroscope_range: 500.0,
            ..Default::default()
        };
        let mut ahrs = Ahrs::with_settings(settings);
        let accel = Vector3::new(0.0, 0.5, 1.0);
        ahrs.update_no_magnetometer(Vector3::zeros(), accel);

        // Soft restart keeps the quaternion rather than resetting to identity,
        // and linear acceleration still reflects the latest accelerometer
        ahrs.update_no_magnetometer(Vector3::new(600.0, 0.0, 0.0), accel);
        assert!(ahrs.flags().overrange_recovery);
        assert!(ahrs.flags().startup);
        assert_ne!(ahrs.quaternion(), UnitQuaternion::identity());
        assert_eq!(ahrs.linear_acceleration(), accel - ahrs.gravity());
    }

    #[test]
    fn test_residual_opposite_vectors() {
        let sensor = Vector3::new(0.0, 0.0, 1.0);
        let reference = Vector3::new(0.0, 0.0, -0.5);
        let r = residual(sensor, reference);
        assert_eq!(r, Vector3::zeros());

        // Orthogonal vectors are normalised
        let r = residual(Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.5, 0.0));
        assert!((r.magnitude() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_upside_down_start_no_nan() {
        let mut ahrs = Ahrs::new();
        ahrs.update_no_magnetometer(Vector3::zeros(), Vector3::new(0.0, 0.0, -1.0));
        let q = ahrs.quaternion();
        assert!(q.w.is_finite() && q.i.is_finite() && q.j.is_finite() && q.k.is_finite());
    }

    #[test]
    fn test_default_rejection_disabled() {
        let mut ahrs = Ahrs::new();
        ahrs.skip_startup();
        for _ in 0..1000 {
            ahrs.update_no_magnetometer(Vector3::zeros(), Vector3::new(1.0, 1.0, 0.0));
            assert!(!ahrs.internal_states().accelerometer_ignored);
        }
        assert_eq!(ahrs.internal_states().acceleration_recovery_trigger, 0.0);
    }
}
