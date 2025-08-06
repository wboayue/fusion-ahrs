//! Main AHRS algorithm implementation for the Fusion AHRS library

use crate::math::{DEG_TO_RAD, Vector3Ext};
use crate::types::{AhrsFlags, AhrsInternalStates, AhrsSettings, Convention};
use nalgebra::{ComplexField, Quaternion, UnitQuaternion, Vector3};

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
    ///
    /// This creates an AHRS algorithm with the default settings:
    /// - Convention: NWU (North-West-Up)
    /// - Gain: 0.5
    /// - Gyroscope range: 2000 deg/s
    /// - Acceleration rejection: 10°
    /// - Magnetic rejection: 20°
    /// - Recovery trigger period: 2560 samples (5s at 512Hz)
    ///
    /// The algorithm will start in initialization mode with ramped gain.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new();
    /// assert!(ahrs.flags().initialising);
    /// ```
    pub fn new() -> Self {
        Self::with_settings(AhrsSettings::default())
    }

    /// Create a new AHRS instance with specified settings
    ///
    /// This allows customization of all algorithm parameters including
    /// coordinate convention, gain, gyroscope range, and rejection thresholds.
    ///
    /// # Arguments
    /// * `settings` - Configuration for the AHRS algorithm
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::{Ahrs, AhrsSettings, Convention};
    ///
    /// let settings = AhrsSettings {
    ///     convention: Convention::Enu,
    ///     gain: 0.75,
    ///     gyroscope_range: 1000.0,
    ///     acceleration_rejection: 15.0,
    ///     magnetic_rejection: 25.0,
    ///     recovery_trigger_period: 1024,
    /// };
    ///
    /// let mut ahrs = Ahrs::with_settings(settings);
    /// assert_eq!(ahrs.get_settings().gain, 0.75);
    /// ```
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
    ///
    /// Resets the algorithm to its initial state:
    /// - Sets quaternion to identity (no rotation)
    /// - Clears all internal state variables
    /// - Enters initialization mode with ramped gain
    /// - Resets all recovery mechanisms
    ///
    /// This is automatically called during construction and can be used
    /// to restart the algorithm if needed.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new();
    /// // ... use the AHRS ...
    /// ahrs.initialise(); // Reset to initial state
    /// assert!(ahrs.flags().initialising);
    /// ```
    pub fn initialise(&mut self) {
        self.quaternion = UnitQuaternion::identity();
        self.accelerometer = Vector3::zeros();
        self.initialising = true;
        self.ramped_gain = INITIAL_GAIN;
        self.ramped_gain_step = (INITIAL_GAIN - self.settings.gain) / INITIALISATION_PERIOD;
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
    ///
    /// Convenience method that calls `initialise()`. Useful for
    /// applications that prefer the term "reset" over "initialise".
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new();
    /// ahrs.reset(); // Same as ahrs.initialise()
    /// ```
    pub fn reset(&mut self) {
        self.initialise();
    }

    /// Update algorithm settings
    ///
    /// Changes the algorithm configuration and recalculates derived values.
    /// If not currently initializing, the gain is updated immediately.
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
        self.process_settings();
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

    /// Update AHRS with gyroscope, accelerometer, and magnetometer data
    ///
    /// This is the main algorithm function that fuses all sensor readings
    /// to estimate orientation. The algorithm automatically:
    /// - Detects and rejects accelerometer readings during motion
    /// - Detects and rejects magnetometer readings during magnetic interference
    /// - Manages initialization with ramped gain
    /// - Handles gyroscope overflow detection and recovery
    ///
    /// # Arguments
    /// * `gyroscope` - Gyroscope reading in degrees per second
    /// * `accelerometer` - Accelerometer reading in g (normalized gravity)
    /// * `magnetometer` - Magnetometer reading in µT (any units, will be normalized)
    /// * `delta_time` - Time step in seconds since last update
    ///
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use fusion_ahrs::Ahrs;
    ///
    /// let mut ahrs = Ahrs::new();
    ///
    /// // Typical sensor readings
    /// let gyro = Vector3::new(0.1, -0.2, 0.05);     // Small rotation rates
    /// let accel = Vector3::new(0.0, 0.0, 1.0);      // Gravity pointing up (NWU)
    /// let mag = Vector3::new(25.0, 2.0, -15.0);     // Earth's magnetic field
    ///
    /// ahrs.update(gyro, accel, mag, 0.01);  // 10ms time step (100Hz)
    ///
    /// let orientation = ahrs.quaternion();
    /// let gravity = ahrs.gravity();
    /// ```
    pub fn update(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        magnetometer: Vector3<f32>,
        delta_time: f32,
    ) {
        // Store accelerometer for linear acceleration calculation
        self.accelerometer = accelerometer;

        // Reinitialise if gyroscope range exceeded
        if (gyroscope.x.abs() > self.gyroscope_range_threshold)
            || (gyroscope.y.abs() > self.gyroscope_range_threshold)
            || (gyroscope.z.abs() > self.gyroscope_range_threshold)
        {
            let quaternion = self.quaternion;
            self.initialise();
            self.quaternion = quaternion;
            self.angular_rate_recovery = true;
        }

        // Ramp down gain during initialization - match C implementation exactly
        if self.initialising {
            self.ramped_gain -= self.ramped_gain_step * delta_time;
            if (self.ramped_gain < self.settings.gain) || (self.settings.gain == 0.0) {
                self.ramped_gain = self.settings.gain;
                self.initialising = false;
                self.angular_rate_recovery = false;
            }
        }

        // Calculate gravity direction in sensor frame
        let half_gravity = self.calculate_half_gravity();

        // Calculate accelerometer feedback
        let mut half_accelerometer_feedback = Vector3::zeros();
        self.accelerometer_ignored = true;
        if accelerometer.magnitude() > 0.0 {
            let accelerometer_normalized = accelerometer.safe_normalize();

            // Calculate accelerometer feedback scaled by 0.5
            self.half_accelerometer_feedback =
                self.calculate_feedback(accelerometer_normalized, half_gravity);

            // Don't ignore accelerometer if acceleration error below threshold
            if self.initialising
                || (self.half_accelerometer_feedback.magnitude_squared()
                    <= self.acceleration_rejection_squared)
            {
                self.accelerometer_ignored = false;
                self.acceleration_recovery_trigger -= RECOVERY_DECREMENT;
            } else {
                self.acceleration_recovery_trigger += 1;
            }

            // Don't ignore accelerometer during acceleration recovery
            if self.acceleration_recovery_trigger > self.acceleration_recovery_timeout as i32 {
                self.acceleration_recovery_timeout = 0;
                self.accelerometer_ignored = false;
            } else {
                self.acceleration_recovery_timeout = self.settings.recovery_trigger_period;
            }

            // Clamp recovery trigger
            self.acceleration_recovery_trigger = self
                .acceleration_recovery_trigger
                .clamp(0, self.settings.recovery_trigger_period as i32);

            // Apply accelerometer feedback
            if !self.accelerometer_ignored {
                half_accelerometer_feedback = self.half_accelerometer_feedback * 0.5;
            }
        }

        // Note: half_magnetic is calculated inside magnetometer processing block

        // Calculate magnetometer feedback
        let mut half_magnetometer_feedback = Vector3::zeros();
        self.magnetometer_ignored = true;
        if magnetometer.magnitude() > 0.0 {
            // Calculate direction of magnetic field indicated by algorithm
            let half_magnetic = self.calculate_half_magnetic();

            // Calculate magnetometer feedback scaled by 0.5 with cross product preprocessing
            let cross_product = half_gravity.cross(&magnetometer);
            let magnetometer_normalized = cross_product.safe_normalize();
            self.half_magnetometer_feedback =
                self.calculate_feedback(magnetometer_normalized, half_magnetic);

            // Don't ignore magnetometer if magnetic error below threshold
            if self.initialising
                || (self.half_magnetometer_feedback.magnitude_squared()
                    <= self.magnetic_rejection_squared)
            {
                self.magnetometer_ignored = false;
                self.magnetic_recovery_trigger -= RECOVERY_DECREMENT;
            } else {
                self.magnetic_recovery_trigger += 1;
            }

            // Don't ignore magnetometer during magnetic recovery
            if self.magnetic_recovery_trigger > self.magnetic_recovery_timeout as i32 {
                self.magnetic_recovery_timeout = 0;
                self.magnetometer_ignored = false;
            } else {
                self.magnetic_recovery_timeout = self.settings.recovery_trigger_period;
            }

            // Clamp recovery trigger
            self.magnetic_recovery_trigger = self
                .magnetic_recovery_trigger
                .clamp(0, self.settings.recovery_trigger_period as i32);

            // Apply magnetometer feedback
            if !self.magnetometer_ignored {
                half_magnetometer_feedback = self.half_magnetometer_feedback * 0.5;
            }
        }

        // Convert gyroscope to half-radians per second
        let half_gyroscope = gyroscope * (DEG_TO_RAD * 0.5);

        // Apply feedback to gyroscope
        let adjusted_half_gyroscope = half_gyroscope
            + (half_accelerometer_feedback + half_magnetometer_feedback) * self.ramped_gain;

        // Integrate quaternion
        self.integrate_quaternion(adjusted_half_gyroscope, delta_time);
    }

    /// Update AHRS without magnetometer (gyroscope and accelerometer only)
    ///
    /// Use this when magnetometer data is unavailable or unreliable.
    /// The algorithm will still estimate roll and pitch from the accelerometer
    /// but heading will drift over time. During initialization, heading is
    /// automatically zeroed to prevent drift.
    ///
    /// # Arguments
    /// * `gyroscope` - Gyroscope reading in degrees per second
    /// * `accelerometer` - Accelerometer reading in g
    /// * `delta_time` - Time step in seconds since last update
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
    /// ahrs.update_no_magnetometer(gyro, accel, 0.01);
    ///
    /// // Roll and pitch will be accurate, heading may drift
    /// let euler = ahrs.quaternion().euler_angles();
    /// ```
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
    ///
    /// Use this when you have an external heading reference (GPS, compass,
    /// etc.) instead of a magnetometer. The function synthesizes a virtual
    /// magnetometer reading from the provided heading angle.
    ///
    /// # Arguments
    /// * `gyroscope` - Gyroscope reading in degrees per second
    /// * `accelerometer` - Accelerometer reading in g
    /// * `heading` - Heading angle in degrees (0° = North, positive = clockwise)
    /// * `delta_time` - Time step in seconds since last update
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
    /// ahrs.update_external_heading(gyro, accel, heading_from_gps, 0.01);
    /// ```
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
    /// ahrs.update(Vector3::zeros(), accel_with_motion, Vector3::zeros(), 0.01);
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
    /// ahrs.update(
    ///     Vector3::zeros(),
    ///     Vector3::new(0.5, 0.0, 1.0),
    ///     Vector3::zeros(),
    ///     0.01
    /// );
    ///
    /// let earth_accel = ahrs.earth_acceleration();
    /// // Acceleration now expressed in Earth coordinates
    /// ```
    pub fn earth_acceleration(&self) -> Vector3<f32> {
        self.quaternion * self.linear_acceleration()
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
    ///
    /// Returns status flags indicating the current operating mode of the
    /// algorithm, including initialization and recovery states.
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
    /// if flags.initialising {
    ///     println!("Algorithm is still initializing");
    /// }
    /// if flags.angular_rate_recovery {
    ///     println!("Recovering from gyroscope overflow");
    /// }
    /// ```
    pub fn flags(&self) -> AhrsFlags {
        AhrsFlags {
            initialising: self.initialising,
            angular_rate_recovery: self.angular_rate_recovery,
            acceleration_recovery: self.acceleration_recovery_trigger > 0,
            magnetic_recovery: self.magnetic_recovery_trigger > 0,
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
        let heading_rad = heading * DEG_TO_RAD;

        // Extract current roll and pitch
        let (roll, pitch, _) = self.quaternion.euler_angles();

        // Create new quaternion with same roll/pitch but new heading
        self.quaternion = UnitQuaternion::from_euler_angles(roll, pitch, heading_rad);
    }

    // Private helper methods

    /// Process settings and calculate derived values
    fn process_settings(&mut self) {
        // Process gyroscope range - match C implementation exactly
        self.gyroscope_range_threshold = if self.settings.gyroscope_range == 0.0 {
            f32::MAX
        } else {
            self.settings.gyroscope_range * GYROSCOPE_RANGE_FACTOR
        };

        // Process rejection thresholds (convert to squared for efficiency) - match C implementation
        self.acceleration_rejection_squared = if self.settings.acceleration_rejection == 0.0 {
            f32::MAX
        } else {
            let accel_rad = self.settings.acceleration_rejection * DEG_TO_RAD;
            (0.5 * accel_rad.sin()).powi(2)
        };

        self.magnetic_rejection_squared = if self.settings.magnetic_rejection == 0.0 {
            f32::MAX
        } else {
            let mag_rad = self.settings.magnetic_rejection * DEG_TO_RAD;
            (0.5 * mag_rad.sin()).powi(2)
        };

        // Disable rejection features if gain is zero or recovery trigger period is zero
        if self.settings.gain == 0.0 || self.settings.recovery_trigger_period == 0 {
            self.acceleration_rejection_squared = f32::MAX;
            self.magnetic_rejection_squared = f32::MAX;
        }

        // Set recovery timeouts
        self.acceleration_recovery_timeout = self.settings.recovery_trigger_period;
        self.magnetic_recovery_timeout = self.settings.recovery_trigger_period;

        // Set ramped gain step if not initializing
        if !self.initialising {
            self.ramped_gain = self.settings.gain;
        }
        self.ramped_gain_step = (INITIAL_GAIN - self.settings.gain) / INITIALISATION_PERIOD;
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
                    qy * qz - qw * qx,
                )
            }
            Convention::Enu => {
                // First column of rotation matrix * -0.5
                Vector3::new(
                    -(0.5 - qy * qy - qz * qz),
                    -(qx * qy - qw * qz),
                    -(qx * qz + qw * qy),
                )
            }
            Convention::Ned => {
                // Second column of rotation matrix * -0.5
                Vector3::new(
                    -(qw * qy + qx * qz),
                    -(0.5 - qx * qx - qz * qz),
                    -(qy * qz - qw * qx),
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
        let gyro_quat = Quaternion::from_parts(0.0, half_gyroscope);

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
        let settings = AhrsSettings {
            acceleration_rejection: 10.0, // 10 degree threshold
            recovery_trigger_period: 100,
            ..Default::default()
        };

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
        for _i in 0..150 {
            ahrs.update(gyro, large_accel, mag, 0.01);
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
}
