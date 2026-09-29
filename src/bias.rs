//! Gyroscope bias (offset) correction, mirroring the C library's `FusionBias`

use crate::math::Vector;
use crate::types::BiasSettings;

/// Gyroscope bias correction
///
/// Provides runtime estimation and correction of the gyroscope offset,
/// which drifts with temperature. The offset is estimated with a low-pass
/// filter while the sensor is stationary for longer than the stationary
/// period.
///
/// # Example
/// ```
/// use fusion_ahrs::{Bias, Vector};
///
/// let mut bias = Bias::new(); // default settings, 100 Hz
///
/// let corrected = bias.update(Vector::new(0.1, -0.05, 0.02));
/// let offset = bias.offset();
/// ```
#[derive(Debug, Clone, Copy)]
pub struct Bias {
    /// Settings as provided by the user
    settings: BiasSettings,
    /// Filter coefficient for offset estimation
    filter_coefficient: f32,
    /// Stationary period in samples
    timeout: u32,
    /// Samples the gyroscope has been stationary
    timer: u32,
    /// Estimated gyroscope offset
    offset: Vector,
}

impl Bias {
    /// Creates bias correction with default settings
    /// ([`BiasSettings::default`]: 100 Hz, 3 deg/s threshold, 3 s period).
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::{Bias, Vector};
    ///
    /// let bias = Bias::new();
    /// assert_eq!(bias.offset(), Vector::ZERO);
    /// ```
    pub fn new() -> Self {
        Self::with_settings(BiasSettings::default())
    }

    /// Creates bias correction with the given settings.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::{Bias, BiasSettings};
    ///
    /// let bias = Bias::with_settings(BiasSettings {
    ///     sample_rate: 400.0,
    ///     ..Default::default()
    /// });
    /// assert_eq!(bias.settings().sample_rate, 400.0);
    /// ```
    pub fn with_settings(settings: BiasSettings) -> Self {
        let mut bias = Self {
            settings,
            filter_coefficient: 0.0,
            timeout: 0,
            timer: 0,
            offset: Vector::ZERO,
        };
        bias.set_settings(settings);
        bias
    }

    /// Updates the settings, keeping the current offset estimate.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::{Bias, BiasSettings};
    ///
    /// let mut bias = Bias::new();
    /// let mut settings = bias.settings();
    /// settings.stationary_period = 5.0;
    /// bias.set_settings(settings);
    /// assert_eq!(bias.settings().stationary_period, 5.0);
    /// ```
    pub fn set_settings(&mut self, settings: BiasSettings) {
        self.settings = settings;
        // C: 2π × fc × (1 / fs)
        self.filter_coefficient =
            2.0 * core::f32::consts::PI * settings.cutoff_frequency * (1.0 / settings.sample_rate);
        self.timeout = (settings.stationary_period * settings.sample_rate) as u32;
    }

    /// Returns the current settings.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::Bias;
    ///
    /// assert_eq!(Bias::new().settings().stationary_threshold, 3.0);
    /// ```
    pub fn settings(&self) -> BiasSettings {
        self.settings
    }

    /// Updates the offset estimate and returns the corrected gyroscope.
    ///
    /// The current offset is subtracted from the reading. If every axis of
    /// the corrected reading stays within the stationary threshold for the
    /// stationary period, the offset estimate starts tracking it through a
    /// low-pass filter.
    ///
    /// # Arguments
    /// * `gyroscope` - Gyroscope reading in degrees per second
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::{Bias, Vector};
    ///
    /// let mut bias = Bias::new();
    /// let corrected = bias.update([0.1, 0.2, 0.3]);
    /// assert_eq!(corrected, Vector::new(0.1, 0.2, 0.3)); // no offset yet
    /// ```
    pub fn update(&mut self, gyroscope: impl Into<Vector>) -> Vector {
        let gyroscope = gyroscope.into() - self.offset;

        // Reset timer if gyroscope not stationary
        let threshold = self.settings.stationary_threshold;
        if libm::fabsf(gyroscope.x) > threshold
            || libm::fabsf(gyroscope.y) > threshold
            || libm::fabsf(gyroscope.z) > threshold
        {
            self.timer = 0;
            return gyroscope;
        }

        // Increment timer while gyroscope stationary
        if self.timer < self.timeout {
            self.timer += 1;
            return gyroscope;
        }

        // Update low-pass filter while timer has elapsed
        self.offset += gyroscope * self.filter_coefficient;
        gyroscope
    }

    /// Returns the estimated gyroscope offset in degrees per second.
    pub fn offset(&self) -> Vector {
        self.offset
    }

    /// Sets the offset estimate, for example to restore a value saved from a
    /// previous run so correction starts immediately.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::{Bias, Vector};
    ///
    /// let mut bias = Bias::new();
    /// bias.set_offset([0.5, -0.2, 0.1]);
    /// assert_eq!(bias.update([0.5, -0.2, 0.1]), Vector::ZERO);
    /// ```
    pub fn set_offset(&mut self, offset: impl Into<Vector>) {
        self.offset = offset.into();
    }

    /// Restarts estimation: clears the offset and the stationary timer,
    /// keeping the settings.
    ///
    /// # Example
    /// ```
    /// use fusion_ahrs::{Bias, Vector};
    ///
    /// let mut bias = Bias::new();
    /// bias.set_offset([1.0, 0.0, 0.0]);
    /// bias.restart();
    /// assert_eq!(bias.offset(), Vector::ZERO);
    /// ```
    pub fn restart(&mut self) {
        self.timer = 0;
        self.offset = Vector::ZERO;
    }

    /// Returns true once the gyroscope has been stationary for the
    /// stationary period, meaning the offset estimate is being updated.
    pub fn is_active(&self) -> bool {
        self.timer >= self.timeout
    }
}

impl Default for Bias {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn with_sample_rate(sample_rate: f32) -> Bias {
        Bias::with_settings(BiasSettings {
            sample_rate,
            ..Default::default()
        })
    }

    #[test]
    fn test_initialization() {
        let bias = Bias::new();
        let settings = bias.settings();

        assert_eq!(bias.offset(), Vector::ZERO);
        assert!(!bias.is_active());
        assert_eq!(bias.timer, 0);
        assert_eq!(
            bias.timeout,
            (settings.stationary_period * settings.sample_rate) as u32
        );
    }

    #[test]
    fn test_motion_detection() {
        let mut bias = Bias::new();

        // Stationary reading (all axes below 3 deg/s) increments the timer
        let stationary = Vector::new(2.0, 1.0, 1.5);
        assert_eq!(bias.update(stationary), stationary); // no offset yet
        assert_eq!(bias.timer, 1);

        // Motion resets the timer
        bias.update(Vector::new(5.0, 0.0, 0.0));
        assert_eq!(bias.timer, 0);
        assert!(!bias.is_active());
    }

    #[test]
    fn test_stationary_period() {
        let mut bias = with_sample_rate(10.0);
        let stationary = Vector::new(0.1, 0.1, 0.1);

        for i in 0..bias.timeout {
            assert_eq!(bias.update(stationary), stationary);
            assert_eq!(bias.timer, i + 1);
            if i + 1 < bias.timeout {
                assert!(!bias.is_active());
            }
        }

        // One more update starts offset estimation
        bias.update(stationary);
        assert!(bias.is_active());
        assert!(bias.offset().norm() > 0.0);
    }

    #[test]
    fn test_estimation_convergence() {
        let mut bias = Bias::new();
        let true_bias = Vector::new(0.5, -0.3, 0.2);

        for _ in 0..bias.timeout {
            bias.update(true_bias);
        }
        assert!(bias.is_active());

        // The filter is slow (0.02 Hz cutoff), so it needs many samples
        for _ in 0..5000 {
            bias.update(true_bias);
        }

        let error = (bias.offset() - true_bias).norm();
        assert!(error < true_bias.norm() * 0.5);
    }

    #[test]
    fn test_correction_application() {
        let mut bias = Bias::new();
        for _ in 0..500 {
            bias.update(Vector::new(1.0, 2.0, 3.0));
        }

        let raw = Vector::new(5.0, 6.0, 7.0);
        let corrected = bias.update(raw);
        assert!((corrected - (raw - bias.offset())).norm() < 1e-6);
    }

    #[test]
    fn test_restart() {
        let mut bias = Bias::new();
        for _ in 0..100 {
            bias.update(Vector::new(0.1, 0.1, 0.1));
        }
        assert!(bias.timer > 0);

        bias.restart();
        assert_eq!(bias.timer, 0);
        assert_eq!(bias.offset(), Vector::ZERO);
        assert!(!bias.is_active());
    }

    #[test]
    fn test_set_settings_keeps_offset() {
        let mut bias = Bias::new();
        bias.set_offset([0.5, 0.0, 0.0]);
        bias.set_settings(BiasSettings {
            sample_rate: 200.0,
            ..Default::default()
        });
        assert_eq!(bias.offset(), Vector::new(0.5, 0.0, 0.0));
        assert_eq!(bias.timeout, 600);
    }

    #[test]
    fn test_filter_coefficient() {
        for sample_rate in [10.0, 50.0, 100.0, 500.0, 1000.0] {
            let bias = with_sample_rate(sample_rate);
            let expected = 2.0 * core::f32::consts::PI * 0.02 / sample_rate;
            assert!((bias.filter_coefficient - expected).abs() < 1e-6);
            assert!(bias.filter_coefficient > 0.0 && bias.filter_coefficient < 1.0);
        }
    }

    #[test]
    fn test_threshold_is_exclusive() {
        let mut bias = Bias::new();
        let threshold = bias.settings().stationary_threshold;

        bias.update(Vector::new(threshold - 0.1, 0.0, 0.0));
        assert_eq!(bias.timer, 1);

        bias.update(Vector::new(threshold + 0.1, 0.0, 0.0));
        assert_eq!(bias.timer, 0);

        // Exactly at the threshold counts as stationary (> not >=)
        bias.update(Vector::new(threshold - 0.1, 0.0, 0.0));
        bias.update(Vector::new(threshold, 0.0, 0.0));
        assert_eq!(bias.timer, 2);
    }
}
