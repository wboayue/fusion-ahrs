//! Gyroscope offset correction for the Fusion AHRS library

use nalgebra::Vector3;
use crate::types::OffsetSettings;

/// Offset correction algorithm constants
const CUTOFF_FREQUENCY: f32 = 0.02; // Hz
const TIMEOUT_SECONDS: f32 = 5.0; // seconds
const THRESHOLD: f32 = 3.0; // deg/s

/// Gyroscope offset correction structure
/// 
/// Provides runtime estimation and correction of gyroscope bias drift,
/// typically caused by temperature changes. Uses a low-pass filter to
/// estimate the offset when the sensor is stationary for a sufficient period.
#[derive(Debug, Clone, Copy)]
pub struct Offset {
    /// Filter coefficient for offset estimation
    filter_coefficient: f32,
    /// Timeout period in samples
    timeout: u32,
    /// Current timer value (counts samples while stationary)
    timer: u32,
    /// Estimated gyroscope offset
    gyroscope_offset: Vector3<f32>,
}

impl Offset {
    /// Initialize offset correction with the given settings and sample rate
    /// 
    /// # Arguments
    /// * `settings` - Offset correction configuration
    /// * `sample_rate` - Sensor sample rate in Hz
    /// 
    /// # Example
    /// ```
    /// use fusion_ahrs::{Offset, OffsetSettings};
    /// 
    /// let settings = OffsetSettings::default();
    /// let mut offset = Offset::new(settings, 100.0); // 100 Hz sample rate
    /// ```
    pub fn new(_settings: OffsetSettings, sample_rate: f32) -> Self {
        // Calculate filter coefficient based on cutoff frequency and sample rate
        // filterCoefficient = 2π × fc / fs
        let filter_coefficient = 2.0 * core::f32::consts::PI * CUTOFF_FREQUENCY / sample_rate;
        
        // Convert timeout from seconds to sample count
        let timeout = (TIMEOUT_SECONDS * sample_rate) as u32;
        
        Self {
            filter_coefficient,
            timeout,
            timer: 0,
            gyroscope_offset: Vector3::zeros(),
        }
    }

    /// Update offset estimation and return corrected gyroscope reading
    /// 
    /// This function implements the complete offset correction algorithm:
    /// 1. Apply current offset correction to input
    /// 2. Check if corrected values indicate stationary behavior
    /// 3. Update offset estimate using low-pass filter if stationary long enough
    /// 4. Return offset-corrected gyroscope reading
    /// 
    /// # Arguments
    /// * `gyroscope` - Raw gyroscope reading in degrees per second
    /// 
    /// # Returns
    /// Offset-corrected gyroscope reading
    /// 
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use fusion_ahrs::{Offset, OffsetSettings};
    /// 
    /// let mut offset = Offset::new(OffsetSettings::default(), 100.0);
    /// let gyro_raw = Vector3::new(0.1, 0.2, 0.3);
    /// let gyro_corrected = offset.update(gyro_raw);
    /// ```
    pub fn update(&mut self, gyroscope: Vector3<f32>) -> Vector3<f32> {
        // Step 1: Apply current offset correction
        let corrected_gyroscope = gyroscope - self.gyroscope_offset;
        
        // Step 2: Check if gyroscope indicates stationary motion
        // All axes must be below threshold simultaneously
        if corrected_gyroscope.x.abs() > THRESHOLD ||
           corrected_gyroscope.y.abs() > THRESHOLD ||
           corrected_gyroscope.z.abs() > THRESHOLD {
            // Motion detected - reset timer
            self.timer = 0;
            return corrected_gyroscope;
        }
        
        // Step 3: Increment timer while stationary
        if self.timer < self.timeout {
            self.timer += 1;
            return corrected_gyroscope;
        }
        
        // Step 4: Update offset using low-pass filter
        // offset_new = offset_old + α × gyroscope_corrected
        let offset_update = corrected_gyroscope * self.filter_coefficient;
        self.gyroscope_offset += offset_update;
        
        corrected_gyroscope
    }

    /// Get current offset estimate
    /// 
    /// # Returns
    /// Current gyroscope offset estimate in degrees per second
    pub fn offset(&self) -> Vector3<f32> {
        self.gyroscope_offset
    }
    
    /// Reset the offset correction to initial state
    /// 
    /// Clears the offset estimate and timer, returning the algorithm
    /// to its initial uncalibrated state.
    pub fn reset(&mut self) {
        self.timer = 0;
        self.gyroscope_offset = Vector3::zeros();
    }
    
    /// Check if the offset correction is actively estimating
    /// 
    /// # Returns
    /// True if the timeout period has been reached and offset estimation is active
    pub fn is_active(&self) -> bool {
        self.timer >= self.timeout
    }
    
    /// Get the current timer value
    /// 
    /// # Returns
    /// Number of samples the sensor has been stationary
    pub fn timer(&self) -> u32 {
        self.timer
    }
    
    /// Get the filter coefficient
    /// 
    /// # Returns
    /// Low-pass filter coefficient used for offset updates
    pub fn filter_coefficient(&self) -> f32 {
        self.filter_coefficient
    }
    
    /// Get the timeout threshold
    /// 
    /// # Returns
    /// Number of samples required before offset estimation begins
    pub fn timeout(&self) -> u32 {
        self.timeout
    }
}

impl Default for Offset {
    fn default() -> Self {
        Self::new(OffsetSettings::default(), 100.0) // Assume 100 Hz default
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_offset_initialization() {
        let settings = OffsetSettings::default();
        let sample_rate = 100.0;
        let offset = Offset::new(settings, sample_rate);
        
        // Check initial state
        assert_eq!(offset.offset(), Vector3::zeros());
        assert!(!offset.is_active());
        assert_eq!(offset.timer(), 0);
        
        // Check calculated values
        let expected_coefficient = 2.0 * core::f32::consts::PI * CUTOFF_FREQUENCY / sample_rate;
        let expected_timeout = (TIMEOUT_SECONDS * sample_rate) as u32;
        
        assert!((offset.filter_coefficient() - expected_coefficient).abs() < 1e-6);
        assert_eq!(offset.timeout(), expected_timeout);
    }
    
    #[test]
    fn test_offset_motion_detection() {
        let mut offset = Offset::new(OffsetSettings::default(), 100.0);
        
        // Test stationary readings (below threshold)
        let stationary_reading = Vector3::new(2.0, 1.0, 1.5); // All below 3.0 deg/s
        
        // Should not be active initially
        assert!(!offset.is_active());
        assert_eq!(offset.timer(), 0);
        
        // Apply stationary reading - timer should increment
        let corrected = offset.update(stationary_reading);
        assert_eq!(offset.timer(), 1);
        assert_eq!(corrected, stationary_reading); // No offset yet
        
        // Test motion detection (above threshold)
        let motion_reading = Vector3::new(5.0, 0.0, 0.0); // X exceeds 3.0 deg/s
        offset.update(motion_reading);
        
        // Timer should reset
        assert_eq!(offset.timer(), 0);
        assert!(!offset.is_active());
    }
    
    #[test]
    fn test_offset_stationary_timeout() {
        let sample_rate = 10.0; // Use low sample rate for faster testing
        let mut offset = Offset::new(OffsetSettings::default(), sample_rate);
        
        let timeout_samples = (TIMEOUT_SECONDS * sample_rate) as u32;
        let stationary_reading = Vector3::new(0.1, 0.1, 0.1);
        
        // Apply stationary readings up to timeout (but not including)
        for i in 0..timeout_samples {
            let corrected = offset.update(stationary_reading);
            assert_eq!(offset.timer(), i + 1);
            if i + 1 < timeout_samples {
                assert!(!offset.is_active()); // Not active until timeout reached
            }
            assert_eq!(corrected, stationary_reading); // No offset correction yet
        }
        
        // One more update should activate offset estimation
        offset.update(stationary_reading);
        assert!(offset.is_active());
        
        // Offset should now be updated (but still small due to low filter coefficient)
        let estimated_offset = offset.offset();
        assert!(estimated_offset.magnitude() > 0.0);
    }
    
    #[test]
    fn test_offset_estimation_convergence() {
        let sample_rate = 100.0;
        let mut offset = Offset::new(OffsetSettings::default(), sample_rate);
        
        // Simulate constant bias
        let true_bias = Vector3::new(0.5, -0.3, 0.2);
        let timeout_samples = (TIMEOUT_SECONDS * sample_rate) as u32;
        
        // First, reach timeout with stationary readings
        for _ in 0..timeout_samples {
            offset.update(true_bias);
        }
        assert!(offset.is_active());
        
        // Continue with many stationary readings to allow convergence
        // The filter is very slow (0.02 Hz cutoff), so we need many samples
        for _ in 0..5000 {
            offset.update(true_bias);
        }
        
        // Offset should converge toward the true bias
        let estimated_offset = offset.offset();
        let error = (estimated_offset - true_bias).magnitude();
        
        // Should be reasonably close (within 50% due to very slow filter)
        // The filter is designed to be conservative and slow
        assert!(error < true_bias.magnitude() * 0.5);
    }
    
    #[test]
    fn test_offset_correction_application() {
        let mut offset = Offset::new(OffsetSettings::default(), 100.0);
        
        // Manually set an offset to test correction
        let test_offset = Vector3::new(1.0, 2.0, 3.0);
        
        // Simulate the algorithm reaching steady state with this offset
        // (In real usage, this would happen through the update process)
        for _ in 0..500 {
            offset.update(test_offset);
        }
        
        // Now test that the offset is being applied
        let raw_reading = Vector3::new(5.0, 6.0, 7.0);
        let corrected = offset.update(raw_reading);
        
        // Corrected reading should have offset subtracted
        let expected = raw_reading - offset.offset();
        let error = (corrected - expected).magnitude();
        assert!(error < 1e-6);
    }
    
    #[test]
    fn test_offset_reset() {
        let mut offset = Offset::new(OffsetSettings::default(), 100.0);
        
        // Build up some state
        let stationary_reading = Vector3::new(0.1, 0.1, 0.1);
        for _ in 0..100 {
            offset.update(stationary_reading);
        }
        
        // Verify state exists
        assert!(offset.timer() > 0);
        
        // Reset and verify clean state
        offset.reset();
        assert_eq!(offset.timer(), 0);
        assert_eq!(offset.offset(), Vector3::zeros());
        assert!(!offset.is_active());
    }
    
    #[test]
    fn test_offset_filter_coefficient_calculation() {
        let sample_rates = [10.0, 50.0, 100.0, 500.0, 1000.0];
        
        for &sample_rate in &sample_rates {
            let offset = Offset::new(OffsetSettings::default(), sample_rate);
            let expected = 2.0 * core::f32::consts::PI * CUTOFF_FREQUENCY / sample_rate;
            
            assert!((offset.filter_coefficient() - expected).abs() < 1e-6);
            
            // Coefficient should be smaller for higher sample rates
            assert!(offset.filter_coefficient() > 0.0);
            assert!(offset.filter_coefficient() < 1.0); // Should be a fraction
        }
    }
    
    #[test]
    fn test_offset_threshold_behavior() {
        let mut offset = Offset::new(OffsetSettings::default(), 100.0);
        
        // Test readings right at threshold boundary
        let at_threshold = Vector3::new(THRESHOLD, 0.0, 0.0);
        let below_threshold = Vector3::new(THRESHOLD - 0.1, 0.0, 0.0);
        let above_threshold = Vector3::new(THRESHOLD + 0.1, 0.0, 0.0);
        
        // Below threshold should increment timer
        offset.update(below_threshold);
        assert_eq!(offset.timer(), 1);
        
        // Above threshold should reset timer
        offset.update(above_threshold);
        assert_eq!(offset.timer(), 0);
        
        // At threshold should NOT reset timer (threshold is exclusive, > not >=)
        offset.update(below_threshold);
        assert_eq!(offset.timer(), 1);
        offset.update(at_threshold);
        assert_eq!(offset.timer(), 2); // Should increment, not reset
    }
}