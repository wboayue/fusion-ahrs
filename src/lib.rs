#![no_std]

//! Fusion AHRS - A sensor fusion library for attitude and heading reference systems
//!
//! This library provides a complete implementation of an AHRS algorithm that fuses
//! gyroscope, accelerometer, and magnetometer data to estimate device orientation.
//! It features automatic sensor rejection during motion/interference and recovery
//! mechanisms for robust operation.
//!
//! # Features
//!
//! - Complementary filter with sensor fusion
//! - Automatic accelerometer rejection during motion
//! - Automatic magnetometer rejection during magnetic interference  
//! - Gyroscope offset correction for temperature drift
//! - Support for multiple Earth coordinate conventions (NWU, ENU, NED)
//! - `#![no_std]` compatible for embedded systems
//!
//! # Quick Start
//!
//! ```rust
//! use nalgebra::Vector3;
//! use fusion_ahrs::{Ahrs, AhrsSettings};
//!
//! let mut ahrs = Ahrs::new();
//!
//! // Sensor readings
//! let gyroscope = Vector3::new(0.1, 0.2, 0.3);      // deg/s
//! let accelerometer = Vector3::new(0.0, 0.0, 1.0);  // g
//! let magnetometer = Vector3::new(1.0, 0.0, 0.0);   // µT
//!
//! // Update AHRS
//! ahrs.update(gyroscope, accelerometer, magnetometer, 0.01); // 10ms
//!
//! // Get orientation
//! let quaternion = ahrs.quaternion();
//! ```

mod ahrs;
pub mod calibration;
pub mod compass;
mod math;
pub mod offset;
mod types;

// Re-export all public types and functions
pub use ahrs::Ahrs;
pub use calibration::{calibrate_inertial, calibrate_magnetic};
pub use compass::calculate_heading;
pub use math::{DEG_TO_RAD, QuaternionExt, RAD_TO_DEG, Vector3Ext};
pub use offset::Offset;
pub use types::*;
