#![no_std]

//! [![github]](https://github.com/wboayue/fusion-ahrs)&ensp;[![crates-io]](https://crates.io/crates/fusion-ahrs)&ensp;[![license]](https://opensource.org/licenses/MIT)
//!
//! [github]: https://img.shields.io/badge/github-8da0cb?style=for-the-badge&labelColor=555555&logo=github
//! [crates-io]: https://img.shields.io/badge/crates.io-fc8d62?style=for-the-badge&labelColor=555555&logo=rust
//! [license]: https://img.shields.io/badge/License-MIT-blue.svg?style=for-the-badge&labelColor=555555
//!
//! Fusion AHRS - A sensor fusion library for attitude and heading reference systems
//!
//! This is a Rust port of the C library by xioTechnologies: <https://github.com/xioTechnologies/Fusion>
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
//!
//! // Convert to Euler angles (roll, pitch, yaw)
//! let (roll, pitch, yaw) = quaternion.euler_angles();
//! ```
//!
//! For more documentation and examples, see: <https://github.com/wboayue/fusion-ahrs>

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
