#![no_std]
#![cfg_attr(docsrs, feature(doc_cfg))]

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
//! - Gyroscope bias (offset) correction for temperature drift
//! - Support for multiple Earth coordinate conventions (NWU, ENU, NED)
//! - `#![no_std]` compatible for embedded systems
//!
//! # Quick Start
//!
//! ```rust
//! use fusion_ahrs::{Ahrs, Vector};
//!
//! // Default settings: 100 Hz sample rate
//! let mut ahrs = Ahrs::new();
//!
//! // Sensor readings
//! let gyroscope = Vector::new(0.1, 0.2, 0.3);      // deg/s
//! let accelerometer = Vector::new(0.0, 0.0, 1.0);  // g
//! let magnetometer = Vector::new(1.0, 0.0, 0.0);   // µT
//!
//! // Update AHRS once per sample
//! ahrs.update(gyroscope, accelerometer, magnetometer);
//!
//! // Get orientation
//! let quaternion = ahrs.quaternion();
//!
//! // Convert to Euler angles in degrees
//! let euler = quaternion.to_euler();
//! println!("roll {:.1}°, pitch {:.1}°, yaw {:.1}°", euler.roll, euler.pitch, euler.yaw);
//! ```
//!
//! For more documentation and examples, see: <https://github.com/wboayue/fusion-ahrs>

mod ahrs;
mod bias;
mod calibration;
mod compass;
pub mod interop;
mod math;
mod remap;
mod types;

// All items are exported from the crate root
pub use ahrs::Ahrs;
pub use bias::Bias;
pub use calibration::{calibrate_inertial, calibrate_magnetic};
pub use compass::calculate_heading;
pub use math::{DEG_TO_RAD, Euler, Matrix, Quaternion, RAD_TO_DEG, Vector};
pub use remap::{RemapAlignment, remap};
pub use types::*;
