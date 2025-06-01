# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Rust port of the Fusion AHRS (Attitude and Heading Reference System) library, originally written in C by xioTechnologies. The library provides sensor fusion for Inertial Measurement Units (IMUs), combining gyroscope, accelerometer, and magnetometer data to calculate orientation relative to Earth.

### Key Architecture

- **Core Library**: `src/lib.rs` contains the main `FusionAhrs` struct and implementation
- **C Reference**: The `fusion_c/` directory contains the original C implementation for reference
- **Examples**: `examples/` directory demonstrates usage patterns
- **Dependencies**: Uses `nalgebra` for all vector/matrix operations and quaternion math

### Current Implementation Status

The Rust implementation is in early development:
- Basic `FusionAhrs` struct exists with placeholder methods
- Uses `nalgebra` types (`Vector3<f32>`, `UnitQuaternion<f32>`)
- Implements `#![no_std]` for embedded compatibility
- The actual AHRS algorithm logic from the C implementation needs to be ported

## Development Commands

### Build and Test
```bash
# Build the library
cargo build

# Run tests
cargo test

# Build for no-std (embedded)
cargo build --no-default-features

# Run examples
cargo run --example simple
cargo run --example advanced
```

### Code Quality
```bash
# Format code
cargo fmt

# Run clippy for lints
cargo clippy

# Check without building
cargo check
```

## Implementation Guidelines

### Sensor Data Flow
1. Raw sensor data comes in as `Vector3<f32>` (gyroscope, accelerometer, magnetometer)
2. Data flows through the AHRS algorithm (to be implemented)
3. Output is `UnitQuaternion<f32>` representing orientation
4. Quaternions can be converted to Euler angles or rotation matrices using nalgebra

### Key Types to Implement
Based on the C reference and README, these core types need Rust equivalents:
- `AhrsSettings` - algorithm configuration
- `AhrsInternalStates` - internal algorithm state
- `AhrsFlags` - algorithm status flags
- `Offset` and `OffsetSettings` - gyroscope offset correction
- Calibration functions for sensors

### nalgebra Integration
- All vector operations use `nalgebra::Vector3<f32>`
- Quaternions use `nalgebra::UnitQuaternion<f32>`
- Matrix operations use `nalgebra::Matrix3<f32>`
- Follow nalgebra conventions for method names and patterns

### No-std Compatibility
- Keep `#![no_std]` directive
- Avoid standard library dependencies
- Use `nalgebra`'s no-std features
- Test embedded compatibility regularly

## C Reference Implementation

The `fusion_c/` directory contains the original C implementation that serves as the specification:
- `FusionAhrs.c/h` - Main AHRS algorithm
- `FusionOffset.c/h` - Gyroscope offset correction
- `FusionCalibration.h` - Sensor calibration functions
- Examples demonstrate complete usage patterns

When implementing Rust equivalents, maintain the same algorithm behavior while adapting to Rust idioms and nalgebra types.