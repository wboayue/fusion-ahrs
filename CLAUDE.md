## Objective

Port a C sensor fusion library (Fusion AHRS) to Rust, maintaining performance while following Rust best practices.

## Core Requirements

### Architecture
- **Main Library**: Implement AHRS (Attitude and Heading Reference System) for IMU sensor fusion
- **Input**: Gyroscope, accelerometer, magnetometer data as `nalgebra::Vector3<f32>`
- **Output**: Orientation as `nalgebra::UnitQuaternion<f32>`
- **Compatibility**: `#![no_std]` for embedded systems

### Key Components to Implement
1. **AhrsSettings** - Algorithm configuration (gain, thresholds, conventions)
2. **Ahrs** - Main algorithm struct with `update()` method
3. **Offset** - Gyroscope offset correction for temperature drift
4. **Calibration** - Sensor calibration functions
5. **Internal States & Flags** - Algorithm diagnostics

### Code Quality Standards
- **Modularity**: Single-responsibility, minimal public APIs
- **Performance**: Zero-cost abstractions, minimal allocations
- **Testability**: Comprehensive unit tests with provided test data
- **Documentation**: Rustdoc for all public APIs with examples

### Dependencies
- `nalgebra` for all vector/matrix operations (no-std compatible)
- `plotters` for visualizations
- C reference implementation available in `fusion-c/` directory

### Algorithm Features
- Complementary filter combining high-pass gyroscope + low-pass accel/mag
- Acceleration/magnetic rejection for motion artifacts
- Automatic initialization and recovery modes
- Support for NWU, ENU, NED coordinate conventions

## Test Data
Test sensor data is available in `testdata/sensor_data.csv` with columns:
- Time (s)
- Gyroscope X, Y, Z (deg/s)  
- Accelerometer X, Y, Z (g)
- Magnetometer X, Y, Z (µT)

## Development Guidelines
- Follow the C implementation's algorithm behavior exactly
- Use nalgebra types consistently (`Vector3`, `UnitQuaternion`, `Matrix3`)
- Maintain embedded compatibility (`cargo build --no-default-features`)
- Include comprehensive examples demonstrating real-world usage

## Success Criteria
- Matches C library performance benchmarks
- Passes all test cases with provided sensor data
- Clear documentation with practical examples
- Modular, testable codebase following Rust idioms