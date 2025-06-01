# Fusion AHRS

Fusion AHRS is a sensor fusion library for Inertial Measurement Units (IMUs), optimised for embedded systems. This is a Rust port of the original C library, providing memory safety and zero-cost abstractions while maintaining the same performance characteristics. The library is available on [crates.io](https://crates.io/crates/fusion-ahrs) and includes comprehensive examples demonstrating usage with sample sensor data.

## Features

- **Memory Safe**: Written in Rust with compile-time safety guarantees
- **No-std Compatible**: Works in embedded environments without the standard library
- **Zero-cost Abstractions**: High-level API with no runtime overhead
- **AHRS Algorithm**: Sensor fusion combining gyroscope, accelerometer, and magnetometer data
- **Gyroscope Offset Correction**: Runtime calibration for temperature compensation
- **Sensor Calibration**: Built-in calibration functions for all sensor types
- **nalgebra Integration**: Leverages the robust nalgebra ecosystem for matrix operations

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
fusion-ahrs = "1.0"
nalgebra = "0.32"
```

For embedded (no-std) environments:

```toml
[dependencies]
fusion-ahrs = { version = "1.0", default-features = false }
nalgebra = { version = "0.32", default-features = false }
```

## AHRS Algorithm

The Attitude And Heading Reference System (AHRS) algorithm combines gyroscope, accelerometer, and magnetometer data into a single measurement of orientation relative to the Earth. The algorithm also supports systems that use only a gyroscope and accelerometer, and systems that use a gyroscope and accelerometer combined with an external source of heading measurement such as GPS.

The algorithm is based on the revised AHRS algorithm presented in chapter 7 of [Madgwick's PhD thesis](https://x-io.co.uk/downloads/madgwick-phd-thesis.pdf). This is a different algorithm to the better-known initial AHRS algorithm presented in chapter 3, commonly referred to as the *Madgwick algorithm*.

The algorithm calculates the orientation as the integration of the gyroscope summed with a feedback term. The feedback term is equal to the error in the current measurement of orientation as determined by the other sensors, multiplied by a gain. The algorithm therefore functions as a complementary filter that combines high-pass filtered gyroscope measurements with low-pass filtered measurements from other sensors with a corner frequency determined by the gain. A low gain will 'trust' the gyroscope more and so be more susceptible to drift. A high gain will increase the influence of other sensors and the errors that result from accelerations and magnetic distortions. A gain of zero will ignore the other sensors so that the measurement of orientation is determined by only the gyroscope.

### Basic Example

```rust
use fusion_ahrs::{Ahrs, AhrsSettings};
use nalgebra::{Vector3, UnitQuaternion};

// Create AHRS instance with default settings
let settings = AhrsSettings::default();
let mut ahrs = Ahrs::new(settings);

// Sample sensor data using nalgebra vectors
let gyroscope = Vector3::new(0.0, 0.0, 0.0);      // rad/s
let accelerometer = Vector3::new(0.0, 0.0, 1.0);  // g
let magnetometer = Vector3::new(1.0, 0.0, 0.0);   // normalized

// Update algorithm (typically called at 100Hz or higher)
let sample_period = 0.01; // 10ms
ahrs.update(gyroscope, accelerometer, magnetometer, sample_period);

// Get orientation quaternion
let quaternion = ahrs.quaternion();
let euler_angles = quaternion.euler_angles();

println!("Roll: {:.1}°, Pitch: {:.1}°, Yaw: {:.1}°", 
         euler_angles.0.to_degrees(), // roll
         euler_angles.1.to_degrees(), // pitch
         euler_angles.2.to_degrees()  // yaw
);
```

### Initialisation

Initialisation occurs when the algorithm starts for the first time and during angular rate recovery. During initialisation, the acceleration and magnetic rejection features are disabled and the gain is ramped down from 10 to the final value over a 3 second period. This allows the measurement of orientation to rapidly converge from an arbitrary initial value to the value indicated by the sensors.

### Angular Rate Recovery

Angular rates that exceed the gyroscope measurement range cannot be tracked and will trigger an angular rate recovery. Angular rate recovery is activated when the angular rate exceeds 98% of the gyroscope measurement range and is equivalent to a reinitialisation of the algorithm.

### Acceleration Rejection

The acceleration rejection feature reduces the errors that result from the accelerations of linear and rotational motion. Acceleration rejection works by calculating an error as the angular difference between the instantaneous measurement of inclination indicated by the accelerometer, and the current measurement of inclination provided by the algorithm output. If the error is greater than a threshold then the accelerometer will be ignored for that algorithm update. This is equivalent to a dynamic gain that decreases as accelerations increase.

Prolonged accelerations risk an overdependency on the gyroscope and will trigger an acceleration recovery. Acceleration recovery activates when the error exceeds the threshold for more than 90% of algorithm updates over a period of *t / (0.1p - 9)*, where *t* is the recovery trigger period and *p* is the percentage of algorithm updates where the error exceeds the threshold. The recovery will remain active until the error exceeds the threshold for less than 90% of algorithm updates over the period *-t / (0.1p - 9)*. The accelerometer will be used by every algorithm update during recovery.

### Magnetic Rejection

The magnetic rejection feature reduces the errors that result from temporary magnetic distortions. Magnetic rejection works using the same principle as acceleration rejection, operating on the magnetometer instead of the accelerometer and by comparing the measurements of heading instead of inclination.

### Algorithm Outputs

The algorithm provides four outputs: quaternion, gravity, linear acceleration, and Earth acceleration. The quaternion describes the orientation of the sensor relative to the Earth. This can be converted to a rotation matrix using nalgebra's `to_rotation_matrix()` method or to Euler angles using the `euler_angles()` method. Gravity is a direction of gravity in the sensor coordinate frame. Linear acceleration is the accelerometer measurement with gravity removed. Earth acceleration is the accelerometer measurement in the Earth coordinate frame with gravity removed. The algorithm supports North-West-Up (NWU), East-North-Up (ENU), and North-East-Down (NED) axes conventions.

```rust
use nalgebra::{Matrix3, Vector3, UnitQuaternion};

// Get all algorithm outputs
let quaternion: UnitQuaternion<f32> = ahrs.quaternion();
let gravity: Vector3<f32> = ahrs.gravity();
let linear_acceleration: Vector3<f32> = ahrs.linear_acceleration();
let earth_acceleration: Vector3<f32> = ahrs.earth_acceleration();

// Convert quaternion to different representations using nalgebra
let rotation_matrix: Matrix3<f32> = quaternion.to_rotation_matrix().into_inner();
let euler_angles = quaternion.euler_angles(); // (roll, pitch, yaw)
```

### Algorithm Settings

The AHRS algorithm settings are defined by the `AhrsSettings` struct:

```rust
use fusion_ahrs::{AhrsSettings, Convention};

let settings = AhrsSettings {
    convention: Convention::Nwu,
    gain: 0.5,
    gyroscope_range: 2000.0,
    acceleration_rejection: 10.0,
    magnetic_rejection: 10.0,
    recovery_trigger_period: 500,
};

let mut ahrs = Ahrs::new(settings);
```

| Setting                   | Type       | Description |
|---------------------------|------------|-------------|
| `convention`              | `Convention` | Earth axes convention (NWU, ENU, or NED) |
| `gain`                    | `f32`      | Determines the influence of the gyroscope relative to other sensors. A value of zero will disable initialisation and the acceleration and magnetic rejection features. A value of 0.5 is appropriate for most applications |
| `gyroscope_range`         | `f32`      | Gyroscope range (in degrees per second). Angular rate recovery will activate if the gyroscope measurement exceeds 98% of this value. A value of zero will disable this feature |
| `acceleration_rejection`  | `f32`      | Threshold (in degrees) used by the acceleration rejection feature. A value of zero will disable this feature. A value of 10 degrees is appropriate for most applications |
| `magnetic_rejection`      | `f32`      | Threshold (in degrees) used by the magnetic rejection feature. A value of zero will disable the feature. A value of 10 degrees is appropriate for most applications |
| `recovery_trigger_period` | `u32`      | Acceleration and magnetic recovery trigger period (in samples). A value of zero will disable the acceleration and magnetic rejection features. A period of 5 seconds is appropriate for most applications |

### Algorithm Internal States

The AHRS algorithm internal states can be accessed through the `internal_states()` method:

```rust
let states = ahrs.internal_states();
println!("Acceleration error: {:.1}°", states.acceleration_error);
println!("Accelerometer ignored: {}", states.accelerometer_ignored);
```

| Field                           | Type   | Description |
|--------------------------------|--------|-------------|
| `acceleration_error`           | `f32`  | Angular error (in degrees) of the algorithm output relative to the instantaneous measurement of inclination indicated by the accelerometer |
| `accelerometer_ignored`        | `bool` | `true` if the accelerometer was ignored by the previous algorithm update |
| `acceleration_recovery_trigger`| `f32`  | Acceleration recovery trigger value between 0.0 and 1.0. Acceleration recovery will activate when this value reaches 1.0 |
| `magnetic_error`               | `f32`  | Angular error (in degrees) of the algorithm output relative to the instantaneous measurement of heading indicated by the magnetometer |
| `magnetometer_ignored`         | `bool` | `true` if the magnetometer was ignored by the previous algorithm update |
| `magnetic_recovery_trigger`    | `f32`  | Magnetic recovery trigger value between 0.0 and 1.0. Magnetic recovery will activate when this value reaches 1.0 |

### Algorithm Flags

The AHRS algorithm flags can be accessed through the `flags()` method:

```rust
let flags = ahrs.flags();
if flags.initialising {
    println!("Algorithm is still initialising");
}
```

| Flag                     | Type   | Description |
|--------------------------|--------|-------------|
| `initialising`           | `bool` | `true` if the algorithm is initialising |
| `angular_rate_recovery`  | `bool` | `true` if angular rate recovery is active |
| `acceleration_recovery`  | `bool` | `true` if acceleration recovery is active |
| `magnetic_recovery`      | `bool` | `true` if magnetic recovery is active |

## Gyroscope Offset Correction Algorithm

The gyroscope offset correction algorithm provides run-time calibration of the gyroscope offset to compensate for variations in temperature and fine-tune existing offset calibration that may already be in place. This algorithm should be used in conjunction with the AHRS algorithm to achieve best performance.

```rust
use fusion_ahrs::{Offset, OffsetSettings};
use nalgebra::Vector3;

let settings = OffsetSettings::default();
let mut offset = Offset::new(settings);

// Update with gyroscope measurements
let gyroscope = Vector3::new(0.1, -0.05, 0.02); // Small offsets while stationary
offset.update(gyroscope);

// Get the calculated offset
let calculated_offset: Vector3<f32> = offset.offset();

// Apply offset correction to raw gyroscope data
let corrected_gyroscope = gyroscope - calculated_offset;
```

The algorithm calculates the gyroscope offset by detecting the stationary periods that occur naturally in most applications. Gyroscope measurements are sampled during these periods and low-pass filtered to obtain the gyroscope offset. The algorithm requires that gyroscope measurements do not exceed ±3 degrees per second while stationary. Basic gyroscope offset calibration may be necessary to ensure that the initial offset plus measurement noise is within these bounds.

## Sensor Calibration

Sensor calibration is essential for accurate measurements. This library provides functions to apply calibration parameters to the gyroscope, accelerometer, and magnetometer. This library does not provide a solution for calculating the calibration parameters.

### Inertial Calibration

The `calibrate_inertial` function applies gyroscope and accelerometer calibration parameters:

```rust
use fusion_ahrs::calibrate_inertial;
use nalgebra::{Matrix3, Vector3};

let uncalibrated = Vector3::new(1.0, 2.0, 3.0);
let misalignment = Matrix3::identity();
let sensitivity = Vector3::new(1.0, 1.0, 1.0);
let offset = Vector3::new(0.1, 0.2, 0.3);

let calibrated = calibrate_inertial(uncalibrated, misalignment, sensitivity, offset);
```

Using the calibration model: **i**<sub>c</sub> = **Ms**(**i**<sub>u</sub> - **b**)

- **i**<sub>c</sub> is the calibrated inertial measurement (return value)
- **i**<sub>u</sub> is the uncalibrated inertial measurement
- **M** is the misalignment matrix
- **s** is the sensitivity diagonal matrix
- **b** is the offset vector

### Magnetic Calibration

The `calibrate_magnetic` function applies magnetometer calibration parameters:

```rust
use fusion_ahrs::calibrate_magnetic;
use nalgebra::{Matrix3, Vector3};

let uncalibrated = Vector3::new(0.5, 0.3, 0.8);
let soft_iron_matrix = Matrix3::identity();
let hard_iron_offset = Vector3::new(0.1, -0.2, 0.05);

let calibrated = calibrate_magnetic(uncalibrated, soft_iron_matrix, hard_iron_offset);
```

Using the calibration model: **m**<sub>c</sub> = **S**(**m**<sub>u</sub> - **h**)

- **m**<sub>c</sub> is the calibrated magnetometer measurement (return value)
- **m**<sub>u</sub> is the uncalibrated magnetometer measurement
- **S** is the soft iron matrix
- **h** is the hard iron offset vector

## nalgebra Integration

Fusion AHRS leverages the powerful nalgebra crate for all matrix and vector operations, providing:

- **Type Safety**: Compile-time dimensional analysis prevents matrix dimension mismatches
- **Performance**: Highly optimized SIMD operations where available
- **Interoperability**: Seamless integration with the broader Rust scientific computing ecosystem
- **Generic Programming**: Support for different numeric types (f32, f64) and compile-time sizing

### Working with nalgebra Types

All sensor inputs and algorithm outputs use standard nalgebra types:

```rust
use nalgebra::{Vector3, UnitQuaternion, Matrix3};
use fusion_ahrs::Ahrs;

// All sensor data uses Vector3<f32>
let gyro_data: Vector3<f32> = Vector3::new(0.1, 0.05, -0.02);
let accel_data: Vector3<f32> = Vector3::new(0.0, 0.0, 9.81);
let mag_data: Vector3<f32> = Vector3::new(0.3, 0.1, 0.8);

// Quaternion outputs are UnitQuaternion<f32>
let orientation: UnitQuaternion<f32> = ahrs.quaternion();

// Easy conversion to other representations
let rotation_matrix: Matrix3<f32> = orientation.to_rotation_matrix().into_inner();
let axis_angle = orientation.axis_angle();
```

This integration allows you to easily combine Fusion AHRS with other nalgebra-based libraries in the Rust ecosystem for robotics, computer vision, and scientific computing applications.

Fusion AHRS uses an optimized implementation of the fast inverse square root algorithm for vector and quaternion normalisation. You can enable standard library square root operations by using the `std-sqrt` feature:

```toml
[dependencies]
fusion-ahrs = { version = "1.0", features = ["std-sqrt"] }
```

## Fast Inverse Square Root

Fusion AHRS uses an optimized implementation of the fast inverse square root algorithm for vector and quaternion normalisation. You can enable standard library square root operations by using the `std-sqrt` feature:

```toml
[dependencies]
fusion-ahrs = { version = "1.0", features = ["std-sqrt"] }
```

This will use standard square root operations for all normalisation calculations, which will slightly slow down execution speed for a small increase in accuracy. The increase in accuracy will typically be too small to observe in any practical system.

## Examples

The library includes comprehensive examples:

- `simple_example.rs` - Basic AHRS usage with sample data
- `advanced_example.rs` - Complete sensor fusion with calibration and offset correction
- `embedded_example.rs` - No-std usage for embedded systems

Run examples with:

```bash
cargo run --example simple_example
```

## Performance

This Rust implementation maintains the same computational efficiency as the original C library while providing additional safety guarantees. Benchmarks show comparable performance with zero-cost abstractions ensuring no runtime overhead for the high-level API.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.