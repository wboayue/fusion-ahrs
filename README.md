[![Build](https://github.com/wboayue/fusion-ahrs/workflows/build/badge.svg)](https://github.com/wboayue/fusion-ahrs/actions/workflows/build.yml)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](#license)
[![crates.io](https://img.shields.io/crates/v/fusion-ahrs.svg)](https://crates.io/crates/fusion-ahrs)
[![Documentation](https://img.shields.io/badge/Documentation-green.svg)](https://docs.rs/fusion-ahrs/latest/fusion_ahrs/)
[![Coverage Status](https://coveralls.io/repos/github/wboayue/fusion-ahrs/badge.png?branch=main)](https://coveralls.io/github/wboayue/fusion-ahrs?branch=main)

# Fusion AHRS

Rust port of xioTechnologies' [Fusion AHRS C library](https://github.com/xioTechnologies/Fusion) — sensor fusion for gyroscope, accelerometer, and magnetometer with `no_std` support. Available on [crates.io](https://crates.io/crates/fusion-ahrs); runnable examples in [`examples/`](examples/).

## Features

- **Memory Safe**: Written in Rust with compile-time safety guarantees
- **No-std Compatible**: Works in embedded environments without the standard library
- **Zero-cost Abstractions**: High-level API with no runtime overhead
- **AHRS Algorithm**: Sensor fusion combining gyroscope, accelerometer, and magnetometer data
- **Gyroscope Bias Correction**: Runtime offset calibration for temperature compensation
- **Sensor Calibration**: Built-in calibration functions for all sensor types
- **Self-contained Math Types**: `Vector`, `Quaternion`, `Matrix`, and `Euler` mirror the C library, with no third-party types in the public API

## Installation

Requires Rust 1.85+. `no_std` is the default — no feature flags needed.

| Feature | Adds | Requires |
|---------|------|----------|
| `nalgebra-0_35` | Conversions to and from `nalgebra` 0.35 types (see [nalgebra Interop](#nalgebra-interop)) | Rust 1.89+ (nalgebra's MSRV) |

```bash
cargo add fusion-ahrs
```

## AHRS Algorithm

The Attitude And Heading Reference System (AHRS) algorithm combines gyroscope, accelerometer, and magnetometer data into a single measurement of orientation relative to the Earth. The algorithm also supports systems that use only a gyroscope and accelerometer, and systems that use a gyroscope and accelerometer combined with an external source of heading measurement such as GPS.

The algorithm is based on the revised AHRS algorithm presented in chapter 7 of [Madgwick's PhD thesis](https://x-io.co.uk/downloads/madgwick-phd-thesis.pdf). This is a different algorithm to the better-known initial AHRS algorithm presented in chapter 3, commonly referred to as the *Madgwick algorithm*.

### How it works

The algorithm calculates the orientation as the integration of the gyroscope summed with a feedback term. The feedback term is equal to the error in the current measurement of orientation as determined by the other sensors, multiplied by a gain. The algorithm therefore functions as a complementary filter that combines high-pass filtered gyroscope measurements with low-pass filtered measurements from other sensors with a corner frequency determined by the gain. A low gain will 'trust' the gyroscope more and so be more susceptible to drift. A high gain will increase the influence of other sensors and the errors that result from accelerations and magnetic distortions. A gain of zero will ignore the other sensors so that the measurement of orientation is determined by only the gyroscope.

### Basic Example

```rust
use fusion_ahrs::{Ahrs, Vector};

// Option 1: default settings (100 Hz sample rate)
let mut ahrs = Ahrs::new();

// Option 2: custom settings (see "Algorithm Settings" below)
// let mut ahrs = Ahrs::with_settings(settings);

// Sample sensor data
let gyroscope = Vector::new(0.0, 0.0, 0.0);      // deg/s
let accelerometer = Vector::new(0.0, 0.0, 1.0);  // g
let magnetometer = Vector::new(1.0, 0.0, 0.0);   // µT (or any units; will be normalized)

// Update algorithm once per sample at the configured sample rate
ahrs.update(gyroscope, accelerometer, magnetometer);

// Inputs accept anything convertible into a Vector, such as arrays
ahrs.update([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);

// Get orientation as a quaternion or Euler angles in degrees
let euler = ahrs.quaternion().to_euler();
println!("Roll: {:.1}°, Pitch: {:.1}°, Yaw: {:.1}°", euler.roll, euler.pitch, euler.yaw);
```

### Sample Rate

The gyroscope is integrated over a fixed sample period of `1 / sample_rate`, set through `AhrsSettings::sample_rate`. If sample timing jitters, call `set_sample_period` with the measured period before each update:

```rust
use fusion_ahrs::{Ahrs, Vector};

let mut ahrs = Ahrs::new();
let (gyroscope, accelerometer) = (Vector::ZERO, Vector::new(0.0, 0.0, 1.0));

let measured_period = 0.0102; // seconds since previous sample
ahrs.set_sample_period(measured_period);
ahrs.update_no_magnetometer(gyroscope, accelerometer);
```

### Startup

Startup occurs when the algorithm starts for the first time, after `restart()`, and during gyroscope overrange recovery. During startup, the acceleration and magnetic rejection features are disabled and the gain is ramped down from 10 to the final value over a 3 second period. This allows the measurement of orientation to rapidly converge from an arbitrary initial value to the value indicated by the sensors. If the initial orientation is already known, set it with `set_quaternion()` and call `skip_startup()` before the first update.

### Gyroscope Overrange Recovery

Angular rates that exceed the gyroscope measurement range cannot be tracked and will trigger an overrange recovery. Overrange recovery is activated when the angular rate exceeds 98% of the gyroscope measurement range and is equivalent to a restart of the algorithm that preserves the algorithm outputs.

### Acceleration Rejection

The acceleration rejection feature reduces the errors that result from the accelerations of linear and rotational motion. Acceleration rejection works by calculating an error as the angular difference between the instantaneous measurement of inclination indicated by the accelerometer, and the current measurement of inclination provided by the algorithm output. If the error is greater than a threshold then the accelerometer will be ignored for that algorithm update. This is equivalent to a dynamic gain that decreases as accelerations increase.

Prolonged accelerations risk an overdependency on the gyroscope and will trigger an acceleration recovery. Acceleration recovery activates when the error exceeds the threshold for more than 90% of algorithm updates over a period of *t / (0.1p - 9)*, where *t* is the rejection timeout and *p* is the percentage of algorithm updates where the error exceeds the threshold. The recovery will remain active until the error exceeds the threshold for less than 90% of algorithm updates over the period *-t / (0.1p - 9)*. The accelerometer will be used by every algorithm update during recovery.

### Magnetic Rejection

The magnetic rejection feature reduces the errors that result from temporary magnetic distortions. Magnetic rejection works using the same principle as acceleration rejection, operating on the magnetometer instead of the accelerometer and by comparing the measurements of heading instead of inclination.

### Algorithm Outputs

The algorithm provides four outputs: quaternion, gravity, linear acceleration, and Earth acceleration. The quaternion describes the orientation of the sensor relative to the Earth. It can be converted to a rotation matrix with `to_matrix()` or to Euler angles in degrees with `to_euler()`. Gravity is a direction of gravity in the sensor coordinate frame. Linear acceleration is the accelerometer measurement with gravity removed. Earth acceleration is the accelerometer measurement in the Earth coordinate frame with gravity removed. The algorithm supports North-West-Up (NWU), East-North-Up (ENU), and North-East-Down (NED) axes conventions.

```rust
use fusion_ahrs::{Ahrs, Euler, Matrix, Quaternion, Vector};

let mut ahrs = Ahrs::new();

// Get all algorithm outputs
let quaternion: Quaternion = ahrs.quaternion();
let gravity: Vector = ahrs.gravity();
let linear_acceleration: Vector = ahrs.linear_acceleration();
let earth_acceleration: Vector = ahrs.earth_acceleration();

// Convert the quaternion to other representations
let rotation_matrix: Matrix = quaternion.to_matrix();
let euler: Euler = quaternion.to_euler(); // roll, pitch, yaw in degrees
```

### Algorithm Settings

The AHRS algorithm settings are defined by the `AhrsSettings` struct:

```rust
use fusion_ahrs::{Ahrs, AhrsSettings, Convention};

let settings = AhrsSettings {
    sample_rate: 100.0,
    convention: Convention::Nwu,
    gain: 0.5,
    gyroscope_range: 2000.0,
    acceleration_rejection: 10.0,
    magnetic_rejection: 10.0,
    rejection_timeout: 5.0,
};

let mut ahrs = Ahrs::with_settings(settings);
```

| Setting                   | Type       | Description |
|---------------------------|------------|-------------|
| `sample_rate`             | `f32`      | Sample rate (in Hz). Default 100 |
| `convention`              | `Convention` | Earth axes convention (NWU, ENU, or NED) |
| `gain`                    | `f32`      | Determines the influence of the gyroscope relative to other sensors. A value of zero will disable startup and the acceleration and magnetic rejection features. A value of 0.5 is appropriate for most applications |
| `gyroscope_range`         | `f32`      | Gyroscope range (in degrees per second). Overrange recovery will activate if the gyroscope measurement exceeds 98% of this value. A value of zero (default) will disable this feature |
| `acceleration_rejection`  | `f32`      | Threshold (in degrees) used by the acceleration rejection feature. A value of zero (default) will disable this feature. A value of 10 degrees is appropriate for most applications |
| `magnetic_rejection`      | `f32`      | Threshold (in degrees) used by the magnetic rejection feature. A value of zero (default) will disable the feature. A value of 10 degrees is appropriate for most applications |
| `rejection_timeout`       | `f32`      | Acceleration and magnetic rejection timeout (in seconds). A value of zero (default) will disable the acceleration and magnetic rejection features. A timeout of 5 seconds is appropriate for most applications |

### Algorithm Internal States

The AHRS algorithm internal states can be accessed through the `internal_states()` method:

```rust
use fusion_ahrs::Ahrs;

let ahrs = Ahrs::new();
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
use fusion_ahrs::Ahrs;

let ahrs = Ahrs::new();
let flags = ahrs.flags();
if flags.startup {
    println!("Algorithm is still starting up");
}
```

| Flag                     | Type   | Description |
|--------------------------|--------|-------------|
| `startup`                | `bool` | `true` if the algorithm is in startup |
| `overrange_recovery`     | `bool` | `true` if gyroscope overrange recovery is active |
| `acceleration_recovery`  | `bool` | `true` if acceleration recovery is active |
| `magnetic_recovery`      | `bool` | `true` if magnetic recovery is active |

## Gyroscope Bias Correction Algorithm

The gyroscope bias correction algorithm (`Bias`, C `FusionBias`) provides run-time calibration of the gyroscope offset to compensate for variations in temperature and fine-tune existing offset calibration that may already be in place. This algorithm should be used in conjunction with the AHRS algorithm to achieve best performance.

```rust
use fusion_ahrs::{Bias, BiasSettings, Vector};

let mut bias = Bias::with_settings(BiasSettings {
    sample_rate: 100.0,        // Hz
    stationary_threshold: 3.0, // deg/s
    stationary_period: 3.0,    // seconds
    ..Default::default()
});

// Apply bias correction — update() returns the corrected reading
let gyroscope = Vector::new(0.1, -0.05, 0.02); // Small offsets while stationary
let corrected_gyroscope: Vector = bias.update(gyroscope);

// Inspect the current offset estimate at any time, e.g. to save it
let offset: Vector = bias.offset();

// Restore a saved offset at startup so correction begins immediately
bias.set_offset(offset);
```

The algorithm calculates the gyroscope offset by detecting the stationary periods that occur naturally in most applications. Gyroscope measurements are sampled during these periods and low-pass filtered to obtain the gyroscope offset. With default settings, the algorithm requires that gyroscope measurements do not exceed ±3 degrees per second (`stationary_threshold`) while stationary. Basic gyroscope offset calibration may be necessary to ensure that the initial offset plus measurement noise is within these bounds.

## Sensor Calibration

Sensor calibration is essential for accurate measurements. This library provides functions to apply calibration parameters to the gyroscope, accelerometer, and magnetometer. This library does not provide a solution for calculating the calibration parameters.

### Inertial Calibration

The `calibrate_inertial` function applies gyroscope and accelerometer calibration parameters:

```rust
use fusion_ahrs::{Matrix, Vector, calibrate_inertial};

let uncalibrated = Vector::new(1.0, 2.0, 3.0);
let misalignment = Matrix::IDENTITY;
let sensitivity = Vector::ONES;
let offset = Vector::new(0.1, 0.2, 0.3);

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
use fusion_ahrs::{Matrix, Vector, calibrate_magnetic};

let uncalibrated = Vector::new(0.5, 0.3, 0.8);
let soft_iron_matrix = Matrix::IDENTITY;
let hard_iron_offset = Vector::new(0.1, -0.2, 0.05);

let calibrated = calibrate_magnetic(uncalibrated, soft_iron_matrix, hard_iron_offset);
```

Using the calibration model: **m**<sub>c</sub> = **S**(**m**<sub>u</sub> - **h**)

- **m**<sub>c</sub> is the calibrated magnetometer measurement (return value)
- **m**<sub>u</sub> is the uncalibrated magnetometer measurement
- **S** is the soft iron matrix
- **h** is the hard iron offset vector

## Math Types

Sensor inputs and algorithm outputs use the crate's own `f32` math types, which mirror the C library's `FusionMath.h`:

| Type | Fields | Notes |
|------|--------|-------|
| `Vector` | `x, y, z` | Sensor readings and vector outputs |
| `Quaternion` | `w, x, y, z` | Orientation; scalar first |
| `Matrix` | `xx, xy, …, zz` | Row-major 3x3, used for calibration |
| `Euler` | `roll, pitch, yaw` | Degrees, ZYX order |

Their arithmetic follows the C library's operation order, so results match C when it is built with `FUSION_USE_NORMAL_SQRT` (the default C build uses a fast approximate inverse square root). Functions that take vectors accept anything convertible into a `Vector`, including `[f32; 3]`; quaternions convert from `[f32; 4]` (scalar first) and matrices from `[[f32; 3]; 3]` rows.

```rust
use fusion_ahrs::{Euler, Quaternion, Vector, remap, RemapAlignment};

let v = Vector::new(1.0, 2.0, 3.0);
let w: Vector = [4.0, 5.0, 6.0].into();
assert_eq!(v.dot(w), 32.0);

let q = Quaternion::from_euler(Euler::new(0.0, 0.0, 90.0));
let rotated = q.rotate(Vector::new(1.0, 0.0, 0.0)); // ≈ (0, 1, 0)

let body = remap([1.0, 2.0, 3.0], RemapAlignment::PyNxPz);
assert_eq!(body, Vector::new(2.0, -1.0, 3.0));
```

## nalgebra Interop

Enable the feature matching your `nalgebra` version to convert between the crate types and `nalgebra` types. Each `nalgebra` version gets its own feature, so a new version can be added without breaking existing users.

```toml
[dependencies]
fusion-ahrs = { version = "0.9", features = ["nalgebra-0_35"] }
nalgebra = "0.35"
```

`nalgebra` values can be passed directly to any function taking `impl Into<Vector>`, `impl Into<Quaternion>`, or `impl Into<Matrix>`, and outputs convert back with `.into()`:

```rust
use fusion_ahrs::Ahrs;
use nalgebra::{UnitQuaternion, Vector3};

let mut ahrs = Ahrs::new();
ahrs.update(
    Vector3::new(0.0, 0.0, 0.0),
    Vector3::new(0.0, 0.0, 1.0),
    Vector3::new(1.0, 0.0, 0.0),
);

let orientation: UnitQuaternion<f32> = ahrs.quaternion().into();
let gravity: Vector3<f32> = ahrs.gravity().into();
```

| fusion-ahrs | nalgebra |
|-------------|----------|
| `Vector` | `Vector3<f32>` |
| `Quaternion` | `Quaternion<f32>`, `UnitQuaternion<f32>` (normalised on conversion) |
| `Matrix` | `Matrix3<f32>` (row-major on both sides) |

## Examples

The library includes two runnable examples:

- `simple.rs` — basic 6-DOF AHRS usage with sample data and plots
- `advanced.rs` — 9-DOF sensor fusion with bias correction, custom settings, and internal-state diagnostics

```bash
cargo run --example simple
cargo run --example advanced
```

## Benchmarks

Criterion benchmarks live in [`benches/ahrs_benchmarks.rs`](benches/ahrs_benchmarks.rs) and cover `update`, `update_no_magnetometer`, startup, steady state, batch updates, and per-output accessors. Reports land in `target/criterion/`.

```bash
cargo bench
```

## Versioning

The crate follows [Semantic Versioning](https://semver.org/). From 1.0:

- **Bug fixes**: fixes to the Rust port that don't change the API ship in patch releases, even when they change outputs; the changelog notes any output change.
- **Upstream changes**: the crate tracks the [Fusion C library](https://github.com/xioTechnologies/Fusion). Upstream fixes that change outputs ship in minor releases; upstream API changes that add or rename settings, flags, or functions ship in a new major release, since the public settings and state structs have public fields.
- **MSRV**: the minimum supported Rust version (currently 1.85) is raised only in a minor release, never a patch release, and each bump is noted in the changelog.
- **nalgebra**: each supported nalgebra version has its own feature (`nalgebra-0_35`, …). New versions are added alongside existing ones in minor releases; removing one is a major change. A feature may require a newer Rust than the crate MSRV if nalgebra does.
- **Numeric output**: results may change in the last bits between minor releases when parity with the C library improves; such changes are noted in the changelog.

## License

Licensed under the [MIT license](LICENSE).

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you shall be licensed as above, without any additional terms or conditions.
