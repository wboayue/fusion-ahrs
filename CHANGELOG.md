# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added
- `Vector`, `Quaternion`, `Matrix`, and `Euler` math types mirroring the C library's `FusionMath.h`. Arithmetic follows C's operation order and matches C built with `FUSION_USE_NORMAL_SQRT` bit for bit; the default C build uses a fast approximate inverse square root, and normalising a zero vector returns zero where C returns NaN.
- Functions that take vectors accept `impl Into<Vector>`, so `[f32; 3]` arrays work directly; `set_quaternion` accepts `impl Into<Quaternion>`, and the calibration functions accept `impl Into<Matrix>`.

### Changed
- **Breaking:** the public API uses the crate's own math types instead of `nalgebra`. `Ahrs::quaternion` returns `Quaternion`; `gravity`, `linear_acceleration`, `earth_acceleration`, `Offset::update`, `Offset::offset`, `axes_swap`, `calibrate_inertial`, and `calibrate_magnetic` return `Vector`. `nalgebra` is no longer a dependency; optional conversions will follow behind a feature.
- In 9-axis mode, the quaternion and acceleration outputs now match the C library (built with `FUSION_USE_NORMAL_SQRT`) bit for bit on the test data (previously within about 5e-7). Remaining differences come only from `libm` trigonometric functions (`asinf`, `atan2f`, `sinf`, `cosf`), used by the error angles, `set_heading`, and external heading updates.

### Removed
- **Breaking:** `Vector3Ext` and `QuaternionExt` traits. Their methods are on the new types: `Vector::norm`, `Vector::normalize` (was `safe_normalize`), `Vector::to_radians`/`to_degrees`, `Quaternion::to_euler` and `Quaternion::from_euler` (degrees, via `Euler`).

## [0.8.0] - 2026-09-28

Synced with upstream Fusion C `a8d7224` (2026-09-18) (#40). This release contains breaking API changes that mirror upstream.

### Added
- `AhrsSettings::sample_rate` (Hz, default 100). The gyroscope is integrated over `1 / sample_rate`.
- `Ahrs::set_sample_period` to compensate for per-sample timing jitter.
- `Ahrs::skip_startup` to skip the startup gain ramp when the initial orientation is already known.
- `Ahrs::restart`, replacing `initialise`/`reset`.
- `Convention::ALL` and `AxesAlignment::ALL` constants for iterating over every variant.
- `Ahrs` now implements `Debug` and `Clone`.
- `Display` and `as_str()` for `Convention` (e.g. `"North, West, Up (NWU)"`) and `AxesAlignment` (e.g. `"+Y-X+Z"`), matching upstream's to-string functions.

### Changed
- **Breaking:** `update`, `update_no_magnetometer`, and `update_external_heading` no longer take a `delta_time` argument. Set `sample_rate` in settings, and call `set_sample_period` before each update if timing varies.
- **Breaking:** `AhrsSettings::recovery_trigger_period` (`u32`, samples) replaced by `rejection_timeout` (`f32`, seconds).
- **Breaking:** `AhrsSettings::default()` now disables acceleration and magnetic rejection (`acceleration_rejection` and `magnetic_rejection` are `0.0`, previously `90.0`). Behavior is unchanged, since rejection was already disabled by the zero timeout.
- **Breaking:** `AhrsFlags::initialising` renamed to `startup`; `AhrsFlags::angular_rate_recovery` renamed to `overrange_recovery`.
- Vector and quaternion normalisation now multiply by the reciprocal of the norm, as the C library does. This affects `Vector3Ext::safe_normalize` and roughly halves the remaining numeric difference from C.
- The startup gain ramp now steps once per update based on the configured sample rate, rather than on the per-call time step.

### Deprecated
- `Ahrs::initialise` and `Ahrs::reset`; use `Ahrs::restart`.

### Fixed
- The crate's `documentation` link now points to the docs.rs page; it previously returned 404.
- Gyroscope overrange recovery now preserves the last accelerometer reading, so `linear_acceleration` and `earth_acceleration` remain valid during recovery.
- The accelerometer/magnetometer residual is now normalised when the sensor and reference are exactly perpendicular, matching upstream.
- `earth_acceleration` now uses the upstream formulation (rotated accelerometer minus gravity) for closer numeric parity.

## [0.7.0] - 2026-06-22

### Changed
- Bump `nalgebra` requirement from 0.34 to 0.35 (#37). `nalgebra` types are part of the public API, so downstream crates must move to 0.35-compatible `Vector3`/`UnitQuaternion`.

## [0.6.0] - 2026-05-19

### Added
- Declared MSRV in `Cargo.toml` via `rust-version = "1.85"`, matching the edition 2024 baseline (#35).

### Changed
- Dual-licensed under MIT OR Apache-2.0, the standard pattern for Rust crates; contributions are dual-licensed by default (#36).

### Fixed
- `set_heading` no longer drifts near pitch ≈ ±90°; now matches the C library with direct yaw extraction and a singularity-free pure-Z rotation (#35).
- `internal_states().magnetic_error` now retains the last real magnetic error after a mag-on → mag-off transition instead of force-zeroing it, restoring C parity (#35).

## [0.5.0] - 2026-05-19

### Changed
- Synced the `fusion-c` submodule to upstream `bce206d` (2026-03-24), porting the bias-algorithm refactor (#34).
- `OffsetSettings::default().timeout` lowered from `5.0` to `3.0` seconds to match the upstream default; offset estimation begins 2 s sooner on default settings (#34).

## [0.4.1] - 2026-05-19

### Fixed
- Corrected three README snippets: the non-existent `Ahrs::new(settings)` constructor (use `with_settings`), a gyroscope unit comment (`deg/s`, not `rad/s`), and the offset-correction example now using the return value of `Offset::update`.

## [0.4.0] - 2026-03-26

### Added
- `OffsetSettings` now exposes `cutoff_frequency`, `timeout`, and `threshold` fields, defaulting to the C reference values (#31).

### Fixed
- `OffsetSettings` values were ignored in `Offset::new` (parameters were hardcoded); all settings are now applied (#30, #31).

## [0.3.0] - 2026-01-03

### Added
- Axes module: `AxesAlignment` enum with 24 sensor mounting orientations and `axes_swap()` for sensor-to-body remapping.
- Comprehensive C parity tests.

### Changed
- **Breaking:** `AhrsSettings` defaults now match the C library — `gyroscope_range` 0.0 (disabled), `acceleration_rejection` 90.0°, `magnetic_rejection` 90.0°, `recovery_trigger_period` 0 (disabled).
- Removed `Cargo.lock` from version control.

### Fixed
- Feedback scaling in AHRS update (removed extra 0.5 factor).
- Added `asin()` to `internal_states` error calculation.
- Calibration order corrected to `(uncalibrated - offset) * sensitivity`.
- `half_magnetic` formulas for NWU, ENU, and NED conventions.
- `flags()` now compares trigger > timeout.
- `internal_states` now returns a normalized trigger (0.0–1.0).
- `update_external_heading` now matches the C algorithm.
- `initialise()` now sets recovery timeouts to the period.
- `no_std` build: import `RealField` for `atan2`.

## [0.2.1] - 2026-01-02

### Changed
- Bumped `nalgebra` 0.34.0 → 0.34.1, `serde` 1.0.219 → 1.0.228, `csv` 1.3 → 1.4, `criterion` 0.5 → 0.8.

### Fixed
- Unused import warnings and CI test reporting (#24).

## [0.2.0] - 2025-08-06

### Added
- Criterion benchmarking suite with realistic sensor data generation and HTML reports.

### Changed
- Improved `no_std` support via nalgebra's `libm` feature for better embedded compatibility.
- Cleaned up code examples and API documentation; enhanced CI/CD with Coveralls coverage.
- Bumped `nalgebra` 0.33.2 → 0.34.0 (dev: `criterion` 0.5.1 → 0.7.0, `rand` 0.8.5 → 0.9.2, `rand_pcg` 0.3.1 → 0.9.0).

## [0.1.0] - 2025-06-02

### Added
- Initial release: complete Rust port of the xioTechnologies Fusion AHRS C library.
- Core sensor fusion combining gyroscope, accelerometer, and magnetometer data with automatic sensor rejection, initialization, and recovery.
- Support for NWU, ENU, and NED coordinate conventions.
- Gyroscope offset correction (`Offset`, `OffsetSettings`) and calibration functions (`calibrate_inertial()`, `calibrate_magnetic()`).
- Tilt-compensated heading via `calculate_heading()`.
- Real-time diagnostics through internal states and algorithm flags.
- `#![no_std]` compatibility with nalgebra integration.
- Simple and advanced examples plus included test data.

[Unreleased]: https://github.com/wboayue/fusion-ahrs/compare/v0.8.0...HEAD
[0.8.0]: https://github.com/wboayue/fusion-ahrs/compare/v0.7.0...v0.8.0
[0.7.0]: https://github.com/wboayue/fusion-ahrs/compare/v0.6.0...v0.7.0
[0.6.0]: https://github.com/wboayue/fusion-ahrs/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/wboayue/fusion-ahrs/compare/v0.4.1...v0.5.0
[0.4.1]: https://github.com/wboayue/fusion-ahrs/compare/v0.4.0...v0.4.1
[0.4.0]: https://github.com/wboayue/fusion-ahrs/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/wboayue/fusion-ahrs/compare/v0.2.1...v0.3.0
[0.2.1]: https://github.com/wboayue/fusion-ahrs/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/wboayue/fusion-ahrs/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/wboayue/fusion-ahrs/releases/tag/v0.1.0
