# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

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

[Unreleased]: https://github.com/wboayue/fusion-ahrs/compare/v0.7.0...HEAD
[0.7.0]: https://github.com/wboayue/fusion-ahrs/compare/v0.6.0...v0.7.0
[0.6.0]: https://github.com/wboayue/fusion-ahrs/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/wboayue/fusion-ahrs/compare/v0.4.1...v0.5.0
[0.4.1]: https://github.com/wboayue/fusion-ahrs/compare/v0.4.0...v0.4.1
[0.4.0]: https://github.com/wboayue/fusion-ahrs/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/wboayue/fusion-ahrs/compare/v0.2.1...v0.3.0
[0.2.1]: https://github.com/wboayue/fusion-ahrs/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/wboayue/fusion-ahrs/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/wboayue/fusion-ahrs/releases/tag/v0.1.0
