## Objective

Rust port of the Fusion AHRS C library, maintaining algorithm parity while following Rust best practices.

## Quick Reference

```bash
cargo test                          # run all tests
cargo build --no-default-features   # verify no_std / embedded compatibility
cargo fmt                           # format (required before commit)
cargo clippy                        # lint
cargo bench                         # criterion benchmarks (ahrs_benchmarks)
cargo run --example simple          # basic 6-DOF usage with plots
cargo run --example advanced        # full 9-DOF with offset & diagnostics
```

## Architecture

- **Input**: Gyroscope, accelerometer, magnetometer data as `nalgebra::Vector3<f32>`
- **Output**: Orientation as `nalgebra::UnitQuaternion<f32>`
- **Compatibility**: `#![no_std]` (edition 2024)

### Source Layout

```
src/
  lib.rs          – public API re-exports
  ahrs.rs         – core AHRS algorithm (update, quaternion, gravity, linear/earth acceleration)
  types.rs        – AhrsSettings, AhrsInternalStates, AhrsFlags, Convention, OffsetSettings
  offset.rs       – gyroscope offset correction
  calibration.rs  – calibrate_inertial(), calibrate_magnetic()
  math.rs         – math utilities, Vector3Ext / QuaternionExt traits
  axes.rs         – sensor axes alignment (axes_swap, AxesAlignment)
  compass.rs      – tilt-compensated magnetic heading (calculate_heading)
```

### Dependencies
- `nalgebra` — vector/matrix ops (no-std compatible, `libm` feature)
- Dev only: `csv`, `serde`, `plotters`, `criterion`, `rand`, `rand_pcg`
- C reference implementation in `fusion-c/` (git submodule — `git submodule update --init`)

### Code Quality Standards
- **Modularity**: Single-responsibility, minimal public APIs
- **Performance**: Zero-cost abstractions, minimal allocations
- **Testability**: Comprehensive unit tests with provided test data
- **Documentation**: Rustdoc for all public APIs with examples

### Algorithm Features
- Complementary filter combining high-pass gyroscope + low-pass accel/mag
- Acceleration/magnetic rejection for motion artifacts
- Automatic initialization and recovery modes
- Support for NWU, ENU, NED coordinate conventions

## Test Data

`testdata/sensor_data.csv` — columns: Time (s), Gyro X/Y/Z (deg/s), Accel X/Y/Z (g), Mag X/Y/Z (µT). No pre-computed reference output; tests validate algorithm behavior and C parity directly.

## Development Guidelines
- Follow the C implementation's algorithm behavior exactly
- Use nalgebra types consistently (`Vector3`, `UnitQuaternion`, `Matrix3`)
- Maintain embedded compatibility (`cargo build --no-default-features`)
- Most modules (`ahrs`, `axes`, `calibration`, `compass`, `math`, `offset`) carry inline unit tests in a `#[cfg(test)] mod tests` block; integration tests live in `tests/`
- Commit messages follow conventional-commit style: `fix(scope): …`, `docs: …`, `feat(scope): …`, `chore(deps): …`, `fmt: …`

## README Maintenance
Keep `README.md` in sync with the code in the same PR that introduces the change — a stale README is worse than no README.
- **Public API changes**: when a function signature, type name, constructor, or default value changes, update every README snippet that shows it. Grep the README for the old name before assuming nothing references it.
- **Usage patterns**: if the recommended way to initialize or call something shifts (e.g. builder vs. direct struct, new required setting), rewrite the quickstart and any example snippets to match.
- **Examples**: when `examples/` gains, loses, or renames a file, update the example list and any `cargo run --example …` invocations in the README.
- **Features & conventions**: when adding/removing a coordinate convention, feature flag, or supported sensor mode, update the feature list and any compatibility notes.
- **Dependencies & MSRV**: bumping `nalgebra`, the Rust edition, or MSRV requires updating the README's dependency snippet and any version callouts.
- **Verify**: every code block in the README must compile against current `src/`. If unsure, copy the snippet into an example or doctest and run it.

## Release Notes Guidelines
- Published as GitHub Releases (no in-repo CHANGELOG); body is authored when tagging
- Group changes under ## What's New and ## Bug Fixes headings as applicable
- Each item gets an ### H3 heading with short description and PR number (e.g., ### Feature name (#123))
- One-sentence summary below the heading
- A code sample showing typical usage in a fenced ```rust block
- Order items by significance (most impactful first)

## Success Criteria
- Matches C library performance benchmarks
- Passes all test cases with provided sensor data
- Clear documentation with practical examples
- Modular, testable codebase following Rust idioms