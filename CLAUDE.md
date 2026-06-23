## Objective

Rust port of the Fusion AHRS C library, maintaining algorithm parity while following Rust best practices.

## Quick Reference

```bash
git submodule update --init         # required for C parity tests (pulls fusion-c/)
cargo test                          # run all tests
cargo fmt                           # format (required before commit)
cargo clippy                        # lint
cargo doc --no-deps                 # build rustdoc for public API
cargo bench                         # criterion benchmarks (ahrs_benchmarks)
cargo run --example simple          # basic 6-DOF usage with plots
cargo run --example advanced        # full 9-DOF with offset & diagnostics
```

## Architecture

- **Input**: Gyroscope, accelerometer, magnetometer data as `nalgebra::Vector3<f32>`
- **Output**: Orientation as `nalgebra::UnitQuaternion<f32>`
- **Compatibility**: `#![no_std]` (edition 2024, MSRV 1.85)

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

### Project-Specific Conventions
- Extend math behavior through the `Vector3Ext` / `QuaternionExt` traits in `math.rs` rather than free functions
- Settings types (`AhrsSettings`, `OffsetSettings`) are plain structs constructed directly — no builders
- All public APIs carry rustdoc with at least one example; doctests should compile

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
- Maintain embedded compatibility: `src/lib.rs` is `#![no_std]` unconditionally — do not introduce `std`-only dependencies or APIs
- Most modules (`ahrs`, `axes`, `calibration`, `compass`, `math`, `offset`) carry inline unit tests in a `#[cfg(test)] mod tests` block; integration tests live in `tests/`
- Commit messages follow conventional-commit style. Common prefixes: `feat(scope): …`, `fix(scope): …`, `docs: …`, `test: …`, `refactor: …`, `bench: …`, `chore(scope): …` (e.g. `chore(deps)`, `chore(cargo)`, `chore(parity)`). `fmt: …` is the project-specific prefix for pure `cargo fmt` commits

## C Parity Workflow
Algorithm parity with the upstream C library is enforced via integration tests:
- `tests/c_parity_tests.rs` — pure-Rust assertions that mirror C behavior on synthetic inputs
- `tests/c_comparison_test.rs` — runs both implementations on `testdata/sensor_data.csv` and compares outputs (requires `fusion-c/` submodule)
- `tests/verification_tests.rs` — broader algorithm-behavior checks

When Rust output diverges from C, the C side is authoritative — port the C fix into the Rust implementation rather than adjusting the Rust output. If a deliberate divergence is unavoidable, document it inline and in the PR description.

## README Maintenance
Keep `README.md` in sync with the code in the same PR that introduces the change — a stale README is worse than no README.
- **Public API changes**: when a function signature, type name, constructor, or default value changes, update every README snippet that shows it. Grep the README for the old name before assuming nothing references it.
- **Usage patterns**: if the recommended way to initialize or call something shifts (e.g. builder vs. direct struct, new required setting), rewrite the quickstart and any example snippets to match.
- **Examples**: when `examples/` gains, loses, or renames a file, update the example list and any `cargo run --example …` invocations in the README.
- **Features & conventions**: when adding/removing a coordinate convention, feature flag, or supported sensor mode, update the feature list and any compatibility notes.
- **Dependencies & MSRV**: bumping `nalgebra`, the Rust edition, or MSRV requires updating the README's dependency snippet and any version callouts.
- **Verify**: every code block in the README must compile against current `src/`. If unsure, copy the snippet into an example or doctest and run it.

## Changelog Maintenance
Keep `CHANGELOG.md` following [Keep a Changelog 1.1.0](https://keepachangelog.com/en/1.1.0/). The project follows [Semantic Versioning](https://semver.org/).
- **Update in the same PR** that makes a user-facing change — never batch changelog edits at release time. A stale changelog is worse than none.
- Maintain an `## [Unreleased]` section at the top; add new entries there as work lands.
- Group entries under these headings (only include those that apply): `Added`, `Changed`, `Deprecated`, `Removed`, `Fixed`, `Security`.
- Write for humans, not machines: describe the impact, don't dump commit messages or diffs. Skip purely internal churn (refactors, fmt, CI) unless it affects users.
- On release: rename `## [Unreleased]` to `## [x.y.z] - YYYY-MM-DD`, then start a fresh empty `## [Unreleased]` above it. Keep latest version first.
- Maintain linkable version reference links at the bottom (compare URLs, e.g. `[0.6.0]: https://github.com/wboayue/fusion-ahrs/compare/v0.5.0...v0.6.0`).
- Bump the version in `Cargo.toml` to match the released version in the same PR.

## Release Notes Guidelines
- Published as GitHub Releases, derived from the `CHANGELOG.md` entry for that version; body is authored/expanded when tagging
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