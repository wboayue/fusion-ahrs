## Objective

Rust port of the Fusion AHRS C library, maintaining algorithm parity while following Rust best practices.

## Quick Reference

```bash
git submodule update --init         # required for C parity tests (pulls fusion-c/)
cargo test --all-features           # run all tests (needs submodule + C compiler)
cargo test --test c_comparison_test # AHRS vs C library, every sample
cargo test --test c_math_test       # math types vs FusionMath.h, bit-exact
cargo fmt --all                     # format (required before commit)
cargo clippy --workspace --all-targets -- -D warnings   # lint as CI does
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps          # rustdoc for public API
cargo +1.85 build --lib --target thumbv7em-none-eabihf  # MSRV + no_std check
cargo bench                         # criterion benchmarks (ahrs_benchmarks)
cargo run --example simple          # basic 6-DOF usage with plots
cargo run --example advanced        # full 9-DOF with offset & diagnostics
```

## Architecture

- **Input**: Gyroscope, accelerometer, magnetometer data as `impl Into<Vector>` (crate type; `[f32; 3]` works)
- **Output**: Orientation as `Quaternion`, vectors as `Vector` (crate types mirroring C `FusionMath.h`)
- **Compatibility**: `#![no_std]` (edition 2024, MSRV 1.85)

### Source Layout

```
src/
  lib.rs          – public API re-exports
  ahrs.rs         – core AHRS algorithm (update, quaternion, gravity, linear/earth acceleration)
  types.rs        – AhrsSettings, AhrsInternalStates, AhrsFlags, Convention, OffsetSettings
  offset.rs       – gyroscope offset correction
  calibration.rs  – calibrate_inertial(), calibrate_magnetic()
  math/           – Vector, Quaternion, Matrix, Euler (mirror FusionMath.h), DEG_TO_RAD / RAD_TO_DEG
  axes.rs         – sensor axes alignment (axes_swap, AxesAlignment)
  compass.rs      – tilt-compensated magnetic heading (calculate_heading)
fusion-c-sys/     – test-only workspace crate: builds fusion-c/ via `cc`, safe FFI wrappers (publish = false)
```

### Dependencies
- `libm` — `no_std` float functions (sqrt, trig); private, not part of the public API
- Optional `nalgebra_0_35` (package `nalgebra`) behind feature `nalgebra-0_35` — conversions only, in `src/interop.rs`. Needs Rust 1.89, above the crate MSRV
- Adding a nalgebra version: new optional dependency `nalgebra_X_Y = { package = "nalgebra", version = "X.Y", optional = true, default-features = false }`, feature `nalgebra-X_Y = ["dep:nalgebra_X_Y"]`, one `nalgebra_conversions!(nalgebra_X_Y, "nalgebra-X_Y")` line, and a copy of `tests/nalgebra_interop.rs`. Keep older versions; removing one is a breaking change
- Dev only: `csv`, `serde`, `plotters`, `criterion`, `rand`, `rand_pcg`, `fusion-c-sys` (path-only, stripped on publish)
- C reference implementation in `fusion-c/` (git submodule — `git submodule update --init`)

### Project-Specific Conventions
- Math operations are inherent methods on the `math/` types; port new C math functions there with C's exact operation order (C parity is checked bit for bit in `tests/c_math_test.rs`)
- Public functions take `impl Into<Vector>` (etc.) and delegate to a private non-generic body, so the algorithm is compiled and inlined in this crate rather than in each caller. Small math ops carry `#[inline]`
- Settings types (`AhrsSettings`, `OffsetSettings`) are plain structs constructed directly — no builders
- All public APIs carry rustdoc with at least one example; doctests should compile

### Algorithm Features
- Complementary filter combining high-pass gyroscope + low-pass accel/mag
- Acceleration/magnetic rejection for motion artifacts
- Startup gain ramp, gyroscope overrange recovery, acceleration/magnetic recovery
- Support for NWU, ENU, NED coordinate conventions

## Test Data

`testdata/sensor_data.csv` — columns: Time (s), Gyro X/Y/Z (deg/s), Accel X/Y/Z (g), Mag X/Y/Z (µT). No pre-computed reference output; tests validate algorithm behavior and C parity directly.

## Development Guidelines
- Follow the C implementation's algorithm behavior exactly
- Use the crate math types (`Vector`, `Quaternion`, `Matrix`, `Euler`) consistently; no third-party types in the public API
- In library code use `libm` for float functions (`libm::sqrtf`, `libm::fabsf`, …), since `no_std` on the MSRV lacks `f32` methods
- Maintain embedded compatibility: `src/lib.rs` is `#![no_std]` unconditionally — do not introduce `std`-only dependencies or APIs
- Most modules (`ahrs`, `axes`, `calibration`, `compass`, `math`, `offset`) carry inline unit tests in a `#[cfg(test)] mod tests` block; integration tests live in `tests/`
- Exact `f32` test constants (e.g. adjacent values around a boundary): use `f32::from_bits(0x…)`; long literals trip clippy `excessive_precision`
- Commit messages follow conventional-commit style. Common prefixes: `feat(scope): …`, `fix(scope): …`, `docs: …`, `test: …`, `refactor: …`, `bench: …`, `chore(scope): …` (e.g. `chore(deps)`, `chore(cargo)`, `chore(parity)`). `fmt: …` is the project-specific prefix for pure `cargo fmt` commits

## C Parity Workflow
Algorithm parity with the upstream C library is enforced via integration tests:
- `tests/c_parity_tests.rs` — pure-Rust assertions that mirror C behavior on synthetic inputs
- `tests/c_comparison_test.rs` — `c_*` tests run the C library (via `fusion-c-sys`) and Rust side by side on `testdata/sensor_data.csv`, comparing every output on every sample; also covers offset, compass, remap, calibration models, and to-string. Requires the `fusion-c/` submodule and a C compiler
- When syncing upstream, bump the submodule, run `cargo test --test c_comparison_test`, and add shim/wrapper coverage in `fusion-c-sys` for any new C API
- `tests/c_math_test.rs` — math types vs `FusionMath.h` on random inputs; arithmetic must match bit for bit
- `tests/verification_tests.rs` — broader algorithm-behavior checks

`fusion-c-sys` compiles C with `FUSION_USE_NORMAL_SQRT` and `-ffp-contract=off`. Any "matches C" claim (docs, changelog, PRs) must state that assumption: the default C build uses a fast approximate inverse square root. Expected results under that build:
- 9-axis update outputs (quaternion, gravity, linear/earth acceleration, flags, triggers) are bit-identical to C on `sensor_data.csv`
- Accepted divergences: trig-based values differ by ~1 ULP because C links the platform `libm` and Rust uses the `libm` crate (error angles, `set_heading`, external-heading updates); `Vector::normalize` of zero returns zero where C returns NaN

After adding or changing a parity test, prove it can fail: perturb one constant or operation order (e.g. 0.98 → 0.97, reassociate a sum), confirm the test fails, then revert.

When Rust output diverges from C, the C side is authoritative — port the C fix into the Rust implementation rather than adjusting the Rust output. If a deliberate divergence is unavoidable, document it inline and in the PR description.

## Performance
- Dev-machine benchmark noise is roughly ±8%; don't trust eyeballed before/after runs. Compare with criterion baselines sharing one target dir:
  ```bash
  export CARGO_TARGET_DIR=/tmp/bench-tgt
  git worktree add /tmp/main-wt main && (cd /tmp/main-wt && git submodule update --init)  # benches build fusion-c-sys
  (cd /tmp/main-wt && cargo bench --bench ahrs_benchmarks -- --save-baseline main)
  cargo bench --bench ahrs_benchmarks -- --baseline main
  ```
- Generic public functions are monomorphized in the caller's crate, where this crate's private non-`#[inline]` helpers can't inline. Keep generic shells thin (convert, then call a private non-generic body). Don't mark the private bodies `#[inline]`: that moves them back into the caller and regressed `update` 20–38%

## README Maintenance
Keep `README.md` in sync with the code in the same PR that introduces the change — a stale README is worse than no README.
- **Public API changes**: when a function signature, type name, constructor, or default value changes, update every README snippet that shows it. Grep the README for the old name before assuming nothing references it.
- **Usage patterns**: if the recommended way to initialize or call something shifts (e.g. builder vs. direct struct, new required setting), rewrite the quickstart and any example snippets to match.
- **Examples**: when `examples/` gains, loses, or renames a file, update the example list and any `cargo run --example …` invocations in the README.
- **Features & conventions**: when adding/removing a coordinate convention, feature flag, or supported sensor mode, update the feature list and any compatibility notes.
- **Dependencies & MSRV**: bumping a public-facing dependency, the Rust edition, or MSRV requires updating the README's dependency snippet and any version callouts.
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
- Title is the tag (e.g. `v0.8.0`); open with a short summary and the `Cargo.toml` dependency line
- Group changes under ## What's New, ## Breaking Changes, and ## Bug Fixes headings as applicable
- Breaking releases start with an "Upgrading from x.y" table (old API → new API), and each breaking item shows Before (x.y) / After snippets
- Each item gets an ### H3 heading with short description and PR number (e.g., ### Feature name (#123))
- One-sentence summary below the heading
- A code sample showing typical usage in a fenced ```rust block
- Order items by significance (most impactful first)
- Verify every snippet compiles and runs: "Before" snippets against the previous crates.io version, the rest against the release tag
- Show the draft to the maintainer before creating the GitHub release

## Release Workflow
1. Feature PRs are squash-merged with a conventional title ending in `(#N)`
2. Release PR `chore(release): vX.Y.Z`: bump `Cargo.toml`, promote `[Unreleased]`, update compare links; run `cargo package` (verifies the crate builds without `fusion-c-sys`)
3. After merge: `just tag vX.Y.Z`, then create the GitHub release from the approved notes
4. The maintainer runs `cargo publish`; afterwards confirm crates.io and docs.rs (`https://docs.rs/fusion-ahrs/X.Y.Z/fusion_ahrs/`) show the new version

## Success Criteria
- Matches C library performance benchmarks
- Passes all test cases with provided sensor data
- Clear documentation with practical examples
- Modular, testable codebase following Rust idioms