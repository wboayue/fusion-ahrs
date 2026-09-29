//! Test-only bindings to the upstream Fusion C library (`fusion-c/`).
//!
//! Used by the parity tests in the `fusion-ahrs` crate to run the C reference
//! implementation side by side with the Rust port. Not published.

use std::ffi::{CStr, c_char};

/// Mirrors `FusionAhrsSettings`.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct AhrsSettings {
    pub sample_rate: f32,
    /// `FusionConvention`: 0 = NWU, 1 = ENU, 2 = NED
    pub convention: u32,
    pub gain: f32,
    pub gyroscope_range: f32,
    pub acceleration_rejection: f32,
    pub magnetic_rejection: f32,
    pub rejection_timeout: f32,
}

/// Mirrors `FusionAhrsInternalStates`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct AhrsInternalStates {
    pub acceleration_error: f32,
    pub accelerometer_ignored: bool,
    pub acceleration_recovery_trigger: f32,
    pub magnetic_error: f32,
    pub magnetometer_ignored: bool,
    pub magnetic_recovery_trigger: f32,
}

/// Mirrors `FusionAhrsFlags`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct AhrsFlags {
    pub startup: bool,
    pub overrange_recovery: bool,
    pub acceleration_recovery: bool,
    pub magnetic_recovery: bool,
}

/// All AHRS outputs after an update. Mirrors `ShimOutputs` in `shim.c`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct AhrsOutputs {
    /// w, x, y, z
    pub quaternion: [f32; 4],
    pub gravity: [f32; 3],
    pub linear_acceleration: [f32; 3],
    pub earth_acceleration: [f32; 3],
    pub internal_states: AhrsInternalStates,
    pub flags: AhrsFlags,
}

/// Mirrors `FusionBiasSettings`.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct BiasSettings {
    pub sample_rate: f32,
    pub stationary_threshold: f32,
    pub stationary_period: f32,
}

#[repr(C)]
struct RawAhrs {
    _private: [u8; 0],
}

#[repr(C)]
struct RawBias {
    _private: [u8; 0],
}

unsafe extern "C" {
    fn ShimAhrsNew(settings: *const AhrsSettings) -> *mut RawAhrs;
    fn ShimAhrsFree(ahrs: *mut RawAhrs);
    fn ShimAhrsSetSettings(ahrs: *mut RawAhrs, settings: *const AhrsSettings);
    fn ShimAhrsSetSamplePeriod(ahrs: *mut RawAhrs, sample_period: f32);
    fn ShimAhrsRestart(ahrs: *mut RawAhrs);
    fn ShimAhrsSkipStartup(ahrs: *mut RawAhrs);
    fn ShimAhrsUpdate(ahrs: *mut RawAhrs, g: *const f32, a: *const f32, m: *const f32);
    fn ShimAhrsUpdateNoMagnetometer(ahrs: *mut RawAhrs, g: *const f32, a: *const f32);
    fn ShimAhrsUpdateExternalHeading(ahrs: *mut RawAhrs, g: *const f32, a: *const f32, h: f32);
    fn ShimAhrsSetQuaternion(ahrs: *mut RawAhrs, q: *const f32);
    fn ShimAhrsSetHeading(ahrs: *mut RawAhrs, heading: f32);
    fn ShimAhrsOutputs(ahrs: *const RawAhrs, outputs: *mut AhrsOutputs);

    fn ShimBiasNew(settings: *const BiasSettings) -> *mut RawBias;
    fn ShimBiasFree(bias: *mut RawBias);
    fn ShimBiasUpdate(bias: *mut RawBias, g: *const f32, out: *mut f32);
    fn ShimBiasGetOffset(bias: *const RawBias, out: *mut f32);

    fn ShimCompass(a: *const f32, m: *const f32, convention: u32) -> f32;
    fn ShimRemap(sensor: *const f32, alignment: u32, out: *mut f32);
    fn ShimModelInertial(
        uncalibrated: *const f32,
        misalignment: *const f32,
        sensitivity: *const f32,
        offset: *const f32,
        out: *mut f32,
    );
    fn ShimModelMagnetic(
        uncalibrated: *const f32,
        soft_iron_matrix: *const f32,
        hard_iron_offset: *const f32,
        out: *mut f32,
    );
    fn ShimConventionToString(convention: u32) -> *const c_char;
    fn ShimRemapAlignmentToString(alignment: u32) -> *const c_char;
}

/// Owned C `FusionAhrs` instance.
pub struct Ahrs(*mut RawAhrs);

impl Ahrs {
    /// `FusionAhrsInitialise` followed by `FusionAhrsSetSettings`.
    pub fn new(settings: &AhrsSettings) -> Self {
        Self(unsafe { ShimAhrsNew(settings) })
    }

    pub fn set_settings(&mut self, settings: &AhrsSettings) {
        unsafe { ShimAhrsSetSettings(self.0, settings) }
    }

    pub fn set_sample_period(&mut self, sample_period: f32) {
        unsafe { ShimAhrsSetSamplePeriod(self.0, sample_period) }
    }

    pub fn restart(&mut self) {
        unsafe { ShimAhrsRestart(self.0) }
    }

    pub fn skip_startup(&mut self) {
        unsafe { ShimAhrsSkipStartup(self.0) }
    }

    pub fn update(&mut self, gyroscope: [f32; 3], accelerometer: [f32; 3], magnetometer: [f32; 3]) {
        unsafe {
            ShimAhrsUpdate(
                self.0,
                gyroscope.as_ptr(),
                accelerometer.as_ptr(),
                magnetometer.as_ptr(),
            )
        }
    }

    pub fn update_no_magnetometer(&mut self, gyroscope: [f32; 3], accelerometer: [f32; 3]) {
        unsafe { ShimAhrsUpdateNoMagnetometer(self.0, gyroscope.as_ptr(), accelerometer.as_ptr()) }
    }

    pub fn update_external_heading(
        &mut self,
        gyroscope: [f32; 3],
        accelerometer: [f32; 3],
        heading: f32,
    ) {
        unsafe {
            ShimAhrsUpdateExternalHeading(
                self.0,
                gyroscope.as_ptr(),
                accelerometer.as_ptr(),
                heading,
            )
        }
    }

    /// Quaternion as w, x, y, z.
    pub fn set_quaternion(&mut self, quaternion: [f32; 4]) {
        unsafe { ShimAhrsSetQuaternion(self.0, quaternion.as_ptr()) }
    }

    pub fn set_heading(&mut self, heading: f32) {
        unsafe { ShimAhrsSetHeading(self.0, heading) }
    }

    pub fn outputs(&self) -> AhrsOutputs {
        let mut outputs = AhrsOutputs::default();
        unsafe { ShimAhrsOutputs(self.0, &mut outputs) };
        outputs
    }
}

impl Drop for Ahrs {
    fn drop(&mut self) {
        unsafe { ShimAhrsFree(self.0) }
    }
}

/// Owned C `FusionBias` instance.
pub struct Bias(*mut RawBias);

impl Bias {
    /// `FusionBiasInitialise` followed by `FusionBiasSetSettings`.
    pub fn new(settings: &BiasSettings) -> Self {
        Self(unsafe { ShimBiasNew(settings) })
    }

    pub fn update(&mut self, gyroscope: [f32; 3]) -> [f32; 3] {
        let mut out = [0.0; 3];
        unsafe { ShimBiasUpdate(self.0, gyroscope.as_ptr(), out.as_mut_ptr()) };
        out
    }

    pub fn offset(&self) -> [f32; 3] {
        let mut out = [0.0; 3];
        unsafe { ShimBiasGetOffset(self.0, out.as_mut_ptr()) };
        out
    }
}

impl Drop for Bias {
    fn drop(&mut self) {
        unsafe { ShimBiasFree(self.0) }
    }
}

/// `FusionCompass`. Convention: 0 = NWU, 1 = ENU, 2 = NED.
pub fn compass(accelerometer: [f32; 3], magnetometer: [f32; 3], convention: u32) -> f32 {
    unsafe { ShimCompass(accelerometer.as_ptr(), magnetometer.as_ptr(), convention) }
}

/// `FusionRemap`. `alignment` is the `FusionRemapAlignment` enum value (0..24).
pub fn remap(sensor: [f32; 3], alignment: u32) -> [f32; 3] {
    let mut out = [0.0; 3];
    unsafe { ShimRemap(sensor.as_ptr(), alignment, out.as_mut_ptr()) };
    out
}

/// `FusionModelInertial`. `misalignment` is row-major.
pub fn model_inertial(
    uncalibrated: [f32; 3],
    misalignment: [f32; 9],
    sensitivity: [f32; 3],
    offset: [f32; 3],
) -> [f32; 3] {
    let mut out = [0.0; 3];
    unsafe {
        ShimModelInertial(
            uncalibrated.as_ptr(),
            misalignment.as_ptr(),
            sensitivity.as_ptr(),
            offset.as_ptr(),
            out.as_mut_ptr(),
        )
    };
    out
}

/// `FusionModelMagnetic`. `soft_iron_matrix` is row-major.
pub fn model_magnetic(
    uncalibrated: [f32; 3],
    soft_iron_matrix: [f32; 9],
    hard_iron_offset: [f32; 3],
) -> [f32; 3] {
    let mut out = [0.0; 3];
    unsafe {
        ShimModelMagnetic(
            uncalibrated.as_ptr(),
            soft_iron_matrix.as_ptr(),
            hard_iron_offset.as_ptr(),
            out.as_mut_ptr(),
        )
    };
    out
}

/// `FusionConventionToString`.
pub fn convention_to_string(convention: u32) -> &'static str {
    unsafe { CStr::from_ptr(ShimConventionToString(convention)) }
        .to_str()
        .unwrap()
}

/// `FusionRemapAlignmentToString`.
pub fn remap_alignment_to_string(alignment: u32) -> &'static str {
    unsafe { CStr::from_ptr(ShimRemapAlignmentToString(alignment)) }
        .to_str()
        .unwrap()
}
