use criterion::{Criterion, criterion_group, criterion_main};
use fusion_ahrs::{Ahrs, Vector};
use rand::prelude::*;
use rand_pcg::Pcg64;
use std::f32::consts::PI;
use std::hint::black_box;

// Pre-generated sensor data to eliminate RNG overhead during benchmarks
struct PreGeneratedData {
    samples: Vec<(Vector, Vector, Vector)>,
    index: usize,
}

impl PreGeneratedData {
    fn new(count: usize, seed: u64) -> Self {
        let mut rng = Pcg64::seed_from_u64(seed);
        let mut samples = Vec::with_capacity(count);

        for i in 0..count {
            let time = i as f32 * 0.01; // 100Hz sample rate

            // Generate realistic motion patterns without per-sample RNG overhead
            let motion_phase = time * 0.5 * 2.0 * PI;

            let gyroscope = Vector::new(
                0.2 * motion_phase.sin() + rng.random_range(-0.01..0.01),
                0.2 * (motion_phase * 1.3).cos() + rng.random_range(-0.01..0.01),
                0.2 * (motion_phase * 0.7).sin() + rng.random_range(-0.01..0.01),
            );

            let accelerometer = Vector::new(
                -0.1 * motion_phase.sin() + rng.random_range(-0.002..0.002),
                0.1 * motion_phase.cos() + rng.random_range(-0.002..0.002),
                1.0 + rng.random_range(-0.002..0.002),
            );

            let magnetometer = Vector::new(
                0.6 + 0.05 * motion_phase.cos() + rng.random_range(-0.05..0.05),
                0.05 * motion_phase.sin() + rng.random_range(-0.05..0.05),
                -0.8 + rng.random_range(-0.05..0.05),
            );

            samples.push((gyroscope, accelerometer, magnetometer));
        }

        Self { samples, index: 0 }
    }

    fn next(&mut self) -> (Vector, Vector, Vector) {
        let sample = self.samples[self.index];
        self.index = (self.index + 1) % self.samples.len();
        sample
    }
}

/// Benchmark the main AHRS update method with all sensors
fn bench_update(c: &mut Criterion) {
    let mut ahrs = Ahrs::new();
    let mut data = PreGeneratedData::new(1000, 42);

    c.bench_function("ahrs_update", |b| {
        b.iter(|| {
            let (gyroscope, accelerometer, magnetometer) = data.next();
            ahrs.update(
                black_box(gyroscope),
                black_box(accelerometer),
                black_box(magnetometer),
            )
        })
    });
}

/// Benchmark the AHRS update method without magnetometer
fn bench_update_no_magnetometer(c: &mut Criterion) {
    let mut ahrs = Ahrs::new();
    let mut data = PreGeneratedData::new(1000, 42);

    c.bench_function("ahrs_update_no_magnetometer", |b| {
        b.iter(|| {
            let (gyroscope, accelerometer, _) = data.next();
            ahrs.update_no_magnetometer(black_box(gyroscope), black_box(accelerometer))
        })
    });
}

/// Benchmark AHRS update during initialization phase (higher gain)
fn bench_update_initialization(c: &mut Criterion) {
    let mut data = PreGeneratedData::new(1000, 42);

    c.bench_function("ahrs_update_initialization", |b| {
        b.iter(|| {
            // Create fresh AHRS for each iteration to stay in initialization mode
            let mut ahrs = Ahrs::new();
            let (gyroscope, accelerometer, magnetometer) = data.next();
            ahrs.update(
                black_box(gyroscope),
                black_box(accelerometer),
                black_box(magnetometer),
            )
        })
    });
}

/// Benchmark AHRS update during steady state (after initialization)
fn bench_update_steady_state(c: &mut Criterion) {
    let mut ahrs = Ahrs::new();
    let mut data = PreGeneratedData::new(1000, 42);

    // Complete initialization by running for several seconds
    for _ in 0..400 {
        let (gyroscope, accelerometer, magnetometer) = data.next();
        ahrs.update(gyroscope, accelerometer, magnetometer);
    }

    c.bench_function("ahrs_update_steady_state", |b| {
        b.iter(|| {
            let (gyroscope, accelerometer, magnetometer) = data.next();
            ahrs.update(
                black_box(gyroscope),
                black_box(accelerometer),
                black_box(magnetometer),
            )
        })
    });
}

/// Benchmark batch processing of sensor updates
fn bench_batch_updates(c: &mut Criterion) {
    let mut ahrs = Ahrs::new();
    let mut data = PreGeneratedData::new(1000, 42);

    c.bench_function("ahrs_batch_100_updates", |b| {
        b.iter(|| {
            for _ in 0..100 {
                let (gyroscope, accelerometer, magnetometer) = data.next();
                ahrs.update(
                    black_box(gyroscope),
                    black_box(accelerometer),
                    black_box(magnetometer),
                )
            }
        })
    });
}

/// Benchmark AHRS creation and initialization
fn bench_ahrs_creation(c: &mut Criterion) {
    c.bench_function("ahrs_new", |b| b.iter(|| black_box(Ahrs::new())));
}

/// Benchmark quaternion retrieval
fn bench_quaternion_access(c: &mut Criterion) {
    let ahrs = Ahrs::new();

    c.bench_function("ahrs_quaternion", |b| {
        b.iter(|| black_box(ahrs.quaternion()))
    });
}

/// Benchmark gravity calculation
fn bench_gravity_calculation(c: &mut Criterion) {
    let ahrs = Ahrs::new();

    c.bench_function("ahrs_gravity", |b| b.iter(|| black_box(ahrs.gravity())));
}

/// Benchmark linear acceleration calculation
fn bench_linear_acceleration(c: &mut Criterion) {
    let mut ahrs = Ahrs::new();
    let mut data = PreGeneratedData::new(1000, 42);

    // Update once to set accelerometer reading
    let (gyroscope, accelerometer, magnetometer) = data.next();
    ahrs.update(gyroscope, accelerometer, magnetometer);

    c.bench_function("ahrs_linear_acceleration", |b| {
        b.iter(|| black_box(ahrs.linear_acceleration()))
    });
}

criterion_group!(
    benches,
    bench_update,
    bench_update_no_magnetometer,
    bench_update_initialization,
    bench_update_steady_state,
    bench_batch_updates,
    bench_ahrs_creation,
    bench_quaternion_access,
    bench_gravity_calculation,
    bench_linear_acceleration
);

criterion_main!(benches);
