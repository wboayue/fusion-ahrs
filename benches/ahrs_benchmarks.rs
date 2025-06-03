use criterion::{black_box, criterion_group, criterion_main, Criterion};
use fusion_ahrs::Ahrs;
use nalgebra::Vector3;

/// Generate realistic sensor data for benchmarking
fn generate_sensor_data() -> (Vector3<f32>, Vector3<f32>, Vector3<f32>) {
    // Realistic gyroscope data (small rotation rates in deg/s)
    let gyroscope = Vector3::new(0.1, -0.2, 0.05);
    
    // Realistic accelerometer data (gravity + some noise in g)
    let accelerometer = Vector3::new(0.01, -0.02, 1.0);
    
    // Realistic magnetometer data (Earth's magnetic field in µT)
    let magnetometer = Vector3::new(25.0, 2.0, -15.0);
    
    (gyroscope, accelerometer, magnetometer)
}

/// Benchmark the main AHRS update method with all sensors
fn bench_update(c: &mut Criterion) {
    let mut ahrs = Ahrs::new();
    let (gyroscope, accelerometer, magnetometer) = generate_sensor_data();
    let delta_time = 0.01f32; // 10ms (100Hz)
    
    c.bench_function("ahrs_update", |b| {
        b.iter(|| {
            ahrs.update(
                black_box(gyroscope),
                black_box(accelerometer), 
                black_box(magnetometer),
                black_box(delta_time)
            )
        })
    });
}

/// Benchmark the AHRS update method without magnetometer
fn bench_update_no_magnetometer(c: &mut Criterion) {
    let mut ahrs = Ahrs::new();
    let (gyroscope, accelerometer, _) = generate_sensor_data();
    let delta_time = 0.01f32; // 10ms (100Hz)
    
    c.bench_function("ahrs_update_no_magnetometer", |b| {
        b.iter(|| {
            ahrs.update_no_magnetometer(
                black_box(gyroscope),
                black_box(accelerometer),
                black_box(delta_time)
            )
        })
    });
}

/// Benchmark AHRS update during initialization phase (higher gain)
fn bench_update_initialization(c: &mut Criterion) {
    let (gyroscope, accelerometer, magnetometer) = generate_sensor_data();
    let delta_time = 0.01f32; // 10ms (100Hz)
    
    c.bench_function("ahrs_update_initialization", |b| {
        b.iter(|| {
            // Create fresh AHRS for each iteration to stay in initialization mode
            let mut ahrs = Ahrs::new();
            ahrs.update(
                black_box(gyroscope),
                black_box(accelerometer),
                black_box(magnetometer),
                black_box(delta_time)
            )
        })
    });
}

/// Benchmark AHRS update during steady state (after initialization)
fn bench_update_steady_state(c: &mut Criterion) {
    let mut ahrs = Ahrs::new();
    let (gyroscope, accelerometer, magnetometer) = generate_sensor_data();
    let delta_time = 0.01f32; // 10ms (100Hz)
    
    // Complete initialization by running for several seconds
    for _ in 0..400 {
        ahrs.update(gyroscope, accelerometer, magnetometer, delta_time);
    }
    
    c.bench_function("ahrs_update_steady_state", |b| {
        b.iter(|| {
            ahrs.update(
                black_box(gyroscope),
                black_box(accelerometer),
                black_box(magnetometer),
                black_box(delta_time)
            )
        })
    });
}

/// Benchmark batch processing of sensor updates
fn bench_batch_updates(c: &mut Criterion) {
    let mut ahrs = Ahrs::new();
    let (gyroscope, accelerometer, magnetometer) = generate_sensor_data();
    let delta_time = 0.01f32; // 10ms (100Hz)
    
    c.bench_function("ahrs_batch_100_updates", |b| {
        b.iter(|| {
            for _ in 0..100 {
                ahrs.update(
                    black_box(gyroscope),
                    black_box(accelerometer),
                    black_box(magnetometer),
                    black_box(delta_time)
                )
            }
        })
    });
}

/// Benchmark AHRS creation and initialization
fn bench_ahrs_creation(c: &mut Criterion) {
    c.bench_function("ahrs_new", |b| {
        b.iter(|| {
            black_box(Ahrs::new())
        })
    });
}

/// Benchmark quaternion retrieval
fn bench_quaternion_access(c: &mut Criterion) {
    let ahrs = Ahrs::new();
    
    c.bench_function("ahrs_quaternion", |b| {
        b.iter(|| {
            black_box(ahrs.quaternion())
        })
    });
}

/// Benchmark gravity calculation
fn bench_gravity_calculation(c: &mut Criterion) {
    let ahrs = Ahrs::new();
    
    c.bench_function("ahrs_gravity", |b| {
        b.iter(|| {
            black_box(ahrs.gravity())
        })
    });
}

/// Benchmark linear acceleration calculation
fn bench_linear_acceleration(c: &mut Criterion) {
    let mut ahrs = Ahrs::new();
    let (gyroscope, accelerometer, magnetometer) = generate_sensor_data();
    
    // Update once to set accelerometer reading
    ahrs.update(gyroscope, accelerometer, magnetometer, 0.01);
    
    c.bench_function("ahrs_linear_acceleration", |b| {
        b.iter(|| {
            black_box(ahrs.linear_acceleration())
        })
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