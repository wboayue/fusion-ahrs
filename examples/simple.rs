use fusion_ahrs::FusionAhrs;
use nalgebra::Vector3;

const SAMPLE_PERIOD: f32 = 0.01; // 10 ms sample period

fn main() {
    let mut ahrs = FusionAhrs::new();

    for _ in 0..10 {
        // this loop should repeat each time new gyroscope data is available
        let gyroscope = Vector3::new(0.0, 0.0, 0.0); // replace this with actual gyroscope data in degrees/s
        let accelerometer = Vector3::new(0.0, 0.0, 1.0); // replace this with actual accelerometer data in g

        ahrs.update_no_magnetometer(gyroscope, accelerometer, SAMPLE_PERIOD);

        let quaternion = ahrs.get_quaternion();
        let (roll, pitch, yaw) = quaternion.euler_angles();

        println!(
            "Roll: {:.2}, Pitch: {:.2}, Yaw: {:.2}",
            roll.to_degrees(),
            pitch.to_degrees(),
            yaw.to_degrees()
        );
    }
}
