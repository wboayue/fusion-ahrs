use fusion_ahrs::Ahrs;
use nalgebra::Vector3;
use plotters::prelude::*;
use serde::Deserialize;
use std::error::Error;

#[derive(Debug, Deserialize)]
struct SensorData {
    #[serde(rename = "Time (s)")]
    time: f32,
    #[serde(rename = "Gyroscope X (deg/s)")]
    gyro_x: f32,
    #[serde(rename = "Gyroscope Y (deg/s)")]
    gyro_y: f32,
    #[serde(rename = "Gyroscope Z (deg/s)")]
    gyro_z: f32,
    #[serde(rename = "Accelerometer X (g)")]
    accel_x: f32,
    #[serde(rename = "Accelerometer Y (g)")]
    accel_y: f32,
    #[serde(rename = "Accelerometer Z (g)")]
    accel_z: f32,
    #[serde(rename = "Magnetometer X (uT)")]
    #[allow(dead_code)]
    mag_x: f32,
    #[serde(rename = "Magnetometer Y (uT)")]
    #[allow(dead_code)]
    mag_y: f32,
    #[serde(rename = "Magnetometer Z (uT)")]
    #[allow(dead_code)]
    mag_z: f32,
}

const SAMPLE_RATE: f32 = 100.0; // 100 Hz

fn main() -> Result<(), Box<dyn Error>> {
    // Load sensor data from CSV
    let mut reader = csv::Reader::from_path("testdata/sensor_data.csv")?;
    let mut sensor_data = Vec::new();

    for result in reader.deserialize() {
        let record: SensorData = result?;
        sensor_data.push(record);
    }

    // Process sensor data
    let mut ahrs = Ahrs::new();
    let mut euler_angles = Vec::new();

    for data in &sensor_data {
        let gyroscope = Vector3::new(data.gyro_x, data.gyro_y, data.gyro_z);
        let accelerometer = Vector3::new(data.accel_x, data.accel_y, data.accel_z);

        ahrs.update_no_magnetometer(gyroscope, accelerometer, 1.0 / SAMPLE_RATE);

        let quaternion = ahrs.quaternion();
        let (roll, pitch, yaw) = quaternion.euler_angles();

        euler_angles.push((roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees()));
    }

    // Generate plots
    create_plots(&sensor_data, &euler_angles)?;

    println!("Plots saved to sensor_plots.png");
    Ok(())
}

fn create_plots(
    sensor_data: &[SensorData],
    euler_angles: &[(f32, f32, f32)],
) -> Result<(), Box<dyn Error>> {
    let root = BitMapBackend::new("sensor_plots.png", (800, 900)).into_drawing_area();
    root.fill(&WHITE)?;

    let upper = root.split_evenly((3, 1));

    // Plot gyroscope data
    let mut gyro_chart = ChartBuilder::on(&upper[0])
        .caption("Gyroscope", ("sans-serif", 30))
        .margin(10)
        .x_label_area_size(40)
        .y_label_area_size(50)
        .build_cartesian_2d(
            sensor_data[0].time..sensor_data.last().unwrap().time,
            -200f32..200f32,
        )?;

    gyro_chart
        .configure_mesh()
        .x_desc("Time (s)")
        .y_desc("Degrees/s")
        .draw()?;

    gyro_chart
        .draw_series(LineSeries::new(
            sensor_data.iter().map(|d| (d.time, d.gyro_x)),
            &RED,
        ))?
        .label("X")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], RED));

    gyro_chart
        .draw_series(LineSeries::new(
            sensor_data.iter().map(|d| (d.time, d.gyro_y)),
            &GREEN,
        ))?
        .label("Y")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], GREEN));

    gyro_chart
        .draw_series(LineSeries::new(
            sensor_data.iter().map(|d| (d.time, d.gyro_z)),
            &BLUE,
        ))?
        .label("Z")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], BLUE));

    gyro_chart.configure_series_labels().draw()?;

    // Plot accelerometer data
    let mut accel_chart = ChartBuilder::on(&upper[1])
        .caption("Accelerometer", ("sans-serif", 30))
        .margin(10)
        .x_label_area_size(40)
        .y_label_area_size(50)
        .build_cartesian_2d(
            sensor_data[0].time..sensor_data.last().unwrap().time,
            -2f32..2f32,
        )?;

    accel_chart
        .configure_mesh()
        .x_desc("Time (s)")
        .y_desc("g")
        .draw()?;

    accel_chart
        .draw_series(LineSeries::new(
            sensor_data.iter().map(|d| (d.time, d.accel_x)),
            &RED,
        ))?
        .label("X")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], RED));

    accel_chart
        .draw_series(LineSeries::new(
            sensor_data.iter().map(|d| (d.time, d.accel_y)),
            &GREEN,
        ))?
        .label("Y")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], GREEN));

    accel_chart
        .draw_series(LineSeries::new(
            sensor_data.iter().map(|d| (d.time, d.accel_z)),
            &BLUE,
        ))?
        .label("Z")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], BLUE));

    accel_chart.configure_series_labels().draw()?;

    // Plot Euler angles
    let mut euler_chart = ChartBuilder::on(&upper[2])
        .caption("Euler Angles", ("sans-serif", 30))
        .margin(10)
        .x_label_area_size(40)
        .y_label_area_size(50)
        .build_cartesian_2d(
            sensor_data[0].time..sensor_data.last().unwrap().time,
            -180f32..180f32,
        )?;

    euler_chart
        .configure_mesh()
        .x_desc("Time (s)")
        .y_desc("Degrees")
        .draw()?;

    euler_chart
        .draw_series(LineSeries::new(
            sensor_data
                .iter()
                .zip(euler_angles.iter())
                .map(|(d, e)| (d.time, e.0)),
            &RED,
        ))?
        .label("Roll")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], RED));

    euler_chart
        .draw_series(LineSeries::new(
            sensor_data
                .iter()
                .zip(euler_angles.iter())
                .map(|(d, e)| (d.time, e.1)),
            &GREEN,
        ))?
        .label("Pitch")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], GREEN));

    euler_chart
        .draw_series(LineSeries::new(
            sensor_data
                .iter()
                .zip(euler_angles.iter())
                .map(|(d, e)| (d.time, e.2)),
            &BLUE,
        ))?
        .label("Yaw")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], BLUE));

    euler_chart.configure_series_labels().draw()?;

    root.present()?;
    Ok(())
}
