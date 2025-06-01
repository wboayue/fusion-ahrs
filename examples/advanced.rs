use fusion_ahrs::{Ahrs, Offset, AhrsSettings, Convention, OffsetSettings};
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
    mag_x: f32,
    #[serde(rename = "Magnetometer Y (uT)")]
    mag_y: f32,
    #[serde(rename = "Magnetometer Z (uT)")]
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

    // Instantiate algorithms
    let mut offset = Offset::new(OffsetSettings::default(), SAMPLE_RATE);
    let mut ahrs = Ahrs::new();

    // Configure AHRS settings to match Python example
    let settings = AhrsSettings {
        convention: Convention::Nwu,
        gain: 0.5,
        gyroscope_range: 2000.0,
        acceleration_rejection: 10.0,
        magnetic_rejection: 10.0,
        recovery_trigger_period: (5.0 * SAMPLE_RATE) as u32, // 5 seconds
    };
    ahrs.set_settings(settings);

    // Process sensor data
    let mut euler_angles = Vec::new();
    let mut internal_states = Vec::new();
    let mut flags = Vec::new();
    let mut delta_times = Vec::new();

    // Calculate delta times
    for i in 0..sensor_data.len() {
        let delta_time = if i == 0 {
            sensor_data[0].time
        } else {
            sensor_data[i].time - sensor_data[i-1].time
        };
        delta_times.push(delta_time);
    }

    for (i, data) in sensor_data.iter().enumerate() {
        let mut gyroscope = Vector3::new(data.gyro_x, data.gyro_y, data.gyro_z);
        let accelerometer = Vector3::new(data.accel_x, data.accel_y, data.accel_z);
        let magnetometer = Vector3::new(data.mag_x, data.mag_y, data.mag_z);

        // Apply offset correction
        gyroscope = offset.update(gyroscope);

        // Update AHRS with magnetometer
        ahrs.update(gyroscope, accelerometer, magnetometer, delta_times[i]);

        // Get results
        let quaternion = ahrs.quaternion();
        let (roll, pitch, yaw) = quaternion.euler_angles();
        
        euler_angles.push((
            roll.to_degrees(),
            pitch.to_degrees(), 
            yaw.to_degrees()
        ));

        // Get internal states
        let states = ahrs.internal_states();
        internal_states.push((
            states.acceleration_error,
            if states.accelerometer_ignored { 1.0 } else { 0.0 },
            states.acceleration_recovery_trigger,
            states.magnetic_error,
            if states.magnetometer_ignored { 1.0 } else { 0.0 },
            states.magnetic_recovery_trigger,
        ));

        // Get flags
        let ahrs_flags = ahrs.flags();
        flags.push((
            if ahrs_flags.initialising { 1.0 } else { 0.0 },
            if ahrs_flags.angular_rate_recovery { 1.0 } else { 0.0 },
            if ahrs_flags.acceleration_recovery { 1.0 } else { 0.0 },
            if ahrs_flags.magnetic_recovery { 1.0 } else { 0.0 },
        ));
    }

    // Generate plots
    create_advanced_plots(&sensor_data, &euler_angles, &internal_states, &flags)?;
    
    println!("Advanced plots saved to advanced_plots.png");
    Ok(())
}

fn create_advanced_plots(
    sensor_data: &[SensorData], 
    euler_angles: &[(f32, f32, f32)],
    internal_states: &[(f32, f32, f32, f32, f32, f32)],
    flags: &[(f32, f32, f32, f32)]
) -> Result<(), Box<dyn Error>> {
    let root = BitMapBackend::new("advanced_plots.png", (1000, 1400)).into_drawing_area();
    root.fill(&WHITE)?;
    
    // Split into 11 rows with height ratios similar to Python
    let height_ratios = [6, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1];
    let total_height: u32 = height_ratios.iter().sum();
    let mut y_offset = 0;
    let mut charts = Vec::new();
    
    for &ratio in &height_ratios {
        let height = (ratio * 1400) / total_height;
        let area = root.margin(10, 10, y_offset, 1400 - y_offset - height);
        charts.push(area);
        y_offset += height;
    }

    let time_range = sensor_data[0].time..sensor_data.last().unwrap().time;

    // 1. Euler angles
    let mut euler_chart = ChartBuilder::on(&charts[0])
        .caption("Euler Angles, Internal States, and Flags", ("sans-serif", 20))
        .margin(5)
        .x_label_area_size(0)
        .y_label_area_size(50)
        .build_cartesian_2d(time_range.clone(), -180f32..180f32)?;

    euler_chart.configure_mesh()
        .y_desc("Degrees")
        .draw()?;

    euler_chart.draw_series(LineSeries::new(
        sensor_data.iter().zip(euler_angles.iter()).map(|(d, e)| (d.time, e.0)),
        &RED,
    ))?.label("Roll").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], RED));

    euler_chart.draw_series(LineSeries::new(
        sensor_data.iter().zip(euler_angles.iter()).map(|(d, e)| (d.time, e.1)),
        &GREEN,
    ))?.label("Pitch").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], GREEN));

    euler_chart.draw_series(LineSeries::new(
        sensor_data.iter().zip(euler_angles.iter()).map(|(d, e)| (d.time, e.2)),
        &BLUE,
    ))?.label("Yaw").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], BLUE));

    euler_chart.configure_series_labels().draw()?;

    // 2. Initialising flag
    create_bool_plot(&charts[1], sensor_data, &flags.iter().map(|f| f.0).collect::<Vec<_>>(), "Initialising", time_range.clone())?;

    // 3. Angular rate recovery flag
    create_bool_plot(&charts[2], sensor_data, &flags.iter().map(|f| f.1).collect::<Vec<_>>(), "Angular rate recovery", time_range.clone())?;

    // 4. Acceleration error
    let mut accel_error_chart = ChartBuilder::on(&charts[3])
        .margin(5)
        .x_label_area_size(0)
        .y_label_area_size(50)
        .build_cartesian_2d(time_range.clone(), 0f32..180f32)?;

    accel_error_chart.configure_mesh()
        .y_desc("Degrees")
        .draw()?;

    accel_error_chart.draw_series(LineSeries::new(
        sensor_data.iter().zip(internal_states.iter()).map(|(d, s)| (d.time, s.0)),
        &RGBColor(128, 128, 0), // olive
    ))?.label("Acceleration error").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], RGBColor(128, 128, 0)));

    accel_error_chart.configure_series_labels().draw()?;

    // 5. Accelerometer ignored
    create_bool_plot(&charts[4], sensor_data, &internal_states.iter().map(|s| s.1).collect::<Vec<_>>(), "Accelerometer ignored", time_range.clone())?;

    // 6. Acceleration recovery trigger
    let mut accel_trigger_chart = ChartBuilder::on(&charts[5])
        .margin(5)
        .x_label_area_size(0)
        .y_label_area_size(50)
        .build_cartesian_2d(time_range.clone(), 0f32..500f32)?;

    accel_trigger_chart.configure_mesh().draw()?;

    accel_trigger_chart.draw_series(LineSeries::new(
        sensor_data.iter().zip(internal_states.iter()).map(|(d, s)| (d.time, s.2)),
        &RGBColor(255, 165, 0), // orange
    ))?.label("Acceleration recovery trigger").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], RGBColor(255, 165, 0)));

    accel_trigger_chart.configure_series_labels().draw()?;

    // 7. Acceleration recovery flag
    create_bool_plot(&charts[6], sensor_data, &flags.iter().map(|f| f.2).collect::<Vec<_>>(), "Acceleration recovery", time_range.clone())?;

    // 8. Magnetic error
    let mut mag_error_chart = ChartBuilder::on(&charts[7])
        .margin(5)
        .x_label_area_size(0)
        .y_label_area_size(50)
        .build_cartesian_2d(time_range.clone(), 0f32..180f32)?;

    mag_error_chart.configure_mesh()
        .y_desc("Degrees")
        .draw()?;

    mag_error_chart.draw_series(LineSeries::new(
        sensor_data.iter().zip(internal_states.iter()).map(|(d, s)| (d.time, s.3)),
        &RGBColor(128, 128, 0), // olive
    ))?.label("Magnetic error").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], RGBColor(128, 128, 0)));

    mag_error_chart.configure_series_labels().draw()?;

    // 9. Magnetometer ignored
    create_bool_plot(&charts[8], sensor_data, &internal_states.iter().map(|s| s.4).collect::<Vec<_>>(), "Magnetometer ignored", time_range.clone())?;

    // 10. Magnetic recovery trigger
    let mut mag_trigger_chart = ChartBuilder::on(&charts[9])
        .margin(5)
        .x_label_area_size(0)
        .y_label_area_size(50)
        .build_cartesian_2d(time_range.clone(), 0f32..500f32)?;

    mag_trigger_chart.configure_mesh().draw()?;

    mag_trigger_chart.draw_series(LineSeries::new(
        sensor_data.iter().zip(internal_states.iter()).map(|(d, s)| (d.time, s.5)),
        &RGBColor(255, 165, 0), // orange
    ))?.label("Magnetic recovery trigger").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], RGBColor(255, 165, 0)));

    mag_trigger_chart.configure_series_labels().draw()?;

    // 11. Magnetic recovery flag
    create_bool_plot(&charts[10], sensor_data, &flags.iter().map(|f| f.3).collect::<Vec<_>>(), "Magnetic recovery", time_range.clone())?;

    root.present()?;
    Ok(())
}

fn create_bool_plot(
    area: &DrawingArea<BitMapBackend, plotters::coord::Shift>,
    sensor_data: &[SensorData],
    values: &[f32],
    label: &str,
    time_range: std::ops::Range<f32>
) -> Result<(), Box<dyn Error>> {
    let mut chart = ChartBuilder::on(area)
        .margin(5)
        .x_label_area_size(if label == "Magnetic recovery" { 40 } else { 0 })
        .y_label_area_size(50)
        .build_cartesian_2d(time_range, -0.1f32..1.1f32)?;

    chart.configure_mesh()
        .y_desc("")
        .y_label_formatter(&|y| if *y < 0.5 { "False".to_string() } else { "True".to_string() })
        .draw()?;

    if label == "Magnetic recovery" {
        chart.configure_mesh().x_desc("Time (s)").draw()?;
    }

    chart.draw_series(LineSeries::new(
        sensor_data.iter().zip(values.iter()).map(|(d, &v)| (d.time, v)),
        &CYAN,
    ))?.label(label).legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], CYAN));

    chart.configure_series_labels().draw()?;
    Ok(())
}
