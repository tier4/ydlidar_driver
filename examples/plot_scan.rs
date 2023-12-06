use clap::{Arg, Command};
use piston_window::{EventLoop, PistonWindow, WindowSettings};
use plotters::drawing::IntoDrawingArea;
use plotters::prelude::{ChartBuilder, Circle, GREEN, WHITE};
use plotters::style::Color;
use plotters_piston::{draw_piston_window, PistonBackend};
use ydlidar_driver::run_driver;

fn get_port_name() -> String {
    let matches = Command::new("LiDAR data receiver.")
        .about("Reads data from LiDAR and plot scan.")
        .disable_version_flag(true)
        .arg(
            Arg::new("port")
                .help("The device path to a serial port")
                .use_value_delimiter(false)
                .required(true),
        )
        .get_matches();

    let port_name = matches.value_of("port").unwrap();
    port_name.to_string()
}

const WINDOW_RANGE: f64 = 4000.;
const FPS: u64 = 60;
fn main() {
    let port_name = get_port_name();
    let (driver_threads, scan_rx) = run_driver(&port_name);

    let mut window: PistonWindow = WindowSettings::new("LiDAR scan", [800, 800])
        .build()
        .unwrap();

    window.set_max_fps(FPS);
    let draw = |b: PistonBackend| {
        let scan = match scan_rx.try_recv() {
            Ok(s) => s,
            Err(_) => return Ok(()),
        };
        println!("Received {} points.", scan.angles_radian.len());

        let root = b.into_drawing_area();
        root.fill(&WHITE)?;

        let mut cc = ChartBuilder::on(&root)
            .build_cartesian_2d(-WINDOW_RANGE..WINDOW_RANGE, -WINDOW_RANGE..WINDOW_RANGE)?;

        let circles = scan
            .angles_radian
            .iter()
            .zip(scan.distances.iter())
            .map(|(w, d)| {
                let x = (*d as f64) * f64::cos(*w);
                let y = (*d as f64) * f64::sin(*w);
                Circle::new((x, y), 2, GREEN.filled())
            });
        cc.draw_series(circles)?;

        Ok(())
    };

    while let Some(_) = draw_piston_window(&mut window, draw) {}
    drop(driver_threads);
}
