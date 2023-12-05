use clap::{Arg, Command};
use ydlidar_driver::run_driver;

fn get_port_name() -> String {
    let matches = Command::new("LiDAR data receiver.")
        .about("Reads data from LiDAR.")
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

fn main() {
    let port_name = get_port_name();

    let (driver_threads, scan_rx) = run_driver(&port_name);

    for _ in 0..100 {
        let scan = scan_rx.recv().unwrap();
        println!("Received {:03} samples.", scan.distances.len());
    }

    drop(driver_threads);
}
