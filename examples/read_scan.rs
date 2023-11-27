use std::fs::File;
use std::io::Write;

use clap::{Arg, Command};
use ydlidar_driver::{run_driver, Scan};

fn write(filename: String, scan: Scan) {
    assert_eq!(scan.angles_radian.len(), scan.distances.len());
    assert_eq!(scan.angles_radian.len(), scan.flags.len());
    assert_eq!(scan.angles_radian.len(), scan.intensities.len());

    let mut f = File::create(filename).unwrap();
    for i in 0..scan.angles_radian.len() {
        let w = scan.angles_radian[i];
        let d = scan.distances[i];
        f.write_all(format!("{} {}\n", w, d).as_bytes()).unwrap();
    }
}

fn get_port_name() -> String {
    let matches = Command::new("Serialport Example - Receive Data")
        .about("Reads data from a serial port and echoes it to stdout")
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

    for i in 0..10 {
        let scan = scan_rx.recv().unwrap();
        write(format!("scan/{:03}.txt", i), scan);
    }

    drop(driver_threads);
}
