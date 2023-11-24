use clap::{Arg, Command};
use ydlidar_driver::run_driver;

fn sleep_ms(duration: u64) {
    std::thread::sleep(std::time::Duration::from_millis(duration));
}

fn main() {
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

    let (mut driver_threads, scan_rx) = run_driver(port_name);

    for i in 0..200 {
        match scan_rx.try_recv() {
            Ok(scan) => println!("Received {} scan samples.", scan.angles_radian.len()),
            Err(_) => sleep_ms(10),
        }
    }

    drop(driver_threads);
}
