use std::io::{self, Write};
use std::time::Duration;
use std::{thread, time};

use serialport::SerialPort;

use clap::{Arg, Command};

const LIDAR_CMD_GET_DEVICE_HEALTH : u8 = 0x92;
const LIDAR_CMD_SYNC_BYTE : u8 = 0xA5;
const LIDAR_CMD_FORCE_STOP : u8 = 0x00;
const LIDAR_CMD_STOP : u8 = 0x65;
const LIDAR_CMD_SCAN : u8 = 0x60;

fn to_string(data: &[u8]) -> String {
    return data.iter().map(|e| format!("{:02x}", e)).collect::<Vec<_>>().join(" ");
}

fn send_data(port: &mut Box<dyn SerialPort>, data: &[u8]) {
    match port.write(data) {
        Ok(_) => {
            println!("Wrote {}", to_string(data));
            std::io::stdout().flush().unwrap();
        }
        Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
        Err(e) => eprintln!("{:?}", e),
    }
}

fn send_command(port: &mut Box<dyn SerialPort>, command: u8) {
    let data : [u8; 2] = [LIDAR_CMD_SYNC_BYTE, command];
    send_data(port, &data);  // We need to do a lot more
}

fn sleep_ms(duration: u64) {
    thread::sleep(time::Duration::from_millis(duration));
}

fn timeout_error(message: &str) -> io::Error {
    return io::Error::new(io::ErrorKind::TimedOut, message);
}

fn other_error(message: &str) -> io::Error {
    return io::Error::new(io::ErrorKind::Other, message);
}

fn read(port: &mut Box<dyn SerialPort>, data_size: usize, n_trials: u32) -> Result<Vec<u8>, io::Error> {
    for _ in 0..n_trials {
        let n_u32: u32 = port.bytes_to_read().unwrap();
        let n_read: usize = n_u32.try_into().unwrap();

        if n_read == 0 {
            sleep_ms(10);
            continue;
        }

        if n_read < data_size {
            let m = format!("Expected {} bytes but obtained {} bytes.", data_size, n_read);
            return Err(other_error(&m));
        }

        let mut serial_buf: Vec<u8> = vec![0; data_size];
        match port.read(serial_buf.as_mut_slice()) {
            Ok(n) => println!("Read {} bytes", n),
            Err(e) => return Err(e),
        }
        return Ok(serial_buf);
    }
    return Err(timeout_error("Operation timed out"));
}

fn get_device_health(port: &mut Box<dyn SerialPort>) {
    send_command(port, LIDAR_CMD_GET_DEVICE_HEALTH);
    let data = match read(port, 7, 10) {
        Ok(d) => d,
        Err(e) => panic!("{:?}", e),
    };
    println!("Received data = {}", to_string(&data));
}

fn stop_scan(port: &mut Box<dyn SerialPort>) {
    send_command(port, LIDAR_CMD_FORCE_STOP);
    sleep_ms(10);
    send_command(port, LIDAR_CMD_STOP);
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
    let baud_rate = 230400;

    let port = serialport::new(port_name, baud_rate)
        .timeout(Duration::from_millis(10))
        .open();

    match port {
        Ok(mut port) => {
            stop_scan(&mut port);
            get_device_health(&mut port);
        }
        Err(e) => {
            eprintln!("Failed to open \"{}\". Error: {}", port_name, e);
            ::std::process::exit(1);
        }
    }
}
