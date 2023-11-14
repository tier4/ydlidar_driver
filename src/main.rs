use std::io::{self, Write};
use std::time::Duration;
use std::{thread, time};
use std::sync::mpsc;

mod device_info;
mod ydlidar_models;

use serialport::SerialPort;

use clap::{Arg, Command};

const HEADER_SIZE : usize = 7;
const LIDAR_CMD_GET_DEVICE_HEALTH : u8 = 0x92;
const LIDAR_CMD_GET_DEVICE_INFO : u8 = 0x90;
const LIDAR_CMD_SYNC_BYTE : u8 = 0xA5;
const LIDAR_CMD_FORCE_STOP : u8 = 0x00;
const LIDAR_CMD_STOP : u8 = 0x65;
const LIDAR_CMD_SCAN : u8 = 0x60;
const LIDAR_ANS_TYPE_DEVINFO : u8 = 0x4;
const LIDAR_ANS_TYPE_DEVHEALTH : u8 = 0x6;
const LIDAR_ANS_TYPE_MEASUREMENT : u8 = 0x81;

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

fn flush(port: &mut Box<dyn SerialPort>) {
    let mut f = || -> () {
        let n_u32: u32 = port.bytes_to_read().unwrap();
        let n_read: usize = n_u32.try_into().unwrap();
        if n_read == 0 {
            return;
        }
        let mut serial_buf: Vec<u8> = vec![0; n_read];
        port.read(serial_buf.as_mut_slice()).unwrap();
    };

    for _ in 0..10 {
        f();
    }
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
            let m = format!("Tried to read {} bytes but obtained {} bytes.", data_size, n_read);
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

fn validate_response_header(header: &Vec<u8>, maybe_response_length: Option<u8>, type_code: u8) -> Result<(), String> {
    if header.len() != HEADER_SIZE {
        return Err(format!("Response header must be always seven bytes. Observed = {} bytes.", header.len()));
    }
    if header[0] != 0xA5 || header[1] != 0x5A {
        return Err(format!("Header sign must with 0xA55A. Observed = {}", to_string(&header[0..2])));
    }
    match maybe_response_length {
        None => (),
        Some(len) => {
            if header[2] != len {
                return Err(format!("Expected response length of {} bytes. Observed = {}", len, header[2]));
            }
        }
    }
    if header[6] != type_code {
        return Err(format!("Expected type code {}. Observed = {}", type_code, header[2]));
    }
    return Ok(());
}

fn check_device_health(port: &mut Box<dyn SerialPort>) -> Result<(), String> {
    send_command(port, LIDAR_CMD_GET_DEVICE_HEALTH);
    let header = read(port, HEADER_SIZE, 10).unwrap();
    println!("Received header = {}", to_string(&header));
    validate_response_header(&header, Some(3), LIDAR_ANS_TYPE_DEVHEALTH).unwrap();
    let health = read(port, 3, 10).unwrap();
    println!("Response = {}", to_string(&health));

    if health[0] != 0 {
        return Err(format!(
                "Device health error. Error code = {:08b}. \
                 See the development manual for details.", health[0]));
    }
    return Ok(());
}

fn get_device_info(port: &mut Box<dyn SerialPort>) -> device_info::DeviceInfo {
    send_command(port, LIDAR_CMD_GET_DEVICE_INFO);
    let header = read(port, HEADER_SIZE, 10).unwrap();
    validate_response_header(&header, Some(20), LIDAR_ANS_TYPE_DEVINFO).unwrap();
    let info = read(port, 20, 10).unwrap();
    println!("Received header = {}", to_string(&header));
    println!("Response = {}", to_string(&info));
    return device_info::DeviceInfo {
        model_number: info[0],
        firmware_major_version: info[1],
        firmware_minor_version: info[2],
        hardware_version: info[3],
        serial_number: info[4..20].try_into().unwrap(),
    };
}

fn start_scan(port: &mut Box<dyn SerialPort>) {
    send_command(port, LIDAR_CMD_SCAN);
    let header = read(port, HEADER_SIZE, 10).unwrap();
    validate_response_header(&header, None, LIDAR_ANS_TYPE_MEASUREMENT).unwrap();
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

    let maybe_port = serialport::new(port_name, baud_rate)
        .timeout(Duration::from_millis(10))
        .open();

    let mut port = match maybe_port {
        Ok(port) => port,
        Err(e) => {
            eprintln!("Failed to open \"{}\". Error: {}", port_name, e);
            ::std::process::exit(1);
        }
    };

    stop_scan(&mut port);
    flush(&mut port);
    check_device_health(&mut port).unwrap();
    let device_info = get_device_info(&mut port);
    if device_info.model_number != ydlidar_models::YdlidarModels::T_MINI_PRO {
        println!("This package can handle only YDLiDAR T-mini Pro.");
        std::process::exit(1);
    }

    start_scan(&mut port);

    let (tx, rx) = mpsc::channel::<bool>();

    let handle = std::thread::spawn(move || -> Box<dyn SerialPort> {
        let mut is_scanning: bool = true;
        while is_scanning {
            let n_u32: u32 = port.bytes_to_read().unwrap();
            let n_read: usize = n_u32.try_into().unwrap();
            if n_read == 0 {
                continue;
            }
            let mut serial_buf: Vec<u8> = vec![0; n_read];
            match port.read(serial_buf.as_mut_slice()) {
                Ok(n) => println!("Read {} bytes: {}", n, to_string(&serial_buf)),
                Err(e) => eprintln!("{e}"),
            }

            is_scanning = match rx.try_recv() {
                Ok(s) => s,
                Err(_) => true,
            }
        }
        return port;
    });

    sleep_ms(4000);
    tx.send(false).unwrap();
    let mut port = handle.join().unwrap();
    stop_scan(&mut port);
}
