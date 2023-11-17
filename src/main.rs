use std::io::{self, Write};
use std::time::Duration;
use std::{thread, time};
use std::sync::mpsc;

use std::fs::File;

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
    return data.iter().map(|e| format!("{:02X}", e)).collect::<Vec<_>>().join(" ");
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
        sleep_ms(10);
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

    if health[0] != 0 {  // Last two bit are reserved bits, which should be ignored.
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

fn to_angle(bit1: u8, bit2: u8) -> f64 {
    assert_eq!((bit2 as u16) * 0x100, (bit2 as u16) << 8);
    let a = ((bit1 as u16) + ((bit2 as u16) << 8)) >> 1;
    return (a as f64) / 64.;
}

fn n_scan_samples(packet : &[u8]) -> usize {
    packet[3] as usize
}

fn calc_angles(packet : &[u8]) -> Vec<f64> {
    let start_angle = to_angle(packet[4], packet[5]);
    let end_angle = to_angle(packet[6], packet[7]);

    let n = n_scan_samples(packet);
    let angle_rate: f64 = if start_angle < end_angle {
        (end_angle - start_angle) / ((n - 1) as f64)
    } else {
        (end_angle - start_angle + 360.) / ((n - 1) as f64)
    };

    return (0..n).map(|i| ((i as f64) * angle_rate + start_angle) % 360.).collect::<Vec<_>>();
}

fn scan_indices(n_scan_samples: usize) -> impl Iterator<Item = usize> {
    (0..n_scan_samples).map(|i| (10 + i * 3) as usize)
}

fn get_flags(packet : &[u8]) -> Vec<u8> {
    let indices = scan_indices(n_scan_samples(packet));
    indices.map(|i| packet[i + 1] & 0x03).collect::<Vec<_>>()
}

fn get_intensities(packet : &[u8]) -> Vec<u8> {
    let indices = scan_indices(n_scan_samples(packet));
    indices.map(|i| packet[i] as u8).collect::<Vec<_>>()
}

fn calc_distance(b1: u8, b2 : u8) -> u16 {
    ((b2 as u16) << 6) + ((b1 as u16) >> 2)
}

fn calc_distances(packet : &[u8]) -> Vec<u16> {
    let indices = scan_indices(n_scan_samples(packet));
    indices.map(|i| calc_distance(packet[i + 1], packet[i + 2])).collect::<Vec<_>>()
}

fn check_packet_size(packet: &[u8]) {
    let expected = 10 + n_scan_samples(packet) * 3;
    if packet.len() < expected {
        panic!("Scan packet size is insufficient. Required at least {expected} bytes.");
    }
}

fn check_scan_packet_header(packet: &[u8]) {
    if packet[0] == 0xAA && packet[1] == 0x55 {
        return;
    }
    panic!("Scan packet must start with 0xAA55");
}

fn is_start_packet(packet: &[u8]) -> bool {
    packet[2] & 0x01 == 1
}

fn parse_scan(packet: &[u8]) -> (Vec<f64>, Vec<u8>, Vec<u8>, Vec<u16>) {
    check_scan_packet_header(packet);
    check_packet_size(packet);
    let angles = calc_angles(packet);
    let intensities = get_intensities(packet);
    let flags = get_flags(packet);
    let distances = calc_distances(packet);
    (angles, intensities, flags, distances)
}

fn write(filename: &str, xs: &[f64], ys: &[f64]) -> std::io::Result<()> {
    assert_eq!(xs.len(), ys.len());

    let mut file = File::create(filename)?;
    for i in 0..xs.len() {
        let s = format!("{} {}\n", xs[i], ys[i]);
        file.write_all(s.as_bytes())?;
    }
    Ok(())
}

fn polar_to_cartesian(angles: &[f64], distances: &[f64]) -> (Vec<f64>, Vec<f64>) {
    assert_eq!(angles.len(), distances.len());
    let xs = (0..angles.len()).map(|i| f64::cos(angles[i]) * distances[i]).collect::<Vec<_>>();
    let ys = (0..angles.len()).map(|i| f64::sin(angles[i]) * distances[i]).collect::<Vec<_>>();
    (xs, ys)
}

fn split(packet: &[u8]) -> Vec<usize> {
    let mut indices = Vec::new();
    for i in 0..(packet.len()-1) {
        if packet[i+0] == 0xAA && packet[i+1] == 0x55 {
            indices.push(i);
        }
    }
    indices.push(packet.len());
    return indices;
}

#[cfg(test)]
mod tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    fn test_split() {
        let xs = split(
        //      0     1     2     3     4     5     6     7     8     9
           &[0xAA, 0x55, 0x00, 0x28, 0xF5, 0x82, 0xCF, 0x94, 0xD8, 0x6F,
             0xAA, 0x55, 0x82, 0x28, 0x41, 0x95, 0xE3, 0xA6, 0x6A, 0x4F,
             0xAA, 0x55, 0x00, 0x28, 0x55, 0xA7, 0xD9, 0x04, 0xB8, 0xDC]);
        assert_eq!(xs.len(), 4);
        assert_eq!(xs, vec![0, 10, 20, 30]);
    }
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
        let mut index = 0;
        let mut starts = Vec::new();
        while is_scanning {
            let n_u32: u32 = port.bytes_to_read().unwrap();
            let n_read: usize = n_u32.try_into().unwrap();
            if n_read == 0 {
                sleep_ms(10);
                continue;
            }
            let mut serial_buf: Vec<u8> = vec![0; n_read];
            port.read(serial_buf.as_mut_slice()).unwrap();
            // try_split(&serial_buf);
            let (angles_degree, intensities, flags, distances) = parse_scan(&serial_buf);
            // let angles_radian = angles_degree.iter().map(|e| e * std::f64::consts::PI / 180.).collect::<Vec<f64>>();
            let float_distances = distances.iter().map(|e| *e as f64).collect::<Vec<f64>>();
            write(&format!("scan/{:04}.txt", index), &angles_degree, &float_distances).unwrap();
            if is_start_packet(&serial_buf) {
                starts.push(index);
            }
            index += 1;

            is_scanning = match rx.try_recv() {
                Ok(s) => s,
                Err(_) => true,
            }
        }
        println!("starts = {:?}", starts);
        return port;
    });

    sleep_ms(10000);
    tx.send(false).unwrap();
    let mut port = handle.join().unwrap();
    stop_scan(&mut port);
    flush(&mut port);
}
