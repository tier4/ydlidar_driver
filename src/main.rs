use std::io::{self, Write};
use std::time::Duration;
use std::{thread, time};
use std::sync::mpsc;
use std::collections::VecDeque;
use std::sync::mpsc::{SyncSender, Sender, Receiver};

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

fn get_n_read(port: &mut Box<dyn SerialPort>) -> usize {
    let n_u32: u32 = port.bytes_to_read().unwrap();
    let n_read: usize = n_u32.try_into().unwrap();
    n_read
}

fn flush(port: &mut Box<dyn SerialPort>) {
    let mut f = || -> () {
        let n_read: usize = get_n_read(port);
        if n_read == 0 {
            return;
        }
        let mut packet: Vec<u8> = vec![0; n_read];
        port.read(packet.as_mut_slice()).unwrap();
    };

    for _ in 0..10 {
        f();
        sleep_ms(10);
    }
}

fn read(port: &mut Box<dyn SerialPort>, data_size: usize, n_trials: u32) -> Result<Vec<u8>, io::Error> {
    for _ in 0..n_trials {
        let n_read: usize = get_n_read(port);

        if n_read == 0 {
            sleep_ms(10);
            continue;
        }

        if n_read < data_size {
            let m = format!("Tried to read {} bytes but obtained {} bytes.", data_size, n_read);
            return Err(other_error(&m));
        }

        let mut packet: Vec<u8> = vec![0; data_size];
        match port.read(packet.as_mut_slice()) {
            Ok(n) => println!("Read {} bytes", n),
            Err(e) => return Err(e),
        }
        return Ok(packet);
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

fn calc_angles(packet : &[u8], angles: &mut Vec<f64>) {
    let start_angle = to_angle(packet[4], packet[5]);
    let end_angle = to_angle(packet[6], packet[7]);

    let n = n_scan_samples(packet);
    let angle_rate: f64 = if start_angle < end_angle {
        (end_angle - start_angle) / ((n - 1) as f64)
    } else {
        (end_angle - start_angle + 360.) / ((n - 1) as f64)
    };

    for i in 0..n {
        let angle_degree = ((i as f64) * angle_rate + start_angle) % 360.;
        let angle_radian = angle_degree * std::f64::consts::PI / 180.;
        angles.push(angle_radian);
    }
}

fn scan_indices(n_scan_samples: usize) -> impl Iterator<Item = usize> {
    (0..n_scan_samples).map(|i| (10 + i * 3) as usize)
}

fn get_flags(packet : &[u8], flags: &mut Vec<u8>) {
    for i in scan_indices(n_scan_samples(packet)) {
        flags.push(packet[i + 1] & 0x03)
    }
}

fn get_intensities(packet : &[u8], intensities: &mut Vec<u8>) {
    for i in scan_indices(n_scan_samples(packet)) {
        intensities.push(packet[i] as u8)
    }
}

fn calc_distance(b1: u8, b2 : u8) -> u16 {
    ((b2 as u16) << 6) + ((b1 as u16) >> 2)
}

fn calc_distances(packet : &[u8], distances: &mut Vec<u16>) {
    for i in scan_indices(n_scan_samples(packet)) {
        let d = calc_distance(packet[i + 1], packet[i + 2]);
        distances.push(d);
    }
}

fn check_packet_size(packet: &[u8]) -> Result<(), String> {
    let expected = 10 + n_scan_samples(packet) * 3;
    if packet.len() >= expected {
        return Ok(());
    }
    Err(format!("Scan packet size is insufficient. Required {} bytes. Actual {} bytes.",
                expected, packet.len()))
}

fn is_packet_header(element0: u8, element1: u8) -> bool {
    element0 == 0xAA && element1 == 0x55
}

fn check_scan_packet_header(packet: &[u8]) -> Result<(), String> {
    if is_packet_header(packet[0], packet[1]) {
        return Ok(());
    }
    Err("Scan packet must start with 0xAA55".to_string())
}

fn is_beginning_of_cycle(packet: &[u8]) -> bool {
    packet[2] & 0x01 == 1
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

fn polar_to_cartesian(
        angle_iter: impl Iterator<Item = f64>,
        distance_iter: impl Iterator<Item = f64>) -> (Vec<f64>, Vec<f64>) {
    let mut xs = Vec::new();
    let mut ys = Vec::new();
    for (d, w) in distance_iter.zip(angle_iter) {
        xs.push(d * f64::cos(w));
        ys.push(d * f64::sin(w));
    }
    (xs, ys)
}

fn split(packet: &[u8]) -> Vec<usize> {
    let mut indices = Vec::new();
    if packet.len() == 0 {
        return indices;
    }
    for i in 0..(packet.len()-1) {
        if packet[i+0] == 0xAA && packet[i+1] == 0x55 {
            indices.push(i);
        }
    }
    return indices;
}

#[cfg(test)]
mod tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    fn test_sendable_packet_range() {
    }

    #[test]
    fn test_split() {
        let xs = split(
        //      0     1     2     3     4     5     6     7     8     9
           &[0xAA, 0x55, 0x00, 0x28, 0xF5, 0x82, 0xCF, 0x94, 0xD8, 0x6F,
             0xAA, 0x55, 0x82, 0x28, 0x41, 0x95, 0xE3, 0xA6, 0x6A, 0x4F,
             0xAA, 0x55, 0x00, 0x28, 0x55, 0xA7, 0xD9, 0x04, 0xB8, 0xDC]);
        assert_eq!(xs.len(), 3);
        assert_eq!(xs, vec![0, 10, 20]);
    }
}

fn find_start_index(buffer: &VecDeque<u8>) -> Result<usize, ()> {
    if buffer.len() == 0 {
        return Err(());
    }
    for i in 0..(buffer.len()-1) {
        let e0 = match buffer.get(i+0) {
            Some(e) => e,
            None => continue,
        };
        let e1 = match buffer.get(i+1) {
            Some(e) => e,
            None => continue,
        };
        if is_packet_header(*e0, *e1) {
            return Ok(i);
        }
    }
    Err(())
}

fn get_packet_size(buffer: &VecDeque<u8>, start_index: usize) -> Result<usize, ()> {
    let index = start_index + 3;
    if index >= buffer.len() {
        return Err(());
    }
    let n_scan_samples = match buffer.get(index) {
        Some(n) => n,
        None => return Err(()),
    };
    Ok((10 + n_scan_samples * 3) as usize)
}

fn sendable_packet_range(buffer: &VecDeque<u8>) -> Result<(usize, usize), ()> {
    let start_index = find_start_index(buffer)?;
    let end_index = get_packet_size(buffer, start_index)?;
    Ok((start_index, end_index))
}

pub struct Scan {
    angles_radian: Vec<f64>,
    distances: Vec<u16>,
    flags: Vec<u8>,
    intensities: Vec<u8>
}

impl Scan {
    fn new() -> Scan {
        Scan {
            angles_radian: Vec::new(),
            distances: Vec::new(),
            flags: Vec::new(),
            intensities: Vec::new()
        }
    }
}

pub struct Driver {
    driver: String,
    tx: Sender<Scan>
}

fn read_signal(port: &mut Box<dyn SerialPort>, scan_data_tx: SyncSender<Vec<u8>>, reader_terminator_rx: Receiver<bool>) {
    let mut is_scanning: bool = true;
    while is_scanning {
        let n_read: usize = get_n_read(port);
        if n_read == 0 {
            sleep_ms(10);
            continue;
        }
        let mut signal: Vec<u8> = vec![0; n_read];
        port.read(signal.as_mut_slice()).unwrap();
        println!("Read {} bytes.", signal.len());
        if let Err(e) = scan_data_tx.send(signal) {
            eprintln!("error: {e}");
        }
        is_scanning = match reader_terminator_rx.try_recv() {
            Ok(s) => s,
            Err(_) => true,
        }
    }
}

fn receive_scan(scan_data_rx: Receiver<Vec<u8>>, parser_terminator_rx: Receiver<bool>) {
    let mut is_scanning: bool = true;
    let mut buffer = VecDeque::<u8>::new();
    let mut scan = Scan::new();
    while is_scanning {
        match scan_data_rx.try_recv() {
            Ok(data) => {
                println!("received data    : {:4} bytes = {}", data.len(), to_string(&data));
                buffer.extend(data);
            },
            Err(_) => {
                sleep_ms(10);
            }
        }

        if buffer.len() == 0 {
            continue;
        }

        let (start_index, n_packet_bytes) = match sendable_packet_range(&buffer) {
            Ok(t) => t,
            Err(_) => continue,
        };
        let leading_elements = buffer.drain(..start_index).collect::<Vec<_>>();
        if buffer.len() < n_packet_bytes {
            // insufficient buffer size to extract a packet
            continue;
        }
        let packet = buffer.drain(0..n_packet_bytes).collect::<Vec<_>>();
        println!("leading elements : {:4} bytes = {}", leading_elements.len(), to_string(&leading_elements));
        println!("packet           : {:4} bytes = {}", packet.len(), to_string(&packet));
        println!("\n");
        calc_angles(&packet, &mut scan.angles_radian);
        calc_distances(&packet, &mut scan.distances);
        get_intensities(&packet, &mut scan.intensities);
        get_flags(&packet, &mut scan.flags);
        if is_beginning_of_cycle(&packet) {
            // tx.send(scan);
            scan = Scan::new();
        }

        is_scanning = match parser_terminator_rx.try_recv() {
            Ok(s) => s,
            Err(_) => true,
        }
    }
}

fn launch_scan_thread(mut port: Box<dyn SerialPort>) -> Box<dyn SerialPort> {
    let (reader_terminator_tx, reader_terminator_rx) = mpsc::channel::<bool>();
    let (parser_terminator_tx, parser_terminator_rx) = mpsc::channel::<bool>();

    let (scan_data_tx, scan_data_rx) = mpsc::sync_channel::<Vec<u8>>(200);
    let scan_data_receiver = std::thread::spawn(move || -> Box<dyn SerialPort> {
        read_signal(&mut port, scan_data_tx, reader_terminator_rx);
        port
    });

    let scan_parser = std::thread::spawn(|| {
        receive_scan(scan_data_rx, parser_terminator_rx);
    });

    sleep_ms(10000);
    reader_terminator_tx.send(false).unwrap();
    parser_terminator_tx.send(false).unwrap();
    scan_parser.join().unwrap();
    return scan_data_receiver.join().unwrap();
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

    let mut port = launch_scan_thread(port);

    stop_scan(&mut port);
    flush(&mut port);
}
