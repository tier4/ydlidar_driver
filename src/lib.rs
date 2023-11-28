use std::io::{Read, Write};
use std::io;
use std::sync::mpsc;
use std::collections::VecDeque;
use std::thread::JoinHandle;

mod debug;
mod device_info;
mod ydlidar_models;

use serialport::SerialPort;
use crossbeam_channel::{Sender, Receiver, bounded};

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

fn send_data(port: &mut Box<dyn SerialPort>, data: &[u8]) {
    if let Err(e) = port.write(data) {
        eprintln!("{:?}", e);
    }
}

fn send_command(port: &mut Box<dyn SerialPort>, command: u8) {
    let data : [u8; 2] = [LIDAR_CMD_SYNC_BYTE, command];
    send_data(port, &data);
}

fn sleep_ms(duration: u64) {
    std::thread::sleep(std::time::Duration::from_millis(duration));
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
    let n_read: usize = get_n_read(port);
    if n_read == 0 {
        return;
    }
    let mut packet: Vec<u8> = vec![0; n_read];
    port.read(packet.as_mut_slice()).unwrap();
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
        if let Err(e) = port.read(packet.as_mut_slice()) {
            return Err(e);
        }
        return Ok(packet);
    }
    return Err(timeout_error("Operation timed out"));
}

fn validate_response_header(header: &Vec<u8>, maybe_response_length: Option<u8>, type_code: u8) -> Result<(), String> {
    if header.len() != HEADER_SIZE {
        return Err(format!("Response header must be always seven bytes. Actually {} bytes.", header.len()));
    }
    if header[0] != 0xA5 || header[1] != 0x5A {
        return Err(format!("Header sign must start with 0xA55A. Observed = {}.", debug::to_string(&header[0..2])));
    }
    match maybe_response_length {
        None => (),
        Some(len) => {
            if header[2] != len {
                return Err(format!("Expected response length of {} bytes but found {} bytes.", len, header[2]));
            }
        }
    }
    if header[6] != type_code {
        return Err(format!("Expected type code {} but obtained {}.", type_code, header[6]));
    }
    return Ok(());
}

fn check_device_health(port: &mut Box<dyn SerialPort>) -> Result<(), String> {
    send_command(port, LIDAR_CMD_GET_DEVICE_HEALTH);
    let header = read(port, HEADER_SIZE, 10).unwrap();
    validate_response_header(&header, Some(3), LIDAR_ANS_TYPE_DEVHEALTH).unwrap();
    let health = read(port, 3, 10).unwrap();

    if health[0] != 0 {  // Last two bit are reserved bits, which should be ignored.
        return Err(format!(
                "Device health error. Error code = {:#010b}. \
                 See the development manual for details.", health[0]));
    }
    return Ok(());
}

fn get_device_info(port: &mut Box<dyn SerialPort>) -> device_info::DeviceInfo {
    send_command(port, LIDAR_CMD_GET_DEVICE_INFO);
    let header = read(port, HEADER_SIZE, 10).unwrap();
    validate_response_header(&header, Some(20), LIDAR_ANS_TYPE_DEVINFO).unwrap();
    let info = read(port, 20, 10).unwrap();
    return device_info::DeviceInfo {
        model_number: info[0],
        firmware_major_version: info[2],
        firmware_minor_version: info[1],
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
    send_command(port, LIDAR_CMD_STOP);
}

fn stop_scan_and_flush(port: &mut Box<dyn SerialPort>) {
    stop_scan(port);
    flush(port);
}

fn to_angle(bit1: u8, bit2: u8) -> f64 {
    let a = ((bit1 as u16) + ((bit2 as u16) << 8)) >> 1;
    return (a as f64) / 64.;
}

fn n_scan_samples(packet : &[u8]) -> usize {
    packet[3] as usize
}

fn degree_to_radian(degree: f64) -> f64 {
    degree * std::f64::consts::PI / 180.
}

fn calc_angles(packet : &[u8], angles_radian: &mut Vec<f64>) {
    let start_angle = to_angle(packet[4], packet[5]);
    let end_angle = to_angle(packet[6], packet[7]);

    let n = n_scan_samples(packet);
    let angle_rate: f64 = if start_angle < end_angle {
        (end_angle - start_angle) / ((n - 1) as f64)
    } else {
        (end_angle - start_angle + 360.) / ((n - 1) as f64)
    };

    for i in 0..n {
        let angle_degree = (start_angle + (i as f64) * angle_rate) % 360.;
        let angle_radian = degree_to_radian(angle_degree);
        angles_radian.push(angle_radian);
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
    pub angles_radian: Vec<f64>,
    pub distances: Vec<u16>,
    pub flags: Vec<u8>,
    pub intensities: Vec<u8>
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

fn read_device_signal(
    port: &mut Box<dyn SerialPort>,
    scan_data_tx: mpsc::SyncSender<Vec<u8>>,
    reader_terminator_rx: Receiver<bool>) {
    let mut is_scanning: bool = true;
    while is_scanning {
        let n_read: usize = get_n_read(port);
        if n_read == 0 {
            sleep_ms(10);
            continue;
        }
        let mut signal: Vec<u8> = vec![0; n_read];
        port.read(signal.as_mut_slice()).unwrap();
        if let Err(e) = scan_data_tx.send(signal) {
            eprintln!("error: {e}");
        }
        is_scanning = match reader_terminator_rx.try_recv() {
            Ok(s) => s,
            Err(_) => true,
        }
    }
    stop_scan_and_flush(port);
}

fn parse_stream(
        scan_data_rx: mpsc::Receiver<Vec<u8>>,
        parser_terminator_rx: Receiver<bool>,
        scan_tx: mpsc::SyncSender<Scan>) {
    let mut is_scanning: bool = true;
    let mut buffer = VecDeque::<u8>::new();
    let mut scan = Scan::new();
    while is_scanning {
        match scan_data_rx.try_recv() {
            Ok(data) => {
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
        buffer.drain(..start_index);   // remove leading bytes
        if buffer.len() < n_packet_bytes {
            // insufficient buffer size to extract a packet
            continue;
        }
        let packet = buffer.drain(0..n_packet_bytes).collect::<Vec<_>>();
        calc_angles(&packet, &mut scan.angles_radian);
        calc_distances(&packet, &mut scan.distances);
        get_intensities(&packet, &mut scan.intensities);
        get_flags(&packet, &mut scan.flags);
        if is_beginning_of_cycle(&packet) {
            scan_tx.send(scan).unwrap();
            scan = Scan::new();
        }

        is_scanning = match parser_terminator_rx.try_recv() {
            Ok(s) => s,
            Err(_) => true,
        }
    }
}

pub struct DriverThreads {
    reader_terminator_tx: Sender<bool>,
    parser_terminator_tx: Sender<bool>,
    reader_thread: Option<JoinHandle<()>>,
    receiver_thread: Option<JoinHandle<()>>,
}

pub fn run_driver(port_name: &str) -> (DriverThreads, mpsc::Receiver<Scan>) {
    let baud_rate = 230400;   // fixed baud rate for YDLiDAR T-mini Pro
    let maybe_port = serialport::new(port_name, baud_rate)
        .timeout(std::time::Duration::from_millis(10))
        .open();

    let mut port = match maybe_port {
        Ok(port) => port,
        Err(e) => {
            eprintln!("Failed to open \"{}\". Error: {}", port_name, e);
            std::process::exit(1);
        }
    };

    stop_scan_and_flush(&mut port);
    check_device_health(&mut port).unwrap();
    let device_info = get_device_info(&mut port);
    if device_info.model_number != ydlidar_models::YdlidarModels::T_MINI_PRO {
        eprintln!("This package can handle only YDLiDAR T-mini Pro.");
        std::process::exit(1);
    }

    let (reader_terminator_tx, reader_terminator_rx) = bounded(10);
    let (parser_terminator_tx, parser_terminator_rx) = bounded(10);
    let (scan_data_tx, scan_data_rx) = mpsc::sync_channel::<Vec<u8>>(200);

    start_scan(&mut port);

    let reader_thread = Some(std::thread::spawn(move || {
        read_device_signal(&mut port, scan_data_tx, reader_terminator_rx);
    }));

    let (scan_tx, scan_rx) = mpsc::sync_channel::<Scan>(10);
    let receiver_thread = Some(std::thread::spawn(move || {
        parse_stream(scan_data_rx, parser_terminator_rx, scan_tx);
    }));

    let driver_threads = DriverThreads {
        reader_thread: reader_thread,
        receiver_thread: receiver_thread,
        reader_terminator_tx: reader_terminator_tx,
        parser_terminator_tx: parser_terminator_tx
    };

    (driver_threads, scan_rx)
}

fn join(driver_threads: &mut DriverThreads) {
    driver_threads.reader_terminator_tx.send(false).unwrap();
    driver_threads.parser_terminator_tx.send(false).unwrap();

    if !driver_threads.reader_thread.is_none() {
        let thread = driver_threads.reader_thread.take().unwrap();
        thread.join().unwrap();
    }
    if !driver_threads.receiver_thread.is_none() {
        let thread = driver_threads.receiver_thread.take().unwrap();
        thread.join().unwrap();
    }
}

impl Drop for DriverThreads {
    fn drop(&mut self) {
        join(self);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serialport::TTYPort;

    #[test]
    fn test_validate_response_header() {
        assert_eq!(
            validate_response_header(
                &vec![0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04], Some(0x14), 0x04),
            Ok(()));

        assert_eq!(
            validate_response_header(
                &vec![0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04, 0x09], Some(0x14), 0x04),
            Err("Response header must be always seven bytes. Actually 8 bytes.".to_string()));

        assert_eq!(
            validate_response_header(
                &vec![0xA6, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04], Some(0x14), 0x04),
            Err("Header sign must start with 0xA55A. Observed = A6 5A.".to_string()));

        assert_eq!(
            validate_response_header(
                &vec![0xA5, 0x2A, 0x14, 0x00, 0x00, 0x00, 0x04], Some(0x14), 0x04),
            Err("Header sign must start with 0xA55A. Observed = A5 2A.".to_string()));

        assert_eq!(
            validate_response_header(
                &vec![0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04], Some(0x12), 0x04),
            Err("Expected response length of 18 bytes but found 20 bytes.".to_string()));

        assert_eq!(
            validate_response_header(
                &vec![0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x08], Some(0x14), 0x04),
            Err("Expected type code 4 but obtained 8.".to_string()));
    }

    #[test]
    fn test_send_command() {
        let (master, mut slave) = TTYPort::pair().expect("Unable to create ptty pair");
        let mut master_ptr = Box::new(master) as Box<dyn SerialPort>;
        send_command(&mut master_ptr, 0x68);

        sleep_ms(10);
        let mut buf = [0u8; 2];
        slave.read(&mut buf).unwrap();
        assert_eq!(buf, [0xA5, 0x68]);
    }

    #[test]
    fn test_check_device_health() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");
        let mut slave_ptr = Box::new(slave) as Box<dyn SerialPort>;

        master.write(&[0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00]).unwrap();
        sleep_ms(10);
        assert_eq!(check_device_health(&mut slave_ptr), Ok(()));

        master.write(&[0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x02, 0x00, 0x00]).unwrap();
        sleep_ms(10);
        assert_eq!(
            check_device_health(&mut slave_ptr),
            Err("Device health error. Error code = 0b00000010. \
                 See the development manual for details.".to_string()));
    }

    #[test]
    fn test_flush() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");
        master.write(&[0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00]).unwrap();

        let mut slave_ptr = Box::new(slave) as Box<dyn SerialPort>;

        sleep_ms(10);

        assert_eq!(slave_ptr.bytes_to_read().unwrap(), 10);
        flush(&mut slave_ptr);
        assert_eq!(slave_ptr.bytes_to_read().unwrap(), 0);

        // when zero bytes to read
        flush(&mut slave_ptr);
        assert_eq!(slave_ptr.bytes_to_read().unwrap(), 0);
    }

    #[test]
    fn test_get_device_info() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");
        master.write(
            &[0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04,
              0x96, 0x00, 0x01, 0x02, 0x02, 0x00, 0x02,
              0x02, 0x01, 0x01, 0x00, 0x03, 0x00, 0x01,
              0x01, 0x01, 0x01, 0x01, 0x01, 0x01]).unwrap();

        sleep_ms(10);

        let mut slave_ptr = Box::new(slave) as Box<dyn SerialPort>;
        let info = get_device_info(&mut slave_ptr);
        assert_eq!(info.model_number, 150);
        assert_eq!(info.firmware_major_version, 1);
        assert_eq!(info.firmware_minor_version, 0);
        assert_eq!(info.hardware_version, 2);
        assert_eq!(info.serial_number, [2, 0, 2, 2, 1, 1, 0, 3, 0, 1, 1, 1, 1, 1, 1, 1]);
    }

    #[test]
    fn test_start_scan() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");
        master.write(&[0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81]).unwrap();

        let mut slave_ptr = Box::new(slave) as Box<dyn SerialPort>;
        start_scan(&mut slave_ptr);

        sleep_ms(10);

        let mut buf = [0u8; 2];
        master.read(&mut buf).unwrap();
        assert_eq!(buf, [0xA5, 0x60]);
    }

    #[test]
    fn test_stop_scan() {
        let (master, mut slave) = TTYPort::pair().expect("Unable to create ptty pair");
        let mut master_ptr = Box::new(master) as Box<dyn SerialPort>;
        stop_scan(&mut master_ptr);

        sleep_ms(10);

        let mut buf = [0u8; 4];
        slave.read(&mut buf).unwrap();
        assert_eq!(buf, [0xA5, 0x00, 0xA5, 0x65]);
    }

    #[test]
    fn test_calc_angles() {
        let epsilon = f64::EPSILON;
        let pi = std::f64::consts::PI;

        let mut angles = Vec::new();
        calc_angles(&[0xAA, 0x55, 0x00, 0x28, 0x83, 0xA6, 0x07, 0x04], &mut angles);

        assert_eq!(angles.len(), 40);
        let angle_degree0 = to_angle(0x83, 0xA6);
        let angle_degree1 = to_angle(0x07, 0x04);
        let angle_radian0 = degree_to_radian(angle_degree0);
        let angle_radian1 = degree_to_radian(angle_degree1);

        assert!(angle_radian1 < angle_radian0);

        let diff = angle_radian1 - angle_radian0 + 2. * pi;
        for i in 0..40 {
            let a = angle_radian0 + diff * (i as f64) / (40. - 1.);
            let b = a % (2. * pi);
            assert!(f64::abs(angles[i] - b) < 8. * epsilon);
        }

        calc_angles(&[0xAA, 0x55, 0x56, 0x28, 0x79, 0x04, 0xD1, 0x15], &mut angles);

        assert_eq!(angles.len(), 80);
        let angle_degree0 = to_angle(0x79, 0x04);
        let angle_degree1 = to_angle(0xD1, 0x15);
        let angle_radian0 = degree_to_radian(angle_degree0);
        let angle_radian1 = degree_to_radian(angle_degree1);

        let diff = angle_radian1 - angle_radian0;
        for i in 0..40 {
            let a = angle_radian0 + diff * (i as f64) / (40. - 1.);
            let b = a % (2. * pi);
            assert!(f64::abs(angles[i + 40] - b) < 8. * epsilon);
        }

    }
}
