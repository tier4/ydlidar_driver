#[macro_use]
extern crate alloc;

use alloc::collections::VecDeque;
use alloc::string::String;
use alloc::vec::Vec;

use std::io;
use std::io::{Read, Write};
use std::sync::mpsc;
use std::thread::JoinHandle;

mod checksum;
mod device_info;
mod scan;
mod ydlidar_models;

use crossbeam_channel::{bounded, Receiver, Sender};
use serialport::SerialPort;

const HEADER_SIZE: usize = 7;
const LIDAR_CMD_GET_DEVICE_HEALTH: u8 = 0x92;
const LIDAR_CMD_GET_DEVICE_INFO: u8 = 0x90;
const LIDAR_CMD_SYNC_BYTE: u8 = 0xA5;
const LIDAR_CMD_FORCE_STOP: u8 = 0x00;
const LIDAR_CMD_STOP: u8 = 0x65;
const LIDAR_CMD_SCAN: u8 = 0x60;
const LIDAR_ANS_TYPE_DEVINFO: u8 = 0x4;
const LIDAR_ANS_TYPE_DEVHEALTH: u8 = 0x6;
const LIDAR_ANS_TYPE_MEASUREMENT: u8 = 0x81;
const N_READ_TRIALS: usize = 3;

fn to_string(data: &[u8]) -> String {
    return data
        .iter()
        .map(|e| format!("{:02X}", e))
        .collect::<Vec<_>>()
        .join(" ");
}

fn send_data(port: &mut Box<dyn SerialPort>, data: &[u8]) {
    if let Err(e) = port.write(data) {
        eprintln!("{:?}", e);
    }
}

fn send_command(port: &mut Box<dyn SerialPort>, command: u8) {
    let data: [u8; 2] = [LIDAR_CMD_SYNC_BYTE, command];
    send_data(port, &data);
}

fn sleep_ms(duration: u64) {
    std::thread::sleep(std::time::Duration::from_millis(duration));
}

fn timeout_error(message: &str) -> io::Error {
    return io::Error::new(io::ErrorKind::TimedOut, message);
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

fn read(port: &mut Box<dyn SerialPort>, data_size: usize) -> Result<Vec<u8>, io::Error> {
    assert!(data_size > 0);
    for _ in 0..N_READ_TRIALS {
        let n_read: usize = get_n_read(port);

        if n_read < data_size {
            sleep_ms(10);
            continue;
        }

        let mut packet: Vec<u8> = vec![0; data_size];
        if let Err(e) = port.read(packet.as_mut_slice()) {
            return Err(e);
        }
        return Ok(packet);
    }
    return Err(timeout_error("Operation timed out"));
}

fn validate_response_header(
    header: &Vec<u8>,
    maybe_response_length: Option<u8>,
    type_code: u8,
) -> Result<(), String> {
    if header.len() != HEADER_SIZE {
        return Err(format!(
            "Response header must be always seven bytes. Actually {} bytes.",
            header.len()
        ));
    }
    if header[0] != 0xA5 || header[1] != 0x5A {
        return Err(format!(
            "Header sign must start with 0xA55A. Observed = {}.",
            to_string(&header[0..2])
        ));
    }
    match maybe_response_length {
        None => (),
        Some(len) => {
            if header[2] != len {
                return Err(format!(
                    "Expected response length of {} bytes but found {} bytes.",
                    len, header[2]
                ));
            }
        }
    }
    if header[6] != type_code {
        return Err(format!(
            "Expected type code {} but obtained {}.",
            type_code, header[6]
        ));
    }
    return Ok(());
}

fn check_device_health(port: &mut Box<dyn SerialPort>) -> Result<(), String> {
    send_command(port, LIDAR_CMD_GET_DEVICE_HEALTH);
    let header = read(port, HEADER_SIZE).unwrap();
    validate_response_header(&header, Some(3), LIDAR_ANS_TYPE_DEVHEALTH).unwrap();
    let health = read(port, 3).unwrap();

    if health[0] != 0 {
        // Last two bit are reserved bits, which should be ignored.
        return Err(format!(
            "Device health error. Error code = {:#010b}. \
                 See the development manual for details.",
            health[0]
        ));
    }
    return Ok(());
}

fn get_device_info(port: &mut Box<dyn SerialPort>) -> device_info::DeviceInfo {
    send_command(port, LIDAR_CMD_GET_DEVICE_INFO);
    let header = read(port, HEADER_SIZE).unwrap();
    validate_response_header(&header, Some(20), LIDAR_ANS_TYPE_DEVINFO).unwrap();
    let info = read(port, 20).unwrap();
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
    let header = read(port, HEADER_SIZE).unwrap();
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

fn find_start_index(buffer: &VecDeque<u8>) -> Result<usize, ()> {
    if buffer.len() == 0 {
        return Err(());
    }
    for i in 0..(buffer.len() - 1) {
        let e0 = match buffer.get(i + 0) {
            Some(e) => e,
            None => continue,
        };
        let e1 = match buffer.get(i + 1) {
            Some(e) => e,
            None => continue,
        };
        if scan::is_packet_header(*e0, *e1) {
            return Ok(i);
        }
    }
    Err(())
}

fn sendable_packet_range(buffer: &VecDeque<u8>) -> Result<(usize, usize), ()> {
    let start_index = find_start_index(buffer)?;
    let end_index = get_packet_size(buffer, start_index)?;
    Ok((start_index, end_index))
}

/// Struct to hold one lap of lidar scan data.
pub struct Scan {
    /// Scan angle in radian.
    pub angles_radian: Vec<f64>,
    /// Distance to an object.
    pub distances: Vec<u16>,
    /// Interference status of the returned signal.
    pub flags: Vec<scan::InterferenceFlag>,
    /// Return strength of the laser pulse.
    pub intensities: Vec<u8>,
    /// Checksum valiadtion result of the scan signal.
    pub checksum_correct: bool,
}

impl Scan {
    fn new() -> Scan {
        Scan {
            angles_radian: Vec::new(),
            distances: Vec::new(),
            flags: Vec::new(),
            intensities: Vec::new(),
            checksum_correct: true,
        }
    }
}

fn do_terminate(terminator_rx: &Receiver<bool>) -> bool {
    if let Ok(terminate) = terminator_rx.try_recv() {
        return terminate;
    }
    return false;
}

fn read_device_signal(
    port: &mut Box<dyn SerialPort>,
    scan_data_tx: mpsc::SyncSender<Vec<u8>>,
    reader_terminator_rx: Receiver<bool>,
) {
    loop {
        if do_terminate(&reader_terminator_rx) {
            stop_scan_and_flush(port);
            return;
        }
        let n_read: usize = get_n_read(port);
        if n_read == 0 {
            continue;
        }
        let signal = read(port, n_read).unwrap();
        if let Err(e) = scan_data_tx.send(signal) {
            eprintln!("error: {e}");
        }
    }
}

fn parse_packets(
    scan_data_rx: mpsc::Receiver<Vec<u8>>,
    parser_terminator_rx: Receiver<bool>,
    scan_tx: mpsc::SyncSender<Scan>,
) {
    let mut buffer = VecDeque::<u8>::new();
    let mut scan = Scan::new();
    loop {
        if do_terminate(&parser_terminator_rx) {
            return;
        }

        match scan_data_rx.try_recv() {
            Ok(data) => {
                buffer.extend(data);
            }
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
        buffer.drain(..start_index); // remove leading bytes
        if buffer.len() < n_packet_bytes {
            // insufficient buffer size to extract a packet
            continue;
        }
        let packet = buffer.drain(0..n_packet_bytes).collect::<Vec<_>>();
        if scan::is_beginning_of_cycle(&packet) {
            scan_tx.send(scan).unwrap();
            scan = Scan::new();
        }

        if let Err(e) = checksum::err_if_checksum_mismatched(&packet) {
            eprintln!("{:?}", e);
            scan.checksum_correct = false;
        }

        scan::calc_angles(&packet, &mut scan.angles_radian);
        scan::calc_distances(&packet, &mut scan.distances);
        scan::get_intensities(&packet, &mut scan.intensities);
        scan::get_flags(&packet, &mut scan.flags);
    }
}

/// Struct that contains driver threads.
pub struct DriverThreads {
    reader_terminator_tx: Sender<bool>,
    parser_terminator_tx: Sender<bool>,
    reader_thread: Option<JoinHandle<()>>,
    receiver_thread: Option<JoinHandle<()>>,
}

/// Function to launch YDLiDAR.
/// # Arguments
///
/// * `port_name` - Serial port name such as `/dev/ttyUSB0`.
pub fn run_driver(port_name: &str) -> (DriverThreads, mpsc::Receiver<Scan>) {
    let baud_rate = 230400; // fixed baud rate for YDLiDAR T-mini Pro
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

    if !(cfg!(test)) {
        // In testing, disable flushing to receive dummy signals
        stop_scan_and_flush(&mut port);
        sleep_ms(10);
        stop_scan_and_flush(&mut port);
    }

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
        parse_packets(scan_data_rx, parser_terminator_rx, scan_tx);
    }));

    let driver_threads = DriverThreads {
        reader_thread: reader_thread,
        receiver_thread: receiver_thread,
        reader_terminator_tx: reader_terminator_tx,
        parser_terminator_tx: parser_terminator_tx,
    };

    (driver_threads, scan_rx)
}

/// Function to join driver threads.
/// This function is automatically called when `driver_threads` is dropped.
pub fn join(driver_threads: &mut DriverThreads) {
    driver_threads.reader_terminator_tx.send(true).unwrap();
    driver_threads.parser_terminator_tx.send(true).unwrap();

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

    fn radian_to_degree(e: f64) -> f64 {
        e * 180. / std::f64::consts::PI
    }

    #[test]
    fn test_split() {
        let s = to_string(&[0xAA, 0x55, 0x00, 0x28]);
        assert_eq!(s, "AA 55 00 28");
    }

    #[test]
    fn test_validate_response_header() {
        assert_eq!(
            validate_response_header(
                &vec![0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04],
                Some(0x14),
                0x04
            ),
            Ok(())
        );

        assert_eq!(
            validate_response_header(
                &vec![0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04, 0x09],
                Some(0x14),
                0x04
            ),
            Err("Response header must be always seven bytes. Actually 8 bytes.".to_string())
        );

        assert_eq!(
            validate_response_header(
                &vec![0xA6, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04],
                Some(0x14),
                0x04
            ),
            Err("Header sign must start with 0xA55A. Observed = A6 5A.".to_string())
        );

        assert_eq!(
            validate_response_header(
                &vec![0xA5, 0x2A, 0x14, 0x00, 0x00, 0x00, 0x04],
                Some(0x14),
                0x04
            ),
            Err("Header sign must start with 0xA55A. Observed = A5 2A.".to_string())
        );

        assert_eq!(
            validate_response_header(
                &vec![0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04],
                Some(0x12),
                0x04
            ),
            Err("Expected response length of 18 bytes but found 20 bytes.".to_string())
        );

        assert_eq!(
            validate_response_header(
                &vec![0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x08],
                Some(0x14),
                0x04
            ),
            Err("Expected type code 4 but obtained 8.".to_string())
        );
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

        master
            .write(&[0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00])
            .unwrap();
        sleep_ms(10);
        assert_eq!(check_device_health(&mut slave_ptr), Ok(()));

        master
            .write(&[0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x02, 0x00, 0x00])
            .unwrap();
        sleep_ms(10);
        assert_eq!(
            check_device_health(&mut slave_ptr),
            Err("Device health error. Error code = 0b00000010. \
                 See the development manual for details."
                .to_string())
        );
    }

    #[test]
    fn test_flush() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");
        master
            .write(&[0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00])
            .unwrap();

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
        master
            .write(&[
                0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04, 0x96, 0x00, 0x01, 0x02, 0x02, 0x00, 0x02,
                0x02, 0x01, 0x01, 0x00, 0x03, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
            ])
            .unwrap();

        sleep_ms(10);

        let mut slave_ptr = Box::new(slave) as Box<dyn SerialPort>;
        let info = get_device_info(&mut slave_ptr);
        assert_eq!(info.model_number, 150);
        assert_eq!(info.firmware_major_version, 1);
        assert_eq!(info.firmware_minor_version, 0);
        assert_eq!(info.hardware_version, 2);
        assert_eq!(
            info.serial_number,
            [2, 0, 2, 2, 1, 1, 0, 3, 0, 1, 1, 1, 1, 1, 1, 1]
        );
    }

    #[test]
    fn test_start_scan() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");
        master
            .write(&[0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81])
            .unwrap();

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
    fn test_run_driver_normal_data() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");

        let device_health_packet = [0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00];
        master.write(&device_health_packet).unwrap();

        let device_info_packet = [
            0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04, 0x96, 0x00, 0x01, 0x02, 0x02, 0x00, 0x02,
            0x02, 0x01, 0x01, 0x00, 0x03, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
        ];
        master.write(&device_info_packet).unwrap();

        let start_scan_response_header = [0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81];
        master.write(&start_scan_response_header).unwrap();

        sleep_ms(10);

        let name = slave.name().unwrap();
        let (thread, scan_rx) = run_driver(&name);

        let packet = [
            // beginning of a lap
            0xAA, 0x55, 0xC7, 0x01, 0x01, 0x15, 0x01, 0x15, 0x1B, 0x56, // packet header
            0x14, 0x62, 0x02, // SpecularReflection
            // lap data
            0xAA, 0x55, 0xB0, 0x10, 0x81, 0x16, 0x01, 0x2D, 0x57, 0x7D, // packet header
            0xDD, 0x76, 0x03, // Specular Reflection
            0xD4, 0x76, 0x03, // Specular Reflection
            0xC3, 0x72, 0x03, // Specular Reflection
            0xB3, 0x7B, 0x03, // Ambient Light
            0x8E, 0x8A, 0x03, // Specular Reflection
            0x97, 0x6E, 0x04, // Specular Reflection
            0x9C, 0x22, 0x05, // Specular Reflection
            0xA7, 0x6A, 0x05, // Specular Reflection
            0xAB, 0x7A, 0x05, // Specular Reflection
            0x93, 0x82, 0x05, // Specular Reflection
            0x6D, 0xC2, 0x05, // Specular Reflection
            0x55, 0xA6, 0x05, // Specular Reflection
            0x57, 0x16, 0x05, // Specular Reflection
            0x67, 0x62, 0x02, // Specular Reflection
            0x80, 0x16, 0x02, // Specular Reflection
            0x9B, 0xE6, 0x01, // Specular Reflection
            // new lap
            0xAA, 0x55, 0xC7, 0x01, 0x81, 0x2E, 0x81, 0x2E, 0x1B, 0x56, // packet header
            0x14, 0x62, 0x02, // This signal will be regarded as a new lap
        ];
        master.write(&packet).unwrap();

        let scan = scan_rx.recv().unwrap();
        assert_eq!(scan.angles_radian.len(), 0);

        let scan = scan_rx.recv().unwrap();
        assert_eq!(scan.angles_radian.len(), 17);

        let expected = vec![
            42., 45., 48., 51., 54., 57., 60., 63., 66., 69., 72., 75., 78., 81., 84., 87., 90.,
        ];

        assert_eq!(scan.angles_radian.len(), expected.len());
        for i in 0..expected.len() {
            let degree = radian_to_degree(scan.angles_radian[i]);
            assert!(f64::abs(degree - expected[i]) < 1e-8);
        }

        let expected = vec![
            0x14, 0xDD, 0xD4, 0xC3, 0xB3, 0x8E, 0x97, 0x9C, 0xA7, 0xAB, 0x93, 0x6D, 0x55, 0x57,
            0x67, 0x80, 0x9B,
        ];
        assert_eq!(scan.intensities, expected);

        let expected = vec![
            ((0x62 as u16) >> 2) + ((0x02 as u16) << 6),
            ((0x76 as u16) >> 2) + ((0x03 as u16) << 6),
            ((0x76 as u16) >> 2) + ((0x03 as u16) << 6),
            ((0x72 as u16) >> 2) + ((0x03 as u16) << 6),
            ((0x7B as u16) >> 2) + ((0x03 as u16) << 6),
            ((0x8A as u16) >> 2) + ((0x03 as u16) << 6),
            ((0x6E as u16) >> 2) + ((0x04 as u16) << 6),
            ((0x22 as u16) >> 2) + ((0x05 as u16) << 6),
            ((0x6A as u16) >> 2) + ((0x05 as u16) << 6),
            ((0x7A as u16) >> 2) + ((0x05 as u16) << 6),
            ((0x82 as u16) >> 2) + ((0x05 as u16) << 6),
            ((0xC2 as u16) >> 2) + ((0x05 as u16) << 6),
            ((0xA6 as u16) >> 2) + ((0x05 as u16) << 6),
            ((0x16 as u16) >> 2) + ((0x05 as u16) << 6),
            ((0x62 as u16) >> 2) + ((0x02 as u16) << 6),
            ((0x16 as u16) >> 2) + ((0x02 as u16) << 6),
            ((0xE6 as u16) >> 2) + ((0x01 as u16) << 6),
        ];
        assert_eq!(scan.distances, expected);

        let expected = vec![
            scan::InterferenceFlag::SpecularReflection,
            scan::InterferenceFlag::SpecularReflection,
            scan::InterferenceFlag::SpecularReflection,
            scan::InterferenceFlag::SpecularReflection,
            scan::InterferenceFlag::AmbientLight,
            scan::InterferenceFlag::SpecularReflection,
            scan::InterferenceFlag::SpecularReflection,
            scan::InterferenceFlag::SpecularReflection,
            scan::InterferenceFlag::SpecularReflection,
            scan::InterferenceFlag::SpecularReflection,
            scan::InterferenceFlag::SpecularReflection,
            scan::InterferenceFlag::SpecularReflection,
            scan::InterferenceFlag::SpecularReflection,
            scan::InterferenceFlag::SpecularReflection,
            scan::InterferenceFlag::SpecularReflection,
            scan::InterferenceFlag::SpecularReflection,
            scan::InterferenceFlag::SpecularReflection,
        ];
        assert_eq!(scan.flags, expected);

        assert!(scan.checksum_correct);

        drop(thread);
    }

    #[test]
    fn test_run_driver_mod_at_360() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");

        let device_health_packet = [0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00];
        master.write(&device_health_packet).unwrap();

        let device_info_packet = [
            0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04, 0x96, 0x00, 0x01, 0x02, 0x02, 0x00, 0x02,
            0x02, 0x01, 0x01, 0x00, 0x03, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
        ];
        master.write(&device_info_packet).unwrap();

        let start_scan_response_header = [0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81];
        master.write(&start_scan_response_header).unwrap();

        sleep_ms(10);

        let name = slave.name().unwrap();
        let (thread, scan_rx) = run_driver(&name);

        let packet = [
            // lap data
            0xAA, 0x55, 0xB0, 0x10, 0x01, 0x96, 0x01, 0x0F, 0xD6, 0xDF, 0xDD, 0x76, 0x03, 0xD4,
            0x76, 0x03, 0xC3, 0x72, 0x03, 0xB3, 0x7A, 0x03, 0x8E, 0x8A, 0x03, 0x97, 0x6E, 0x04,
            0x9C, 0x22, 0x05, 0xA7, 0x6A, 0x05, 0xAB, 0x7A, 0x05, 0x93, 0x82, 0x05, 0x6D, 0xC2,
            0x05, 0x55, 0xA6, 0x05, 0x57, 0x16, 0x05, 0x67, 0x62, 0x02, 0x80, 0x16, 0x02, 0x9B,
            0xE6, 0x01, // new lap
            0xAA, 0x55, 0xC7, 0x01, 0x81, 0x2E, 0x81, 0x2E, 0x1B, 0x56, 0x14, 0x62, 0x02,
        ];
        master.write(&packet).unwrap();

        sleep_ms(10);

        let scan = scan_rx.recv().unwrap();
        assert_eq!(scan.angles_radian.len(), 16);

        let expected = vec![
            300., 306., 312., 318., 324., 330., 336., 342., 348., 354., 0., 6., 12., 18., 24., 30.,
        ];

        assert_eq!(scan.angles_radian.len(), expected.len());
        for i in 0..expected.len() {
            let degree = radian_to_degree(scan.angles_radian[i]);
            assert!(f64::abs(degree - expected[i]) < 1e-8);
        }

        assert!(scan.checksum_correct);

        drop(thread);
    }

    #[test]
    fn test_run_driver_checksum() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");

        let device_health_packet = [0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00];
        master.write(&device_health_packet).unwrap();

        let device_info_packet = [
            0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04, 0x96, 0x00, 0x01, 0x02, 0x02, 0x00, 0x02,
            0x02, 0x01, 0x01, 0x00, 0x03, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
        ];
        master.write(&device_info_packet).unwrap();

        let start_scan_response_header = [0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81];
        master.write(&start_scan_response_header).unwrap();

        sleep_ms(10);

        let name = slave.name().unwrap();
        let (thread, scan_rx) = run_driver(&name);

        let packet = [
            // lap data
            0xAA, 0x55, 0xB0, 0x10, 0x01, 0x96, 0x01, 0x0F, 0xD6, 0xDA, 0xDD, 0x76, 0x03, 0xD4,
            0x76, 0x03, 0xC3, 0x72, 0x03, 0xB3, 0x7A, 0x03, 0x8E, 0x8A, 0x03, 0x97, 0x6E, 0x04,
            0x9C, 0x22, 0x05, 0xA7, 0x6A, 0x05, 0xAB, 0x7A, 0x05, 0x93, 0x82, 0x05, 0x6D, 0xC2,
            0x05, 0x55, 0xA6, 0x05, 0x57, 0x16, 0x05, 0x67, 0x62, 0x02, 0x80, 0x16, 0x02, 0x9B,
            0xE6, 0x01, // new lap
            0xAA, 0x55, 0xC7, 0x01, 0x81, 0x2E, 0x81, 0x2E, 0x1B, 0x56, 0x14, 0x62, 0x02,
        ];
        master.write(&packet).unwrap();

        let scan = scan_rx.recv().unwrap();
        assert!(!scan.checksum_correct);

        drop(thread);
    }
}
