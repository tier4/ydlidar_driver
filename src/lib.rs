#[macro_use]
extern crate alloc;

use alloc::collections::VecDeque;
use alloc::string::String;
use alloc::vec::Vec;

use std::sync::mpsc;
use std::thread::JoinHandle;

mod device_info;
mod serial;
mod system_command;
mod timer;
mod type_code;
mod ydlidar_models;

use crossbeam_channel::{bounded, Receiver, Sender};
use serialport::SerialPort;
use ydlidar_signal_parser;
use ydlidar_signal_parser::{validate_response_header, Scan, sendable_packet_range};
use system_command::SystemCommand;
use ydlidar_models::YdlidarModels;


fn check_device_health(
    port: &mut Box<dyn SerialPort>,
    commands: &system_command::SystemCommand)
-> Result<(), String> {
    serial::send_command(port, commands.get_devcice_health().unwrap());
    let header = serial::read(port, ydlidar_signal_parser::HEADER_SIZE).unwrap();
    validate_response_header(&header, Some(3), type_code::DEVHEALTH).unwrap();
    let health = serial::read(port, 3).unwrap();

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
    serial::send_command(port, system_command::GET_DEVICE_INFO);
    let header = serial::read(port, ydlidar_signal_parser::HEADER_SIZE).unwrap();
    validate_response_header(&header, Some(20), type_code::DEVINFO).unwrap();
    let info = serial::read(port, 20).unwrap();
    return device_info::DeviceInfo {
        model_number: info[0],
        firmware_major_version: info[2],
        firmware_minor_version: info[1],
        hardware_version: info[3],
        serial_number: info[4..20].try_into().unwrap(),
    };
}

fn do_terminate(terminator_rx: &Receiver<bool>) -> bool {
    if let Ok(terminate) = terminator_rx.try_recv() {
        return terminate;
    }
    return false;
}

fn read_device_signal(
    port: &mut Box<dyn SerialPort>,
    commands: &system_command::SystemCommand,
    scan_data_tx: mpsc::SyncSender<Vec<u8>>,
    reader_terminator_rx: Receiver<bool>,
) {
    loop {
        if do_terminate(&reader_terminator_rx) {
            serial::stop_scan_and_flush(port, commands);
            return;
        }
        let n_read: usize = serial::get_n_read(port);
        if n_read == 0 {
            continue;
        }
        let signal = serial::read(port, n_read).unwrap();
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
                timer::sleep_ms(10);
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
        if ydlidar_signal_parser::is_beginning_of_cycle(&packet) {
            scan_tx.send(scan).unwrap();
            scan = Scan::new();
        }

        if let Err(e) = ydlidar_signal_parser::err_if_checksum_mismatched(&packet) {
            eprintln!("{:?}", e);
            scan.checksum_correct = false;
        }

        ydlidar_signal_parser::calc_angles(&packet, &mut scan.angles_radian);
        ydlidar_signal_parser::calc_distances(&packet, &mut scan.distances);
        ydlidar_signal_parser::get_intensities(&packet, &mut scan.intensities);
        ydlidar_signal_parser::get_flags(&packet, &mut scan.flags);
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
    // let baud_rate = 230400; // fixed baud rate for YDLiDAR T-mini Pro
    let baud_rate = 512000; // fixed baud rate for YDLiDAR T-mini Pro
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

    let device_info = get_device_info(&mut port);
    let ydlidar_model = YdlidarModels::new(device_info.model_number).unwrap();
    let commands = SystemCommand::new(ydlidar_model);

    if !(cfg!(test)) {
        // In testing, disable flushing to receive dummy signals
        serial::stop_scan_and_flush(&mut port, &commands);
        timer::sleep_ms(10);
        serial::stop_scan_and_flush(&mut port, &commands);
    }

    check_device_health(&mut port, &commands).unwrap();

    let (reader_terminator_tx, reader_terminator_rx) = bounded(10);
    let (parser_terminator_tx, parser_terminator_rx) = bounded(10);
    let (scan_data_tx, scan_data_rx) = mpsc::sync_channel::<Vec<u8>>(200);

    serial::start_scan(&mut port, &commands);

    let reader_thread = Some(std::thread::spawn(move || {
        read_device_signal(&mut port, &commands, scan_data_tx, reader_terminator_rx);
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
    use std::io::Write;
    use ydlidar_signal_parser::InterferenceFlag;

    fn radian_to_degree(e: f64) -> f64 {
        e * 180. / std::f64::consts::PI
    }

    #[test]
    fn test_check_device_health() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");
        let mut slave_ptr = Box::new(slave) as Box<dyn SerialPort>;

        master
            .write(&[0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00])
            .unwrap();
        timer::sleep_ms(10);
        assert_eq!(check_device_health(&mut slave_ptr), Ok(()));

        master
            .write(&[0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x02, 0x00, 0x00])
            .unwrap();
        timer::sleep_ms(10);
        assert_eq!(
            check_device_health(&mut slave_ptr),
            Err("Device health error. Error code = 0b00000010. \
                 See the development manual for details."
                .to_string())
        );
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

        timer::sleep_ms(10);

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

        timer::sleep_ms(10);

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
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::AmbientLight,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
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

        timer::sleep_ms(10);

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

        timer::sleep_ms(10);

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

        timer::sleep_ms(10);

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
