use serialport::SerialPort;
use std::io;
use std::io::{Read, Write};

use crate::system_command;
use crate::timer;
use crate::type_code;

const N_READ_TRIALS: usize = 3;

pub fn start_scan(port: &mut Box<dyn SerialPort>) {
    send_command(port, system_command::SCAN);
    let header = read(port, signal_parser::HEADER_SIZE).unwrap();
    signal_parser::validate_response_header(&header, None, type_code::MEASUREMENT).unwrap();
}

fn stop_scan(port: &mut Box<dyn SerialPort>) {
    send_command(port, system_command::FORCE_STOP);
    send_command(port, system_command::STOP);
}

fn flush(port: &mut Box<dyn SerialPort>) {
    let n_read: usize = get_n_read(port);
    if n_read == 0 {
        return;
    }
    let mut packet: Vec<u8> = vec![0; n_read];
    port.read(packet.as_mut_slice()).unwrap();
}

pub fn stop_scan_and_flush(port: &mut Box<dyn SerialPort>) {
    stop_scan(port);
    flush(port);
}

fn send_data(port: &mut Box<dyn SerialPort>, data: &[u8]) {
    if let Err(e) = port.write(data) {
        eprintln!("{:?}", e);
    }
}

pub fn send_command(port: &mut Box<dyn SerialPort>, command: u8) {
    let data: [u8; 2] = [system_command::SYNC_BYTE, command];
    send_data(port, &data);
}

fn timeout_error(message: &str) -> io::Error {
    return io::Error::new(io::ErrorKind::TimedOut, message);
}

pub fn get_n_read(port: &mut Box<dyn SerialPort>) -> usize {
    let n_u32: u32 = port.bytes_to_read().unwrap();
    let n_read: usize = n_u32.try_into().unwrap();
    n_read
}

pub fn read(port: &mut Box<dyn SerialPort>, data_size: usize) -> Result<Vec<u8>, io::Error> {
    assert!(data_size > 0);
    for _ in 0..N_READ_TRIALS {
        let n_read: usize = get_n_read(port);

        if n_read < data_size {
            timer::sleep_ms(10);
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

#[cfg(test)]
mod tests {
    use super::*;
    use serialport::TTYPort;

    #[test]
    fn test_send_command() {
        let (master, mut slave) = TTYPort::pair().expect("Unable to create ptty pair");
        let mut master_ptr = Box::new(master) as Box<dyn SerialPort>;
        send_command(&mut master_ptr, 0x68);

        timer::sleep_ms(10);
        let mut buf = [0u8; 2];
        slave.read(&mut buf).unwrap();
        assert_eq!(buf, [0xA5, 0x68]);
    }

    #[test]
    fn test_flush() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");
        master
            .write(&[0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00])
            .unwrap();

        let mut slave_ptr = Box::new(slave) as Box<dyn SerialPort>;

        timer::sleep_ms(10);

        assert_eq!(slave_ptr.bytes_to_read().unwrap(), 10);
        flush(&mut slave_ptr);
        assert_eq!(slave_ptr.bytes_to_read().unwrap(), 0);

        // when zero bytes to read
        flush(&mut slave_ptr);
        assert_eq!(slave_ptr.bytes_to_read().unwrap(), 0);
    }

    #[test]
    fn test_start_scan() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");
        master
            .write(&[0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81])
            .unwrap();

        let mut slave_ptr = Box::new(slave) as Box<dyn SerialPort>;
        start_scan(&mut slave_ptr);

        timer::sleep_ms(10);

        let mut buf = [0u8; 2];
        master.read(&mut buf).unwrap();
        assert_eq!(buf, [0xA5, 0x60]);
    }

    #[test]
    fn test_stop_scan() {
        let (master, mut slave) = TTYPort::pair().expect("Unable to create ptty pair");
        let mut master_ptr = Box::new(master) as Box<dyn SerialPort>;
        stop_scan(&mut master_ptr);

        timer::sleep_ms(10);

        let mut buf = [0u8; 4];
        slave.read(&mut buf).unwrap();
        assert_eq!(buf, [0xA5, 0x00, 0xA5, 0x65]);
    }
}
