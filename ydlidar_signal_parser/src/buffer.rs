use alloc::collections::VecDeque;

const START_ELEMENT0: u8 = 0xAA;
const START_ELEMENT1: u8 = 0x55;

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
        if *e0 != START_ELEMENT0 {
            continue;
        }
        let e1 = match buffer.get(i + 1) {
            Some(e) => e,
            None => continue,
        };
        if *e1 != START_ELEMENT1 {
            continue;
        }
        return Ok(i);
    }
    Err(())
}

pub fn sendable_packet_range(buffer: &VecDeque<u8>) -> Result<(usize, usize), ()> {
    let start_index = find_start_index(buffer)?;
    let end_index = get_packet_size(buffer, start_index)?;
    Ok((start_index, end_index))
}
