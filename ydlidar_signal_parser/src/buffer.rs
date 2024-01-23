use alloc::collections::VecDeque;

const START_ELEMENT0: u8 = 0xAA;
const START_ELEMENT1: u8 = 0x55;

fn get_packet_size(
    buffer: &VecDeque<u8>,
    start_index: usize,
    size_per_sample: usize,
) -> Result<usize, &'static str> {
    let index = start_index + 3;
    if index >= buffer.len() {
        return Err("Insufficient buffer size to extract the packet size.");
    }
    let Some(n_scan_samples) = buffer.get(index) else {
        return Err("Failed to get the number of scan samples.");
    };
    let n = *n_scan_samples as usize;
    Ok((10 + n * size_per_sample) as usize)
}

fn find_start_index(buffer: &VecDeque<u8>) -> Result<usize, &'static str> {
    if buffer.len() == 0 {
        return Err("Buffer is empty.");
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
    Err("Start index could not be found.")
}

pub fn sendable_packet_range(
    buffer: &VecDeque<u8>,
    size_per_sample: usize,
) -> Result<(usize, usize), &'static str> {
    let start_index = find_start_index(buffer)?;
    let size = get_packet_size(buffer, start_index, size_per_sample)?;
    Ok((start_index, size))
}
