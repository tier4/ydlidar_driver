use crate::numeric::to_u16;
use alloc::string::String;
use alloc::string::ToString;

fn check_packet_size(packet: &[u8], size_per_sample: usize) -> Result<(), String> {
    if packet.len() < 10 {
        return Err("Packet length must be at least 10 bytes.".to_string());
    }

    let n_scan = packet[3] as usize;
    let expected = 10 + size_per_sample * n_scan;
    if packet.len() < expected {
        return Err(format!(
            "Packet length does not match the expected size. \
                     Expected = {}. Actual = {}.",
            expected,
            packet.len()
        ));
    }

    Ok(())
}

fn calc_checksum(packet: &[u8]) -> u16 {
    let n_scan = packet[3] as usize;

    let mut checksum: u16 = to_u16(packet[1], packet[0]);
    checksum ^= to_u16(packet[5], packet[4]);
    for i in 0..n_scan {
        let s0 = packet[10 + 3 * i + 0];
        let s1 = packet[10 + 3 * i + 1];
        let s2 = packet[10 + 3 * i + 2];
        checksum ^= to_u16(0x00, s0);
        checksum ^= to_u16(s2, s1);
    }
    checksum ^= to_u16(packet[3], packet[2]);
    checksum ^= to_u16(packet[7], packet[6]);
    checksum
}

fn calc_checksum_tg15(packet: &[u8]) -> u16 {
    let n_scan = packet[3] as usize;
    let mut checksum = to_u16(packet[1], packet[0]);
    checksum ^= to_u16(packet[5], packet[4]);
    for i in 0..n_scan {
        let s0 = packet[10 + 2 * i + 0];
        let s1 = packet[10 + 2 * i + 1];
        checksum ^= to_u16(s1, s0);
    }
    checksum ^= to_u16(packet[3], packet[2]);
    checksum ^= to_u16(packet[7], packet[6]);
    checksum
}

pub fn err_if_checksum_mismatched(packet: &[u8], size_per_sample: usize) -> Result<(), String> {
    check_packet_size(packet, size_per_sample)?;
    let calculated = calc_checksum_tg15(&packet);
    let expected = to_u16(packet[9], packet[8]);
    if calculated != expected {
        return Err(format!(
            "Checksum mismatched. Calculated = {:04X}, expected = {:04X}.",
            calculated, expected
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_check_packet_size() {
        assert_eq!(
            check_packet_size(&[0xAA, 0x55, 0xB0, 0x00, 0xE3, 0x28, 0xF3, 0x39, 0x0E]),
            Err("Packet length must be at least 10 bytes.".to_string())
        );

        assert_eq!(
            check_packet_size(&[0xAA, 0x55, 0xB0, 0x00, 0xE3, 0x28, 0xF3, 0x39, 0x0E, 0x61]),
            Ok(())
        );

        assert_eq!(
            check_packet_size(&[
                0xAA, 0x55, 0x24, 0x28, 0xF5, 0x4C, 0x85, 0x5E, 0x9D, 0x70, 0xCE, 0xE2
            ]),
            Err("Packet length does not match the expected size. \
                 Expected = 130. Actual = 12."
                .to_string())
        );
    }

    #[test]
    fn test_calc_checksum() {
        let packet = vec![
            0xAA, 0x55, 0xB0, 0x27, 0xE3, 0x28, 0xF3, 0x39, 0x0E, 0x61, 0x79, 0xB6, 0x05, 0x6F,
            0x4E, 0x06, 0x61, 0x06, 0x06, 0x7A, 0x9A, 0x02, 0x9E, 0x5E, 0x02, 0xA6, 0x0A, 0x02,
            0xA7, 0xE6, 0x01, 0xAE, 0xD6, 0x01, 0xBA, 0xD6, 0x01, 0xB8, 0xD2, 0x01, 0xB2, 0xD6,
            0x01, 0xBD, 0xD2, 0x01, 0xDF, 0xDA, 0x01, 0xE1, 0xDA, 0x01, 0xDF, 0xDA, 0x01, 0xDC,
            0xDE, 0x01, 0xDE, 0xDE, 0x01, 0xD8, 0xE2, 0x01, 0xD4, 0xDE, 0x01, 0xBA, 0xDE, 0x01,
            0x84, 0xDF, 0x01, 0x2F, 0xAB, 0x01, 0x17, 0xEE, 0x01, 0x0F, 0x22, 0x02, 0x0C, 0x7E,
            0x02, 0x0A, 0x02, 0x00, 0x0C, 0x9E, 0x02, 0x16, 0xA6, 0x02, 0x21, 0xA2, 0x02, 0x3A,
            0x32, 0x03, 0x55, 0x4E, 0x0A, 0x87, 0x46, 0x0A, 0x85, 0x5A, 0x0A, 0x8A, 0x6E, 0x0A,
            0x84, 0x9A, 0x0A, 0x7E, 0xCE, 0x0A, 0x4E, 0x7E, 0x04, 0x51, 0x6E, 0x03, 0x66, 0xA6,
            0x02,
        ];
        let checksum = calc_checksum(&packet);
        let expected = to_u16(packet[9], packet[8]);
        assert_eq!(checksum, expected);

        let packet = vec![
            0xAA, 0x55, 0x24, 0x28, 0xF5, 0x4C, 0x85, 0x5E, 0x9D, 0x70, 0xCE, 0xE2, 0x07, 0xBC,
            0xFA, 0x07, 0xCC, 0xB6, 0x07, 0xC8, 0xB6, 0x07, 0xC4, 0xBA, 0x07, 0xCB, 0xCA, 0x07,
            0xC8, 0xAE, 0x09, 0xC5, 0x9E, 0x09, 0xC7, 0x9E, 0x09, 0xC2, 0x9E, 0x09, 0xC1, 0x92,
            0x09, 0xC0, 0x8A, 0x09, 0xC1, 0x86, 0x09, 0xBE, 0x86, 0x09, 0xC5, 0x86, 0x09, 0xC3,
            0x8A, 0x09, 0xBC, 0x8A, 0x09, 0xC6, 0x8A, 0x09, 0xC6, 0x8A, 0x09, 0xC2, 0x8E, 0x09,
            0xC5, 0x8E, 0x09, 0xC3, 0x92, 0x09, 0xC4, 0xAA, 0x09, 0xC9, 0xB2, 0x09, 0xC9, 0xBA,
            0x09, 0xC5, 0xC2, 0x09, 0xC9, 0xCE, 0x09, 0xBF, 0xCE, 0x09, 0xBE, 0xCE, 0x09, 0xBA,
            0xCE, 0x09, 0xBE, 0xD6, 0x09, 0xBB, 0xD6, 0x09, 0xBF, 0xE2, 0x09, 0xBB, 0xF2, 0x09,
            0xC1, 0x0A, 0x0A, 0xBF, 0x1A, 0x0A, 0xB9, 0x1E, 0x0A, 0xAA, 0x22, 0x0A, 0x9E, 0x2A,
            0x0A, 0xCB, 0x7A, 0x15,
        ];
        let checksum = calc_checksum(&packet);
        let expected = to_u16(packet[9], packet[8]);
        assert_eq!(checksum, expected);
    }

    #[test]
    fn test_calc_checksum_tg15() {
        let packet = vec![
            0xAA, 0x55, 0x00, 0x28, 0xA9, 0x89, 0x3B, 0x8C, 0x34, 0x78, 0x9D, 0x09, 0xA3, 0x09,
            0x9B, 0x09, 0x9A, 0x09, 0x9C, 0x09, 0x98, 0x09, 0xA0, 0x09, 0x9D, 0x09, 0x8E, 0x09,
            0x8D, 0x09, 0x88, 0x09, 0x88, 0x09, 0x8D, 0x09, 0x88, 0x09, 0x88, 0x09, 0x8D, 0x09,
            0x80, 0x09, 0x89, 0x09, 0x7B, 0x09, 0x82, 0x09, 0x7A, 0x09, 0x85, 0x09, 0x78, 0x09,
            0x7E, 0x09, 0x77, 0x09, 0x7C, 0x09, 0x6F, 0x09, 0x6E, 0x09, 0x6E, 0x09, 0x64, 0x09,
            0x5F, 0x09, 0x43, 0x09, 0x17, 0x09, 0x12, 0x09, 0x05, 0x09, 0x13, 0x09, 0x08, 0x09,
            0x01, 0x09, 0x03, 0x09, 0x05, 0x09, 0xAA, 0x55, 0x00, 0x28, 0x49, 0x8C, 0xE3, 0x8E,
            0xEF, 0x7E, 0x0A, 0x09, 0x0C, 0x09, 0x04, 0x09, 0xFF, 0x08, 0x0B, 0x09, 0x0F, 0x09,
            0x0A, 0x09, 0x0D, 0x09, 0x16, 0x09, 0x08, 0x09, 0x04, 0x09, 0x0F, 0x09, 0x0C, 0x09,
            0xFE, 0x08, 0x0B, 0x09,
        ];
        let checksum = calc_checksum_tg15(&packet);
        let expected = to_u16(packet[9], packet[8]);
        assert_eq!(checksum, expected);
    }
}
