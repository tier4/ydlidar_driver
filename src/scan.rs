fn to_angle(bit1: u8, bit2: u8) -> f64 {
    let a = ((bit1 as u16) + ((bit2 as u16) << 8)) >> 1;
    return (a as f64) / 64.;
}

fn n_scan_samples(packet: &[u8]) -> usize {
    packet[3] as usize
}

fn degree_to_radian(degree: f64) -> f64 {
    degree * std::f64::consts::PI / 180.
}

pub fn calc_angles(packet: &[u8], angles_radian: &mut Vec<f64>) {
    let n = n_scan_samples(packet);
    if n == 1 {
        assert_eq!(packet[4], packet[6]);
        assert_eq!(packet[5], packet[7]);
        let angle_degree = to_angle(packet[4], packet[5]);
        let angle_radian = degree_to_radian(angle_degree);
        angles_radian.push(angle_radian);
        return;
    }

    let start_angle = to_angle(packet[4], packet[5]);
    let end_angle = to_angle(packet[6], packet[7]);

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

/// Interference flag corresponding to the scan signal.
#[derive(Clone, Debug, PartialEq)]
pub enum InterferenceFlag {
    /// The signal has the interference of specular reflection
    SpecularReflection,
    /// The signal is interfered by ambient light
    AmbientLight,
    /// Interference was not observed
    Nothing,
}

fn to_flag(value: u8) -> InterferenceFlag {
    if value == 2 {
        return InterferenceFlag::SpecularReflection;
    }
    if value == 3 {
        return InterferenceFlag::AmbientLight;
    }
    InterferenceFlag::Nothing
}

pub fn get_flags(packet: &[u8], flags: &mut Vec<InterferenceFlag>) {
    for i in scan_indices(n_scan_samples(packet)) {
        flags.push(to_flag(packet[i + 1] & 0x03));
    }
}

pub fn get_intensities(packet: &[u8], intensities: &mut Vec<u8>) {
    for i in scan_indices(n_scan_samples(packet)) {
        intensities.push(packet[i] as u8)
    }
}

fn calc_distance(b1: u8, b2: u8) -> u16 {
    ((b2 as u16) << 6) + ((b1 as u16) >> 2)
}

pub fn calc_distances(packet: &[u8], distances: &mut Vec<u16>) {
    for i in scan_indices(n_scan_samples(packet)) {
        let d = calc_distance(packet[i + 1], packet[i + 2]);
        distances.push(d);
    }
}

pub fn is_packet_header(element0: u8, element1: u8) -> bool {
    element0 == 0xAA && element1 == 0x55
}

pub fn is_beginning_of_cycle(packet: &[u8]) -> bool {
    packet[2] & 0x01 == 1
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_to_flag() {
        assert_eq!(to_flag(2), InterferenceFlag::SpecularReflection);
        assert_eq!(to_flag(3), InterferenceFlag::AmbientLight);
        assert_eq!(to_flag(1), InterferenceFlag::Nothing);
    }
}
