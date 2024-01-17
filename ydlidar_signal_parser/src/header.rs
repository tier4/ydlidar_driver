use alloc::string::String;
use alloc::vec::Vec;

pub const HEADER_SIZE: usize = 7;

fn to_string(data: &[u8]) -> String {
    return data
        .iter()
        .map(|e| format!("{:02X}", e))
        .collect::<Vec<_>>()
        .join(" ");
}

pub fn validate_response_header(
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

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    #[test]
    fn test_to_string() {
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
}
