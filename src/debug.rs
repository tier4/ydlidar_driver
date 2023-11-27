pub fn to_string(data: &[u8]) -> String {
    return data.iter().map(|e| format!("{:02X}", e)).collect::<Vec<_>>().join(" ");
}

#[cfg(test)]
mod tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    fn test_split() {
        let s = to_string(&[0xAA, 0x55, 0x00, 0x28]);
        assert_eq!(s, "AA 55 00 28");
    }
}
