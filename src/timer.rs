pub fn sleep_ms(duration: u64) {
    std::thread::sleep(std::time::Duration::from_millis(duration));
}
