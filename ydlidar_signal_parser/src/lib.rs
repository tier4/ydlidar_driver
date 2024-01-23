#![no_std]
#[macro_use]
extern crate alloc;

mod buffer;
mod checksum;
mod header;
mod numeric;
mod scan;

pub use buffer::sendable_packet_range;
pub use checksum::err_if_checksum_mismatched;
pub use header::{validate_response_header, HEADER_SIZE};
pub use scan::{
    calc_angles, calc_distances, calc_distances_tg15, get_flags, get_intensities,
    is_beginning_of_cycle, InterferenceFlag, Scan,
};
