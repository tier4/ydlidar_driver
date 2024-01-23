use crate::ydlidar_models::YdlidarModels;

pub const GET_DEVICE_INFO: u8 = 0x90;
pub const SYNC_BYTE: u8 = 0xA5;

pub struct SystemCommand {
    model_number: YdlidarModels,
}

impl SystemCommand {
    pub fn new(model_number: YdlidarModels) -> Self {
        SystemCommand {
            model_number: model_number,
        }
    }

    pub fn get_devcice_health(&self) -> Result<u8, String> {
        match self.model_number {
            YdlidarModels::TMiniPro => Ok(0x92),
            YdlidarModels::TG15 => Ok(0x91),
        }
    }

    pub fn stop_scan(&self) -> Result<u8, String> {
        match self.model_number {
            YdlidarModels::TMiniPro => Ok(0x65),
            YdlidarModels::TG15 => Ok(0x65),
        }
    }

    pub fn start_scan(&self) -> Result<u8, String> {
        match self.model_number {
            YdlidarModels::TMiniPro => Ok(0x60),
            YdlidarModels::TG15 => Ok(0x60),
        }
    }
}
