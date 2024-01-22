#[non_exhaustive]

pub enum YdlidarModels {
    // F4,
    // T1,
    // F2,
    // S4,
    // S2_PRO,
    // G4,
    // X4,
    // G4_PRO,
    // F4_PRO,
    // R2,
    // G10,
    // S4B,
    // S2,
    // G6,
    // G2A,
    // G2B,
    // G2C,
    // G4B,
    // G4C,
    // G1,
    // G5,
    // G7,
    // SCL,
    // GS2,
    // GS1,
    // GS5,
    // GS6,
    TG15,
    // TG30,
    // TG50,
    // TEA,
    // TSA,
    // T_MINI,
    TMiniPro,
    // SDM15,
    // T15,
}

impl YdlidarModels {
    pub fn new(model_numer: u8) -> Result<Self, &'static str> {
        match model_numer {
            100 => Ok(YdlidarModels::TG15),
            150 => Ok(YdlidarModels::TMiniPro),
            _ => Err("This device is not supported"),
        }
    }
}
