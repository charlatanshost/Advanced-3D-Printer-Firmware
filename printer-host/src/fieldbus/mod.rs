// Fieldbus protocol FFI bindings for RT-Labs libraries
pub mod profinet;
pub mod modbus_rtu;
pub mod canopen;
pub mod cclink;
pub mod iolink;

// Common fieldbus types and traits
use std::ffi::CString;

#[derive(Debug, Clone)]
pub struct FieldbusError {
    pub code: i32,
    pub message: String,
}

pub trait FieldbusProtocol {
    fn initialize(&mut self) -> Result<(), FieldbusError>;
    fn connect(&mut self) -> Result<(), FieldbusError>;
    fn disconnect(&mut self) -> Result<(), FieldbusError>;
    fn read_data(&mut self, address: u16) -> Result<Vec<u8>, FieldbusError>;
    fn write_data(&mut self, address: u16, data: &[u8]) -> Result<(), FieldbusError>;
    fn get_status(&self) -> String;
}

// Safety wrapper for C string conversion
pub(crate) fn to_c_string(s: &str) -> Result<CString, FieldbusError> {
    CString::new(s).map_err(|_| FieldbusError {
        code: -1,
        message: "Invalid string for C conversion".to_string(),
    })
}