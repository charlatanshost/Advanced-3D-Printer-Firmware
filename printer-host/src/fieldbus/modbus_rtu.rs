// Modbus RTU (M-Bus) FFI bindings
use super::{FieldbusError, FieldbusProtocol};
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int, c_uint, c_uchar};

// M-Bus C API bindings
#[repr(C)]
struct MbusConfig {
    device: *const c_char,
    baudrate: c_uint,
    parity: c_char,
    data_bits: c_uchar,
    stop_bits: c_uchar,
}

extern "C" {
    fn mbus_init(config: *const MbusConfig) -> c_int;
    fn mbus_connect() -> c_int;
    fn mbus_disconnect() -> c_int;
    fn mbus_read_coils(slave_id: c_uchar, address: c_uint, count: c_uint, data: *mut u8) -> c_int;
    fn mbus_read_discrete_inputs(slave_id: c_uchar, address: c_uint, count: c_uint, data: *mut u8) -> c_int;
    fn mbus_read_holding_registers(slave_id: c_uchar, address: c_uint, count: c_uint, data: *mut u16) -> c_int;
    fn mbus_read_input_registers(slave_id: c_uchar, address: c_uint, count: c_uint, data: *mut u16) -> c_int;
    fn mbus_write_single_coil(slave_id: c_uchar, address: c_uint, value: c_uchar) -> c_int;
    fn mbus_write_single_register(slave_id: c_uchar, address: c_uint, value: c_uint) -> c_int;
    fn mbus_write_multiple_coils(slave_id: c_uchar, address: c_uint, count: c_uint, data: *const u8) -> c_int;
    fn mbus_write_multiple_registers(slave_id: c_uchar, address: c_uint, count: c_uint, data: *const u16) -> c_int;
    fn mbus_get_status() -> *const c_char;
}

pub struct ModbusRtuMaster {
    device: CString,
    baudrate: u32,
    parity: char,
    data_bits: u8,
    stop_bits: u8,
    initialized: bool,
    connected: bool,
}

impl ModbusRtuMaster {
    pub fn new(device: &str, baudrate: u32, parity: char, data_bits: u8, stop_bits: u8) -> Self {
        Self {
            device: CString::new(device).unwrap(),
            baudrate,
            parity,
            data_bits,
            stop_bits,
            initialized: false,
            connected: false,
        }
    }

    pub fn read_coils(&mut self, slave_id: u8, address: u32, count: u32) -> Result<Vec<u8>, FieldbusError> {
        if !self.connected {
            return Err(FieldbusError {
                code: -1,
                message: "Modbus RTU not connected".to_string(),
            });
        }

        let mut buffer = vec![0u8; (count + 7) / 8]; // Coils are packed as bits
        let result = unsafe {
            mbus_read_coils(slave_id, address, count, buffer.as_mut_ptr())
        };

        if result == 0 {
            Ok(buffer)
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to read Modbus coils".to_string(),
            })
        }
    }

    pub fn read_holding_registers(&mut self, slave_id: u8, address: u32, count: u32) -> Result<Vec<u16>, FieldbusError> {
        if !self.connected {
            return Err(FieldbusError {
                code: -1,
                message: "Modbus RTU not connected".to_string(),
            });
        }

        let mut buffer = vec![0u16; count as usize];
        let result = unsafe {
            mbus_read_holding_registers(slave_id, address, count, buffer.as_mut_ptr())
        };

        if result == 0 {
            Ok(buffer)
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to read Modbus holding registers".to_string(),
            })
        }
    }

    pub fn write_single_coil(&mut self, slave_id: u8, address: u32, value: bool) -> Result<(), FieldbusError> {
        if !self.connected {
            return Err(FieldbusError {
                code: -1,
                message: "Modbus RTU not connected".to_string(),
            });
        }

        let result = unsafe {
            mbus_write_single_coil(slave_id, address, if value { 1 } else { 0 })
        };

        if result == 0 {
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to write Modbus coil".to_string(),
            })
        }
    }

    pub fn write_single_register(&mut self, slave_id: u8, address: u32, value: u16) -> Result<(), FieldbusError> {
        if !self.connected {
            return Err(FieldbusError {
                code: -1,
                message: "Modbus RTU not connected".to_string(),
            });
        }

        let result = unsafe {
            mbus_write_single_register(slave_id, address, value as c_uint)
        };

        if result == 0 {
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to write Modbus register".to_string(),
            })
        }
    }

    pub fn write_multiple_registers(&mut self, slave_id: u8, address: u32, values: &[u16]) -> Result<(), FieldbusError> {
        if !self.connected {
            return Err(FieldbusError {
                code: -1,
                message: "Modbus RTU not connected".to_string(),
            });
        }

        let result = unsafe {
            mbus_write_multiple_registers(slave_id, address, values.len() as c_uint, values.as_ptr())
        };

        if result == 0 {
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to write Modbus registers".to_string(),
            })
        }
    }
}

impl FieldbusProtocol for ModbusRtuMaster {
    fn initialize(&mut self) -> Result<(), FieldbusError> {
        let config = MbusConfig {
            device: self.device.as_ptr(),
            baudrate: self.baudrate,
            parity: self.parity as c_char,
            data_bits: self.data_bits,
            stop_bits: self.stop_bits,
        };

        let result = unsafe { mbus_init(&config) };
        if result == 0 {
            self.initialized = true;
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to initialize Modbus RTU".to_string(),
            })
        }
    }

    fn connect(&mut self) -> Result<(), FieldbusError> {
        if !self.initialized {
            return Err(FieldbusError {
                code: -1,
                message: "Modbus RTU not initialized".to_string(),
            });
        }

        let result = unsafe { mbus_connect() };
        if result == 0 {
            self.connected = true;
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to connect Modbus RTU".to_string(),
            })
        }
    }

    fn disconnect(&mut self) -> Result<(), FieldbusError> {
        let result = unsafe { mbus_disconnect() };
        if result == 0 {
            self.connected = false;
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to disconnect Modbus RTU".to_string(),
            })
        }
    }

    fn read_data(&mut self, address: u16) -> Result<Vec<u8>, FieldbusError> {
        // Default implementation - read holding registers from slave 1
        let registers = self.read_holding_registers(1, address as u32, 1)?;
        Ok(registers.iter().flat_map(|r| r.to_be_bytes()).collect())
    }

    fn write_data(&mut self, address: u16, data: &[u8]) -> Result<(), FieldbusError> {
        // Default implementation - write single register to slave 1
        if data.len() >= 2 {
            let value = u16::from_be_bytes([data[0], data[1]]);
            self.write_single_register(1, address as u32, value)
        } else {
            Err(FieldbusError {
                code: -1,
                message: "Insufficient data for register write".to_string(),
            })
        }
    }

    fn get_status(&self) -> String {
        unsafe {
            let status_ptr = mbus_get_status();
            if status_ptr.is_null() {
                "Unknown status".to_string()
            } else {
                CStr::from_ptr(status_ptr)
                    .to_string_lossy()
                    .to_string()
            }
        }
    }
}