// CANopen (C-Open) FFI bindings
use super::{FieldbusError, FieldbusProtocol};
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int, c_uint, c_uchar};

// C-Open C API bindings
#[repr(C)]
struct CopenConfig {
    node_id: c_uchar,
    baudrate: c_uint,
    interface: *const c_char,
}

extern "C" {
    fn copen_init(config: *const CopenConfig) -> c_int;
    fn copen_start() -> c_int;
    fn copen_stop() -> c_int;
    fn copen_sdo_read(node_id: c_uchar, index: c_uint, subindex: c_uchar, data: *mut u8, size: *mut c_uint) -> c_int;
    fn copen_sdo_write(node_id: c_uchar, index: c_uint, subindex: c_uchar, data: *const u8, size: c_uint) -> c_int;
    fn copen_pdo_read(node_id: c_uchar, data: *mut u8, size: c_uint) -> c_int;
    fn copen_pdo_write(node_id: c_uchar, data: *const u8, size: c_uint) -> c_int;
    fn copen_get_status() -> *const c_char;
}

pub struct CanopenMaster {
    node_id: u8,
    baudrate: u32,
    interface: CString,
    initialized: bool,
    connected: bool,
}

impl CanopenMaster {
    pub fn new(node_id: u8, baudrate: u32, interface: &str) -> Self {
        Self {
            node_id,
            baudrate,
            interface: CString::new(interface).unwrap(),
            initialized: false,
            connected: false,
        }
    }

    pub fn sdo_read(&mut self, node_id: u8, index: u32, subindex: u8) -> Result<Vec<u8>, FieldbusError> {
        if !self.connected {
            return Err(FieldbusError {
                code: -1,
                message: "CANopen not connected".to_string(),
            });
        }

        let mut buffer = vec![0u8; 256]; // Max SDO size
        let mut actual_size = buffer.len() as c_uint;

        let result = unsafe {
            copen_sdo_read(node_id, index, subindex, buffer.as_mut_ptr(), &mut actual_size)
        };

        if result == 0 {
            buffer.truncate(actual_size as usize);
            Ok(buffer)
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to read CANopen SDO".to_string(),
            })
        }
    }

    pub fn sdo_write(&mut self, node_id: u8, index: u32, subindex: u8, data: &[u8]) -> Result<(), FieldbusError> {
        if !self.connected {
            return Err(FieldbusError {
                code: -1,
                message: "CANopen not connected".to_string(),
            });
        }

        let result = unsafe {
            copen_sdo_write(node_id, index, subindex, data.as_ptr(), data.len() as c_uint)
        };

        if result == 0 {
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to write CANopen SDO".to_string(),
            })
        }
    }

    pub fn pdo_read(&mut self, node_id: u8, size: usize) -> Result<Vec<u8>, FieldbusError> {
        if !self.connected {
            return Err(FieldbusError {
                code: -1,
                message: "CANopen not connected".to_string(),
            });
        }

        let mut buffer = vec![0u8; size];
        let result = unsafe {
            copen_pdo_read(node_id, buffer.as_mut_ptr(), size as c_uint)
        };

        if result == 0 {
            Ok(buffer)
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to read CANopen PDO".to_string(),
            })
        }
    }

    pub fn pdo_write(&mut self, node_id: u8, data: &[u8]) -> Result<(), FieldbusError> {
        if !self.connected {
            return Err(FieldbusError {
                code: -1,
                message: "CANopen not connected".to_string(),
            });
        }

        let result = unsafe {
            copen_pdo_write(node_id, data.as_ptr(), data.len() as c_uint)
        };

        if result == 0 {
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to write CANopen PDO".to_string(),
            })
        }
    }
}

impl FieldbusProtocol for CanopenMaster {
    fn initialize(&mut self) -> Result<(), FieldbusError> {
        let config = CopenConfig {
            node_id: self.node_id,
            baudrate: self.baudrate,
            interface: self.interface.as_ptr(),
        };

        let result = unsafe { copen_init(&config) };
        if result == 0 {
            self.initialized = true;
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to initialize CANopen".to_string(),
            })
        }
    }

    fn connect(&mut self) -> Result<(), FieldbusError> {
        if !self.initialized {
            return Err(FieldbusError {
                code: -1,
                message: "CANopen not initialized".to_string(),
            });
        }

        let result = unsafe { copen_start() };
        if result == 0 {
            self.connected = true;
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to start CANopen".to_string(),
            })
        }
    }

    fn disconnect(&mut self) -> Result<(), FieldbusError> {
        let result = unsafe { copen_stop() };
        if result == 0 {
            self.connected = false;
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to stop CANopen".to_string(),
            })
        }
    }

    fn read_data(&mut self, address: u16) -> Result<Vec<u8>, FieldbusError> {
        // Default implementation - read PDO from node 1
        self.pdo_read(1, address as usize)
    }

    fn write_data(&mut self, _address: u16, data: &[u8]) -> Result<(), FieldbusError> {
        // Default implementation - write PDO to node 1
        self.pdo_write(1, data)
    }

    fn get_status(&self) -> String {
        unsafe {
            let status_ptr = copen_get_status();
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