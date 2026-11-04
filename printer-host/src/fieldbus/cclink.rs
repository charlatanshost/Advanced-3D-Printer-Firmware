// CC-Link IE (C-Link) FFI bindings
use super::{FieldbusError, FieldbusProtocol};
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int, c_uint, c_uchar};

// C-Link C API bindings
#[repr(C)]
struct ClinkConfig {
    interface_name: *const c_char,
    station_no: c_uchar,
    group_no: c_uchar,
    ip_address: *const c_char,
}

extern "C" {
    fn clink_init(config: *const ClinkConfig) -> c_int;
    fn clink_start() -> c_int;
    fn clink_stop() -> c_int;
    fn clink_read_ry(station: c_uchar, address: c_uint, size: c_uint, data: *mut u8) -> c_int;
    fn clink_write_ry(station: c_uchar, address: c_uint, size: c_uint, data: *const u8) -> c_int;
    fn clink_read_rwr(station: c_uchar, address: c_uint, size: c_uint, data: *mut u16) -> c_int;
    fn clink_write_rwr(station: c_uchar, address: c_uint, size: c_uint, data: *const u16) -> c_int;
    fn clink_get_status() -> *const c_char;
}

pub struct CclinkMaster {
    interface_name: CString,
    station_no: u8,
    group_no: u8,
    ip_address: CString,
    initialized: bool,
    connected: bool,
}

impl CclinkMaster {
    pub fn new(interface_name: &str, station_no: u8, group_no: u8, ip_address: &str) -> Self {
        Self {
            interface_name: CString::new(interface_name).unwrap(),
            station_no,
            group_no,
            ip_address: CString::new(ip_address).unwrap(),
            initialized: false,
            connected: false,
        }
    }

    pub fn read_ry_data(&mut self, station: u8, address: u32, size: u32) -> Result<Vec<u8>, FieldbusError> {
        if !self.connected {
            return Err(FieldbusError {
                code: -1,
                message: "CC-Link IE not connected".to_string(),
            });
        }

        let mut buffer = vec![0u8; size as usize];
        let result = unsafe {
            clink_read_ry(station, address, size, buffer.as_mut_ptr())
        };

        if result == 0 {
            Ok(buffer)
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to read CC-Link IE RY data".to_string(),
            })
        }
    }

    pub fn write_ry_data(&mut self, station: u8, address: u32, data: &[u8]) -> Result<(), FieldbusError> {
        if !self.connected {
            return Err(FieldbusError {
                code: -1,
                message: "CC-Link IE not connected".to_string(),
            });
        }

        let result = unsafe {
            clink_write_ry(station, address, data.len() as c_uint, data.as_ptr())
        };

        if result == 0 {
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to write CC-Link IE RY data".to_string(),
            })
        }
    }

    pub fn read_rwr_data(&mut self, station: u8, address: u32, size: u32) -> Result<Vec<u16>, FieldbusError> {
        if !self.connected {
            return Err(FieldbusError {
                code: -1,
                message: "CC-Link IE not connected".to_string(),
            });
        }

        let mut buffer = vec![0u16; size as usize];
        let result = unsafe {
            clink_read_rwr(station, address, size, buffer.as_mut_ptr())
        };

        if result == 0 {
            Ok(buffer)
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to read CC-Link IE RWr data".to_string(),
            })
        }
    }

    pub fn write_rwr_data(&mut self, station: u8, address: u32, data: &[u16]) -> Result<(), FieldbusError> {
        if !self.connected {
            return Err(FieldbusError {
                code: -1,
                message: "CC-Link IE not connected".to_string(),
            });
        }

        let result = unsafe {
            clink_write_rwr(station, address, data.len() as c_uint, data.as_ptr())
        };

        if result == 0 {
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to write CC-Link IE RWr data".to_string(),
            })
        }
    }
}

impl FieldbusProtocol for CclinkMaster {
    fn initialize(&mut self) -> Result<(), FieldbusError> {
        let config = ClinkConfig {
            interface_name: self.interface_name.as_ptr(),
            station_no: self.station_no,
            group_no: self.group_no,
            ip_address: self.ip_address.as_ptr(),
        };

        let result = unsafe { clink_init(&config) };
        if result == 0 {
            self.initialized = true;
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to initialize CC-Link IE".to_string(),
            })
        }
    }

    fn connect(&mut self) -> Result<(), FieldbusError> {
        if !self.initialized {
            return Err(FieldbusError {
                code: -1,
                message: "CC-Link IE not initialized".to_string(),
            });
        }

        let result = unsafe { clink_start() };
        if result == 0 {
            self.connected = true;
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to start CC-Link IE".to_string(),
            })
        }
    }

    fn disconnect(&mut self) -> Result<(), FieldbusError> {
        let result = unsafe { clink_stop() };
        if result == 0 {
            self.connected = false;
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to stop CC-Link IE".to_string(),
            })
        }
    }

    fn read_data(&mut self, address: u16) -> Result<Vec<u8>, FieldbusError> {
        // Default implementation - read RY data from station 1
        self.read_ry_data(1, address as u32, 4)
    }

    fn write_data(&mut self, address: u16, data: &[u8]) -> Result<(), FieldbusError> {
        // Default implementation - write RY data to station 1
        self.write_ry_data(1, address as u32, data)
    }

    fn get_status(&self) -> String {
        unsafe {
            let status_ptr = clink_get_status();
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