// IO-Link (I-Link) FFI bindings  
use super::{FieldbusError, FieldbusProtocol};
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int, c_uint, c_uchar};

// I-Link C API bindings
#[repr(C)]
struct IlinkConfig {
    interface_name: *const c_char,
    baudrate: c_uint,
    cycle_time: c_uint,
}

#[repr(C)]
struct IlinkPortConfig {
    port_number: c_uchar,
    device_id: c_uint,
    vendor_id: c_uint,
    cycle_time: c_uint,
}

extern "C" {
    fn ilink_init(config: *const IlinkConfig) -> c_int;
    fn ilink_port_init(port_config: *const IlinkPortConfig) -> c_int;
    fn ilink_start() -> c_int;
    fn ilink_stop() -> c_int;
    fn ilink_read_process_data(port: c_uchar, data: *mut u8, size: c_uint) -> c_int;
    fn ilink_write_process_data(port: c_uchar, data: *const u8, size: c_uint) -> c_int;
    fn ilink_read_parameter(port: c_uchar, index: c_uint, subindex: c_uchar, data: *mut u8, size: *mut c_uint) -> c_int;
    fn ilink_write_parameter(port: c_uchar, index: c_uint, subindex: c_uchar, data: *const u8, size: c_uint) -> c_int;
    fn ilink_get_port_status(port: c_uchar) -> c_int;
    fn ilink_get_status() -> *const c_char;
}

pub struct IOLinkMaster {
    interface_name: CString,
    baudrate: u32,
    cycle_time: u32,
    initialized: bool,
    connected: bool,
    ports: Vec<IOLinkPort>,
}

#[derive(Clone)]
pub struct IOLinkPort {
    pub port_number: u8,
    pub device_id: u32,
    pub vendor_id: u32,
    pub cycle_time: u32,
    pub configured: bool,
}

impl IOLinkMaster {
    pub fn new(interface_name: &str, baudrate: u32, cycle_time: u32) -> Self {
        Self {
            interface_name: CString::new(interface_name).unwrap(),
            baudrate,
            cycle_time,
            initialized: false,
            connected: false,
            ports: Vec::new(),
        }
    }

    pub fn add_port(&mut self, port_number: u8, device_id: u32, vendor_id: u32, cycle_time: u32) -> Result<(), FieldbusError> {
        let port_config = IlinkPortConfig {
            port_number,
            device_id,
            vendor_id,
            cycle_time,
        };

        let result = unsafe { ilink_port_init(&port_config) };
        if result == 0 {
            self.ports.push(IOLinkPort {
                port_number,
                device_id,
                vendor_id,
                cycle_time,
                configured: true,
            });
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: format!("Failed to configure IO-Link port {}", port_number),
            })
        }
    }

    pub fn read_process_data(&mut self, port: u8, size: u32) -> Result<Vec<u8>, FieldbusError> {
        if !self.connected {
            return Err(FieldbusError {
                code: -1,
                message: "IO-Link master not connected".to_string(),
            });
        }

        let mut buffer = vec![0u8; size as usize];
        let result = unsafe {
            ilink_read_process_data(port, buffer.as_mut_ptr(), size)
        };

        if result == 0 {
            Ok(buffer)
        } else {
            Err(FieldbusError {
                code: result,
                message: format!("Failed to read IO-Link process data from port {}", port),
            })
        }
    }

    pub fn write_process_data(&mut self, port: u8, data: &[u8]) -> Result<(), FieldbusError> {
        if !self.connected {
            return Err(FieldbusError {
                code: -1,
                message: "IO-Link master not connected".to_string(),
            });
        }

        let result = unsafe {
            ilink_write_process_data(port, data.as_ptr(), data.len() as c_uint)
        };

        if result == 0 {
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: format!("Failed to write IO-Link process data to port {}", port),
            })
        }
    }

    pub fn read_parameter(&mut self, port: u8, index: u32, subindex: u8) -> Result<Vec<u8>, FieldbusError> {
        if !self.connected {
            return Err(FieldbusError {
                code: -1,
                message: "IO-Link master not connected".to_string(),
            });
        }

        let mut buffer = vec![0u8; 256]; // Max parameter size
        let mut actual_size = buffer.len() as c_uint;

        let result = unsafe {
            ilink_read_parameter(port, index, subindex, buffer.as_mut_ptr(), &mut actual_size)
        };

        if result == 0 {
            buffer.truncate(actual_size as usize);
            Ok(buffer)
        } else {
            Err(FieldbusError {
                code: result,
                message: format!("Failed to read IO-Link parameter from port {}", port),
            })
        }
    }

    pub fn write_parameter(&mut self, port: u8, index: u32, subindex: u8, data: &[u8]) -> Result<(), FieldbusError> {
        if !self.connected {
            return Err(FieldbusError {
                code: -1,
                message: "IO-Link master not connected".to_string(),
            });
        }

        let result = unsafe {
            ilink_write_parameter(port, index, subindex, data.as_ptr(), data.len() as c_uint)
        };

        if result == 0 {
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: format!("Failed to write IO-Link parameter to port {}", port),
            })
        }
    }

    pub fn get_port_status(&self, port: u8) -> i32 {
        unsafe { ilink_get_port_status(port) }
    }

    pub fn get_configured_ports(&self) -> &[IOLinkPort] {
        &self.ports
    }
}

impl FieldbusProtocol for IOLinkMaster {
    fn initialize(&mut self) -> Result<(), FieldbusError> {
        let config = IlinkConfig {
            interface_name: self.interface_name.as_ptr(),
            baudrate: self.baudrate,
            cycle_time: self.cycle_time,
        };

        let result = unsafe { ilink_init(&config) };
        if result == 0 {
            self.initialized = true;
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to initialize IO-Link master".to_string(),
            })
        }
    }

    fn connect(&mut self) -> Result<(), FieldbusError> {
        if !self.initialized {
            return Err(FieldbusError {
                code: -1,
                message: "IO-Link master not initialized".to_string(),
            });
        }

        let result = unsafe { ilink_start() };
        if result == 0 {
            self.connected = true;
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to start IO-Link master".to_string(),
            })
        }
    }

    fn disconnect(&mut self) -> Result<(), FieldbusError> {
        let result = unsafe { ilink_stop() };
        if result == 0 {
            self.connected = false;
            Ok(())
        } else {
            Err(FieldbusError {
                code: result,
                message: "Failed to stop IO-Link master".to_string(),
            })
        }
    }

    fn read_data(&mut self, address: u16) -> Result<Vec<u8>, FieldbusError> {
        // Default implementation - read process data from port 1
        self.read_process_data(1, address as u32)
    }

    fn write_data(&mut self, _address: u16, data: &[u8]) -> Result<(), FieldbusError> {
        // Default implementation - write process data to port 1
        self.write_process_data(1, data)
    }

    fn get_status(&self) -> String {
        unsafe {
            let status_ptr = ilink_get_status();
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