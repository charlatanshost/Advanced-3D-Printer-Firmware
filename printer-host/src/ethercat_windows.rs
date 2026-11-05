// Windows EtherCAT Master Implementation using WinPcap/Npcap
// This module provides Windows-compatible EtherCAT communication

use std::sync::{Arc, Mutex};
use std::collections::HashMap;
use std::time::{Duration, Instant};
use tokio::sync::mpsc;
use log::{info, warn, error, debug};

#[cfg(all(windows, feature = "windows-ethercat"))]
use pcap::{Device, Capture, Active};
#[cfg(all(windows, feature = "windows-ethercat"))]
use winapi::um::iphlpapi::GetAdaptersInfo;
#[cfg(all(windows, feature = "windows-ethercat"))]
use winapi::um::iptypes::IP_ADAPTER_INFO;

/// Windows-specific EtherCAT network adapter information
#[derive(Debug, Clone)]
pub struct WindowsEtherCATAdapter {
    pub name: String,
    pub description: String,
    pub mac_address: [u8; 6],
    pub is_up: bool,
    pub is_ethernet: bool,
}

/// Windows EtherCAT Master state
#[derive(Debug, Clone)]
pub struct WindowsEtherCATMaster {
    adapter: Option<WindowsEtherCATAdapter>,
    slaves: HashMap<u16, EtherCATSlaveInfo>,
    cycle_time_us: u32,
    is_running: bool,
    error_count: u32,
    last_cycle: Option<Instant>,
}

/// EtherCAT slave information
#[derive(Debug, Clone)]
pub struct EtherCATSlaveInfo {
    pub station_address: u16,
    pub vendor_id: u32,
    pub product_code: u32,
    pub revision_number: u32,
    pub serial_number: u32,
    pub input_size: u16,
    pub output_size: u16,
    pub input_data: Vec<u8>,
    pub output_data: Vec<u8>,
    pub state: EtherCATSlaveState,
    pub last_seen: Option<Instant>,
    pub error_count: u32,
}

/// EtherCAT slave states according to ETG.1000.6
#[derive(Debug, Clone, PartialEq)]
pub enum EtherCATSlaveState {
    Init = 0x01,
    PreOp = 0x02,
    BootStrap = 0x03,
    SafeOp = 0x04,
    Op = 0x08,
    Unknown = 0xFF,
}

/// EtherCAT frame structure
#[repr(C)]
#[derive(Debug, Clone)]
pub struct EtherCATFrame {
    pub ethernet_header: [u8; 14],  // Destination MAC (6) + Source MAC (6) + EtherType (2)
    pub ethercat_header: [u8; 2],   // Length and Type
    pub datagram: Vec<u8>,          // EtherCAT datagram
    pub working_counter: u16,       // Working counter
}

impl WindowsEtherCATMaster {
    pub fn new() -> Self {
        Self {
            adapter: None,
            slaves: HashMap::new(),
            cycle_time_us: 1000, // 1ms default
            is_running: false,
            error_count: 0,
            last_cycle: None,
        }
    }

    /// Discover available EtherCAT-capable network adapters on Windows
    /// Takes an optional npcap_path parameter to specify where to find Npcap libraries
    pub fn discover_adapters() -> Result<Vec<WindowsEtherCATAdapter>, anyhow::Error> {
        Self::discover_adapters_with_npcap_path(None, true)
    }

    /// Discover adapters with specific Npcap path and enable/disable real Npcap usage
    pub fn discover_adapters_with_npcap_path(
        npcap_path: Option<&str>,
        use_npcap: bool
    ) -> Result<Vec<WindowsEtherCATAdapter>, anyhow::Error> {
        let mut adapters = Vec::new();

        #[cfg(all(windows, feature = "windows-ethercat"))]
        {
            if use_npcap {
                info!("Attempting to use real Npcap for EtherCAT adapter discovery");
                if let Some(path) = npcap_path {
                    info!("Using Npcap path: {}", path);
                    // In a full implementation, we would set the library path here
                    // For now, we'll proceed with the default pcap behavior
                }
            } else {
                info!("Using mock EtherCAT adapters (Npcap disabled)");
            }

            // Use pcap to enumerate network devices (only if use_npcap is true)
            if use_npcap {
                match Device::list() {
                    Ok(devices) => {
                        for device in devices {
                            if let Some(desc) = &device.desc {
                                // Filter for Ethernet devices that could support EtherCAT
                                if desc.to_lowercase().contains("ethernet") ||
                                   desc.to_lowercase().contains("intel") ||
                                   desc.to_lowercase().contains("realtek") ||
                                   desc.to_lowercase().contains("broadcom") {

                                    let adapter = WindowsEtherCATAdapter {
                                        name: device.name.clone(),
                                        description: desc.clone(),
                                        mac_address: [0, 0, 0, 0, 0, 0], // Would need to get actual MAC
                                        is_up: true, // Assume up for now
                                        is_ethernet: true,
                                    };
                                    adapters.push(adapter);
                                }
                            }
                        }
                    }
                    Err(e) => {
                        warn!("Failed to enumerate network devices: {}", e);
                    }
                }
            }
            } // Close the if use_npcap block

        // Use mock adapters when pcap is not available OR when use_npcap is false
        let use_mock = if cfg!(not(all(windows, feature = "windows-ethercat"))) {
            true
        } else {
            !use_npcap
        };

        if use_mock {
            if !use_npcap {
                info!("Using mock EtherCAT adapters (Npcap disabled by user)");
            } else {
                info!("Using mock EtherCAT adapters (Npcap not installed)");
            }
            adapters.push(WindowsEtherCATAdapter {
                name: "mock_adapter_0".to_string(),
                description: "Mock Ethernet Adapter (Intel I219-V Compatible)".to_string(),
                mac_address: [0x00, 0x1B, 0x21, 0x3C, 0x4D, 0x5E],
                is_up: true,
                is_ethernet: true,
            });
            adapters.push(WindowsEtherCATAdapter {
                name: "mock_adapter_1".to_string(),
                description: "Mock Realtek PCIe GbE Family Controller".to_string(),
                mac_address: [0x08, 0x00, 0x27, 0x12, 0x34, 0x56],
                is_up: true,
                is_ethernet: true,
            });
        }

        Ok(adapters)
    }

    /// Connect to a specific network adapter for EtherCAT communication
    pub fn connect_adapter(&mut self, adapter_name: &str) -> Result<(), anyhow::Error> {
        let adapters = Self::discover_adapters()?;

        if let Some(adapter) = adapters.into_iter().find(|a| a.name == adapter_name) {
            info!("Connecting to EtherCAT adapter: {}", adapter.description);
            self.adapter = Some(adapter);
            Ok(())
        } else {
            Err(anyhow::anyhow!("Adapter '{}' not found", adapter_name))
        }
    }

    /// Start EtherCAT master operation
    pub async fn start(&mut self) -> Result<(), anyhow::Error> {
        if self.adapter.is_none() {
            return Err(anyhow::anyhow!("No adapter connected"));
        }

        warn!("Windows EtherCAT master: slave scanning only. Cyclic exchange is simulated, so no outputs reach the drives.");
        self.is_running = true;

        // In a real implementation, this would:
        // 1. Open the network adapter with pcap
        // 2. Start the cyclic EtherCAT communication
        // 3. Scan for slaves
        // 4. Configure slaves
        // 5. Start operational mode

        // For now, simulate a successful start
        self.last_cycle = Some(Instant::now());

        Ok(())
    }

    /// Stop EtherCAT master operation
    pub fn stop(&mut self) {
        info!("Stopping Windows EtherCAT Master");
        self.is_running = false;
        self.slaves.clear();
    }

    /// Scan for EtherCAT slaves on the network
    pub async fn scan_slaves(&mut self) -> Result<Vec<u16>, anyhow::Error> {
        if !self.is_running {
            return Err(anyhow::anyhow!("EtherCAT master not running"));
        }

        info!("Scanning for EtherCAT slaves...");
        self.slaves.clear(); // Clear any existing slaves

        let mut found_slaves = Vec::new();

        #[cfg(all(windows, feature = "windows-ethercat"))]
        {
            // Try to perform real EtherCAT slave scanning
            if let Some(adapter) = &self.adapter {
                info!("Performing real EtherCAT slave scan on adapter: {}", adapter.name);

                // Attempt to open capture on the adapter
                match Device::list() {
                    Ok(devices) => {
                        for device in devices {
                            if device.name == adapter.name {
                                // The read timeout matters: with none, next_packet would block
                                // forever on a quiet network and this could never finish.
                                match Capture::from_device(device).map(|cap| cap.timeout(100)) {
                                    Ok(cap) => {
                                        info!("Opened capture on EtherCAT adapter");

                                        // Activate the capture for sending/receiving
                                        match cap.open() {
                                            Ok(mut active_cap) => {
                                                // Active capture is ready to use

                                                // Perform actual EtherCAT slave discovery using proper protocol
                                                info!("Starting comprehensive EtherCAT slave scan...");

                                                // Use multiple scan strategies for better detection
                                                let scan_attempts = vec![
                                                    // 1. Auto-increment read of slave count register
                                                    ("Slave Count Scan", Self::create_slave_count_frame(&adapter.mac_address)),
                                                    // 2. Auto-increment read of device type register
                                                    ("Device Type Scan", Self::create_device_type_frame(&adapter.mac_address)),
                                                    // 3. Broadcast read for any EtherCAT device
                                                    ("Broadcast Scan", Self::create_broadcast_frame(&adapter.mac_address)),
                                                ];

                                                for (scan_name, ethernet_frame) in scan_attempts {
                                                    info!("Attempting {}", scan_name);

                                                    // Try to send the frame
                                                    match active_cap.sendpacket(ethernet_frame.as_slice()) {
                                                        Ok(_) => {
                                                            info!("Sent {} frame", scan_name);

                                                            // Wait a bit for device response
                                                            std::thread::sleep(std::time::Duration::from_millis(100));

                                                            // Listen for responses
                                                            let start_time = std::time::Instant::now();
                                                            while start_time.elapsed() < std::time::Duration::from_millis(1000) {
                                                                match active_cap.next_packet() {
                                                                    Ok(packet) => {
                                                                        // Check if this is an EtherCAT response
                                                                        if packet.data.len() > 16 &&
                                                                           packet.data[12] == 0x88 && packet.data[13] == 0xA4 {
                                                                            info!("Received EtherCAT response packet from {}! Length: {}", scan_name, packet.data.len());

                                                                            // Parse the response to identify slaves
                                                                            if packet.data.len() > 28 {
                                                                                let working_counter = Self::parse_working_counter(packet.data).unwrap_or(0);

                                                                                info!("Working counter: {}", working_counter);

                                                                                if working_counter > 0 {
                                                                                    info!("🎉 Found {} responding EtherCAT slave(s) via {}!", working_counter, scan_name);

                                                                                    // Create slave entries for each responding slave
                                                                                    for i in 1..=working_counter {
                                                                                        let slave = EtherCATSlaveInfo {
                                                                                            station_address: i,
                                                                                            vendor_id: 0x00000000, // Not read yet
                                                                                            product_code: 0x00000000, // To be read later
                                                                                            revision_number: 0x00000000,
                                                                                            serial_number: 0x00000000,
                                                                                            input_size: 0,
                                                                                            output_size: 0,
                                                                                            input_data: vec![],
                                                                                            output_data: vec![],
                                                                                            state: EtherCATSlaveState::Init,
                                                                                            last_seen: Some(Instant::now()),
                                                                                            error_count: 0,
                                                                                        };

                                                                                        info!("Detected real EtherCAT slave at position {}", i);
                                                                                        found_slaves.push(i);
                                                                                        self.slaves.insert(i, slave);
                                                                                    }
                                                                                }
                                                                            }
                                                                            break;
                                                                        }
                                                                    }
                                                                    Err(pcap::Error::TimeoutExpired) => {
                                                                        // Timeout is expected, continue
                                                                        break;
                                                                    }
                                                                    Err(e) => {
                                                                        warn!("Error reading packet: {}", e);
                                                                        break;
                                                                    }
                                                                }

                                                                // Small delay to avoid overwhelming the network
                                                                std::thread::sleep(std::time::Duration::from_millis(1));
                                                            }
                                                        }
                                                        Err(e) => {
                                                            warn!("Failed to send EtherCAT discovery frame: {}", e);
                                                        }
                                                    }
                                                }
                                            }
                                            Err(e) => {
                                                warn!("Failed to open capture on EtherCAT adapter: {}", e);
                                            }
                                        }
                                    }
                                    Err(e) => {
                                        warn!("Failed to create capture from device: {}", e);
                                    }
                                }
                            }
                        }
                    }
                    Err(e) => {
                        warn!("Failed to list devices for EtherCAT scan: {}", e);
                    }
                }
            }
        }

        // If no real slaves found, inform user
        if found_slaves.is_empty() {
            warn!("No EtherCAT slaves detected on the network");
            warn!("Please check:");
            warn!("1. EtherCAT modules are powered on");
            warn!("2. Ethernet cable is connected to EtherCAT IN port");
            warn!("3. EtherCAT modules are properly terminated");
            warn!("4. Module is in correct state (some need INIT state)");
            warn!("5. Network adapter is not filtering EtherCAT traffic");

            // Also log technical details for debugging
            info!("Scanning details:");
            info!("- Adapter used: {}", self.adapter.as_ref().map(|a| a.name.as_str()).unwrap_or("None"));
            info!("- Packets sent: EtherCAT discovery frames with 0x88A4 ethertype");
            info!("- Timeout: 500ms for responses");
            info!("- Expected: Working counter > 0 in response packets");
        } else {
            info!("EtherCAT slave scan completed successfully");
        }

        Ok(found_slaves)
    }

    /// Configure a slave to operational state
    pub async fn configure_slave(&mut self, station_address: u16) -> Result<(), anyhow::Error> {
        if let Some(slave) = self.slaves.get_mut(&station_address) {
            // This would need to take the slave through PREOP (with PDO mapping) and SAFEOP
            // to OP. Nothing does yet, so it must not report that the slave is operational.
            slave.state = EtherCATSlaveState::Init;
            Err(anyhow::anyhow!("Taking slave {} to operational state is not implemented on Windows", station_address))
        } else {
            Err(anyhow::anyhow!("Slave {} not found", station_address))
        }
    }

    /// Read input data from a slave
    pub fn read_slave_inputs(&self, station_address: u16) -> Option<Vec<u8>> {
        self.slaves.get(&station_address).map(|slave| slave.input_data.clone())
    }

    /// Write output data to a slave
    pub fn write_slave_outputs(&mut self, station_address: u16, data: Vec<u8>) -> Result<(), anyhow::Error> {
        if let Some(slave) = self.slaves.get_mut(&station_address) {
            if data.len() == slave.output_size as usize {
                slave.output_data = data;
                slave.last_seen = Some(Instant::now());
                Ok(())
            } else {
                Err(anyhow::anyhow!("Data size mismatch: expected {} bytes, got {}",
                           slave.output_size, data.len()))
            }
        } else {
            Err(anyhow::anyhow!("Slave {} not found", station_address))
        }
    }

    /// Get current status of all slaves
    pub fn get_slave_status(&self) -> HashMap<u16, (EtherCATSlaveState, u32)> {
        self.slaves.iter()
            .map(|(&addr, slave)| (addr, (slave.state.clone(), slave.error_count)))
            .collect()
    }

    /// Simulate cyclic EtherCAT operation
    pub async fn run_cyclic(&mut self) {
        if !self.is_running {
            return;
        }

        let cycle_duration = Duration::from_micros(self.cycle_time_us as u64);

        loop {
            if !self.is_running {
                break;
            }

            let cycle_start = Instant::now();

            // Simulate reading inputs and writing outputs for all slaves
            for (addr, slave) in &mut self.slaves {
                if slave.state == EtherCATSlaveState::Op {
                    // Simulate some input data changes
                    if slave.input_size > 0 {
                        slave.input_data[0] = (cycle_start.elapsed().as_millis() % 256) as u8;
                    }
                    slave.last_seen = Some(cycle_start);
                }
            }

            self.last_cycle = Some(cycle_start);

            // Wait for next cycle
            let elapsed = cycle_start.elapsed();
            if elapsed < cycle_duration {
                tokio::time::sleep(cycle_duration - elapsed).await;
            } else {
                warn!("EtherCAT cycle overrun: {}μs", elapsed.as_micros());
                self.error_count += 1;
            }
        }
    }

    /// Check if master is running
    pub fn is_running(&self) -> bool {
        self.is_running
    }

    /// Build one EtherCAT frame holding a single read datagram.
    ///
    /// The frame header is 16 bits, little-endian: 11 bits of length and a 4-bit type at the top.
    /// The datagram is command, index, 16-bit position, 16-bit register, 16-bit length, 16-bit
    /// IRQ (10 bytes), then the data and a 16-bit working counter.
    pub fn build_read_frame(mac_address: &[u8; 6], command: u8, register: u16, data_length: u16) -> Vec<u8> {
        const ETHERCAT_COMMAND_TYPE: u16 = 1;
        let mut frame = Vec::new();

        // Ethernet header
        frame.extend_from_slice(&[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]); // Destination MAC (broadcast)
        frame.extend_from_slice(mac_address); // Source MAC
        frame.extend_from_slice(&[0x88, 0xA4]); // EtherType (EtherCAT)

        // EtherCAT header
        let payload_length = 10 + data_length + 2;
        frame.extend_from_slice(&((ETHERCAT_COMMAND_TYPE << 12) | payload_length).to_le_bytes());

        // Datagram
        frame.extend_from_slice(&[command, 0x00]); // Command, index
        frame.extend_from_slice(&0u16.to_le_bytes()); // Position: first slave
        frame.extend_from_slice(&register.to_le_bytes()); // Register
        frame.extend_from_slice(&data_length.to_le_bytes());
        frame.extend_from_slice(&0u16.to_le_bytes()); // IRQ
        frame.resize(frame.len() + data_length as usize, 0); // Data
        frame.extend_from_slice(&0u16.to_le_bytes()); // Working counter

        // Ethernet's minimum frame size, excluding the CRC the adapter adds
        if frame.len() < 60 {
            frame.resize(60, 0);
        }
        frame
    }

    /// The working counter of a received frame. It sits where the header's length says the
    /// datagram ends, not at the end of the packet, since short frames arrive padded.
    pub fn parse_working_counter(packet: &[u8]) -> Option<u16> {
        const HEADERS: usize = 14 + 2;
        if packet.len() < HEADERS || packet[12] != 0x88 || packet[13] != 0xA4 {
            return None;
        }
        let payload_length = (u16::from_le_bytes([packet[14], packet[15]]) & 0x07FF) as usize;
        let end = HEADERS + payload_length;
        if payload_length < 2 || end > packet.len() {
            return None;
        }
        Some(u16::from_le_bytes([packet[end - 2], packet[end - 1]]))
    }

    /// Create an EtherCAT frame for slave count scanning
    fn create_slave_count_frame(mac_address: &[u8; 6]) -> Vec<u8> {
        const APRD: u8 = 0x01;
        Self::build_read_frame(mac_address, APRD, 0x0000, 2)
    }

    /// Create an EtherCAT frame for device type scanning
    fn create_device_type_frame(mac_address: &[u8; 6]) -> Vec<u8> {
        const APRD: u8 = 0x01;
        Self::build_read_frame(mac_address, APRD, 0x0000, 4)
    }

    /// Create a broadcast EtherCAT frame for general scanning
    fn create_broadcast_frame(mac_address: &[u8; 6]) -> Vec<u8> {
        const BRD: u8 = 0x07;
        Self::build_read_frame(mac_address, BRD, 0x0000, 8)
    }

    /// Get current cycle time
    pub fn get_cycle_time(&self) -> u32 {
        self.cycle_time_us
    }

    /// Set cycle time
    pub fn set_cycle_time(&mut self, cycle_time_us: u32) {
        self.cycle_time_us = cycle_time_us;
    }

    /// Get adapter information
    pub fn get_adapter(&self) -> Option<&WindowsEtherCATAdapter> {
        self.adapter.as_ref()
    }

    /// Get error count
    pub fn get_error_count(&self) -> u32 {
        self.error_count
    }
}

impl Default for WindowsEtherCATMaster {
    fn default() -> Self {
        Self::new()
    }
}

/// Create EtherCAT frame for Windows implementation
impl EtherCATFrame {
    pub fn new_broadcast_read(address: u16, length: u16) -> Self {
        let mut frame = EtherCATFrame {
            ethernet_header: [0; 14],
            ethercat_header: [0; 2],
            datagram: Vec::new(),
            working_counter: 0,
        };

        // Set EtherCAT EtherType (0x88A4)
        frame.ethernet_header[12] = 0x88;
        frame.ethernet_header[13] = 0xA4;

        // Create basic read datagram
        frame.datagram.extend_from_slice(&[
            0x01,                           // Command: APRD (Auto-increment Physical Read)
            0x00,                           // Index
            (address & 0xFF) as u8,         // Address low
            ((address >> 8) & 0xFF) as u8,  // Address high
            (length & 0xFF) as u8,          // Length low
            ((length >> 8) & 0xFF) as u8,   // Length high
            0x00, 0x00,                     // Reserved/IRQ
        ]);

        // Add data bytes (filled with zeros for read)
        frame.datagram.extend(vec![0; length as usize]);

        frame
    }
}