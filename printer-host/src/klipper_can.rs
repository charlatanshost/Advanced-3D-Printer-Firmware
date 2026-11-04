// Klipper CAN Bus Toolhead Implementation
// This module implements real-time communication with Klipper-compatible CAN bus toolheads

use std::collections::HashMap;
use std::time::{Duration, Instant};
use std::sync::{Arc, Mutex};
use heapless::{Vec as _HeaplessVec, spsc::{Producer as _Producer, Consumer as _Consumer}};
use tokio::sync::mpsc;
use log::{info, warn, error, debug};

// Klipper CAN Protocol Constants
pub const KLIPPER_CAN_ID_MASK: u32 = 0x1FFFFFFF;
pub const KLIPPER_CAN_ID_SHIFT: u8 = 8;
pub const KLIPPER_CAN_UUID_MASK: u32 = 0x00FFFFFF;

// Klipper Command Types (based on src/generic/serial_can.c)
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum KlipperCanCmd {
    // Motion commands
    MoveQueue = 0x01,
    Home = 0x02,
    Move = 0x03,
    SetPosition = 0x04,
    
    // Temperature commands
    SetTemperature = 0x10,
    GetTemperature = 0x11,
    SetFan = 0x12,
    GetSensors = 0x13,
    
    // Extruder commands
    MoveExtruder = 0x20,
    RetractExtruder = 0x21,
    SetPressureAdvance = 0x22,
    
    // Status and control
    GetStatus = 0x30,
    EmergencyStop = 0x31,
    Reset = 0x32,
    GetConfig = 0x33,
    
    // Discovery and identification
    QueryUuid = 0xF0,
    SetUuid = 0xF1,
    GetVersion = 0xF2,
    
    // Response/Error codes
    Response = 0x80,
    Error = 0xFF,
}

// CAN Message Structure for Klipper Protocol
#[derive(Debug, Clone)]
pub struct KlipperCanMessage {
    pub uuid: u32,              // Toolhead UUID (24-bit)
    pub cmd: KlipperCanCmd,      // Command type
    pub seq: u8,                 // Sequence number
    pub data: [u8; 7],          // Payload (7 bytes max after cmd byte)
    pub len: u8,                 // Data length
    pub timestamp: Instant,      // For latency measurement
}

// Toolhead Configuration
#[derive(Debug, Clone)]
pub struct ToolheadConfig {
    pub uuid: u32,
    pub name: String,
    pub can_bitrate: u32,        // Usually 250000 or 500000
    pub max_temp: f32,
    pub min_extrude_temp: f32,
    pub max_extrude_speed: f32,
    pub pressure_advance: f32,
    pub extruder_steps_per_mm: f32,
    pub capabilities: ToolheadCapabilities,
}

#[derive(Debug, Clone)]
pub struct ToolheadCapabilities {
    pub has_extruder: bool,
    pub has_hotend: bool,
    pub has_fan: bool,
    pub has_probe: bool,
    pub has_accelerometer: bool,
    pub max_accel: f32,
    pub max_velocity: f32,
}

// Toolhead State
#[derive(Debug, Clone)]
pub struct ToolheadState {
    pub uuid: u32,
    pub connected: bool,
    pub temperature: f32,
    pub target_temperature: f32,
    pub fan_speed: u8,           // 0-255
    pub position: [f32; 4],      // X, Y, Z, E
    pub last_command_time: Instant,
    pub error_count: u32,
    pub firmware_version: String,
}

// Toolhead Statistics
#[derive(Debug, Clone)]
pub struct ToolheadStats {
    pub messages_sent: u64,
    pub messages_received: u64,
    pub bytes_sent: u64,
    pub bytes_received: u64,
    pub average_latency_ms: f64,
    pub error_count: u32,
    pub last_error: Option<String>,
    pub uptime: Duration,
    pub connection_time: Instant,
}

// CAN Bus Interface Abstraction
pub trait CanBusInterface: Send + Sync {
    fn send_frame(&mut self, id: u32, data: &[u8]) -> Result<(), String>;
    fn receive_frame(&mut self) -> Option<(u32, Vec<u8>)>;
    fn set_bitrate(&mut self, bitrate: u32) -> Result<(), String>;
    fn set_filter(&mut self, filter_id: u32, mask: u32) -> Result<(), String>;
    fn get_error_count(&self) -> (u32, u32); // (tx_errors, rx_errors)
}

// Linux SocketCAN Implementation
#[cfg(target_os = "linux")]
pub struct SocketCanInterface {
    socket: Option<socketcan::CANSocket>,
    interface_name: String,
    tx_error_count: u32,
    rx_error_count: u32,
}

#[cfg(target_os = "linux")]
impl SocketCanInterface {
    pub fn new(interface: &str) -> Self {
        Self {
            socket: None,
            interface_name: interface.to_string(),
            tx_error_count: 0,
            rx_error_count: 0,
        }
    }
    
    pub fn connect(&mut self) -> Result<(), String> {
        match socketcan::CANSocket::open(&self.interface_name) {
            Ok(socket) => {
                self.socket = Some(socket);
                info!("Connected to CAN interface: {}", self.interface_name);
                Ok(())
            }
            Err(e) => Err(format!("Failed to connect to CAN interface: {}", e))
        }
    }
}

#[cfg(target_os = "linux")]
impl CanBusInterface for SocketCanInterface {
    fn send_frame(&mut self, id: u32, data: &[u8]) -> Result<(), String> {
        if let Some(ref socket) = self.socket {
            let frame = socketcan::CANFrame::new(id, data, false, false)
                .map_err(|e| {
                    self.tx_error_count += 1;
                    format!("Failed to create CAN frame: {}", e)
                })?;
            
            socket.write_frame(&frame)
                .map_err(|e| {
                    self.tx_error_count += 1;
                    format!("Failed to send CAN frame: {}", e)
                })?;
            
            Ok(())
        } else {
            self.tx_error_count += 1;
            Err("CAN socket not connected".to_string())
        }
    }
    
    fn receive_frame(&mut self) -> Option<(u32, Vec<u8>)> {
        if let Some(ref socket) = self.socket {
            match socket.read_frame() {
                Ok(frame) => Some((frame.id(), frame.data().to_vec())),
                Err(_) => {
                    self.rx_error_count += 1;
                    None
                }
            }
        } else {
            self.rx_error_count += 1;
            None
        }
    }
    
    fn set_bitrate(&mut self, _bitrate: u32) -> Result<(), String> {
        // Note: Bitrate is typically set at the system level for SocketCAN
        // This would require system commands like: ip link set can0 type can bitrate 250000
        Ok(())
    }
    
    fn set_filter(&mut self, filter_id: u32, mask: u32) -> Result<(), String> {
        if let Some(ref socket) = self.socket {
            let filter = socketcan::CANFilter::new(filter_id, mask);
            socket.set_filter(&[filter])
                .map_err(|e| format!("Failed to set CAN filter: {}", e))?;
            Ok(())
        } else {
            Err("CAN socket not connected".to_string())
        }
    }
    
    fn get_error_count(&self) -> (u32, u32) {
        (self.tx_error_count, self.rx_error_count)
    }
}

// Mock CAN Interface for Testing/Windows
pub struct MockCanInterface {
    sent_frames: Vec<(u32, Vec<u8>)>,
    receive_queue: std::collections::VecDeque<(u32, Vec<u8>)>,
    tx_error_count: u32,
    rx_error_count: u32,
}

impl MockCanInterface {
    pub fn new() -> Self {
        Self {
            sent_frames: Vec::new(),
            receive_queue: std::collections::VecDeque::new(),
            tx_error_count: 0,
            rx_error_count: 0,
        }
    }
    
    pub fn add_mock_response(&mut self, id: u32, data: Vec<u8>) {
        self.receive_queue.push_back((id, data));
    }
    
    pub fn simulate_tx_error(&mut self) {
        self.tx_error_count += 1;
    }
    
    pub fn simulate_rx_error(&mut self) {
        self.rx_error_count += 1;
    }
    
    pub fn reset_error_counters(&mut self) {
        self.tx_error_count = 0;
        self.rx_error_count = 0;
    }
}

impl CanBusInterface for MockCanInterface {
    fn send_frame(&mut self, id: u32, data: &[u8]) -> Result<(), String> {
        self.sent_frames.push((id, data.to_vec()));
        debug!("Mock CAN TX: ID=0x{:X}, Data={:?}", id, data);
        Ok(())
    }
    
    fn receive_frame(&mut self) -> Option<(u32, Vec<u8>)> {
        self.receive_queue.pop_front()
    }
    
    fn set_bitrate(&mut self, bitrate: u32) -> Result<(), String> {
        info!("Mock CAN bitrate set to: {}", bitrate);
        Ok(())
    }
    
    fn set_filter(&mut self, filter_id: u32, mask: u32) -> Result<(), String> {
        info!("Mock CAN filter set: ID=0x{:X}, Mask=0x{:X}", filter_id, mask);
        Ok(())
    }
    
    fn get_error_count(&self) -> (u32, u32) {
        (self.tx_error_count, self.rx_error_count)
    }
}

// Main Toolhead Manager
pub struct KlipperCanManager {
    can_interface: Box<dyn CanBusInterface>,
    toolheads: HashMap<u32, ToolheadState>,
    configurations: HashMap<u32, ToolheadConfig>,
    statistics: HashMap<u32, ToolheadStats>,
    message_queue: Arc<Mutex<Vec<KlipperCanMessage>>>,
    running: Arc<Mutex<bool>>,
    sequence_counter: u8,
    discovery_enabled: bool,
}

impl KlipperCanManager {
    pub fn new(can_interface: Box<dyn CanBusInterface>) -> Self {
        Self {
            can_interface,
            toolheads: HashMap::new(),
            configurations: HashMap::new(),
            statistics: HashMap::new(),
            message_queue: Arc::new(Mutex::new(Vec::new())),
            running: Arc::new(Mutex::new(false)),
            sequence_counter: 0,
            discovery_enabled: true,
        }
    }
    
    pub fn start(&mut self) -> Result<(), String> {
        info!("Starting Klipper CAN manager");
        
        // Set CAN bitrate (typically 250kbps for Klipper toolheads)
        self.can_interface.set_bitrate(250_000)?;
        
        // Set filter for Klipper CAN IDs
        self.can_interface.set_filter(0x100, 0xF00)?;
        
        *self.running.lock().unwrap() = true;
        
        Ok(())
    }
    
    pub fn stop(&mut self) {
        info!("Stopping Klipper CAN manager");
        *self.running.lock().unwrap() = false;
    }
    
    pub fn discover_toolheads(&mut self) -> Result<Vec<u32>, String> {
        info!("Starting toolhead discovery");
        
        let mut discovered_uuids = Vec::new();
        
        // Send broadcast discovery message
        let discovery_msg = KlipperCanMessage {
            uuid: 0xFFFFFF, // Broadcast UUID
            cmd: KlipperCanCmd::QueryUuid,
            seq: self.next_sequence(),
            data: [0; 7],
            len: 0,
            timestamp: Instant::now(),
        };
        
        self.send_message(discovery_msg)?;
        
        // Wait for responses
        std::thread::sleep(Duration::from_millis(500));
        
        // Process received messages
        while let Some((id, data)) = self.can_interface.receive_frame() {
            if let Ok(msg) = self.parse_can_frame(id, &data) {
                if msg.cmd == KlipperCanCmd::Response {
                    discovered_uuids.push(msg.uuid);
                    self.add_toolhead(msg.uuid);
                    info!("Discovered toolhead with UUID: 0x{:06X}", msg.uuid);
                }
            }
        }
        
        info!("Discovery complete. Found {} toolheads", discovered_uuids.len());
        Ok(discovered_uuids)
    }
    
    pub fn send_toolhead_command(&mut self, uuid: u32, cmd: KlipperCanCmd, data: &[u8]) -> Result<(), String> {
        let message = KlipperCanMessage {
            uuid,
            cmd,
            seq: self.next_sequence(),
            data: {
                let mut payload = [0u8; 7];
                let len = std::cmp::min(data.len(), 7);
                payload[..len].copy_from_slice(&data[..len]);
                payload
            },
            len: data.len() as u8,
            timestamp: Instant::now(),
        };
        
        self.send_message(message)
    }
    
    pub fn move_extruder(&mut self, uuid: u32, distance: f32, speed: f32) -> Result<(), String> {
        let mut data = Vec::new();
        data.extend_from_slice(&distance.to_le_bytes());
        data.extend_from_slice(&speed.to_le_bytes()[..3]); // Truncate to fit in 7 bytes
        
        self.send_toolhead_command(uuid, KlipperCanCmd::MoveExtruder, &data)
    }
    
    pub fn set_temperature(&mut self, uuid: u32, temperature: f32) -> Result<(), String> {
        let data = temperature.to_le_bytes();
        self.send_toolhead_command(uuid, KlipperCanCmd::SetTemperature, &data)
    }
    
    pub fn set_fan_speed(&mut self, uuid: u32, speed: u8) -> Result<(), String> {
        let data = [speed];
        self.send_toolhead_command(uuid, KlipperCanCmd::SetFan, &data)
    }
    
    pub fn home_toolhead(&mut self, uuid: u32) -> Result<(), String> {
        self.send_toolhead_command(uuid, KlipperCanCmd::Home, &[])
    }
    
    pub fn emergency_stop(&mut self, uuid: u32) -> Result<(), String> {
        warn!("Emergency stop for toolhead: 0x{:06X}", uuid);
        self.send_toolhead_command(uuid, KlipperCanCmd::EmergencyStop, &[])
    }
    
    pub fn get_toolhead_status(&mut self, uuid: u32) -> Result<(), String> {
        self.send_toolhead_command(uuid, KlipperCanCmd::GetStatus, &[])
    }
    
    pub fn get_toolheads(&self) -> &HashMap<u32, ToolheadState> {
        &self.toolheads
    }
    
    pub fn get_toolhead_config(&self, uuid: u32) -> Option<&ToolheadConfig> {
        self.configurations.get(&uuid)
    }
    
    pub fn get_toolhead_stats(&self, uuid: u32) -> Option<&ToolheadStats> {
        self.statistics.get(&uuid)
    }
    
    pub fn update_toolhead_config(&mut self, uuid: u32, config: ToolheadConfig) {
        self.configurations.insert(uuid, config);
    }
    
    // Process incoming CAN messages
    pub fn process_messages(&mut self) {
        while let Some((id, data)) = self.can_interface.receive_frame() {
            if let Ok(msg) = self.parse_can_frame(id, &data) {
                self.handle_received_message(msg);
            }
        }
    }
    
    fn send_message(&mut self, msg: KlipperCanMessage) -> Result<(), String> {
        let can_id = self.encode_can_id(msg.uuid, msg.cmd as u8, msg.seq);
        
        let mut frame_data = vec![msg.cmd as u8];
        frame_data.extend_from_slice(&msg.data[..msg.len as usize]);
        
        // Update statistics
        if let Some(stats) = self.statistics.get_mut(&msg.uuid) {
            stats.messages_sent += 1;
            stats.bytes_sent += frame_data.len() as u64;
        }
        
        debug!("Sending CAN message: UUID=0x{:06X}, CMD={:?}, SEQ={}", 
               msg.uuid, msg.cmd, msg.seq);
        
        self.can_interface.send_frame(can_id, &frame_data)
    }
    
    fn parse_can_frame(&self, can_id: u32, data: &[u8]) -> Result<KlipperCanMessage, String> {
        if data.is_empty() {
            return Err("Empty CAN frame".to_string());
        }
        
        let (uuid, seq) = self.decode_can_id(can_id);
        let cmd = match data[0] {
            x if x < 0x80 => KlipperCanCmd::try_from(x).map_err(|_| "Invalid command")?,
            0x80 => KlipperCanCmd::Response,
            0xFF => KlipperCanCmd::Error,
            _ => return Err("Unknown command type".to_string()),
        };
        
        let mut payload = [0u8; 7];
        let data_len = std::cmp::min(data.len() - 1, 7);
        if data_len > 0 {
            payload[..data_len].copy_from_slice(&data[1..=data_len]);
        }
        
        Ok(KlipperCanMessage {
            uuid,
            cmd,
            seq,
            data: payload,
            len: data_len as u8,
            timestamp: Instant::now(),
        })
    }
    
    fn handle_received_message(&mut self, msg: KlipperCanMessage) {
        debug!("Received CAN message: UUID=0x{:06X}, CMD={:?}", msg.uuid, msg.cmd);
        
        // Update statistics
        if let Some(stats) = self.statistics.get_mut(&msg.uuid) {
            stats.messages_received += 1;
            stats.bytes_received += (msg.len + 1) as u64;
        }
        
        match msg.cmd {
            KlipperCanCmd::Response => {
                self.handle_toolhead_response(msg);
            }
            KlipperCanCmd::Error => {
                self.handle_toolhead_error(msg);
            }
            _ => {
                // Handle other message types
            }
        }
    }
    
    fn handle_toolhead_response(&mut self, msg: KlipperCanMessage) {
        if let Some(toolhead) = self.toolheads.get_mut(&msg.uuid) {
            toolhead.last_command_time = Instant::now();
            
            // Parse response data based on original command
            match msg.len {
                4 => {
                    // Temperature response
                    let temp = f32::from_le_bytes([msg.data[0], msg.data[1], msg.data[2], msg.data[3]]);
                    toolhead.temperature = temp;
                }
                16 => {
                    // Position response
                    for i in 0..4 {
                        let start_idx = i * 4;
                        if start_idx + 3 < msg.len as usize {
                            let pos = f32::from_le_bytes([
                                msg.data[start_idx], msg.data[start_idx + 1],
                                msg.data[start_idx + 2], msg.data[start_idx + 3]
                            ]);
                            toolhead.position[i] = pos;
                        }
                    }
                }
                _ => {
                    // Other response types
                }
            }
        }
    }
    
    fn handle_toolhead_error(&mut self, msg: KlipperCanMessage) {
        warn!("Toolhead error from UUID 0x{:06X}: {:?}", msg.uuid, msg.data);
        
        if let Some(toolhead) = self.toolheads.get_mut(&msg.uuid) {
            toolhead.error_count += 1;
        }
        
        if let Some(stats) = self.statistics.get_mut(&msg.uuid) {
            stats.error_count += 1;
            stats.last_error = Some(format!("Error code: 0x{:02X}", msg.data[0]));
        }
    }
    
    fn add_toolhead(&mut self, uuid: u32) {
        let now = Instant::now();
        
        let state = ToolheadState {
            uuid,
            connected: true,
            temperature: 0.0,
            target_temperature: 0.0,
            fan_speed: 0,
            position: [0.0; 4],
            last_command_time: now,
            error_count: 0,
            firmware_version: "Unknown".to_string(),
        };
        
        let stats = ToolheadStats {
            messages_sent: 0,
            messages_received: 0,
            bytes_sent: 0,
            bytes_received: 0,
            average_latency_ms: 0.0,
            error_count: 0,
            last_error: None,
            uptime: Duration::from_secs(0),
            connection_time: now,
        };
        
        let config = ToolheadConfig {
            uuid,
            name: format!("Toolhead_{:06X}", uuid),
            can_bitrate: 250000,
            max_temp: 300.0,
            min_extrude_temp: 170.0,
            max_extrude_speed: 50.0,
            pressure_advance: 0.0,
            extruder_steps_per_mm: 400.0,
            capabilities: ToolheadCapabilities {
                has_extruder: true,
                has_hotend: true,
                has_fan: true,
                has_probe: false,
                has_accelerometer: false,
                max_accel: 3000.0,
                max_velocity: 300.0,
            },
        };
        
        self.toolheads.insert(uuid, state);
        self.statistics.insert(uuid, stats);
        self.configurations.insert(uuid, config);
        
        info!("Added toolhead: UUID=0x{:06X}", uuid);
    }
    
    fn encode_can_id(&self, uuid: u32, cmd: u8, seq: u8) -> u32 {
        // Klipper CAN ID format: [Priority:3][UUID:24][CMD:3][SEQ:2]
        (1u32 << 29) | ((uuid & 0xFFFFFF) << 5) | ((cmd as u32 & 0x07) << 2) | (seq as u32 & 0x03)
    }
    
    fn decode_can_id(&self, can_id: u32) -> (u32, u8) {
        let uuid = (can_id >> 5) & 0xFFFFFF;
        let seq = (can_id & 0x03) as u8;
        (uuid, seq)
    }
    
    fn next_sequence(&mut self) -> u8 {
        self.sequence_counter = (self.sequence_counter + 1) % 4;
        self.sequence_counter
    }
}

impl TryFrom<u8> for KlipperCanCmd {
    type Error = ();
    
    fn try_from(value: u8) -> Result<Self, <KlipperCanCmd as TryFrom<u8>>::Error> {
        match value {
            0x01 => Ok(KlipperCanCmd::MoveQueue),
            0x02 => Ok(KlipperCanCmd::Home),
            0x03 => Ok(KlipperCanCmd::Move),
            0x04 => Ok(KlipperCanCmd::SetPosition),
            0x10 => Ok(KlipperCanCmd::SetTemperature),
            0x11 => Ok(KlipperCanCmd::GetTemperature),
            0x12 => Ok(KlipperCanCmd::SetFan),
            0x13 => Ok(KlipperCanCmd::GetSensors),
            0x20 => Ok(KlipperCanCmd::MoveExtruder),
            0x21 => Ok(KlipperCanCmd::RetractExtruder),
            0x22 => Ok(KlipperCanCmd::SetPressureAdvance),
            0x30 => Ok(KlipperCanCmd::GetStatus),
            0x31 => Ok(KlipperCanCmd::EmergencyStop),
            0x32 => Ok(KlipperCanCmd::Reset),
            0x33 => Ok(KlipperCanCmd::GetConfig),
            0xF0 => Ok(KlipperCanCmd::QueryUuid),
            0xF1 => Ok(KlipperCanCmd::SetUuid),
            0xF2 => Ok(KlipperCanCmd::GetVersion),
            0x80 => Ok(KlipperCanCmd::Response),
            0xFF => Ok(KlipperCanCmd::Error),
            _ => Err(()),
        }
    }
}

// Async task for continuous message processing
pub async fn can_message_processor(
    manager: Arc<Mutex<KlipperCanManager>>,
    mut shutdown_rx: tokio::sync::oneshot::Receiver<()>
) {
    let mut interval = tokio::time::interval(Duration::from_millis(10)); // 100Hz processing
    
    loop {
        tokio::select! {
            _ = interval.tick() => {
                if let Ok(mut mgr) = manager.try_lock() {
                    mgr.process_messages();
                }
            }
            _ = &mut shutdown_rx => {
                info!("CAN message processor shutting down");
                break;
            }
        }
    }
}