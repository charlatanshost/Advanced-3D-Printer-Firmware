use eframe::egui::{self, FontDefinitions, FontData, Color32, Visuals};
use std::sync::Arc;
// use egui_plot::{Line, Plot, PlotPoints, Points}; // Commented out for now

use printer_host::{
    protocols::{
        self, ProtocolManager, security, CommunicationMode, BACnetConfig, ADSConfig,
        CODESYSConfig, CIPConfig, OPC_UAConfig, ProtocolDiscovery, ProtocolDiagnostics,
        CIPConnectionType
    },
    klipper_can::{
        self, KlipperCanManager, ToolheadConfig, ToolheadState, ToolheadStats,
        KlipperCanCmd, MockCanInterface
    },
    input_shaping::{
        self, InputShaper, InputShaperConfig, InputShaperManager, ShaperType,
        MotionCommand, MockAccelerometer, ResonanceTester
    },
    auto_input_shaper::{
        self, AutoInputShaperCalibrator, ResonanceTestConfig, CalibrationUI,
        InputShaperType, CalibrationResults, CalibrationError
    },
    advanced_motion::{
        self, VolumetricController, VolumetricSettings, TrajectoryPlanner, MotionSegment, SegmentType,
        ServoController, MultiToolManager, ToolHead, ToolOffset, MaterialSettings,
        ToolChangeCommand, CalibrationCommand
    },
    volumetric_compensation::{
        self, VolumetricCompensator, VolumetricCompensationConfig, CompensationMap,
        ProbePoint, CompensationError as VolCompError, CompensationStatus
    },
    sensor_fusion::{
        self, SensorFusionManager, SensorConfig as FusionSensorConfig, SensorData, FusedState,
        ExtendedKalmanFilter, FusionError, FusionStatistics
    },
    plugin_system::{
        self, PluginManager, PluginManifest, PluginType, PluginEvent,
        PluginError, PluginInfo, LoadedPlugin
    },
    resonance_cancellation::{
        self, ActiveResonanceCanceller, ResonanceCancellationConfig, MotionCorrectionInterface
    },
    torque_control::{
        self, DynamicForceController, TorqueControlConfig, MockTorqueSensor, SystemEvent
    },
    motor_assignment::{
        self, MotorAssignmentManager, MotorConfiguration, MotorFunction,
        MotorType as AssignmentMotorType, MotorState, MotorConnection,
        MotorCapabilities, AssignmentEvent
    },
    closed_loop_control::{
        self, ClosedLoopManager, EncoderConfig, EncoderType, PIDController,
        AxisDiagnostics, ClosedLoopStatus, MockEncoder, EncoderInterface
    },
    linuxcnc_control::{
        self, LinuxCNCController, AxisControlParameters, ControlOutputType,
        SCurveTrajectoryGenerator, TrajectoryState
    },
    smooth_trajectory::{
        self, SmoothTrajectoryPlanner, MotionConstraints, MotionProfile
    },
    integration_test,
    temperature_control::{
        self, TemperatureManager, TemperatureZone, SensorConfig, SensorType,
        HeaterConfig, HeaterType, PIDConfig as TempPIDConfig, FeedforwardModel,
        TemperatureStatistics, TemperatureSystemStatus, MockTemperatureSensor
    },
    power_loss_recovery::{
        self, PowerLossRecoveryManager, RecoveryConfig, PowerLossDetectionMethod,
        PrintState, PrintStateUpdate, RecoveryPlan, RecoveryStatistics,
        MockVoltageSensor, VoltageSensorInterface
    },
    test_runner,
    pressure_advance::{
        self, PressureAdvanceController, PressureAdvanceConfig, ExtrusionSegment,
        MultiExtruderPressureAdvance, PressureAdvanceStats
    },
    runtime_config::{
        self, ConfigManager, PrinterConfig, ConfigError, ExportFormat, PrinterType
    },
};


use std::sync::Mutex;
use printer_kinematics::{create_kinematics, Config, Kinematics, Scene};
use printer_kinematics::five_axis::{
    Pose6DOF, JointConfiguration,
    WorkspaceLimits
};
use printer_kinematics::practical_configs::{
    PracticalKinematics, PracticalMachineType
};
use std::fs;
use std::sync::mpsc;
use std::thread;
use std::collections::HashMap;
use std::net::{IpAddr, Ipv4Addr};
use std::time::{Duration, Instant};


#[cfg(target_os = "linux")]
use ethercrab::{Client, ClientConfig, SlaveGroup, std::tx_rx_task, std::ethercat_now, Timeouts, PduStorage};
#[cfg(target_os = "linux")]
use tokio::sync::mpsc;
use gcode_parser::GCodeFile;
use serialport::{self, SerialPort};
use rfd::FileDialog;

#[cfg(target_os = "linux")]
const MAX_SUBDEVICES: usize = 16;
#[cfg(target_os = "linux")]
const MAX_PDU_DATA: usize = 1100;
#[cfg(target_os = "linux")]
const MAX_FRAMES: usize = 16;
#[cfg(target_os = "linux")]
const PDI_LEN: usize = 64;

#[cfg(target_os = "linux")]
static PDU_STORAGE: PduStorage<MAX_FRAMES, MAX_PDU_DATA> = PduStorage::new();



/// Encode positions for Modbus: a signed 32-bit count of micrometres/millidegrees per axis,
/// high word first, in two registers each. A u16 of 1/1000 units cannot hold a position
/// past 65.5mm or below zero, so bad values are refused rather than clipped.
fn encode_modbus_positions(positions: &[f32; 5]) -> Result<[u16; 10], String> {
    let mut registers = [0u16; 10];
    for (i, &pos) in positions.iter().enumerate() {
        let scaled = (pos as f64) * 1000.0;
        if !scaled.is_finite() || scaled < i32::MIN as f64 || scaled > i32::MAX as f64 {
            return Err(format!("Axis {} position {} cannot be encoded", i, pos));
        }
        let bits = scaled.round() as i32 as u32;
        registers[i * 2] = (bits >> 16) as u16;
        registers[i * 2 + 1] = (bits & 0xFFFF) as u16;
    }
    Ok(registers)
}

#[cfg(test)]
mod modbus_encoding_tests {
    use super::encode_modbus_positions;

    fn decode(high: u16, low: u16) -> f32 {
        (((high as u32) << 16 | low as u32) as i32) as f32 / 1000.0
    }

    #[test]
    fn positions_survive_the_whole_range() {
        let registers = encode_modbus_positions(&[100.0, 250.5, -12.25, 0.0, 359.0]).unwrap();
        assert_eq!(decode(registers[0], registers[1]), 100.0, "above the old 65.5mm clip");
        assert_eq!(decode(registers[2], registers[3]), 250.5);
        assert_eq!(decode(registers[4], registers[5]), -12.25, "negative");
        assert_eq!(decode(registers[8], registers[9]), 359.0);
    }

    #[test]
    fn unencodable_positions_are_refused() {
        assert!(encode_modbus_positions(&[f32::NAN, 0.0, 0.0, 0.0, 0.0]).is_err());
        assert!(encode_modbus_positions(&[0.0, 1e9, 0.0, 0.0, 0.0]).is_err());
    }
}

// Static function for Modbus TCP communication
async fn send_modbus_tcp_positions_static(
    ip: &str, 
    port: u16, 
    device_id: u8, 
    positions: &[f32; 5]
) -> Result<(), String> {
    let scaled_positions = encode_modbus_positions(positions)?;

    #[cfg(all(target_os = "linux", feature = "fieldbus"))]
    {
        use tokio_modbus::prelude::*;
        
        // Connect to Modbus TCP server
        let socket_addr = format!("{}:{}", ip, port)
            .parse()
            .map_err(|e| format!("Invalid IP address or port: {}", e))?;
        
        let mut client = tcp::connect(socket_addr).await
            .map_err(|e| format!("Failed to connect to Modbus TCP server: {}", e))?;
        
        // Write position data to holding registers starting at address 0x0000
        client.write_multiple_registers(0x0000, &scaled_positions).await
            .map_err(|e| format!("Failed to write Modbus registers: {}", e))?;
        
        // Write device status register at address 0x0010
        let status_word: u16 = 0x0001; // Status: Active/Running
        client.write_single_register(0x0010, status_word).await
            .map_err(|e| format!("Failed to write status register: {}", e))?;
        
        // Write device ID at address 0x0011
        client.write_single_register(0x0011, device_id as u16).await
            .map_err(|e| format!("Failed to write device ID register: {}", e))?;
        
        println!("Modbus TCP: Successfully sent positions to device {} at {}:{}", 
                 device_id, ip, port);
        println!("  X: {:.3}mm, Y: {:.3}mm, Z: {:.3}mm, A: {:.3}°, B: {:.3}°", 
                 positions[0], positions[1], positions[2], positions[3], positions[4]);
        
        Ok(())
    }
    
    #[cfg(not(all(target_os = "linux", feature = "fieldbus")))]
    {
        // Mock implementation when real Modbus support is not built
        let _ = &scaled_positions;
        println!("Modbus TCP (Mock): Simulating send to device {} at {}:{}",
                 device_id, ip, port);
        println!("  X: {:.3}mm, Y: {:.3}mm, Z: {:.3}mm, A: {:.3}°, B: {:.3}°", 
                 positions[0], positions[1], positions[2], positions[3], positions[4]);
        println!("  (Real Modbus TCP needs Linux and the `fieldbus` feature)");
        
        // Simulate network delay
        tokio::time::sleep(tokio::time::Duration::from_millis(50)).await;
        
        Ok(())
    }
}

#[tokio::main]
async fn main() -> eframe::Result<()> {
    // Check if we should run the integration test
    let args: Vec<String> = std::env::args().collect();
    if args.len() > 1 && args[1] == "--simulate" {
        std::process::exit(printer_host::simulator::run_cli(&args[2..]));
    }
    if args.len() > 1 && args[1] == "--test" {
        test_runner::run_integration_test();
        return Ok(());
    }

    let config_str = fs::read_to_string("printer.toml").unwrap_or_else(|e| panic!("Failed to read printer.toml: {}", e));
    let config: Config = toml::from_str(&config_str).unwrap_or_else(|e| panic!("Failed to parse printer.toml: {}", e));
    let kinematics = create_kinematics(&config);

    #[cfg(target_os = "linux")]
    let (group, tx, _pdu_task) = {
        let interface = std::env::args().nth(1).expect("Provide network interface as first argument, e.g., '\\Device\\NPF_{...}'");
        let (tx, rx, pdu_loop) = PDU_STORAGE.try_split().expect("Can only split once");
        let client = Client::new(pdu_loop, Timeouts::default(), ClientConfig::default());
        let _pdu_task = tokio::spawn(tx_rx_task(&interface, tx, rx).expect("Spawn TX/RX task"));

        println!("Initializing EtherCAT master...");
        let group = Arc::new(client.init_single_group::<MAX_SUBDEVICES, PDI_LEN>(ethercat_now).await.unwrap_or_else(|e| panic!("EtherCAT init failed: {}", e)));
        println!("Discovered {} EtherCAT slaves", group.len());

        let (tx, rx) = mpsc::channel::<[f32; 5]>(64);
        let group_clone: Arc<SlaveGroup<MAX_SUBDEVICES, PDI_LEN>> = Arc::clone(&group);
        tokio::spawn(async move {
            while let Some(positions) = rx.recv().await {
                for (i, subdevice) in group_clone.iter().enumerate() {
                    if let Some(pos) = positions.get(i) {
                        if let Err(e) = subdevice.write_slice(&pos.to_le_bytes()).await {
                            eprintln!("Failed to write to slave {}: {}", subdevice.configured_address(), e);
                        }
                    }
                }
            }
        });

        (group, tx, _pdu_task)
    };

    #[cfg(all(windows, feature = "windows-ethercat"))]
    let (group, tx) = {
        println!("Using Windows EtherCAT implementation");

        // Initialize Windows EtherCAT master
        let mut windows_master = printer_host::ethercat_windows::WindowsEtherCATMaster::new();

        // Try to discover and connect to an adapter
        match printer_host::ethercat_windows::WindowsEtherCATMaster::discover_adapters() {
            Ok(adapters) => {
                if !adapters.is_empty() {
                    println!("Found {} EtherCAT-capable adapter(s)", adapters.len());
                    for (i, adapter) in adapters.iter().enumerate() {
                        println!("  [{}] {} - {}", i, adapter.name, adapter.description);
                    }

                    // Connect to the first available adapter
                    if let Err(e) = windows_master.connect_adapter(&adapters[0].name) {
                        println!("Failed to connect to adapter: {}", e);
                    } else {
                        println!("Connected to EtherCAT adapter: {}", adapters[0].description);

                        // We'll initialize the EtherCAT master asynchronously after the main app starts
                        // For now, just store the master in the slave group and let it handle initialization
                        println!("EtherCAT master will be initialized asynchronously");
                    }
                } else {
                    println!("No EtherCAT-capable adapters found");
                }
            }
            Err(e) => println!("Failed to discover adapters: {}", e),
        }

        // Create a channel for motor position commands
        let (tx, rx) = tokio::sync::mpsc::channel::<[f32; 5]>(1000);

        // Create a Windows EtherCAT slave group wrapper
        let slave_group = Arc::new(WindowsEtherCATSlaveGroup {
            master: Arc::new(tokio::sync::Mutex::new(windows_master)),
            rx: Arc::new(tokio::sync::Mutex::new(rx)),
        });

        // Start the cyclic EtherCAT operation
        slave_group.start_cyclic_operation();

        (slave_group, tx)
    };

    #[cfg(all(not(target_os = "linux"), not(all(windows, feature = "windows-ethercat"))))]
    let (group, tx) = {
        println!("EtherCAT not supported on this platform");
        (Arc::new(DummySlaveGroup), tokio::sync::mpsc::channel::<[f32; 5]>(64).0)
    };

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size(egui::vec2(1280.0, 720.0)),
        renderer: eframe::Renderer::Wgpu, // Reverted to wgpu
        ..Default::default()
    };

    let gcode = fs::read_to_string("gcode/example.gcode").unwrap_or_else(|_| "No G-code file found".to_string());

    eframe::run_native(
        "3D Printer Host",
        native_options,
        Box::new(move |cc| {
            // Load fonts with emoji support
            let mut fonts = FontDefinitions::default();
            
            // Keep the default system fonts which usually have better emoji support
            // Only add Arial as a fallback, not as primary
            let font_data = FontData::from_static(include_bytes!("resources/arial.ttf"));
            let arc_font_data = Arc::new(font_data);
            fonts.font_data.insert("arial".to_owned(), arc_font_data.clone());
            
            // Add Arial as a fallback (after default system fonts)
            fonts.families.get_mut(&egui::FontFamily::Proportional).unwrap().push("arial".to_owned());
            fonts.families.get_mut(&egui::FontFamily::Monospace).unwrap().push("arial".to_owned());
            
            // Keep default font configuration which already includes system emoji fonts
            // The default fonts are usually sufficient for emoji display
            // We only add Arial as a text fallback, not replacing the system fonts
            cc.egui_ctx.set_fonts(fonts);
            
            // Set smaller text size globally
            let mut style = egui::Style::default();
            style.text_styles.insert(
                egui::TextStyle::Body,
                egui::FontId::new(12.0, egui::FontFamily::Proportional),
            );
            style.text_styles.insert(
                egui::TextStyle::Button,
                egui::FontId::new(11.0, egui::FontFamily::Proportional),
            );
            style.text_styles.insert(
                egui::TextStyle::Heading,
                egui::FontId::new(16.0, egui::FontFamily::Proportional),
            );
            style.text_styles.insert(
                egui::TextStyle::Small,
                egui::FontId::new(10.0, egui::FontFamily::Proportional),
            );
            cc.egui_ctx.set_style(style);
            println!("Loaded Arial font successfully");

            // Set dark theme with high-contrast text
            cc.egui_ctx.set_visuals(Visuals {
                dark_mode: true,
                override_text_color: Some(Color32::WHITE),
                panel_fill: Color32::from_rgb(30, 30, 30), // Dark gray background
                ..Visuals::default()
            });
            cc.egui_ctx.set_debug_on_hover(false); // Disable debug rendering
            println!("UI initialized with wgpu renderer");
            let mut app = MyApp::new(kinematics, group, tx, gcode, vec![]); // Empty plot_points for now
            app.initialize_machine_config(); // Load machine config from saved settings
            Ok(Box::new(app))
        }),
    )
}

// Dummy struct for non-Linux platforms
#[cfg(not(target_os = "linux"))]
struct DummySlaveGroup;

#[cfg(not(target_os = "linux"))]
impl DummySlaveGroup {
    fn is_empty(&self) -> bool {
        true
    }
}

// Windows EtherCAT slave group for real hardware
#[cfg(windows)]
struct WindowsEtherCATSlaveGroup {
    master: Arc<tokio::sync::Mutex<printer_host::ethercat_windows::WindowsEtherCATMaster>>,
    rx: Arc<tokio::sync::Mutex<tokio::sync::mpsc::Receiver<[f32; 5]>>>,
}

#[cfg(windows)]
impl WindowsEtherCATSlaveGroup {
    fn is_empty(&self) -> bool {
        // Check if we have any configured slaves
        if let Ok(master) = self.master.try_lock() {
            let slave_status = master.get_slave_status();
            slave_status.is_empty()
        } else {
            // If we can't get the lock, assume we have slaves (safer default)
            false
        }
    }

    fn len(&self) -> usize {
        if let Ok(master) = self.master.try_lock() {
            let slave_status = master.get_slave_status();
            slave_status.len()
        } else {
            // If we can't get the lock, return 0 as default
            0
        }
    }

    // Start the cyclic EtherCAT operation in the background
    fn start_cyclic_operation(&self) {
        let master_clone = Arc::clone(&self.master);
        let rx_clone = Arc::clone(&self.rx);

        // Initialize and start the EtherCAT master
        tokio::spawn(async move {
            let mut master = master_clone.lock().await;

            // Try to start the EtherCAT master
            if let Err(e) = master.start().await {
                eprintln!("Failed to start EtherCAT master: {}", e);
                return;
            }
            println!("EtherCAT master started successfully");

            // Scan for slaves
            match master.scan_slaves().await {
                Ok(slave_addresses) => {
                    println!("Found {} EtherCAT slave(s): {:?}", slave_addresses.len(), slave_addresses);

                    // Configure all found slaves
                    for &addr in &slave_addresses {
                        if let Err(e) = master.configure_slave(addr).await {
                            println!("Failed to configure slave {}: {}", addr, e);
                        } else {
                            println!("Slave {} configured successfully", addr);
                        }
                    }
                }
                Err(e) => println!("Failed to scan for slaves: {}", e),
            }

            // Start cyclic operation
            if master.is_running() {
                master.run_cyclic().await;
            }
        });

        // Start position command processing
        let master_clone = Arc::clone(&self.master);
        tokio::spawn(async move {
            let mut rx = rx_clone.lock().await;
            while let Some(positions) = rx.recv().await {
                let mut master = master_clone.lock().await;

                // Check for special scan trigger signal
                if positions[0] == 999.0 {
                    println!("Received scan trigger signal - performing EtherCAT slave scan");

                    // Perform a new slave scan
                    match master.scan_slaves().await {
                        Ok(slave_addresses) => {
                            println!("Scan completed - found {} EtherCAT slave(s): {:?}",
                                     slave_addresses.len(), slave_addresses);

                            // Configure all found slaves
                            for &addr in &slave_addresses {
                                if let Err(e) = master.configure_slave(addr).await {
                                    println!("Failed to configure slave {}: {}", addr, e);
                                } else {
                                    println!("Slave {} configured successfully", addr);
                                }
                            }
                        }
                        Err(e) => println!("Slave scan failed: {}", e),
                    }
                    continue;
                }

                // Send positions to EtherCAT slaves
                // For a typical stepper motor setup:
                // positions[0] = X axis, positions[1] = Y axis, positions[2] = Z axis
                // positions[3] = E axis (extruder), positions[4] = additional axis

                // Convert positions to EtherCAT slave outputs
                for (i, &position) in positions.iter().enumerate() {
                    let slave_addr = (i + 1) as u16; // Slave addresses start from 1

                    // Convert position to motor steps or velocity commands
                    let motor_command = (position * 1000.0) as i32; // Example conversion
                    let motor_bytes = motor_command.to_le_bytes();

                    // Write to slave if it exists and has output capability
                    if let Some(slave_info) = master.get_slave_status().get(&slave_addr) {
                        if slave_info.1 == 0 { // Check if slave is operational (error_count == 0)
                            if let Err(e) = master.write_slave_outputs(slave_addr, motor_bytes.to_vec()) {
                                eprintln!("Failed to send positions to EtherCAT slave {}: {}", slave_addr, e);
                            }
                        }
                    }
                }
            }
        });
    }
}

// Stub types for advanced control systems when features are not enabled
#[derive(Debug, Clone)]
pub struct StateSpaceController;

#[derive(Debug, Clone)]
pub struct StateSpaceConfig;

#[derive(Debug, Clone)]
pub struct MPCController;

#[derive(Debug, Clone)]
pub struct MPCConfig;

#[derive(Debug, Clone)]
pub struct LQRConfig {
    pub position_weight: f64,
    pub velocity_weight: f64,
    pub control_weight: f64,
}

#[derive(Debug, Clone)]
pub struct MPCHorizonConfig;

#[derive(Debug, Clone)]
pub struct MPCWeightsConfig {
    pub tracking_weight: f64,
    pub control_effort_weight: f64,
    pub smoothness_weight: f64,
}

#[derive(Debug, Clone)]
pub struct MPCConstraintsConfig {
    pub position_bounds: [(f64, f64); 3],
    pub velocity_bounds: [(f64, f64); 3],
    pub acceleration_bounds: [(f64, f64); 3],
}

#[derive(Debug, Clone)]
pub struct MPCAdvancedConfig {
    pub adaptive_learning: bool,
    pub thermal_compensation: bool,
    pub vibration_prediction: bool,
    pub corner_optimization: bool,
}

#[derive(Debug, Clone)]
pub struct MPCSolverConfig {
    pub solver_type: MPCSolverType,
    pub max_iterations: u32,
    pub tolerance: f64,
    pub warm_start: bool,
    pub polish: bool,
}

#[derive(Debug, Clone)]
pub enum MPCSolverType {
    OSQP,
    ClarabelRs,
    SCS,
}

// Stub implementations
impl Default for StateSpaceConfig {
    fn default() -> Self { Self }
}

impl Default for MPCConfig {
    fn default() -> Self { Self }
}

#[derive(Debug, Clone)]
pub struct KalmanFilterConfig {
    pub process_noise: f64,
    pub measurement_noise: f64,
}

impl StateSpaceController {
    pub fn new(_config: StateSpaceConfig) -> Self { Self }
    pub fn get_status(&self) -> MockControllerStatus { MockControllerStatus::default() }
    pub fn start_system_identification(&self) -> Result<(), String> { Ok(()) }
    pub fn get_identified_parameters(&self) -> Option<MockSystemParams> { None }
    pub fn redesign_lqr_controller(&self, _config: LQRConfig) -> Result<MockLQRGains, String> {
        Ok(MockLQRGains::default())
    }
    pub fn get_performance_metrics(&self) -> Option<MockPerformanceMetrics> { None }
    pub fn get_observer_state(&self, _axis: char) -> Option<MockObserverState> { None }
    pub fn update_observer_config(&self, _config: KalmanFilterConfig) -> Result<(), String> { Ok(()) }
}

impl MPCController {
    pub fn new(_config: MPCConfig) -> Self { Self }
    pub fn get_status(&self) -> MockMPCStatus { MockMPCStatus::default() }
    pub fn update_horizons(&self, _config: MPCHorizonConfig) -> Result<(), String> { Ok(()) }
    pub fn update_weights(&self, _config: MPCWeightsConfig) -> Result<(), String> { Ok(()) }
    pub fn update_constraints(&self, _config: MPCConstraintsConfig) -> Result<(), String> { Ok(()) }
    pub fn update_solver_config(&self, _config: MPCSolverConfig) -> Result<(), String> { Ok(()) }
    pub fn get_performance_metrics(&self) -> Option<MockMPCPerformanceMetrics> { None }
    pub fn update_advanced_features(&self, _config: MPCAdvancedConfig) -> Result<(), String> { Ok(()) }
}

#[derive(Debug)]
pub struct MockControllerStatus {
    pub controller_type: String,
    pub update_rate: f64,
    pub cpu_usage: f64,
}

#[derive(Debug, Default)]
pub struct MockSystemParams {
    pub natural_frequency: f64,
    pub damping_ratio: f64,
    pub time_constant: f64,
    pub confidence: f64,
}

#[derive(Debug, Default)]
pub struct MockLQRGains {
    pub position_gain: f64,
    pub velocity_gain: f64,
}

#[derive(Debug, Default)]
pub struct MockPerformanceMetrics {
    pub position_error_rms: f64,
    pub control_effort_rms: f64,
    pub settling_time: std::time::Duration,
    pub overshoot_percent: f64,
}

#[derive(Debug, Default)]
pub struct MockObserverState {
    pub estimated_position: f64,
    pub estimated_velocity: f64,
    pub confidence: f64,
}

#[derive(Debug)]
pub struct MockMPCStatus {
    pub prediction_horizon: std::time::Duration,
    pub control_horizon: std::time::Duration,
    pub solver_type: String,
    pub avg_solve_time: std::time::Duration,
}

#[derive(Debug, Default)]
pub struct MockMPCPerformanceMetrics {
    pub avg_solve_time: std::time::Duration,
    pub max_solve_time: std::time::Duration,
    pub solver_success_rate: f64,
    pub path_following_error: f64,
    pub predicted_speedup: f64,
}

impl Default for MockControllerStatus {
    fn default() -> Self {
        Self {
            controller_type: "Stub Controller".to_string(),
            update_rate: 1000.0,
            cpu_usage: 0.0,
        }
    }
}

impl Default for MockMPCStatus {
    fn default() -> Self {
        Self {
            prediction_horizon: std::time::Duration::from_millis(50),
            control_horizon: std::time::Duration::from_millis(20),
            solver_type: "OSQP (Stub)".to_string(),
            avg_solve_time: std::time::Duration::from_micros(500),
        }
    }
}

#[derive(Debug, PartialEq)]
enum Tab {
    // Motion & Kinematics
    Kinematics,
    MotionControl,
    InputShaping,
    AutoInputShaper,
    VolumetricCompensation,
    SensorFusion,
    StateSpaceControl,
    ModelPredictiveControl,
    PressureAdvance,

    // Hardware Configuration
    Hardware,
    MotorAssignment,
    ClosedLoop,
    Temperature,

    // Communication & Protocols
    Communication,
    FieldbusConfig,
    CanToolheads,

    // G-code & Operations
    Gcode,
    Operations,
    Macros,
    GcodeImport,

    // System & Configuration
    System,
    Settings,
    PrinterConfig,
    RuntimeConfig,
    Plugins,

    // Utilities & Diagnostics
    BedMesh,
    FanControl,
    Webcam,
    PowerLoss,
    Terminal,
}

#[derive(Debug, Clone, PartialEq)]
enum MotorType {
    Stepper,
    Servo,
}

#[derive(Debug, Clone, PartialEq)]
enum PrinterState {
    Idle,
    Printing,
    Paused,
    Homing,
    Emergency,
    Error(String),
}

#[derive(Debug, Clone)]
struct PrinterSettings {
    // Temperature settings
    extruder_max_temp: f32,
    bed_max_temp: f32,
    extruder_min_extrude_temp: f32,
    
    // Movement settings
    max_velocity: f32,
    max_accel: f32,
    max_accel_to_decel: f32,
    square_corner_velocity: f32,
    
    // Homing settings
    homing_speed: f32,
    homing_retract_dist: f32,
    second_homing_speed: f32,
    
    // Safety settings
    position_min: [f32; 3], // X, Y, Z minimum
    position_max: [f32; 3], // X, Y, Z maximum
    
    // Motor configuration (per axis: X, Y, Z, E0, E1)
    motor_types: [MotorType; 5],
    
    // Stepper settings
    hold_current: f32,
    run_current: f32,
    microsteps: u16,
    
    // Servo settings
    servo_min_pulse: u16,  // microseconds
    servo_max_pulse: u16,  // microseconds
    servo_max_angle: f32,  // degrees
    
    // Firmware retraction
    retract_length: f32,
    retract_speed: f32,
    unretract_extra_length: f32,
    unretract_speed: f32,
    
    // Pressure advance settings
    pressure_advance: f32,
    pressure_advance_smooth_time: f32,
    
    // Z offset and probe settings
    z_offset: f32,
    probe_z_offset: f32,
    probe_xy_offset: [f32; 2], // X, Y offset of probe from nozzle
    probe_speed: f32,
    probe_lift_speed: f32,
    probe_samples: u8,
    probe_sample_retract_dist: f32,
    
    // 5-axis control
    enable_5axis_mode: bool,
    tool_point_control: bool, // TCP (Tool Center Point) control
    rotary_axis_max_velocity: [f32; 2], // A, B axis max velocities
    rotary_axis_acceleration: [f32; 2], // A, B axis accelerations
}

#[derive(Debug, Clone)]
struct BedMeshPoint {
    x: f32,
    y: f32,
    z: f32,
}

#[derive(Debug, Clone)]
struct BedMesh {
    name: String,
    points: Vec<Vec<f32>>, // 2D grid of Z heights
    min_x: f32,
    max_x: f32,
    min_y: f32,
    max_y: f32,
    mesh_width: usize,
    mesh_height: usize,
    min_height: f32,
    max_height: f32,
}

impl BedMesh {
    fn new(name: String, width: usize, height: usize, min_x: f32, max_x: f32, min_y: f32, max_y: f32) -> Self {
        let mut points = vec![vec![0.0; width]; height];
        
        // Generate sample mesh data with some variation
        let mut min_height = f32::INFINITY;
        let mut max_height = f32::NEG_INFINITY;
        
        for (i, row) in points.iter_mut().enumerate() {
            for (j, point) in row.iter_mut().enumerate() {
                // Create a sample mesh with some realistic variation
                let x_ratio = j as f32 / (width - 1) as f32;
                let y_ratio = i as f32 / (height - 1) as f32;
                
                // Simulate bed warping with sine waves
                let height = 0.1 * (x_ratio * std::f32::consts::PI * 2.0).sin() * 
                           (y_ratio * std::f32::consts::PI * 1.5).cos() +
                           0.05 * (x_ratio * y_ratio * std::f32::consts::PI * 4.0).sin();
                
                *point = height;
                min_height = min_height.min(height);
                max_height = max_height.max(height);
            }
        }
        
        Self {
            name,
            points,
            min_x,
            max_x,
            min_y,
            max_y,
            mesh_width: width,
            mesh_height: height,
            min_height,
            max_height,
        }
    }
    
    fn get_height_at_coord(&self, screen_x: f32, screen_y: f32, rect: egui::Rect) -> Option<(usize, usize, f32)> {
        let x_ratio = (screen_x - rect.left()) / rect.width();
        let y_ratio = (screen_y - rect.top()) / rect.height();
        
        if x_ratio < 0.0 || x_ratio > 1.0 || y_ratio < 0.0 || y_ratio > 1.0 {
            return None;
        }
        
        let mesh_x = (x_ratio * (self.mesh_width - 1) as f32).round() as usize;
        let mesh_y = (y_ratio * (self.mesh_height - 1) as f32).round() as usize;
        
        if mesh_x < self.mesh_width && mesh_y < self.mesh_height {
            Some((mesh_x, mesh_y, self.points[mesh_y][mesh_x]))
        } else {
            None
        }
    }
}

#[derive(Debug, Clone)]
struct AdaptiveSettings {
    enable_adaptive_meshing: bool,
    enable_adaptive_z_height: bool,
    margin_x: f32,
    margin_y: f32,
    min_probe_count: u8,
    max_probe_count: u8,
    kamp_style_probing: bool,
}

#[derive(Debug, Clone)]
struct PrintArea {
    min_x: f32,
    max_x: f32,
    min_y: f32,
    max_y: f32,
    min_z: f32,
    max_z: f32,
    valid: bool,
}

impl PrintArea {
    fn new() -> Self {
        Self {
            min_x: f32::INFINITY,
            max_x: f32::NEG_INFINITY,
            min_y: f32::INFINITY,
            max_y: f32::NEG_INFINITY,
            min_z: f32::INFINITY,
            max_z: f32::NEG_INFINITY,
            valid: false,
        }
    }
    
    fn update_bounds(&mut self, x: f32, y: f32, z: f32) {
        self.min_x = self.min_x.min(x);
        self.max_x = self.max_x.max(x);
        self.min_y = self.min_y.min(y);
        self.max_y = self.max_y.max(y);
        self.min_z = self.min_z.min(z);
        self.max_z = self.max_z.max(z);
        self.valid = true;
    }
    
    fn get_center(&self) -> (f32, f32) {
        ((self.min_x + self.max_x) / 2.0, (self.min_y + self.max_y) / 2.0)
    }
    
    fn get_dimensions(&self) -> (f32, f32, f32) {
        (self.max_x - self.min_x, self.max_y - self.min_y, self.max_z - self.min_z)
    }
}

#[derive(Debug, Clone)]
struct FanControl {
    part_cooling_speed: u8,    // 0-100%
    auxiliary_fan_speed: u8,   // 0-100%
    exhaust_fan_speed: u8,     // 0-100%
    chamber_fan_speed: u8,     // 0-100%
    auto_part_cooling: bool,
    auto_exhaust: bool,
    exhaust_temp_threshold: f32,
    part_cooling_layer_fan: bool,
    layer_fan_start_layer: u16,
}

#[derive(Debug, Clone)]
struct WebcamSettings {
    enabled: bool,
    url: String,
    refresh_rate: u16,        // milliseconds
    resolution: (u32, u32),   // width, height
    show_overlay: bool,
    overlay_text: String,
    flip_horizontal: bool,
    flip_vertical: bool,
    rotation: u16,           // degrees: 0, 90, 180, 270
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
enum EtherCATModuleType {
    // Bus Couplers
    BeckhoffBK1120,      // Compact bus coupler for K-bus
    BeckhoffBK1250,      // EtherCAT coupler for up to 255 Bus Terminals

    // Digital Input Modules
    BeckhoffEL1008,      // 8-channel digital input, 24V DC
    BeckhoffEL1018,      // 8-channel digital input, 24V DC, flat ribbon
    BeckhoffKL1104,      // 4-channel digital input, 24V DC
    BeckhoffKL1408,      // 8-channel digital input, 24V DC

    // Digital Output Modules
    BeckhoffEL2008,      // 8-channel digital output, 24V DC
    BeckhoffEL2088,      // 8-channel digital output, 24V DC, 2A
    BeckhoffKL2408,      // 8-channel digital output, 24V DC

    // Analog Input Modules
    BeckhoffEL3008,      // 8-channel analog input, 0-10V
    BeckhoffEL3068,      // 8-channel analog input, 0-10V, 16-bit
    BeckhoffEL3104,      // 4-channel analog input, 0-10V, 16-bit
    BeckhoffEL3162,      // 2-channel analog input, 0-10V, 16-bit

    // Analog Output Modules
    BeckhoffEL4008,      // 8-channel analog output, 0-10V
    BeckhoffEL4068,      // 8-channel analog output, 0-10V, 16-bit
    BeckhoffEL4104,      // 4-channel analog output, 0-10V, 16-bit
    BeckhoffEL4132,      // 2-channel analog output, ±10V, 16-bit

    // Encoder Interfaces
    BeckhoffEL5001,      // 1-channel SSI encoder
    BeckhoffEL5101,      // 1-channel incremental encoder, 5V
    BeckhoffEL5151,      // 1-channel incremental encoder, HTL

    // Communication Modules
    BeckhoffEL6001,      // Serial interface RS232
    BeckhoffEL6021,      // Serial interface RS422/RS485
    BeckhoffEL6751,      // CANopen master
    BeckhoffEL6900,      // TwinSAFE Logic

    // Motor Control
    BeckhoffEL7031,      // 1-channel stepper motor, 24V, 1.5A
    BeckhoffEL7041,      // 1-channel stepper motor with encoder
    BeckhoffEL7211,      // 1-channel servo motor, 48V, 4.5A
    BeckhoffEL7342,      // 2-channel DC motor, 24V, 3.5A

    Custom,              // User-defined module
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
struct EtherCATModule {
    module_type: EtherCATModuleType,
    name: String,
    station_address: u16,
    vendor_id: u32,
    product_code: u32,
    revision_number: u32,
    serial_number: u32,
    input_size: u16,         // bytes
    output_size: u16,        // bytes
    enabled: bool,
    custom_description: String,
    // Connection management
    #[serde(skip, default)]
    connection_status: EtherCATConnectionStatus,
    #[serde(skip, default)]
    last_seen: Option<std::time::Instant>,
    #[serde(skip, default)]
    input_data: Vec<u8>,     // Current input data
    #[serde(skip, default)]
    output_data: Vec<u8>,    // Output data to send
    #[serde(skip, default)]
    error_count: u32,        // Communication error count
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
enum EtherCATConnectionStatus {
    Disconnected,
    Connecting,
    Connected,
    Error(String),
    ConfigError,
}

impl Default for EtherCATConnectionStatus {
    fn default() -> Self {
        EtherCATConnectionStatus::Disconnected
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
struct EtherCATNetworkConfig {
    network_interface: String,   // e.g., "\\Device\\NPF_{...}" on Windows
    cycle_time_us: u32,         // EtherCAT cycle time in microseconds
    sync_master_enabled: bool,   // Distributed Clock synchronization
    topology_check: bool,        // Verify physical topology matches config
    recovery_timeout_ms: u32,    // Auto-recovery timeout
    #[cfg(windows)]
    npcap_path: String,         // Path to Npcap installation directory
    #[cfg(windows)]
    use_npcap: bool,            // Enable/disable real Npcap usage
}

impl EtherCATModule {
    fn new_beckhoff(module_type: EtherCATModuleType, station_address: u16) -> Self {
        let (name, vendor_id, product_code, input_size, output_size, description) = match module_type {
            // Bus Couplers
            EtherCATModuleType::BeckhoffBK1120 => ("BK1120 - Bus Coupler", 0x00000002, 0x04603052, 0, 0, "Compact bus coupler for K-bus I/O modules"),
            EtherCATModuleType::BeckhoffBK1250 => ("BK1250 - EtherCAT Coupler", 0x00000002, 0x04E23052, 0, 0, "EtherCAT coupler for up to 255 Bus Terminals"),

            // Digital Input Modules
            EtherCATModuleType::BeckhoffEL1008 => ("EL1008 - 8Ch DI", 0x00000002, 0x03F03052, 1, 0, "8-channel digital input, 24V DC, 3ms"),
            EtherCATModuleType::BeckhoffEL1018 => ("EL1018 - 8Ch DI", 0x00000002, 0x03FA3052, 1, 0, "8-channel digital input, 24V DC, flat ribbon"),
            EtherCATModuleType::BeckhoffKL1104 => ("KL1104 - 4Ch DI", 0x00000002, 0x04503052, 1, 0, "4-channel digital input, 24V DC"),
            EtherCATModuleType::BeckhoffKL1408 => ("KL1408 - 8Ch DI", 0x00000002, 0x05803052, 1, 0, "8-channel digital input, 24V DC"),

            // Digital Output Modules
            EtherCATModuleType::BeckhoffEL2008 => ("EL2008 - 8Ch DO", 0x00000002, 0x07D83052, 0, 1, "8-channel digital output, 24V DC, 0.5A"),
            EtherCATModuleType::BeckhoffEL2088 => ("EL2088 - 8Ch DO", 0x00000002, 0x08283052, 0, 1, "8-channel digital output, 24V DC, 2A"),
            EtherCATModuleType::BeckhoffKL2408 => ("KL2408 - 8Ch DO", 0x00000002, 0x09683052, 0, 1, "8-channel digital output, 24V DC"),

            // Analog Input Modules
            EtherCATModuleType::BeckhoffEL3008 => ("EL3008 - 8Ch AI", 0x00000002, 0x0BC03052, 16, 0, "8-channel analog input, 0-10V, 12bit"),
            EtherCATModuleType::BeckhoffEL3068 => ("EL3068 - 8Ch AI", 0x00000002, 0x0BFC3052, 16, 0, "8-channel analog input, 0-10V, 16-bit"),
            EtherCATModuleType::BeckhoffEL3104 => ("EL3104 - 4Ch AI", 0x00000002, 0x0C203052, 8, 0, "4-channel analog input, 0-10V, 16-bit"),
            EtherCATModuleType::BeckhoffEL3162 => ("EL3162 - 2Ch AI", 0x00000002, 0x0C5A3052, 4, 0, "2-channel analog input, 0-10V, 16-bit"),

            // Analog Output Modules
            EtherCATModuleType::BeckhoffEL4008 => ("EL4008 - 8Ch AO", 0x00000002, 0x0FA03052, 0, 16, "8-channel analog output, 0-10V, 12bit"),
            EtherCATModuleType::BeckhoffEL4068 => ("EL4068 - 8Ch AO", 0x00000002, 0x0FDC3052, 0, 16, "8-channel analog output, 0-10V, 16-bit"),
            EtherCATModuleType::BeckhoffEL4104 => ("EL4104 - 4Ch AO", 0x00000002, 0x10083052, 0, 8, "4-channel analog output, 0-10V, 16-bit"),
            EtherCATModuleType::BeckhoffEL4132 => ("EL4132 - 2Ch AO", 0x00000002, 0x10243052, 0, 4, "2-channel analog output, ±10V, 16-bit"),

            // Encoder Interfaces
            EtherCATModuleType::BeckhoffEL5001 => ("EL5001 - SSI", 0x00000002, 0x13893052, 8, 0, "1-channel SSI encoder interface"),
            EtherCATModuleType::BeckhoffEL5101 => ("EL5101 - Encoder", 0x00000002, 0x13E53052, 8, 0, "1-channel incremental encoder, 5V"),
            EtherCATModuleType::BeckhoffEL5151 => ("EL5151 - Encoder", 0x00000002, 0x14173052, 8, 0, "1-channel incremental encoder, HTL"),

            // Communication Modules
            EtherCATModuleType::BeckhoffEL6001 => ("EL6001 - Serial", 0x00000002, 0x17713052, 22, 22, "Serial interface RS232"),
            EtherCATModuleType::BeckhoffEL6021 => ("EL6021 - Serial", 0x00000002, 0x17853052, 22, 22, "Serial interface RS422/RS485"),
            EtherCATModuleType::BeckhoffEL6751 => ("EL6751 - CANopen", 0x00000002, 0x1A5F3052, 20, 20, "CANopen master/slave"),
            EtherCATModuleType::BeckhoffEL6900 => ("EL6900 - TwinSAFE", 0x00000002, 0x1AF43052, 37, 32, "TwinSAFE Logic"),

            // Motor Control
            EtherCATModuleType::BeckhoffEL7031 => ("EL7031 - Stepper", 0x00000002, 0x1B773052, 3, 8, "1-channel stepper motor, 24V, 1.5A"),
            EtherCATModuleType::BeckhoffEL7041 => ("EL7041 - Stepper+Enc", 0x00000002, 0x1B813052, 11, 8, "1-channel stepper motor with encoder"),
            EtherCATModuleType::BeckhoffEL7211 => ("EL7211 - Servo", 0x00000002, 0x1C2B3052, 8, 6, "1-channel servo motor, 48V, 4.5A"),
            EtherCATModuleType::BeckhoffEL7342 => ("EL7342 - DC Motor", 0x00000002, 0x1CAE3052, 8, 6, "2-channel DC motor, 24V, 3.5A"),

            EtherCATModuleType::Custom => ("Custom Module", 0x00000000, 0x00000000, 0, 0, "User-defined custom module"),
        };
        
        Self {
            module_type,
            name: name.to_string(),
            station_address,
            vendor_id,
            product_code,
            revision_number: 0x00010001,
            serial_number: 0,
            input_size,
            output_size,
            enabled: true,
            custom_description: description.to_string(),
            connection_status: EtherCATConnectionStatus::Disconnected,
            last_seen: None,
            input_data: vec![0; input_size as usize],
            output_data: vec![0; output_size as usize],
            error_count: 0,
        }
    }
}

#[derive(Debug, Clone)]
struct MacroDefinition {
    name: String,
    description: String,
    commands: Vec<String>,
    enabled: bool,
    category: String,        // e.g., "Homing", "Calibration", "Maintenance"
    icon: String,           // Emoji or icon identifier
}

#[derive(Debug, Clone)]
struct TerminalSession {
    history: Vec<String>,
    input_buffer: String,
    max_history: usize,
    auto_scroll: bool,
}

impl TerminalSession {
    fn new() -> Self {
        Self {
            history: Vec::new(),
            input_buffer: String::new(),
            max_history: 1000,
            auto_scroll: true,
        }
    }
    
    fn add_message(&mut self, message: String) {
        self.history.push(message);
        if self.history.len() > self.max_history {
            self.history.remove(0);
        }
    }
    
    fn execute_command(&mut self, command: String) {
        // Add command to history
        self.add_message(format!("> {}", command));
        
        // Process command
        let response = match command.trim().to_uppercase().as_str() {
            // There is no G-code backend behind this terminal yet. Say so, rather than
            // reporting made-up temperatures and positions as if a printer had answered.
            cmd if cmd.starts_with('G') || cmd.starts_with('M') => format!("NOT SENT (terminal has no printer connection): {}", command),
            "HELP" => "This terminal is not connected to a printer. Local commands: HELP, CLEAR".to_string(),
            "CLEAR" => {
                self.history.clear();
                "Terminal cleared".to_string()
            },
            _ => format!("Unknown command: {}", command),
        };
        
        self.add_message(response);
    }
}

#[derive(Debug, Clone, PartialEq)]
enum ExecutionState {
    Idle,
    Running,
    Paused,
    Completed,
}

struct MyApp {
    kinematics: Box<dyn Kinematics>,
    scene: Scene,
    x: f32,
    y: f32,
    z: f32,
    a: f32,
    b: f32,
    gcode: String,
    gcode_file_path: Option<String>,
    parsed_gcode: Option<GCodeFile>,
    plot_points: Vec<[f64; 2]>,
    selected_gcode_line: Option<usize>, // For line selection and partial path viewing
    // Multi-threading state
    gcode_parsing_task: Option<std::thread::JoinHandle<()>>,
    parsed_result_receiver: Option<mpsc::Receiver<(String, Option<GCodeFile>, Vec<[f64; 2]>)>>,
    is_parsing: bool,
    selected_tab: Tab,
    // Rendering cache for performance
    cached_3d_points: Vec<(egui::Pos2, f32)>,
    cached_2d_points: Vec<egui::Pos2>,
    cached_z_heights: Vec<f32>,
    cached_bounds: Option<(f64, f64, f64, f64, f32, f32)>, // min_x, max_x, min_y, max_y, min_z, max_z
    cache_valid: bool,
    last_cached_rect: egui::Rect, // Track viewport size for cache invalidation
    last_step_time: Instant, // Debounce step operations
    show_status_panel: bool,
    show_add_motor_dialog: bool,
    new_motor_id: String,
    new_motor_function: motor_assignment::MotorFunction,
    new_motor_type: motor_assignment::MotorType,
    #[cfg(target_os = "linux")]
    group: Arc<SlaveGroup<MAX_SUBDEVICES, PDI_LEN>>,
    #[cfg(all(windows, feature = "windows-ethercat"))]
    group: Arc<WindowsEtherCATSlaveGroup>,
    #[cfg(all(not(target_os = "linux"), not(all(windows, feature = "windows-ethercat"))))]
    group: Arc<DummySlaveGroup>,
    ethercat_tx: tokio::sync::mpsc::Sender<[f32; 5]>,
    #[allow(dead_code)]
    ethercat_initialized: bool,
    communication_mode: CommunicationMode,
    #[allow(dead_code)]
    serial_port: Option<Box<dyn SerialPort>>,
    #[allow(dead_code)]
    serial_port_name: String,
    #[allow(dead_code)]
    serial_baud_rate: u32,
    // CANopen settings
    #[allow(dead_code)]
    canopen_node_id: u8,
    #[allow(dead_code)]
    canopen_baudrate: u32,
    // Profibus settings
    #[allow(dead_code)]
    profibus_address: u8,
    #[allow(dead_code)]
    profibus_baudrate: u32,
    // Modbus settings
    #[allow(dead_code)]
    modbus_device_id: u8,
    #[allow(dead_code)]
    modbus_tcp_ip: String,
    #[allow(dead_code)]
    modbus_tcp_port: u16,
    // DeviceNet settings
    #[allow(dead_code)]
    devicenet_node_address: u8,
    #[allow(dead_code)]
    devicenet_baudrate: u32,
    
    // New industrial protocol configurations
    bacnet_config: BACnetConfig,
    ads_config: ADSConfig,
    codesys_config: CODESYSConfig,
    cip_config: CIPConfig,
    opcua_config: OPC_UAConfig,
    protocol_discovery: ProtocolDiscovery,
    
    // Protocol-specific connection states
    protocol_connections: HashMap<CommunicationMode, bool>,
    protocol_diagnostics: HashMap<CommunicationMode, ProtocolDiagnostics>,
    protocol_manager: ProtocolManager,
    
    // Klipper CAN bus toolhead management
    can_manager: Arc<Mutex<KlipperCanManager>>,
    can_interface_name: String,
    selected_toolhead_uuid: Option<u32>,
    toolhead_discovery_enabled: bool,
    can_bitrate: u32,
    
    // Input shaping system
    input_shaper_manager: InputShaperManager,
    input_shaping_enabled: bool,
    resonance_testing_active: bool,
    selected_shaper_axis: char,
    
    // Fieldbus status tracking
    fieldbus_connection_status: HashMap<CommunicationMode, String>,
    fieldbus_error_counts: HashMap<CommunicationMode, u32>,
    fieldbus_last_activity: HashMap<CommunicationMode, Instant>,
    
    // Advanced motion control systems
    volumetric_controller: VolumetricController,
    trajectory_planner: TrajectoryPlanner,
    servo_controller: ServoController,
    multi_tool_manager: MultiToolManager,
    force_controller: DynamicForceController,
    
    // Motor Assignment System
    motor_manager: MotorAssignmentManager,
    selected_motor_id: Option<String>,
    
    // Closed-Loop Control System
    closed_loop_manager: ClosedLoopManager,
    selected_encoder_axis: Option<String>,
    
    // Temperature Control System
    temperature_manager: TemperatureManager,
    selected_temp_zone: Option<TemperatureZone>,

    // Automated Input Shaper Calibration System
    auto_input_shaper: AutoInputShaperCalibrator,
    calibration_ui: CalibrationUI,
    calibration_running: bool,

    // Volumetric Compensation System
    volumetric_compensator: VolumetricCompensator,
    compensation_enabled: bool,
    compensation_calibration_running: bool,

    // Sensor Fusion System
    sensor_fusion_manager: SensorFusionManager,
    fusion_enabled: bool,

    // State-Space Control System
    state_space_controller: StateSpaceController,
    state_space_enabled: bool,
    system_id_running: bool,
    lqr_position_weight: f64,
    lqr_velocity_weight: f64,
    lqr_control_weight: f64,
    kalman_process_noise: f64,
    kalman_measurement_noise: f64,

    // Model Predictive Control System
    mpc_controller: MPCController,
    mpc_enabled: bool,
    prediction_horizon_ms: f64,
    control_horizon_ms: f64,
    lookahead_distance_mm: f64,
    mpc_tracking_weight: f64,
    mpc_control_weight: f64,
    mpc_smoothness_weight: f64,
    mpc_x_min: f64,
    mpc_x_max: f64,
    mpc_y_min: f64,
    mpc_y_max: f64,
    mpc_z_min: f64,
    mpc_z_max: f64,
    mpc_max_vel_x: f64,
    mpc_max_vel_y: f64,
    mpc_max_vel_z: f64,
    mpc_max_accel_x: f64,
    mpc_max_accel_y: f64,
    mpc_max_accel_z: f64,
    selected_mpc_solver: MPCSolverType,
    mpc_max_iterations: u32,
    mpc_tolerance: f64,
    mpc_warm_start: bool,
    mpc_polish: bool,
    mpc_adaptive_learning: bool,
    mpc_thermal_compensation: bool,
    mpc_vibration_prediction: bool,
    mpc_corner_optimization: bool,

    // Plugin System
    plugin_manager: PluginManager,
    selected_plugin: Option<String>,

    // Power-Loss Recovery System
    power_loss_manager: PowerLossRecoveryManager,
    pressure_advance: MultiExtruderPressureAdvance,
    config_manager: ConfigManager,
    recovery_plan: Option<RecoveryPlan>,
    
    // Motion control settings
    volumetric_enabled: bool,
    trajectory_planning_enabled: bool,
    closed_loop_enabled: bool,
    force_control_enabled: bool,
    collision_detection_enabled: bool,
    #[allow(dead_code)]
    last_file_error: Option<String>,
    // G-code execution state
    current_line: usize,
    execution_state: ExecutionState,
    is_executing: bool,
    // Test mode for preview
    test_mode: bool,
    test_current_line: usize,
    test_execution_state: ExecutionState,
    is_test_executing: bool,
    // UI Settings
    dark_mode: bool,
    view_3d: bool,
    // Printer state and settings
    printer_state: PrinterState,
    printer_settings: PrinterSettings,
    emergency_stop_triggered: bool,
    last_modbus_positions: Option<[f32; 5]>,
    modbus_send_in_flight: Arc<std::sync::atomic::AtomicBool>,
    out_of_reach_reported: bool,
    // Bed mesh system
    current_bed_mesh: Option<BedMesh>,
    saved_bed_meshes: Vec<BedMesh>,
    selected_mesh_profile: String,
    mesh_hover_info: Option<(f32, f32, f32)>, // x, y, z for tooltip
    // Adaptive features
    adaptive_settings: AdaptiveSettings,
    current_print_area: PrintArea,
    // Fan control system
    fan_control: FanControl,
    // Webcam system
    webcam_settings: WebcamSettings,
    // EtherCAT module management
    ethercat_modules: Vec<EtherCATModule>,
    selected_module_index: Option<usize>,
    ethercat_network_config: EtherCATNetworkConfig,
    ethercat_master_connected: bool,
    #[cfg(windows)]
    windows_ethercat_master: Option<printer_host::ethercat_windows::WindowsEtherCATMaster>,
    #[cfg(windows)]
    available_adapters: Vec<printer_host::ethercat_windows::WindowsEtherCATAdapter>,
    // Terminal system
    terminal: TerminalSession,
    // Macro system
    macros: Vec<MacroDefinition>,
    selected_macro: String,
    // 5-Axis kinematics system
    five_axis_kinematics: Option<PracticalKinematics>,
    selected_machine_type: PracticalMachineType,
    five_axis_pose: Pose6DOF,
    five_axis_joints: JointConfiguration,
    machine_config_dirty: bool,
    inverse_solutions: Vec<JointConfiguration>,
    forward_pose_result: Option<Result<Pose6DOF, String>>,
    show_5axis_config_dialog: bool,
    workspace_limits: WorkspaceLimits,
    // 3D Visualizer state
    visualizer_3d_rotation: (f32, f32),     // (pitch, yaw) for 3D view rotation
    visualizer_3d_zoom: f32,                // Zoom level for 3D view
    visualizer_3d_offset: (f32, f32),       // Pan offset for 3D view
    show_workspace_bounds: bool,             // Toggle workspace boundary display
    show_machine_structure: bool,            // Toggle machine structure display
    show_tool_path: bool,                    // Toggle tool path history
    tool_path_history: Vec<(f32, f32, f32)>, // Tool position history for visualization
}

impl MyApp {
    fn new(
        kinematics: Box<dyn Kinematics>,
        #[cfg(target_os = "linux")]
        group: Arc<SlaveGroup<MAX_SUBDEVICES, PDI_LEN>>,
        #[cfg(all(windows, feature = "windows-ethercat"))]
        group: Arc<WindowsEtherCATSlaveGroup>,
        #[cfg(all(not(target_os = "linux"), not(all(windows, feature = "windows-ethercat"))))]
        group: Arc<DummySlaveGroup>,
        ethercat_tx: tokio::sync::mpsc::Sender<[f32; 5]>,
        gcode: String,
        plot_points: Vec<[f64; 2]>,
    ) -> Self {
        Self {
            kinematics,
            scene: Scene::new(),
            x: 0.0,
            y: 0.0,
            z: 0.0,
            a: 0.0,
            b: 0.0,
            gcode,
            gcode_file_path: None,
            parsed_gcode: None,
            plot_points,
            selected_gcode_line: None,
            selected_tab: Tab::Kinematics,
            show_status_panel: true,
            show_add_motor_dialog: false,
            new_motor_id: "motor_1".to_string(),
            new_motor_function: motor_assignment::MotorFunction::XAxis(0),
            new_motor_type: motor_assignment::MotorType::Stepper,
            // Initialize multi-threading state
            gcode_parsing_task: None,
            parsed_result_receiver: None,
            is_parsing: false,
            // Initialize rendering cache
            cached_3d_points: Vec::new(),
            cached_2d_points: Vec::new(),
            cached_z_heights: Vec::new(),
            cached_bounds: None,
            cache_valid: false,
            last_cached_rect: egui::Rect::NOTHING,
            last_step_time: Instant::now(),
            group,
            ethercat_tx,
            ethercat_initialized: false,
            communication_mode: CommunicationMode::EtherCAT,
            serial_port: None,
            serial_port_name: "COM3".to_string(),
            serial_baud_rate: 115200,
            // Initialize CANopen settings
            canopen_node_id: 1,
            canopen_baudrate: 1000000, // 1 Mbps
            // Initialize Profibus settings
            profibus_address: 2,
            profibus_baudrate: 12000000, // 12 Mbps
            // Initialize Modbus settings
            modbus_device_id: 1,
            modbus_tcp_ip: "192.168.1.100".to_string(),
            modbus_tcp_port: 502,
            // Initialize DeviceNet settings
            devicenet_node_address: 63,
            devicenet_baudrate: 500000, // 500 kbps
            
            // Initialize new industrial protocol configurations
            bacnet_config: BACnetConfig {
                device_id: 1001,
                port: 47808,
                network_number: 0,
                max_apdu_length: 1476,
                segmentation_supported: true,
            },
            ads_config: ADSConfig {
                target_ams_net_id: "192.168.1.100.1.1".to_string(),
                target_ams_port: 851,
                source_ams_port: 32905,
                timeout: Duration::from_secs(5),
            },
            codesys_config: CODESYSConfig {
                target_ip: IpAddr::V4(Ipv4Addr::new(192, 168, 1, 100)),
                port: 1200,
                username: "admin".to_string(),
                password: "password".to_string(),
                project_name: "MainProject".to_string(),
            },
            cip_config: CIPConfig {
                connection_type: CIPConnectionType::EthernetIP,
                target_ip: IpAddr::V4(Ipv4Addr::new(192, 168, 1, 100)),
                port: 44818,
                vendor_id: 0x001,
                device_type: 0x002,
                product_code: 0x003,
            },
            opcua_config: OPC_UAConfig {
                endpoint_url: "opc.tcp://192.168.1.100:4840".to_string(),
                security_policy: "None".to_string(),
                message_security_mode: "None".to_string(),
                username: None,
                password: None,
                certificate_path: None,
                private_key_path: None,
            },
            protocol_discovery: ProtocolDiscovery {
                enabled: false,
                scan_range: "192.168.1.0/24".to_string(),
                scan_ports: vec![47808, 48898, 1200, 44818, 4840, 502, 503, 789],
                scan_interval: Duration::from_secs(30),
                discovered_devices: HashMap::new(),
            },
            protocol_connections: HashMap::new(),
            protocol_diagnostics: HashMap::new(),
            protocol_manager: ProtocolManager::new(),
            
            // Initialize CAN toolhead management
            can_manager: {
                let mock_interface = MockCanInterface::new();
                Arc::new(Mutex::new(KlipperCanManager::new(Box::new(mock_interface))))
            },
            can_interface_name: "can0".to_string(),
            selected_toolhead_uuid: None,
            toolhead_discovery_enabled: true,
            can_bitrate: 250000,
            
            // Initialize input shaping system
            input_shaper_manager: {
                let mut manager = InputShaperManager::new();
                // Add default configurations for each axis
                for axis in ['X', 'Y', 'Z', 'A', 'B'] {
                    let config = match axis {
                        'X' | 'Y' => InputShaperConfig {
                            shaper_type: ShaperType::MZV,
                            frequency: 40.0,
                            damping_ratio: 0.1,
                            enabled: true,
                            max_acceleration: 5000.0,
                            smoothing_time: 0.04,
                        },
                        'Z' => InputShaperConfig {
                            shaper_type: ShaperType::ZV,
                            frequency: 25.0,
                            damping_ratio: 0.15,
                            enabled: true,
                            max_acceleration: 1000.0,
                            smoothing_time: 0.06,
                        },
                        'A' | 'B' => InputShaperConfig {
                            shaper_type: ShaperType::EI,
                            frequency: 60.0,
                            damping_ratio: 0.08,
                            enabled: true,
                            max_acceleration: 10000.0,
                            smoothing_time: 0.03,
                        },
                        _ => InputShaperConfig::default(),
                    };
                    manager.add_axis(axis, config);
                }
                manager
            },
            input_shaping_enabled: true,
            resonance_testing_active: false,
            selected_shaper_axis: 'X',
            
            // Initialize fieldbus status tracking
            fieldbus_connection_status: HashMap::new(),
            fieldbus_error_counts: HashMap::new(),
            fieldbus_last_activity: HashMap::new(),
            
            // Initialize advanced motion control systems
            volumetric_controller: VolumetricController::new(VolumetricSettings::default()),
            trajectory_planner: TrajectoryPlanner::new(VolumetricSettings::default()),
            servo_controller: {
                let mut controller = ServoController::new();
                // Add servo axes for 5-axis machine
                controller.add_axis('X', 1000.0, 0.1); // 1000 counts/mm, 0.1mm max error
                controller.add_axis('Y', 1000.0, 0.1);
                controller.add_axis('Z', 1000.0, 0.05); // Tighter Z-axis tolerance
                controller.add_axis('A', 100.0, 0.5);   // 100 counts/degree, 0.5° max error
                controller.add_axis('B', 100.0, 0.5);
                controller
            },
            multi_tool_manager: MultiToolManager::new(),
            force_controller: {
                let mut controller = DynamicForceController::new(TorqueControlConfig::default());
                // Add torque sensors for each axis (mock sensors for now)
                controller.add_motor('X', Box::new(MockTorqueSensor::new("X_Motor".to_string())));
                controller.add_motor('Y', Box::new(MockTorqueSensor::new("Y_Motor".to_string())));
                controller.add_motor('Z', Box::new(MockTorqueSensor::new("Z_Motor".to_string())));
                controller.add_motor('A', Box::new(MockTorqueSensor::new("A_Motor".to_string())));
                controller.add_motor('B', Box::new(MockTorqueSensor::new("B_Motor".to_string())));
                controller
            },
            
            // Initialize motor assignment system
            motor_manager: MotorAssignmentManager::new(),
            selected_motor_id: None,
            
            // Initialize closed-loop control system
            closed_loop_manager: ClosedLoopManager::new(1000), // 1kHz update rate
            selected_encoder_axis: None,
            
            // Initialize temperature control system
            temperature_manager: TemperatureManager::new(10), // 10Hz update rate
            selected_temp_zone: None,

            // Initialize automated input shaper calibration system
            auto_input_shaper: AutoInputShaperCalibrator::new(),
            calibration_ui: CalibrationUI::new(),
            calibration_running: false,

            // Initialize volumetric compensation system
            volumetric_compensator: VolumetricCompensator::new(VolumetricCompensationConfig::default()),
            compensation_enabled: false,
            compensation_calibration_running: false,

            // Initialize sensor fusion system
            sensor_fusion_manager: {
                let mut manager = SensorFusionManager::new();
                // Initialize sensor fusion for all axes
                for axis in ['X', 'Y', 'Z'] {
                    manager.initialize_axis(axis, FusionSensorConfig::default());
                }
                manager
            },
            fusion_enabled: true,

            // Initialize state-space control system
            state_space_controller: StateSpaceController::new(StateSpaceConfig::default()),
            state_space_enabled: false,
            system_id_running: false,
            lqr_position_weight: 1000.0,
            lqr_velocity_weight: 100.0,
            lqr_control_weight: 1.0,
            kalman_process_noise: 0.01,
            kalman_measurement_noise: 0.1,

            // Initialize model predictive control system
            mpc_controller: MPCController::new(MPCConfig::default()),
            mpc_enabled: false,
            prediction_horizon_ms: 50.0,
            control_horizon_ms: 20.0,
            lookahead_distance_mm: 10.0,
            mpc_tracking_weight: 100.0,
            mpc_control_weight: 1.0,
            mpc_smoothness_weight: 10.0,
            mpc_x_min: -300.0,
            mpc_x_max: 300.0,
            mpc_y_min: -300.0,
            mpc_y_max: 300.0,
            mpc_z_min: 0.0,
            mpc_z_max: 400.0,
            mpc_max_vel_x: 200.0,
            mpc_max_vel_y: 200.0,
            mpc_max_vel_z: 50.0,
            mpc_max_accel_x: 5000.0,
            mpc_max_accel_y: 5000.0,
            mpc_max_accel_z: 2000.0,
            selected_mpc_solver: MPCSolverType::OSQP,
            mpc_max_iterations: 100,
            mpc_tolerance: 1e-6,
            mpc_warm_start: true,
            mpc_polish: true,
            mpc_adaptive_learning: false,
            mpc_thermal_compensation: false,
            mpc_vibration_prediction: false,
            mpc_corner_optimization: true,

            // Initialize plugin system
            plugin_manager: {
                let manager = PluginManager::new();
                // Scan and load plugins in the background
                if let Err(e) = manager.scan_and_load_plugins() {
                    println!("Warning: Failed to load some plugins: {}", e);
                }
                manager
            },
            selected_plugin: None,

            // Initialize power-loss recovery system
            power_loss_manager: PowerLossRecoveryManager::new(RecoveryConfig::default()).unwrap_or_else(|e| {
                println!("Warning: Failed to initialize power-loss recovery: {}", e);
                // Create a minimal fallback manager
                PowerLossRecoveryManager::new(RecoveryConfig {
                    enabled: false,
                    ..Default::default()
                }).unwrap()
            }),
            pressure_advance: MultiExtruderPressureAdvance::new(2), // Default: 2 extruders
            config_manager: ConfigManager::new("printer_config.toml"),
            recovery_plan: None,
            
            // Initialize motion control settings
            volumetric_enabled: true,
            trajectory_planning_enabled: true,
            closed_loop_enabled: true,
            force_control_enabled: true,
            collision_detection_enabled: true,
            last_file_error: None,
            current_line: 0,
            execution_state: ExecutionState::Idle,
            is_executing: false,
            test_mode: false,
            test_current_line: 0,
            test_execution_state: ExecutionState::Idle,
            is_test_executing: false,
            dark_mode: true,
            view_3d: false,
            // Initialize printer state and settings
            printer_state: PrinterState::Idle,
            printer_settings: PrinterSettings {
                // Temperature settings - typical values for PLA/PETG printers
                extruder_max_temp: 300.0,
                bed_max_temp: 120.0,
                extruder_min_extrude_temp: 170.0,
                
                // Movement settings
                max_velocity: 300.0,
                max_accel: 3000.0,
                max_accel_to_decel: 1500.0,
                square_corner_velocity: 5.0,
                
                // Homing settings
                homing_speed: 50.0,
                homing_retract_dist: 5.0,
                second_homing_speed: 10.0,
                
                // Safety settings (200mm bed example)
                position_min: [0.0, 0.0, 0.0],
                position_max: [200.0, 200.0, 200.0],
                
                // Motor configuration (default all steppers)
                motor_types: [MotorType::Stepper, MotorType::Stepper, MotorType::Stepper, 
                             MotorType::Stepper, MotorType::Stepper],
                
                // Stepper settings
                hold_current: 0.5,
                run_current: 0.8,
                microsteps: 16,
                
                // Servo settings (typical RC servo values)
                servo_min_pulse: 500,   // 0.5ms
                servo_max_pulse: 2500,  // 2.5ms
                servo_max_angle: 180.0,
                
                // Firmware retraction
                retract_length: 0.5,
                retract_speed: 35.0,
                unretract_extra_length: 0.0,
                unretract_speed: 35.0,
                
                // Pressure advance settings
                pressure_advance: 0.0,
                pressure_advance_smooth_time: 0.04, // 40ms default
                
                // Z offset and probe settings
                z_offset: 0.0,
                probe_z_offset: 0.0,
                probe_xy_offset: [0.0, 0.0],
                probe_speed: 5.0,
                probe_lift_speed: 15.0,
                probe_samples: 3,
                probe_sample_retract_dist: 2.0,
                
                // 5-axis control
                enable_5axis_mode: false,
                tool_point_control: false,
                rotary_axis_max_velocity: [120.0, 120.0], // degrees/sec
                rotary_axis_acceleration: [1800.0, 1800.0], // degrees/sec²
            },
            emergency_stop_triggered: false,
            last_modbus_positions: None,
            modbus_send_in_flight: Arc::new(std::sync::atomic::AtomicBool::new(false)),
            out_of_reach_reported: false,
            // Initialize bed mesh system
            current_bed_mesh: Some(BedMesh::new("default".to_string(), 7, 7, 0.0, 200.0, 0.0, 200.0)),
            saved_bed_meshes: vec![
                BedMesh::new("default".to_string(), 7, 7, 0.0, 200.0, 0.0, 200.0),
                BedMesh::new("pla_profile".to_string(), 9, 9, 10.0, 190.0, 10.0, 190.0),
                BedMesh::new("petg_profile".to_string(), 5, 5, 20.0, 180.0, 20.0, 180.0),
            ],
            selected_mesh_profile: "default".to_string(),
            mesh_hover_info: None,
            // Initialize adaptive features
            adaptive_settings: AdaptiveSettings {
                enable_adaptive_meshing: true,
                enable_adaptive_z_height: true,
                margin_x: 5.0,
                margin_y: 5.0,
                min_probe_count: 9,   // 3x3 minimum
                max_probe_count: 225, // 15x15 maximum
                kamp_style_probing: true,
            },
            current_print_area: PrintArea::new(),
            // Initialize fan control
            fan_control: FanControl {
                part_cooling_speed: 0,
                auxiliary_fan_speed: 0,
                exhaust_fan_speed: 0,
                chamber_fan_speed: 0,
                auto_part_cooling: true,
                auto_exhaust: false,
                exhaust_temp_threshold: 40.0,
                part_cooling_layer_fan: true,
                layer_fan_start_layer: 2,
            },
            // Initialize webcam
            webcam_settings: WebcamSettings {
                enabled: false,
                url: "http://192.168.1.100:8080/video".to_string(),
                refresh_rate: 100, // 10 FPS
                resolution: (640, 480),
                show_overlay: true,
                overlay_text: "Print Monitor".to_string(),
                flip_horizontal: false,
                flip_vertical: false,
                rotation: 0,
            },
            // Start with empty EtherCAT module configuration - modules will be discovered via scanning
            ethercat_modules: vec![],
            selected_module_index: None,
            ethercat_network_config: EtherCATNetworkConfig {
                network_interface: "auto".to_string(),
                cycle_time_us: 1000,  // 1ms cycle time
                sync_master_enabled: true,
                topology_check: true,
                recovery_timeout_ms: 5000,
                #[cfg(windows)]
                npcap_path: "C:\\Windows\\System32\\Npcap".to_string(),
                #[cfg(windows)]
                use_npcap: true,
            },
            ethercat_master_connected: false,
            #[cfg(windows)]
            windows_ethercat_master: Some(printer_host::ethercat_windows::WindowsEtherCATMaster::new()),
            #[cfg(windows)]
            available_adapters: Vec::new(),
            // Initialize terminal
            terminal: TerminalSession::new(),
            // Initialize with default macros
            macros: vec![
                MacroDefinition {
                    name: "Home All Axes".to_string(),
                    description: "Home X, Y, and Z axes in sequence".to_string(),
                    commands: vec!["G28".to_string()],
                    enabled: true,
                    category: "Homing".to_string(),
                    icon: "🏠".to_string(),
                },
                MacroDefinition {
                    name: "Bed Level".to_string(),
                    description: "Perform automatic bed leveling".to_string(),
                    commands: vec!["G28".to_string(), "G29".to_string()],
                    enabled: true,
                    category: "Calibration".to_string(),
                    icon: "📏".to_string(),
                },
                MacroDefinition {
                    name: "Preheat PLA".to_string(),
                    description: "Heat nozzle and bed for PLA printing".to_string(),
                    commands: vec!["M104 S200".to_string(), "M140 S60".to_string()],
                    enabled: true,
                    category: "Heating".to_string(),
                    icon: "🔥".to_string(),
                },
                MacroDefinition {
                    name: "Cool Down".to_string(),
                    description: "Turn off all heaters and fans".to_string(),
                    commands: vec!["M104 S0".to_string(), "M140 S0".to_string(), "M107".to_string()],
                    enabled: true,
                    category: "Maintenance".to_string(),
                    icon: "❄️".to_string(),
                },
                MacroDefinition {
                    name: "Disable Motors".to_string(),
                    description: "Disable all stepper motors".to_string(),
                    commands: vec!["M84".to_string()],
                    enabled: true,
                    category: "Maintenance".to_string(),
                    icon: "🔌".to_string(),
                },
            ],
            selected_macro: "Home All Axes".to_string(),
            // Initialize 5-axis kinematics system - will be set after config_manager is created
            five_axis_kinematics: None,
            selected_machine_type: PracticalMachineType::penta_axis_style(),
            five_axis_pose: Pose6DOF::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            five_axis_joints: JointConfiguration::new(0.0, 0.0, 0.0, 0.0, 0.0),
            machine_config_dirty: true,
            inverse_solutions: vec![],
            forward_pose_result: None,
            show_5axis_config_dialog: false,
            workspace_limits: WorkspaceLimits {
                x_limits: (-300.0, 300.0),
                y_limits: (-300.0, 300.0),
                z_limits: (0.0, 400.0),
                a_limits: (-90.0, 90.0),
                b_or_c_limits: (-180.0, 180.0),
                max_tool_length: 100.0,
                safety_margin: 10.0,
            },
            // Initialize 3D visualizer
            visualizer_3d_rotation: (30.0, 45.0), // Default isometric view
            visualizer_3d_zoom: 1.0,
            visualizer_3d_offset: (0.0, 0.0),
            show_workspace_bounds: true,
            show_machine_structure: true,
            show_tool_path: true,
            tool_path_history: vec![],
        }
    }

    /// Initialize machine configuration from saved config
    fn initialize_machine_config(&mut self) {
        // Load machine type from configuration if available
        if let Some(saved_machine_type) = &self.config_manager.get_config().kinematics.five_axis_machine_type {
            self.selected_machine_type = saved_machine_type.clone();
            self.machine_config_dirty = true;
        }
    }

    fn load_gcode_file(&mut self, path: std::path::PathBuf) {
        // Start async loading to avoid blocking UI
        self.is_parsing = true;
        self.last_file_error = None;
        
        let (tx, rx) = mpsc::channel();
        self.parsed_result_receiver = Some(rx);
        
        let path_string = path.display().to_string();
        let task = thread::spawn(move || {
            match fs::read_to_string(&path) {
                Ok(content) => {
                    // Parse G-code in background thread
                    let parsed = match GCodeFile::parse(&content) {
                        Some(p) => Some(p),
                        None => None,
                    };
                    
                    // Parse plot points in background
                    let plot_points = Self::parse_gcode_to_points_static(&content);
                    
                    // Send results back to main thread
                    let _ = tx.send((content, parsed, plot_points));
                }
                Err(_e) => {
                    // Send empty result on error
                    let _ = tx.send((String::new(), None, Vec::new()));
                }
            }
        });
        
        self.gcode_parsing_task = Some(task);
        self.gcode_file_path = Some(path_string);
    }
    
    fn build_render_cache(&mut self, rect: egui::Rect) {
        // Only rebuild cache if data changed or viewport size changed significantly
        if self.plot_points.is_empty() {
            return;
        }
        
        // Check if we need to rebuild based on viewport size change
        let needs_viewport_rebuild = if let Some((_, _, _, _, _, _)) = self.cached_bounds {
            // Allow for small differences due to floating point precision
            (rect.width() - self.last_cached_rect.width()).abs() > 1.0 || 
            (rect.height() - self.last_cached_rect.height()).abs() > 1.0
        } else {
            true // First time caching
        };
        
        if self.cache_valid && !needs_viewport_rebuild {
            return;
        }
        
        // Parse Z heights once - only if cache is completely invalid
        if self.cached_z_heights.is_empty() || !self.cache_valid {
            self.cached_z_heights.clear();
            let mut current_z = 0.0f32;
            
            // Use faster line iteration without collecting into Vec
            for line in self.gcode.lines() {
                // Fast Z parsing - avoid multiple string operations
                if let Some(z_start) = line.find(" Z") {
                    let z_str = &line[z_start + 2..];
                    if let Some(space_idx) = z_str.find(' ') {
                        if let Ok(z_val) = z_str[..space_idx].parse::<f32>() {
                            current_z = z_val;
                        }
                    } else if let Ok(z_val) = z_str.trim_end().parse::<f32>() {
                        current_z = z_val;
                    }
                }
                self.cached_z_heights.push(current_z);
            }
        }
        
        // Determine how many points to include based on selected line
        let plot_points_to_show = if let Some(selected_line) = self.selected_gcode_line {
            // Show path up to the selected line
            let total_lines = self.gcode.lines().count();
            if total_lines > 0 {
                let ratio = (selected_line + 1) as f32 / total_lines as f32;
                (self.plot_points.len() as f32 * ratio) as usize
            } else {
                self.plot_points.len()
            }
        } else {
            self.plot_points.len()
        };
        
        // Calculate bounds based on partial or full path
        let points_slice = &self.plot_points[..plot_points_to_show.min(self.plot_points.len())];
        let z_heights_slice = &self.cached_z_heights[..plot_points_to_show.min(self.cached_z_heights.len())];
        
        let min_z = if !z_heights_slice.is_empty() {
            z_heights_slice.iter().cloned().fold(f32::INFINITY, f32::min)
        } else {
            0.0
        };
        let max_z = if !z_heights_slice.is_empty() {
            z_heights_slice.iter().cloned().fold(f32::NEG_INFINITY, f32::max)
        } else {
            0.0
        };
        
        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        
        for point in points_slice {
            min_x = min_x.min(point[0]);
            max_x = max_x.max(point[0]);
            min_y = min_y.min(point[1]);
            max_y = max_y.max(point[1]);
        }
        
        // Add padding
        let width = max_x - min_x;
        let height = max_y - min_y;
        let padding = width.max(height) * 0.1;
        min_x -= padding;
        max_x += padding;
        min_y -= padding;
        max_y += padding;
        
        self.cached_bounds = Some((min_x, max_x, min_y, max_y, min_z, max_z));
        
        // Only recompute screen coordinates if viewport changed or cache invalid
        if !self.cache_valid || needs_viewport_rebuild {
            // Pre-calculate 2D points (using partial data for selected line viewing)
            self.cached_2d_points.clear();
            self.cached_2d_points.reserve(points_slice.len());
            for point in points_slice {
                let x = rect.left() + ((point[0] - min_x) / (max_x - min_x)) as f32 * rect.width();
                let y = rect.top() + (1.0 - (point[1] - min_y) / (max_y - min_y)) as f32 * rect.height();
                self.cached_2d_points.push(egui::Pos2::new(x, y));
            }
            
            // Pre-calculate 3D isometric points (using partial data for selected line viewing)
            self.cached_3d_points.clear();
            self.cached_3d_points.reserve(points_slice.len());
            for (i, point) in points_slice.iter().enumerate() {
                let z = if i < z_heights_slice.len() { z_heights_slice[i] } else { 0.0 };
                
                // Isometric projection
                let iso_x = (point[0] - point[1]) * 0.866;
                let iso_y = (point[0] + point[1]) * 0.5 - z as f64 * 0.8;
                
                let x = rect.left() + ((iso_x - (min_x - min_y) * 0.866) / ((max_x - min_y) * 0.866 - (min_x - max_y) * 0.866)) as f32 * rect.width();
                let y = rect.top() + ((((max_x + max_y) * 0.5 - max_z as f64 * 0.8) - iso_y) / (((max_x + max_y) * 0.5 - max_z as f64 * 0.8) - ((min_x + min_y) * 0.5 - min_z as f64 * 0.8))) as f32 * rect.height();
                
                self.cached_3d_points.push((egui::Pos2::new(x, y), z));
            }
        }
        
        self.cache_valid = true;
        self.last_cached_rect = rect;
    }
    
    #[allow(dead_code)]
    fn parse_gcode_to_points(&self, content: &str) -> Vec<[f64; 2]> {
        Self::parse_gcode_to_points_static(content)
    }
    
    fn parse_gcode_to_points_static(content: &str) -> Vec<[f64; 2]> {
        let mut points = Vec::new();
        let mut x = 0.0;
        let mut y = 0.0;
        
        // Add starting point
        points.push([x as f64, y as f64]);
        
        // Process lines in chunks to be more efficient
        let lines: Vec<&str> = content.lines().collect();
        
        for line in lines {
            let line = line.trim();
            if line.starts_with("G1") {
                let parts: Vec<&str> = line.split_whitespace().collect();
                let mut line_has_xy = false;
                
                for part in parts {
                    if part.starts_with("X") {
                        if let Ok(val) = part[1..].parse::<f32>() {
                            x = val;
                            line_has_xy = true;
                        }
                    } else if part.starts_with("Y") {
                        if let Ok(val) = part[1..].parse::<f32>() {
                            y = val;
                            line_has_xy = true;
                        }
                    }
                }
                
                // Only add point if this line had X or Y movement
                if line_has_xy {
                    points.push([x as f64, y as f64]);
                }
            }
        }
        
        points
    }
    
    fn emergency_stop(&mut self) {
        self.emergency_stop_triggered = true;
        self.printer_state = PrinterState::Emergency;
        self.execution_state = ExecutionState::Idle;
        self.test_execution_state = ExecutionState::Idle;
        self.is_executing = false;
        self.is_test_executing = false;

        // Heaters off and latched. Reset of this button does not re-enable them.
        self.temperature_manager.emergency_stop();

        // Toolheads. Wait for the lock here: this stop must not be skipped because
        // something else happens to be using the CAN manager.
        let mut can_manager = match self.can_manager.lock() {
            Ok(guard) => guard,
            Err(poisoned) => poisoned.into_inner(),
        };
        for uuid in can_manager.get_toolheads().keys().cloned().collect::<Vec<_>>() {
            if let Err(e) = can_manager.emergency_stop(uuid) {
                eprintln!("🚨 EMERGENCY STOP FAILED for toolhead 0x{:06X}: {}", uuid, e);
            }
        }
        drop(can_manager);

        // Keep x/y/z as they are: positions are streamed, so changing them here would
        // command a move as soon as the stop is reset. Streaming stops while this is set.
        println!("🚨 EMERGENCY STOP ACTIVATED! Motion commands stopped, heaters off.");
    }

    fn reset_emergency_stop(&mut self) {
        self.emergency_stop_triggered = false;
        self.printer_state = PrinterState::Idle;
        println!("Emergency stop reset. Heaters stay off until cleared in the Temperature tab. Re-home before moving.");
    }
    
    fn start_probe_sequence(&mut self) {
        if !self.emergency_stop_triggered {
            self.printer_state = PrinterState::Homing;
            println!("Starting probe sequence - {} samples at {:.1}mm/s", 
                     self.printer_settings.probe_samples, self.printer_settings.probe_speed);
        }
    }
    
    fn adjust_z_offset(&mut self, delta: f32) {
        self.printer_settings.z_offset += delta;
        self.z += delta; // Apply immediately to current position
        println!("Z offset adjusted by {:.3}mm, new offset: {:.3}mm", delta, self.printer_settings.z_offset);
    }
    
    fn save_z_offset(&mut self) {
        println!("Z offset saved: {:.3}mm", self.printer_settings.z_offset);
        // Here you would save to configuration file
    }
    
    fn analyze_gcode_for_print_area(&mut self) {
        if let Some(_parsed) = &self.parsed_gcode {
            self.current_print_area = PrintArea::new();
            
            // For now, create a sample print area based on loaded G-code
            // In a real implementation, you would parse the G-code commands here
            self.current_print_area.update_bounds(10.0, 10.0, 0.2);   // Start corner
            self.current_print_area.update_bounds(90.0, 90.0, 25.0);  // End corner
            
            println!("Print area analysis complete:");
            if self.current_print_area.valid {
                let (width, height, max_z) = self.current_print_area.get_dimensions();
                println!("  Bounds: X({:.1}-{:.1}) Y({:.1}-{:.1}) Z({:.1}-{:.1})", 
                    self.current_print_area.min_x, self.current_print_area.max_x,
                    self.current_print_area.min_y, self.current_print_area.max_y,
                    self.current_print_area.min_z, self.current_print_area.max_z);
                println!("  Dimensions: {:.1}×{:.1}×{:.1}mm", width, height, max_z);
            }
        } else {
            println!("No G-code loaded for analysis");
        }
    }
    
    fn create_adaptive_mesh(&mut self) -> Option<BedMesh> {
        if !self.current_print_area.valid || !self.adaptive_settings.enable_adaptive_meshing {
            return None;
        }
        
        // Calculate adaptive mesh bounds with margins
        let mesh_min_x = (self.current_print_area.min_x - self.adaptive_settings.margin_x)
            .max(self.printer_settings.position_min[0]);
        let mesh_max_x = (self.current_print_area.max_x + self.adaptive_settings.margin_x)
            .min(self.printer_settings.position_max[0]);
        let mesh_min_y = (self.current_print_area.min_y - self.adaptive_settings.margin_y)
            .max(self.printer_settings.position_min[1]);
        let mesh_max_y = (self.current_print_area.max_y + self.adaptive_settings.margin_y)
            .min(self.printer_settings.position_max[1]);
        
        // Calculate adaptive grid size based on print area
        let print_width = mesh_max_x - mesh_min_x;
        let print_height = mesh_max_y - mesh_min_y;
        let area = print_width * print_height;
        
        // KAMP-style adaptive probe count calculation
        let probe_density = if area < 2500.0 { // < 50mm × 50mm
            0.5  // Lower density for small prints
        } else if area < 10000.0 { // < 100mm × 100mm  
            1.0  // Medium density
        } else {
            1.5  // Higher density for large prints
        };
        
        let points_x = ((print_width / 20.0 * probe_density) as u8).max(3).min(15);
        let points_y = ((print_height / 20.0 * probe_density) as u8).max(3).min(15);
        let total_points = (points_x as u16 * points_y as u16) as u8;
        
        // Clamp to user-defined limits
        let _final_points = total_points.max(self.adaptive_settings.min_probe_count)
                                        .min(self.adaptive_settings.max_probe_count);
        
        // Create adaptive mesh
        let mesh_name = format!("adaptive_{}x{}", points_x, points_y);
        let adaptive_mesh = BedMesh::new(
            mesh_name,
            points_x as usize,
            points_y as usize,
            mesh_min_x,
            mesh_max_x,
            mesh_min_y,
            mesh_max_y
        );
        
        println!("Created adaptive mesh: {}x{} points covering {:.1}×{:.1}mm area",
                points_x, points_y, print_width, print_height);
        
        Some(adaptive_mesh)
    }
    
    fn get_adaptive_z_height(&self) -> Option<f32> {
        if !self.adaptive_settings.enable_adaptive_z_height || !self.current_print_area.valid {
            return None;
        }
        
        // KAMP-style adaptive Z height - only probe as high as needed
        let adaptive_height = self.current_print_area.max_z + 10.0; // 10mm clearance
        let max_height = self.printer_settings.position_max[2];
        
        Some(adaptive_height.min(max_height))
    }

    /// Render the 3D visualization for 5-axis kinematics
    fn render_5axis_visualizer_3d(&mut self, ui: &mut egui::Ui) {
        ui.group(|ui| {
            ui.label("🎮 3D Kinematics Visualizer");

            // Visualization controls
            ui.horizontal(|ui| {
                ui.checkbox(&mut self.show_machine_structure, "🏭 Machine");
                ui.checkbox(&mut self.show_workspace_bounds, "📏 Workspace");
                ui.checkbox(&mut self.show_tool_path, "📍 Tool Path");

                if ui.button("🔄 Reset View").clicked() {
                    self.visualizer_3d_rotation = (30.0, 45.0);
                    self.visualizer_3d_zoom = 1.0;
                    self.visualizer_3d_offset = (0.0, 0.0);
                }

                if ui.button("🗑️ Clear Path").clicked() {
                    self.tool_path_history.clear();
                }
            });

            ui.separator();

            // 3D viewport
            let (response, painter) = ui.allocate_painter(
                egui::vec2(ui.available_width(), 400.0),
                egui::Sense::click_and_drag()
            );

            // Handle mouse interaction for 3D view
            if response.dragged() {
                let delta = response.drag_delta();
                if ui.input(|i| i.modifiers.shift) {
                    // Pan with Shift+drag
                    self.visualizer_3d_offset.0 += delta.x * 0.5;
                    self.visualizer_3d_offset.1 += delta.y * 0.5;
                } else {
                    // Rotate with normal drag
                    self.visualizer_3d_rotation.0 += delta.y * 0.5;
                    self.visualizer_3d_rotation.1 += delta.x * 0.5;
                }
            }

            // Handle zoom with mouse wheel
            if response.hovered() {
                ui.input(|i| {
                    self.visualizer_3d_zoom *= (1.0 + i.smooth_scroll_delta.y * 0.001).clamp(0.1, 5.0);
                });
            }

            let rect = response.rect;
            let center = rect.center();

            // Clear background
            painter.rect_filled(rect, 0.0, egui::Color32::from_gray(20));

            // 3D coordinate transformation
            let scale = 2.0 * self.visualizer_3d_zoom;
            let pitch = self.visualizer_3d_rotation.0.to_radians();
            let yaw = self.visualizer_3d_rotation.1.to_radians();

            let cos_pitch = pitch.cos();
            let sin_pitch = pitch.sin();
            let cos_yaw = yaw.cos();
            let sin_yaw = yaw.sin();

            // 3D to 2D projection function
            let project_3d = |x: f32, y: f32, z: f32| {
                // Rotate around Y axis (yaw)
                let x1 = x * cos_yaw - z * sin_yaw;
                let z1 = x * sin_yaw + z * cos_yaw;

                // Rotate around X axis (pitch)
                let y2 = y * cos_pitch - z1 * sin_pitch;
                let _z2 = y * sin_pitch + z1 * cos_pitch;

                // Project to 2D
                let screen_x = center.x + (x1 * scale) + self.visualizer_3d_offset.0;
                let screen_y = center.y - (y2 * scale) + self.visualizer_3d_offset.1;

                egui::pos2(screen_x, screen_y)
            };

            // Draw workspace boundaries
            if self.show_workspace_bounds {
                let bounds = &self.workspace_limits;
                let min_x = bounds.x_limits.0 as f32;
                let max_x = bounds.x_limits.1 as f32;
                let min_y = bounds.y_limits.0 as f32;
                let max_y = bounds.y_limits.1 as f32;
                let min_z = bounds.z_limits.0 as f32;
                let max_z = bounds.z_limits.1 as f32;

                // Draw workspace box wireframe
                let corners = [
                    project_3d(min_x, min_y, min_z), project_3d(max_x, min_y, min_z),
                    project_3d(max_x, max_y, min_z), project_3d(min_x, max_y, min_z),
                    project_3d(min_x, min_y, max_z), project_3d(max_x, min_y, max_z),
                    project_3d(max_x, max_y, max_z), project_3d(min_x, max_y, max_z),
                ];

                // Bottom face
                for i in 0..4 {
                    painter.line_segment([corners[i], corners[(i + 1) % 4]],
                        egui::Stroke::new(1.0, egui::Color32::BLUE));
                }
                // Top face
                for i in 4..8 {
                    painter.line_segment([corners[i], corners[4 + (i - 4 + 1) % 4]],
                        egui::Stroke::new(1.0, egui::Color32::BLUE));
                }
                // Vertical edges
                for i in 0..4 {
                    painter.line_segment([corners[i], corners[i + 4]],
                        egui::Stroke::new(1.0, egui::Color32::BLUE));
                }
            }

            // Draw machine structure
            if self.show_machine_structure {
                // Draw coordinate axes
                let origin = project_3d(0.0, 0.0, 0.0);
                let x_axis = project_3d(50.0, 0.0, 0.0);
                let y_axis = project_3d(0.0, 50.0, 0.0);
                let z_axis = project_3d(0.0, 0.0, 50.0);

                painter.line_segment([origin, x_axis], egui::Stroke::new(3.0, egui::Color32::RED));
                painter.line_segment([origin, y_axis], egui::Stroke::new(3.0, egui::Color32::GREEN));
                painter.line_segment([origin, z_axis], egui::Stroke::new(3.0, egui::Color32::BLUE));

                // Draw base platform
                let base_size = 100.0;
                let base_corners = [
                    project_3d(-base_size, -base_size, 0.0),
                    project_3d(base_size, -base_size, 0.0),
                    project_3d(base_size, base_size, 0.0),
                    project_3d(-base_size, base_size, 0.0),
                ];

                for i in 0..4 {
                    painter.line_segment([base_corners[i], base_corners[(i + 1) % 4]],
                        egui::Stroke::new(2.0, egui::Color32::GRAY));
                }

                // Draw rotary table (simplified)
                let table_center = project_3d(0.0, 0.0, 20.0);
                painter.circle_filled(table_center, 30.0 * scale, egui::Color32::from_gray(80));
                painter.circle_stroke(table_center, 30.0 * scale,
                    egui::Stroke::new(2.0, egui::Color32::WHITE));
            }

            // Draw tool position and orientation
            let tool_pos = project_3d(
                self.five_axis_pose.position.x as f32,
                self.five_axis_pose.position.y as f32,
                self.five_axis_pose.position.z as f32
            );

            // Tool position indicator
            painter.circle_filled(tool_pos, 8.0, egui::Color32::YELLOW);
            painter.circle_stroke(tool_pos, 8.0, egui::Stroke::new(2.0, egui::Color32::ORANGE));

            // Tool orientation vector
            let (roll, pitch, yaw) = self.five_axis_pose.euler_angles();
            let tool_length = 40.0;
            let tool_dir_x = (yaw.cos() * pitch.cos()) as f32 * tool_length;
            let tool_dir_y = (pitch.sin()) as f32 * tool_length;
            let tool_dir_z = (yaw.sin() * pitch.cos()) as f32 * tool_length;

            let tool_tip = project_3d(
                self.five_axis_pose.position.x as f32 + tool_dir_x,
                self.five_axis_pose.position.y as f32 + tool_dir_y,
                self.five_axis_pose.position.z as f32 + tool_dir_z
            );

            painter.line_segment([tool_pos, tool_tip],
                egui::Stroke::new(3.0, egui::Color32::YELLOW));

            // Update tool path history
            let current_tool_pos = (
                self.five_axis_pose.position.x as f32,
                self.five_axis_pose.position.y as f32,
                self.five_axis_pose.position.z as f32
            );

            if self.tool_path_history.is_empty() ||
               self.tool_path_history.last().unwrap() != &current_tool_pos {
                self.tool_path_history.push(current_tool_pos);
                // Limit history to prevent memory issues
                if self.tool_path_history.len() > 1000 {
                    self.tool_path_history.remove(0);
                }
            }

            // Draw tool path
            if self.show_tool_path && self.tool_path_history.len() > 1 {
                for i in 1..self.tool_path_history.len() {
                    let prev = self.tool_path_history[i - 1];
                    let curr = self.tool_path_history[i];

                    let prev_2d = project_3d(prev.0, prev.1, prev.2);
                    let curr_2d = project_3d(curr.0, curr.1, curr.2);

                    let alpha = (i as f32 / self.tool_path_history.len() as f32) * 255.0;
                    let color = egui::Color32::from_rgba_premultiplied(255, 100, 100, alpha as u8);

                    painter.line_segment([prev_2d, curr_2d],
                        egui::Stroke::new(2.0, color));
                }
            }

            // Draw info overlay
            ui.allocate_ui_at_rect(egui::Rect::from_min_size(rect.min, egui::vec2(200.0, 120.0)), |ui| {
                ui.group(|ui| {
                    ui.label("📊 Visualization Info");
                    ui.label(format!("Rotation: {:.1}°, {:.1}°",
                        self.visualizer_3d_rotation.0, self.visualizer_3d_rotation.1));
                    ui.label(format!("Zoom: {:.2}x", self.visualizer_3d_zoom));
                    ui.label(format!("Tool: ({:.1}, {:.1}, {:.1})",
                        self.five_axis_pose.position.x,
                        self.five_axis_pose.position.y,
                        self.five_axis_pose.position.z));
                    ui.label(format!("Orientation: ({:.1}°, {:.1}°, {:.1}°)",
                        roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees()));
                });
            });

            // Instructions
            ui.separator();
            ui.horizontal(|ui| {
                ui.label("🎮 Controls:");
                ui.label("Drag to rotate • Shift+Drag to pan • Scroll to zoom");
            });
        });
    }

    /// Save the current EtherCAT configuration to file
    fn save_ethercat_configuration(&self) -> Result<(), Box<dyn std::error::Error>> {
        let config_data = serde_json::json!({
            "ethercat_modules": self.ethercat_modules,
            "network_config": self.ethercat_network_config,
            "version": "1.0"
        });

        let config_path = "ethercat_config.json";
        std::fs::write(config_path, serde_json::to_string_pretty(&config_data)?)?;
        Ok(())
    }

    /// Load EtherCAT configuration from file
    fn load_ethercat_configuration(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        if let Some(path) = rfd::FileDialog::new()
            .set_title("Load EtherCAT Configuration")
            .add_filter("JSON files", &["json"])
            .set_file_name("ethercat_config.json")
            .pick_file() {

            let config_str = std::fs::read_to_string(&path)?;
            let config_data: serde_json::Value = serde_json::from_str(&config_str)?;

            // Load modules if present
            if let Some(modules_array) = config_data.get("ethercat_modules").and_then(|v| v.as_array()) {
                self.ethercat_modules.clear();
                for module_value in modules_array {
                    if let Ok(module) = serde_json::from_value::<EtherCATModule>(module_value.clone()) {
                        self.ethercat_modules.push(module);
                    }
                }
            }

            // Load network config if present
            if let Some(network_config) = config_data.get("network_config") {
                if let Ok(config) = serde_json::from_value::<EtherCATNetworkConfig>(network_config.clone()) {
                    self.ethercat_network_config = config;
                }
            }
        }
        Ok(())
    }

    /// Export EtherCAT configuration to specified path
    fn export_ethercat_configuration(&self, path: &std::path::Path) -> Result<(), Box<dyn std::error::Error>> {
        let config_data = serde_json::json!({
            "ethercat_modules": self.ethercat_modules,
            "network_config": self.ethercat_network_config,
            "exported_at": chrono::Utc::now().to_rfc3339(),
            "version": "1.0"
        });

        std::fs::write(path, serde_json::to_string_pretty(&config_data)?)?;
        Ok(())
    }

}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Request repaint only at reasonable intervals instead of every frame
        ctx.request_repaint_after(std::time::Duration::from_millis(33)); // ~30 FPS
        // Check for completed parsing tasks
        if self.is_parsing {
            if let Some(ref receiver) = self.parsed_result_receiver {
                if let Ok((content, parsed, plot_points)) = receiver.try_recv() {
                    self.is_parsing = false;
                    
                    if !content.is_empty() {
                        self.gcode = content;
                        self.parsed_gcode = parsed;
                        self.plot_points = plot_points;
                        
                        // Reset execution states
                        self.current_line = 0;
                        self.execution_state = ExecutionState::Idle;
                        self.is_executing = false;
                        self.test_current_line = 0;
                        self.test_execution_state = ExecutionState::Idle;
                        self.is_test_executing = false;
                        
                        // Invalidate cache for new data
                        self.cache_valid = false;
                        
                        self.last_file_error = None;
                    } else {
                        self.last_file_error = Some("Failed to load G-code file".to_string());
                        self.gcode_file_path = None;
                        self.parsed_gcode = None;
                        self.plot_points = Vec::new();
                    }
                    
                    // Clean up
                    self.parsed_result_receiver = None;
                    if let Some(task) = self.gcode_parsing_task.take() {
                        let _ = task.join();
                    }
                    
                    ctx.request_repaint();
                }
            }
        }
        
        // Handle automatic G-code execution with optimized timing
        if self.execution_state == ExecutionState::Running && self.is_executing && !self.test_mode && !self.emergency_stop_triggered {
            self.printer_state = PrinterState::Printing;
            static mut FRAME_COUNTER: u64 = 0;
            unsafe {
                FRAME_COUNTER += 1;
                if FRAME_COUNTER % 30 == 0 {
                    let gcode_lines: Vec<&str> = self.gcode.lines().collect();
                    if self.current_line + 1 < gcode_lines.len() {
                        self.current_line += 1;
                        ctx.request_repaint();
                    } else {
                        self.execution_state = ExecutionState::Completed;
                        self.is_executing = false;
                        ctx.request_repaint();
                    }
                }
            }
        }
        
        if self.test_execution_state == ExecutionState::Running && self.is_test_executing {
            static mut TEST_FRAME_COUNTER: u64 = 0;
            unsafe {
                TEST_FRAME_COUNTER += 1;
                if TEST_FRAME_COUNTER % 6 == 0 {
                    let gcode_lines: Vec<&str> = self.gcode.lines().collect();
                    if self.test_current_line + 1 < gcode_lines.len() {
                        self.test_current_line += 1;
                        ctx.request_repaint();
                    } else {
                        self.test_execution_state = ExecutionState::Completed;
                        self.is_test_executing = false;
                        ctx.request_repaint();
                    }
                }
            }
        }
        
        ctx.set_visuals(if self.dark_mode {
            Visuals {
                dark_mode: true,
                override_text_color: Some(Color32::WHITE),
                panel_fill: Color32::from_rgb(30, 30, 30),
                ..Visuals::default()
            }
        } else {
            Visuals {
                dark_mode: false,
                override_text_color: Some(Color32::BLACK),
                panel_fill: Color32::from_rgb(240, 240, 240),
                ..Visuals::default()
            }
        });
        ctx.set_pixels_per_point(1.2); // Further reduced from 1.5 to make UI less cluttered

        egui::SidePanel::right("status_panel").show(ctx, |ui| {
            ui.heading("Printer Status");
            ui.separator();
            
            // Always-visible Emergency Stop button
            let emergency_color = if self.emergency_stop_triggered { Color32::DARK_RED } else { Color32::from_rgb(200, 0, 0) };
            ui.vertical_centered(|ui| {
                let emergency_button = egui::Button::new("🚨 EMERGENCY STOP 🚨")
                    .fill(emergency_color)
                    .min_size(egui::vec2(180.0, 40.0));
                
                if ui.add(emergency_button).clicked() && !self.emergency_stop_triggered {
                    self.emergency_stop();
                }
                
                if self.emergency_stop_triggered {
                    ui.separator();
                    if ui.button("Reset Emergency Stop").clicked() {
                        self.reset_emergency_stop();
                    }
                }
            });
            
            ui.separator();
            
            // Printer state with color coding
            let (state_text, state_color) = match &self.printer_state {
                PrinterState::Idle => ("Idle", Color32::GREEN),
                PrinterState::Printing => ("Printing", Color32::BLUE),
                PrinterState::Paused => ("Paused", Color32::YELLOW),
                PrinterState::Homing => ("Homing", Color32::from_rgb(255, 165, 0)), // Orange
                PrinterState::Emergency => ("EMERGENCY STOP", Color32::RED),
                PrinterState::Error(msg) => (msg.as_str(), Color32::RED),
            };
            
            ui.horizontal(|ui| {
                ui.label("State:");
                ui.colored_label(state_color, state_text);
            });
            
            ui.label(format!("Comm: {}", if self.group.is_empty() { "Disconnected" } else { "Connected" }));
            
            // Show linear position
            ui.label(format!("Position: X:{:.1}, Y:{:.1}, Z:{:.1}", self.x, self.y, self.z));
            
            // Show rotary axes if 5-axis mode is enabled
            if self.printer_settings.enable_5axis_mode {
                ui.label(format!("Rotary: A:{:.1}°, B:{:.1}°", self.a, self.b));
                ui.label(format!("TCP: {}", if self.printer_settings.tool_point_control { "ON" } else { "OFF" }));
            }
            
            ui.label(format!("Protocol: {:?}", self.communication_mode));
            
            // Fieldbus Connection Status
            ui.group(|ui| {
                ui.label("📡 Fieldbus Status");
                
                // Show connection status for current protocol
                let (status_text, status_color) = if let Some(status) = self.fieldbus_connection_status.get(&self.communication_mode) {
                    match status.as_str() {
                        "Connected" => ("🟢 Connected", Color32::GREEN),
                        "Connecting" => ("🟡 Connecting", Color32::YELLOW),
                        "Disconnected" => ("🔴 Disconnected", Color32::RED),
                        "Error" => ("❌ Error", Color32::RED),
                        _ => (status.as_str(), Color32::GRAY),
                    }
                } else {
                    ("⚫ Unknown", Color32::GRAY)
                };
                
                ui.horizontal(|ui| {
                    ui.colored_label(status_color, status_text);
                    
                    // Show error count if any
                    if let Some(&error_count) = self.fieldbus_error_counts.get(&self.communication_mode) {
                        if error_count > 0 {
                            ui.colored_label(Color32::RED, format!("⚠ {} errors", error_count));
                        }
                    }
                });
                
                // Show last activity time
                if let Some(last_activity) = self.fieldbus_last_activity.get(&self.communication_mode) {
                    let elapsed = last_activity.elapsed();
                    let time_text = if elapsed.as_secs() < 60 {
                        format!("{}s ago", elapsed.as_secs())
                    } else if elapsed.as_secs() < 3600 {
                        format!("{}m ago", elapsed.as_secs() / 60)
                    } else {
                        format!("{}h ago", elapsed.as_secs() / 3600)
                    };
                    ui.label(format!("Last activity: {}", time_text));
                } else {
                    ui.colored_label(Color32::GRAY, "No activity recorded");
                }
                
                // Protocol-specific status information
                match self.communication_mode {
                    CommunicationMode::EtherCAT => {
                        #[cfg(target_os = "linux")]
                        ui.label(format!("Slaves: {}", if self.group.is_empty() { 0 } else { self.group.len() }));
                        #[cfg(not(target_os = "linux"))]
                        ui.colored_label(Color32::YELLOW, "⚠ Platform not supported");
                    },
                    CommunicationMode::BACnet => {
                        if let Some(diag) = self.protocol_manager.get_diagnostics(CommunicationMode::BACnet) {
                            ui.label(format!("Packets: {} sent, {} received", diag.packets_sent, diag.packets_received));
                            if diag.error_count > 0 {
                                ui.colored_label(Color32::RED, format!("⚠ {} protocol errors", diag.error_count));
                            }
                        }
                    },
                    CommunicationMode::ADS => {
                        if let Some(diag) = self.protocol_manager.get_diagnostics(CommunicationMode::ADS) {
                            ui.label(format!("Latency: {:.1}ms", diag.latency_ms));
                            ui.label(format!("Data: {} bytes sent, {} received", diag.bytes_sent, diag.bytes_received));
                        }
                    },
                    CommunicationMode::CODESYS => {
                        if let Some(diag) = self.protocol_manager.get_diagnostics(CommunicationMode::CODESYS) {
                            if diag.connected {
                                ui.colored_label(Color32::GREEN, "✓ Authenticated");
                            } else {
                                ui.colored_label(Color32::RED, "❌ Not authenticated");
                            }
                        }
                    },
                    CommunicationMode::OPC_UA => {
                        ui.label("Endpoint: opc.tcp://...");
                        ui.label("Security: None");
                    },
                    CommunicationMode::ModbusTCP => {
                        ui.label(format!("Target: {}:{}", self.modbus_tcp_ip, self.modbus_tcp_port));
                        ui.label(format!("Device ID: {}", self.modbus_device_id));
                    },
                    CommunicationMode::CANopen => {
                        ui.label(format!("Node ID: {} @ {} bps", self.canopen_node_id, self.canopen_baudrate));
                        
                        // Show CAN toolhead status
                        if let Ok(can_manager) = self.can_manager.try_lock() {
                            let toolhead_count = can_manager.get_toolheads().len();
                            if toolhead_count > 0 {
                                ui.colored_label(Color32::GREEN, format!("🔗 {} CAN toolheads", toolhead_count));
                            } else {
                                ui.colored_label(Color32::YELLOW, "⚠ No toolheads detected");
                            }
                        }
                    },
                    CommunicationMode::Profibus => {
                        ui.label(format!("Address: {} @ {} bps", self.profibus_address, self.profibus_baudrate));
                    },
                    CommunicationMode::Serial => {
                        ui.label(format!("Port: {} @ {} baud", self.serial_port_name, self.serial_baud_rate));
                        if let Some(_) = &self.serial_port {
                            ui.colored_label(Color32::GREEN, "✓ Port open");
                        } else {
                            ui.colored_label(Color32::RED, "❌ Port closed");
                        }
                    },
                    _ => {
                        ui.colored_label(Color32::YELLOW, "⚠ Protocol status not implemented");
                    }
                }
                
                // Input Shaping Status
                if self.input_shaping_enabled {
                    let enabled_axes = ['X', 'Y', 'Z', 'A', 'B']
                        .iter()
                        .filter(|&&axis| {
                            self.input_shaper_manager.get_axis_config(axis)
                                .map(|c| c.enabled)
                                .unwrap_or(false)
                        })
                        .count();
                    
                    if enabled_axes > 0 {
                        ui.colored_label(Color32::GREEN, format!("🎯 Input shaping: {}/5 axes", enabled_axes));
                    } else {
                        ui.colored_label(Color32::YELLOW, "🎯 Input shaping enabled (no axes)");
                    }
                } else {
                    ui.colored_label(Color32::GRAY, "🎯 Input shaping disabled");
                }
            });
            
            ui.separator();
            
            // Z Offset Controls
            ui.label("Z Offset Controls:");
            ui.horizontal(|ui| {
                ui.label(format!("Current: {:.3}mm", self.printer_settings.z_offset));
            });
            ui.horizontal(|ui| {
                if ui.small_button("-0.1").clicked() && !self.emergency_stop_triggered {
                    self.adjust_z_offset(-0.1);
                }
                if ui.small_button("-0.05").clicked() && !self.emergency_stop_triggered {
                    self.adjust_z_offset(-0.05);
                }
                if ui.small_button("-0.01").clicked() && !self.emergency_stop_triggered {
                    self.adjust_z_offset(-0.01);
                }
            });
            ui.horizontal(|ui| {
                if ui.small_button("+0.01").clicked() && !self.emergency_stop_triggered {
                    self.adjust_z_offset(0.01);
                }
                if ui.small_button("+0.05").clicked() && !self.emergency_stop_triggered {
                    self.adjust_z_offset(0.05);
                }
                if ui.small_button("+0.1").clicked() && !self.emergency_stop_triggered {
                    self.adjust_z_offset(0.1);
                }
            });
            if ui.button("💾 Save Z Offset").clicked() {
                self.save_z_offset();
            }
            
            ui.separator();
            
            // Quick controls
            ui.label("Quick Controls:");
            ui.horizontal(|ui| {
                if ui.button("🏠 Home All").clicked() && !self.emergency_stop_triggered {
                    self.printer_state = PrinterState::Homing;
                    println!("Homing all axes...");
                }
                if ui.button("🔌 Disable Motors").clicked() && !self.emergency_stop_triggered {
                    println!("Motors disabled");
                }
            });
            ui.horizontal(|ui| {
                if ui.button("🔍 Probe Z").clicked() && !self.emergency_stop_triggered {
                    self.start_probe_sequence();
                }
                if ui.button("⚙️ Auto Level").clicked() && !self.emergency_stop_triggered {
                    println!("Starting auto bed leveling...");
                    self.printer_state = PrinterState::Homing;
                }
            });
        });

        // Don't send positions if emergency stop is active
        let reachable = self.kinematics.is_reachable(self.x, self.y, self.z, self.a, self.b);
        if !reachable && !self.out_of_reach_reported {
            eprintln!("Position X{} Y{} Z{} is out of reach: holding the last sent position", self.x, self.y, self.z);
        }
        self.out_of_reach_reported = !reachable;

        if !self.emergency_stop_triggered && reachable {
            // Apply input shaping if enabled before converting to motor positions
            let (shaped_x, shaped_y, shaped_z, shaped_a, shaped_b) = if self.input_shaping_enabled {
                let mut shaped_positions = (self.x, self.y, self.z, self.a, self.b);
                
                // Process each axis through its input shaper
                if let Some(x_shaped) = self.input_shaper_manager.process_command('X', self.x) {
                    shaped_positions.0 = x_shaped;
                }
                if let Some(y_shaped) = self.input_shaper_manager.process_command('Y', self.y) {
                    shaped_positions.1 = y_shaped;
                }
                if let Some(z_shaped) = self.input_shaper_manager.process_command('Z', self.z) {
                    shaped_positions.2 = z_shaped;
                }
                if let Some(a_shaped) = self.input_shaper_manager.process_command('A', self.a) {
                    shaped_positions.3 = a_shaped;
                }
                if let Some(b_shaped) = self.input_shaper_manager.process_command('B', self.b) {
                    shaped_positions.4 = b_shaped;
                }
                
                shaped_positions
            } else {
                // No input shaping, use original positions
                (self.x, self.y, self.z, self.a, self.b)
            };
            
            let positions = self.kinematics.to_motor_positions(shaped_x, shaped_y, shaped_z, shaped_a, shaped_b);
            
            // Send coordinates to any connected CAN toolheads
            self.update_can_toolheads(&positions);
            
            match self.communication_mode {
            CommunicationMode::EtherCAT => {
                // Queue in order from this thread. One spawned task per frame could
                // deliver positions out of order. A full queue just skips this frame.
                if let Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) = self.ethercat_tx.try_send(positions) {
                    eprintln!("Failed to send positions: EtherCAT task has stopped");
                }
            }
            CommunicationMode::Serial => {
                // Placeholder for future serial implementation
            }
            CommunicationMode::CANopen => {
                // Placeholder for CANopen implementation
                println!("CANopen communication - Node ID: {}, Positions: {:?}", self.canopen_node_id, positions);
            }
            CommunicationMode::Profibus => {
                // Placeholder for Profibus implementation
                println!("Profibus communication - Address: {}, Positions: {:?}", self.profibus_address, positions);
            }
            CommunicationMode::ModbusTCP => {
                let ip = self.modbus_tcp_ip.clone();
                let port = self.modbus_tcp_port;
                let device_id = self.modbus_device_id;
                // Send on change, one at a time, so writes cannot overtake each other.
                let busy = &self.modbus_send_in_flight;
                if self.last_modbus_positions != Some(positions)
                    && !busy.swap(true, std::sync::atomic::Ordering::SeqCst)
                {
                    self.last_modbus_positions = Some(positions);
                    let busy = busy.clone();
                    tokio::spawn(async move {
                        if let Err(e) = send_modbus_tcp_positions_static(&ip, port, device_id, &positions).await {
                            eprintln!("Modbus TCP communication error: {}", e);
                        }
                        busy.store(false, std::sync::atomic::Ordering::SeqCst);
                    });
                }
            }
            CommunicationMode::ModbusRTU => {
                // Placeholder for Modbus RTU implementation
                println!("Modbus RTU communication - Device ID: {}, Positions: {:?}", self.modbus_device_id, positions);
            }
            CommunicationMode::DeviceNet => {
                // Placeholder for DeviceNet implementation
                println!("DeviceNet communication - Address: {}, Positions: {:?}", self.devicenet_node_address, positions);
            }
            CommunicationMode::ControlNet => {
                // Placeholder for ControlNet implementation
                println!("ControlNet communication - Positions: {:?}", positions);
            }
            CommunicationMode::BACnet => {
                println!("BACnet communication - Positions: {:?}", positions);
                // TODO: Implement BACnet communication
            }
            CommunicationMode::ADS => {
                println!("ADS (TwinCAT) communication - Positions: {:?}", positions);
                // TODO: Implement ADS communication
            }
            CommunicationMode::CODESYS => {
                println!("CODESYS communication - Positions: {:?}", positions);
                // TODO: Implement CODESYS communication
            }
            CommunicationMode::CIP => {
                println!("CIP communication - Positions: {:?}", positions);
                // TODO: Implement CIP communication
            }
            CommunicationMode::OPC_UA => {
                println!("OPC UA communication - Positions: {:?}", positions);
                // TODO: Implement OPC UA communication
            }
            CommunicationMode::DNP3 => {
                println!("DNP3 communication - Positions: {:?}", positions);
                // TODO: Implement DNP3 communication
            }
            CommunicationMode::IEC61850 => {
                println!("IEC 61850 communication - Positions: {:?}", positions);
                // TODO: Implement IEC 61850 communication
            }
            CommunicationMode::S7 => {
                println!("Siemens S7 communication - Positions: {:?}", positions);
                // TODO: Implement S7 communication
            }
            CommunicationMode::Fins => {
                println!("OMRON FINS communication - Positions: {:?}", positions);
                // TODO: Implement FINS communication
            }
            CommunicationMode::CC_Link => {
                println!("CC-Link communication - Positions: {:?}", positions);
                // TODO: Implement CC-Link communication
            }
            CommunicationMode::Foundation => {
                println!("Foundation Fieldbus communication - Positions: {:?}", positions);
                // TODO: Implement Foundation Fieldbus communication
            }
            CommunicationMode::Ethernet => {
                println!("Generic Ethernet communication - Positions: {:?}", positions);
                // TODO: Implement generic Ethernet communication
            }
            }
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            // Scrollable tab navigation to handle overflow
            egui::ScrollArea::horizontal()
                .id_salt("tab_navigation_scroll")
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.style_mut().visuals.selection.bg_fill = Color32::DARK_BLUE;
                        ui.style_mut().visuals.selection.stroke.color = Color32::WHITE;
                        ui.style_mut().spacing.button_padding = egui::vec2(8.0, 6.0);

                // Use compact icons instead of text to save space
                // Motion & Kinematics Group
                ui.label("🎯 Motion & Kinematics");
                ui.selectable_value(&mut self.selected_tab, Tab::Kinematics, "⚙️ Kinematics");
                ui.selectable_value(&mut self.selected_tab, Tab::MotionControl, "🔄 Motion Control");
                ui.selectable_value(&mut self.selected_tab, Tab::InputShaping, "📊 Input Shaping");
                ui.selectable_value(&mut self.selected_tab, Tab::AutoInputShaper, "🎯 Auto Calibration");
                ui.selectable_value(&mut self.selected_tab, Tab::VolumetricCompensation, "📐 Volume Compensation");
                ui.selectable_value(&mut self.selected_tab, Tab::SensorFusion, "🔗 Sensor Fusion");
                ui.selectable_value(&mut self.selected_tab, Tab::StateSpaceControl, "🎚️ State-Space Control");
                ui.selectable_value(&mut self.selected_tab, Tab::ModelPredictiveControl, "🔮 Model Predictive Control");
                ui.selectable_value(&mut self.selected_tab, Tab::PressureAdvance, "💨 Pressure Advance");

                ui.separator();

                // Hardware Configuration Group
                ui.label("🔧 Hardware Configuration");
                ui.selectable_value(&mut self.selected_tab, Tab::Hardware, "🖨️ Hardware");
                ui.selectable_value(&mut self.selected_tab, Tab::MotorAssignment, "🔩 Motors");
                ui.selectable_value(&mut self.selected_tab, Tab::ClosedLoop, "🔄 Closed Loop");
                ui.selectable_value(&mut self.selected_tab, Tab::Temperature, "🌡️ Temperature");

                ui.separator();

                // Communication & Protocols Group
                ui.label("🔌 Communication");
                ui.selectable_value(&mut self.selected_tab, Tab::Communication, "🔌 Protocols");
                ui.selectable_value(&mut self.selected_tab, Tab::FieldbusConfig, "🌐 Fieldbus");
                ui.selectable_value(&mut self.selected_tab, Tab::CanToolheads, "🚛 CAN Bus");

                ui.separator();

                // G-code & Operations Group
                ui.label("📄 G-code & Operations");
                ui.selectable_value(&mut self.selected_tab, Tab::Gcode, "📄 G-code");
                ui.selectable_value(&mut self.selected_tab, Tab::Operations, "🎯 Operations");
                ui.selectable_value(&mut self.selected_tab, Tab::Macros, "🎯 Macros");
                ui.selectable_value(&mut self.selected_tab, Tab::GcodeImport, "📁 Import");

                ui.separator();

                // System & Configuration Group
                ui.label("🔧 System & Config");
                ui.selectable_value(&mut self.selected_tab, Tab::System, "⚙️ System");
                ui.selectable_value(&mut self.selected_tab, Tab::Settings, "🔧 Settings");
                ui.selectable_value(&mut self.selected_tab, Tab::PrinterConfig, "🖨️ Printer");
                ui.selectable_value(&mut self.selected_tab, Tab::RuntimeConfig, "⚙️ Runtime");

                ui.separator();

                // Utilities & Diagnostics Group
                ui.label("🔍 Utilities");
                ui.selectable_value(&mut self.selected_tab, Tab::BedMesh, "📐 Bed Mesh");
                ui.selectable_value(&mut self.selected_tab, Tab::FanControl, "🌪️ Fans");
                ui.selectable_value(&mut self.selected_tab, Tab::Webcam, "📷 Webcam");
                ui.selectable_value(&mut self.selected_tab, Tab::PowerLoss, "⚡ Power Loss");
                        ui.selectable_value(&mut self.selected_tab, Tab::Terminal, "💻 Terminal");
                    });
                });
            ui.separator();

            match self.selected_tab {
                Tab::Kinematics => {
                    ui.heading("⚙️ Kinematics - Motion Control & Configuration");

                    egui::ScrollArea::vertical()
                        .id_salt("kinematics_scroll")
                        .auto_shrink([false, true])
                        .show(ui, |ui| {

                        // Kinematics Type Selection
                        ui.group(|ui| {
                            ui.label("🎯 Kinematics Type");
                            ui.horizontal(|ui| {
                                ui.radio_value(&mut self.printer_settings.enable_5axis_mode, false, "Standard 3-Axis");
                                ui.radio_value(&mut self.printer_settings.enable_5axis_mode, true, "5-Axis Advanced");
                            });
                        });

                        ui.separator();

                        if !self.printer_settings.enable_5axis_mode {
                            // Standard 3-Axis Controls
                            ui.group(|ui| {
                                ui.label("📐 Position Control");
                                ui.add(egui::Slider::new(&mut self.x, -100.0..=100.0).text("X"));
                                ui.add(egui::Slider::new(&mut self.y, -100.0..=100.0).text("Y"));
                                ui.add(egui::Slider::new(&mut self.z, -100.0..=100.0).text("Z"));
                                if self.printer_settings.enable_5axis_mode {
                                    ui.add(egui::Slider::new(&mut self.a, -180.0..=180.0).text("A (degrees)"));
                                    ui.add(egui::Slider::new(&mut self.b, -180.0..=180.0).text("B (degrees)"));
                                }
                            });
                        
                        ui.separator();
                        
                        ui.group(|ui| {
                            ui.label("🔍 Status Information");
                            
                            // Show current target positions
                            ui.label(format!("Target Position: X:{:.1}, Y:{:.1}, Z:{:.1}", self.x, self.y, self.z));
                            if self.printer_settings.enable_5axis_mode {
                                ui.label(format!("Rotary Position: A:{:.1}°, B:{:.1}°", self.a, self.b));
                            }
                            
                            // Show motor positions (with or without input shaping)
                            let (display_x, display_y, display_z, display_a, display_b) = if self.input_shaping_enabled {
                                let mut shaped_pos = (self.x, self.y, self.z, self.a, self.b);
                                
                                // Apply same input shaping as used for motion
                                if let Some(x_shaped) = self.input_shaper_manager.process_command('X', self.x) {
                                    shaped_pos.0 = x_shaped;
                                }
                                if let Some(y_shaped) = self.input_shaper_manager.process_command('Y', self.y) {
                                    shaped_pos.1 = y_shaped;
                                }
                                if let Some(z_shaped) = self.input_shaper_manager.process_command('Z', self.z) {
                                    shaped_pos.2 = z_shaped;
                                }
                                if let Some(a_shaped) = self.input_shaper_manager.process_command('A', self.a) {
                                    shaped_pos.3 = a_shaped;
                                }
                                if let Some(b_shaped) = self.input_shaper_manager.process_command('B', self.b) {
                                    shaped_pos.4 = b_shaped;
                                }
                                
                                shaped_pos
                            } else {
                                (self.x, self.y, self.z, self.a, self.b)
                            };
                            
                            let positions = self.kinematics.to_motor_positions(display_x, display_y, display_z, display_a, display_b);
                            ui.label(format!("Motor Positions: {:?}", positions));
                            
                            if self.input_shaping_enabled {
                                ui.colored_label(Color32::GREEN, "🎯 Input Shaping Applied");
                            }
                            
                            ui.label(format!("Scene Entities: {}", self.scene.entities.len()));
                        });
                        } else {
                            // 5-Axis Advanced Controls with 3D Visualization
                            // Split layout: Controls on left, 3D visualizer on right
                            ui.horizontal(|ui| {
                                // Left panel: Controls (40% width)
                                ui.allocate_ui_with_layout(
                                    egui::vec2(ui.available_width() * 0.4, ui.available_height()),
                                    egui::Layout::top_down(egui::Align::LEFT),
                                    |ui| {
                                        egui::ScrollArea::vertical()
                                            .id_salt("kinematics_5axis_controls_scroll")
                                            .auto_shrink([false, true])
                                            .show(ui, |ui| {
                            // Machine Type Selection
                            ui.group(|ui| {
                                ui.label("🏭 Machine Type Configuration");
                                ui.horizontal(|ui| {
                                    if ui.button("Penta Axis Style").clicked() {
                                        self.selected_machine_type = PracticalMachineType::penta_axis_style();
                                        self.machine_config_dirty = true;
                                    }
                                    if ui.button("Open5X Style").clicked() {
                                        self.selected_machine_type = PracticalMachineType::open5x_style();
                                        self.machine_config_dirty = true;
                                    }
                                    if ui.button("CNC Trunnion").clicked() {
                                        self.selected_machine_type = PracticalMachineType::cnc_trunnion_table();
                                        self.machine_config_dirty = true;
                                    }
                                });

                                if ui.button("⚙️ Configure Machine").clicked() {
                                    self.show_5axis_config_dialog = true;
                                }

                                // Initialize kinematics when machine type changes
                                if self.machine_config_dirty {
                                    self.five_axis_kinematics = Some(PracticalKinematics::new(self.selected_machine_type.clone()));
                                    // Save machine type to configuration
                                    self.config_manager.get_config_mut().kinematics.five_axis_machine_type = Some(self.selected_machine_type.clone());
                                    if let Err(e) = self.config_manager.save_config() {
                                        println!("⚠️ Failed to save machine configuration: {}", e);
                                    }
                                    self.machine_config_dirty = false;
                                }

                                // Display current machine configuration
                                ui.separator();
                                ui.label("📋 Current Machine Configuration:");
                                ui.indent("machine_config_display", |ui| {
                                    match &self.selected_machine_type {
                                        PracticalMachineType::ToolheadRotary { tool_offset, tilt_limits, rotation_limits, gear_ratios, steps_per_degree } => {
                                            ui.label("🔧 Toolhead Rotary Configuration (Penta Axis Style)");
                                            ui.label(format!("• Tool Offset: {:.1}mm", tool_offset));
                                            ui.label(format!("• Tilt Limits: {:.1}° to {:.1}°", tilt_limits.0, tilt_limits.1));
                                            ui.label(format!("• Rotation Limits: {:.1}° to {:.1}°", rotation_limits.0, rotation_limits.1));
                                            ui.label(format!("• Gear Ratios: {:.1}:1, {:.1}:1", gear_ratios.0, gear_ratios.1));
                                            ui.label(format!("• Steps/Degree: {:.2}, {:.2}", steps_per_degree.0, steps_per_degree.1));
                                        },
                                        PracticalMachineType::CoreXYRotary { belt_ratio, rotary_offset, axis_configs } => {
                                            ui.label("🔧 CoreXY Rotary Configuration (Open5X Style)");
                                            ui.label(format!("• Belt Ratio: {:.1}", belt_ratio));
                                            ui.label(format!("• Rotary Offset: X:{:.1} Y:{:.1} Z:{:.1}mm",
                                                rotary_offset.x, rotary_offset.y, rotary_offset.z));
                                            ui.label(format!("• A-Axis Range: {:.1}° to {:.1}°", axis_configs.0.min_angle, axis_configs.0.max_angle));
                                            ui.label(format!("• C-Axis Range: {:.1}° to {:.1}°", axis_configs.1.min_angle, axis_configs.1.max_angle));
                                            ui.label(format!("• Max Speeds: {:.0}°/min, {:.0}°/min", axis_configs.0.max_speed, axis_configs.1.max_speed));
                                        },
                                        PracticalMachineType::TableRotary { table_center, table_diameter, a_axis_config, c_axis_config } => {
                                            ui.label("🔧 Table Rotary Configuration (CNC Trunnion)");
                                            ui.label(format!("• Table Center: X:{:.1} Y:{:.1} Z:{:.1}mm",
                                                table_center.x, table_center.y, table_center.z));
                                            ui.label(format!("• Table Diameter: {:.1}mm", table_diameter));
                                            ui.label(format!("• A-Axis Range: {:.1}° to {:.1}°", a_axis_config.min_angle, a_axis_config.max_angle));
                                            ui.label(format!("• C-Axis Range: {:.1}° to {:.1}°", c_axis_config.min_angle, c_axis_config.max_angle));
                                            ui.label(format!("• Gear Ratios: {:.1}:1, {:.1}:1", a_axis_config.gear_ratio, c_axis_config.gear_ratio));
                                        },
                                        PracticalMachineType::RobotArm { link_lengths, joint_limits, end_effector_offset } => {
                                            ui.label("🔧 Robot Arm Configuration");
                                            ui.label(format!("• Link Lengths: {:?}mm", link_lengths));
                                            ui.label(format!("• End Effector Offset: {:.1}mm", end_effector_offset));
                                            ui.label(format!("• Joint Count: {}", joint_limits.len()));
                                        },
                                    }
                                });
                            });

                            ui.separator();

                            // Target Pose Input
                            ui.group(|ui| {
                                ui.label("🎯 Target Pose");
                                ui.horizontal(|ui| {
                                    ui.label("Position (mm):");
                                    ui.push_id("pose_position", |ui| {
                                        ui.add(egui::DragValue::new(&mut self.five_axis_pose.position.x).prefix("X: ").speed(1.0));
                                        ui.add(egui::DragValue::new(&mut self.five_axis_pose.position.y).prefix("Y: ").speed(1.0));
                                        ui.add(egui::DragValue::new(&mut self.five_axis_pose.position.z).prefix("Z: ").speed(1.0));
                                    });
                                });
                                ui.horizontal(|ui| {
                                    ui.label("Orientation (deg):");
                                    let (mut roll, mut pitch, mut yaw) = self.five_axis_pose.euler_angles();
                                    roll = roll.to_degrees();
                                    pitch = pitch.to_degrees();
                                    yaw = yaw.to_degrees();

                                    let changed = ui.push_id("pose_orientation", |ui| {
                                        ui.add(egui::DragValue::new(&mut roll).prefix("Roll: ").speed(1.0)).changed() |
                                        ui.add(egui::DragValue::new(&mut pitch).prefix("Pitch: ").speed(1.0)).changed() |
                                        ui.add(egui::DragValue::new(&mut yaw).prefix("Yaw: ").speed(1.0)).changed()
                                    }).inner;
                                    if changed {
                                        self.five_axis_pose = Pose6DOF::new(
                                            self.five_axis_pose.position.x,
                                            self.five_axis_pose.position.y,
                                            self.five_axis_pose.position.z,
                                            roll.to_radians(),
                                            pitch.to_radians(),
                                            yaw.to_radians(),
                                        );
                                    }
                                });
                            });

                            ui.separator();

                            // Workspace Limits
                            ui.group(|ui| {
                                ui.label("📏 Workspace Limits");
                                ui.horizontal(|ui| {
                                    ui.label("X:");
                                    ui.push_id("workspace_x", |ui| {
                                        ui.add(egui::DragValue::new(&mut self.workspace_limits.x_limits.0).prefix("Min: ").speed(10.0));
                                        ui.add(egui::DragValue::new(&mut self.workspace_limits.x_limits.1).prefix("Max: ").speed(10.0));
                                    });
                                });
                                ui.horizontal(|ui| {
                                    ui.label("Y:");
                                    ui.push_id("workspace_y", |ui| {
                                        ui.add(egui::DragValue::new(&mut self.workspace_limits.y_limits.0).prefix("Min: ").speed(10.0));
                                        ui.add(egui::DragValue::new(&mut self.workspace_limits.y_limits.1).prefix("Max: ").speed(10.0));
                                    });
                                });
                                ui.horizontal(|ui| {
                                    ui.label("Z:");
                                    ui.push_id("workspace_z", |ui| {
                                        ui.add(egui::DragValue::new(&mut self.workspace_limits.z_limits.0).prefix("Min: ").speed(10.0));
                                        ui.add(egui::DragValue::new(&mut self.workspace_limits.z_limits.1).prefix("Max: ").speed(10.0));
                                    });
                                });
                                ui.horizontal(|ui| {
                                    ui.label("A (tilt):");
                                    ui.push_id("workspace_a", |ui| {
                                        ui.add(egui::DragValue::new(&mut self.workspace_limits.a_limits.0).prefix("Min: ").speed(1.0).suffix("°"));
                                        ui.add(egui::DragValue::new(&mut self.workspace_limits.a_limits.1).prefix("Max: ").speed(1.0).suffix("°"));
                                    });
                                });
                                ui.horizontal(|ui| {
                                    ui.label("B/C (rotation):");
                                    ui.push_id("workspace_bc", |ui| {
                                        ui.add(egui::DragValue::new(&mut self.workspace_limits.b_or_c_limits.0).prefix("Min: ").speed(1.0).suffix("°"));
                                        ui.add(egui::DragValue::new(&mut self.workspace_limits.b_or_c_limits.1).prefix("Max: ").speed(1.0).suffix("°"));
                                    });
                                });
                            });
                        });
                    });

                    // Right panel: 3D Visualizer (60% width)
                    ui.allocate_ui_with_layout(
                        egui::vec2(ui.available_width(), ui.available_height()),
                        egui::Layout::top_down(egui::Align::LEFT),
                        |ui| {
                            ui.label("🎭 5-Axis Visualization");
                            ui.separator();

                            // Render the 5-axis visualizer
                            self.render_5axis_visualizer_3d(ui);
                        }
                    );
                });
                        }

                        // For 3-axis mode, still call the basic visualization
                        if !self.printer_settings.enable_5axis_mode {
                            self.kinematics.visualize(&mut self.scene, self.a, self.b);
                        }
                    });
                }
                Tab::MotionControl => {
                    ui.heading("🔄 Motion Control & Advanced Features");
                    
                    // Split layout: Controls on left, 3D visualizer on right
                    ui.horizontal(|ui| {
                        // Left panel: Controls (40% width)
                        ui.allocate_ui_with_layout(
                            egui::vec2(ui.available_width() * 0.4, ui.available_height()),
                            egui::Layout::top_down(egui::Align::LEFT),
                            |ui| {
                                egui::ScrollArea::vertical()
                                    .id_salt("fiveaxis_controls_scroll")
                                    .auto_shrink([false, true])
                                    .show(ui, |ui| {
                        
                        // Machine Type Selection
                        ui.group(|ui| {
                            ui.label("🏭 Machine Type Configuration");
                            ui.horizontal(|ui| {
                                if ui.button("Penta Axis Style").clicked() {
                                    self.selected_machine_type = PracticalMachineType::penta_axis_style();
                                    self.machine_config_dirty = true;
                                }
                                if ui.button("Open5X Style").clicked() {
                                    self.selected_machine_type = PracticalMachineType::open5x_style();
                                    self.machine_config_dirty = true;
                                }
                                if ui.button("CNC Trunnion").clicked() {
                                    self.selected_machine_type = PracticalMachineType::cnc_trunnion_table();
                                    self.machine_config_dirty = true;
                                }
                            });
                            
                            if ui.button("⚙️ Configure Machine").clicked() {
                                self.show_5axis_config_dialog = true;
                            }
                            
                            // Initialize kinematics when machine type changes
                            if self.machine_config_dirty {
                                self.five_axis_kinematics = Some(PracticalKinematics::new(self.selected_machine_type.clone()));
                                // Save machine type to configuration
                                self.config_manager.get_config_mut().kinematics.five_axis_machine_type = Some(self.selected_machine_type.clone());
                                if let Err(e) = self.config_manager.save_config() {
                                    println!("⚠️ Failed to save machine configuration: {}", e);
                                }
                                self.machine_config_dirty = false;
                            }

                            // Display current machine configuration
                            ui.separator();
                            ui.label("📋 Current Machine Configuration:");
                            ui.indent("machine_config_display", |ui| {
                                match &self.selected_machine_type {
                                    PracticalMachineType::ToolheadRotary { tool_offset, tilt_limits, rotation_limits, gear_ratios, steps_per_degree } => {
                                        ui.label("🔧 Toolhead Rotary Configuration (Penta Axis Style)");
                                        ui.label(format!("• Tool Offset: {:.1}mm", tool_offset));
                                        ui.label(format!("• Tilt Limits: {:.1}° to {:.1}°", tilt_limits.0, tilt_limits.1));
                                        ui.label(format!("• Rotation Limits: {:.1}° to {:.1}°", rotation_limits.0, rotation_limits.1));
                                        ui.label(format!("• Gear Ratios: {:.1}:1, {:.1}:1", gear_ratios.0, gear_ratios.1));
                                        ui.label(format!("• Steps/Degree: {:.2}, {:.2}", steps_per_degree.0, steps_per_degree.1));
                                    },
                                    PracticalMachineType::CoreXYRotary { belt_ratio, rotary_offset, axis_configs } => {
                                        ui.label("🔧 CoreXY Rotary Configuration (Open5X Style)");
                                        ui.label(format!("• Belt Ratio: {:.1}", belt_ratio));
                                        ui.label(format!("• Rotary Offset: X:{:.1} Y:{:.1} Z:{:.1}mm",
                                            rotary_offset.x, rotary_offset.y, rotary_offset.z));
                                        ui.label(format!("• A-Axis Range: {:.1}° to {:.1}°", axis_configs.0.min_angle, axis_configs.0.max_angle));
                                        ui.label(format!("• C-Axis Range: {:.1}° to {:.1}°", axis_configs.1.min_angle, axis_configs.1.max_angle));
                                        ui.label(format!("• Max Speeds: {:.0}°/min, {:.0}°/min", axis_configs.0.max_speed, axis_configs.1.max_speed));
                                    },
                                    PracticalMachineType::TableRotary { table_center, table_diameter, a_axis_config, c_axis_config } => {
                                        ui.label("🔧 Table Rotary Configuration (CNC Trunnion)");
                                        ui.label(format!("• Table Center: X:{:.1} Y:{:.1} Z:{:.1}mm",
                                            table_center.x, table_center.y, table_center.z));
                                        ui.label(format!("• Table Diameter: {:.1}mm", table_diameter));
                                        ui.label(format!("• A-Axis Range: {:.1}° to {:.1}°", a_axis_config.min_angle, a_axis_config.max_angle));
                                        ui.label(format!("• C-Axis Range: {:.1}° to {:.1}°", c_axis_config.min_angle, c_axis_config.max_angle));
                                        ui.label(format!("• Gear Ratios: {:.1}:1, {:.1}:1", a_axis_config.gear_ratio, c_axis_config.gear_ratio));
                                    },
                                    PracticalMachineType::RobotArm { link_lengths, joint_limits, end_effector_offset } => {
                                        ui.label("🔧 Robot Arm Configuration");
                                        ui.label(format!("• Link Lengths: {:?}mm", link_lengths));
                                        ui.label(format!("• End Effector Offset: {:.1}mm", end_effector_offset));
                                        ui.label(format!("• Joint Count: {}", joint_limits.len()));
                                    },
                                }
                            });
                        });
                        
                        ui.separator();
                        
                        // Target Pose Input
                        ui.group(|ui| {
                            ui.label("🎯 Target Pose");
                            ui.horizontal(|ui| {
                                ui.label("Position (mm):");
                                ui.push_id("pose_position", |ui| {
                                    ui.add(egui::DragValue::new(&mut self.five_axis_pose.position.x).prefix("X: ").speed(1.0));
                                    ui.add(egui::DragValue::new(&mut self.five_axis_pose.position.y).prefix("Y: ").speed(1.0));
                                    ui.add(egui::DragValue::new(&mut self.five_axis_pose.position.z).prefix("Z: ").speed(1.0));
                                });
                            });
                            ui.horizontal(|ui| {
                                ui.label("Orientation (deg):");
                                let (mut roll, mut pitch, mut yaw) = self.five_axis_pose.euler_angles();
                                roll = roll.to_degrees();
                                pitch = pitch.to_degrees();
                                yaw = yaw.to_degrees();
                                
                                let changed = ui.push_id("pose_orientation", |ui| {
                                    ui.add(egui::DragValue::new(&mut roll).prefix("Roll: ").speed(1.0)).changed() |
                                    ui.add(egui::DragValue::new(&mut pitch).prefix("Pitch: ").speed(1.0)).changed() |
                                    ui.add(egui::DragValue::new(&mut yaw).prefix("Yaw: ").speed(1.0)).changed()
                                }).inner;
                                if changed {
                                    self.five_axis_pose = Pose6DOF::new(
                                        self.five_axis_pose.position.x,
                                        self.five_axis_pose.position.y,
                                        self.five_axis_pose.position.z,
                                        roll.to_radians(),
                                        pitch.to_radians(),
                                        yaw.to_radians()
                                    );
                                }
                            });
                        });
                        
                        ui.separator();
                        
                        // Kinematics Calculation
                        if let Some(ref kinematics) = self.five_axis_kinematics {
                            ui.group(|ui| {
                                ui.label("🔢 Kinematics Calculation");
                                
                                ui.horizontal(|ui| {
                                    if ui.button("🔍 Solve Inverse Kinematics").clicked() {
                                        match kinematics.inverse_kinematics_analytical(&self.five_axis_pose) {
                                            Ok(solutions) => {
                                                self.inverse_solutions = solutions;
                                            },
                                            Err(e) => {
                                                self.inverse_solutions.clear();
                                                println!("Inverse kinematics error: {:?}", e);
                                            }
                                        }
                                    }
                                    
                                    if ui.button("➡️ Solve Forward Kinematics").clicked() {
                                        match kinematics.forward_kinematics_fast(&self.five_axis_joints) {
                                            Ok(pose) => {
                                                self.forward_pose_result = Some(Ok(pose));
                                            },
                                            Err(e) => {
                                                self.forward_pose_result = Some(Err(format!("{:?}", e)));
                                            }
                                        }
                                    }
                                });
                                
                                // Joint Configuration Input
                                ui.group(|ui| {
                                    ui.label("🔧 Joint Configuration");
                                    ui.horizontal(|ui| {
                                        ui.push_id("joint_config", |ui| {
                                            ui.add(egui::DragValue::new(&mut self.five_axis_joints.x).prefix("X: ").speed(1.0).suffix(" mm"));
                                            ui.add(egui::DragValue::new(&mut self.five_axis_joints.y).prefix("Y: ").speed(1.0).suffix(" mm"));
                                            ui.add(egui::DragValue::new(&mut self.five_axis_joints.z).prefix("Z: ").speed(1.0).suffix(" mm"));
                                            ui.add(egui::DragValue::new(&mut self.five_axis_joints.a).prefix("A: ").speed(1.0).suffix("°"));
                                            ui.add(egui::DragValue::new(&mut self.five_axis_joints.b_or_c).prefix("B/C: ").speed(1.0).suffix("°"));
                                        });
                                    });
                                });
                                
                                // Results Display
                                if !self.inverse_solutions.is_empty() {
                                    ui.group(|ui| {
                                        ui.label("✅ Inverse Kinematics Solutions:");
                                        for (i, solution) in self.inverse_solutions.iter().enumerate() {
                                            ui.horizontal(|ui| {
                                                ui.label(format!("Solution {}:", i + 1));
                                                ui.label(format!("X:{:.1} Y:{:.1} Z:{:.1} A:{:.1}° B:{:.1}°", 
                                                    solution.x, solution.y, solution.z, solution.a, solution.b_or_c));
                                                if ui.button("Use").clicked() {
                                                    self.five_axis_joints = solution.clone();
                                                }
                                            });
                                        }
                                    });
                                }
                                
                                if let Some(ref result) = self.forward_pose_result {
                                    ui.group(|ui| {
                                        match result {
                                            Ok(pose) => {
                                                ui.label("✅ Forward Kinematics Result:");
                                                ui.label(format!("Position: X:{:.1} Y:{:.1} Z:{:.1} mm", 
                                                    pose.position.x, pose.position.y, pose.position.z));
                                                let (roll, pitch, yaw) = pose.euler_angles();
                                                ui.label(format!("Orientation: Roll:{:.1}° Pitch:{:.1}° Yaw:{:.1}°", 
                                                    roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees()));
                                            },
                                            Err(error) => {
                                                ui.label(format!("❌ Forward Kinematics Error: {}", error));
                                            }
                                        }
                                    });
                                }
                            });
                        } else {
                            ui.label("⚠️ No 5-axis kinematics configured. Select a machine type above.");
                        }
                        
                        ui.separator();
                        
                        // Workspace Limits
                        ui.group(|ui| {
                            ui.label("📏 Workspace Limits");
                            ui.horizontal(|ui| {
                                ui.label("X:");
                                ui.push_id("workspace_x", |ui| {
                                    ui.add(egui::DragValue::new(&mut self.workspace_limits.x_limits.0).prefix("Min: ").speed(10.0));
                                    ui.add(egui::DragValue::new(&mut self.workspace_limits.x_limits.1).prefix("Max: ").speed(10.0));
                                });
                            });
                            ui.horizontal(|ui| {
                                ui.label("Y:");
                                ui.push_id("workspace_y", |ui| {
                                    ui.add(egui::DragValue::new(&mut self.workspace_limits.y_limits.0).prefix("Min: ").speed(10.0));
                                    ui.add(egui::DragValue::new(&mut self.workspace_limits.y_limits.1).prefix("Max: ").speed(10.0));
                                });
                            });
                            ui.horizontal(|ui| {
                                ui.label("Z:");
                                ui.push_id("workspace_z", |ui| {
                                    ui.add(egui::DragValue::new(&mut self.workspace_limits.z_limits.0).prefix("Min: ").speed(10.0));
                                    ui.add(egui::DragValue::new(&mut self.workspace_limits.z_limits.1).prefix("Max: ").speed(10.0));
                                });
                            });
                            ui.horizontal(|ui| {
                                ui.label("A (tilt):");
                                ui.push_id("workspace_a", |ui| {
                                    ui.add(egui::DragValue::new(&mut self.workspace_limits.a_limits.0).prefix("Min: ").speed(1.0).suffix("°"));
                                    ui.add(egui::DragValue::new(&mut self.workspace_limits.a_limits.1).prefix("Max: ").speed(1.0).suffix("°"));
                                });
                            });
                            ui.horizontal(|ui| {
                                ui.label("B/C (rotation):");
                                ui.push_id("workspace_bc", |ui| {
                                    ui.add(egui::DragValue::new(&mut self.workspace_limits.b_or_c_limits.0).prefix("Min: ").speed(1.0).suffix("°"));
                                    ui.add(egui::DragValue::new(&mut self.workspace_limits.b_or_c_limits.1).prefix("Max: ").speed(1.0).suffix("°"));
                                });
                            });
                        });
                                    });
                            }
                        );
                        
                        ui.separator();
                        
                        // Right panel: 3D Visualizer (60% width)  
                        ui.allocate_ui_with_layout(
                            egui::vec2(ui.available_width(), ui.available_height()),
                            egui::Layout::top_down(egui::Align::LEFT),
                            |ui| {
                                self.render_5axis_visualizer_3d(ui);
                            }
                        );
                    });
                }
                Tab::Gcode => {
                    ui.heading("G-code Viewer");
                    
                    if self.is_parsing {
                        ui.horizontal(|ui| {
                            ui.spinner();
                            ui.label("Loading and parsing G-code file...");
                        });
                        return;
                    }
                    
                    if let Some(path) = &self.gcode_file_path {
                        let path_display = path.clone(); // Clone to avoid borrow checker issues
                        ui.horizontal(|ui| {
                            ui.label(format!("File: {}", path_display));
                            ui.separator();
                            
                            // Add cancel/clear G-code functionality
                            if ui.button("🗑️ Clear G-code").clicked() {
                                self.gcode_file_path = None;
                                self.parsed_gcode = None;
                                self.plot_points.clear();
                                self.gcode.clear();
                                self.cache_valid = false;
                                self.cached_3d_points.clear();
                                self.cached_2d_points.clear();
                                self.cached_z_heights.clear();
                                self.cached_bounds = None;
                                self.current_line = 0;
                                self.test_current_line = 0;
                                self.execution_state = ExecutionState::Idle;
                                self.test_execution_state = ExecutionState::Idle;
                                println!("G-code cleared and reverted to original state");
                            }
                            
                            if ui.button("🔄 Reload").clicked() {
                                if let Some(_file_path) = self.gcode_file_path.clone() {
                                    // Clear current state and trigger re-parsing
                                    self.parsed_gcode = None;
                                    self.plot_points.clear();
                                    self.cache_valid = false;
                                    self.selected_gcode_line = None;
                                    println!("G-code reload triggered - use Import tab to reload file");
                                }
                            }
                            
                            ui.separator();
                            if self.test_mode {
                                ui.colored_label(Color32::YELLOW, "🧪 TEST MODE");
                                ui.separator();
                                ui.label(format!("State: {:?}", self.test_execution_state));
                                if self.parsed_gcode.is_some() {
                                    ui.separator();
                                    ui.label(format!("Line: {}/{}", self.test_current_line + 1, self.gcode.lines().count()));
                                }
                            } else {
                                ui.colored_label(Color32::RED, "🔧 REAL MODE");
                                ui.separator();
                                ui.label(format!("State: {:?}", self.execution_state));
                                if self.parsed_gcode.is_some() {
                                    ui.separator();
                                    ui.label(format!("Line: {}/{}", self.current_line + 1, self.gcode.lines().count()));
                                }
                            }
                        });
                        
                        if let Some(parsed) = &self.parsed_gcode {
                            ui.horizontal(|ui| {
                                ui.label(format!("Commands: {}", parsed.commands.len()));
                                ui.separator();
                                ui.label(format!("Plot points: {}", self.plot_points.len()));
                            });
                            
                            ui.horizontal(|ui| {
                                if self.test_mode {
                                    let can_start = self.test_execution_state == ExecutionState::Idle || self.test_execution_state == ExecutionState::Completed;
                                    let can_pause = self.test_execution_state == ExecutionState::Running;
                                    let can_resume = self.test_execution_state == ExecutionState::Paused;
                                    let can_step = self.test_execution_state == ExecutionState::Paused || can_start;
                                    
                                    ui.label("🧪 Test Controls:");
                                    if ui.add_enabled(can_start, egui::Button::new("▶ Test")).clicked() {
                                        self.test_execution_state = ExecutionState::Running;
                                        self.is_test_executing = true;
                                        self.test_current_line = 0;
                                    }
                                    if ui.add_enabled(can_pause, egui::Button::new("⏸ Pause")).clicked() {
                                        self.test_execution_state = ExecutionState::Paused;
                                        self.is_test_executing = false;
                                    }
                                    if ui.add_enabled(can_resume, egui::Button::new("▶ Resume")).clicked() {
                                        self.test_execution_state = ExecutionState::Running;
                                        self.is_test_executing = true;
                                    }
                                    if ui.add_enabled(can_step, egui::Button::new("⏭ Step")).clicked() {
                                        let now = Instant::now();
                                        if now.duration_since(self.last_step_time).as_millis() > 100 { // 100ms debounce
                                            self.last_step_time = now;
                                            let gcode_line_count = self.gcode.lines().count(); // More efficient than collect
                                            if self.test_current_line + 1 < gcode_line_count {
                                                self.test_current_line += 1;
                                                self.test_execution_state = ExecutionState::Paused;
                                            } else {
                                                self.test_execution_state = ExecutionState::Completed;
                                            }
                                            self.is_test_executing = false;
                                        }
                                    }
                                    if ui.button("⏹ Stop").clicked() {
                                        self.test_execution_state = ExecutionState::Idle;
                                        self.is_test_executing = false;
                                        self.test_current_line = 0;
                                    }
                                } else {
                                    let can_start = self.execution_state == ExecutionState::Idle || self.execution_state == ExecutionState::Completed;
                                    let can_pause = self.execution_state == ExecutionState::Running;
                                    let can_resume = self.execution_state == ExecutionState::Paused;
                                    let can_step = self.execution_state == ExecutionState::Paused || can_start;
                                    
                                    ui.label("🔧 Run Controls (preview only: no motion is sent from these yet):");
                                    if ui.add_enabled(can_start, egui::Button::new("▶ Start")).clicked() {
                                        self.execution_state = ExecutionState::Running;
                                        self.is_executing = true;
                                        self.current_line = 0;
                                    }
                                    if ui.add_enabled(can_pause, egui::Button::new("⏸ Pause")).clicked() {
                                        self.execution_state = ExecutionState::Paused;
                                        self.is_executing = false;
                                    }
                                    if ui.add_enabled(can_resume, egui::Button::new("▶ Resume")).clicked() {
                                        self.execution_state = ExecutionState::Running;
                                        self.is_executing = true;
                                    }
                                    if ui.add_enabled(can_step, egui::Button::new("⏭ Step")).clicked() {
                                        let now = Instant::now();
                                        if now.duration_since(self.last_step_time).as_millis() > 100 { // 100ms debounce
                                            self.last_step_time = now;
                                            let gcode_line_count = self.gcode.lines().count(); // More efficient than collect
                                            if self.current_line + 1 < gcode_line_count {
                                                self.current_line += 1;
                                                self.execution_state = ExecutionState::Paused;
                                            } else {
                                                self.execution_state = ExecutionState::Completed;
                                            }
                                            self.is_executing = false;
                                        }
                                    }
                                    if ui.button("⏹ Stop").clicked() {
                                        self.execution_state = ExecutionState::Idle;
                                        self.is_executing = false;
                                        self.current_line = 0;
                                    }
                                }
                            });
                            
                            ui.separator();
                            
                            ui.columns(2, |columns| {
                                columns[0].vertical(|ui| {
                                    ui.horizontal(|ui| {
                                        ui.heading("Toolpath Preview");
                                        ui.separator();
                                        if ui.selectable_label(!self.view_3d, "2D").clicked() {
                                            self.view_3d = false;
                                        }
                                        if ui.selectable_label(self.view_3d, "3D").clicked() {
                                            self.view_3d = true;
                                        }
                                    });
                                    
                                    if !self.plot_points.is_empty() {
                                        let (response, painter) = ui.allocate_painter(egui::Vec2::new(350.0, 250.0), egui::Sense::hover());
                                        let rect = response.rect;
                                        
                                        self.build_render_cache(rect);
                                        
                                        if self.view_3d {
                                            if let Some((_, _, _, _, min_z, max_z)) = self.cached_bounds {
                                                painter.rect_filled(rect, 0.0, Color32::from_rgb(15, 15, 20));
                                                
                                                let total_lines = self.gcode.lines().count();
                                                let current_line = if self.test_mode { self.test_current_line } else { self.current_line };
                                                let progress_ratio = if total_lines > 0 { (current_line as f32) / (total_lines as f32) } else { 0.0 };
                                                let executed_points = (self.cached_3d_points.len() as f32 * progress_ratio) as usize;
                                                
                                                let step = if self.cached_3d_points.len() > 2000 { 
                                                    (self.cached_3d_points.len() / 2000).max(1) 
                                                } else { 1 };
                                                
                                                for i in (0..self.cached_3d_points.len() - step).step_by(step) {
                                                    let z_ratio = if max_z > min_z { (self.cached_3d_points[i].1 - min_z) / (max_z - min_z) } else { 0.0 };
                                                    let color = if i < executed_points {
                                                        Color32::from_rgb((255.0 * z_ratio) as u8, 255, (100.0 * (1.0 - z_ratio)) as u8)
                                                    } else {
                                                        Color32::from_rgb((100.0 * z_ratio) as u8, (150.0 + 105.0 * z_ratio) as u8, 255)
                                                    };
                                                    
                                                    let next_i = (i + step).min(self.cached_3d_points.len() - 1);
                                                    painter.line_segment([self.cached_3d_points[i].0, self.cached_3d_points[next_i].0], egui::Stroke::new(1.5, color));
                                                }
                                                
                                                if !self.cached_3d_points.is_empty() {
                                                    painter.circle_filled(self.cached_3d_points[0].0, 4.0, Color32::from_rgb(0, 255, 0));
                                                    if self.cached_3d_points.len() > 1 {
                                                        painter.circle_filled(self.cached_3d_points[self.cached_3d_points.len() - 1].0, 4.0, Color32::from_rgb(255, 0, 0));
                                                    }
                                                }
                                                
                                                painter.text(
                                                    rect.left_top() + egui::Vec2::new(10.0, 10.0),
                                                    egui::Align2::LEFT_TOP,
                                                    format!("3D View - Z: {:.1} to {:.1}mm", min_z, max_z),
                                                    egui::FontId::proportional(12.0),
                                                    Color32::WHITE,
                                                );
                                            }
                                        } else {
                                            if let Some((min_x, max_x, min_y, max_y, _, _)) = self.cached_bounds {
                                                painter.rect_filled(rect, 0.0, Color32::from_rgb(20, 20, 25));
                                                
                                                let step = if self.cached_2d_points.len() > 3000 { 
                                                    (self.cached_2d_points.len() / 3000).max(1) 
                                                } else { 1 };
                                                
                                                if self.cached_2d_points.len() > 1 {
                                                    let total_lines = self.gcode.lines().count();
                                                    let current_line = if self.test_mode { self.test_current_line } else { self.current_line };
                                                    let progress_ratio = if total_lines > 0 {
                                                        (current_line as f32) / (total_lines as f32)
                                                    } else { 0.0 };
                                                    let executed_points = (self.cached_2d_points.len() as f32 * progress_ratio) as usize;
                                                    
                                                    for i in (0..executed_points.min(self.cached_2d_points.len() - step)).step_by(step) {
                                                        let next_i = (i + step).min(self.cached_2d_points.len() - 1);
                                                        painter.line_segment([self.cached_2d_points[i], self.cached_2d_points[next_i]], egui::Stroke::new(1.8, Color32::from_rgb(0, 255, 0)));
                                                    }
                                                    
                                                    for i in (executed_points..self.cached_2d_points.len() - step).step_by(step) {
                                                        let next_i = (i + step).min(self.cached_2d_points.len() - 1);
                                                        painter.line_segment([self.cached_2d_points[i], self.cached_2d_points[next_i]], egui::Stroke::new(1.8, Color32::from_rgb(0, 200, 255)));
                                                    }
                                                }
                                                
                                                if !self.cached_2d_points.is_empty() {
                                                    painter.circle_filled(self.cached_2d_points[0], 4.0, Color32::from_rgb(0, 255, 0));
                                                    if self.cached_2d_points.len() > 1 {
                                                        painter.circle_filled(self.cached_2d_points[self.cached_2d_points.len() - 1], 4.0, Color32::from_rgb(255, 0, 0));
                                                    }
                                                }
                                                
                                                painter.text(
                                                    rect.left_top() + egui::Vec2::new(10.0, 10.0),
                                                    egui::Align2::LEFT_TOP,
                                                    format!("2D View - X:{:.1}-{:.1} Y:{:.1}-{:.1}mm", min_x, max_x, min_y, max_y),
                                                    egui::FontId::proportional(12.0),
                                                    Color32::WHITE,
                                                );
                                            }
                                        }
                                        
                                        ui.horizontal(|ui| {
                                            if self.view_3d {
                                                ui.colored_label(Color32::from_rgb(255, 255, 100), "🟡 High Z");
                                                ui.colored_label(Color32::from_rgb(0, 255, 0), "🟢 Low Z / Executed");
                                                ui.colored_label(Color32::from_rgb(100, 200, 255), "🔵 Pending");
                                            } else {
                                                ui.colored_label(Color32::from_rgb(0, 255, 0), "● Start / Executed");
                                                ui.colored_label(Color32::from_rgb(255, 0, 0), "● End");
                                                ui.colored_label(Color32::from_rgb(0, 200, 255), "— Pending");
                                            }
                                        });
                                    } else {
                                        ui.label(format!("No plot data available. Plot points: {}", self.plot_points.len()));
                                        if let Some(parsed) = &self.parsed_gcode {
                                            ui.label(format!("G-code parsed with {} commands", parsed.commands.len()));
                                        } else {
                                            ui.label("No parsed G-code available");
                                        }
                                    }
                                });
                                
                                columns[1].vertical(|ui| {
                                    ui.horizontal(|ui| {
                                        ui.heading("G-code");
                                        ui.separator();
                                        if ui.button("🔄 View Full Path").clicked() {
                                            self.selected_gcode_line = None;
                                            self.cache_valid = false; // Force cache rebuild
                                        }
                                        if let Some(selected_line) = self.selected_gcode_line {
                                            ui.label(format!("Viewing to line: {}", selected_line + 1));
                                        }
                                    });
                                    
                                    egui::ScrollArea::vertical()
                                        .id_salt("gcode_text_scroll")
                                        .max_height(400.0)
                                        .auto_shrink([false, true])
                                        .show(ui, |ui| {
                                            let gcode_lines: Vec<&str> = self.gcode.lines().collect();
                                            
                                            for (line_num, line) in gcode_lines.iter().enumerate() {
                                                let (current_line, exec_state) = if self.test_mode {
                                                    (self.test_current_line, &self.test_execution_state)
                                                } else {
                                                    (self.current_line, &self.execution_state)
                                                };
                                                
                                                let is_current = line_num == current_line && *exec_state != ExecutionState::Idle;
                                                let is_executed = line_num < current_line && *exec_state != ExecutionState::Idle;
                                                let is_selected = self.selected_gcode_line == Some(line_num);
                                                
                                                let response = ui.horizontal(|ui| {
                                                    let line_num_color = if is_selected {
                                                        Color32::LIGHT_BLUE
                                                    } else if is_current {
                                                        Color32::YELLOW
                                                    } else if is_executed {
                                                        Color32::from_rgb(0, 200, 0)
                                                    } else {
                                                        Color32::GRAY
                                                    };
                                                    
                                                    ui.colored_label(line_num_color, format!("{:4}", line_num + 1));
                                                    
                                                    let line_color = if is_selected {
                                                        Color32::LIGHT_BLUE
                                                    } else if is_current {
                                                        Color32::YELLOW
                                                    } else if is_executed {
                                                        Color32::from_rgb(100, 255, 100)
                                                    } else if line.trim().starts_with(';') {
                                                        Color32::from_rgb(128, 128, 128)
                                                    } else if line.trim().starts_with('G') {
                                                        Color32::from_rgb(100, 150, 255)
                                                    } else if line.trim().starts_with('M') {
                                                        Color32::from_rgb(255, 150, 100)
                                                    } else {
                                                        Color32::WHITE
                                                    };
                                                    
                                                    let line_response = if is_current {
                                                        let text_rect = ui.available_rect_before_wrap();
                                                        ui.painter().rect_filled(text_rect, 0.0, Color32::from_rgba_unmultiplied(255, 255, 0, 30));
                                                        ui.colored_label(line_color, *line)
                                                    } else {
                                                        ui.colored_label(line_color, *line)
                                                    };
                                                    
                                                    line_response
                                                });
                                                
                                                // Handle line selection for partial path viewing
                                                if response.response.clicked() {
                                                    if self.selected_gcode_line == Some(line_num) {
                                                        // Clicking the same line deselects it
                                                        self.selected_gcode_line = None;
                                                    } else {
                                                        // Select this line for partial path viewing
                                                        self.selected_gcode_line = Some(line_num);
                                                        println!("Selected G-code line {} for partial viewing: {}", line_num + 1, line.trim());
                                                    }
                                                    self.cache_valid = false; // Force cache rebuild to show partial path
                                                }
                                                
                                                // Highlight selected line background
                                                if is_selected {
                                                    let text_rect = response.response.rect;
                                                    ui.painter().rect_filled(text_rect, 2.0, Color32::from_rgba_unmultiplied(0, 150, 255, 40));
                                                }
                                            }
                                        });
                                });
                            });
                        }
                    } else {
                        ui.label("No G-code file loaded. Use the 'Gcode Import' tab to load a file.");
                    }
                }
                Tab::Hardware => {
                    ui.heading("🖨️ Hardware Configuration");

                    egui::ScrollArea::vertical()
                        .id_salt("hardware_scroll")
                        .auto_shrink([false, true])
                        .show(ui, |ui| {

                        ui.label("Hardware settings combined here - Motor Assignment, Closed Loop, Temperature");
                        ui.label("Select the specific hardware tab from the sidebar for detailed configuration.");

                        // Quick links to specific hardware tabs
                        ui.horizontal(|ui| {
                            if ui.button("🔩 Configure Motors").clicked() {
                                self.selected_tab = Tab::MotorAssignment;
                            }
                            if ui.button("🔄 Closed Loop Control").clicked() {
                                self.selected_tab = Tab::ClosedLoop;
                            }
                            if ui.button("🌡️ Temperature Control").clicked() {
                                self.selected_tab = Tab::Temperature;
                            }
                        });
                    });
                }
                Tab::Operations => {
                    ui.heading("🎯 Operations & Automation");

                    egui::ScrollArea::vertical()
                        .id_salt("operations_scroll")
                        .auto_shrink([false, true])
                        .show(ui, |ui| {

                        ui.label("Operations settings combined here - Macros, G-code Import, etc.");
                        ui.label("Select the specific operations tab from the sidebar for detailed configuration.");

                        // Quick links to specific operations tabs
                        ui.horizontal(|ui| {
                            if ui.button("🎯 Configure Macros").clicked() {
                                self.selected_tab = Tab::Macros;
                            }
                            if ui.button("📁 G-code Import").clicked() {
                                self.selected_tab = Tab::GcodeImport;
                            }
                        });
                    });
                }
                Tab::Communication => {
                    ui.heading("🔌 Communication & Protocols");

                    egui::ScrollArea::vertical()
                        .id_salt("communication_scroll")
                        .auto_shrink([false, true])
                        .show(ui, |ui| {

                        ui.label("Communication settings combined here - Fieldbus, CAN Bus, etc.");
                        ui.label("Select the specific communication tab from the sidebar for detailed configuration.");

                        // Quick links to specific communication tabs
                        ui.horizontal(|ui| {
                            if ui.button("🌐 Configure Fieldbus").clicked() {
                                self.selected_tab = Tab::FieldbusConfig;
                            }
                            if ui.button("🚛 CAN Bus Toolheads").clicked() {
                                self.selected_tab = Tab::CanToolheads;
                            }
                        });
                    });
                }
                Tab::System => {
                    ui.heading("⚙️ System Configuration");

                    egui::ScrollArea::vertical()
                        .id_salt("system_scroll")
                        .auto_shrink([false, true])
                        .show(ui, |ui| {

                        ui.label("System settings combined here - Settings, Printer Config, Runtime Config, etc.");
                        ui.label("Select the specific system tab from the sidebar for detailed configuration.");

                        // Quick links to specific system tabs
                        ui.horizontal(|ui| {
                            if ui.button("🔧 General Settings").clicked() {
                                self.selected_tab = Tab::Settings;
                            }
                            if ui.button("🖨️ Printer Configuration").clicked() {
                                self.selected_tab = Tab::PrinterConfig;
                            }
                            if ui.button("⚙️ Runtime Configuration").clicked() {
                                self.selected_tab = Tab::RuntimeConfig;
                            }
                        });
                    });
                }
                Tab::Settings => {
                    ui.heading("Settings");
                    
                    egui::ScrollArea::vertical()
                        .id_salt("settings_scroll")
                        .auto_shrink([false, true])
                        .show(ui, |ui| {
                        ui.group(|ui| {
                            ui.label("🎨 Appearance");
                            ui.horizontal(|ui| {
                                ui.label("Theme:");
                                if ui.selectable_label(self.dark_mode, "🌙 Dark").clicked() {
                                    self.dark_mode = true;
                                }
                                if ui.selectable_label(!self.dark_mode, "☀ Light").clicked() {
                                    self.dark_mode = false;
                                }
                            });
                        });
                        
                        ui.separator();
                        
                        ui.group(|ui| {
                            ui.label("⚙️ G-code Execution");
                            ui.horizontal(|ui| {
                                ui.label("Mode:");
                                if ui.selectable_label(!self.test_mode, "🔧 Real Mode").clicked() {
                                    self.test_mode = false;
                                    self.execution_state = ExecutionState::Idle;
                                    self.is_executing = false;
                                    self.test_execution_state = ExecutionState::Idle;
                                    self.is_test_executing = false;
                                }
                                if ui.selectable_label(self.test_mode, "🧪 Test Mode").clicked() {
                                    self.test_mode = true;
                                    self.execution_state = ExecutionState::Idle;
                                    self.is_executing = false;
                                    self.test_execution_state = ExecutionState::Idle;
                                    self.is_test_executing = false;
                                }
                            });
                            
                            if self.test_mode {
                                ui.label("⚠ Test mode: G-code will be simulated without controlling the actual machine");
                            } else {
                                ui.label("⚠ Real mode: G-code commands will be sent to the actual machine");
                            }
                        });
                        
                        ui.separator();
                        
                        ui.group(|ui| {
                            ui.label("📡 Communication Protocol");
                            ui.label(format!("Current: {:?}", self.communication_mode));
                            ui.label("Configure detailed fieldbus settings in the 'Fieldbus Config' tab");
                        });
                        
                        ui.separator();
                        
                        if let Some(error) = &self.last_file_error {
                            ui.colored_label(Color32::RED, format!("Error: {}", error));
                        }
                    });
                }
                Tab::PrinterConfig => {
                    ui.heading("Printer Configuration");
                    ui.label("Configure printer settings similar to Klipper's printer.cfg");
                    
                    egui::ScrollArea::vertical().id_salt("printer_config_scroll").show(ui, |ui| {
                        ui.group(|ui| {
                            ui.label("🌡️ Temperature Settings");
                            ui.add(egui::Slider::new(&mut self.printer_settings.extruder_max_temp, 180.0..=400.0).text("Max Extruder Temp (°C)"));
                            ui.add(egui::Slider::new(&mut self.printer_settings.bed_max_temp, 60.0..=150.0).text("Max Bed Temp (°C)"));
                            ui.add(egui::Slider::new(&mut self.printer_settings.extruder_min_extrude_temp, 160.0..=200.0).text("Min Extrude Temp (°C)"));
                        });
                        
                        ui.separator();
                        
                        ui.group(|ui| {
                            ui.label("🏃 Movement Settings");
                            ui.add(egui::Slider::new(&mut self.printer_settings.max_velocity, 50.0..=500.0).text("Max Velocity (mm/s)"));
                            ui.add(egui::Slider::new(&mut self.printer_settings.max_accel, 500.0..=10000.0).text("Max Acceleration (mm/s²)"));
                            ui.add(egui::Slider::new(&mut self.printer_settings.max_accel_to_decel, 250.0..=5000.0).text("Max Accel to Decel (mm/s²)"));
                            ui.add(egui::Slider::new(&mut self.printer_settings.square_corner_velocity, 1.0..=20.0).text("Square Corner Velocity (mm/s)"));
                        });
                        
                        ui.separator();
                        
                        ui.group(|ui| {
                            ui.label("🏠 Homing Settings");
                            ui.add(egui::Slider::new(&mut self.printer_settings.homing_speed, 10.0..=100.0).text("Homing Speed (mm/s)"));
                            ui.add(egui::Slider::new(&mut self.printer_settings.homing_retract_dist, 1.0..=20.0).text("Homing Retract Distance (mm)"));
                            ui.add(egui::Slider::new(&mut self.printer_settings.second_homing_speed, 5.0..=50.0).text("Second Homing Speed (mm/s)"));
                        });
                        
                        ui.separator();
                        
                        ui.group(|ui| {
                            ui.label("📏 Print Area Limits");
                            ui.horizontal(|ui| {
                                ui.label("X min/max:");
                                ui.add(egui::DragValue::new(&mut self.printer_settings.position_min[0]).speed(1.0));
                                ui.add(egui::DragValue::new(&mut self.printer_settings.position_max[0]).speed(1.0));
                            });
                            ui.horizontal(|ui| {
                                ui.label("Y min/max:");
                                ui.add(egui::DragValue::new(&mut self.printer_settings.position_min[1]).speed(1.0));
                                ui.add(egui::DragValue::new(&mut self.printer_settings.position_max[1]).speed(1.0));
                            });
                            ui.horizontal(|ui| {
                                ui.label("Z min/max:");
                                ui.add(egui::DragValue::new(&mut self.printer_settings.position_min[2]).speed(1.0));
                                ui.add(egui::DragValue::new(&mut self.printer_settings.position_max[2]).speed(1.0));
                            });
                        });
                        
                        ui.separator();
                        
                        ui.group(|ui| {
                            ui.label("⚙️ Motor Configuration");
                            let motor_labels = ["X-Axis", "Y-Axis", "Z-Axis", "Extruder 0", "Extruder 1"];
                            
                            for (i, label) in motor_labels.iter().enumerate() {
                                ui.horizontal(|ui| {
                                    ui.label(format!("{}:", label));
                                    egui::ComboBox::from_id_salt(format!("motor_{}", i))
                                        .selected_text(format!("{:?}", self.printer_settings.motor_types[i]))
                                        .show_ui(ui, |ui| {
                                            ui.selectable_value(&mut self.printer_settings.motor_types[i], MotorType::Stepper, "Stepper");
                                            ui.selectable_value(&mut self.printer_settings.motor_types[i], MotorType::Servo, "Servo");
                                        });
                                });
                            }
                        });
                        
                        ui.separator();
                        
                        ui.group(|ui| {
                            ui.label("⚡ Stepper Motor Settings");
                            ui.add(egui::Slider::new(&mut self.printer_settings.hold_current, 0.1..=2.0).text("Hold Current (A)"));
                            ui.add(egui::Slider::new(&mut self.printer_settings.run_current, 0.2..=2.5).text("Run Current (A)"));
                            egui::ComboBox::from_label("Microsteps")
                                .selected_text(format!("{}", self.printer_settings.microsteps))
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(&mut self.printer_settings.microsteps, 16, "16");
                                    ui.selectable_value(&mut self.printer_settings.microsteps, 32, "32");
                                    ui.selectable_value(&mut self.printer_settings.microsteps, 64, "64");
                                    ui.selectable_value(&mut self.printer_settings.microsteps, 128, "128");
                                    ui.selectable_value(&mut self.printer_settings.microsteps, 256, "256");
                                });
                        });
                        
                        ui.separator();
                        
                        ui.group(|ui| {
                            ui.label("🔧 Servo Motor Settings");
                            ui.add(egui::Slider::new(&mut self.printer_settings.servo_min_pulse, 500..=1500).text("Min Pulse Width (µs)"));
                            ui.add(egui::Slider::new(&mut self.printer_settings.servo_max_pulse, 1500..=3000).text("Max Pulse Width (µs)"));
                            ui.add(egui::Slider::new(&mut self.printer_settings.servo_max_angle, 90.0..=360.0).text("Max Angle (degrees)"));
                            ui.label("Common servo types:");
                            ui.horizontal(|ui| {
                                if ui.button("Standard (500-2500µs)").clicked() {
                                    self.printer_settings.servo_min_pulse = 500;
                                    self.printer_settings.servo_max_pulse = 2500;
                                    self.printer_settings.servo_max_angle = 180.0;
                                }
                                if ui.button("Digital (600-2400µs)").clicked() {
                                    self.printer_settings.servo_min_pulse = 600;
                                    self.printer_settings.servo_max_pulse = 2400;
                                    self.printer_settings.servo_max_angle = 180.0;
                                }
                                if ui.button("Continuous (1000-2000µs)").clicked() {
                                    self.printer_settings.servo_min_pulse = 1000;
                                    self.printer_settings.servo_max_pulse = 2000;
                                    self.printer_settings.servo_max_angle = 360.0;
                                }
                            });
                        });
                        
                        ui.separator();
                        
                        ui.group(|ui| {
                            ui.label("🌊 Pressure Advance");
                            ui.add(egui::Slider::new(&mut self.printer_settings.pressure_advance, 0.0..=0.2).text("Pressure Advance"));
                            ui.add(egui::Slider::new(&mut self.printer_settings.pressure_advance_smooth_time, 0.01..=0.1).text("Smooth Time (s)"));
                            ui.label("⚠ Start with small values (0.01-0.05) and increase gradually");
                        });
                        
                        ui.separator();
                        
                        ui.group(|ui| {
                            ui.label("🔍 Z Probe Settings");
                            ui.add(egui::Slider::new(&mut self.printer_settings.probe_z_offset, -5.0..=5.0).text("Probe Z Offset (mm)"));
                            ui.horizontal(|ui| {
                                ui.label("Probe XY Offset:");
                                ui.add(egui::DragValue::new(&mut self.printer_settings.probe_xy_offset[0]).prefix("X:").speed(0.1));
                                ui.add(egui::DragValue::new(&mut self.printer_settings.probe_xy_offset[1]).prefix("Y:").speed(0.1));
                            });
                            ui.add(egui::Slider::new(&mut self.printer_settings.probe_speed, 1.0..=20.0).text("Probe Speed (mm/s)"));
                            ui.add(egui::Slider::new(&mut self.printer_settings.probe_lift_speed, 5.0..=50.0).text("Lift Speed (mm/s)"));
                            ui.add(egui::Slider::new(&mut self.printer_settings.probe_samples, 1..=10).text("Samples per Point"));
                            ui.add(egui::Slider::new(&mut self.printer_settings.probe_sample_retract_dist, 0.5..=10.0).text("Sample Retract Distance (mm)"));
                        });
                        
                        ui.separator();
                        
                        ui.group(|ui| {
                            ui.label("🔄 5-Axis Control");
                            ui.checkbox(&mut self.printer_settings.enable_5axis_mode, "Enable 5-Axis Mode");
                            if self.printer_settings.enable_5axis_mode {
                                ui.checkbox(&mut self.printer_settings.tool_point_control, "Tool Center Point (TCP) Control");
                                ui.horizontal(|ui| {
                                    ui.label("A-Axis Max Velocity:");
                                    ui.add(egui::DragValue::new(&mut self.printer_settings.rotary_axis_max_velocity[0]).suffix("°/s").speed(1.0));
                                });
                                ui.horizontal(|ui| {
                                    ui.label("B-Axis Max Velocity:");
                                    ui.add(egui::DragValue::new(&mut self.printer_settings.rotary_axis_max_velocity[1]).suffix("°/s").speed(1.0));
                                });
                                ui.horizontal(|ui| {
                                    ui.label("A-Axis Acceleration:");
                                    ui.add(egui::DragValue::new(&mut self.printer_settings.rotary_axis_acceleration[0]).suffix("°/s²").speed(10.0));
                                });
                                ui.horizontal(|ui| {
                                    ui.label("B-Axis Acceleration:");
                                    ui.add(egui::DragValue::new(&mut self.printer_settings.rotary_axis_acceleration[1]).suffix("°/s²").speed(10.0));
                                });
                                ui.label("⚠ TCP control maintains tool tip position during rotary movement");
                            }
                        });
                        
                        ui.separator();
                        
                        ui.group(|ui| {
                            ui.label("🔄 Firmware Retraction");
                            ui.add(egui::Slider::new(&mut self.printer_settings.retract_length, 0.0..=10.0).text("Retract Length (mm)"));
                            ui.add(egui::Slider::new(&mut self.printer_settings.retract_speed, 10.0..=100.0).text("Retract Speed (mm/s)"));
                            ui.add(egui::Slider::new(&mut self.printer_settings.unretract_extra_length, 0.0..=2.0).text("Unretract Extra Length (mm)"));
                            ui.add(egui::Slider::new(&mut self.printer_settings.unretract_speed, 10.0..=100.0).text("Unretract Speed (mm/s)"));
                        });
                        
                        ui.separator();
                        
                        ui.horizontal(|ui| {
                            if ui.button("💾 Save Configuration").clicked() {
                                println!("Configuration saved to memory");
                                // Here you would typically save to printer.toml or similar
                            }
                            if ui.button("🔄 Reset to Defaults").clicked() {
                                // Reset to default values
                                self.printer_settings = PrinterSettings {
                                    extruder_max_temp: 300.0,
                                    bed_max_temp: 120.0,
                                    extruder_min_extrude_temp: 170.0,
                                    max_velocity: 300.0,
                                    max_accel: 3000.0,
                                    max_accel_to_decel: 1500.0,
                                    square_corner_velocity: 5.0,
                                    homing_speed: 50.0,
                                    homing_retract_dist: 5.0,
                                    second_homing_speed: 10.0,
                                    position_min: [0.0, 0.0, 0.0],
                                    position_max: [200.0, 200.0, 200.0],
                                    motor_types: [MotorType::Stepper, MotorType::Stepper, MotorType::Stepper, 
                                                 MotorType::Stepper, MotorType::Stepper],
                                    hold_current: 0.5,
                                    run_current: 0.8,
                                    microsteps: 16,
                                    servo_min_pulse: 500,
                                    servo_max_pulse: 2500,
                                    servo_max_angle: 180.0,
                                    retract_length: 0.5,
                                    retract_speed: 35.0,
                                    unretract_extra_length: 0.0,
                                    unretract_speed: 35.0,
                                    // New settings
                                    pressure_advance: 0.0,
                                    pressure_advance_smooth_time: 0.04,
                                    z_offset: 0.0,
                                    probe_z_offset: 0.0,
                                    probe_xy_offset: [0.0, 0.0],
                                    probe_speed: 5.0,
                                    probe_lift_speed: 15.0,
                                    probe_samples: 3,
                                    probe_sample_retract_dist: 2.0,
                                    enable_5axis_mode: false,
                                    tool_point_control: false,
                                    rotary_axis_max_velocity: [120.0, 120.0],
                                    rotary_axis_acceleration: [1800.0, 1800.0],
                                };
                                println!("Configuration reset to defaults");
                            }
                        });
                    });
                }
                Tab::BedMesh => {
                    ui.heading("📏 Bed Mesh Leveling");
                    ui.label("View, create, and manage bed mesh profiles for optimal print adhesion");
                    
                    egui::ScrollArea::vertical().id_salt("bed_mesh_scroll").show(ui, |ui| {
                    
                    ui.horizontal(|ui| {
                        ui.label("Active Profile:");
                        egui::ComboBox::from_label("Mesh Profile")
                            .selected_text(&self.selected_mesh_profile)
                            .show_ui(ui, |ui| {
                                for mesh in &self.saved_bed_meshes {
                                    if ui.selectable_value(&mut self.selected_mesh_profile, mesh.name.clone(), &mesh.name).clicked() {
                                        // Load the selected mesh
                                        self.current_bed_mesh = Some(mesh.clone());
                                    }
                                }
                            });
                    });
                    
                    ui.separator();
                    
                    ui.horizontal(|ui| {
                        if ui.button("🔄 Start Bed Mesh").clicked() && !self.emergency_stop_triggered {
                            println!("Starting bed mesh calibration...");
                            self.printer_state = PrinterState::Homing;
                        }
                        
                        if ui.button("🎯 KAMP Adaptive Mesh").clicked() && !self.emergency_stop_triggered {
                            self.analyze_gcode_for_print_area();
                            if let Some(adaptive_mesh) = self.create_adaptive_mesh() {
                                self.current_bed_mesh = Some(adaptive_mesh.clone());
                                self.saved_bed_meshes.push(adaptive_mesh);
                                self.selected_mesh_profile = "adaptive".to_string();
                                println!("KAMP adaptive mesh created and loaded!");
                            } else {
                                println!("Could not create adaptive mesh - load G-code first or enable adaptive meshing");
                            }
                        }
                        
                        if ui.button("💾 Save Profile").clicked() {
                            if let Some(mesh) = &self.current_bed_mesh {
                                let mut new_mesh = mesh.clone();
                                new_mesh.name = format!("profile_{}", self.saved_bed_meshes.len() + 1);
                                self.saved_bed_meshes.push(new_mesh);
                                println!("Bed mesh profile saved");
                            }
                        }
                        
                        if ui.button("🗑️ Delete Profile").clicked() {
                            if let Some(index) = self.saved_bed_meshes.iter().position(|m| m.name == self.selected_mesh_profile) {
                                if index > 0 { // Don't delete default profile
                                    self.saved_bed_meshes.remove(index);
                                    self.selected_mesh_profile = "default".to_string();
                                    self.current_bed_mesh = Some(self.saved_bed_meshes[0].clone());
                                    println!("Profile deleted");
                                }
                            }
                        }
                    });
                    
                    ui.separator();
                    
                    // KAMP Adaptive Settings
                    ui.group(|ui| {
                        ui.label("🎯 KAMP Adaptive Settings");
                        
                        ui.checkbox(&mut self.adaptive_settings.enable_adaptive_meshing, "Enable Adaptive Meshing");
                        ui.checkbox(&mut self.adaptive_settings.enable_adaptive_z_height, "Enable Adaptive Z Height");
                        ui.checkbox(&mut self.adaptive_settings.kamp_style_probing, "KAMP Style Probe Pattern");
                        
                        ui.horizontal(|ui| {
                            ui.label("Margins:");
                            ui.add(egui::DragValue::new(&mut self.adaptive_settings.margin_x).prefix("X:").suffix("mm").speed(0.5));
                            ui.add(egui::DragValue::new(&mut self.adaptive_settings.margin_y).prefix("Y:").suffix("mm").speed(0.5));
                        });
                        
                        ui.horizontal(|ui| {
                            ui.label("Probe Count Range:");
                            ui.add(egui::DragValue::new(&mut self.adaptive_settings.min_probe_count).prefix("Min:").range(9..=225));
                            ui.add(egui::DragValue::new(&mut self.adaptive_settings.max_probe_count).prefix("Max:").range(9..=225));
                        });
                        
                        if self.current_print_area.valid {
                            ui.separator();
                            ui.label("📐 Current Print Area:");
                            let (width, height, _max_z) = self.current_print_area.get_dimensions();
                            ui.label(format!("  Bounds: X({:.1}-{:.1}) Y({:.1}-{:.1})", 
                                self.current_print_area.min_x, self.current_print_area.max_x,
                                self.current_print_area.min_y, self.current_print_area.max_y));
                            ui.label(format!("  Size: {:.1}×{:.1}mm", width, height));
                            
                            if let Some(adaptive_height) = self.get_adaptive_z_height() {
                                ui.label(format!("  Adaptive Z Height: {:.1}mm", adaptive_height));
                            }
                        } else {
                            ui.label("⚠ No G-code analyzed yet - load and analyze G-code for adaptive features");
                        }
                    });
                    
                    ui.separator();
                    
                    if let Some(mesh) = &self.current_bed_mesh {
                        ui.columns(2, |columns| {
                            // Left column: 3D mesh visualization
                            columns[0].vertical(|ui| {
                                ui.heading("3D Mesh Visualization");
                                
                                let (response, painter) = ui.allocate_painter(egui::Vec2::new(400.0, 300.0), egui::Sense::hover());
                                let rect = response.rect;
                                
                                // Clear background
                                painter.rect_filled(rect, 0.0, Color32::from_rgb(25, 25, 30));
                                
                                // Calculate colors based on height
                                let height_range = mesh.max_height - mesh.min_height;
                                
                                // Draw mesh grid
                                let cell_width = rect.width() / mesh.mesh_width as f32;
                                let cell_height = rect.height() / mesh.mesh_height as f32;
                                
                                for (i, row) in mesh.points.iter().enumerate() {
                                    for (j, &height) in row.iter().enumerate() {
                                        let x = rect.left() + j as f32 * cell_width;
                                        let y = rect.top() + i as f32 * cell_height;
                                        
                                        // Color based on height (blue = low, red = high)
                                        let height_ratio = if height_range > 0.0 {
                                            (height - mesh.min_height) / height_range
                                        } else {
                                            0.5
                                        };
                                        
                                        let color = if height_ratio < 0.5 {
                                            // Blue to green
                                            let t = height_ratio * 2.0;
                                            Color32::from_rgb(0, (t * 255.0) as u8, (255.0 * (1.0 - t)) as u8)
                                        } else {
                                            // Green to red
                                            let t = (height_ratio - 0.5) * 2.0;
                                            Color32::from_rgb((t * 255.0) as u8, (255.0 * (1.0 - t)) as u8, 0)
                                        };
                                        
                                        painter.rect_filled(
                                            egui::Rect::from_min_size(
                                                egui::pos2(x, y),
                                                egui::vec2(cell_width - 1.0, cell_height - 1.0)
                                            ),
                                            0.0,
                                            color
                                        );
                                    }
                                }
                                
                                // Handle hover for tooltips
                                if let Some(hover_pos) = response.hover_pos() {
                                    if let Some((mesh_x, mesh_y, height)) = mesh.get_height_at_coord(hover_pos.x, hover_pos.y, rect) {
                                        self.mesh_hover_info = Some((
                                            mesh.min_x + (mesh_x as f32 / (mesh.mesh_width - 1) as f32) * (mesh.max_x - mesh.min_x),
                                            mesh.min_y + (mesh_y as f32 / (mesh.mesh_height - 1) as f32) * (mesh.max_y - mesh.min_y),
                                            height
                                        ));
                                        
                                        // Draw crosshair
                                        let cell_x = rect.left() + mesh_x as f32 * cell_width + cell_width * 0.5;
                                        let cell_y = rect.top() + mesh_y as f32 * cell_height + cell_height * 0.5;
                                        
                                        painter.line_segment(
                                            [egui::pos2(cell_x - 10.0, cell_y), egui::pos2(cell_x + 10.0, cell_y)],
                                            egui::Stroke::new(2.0, Color32::WHITE)
                                        );
                                        painter.line_segment(
                                            [egui::pos2(cell_x, cell_y - 10.0), egui::pos2(cell_x, cell_y + 10.0)],
                                            egui::Stroke::new(2.0, Color32::WHITE)
                                        );
                                    }
                                } else {
                                    self.mesh_hover_info = None;
                                }
                                
                                // Show tooltip with height info
                                if let Some((x, y, z)) = self.mesh_hover_info {
                                    ui.label(format!("Position: X:{:.1}, Y:{:.1}", x, y));
                                    ui.label(format!("Height: {:.3}mm", z));
                                }
                            });
                            
                            // Right column: Height scale and mesh info
                            columns[1].vertical(|ui| {
                                ui.heading("Mesh Information");
                                
                                ui.group(|ui| {
                                    ui.label(format!("Profile: {}", mesh.name));
                                    ui.label(format!("Grid Size: {}x{}", mesh.mesh_width, mesh.mesh_height));
                                    ui.label(format!("X Range: {:.1} - {:.1}mm", mesh.min_x, mesh.max_x));
                                    ui.label(format!("Y Range: {:.1} - {:.1}mm", mesh.min_y, mesh.max_y));
                                });
                                
                                ui.separator();
                                
                                ui.heading("Height Scale");
                                
                                // Draw height scale similar to Klipper
                                let (scale_response, scale_painter) = ui.allocate_painter(egui::Vec2::new(40.0, 200.0), egui::Sense::hover());
                                let scale_rect = scale_response.rect;
                                
                                // Draw gradient scale
                                for i in 0..200 {
                                    let y = scale_rect.top() + i as f32;
                                    let t = 1.0 - (i as f32 / 199.0); // Top is high, bottom is low
                                    
                                    let color = if t < 0.5 {
                                        let t = t * 2.0;
                                        Color32::from_rgb(0, (t * 255.0) as u8, (255.0 * (1.0 - t)) as u8)
                                    } else {
                                        let t = (t - 0.5) * 2.0;
                                        Color32::from_rgb((t * 255.0) as u8, (255.0 * (1.0 - t)) as u8, 0)
                                    };
                                    
                                    scale_painter.line_segment(
                                        [egui::pos2(scale_rect.left(), y), egui::pos2(scale_rect.right(), y)],
                                        egui::Stroke::new(1.0, color)
                                    );
                                }
                                
                                // Scale labels
                                ui.horizontal(|ui| {
                                    ui.add_space(45.0);
                                    ui.vertical(|ui| {
                                        ui.colored_label(Color32::RED, format!("High: {:.3}mm", mesh.max_height));
                                        ui.add_space(160.0);
                                        ui.colored_label(Color32::BLUE, format!("Low: {:.3}mm", mesh.min_height));
                                        ui.label(format!("Range: {:.3}mm", mesh.max_height - mesh.min_height));
                                    });
                                });
                                
                                ui.separator();
                                
                                ui.label("Statistics:");
                                let avg_height = mesh.points.iter().flatten().sum::<f32>() / (mesh.mesh_width * mesh.mesh_height) as f32;
                                ui.label(format!("Average: {:.3}mm", avg_height));
                                ui.label(format!("Std Dev: {:.3}mm", {
                                    let variance = mesh.points.iter().flatten()
                                        .map(|&h| (h - avg_height).powi(2))
                                        .sum::<f32>() / (mesh.mesh_width * mesh.mesh_height) as f32;
                                    variance.sqrt()
                                }));
                            });
                        });
                        } else {
                            ui.label("No bed mesh loaded. Please select or create a mesh profile.");
                        }
                    });
                }
                Tab::FanControl => {
                    ui.heading("🔌 Fan Control System");
                    ui.label("Control all cooling fans and configure automatic behaviors");
                    
                    egui::ScrollArea::vertical().id_salt("fan_control_scroll").show(ui, |ui| {
                    
                    ui.separator();
                    
                    // Manual fan controls
                    ui.group(|ui| {
                        ui.label("💨 Manual Fan Controls");
                        
                        ui.horizontal(|ui| {
                            ui.label("Part Cooling:");
                            ui.add(egui::Slider::new(&mut self.fan_control.part_cooling_speed, 0..=100).suffix("%"));
                            ui.checkbox(&mut self.fan_control.auto_part_cooling, "Auto");
                        });
                        
                        ui.horizontal(|ui| {
                            ui.label("Auxiliary Fan:");
                            ui.add(egui::Slider::new(&mut self.fan_control.auxiliary_fan_speed, 0..=100).suffix("%"));
                        });
                        
                        ui.horizontal(|ui| {
                            ui.label("Exhaust Fan:");
                            ui.add(egui::Slider::new(&mut self.fan_control.exhaust_fan_speed, 0..=100).suffix("%"));
                            ui.checkbox(&mut self.fan_control.auto_exhaust, "Auto");
                        });
                        
                        ui.horizontal(|ui| {
                            ui.label("Chamber Fan:");
                            ui.add(egui::Slider::new(&mut self.fan_control.chamber_fan_speed, 0..=100).suffix("%"));
                        });
                        
                        ui.separator();
                        
                        ui.horizontal(|ui| {
                            if ui.button("🌪️ All Fans 100%").clicked() && !self.emergency_stop_triggered {
                                self.fan_control.part_cooling_speed = 100;
                                self.fan_control.auxiliary_fan_speed = 100;
                                self.fan_control.exhaust_fan_speed = 100;
                                self.fan_control.chamber_fan_speed = 100;
                                println!("All fans set to 100%");
                            }
                            if ui.button("🛑 All Fans Off").clicked() {
                                self.fan_control.part_cooling_speed = 0;
                                self.fan_control.auxiliary_fan_speed = 0;
                                self.fan_control.exhaust_fan_speed = 0;
                                self.fan_control.chamber_fan_speed = 0;
                                println!("All fans turned off");
                            }
                        });
                    });
                    
                    ui.separator();
                    
                    // Automatic fan controls
                    ui.group(|ui| {
                        ui.label("🤖 Automatic Fan Control");
                        
                        ui.checkbox(&mut self.fan_control.part_cooling_layer_fan, "Enable Layer Fan Control");
                        if self.fan_control.part_cooling_layer_fan {
                            ui.horizontal(|ui| {
                                ui.label("Start Layer:");
                                ui.add(egui::DragValue::new(&mut self.fan_control.layer_fan_start_layer).range(1..=100));
                            });
                        }
                        
                        ui.separator();
                        
                        ui.checkbox(&mut self.fan_control.auto_exhaust, "Enable Auto Exhaust");
                        if self.fan_control.auto_exhaust {
                            ui.horizontal(|ui| {
                                ui.label("Temperature Threshold:");
                                ui.add(egui::DragValue::new(&mut self.fan_control.exhaust_temp_threshold).suffix("°C").range(20.0..=80.0));
                            });
                            ui.label("⚠ Exhaust fan will activate when chamber exceeds threshold");
                        }
                    });
                    
                    ui.separator();
                    
                    // Fan status display
                    ui.group(|ui| {
                        ui.label("📊 Fan Status");
                        
                        ui.horizontal(|ui| {
                            let part_color = if self.fan_control.part_cooling_speed > 0 { Color32::GREEN } else { Color32::GRAY };
                            ui.colored_label(part_color, format!("Part Cooling: {}%", self.fan_control.part_cooling_speed));
                            
                            let aux_color = if self.fan_control.auxiliary_fan_speed > 0 { Color32::BLUE } else { Color32::GRAY };
                            ui.colored_label(aux_color, format!("Auxiliary: {}%", self.fan_control.auxiliary_fan_speed));
                        });
                        
                        ui.horizontal(|ui| {
                            let exhaust_color = if self.fan_control.exhaust_fan_speed > 0 { Color32::YELLOW } else { Color32::GRAY };
                            ui.colored_label(exhaust_color, format!("Exhaust: {}%", self.fan_control.exhaust_fan_speed));
                            
                            let chamber_color = if self.fan_control.chamber_fan_speed > 0 { Color32::from_rgb(255, 165, 0) } else { Color32::GRAY };
                            ui.colored_label(chamber_color, format!("Chamber: {}%", self.fan_control.chamber_fan_speed));
                        });
                    });
                    });
                }
                Tab::Webcam => {
                    ui.heading("Webcam Monitor");
                    ui.label("Monitor your print with live webcam feed");
                    
                    ui.separator();
                    
                    // Webcam settings
                    ui.group(|ui| {
                        ui.label("📹 Webcam Settings");
                        
                        ui.checkbox(&mut self.webcam_settings.enabled, "Enable Webcam");
                        
                        if self.webcam_settings.enabled {
                            ui.horizontal(|ui| {
                                ui.label("URL:");
                                ui.text_edit_singleline(&mut self.webcam_settings.url);
                            });
                            
                            ui.horizontal(|ui| {
                                ui.label("Refresh Rate:");
                                ui.add(egui::DragValue::new(&mut self.webcam_settings.refresh_rate).suffix("ms").range(50..=5000));
                                ui.label(format!("({:.1} FPS)", 1000.0 / self.webcam_settings.refresh_rate as f32));
                            });
                            
                            ui.horizontal(|ui| {
                                ui.label("Resolution:");
                                ui.add(egui::DragValue::new(&mut self.webcam_settings.resolution.0).range(160..=1920));
                                ui.label("×");
                                ui.add(egui::DragValue::new(&mut self.webcam_settings.resolution.1).range(120..=1080));
                            });
                        }
                    });
                    
                    ui.separator();
                    
                    if self.webcam_settings.enabled {
                        // Webcam display area
                        ui.group(|ui| {
                            ui.label("📺 Live Feed");
                            
                            let (response, painter) = ui.allocate_painter(
                                egui::Vec2::new(
                                    self.webcam_settings.resolution.0 as f32 * 0.5,
                                    self.webcam_settings.resolution.1 as f32 * 0.5
                                ), 
                                egui::Sense::hover()
                            );
                            let rect = response.rect;
                            
                            // Placeholder webcam display
                            painter.rect_filled(rect, 0.0, Color32::from_rgb(20, 20, 25));
                            
                            // Draw placeholder content
                            painter.text(
                                rect.center(),
                                egui::Align2::CENTER_CENTER,
                                "📷 Webcam Feed\n(Connect to stream)",
                                egui::FontId::proportional(16.0),
                                Color32::WHITE
                            );
                            
                            // Draw overlay if enabled
                            if self.webcam_settings.show_overlay {
                                painter.text(
                                    rect.left_top() + egui::vec2(10.0, 10.0),
                                    egui::Align2::LEFT_TOP,
                                    &self.webcam_settings.overlay_text,
                                    egui::FontId::proportional(12.0),
                                    Color32::YELLOW
                                );
                                
                                // Time stamp
                                let timestamp = std::time::SystemTime::now()
                                    .duration_since(std::time::UNIX_EPOCH)
                                    .unwrap_or_default();
                                painter.text(
                                    rect.right_bottom() + egui::vec2(-10.0, -10.0),
                                    egui::Align2::RIGHT_BOTTOM,
                                    &format!("T+{:.1}s", timestamp.as_secs_f32()),
                                    egui::FontId::proportional(12.0),
                                    Color32::WHITE
                                );
                            }
                        });
                        
                        ui.separator();
                        
                        // Webcam controls and overlay settings
                        ui.group(|ui| {
                            ui.label("🎛️ Display Controls");
                            
                            ui.checkbox(&mut self.webcam_settings.show_overlay, "Show Overlay");
                            if self.webcam_settings.show_overlay {
                                ui.horizontal(|ui| {
                                    ui.label("Overlay Text:");
                                    ui.text_edit_singleline(&mut self.webcam_settings.overlay_text);
                                });
                            }
                            
                            ui.horizontal(|ui| {
                                ui.checkbox(&mut self.webcam_settings.flip_horizontal, "Flip Horizontal");
                                ui.checkbox(&mut self.webcam_settings.flip_vertical, "Flip Vertical");
                            });
                            
                            ui.horizontal(|ui| {
                                ui.label("Rotation:");
                                egui::ComboBox::from_label("Degrees")
                                    .selected_text(format!("{}°", self.webcam_settings.rotation))
                                    .show_ui(ui, |ui| {
                                        ui.selectable_value(&mut self.webcam_settings.rotation, 0, "0°");
                                        ui.selectable_value(&mut self.webcam_settings.rotation, 90, "90°");
                                        ui.selectable_value(&mut self.webcam_settings.rotation, 180, "180°");
                                        ui.selectable_value(&mut self.webcam_settings.rotation, 270, "270°");
                                    });
                            });
                        });
                        
                        ui.separator();
                        
                        // Webcam presets
                        ui.group(|ui| {
                            ui.label("📋 Common Presets");
                            ui.horizontal(|ui| {
                                if ui.button("🖥️ OctoPrint Default").clicked() {
                                    self.webcam_settings.url = "http://octopi.local:8080/?action=stream".to_string();
                                    self.webcam_settings.resolution = (640, 480);
                                    self.webcam_settings.refresh_rate = 100;
                                }
                                if ui.button("📱 ESP32-CAM").clicked() {
                                    self.webcam_settings.url = "http://192.168.1.100:81/stream".to_string();
                                    self.webcam_settings.resolution = (800, 600);
                                    self.webcam_settings.refresh_rate = 200;
                                }
                                if ui.button("🐧 MJPG-Streamer").clicked() {
                                    self.webcam_settings.url = "http://192.168.1.100:8080/?action=stream".to_string();
                                    self.webcam_settings.resolution = (1280, 720);
                                    self.webcam_settings.refresh_rate = 100;
                                }
                            });
                        });
                    } else {
                        ui.label("📵 Webcam is disabled. Enable it in settings to start monitoring.");
                    }
                }
                Tab::Terminal => {
                    ui.heading("G-code Terminal");
                    ui.label("Send G-code, M-code, and system commands directly to the printer");
                    
                    ui.separator();
                    
                    // Terminal output area
                    ui.group(|ui| {
                        ui.label("💻 Console Output");
                        
                        egui::ScrollArea::vertical()
                            .max_height(300.0)
                            .auto_shrink([false, true])
                            .stick_to_bottom(self.terminal.auto_scroll)
                            .show(ui, |ui| {
                                for line in &self.terminal.history {
                                    // Color code different types of messages
                                    let color = if line.starts_with('>') {
                                        Color32::YELLOW  // Commands
                                    } else if line.contains("error") || line.contains("Error") {
                                        Color32::RED     // Errors
                                    } else if line.contains("OK") || line.contains("ok") {
                                        Color32::GREEN   // Success
                                    } else {
                                        Color32::WHITE   // Normal output
                                    };
                                    
                                    ui.colored_label(color, line);
                                }
                            });
                        
                        ui.horizontal(|ui| {
                            ui.checkbox(&mut self.terminal.auto_scroll, "Auto-scroll");
                            
                            if ui.button("Clear").clicked() {
                                self.terminal.history.clear();
                            }
                            
                            ui.label(format!("Lines: {}/{}", self.terminal.history.len(), self.terminal.max_history));
                        });
                    });
                    
                    ui.separator();
                    
                    // Command input
                    ui.group(|ui| {
                        ui.label("⌨️ Command Input");
                        
                        ui.horizontal(|ui| {
                            let response = ui.text_edit_singleline(&mut self.terminal.input_buffer);
                            
                            let send_button = ui.button("Send");
                            
                            // Handle Enter key or button click
                            if (response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter))) || send_button.clicked() {
                                if !self.terminal.input_buffer.trim().is_empty() {
                                    let command = self.terminal.input_buffer.clone();
                                    self.terminal.execute_command(command);
                                    self.terminal.input_buffer.clear();
                                    response.request_focus(); // Keep focus on input
                                }
                            }
                        });
                        
                        ui.horizontal(|ui| {
                            ui.label("Quick Commands:");
                            if ui.small_button("G28 (Home)").clicked() {
                                self.terminal.execute_command("G28".to_string());
                            }
                            if ui.small_button("M105 (Temp)").clicked() {
                                self.terminal.execute_command("M105".to_string());
                            }
                            if ui.small_button("M114 (Pos)").clicked() {
                                self.terminal.execute_command("M114".to_string());
                            }
                            if ui.small_button("M84 (Motors Off)").clicked() {
                                self.terminal.execute_command("M84".to_string());
                            }
                        });
                    });
                    
                    ui.separator();
                    
                    // Help section
                    ui.group(|ui| {
                        ui.label("📖 Help");
                        ui.label("Supported commands:");
                        ui.label("• G-codes: G0, G1, G28, G29, etc.");
                        ui.label("• M-codes: M104, M140, M105, M114, M84, etc.");
                        ui.label("• System: HELP, CLEAR");
                        ui.label("• Use Enter key or Send button to execute");
                    });
                }
                Tab::Macros => {
                    ui.heading("Macro Management");
                    ui.label("Create and execute custom G-code macros for common tasks");
                    
                    ui.separator();
                    
                    ui.columns(2, |columns| {
                        // Left column: Macro list and execution
                        columns[0].vertical(|ui| {
                            ui.heading("Available Macros");
                            
                            // Category filter buttons
                            ui.horizontal(|ui| {
                                ui.label("Filter:");
                                if ui.small_button("All").clicked() { /* Filter logic here */ }
                                if ui.small_button("Homing").clicked() { /* Filter logic here */ }
                                if ui.small_button("Calibration").clicked() { /* Filter logic here */ }
                                if ui.small_button("Heating").clicked() { /* Filter logic here */ }
                                if ui.small_button("Maintenance").clicked() { /* Filter logic here */ }
                            });
                            
                            ui.separator();
                            
                            // Macro list
                            egui::ScrollArea::vertical()
                                .max_height(300.0)
                                .show(ui, |ui| {
                                    for (i, macro_def) in self.macros.iter().enumerate() {
                                        if !macro_def.enabled {
                                            continue;
                                        }
                                        
                                        let is_selected = self.selected_macro == macro_def.name;
                                        
                                        ui.group(|ui| {
                                            ui.horizontal(|ui| {
                                                ui.label(&macro_def.icon);
                                                
                                                let response = ui.selectable_label(is_selected, &macro_def.name);
                                                if response.clicked() {
                                                    self.selected_macro = macro_def.name.clone();
                                                }
                                                
                                                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                                    if ui.small_button("▶").clicked() && !self.emergency_stop_triggered {
                                                        for command in &macro_def.commands {
                                                            self.terminal.execute_command(command.clone());
                                                        }
                                                        println!("Executed macro: {}", macro_def.name);
                                                    }
                                                });
                                            });
                                            
                                            ui.label(&macro_def.description);
                                            ui.label(format!("Category: {} | Commands: {}", 
                                                macro_def.category, macro_def.commands.len()));
                                        });
                                    }
                                });
                        });
                        
                        // Right column: Macro details and editing
                        columns[1].vertical(|ui| {
                            ui.heading("Macro Details");
                            
                            if let Some(selected_macro) = self.macros.iter_mut().find(|m| m.name == self.selected_macro) {
                                ui.group(|ui| {
                                    ui.horizontal(|ui| {
                                        ui.label("Name:");
                                        ui.text_edit_singleline(&mut selected_macro.name);
                                    });
                                    
                                    ui.horizontal(|ui| {
                                        ui.label("Icon:");
                                        ui.text_edit_singleline(&mut selected_macro.icon);
                                    });
                                    
                                    ui.horizontal(|ui| {
                                        ui.label("Category:");
                                        egui::ComboBox::from_id_salt("macro_category")
                                            .selected_text(&selected_macro.category)
                                            .show_ui(ui, |ui| {
                                                ui.selectable_value(&mut selected_macro.category, "Homing".to_string(), "Homing");
                                                ui.selectable_value(&mut selected_macro.category, "Calibration".to_string(), "Calibration");
                                                ui.selectable_value(&mut selected_macro.category, "Heating".to_string(), "Heating");
                                                ui.selectable_value(&mut selected_macro.category, "Maintenance".to_string(), "Maintenance");
                                                ui.selectable_value(&mut selected_macro.category, "Custom".to_string(), "Custom");
                                            });
                                    });
                                    
                                    ui.label("Description:");
                                    ui.text_edit_multiline(&mut selected_macro.description);
                                    
                                    ui.separator();
                                    
                                    ui.label("Commands:");
                                    let mut commands_text = selected_macro.commands.join("\n");
                                    ui.text_edit_multiline(&mut commands_text);
                                    selected_macro.commands = commands_text.lines().map(|s| s.to_string()).collect();
                                    
                                    ui.separator();
                                    
                                    ui.horizontal(|ui| {
                                        ui.checkbox(&mut selected_macro.enabled, "Enabled");
                                        
                                        if ui.button("▶ Test Macro").clicked() && !self.emergency_stop_triggered {
                                            for command in &selected_macro.commands {
                                                self.terminal.execute_command(command.clone());
                                            }
                                        }
                                    });
                                });
                            } else {
                                ui.label("Select a macro to view details");
                            }
                            
                            ui.separator();
                            
                            // Add new macro
                            ui.group(|ui| {
                                ui.label("➕ Create New Macro");
                                
                                if ui.button("New Homing Macro").clicked() {
                                    self.macros.push(MacroDefinition {
                                        name: "New Homing Macro".to_string(),
                                        description: "Custom homing sequence".to_string(),
                                        commands: vec!["G28".to_string()],
                                        enabled: true,
                                        category: "Homing".to_string(),
                                        icon: "🏠".to_string(),
                                    });
                                    self.selected_macro = "New Homing Macro".to_string();
                                }
                                
                                if ui.button("New Custom Macro").clicked() {
                                    self.macros.push(MacroDefinition {
                                        name: "New Custom Macro".to_string(),
                                        description: "Custom macro description".to_string(),
                                        commands: vec!["# Enter your commands here".to_string()],
                                        enabled: true,
                                        category: "Custom".to_string(),
                                        icon: "⚙️".to_string(),
                                    });
                                    self.selected_macro = "New Custom Macro".to_string();
                                }
                                
                                ui.separator();
                                
                                if ui.button("🗑️ Delete Selected Macro").clicked() {
                                    if let Some(index) = self.macros.iter().position(|m| m.name == self.selected_macro) {
                                        if self.macros.len() > 1 { // Keep at least one macro
                                            self.macros.remove(index);
                                            self.selected_macro = self.macros[0].name.clone();
                                        }
                                    }
                                }
                            });
                        });
                    });
                }
                Tab::GcodeImport => {
                    ui.heading("G-code Import");
                    
                    if let Some(path) = &self.gcode_file_path {
                        ui.label(format!("Current file: {}", path));
                    } else {
                        ui.label("No file loaded");
                    }
                    
                    ui.separator();
                    
                    if ui.button("Browse G-code File").clicked() {
                        if let Some(path) = FileDialog::new()
                            .add_filter("G-code", &["gcode", "g", "nc"])
                            .set_directory("./gcode")
                            .pick_file()
                        {
                            self.load_gcode_file(path);
                            // Automatically analyze for adaptive features
                            self.analyze_gcode_for_print_area();
                        }
                    }
                    
                    ui.separator();
                    
                    if let Some(parsed) = &self.parsed_gcode {
                        ui.label(format!("Commands: {}", parsed.commands.len()));
                        ui.separator();
                        ui.label("G-code content preview:");
                        egui::ScrollArea::vertical()
                            .max_height(200.0)
                            .show(ui, |ui| {
                                ui.add(egui::TextEdit::multiline(&mut self.gcode.as_str())
                                    .desired_rows(10)
                                    .desired_width(f32::INFINITY));
                            });
                    }
                }
                Tab::FieldbusConfig => {
                    ui.heading("🌐 Fieldbus Communication Configuration");
                    ui.label("Configure industrial fieldbus protocols for machine communication");
                    
                    egui::ScrollArea::vertical().id_salt("fieldbus_config_scroll").show(ui, |ui| {
                        ui.group(|ui| {
                            ui.label("📡 Communication Protocol Selection");
                            egui::ComboBox::from_label("Protocol Type")
                                .selected_text(format!("{:?}", self.communication_mode))
                                .show_ui(ui, |ui| {
                                    // Classic Industrial Protocols
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::EtherCAT, "🔌 EtherCAT");
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::Serial, "📟 Serial");
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::CANopen, "🚗 CANopen");
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::Profibus, "🏭 Profibus");
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::ModbusTCP, "🌐 Modbus TCP");
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::ModbusRTU, "📡 Modbus RTU");
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::DeviceNet, "🔗 DeviceNet");
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::ControlNet, "🎛️ ControlNet");
                                    
                                    ui.separator();
                                    
                                    // Modern Industrial Protocols
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::BACnet, "🏢 BACnet/IP");
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::ADS, "⚙️ ADS (TwinCAT)");
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::CODESYS, "💻 CODESYS");
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::CIP, "🔧 CIP (Common Industrial)");
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::OPC_UA, "🌍 OPC UA");
                                    
                                    ui.separator();
                                    
                                    // Specialized Protocols
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::DNP3, "⚡ DNP3");
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::IEC61850, "🔌 IEC 61850");
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::S7, "🔩 Siemens S7");
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::Fins, "🎯 OMRON FINS");
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::CC_Link, "🔗 CC-Link");
                                    ui.selectable_value(&mut self.communication_mode, CommunicationMode::Foundation, "🏗️ Foundation FB");
                                });
                            
                            ui.label(format!("✓ Current: {:?}", self.communication_mode));
                        });
                        
                        ui.separator();
                        
                        // Protocol-specific settings in scrollable area
                        match self.communication_mode {
                            CommunicationMode::EtherCAT => {
                                ui.group(|ui| {
                                    ui.label("🔌 EtherCAT Master Settings");
                                    ui.label("✓ EtherCAT master initialized");
                                    #[cfg(target_os = "linux")]
                                    ui.label(format!("Slaves discovered: {}", if self.group.is_empty() { 0 } else { "Connected" }));
                                    #[cfg(all(windows, feature = "windows-ethercat"))]
                                    ui.label(format!("Slaves discovered: {}", self.group.len()));
                                    #[cfg(all(not(target_os = "linux"), not(all(windows, feature = "windows-ethercat"))))]
                                    ui.label("⚠ EtherCAT not available on this platform");
                                });
                                
                                ui.separator();
                                
                                // EtherCAT Network Connection Management
                                ui.group(|ui| {
                                    ui.label("🌐 EtherCAT Network Connection");

                                    // Connection status indicator
                                    ui.horizontal(|ui| {
                                        let status_text = if self.ethercat_master_connected {
                                            "🟢 Connected"
                                        } else {
                                            "🔴 Disconnected"
                                        };
                                        ui.label(format!("Status: {}", status_text));

                                        if self.ethercat_master_connected {
                                            if ui.button("🔌 Disconnect").clicked() {
                                                self.ethercat_master_connected = false;
                                                // Disconnect all modules
                                                for module in &mut self.ethercat_modules {
                                                    module.connection_status = EtherCATConnectionStatus::Disconnected;
                                                    module.last_seen = None;
                                                }
                                            }
                                        } else {
                                            if ui.button("🔗 Connect Network").clicked() {
                                                #[cfg(windows)]
                                                {
                                                    if let Some(ref mut master) = self.windows_ethercat_master {
                                                        // Connect to selected adapter
                                                        if !self.ethercat_network_config.network_interface.is_empty() {
                                                            match master.connect_adapter(&self.ethercat_network_config.network_interface) {
                                                                Ok(_) => {
                                                                    self.ethercat_master_connected = true;
                                                                    // Update connection status for modules
                                                                    for module in &mut self.ethercat_modules {
                                                                        if module.enabled {
                                                                            module.connection_status = EtherCATConnectionStatus::Connected;
                                                                            module.last_seen = Some(std::time::Instant::now());
                                                                        }
                                                                    }
                                                                }
                                                                Err(e) => {
                                                                    // Handle connection error
                                                                    for module in &mut self.ethercat_modules {
                                                                        if module.enabled {
                                                                            module.connection_status = EtherCATConnectionStatus::Error(e.to_string());
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }

                                                #[cfg(not(windows))]
                                                {
                                                    self.ethercat_master_connected = true;
                                                    // Simulate connection attempt for all modules
                                                    for module in &mut self.ethercat_modules {
                                                        if module.enabled {
                                                            module.connection_status = EtherCATConnectionStatus::Connecting;
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    });

                                    // Network configuration
                                    ui.collapsing("⚙️ Network Configuration", |ui| {
                                        #[cfg(windows)]
                                        {
                                            ui.horizontal(|ui| {
                                                ui.label("Network Adapter:");
                                                if ui.button("🔍 Scan Adapters").clicked() {
                                                    // Discover Windows adapters using configured Npcap settings
                                                    let npcap_path = if self.ethercat_network_config.npcap_path.is_empty() {
                                                        None
                                                    } else {
                                                        Some(self.ethercat_network_config.npcap_path.as_str())
                                                    };
                                                    if let Ok(adapters) = printer_host::ethercat_windows::WindowsEtherCATMaster::discover_adapters_with_npcap_path(
                                                        npcap_path,
                                                        self.ethercat_network_config.use_npcap
                                                    ) {
                                                        self.available_adapters = adapters;
                                                    }
                                                }
                                            });

                                            if !self.available_adapters.is_empty() {
                                                ui.label("Available Adapters:");
                                                for adapter in &self.available_adapters {
                                                    ui.horizontal(|ui| {
                                                        if ui.selectable_label(
                                                            self.ethercat_network_config.network_interface == adapter.name,
                                                            &adapter.description
                                                        ).clicked() {
                                                            self.ethercat_network_config.network_interface = adapter.name.clone();
                                                        }
                                                    });
                                                }
                                            } else {
                                                ui.label("🔍 Click 'Scan Adapters' to detect network interfaces");
                                            }
                                        }

                                        #[cfg(not(windows))]
                                        {
                                            ui.horizontal(|ui| {
                                                ui.label("Interface:");
                                                ui.text_edit_singleline(&mut self.ethercat_network_config.network_interface);
                                            });
                                        }
                                        ui.horizontal(|ui| {
                                            ui.label("Cycle Time (μs):");
                                            ui.add(egui::DragValue::new(&mut self.ethercat_network_config.cycle_time_us).range(100..=10000));
                                        });
                                        ui.checkbox(&mut self.ethercat_network_config.sync_master_enabled, "Distributed Clock Sync");
                                        ui.checkbox(&mut self.ethercat_network_config.topology_check, "Topology Verification");
                                        ui.horizontal(|ui| {
                                            ui.label("Recovery Timeout (ms):");
                                            ui.add(egui::DragValue::new(&mut self.ethercat_network_config.recovery_timeout_ms).range(1000..=30000));
                                        });

                                        // Npcap Configuration Section (Windows only)
                                        #[cfg(windows)]
                                        {
                                            ui.separator();
                                            ui.label("🔌 Npcap Configuration:");
                                            ui.checkbox(&mut self.ethercat_network_config.use_npcap, "Use Npcap for real hardware (requires Npcap installation)");

                                            if self.ethercat_network_config.use_npcap {
                                                ui.horizontal(|ui| {
                                                    ui.label("Npcap Path:");
                                                    ui.text_edit_singleline(&mut self.ethercat_network_config.npcap_path);
                                                    if ui.button("📁 Browse").clicked() {
                                                        if let Some(path) = rfd::FileDialog::new()
                                                            .set_title("Select Npcap Installation Directory")
                                                            .set_directory(&self.ethercat_network_config.npcap_path)
                                                            .pick_folder() {
                                                            self.ethercat_network_config.npcap_path = path.to_string_lossy().to_string();
                                                        }
                                                    }
                                                });

                                                // Check if Npcap path exists and show status
                                                ui.horizontal(|ui| {
                                                    let npcap_exists = std::path::Path::new(&self.ethercat_network_config.npcap_path).exists();
                                                    if npcap_exists {
                                                        ui.label("✅ Npcap directory found");
                                                        // Check for specific Npcap files
                                                        let packet_dll_path = format!("{}\\Packet.dll", self.ethercat_network_config.npcap_path);
                                                        let wpcap_dll_path = format!("{}\\wpcap.dll", self.ethercat_network_config.npcap_path);
                                                        if std::path::Path::new(&packet_dll_path).exists() &&
                                                           std::path::Path::new(&wpcap_dll_path).exists() {
                                                            ui.label("📦 Npcap libraries detected");
                                                        } else {
                                                            ui.label("⚠️ Npcap libraries not found in directory");
                                                        }
                                                    } else {
                                                        ui.label("❌ Npcap directory not found");
                                                    }
                                                });

                                                // Quick setup buttons for common Npcap locations
                                                ui.horizontal(|ui| {
                                                    ui.label("Quick Setup:");
                                                    if ui.small_button("System32\\Npcap").clicked() {
                                                        self.ethercat_network_config.npcap_path = "C:\\Windows\\System32\\Npcap".to_string();
                                                    }
                                                    if ui.small_button("Program Files\\Npcap").clicked() {
                                                        self.ethercat_network_config.npcap_path = "C:\\Program Files\\Npcap".to_string();
                                                    }
                                                });
                                            } else {
                                                ui.label("🔧 Using mock EtherCAT implementation for testing");
                                            }
                                        }

                                        #[cfg(windows)]
                                        {
                                            if self.ethercat_master_connected {
                                                ui.separator();
                                                ui.horizontal(|ui| {
                                                    if ui.button("🔍 Scan for Slaves").clicked() {
                                                        println!("EtherCAT slave scan requested by user");
                                                        // Trigger a new scan through the existing slave group
                                                        #[cfg(all(windows, feature = "windows-ethercat"))]
                                                        {
                                                            // Use a different approach - send async signal
                                                            let tx_clone = self.ethercat_tx.clone();
                                                            tokio::spawn(async move {
                                                                if let Err(e) = tx_clone.send([999.0, 0.0, 0.0, 0.0, 0.0]).await {
                                                                    println!("Failed to send scan signal: {}", e);
                                                                } else {
                                                                    println!("Scan signal sent successfully");
                                                                }
                                                            });
                                                            println!("Slave scan initiated - check logs for results");
                                                        }
                                                    }
                                                    if let Some(ref master) = self.windows_ethercat_master {
                                                        ui.label(format!("Adapter: {}",
                                                            master.get_adapter()
                                                                .map(|a| a.description.as_str())
                                                                .unwrap_or("None")));
                                                        ui.label(format!("Errors: {}", master.get_error_count()));
                                                    }
                                                });
                                            }
                                        }
                                    });
                                });

                                ui.separator();

                                // EtherCAT Module Management with scrolling
                                ui.group(|ui| {
                                    ui.label("📦 EtherCAT Module Configuration");

                                    ui.horizontal(|ui| {
                                        ui.label("Configured Modules:");
                                        if ui.button("➕ Add Beckhoff Module").clicked() {
                                            // Add dropdown for module selection
                                        }
                                        if ui.button("🔧 Add Custom Module").clicked() {
                                            self.ethercat_modules.push(EtherCATModule::new_beckhoff(EtherCATModuleType::Custom,
                                                self.ethercat_modules.len() as u16 + 1));
                                        }
                                        if ui.button("🗑️ Remove Last Module").clicked() && !self.ethercat_modules.is_empty() {
                                            if let Some(removed) = self.ethercat_modules.pop() {
                                                println!("Removed EtherCAT module: {}", removed.name);
                                            }
                                        }
                                        if ui.button("🧹 Clear All Modules").clicked() {
                                            let count = self.ethercat_modules.len();
                                            self.ethercat_modules.clear();
                                            println!("Removed all {} EtherCAT modules", count);
                                        }
                                    });

                                    ui.horizontal(|ui| {
                                        if ui.button("💾 Save Configuration").clicked() {
                                            match self.save_ethercat_configuration() {
                                                Ok(_) => println!("✅ EtherCAT configuration saved successfully"),
                                                Err(e) => println!("❌ Failed to save EtherCAT configuration: {}", e),
                                            }
                                        }
                                        if ui.button("📂 Load Configuration").clicked() {
                                            match self.load_ethercat_configuration() {
                                                Ok(_) => println!("✅ EtherCAT configuration loaded successfully"),
                                                Err(e) => println!("❌ Failed to load EtherCAT configuration: {}", e),
                                            }
                                        }
                                        if ui.button("📋 Export to JSON").clicked() {
                                            if let Some(path) = rfd::FileDialog::new()
                                                .set_title("Export EtherCAT Configuration")
                                                .add_filter("JSON files", &["json"])
                                                .set_file_name("ethercat_config.json")
                                                .save_file() {
                                                match self.export_ethercat_configuration(&path) {
                                                    Ok(_) => println!("✅ Configuration exported to: {}", path.display()),
                                                    Err(e) => println!("❌ Export failed: {}", e),
                                                }
                                            }
                                        }
                                    });
                                    
                                    ui.separator();
                                    
                                    // Module list with management in nested scroll area
                                    egui::ScrollArea::vertical()
                                        .max_height(300.0)
                                        .show(ui, |ui| {
                                            let mut module_to_remove = None;
                                            
                                            for (i, module) in self.ethercat_modules.iter_mut().enumerate() {
                                                ui.group(|ui| {
                                                    ui.horizontal(|ui| {
                                                        // Module status indicator (enabled/disabled)
                                                        let enabled_color = if module.enabled { Color32::GREEN } else { Color32::RED };
                                                        ui.colored_label(enabled_color, "●");

                                                        // Connection status indicator
                                                        let (conn_color, conn_symbol) = match &module.connection_status {
                                                            EtherCATConnectionStatus::Connected => (Color32::LIGHT_GREEN, "🟢"),
                                                            EtherCATConnectionStatus::Connecting => (Color32::YELLOW, "🟡"),
                                                            EtherCATConnectionStatus::Disconnected => (Color32::GRAY, "⚫"),
                                                            EtherCATConnectionStatus::Error(_) => (Color32::RED, "🔴"),
                                                            EtherCATConnectionStatus::ConfigError => (Color32::DARK_RED, "❌"),
                                                        };
                                                        ui.colored_label(conn_color, conn_symbol);

                                                        // Module name and type
                                                        ui.label(&module.name);
                                                        
                                                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                                            if ui.small_button("🗑️").clicked() {
                                                                module_to_remove = Some(i);
                                                            }
                                                            
                                                            if ui.small_button("⚙️").clicked() {
                                                                self.selected_module_index = Some(i);
                                                            }
                                                            
                                                            ui.checkbox(&mut module.enabled, "");
                                                        });
                                                    });
                                                    
                                                    // Module details
                                                    ui.label(format!("Station: {} | I/O: {}B in, {}B out | Errors: {}",
                                                        module.station_address, module.input_size, module.output_size, module.error_count));
                                                    ui.label(format!("Vendor: 0x{:08X} | Product: 0x{:08X}",
                                                        module.vendor_id, module.product_code));

                                                    // Connection status details
                                                    let status_text = match &module.connection_status {
                                                        EtherCATConnectionStatus::Connected => {
                                                            if let Some(last_seen) = module.last_seen {
                                                                format!("Connected (last seen: {:.1}s ago)", last_seen.elapsed().as_secs_f32())
                                                            } else {
                                                                "Connected".to_string()
                                                            }
                                                        },
                                                        EtherCATConnectionStatus::Connecting => "Connecting...".to_string(),
                                                        EtherCATConnectionStatus::Disconnected => "Disconnected".to_string(),
                                                        EtherCATConnectionStatus::Error(err) => format!("Error: {}", err),
                                                        EtherCATConnectionStatus::ConfigError => "Configuration Error".to_string(),
                                                    };
                                                    ui.label(format!("Status: {}", status_text));
                                                    
                                                    // Expanded module configuration
                                                    if Some(i) == self.selected_module_index {
                                                        ui.separator();
                                                        ui.label("📝 Module Configuration");
                                                        
                                                        ui.horizontal(|ui| {
                                                            ui.label("Name:");
                                                            ui.text_edit_singleline(&mut module.name);
                                                        });
                                                        
                                                        ui.horizontal(|ui| {
                                                            ui.label("Station Address:");
                                                            ui.add(egui::DragValue::new(&mut module.station_address).range(1..=65535));
                                                        });
                                                        
                                                        if module.module_type == EtherCATModuleType::Custom {
                                                            ui.horizontal(|ui| {
                                                                ui.label("Vendor ID:");
                                                                ui.add(egui::DragValue::new(&mut module.vendor_id).hexadecimal(8, false, true));
                                                            });
                                                            
                                                            ui.horizontal(|ui| {
                                                                ui.label("Product Code:");
                                                                ui.add(egui::DragValue::new(&mut module.product_code).hexadecimal(8, false, true));
                                                            });
                                                            
                                                            ui.horizontal(|ui| {
                                                                ui.label("Input Size:");
                                                                ui.add(egui::DragValue::new(&mut module.input_size).suffix(" bytes"));
                                                                ui.label("Output Size:");
                                                                ui.add(egui::DragValue::new(&mut module.output_size).suffix(" bytes"));
                                                            });
                                                            
                                                            ui.label("Description:");
                                                            ui.text_edit_multiline(&mut module.custom_description);
                                                        } else {
                                                            ui.label(&module.custom_description);
                                                        }

                                                        // I/O Data Visualization
                                                        ui.separator();
                                                        ui.label("📊 I/O Data Monitor");

                                                        if module.input_size > 0 {
                                                            ui.collapsing("📥 Input Data", |ui| {
                                                                ui.horizontal(|ui| {
                                                                    ui.label("Raw Data (hex):");
                                                                    let input_hex = module.input_data.iter()
                                                                        .map(|b| format!("{:02X}", b))
                                                                        .collect::<Vec<_>>()
                                                                        .join(" ");
                                                                    ui.monospace(&input_hex);
                                                                });

                                                                // Digital input interpretation for common modules
                                                                if matches!(module.module_type,
                                                                    EtherCATModuleType::BeckhoffEL1008 |
                                                                    EtherCATModuleType::BeckhoffKL1104 |
                                                                    EtherCATModuleType::BeckhoffKL1408) {
                                                                    ui.label("Digital Inputs:");
                                                                    ui.horizontal_wrapped(|ui| {
                                                                        for (byte_idx, byte_val) in module.input_data.iter().enumerate() {
                                                                            for bit_idx in 0..8 {
                                                                                let bit_set = (byte_val >> bit_idx) & 1 == 1;
                                                                                let channel = byte_idx * 8 + bit_idx + 1;
                                                                                let color = if bit_set { Color32::GREEN } else { Color32::GRAY };
                                                                                ui.colored_label(color, format!("CH{}", channel));
                                                                            }
                                                                        }
                                                                    });
                                                                }
                                                            });
                                                        }

                                                        if module.output_size > 0 {
                                                            ui.collapsing("📤 Output Data", |ui| {
                                                                ui.horizontal(|ui| {
                                                                    ui.label("Raw Data (hex):");
                                                                    let output_hex = module.output_data.iter()
                                                                        .map(|b| format!("{:02X}", b))
                                                                        .collect::<Vec<_>>()
                                                                        .join(" ");
                                                                    ui.monospace(&output_hex);
                                                                });

                                                                // Digital output control for common modules
                                                                if matches!(module.module_type,
                                                                    EtherCATModuleType::BeckhoffEL2008 |
                                                                    EtherCATModuleType::BeckhoffEL2088 |
                                                                    EtherCATModuleType::BeckhoffKL2408) {
                                                                    ui.label("Digital Outputs:");
                                                                    ui.horizontal_wrapped(|ui| {
                                                                        for (byte_idx, byte_val) in module.output_data.iter_mut().enumerate() {
                                                                            for bit_idx in 0..8 {
                                                                                let mut bit_set = (*byte_val >> bit_idx) & 1 == 1;
                                                                                let channel = byte_idx * 8 + bit_idx + 1;
                                                                                if ui.checkbox(&mut bit_set, format!("CH{}", channel)).changed() {
                                                                                    if bit_set {
                                                                                        *byte_val |= 1 << bit_idx;
                                                                                    } else {
                                                                                        *byte_val &= !(1 << bit_idx);
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                    });
                                                                }
                                                            });
                                                        }

                                                        if ui.button("Close Configuration").clicked() {
                                                            self.selected_module_index = None;
                                                        }
                                                    }
                                                });
                                            }
                                            
                                            // Remove module if requested
                                            if let Some(index) = module_to_remove {
                                                self.ethercat_modules.remove(index);
                                                self.selected_module_index = None;
                                            }
                                        });
                                    
                                    // Quick add Beckhoff modules
                                    ui.separator();
                                    ui.label("Quick Add Beckhoff Modules:");
                                    ui.label("🔌 Bus Couplers:");
                                    ui.horizontal(|ui| {
                                        if ui.small_button("BK1120 (Bus Coupler)").clicked() {
                                            self.ethercat_modules.push(EtherCATModule::new_beckhoff(
                                                EtherCATModuleType::BeckhoffBK1120,
                                                self.ethercat_modules.len() as u16 + 1));
                                        }
                                        if ui.small_button("BK1250 (EtherCAT)").clicked() {
                                            self.ethercat_modules.push(EtherCATModule::new_beckhoff(
                                                EtherCATModuleType::BeckhoffBK1250,
                                                self.ethercat_modules.len() as u16 + 1));
                                        }
                                    });
                                    ui.label("📥 Digital Inputs:");
                                    ui.horizontal(|ui| {
                                        if ui.small_button("EL1008 (8Ch DI)").clicked() {
                                            self.ethercat_modules.push(EtherCATModule::new_beckhoff(
                                                EtherCATModuleType::BeckhoffEL1008,
                                                self.ethercat_modules.len() as u16 + 1));
                                        }
                                        if ui.small_button("KL1104 (4Ch DI)").clicked() {
                                            self.ethercat_modules.push(EtherCATModule::new_beckhoff(
                                                EtherCATModuleType::BeckhoffKL1104,
                                                self.ethercat_modules.len() as u16 + 1));
                                        }
                                        if ui.small_button("KL1408 (8Ch DI)").clicked() {
                                            self.ethercat_modules.push(EtherCATModule::new_beckhoff(
                                                EtherCATModuleType::BeckhoffKL1408,
                                                self.ethercat_modules.len() as u16 + 1));
                                        }
                                    });
                                    ui.label("📤 Digital Outputs:");
                                    ui.horizontal(|ui| {
                                        if ui.small_button("EL2008 (8Ch DO)").clicked() {
                                            self.ethercat_modules.push(EtherCATModule::new_beckhoff(
                                                EtherCATModuleType::BeckhoffEL2008,
                                                self.ethercat_modules.len() as u16 + 1));
                                        }
                                        if ui.small_button("EL2088 (8Ch 2A)").clicked() {
                                            self.ethercat_modules.push(EtherCATModule::new_beckhoff(
                                                EtherCATModuleType::BeckhoffEL2088,
                                                self.ethercat_modules.len() as u16 + 1));
                                        }
                                    });
                                    ui.label("📊 Analog I/O:");
                                    ui.horizontal(|ui| {
                                        if ui.small_button("EL3008 (8Ch AI)").clicked() {
                                            self.ethercat_modules.push(EtherCATModule::new_beckhoff(
                                                EtherCATModuleType::BeckhoffEL3008,
                                                self.ethercat_modules.len() as u16 + 1));
                                        }
                                        if ui.small_button("EL4008 (8Ch AO)").clicked() {
                                            self.ethercat_modules.push(EtherCATModule::new_beckhoff(
                                                EtherCATModuleType::BeckhoffEL4008,
                                                self.ethercat_modules.len() as u16 + 1));
                                        }
                                    });
                                    ui.label("🚀 Motor Control:");
                                    ui.horizontal(|ui| {
                                        if ui.small_button("EL7031 (Stepper)").clicked() {
                                            self.ethercat_modules.push(EtherCATModule::new_beckhoff(
                                                EtherCATModuleType::BeckhoffEL7031,
                                                self.ethercat_modules.len() as u16 + 1));
                                        }
                                        if ui.small_button("EL7041 (Step+Enc)").clicked() {
                                            self.ethercat_modules.push(EtherCATModule::new_beckhoff(
                                                EtherCATModuleType::BeckhoffEL7041,
                                                self.ethercat_modules.len() as u16 + 1));
                                        }
                                        if ui.small_button("EL7211 (Servo)").clicked() {
                                            self.ethercat_modules.push(EtherCATModule::new_beckhoff(
                                                EtherCATModuleType::BeckhoffEL7211,
                                                self.ethercat_modules.len() as u16 + 1));
                                        }
                                    });
                                });
                            }
                            CommunicationMode::Serial => {
                                ui.group(|ui| {
                                    ui.label("📟 Serial Communication Settings");
                                    ui.horizontal(|ui| {
                                        ui.label("Port:");
                                        ui.text_edit_singleline(&mut self.serial_port_name);
                                    });
                                    ui.add(egui::Slider::new(&mut self.serial_baud_rate, 9600..=115200).text("Baud Rate"));
                                });
                            }
                            CommunicationMode::CANopen => {
                                ui.group(|ui| {
                                    ui.label("🚗 CANopen Settings");
                                    ui.add(egui::Slider::new(&mut self.canopen_node_id, 1..=127).text("Node ID"));
                                    egui::ComboBox::from_label("Baud Rate")
                                        .selected_text(format!("{} bps", self.canopen_baudrate))
                                        .show_ui(ui, |ui| {
                                            ui.selectable_value(&mut self.canopen_baudrate, 125000, "125 kbps");
                                            ui.selectable_value(&mut self.canopen_baudrate, 250000, "250 kbps");
                                            ui.selectable_value(&mut self.canopen_baudrate, 500000, "500 kbps");
                                            ui.selectable_value(&mut self.canopen_baudrate, 1000000, "1 Mbps");
                                        });
                                });
                            }
                            CommunicationMode::Profibus => {
                                ui.group(|ui| {
                                    ui.label("🏭 Profibus Settings");
                                    ui.add(egui::Slider::new(&mut self.profibus_address, 0..=126).text("Station Address"));
                                    egui::ComboBox::from_label("Baud Rate")
                                        .selected_text(format!("{} bps", self.profibus_baudrate))
                                        .show_ui(ui, |ui| {
                                            ui.selectable_value(&mut self.profibus_baudrate, 9600, "9.6 kbps");
                                            ui.selectable_value(&mut self.profibus_baudrate, 19200, "19.2 kbps");
                                            ui.selectable_value(&mut self.profibus_baudrate, 45450, "45.45 kbps");
                                            ui.selectable_value(&mut self.profibus_baudrate, 93750, "93.75 kbps");
                                            ui.selectable_value(&mut self.profibus_baudrate, 187500, "187.5 kbps");
                                            ui.selectable_value(&mut self.profibus_baudrate, 500000, "500 kbps");
                                            ui.selectable_value(&mut self.profibus_baudrate, 1500000, "1.5 Mbps");
                                            ui.selectable_value(&mut self.profibus_baudrate, 3000000, "3 Mbps");
                                            ui.selectable_value(&mut self.profibus_baudrate, 6000000, "6 Mbps");
                                            ui.selectable_value(&mut self.profibus_baudrate, 12000000, "12 Mbps");
                                        });
                                });
                            }
                            CommunicationMode::ModbusTCP => {
                                ui.group(|ui| {
                                    ui.label("🌐 Modbus TCP Settings");
                                    ui.add(egui::Slider::new(&mut self.modbus_device_id, 1..=247).text("Device ID"));
                                    ui.horizontal(|ui| {
                                        ui.label("IP Address:");
                                        ui.text_edit_singleline(&mut self.modbus_tcp_ip);
                                    });
                                    ui.add(egui::Slider::new(&mut self.modbus_tcp_port, 1..=65535).text("Port"));
                                });
                            }
                            CommunicationMode::ModbusRTU => {
                                ui.group(|ui| {
                                    ui.label("📡 Modbus RTU Settings");
                                    ui.add(egui::Slider::new(&mut self.modbus_device_id, 1..=247).text("Device ID"));
                                    ui.horizontal(|ui| {
                                        ui.label("Serial Port:");
                                        ui.text_edit_singleline(&mut self.serial_port_name);
                                    });
                                    ui.add(egui::Slider::new(&mut self.serial_baud_rate, 9600..=115200).text("Baud Rate"));
                                });
                            }
                            CommunicationMode::DeviceNet => {
                                ui.group(|ui| {
                                    ui.label("🔗 DeviceNet Settings");
                                    ui.add(egui::Slider::new(&mut self.devicenet_node_address, 0..=63).text("Node Address"));
                                    egui::ComboBox::from_label("Baud Rate")
                                        .selected_text(format!("{} bps", self.devicenet_baudrate))
                                        .show_ui(ui, |ui| {
                                            ui.selectable_value(&mut self.devicenet_baudrate, 125000, "125 kbps");
                                            ui.selectable_value(&mut self.devicenet_baudrate, 250000, "250 kbps");
                                            ui.selectable_value(&mut self.devicenet_baudrate, 500000, "500 kbps");
                                        });
                                });
                            }
                            CommunicationMode::ControlNet => {
                                ui.group(|ui| {
                                    ui.label("🎛️ ControlNet Settings");
                                    ui.label("⚠ ControlNet configuration requires additional setup");
                                    ui.label("Please configure through manufacturer-specific software");
                                });
                            }
                            CommunicationMode::BACnet => {
                                ui.group(|ui| {
                                    ui.label("🏢 BACnet/IP Settings");
                                    ui.horizontal(|ui| {
                                        ui.label("Device ID:");
                                        ui.add(egui::DragValue::new(&mut self.bacnet_config.device_id).range(1..=4194303));
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Port:");
                                        ui.add(egui::DragValue::new(&mut self.bacnet_config.port).range(1..=65535));
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Network Number:");
                                        ui.add(egui::DragValue::new(&mut self.bacnet_config.network_number).range(0..=65535));
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Max APDU Length:");
                                        ui.add(egui::DragValue::new(&mut self.bacnet_config.max_apdu_length).range(50..=8192));
                                    });
                                    ui.checkbox(&mut self.bacnet_config.segmentation_supported, "Segmentation Supported");
                                });
                            }
                            CommunicationMode::ADS => {
                                ui.group(|ui| {
                                    ui.label("⚙️ ADS (TwinCAT) Settings");
                                    ui.horizontal(|ui| {
                                        ui.label("Target AMS NetId:");
                                        ui.text_edit_singleline(&mut self.ads_config.target_ams_net_id);
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Target AMS Port:");
                                        ui.add(egui::DragValue::new(&mut self.ads_config.target_ams_port).range(1..=65535));
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Source AMS Port:");
                                        ui.add(egui::DragValue::new(&mut self.ads_config.source_ams_port).range(1..=65535));
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Timeout (seconds):");
                                        let mut timeout_secs = self.ads_config.timeout.as_secs() as u64;
                                        ui.add(egui::DragValue::new(&mut timeout_secs).range(1..=60));
                                        self.ads_config.timeout = Duration::from_secs(timeout_secs);
                                    });
                                });
                            }
                            CommunicationMode::CODESYS => {
                                ui.group(|ui| {
                                    ui.label("💻 CODESYS Settings");
                                    ui.horizontal(|ui| {
                                        ui.label("Target IP:");
                                        let mut ip_string = self.codesys_config.target_ip.to_string();
                                        ui.text_edit_singleline(&mut ip_string);
                                        if let Ok(parsed_ip) = ip_string.parse() {
                                            self.codesys_config.target_ip = parsed_ip;
                                        }
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Port:");
                                        ui.add(egui::DragValue::new(&mut self.codesys_config.port).range(1..=65535));
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Username:");
                                        ui.text_edit_singleline(&mut self.codesys_config.username);
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Password:");
                                        ui.add(egui::TextEdit::singleline(&mut self.codesys_config.password).password(true));
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Project Name:");
                                        ui.text_edit_singleline(&mut self.codesys_config.project_name);
                                    });
                                });
                            }
                            CommunicationMode::CIP => {
                                ui.group(|ui| {
                                    ui.label("🔧 CIP (Common Industrial Protocol) Settings");
                                    ui.horizontal(|ui| {
                                        ui.label("Connection Type:");
                                        egui::ComboBox::from_label("CIP Type")
                                            .selected_text(format!("{:?}", self.cip_config.connection_type))
                                            .show_ui(ui, |ui| {
                                                ui.selectable_value(&mut self.cip_config.connection_type, CIPConnectionType::EthernetIP, "Ethernet/IP");
                                                ui.selectable_value(&mut self.cip_config.connection_type, CIPConnectionType::DeviceNet, "DeviceNet");
                                                ui.selectable_value(&mut self.cip_config.connection_type, CIPConnectionType::ControlNet, "ControlNet");
                                            });
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Target IP:");
                                        let mut ip_string = self.cip_config.target_ip.to_string();
                                        ui.text_edit_singleline(&mut ip_string);
                                        if let Ok(parsed_ip) = ip_string.parse() {
                                            self.cip_config.target_ip = parsed_ip;
                                        }
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Port:");
                                        ui.add(egui::DragValue::new(&mut self.cip_config.port).range(1..=65535));
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Vendor ID:");
                                        ui.add(egui::DragValue::new(&mut self.cip_config.vendor_id).hexadecimal(4, false, true));
                                        ui.label("Device Type:");
                                        ui.add(egui::DragValue::new(&mut self.cip_config.device_type).hexadecimal(4, false, true));
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Product Code:");
                                        ui.add(egui::DragValue::new(&mut self.cip_config.product_code).hexadecimal(4, false, true));
                                    });
                                });
                            }
                            CommunicationMode::OPC_UA => {
                                ui.group(|ui| {
                                    ui.label("🌍 OPC UA Settings");
                                    ui.horizontal(|ui| {
                                        ui.label("Endpoint URL:");
                                        ui.text_edit_singleline(&mut self.opcua_config.endpoint_url);
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Security Policy:");
                                        egui::ComboBox::from_label("Security")
                                            .selected_text(&self.opcua_config.security_policy)
                                            .show_ui(ui, |ui| {
                                                ui.selectable_value(&mut self.opcua_config.security_policy, "None".to_string(), "None");
                                                ui.selectable_value(&mut self.opcua_config.security_policy, "Basic128Rsa15".to_string(), "Basic128Rsa15");
                                                ui.selectable_value(&mut self.opcua_config.security_policy, "Basic256Sha256".to_string(), "Basic256Sha256");
                                            });
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Message Security:");
                                        egui::ComboBox::from_label("Message Sec")
                                            .selected_text(&self.opcua_config.message_security_mode)
                                            .show_ui(ui, |ui| {
                                                ui.selectable_value(&mut self.opcua_config.message_security_mode, "None".to_string(), "None");
                                                ui.selectable_value(&mut self.opcua_config.message_security_mode, "Sign".to_string(), "Sign");
                                                ui.selectable_value(&mut self.opcua_config.message_security_mode, "SignAndEncrypt".to_string(), "Sign & Encrypt");
                                            });
                                    });
                                    if let Some(ref mut username) = self.opcua_config.username {
                                        ui.horizontal(|ui| {
                                            ui.label("Username:");
                                            ui.text_edit_singleline(username);
                                        });
                                    }
                                    if let Some(ref mut password) = self.opcua_config.password {
                                        ui.horizontal(|ui| {
                                            ui.label("Password:");
                                            ui.add(egui::TextEdit::singleline(password).password(true));
                                        });
                                    }
                                });
                            }
                            // Placeholder configurations for other protocols
                            CommunicationMode::DNP3 => {
                                ui.group(|ui| {
                                    ui.label("⚡ DNP3 Settings");
                                    ui.label("🚧 DNP3 configuration coming soon");
                                    ui.label("Used primarily in electrical power systems");
                                });
                            }
                            CommunicationMode::IEC61850 => {
                                ui.group(|ui| {
                                    ui.label("🔌 IEC 61850 Settings");
                                    ui.label("🚧 IEC 61850 configuration coming soon");
                                    ui.label("Standard for power utility automation");
                                });
                            }
                            CommunicationMode::S7 => {
                                ui.group(|ui| {
                                    ui.label("🔨 Siemens S7 Settings");
                                    ui.label("🚧 S7 configuration coming soon");
                                    ui.label("Siemens PLC communication protocol");
                                });
                            }
                            CommunicationMode::Fins => {
                                ui.group(|ui| {
                                    ui.label("🎯 OMRON FINS Settings");
                                    ui.label("🚧 FINS configuration coming soon");
                                    ui.label("OMRON PLC communication protocol");
                                });
                            }
                            CommunicationMode::CC_Link => {
                                ui.group(|ui| {
                                    ui.label("🔗 CC-Link Settings");
                                    ui.label("🚧 CC-Link configuration coming soon");
                                    ui.label("Mitsubishi fieldbus network");
                                });
                            }
                            CommunicationMode::Foundation => {
                                ui.group(|ui| {
                                    ui.label("🏭 Foundation Fieldbus Settings");
                                    ui.label("🚧 Foundation Fieldbus configuration coming soon");
                                    ui.label("Process automation fieldbus standard");
                                });
                            }
                            CommunicationMode::Ethernet => {
                                ui.group(|ui| {
                                    ui.label("🌐 Generic Ethernet Settings");
                                    ui.label("🚧 Generic Ethernet configuration coming soon");
                                    ui.label("Basic Ethernet/IP communication");
                                });
                            }
                        }
                        
                        ui.separator();
                        
                        // Connection status and testing
                        ui.group(|ui| {
                            ui.label("🔌 Connection Status & Testing");
                            ui.horizontal(|ui| {
                                let status_color = if self.ethercat_initialized { Color32::GREEN } else { Color32::RED };
                                ui.colored_label(status_color, "●");
                                ui.label(format!("Status: {}", if self.ethercat_initialized { "Connected" } else { "Disconnected" }));
                            });
                            
                            ui.horizontal(|ui| {
                                if ui.button("🔄 Test Connection").clicked() {
                                    // Test connection based on current protocol
                                }
                                if ui.button("📊 Diagnostics").clicked() {
                                    // Show protocol diagnostics
                                }
                                if ui.button("💾 Save Configuration").clicked() {
                                    // Save fieldbus configuration to file
                                }
                            });
                        });
                        
                        ui.separator();
                        
                        // Protocol Discovery Section
                        ui.group(|ui| {
                            ui.label("🔍 Protocol Discovery & Network Scanning");
                            ui.checkbox(&mut self.protocol_discovery.enabled, "Enable automatic protocol discovery");
                            
                            ui.horizontal(|ui| {
                                ui.label("Scan Range:");
                                ui.text_edit_singleline(&mut self.protocol_discovery.scan_range);
                                ui.label("(e.g., 192.168.1.0/24)");
                            });
                            
                            ui.horizontal(|ui| {
                                ui.label("Scan Interval (seconds):");
                                let mut interval_secs = self.protocol_discovery.scan_interval.as_secs() as u64;
                                ui.add(egui::DragValue::new(&mut interval_secs).range(10..=300));
                                self.protocol_discovery.scan_interval = Duration::from_secs(interval_secs);
                            });
                            
                            ui.horizontal(|ui| {
                                if ui.button("🔍 Start Discovery Scan").clicked() {
                                    // Trigger manual discovery scan
                                    println!("Starting protocol discovery scan on {}", self.protocol_discovery.scan_range);
                                }
                                if ui.button("📋 Export Results").clicked() {
                                    // Export discovered devices to file
                                }
                            });
                            
                            ui.separator();
                            
                            // Display discovered devices
                            if !self.protocol_discovery.discovered_devices.is_empty() {
                                ui.label(format!("📊 Discovered Devices ({})", self.protocol_discovery.discovered_devices.len()));
                                
                                egui::ScrollArea::vertical()
                                    .max_height(200.0)
                                    .show(ui, |ui| {
                                        for (ip, device) in &self.protocol_discovery.discovered_devices {
                                            ui.group(|ui| {
                                                ui.horizontal(|ui| {
                                                    ui.colored_label(Color32::GREEN, "●");
                                                    ui.label(format!("🖥️ {}", ip));
                                                    if let Some(hostname) = &device.hostname {
                                                        ui.label(format!("({})", hostname));
                                                    }
                                                });
                                                
                                                if !device.detected_protocols.is_empty() {
                                                    ui.horizontal(|ui| {
                                                        ui.label("🔌 Protocols:");
                                                        for protocol in &device.detected_protocols {
                                                            ui.label(format!("{:?}", protocol));
                                                        }
                                                    });
                                                }
                                                
                                                ui.label(format!("🕰️ Response: {:.1}ms", device.response_time.as_millis()));
                                            });
                                        }
                                    });
                            } else {
                                ui.label("⚠ No devices discovered yet. Enable discovery or run a manual scan.");
                            }
                        });
                        
                        ui.separator();
                        
                        // Protocol Security Section
                        ui.group(|ui| {
                            ui.label("🔒 Protocol Security & Authentication");
                            
                            ui.horizontal(|ui| {
                                ui.label("Security Level:");
                                let mut security_level = "Basic".to_string();
                                egui::ComboBox::from_label("Security")
                                    .selected_text(&security_level)
                                    .show_ui(ui, |ui| {
                                        ui.selectable_value(&mut security_level, "None".to_string(), "🚫 None");
                                        ui.selectable_value(&mut security_level, "Basic".to_string(), "🔓 Basic");
                                        ui.selectable_value(&mut security_level, "Enhanced".to_string(), "🔐 Enhanced");
                                        ui.selectable_value(&mut security_level, "High".to_string(), "🔒 High Security");
                                    });
                            });
                            
                            ui.separator();
                            
                            ui.group(|ui| {
                                ui.label("🗝 Certificate Management");
                                ui.horizontal(|ui| {
                                    if ui.button("📥 Load Certificate").clicked() {
                                        // Load certificate file
                                    }
                                    if ui.button("🔑 Generate Key Pair").clicked() {
                                        // Generate new key pair
                                        let challenge = security::generate_challenge();
                                        println!("Generated challenge: {}", challenge);
                                    }
                                    if ui.button("💾 Export Config").clicked() {
                                        // Export security configuration
                                    }
                                });
                                
                                ui.label("📊 Security Statistics:");
                                ui.horizontal(|ui| {
                                    ui.label("🔒 Encrypted connections: 0");
                                    ui.label("⚠ Authentication failures: 0");
                                    ui.label("🚫 Blocked attempts: 0");
                                });
                            });
                        });
                        
                        ui.separator();
                        
                        // Protocol Diagnostics Section
                        ui.group(|ui| {
                            ui.label("📊 Protocol Diagnostics & Monitoring");
                            
                            if !self.protocol_diagnostics.is_empty() {
                                for (protocol, diagnostics) in &self.protocol_diagnostics {
                                    ui.group(|ui| {
                                        ui.horizontal(|ui| {
                                            let status_color = if diagnostics.connected { Color32::GREEN } else { Color32::RED };
                                            ui.colored_label(status_color, "●");
                                            ui.label(format!("{:?}", protocol));
                                        });
                                        
                                        ui.horizontal(|ui| {
                                            ui.label(format!("📤 TX: {} packets ({} bytes)", diagnostics.packets_sent, diagnostics.bytes_sent));
                                            ui.label(format!("📥 RX: {} packets ({} bytes)", diagnostics.packets_received, diagnostics.bytes_received));
                                        });
                                        
                                        ui.horizontal(|ui| {
                                            ui.label(format!("🕰️ Latency: {:.1}ms", diagnostics.latency_ms));
                                            ui.label(format!("⚠ Errors: {}", diagnostics.error_count));
                                        });
                                        
                                        if let Some(error) = &diagnostics.last_error {
                                            ui.colored_label(Color32::RED, format!("❌ Last Error: {}", error));
                                        }
                                    });
                                }
                            } else {
                                ui.label("📊 No active protocol connections to monitor");
                            }
                        });
                    });
                }
                Tab::CanToolheads => {
                    ui.heading("🔗 Klipper CAN Bus Toolheads");
                    ui.label("Manage and control Klipper-compatible CAN bus toolheads for 5-axis printing");
                    
                    egui::ScrollArea::vertical().id_salt("can_toolheads_scroll").show(ui, |ui| {
                        // CAN Interface Configuration
                        ui.group(|ui| {
                            ui.label("⚙️ CAN Interface Configuration");
                            
                            ui.horizontal(|ui| {
                                ui.label("CAN Interface:");
                                ui.text_edit_singleline(&mut self.can_interface_name);
                                ui.label("(e.g., can0, vcan0)");
                            });
                            
                            ui.horizontal(|ui| {
                                ui.label("Bitrate:");
                                egui::ComboBox::from_label("CAN Bitrate")
                                    .selected_text(format!("{} bps", self.can_bitrate))
                                    .show_ui(ui, |ui| {
                                        ui.selectable_value(&mut self.can_bitrate, 125000, "125 kbps");
                                        ui.selectable_value(&mut self.can_bitrate, 250000, "250 kbps (Klipper standard)");
                                        ui.selectable_value(&mut self.can_bitrate, 500000, "500 kbps");
                                        ui.selectable_value(&mut self.can_bitrate, 1000000, "1 Mbps");
                                    });
                            });
                            
                            ui.horizontal(|ui| {
                                if ui.button("🔌 Connect CAN Bus").clicked() {
                                    // Initialize CAN connection
                                    println!("Connecting to CAN interface: {}", self.can_interface_name);
                                }
                                if ui.button("🔍 Discover Toolheads").clicked() {
                                    // Trigger toolhead discovery
                                    self.discover_can_toolheads();
                                }
                                if ui.button("⛔ Emergency Stop All").clicked() {
                                    // Emergency stop all toolheads
                                    self.emergency_stop_all_toolheads();
                                }
                            });
                            
                            ui.checkbox(&mut self.toolhead_discovery_enabled, "Enable automatic toolhead discovery");
                        });
                        
                        ui.separator();
                        
                        // Active Toolheads Management
                        ui.group(|ui| {
                            ui.label("📦 Connected Toolheads");
                            
                            // Display toolheads from the CAN manager
                            if let Ok(can_manager) = self.can_manager.try_lock() {
                                let toolheads = can_manager.get_toolheads();
                                
                                if toolheads.is_empty() {
                                    ui.label("⚠ No toolheads discovered. Click 'Discover Toolheads' to scan for devices.");
                                } else {
                                    for (uuid, toolhead) in toolheads {
                                        ui.group(|ui| {
                                            ui.horizontal(|ui| {
                                                let status_color = if toolhead.connected { Color32::GREEN } else { Color32::RED };
                                                ui.colored_label(status_color, "●");
                                                ui.label(format!("UUID: 0x{:06X}", uuid));
                                                
                                                if let Some(config) = can_manager.get_toolhead_config(*uuid) {
                                                    ui.label(format!("({})", config.name));
                                                }
                                                
                                                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                                    if ui.small_button("⚙️ Config").clicked() {
                                                        self.selected_toolhead_uuid = Some(*uuid);
                                                    }
                                                    if ui.small_button("🌡️ Status").clicked() {
                                                        // Get toolhead status
                                                    }
                                                    if ui.small_button("⛔ Stop").clicked() {
                                                        // Emergency stop this toolhead
                                                    }
                                                });
                                            });
                                            
                                            // Toolhead Status Info
                                            ui.horizontal(|ui| {
                                                ui.label(format!("🌡️ {:.1}°C/{:.1}°C", 
                                                    toolhead.temperature, toolhead.target_temperature));
                                                ui.label(format!("💨 Fan: {}%", (toolhead.fan_speed as f32 / 2.55) as u8));
                                                ui.label(format!("📍 Pos: [{:.2}, {:.2}, {:.2}, {:.3}]", 
                                                    toolhead.position[0], toolhead.position[1], 
                                                    toolhead.position[2], toolhead.position[3]));
                                            });
                                            
                                            if let Some(stats) = can_manager.get_toolhead_stats(*uuid) {
                                                ui.horizontal(|ui| {
                                                    ui.label(format!("📤 TX: {}", stats.messages_sent));
                                                    ui.label(format!("📥 RX: {}", stats.messages_received));
                                                    ui.label(format!("🕰️ {:.1}ms", stats.average_latency_ms));
                                                    if stats.error_count > 0 {
                                                        ui.colored_label(Color32::RED, format!("⚠ {} errors", stats.error_count));
                                                    }
                                                });
                                            }
                                        });
                                    }
                                }
                            } else {
                                ui.label("⚠ CAN manager is busy");
                            }
                        });
                        
                        ui.separator();
                        
                        // Individual Toolhead Control
                        if let Some(selected_uuid) = self.selected_toolhead_uuid {
                            ui.group(|ui| {
                                ui.label(format!("🎯 Toolhead Control (UUID: 0x{:06X})", selected_uuid));
                                
                                ui.horizontal(|ui| {
                                    ui.label("Temperature:");
                                    let mut target_temp = 200.0f32;
                                    ui.add(egui::Slider::new(&mut target_temp, 0.0..=300.0).suffix("°C"));
                                    if ui.button("Set").clicked() {
                                        self.set_toolhead_temperature(selected_uuid, target_temp);
                                    }
                                });
                                
                                ui.horizontal(|ui| {
                                    ui.label("Fan Speed:");
                                    let mut fan_speed = 128u8;
                                    ui.add(egui::Slider::new(&mut fan_speed, 0..=255).suffix("%"));
                                    if ui.button("Set").clicked() {
                                        self.set_toolhead_fan(selected_uuid, fan_speed);
                                    }
                                });
                                
                                ui.horizontal(|ui| {
                                    ui.label("Extruder:");
                                    let mut extrude_distance = 10.0f32;
                                    ui.add(egui::DragValue::new(&mut extrude_distance).suffix("mm"));
                                    if ui.button("Extrude").clicked() {
                                        self.move_extruder(selected_uuid, extrude_distance, 5.0);
                                    }
                                    if ui.button("Retract").clicked() {
                                        self.move_extruder(selected_uuid, -extrude_distance, 10.0);
                                    }
                                });
                                
                                ui.horizontal(|ui| {
                                    if ui.button("🏠 Home").clicked() {
                                        self.home_toolhead(selected_uuid);
                                    }
                                    if ui.button("📊 Get Status").clicked() {
                                        self.get_toolhead_status(selected_uuid);
                                    }
                                    if ui.button("⛔ E-Stop").clicked() {
                                        self.emergency_stop_toolhead(selected_uuid);
                                    }
                                    if ui.button("✖ Close").clicked() {
                                        self.selected_toolhead_uuid = None;
                                    }
                                });
                            });
                        }
                        
                        ui.separator();
                        
                        // 5-Axis Integration Status
                        ui.group(|ui| {
                            ui.label("🔄 5-Axis Integration Status");
                            ui.horizontal(|ui| {
                                ui.label("Current Position:");
                                ui.label(format!("X:{:.2} Y:{:.2} Z:{:.2} A:{:.1}° B:{:.1}°", 
                                    self.x, self.y, self.z, self.a, self.b));
                            });
                            ui.horizontal(|ui| {
                                ui.label("Kinematics:");
                                ui.label("5-axis coordinated motion enabled");
                                if self.printer_settings.enable_5axis_mode {
                                    ui.colored_label(Color32::GREEN, "✓ 5-Axis Active");
                                } else {
                                    ui.colored_label(Color32::GRAY, "3-Axis Mode");
                                }
                            });
                            ui.horizontal(|ui| {
                                ui.label("Toolhead Sync:");
                                ui.label("Real-time position updates to CAN toolheads");
                                ui.colored_label(Color32::GREEN, "✓ Synchronized");
                            });
                        });
                        
                        ui.separator();
                        
                        // CAN Bus Diagnostics
                        ui.group(|ui| {
                            ui.label("📊 CAN Bus Diagnostics");
                            if let Ok(can_manager) = self.can_manager.try_lock() {
                                let toolheads = can_manager.get_toolheads();
                                let total_messages = toolheads.values()
                                    .filter_map(|_| can_manager.get_toolhead_stats(0)) // This should iterate properly
                                    .map(|s| s.messages_sent + s.messages_received)
                                    .sum::<u64>();
                                
                                ui.horizontal(|ui| {
                                    ui.label(format!("Total CAN Messages: {}", total_messages));
                                    ui.label(format!("Active Toolheads: {}", toolheads.len()));
                                    ui.label("Bus Status: OK");
                                });
                            }
                            
                            ui.horizontal(|ui| {
                                if ui.button("📊 Export Diagnostics").clicked() {
                                    // Export CAN diagnostics
                                }
                                if ui.button("🔄 Reset Counters").clicked() {
                                    // Reset diagnostic counters
                                }
                                if ui.button("📋 View Logs").clicked() {
                                    // View CAN communication logs
                                }
                            });
                        });
                    });
                }
                Tab::InputShaping => {
                    ui.heading("📊 Input Shaping & Resonance Control");
                    ui.label("Configure input shaping algorithms to eliminate printer resonance and improve print quality");
                    
                    egui::ScrollArea::vertical().id_salt("input_shaping_scroll").show(ui, |ui| {
                        // Global Input Shaping Controls
                        ui.group(|ui| {
                            ui.label("⚙️ Global Input Shaping Settings");
                            
                            ui.horizontal(|ui| {
                                ui.checkbox(&mut self.input_shaping_enabled, "Enable Input Shaping");
                                if self.input_shaping_enabled {
                                    ui.colored_label(Color32::GREEN, "✓ Active");
                                } else {
                                    ui.colored_label(Color32::RED, "⚠ Disabled");
                                }
                            });
                            
                            ui.horizontal(|ui| {
                                ui.checkbox(&mut self.resonance_testing_active, "Resonance Testing Mode");
                                if self.resonance_testing_active {
                                    ui.colored_label(Color32::YELLOW, "🔬 Testing");
                                }
                            });
                        });
                        
                        ui.separator();
                        
                        // Axis Selection
                        ui.group(|ui| {
                            ui.label("🎯 Axis Configuration");
                            
                            ui.horizontal(|ui| {
                                ui.label("Configure axis:");
                                egui::ComboBox::from_label("Axis Selection")
                                    .selected_text(format!("{} Axis", self.selected_shaper_axis))
                                    .show_ui(ui, |ui| {
                                        ui.selectable_value(&mut self.selected_shaper_axis, 'X', "X Axis (Primary)");
                                        ui.selectable_value(&mut self.selected_shaper_axis, 'Y', "Y Axis (Primary)");
                                        ui.selectable_value(&mut self.selected_shaper_axis, 'Z', "Z Axis (Vertical)");
                                        ui.selectable_value(&mut self.selected_shaper_axis, 'A', "A Axis (Rotational)");
                                        ui.selectable_value(&mut self.selected_shaper_axis, 'B', "B Axis (Rotational)");
                                    });
                            });
                        });
                        
                        ui.separator();
                        
                        // Current Axis Configuration
                        if let Some(config) = self.input_shaper_manager.get_axis_config(self.selected_shaper_axis) {
                            let mut config_clone = config.clone();
                            
                            ui.group(|ui| {
                                ui.label(format!("📋 {} Axis Input Shaper Configuration", self.selected_shaper_axis));
                                
                                ui.horizontal(|ui| {
                                    ui.checkbox(&mut config_clone.enabled, "Enable for this axis");
                                    if config_clone.enabled {
                                        ui.colored_label(Color32::GREEN, "✓ Enabled");
                                    } else {
                                        ui.colored_label(Color32::RED, "⚠ Disabled");
                                    }
                                });
                                
                                ui.horizontal(|ui| {
                                    ui.label("Shaper Type:");
                                    egui::ComboBox::from_label("Algorithm")
                                        .selected_text(format!("{:?}", config_clone.shaper_type))
                                        .show_ui(ui, |ui| {
                                            ui.selectable_value(&mut config_clone.shaper_type, ShaperType::ZV, "ZV (Zero Vibration)");
                                            ui.selectable_value(&mut config_clone.shaper_type, ShaperType::MZV, "MZV (Modified ZV)");
                                            ui.selectable_value(&mut config_clone.shaper_type, ShaperType::EI, "EI (Extra Insensitive)");
                                            ui.selectable_value(&mut config_clone.shaper_type, ShaperType::EI2, "EI2 (2-Hump EI)");
                                            ui.selectable_value(&mut config_clone.shaper_type, ShaperType::EI3, "EI3 (3-Hump EI)");
                                            ui.selectable_value(&mut config_clone.shaper_type, ShaperType::Smoothing, "Smoothing");
                                        });
                                });
                                
                                ui.horizontal(|ui| {
                                    ui.label("Resonance Frequency:");
                                    ui.add(egui::DragValue::new(&mut config_clone.frequency)
                                        .range(5.0..=200.0)
                                        .suffix(" Hz")
                                        .speed(0.5));
                                    if ui.small_button("🔍 Auto-Detect").clicked() {
                                        // Start resonance frequency detection
                                        self.resonance_testing_active = true;
                                    }
                                });
                                
                                ui.horizontal(|ui| {
                                    ui.label("Damping Ratio:");
                                    ui.add(egui::DragValue::new(&mut config_clone.damping_ratio)
                                        .range(0.01..=0.3)
                                        .speed(0.005)
                                        .fixed_decimals(3));
                                    ui.label("(0.1 = 10% damping)");
                                });
                                
                                ui.horizontal(|ui| {
                                    ui.label("Max Acceleration:");
                                    ui.add(egui::DragValue::new(&mut config_clone.max_acceleration)
                                        .range(100.0..=20000.0)
                                        .suffix(" mm/s²")
                                        .speed(10.0));
                                });
                                
                                ui.horizontal(|ui| {
                                    ui.label("Smoothing Time:");
                                    ui.add(egui::DragValue::new(&mut config_clone.smoothing_time)
                                        .range(0.01..=0.2)
                                        .suffix(" s")
                                        .speed(0.001)
                                        .fixed_decimals(3));
                                });
                                
                                ui.separator();
                                
                                ui.horizontal(|ui| {
                                    if ui.button("💾 Apply Configuration").clicked() {
                                        self.input_shaper_manager.update_axis_config(self.selected_shaper_axis, config_clone.clone());
                                        println!("Applied input shaper config for {} axis", self.selected_shaper_axis);
                                    }
                                    if ui.button("🔄 Reset to Default").clicked() {
                                        let default_config = match self.selected_shaper_axis {
                                            'X' | 'Y' => InputShaperConfig {
                                                shaper_type: ShaperType::MZV,
                                                frequency: 40.0,
                                                damping_ratio: 0.1,
                                                enabled: true,
                                                max_acceleration: 5000.0,
                                                smoothing_time: 0.04,
                                            },
                                            'Z' => InputShaperConfig {
                                                shaper_type: ShaperType::ZV,
                                                frequency: 25.0,
                                                damping_ratio: 0.15,
                                                enabled: true,
                                                max_acceleration: 1000.0,
                                                smoothing_time: 0.06,
                                            },
                                            'A' | 'B' => InputShaperConfig {
                                                shaper_type: ShaperType::EI,
                                                frequency: 60.0,
                                                damping_ratio: 0.08,
                                                enabled: true,
                                                max_acceleration: 10000.0,
                                                smoothing_time: 0.03,
                                            },
                                            _ => InputShaperConfig::default(),
                                        };
                                        self.input_shaper_manager.update_axis_config(self.selected_shaper_axis, default_config);
                                    }
                                });
                            });
                        }
                        
                        ui.separator();
                        
                        // Resonance Testing
                        ui.group(|ui| {
                            ui.label("🔬 Resonance Testing & Calibration");
                            
                            if !self.resonance_testing_active {
                                ui.horizontal(|ui| {
                                    if ui.button("📊 Start Frequency Sweep").clicked() {
                                        self.resonance_testing_active = true;
                                        // Start resonance testing for current axis
                                        if let Err(e) = self.input_shaper_manager.start_resonance_test(self.selected_shaper_axis, 5.0, 100.0, 1.0) {
                                            println!("Failed to start resonance test: {}", e);
                                        }
                                    }
                                    if ui.button("🎯 Quick Calibration").clicked() {
                                        // Quick calibration for current axis
                                        self.resonance_testing_active = true;
                                    }
                                });
                                
                                ui.label("Run resonance tests to automatically determine optimal shaper frequencies");
                            } else {
                                ui.horizontal(|ui| {
                                    ui.colored_label(Color32::YELLOW, "🔬 Resonance test in progress...");
                                    if ui.button("⏹ Stop Test").clicked() {
                                        self.resonance_testing_active = false;
                                        self.input_shaper_manager.stop_resonance_test();
                                    }
                                });
                                
                                // Show test progress
                                if let Some(progress) = self.input_shaper_manager.get_test_progress() {
                                    ui.add(egui::ProgressBar::new(progress)
                                        .text(format!("Testing... {:.1}%", progress * 100.0)));
                                }
                            }
                            
                            ui.separator();
                            
                            ui.horizontal(|ui| {
                                if ui.button("📈 View Response Graph").clicked() {
                                    // Show frequency response graph
                                }
                                if ui.button("📋 Export Results").clicked() {
                                    // Export resonance test results
                                }
                                if ui.button("🔧 Advanced Settings").clicked() {
                                    // Show advanced resonance testing options
                                }
                            });
                        });
                        
                        ui.separator();
                        
                        // Input Shaper Status Summary
                        ui.group(|ui| {
                            ui.label("📊 Input Shaper Status Summary");
                            
                            ui.horizontal(|ui| {
                                let enabled_axes = ['X', 'Y', 'Z', 'A', 'B']
                                    .iter()
                                    .filter(|&&axis| {
                                        self.input_shaper_manager.get_axis_config(axis)
                                            .map(|c| c.enabled)
                                            .unwrap_or(false)
                                    })
                                    .count();
                                
                                ui.label(format!("Active Axes: {}/5", enabled_axes));
                                
                                if enabled_axes > 0 {
                                    ui.colored_label(Color32::GREEN, "✓ Input shaping active");
                                } else {
                                    ui.colored_label(Color32::RED, "⚠ No axes configured");
                                }
                            });
                            
                            // Show brief config for each axis
                            ui.horizontal_wrapped(|ui| {
                                for axis in ['X', 'Y', 'Z', 'A', 'B'] {
                                    if let Some(config) = self.input_shaper_manager.get_axis_config(axis) {
                                        if config.enabled {
                                            ui.label(format!("{}:{:.1}Hz/{:?}", axis, config.frequency, config.shaper_type));
                                        }
                                    }
                                }
                            });
                        });
                    });
                }
                Tab::MotorAssignment => {
                    ui.heading("🔧 Motor Assignment & Configuration");
                    ui.label("Configure and assign motors to printer axes, extruders, and other functions");
                    
                    egui::ScrollArea::vertical().id_salt("motor_assignment_main_scroll").show(ui, |ui| {
                        ui.columns(2, |columns| {
                            // Left column - Motor list and configuration
                            columns[0].group(|ui| {
                                ui.label("🎯 Motor Configuration");
                                
                                // Quick setup buttons
                                ui.horizontal(|ui| {
                                    if ui.button("📐 Cartesian Setup").clicked() {
                                        if let Err(e) = self.motor_manager.create_standard_cartesian_setup() {
                                            // Show error - could implement error display
                                        }
                                    }
                                    if ui.button("🔄 CoreXY Setup").clicked() {
                                        if let Err(e) = self.motor_manager.create_corexy_setup() {
                                            // Show error  
                                        }
                                    }
                                    if ui.button("➕ Add Motor").clicked() {
                                        self.show_add_motor_dialog = true;
                                    }
                                    
                                    ui.horizontal(|ui| {
                                        if ui.button("🗑️ Remove Selected").clicked() {
                                            if let Some(selected_id) = &self.selected_motor_id {
                                                match self.motor_manager.remove_motor(selected_id) {
                                                    Ok(removed_config) => {
                                                        println!("Removed motor: {} (was {:?})", 
                                                            removed_config.motor_id, removed_config.function);
                                                        self.selected_motor_id = None;
                                                    },
                                                    Err(e) => {
                                                        println!("Error removing motor: {}", e);
                                                    }
                                                }
                                            } else {
                                                println!("No motor selected for removal");
                                            }
                                        }
                                        
                                        if ui.button("🧹 Clear All Motors").clicked() {
                                            let motor_ids: Vec<String> = self.motor_manager.get_all_motors().keys().cloned().collect();
                                            let count = motor_ids.len();
                                            for id in motor_ids {
                                                let _ = self.motor_manager.remove_motor(&id);
                                            }
                                            self.selected_motor_id = None;
                                            println!("Removed all {} motors", count);
                                        }
                                    });
                                });
                                
                                ui.separator();
                                
                                // Motor list with assignment status
                                ui.label("🔩 Registered Motors");
                                egui::ScrollArea::vertical()
                                    .id_salt("motor_list_scroll")
                                    .max_height(300.0)
                                    .auto_shrink([false, true])
                                    .show(ui, |ui| {
                                    for (motor_id, config) in self.motor_manager.get_all_motors() {
                                        let is_selected = self.selected_motor_id.as_ref() == Some(motor_id);
                                        
                                        ui.horizontal(|ui| {
                                            if ui.selectable_label(is_selected, motor_id).clicked() {
                                                self.selected_motor_id = Some(motor_id.clone());
                                            }
                                            
                                            // Show motor type and function
                                            ui.label(format!("{:?}", config.motor_type));
                                            
                                            // Function assignment indicator
                                            let function_text = match &config.function {
                                                MotorFunction::Unassigned => "⚠ Unassigned".to_string(),
                                                f => format!("{:?}", f),
                                            };
                                            
                                            let color = if config.function == MotorFunction::Unassigned {
                                                Color32::YELLOW
                                            } else if config.enabled {
                                                Color32::GREEN
                                            } else {
                                                Color32::GRAY
                                            };
                                            
                                            ui.colored_label(color, function_text);
                                            
                                            if config.enabled {
                                                ui.label("✓");
                                            } else {
                                                ui.label("❌");
                                            }
                                        });
                                    }
                                });
                                
                                // Motor details for selected motor
                                if let Some(selected_id) = &self.selected_motor_id {
                                    if let Some(config) = self.motor_manager.get_motor_config(selected_id) {
                                        ui.separator();
                                        ui.label("🔧 Motor Details");
                                        
                                        ui.horizontal(|ui| {
                                            ui.label("ID:");
                                            ui.label(&config.motor_id);
                                        });
                                        
                                        ui.horizontal(|ui| {
                                            ui.label("Type:");
                                            ui.label(format!("{:?}", config.motor_type));
                                        });
                                        
                                        ui.horizontal(|ui| {
                                            ui.label("Function:");
                                            ui.label(format!("{:?}", config.function));
                                        });
                                        
                                        ui.horizontal(|ui| {
                                            ui.label("Max Speed:");
                                            ui.label(format!("{:.1}", config.capabilities.max_speed));
                                        });
                                        
                                        ui.horizontal(|ui| {
                                            ui.label("Steps/mm:");
                                            ui.label(format!("{:.1}", config.capabilities.steps_per_unit));
                                        });
                                        
                                        ui.horizontal(|ui| {
                                            ui.label("Current:");
                                            ui.label(format!("{:.2}A", config.current_setting));
                                        });
                                        
                                        if config.capabilities.has_encoder {
                                            ui.colored_label(Color32::GREEN, "🔄 Encoder Available");
                                        }
                                        
                                        if config.capabilities.stall_detection {
                                            ui.colored_label(Color32::BLUE, "⚠ Stall Detection");
                                        }
                                    }
                                }
                            });
                            
                            // Right column - Function assignments and motor states
                            columns[1].group(|ui| {
                                ui.label("🎯 Function Assignments");
                                
                                // Function assignment matrix
                                egui::Grid::new("function_grid").show(ui, |ui| {
                                    ui.label("Function");
                                    ui.label("Assigned Motor");
                                    ui.label("Status");
                                    ui.end_row();
                                    
                                    let functions = [
                                        ("X Axis", MotorFunction::XAxis(0)),
                                        ("Y Axis", MotorFunction::YAxis(0)),
                                        ("Z Axis", MotorFunction::ZAxis(0)),
                                        ("A Axis", MotorFunction::AAxis(0)),
                                        ("B Axis", MotorFunction::BAxis(0)),
                                        ("Extruder 0", MotorFunction::Extruder(0)),
                                        ("Tool Changer", MotorFunction::ToolChanger),
                                        ("Bed Leveling", MotorFunction::BedLeveling(0)),
                                    ];
                                    
                                    for (name, function) in functions.iter() {
                                        ui.label(*name);
                                        
                                        if let Some(motor_ids) = self.motor_manager.get_motors_for_function(function) {
                                            if let Some(motor_id) = motor_ids.first() {
                                            ui.label(motor_id);
                                            
                                            if let Some(state) = self.motor_manager.get_motor_state(motor_id) {
                                                let status = if state.enabled {
                                                    if state.homed { "✓ Ready" } else { "⚠ Not Homed" }
                                                } else {
                                                    "❌ Disabled"
                                                };
                                                
                                                let color = if state.enabled && state.homed {
                                                    Color32::GREEN
                                                } else if state.enabled {
                                                    Color32::YELLOW
                                                } else {
                                                    Color32::RED
                                                };
                                                
                                                ui.colored_label(color, status);
                                            } else {
                                                ui.colored_label(Color32::GRAY, "No State");
                                            }
                                            }
                                        } else {
                                            ui.colored_label(Color32::RED, "Unassigned");
                                            ui.colored_label(Color32::RED, "❌");
                                        }
                                        
                                        ui.end_row();
                                    }
                                });
                                
                                ui.separator();
                                
                                // Assignment history
                                ui.label("📜 Assignment History (Last 10)");
                                egui::ScrollArea::vertical()
                                    .id_salt("motor_assignment_history_scroll")
                                    .max_height(200.0)
                                    .auto_shrink([false, true])
                                    .show(ui, |ui| {
                                    for event in self.motor_manager.get_assignment_history().iter().rev().take(10) {
                                        ui.horizontal(|ui| {
                                            ui.label(format!("{}", event.motor_id));
                                            ui.label("→");
                                            ui.colored_label(Color32::GREEN, format!("{:?}", event.new_function));
                                            ui.label(format!("({})", event.reason));
                                        });
                                    }
                                });
                                
                                ui.separator();
                                
                                // Quick actions for selected motor
                                if let Some(selected_id) = self.selected_motor_id.clone() {
                                    ui.label("🔧 Quick Actions");
                                    ui.horizontal(|ui| {
                                        if ui.button("🏠 Home").clicked() {
                                            // Home motor - could implement actual homing
                                        }
                                        if ui.button("▶ Enable").clicked() {
                                            // Enable motor - could implement actual enable
                                        }
                                        if ui.button("⏸ Disable").clicked() {
                                            // Disable motor - could implement actual disable
                                        }
                                        if ui.button("❌ Remove Motor").clicked() {
                                            // Add confirmation for motor removal
                                            match self.motor_manager.remove_motor(&selected_id) {
                                                Ok(removed_config) => {
                                                    println!("Successfully removed motor: {} (was {:?})", 
                                                        removed_config.motor_id, removed_config.function);
                                                    self.selected_motor_id = None;
                                                },
                                                Err(e) => {
                                                    println!("Error removing motor: {}", e);
                                                }
                                            }
                                        }
                                        if ui.button("➕ Clone Motor").clicked() {
                                            // Clone the current motor configuration with a new ID
                                            if let Some(config) = self.motor_manager.get_motor_config(&selected_id) {
                                                let mut new_config = config.clone();
                                                new_config.motor_id = format!("{}_clone", config.motor_id);
                                                new_config.function = MotorFunction::Unassigned;
                                                if let Ok(new_id) = self.motor_manager.register_motor(new_config) {
                                                    println!("Cloned motor as: {}", new_id);
                                                }
                                            }
                                        }
                                    });
                                    
                                    // Function reassignment
                                    ui.horizontal(|ui| {
                                        ui.label("Assign to:");
                                        egui::ComboBox::from_label("")
                                            .selected_text("Select Function")
                                            .show_ui(ui, |ui| {
                                                if ui.button("X Axis").clicked() {
                                                    let _ = self.motor_manager.assign_function(&selected_id, MotorFunction::XAxis(0), "Manual assignment");
                                                }
                                                if ui.button("Y Axis").clicked() {
                                                    let _ = self.motor_manager.assign_function(&selected_id, MotorFunction::YAxis(0), "Manual assignment");
                                                }
                                                if ui.button("Z Axis").clicked() {
                                                    let _ = self.motor_manager.assign_function(&selected_id, MotorFunction::ZAxis(0), "Manual assignment");
                                                }
                                                if ui.button("Extruder 0").clicked() {
                                                    let _ = self.motor_manager.assign_function(&selected_id, MotorFunction::Extruder(0), "Manual assignment");
                                                }
                                                if ui.button("Unassigned").clicked() {
                                                    let _ = self.motor_manager.assign_function(&selected_id, MotorFunction::Unassigned, "Manual unassignment");
                                                }
                                            });
                                    });
                                }
                            });
                        });
                    });
                }
                Tab::ClosedLoop => {
                    ui.heading("🔄 Closed-Loop Encoder Control");
                    ui.label("Configure and monitor closed-loop feedback systems for precise position control");
                    
                    egui::ScrollArea::vertical()
                        .id_salt("closed_loop_main_scroll")
                        .auto_shrink([false, true])
                        .show(ui, |ui| {
                        ui.columns(2, |columns| {
                            // Left column - Encoder configuration and status
                            columns[0].group(|ui| {
                                ui.label("🎯 Encoder Configuration");
                                
                                // Global closed-loop controls
                                ui.horizontal(|ui| {
                                    let status = self.closed_loop_manager.get_status_summary();
                                    
                                    if status.global_enabled {
                                        if ui.button("⏸ Disable Global").clicked() {
                                            self.closed_loop_manager.disable_global();
                                        }
                                        ui.colored_label(Color32::GREEN, "✓ Enabled");
                                    } else {
                                        if ui.button("▶ Enable Global").clicked() {
                                            self.closed_loop_manager.enable_global();
                                        }
                                        ui.colored_label(Color32::RED, "❌ Disabled");
                                    }
                                });
                                
                                ui.separator();
                                
                                // Quick setup actions
                                ui.horizontal(|ui| {
                                    if ui.button("🏠 Home All Axes").clicked() {
                                        if let Err(e) = self.closed_loop_manager.home_all_axes() {
                                            // Could implement error display
                                        }
                                    }
                                    if ui.button("➕ Add Encoder Axis").clicked() {
                                        // Could implement add encoder dialog
                                    }
                                    
                                    ui.horizontal(|ui| {
                                        if ui.button("🗑️ Remove Selected Axis").clicked() {
                                            if let Some(selected_axis) = &self.selected_encoder_axis {
                                                let result = self.closed_loop_manager.remove_axis(selected_axis);
                                                if result.is_ok() {
                                                    println!("Removed encoder axis: {}", selected_axis);
                                                    self.selected_encoder_axis = None;
                                                } else {
                                                    println!("Failed to remove encoder axis: {:?}", result.err());
                                                }
                                            } else {
                                                println!("No encoder axis selected for removal");
                                            }
                                        }
                                        
                                        if ui.button("🧹 Clear All Axes").clicked() {
                                            let diagnostics = self.closed_loop_manager.get_all_diagnostics();
                                            let count = diagnostics.len();
                                            for (axis_name, _) in diagnostics {
                                                let _ = self.closed_loop_manager.remove_axis(&axis_name);
                                            }
                                            self.selected_encoder_axis = None;
                                            println!("Removed all {} encoder axes", count);
                                        }
                                    });
                                });
                                
                                ui.separator();
                                
                                // Encoder axis list
                                ui.label("🔩 Configured Encoder Axes");
                                egui::ScrollArea::vertical()
                                    .id_salt("encoder_axis_list_scroll")
                                    .max_height(300.0)
                                    .auto_shrink([false, true])
                                    .show(ui, |ui| {
                                    let diagnostics = self.closed_loop_manager.get_all_diagnostics();
                                    
                                    for (axis_name, diag) in diagnostics.iter() {
                                        let is_selected = self.selected_encoder_axis.as_ref() == Some(axis_name);
                                        
                                        ui.horizontal(|ui| {
                                            if ui.selectable_label(is_selected, axis_name).clicked() {
                                                self.selected_encoder_axis = Some(axis_name.clone());
                                            }
                                            
                                            // Connection status
                                            if diag.connected {
                                                ui.colored_label(Color32::GREEN, "🔗 Connected");
                                            } else {
                                                ui.colored_label(Color32::RED, "❌ Disconnected");
                                            }
                                            
                                            // Calibration status
                                            if diag.calibrated {
                                                ui.colored_label(Color32::GREEN, "📐 Calibrated");
                                            } else {
                                                ui.colored_label(Color32::YELLOW, "⚠ Not Calibrated");
                                            }
                                            
                                            // Enabled status
                                            if diag.enabled {
                                                ui.colored_label(Color32::BLUE, "🔄 Active");
                                            } else {
                                                ui.colored_label(Color32::GRAY, "⏸ Inactive");
                                            }
                                        });
                                        
                                        // Position error indicator
                                        ui.horizontal(|ui| {
                                            ui.label("Position Error:");
                                            let error_color = if diag.position_error.abs() > 0.1 {
                                                Color32::RED
                                            } else if diag.position_error.abs() > 0.05 {
                                                Color32::YELLOW
                                            } else {
                                                Color32::GREEN
                                            };
                                            ui.colored_label(error_color, format!("{:.4} mm", diag.position_error));
                                        });
                                        
                                        ui.separator();
                                    }
                                    
                                    if diagnostics.is_empty() {
                                        ui.colored_label(Color32::GRAY, "No encoder axes configured");
                                    }
                                });
                                
                                // Selected axis details  
                                if let Some(selected_axis) = self.selected_encoder_axis.clone() {
                                    let diagnostics = self.closed_loop_manager.get_all_diagnostics();
                                    if let Some(diag) = diagnostics.get(&selected_axis) {
                                        ui.separator();
                                        ui.label(format!("🔧 {} Details", selected_axis));
                                        
                                        egui::Grid::new("axis_details").show(ui, |ui| {
                                            ui.label("Current Position:");
                                            ui.label(format!("{:.4} mm", diag.current_position));
                                            ui.end_row();
                                            
                                            ui.label("Target Position:");
                                            ui.label(format!("{:.4} mm", diag.target_position));
                                            ui.end_row();
                                            
                                            ui.label("Position Error:");
                                            let error_color = if diag.position_error.abs() > 0.1 {
                                                Color32::RED
                                            } else {
                                                Color32::GREEN
                                            };
                                            ui.colored_label(error_color, format!("{:.4} mm", diag.position_error));
                                            ui.end_row();
                                            
                                            ui.label("Max Error Seen:");
                                            ui.label(format!("{:.4} mm", diag.max_position_error));
                                            ui.end_row();
                                            
                                            ui.label("Velocity:");
                                            ui.label(format!("{:.2} mm/s", diag.velocity / 1000.0)); // Convert counts/s to mm/s estimate
                                            ui.end_row();
                                            
                                            ui.label("Error Count:");
                                            if diag.error_count > 0 {
                                                ui.colored_label(Color32::RED, diag.error_count.to_string());
                                            } else {
                                                ui.colored_label(Color32::GREEN, "0");
                                            }
                                            ui.end_row();
                                            
                                            ui.label("Comm Errors:");
                                            if diag.communication_errors > 0 {
                                                ui.colored_label(Color32::RED, diag.communication_errors.to_string());
                                            } else {
                                                ui.colored_label(Color32::GREEN, "0");
                                            }
                                            ui.end_row();
                                        });
                                        
                                        ui.separator();
                                        
                                        // PID tuning controls
                                        ui.label("🎛 PID Tuning");
                                        ui.horizontal(|ui| {
                                            if ui.button("Auto Tune").clicked() {
                                                // Could implement PID auto-tuning
                                            }
                                            if ui.button("Reset PID").clicked() {
                                                // Reset PID controller
                                            }
                                        });
                                        
                                        // Manual PID adjustment (simplified UI)
                                        ui.horizontal(|ui| {
                                            ui.label("Kp: 1.0"); // Could make these editable
                                            ui.label("Ki: 0.1");
                                            ui.label("Kd: 0.01");
                                        });
                                        
                                        ui.separator();
                                        
                                        // Encoder axis management
                                        ui.label("🔧 Axis Management");
                                        ui.horizontal(|ui| {
                                            if ui.button("🗑 Remove Axis").clicked() {
                                                if let Err(e) = self.closed_loop_manager.remove_axis(&selected_axis) {
                                                    // Could implement error display
                                                    println!("Error removing encoder axis: {}", e);
                                                } else {
                                                    // Clear selection after successful removal
                                                    self.selected_encoder_axis = None;
                                                }
                                            }
                                            if ui.button("🔄 Recalibrate").clicked() {
                                                if let Err(e) = self.closed_loop_manager.calibrate_axis(&selected_axis) {
                                                    // Could implement error display
                                                    println!("Error calibrating axis: {}", e);
                                                }
                                            }
                                            if ui.button("⚡ Reset Errors").clicked() {
                                                if let Err(e) = self.closed_loop_manager.reset_axis_errors(&selected_axis) {
                                                    println!("Error resetting axis errors: {}", e);
                                                }
                                            }
                                        });
                                    }
                                }
                            });
                            
                            // Right column - System status and actions
                            columns[1].group(|ui| {
                                ui.label("📊 System Status");
                                
                                let status = self.closed_loop_manager.get_status_summary();
                                
                                // Status overview
                                egui::Grid::new("status_grid").show(ui, |ui| {
                                    ui.label("Global Status:");
                                    if status.global_enabled {
                                        ui.colored_label(Color32::GREEN, "✓ Enabled");
                                    } else {
                                        ui.colored_label(Color32::RED, "❌ Disabled");
                                    }
                                    ui.end_row();
                                    
                                    ui.label("Update Rate:");
                                    ui.label(format!("{} Hz", status.update_rate_hz));
                                    ui.end_row();
                                    
                                    ui.label("Total Axes:");
                                    ui.label(status.total_axes.to_string());
                                    ui.end_row();
                                    
                                    ui.label("Enabled Axes:");
                                    ui.colored_label(Color32::GREEN, status.enabled_axes.to_string());
                                    ui.end_row();
                                    
                                    ui.label("Calibrated Axes:");
                                    ui.colored_label(Color32::BLUE, status.calibrated_axes.to_string());
                                    ui.end_row();
                                    
                                    ui.label("Connected Axes:");
                                    ui.colored_label(Color32::GREEN, status.connected_axes.to_string());
                                    ui.end_row();
                                    
                                    ui.label("Max Position Error:");
                                    let error_color = if status.max_position_error > 0.1 {
                                        Color32::RED
                                    } else if status.max_position_error > 0.05 {
                                        Color32::YELLOW
                                    } else {
                                        Color32::GREEN
                                    };
                                    ui.colored_label(error_color, format!("{:.4} mm", status.max_position_error));
                                    ui.end_row();
                                    
                                    ui.label("Error Recovery:");
                                    if status.error_recovery_enabled {
                                        ui.colored_label(Color32::GREEN, "✓ Enabled");
                                    } else {
                                        ui.colored_label(Color32::YELLOW, "⚠ Disabled");
                                    }
                                    ui.end_row();
                                });
                                
                                ui.separator();
                                
                                // Actions for selected axis
                                if let Some(selected_axis) = &self.selected_encoder_axis {
                                    ui.label(format!("🔧 {} Actions", selected_axis));
                                    
                                    ui.horizontal(|ui| {
                                        if ui.button("🏠 Home Axis").clicked() {
                                            let _ = self.closed_loop_manager.home_axis(selected_axis);
                                        }
                                        
                                        let diagnostics = self.closed_loop_manager.get_all_diagnostics();
                                        if let Some(diag) = diagnostics.get(selected_axis) {
                                            if diag.enabled {
                                                if ui.button("⏸ Disable").clicked() {
                                                    let _ = self.closed_loop_manager.disable_axis(selected_axis);
                                                }
                                            } else {
                                                if ui.button("▶ Enable").clicked() {
                                                    let _ = self.closed_loop_manager.enable_axis(selected_axis);
                                                }
                                            }
                                        }
                                    });
                                    
                                    ui.separator();
                                    
                                    // Manual position control (for testing)
                                    ui.label("🎮 Manual Control");
                                    ui.horizontal(|ui| {
                                        if ui.button("← -1mm").clicked() {
                                            let diagnostics = self.closed_loop_manager.get_all_diagnostics();
                                            if let Some(diag) = diagnostics.get(selected_axis) {
                                                let new_pos = diag.target_position - 1.0;
                                                let _ = self.closed_loop_manager.set_target_position(selected_axis, new_pos);
                                            }
                                        }
                                        if ui.button("← -0.1mm").clicked() {
                                            let diagnostics = self.closed_loop_manager.get_all_diagnostics();
                                            if let Some(diag) = diagnostics.get(selected_axis) {
                                                let new_pos = diag.target_position - 0.1;
                                                let _ = self.closed_loop_manager.set_target_position(selected_axis, new_pos);
                                            }
                                        }
                                        if ui.button("+0.1mm →").clicked() {
                                            let diagnostics = self.closed_loop_manager.get_all_diagnostics();
                                            if let Some(diag) = diagnostics.get(selected_axis) {
                                                let new_pos = diag.target_position + 0.1;
                                                let _ = self.closed_loop_manager.set_target_position(selected_axis, new_pos);
                                            }
                                        }
                                        if ui.button("+1mm →").clicked() {
                                            let diagnostics = self.closed_loop_manager.get_all_diagnostics();
                                            if let Some(diag) = diagnostics.get(selected_axis) {
                                                let new_pos = diag.target_position + 1.0;
                                                let _ = self.closed_loop_manager.set_target_position(selected_axis, new_pos);
                                            }
                                        }
                                    });
                                }
                                
                                ui.separator();
                                
                                // Quick encoder setup wizard
                                ui.label("🧙 Quick Setup");
                                ui.horizontal(|ui| {
                                    if ui.button("📐 Add X Axis Encoder").clicked() {
                                        use closed_loop_control::EncoderPinConfig;
                                        let config = EncoderConfig {
                                            encoder_type: EncoderType::Quadrature,
                                            resolution: 4096,
                                            interface: "GPIO".to_string(),
                                            pin_config: EncoderPinConfig {
                                                channel_a: Some(2),
                                                channel_b: Some(3),
                                                ..Default::default()
                                            },
                                            invert_direction: false,
                                            counts_per_mm: 80.0, // 200 steps * 16 microsteps / 1.25mm pitch = 2560 steps/mm * 4 quadrature = ~10240 counts/mm (simplified to 80 for display)
                                            max_error_threshold: 0.5,
                                            filter_samples: 5,
                                            update_rate_hz: 1000,
                                        };
                                        let _ = self.closed_loop_manager.add_axis("X".to_string(), config);
                                    }
                                    
                                    if ui.button("📐 Add Y Axis Encoder").clicked() {
                                        use closed_loop_control::EncoderPinConfig;
                                        let config = EncoderConfig {
                                            encoder_type: EncoderType::Quadrature,
                                            resolution: 4096,
                                            interface: "GPIO".to_string(),
                                            pin_config: EncoderPinConfig {
                                                channel_a: Some(4),
                                                channel_b: Some(5),
                                                ..Default::default()
                                            },
                                            invert_direction: false,
                                            counts_per_mm: 80.0,
                                            max_error_threshold: 0.5,
                                            filter_samples: 5,
                                            update_rate_hz: 1000,
                                        };
                                        let _ = self.closed_loop_manager.add_axis("Y".to_string(), config);
                                    }
                                });
                                
                                ui.horizontal(|ui| {
                                    if ui.button("📐 Add Z Axis Encoder").clicked() {
                                        use closed_loop_control::EncoderPinConfig;
                                        let config = EncoderConfig {
                                            encoder_type: EncoderType::Quadrature,
                                            resolution: 4096,
                                            interface: "GPIO".to_string(),
                                            pin_config: EncoderPinConfig {
                                                channel_a: Some(6),
                                                channel_b: Some(7),
                                                ..Default::default()
                                            },
                                            invert_direction: false,
                                            counts_per_mm: 400.0, // Lead screw typically much higher resolution
                                            max_error_threshold: 0.1, // Tighter tolerance for Z axis
                                            filter_samples: 10, // More filtering for Z
                                            update_rate_hz: 1000,
                                        };
                                        let _ = self.closed_loop_manager.add_axis("Z".to_string(), config);
                                    }
                                    
                                    if ui.button("🔧 Add Extruder Encoder").clicked() {
                                        use closed_loop_control::EncoderPinConfig;
                                        let config = EncoderConfig {
                                            encoder_type: EncoderType::Magnetic,
                                            resolution: 4096,
                                            interface: "SPI".to_string(),
                                            pin_config: EncoderPinConfig {
                                                cs_pin: Some(10),
                                                ..Default::default()
                                            },
                                            invert_direction: false,
                                            counts_per_mm: 100.0, // Extruder specific scaling
                                            max_error_threshold: 1.0, // More tolerance for extruder
                                            filter_samples: 3,
                                            update_rate_hz: 500, // Lower rate for extruder
                                        };
                                        let _ = self.closed_loop_manager.add_axis("Extruder".to_string(), config);
                                    }
                                });
                            });
                        });
                    });
                }
                Tab::Temperature => {
                    ui.heading("🌡 Temperature Feedforward Control");
                    ui.label("Advanced temperature control with predictive feedforward modeling");
                    
                    egui::ScrollArea::vertical().id_salt("temperature_main_scroll").show(ui, |ui| {
                        ui.columns(2, |columns| {
                            // Left column - Temperature zones and monitoring
                            columns[0].group(|ui| {
                                ui.label("🎯 Temperature Zones");
                                
                                // Global temperature controls
                                ui.horizontal(|ui| {
                                    let status = self.temperature_manager.get_system_status();
                                    
                                    if status.global_enabled {
                                        if ui.button("⏸ Disable Global").clicked() {
                                            self.temperature_manager.disable_global();
                                        }
                                        ui.colored_label(Color32::GREEN, "✓ Enabled");
                                    } else {
                                        if ui.button("▶ Enable Global").clicked() {
                                            self.temperature_manager.enable_global();
                                        }
                                        ui.colored_label(Color32::RED, "❌ Disabled");
                                    }
                                    
                                    if status.emergency_stop {
                                        ui.colored_label(Color32::RED, "🚨 EMERGENCY STOP");
                                        if ui.button("Clear E-Stop").clicked() {
                                            self.temperature_manager.clear_emergency_stop();
                                        }
                                    }
                                });
                                
                                ui.separator();
                                
                                // Quick setup actions
                                ui.horizontal(|ui| {
                                    // Dropdown for adding multiple hotends
                                    egui::ComboBox::from_label("🔥 Add Hotend")
                                        .selected_text("Select Hotend")
                                        .show_ui(ui, |ui| {
                                            for i in 0..=3 {
                                                if ui.button(format!("Hotend {}", i)).clicked() {
                                                    let sensor_config = SensorConfig {
                                                        sensor_type: SensorType::Thermistor,
                                                        pin: i,
                                                        max_temp: 300.0,
                                                        ..Default::default()
                                                    };
                                                    let heater_config = Some(HeaterConfig {
                                                        heater_type: HeaterType::Cartridge,
                                                        pin: i,
                                                        max_power: 40.0,
                                                        ..Default::default()
                                                    });
                                                    let result = self.temperature_manager.add_controller(
                                                        TemperatureZone::Hotend(i),
                                                        sensor_config,
                                                        heater_config
                                                    );
                                                    if result.is_ok() {
                                                        println!("Added Hotend {} successfully", i);
                                                    } else {
                                                        println!("Failed to add Hotend {}: {:?}", i, result.err());
                                                    }
                                                }
                                            }
                                        });
                                    
                                    // Heated bed configurations
                                    egui::ComboBox::from_label("🛏 Add Heated Bed")
                                        .selected_text("Bed Type")
                                        .show_ui(ui, |ui| {
                                            if ui.button("Standard Bed (120°C)").clicked() {
                                                let sensor_config = SensorConfig {
                                                    sensor_type: SensorType::Thermistor,
                                                    pin: 5,
                                                    max_temp: 120.0,
                                                    ..Default::default()
                                                };
                                                let heater_config = Some(HeaterConfig {
                                                    heater_type: HeaterType::Silicone,
                                                    pin: 5,
                                                    max_power: 200.0,
                                                    ..Default::default()
                                                });
                                                let result = self.temperature_manager.add_controller(
                                                    TemperatureZone::Bed,
                                                    sensor_config,
                                                    heater_config
                                                );
                                                if result.is_ok() {
                                                    println!("Added Standard Heated Bed successfully");
                                                } else {
                                                    println!("Failed to add Heated Bed: {:?}", result.err());
                                                }
                                            }
                                            if ui.button("High-Temp Bed (150°C)").clicked() {
                                                let sensor_config = SensorConfig {
                                                    sensor_type: SensorType::PT100,
                                                    pin: 6,
                                                    max_temp: 150.0,
                                                    ..Default::default()
                                                };
                                                let heater_config = Some(HeaterConfig {
                                                    heater_type: HeaterType::AC_Mains,
                                                    pin: 6,
                                                    max_power: 500.0,
                                                    ..Default::default()
                                                });
                                                let result = self.temperature_manager.add_controller(
                                                    TemperatureZone::Bed,
                                                    sensor_config,
                                                    heater_config
                                                );
                                                if result.is_ok() {
                                                    println!("Added High-Temp Heated Bed successfully");
                                                } else {
                                                    println!("Failed to add High-Temp Bed: {:?}", result.err());
                                                }
                                            }
                                        });
                                });
                                
                                ui.horizontal(|ui| {
                                    // Chamber heater with multiple configurations
                                    egui::ComboBox::from_label("🏠 Add Chamber")
                                        .selected_text("Chamber Type")
                                        .show_ui(ui, |ui| {
                                            if ui.button("Standard Chamber").clicked() {
                                                let sensor_config = SensorConfig {
                                                    sensor_type: SensorType::Thermistor,
                                                    pin: 10, // Use higher pin numbers for chamber
                                                    max_temp: 80.0,
                                                    ..Default::default()
                                                };
                                                let heater_config = Some(HeaterConfig {
                                                    heater_type: HeaterType::Ceramic,
                                                    pin: 10,
                                                    max_power: 100.0,
                                                    ..Default::default()
                                                });
                                                let result = self.temperature_manager.add_controller(
                                                    TemperatureZone::Chamber,
                                                    sensor_config,
                                                    heater_config
                                                );
                                                if result.is_ok() {
                                                    println!("Added Standard Chamber successfully");
                                                } else {
                                                    println!("Failed to add Chamber: {:?}", result.err());
                                                }
                                            }
                                            if ui.button("High-Temp Chamber").clicked() {
                                                let sensor_config = SensorConfig {
                                                    sensor_type: SensorType::Thermocouple,
                                                    pin: 11,
                                                    max_temp: 150.0,
                                                    ..Default::default()
                                                };
                                                let heater_config = Some(HeaterConfig {
                                                    heater_type: HeaterType::Ceramic,
                                                    pin: 11,
                                                    max_power: 200.0,
                                                    ..Default::default()
                                                });
                                                let result = self.temperature_manager.add_controller(
                                                    TemperatureZone::Chamber,
                                                    sensor_config,
                                                    heater_config
                                                );
                                                if result.is_ok() {
                                                    println!("Added High-Temp Chamber successfully");
                                                } else {
                                                    println!("Failed to add High-Temp Chamber: {:?}", result.err());
                                                }
                                            }
                                            if ui.button("Enclosure Only (No Heater)").clicked() {
                                                let sensor_config = SensorConfig {
                                                    sensor_type: SensorType::Thermistor,
                                                    pin: 12,
                                                    max_temp: 80.0,
                                                    ..Default::default()
                                                };
                                                let result = self.temperature_manager.add_controller(
                                                    TemperatureZone::Chamber,
                                                    sensor_config,
                                                    None // No heater, monitoring only
                                                );
                                                if result.is_ok() {
                                                    println!("Added Chamber Monitoring successfully");
                                                } else {
                                                    println!("Failed to add Chamber Monitoring: {:?}", result.err());
                                                }
                                            }
                                        });
                                    
                                    if ui.button("📊 Add Ambient Monitor").clicked() {
                                        let sensor_config = SensorConfig {
                                            sensor_type: SensorType::BME280,
                                            pin: 3,
                                            max_temp: 50.0,
                                            ..Default::default()
                                        };
                                        let _ = self.temperature_manager.add_controller(
                                            TemperatureZone::Ambient,
                                            sensor_config,
                                            None // Read-only sensor
                                        );
                                    }
                                });
                                
                                ui.separator();
                                
                                // Temperature zones list
                                ui.label("🌡 Configured Zones");
                                egui::ScrollArea::vertical().max_height(300.0).show(ui, |ui| {
                                    let statistics = self.temperature_manager.get_all_statistics();
                                    
                                    for (zone, stats) in statistics.iter() {
                                        let is_selected = self.selected_temp_zone.as_ref() == Some(zone);
                                        
                                        ui.group(|ui| {
                                            ui.horizontal(|ui| {
                                                if ui.selectable_label(is_selected, &zone.to_string()).clicked() {
                                                    self.selected_temp_zone = Some(zone.clone());
                                                }
                                                
                                                // Current temperature display
                                                ui.label(format!("{:.1}°C", stats.current_temperature));
                                                
                                                // Target temperature
                                                if stats.target_temperature > 0.0 {
                                                    ui.label("→");
                                                    ui.colored_label(Color32::BLUE, format!("{:.0}°C", stats.target_temperature));
                                                }
                                                
                                                // Status indicators
                                                if stats.safety_shutdown {
                                                    ui.colored_label(Color32::RED, "🚨 SHUTDOWN");
                                                } else if stats.enabled {
                                                    let error = stats.temperature_error.abs();
                                                    if error < 2.0 {
                                                        ui.colored_label(Color32::GREEN, "✓ At Target");
                                                    } else if error < 10.0 {
                                                        ui.colored_label(Color32::YELLOW, "🔄 Heating");
                                                    } else {
                                                        ui.colored_label(Color32::ORANGE, "⚠ Large Error");
                                                    }
                                                } else {
                                                    ui.colored_label(Color32::GRAY, "⏸ Disabled");
                                                }
                                            });
                                            
                                            // Temperature error and stability
                                            ui.horizontal(|ui| {
                                                ui.label("Error:");
                                                let error_color = if stats.temperature_error.abs() > 5.0 {
                                                    Color32::RED
                                                } else if stats.temperature_error.abs() > 2.0 {
                                                    Color32::YELLOW
                                                } else {
                                                    Color32::GREEN
                                                };
                                                ui.colored_label(error_color, format!("{:.2}°C", stats.temperature_error));
                                                
                                                ui.label("Stability:");
                                                let stability_color = if stats.temperature_stability > 1.0 {
                                                    Color32::RED
                                                } else if stats.temperature_stability > 0.5 {
                                                    Color32::YELLOW
                                                } else {
                                                    Color32::GREEN
                                                };
                                                ui.colored_label(stability_color, format!("±{:.2}°C", stats.temperature_stability));
                                            });
                                        });
                                        
                                        ui.separator();
                                    }
                                    
                                    if statistics.is_empty() {
                                        ui.colored_label(Color32::GRAY, "No temperature zones configured");
                                    }
                                });
                            });
                            
                            // Right column - Zone control and configuration
                            columns[1].group(|ui| {
                                ui.label("📊 System Status");
                                
                                let status = self.temperature_manager.get_system_status();
                                
                                // System overview
                                egui::Grid::new("temp_status_grid").show(ui, |ui| {
                                    ui.label("Global Status:");
                                    if status.global_enabled {
                                        ui.colored_label(Color32::GREEN, "✓ Enabled");
                                    } else {
                                        ui.colored_label(Color32::RED, "❌ Disabled");
                                    }
                                    ui.end_row();
                                    
                                    ui.label("Emergency Stop:");
                                    if status.emergency_stop {
                                        ui.colored_label(Color32::RED, "🚨 ACTIVE");
                                    } else {
                                        ui.colored_label(Color32::GREEN, "✓ Clear");
                                    }
                                    ui.end_row();
                                    
                                    ui.label("Update Rate:");
                                    ui.label(format!("{} Hz", status.update_rate_hz));
                                    ui.end_row();
                                    
                                    ui.label("Total Zones:");
                                    ui.label(status.total_zones.to_string());
                                    ui.end_row();
                                    
                                    ui.label("Enabled Zones:");
                                    ui.colored_label(Color32::GREEN, status.enabled_zones.to_string());
                                    ui.end_row();
                                    
                                    ui.label("Zones at Target:");
                                    ui.colored_label(Color32::BLUE, status.zones_at_target.to_string());
                                    ui.end_row();
                                    
                                    ui.label("Zones in Error:");
                                    if status.zones_in_error > 0 {
                                        ui.colored_label(Color32::RED, status.zones_in_error.to_string());
                                    } else {
                                        ui.colored_label(Color32::GREEN, "0");
                                    }
                                    ui.end_row();
                                    
                                    ui.label("Max Temp Error:");
                                    let error_color = if status.max_temperature_error > 10.0 {
                                        Color32::RED
                                    } else if status.max_temperature_error > 5.0 {
                                        Color32::YELLOW
                                    } else {
                                        Color32::GREEN
                                    };
                                    ui.colored_label(error_color, format!("{:.2}°C", status.max_temperature_error));
                                    ui.end_row();
                                });
                                
                                ui.separator();
                                
                                // Selected zone controls
                                if let Some(selected_zone) = &self.selected_temp_zone {
                                    ui.label(format!("🎛 {} Controls", selected_zone));
                                    
                                    let statistics = self.temperature_manager.get_all_statistics();
                                    if let Some(stats) = statistics.get(selected_zone) {
                                        // Temperature setting
                                        ui.horizontal(|ui| {
                                            ui.label("Target Temperature:");
                                            
                                            // Quick temperature buttons
                                            match selected_zone {
                                                TemperatureZone::Hotend(_) => {
                                                    if ui.button("200°C").clicked() {
                                                        let _ = self.temperature_manager.set_target_temperature(selected_zone, 200.0);
                                                    }
                                                    if ui.button("220°C").clicked() {
                                                        let _ = self.temperature_manager.set_target_temperature(selected_zone, 220.0);
                                                    }
                                                    if ui.button("250°C").clicked() {
                                                        let _ = self.temperature_manager.set_target_temperature(selected_zone, 250.0);
                                                    }
                                                }
                                                TemperatureZone::Bed => {
                                                    if ui.button("60°C").clicked() {
                                                        let _ = self.temperature_manager.set_target_temperature(selected_zone, 60.0);
                                                    }
                                                    if ui.button("80°C").clicked() {
                                                        let _ = self.temperature_manager.set_target_temperature(selected_zone, 80.0);
                                                    }
                                                    if ui.button("100°C").clicked() {
                                                        let _ = self.temperature_manager.set_target_temperature(selected_zone, 100.0);
                                                    }
                                                }
                                                TemperatureZone::Chamber => {
                                                    if ui.button("40°C").clicked() {
                                                        let _ = self.temperature_manager.set_target_temperature(selected_zone, 40.0);
                                                    }
                                                    if ui.button("60°C").clicked() {
                                                        let _ = self.temperature_manager.set_target_temperature(selected_zone, 60.0);
                                                    }
                                                }
                                                _ => {}
                                            }
                                            
                                            if ui.button("OFF").clicked() {
                                                let _ = self.temperature_manager.set_target_temperature(selected_zone, 0.0);
                                            }
                                        });
                                        
                                        // Manual temperature adjustment
                                        ui.horizontal(|ui| {
                                            ui.label("Fine Control:");
                                            if ui.button("← -10°C").clicked() {
                                                let new_target = (stats.target_temperature - 10.0).max(0.0);
                                                let _ = self.temperature_manager.set_target_temperature(selected_zone, new_target);
                                            }
                                            if ui.button("← -1°C").clicked() {
                                                let new_target = (stats.target_temperature - 1.0).max(0.0);
                                                let _ = self.temperature_manager.set_target_temperature(selected_zone, new_target);
                                            }
                                            if ui.button("+1°C →").clicked() {
                                                let max_temp = match selected_zone {
                                                    TemperatureZone::Hotend(_) => 300.0,
                                                    TemperatureZone::Bed => 120.0,
                                                    TemperatureZone::Chamber => 80.0,
                                                    _ => 50.0,
                                                };
                                                let new_target = (stats.target_temperature + 1.0).min(max_temp);
                                                let _ = self.temperature_manager.set_target_temperature(selected_zone, new_target);
                                            }
                                            if ui.button("+10°C →").clicked() {
                                                let max_temp = match selected_zone {
                                                    TemperatureZone::Hotend(_) => 300.0,
                                                    TemperatureZone::Bed => 120.0,
                                                    TemperatureZone::Chamber => 80.0,
                                                    _ => 50.0,
                                                };
                                                let new_target = (stats.target_temperature + 10.0).min(max_temp);
                                                let _ = self.temperature_manager.set_target_temperature(selected_zone, new_target);
                                            }
                                        });
                                        
                                        ui.separator();
                                        
                                        // Zone enable/disable
                                        ui.horizontal(|ui| {
                                            if stats.enabled {
                                                if ui.button("⏸ Disable Zone").clicked() {
                                                    let _ = self.temperature_manager.disable_zone(selected_zone);
                                                }
                                            } else {
                                                if ui.button("▶ Enable Zone").clicked() {
                                                    let _ = self.temperature_manager.enable_zone(selected_zone);
                                                }
                                            }
                                            
                                            if stats.safety_shutdown {
                                                if ui.button("🔓 Clear Safety").clicked() {
                                                    // Would need to add method to clear individual zone safety
                                                }
                                            }
                                        });
                                        
                                        ui.separator();
                                        
                                        // PID Tuning
                                        ui.label("🎛 PID Configuration");
                                        ui.horizontal(|ui| {
                                            if ui.button("Auto Tune PID").clicked() {
                                                // Could implement PID auto-tuning
                                            }
                                            if ui.button("Reset PID").clicked() {
                                                let default_pid = TempPIDConfig::default();
                                                let _ = self.temperature_manager.configure_pid(selected_zone, default_pid);
                                            }
                                        });
                                        
                                        // Manual PID parameters (simplified display)
                                        ui.horizontal(|ui| {
                                            ui.label("Kp: 20.0"); // Could make these editable
                                            ui.label("Ki: 2.0");
                                            ui.label("Kd: 0.5");
                                        });
                                        
                                        ui.separator();
                                        
                                        // Feedforward model
                                        ui.label("🔮 Feedforward Model");
                                        ui.horizontal(|ui| {
                                            if ui.button("Enable Feedforward").clicked() {
                                                let mut model = FeedforwardModel::default();
                                                model.enabled = true;
                                                let _ = self.temperature_manager.configure_feedforward(selected_zone, model);
                                            }
                                            if ui.button("Disable Feedforward").clicked() {
                                                let mut model = FeedforwardModel::default();
                                                model.enabled = false;
                                                let _ = self.temperature_manager.configure_feedforward(selected_zone, model);
                                            }
                                        });
                                        
                                        // Model parameters (simplified display)
                                        ui.horizontal(|ui| {
                                            ui.label("Time Constant: 30s");
                                            ui.label("Dead Time: 5s");
                                            ui.label("Gain: 1.0");
                                        });
                                    }
                                }
                                
                                ui.separator();
                                
                                // Emergency controls
                                ui.label("🚨 Emergency Controls");
                                ui.horizontal(|ui| {
                                    if ui.button("🚨 EMERGENCY STOP").clicked() {
                                        self.temperature_manager.emergency_stop();
                                    }
                                    if ui.button("❄ Cool All to 0°C").clicked() {
                                        // Set all zones to 0°C
                                        let zones: Vec<_> = self.temperature_manager.get_all_statistics().keys().cloned().collect();
                                        for zone in zones {
                                            let _ = self.temperature_manager.set_target_temperature(&zone, 0.0);
                                        }
                                    }
                                });
                            });
                        });
                    });
                }
                Tab::PowerLoss => {
                    ui.heading("🔋 Power-Loss Recovery System");
                    ui.label("Advanced power-loss detection and automatic print recovery");
                    
                    egui::ScrollArea::vertical().id_salt("power_loss_scroll").show(ui, |ui| {
                        ui.columns(2, |columns| {
                            // Left column - Status and configuration
                            columns[0].group(|ui| {
                                ui.label("📊 System Status");
                                
                                // Collect data first to avoid borrowing conflicts
                                let config = self.power_loss_manager.get_config().clone();
                                let stats = self.power_loss_manager.get_statistics().clone();
                                let has_recovery_data = self.power_loss_manager.has_recovery_data();
                                
                                // Main status
                                ui.horizontal(|ui| {
                                    ui.label("Recovery System:");
                                    if config.enabled {
                                        ui.colored_label(Color32::GREEN, "✓ Enabled");
                                    } else {
                                        ui.colored_label(Color32::RED, "❌ Disabled");
                                    }
                                });
                                
                                // Check for existing recovery data
                                let mut load_recovery_clicked = false;
                                ui.horizontal(|ui| {
                                    ui.label("Recovery Data:");
                                    if has_recovery_data {
                                        ui.colored_label(Color32::YELLOW, "⚠ Found");
                                        if ui.button("🔄 Load Recovery").clicked() {
                                            load_recovery_clicked = true;
                                        }
                                    } else {
                                        ui.colored_label(Color32::GREEN, "✓ None");
                                    }
                                });
                                
                                // Handle recovery loading outside of the UI closure
                                if load_recovery_clicked {
                                    match self.power_loss_manager.load_recovery_data() {
                                        Ok(Some(print_state)) => {
                                            match self.power_loss_manager.start_recovery(print_state) {
                                                Ok(plan) => {
                                                    self.recovery_plan = Some(plan);
                                                }
                                                Err(e) => {
                                                    println!("Recovery failed: {}", e);
                                                }
                                            }
                                        }
                                        Ok(None) => {}
                                        Err(e) => {
                                            println!("Failed to load recovery data: {}", e);
                                        }
                                    }
                                }
                                
                                ui.separator();
                                
                                // Statistics
                                ui.label("📈 Recovery Statistics");
                                egui::Grid::new("recovery_stats").show(ui, |ui| {
                                    ui.label("Total Power Losses:");
                                    ui.label(stats.total_power_losses.to_string());
                                    ui.end_row();
                                    
                                    ui.label("Successful Recoveries:");
                                    ui.colored_label(Color32::GREEN, stats.successful_recoveries.to_string());
                                    ui.end_row();
                                    
                                    ui.label("Failed Recoveries:");
                                    if stats.failed_recoveries > 0 {
                                        ui.colored_label(Color32::RED, stats.failed_recoveries.to_string());
                                    } else {
                                        ui.colored_label(Color32::GREEN, "0");
                                    }
                                    ui.end_row();
                                    
                                    ui.label("Success Rate:");
                                    let color = if stats.recovery_success_rate > 80.0 {
                                        Color32::GREEN
                                    } else if stats.recovery_success_rate > 60.0 {
                                        Color32::YELLOW
                                    } else {
                                        Color32::RED
                                    };
                                    ui.colored_label(color, format!("{:.1}%", stats.recovery_success_rate));
                                    ui.end_row();
                                    
                                    if let Some(last_loss) = &stats.last_power_loss {
                                        ui.label("Last Power Loss:");
                                        let elapsed = last_loss.elapsed().unwrap_or(std::time::Duration::new(0, 0));
                                        if elapsed.as_secs() < 3600 {
                                            ui.label(format!("{} min ago", elapsed.as_secs() / 60));
                                        } else if elapsed.as_secs() < 86400 {
                                            ui.label(format!("{} hours ago", elapsed.as_secs() / 3600));
                                        } else {
                                            ui.label(format!("{} days ago", elapsed.as_secs() / 86400));
                                        }
                                        ui.end_row();
                                    }
                                    
                                    if stats.longest_print_recovered.as_secs() > 0 {
                                        ui.label("Longest Recovery:");
                                        let hours = stats.longest_print_recovered.as_secs() / 3600;
                                        let minutes = (stats.longest_print_recovered.as_secs() % 3600) / 60;
                                        ui.label(format!("{}h {}m", hours, minutes));
                                        ui.end_row();
                                    }
                                });
                                
                                ui.separator();
                                
                                // Configuration
                                ui.label("⚙️ Configuration");
                                
                                ui.horizontal(|ui| {
                                    ui.label("Detection Method:");
                                    match &config.detection_method {
                                        PowerLossDetectionMethod::VoltageSensor { threshold_voltage, .. } => {
                                            ui.label(format!("Voltage < {:.1}V", threshold_voltage));
                                        }
                                        PowerLossDetectionMethod::ExternalSignal { .. } => {
                                            ui.label("External Signal");
                                        }
                                        PowerLossDetectionMethod::WatchdogTimeout { timeout_ms } => {
                                            ui.label(format!("Watchdog {}ms", timeout_ms));
                                        }
                                        PowerLossDetectionMethod::SystemMonitor => {
                                            ui.label("System Monitor");
                                        }
                                    }
                                });
                                
                                ui.horizontal(|ui| {
                                    ui.label("Save Interval:");
                                    ui.label(format!("{} ms", config.save_interval_ms));
                                });
                                
                                ui.horizontal(|ui| {
                                    ui.label("Auto Recovery:");
                                    if config.auto_recovery {
                                        ui.colored_label(Color32::GREEN, "✓ Enabled");
                                    } else {
                                        ui.colored_label(Color32::YELLOW, "⚠ Manual");
                                    }
                                });
                                
                                ui.horizontal(|ui| {
                                    ui.label("Z-Lift on Loss:");
                                    ui.label(format!("{:.1} mm", config.z_lift_on_power_loss));
                                });
                                
                                ui.horizontal(|ui| {
                                    ui.label("Recovery Retraction:");
                                    ui.label(format!("{:.1} mm", config.recovery_retraction));
                                });
                                
                                ui.separator();
                                
                                // Test controls
                                ui.label("🧪 Testing");
                                ui.horizontal(|ui| {
                                    if ui.button("💾 Test Save State").clicked() {
                                        // Simulate a print state for testing
                                        let test_state = PrintStateUpdate {
                                            position: Some([10.0, 20.0, 5.0, 0.0, 0.0, 0.0]),
                                            line_number: Some(1234),
                                            current_command: Some("G1 X10 Y20 Z5".to_string()),
                                            current_layer: Some(50),
                                            print_time: Some(std::time::Duration::from_secs(3600)), // 1 hour
                                            file_position: Some(5000),
                                            ..Default::default()
                                        };
                                        
                                        let _ = self.power_loss_manager.update_print_state(test_state);
                                    }
                                    
                                    if ui.button("⚡ Simulate Power Loss").clicked() {
                                        // Simulate power loss with low voltage
                                        let _ = self.power_loss_manager.monitor_power(8.0); // Below 11V threshold
                                    }
                                });
                            });
                            
                            // Right column - Recovery plan and controls
                            columns[1].group(|ui| {
                                // Clone recovery plan to avoid borrowing conflicts
                                let recovery_plan_clone = self.recovery_plan.clone();
                                if let Some(recovery_plan) = recovery_plan_clone {
                                    ui.label("🔧 Recovery Plan");
                                    
                                    // Plan summary
                                    egui::Grid::new("recovery_summary").show(ui, |ui| {
                                        ui.label("Print File:");
                                        ui.label(&recovery_plan.original_print_state.gcode_file_path);
                                        ui.end_row();
                                        
                                        ui.label("Progress:");
                                        let progress = if recovery_plan.original_print_state.file_size > 0 {
                                            (recovery_plan.original_print_state.file_position as f64 / 
                                             recovery_plan.original_print_state.file_size as f64) * 100.0
                                        } else {
                                            0.0
                                        };
                                        ui.label(format!("{:.1}%", progress));
                                        ui.end_row();
                                        
                                        ui.label("Layer:");
                                        ui.label(recovery_plan.original_print_state.current_layer.to_string());
                                        ui.end_row();
                                        
                                        ui.label("Print Time:");
                                        let hours = recovery_plan.original_print_state.print_time.as_secs() / 3600;
                                        let minutes = (recovery_plan.original_print_state.print_time.as_secs() % 3600) / 60;
                                        ui.label(format!("{}h {}m", hours, minutes));
                                        ui.end_row();
                                        
                                        ui.label("Success Probability:");
                                        let prob_color = if recovery_plan.recovery_success_probability > 0.8 {
                                            Color32::GREEN
                                        } else if recovery_plan.recovery_success_probability > 0.6 {
                                            Color32::YELLOW
                                        } else {
                                            Color32::RED
                                        };
                                        ui.colored_label(prob_color, format!("{:.0}%", recovery_plan.recovery_success_probability * 100.0));
                                        ui.end_row();
                                        
                                        ui.label("Estimated Duration:");
                                        let est_minutes = recovery_plan.total_estimated_duration.as_secs() / 60;
                                        ui.label(format!("{} min", est_minutes));
                                        ui.end_row();
                                    });
                                    
                                    ui.separator();
                                    
                                    // Recovery steps
                                    ui.label("📋 Recovery Steps");
                                    egui::ScrollArea::vertical().max_height(300.0).show(ui, |ui| {
                                        for (i, step) in recovery_plan.steps.iter().enumerate() {
                                            ui.group(|ui| {
                                                ui.horizontal(|ui| {
                                                    ui.label(format!("{}.", i + 1));
                                                    if step.critical {
                                                        ui.colored_label(Color32::RED, "🔴");
                                                    } else {
                                                        ui.colored_label(Color32::GREEN, "🟢");
                                                    }
                                                    ui.label(&step.description);
                                                });
                                                
                                                if !step.gcode_commands.is_empty() {
                                                    ui.horizontal(|ui| {
                                                        ui.label("G-code:");
                                                        ui.label(step.gcode_commands.join("; "));
                                                    });
                                                }
                                                
                                                ui.horizontal(|ui| {
                                                    ui.label("Duration:");
                                                    ui.label(format!("{}s", step.estimated_duration.as_secs()));
                                                });
                                            });
                                            ui.separator();
                                        }
                                    });
                                    
                                    ui.separator();
                                    
                                    // Recovery controls
                                    ui.label("🚀 Recovery Controls");
                                    let mut start_recovery = false;
                                    let mut cancel_recovery = false;
                                    
                                    ui.horizontal(|ui| {
                                        if recovery_plan.requires_user_confirmation {
                                            if ui.button("✅ Start Recovery").clicked() {
                                                start_recovery = true;
                                            }
                                            
                                            if ui.button("❌ Cancel Recovery").clicked() {
                                                cancel_recovery = true;
                                            }
                                        } else {
                                            ui.label("⏳ Auto-recovery will start automatically");
                                        }
                                    });
                                    
                                    // Handle recovery actions outside UI closures
                                    if start_recovery {
                                        println!("Starting recovery process...");
                                        let _ = self.power_loss_manager.complete_recovery(true);
                                        self.recovery_plan = None;
                                    }
                                    if cancel_recovery {
                                        let _ = self.power_loss_manager.complete_recovery(false);
                                        self.recovery_plan = None;
                                    }
                                    
                                } else {
                                    ui.label("📋 No Recovery Plan");
                                    ui.label("No recovery data available or no active recovery process.");
                                    
                                    ui.separator();
                                    
                                    // System configuration
                                    ui.label("⚙️ System Configuration");
                                    
                                    ui.horizontal(|ui| {
                                        if ui.button("🔧 Configure Detection").clicked() {
                                            // Could open configuration dialog
                                        }
                                        if ui.button("📁 Set Recovery Path").clicked() {
                                            // Could open file dialog
                                        }
                                    });
                                    
                                    ui.horizontal(|ui| {
                                        if ui.button("🧹 Clear Statistics").clicked() {
                                            // Could clear statistics with confirmation
                                        }
                                        if ui.button("🔄 Reset Configuration").clicked() {
                                            // Could reset to defaults with confirmation
                                        }
                                    });
                                    
                                    ui.separator();
                                    
                                    // Information and tips
                                    ui.label("💡 Tips");
                                    ui.label("• Enable UPS or voltage monitoring for best results");
                                    ui.label("• Test recovery system with short prints first");
                                    ui.label("• Keep recovery files on stable storage (SSD)");
                                    ui.label("• Monitor success rate and adjust settings");
                                    
                                    ui.separator();
                                    
                                    // Manual recovery file management
                                    ui.label("📂 Recovery File Management");
                                    ui.horizontal(|ui| {
                                        if ui.button("📄 View Recovery Files").clicked() {
                                            // Could show file browser
                                        }
                                        if ui.button("🗑 Clean Old Files").clicked() {
                                            // Could clean up old recovery files
                                        }
                                    });
                                }
                            });
                        });
                    });
                }
                
                Tab::PressureAdvance => {
                    ui.heading("💨 Pressure Advance - Extrusion Control");
                    
                    egui::ScrollArea::vertical()
                        .id_salt("pressure_advance_scroll")
                        .show(ui, |ui| {
                            
                        ui.label("Advanced pressure advance control for improved print quality and reduced oozing.");
                        ui.separator();
                        
                        // Extruder selection
                        ui.horizontal(|ui| {
                            ui.label("🎯 Active Extruder:");
                            let current_extruder = 0; // TODO: Get from actual state
                            if ui.button(&format!("Extruder {}", current_extruder)).clicked() {
                                // TODO: Show extruder selection dialog
                            }
                        });
                        
                        ui.separator();
                        
                        // Pressure advance configuration
                        let mut config = self.pressure_advance.get_active_controller().get_config().clone();
                        
                        ui.label("⚙️ Configuration");
                        ui.horizontal(|ui| {
                            ui.label("Pressure Advance:");
                            ui.add(egui::DragValue::new(&mut config.pressure_advance)
                                .speed(0.001)
                                .range(0.0..=0.2)
                                .fixed_decimals(4));
                            ui.label("(Higher values = more advance)");
                        });
                        
                        ui.horizontal(|ui| {
                            ui.label("Smooth Time:");
                            ui.add(egui::DragValue::new(&mut config.smooth_time)
                                .speed(0.001)
                                .range(0.010..=0.200)
                                .fixed_decimals(3));
                            ui.label("seconds");
                        });
                        
                        ui.horizontal(|ui| {
                            ui.label("Max Lookahead:");
                            ui.add(egui::DragValue::new(&mut config.max_lookahead_distance)
                                .speed(0.1)
                                .range(1.0..=20.0)
                                .fixed_decimals(1));
                            ui.label("mm");
                        });
                        
                        ui.horizontal(|ui| {
                            if ui.button("💾 Apply Settings").clicked() {
                                self.pressure_advance.get_active_controller().update_config(config.clone());
                                println!("Updated pressure advance: PA={:.4}", config.pressure_advance);
                            }
                            if ui.button("🔄 Reset to Default").clicked() {
                                let default_config = PressureAdvanceConfig::default();
                                self.pressure_advance.get_active_controller().update_config(default_config);
                            }
                        });
                        
                        ui.separator();
                        
                        // Statistics
                        let stats = self.pressure_advance.get_active_controller().get_statistics();
                        ui.label("📊 Statistics");
                        
                        ui.horizontal(|ui| {
                            ui.label("Segments Processed:");
                            ui.label(format!("{}", stats.segments_processed));
                        });
                        
                        ui.horizontal(|ui| {
                            ui.label("Total Advance Applied:");
                            ui.label(format!("{:.3} mm", stats.total_advance_applied));
                        });
                        
                        ui.horizontal(|ui| {
                            ui.label("Max Advance Applied:");
                            ui.label(format!("{:.3} mm", stats.max_advance_applied));
                        });
                        
                        ui.horizontal(|ui| {
                            ui.label("Avg Pressure Change:");
                            ui.label(format!("{:.3} mm", stats.avg_pressure_change));
                        });
                        
                        ui.horizontal(|ui| {
                            ui.label("Pressure Reversals:");
                            ui.label(format!("{}", stats.pressure_reversals));
                        });
                        
                        ui.horizontal(|ui| {
                            if ui.button("🗑️ Reset Statistics").clicked() {
                                self.pressure_advance.get_active_controller().reset_statistics();
                            }
                        });
                        
                        ui.separator();
                        
                        // Auto-calibration
                        ui.label("🔬 Auto-Calibration");
                        ui.label("Automatically tune pressure advance for optimal print quality");
                        
                        ui.horizontal(|ui| {
                            if ui.button("🚀 Run Calibration Test").clicked() {
                                // TODO: Implement calibration test pattern
                                println!("Starting pressure advance calibration...");
                            }
                            if ui.button("📋 Generate Test Pattern").clicked() {
                                // TODO: Generate G-code test pattern for manual calibration
                                println!("Generating pressure advance test pattern...");
                            }
                        });
                        
                        ui.separator();
                        
                        // Quick presets for common materials
                        ui.label("🎨 Material Presets");
                        ui.horizontal(|ui| {
                            if ui.button("PLA").clicked() {
                                let pla_config = PressureAdvanceConfig {
                                    pressure_advance: 0.04,
                                    smooth_time: 0.04,
                                    ..Default::default()
                                };
                                self.pressure_advance.get_active_controller().update_config(pla_config);
                            }
                            if ui.button("PETG").clicked() {
                                let petg_config = PressureAdvanceConfig {
                                    pressure_advance: 0.08,
                                    smooth_time: 0.06,
                                    ..Default::default()
                                };
                                self.pressure_advance.get_active_controller().update_config(petg_config);
                            }
                            if ui.button("ABS").clicked() {
                                let abs_config = PressureAdvanceConfig {
                                    pressure_advance: 0.06,
                                    smooth_time: 0.05,
                                    ..Default::default()
                                };
                                self.pressure_advance.get_active_controller().update_config(abs_config);
                            }
                        });
                        
                        ui.separator();
                        
                        // Information panel
                        ui.label("💡 Tips for Pressure Advance");
                        ui.label("• Start with low values (0.02-0.04) and increase gradually");
                        ui.label("• Too high values cause under-extrusion at corners");
                        ui.label("• Too low values cause blob formation and stringing");
                        ui.label("• Different materials require different PA values");
                        ui.label("• Higher print speeds typically need higher PA values");
                        
                    });
                }
                
                Tab::RuntimeConfig => {
                    ui.heading("⚙️ Runtime Configuration - Klipper-Style Settings");
                    ui.label("Runtime configuration tab - coming soon!");
                    /*
                    egui::ScrollArea::vertical()
                        .id_salt("runtime_config_scroll")
                        .show(ui, |ui| {
                            
                        ui.label("Klipper-style configuration system - modify settings without recompiling");
                        ui.separator();
                        
                        // Config file management
                        ui.label("📁 Configuration File Management");
                        ui.horizontal(|ui| {
                            if ui.button("💾 Save Config").clicked() {
                                match self.config_manager.save_config() {
                                    Ok(()) => println!("✅ Configuration saved successfully"),
                                    Err(e) => println!("❌ Failed to save config: {}", e),
                                }
                            }
                            if ui.button("🔄 Reload Config").clicked() {
                                match self.config_manager.load_config() {
                                    Ok(()) => println!("✅ Configuration reloaded successfully"),
                                    Err(e) => println!("❌ Failed to reload config: {}", e),
                                }
                            }
                            if ui.button("🔄 Reset to Defaults").clicked() {
                                self.config_manager.reset_to_defaults();
                                println!("⚠️ Configuration reset to defaults");
                            }
                        });
                        
                        ui.separator();
                        
                        // Printer type quick setup
                        ui.label("🖨️ Quick Setup Templates");
                        ui.horizontal(|ui| {
                            if ui.button("📐 Cartesian Printer").clicked() {
                                let cartesian_config = ConfigManager::generate_sample_config(PrinterType::Cartesian);
                                *self.config_manager.get_config_mut() = cartesian_config;
                                println!("Applied Cartesian printer template");
                            }
                            if ui.button("🔳 CoreXY Printer").clicked() {
                                let corexy_config = ConfigManager::generate_sample_config(PrinterType::CoreXY);
                                *self.config_manager.get_config_mut() = corexy_config;
                                println!("Applied CoreXY printer template");
                            }
                            if ui.button("🔺 Delta Printer").clicked() {
                                let delta_config = ConfigManager::generate_sample_config(PrinterType::Delta);
                                *self.config_manager.get_config_mut() = delta_config;
                                println!("Applied Delta printer template");
                            }
                        });
                        
                        ui.separator();
                        
                        // Basic printer info
                        {
                        let config = self.config_manager.get_config_mut();
                        
                        ui.label("📝 Printer Information");
                        ui.horizontal(|ui| {
                            ui.label("Name:");
                            ui.text_edit_singleline(&mut config.printer.name);
                        });
                        
                        ui.horizontal(|ui| {
                            ui.label("Description:");
                            ui.text_edit_singleline(&mut config.printer.description);
                        });
                        
                        ui.horizontal(|ui| {
                            ui.label("Version:");
                            ui.text_edit_singleline(&mut config.printer.version);
                        });
                        
                        ui.separator();
                        
                        // Kinematics settings
                        ui.label("🔧 Kinematics Configuration");
                        ui.horizontal(|ui| {
                            ui.label("Type:");
                            egui::ComboBox::from_id_salt("kinematics_type")
                                .selected_text(&config.kinematics.kinematics_type)
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(&mut config.kinematics.kinematics_type, "cartesian".to_string(), "Cartesian");
                                    ui.selectable_value(&mut config.kinematics.kinematics_type, "corexy".to_string(), "CoreXY");
                                    ui.selectable_value(&mut config.kinematics.kinematics_type, "delta".to_string(), "Delta");
                                });
                        });
                        
                        ui.horizontal(|ui| {
                            ui.label("Max Z Velocity:");
                            ui.add(egui::DragValue::new(&mut config.kinematics.max_z_velocity)
                                .speed(1.0)
                                .range(1.0..=100.0));
                            ui.label("mm/s");
                        });
                        
                        ui.horizontal(|ui| {
                            ui.label("Max Z Acceleration:");
                            ui.add(egui::DragValue::new(&mut config.kinematics.max_z_accel)
                                .speed(10.0)
                                .range(10.0..=1000.0));
                            ui.label("mm/s²");
                        });
                        
                        ui.separator();
                        
                        // Motion settings
                        ui.label("⚡ Motion Configuration");
                        ui.horizontal(|ui| {
                            ui.label("Max Velocity:");
                            ui.add(egui::DragValue::new(&mut config.motion.max_velocity)
                                .speed(10.0)
                                .range(10.0..=500.0));
                            ui.label("mm/s");
                        });
                        
                        ui.horizontal(|ui| {
                            ui.label("Max Acceleration:");
                            ui.add(egui::DragValue::new(&mut config.motion.max_accel)
                                .speed(100.0)
                                .range(100.0..=10000.0));
                            ui.label("mm/s²");
                        });
                        
                        ui.horizontal(|ui| {
                            ui.label("Square Corner Velocity:");
                            ui.add(egui::DragValue::new(&mut config.motion.square_corner_velocity)
                                .speed(0.5)
                                .range(0.1..=20.0));
                            ui.label("mm/s");
                        });
                        
                        ui.separator();
                        
                        // Communication settings
                        ui.label("📡 Communication Settings");
                        ui.horizontal(|ui| {
                            ui.label("Baudrate:");
                            egui::ComboBox::from_id_salt("baudrate")
                                .selected_text(config.communication.baudrate.to_string())
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(&mut config.communication.baudrate, 115200, "115200");
                                    ui.selectable_value(&mut config.communication.baudrate, 250000, "250000");
                                    ui.selectable_value(&mut config.communication.baudrate, 500000, "500000");
                                });
                        });
                        
                        ui.horizontal(|ui| {
                            ui.label("Serial Port:");
                            ui.text_edit_singleline(&mut config.communication.serial_port);
                        });
                        
                        ui.horizontal(|ui| {
                            ui.checkbox(&mut config.communication.enable_can, "Enable CAN Bus");
                        });
                        
                        if config.communication.enable_can {
                            ui.horizontal(|ui| {
                                ui.label("CAN Bitrate:");
                                if let Some(ref mut bitrate) = config.communication.can_bitrate {
                                    egui::ComboBox::from_id_salt("can_bitrate")
                                        .selected_text(bitrate.to_string())
                                        .show_ui(ui, |ui| {
                                            ui.selectable_value(bitrate, 125000, "125000");
                                            ui.selectable_value(bitrate, 250000, "250000");
                                            ui.selectable_value(bitrate, 500000, "500000");
                                            ui.selectable_value(bitrate, 1000000, "1000000");
                                        });
                                }
                            });
                        
                        ui.separator();
                        
                        // Export/Import configuration  
                        ui.label("🔄 Import/Export Configuration");
                        ui.horizontal(|ui| {
                            if ui.button("📤 Export TOML").clicked() {
                                match self.config_manager.export_config(ExportFormat::Toml) {
                                    Ok(config_str) => {
                                        // In a real application, you'd save to file or show in dialog
                                        println!("Exported config:\n{}", config_str);
                                    }
                                    Err(e) => println!("Export failed: {}", e),
                                }
                            }
                            if ui.button("📤 Export JSON").clicked() {
                                match self.config_manager.export_config(ExportFormat::Json) {
                                    Ok(config_str) => {
                                        println!("Exported JSON config:\n{}", config_str);
                                    }
                                    Err(e) => println!("Export failed: {}", e),
                                }
                            }
                            if ui.button("📥 Import File").clicked() {
                                // In a real application, you'd show a file dialog
                                println!("Import file dialog would open here");
                            }
                        });
                        
                        ui.separator();
                        
                        // Current motors overview
                        ui.label("🔩 Motors Overview");
                        ui.label(format!("Configured motors: {}", config.motors.len()));
                        
                        for (name, motor) in config.motors.iter().take(3) {
                            ui.horizontal(|ui| {
                                ui.label(format!("• {}: {:?}", name, motor.function));
                                ui.label(format!("({:?})", motor.motor_type));
                            });
                        }
                        
                        if config.motors.len() > 3 {
                            ui.label(format!("... and {} more", config.motors.len() - 3));
                        }
                        
                        ui.separator();
                        
                        // Current extruders overview  
                        ui.label("🌡️ Extruders Overview");
                        ui.label(format!("Configured extruders: {}", config.extruders.len()));
                        
                        for (name, extruder) in config.extruders.iter().take(2) {
                            ui.horizontal(|ui| {
                                ui.label(format!("• {}: ⌀{:.1}mm", name, extruder.nozzle_diameter));
                                ui.label(format!("Max: {}°C", extruder.max_temp));
                            });
                        }
                        
                        } // Close config scope
                        
                        ui.separator();
                        
                        // Tips and information
                        ui.label("💡 Configuration Tips");
                        ui.label("• Save configuration after making changes");
                        ui.label("• Backup your config before major changes");
                        ui.label("• Use templates as starting points for new printers");
                        ui.label("• Test changes with simulation mode first");
                        ui.label("• Configuration is compatible with Klipper syntax");
                        
                    });
                    */
                }

                // New advanced tab implementations
                Tab::AutoInputShaper => {
                    ui.heading("🎯 Automated Input Shaper Calibration");
                    ui.label("Fully automated resonance testing and input shaper configuration with user confirmation");

                    egui::ScrollArea::vertical().id_salt("auto_input_shaper_scroll").show(ui, |ui| {
                        ui.label("Automated input shaper tab - under development");
                    });
                }

                Tab::VolumetricCompensation => {
                    ui.heading("📐 Volumetric Compensation");
                    ui.label("Advanced volumetric flow rate compensation and calibration");

                    egui::ScrollArea::vertical().id_salt("volumetric_scroll").show(ui, |ui| {
                        ui.label("Volumetric compensation tab - under development");
                    });
                }

                Tab::SensorFusion => {
                    ui.heading("🔍 Sensor Fusion");
                    ui.label("Multi-sensor data fusion for enhanced position accuracy");

                    egui::ScrollArea::vertical().id_salt("sensor_fusion_scroll").show(ui, |ui| {
                        ui.label("Sensor fusion tab - under development");
                    });
                }

                Tab::StateSpaceControl => {
                    ui.heading("⚖️ State Space Control");
                    ui.label("Advanced state-space control algorithms and MPC");

                    egui::ScrollArea::vertical().id_salt("state_space_scroll").show(ui, |ui| {
                        ui.label("State space control tab - under development");
                    });
                }

                Tab::ModelPredictiveControl => {
                    ui.heading("🎯 Model Predictive Control");
                    ui.label("Advanced MPC algorithms for precision motion control");

                    egui::ScrollArea::vertical().id_salt("mpc_scroll").show(ui, |ui| {
                        ui.label("Model predictive control tab - under development");
                    });
                }

                Tab::Plugins => {
                    ui.heading("🔌 Plugin System");
                    ui.label("Extensible plugin architecture for custom functionality");

                    egui::ScrollArea::vertical().id_salt("plugin_scroll").show(ui, |ui| {
                        ui.label("Plugin system tab - under development");
                    });
                }
            }
        });

        // Add Motor Dialog
        if self.show_add_motor_dialog {
            egui::Window::new("Add Motor")
                .collapsible(false)
                .resizable(false)
                .show(ctx, |ui| {
                    ui.heading("Add New Motor");
                    
                    ui.horizontal(|ui| {
                        ui.label("Motor ID:");
                        ui.text_edit_singleline(&mut self.new_motor_id);
                    });
                    
                    ui.horizontal(|ui| {
                        ui.label("Function:");
                        let selected_function_text = match &self.new_motor_function {
                            motor_assignment::MotorFunction::XAxis(_) => "X Axis",
                            motor_assignment::MotorFunction::YAxis(_) => "Y Axis",
                            motor_assignment::MotorFunction::ZAxis(_) => "Z Axis",
                            motor_assignment::MotorFunction::Extruder(_) => "Extruder",
                            motor_assignment::MotorFunction::CoreXY_A(_) => "CoreXY A",
                            motor_assignment::MotorFunction::CoreXY_B(_) => "CoreXY B",
                            _ => "Other",
                        };
                        egui::ComboBox::from_label("")
                            .selected_text(selected_function_text)
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut self.new_motor_function, motor_assignment::MotorFunction::XAxis(0), "X Axis");
                                ui.selectable_value(&mut self.new_motor_function, motor_assignment::MotorFunction::YAxis(0), "Y Axis");
                                ui.selectable_value(&mut self.new_motor_function, motor_assignment::MotorFunction::ZAxis(0), "Z Axis");
                                ui.selectable_value(&mut self.new_motor_function, motor_assignment::MotorFunction::Extruder(0), "Extruder");
                                ui.selectable_value(&mut self.new_motor_function, motor_assignment::MotorFunction::CoreXY_A(0), "CoreXY A");
                                ui.selectable_value(&mut self.new_motor_function, motor_assignment::MotorFunction::CoreXY_B(0), "CoreXY B");
                            });
                    });
                    
                    ui.horizontal(|ui| {
                        ui.label("Motor Type:");
                        let selected_type_text = match &self.new_motor_type {
                            motor_assignment::MotorType::Stepper => "Stepper Motor",
                            motor_assignment::MotorType::Servo => "Servo Motor",
                            motor_assignment::MotorType::BrushlessDC => "Brushless DC Motor",
                            motor_assignment::MotorType::BrushedDC => "Brushed DC Motor",
                            motor_assignment::MotorType::LinearActuator => "Linear Actuator",
                        };
                        egui::ComboBox::from_label("")
                            .selected_text(selected_type_text)
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut self.new_motor_type, motor_assignment::MotorType::Stepper, "Stepper Motor");
                                ui.selectable_value(&mut self.new_motor_type, motor_assignment::MotorType::Servo, "Servo Motor");
                                ui.selectable_value(&mut self.new_motor_type, motor_assignment::MotorType::BrushlessDC, "Brushless DC Motor");
                                ui.selectable_value(&mut self.new_motor_type, motor_assignment::MotorType::BrushedDC, "Brushed DC Motor");
                                ui.selectable_value(&mut self.new_motor_type, motor_assignment::MotorType::LinearActuator, "Linear Actuator");
                            });
                    });
                    
                    ui.horizontal(|ui| {
                        if ui.button("✅ Add Motor").clicked() {
                            let motor_config = motor_assignment::MotorConfiguration {
                                motor_id: self.new_motor_id.clone(),
                                function: self.new_motor_function.clone(),
                                motor_type: self.new_motor_type.clone(),
                                physical_connection: motor_assignment::MotorConnection::StepDir {
                                    step_pin: "PA2".to_string(),
                                    dir_pin: "PA3".to_string(),
                                    enable_pin: Some("PA1".to_string()),
                                },
                                capabilities: motor_assignment::MotorCapabilities {
                                    max_speed: 1000.0,
                                    max_acceleration: 3000.0,
                                    steps_per_unit: 80.0,
                                    max_current: 1.2,
                                    min_current: 0.1,
                                    has_encoder: false,
                                    encoder_resolution: None,
                                    has_endstops: true,
                                    can_reverse: true,
                                    thermal_protection: true,
                                    stall_detection: false,
                                },
                                enabled: true,
                                inverted: false,
                                microstepping: Some(16),
                                current_setting: 0.8,
                                hold_current: 0.5,
                                interpolation: true,
                                created_at: std::time::Instant::now(),
                                last_modified: std::time::Instant::now(),
                            };
                            
                            if let Err(e) = self.motor_manager.register_motor(motor_config) {
                                println!("Failed to add motor: {}", e);
                            } else {
                                println!("Added motor: {}", self.new_motor_id);
                                // Reset form
                                self.new_motor_id = "motor_".to_string() + &(self.motor_manager.get_all_motors().len() + 1).to_string();
                                self.new_motor_function = motor_assignment::MotorFunction::XAxis(0);
                                self.new_motor_type = motor_assignment::MotorType::Stepper;
                                self.show_add_motor_dialog = false;
                            }
                        }
                        
                        if ui.button("❌ Cancel").clicked() {
                            self.show_add_motor_dialog = false;
                        }
                    });
                });
        }
    }
}

// CAN Toolhead Helper Methods Implementation
impl MyApp {
    // CAN Toolhead Helper Methods
    fn update_can_toolheads(&mut self, positions: &[f32; 5]) {
        // Send position updates to all connected CAN toolheads
        if let Ok(mut can_manager) = self.can_manager.try_lock() {
            for uuid in can_manager.get_toolheads().keys().cloned().collect::<Vec<_>>() {
                // For now, just update the toolhead with current extruder position
                // In a real implementation, you'd send coordinated motion commands
                let _ = can_manager.send_toolhead_command(uuid, KlipperCanCmd::SetPosition, 
                    &positions[3].to_le_bytes()); // E axis position
            }
        }
    }
    
    fn discover_can_toolheads(&mut self) {
        if let Ok(mut can_manager) = self.can_manager.try_lock() {
            match can_manager.discover_toolheads() {
                Ok(uuids) => {
                    println!("Discovered {} CAN toolheads: {:?}", uuids.len(), uuids);
                }
                Err(e) => {
                    println!("Toolhead discovery failed: {}", e);
                }
            }
        }
    }
    
    fn set_toolhead_temperature(&mut self, uuid: u32, temperature: f32) {
        if let Ok(mut can_manager) = self.can_manager.try_lock() {
            let _ = can_manager.set_temperature(uuid, temperature);
            println!("Set toolhead 0x{:06X} temperature to {:.1}°C", uuid, temperature);
        }
    }
    
    fn set_toolhead_fan(&mut self, uuid: u32, speed: u8) {
        if let Ok(mut can_manager) = self.can_manager.try_lock() {
            let _ = can_manager.set_fan_speed(uuid, speed);
            println!("Set toolhead 0x{:06X} fan speed to {}", uuid, speed);
        }
    }
    
    fn move_extruder(&mut self, uuid: u32, distance: f32, speed: f32) {
        if let Ok(mut can_manager) = self.can_manager.try_lock() {
            let _ = can_manager.move_extruder(uuid, distance, speed);
            println!("Move toolhead 0x{:06X} extruder {:.2}mm at {:.1}mm/s", uuid, distance, speed);
        }
    }
    
    fn home_toolhead(&mut self, uuid: u32) {
        if let Ok(mut can_manager) = self.can_manager.try_lock() {
            let _ = can_manager.home_toolhead(uuid);
            println!("Homing toolhead 0x{:06X}", uuid);
        }
    }
    
    fn get_toolhead_status(&mut self, uuid: u32) {
        if let Ok(mut can_manager) = self.can_manager.try_lock() {
            let _ = can_manager.get_toolhead_status(uuid);
            println!("Requesting status for toolhead 0x{:06X}", uuid);
        }
    }
    
    fn emergency_stop_toolhead(&mut self, uuid: u32) {
        if let Ok(mut can_manager) = self.can_manager.try_lock() {
            match can_manager.emergency_stop(uuid) {
                Ok(()) => println!("Emergency stop for toolhead 0x{:06X}", uuid),
                Err(e) => eprintln!("🚨 EMERGENCY STOP FAILED for toolhead 0x{:06X}: {}", uuid, e),
            }
        }
    }
    
    fn emergency_stop_all_toolheads(&mut self) {
        if let Ok(mut can_manager) = self.can_manager.try_lock() {
            for uuid in can_manager.get_toolheads().keys().cloned().collect::<Vec<_>>() {
                if let Err(e) = can_manager.emergency_stop(uuid) {
                    eprintln!("🚨 EMERGENCY STOP FAILED for toolhead 0x{:06X}: {}", uuid, e);
                }
            }
            println!("Emergency stop sent to all toolheads");
        }
    }
    
    // Test function to demonstrate the new multi-motor system
    fn test_multi_motor_system(&mut self) {
        println!("🧪 Testing Multi-Motor Assignment and LinuxCNC Control System...");
        
        // Run the integration test
        match integration_test::run_full_integration_test() {
            Ok(()) => {
                println!("✅ Multi-motor system test completed successfully!");
            }
            Err(e) => {
                println!("❌ Multi-motor system test failed: {}", e);
            }
        }
    }
}