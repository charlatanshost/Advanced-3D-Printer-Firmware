// Power-Loss Recovery System
// This module provides comprehensive power-loss detection and recovery functionality

use std::collections::HashMap;
use std::fs::{File, OpenOptions};
use std::io::{Read, Write, Seek, SeekFrom};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use serde::{Deserialize, Serialize};

// Recovery file format version for compatibility
const RECOVERY_FILE_VERSION: u32 = 2;
const RECOVERY_FILE_HEADER: &[u8] = b"PLRV"; // Power Loss Recovery Version

// Power loss detection methods
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PowerLossDetectionMethod {
    VoltageSensor {
        pin: u8,
        threshold_voltage: f32,    // Voltage below which power loss is detected
        sample_rate_hz: u32,       // Monitoring frequency
    },
    ExternalSignal {
        pin: u8,
        active_low: bool,          // Whether signal is active low
    },
    WatchdogTimeout {
        timeout_ms: u32,           // Timeout before assuming power loss
    },
    SystemMonitor,                 // OS-level power monitoring (UPS, etc.)
}

impl Default for PowerLossDetectionMethod {
    fn default() -> Self {
        Self::VoltageSensor {
            pin: 0,
            threshold_voltage: 11.0,  // 12V supply minimum
            sample_rate_hz: 100,      // 100Hz monitoring
        }
    }
}

// Print state at time of power loss
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrintState {
    // File information
    pub gcode_file_path: String,
    pub file_size: u64,
    pub file_position: u64,           // Byte position in G-code file
    pub line_number: u32,             // Current line number
    pub current_command: String,      // Last executed G-code command
    
    // Position state
    pub position: [f64; 6],           // X, Y, Z, A, B, C positions
    pub feedrate: f64,                // Current feedrate
    pub acceleration: f64,            // Current acceleration
    pub jerk: f64,                    // Current jerk setting
    
    // Temperature state
    pub hotend_temperatures: HashMap<u8, f64>,    // Hotend targets
    pub bed_temperature: f64,         // Bed target temperature
    pub chamber_temperature: f64,     // Chamber target temperature
    
    // Extrusion state
    pub extruder_position: f64,       // Current extruder position
    pub relative_extrusion: bool,     // Whether in relative extrusion mode
    pub extrusion_multiplier: f64,    // Flow rate multiplier
    
    // Print settings
    pub layer_height: f64,            // Current layer height
    pub current_layer: u32,           // Current layer number
    pub print_speed: f64,             // Print speed percentage
    pub fan_speeds: HashMap<u8, f64>, // Fan speeds (0.0-1.0)
    
    // Timing information
    pub print_time: Duration,         // Total print time elapsed
    pub estimated_time_remaining: Option<Duration>,
    pub power_loss_time: SystemTime,  // When power loss was detected
    
    // Recovery metadata
    pub recovery_attempts: u32,       // Number of recovery attempts
    pub last_recovery_success: bool,  // Whether last recovery was successful
    pub recovery_notes: Vec<String>,  // Recovery-specific notes
    /// How far Z is known to have been lifted at power loss, on top of position[2].
    /// Stays 0 unless the motion system confirms the lift really happened.
    #[serde(default)]
    pub z_lift_applied: f64,
}

impl Default for PrintState {
    fn default() -> Self {
        Self {
            gcode_file_path: String::new(),
            file_size: 0,
            file_position: 0,
            line_number: 0,
            current_command: String::new(),
            position: [0.0; 6],
            feedrate: 1800.0,        // 30mm/s default
            acceleration: 3000.0,     // 3000mm/s² default
            jerk: 10.0,              // 10mm/s jerk default
            hotend_temperatures: HashMap::new(),
            bed_temperature: 0.0,
            chamber_temperature: 0.0,
            extruder_position: 0.0,
            relative_extrusion: false,
            extrusion_multiplier: 1.0,
            layer_height: 0.2,
            current_layer: 0,
            print_speed: 100.0,
            fan_speeds: HashMap::new(),
            print_time: Duration::new(0, 0),
            estimated_time_remaining: None,
            power_loss_time: SystemTime::now(),
            recovery_attempts: 0,
            last_recovery_success: false,
            recovery_notes: Vec::new(),
            z_lift_applied: 0.0,
        }
    }
}

// Recovery configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecoveryConfig {
    pub enabled: bool,
    pub detection_method: PowerLossDetectionMethod,
    pub save_interval_ms: u32,        // How often to save state
    pub recovery_file_path: PathBuf,  // Where to save recovery data
    pub backup_count: u8,             // Number of backup recovery files
    
    // Recovery behavior
    pub auto_recovery: bool,          // Automatically start recovery on power restore
    pub preheat_before_recovery: bool, // Preheat hotend/bed before resuming
    pub home_axes_before_recovery: bool, // Home axes before resuming
    pub resume_temperature_threshold: f64, // Min temp before resuming print
    pub recovery_retraction: f64,     // Retraction distance before power loss
    pub recovery_prime: f64,          // Prime amount after recovery
    pub z_lift_on_power_loss: f64,    // Z-axis lift when power loss detected
    
    // Safety settings
    pub max_recovery_attempts: u32,   // Maximum automatic recovery attempts
    pub recovery_timeout_hours: f64,  // Max time to attempt recovery
    pub require_user_confirmation: bool, // Require user to confirm recovery
}

impl Default for RecoveryConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            detection_method: PowerLossDetectionMethod::default(),
            save_interval_ms: 1000,  // Save every second
            recovery_file_path: PathBuf::from("recovery.plr"),
            backup_count: 3,
            
            auto_recovery: false,     // Safety first - require user confirmation
            preheat_before_recovery: true,
            home_axes_before_recovery: true,
            resume_temperature_threshold: 180.0, // Wait for 180°C before resuming
            recovery_retraction: 2.0, // 2mm retraction
            recovery_prime: 2.5,      // 2.5mm prime (slightly more than retraction)
            z_lift_on_power_loss: 5.0, // 5mm Z lift
            
            max_recovery_attempts: 3,
            recovery_timeout_hours: 24.0, // 24 hour window for recovery
            require_user_confirmation: true,
        }
    }
}

// Recovery file structure
#[derive(Debug, Clone, Serialize, Deserialize)]
struct RecoveryFileData {
    version: u32,
    creation_time: SystemTime,
    print_state: PrintState,
}

/// CRC-32 of the file's JSON bytes. It covers the bytes as written rather than a re-encoding
/// of the parsed data, because re-encoding is not stable: HashMap order is random per
/// process, and parsed floats can differ in the last bit.
fn recovery_checksum(payload: &[u8]) -> u32 {
    crc::Crc::<u32>::new(&crc::CRC_32_ISO_HDLC).checksum(payload)
}

// Recovery statistics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecoveryStatistics {
    pub total_power_losses: u32,
    pub successful_recoveries: u32,
    pub failed_recoveries: u32,
    pub average_recovery_time: Duration,
    pub last_power_loss: Option<SystemTime>,
    pub longest_print_recovered: Duration,
    pub recovery_success_rate: f64,   // Percentage
}

impl Default for RecoveryStatistics {
    fn default() -> Self {
        Self {
            total_power_losses: 0,
            successful_recoveries: 0,
            failed_recoveries: 0,
            average_recovery_time: Duration::new(0, 0),
            last_power_loss: None,
            longest_print_recovered: Duration::new(0, 0),
            recovery_success_rate: 0.0,
        }
    }
}

// Main power loss recovery manager
pub struct PowerLossRecoveryManager {
    config: RecoveryConfig,
    current_print_state: Option<PrintState>,
    last_save_time: Instant,
    power_loss_detected: bool,
    recovery_in_progress: bool,
    statistics: RecoveryStatistics,
    
    // Power monitoring
    voltage_readings: Vec<f32>,       // Recent voltage readings for averaging
    last_voltage_check: Instant,
    
    // Recovery file management
    recovery_file: Option<File>,
    backup_files: Vec<PathBuf>,
}

impl PowerLossRecoveryManager {
    pub fn new(config: RecoveryConfig) -> Result<Self, String> {
        let mut manager = Self {
            config: config.clone(),
            current_print_state: None,
            last_save_time: Instant::now(),
            power_loss_detected: false,
            recovery_in_progress: false,
            statistics: RecoveryStatistics::default(),
            voltage_readings: Vec::with_capacity(10),
            last_voltage_check: Instant::now(),
            recovery_file: None,
            backup_files: Vec::new(),
        };
        
        // Load existing statistics
        manager.load_statistics()?;
        
        // Initialize recovery file
        if config.enabled {
            manager.initialize_recovery_file()?;
        }
        
        println!("Power-Loss Recovery Manager initialized");
        Ok(manager)
    }
    
    // Start tracking a new print
    pub fn start_print_tracking(&mut self, gcode_file_path: String, file_size: u64) -> Result<(), String> {
        if !self.config.enabled {
            return Ok(());
        }
        
        let mut print_state = PrintState::default();
        print_state.gcode_file_path = gcode_file_path;
        print_state.file_size = file_size;
        print_state.power_loss_time = SystemTime::now();
        
        self.current_print_state = Some(print_state);
        self.power_loss_detected = false;
        self.recovery_in_progress = false;
        
        self.save_state()?;
        println!("Started print tracking for power-loss recovery");
        Ok(())
    }
    
    // Stop print tracking (print completed or cancelled)
    pub fn stop_print_tracking(&mut self) -> Result<(), String> {
        self.current_print_state = None;
        self.power_loss_detected = false;
        self.recovery_in_progress = false;
        
        // Remove the recovery file and its backup, or the next boot would offer to resume
        // a print that already finished.
        self.recovery_file = None;
        for path in [
            self.config.recovery_file_path.clone(),
            self.config.recovery_file_path.with_extension("bak"),
        ] {
            if path.exists() {
                std::fs::remove_file(&path)
                    .map_err(|e| format!("Failed to remove recovery file: {}", e))?;
            }
        }
        
        println!("Stopped print tracking");
        Ok(())
    }
    
    // Update current print state
    pub fn update_print_state(&mut self, updates: PrintStateUpdate) -> Result<(), String> {
        // Keep what was saved at the moment of the power loss until power returns.
        if self.power_loss_detected {
            return Ok(());
        }
        if let Some(ref mut state) = self.current_print_state {
            // Update fields based on the update structure
            if let Some(pos) = updates.position {
                state.position = pos;
            }
            if let Some(line) = updates.line_number {
                state.line_number = line;
            }
            if let Some(cmd) = updates.current_command {
                state.current_command = cmd;
            }
            if let Some(temp) = updates.hotend_temperatures {
                state.hotend_temperatures = temp;
            }
            if let Some(bed_temp) = updates.bed_temperature {
                state.bed_temperature = bed_temp;
            }
            if let Some(layer) = updates.current_layer {
                state.current_layer = layer;
            }
            if let Some(time) = updates.print_time {
                state.print_time = time;
            }
            if let Some(pos) = updates.file_position {
                state.file_position = pos;
            }
            if let Some(e) = updates.extruder_position {
                state.extruder_position = e;
            }
            if let Some(relative) = updates.relative_extrusion {
                state.relative_extrusion = relative;
            }
            if let Some(feedrate) = updates.feedrate {
                state.feedrate = feedrate;
            }
            if let Some(fans) = updates.fan_speeds {
                state.fan_speeds = fans;
            }
            
            // Check if it's time to save state
            let now = Instant::now();
            if now.duration_since(self.last_save_time).as_millis() >= self.config.save_interval_ms as u128 {
                self.save_state()?;
                self.last_save_time = now;
            }
        }
        
        Ok(())
    }
    
    // Monitor power and detect power loss
    pub fn monitor_power(&mut self, voltage: f32) -> Result<bool, String> {
        if !self.config.enabled {
            return Ok(false);
        }
        
        let now = Instant::now();
        
        // Add voltage reading to buffer
        self.voltage_readings.push(voltage);
        if self.voltage_readings.len() > 10 {
            self.voltage_readings.remove(0);
        }
        
        // Check power loss based on detection method
        let power_loss = match &self.config.detection_method {
            PowerLossDetectionMethod::VoltageSensor { threshold_voltage, sample_rate_hz, .. } => {
                // Check if enough time has passed for sampling
                let sample_interval = Duration::from_secs_f64(1.0 / (*sample_rate_hz).max(1) as f64);
                if now.duration_since(self.last_voltage_check) >= sample_interval {
                    self.last_voltage_check = now;
                    
                    // Calculate average voltage
                    let avg_voltage: f32 = self.voltage_readings.iter().sum::<f32>() / self.voltage_readings.len() as f32;
                    avg_voltage < *threshold_voltage
                } else {
                    false
                }
            }
            
            PowerLossDetectionMethod::ExternalSignal { .. } => {
                // Would be implemented with GPIO reading
                false // Placeholder
            }
            
            PowerLossDetectionMethod::WatchdogTimeout { .. } => {
                // Would be implemented with watchdog timer
                false // Placeholder
            }
            
            PowerLossDetectionMethod::SystemMonitor => {
                // Would be implemented with system-level monitoring
                false // Placeholder
            }
        };
        
        // Handle power loss detection
        if power_loss && !self.power_loss_detected {
            self.handle_power_loss()?;
        } else if !power_loss && self.power_loss_detected {
            // A brown-out that recovered: re-arm once the average is back above threshold
            if let PowerLossDetectionMethod::VoltageSensor { threshold_voltage, .. } = &self.config.detection_method {
                let average = self.voltage_readings.iter().sum::<f32>() / self.voltage_readings.len().max(1) as f32;
                if average >= *threshold_voltage {
                    self.power_loss_detected = false;
                }
            }
        }
        
        Ok(power_loss)
    }
    
    // Handle power loss event
    fn handle_power_loss(&mut self) -> Result<(), String> {
        println!("Power loss detected! Initiating emergency save...");
        self.power_loss_detected = true;
        
        // What gets saved is the state needed to resume: the print position and the
        // temperatures to reheat to. Heaters going off and the Z lift are actions for the
        // motion system (see emergency_gcode) and must not overwrite that state.
        if let Some(ref mut state) = self.current_print_state {
            state.power_loss_time = SystemTime::now();
        }

        self.save_state()?;
        
        // Update statistics
        self.statistics.total_power_losses += 1;
        self.statistics.last_power_loss = Some(SystemTime::now());
        self.save_statistics()?;
        
        println!("Emergency save completed!");
        Ok(())
    }
    
    /// Commands the motion system should run immediately on detecting power loss.
    pub fn emergency_gcode(&self) -> Vec<String> {
        let mut commands = vec!["M104 S0".to_string(), "M140 S0".to_string()];
        if self.config.recovery_retraction > 0.0 {
            commands.push("M83".to_string());
            commands.push(format!("G1 E{:.3} F2400", -self.config.recovery_retraction));
        }
        if self.config.z_lift_on_power_loss > 0.0 {
            commands.push("G91".to_string());
            commands.push(format!("G1 Z{:.3} F600", self.config.z_lift_on_power_loss));
            commands.push("G90".to_string());
        }
        commands
    }

    /// Record that the emergency Z lift really happened, so recovery can account for it.
    /// Without this, recovery assumes Z never moved: if that is wrong the nozzle ends up
    /// too high, which fails safe. Assuming a lift that never happened would crash it.
    pub fn confirm_emergency_lift(&mut self) -> Result<(), String> {
        let lift = self.config.z_lift_on_power_loss;
        if let Some(ref mut state) = self.current_print_state {
            state.z_lift_applied = lift;
        }
        self.save_state()
    }

    // Check if recovery data exists
    pub fn has_recovery_data(&self) -> bool {
        self.config.recovery_file_path.exists()
    }
    
    // Load recovery data
    pub fn load_recovery_data(&mut self) -> Result<Option<PrintState>, String> {
        if !self.has_recovery_data() {
            return Ok(None);
        }
        
        let mut file = File::open(&self.config.recovery_file_path)
            .map_err(|e| format!("Failed to open recovery file: {}", e))?;
        
        // Read and verify header
        let mut header = [0u8; 4];
        file.read_exact(&mut header)
            .map_err(|e| format!("Failed to read recovery file header: {}", e))?;
        
        if &header != RECOVERY_FILE_HEADER {
            return Err("Invalid recovery file format".to_string());
        }
        
        // Read the rest: one line with the checksum, then the JSON it covers
        let mut contents = Vec::new();
        file.read_to_end(&mut contents)
            .map_err(|e| format!("Failed to read recovery file data: {}", e))?;

        let newline = contents.iter().position(|&b| b == b'\n')
            .ok_or_else(|| "Recovery data corruption detected".to_string())?;
        let (checksum_line, payload) = (&contents[..newline], &contents[newline + 1..]);
        let stored_checksum = std::str::from_utf8(checksum_line).ok()
            .and_then(|text| u32::from_str_radix(text.trim(), 16).ok())
            .ok_or_else(|| "Unsupported recovery file format or corrupted data".to_string())?;

        // Verify data integrity
        if recovery_checksum(payload) != stored_checksum {
            return Err("Recovery data corruption detected".to_string());
        }

        let recovery_data: RecoveryFileData = serde_json::from_slice(payload)
            .map_err(|e| format!("Failed to deserialize recovery data: {}", e))?;

        // Verify version compatibility
        if recovery_data.version != RECOVERY_FILE_VERSION {
            return Err(format!("Unsupported recovery file version: {}", recovery_data.version));
        }

        // Check if recovery is still valid (within timeout window)
        let recovery_age = SystemTime::now()
            .duration_since(recovery_data.creation_time)
            .unwrap_or(Duration::from_secs(0));
        
        let max_age = Duration::from_secs_f64(self.config.recovery_timeout_hours * 3600.0);
        if recovery_age > max_age {
            return Err("Recovery data too old".to_string());
        }
        
        println!("Recovery data loaded successfully");
        Ok(Some(recovery_data.print_state))
    }
    
    // Start recovery process
    pub fn start_recovery(&mut self, mut print_state: PrintState) -> Result<RecoveryPlan, String> {
        if self.recovery_in_progress {
            return Err("Recovery already in progress".to_string());
        }
        if print_state.recovery_attempts >= self.config.max_recovery_attempts {
            return Err(format!(
                "Recovery already attempted {} times (limit {})",
                print_state.recovery_attempts, self.config.max_recovery_attempts
            ));
        }

        print_state.recovery_attempts += 1;
        self.recovery_in_progress = true;
        self.power_loss_detected = false;
        self.current_print_state = Some(print_state.clone());
        self.save_state()?;
        
        // Generate recovery plan
        let recovery_plan = self.generate_recovery_plan(&print_state)?;
        
        println!("Recovery plan generated with {} steps", recovery_plan.steps.len());
        Ok(recovery_plan)
    }
    
    // Generate recovery plan
    fn generate_recovery_plan(&self, state: &PrintState) -> Result<RecoveryPlan, String> {
        let mut steps = Vec::new();
        
        // Step 1: Safety checks
        steps.push(RecoveryStep {
            step_type: RecoveryStepType::SafetyCheck,
            description: "Perform safety checks".to_string(),
            gcode_commands: Vec::new(),
            estimated_duration: Duration::from_secs(5),
            critical: true,
        });
        
        // Step 2: Heat up first, bed before hotend, so the part stays stuck down and the
        // nozzle is not frozen into the print when Z starts moving.
        if self.config.preheat_before_recovery {
            let mut heat_commands = Vec::new();

            // Fixed order regardless of how the map happens to iterate
            let hotends: std::collections::BTreeMap<u8, f64> = state.hotend_temperatures.iter()
                .filter(|(_, temp)| **temp > 0.0)
                .map(|(hotend, temp)| (*hotend, *temp))
                .collect();

            // Start everything heating together
            if state.bed_temperature > 0.0 {
                heat_commands.push(format!("M140 S{}", state.bed_temperature));
            }
            for (hotend, temp) in &hotends {
                heat_commands.push(format!("M104 S{} T{}", temp, hotend));
            }

            // Then wait for each
            if state.bed_temperature > 0.0 {
                heat_commands.push(format!("M190 S{}", state.bed_temperature));
            }
            for (hotend, temp) in &hotends {
                heat_commands.push(format!("M109 S{} T{}", temp, hotend));
            }

            if !heat_commands.is_empty() {
                steps.push(RecoveryStep {
                    step_type: RecoveryStepType::Preheat,
                    description: "Preheat hotend and bed".to_string(),
                    gcode_commands: heat_commands,
                    estimated_duration: Duration::from_secs(300), // 5 minutes estimate
                    critical: true,
                });
            }
        }
        
        // Step 3: Establish Z without homing it, since the print is in the way. Declare where
        // Z is, then raise it clear. Only then is it safe to move in XY at all.
        let print_z = state.position[2];
        let known_z = print_z + state.z_lift_applied;
        let safe_z = known_z + self.config.z_lift_on_power_loss.max(0.0);
        let mut home_commands = vec![
            "G90".to_string(),
            format!("G92 Z{:.3}", known_z),
            format!("G1 Z{:.3} F600", safe_z),
        ];
        if self.config.home_axes_before_recovery {
            home_commands.push("G28 X Y".to_string());
        }
        steps.push(RecoveryStep {
            step_type: RecoveryStepType::HomeAxes,
            description: "Raise Z clear of the print and home X and Y".to_string(),
            gcode_commands: home_commands,
            estimated_duration: Duration::from_secs(30),
            critical: true,
        });

        // Step 4: Prime away from the part, then restore the extruder's counter and mode.
        // Without G92 E the next absolute E move would extrude everything since the start.
        steps.push(RecoveryStep {
            step_type: RecoveryStepType::PrimeExtruder,
            description: "Prime extruder and restore its position".to_string(),
            gcode_commands: vec![
                "M83".to_string(), // Relative extrusion for the prime
                format!("G1 E{:.3} F300", self.config.recovery_prime),
                format!("G92 E{:.5}", state.extruder_position),
                (if state.relative_extrusion { "M83" } else { "M82" }).to_string(),
            ],
            estimated_duration: Duration::from_secs(10),
            critical: true,
        });

        // Step 5: Back over the print, XY first and only then down, with fans and feedrate restored
        let mut position_commands = vec![
            "G90".to_string(),
            format!("G1 X{:.3} Y{:.3} F3000", state.position[0], state.position[1]),
            format!("G1 Z{:.3} F600", print_z),
        ];
        let fans: std::collections::BTreeMap<u8, f64> = state.fan_speeds.iter().map(|(fan, speed)| (*fan, *speed)).collect();
        for (fan, speed) in &fans {
            position_commands.push(format!("M106 P{} S{}", fan, (speed.clamp(0.0, 1.0) * 255.0).round() as u32));
        }
        position_commands.push(format!("G1 F{:.1}", state.feedrate));
        steps.push(RecoveryStep {
            step_type: RecoveryStepType::PositionRecovery,
            description: "Move to recovery position".to_string(),
            gcode_commands: position_commands,
            estimated_duration: Duration::from_secs(10),
            critical: true,
        });
        
        // Step 6: Resume print
        steps.push(RecoveryStep {
            step_type: RecoveryStepType::ResumePoint,
            description: format!("Resume from line {} in {}", state.line_number, state.gcode_file_path),
            gcode_commands: Vec::new(), // Will be handled by file positioning
            estimated_duration: Duration::from_secs(5),
            critical: true,
        });
        
        let total_duration = steps.iter().map(|s| s.estimated_duration).sum();
        
        Ok(RecoveryPlan {
            steps,
            total_estimated_duration: total_duration,
            requires_user_confirmation: self.config.require_user_confirmation,
            recovery_success_probability: self.calculate_success_probability(state),
            original_print_state: state.clone(),
        })
    }
    
    // Calculate success probability based on various factors
    fn calculate_success_probability(&self, state: &PrintState) -> f64 {
        let mut probability = 0.8; // Base 80% success rate
        
        // Adjust based on print progress (early failures are easier to recover)
        let progress = if state.file_size > 0 { state.file_position as f64 / state.file_size as f64 } else { 0.0 };
        if progress < 0.1 {
            probability += 0.15; // Early in print
        } else if progress > 0.8 {
            probability -= 0.1; // Late in print, more risk
        }
        
        // Adjust based on historical success rate
        if self.statistics.total_power_losses > 0 {
            let historical_rate = self.statistics.recovery_success_rate / 100.0;
            probability = (probability + historical_rate) / 2.0; // Average with historical
        }
        
        // Adjust based on recovery attempts
        probability -= (state.recovery_attempts as f64 * 0.1).min(0.3);
        
        probability.clamp(0.1, 0.95) // Keep between 10% and 95%
    }
    
    // Complete recovery
    pub fn complete_recovery(&mut self, success: bool) -> Result<(), String> {
        if !self.recovery_in_progress {
            return Err("No recovery in progress".to_string());
        }
        
        self.recovery_in_progress = false;
        
        // Update statistics
        if success {
            self.statistics.successful_recoveries += 1;
            println!("Recovery completed successfully!");
        } else {
            self.statistics.failed_recoveries += 1;
            println!("Recovery failed!");
        }
        
        // Update success rate
        let total_attempts = self.statistics.successful_recoveries + self.statistics.failed_recoveries;
        if total_attempts > 0 {
            self.statistics.recovery_success_rate = 
                (self.statistics.successful_recoveries as f64 / total_attempts as f64) * 100.0;
        }
        
        self.save_statistics()?;
        
        // Clean up recovery file if successful
        if success {
            self.stop_print_tracking()?;
        }
        
        Ok(())
    }
    
    // Save current state to recovery file
    fn save_state(&mut self) -> Result<(), String> {
        if let Some(ref state) = self.current_print_state {
            let recovery_data = RecoveryFileData {
                version: RECOVERY_FILE_VERSION,
                creation_time: SystemTime::now(),
                print_state: state.clone(),
            };
            
            // Serialize data
            let data_string = serde_json::to_string_pretty(&recovery_data)
                .map_err(|e| format!("Failed to serialize recovery data: {}", e))?;
            
            // Write to file with atomic operation
            let temp_path = self.config.recovery_file_path.with_extension("tmp");
            {
                let mut file = File::create(&temp_path)
                    .map_err(|e| format!("Failed to create recovery file: {}", e))?;
                
                // Write header
                file.write_all(RECOVERY_FILE_HEADER)
                    .map_err(|e| format!("Failed to write recovery header: {}", e))?;

                // Write checksum of the data that follows
                file.write_all(format!("{:08x}\n", recovery_checksum(data_string.as_bytes())).as_bytes())
                    .map_err(|e| format!("Failed to write recovery checksum: {}", e))?;
                
                // Write data
                file.write_all(data_string.as_bytes())
                    .map_err(|e| format!("Failed to write recovery data: {}", e))?;
                
                file.sync_all()
                    .map_err(|e| format!("Failed to sync recovery file: {}", e))?;
            }
            
            // Atomic move
            std::fs::rename(&temp_path, &self.config.recovery_file_path)
                .map_err(|e| format!("Failed to finalize recovery file: {}", e))?;
            
            // Create backup if configured
            if self.config.backup_count > 0 {
                self.create_backup()?;
            }
        }
        
        Ok(())
    }
    
    // Create backup recovery files
    fn create_backup(&mut self) -> Result<(), String> {
        let backup_path = self.config.recovery_file_path.with_extension("bak");
        std::fs::copy(&self.config.recovery_file_path, &backup_path)
            .map_err(|e| format!("Failed to create backup: {}", e))?;
        Ok(())
    }
    
    // Initialize recovery file
    fn initialize_recovery_file(&mut self) -> Result<(), String> {
        // Create parent directory if it doesn't exist
        if let Some(parent) = self.config.recovery_file_path.parent() {
            std::fs::create_dir_all(parent)
                .map_err(|e| format!("Failed to create recovery directory: {}", e))?;
        }
        
        Ok(())
    }
    
    // Load statistics
    fn load_statistics(&mut self) -> Result<(), String> {
        let stats_path = self.config.recovery_file_path.with_extension("stats");
        if stats_path.exists() {
            match std::fs::read_to_string(&stats_path) {
                Ok(data) => {
                    if let Ok(stats) = serde_json::from_str(&data) {
                        self.statistics = stats;
                    }
                }
                Err(_) => {} // Ignore errors, use defaults
            }
        }
        Ok(())
    }
    
    // Save statistics
    fn save_statistics(&self) -> Result<(), String> {
        let stats_path = self.config.recovery_file_path.with_extension("stats");
        let data = serde_json::to_string_pretty(&self.statistics)
            .map_err(|e| format!("Failed to serialize statistics: {}", e))?;
        
        std::fs::write(&stats_path, data)
            .map_err(|e| format!("Failed to save statistics: {}", e))?;
        
        Ok(())
    }
    
    // Get current statistics
    pub fn get_statistics(&self) -> &RecoveryStatistics {
        &self.statistics
    }
    
    // Get current configuration
    pub fn get_config(&self) -> &RecoveryConfig {
        &self.config
    }
    
    // Update configuration
    pub fn update_config(&mut self, config: RecoveryConfig) -> Result<(), String> {
        self.config = config;
        
        if self.config.enabled {
            self.initialize_recovery_file()?;
        }
        
        Ok(())
    }
}

// Update structure for print state
#[derive(Debug, Default)]
pub struct PrintStateUpdate {
    pub position: Option<[f64; 6]>,
    pub line_number: Option<u32>,
    pub current_command: Option<String>,
    pub hotend_temperatures: Option<HashMap<u8, f64>>,
    pub bed_temperature: Option<f64>,
    pub current_layer: Option<u32>,
    pub print_time: Option<Duration>,
    pub file_position: Option<u64>,
    pub extruder_position: Option<f64>,
    pub relative_extrusion: Option<bool>,
    pub feedrate: Option<f64>,
    pub fan_speeds: Option<HashMap<u8, f64>>,
}

// Recovery plan for user review
#[derive(Debug, Clone)]
pub struct RecoveryPlan {
    pub steps: Vec<RecoveryStep>,
    pub total_estimated_duration: Duration,
    pub requires_user_confirmation: bool,
    pub recovery_success_probability: f64,
    pub original_print_state: PrintState,
}

// Individual recovery step
#[derive(Debug, Clone)]
pub struct RecoveryStep {
    pub step_type: RecoveryStepType,
    pub description: String,
    pub gcode_commands: Vec<String>,
    pub estimated_duration: Duration,
    pub critical: bool,
}

// Types of recovery steps
#[derive(Debug, Clone)]
pub enum RecoveryStepType {
    SafetyCheck,
    HomeAxes,
    Preheat,
    PositionRecovery,
    PrimeExtruder,
    ResumePoint,
}

// Mock voltage sensor for testing
pub trait VoltageSensorInterface {
    fn read_voltage(&mut self) -> Result<f32, String>;
    fn is_connected(&self) -> bool;
}

pub struct MockVoltageSensor {
    voltage: f32,
    noise_amplitude: f32,
    connected: bool,
    power_loss_simulation: bool,
}

impl MockVoltageSensor {
    pub fn new(nominal_voltage: f32) -> Self {
        Self {
            voltage: nominal_voltage,
            noise_amplitude: 0.1,
            connected: true,
            power_loss_simulation: false,
        }
    }
    
    pub fn simulate_power_loss(&mut self, enable: bool) {
        self.power_loss_simulation = enable;
        if enable {
            self.voltage = 8.0; // Simulate voltage drop
        } else {
            self.voltage = 12.0; // Restore voltage
        }
    }
    
    pub fn set_noise(&mut self, amplitude: f32) {
        self.noise_amplitude = amplitude;
    }
}

impl VoltageSensorInterface for MockVoltageSensor {
    fn read_voltage(&mut self) -> Result<f32, String> {
        if !self.connected {
            return Err("Voltage sensor not connected".to_string());
        }
        
        use rand::Rng;
        let noise: f32 = if self.noise_amplitude > 0.0 {
            rand::thread_rng().gen_range(-self.noise_amplitude..self.noise_amplitude)
        } else {
            0.0
        };

        Ok(self.voltage + noise)
    }
    
    fn is_connected(&self) -> bool {
        self.connected
    }
}