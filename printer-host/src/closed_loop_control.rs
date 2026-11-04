// Closed-Loop Encoder Support System
// This module provides precise position feedback and error correction for motors

use std::collections::HashMap;
use std::time::{Duration, Instant};
use serde::{Deserialize, Serialize};

// Encoder types supported by the system
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum EncoderType {
    Quadrature,        // Traditional quadrature encoder
    Absolute,          // Absolute position encoder
    Magnetic,          // Magnetic encoder (AS5147, AS5600, etc.)
    Optical,           // Optical encoder
    Capacitive,        // Capacitive encoder
    Inductive,         // Inductive encoder
    Hall,              // Hall effect encoder
    SinCos,            // Sin/Cos encoder for high precision
    SSI,               // Synchronous Serial Interface encoder
    BiSS,              // BiSS interface encoder
    EnDat,             // EnDat interface encoder
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EncoderConfig {
    pub encoder_type: EncoderType,
    pub resolution: u32,           // Counts per revolution or absolute resolution
    pub interface: String,         // Interface connection ("SPI", "I2C", "GPIO", "UART", etc.)
    pub pin_config: EncoderPinConfig,
    pub invert_direction: bool,
    pub counts_per_mm: f64,        // For linear axes
    pub max_error_threshold: f64,   // Maximum position error before alarm
    pub filter_samples: u8,        // Number of samples for position filtering
    pub update_rate_hz: u32,       // Encoder reading frequency
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EncoderPinConfig {
    pub channel_a: Option<u8>,     // Quadrature A channel pin
    pub channel_b: Option<u8>,     // Quadrature B channel pin
    pub index: Option<u8>,         // Index/Z pulse pin
    pub cs_pin: Option<u8>,        // Chip select for SPI encoders
    pub data_pin: Option<u8>,      // Data pin for serial encoders
    pub clock_pin: Option<u8>,     // Clock pin for serial encoders
}

impl Default for EncoderPinConfig {
    fn default() -> Self {
        Self {
            channel_a: None,
            channel_b: None,
            index: None,
            cs_pin: None,
            data_pin: None,
            clock_pin: None,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EncoderState {
    pub position: i64,             // Current encoder position in counts
    pub velocity: f64,             // Current velocity in counts/sec
    pub last_position: i64,        // Previous position reading
    #[serde(skip, default = "Instant::now")]
    pub last_update: Instant,      // Last update timestamp
    pub error_count: u32,          // Number of read errors
    pub communication_errors: u32,  // Communication protocol errors
    pub position_error: f64,       // Current position error vs commanded
    pub max_position_error: f64,   // Maximum recorded position error
    pub connected: bool,           // Encoder communication status
    pub calibrated: bool,          // Whether encoder is calibrated/homed
}

impl Default for EncoderState {
    fn default() -> Self {
        Self {
            position: 0,
            velocity: 0.0,
            last_position: 0,
            last_update: Instant::now(),
            error_count: 0,
            communication_errors: 0,
            position_error: 0.0,
            max_position_error: 0.0,
            connected: false,
            calibrated: false,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PIDController {
    pub kp: f64,                   // Proportional gain
    pub ki: f64,                   // Integral gain  
    pub kd: f64,                   // Derivative gain
    pub integral: f64,             // Integral term accumulator
    pub last_error: f64,           // Last error for derivative calculation
    pub integral_limit: f64,       // Anti-windup limit for integral term
    pub output_limit: f64,         // Maximum output limit
    #[serde(skip, default = "Instant::now")]
    pub last_update: Instant,      // Last PID update time
}

impl Default for PIDController {
    fn default() -> Self {
        Self {
            kp: 1.0,
            ki: 0.1,
            kd: 0.01,
            integral: 0.0,
            last_error: 0.0,
            integral_limit: 100.0,
            output_limit: 100.0,
            last_update: Instant::now(),
        }
    }
}

impl PIDController {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            ..Default::default()
        }
    }
    
    pub fn update(&mut self, setpoint: f64, measured: f64) -> f64 {
        let now = Instant::now();
        let dt = now.duration_since(self.last_update).as_secs_f64();
        
        if dt <= 0.0 {
            return 0.0; // Prevent division by zero
        }
        
        let error = setpoint - measured;
        
        // Proportional term
        let p_term = self.kp * error;
        
        // Integral term with anti-windup
        self.integral += error * dt;
        if self.integral > self.integral_limit {
            self.integral = self.integral_limit;
        } else if self.integral < -self.integral_limit {
            self.integral = -self.integral_limit;
        }
        let i_term = self.ki * self.integral;
        
        // Derivative term
        let d_term = if dt > 0.0 {
            self.kd * (error - self.last_error) / dt
        } else {
            0.0
        };
        
        // Calculate output
        let output = p_term + i_term + d_term;
        
        // Apply output limits
        let limited_output = if output > self.output_limit {
            self.output_limit
        } else if output < -self.output_limit {
            -self.output_limit
        } else {
            output
        };
        
        // Update state
        self.last_error = error;
        self.last_update = now;
        
        limited_output
    }
    
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_error = 0.0;
        self.last_update = Instant::now();
    }
    
    pub fn set_gains(&mut self, kp: f64, ki: f64, kd: f64) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }
}

#[derive(Debug)]
pub struct ClosedLoopAxis {
    pub axis_name: String,
    pub encoder_config: EncoderConfig,
    pub encoder_state: EncoderState,
    pub pid_controller: PIDController,
    pub target_position: f64,      // Target position in mm or degrees
    pub current_position: f64,     // Current position in mm or degrees  
    pub enabled: bool,             // Whether closed-loop control is active
    pub homing_required: bool,     // Whether axis needs homing
    pub position_filter: PositionFilter,
}

impl ClosedLoopAxis {
    pub fn new(axis_name: String, encoder_config: EncoderConfig) -> Self {
        Self {
            axis_name,
            encoder_config,
            encoder_state: EncoderState::default(),
            pid_controller: PIDController::default(),
            target_position: 0.0,
            current_position: 0.0,
            enabled: false,
            homing_required: true,
            position_filter: PositionFilter::new(5), // 5-sample moving average
        }
    }
    
    pub fn update_encoder_position(&mut self, raw_position: i64) -> Result<(), String> {
        let now = Instant::now();
        let dt = now.duration_since(self.encoder_state.last_update).as_secs_f64();
        
        if dt <= 0.0 {
            return Ok(()); // Skip update if no time has passed
        }
        
        // Update velocity calculation
        let position_change = raw_position - self.encoder_state.last_position;
        self.encoder_state.velocity = position_change as f64 / dt;
        
        // Update position with filtering
        self.encoder_state.last_position = self.encoder_state.position;
        self.encoder_state.position = raw_position;
        
        // Convert to real-world units (mm or degrees)
        let filtered_position = self.position_filter.update(raw_position as f64);
        self.current_position = filtered_position / self.encoder_config.counts_per_mm;
        
        // Calculate position error
        self.encoder_state.position_error = self.target_position - self.current_position;
        
        // Track maximum position error
        let abs_error = self.encoder_state.position_error.abs();
        if abs_error > self.encoder_state.max_position_error {
            self.encoder_state.max_position_error = abs_error;
        }
        
        // Check for excessive position error
        if abs_error > self.encoder_config.max_error_threshold {
            return Err(format!(
                "Position error {} exceeds threshold {} on axis {}",
                abs_error, self.encoder_config.max_error_threshold, self.axis_name
            ));
        }
        
        self.encoder_state.last_update = now;
        self.encoder_state.connected = true;
        
        Ok(())
    }
    
    pub fn calculate_correction(&mut self) -> f64 {
        if !self.enabled || !self.encoder_state.calibrated {
            return 0.0;
        }
        
        self.pid_controller.update(self.target_position, self.current_position)
    }
    
    pub fn set_target_position(&mut self, position: f64) {
        self.target_position = position;
    }
    
    pub fn home_axis(&mut self) -> Result<(), String> {
        // Reset position to zero (or known home position)
        self.encoder_state.position = 0;
        self.current_position = 0.0;
        self.target_position = 0.0;
        self.encoder_state.calibrated = true;
        self.homing_required = false;
        self.pid_controller.reset();
        println!("Axis {} homed successfully", self.axis_name);
        Ok(())
    }
    
    pub fn enable_closed_loop(&mut self) {
        if self.encoder_state.calibrated {
            self.enabled = true;
            self.pid_controller.reset();
            println!("Closed-loop control enabled for axis {}", self.axis_name);
        }
    }
    
    pub fn disable_closed_loop(&mut self) {
        self.enabled = false;
        println!("Closed-loop control disabled for axis {}", self.axis_name);
    }
    
    pub fn get_diagnostics(&self) -> AxisDiagnostics {
        AxisDiagnostics {
            axis_name: self.axis_name.clone(),
            position_error: self.encoder_state.position_error,
            max_position_error: self.encoder_state.max_position_error,
            current_position: self.current_position,
            target_position: self.target_position,
            velocity: self.encoder_state.velocity,
            error_count: self.encoder_state.error_count,
            communication_errors: self.encoder_state.communication_errors,
            enabled: self.enabled,
            calibrated: self.encoder_state.calibrated,
            connected: self.encoder_state.connected,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AxisDiagnostics {
    pub axis_name: String,
    pub position_error: f64,
    pub max_position_error: f64,
    pub current_position: f64,
    pub target_position: f64,
    pub velocity: f64,
    pub error_count: u32,
    pub communication_errors: u32,
    pub enabled: bool,
    pub calibrated: bool,
    pub connected: bool,
}

// Position filter for smoothing encoder readings
#[derive(Debug)]
pub struct PositionFilter {
    samples: Vec<f64>,
    sample_index: usize,
    sample_count: usize,
    max_samples: usize,
}

impl PositionFilter {
    pub fn new(max_samples: usize) -> Self {
        Self {
            samples: vec![0.0; max_samples],
            sample_index: 0,
            sample_count: 0,
            max_samples,
        }
    }
    
    pub fn update(&mut self, new_sample: f64) -> f64 {
        self.samples[self.sample_index] = new_sample;
        self.sample_index = (self.sample_index + 1) % self.max_samples;
        
        if self.sample_count < self.max_samples {
            self.sample_count += 1;
        }
        
        // Calculate moving average
        let sum: f64 = self.samples.iter().take(self.sample_count).sum();
        sum / self.sample_count as f64
    }
    
    pub fn reset(&mut self) {
        self.samples.fill(0.0);
        self.sample_index = 0;
        self.sample_count = 0;
    }
}

// Main closed-loop control manager
pub struct ClosedLoopManager {
    axes: HashMap<String, ClosedLoopAxis>,
    update_rate_hz: u32,
    last_update: Instant,
    global_enable: bool,
    error_recovery_enabled: bool,
}

impl ClosedLoopManager {
    pub fn new(update_rate_hz: u32) -> Self {
        Self {
            axes: HashMap::new(),
            update_rate_hz,
            last_update: Instant::now(),
            global_enable: false,
            error_recovery_enabled: true,
        }
    }
    
    pub fn add_axis(&mut self, axis_name: String, encoder_config: EncoderConfig) -> Result<(), String> {
        if self.axes.contains_key(&axis_name) {
            return Err(format!("Axis {} already exists", axis_name));
        }
        
        let axis = ClosedLoopAxis::new(axis_name.clone(), encoder_config);
        self.axes.insert(axis_name.clone(), axis);
        println!("Added closed-loop axis: {}", axis_name);
        Ok(())
    }
    
    pub fn remove_axis(&mut self, axis_name: &str) -> Result<(), String> {
        if self.axes.remove(axis_name).is_some() {
            println!("Removed closed-loop axis: {}", axis_name);
            Ok(())
        } else {
            Err(format!("Axis {} not found", axis_name))
        }
    }
    
    pub fn update_axis_position(&mut self, axis_name: &str, raw_position: i64) -> Result<(), String> {
        if let Some(axis) = self.axes.get_mut(axis_name) {
            axis.update_encoder_position(raw_position)
        } else {
            Err(format!("Axis {} not found", axis_name))
        }
    }
    
    pub fn set_target_position(&mut self, axis_name: &str, position: f64) -> Result<(), String> {
        if let Some(axis) = self.axes.get_mut(axis_name) {
            axis.set_target_position(position);
            Ok(())
        } else {
            Err(format!("Axis {} not found", axis_name))
        }
    }
    
    pub fn calculate_corrections(&mut self) -> HashMap<String, f64> {
        let mut corrections = HashMap::new();
        
        if !self.global_enable {
            return corrections;
        }
        
        for (axis_name, axis) in self.axes.iter_mut() {
            let correction = axis.calculate_correction();
            corrections.insert(axis_name.clone(), correction);
        }
        
        corrections
    }
    
    pub fn home_axis(&mut self, axis_name: &str) -> Result<(), String> {
        if let Some(axis) = self.axes.get_mut(axis_name) {
            axis.home_axis()
        } else {
            Err(format!("Axis {} not found", axis_name))
        }
    }
    
    pub fn home_all_axes(&mut self) -> Result<(), String> {
        let axis_names: Vec<String> = self.axes.keys().cloned().collect();
        
        for axis_name in axis_names {
            self.home_axis(&axis_name)?;
        }
        
        println!("All axes homed successfully");
        Ok(())
    }
    
    pub fn enable_axis(&mut self, axis_name: &str) -> Result<(), String> {
        if let Some(axis) = self.axes.get_mut(axis_name) {
            axis.enable_closed_loop();
            Ok(())
        } else {
            Err(format!("Axis {} not found", axis_name))
        }
    }
    
    pub fn disable_axis(&mut self, axis_name: &str) -> Result<(), String> {
        if let Some(axis) = self.axes.get_mut(axis_name) {
            axis.disable_closed_loop();
            Ok(())
        } else {
            Err(format!("Axis {} not found", axis_name))
        }
    }
    
    pub fn enable_global(&mut self) {
        self.global_enable = true;
        println!("Global closed-loop control enabled");
    }
    
    pub fn disable_global(&mut self) {
        self.global_enable = false;
        println!("Global closed-loop control disabled");
    }
    
    pub fn configure_pid(&mut self, axis_name: &str, kp: f64, ki: f64, kd: f64) -> Result<(), String> {
        if let Some(axis) = self.axes.get_mut(axis_name) {
            axis.pid_controller.set_gains(kp, ki, kd);
            println!("PID gains updated for axis {}: Kp={}, Ki={}, Kd={}", axis_name, kp, ki, kd);
            Ok(())
        } else {
            Err(format!("Axis {} not found", axis_name))
        }
    }
    
    pub fn get_axis_diagnostics(&self, axis_name: &str) -> Option<AxisDiagnostics> {
        self.axes.get(axis_name).map(|axis| axis.get_diagnostics())
    }
    
    pub fn get_all_diagnostics(&self) -> HashMap<String, AxisDiagnostics> {
        self.axes.iter()
            .map(|(name, axis)| (name.clone(), axis.get_diagnostics()))
            .collect()
    }
    
    pub fn process_update(&mut self) -> Result<HashMap<String, f64>, String> {
        let now = Instant::now();
        let dt = now.duration_since(self.last_update);
        let expected_interval = Duration::from_secs_f64(1.0 / self.update_rate_hz as f64);
        
        if dt >= expected_interval {
            self.last_update = now;
            
            // Calculate corrections for all axes
            let corrections = self.calculate_corrections();
            
            // Check for any axes with communication errors
            if self.error_recovery_enabled {
                self.handle_error_recovery()?;
            }
            
            Ok(corrections)
        } else {
            Ok(HashMap::new()) // No update needed yet
        }
    }
    
    fn handle_error_recovery(&mut self) -> Result<(), String> {
        for (axis_name, axis) in self.axes.iter_mut() {
            // Check for communication timeout
            let time_since_update = Instant::now().duration_since(axis.encoder_state.last_update);
            if time_since_update > Duration::from_millis(100) { // 100ms timeout
                axis.encoder_state.connected = false;
                axis.encoder_state.communication_errors += 1;
                
                if axis.encoder_state.communication_errors > 10 {
                    axis.disable_closed_loop();
                    return Err(format!("Excessive communication errors on axis {}", axis_name));
                }
            }
            
            // Check for position errors
            if axis.encoder_state.position_error.abs() > axis.encoder_config.max_error_threshold {
                axis.encoder_state.error_count += 1;
                
                if axis.encoder_state.error_count > 5 {
                    axis.disable_closed_loop();
                    return Err(format!("Excessive position errors on axis {}", axis_name));
                }
            }
        }
        
        Ok(())
    }
    
    pub fn get_status_summary(&self) -> ClosedLoopStatus {
        let total_axes = self.axes.len();
        let enabled_axes = self.axes.values().filter(|axis| axis.enabled).count();
        let calibrated_axes = self.axes.values().filter(|axis| axis.encoder_state.calibrated).count();
        let connected_axes = self.axes.values().filter(|axis| axis.encoder_state.connected).count();
        
        let max_position_error = self.axes.values()
            .map(|axis| axis.encoder_state.position_error.abs())
            .fold(0.0f64, |max, err| max.max(err));
        
        ClosedLoopStatus {
            global_enabled: self.global_enable,
            total_axes,
            enabled_axes,
            calibrated_axes,
            connected_axes,
            max_position_error,
            update_rate_hz: self.update_rate_hz,
            error_recovery_enabled: self.error_recovery_enabled,
        }
    }
    
    pub fn calibrate_axis(&mut self, axis_name: &str) -> Result<(), String> {
        if let Some(axis) = self.axes.get_mut(axis_name) {
            // Perform calibration - reset encoder state and establish baseline
            axis.encoder_state.position_error = 0.0;
            axis.encoder_state.max_position_error = 0.0;
            axis.encoder_state.error_count = 0;
            axis.encoder_state.calibrated = true;
            axis.pid_controller.reset();
            
            println!("Axis {} calibrated successfully", axis_name);
            Ok(())
        } else {
            Err(format!("Axis {} not found", axis_name))
        }
    }
    
    pub fn reset_axis_errors(&mut self, axis_name: &str) -> Result<(), String> {
        if let Some(axis) = self.axes.get_mut(axis_name) {
            // Reset all error counters and states
            axis.encoder_state.error_count = 0;
            axis.encoder_state.communication_errors = 0;
            axis.encoder_state.max_position_error = 0.0;
            axis.encoder_state.position_error = 0.0;
            axis.pid_controller.reset();
            
            // Re-enable the axis if it was disabled due to errors
            if axis.encoder_state.connected && axis.encoder_state.calibrated {
                axis.enable_closed_loop();
            }
            
            println!("Errors reset for axis {}", axis_name);
            Ok(())
        } else {
            Err(format!("Axis {} not found", axis_name))
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClosedLoopStatus {
    pub global_enabled: bool,
    pub total_axes: usize,
    pub enabled_axes: usize,
    pub calibrated_axes: usize,
    pub connected_axes: usize,
    pub max_position_error: f64,
    pub update_rate_hz: u32,
    pub error_recovery_enabled: bool,
}

// Mock encoder interface for testing
pub trait EncoderInterface {
    fn read_position(&mut self) -> Result<i64, String>;
    fn read_velocity(&mut self) -> Result<f64, String>;
    fn is_connected(&self) -> bool;
    fn reset(&mut self) -> Result<(), String>;
}

pub struct MockEncoder {
    position: i64,
    velocity: f64,
    connected: bool,
    noise_amplitude: f64,
}

impl MockEncoder {
    pub fn new() -> Self {
        Self {
            position: 0,
            velocity: 0.0,
            connected: true,
            noise_amplitude: 1.0,
        }
    }
    
    pub fn simulate_movement(&mut self, target_position: i64, dt: f64) {
        use rand::Rng;
        
        let position_diff = target_position - self.position;
        let max_velocity = 1000.0; // counts per second
        
        self.velocity = if position_diff.abs() as f64 * dt > max_velocity * dt {
            if position_diff > 0 {
                max_velocity
            } else {
                -max_velocity
            }
        } else {
            position_diff as f64 / dt
        };
        
        // Update position with some realistic dynamics
        self.position += (self.velocity * dt) as i64;
        
        // Add some noise to simulate real encoder behavior
        let mut rng = rand::thread_rng();
        let noise: f64 = rng.gen_range(-self.noise_amplitude..self.noise_amplitude);
        self.position += noise as i64;
    }
    
    pub fn set_noise_amplitude(&mut self, amplitude: f64) {
        self.noise_amplitude = amplitude;
    }
}

impl EncoderInterface for MockEncoder {
    fn read_position(&mut self) -> Result<i64, String> {
        if self.connected {
            Ok(self.position)
        } else {
            Err("Encoder not connected".to_string())
        }
    }
    
    fn read_velocity(&mut self) -> Result<f64, String> {
        if self.connected {
            Ok(self.velocity)
        } else {
            Err("Encoder not connected".to_string())
        }
    }
    
    fn is_connected(&self) -> bool {
        self.connected
    }
    
    fn reset(&mut self) -> Result<(), String> {
        self.position = 0;
        self.velocity = 0.0;
        Ok(())
    }
}