// Input Shaping Module for Real-Time Servo Control
// Designed for LinuxCNC-style real-time systems with servo motors

use std::collections::VecDeque;
use std::time::{Duration, Instant};
use std::f64::consts::PI;
use tokio::sync::mpsc;
use log::{info, warn, error, debug};

// Fixed-point arithmetic for real-time performance
#[cfg(feature = "fixed")]
use fixed::types::I32F32 as _Fixed;

// Input Shaper Types
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ShaperType {
    None,           // No shaping (pass-through)
    ZV,            // Zero Vibration
    MZV,           // Modified Zero Vibration  
    ZVD,           // Zero Vibration Derivative
    EI,            // Extra Insensitive
    EI2,           // Extra Insensitive 2nd order
    EI3,           // Extra Insensitive 3rd order
    TwoHumpEI,     // 2-Hump Extra Insensitive
    ThreeHumpEI,   // 3-Hump Extra Insensitive
    Smoothing,     // Simple smoothing filter
}

impl ShaperType {
    pub fn name(&self) -> &'static str {
        match self {
            Self::None => "None",
            Self::ZV => "ZV (Zero Vibration)",
            Self::MZV => "MZV (Modified ZV)",
            Self::ZVD => "ZVD (ZV Derivative)",
            Self::EI => "EI (Extra Insensitive)",
            Self::EI2 => "EI2 (Extra Insensitive 2)",
            Self::EI3 => "EI3 (Extra Insensitive 3)",
            Self::TwoHumpEI => "2HUMP_EI",
            Self::ThreeHumpEI => "3HUMP_EI",
            Self::Smoothing => "Smoothing Filter",
        }
    }
}

// Motion Command for servo control
#[derive(Debug, Clone, Copy)]
pub struct MotionCommand {
    pub timestamp: f64,        // Time in seconds
    pub position: f64,         // Target position (mm or degrees)
    pub velocity: f64,         // Target velocity (mm/s or deg/s)
    pub acceleration: f64,     // Target acceleration (mm/s² or deg/s²)
    pub axis: char,            // Axis identifier ('X', 'Y', 'Z', 'A', 'B', 'C', 'E')
}

// Shaper impulse (weight and time offset)
#[derive(Debug, Clone, Copy)]
struct ShaperImpulse {
    weight: f64,    // Impulse weight (0.0 to 1.0)
    time: f64,      // Time offset in seconds
}

// Input Shaper Configuration
#[derive(Debug, Clone)]
pub struct InputShaperConfig {
    pub shaper_type: ShaperType,
    pub frequency: f64,         // Resonance frequency in Hz
    pub damping_ratio: f64,     // Damping ratio (typically 0.05-0.2)
    pub enabled: bool,          // Enable/disable shaper
    pub max_acceleration: f64,  // Maximum allowed acceleration after shaping
    pub smoothing_time: f64,    // Smoothing time constant for Smoothing filter
}

impl Default for InputShaperConfig {
    fn default() -> Self {
        Self {
            shaper_type: ShaperType::MZV,
            frequency: 40.0,        // 40 Hz default
            damping_ratio: 0.1,     // 10% damping
            enabled: true,
            max_acceleration: 5000.0, // 5000 mm/s²
            smoothing_time: 0.04,   // 40ms smoothing
        }
    }
}

// Input Shaper Implementation
pub struct InputShaper {
    config: InputShaperConfig,
    impulses: Vec<ShaperImpulse>,
    command_buffer: VecDeque<MotionCommand>,
    max_buffer_time: f64,       // Maximum buffering time
    last_update_time: f64,
    stats: ShaperStatistics,
}

#[derive(Debug, Clone, Default)]
pub struct ShaperStatistics {
    pub commands_processed: u64,
    pub commands_shaped: u64,
    pub average_processing_time_us: f64,
    pub max_processing_time_us: f64,
    pub buffer_overruns: u32,
    pub last_error: Option<String>,
}

impl InputShaper {
    pub fn new(config: InputShaperConfig) -> Self {
        let mut shaper = Self {
            config: config.clone(),
            impulses: Vec::new(),
            command_buffer: VecDeque::new(),
            max_buffer_time: 1.0, // 1 second max buffer
            last_update_time: 0.0,
            stats: ShaperStatistics::default(),
        };
        shaper.calculate_impulses();
        shaper
    }
    
    pub fn update_config(&mut self, config: InputShaperConfig) {
        self.config = config;
        self.calculate_impulses();
        info!("Input shaper config updated: {:?}", self.config.shaper_type);
    }
    
    pub fn get_config(&self) -> &InputShaperConfig {
        &self.config
    }
    
    pub fn get_stats(&self) -> &ShaperStatistics {
        &self.stats
    }
    
    // Calculate impulse weights and timing for the selected shaper type
    fn calculate_impulses(&mut self) {
        self.impulses.clear();
        
        if !self.config.enabled || self.config.shaper_type == ShaperType::None {
            // Pass-through mode
            self.impulses.push(ShaperImpulse { weight: 1.0, time: 0.0 });
            return;
        }
        
        let freq = self.config.frequency;
        let zeta = self.config.damping_ratio;
        let period = 1.0 / freq;
        
        match self.config.shaper_type {
            ShaperType::None => {
                self.impulses.push(ShaperImpulse { weight: 1.0, time: 0.0 });
            }
            
            ShaperType::ZV => {
                // Zero Vibration: 2 impulses
                let k = (-zeta * PI / (1.0 - zeta * zeta).sqrt()).exp();
                let weight1 = 1.0 / (1.0 + k);
                let weight2 = k / (1.0 + k);
                let delay = period / 2.0;
                
                self.impulses.push(ShaperImpulse { weight: weight1, time: 0.0 });
                self.impulses.push(ShaperImpulse { weight: weight2, time: delay });
            }
            
            ShaperType::MZV => {
                // Modified Zero Vibration: 3 impulses
                let k = (-zeta * PI / (1.0 - zeta * zeta).sqrt()).exp();
                let weight1 = 1.0 / (1.0 + 2.0 * k + k * k);
                let weight2 = 2.0 * k / (1.0 + 2.0 * k + k * k);
                let weight3 = k * k / (1.0 + 2.0 * k + k * k);
                let delay = period / 4.0;
                
                self.impulses.push(ShaperImpulse { weight: weight1, time: 0.0 });
                self.impulses.push(ShaperImpulse { weight: weight2, time: delay });
                self.impulses.push(ShaperImpulse { weight: weight3, time: 2.0 * delay });
            }
            
            ShaperType::ZVD => {
                // Zero Vibration Derivative: 3 impulses
                let k = (-zeta * PI / (1.0 - zeta * zeta).sqrt()).exp();
                let weight1 = 1.0 / (1.0 + 2.0 * k);
                let weight2 = k / (1.0 + 2.0 * k);
                let weight3 = k / (1.0 + 2.0 * k);
                let delay = period / 2.0;
                
                self.impulses.push(ShaperImpulse { weight: weight1, time: 0.0 });
                self.impulses.push(ShaperImpulse { weight: weight2, time: delay });
                self.impulses.push(ShaperImpulse { weight: weight3, time: delay });
            }
            
            ShaperType::EI => {
                // Extra Insensitive: 2-3 impulses depending on damping
                let k = (-zeta * PI / (1.0 - zeta * zeta).sqrt()).exp();
                let a = 0.25 * (1.0 + k).powi(2);
                let weight1 = a / (a + 2.0 * k);
                let weight2 = 2.0 * k / (a + 2.0 * k);
                let weight3 = a / (a + 2.0 * k);
                let delay = period / 2.0;
                
                self.impulses.push(ShaperImpulse { weight: weight1, time: 0.0 });
                self.impulses.push(ShaperImpulse { weight: weight2, time: delay });
                self.impulses.push(ShaperImpulse { weight: weight3, time: period });
            }
            
            ShaperType::TwoHumpEI => {
                // 2-Hump Extra Insensitive: 4 impulses
                let k = (-zeta * PI / (1.0 - zeta * zeta).sqrt()).exp();
                let a = (1.0 + k) / 4.0;
                let delay = period / 4.0;
                
                self.impulses.push(ShaperImpulse { weight: a, time: 0.0 });
                self.impulses.push(ShaperImpulse { weight: (1.0 - a) / 2.0, time: delay });
                self.impulses.push(ShaperImpulse { weight: (1.0 - a) / 2.0, time: 3.0 * delay });
                self.impulses.push(ShaperImpulse { weight: a, time: period });
            }
            
            ShaperType::Smoothing => {
                // Simple exponential smoothing - single impulse with time constant
                self.impulses.push(ShaperImpulse { weight: 1.0, time: 0.0 });
            }
            
            _ => {
                // Default to ZV for unimplemented types
                warn!("Shaper type {:?} not fully implemented, using ZV", self.config.shaper_type);
                self.config.shaper_type = ShaperType::ZV;
                self.calculate_impulses();
                return;
            }
        }
        
        debug!("Calculated {} impulses for {:?} at {:.1} Hz", 
               self.impulses.len(), self.config.shaper_type, freq);
    }
    
    // Process a motion command through the input shaper
    pub fn shape_command(&mut self, command: MotionCommand) -> Vec<MotionCommand> {
        let start_time = Instant::now();
        self.stats.commands_processed += 1;
        
        if !self.config.enabled || self.config.shaper_type == ShaperType::None {
            return vec![command];
        }
        
        let mut shaped_commands = Vec::new();
        
        // Special handling for smoothing filter
        if self.config.shaper_type == ShaperType::Smoothing {
            return self.apply_smoothing_filter(command);
        }
        
        // Apply impulse-based shaping
        for impulse in &self.impulses {
            let mut shaped_cmd = command;
            shaped_cmd.timestamp += impulse.time;
            shaped_cmd.position *= impulse.weight;
            shaped_cmd.velocity *= impulse.weight;
            shaped_cmd.acceleration *= impulse.weight;
            
            // Clamp acceleration to configured maximum
            if shaped_cmd.acceleration.abs() > self.config.max_acceleration {
                shaped_cmd.acceleration = self.config.max_acceleration * shaped_cmd.acceleration.signum();
            }
            
            shaped_commands.push(shaped_cmd);
        }
        
        self.stats.commands_shaped += 1;
        
        // Update processing time statistics
        let processing_time = start_time.elapsed().as_micros() as f64;
        self.stats.average_processing_time_us = 
            (self.stats.average_processing_time_us * 0.95) + (processing_time * 0.05);
        if processing_time > self.stats.max_processing_time_us {
            self.stats.max_processing_time_us = processing_time;
        }
        
        shaped_commands
    }
    
    // Apply exponential smoothing filter
    fn apply_smoothing_filter(&mut self, command: MotionCommand) -> Vec<MotionCommand> {
        // Simple first-order low-pass filter
        let alpha = 1.0 - (-1.0 / (self.config.smoothing_time * self.config.frequency)).exp();
        
        let mut smoothed_cmd = command;
        // Apply smoothing to velocity and acceleration
        smoothed_cmd.velocity *= alpha;
        smoothed_cmd.acceleration *= alpha;
        
        vec![smoothed_cmd]
    }
    
    // Get the total delay introduced by the shaper
    pub fn get_shaper_delay(&self) -> f64 {
        if self.impulses.is_empty() {
            return 0.0;
        }
        
        self.impulses.iter()
            .map(|imp| imp.time)
            .fold(0.0, f64::max)
    }
    
    // Reset statistics
    pub fn reset_stats(&mut self) {
        self.stats = ShaperStatistics::default();
    }
}

// Accelerometer interface for resonance measurement
pub trait Accelerometer {
    fn read_acceleration(&mut self) -> Result<(f64, f64, f64), String>; // (x, y, z) in m/s²
    fn set_sample_rate(&mut self, rate_hz: u32) -> Result<(), String>;
    fn calibrate(&mut self) -> Result<(), String>;
}

// Mock accelerometer for testing
pub struct MockAccelerometer {
    sample_rate: u32,
    noise_level: f64,
}

impl MockAccelerometer {
    pub fn new() -> Self {
        Self {
            sample_rate: 1000, // 1 kHz default
            noise_level: 0.01, // 0.01 m/s² noise
        }
    }
}

impl Accelerometer for MockAccelerometer {
    fn read_acceleration(&mut self) -> Result<(f64, f64, f64), String> {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        
        // Generate mock data with some noise
        let x = rng.gen_range(-self.noise_level..self.noise_level);
        let y = rng.gen_range(-self.noise_level..self.noise_level);
        let z = 9.81 + rng.gen_range(-self.noise_level..self.noise_level); // Gravity + noise
        
        Ok((x, y, z))
    }
    
    fn set_sample_rate(&mut self, rate_hz: u32) -> Result<(), String> {
        if rate_hz < 100 || rate_hz > 10000 {
            return Err("Sample rate must be between 100-10000 Hz".to_string());
        }
        self.sample_rate = rate_hz;
        Ok(())
    }
    
    fn calibrate(&mut self) -> Result<(), String> {
        info!("Mock accelerometer calibrated");
        Ok(())
    }
}

// Resonance Testing System
pub struct ResonanceTester {
    accelerometer: Box<dyn Accelerometer + Send>,
    sample_buffer: VecDeque<(f64, f64, f64, f64)>, // (time, x, y, z)
    test_frequency_range: (f64, f64), // Min and max test frequencies
    frequency_step: f64,
    test_amplitude: f64,
}

impl ResonanceTester {
    pub fn new(accelerometer: Box<dyn Accelerometer + Send>) -> Self {
        Self {
            accelerometer,
            sample_buffer: VecDeque::new(),
            test_frequency_range: (10.0, 200.0), // 10-200 Hz
            frequency_step: 1.0, // 1 Hz steps
            test_amplitude: 10.0, // 10mm test moves
        }
    }
    
    pub async fn run_resonance_test(&mut self, axis: char) -> Result<Vec<(f64, f64)>, String> {
        info!("Starting resonance test for axis {}", axis);
        
        let mut frequency_response = Vec::new();
        let mut current_freq = self.test_frequency_range.0;
        
        while current_freq <= self.test_frequency_range.1 {
            // Generate test motion at this frequency
            let response_amplitude = self.test_frequency(axis, current_freq).await?;
            frequency_response.push((current_freq, response_amplitude));
            
            current_freq += self.frequency_step;
        }
        
        info!("Resonance test complete for axis {}, {} data points", 
              axis, frequency_response.len());
        
        Ok(frequency_response)
    }
    
    async fn test_frequency(&mut self, axis: char, frequency: f64) -> Result<f64, String> {
        // Clear buffer
        self.sample_buffer.clear();
        
        // Set accelerometer sample rate
        let sample_rate = (frequency * 20.0) as u32; // 20x oversampling
        self.accelerometer.set_sample_rate(sample_rate)?;
        
        // Generate test motion (this would interface with your motion system)
        let test_duration = 2.0 / frequency; // 2 cycles
        let mut test_time = 0.0;
        let dt = 1.0 / sample_rate as f64;
        
        while test_time < test_duration {
            // Read accelerometer
            let (ax, ay, az) = self.accelerometer.read_acceleration()?;
            self.sample_buffer.push_back((test_time, ax, ay, az));
            
            test_time += dt;
            tokio::time::sleep(tokio::time::Duration::from_micros((dt * 1_000_000.0) as u64)).await;
        }
        
        // Calculate response amplitude (simplified FFT alternative)
        let response_amplitude = self.calculate_response_amplitude(axis, frequency);
        
        debug!("Frequency {:.1} Hz: amplitude {:.6}", frequency, response_amplitude);
        Ok(response_amplitude)
    }
    
    fn calculate_response_amplitude(&self, axis: char, frequency: f64) -> f64 {
        if self.sample_buffer.is_empty() {
            return 0.0;
        }
        
        // Simple amplitude calculation (in a real system, you'd use FFT)
        let axis_data: Vec<f64> = self.sample_buffer.iter().map(|(_, x, y, z)| {
            match axis {
                'X' | 'x' => *x,
                'Y' | 'y' => *y,
                'Z' | 'z' => *z,
                _ => *x, // Default to X
            }
        }).collect();
        
        // Calculate RMS amplitude
        let mean: f64 = axis_data.iter().sum::<f64>() / axis_data.len() as f64;
        let variance: f64 = axis_data.iter()
            .map(|val| (val - mean).powi(2))
            .sum::<f64>() / axis_data.len() as f64;
        
        variance.sqrt()
    }
    
    pub fn find_resonance_peaks(&self, frequency_response: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut peaks = Vec::new();
        
        if frequency_response.len() < 3 {
            return peaks;
        }
        
        // Simple peak detection
        for i in 1..frequency_response.len()-1 {
            let (freq_prev, amp_prev) = frequency_response[i-1];
            let (freq_curr, amp_curr) = frequency_response[i];
            let (freq_next, amp_next) = frequency_response[i+1];
            
            // Check if this is a local maximum
            if amp_curr > amp_prev && amp_curr > amp_next && amp_curr > 0.001 {
                peaks.push((freq_curr, amp_curr));
            }
        }
        
        // Sort by amplitude (highest first)
        peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        
        info!("Found {} resonance peaks", peaks.len());
        peaks
    }
    
    pub fn start_frequency_sweep(&mut self, _axis: char, start_freq: f64, end_freq: f64, step_size: f64) {
        self.test_frequency_range = (start_freq, end_freq);
        self.frequency_step = step_size;
        info!("Started frequency sweep: {}-{} Hz, step {}", start_freq, end_freq, step_size);
    }
    
    pub fn stop_test(&mut self) {
        info!("Resonance test stopped");
    }
    
    pub fn get_progress(&self) -> f32 {
        // Mock progress for now
        0.5
    }
}

// Input Shaper Manager for multiple axes
pub struct InputShaperManager {
    shapers: std::collections::HashMap<char, InputShaper>,
    global_enabled: bool,
    motion_tx: Option<mpsc::UnboundedSender<MotionCommand>>,
    resonance_tester: Option<ResonanceTester>,
}

impl InputShaperManager {
    pub fn new() -> Self {
        Self {
            shapers: std::collections::HashMap::new(),
            global_enabled: true,
            motion_tx: None,
            resonance_tester: None,
        }
    }
    
    pub fn add_axis(&mut self, axis: char, config: InputShaperConfig) {
        let shaper = InputShaper::new(config);
        self.shapers.insert(axis, shaper);
        info!("Added input shaper for axis {}", axis);
    }
    
    pub fn set_axis_config(&mut self, axis: char, config: InputShaperConfig) {
        if let Some(shaper) = self.shapers.get_mut(&axis) {
            shaper.update_config(config);
        }
    }
    
    pub fn get_axis_config(&self, axis: char) -> Option<&InputShaperConfig> {
        self.shapers.get(&axis).map(|s| s.get_config())
    }
    
    pub fn get_axis_stats(&self, axis: char) -> Option<&ShaperStatistics> {
        self.shapers.get(&axis).map(|s| s.get_stats())
    }
    
    pub fn set_global_enabled(&mut self, enabled: bool) {
        self.global_enabled = enabled;
        info!("Input shaping globally {}", if enabled { "enabled" } else { "disabled" });
    }
    
    pub fn shape_motion_command(&mut self, command: MotionCommand) -> Vec<MotionCommand> {
        if !self.global_enabled {
            return vec![command];
        }
        
        if let Some(shaper) = self.shapers.get_mut(&command.axis) {
            shaper.shape_command(command)
        } else {
            // No shaper for this axis, pass through
            vec![command]
        }
    }
    
    pub fn set_resonance_tester(&mut self, tester: ResonanceTester) {
        self.resonance_tester = Some(tester);
    }
    
    pub fn update_axis_config(&mut self, axis: char, config: InputShaperConfig) {
        self.set_axis_config(axis, config);
    }
    
    pub fn process_command(&mut self, axis: char, position: f32) -> Option<f32> {
        if !self.global_enabled {
            return Some(position);
        }
        
        if let Some(shaper) = self.shapers.get_mut(&axis) {
            // For position-based input shaping, create a motion command and process it
            let command = MotionCommand {
                axis,
                position: position as f64,
                velocity: 0.0,
                acceleration: 0.0,
                timestamp: 0.0, // Use 0.0 for immediate processing
            };
            
            let shaped_commands = shaper.shape_command(command);
            // Return the final position from the shaped commands
            shaped_commands.last().map(|cmd| cmd.position as f32)
        } else {
            // No shaper for this axis, pass through
            Some(position)
        }
    }
    
    pub fn start_resonance_test(&mut self, axis: char, start_freq: f64, end_freq: f64, step_size: f64) -> Result<(), String> {
        if let Some(ref mut tester) = self.resonance_tester {
            tester.start_frequency_sweep(axis, start_freq, end_freq, step_size);
            Ok(())
        } else {
            Err("No resonance tester configured".to_string())
        }
    }
    
    pub fn stop_resonance_test(&mut self) {
        if let Some(ref mut tester) = self.resonance_tester {
            tester.stop_test();
        }
    }
    
    pub fn get_test_progress(&self) -> Option<f32> {
        if let Some(ref tester) = self.resonance_tester {
            Some(tester.get_progress())
        } else {
            None
        }
    }
    
    pub async fn calibrate_axis(&mut self, axis: char) -> Result<InputShaperConfig, String> {
        if let Some(ref mut tester) = self.resonance_tester {
            let frequency_response = tester.run_resonance_test(axis).await?;
            let peaks = tester.find_resonance_peaks(&frequency_response);
            
            if peaks.is_empty() {
                return Err("No resonance peaks found".to_string());
            }
            
            // Use the primary resonance frequency
            let primary_frequency = peaks[0].0;
            
            // Create optimized config
            let mut config = InputShaperConfig::default();
            config.frequency = primary_frequency;
            config.shaper_type = ShaperType::MZV; // MZV is good for most cases
            config.damping_ratio = 0.1; // Conservative damping
            
            // Update the shaper
            self.set_axis_config(axis, config.clone());
            
            info!("Calibrated axis {} with frequency {:.1} Hz", axis, primary_frequency);
            Ok(config)
        } else {
            Err("No resonance tester available".to_string())
        }
    }
    
    pub fn get_all_axes(&self) -> Vec<char> {
        self.shapers.keys().cloned().collect()
    }
}

// Real-time input shaper task for RTIC or Embassy integration
pub struct RealTimeInputShaper {
    manager: InputShaperManager,
    command_rx: mpsc::UnboundedReceiver<MotionCommand>,
    shaped_command_tx: mpsc::UnboundedSender<Vec<MotionCommand>>,
    cycle_time_us: u64,
    max_processing_time_us: u64,
}

impl RealTimeInputShaper {
    pub fn new(
        manager: InputShaperManager,
        command_rx: mpsc::UnboundedReceiver<MotionCommand>,
        shaped_command_tx: mpsc::UnboundedSender<Vec<MotionCommand>>,
        cycle_time_us: u64,
    ) -> Self {
        Self {
            manager,
            command_rx,
            shaped_command_tx,
            cycle_time_us,
            max_processing_time_us: cycle_time_us / 2, // Use at most 50% of cycle time
        }
    }
    
    pub async fn run(&mut self) {
        let mut interval = tokio::time::interval(Duration::from_micros(self.cycle_time_us));
        
        loop {
            tokio::select! {
                _ = interval.tick() => {
                    // Process any pending commands
                    while let Ok(command) = self.command_rx.try_recv() {
                        let start_time = Instant::now();
                        
                        let shaped_commands = self.manager.shape_motion_command(command);
                        
                        let processing_time = start_time.elapsed().as_micros() as u64;
                        if processing_time > self.max_processing_time_us {
                            warn!("Input shaper processing took {}μs (max: {}μs)", 
                                  processing_time, self.max_processing_time_us);
                        }
                        
                        if let Err(_) = self.shaped_command_tx.send(shaped_commands) {
                            error!("Failed to send shaped commands");
                            break;
                        }
                    }
                }
                else => break,
            }
        }
    }
}