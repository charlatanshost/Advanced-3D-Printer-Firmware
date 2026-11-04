// Dynamic Torque and Force Control System
// Advanced motor control with collision detection and adaptive extrusion

use std::collections::{HashMap, VecDeque};
use std::time::{Duration, Instant};
use serde::{Deserialize, Serialize};

// Torque monitoring and control configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TorqueControlConfig {
    pub enabled: bool,
    pub sample_rate: u32,             // Hz - torque sampling rate
    pub collision_threshold: f32,     // N⋅m - torque threshold for collision detection
    pub extrusion_threshold: f32,     // N⋅m - torque threshold for clog detection
    pub max_torque: f32,              // N⋅m - maximum allowable torque
    pub torque_averaging_window: usize, // Number of samples for averaging
    pub response_time: Duration,       // Maximum response time for safety stops
    pub adaptive_gain: f32,            // Gain for adaptive control (0.0-1.0)
}

impl Default for TorqueControlConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            sample_rate: 1000,           // 1kHz torque monitoring
            collision_threshold: 2.0,    // 2 N⋅m collision threshold
            extrusion_threshold: 1.5,    // 1.5 N⋅m clog threshold
            max_torque: 5.0,             // 5 N⋅m maximum torque
            torque_averaging_window: 10,
            response_time: Duration::from_millis(10), // 10ms response time
            adaptive_gain: 0.1,
        }
    }
}

// Motor torque sensor interface
pub trait TorqueSensor: Send + Sync {
    fn read_torque(&self) -> Result<f32, String>;
    fn get_max_torque(&self) -> f32;
    fn calibrate(&mut self) -> Result<(), String>;
    fn get_sensor_id(&self) -> String;
}

// Servo motor with torque feedback
pub struct ServoMotorWithTorque {
    pub motor_id: char,
    pub torque_sensor: Box<dyn TorqueSensor>,
    pub current_torque: f32,          // Current torque reading (N⋅m)
    pub average_torque: f32,          // Moving average torque
    pub peak_torque: f32,             // Peak torque in current session
    pub torque_history: VecDeque<(Instant, f32)>,
    pub baseline_torque: f32,         // Normal operating torque
    pub is_stalled: bool,             // Motor stall detection
    pub collision_detected: bool,     // Collision detection flag
    pub last_update: Instant,
}

impl ServoMotorWithTorque {
    pub fn new(motor_id: char, torque_sensor: Box<dyn TorqueSensor>) -> Self {
        Self {
            motor_id,
            torque_sensor,
            current_torque: 0.0,
            average_torque: 0.0,
            peak_torque: 0.0,
            torque_history: VecDeque::with_capacity(1000),
            baseline_torque: 0.0,
            is_stalled: false,
            collision_detected: false,
            last_update: Instant::now(),
        }
    }
    
    pub fn update_torque(&mut self, config: &TorqueControlConfig) -> Result<TorqueEvent, String> {
        let torque = self.torque_sensor.read_torque()?;
        let now = Instant::now();
        
        self.current_torque = torque;
        self.torque_history.push_back((now, torque));
        self.last_update = now;
        
        // Maintain history window
        let cutoff_time = now - Duration::from_secs(10);
        while let Some((timestamp, _)) = self.torque_history.front() {
            if *timestamp < cutoff_time {
                self.torque_history.pop_front();
            } else {
                break;
            }
        }
        
        // Calculate moving average
        let recent_samples: Vec<f32> = self.torque_history.iter()
            .rev()
            .take(config.torque_averaging_window)
            .map(|(_, torque)| *torque)
            .collect();
        
        if !recent_samples.is_empty() {
            self.average_torque = recent_samples.iter().sum::<f32>() / recent_samples.len() as f32;
        }
        
        // Update peak torque
        if torque.abs() > self.peak_torque {
            self.peak_torque = torque.abs();
        }
        
        // Detect anomalies and generate events
        self.detect_torque_events(config)
    }
    
    fn detect_torque_events(&mut self, config: &TorqueControlConfig) -> Result<TorqueEvent, String> {
        let torque_abs = self.current_torque.abs();
        
        // Check for immediate safety conditions
        if torque_abs > config.max_torque {
            return Ok(TorqueEvent::OverTorque {
                motor_id: self.motor_id,
                torque: self.current_torque,
                max_allowed: config.max_torque,
                severity: TorqueSeverity::Critical,
            });
        }
        
        // Collision detection - sudden torque spike
        if torque_abs > config.collision_threshold {
            if !self.collision_detected {
                self.collision_detected = true;
                return Ok(TorqueEvent::CollisionDetected {
                    motor_id: self.motor_id,
                    torque: self.current_torque,
                    threshold: config.collision_threshold,
                    position: [0.0; 5], // Would be filled with actual position
                });
            }
        } else {
            self.collision_detected = false;
        }
        
        // Stall detection - sustained high torque with low movement
        if self.average_torque > config.collision_threshold * 0.8 {
            if !self.is_stalled {
                self.is_stalled = true;
                return Ok(TorqueEvent::MotorStall {
                    motor_id: self.motor_id,
                    torque: self.average_torque,
                    duration: Duration::from_millis(100), // Estimated stall duration
                });
            }
        } else {
            self.is_stalled = false;
        }
        
        // Normal operation
        Ok(TorqueEvent::Normal {
            motor_id: self.motor_id,
            torque: self.current_torque,
        })
    }
    
    pub fn calibrate_baseline(&mut self) -> Result<(), String> {
        // Calibrate baseline torque during normal operation
        if self.torque_history.len() > 100 {
            let mut baseline_samples: Vec<f32> = self.torque_history.iter()
                .map(|(_, torque)| torque.abs())
                .collect();
            
            baseline_samples.sort_by(|a, b| a.partial_cmp(b).unwrap());
            // Use median as baseline to avoid outliers
            let median_index = baseline_samples.len() / 2;
            self.baseline_torque = baseline_samples[median_index];
        }
        Ok(())
    }
    
    pub fn reset_peak_torque(&mut self) {
        self.peak_torque = 0.0;
    }
    
    pub fn get_torque_statistics(&self) -> TorqueStatistics {
        if self.torque_history.is_empty() {
            return TorqueStatistics::default();
        }
        
        let torques: Vec<f32> = self.torque_history.iter().map(|(_, t)| t.abs()).collect();
        let mean = torques.iter().sum::<f32>() / torques.len() as f32;
        let variance: f32 = torques.iter()
            .map(|&t| (t - mean).powi(2))
            .sum::<f32>() / torques.len() as f32;
        let std_dev = variance.sqrt();
        let min_torque = torques.iter().fold(f32::INFINITY, |acc, &x| acc.min(x));
        let max_torque = torques.iter().fold(0.0f32, |acc, &x| acc.max(x));
        
        TorqueStatistics {
            mean_torque: mean,
            std_dev: std_dev,
            min_torque,
            max_torque,
            baseline_torque: self.baseline_torque,
            sample_count: torques.len(),
        }
    }
}

// Extruder with clog detection
pub struct AdaptiveExtruder {
    pub extruder_id: u8,
    pub motor: ServoMotorWithTorque,
    pub target_flow_rate: f32,       // mm³/s
    pub actual_flow_rate: f32,       // mm³/s measured
    pub pressure: f32,               // Pa - estimated melt pressure
    pub clog_probability: f32,       // 0.0-1.0 - probability of clog
    pub temperature: f32,            // °C - melt temperature
    pub flow_history: VecDeque<(Instant, f32)>,
    pub adaptive_multiplier: f32,    // Dynamic flow multiplier
    pub last_clog_detection: Option<Instant>,
}

impl AdaptiveExtruder {
    pub fn new(extruder_id: u8, motor: ServoMotorWithTorque) -> Self {
        Self {
            extruder_id,
            motor,
            target_flow_rate: 0.0,
            actual_flow_rate: 0.0,
            pressure: 0.0,
            clog_probability: 0.0,
            temperature: 200.0,
            flow_history: VecDeque::with_capacity(1000),
            adaptive_multiplier: 1.0,
            last_clog_detection: None,
        }
    }
    
    pub fn update(&mut self, config: &TorqueControlConfig, target_flow: f32, temperature: f32) 
        -> Result<ExtrusionEvent, String> {
        
        self.target_flow_rate = target_flow;
        self.temperature = temperature;
        
        // Update motor torque
        let torque_event = self.motor.update_torque(config)?;
        
        // Estimate actual flow rate from motor torque and speed
        self.estimate_flow_rate();
        
        // Detect clogs and adjust
        let clog_event = self.detect_clog(config)?;
        
        // Update adaptive multiplier
        self.update_adaptive_multiplier();
        
        // Return the most critical event
        match (torque_event, &clog_event) {
            (TorqueEvent::CollisionDetected { .. }, _) => {
                Ok(ExtrusionEvent::EmergencyStop { 
                    reason: "Collision detected on extruder motor".to_string() 
                })
            },
            (TorqueEvent::OverTorque { .. }, _) => {
                Ok(ExtrusionEvent::EmergencyStop { 
                    reason: "Over-torque on extruder motor".to_string() 
                })
            },
            (_, ExtrusionEvent::ClogDetected { .. }) => Ok(clog_event),
            (_, ExtrusionEvent::FlowAdjusted { .. }) => Ok(clog_event),
            _ => Ok(ExtrusionEvent::Normal { 
                extruder_id: self.extruder_id,
                flow_rate: self.actual_flow_rate,
                multiplier: self.adaptive_multiplier,
            })
        }
    }
    
    fn estimate_flow_rate(&mut self) {
        // Estimate flow rate based on motor torque and theoretical relationship
        let torque_flow_coefficient = 0.5; // mm³/s per N⋅m (material dependent)
        let temperature_factor = 1.0 + (self.temperature - 200.0) * 0.001; // Flow increases with temp
        
        self.actual_flow_rate = self.motor.current_torque * torque_flow_coefficient * temperature_factor;
        
        // Store flow history
        let now = Instant::now();
        self.flow_history.push_back((now, self.actual_flow_rate));
        
        // Maintain history
        let cutoff_time = now - Duration::from_secs(30);
        while let Some((timestamp, _)) = self.flow_history.front() {
            if *timestamp < cutoff_time {
                self.flow_history.pop_front();
            } else {
                break;
            }
        }
    }
    
    fn detect_clog(&mut self, config: &TorqueControlConfig) -> Result<ExtrusionEvent, String> {
        // Check if torque is too high for expected flow rate
        if self.motor.current_torque.abs() > config.extrusion_threshold {
            let flow_efficiency = if self.target_flow_rate > 0.1 {
                self.actual_flow_rate / self.target_flow_rate
            } else {
                1.0
            };
            
            // Clog probability based on torque vs flow efficiency
            self.clog_probability = (1.0 - flow_efficiency).clamp(0.0, 1.0);
            
            if self.clog_probability > 0.8 {
                self.last_clog_detection = Some(Instant::now());
                return Ok(ExtrusionEvent::ClogDetected {
                    extruder_id: self.extruder_id,
                    severity: if self.clog_probability > 0.95 { 
                        ClogSeverity::Complete 
                    } else { 
                        ClogSeverity::Partial 
                    },
                    torque: self.motor.current_torque,
                    flow_reduction: 1.0 - flow_efficiency,
                });
            }
        } else {
            // Reduce clog probability when torque is normal
            self.clog_probability *= 0.9;
        }
        
        Ok(ExtrusionEvent::Normal {
            extruder_id: self.extruder_id,
            flow_rate: self.actual_flow_rate,
            multiplier: self.adaptive_multiplier,
        })
    }
    
    fn update_adaptive_multiplier(&mut self) {
        // Adjust flow multiplier based on detected conditions
        if self.clog_probability > 0.5 {
            // Reduce flow when clog is detected
            self.adaptive_multiplier *= 0.95;
        } else if self.clog_probability < 0.1 && self.motor.current_torque.abs() < 0.5 {
            // Increase flow when everything is working well
            self.adaptive_multiplier *= 1.005;
        }
        
        // Keep multiplier within reasonable bounds
        self.adaptive_multiplier = self.adaptive_multiplier.clamp(0.5, 1.5);
    }
    
    pub fn force_unclog_sequence(&mut self) -> Vec<UnclogCommand> {
        let mut commands = Vec::new();
        
        // Retract filament
        commands.push(UnclogCommand::Retract {
            distance: 5.0,
            speed: 10.0,
        });
        
        // Increase temperature
        commands.push(UnclogCommand::SetTemperature {
            temperature: self.temperature + 10.0,
            wait: true,
        });
        
        // Purge material
        commands.push(UnclogCommand::Purge {
            distance: 10.0,
            speed: 2.0,
            pressure_limit: 2.0,
        });
        
        // Cool back down
        commands.push(UnclogCommand::SetTemperature {
            temperature: self.temperature,
            wait: false,
        });
        
        commands
    }
}

// Dynamic force and torque control system
pub struct DynamicForceController {
    motors: HashMap<char, ServoMotorWithTorque>,
    extruders: HashMap<u8, AdaptiveExtruder>,
    config: TorqueControlConfig,
    safety_enabled: bool,
    last_update: Instant,
    event_history: VecDeque<(Instant, SystemEvent)>,
}

impl DynamicForceController {
    pub fn new(config: TorqueControlConfig) -> Self {
        Self {
            motors: HashMap::new(),
            extruders: HashMap::new(),
            config,
            safety_enabled: true,
            last_update: Instant::now(),
            event_history: VecDeque::with_capacity(1000),
        }
    }
    
    pub fn add_motor(&mut self, motor_id: char, torque_sensor: Box<dyn TorqueSensor>) {
        let motor = ServoMotorWithTorque::new(motor_id, torque_sensor);
        self.motors.insert(motor_id, motor);
    }
    
    pub fn add_extruder(&mut self, extruder_id: u8, motor: ServoMotorWithTorque) {
        let extruder = AdaptiveExtruder::new(extruder_id, motor);
        self.extruders.insert(extruder_id, extruder);
    }
    
    pub async fn update_all(&mut self) -> Result<SystemEvent, String> {
        if !self.config.enabled {
            return Ok(SystemEvent::Disabled);
        }
        
        let now = Instant::now();
        let mut critical_events = Vec::new();
        let mut warnings = Vec::new();
        
        // Update all motors
        for (motor_id, motor) in &mut self.motors {
            match motor.update_torque(&self.config) {
                Ok(TorqueEvent::CollisionDetected { .. }) |
                Ok(TorqueEvent::OverTorque { .. }) => {
                    critical_events.push(format!("Critical torque event on motor {}", motor_id));
                },
                Ok(TorqueEvent::MotorStall { .. }) => {
                    warnings.push(format!("Motor {} stalled", motor_id));
                },
                Ok(_) => {},
                Err(e) => warnings.push(format!("Motor {} error: {}", motor_id, e)),
            }
        }
        
        // Update all extruders
        for (extruder_id, extruder) in &mut self.extruders {
            // Get target flow from motion planner (would be passed as parameter)
            let target_flow = 5.0; // mm³/s - example value
            let temperature = 200.0; // °C - would come from temperature controller
            
            match extruder.update(&self.config, target_flow, temperature) {
                Ok(ExtrusionEvent::EmergencyStop { .. }) => {
                    critical_events.push(format!("Emergency stop on extruder {}", extruder_id));
                },
                Ok(ExtrusionEvent::ClogDetected { .. }) => {
                    warnings.push(format!("Clog detected on extruder {}", extruder_id));
                },
                Ok(_) => {},
                Err(e) => warnings.push(format!("Extruder {} error: {}", extruder_id, e)),
            }
        }
        
        self.last_update = now;
        
        // Generate system event based on collected information
        let event = if !critical_events.is_empty() {
            SystemEvent::EmergencyStop {
                reasons: critical_events,
                timestamp: now,
            }
        } else if !warnings.is_empty() {
            SystemEvent::Warning {
                messages: warnings,
                timestamp: now,
            }
        } else {
            SystemEvent::Normal { timestamp: now }
        };
        
        // Store event in history
        self.event_history.push_back((now, event.clone()));
        if self.event_history.len() > 1000 {
            self.event_history.pop_front();
        }
        
        Ok(event)
    }
    
    pub fn get_motor_status(&self, motor_id: char) -> Option<&ServoMotorWithTorque> {
        self.motors.get(&motor_id)
    }
    
    pub fn get_extruder_status(&self, extruder_id: u8) -> Option<&AdaptiveExtruder> {
        self.extruders.get(&extruder_id)
    }
    
    pub fn emergency_stop(&mut self) {
        self.safety_enabled = false;
        // In real implementation, would immediately stop all motors
        println!("EMERGENCY STOP: All motors stopped due to force/torque event");
    }
    
    pub fn reset_safety(&mut self) {
        self.safety_enabled = true;
        // Clear all error states
        for motor in self.motors.values_mut() {
            motor.collision_detected = false;
            motor.is_stalled = false;
        }
        for extruder in self.extruders.values_mut() {
            extruder.clog_probability = 0.0;
            extruder.last_clog_detection = None;
        }
    }
    
    pub fn tune_thresholds(&mut self, collision_threshold: f32, extrusion_threshold: f32, max_torque: f32) {
        self.config.collision_threshold = collision_threshold;
        self.config.extrusion_threshold = extrusion_threshold;
        self.config.max_torque = max_torque;
    }
}

// Event types and data structures
#[derive(Debug, Clone)]
pub enum TorqueEvent {
    Normal { motor_id: char, torque: f32 },
    CollisionDetected { motor_id: char, torque: f32, threshold: f32, position: [f32; 5] },
    OverTorque { motor_id: char, torque: f32, max_allowed: f32, severity: TorqueSeverity },
    MotorStall { motor_id: char, torque: f32, duration: Duration },
}

#[derive(Debug, Clone)]
pub enum ExtrusionEvent {
    Normal { extruder_id: u8, flow_rate: f32, multiplier: f32 },
    ClogDetected { extruder_id: u8, severity: ClogSeverity, torque: f32, flow_reduction: f32 },
    FlowAdjusted { extruder_id: u8, new_multiplier: f32, reason: String },
    EmergencyStop { reason: String },
}

#[derive(Debug, Clone)]
pub enum SystemEvent {
    Normal { timestamp: Instant },
    Warning { messages: Vec<String>, timestamp: Instant },
    EmergencyStop { reasons: Vec<String>, timestamp: Instant },
    Disabled,
}

#[derive(Debug, Clone)]
pub enum TorqueSeverity {
    Warning,
    Critical,
}

#[derive(Debug, Clone)]
pub enum ClogSeverity {
    Partial,
    Complete,
}

#[derive(Debug, Clone)]
pub enum UnclogCommand {
    Retract { distance: f32, speed: f32 },
    Purge { distance: f32, speed: f32, pressure_limit: f32 },
    SetTemperature { temperature: f32, wait: bool },
}

#[derive(Debug, Default)]
pub struct TorqueStatistics {
    pub mean_torque: f32,
    pub std_dev: f32,
    pub min_torque: f32,
    pub max_torque: f32,
    pub baseline_torque: f32,
    pub sample_count: usize,
}

// Mock torque sensor for testing
pub struct MockTorqueSensor {
    sensor_id: String,
    max_torque: f32,
    current_torque: f32,
}

impl MockTorqueSensor {
    pub fn new(sensor_id: String) -> Self {
        Self {
            sensor_id,
            max_torque: 5.0,
            current_torque: 0.0,
        }
    }
    
    pub fn simulate_torque(&mut self, torque: f32) {
        self.current_torque = torque;
    }
}

impl TorqueSensor for MockTorqueSensor {
    fn read_torque(&self) -> Result<f32, String> {
        Ok(self.current_torque)
    }
    
    fn get_max_torque(&self) -> f32 {
        self.max_torque
    }
    
    fn calibrate(&mut self) -> Result<(), String> {
        self.current_torque = 0.0;
        Ok(())
    }
    
    fn get_sensor_id(&self) -> String {
        self.sensor_id.clone()
    }
}