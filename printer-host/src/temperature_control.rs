// Temperature Feedforward Control System
// This module provides predictive temperature control to minimize thermal lag and improve print quality

use std::collections::{HashMap, VecDeque};
use std::time::{Duration, Instant};
use serde::{Deserialize, Serialize};

// Temperature sensor types supported
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SensorType {
    Thermistor,           // NTC thermistor (most common)
    Thermocouple,         // K-type, J-type, etc.
    PT100,                // RTD sensor
    PT1000,               // RTD sensor  
    MAX31855,             // Thermocouple amplifier IC
    MAX31856,             // Universal thermocouple amplifier
    MAX31865,             // RTD amplifier IC
    ADS1118,              // 16-bit ADC for thermocouples
    LM335,                // Linear temperature sensor
    DS18B20,              // Digital temperature sensor
    BME280,               // Environment sensor (temp/humidity/pressure)
}

// Heater types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HeaterType {
    Cartridge,            // Cartridge heater
    Ceramic,              // Ceramic heater
    Silicone,             // Silicone heating pad
    Kapton,               // Kapton heating element
    AC_Mains,             // AC mains powered heater (bed)
    Induction,            // Induction heating
}

// Temperature zones in the printer
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq, Hash)]
pub enum TemperatureZone {
    Hotend(u8),           // Hotend index (0, 1, 2, etc.)
    Bed,                  // Heated bed
    Chamber,              // Heated chamber
    Ambient,              // Ambient temperature monitoring
    Stepper(String),      // Stepper motor temperature
    MCU,                  // Microcontroller temperature
    PowerSupply,          // Power supply temperature
    Custom(String),       // Custom zone name
}

impl std::fmt::Display for TemperatureZone {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TemperatureZone::Hotend(n) => write!(f, "Hotend {}", n),
            TemperatureZone::Bed => write!(f, "Bed"),
            TemperatureZone::Chamber => write!(f, "Chamber"),
            TemperatureZone::Ambient => write!(f, "Ambient"),
            TemperatureZone::Stepper(name) => write!(f, "Stepper {}", name),
            TemperatureZone::MCU => write!(f, "MCU"),
            TemperatureZone::PowerSupply => write!(f, "Power Supply"),
            TemperatureZone::Custom(name) => write!(f, "{}", name),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorConfig {
    pub sensor_type: SensorType,
    pub pin: u8,                  // ADC pin or digital pin
    pub pullup_resistor: f64,     // Pullup resistor value (ohms)
    pub series_resistor: f64,     // Series resistor for thermistor
    pub beta: f64,                // Beta value for thermistor
    pub r25: f64,                 // Resistance at 25°C
    pub t0: f64,                  // Reference temperature (usually 25°C)
    pub max_temp: f64,            // Maximum safe temperature
    pub min_temp: f64,            // Minimum expected temperature
    pub sample_rate_hz: u32,      // Sampling frequency
    pub filter_alpha: f64,        // Low-pass filter coefficient (0.0-1.0)
}

impl Default for SensorConfig {
    fn default() -> Self {
        Self {
            sensor_type: SensorType::Thermistor,
            pin: 0,
            pullup_resistor: 4700.0,      // 4.7kΩ typical
            series_resistor: 4700.0,      // 4.7kΩ typical
            beta: 3950.0,                 // Common thermistor beta
            r25: 100000.0,                // 100kΩ at 25°C
            t0: 25.0,                     // 25°C reference
            max_temp: 300.0,              // 300°C maximum
            min_temp: 0.0,                // 0°C minimum
            sample_rate_hz: 10,           // 10Hz sampling
            filter_alpha: 0.1,            // Light filtering
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HeaterConfig {
    pub heater_type: HeaterType,
    pub pin: u8,                  // PWM output pin
    pub max_power: f64,           // Maximum power in watts
    pub pwm_frequency: u32,       // PWM frequency in Hz
    pub max_pwm_duty: f64,        // Maximum PWM duty cycle (0.0-1.0)
    pub thermal_mass: f64,        // Thermal mass for modeling (J/K)
    pub heat_loss_coefficient: f64, // Heat loss coefficient (W/K)
    pub power_coefficient: f64,    // Power to temperature coefficient
    pub safety_timeout: Duration, // Safety timeout for heater
}

impl Default for HeaterConfig {
    fn default() -> Self {
        Self {
            heater_type: HeaterType::Cartridge,
            pin: 0,
            max_power: 40.0,              // 40W typical hotend
            pwm_frequency: 1000,          // 1kHz PWM
            max_pwm_duty: 1.0,            // 100% duty cycle allowed
            thermal_mass: 50.0,           // Estimated thermal mass
            heat_loss_coefficient: 0.5,   // Estimated heat loss
            power_coefficient: 1.0,       // Power scaling factor
            safety_timeout: Duration::from_secs(30), // 30 second timeout
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TemperatureReading {
    pub temperature: f64,         // Temperature in Celsius
    pub raw_value: u16,           // Raw ADC reading
    #[serde(skip, default = "Instant::now")]
    pub timestamp: Instant,       // When reading was taken
    pub filtered_temperature: f64, // Filtered temperature value
    pub valid: bool,              // Whether reading is valid
}

impl Default for TemperatureReading {
    fn default() -> Self {
        Self {
            temperature: 0.0,
            raw_value: 0,
            timestamp: Instant::now(),
            filtered_temperature: 0.0,
            valid: false,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PIDConfig {
    pub kp: f64,                  // Proportional gain
    pub ki: f64,                  // Integral gain
    pub kd: f64,                  // Derivative gain
    pub integral_limit: f64,      // Anti-windup limit
    pub derivative_filter: f64,   // Derivative filter coefficient
    pub output_limit: f64,        // Maximum output (0.0-1.0)
}

impl Default for PIDConfig {
    fn default() -> Self {
        Self {
            // Output is a 0..1 duty, so these are the usual 0..255 hotend gains / 255.
            // (A kp of 20 here would saturate at 0.05C of error and act as bang-bang.)
            kp: 0.087,                    // Proportional gain (duty per degC)
            ki: 0.0042,                   // Integral gain
            kd: 0.45,                     // Derivative gain
            integral_limit: 100.0,        // Integral windup limit (degC*s)
            derivative_filter: 0.1,       // Derivative smoothing
            output_limit: 1.0,            // 100% output allowed
        }
    }
}

#[derive(Debug)]
pub struct PIDState {
    pub integral: f64,
    pub last_error: f64,
    pub last_derivative: f64,
    pub last_output: f64,
    pub last_update: Instant,
    /// False until the first sample, so the derivative has no start-up kick.
    pub initialized: bool,
}

impl Default for PIDState {
    fn default() -> Self {
        Self {
            integral: 0.0,
            last_error: 0.0,
            last_derivative: 0.0,
            last_output: 0.0,
            last_update: Instant::now(),
            initialized: false,
        }
    }
}

/// Thermal runaway protection settings for one heater.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThermalRunawayConfig {
    pub enabled: bool,
    /// While heating, the temperature must rise by `watch_rise` within `watch_period`.
    pub watch_period: Duration,
    pub watch_rise: f64,
    /// After reaching target, it may not stay more than `hold_hysteresis` below
    /// target for longer than `hold_period`.
    pub hold_period: Duration,
    pub hold_hysteresis: f64,
}

impl ThermalRunawayConfig {
    /// Slow zones (bed, chamber) get a longer window than a hotend.
    pub fn for_zone(zone: &TemperatureZone) -> Self {
        let slow = matches!(zone, TemperatureZone::Bed | TemperatureZone::Chamber);
        Self {
            enabled: true,
            watch_period: Duration::from_secs(if slow { 120 } else { 40 }),
            watch_rise: 2.0,
            hold_period: Duration::from_secs(if slow { 180 } else { 45 }),
            hold_hysteresis: if slow { 5.0 } else { 15.0 },
        }
    }
}

/// Runtime state of the runaway monitor.
#[derive(Debug, Clone)]
struct RunawayMonitor {
    watch_start: Instant,
    watch_start_temp: f64,
    reached_target: bool,
    below_since: Option<Instant>,
}

impl RunawayMonitor {
    fn new(temp: f64) -> Self {
        Self { watch_start: Instant::now(), watch_start_temp: temp, reached_target: false, below_since: None }
    }
}

// Feedforward model for predictive control
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeedforwardModel {
    pub thermal_time_constant: f64,  // Time constant in seconds
    pub dead_time: f64,              // Dead time in seconds
    pub gain: f64,                   // Steady-state gain
    pub power_curve: Vec<(f64, f64)>, // Power vs temperature curve
    pub enabled: bool,               // Whether feedforward is enabled
}

impl Default for FeedforwardModel {
    fn default() -> Self {
        Self {
            thermal_time_constant: 30.0,    // 30 second time constant
            dead_time: 5.0,                 // 5 second dead time
            gain: 1.0,                      // Unity gain
            power_curve: vec![
                (20.0, 0.0),    // Room temperature, no power
                (200.0, 0.5),   // 200°C, 50% power
                (250.0, 0.8),   // 250°C, 80% power
                (300.0, 1.0),   // 300°C, 100% power
            ],
            enabled: true,
        }
    }
}

#[derive(Debug)]
pub struct TemperatureController {
    pub zone: TemperatureZone,
    pub sensor_config: SensorConfig,
    pub heater_config: Option<HeaterConfig>,  // Some zones are read-only
    pub pid_config: PIDConfig,
    pub feedforward_model: FeedforwardModel,
    
    // State variables
    pub current_reading: TemperatureReading,
    pub target_temperature: f64,
    pub pid_state: PIDState,
    pub enabled: bool,
    pub safety_shutdown: bool,
    pub runaway_config: ThermalRunawayConfig,
    runaway: Option<RunawayMonitor>,

    // History for modeling and display
    pub temperature_history: VecDeque<TemperatureReading>,
    pub setpoint_history: VecDeque<(Instant, f64)>,
    pub output_history: VecDeque<(Instant, f64)>,
    
    // Statistics
    pub error_count: u32,
    pub over_temp_events: u32,
    pub under_temp_events: u32,
    pub last_error: Option<String>,
}

impl TemperatureController {
    pub fn new(
        zone: TemperatureZone,
        sensor_config: SensorConfig,
        heater_config: Option<HeaterConfig>,
    ) -> Self {
        let runaway_config = ThermalRunawayConfig::for_zone(&zone);
        Self {
            zone,
            runaway_config,
            runaway: None,
            sensor_config,
            heater_config,
            pid_config: PIDConfig::default(),
            feedforward_model: FeedforwardModel::default(),
            current_reading: TemperatureReading::default(),
            target_temperature: 0.0,
            pid_state: PIDState::default(),
            enabled: false,
            safety_shutdown: false,
            temperature_history: VecDeque::with_capacity(1000), // 100 seconds at 10Hz
            setpoint_history: VecDeque::with_capacity(1000),
            output_history: VecDeque::with_capacity(1000),
            error_count: 0,
            over_temp_events: 0,
            under_temp_events: 0,
            last_error: None,
        }
    }
    
    pub fn update_reading(&mut self, raw_value: u16) -> Result<(), String> {
        let now = Instant::now();
        let temperature = match self.raw_to_temperature(raw_value) {
            Ok(t) => t,
            Err(e) => {
                // Open/shorted sensor. The old temperature must not stay trusted, or the
                // heater would keep running on a stale reading.
                self.error_count += 1;
                self.current_reading.valid = false;
                self.current_reading.raw_value = raw_value;
                self.current_reading.timestamp = now;
                self.trip_safety(format!("Sensor fault: {}", e));
                return Err(e);
            }
        };
        
        // Apply low-pass filter to reduce noise
        let filtered_temp = if self.temperature_history.is_empty() {
            temperature
        } else {
            let last_temp = self.current_reading.filtered_temperature;
            last_temp + self.sensor_config.filter_alpha * (temperature - last_temp)
        };
        
        // Validate temperature reading
        let valid = temperature >= self.sensor_config.min_temp 
                 && temperature <= self.sensor_config.max_temp;
        
        if !valid {
            self.error_count += 1;
            if temperature > self.sensor_config.max_temp {
                self.over_temp_events += 1;
                self.trip_safety(format!("Over temperature: {:.1}°C", temperature));
            }
            // Implausibly cold almost always means a disconnected sensor.
            if temperature < self.sensor_config.min_temp {
                self.under_temp_events += 1;
                self.trip_safety(format!("Under temperature: {:.1}°C", temperature));
            }
        }
        
        self.current_reading = TemperatureReading {
            temperature,
            raw_value,
            timestamp: now,
            filtered_temperature: filtered_temp,
            valid,
        };
        
        // Store history (keep last 1000 readings)
        self.temperature_history.push_back(self.current_reading.clone());
        if self.temperature_history.len() > 1000 {
            self.temperature_history.pop_front();
        }
        
        Ok(())
    }
    
    /// Latch the safety shutdown. Only `clear_safety_shutdown` releases it.
    fn trip_safety(&mut self, reason: String) {
        self.safety_shutdown = true;
        self.enabled = false;
        self.pid_state.last_output = 0.0;
        self.last_error = Some(reason);
    }

    /// Set the target. Non-finite values turn the heater off, and targets are limited
    /// to 15C below the sensor's shutdown limit, so a normal overshoot cannot trip it.
    pub fn set_target_temperature(&mut self, target: f64) {
        let target = if target.is_finite() {
            target.clamp(0.0, (self.sensor_config.max_temp - 15.0).max(0.0))
        } else {
            0.0
        };
        let previous = self.target_temperature;
        self.target_temperature = target;
        let now = Instant::now();
        
        // Store setpoint history
        self.setpoint_history.push_back((now, target));
        if self.setpoint_history.len() > 1000 {
            self.setpoint_history.pop_front();
        }
        
        // Reset PID state when target changes significantly
        if (target - previous).abs() > 10.0 {
            self.reset_pid();
        }
        if target != previous {
            self.runaway = None; // Restart the runaway watch for the new target
        }
    }
    
    pub fn calculate_output(&mut self) -> f64 {
        if !self.enabled || self.safety_shutdown {
            return 0.0;
        }
        
        // Sensor-only zones have nothing to drive.
        let Some(heater) = self.heater_config.as_ref() else {
            return 0.0;
        };
        let safety_timeout = heater.safety_timeout;
        let max_duty = heater.max_pwm_duty;

        let now = Instant::now();

        // Never heat on a bad or stale reading.
        if !self.current_reading.valid {
            self.pid_state.last_output = 0.0;
            return 0.0;
        }
        if now.duration_since(self.current_reading.timestamp) > safety_timeout {
            self.trip_safety("Temperature reading timed out".to_string());
            return 0.0;
        }

        let dt = now.duration_since(self.pid_state.last_update).as_secs_f64();
        if dt <= 0.0 {
            return self.pid_state.last_output;
        }

        let current_temp = self.current_reading.filtered_temperature;
        if self.check_thermal_runaway(now, current_temp) {
            return 0.0;
        }
        let error = self.target_temperature - current_temp;
        let output_limit = self.pid_config.output_limit.min(max_duty).max(0.0);

        // Feedforward calculation
        let feedforward_output = self.calculate_feedforward();

        // PID + feedforward, with limits and anti-windup
        let total_output = self.calculate_pid(error, dt, feedforward_output, output_limit);
        let limited_output = if self.target_temperature <= 0.0 {
            0.0 // A zero target always means off
        } else {
            total_output.clamp(0.0, output_limit)
        };

        self.pid_state.last_output = limited_output;
        self.pid_state.last_update = now;
        
        // Store output history
        self.output_history.push_back((now, limited_output));
        if self.output_history.len() > 1000 {
            self.output_history.pop_front();
        }
        
        limited_output
    }
    
    /// Returns true (and latches the shutdown) if the heater is not behaving.
    fn check_thermal_runaway(&mut self, now: Instant, temp: f64) -> bool {
        let config = self.runaway_config.clone();
        if !config.enabled || self.target_temperature <= 0.0 {
            self.runaway = None;
            return false;
        }
        let target = self.target_temperature;
        let monitor = self.runaway.get_or_insert_with(|| RunawayMonitor::new(temp));
        let mut fault = None;

        if !monitor.reached_target {
            if temp >= target - 1.0 {
                monitor.reached_target = true;
            } else if temp >= monitor.watch_start_temp + config.watch_rise {
                monitor.watch_start = now;
                monitor.watch_start_temp = temp;
            } else if now.duration_since(monitor.watch_start) > config.watch_period {
                fault = Some(format!(
                    "Heating failed: under {:.1}°C rise in {}s (at {:.1}°C)",
                    config.watch_rise, config.watch_period.as_secs(), temp
                ));
            }
        } else if temp < target - config.hold_hysteresis {
            let since = *monitor.below_since.get_or_insert(now);
            if now.duration_since(since) > config.hold_period {
                fault = Some(format!("Thermal runaway: {:.1}°C, target {:.1}°C", temp, target));
            }
        } else {
            monitor.below_since = None;
        }

        if let Some(reason) = fault {
            self.trip_safety(reason);
            return true;
        }
        false
    }

    fn calculate_pid(&mut self, error: f64, dt: f64, feedforward: f64, output_limit: f64) -> f64 {
        // Proportional term
        let p_term = self.pid_config.kp * error;

        // Derivative term with filtering (nothing to compare against on the first sample)
        let derivative = if self.pid_state.initialized {
            (error - self.pid_state.last_error) / dt
        } else {
            0.0
        };
        self.pid_state.initialized = true;
        self.pid_state.last_derivative = self.pid_state.last_derivative +
            self.pid_config.derivative_filter * (derivative - self.pid_state.last_derivative);
        let d_term = self.pid_config.kd * self.pid_state.last_derivative;
        self.pid_state.last_error = error;

        // Integral term. Anti-windup: stop integrating while saturated in the same
        // direction, and never let it hold the output on by itself.
        let ki = self.pid_config.ki;
        let unclamped = p_term + ki * (self.pid_state.integral + error * dt) + d_term + feedforward;
        let saturating = (unclamped > output_limit && error > 0.0) || (unclamped < 0.0 && error < 0.0);
        if !saturating {
            self.pid_state.integral += error * dt;
        }
        let mut limit = self.pid_config.integral_limit.abs();
        if ki.abs() > f64::EPSILON {
            limit = limit.min(output_limit / ki.abs());
        }
        self.pid_state.integral = self.pid_state.integral.clamp(-limit, limit);
        let i_term = ki * self.pid_state.integral;

        p_term + i_term + d_term + feedforward
    }
    
    fn calculate_feedforward(&self) -> f64 {
        if !self.feedforward_model.enabled {
            return 0.0;
        }
        
        // Simple feedforward based on target temperature
        // In a real implementation, this would be more sophisticated
        let target = self.target_temperature;
        
        // Interpolate power curve
        let power = self.interpolate_power_curve(target);
        
        // Apply gain
        power * self.feedforward_model.gain
    }
    
    fn interpolate_power_curve(&self, temperature: f64) -> f64 {
        let curve = &self.feedforward_model.power_curve;
        
        if curve.is_empty() {
            return 0.0;
        }
        
        if temperature <= curve[0].0 {
            return curve[0].1;
        }
        
        if temperature >= curve.last().unwrap().0 {
            return curve.last().unwrap().1;
        }
        
        // Linear interpolation
        for i in 1..curve.len() {
            let (t1, p1) = curve[i - 1];
            let (t2, p2) = curve[i];
            
            if temperature >= t1 && temperature <= t2 {
                let ratio = (temperature - t1) / (t2 - t1);
                return p1 + ratio * (p2 - p1);
            }
        }
        
        0.0
    }
    
    fn raw_to_temperature(&self, raw_value: u16) -> Result<f64, String> {
        // A reading at either rail means a broken circuit on every sensor type.
        if raw_value == 0 {
            return Err("Invalid ADC reading: 0".to_string());
        }
        if raw_value >= 4095 {
            return Err("ADC saturated".to_string());
        }

        match self.sensor_config.sensor_type {
            SensorType::Thermistor => {
                // Steinhart-Hart equation for thermistor
                let adc_max = 4095.0; // Assuming 12-bit ADC
                let voltage_ratio = raw_value as f64 / adc_max;

                // Calculate resistance
                let resistance = self.sensor_config.series_resistor * voltage_ratio / (1.0 - voltage_ratio);
                
                // Steinhart-Hart equation (simplified beta equation)
                let inv_t = (1.0 / (self.sensor_config.t0 + 273.15)) + 
                           (1.0 / self.sensor_config.beta) * (resistance / self.sensor_config.r25).ln();
                
                let temperature_k = 1.0 / inv_t;
                let temperature_c = temperature_k - 273.15;
                
                Ok(temperature_c)
            }
            
            SensorType::Thermocouple => {
                // Simplified thermocouple conversion (K-type)
                let voltage = (raw_value as f64 / 4095.0) * 3.3; // 3.3V reference
                let temperature = voltage * 250.0; // Approximate conversion for K-type
                Ok(temperature)
            }
            
            SensorType::PT100 | SensorType::PT1000 => {
                // RTD sensor conversion
                let resistance_base = if matches!(self.sensor_config.sensor_type, SensorType::PT100) {
                    100.0
                } else {
                    1000.0
                };
                
                // Callendar-Van Dusen equation (simplified)
                let voltage = (raw_value as f64 / 4095.0) * 3.3;
                let current = 0.001; // 1mA excitation current
                let resistance = voltage / current;
                
                // Linear approximation: R = R0(1 + α*T)
                let alpha = 0.003851; // Temperature coefficient for platinum
                let temperature = (resistance - resistance_base) / (resistance_base * alpha);
                
                Ok(temperature)
            }
            
            _ => {
                // Generic linear conversion for other sensor types
                let temperature = (raw_value as f64 / 4095.0) * 300.0; // 0-300°C range
                Ok(temperature)
            }
        }
    }
    
    pub fn reset_pid(&mut self) {
        self.pid_state = PIDState::default();
    }
    
    pub fn enable(&mut self) {
        if !self.safety_shutdown {
            self.enabled = true;
            self.reset_pid();
            self.runaway = None;
        }
    }
    
    pub fn disable(&mut self) {
        self.enabled = false;
    }
    
    pub fn clear_safety_shutdown(&mut self) {
        self.safety_shutdown = false;
        self.error_count = 0;
    }
    
    pub fn get_statistics(&self) -> TemperatureStatistics {
        let recent_temps: Vec<f64> = self.temperature_history
            .iter()
            .rev()
            .take(100) // Last 10 seconds at 10Hz
            .map(|r| r.filtered_temperature)
            .collect();
        
        let avg_temp = if recent_temps.is_empty() {
            0.0
        } else {
            recent_temps.iter().sum::<f64>() / recent_temps.len() as f64
        };
        
        let temp_variance = if recent_temps.len() > 1 {
            let variance_sum: f64 = recent_temps.iter()
                .map(|t| (t - avg_temp).powi(2))
                .sum();
            variance_sum / (recent_temps.len() - 1) as f64
        } else {
            0.0
        };
        
        let temp_stability = temp_variance.sqrt(); // Standard deviation
        
        TemperatureStatistics {
            zone: self.zone.clone(),
            current_temperature: self.current_reading.filtered_temperature,
            target_temperature: self.target_temperature,
            temperature_error: self.target_temperature - self.current_reading.filtered_temperature,
            average_temperature: avg_temp,
            temperature_stability: temp_stability,
            enabled: self.enabled,
            safety_shutdown: self.safety_shutdown,
            error_count: self.error_count,
            over_temp_events: self.over_temp_events,
            under_temp_events: self.under_temp_events,
            last_update: self.current_reading.timestamp,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TemperatureStatistics {
    pub zone: TemperatureZone,
    pub current_temperature: f64,
    pub target_temperature: f64,
    pub temperature_error: f64,
    pub average_temperature: f64,
    pub temperature_stability: f64,    // Standard deviation of recent readings
    pub enabled: bool,
    pub safety_shutdown: bool,
    pub error_count: u32,
    pub over_temp_events: u32,
    pub under_temp_events: u32,
    #[serde(skip, default = "Instant::now")]
    pub last_update: Instant,
}

// Main temperature management system
pub struct TemperatureManager {
    controllers: HashMap<TemperatureZone, TemperatureController>,
    update_rate_hz: u32,
    last_update: Instant,
    global_enable: bool,
    emergency_stop: bool,
}

impl TemperatureManager {
    pub fn new(update_rate_hz: u32) -> Self {
        Self {
            controllers: HashMap::new(),
            update_rate_hz,
            last_update: Instant::now(),
            global_enable: false,
            emergency_stop: false,
        }
    }
    
    pub fn add_controller(
        &mut self,
        zone: TemperatureZone,
        sensor_config: SensorConfig,
        heater_config: Option<HeaterConfig>,
    ) -> Result<(), String> {
        if self.controllers.contains_key(&zone) {
            return Err(format!("Controller for zone {:?} already exists", zone));
        }
        
        let controller = TemperatureController::new(zone.clone(), sensor_config, heater_config);
        self.controllers.insert(zone.clone(), controller);
        println!("Added temperature controller for zone: {}", zone);
        Ok(())
    }
    
    pub fn remove_controller(&mut self, zone: &TemperatureZone) -> Result<(), String> {
        if self.controllers.remove(zone).is_some() {
            println!("Removed temperature controller for zone: {}", zone);
            Ok(())
        } else {
            Err(format!("Controller for zone {:?} not found", zone))
        }
    }
    
    pub fn update_reading(&mut self, zone: &TemperatureZone, raw_value: u16) -> Result<(), String> {
        if let Some(controller) = self.controllers.get_mut(zone) {
            controller.update_reading(raw_value)
        } else {
            Err(format!("Controller for zone {:?} not found", zone))
        }
    }
    
    pub fn set_target_temperature(&mut self, zone: &TemperatureZone, target: f64) -> Result<(), String> {
        if let Some(controller) = self.controllers.get_mut(zone) {
            controller.set_target_temperature(target);
            Ok(())
        } else {
            Err(format!("Controller for zone {:?} not found", zone))
        }
    }
    
    pub fn enable_zone(&mut self, zone: &TemperatureZone) -> Result<(), String> {
        if let Some(controller) = self.controllers.get_mut(zone) {
            controller.enable();
            Ok(())
        } else {
            Err(format!("Controller for zone {:?} not found", zone))
        }
    }
    
    pub fn disable_zone(&mut self, zone: &TemperatureZone) -> Result<(), String> {
        if let Some(controller) = self.controllers.get_mut(zone) {
            controller.disable();
            Ok(())
        } else {
            Err(format!("Controller for zone {:?} not found", zone))
        }
    }
    
    pub fn enable_global(&mut self) {
        self.global_enable = true;
        println!("Global temperature control enabled");
    }
    
    pub fn disable_global(&mut self) {
        self.global_enable = false;
        // Disable all controllers
        for controller in self.controllers.values_mut() {
            controller.disable();
        }
        println!("Global temperature control disabled");
    }
    
    pub fn emergency_stop(&mut self) {
        self.emergency_stop = true;
        self.global_enable = false;
        
        // Disable all controllers and set outputs to zero
        for controller in self.controllers.values_mut() {
            controller.set_target_temperature(0.0);
            controller.disable();
            controller.safety_shutdown = true;
            controller.last_error = Some("Emergency stop".to_string());
        }

        println!("Temperature emergency stop activated!");
    }
    
    pub fn clear_emergency_stop(&mut self) {
        self.emergency_stop = false;
        
        // Clear safety shutdowns
        for controller in self.controllers.values_mut() {
            controller.clear_safety_shutdown();
        }
        
        println!("Temperature emergency stop cleared");
    }
    
    /// Compute heater duties. Returns an empty map only when it is too soon for an
    /// update (keep the last duties). When stopped or disabled, every zone gets an
    /// explicit 0.0 so the caller can always drive the outputs off.
    pub fn process_update(&mut self) -> Result<HashMap<TemperatureZone, f64>, String> {
        let now = Instant::now();
        let dt = now.duration_since(self.last_update);
        let expected_interval = Duration::from_secs_f64(1.0 / self.update_rate_hz.max(1) as f64);

        // A stop must take effect now, not at the next update tick.
        if self.emergency_stop || !self.global_enable || dt >= expected_interval {
            self.last_update = now;

            let mut outputs = HashMap::new();
            let run = self.global_enable && !self.emergency_stop;
            for (zone, controller) in self.controllers.iter_mut() {
                let output = if run { controller.calculate_output() } else { 0.0 };
                outputs.insert(zone.clone(), output);
            }

            Ok(outputs)
        } else {
            Ok(HashMap::new()) // No update needed yet
        }
    }
    
    pub fn get_zone_statistics(&self, zone: &TemperatureZone) -> Option<TemperatureStatistics> {
        self.controllers.get(zone).map(|controller| controller.get_statistics())
    }
    
    pub fn get_all_statistics(&self) -> HashMap<TemperatureZone, TemperatureStatistics> {
        self.controllers.iter()
            .map(|(zone, controller)| (zone.clone(), controller.get_statistics()))
            .collect()
    }
    
    pub fn configure_pid(&mut self, zone: &TemperatureZone, pid_config: PIDConfig) -> Result<(), String> {
        if let Some(controller) = self.controllers.get_mut(zone) {
            controller.pid_config = pid_config;
            controller.reset_pid();
            println!("PID configuration updated for zone: {}", zone);
            Ok(())
        } else {
            Err(format!("Controller for zone {:?} not found", zone))
        }
    }
    
    pub fn configure_feedforward(&mut self, zone: &TemperatureZone, model: FeedforwardModel) -> Result<(), String> {
        if let Some(controller) = self.controllers.get_mut(zone) {
            controller.feedforward_model = model;
            println!("Feedforward model updated for zone: {}", zone);
            Ok(())
        } else {
            Err(format!("Controller for zone {:?} not found", zone))
        }
    }
    
    pub fn get_system_status(&self) -> TemperatureSystemStatus {
        let total_zones = self.controllers.len();
        let enabled_zones = self.controllers.values().filter(|c| c.enabled).count();
        let zones_at_target = self.controllers.values()
            .filter(|c| (c.target_temperature - c.current_reading.filtered_temperature).abs() < 2.0)
            .count();
        let zones_in_error = self.controllers.values()
            .filter(|c| c.safety_shutdown || c.error_count > 0)
            .count();
        
        let max_temp_error = self.controllers.values()
            .map(|c| (c.target_temperature - c.current_reading.filtered_temperature).abs())
            .fold(0.0f64, |max, err| max.max(err));
        
        TemperatureSystemStatus {
            global_enabled: self.global_enable,
            emergency_stop: self.emergency_stop,
            total_zones,
            enabled_zones,
            zones_at_target,
            zones_in_error,
            max_temperature_error: max_temp_error,
            update_rate_hz: self.update_rate_hz,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TemperatureSystemStatus {
    pub global_enabled: bool,
    pub emergency_stop: bool,
    pub total_zones: usize,
    pub enabled_zones: usize,
    pub zones_at_target: usize,
    pub zones_in_error: usize,
    pub max_temperature_error: f64,
    pub update_rate_hz: u32,
}

// Mock temperature sensor for testing
pub trait TemperatureSensorInterface {
    fn read_raw(&mut self) -> Result<u16, String>;
    fn is_connected(&self) -> bool;
}

pub struct MockTemperatureSensor {
    target_temp: f64,
    current_temp: f64,
    noise_amplitude: f64,
    connected: bool,
    heating_rate: f64, // Degrees per second when heating
}

impl MockTemperatureSensor {
    pub fn new(initial_temp: f64) -> Self {
        Self {
            target_temp: initial_temp,
            current_temp: initial_temp,
            noise_amplitude: 0.5,
            connected: true,
            heating_rate: 1.0, // 1°C per second
        }
    }
    
    pub fn simulate(&mut self, heater_output: f64, dt: f64) {
        if !self.connected {
            return;
        }
        
        // Simple thermal simulation
        let power_factor = heater_output; // 0.0 to 1.0
        let heat_input = power_factor * self.heating_rate * dt;
        
        // Heat loss to ambient (20°C)
        let ambient_temp = 20.0;
        let heat_loss = (self.current_temp - ambient_temp) * 0.01 * dt; // Proportional heat loss
        
        // Update temperature
        self.current_temp += heat_input - heat_loss;
        
        // Ensure temperature doesn't go below ambient
        if self.current_temp < ambient_temp {
            self.current_temp = ambient_temp;
        }
    }
    
    pub fn set_target(&mut self, target: f64) {
        self.target_temp = target;
    }
    
    pub fn add_noise(&mut self, amplitude: f64) {
        self.noise_amplitude = amplitude;
    }
}

impl TemperatureSensorInterface for MockTemperatureSensor {
    fn read_raw(&mut self) -> Result<u16, String> {
        if !self.connected {
            return Err("Sensor disconnected".to_string());
        }

        use rand::Rng;
        let noise = if self.noise_amplitude > 0.0 {
            rand::thread_rng().gen_range(-self.noise_amplitude..self.noise_amplitude)
        } else {
            0.0
        };
        let noisy_temp = self.current_temp + noise;

        // Exact inverse of the default thermistor conversion in raw_to_temperature
        let config = SensorConfig::default();
        let inv_t0 = 1.0 / (config.t0 + 273.15);
        let resistance = config.r25 * (config.beta * (1.0 / (noisy_temp + 273.15) - inv_t0)).exp();
        let ratio = resistance / (resistance + config.series_resistor);
        Ok((ratio * 4095.0).round().clamp(0.0, 4095.0) as u16)
    }
    
    fn is_connected(&self) -> bool {
        self.connected
    }
}