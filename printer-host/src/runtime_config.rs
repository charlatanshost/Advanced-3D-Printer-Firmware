// Runtime Configuration System 
// Klipper-style configuration files for easy parameter changes without recompiling

use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};
use std::collections::HashMap;
use crate::motor_assignment::{MotorFunction, MotorType};
use crate::pressure_advance::PressureAdvanceConfig;
use crate::input_shaping::{ShaperType, InputShaperConfig};
use crate::temperature_control::{SensorType, HeaterType};
use printer_kinematics::practical_configs::PracticalMachineType;

/// Main configuration structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrinterConfig {
    /// Basic printer information
    pub printer: PrinterInfo,
    /// Kinematics configuration
    pub kinematics: KinematicsConfig,
    /// Motor assignments and configuration
    pub motors: HashMap<String, MotorConfig>,
    /// Extruder configurations
    pub extruders: HashMap<String, ExtruderConfig>,
    /// Temperature control settings
    pub temperature: TemperatureConfig,
    /// Input shaping settings
    pub input_shaping: InputShapingConfig,
    /// Motion control settings
    pub motion: MotionConfig,
    /// Communication settings
    pub communication: CommunicationConfig,
    /// Safety and limits
    pub safety: SafetyConfig,
    /// Custom macros
    pub macros: HashMap<String, MacroConfig>,
}

impl Default for PrinterConfig {
    fn default() -> Self {
        let mut motors = HashMap::new();
        motors.insert("stepper_x".to_string(), MotorConfig {
            function: MotorFunction::XAxis(0),
            motor_type: MotorType::Stepper,
            enable_pin: "PA1".to_string(),
            step_pin: "PA2".to_string(),
            dir_pin: "PA3".to_string(),
            steps_per_mm: 80.0,
            max_velocity: 300.0,
            max_acceleration: 3000.0,
            microsteps: 16,
            current: 0.8,
            hold_current: 0.5,
            interpolate: true,
            stealthchop_threshold: Some(999999),
        });

        let mut extruders = HashMap::new();
        extruders.insert("extruder".to_string(), ExtruderConfig {
            motor: "extruder_stepper".to_string(),
            heater_pin: "PA0".to_string(),
            sensor_type: SensorType::Thermistor,
            sensor_pin: "PA4".to_string(),
            min_temp: 0.0,
            max_temp: 250.0,
            max_power: 1.0,
            nozzle_diameter: 0.4,
            filament_diameter: 1.75,
            pressure_advance: PressureAdvanceConfig::default(),
            retraction: RetractionConfig::default(),
        });

        Self {
            printer: PrinterInfo::default(),
            kinematics: KinematicsConfig::default(),
            motors,
            extruders,
            temperature: TemperatureConfig::default(),
            input_shaping: InputShapingConfig::default(),
            motion: MotionConfig::default(),
            communication: CommunicationConfig::default(),
            safety: SafetyConfig::default(),
            macros: HashMap::new(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrinterInfo {
    pub name: String,
    pub description: String,
    pub version: String,
    pub build_date: String,
}

impl Default for PrinterInfo {
    fn default() -> Self {
        Self {
            name: "Advanced 3D Printer".to_string(),
            description: "LinuxCNC-style 3D printer with multi-motor support".to_string(),
            version: "1.0.0".to_string(),
            build_date: chrono::Utc::now().format("%Y-%m-%d").to_string(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KinematicsConfig {
    pub kinematics_type: String, // "cartesian", "corexy", "delta", etc.
    pub max_z_velocity: f64,
    pub max_z_accel: f64,
    pub square_corner_velocity: f64,
    /// 5-axis machine configuration (if applicable)
    pub five_axis_machine_type: Option<PracticalMachineType>,
}

impl Default for KinematicsConfig {
    fn default() -> Self {
        Self {
            kinematics_type: "cartesian".to_string(),
            max_z_velocity: 25.0,
            max_z_accel: 300.0,
            square_corner_velocity: 5.0,
            five_axis_machine_type: Some(PracticalMachineType::penta_axis_style()),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorConfig {
    pub function: MotorFunction,
    pub motor_type: MotorType,
    pub enable_pin: String,
    pub step_pin: String,
    pub dir_pin: String,
    pub steps_per_mm: f64,
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub microsteps: u32,
    pub current: f64,
    pub hold_current: f64,
    pub interpolate: bool,
    pub stealthchop_threshold: Option<u32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExtruderConfig {
    pub motor: String, // Reference to motor config
    pub heater_pin: String,
    pub sensor_type: SensorType,
    pub sensor_pin: String,
    pub min_temp: f64,
    pub max_temp: f64,
    pub max_power: f64,
    pub nozzle_diameter: f64,
    pub filament_diameter: f64,
    pub pressure_advance: PressureAdvanceConfig,
    pub retraction: RetractionConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RetractionConfig {
    pub retract_length: f64,
    pub retract_speed: f64,
    pub unretract_extra_length: f64,
    pub unretract_speed: f64,
}

impl Default for RetractionConfig {
    fn default() -> Self {
        Self {
            retract_length: 0.5,
            retract_speed: 25.0,
            unretract_extra_length: 0.0,
            unretract_speed: 25.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TemperatureConfig {
    pub pid_ki_max: f64,
    pub pid_kd_max: f64,
    pub max_temp_error: f64,
    pub thermal_protection: bool,
    pub thermal_protection_period: f64,
    pub thermal_protection_hysteresis: f64,
}

impl Default for TemperatureConfig {
    fn default() -> Self {
        Self {
            pid_ki_max: 1.0,
            pid_kd_max: 2.0,
            max_temp_error: 30.0,
            thermal_protection: true,
            thermal_protection_period: 40.0,
            thermal_protection_hysteresis: 4.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InputShapingConfig {
    pub shaper_freq_x: Option<f64>,
    pub shaper_freq_y: Option<f64>,
    pub shaper_type_x: ShaperType,
    pub shaper_type_y: ShaperType,
    pub shaper_damping_ratio_x: f64,
    pub shaper_damping_ratio_y: f64,
}

impl Default for InputShapingConfig {
    fn default() -> Self {
        Self {
            shaper_freq_x: None,
            shaper_freq_y: None,
            shaper_type_x: ShaperType::MZV,
            shaper_type_y: ShaperType::MZV,
            shaper_damping_ratio_x: 0.1,
            shaper_damping_ratio_y: 0.1,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotionConfig {
    pub max_velocity: f64,
    pub max_accel: f64,
    pub minimum_cruise_ratio: f64,
    pub square_corner_velocity: f64,
    pub max_accel_to_decel: f64,
}

impl Default for MotionConfig {
    fn default() -> Self {
        Self {
            max_velocity: 300.0,
            max_accel: 3000.0,
            minimum_cruise_ratio: 0.5,
            square_corner_velocity: 5.0,
            max_accel_to_decel: 1500.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommunicationConfig {
    pub baudrate: u32,
    pub serial_port: String,
    pub restart_method: String,
    pub enable_can: bool,
    pub can_bitrate: Option<u32>,
    pub can_interface: Option<String>,
}

impl Default for CommunicationConfig {
    fn default() -> Self {
        Self {
            baudrate: 250000,
            serial_port: "/dev/ttyUSB0".to_string(),
            restart_method: "command".to_string(),
            enable_can: false,
            can_bitrate: Some(500000),
            can_interface: Some("can0".to_string()),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetyConfig {
    pub min_temp: f64,
    pub max_temp: f64,
    pub max_hotend_temp: f64,
    pub max_bed_temp: f64,
    pub timeout_idle: f64,
    pub timeout_long_idle: f64,
    pub homing_speed: f64,
    pub second_homing_speed: f64,
    pub homing_retract_dist: f64,
}

impl Default for SafetyConfig {
    fn default() -> Self {
        Self {
            min_temp: 0.0,
            max_temp: 250.0,
            max_hotend_temp: 275.0,
            max_bed_temp: 110.0,
            timeout_idle: 600.0,      // 10 minutes
            timeout_long_idle: 1800.0, // 30 minutes
            homing_speed: 50.0,
            second_homing_speed: 10.0,
            homing_retract_dist: 5.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MacroConfig {
    pub description: String,
    pub gcode: Vec<String>,
    pub parameters: HashMap<String, String>,
}

/// Configuration manager for loading, saving, and validating config files
pub struct ConfigManager {
    config_path: PathBuf,
    current_config: PrinterConfig,
    backup_enabled: bool,
}

impl ConfigManager {
    pub fn new<P: AsRef<Path>>(config_path: P) -> Self {
        Self {
            config_path: config_path.as_ref().to_path_buf(),
            current_config: PrinterConfig::default(),
            backup_enabled: true,
        }
    }

    /// Load configuration from file
    pub fn load_config(&mut self) -> Result<(), ConfigError> {
        if !self.config_path.exists() {
            // Create default config if it doesn't exist
            self.save_config()?;
            return Ok(());
        }

        let config_str = fs::read_to_string(&self.config_path)
            .map_err(|e| ConfigError::IoError(e.to_string()))?;

        self.current_config = toml::from_str(&config_str)
            .map_err(|e| ConfigError::ParseError(e.to_string()))?;

        self.validate_config()?;
        Ok(())
    }

    /// Save configuration to file
    pub fn save_config(&self) -> Result<(), ConfigError> {
        // Create backup if enabled
        if self.backup_enabled && self.config_path.exists() {
            let backup_path = self.config_path.with_extension("toml.backup");
            fs::copy(&self.config_path, &backup_path)
                .map_err(|e| ConfigError::IoError(e.to_string()))?;
        }

        let config_str = toml::to_string_pretty(&self.current_config)
            .map_err(|e| ConfigError::SerializeError(e.to_string()))?;

        fs::write(&self.config_path, config_str)
            .map_err(|e| ConfigError::IoError(e.to_string()))?;

        Ok(())
    }

    /// Validate the loaded configuration
    fn validate_config(&self) -> Result<(), ConfigError> {
        // Check for required motors
        if self.current_config.motors.is_empty() {
            return Err(ConfigError::ValidationError("No motors configured".to_string()));
        }

        // Check for required extruders
        if self.current_config.extruders.is_empty() {
            return Err(ConfigError::ValidationError("No extruders configured".to_string()));
        }

        // Validate temperature limits
        for (name, extruder) in &self.current_config.extruders {
            if extruder.max_temp <= extruder.min_temp {
                return Err(ConfigError::ValidationError(
                    format!("Invalid temperature range for extruder {}", name)
                ));
            }
        }

        // Validate motion parameters
        let motion = &self.current_config.motion;
        if motion.max_velocity <= 0.0 || motion.max_accel <= 0.0 {
            return Err(ConfigError::ValidationError(
                "Invalid motion parameters".to_string()
            ));
        }

        Ok(())
    }

    /// Get current configuration
    pub fn get_config(&self) -> &PrinterConfig {
        &self.current_config
    }

    /// Get mutable reference to configuration
    pub fn get_config_mut(&mut self) -> &mut PrinterConfig {
        &mut self.current_config
    }

    /// Update a specific motor configuration
    pub fn update_motor(&mut self, motor_id: &str, config: MotorConfig) -> Result<(), ConfigError> {
        self.current_config.motors.insert(motor_id.to_string(), config);
        Ok(())
    }

    /// Update extruder configuration
    pub fn update_extruder(&mut self, extruder_id: &str, config: ExtruderConfig) -> Result<(), ConfigError> {
        self.current_config.extruders.insert(extruder_id.to_string(), config);
        Ok(())
    }

    /// Add a custom macro
    pub fn add_macro(&mut self, name: &str, macro_config: MacroConfig) -> Result<(), ConfigError> {
        self.current_config.macros.insert(name.to_string(), macro_config);
        Ok(())
    }

    /// Remove a macro
    pub fn remove_macro(&mut self, name: &str) -> Result<(), ConfigError> {
        self.current_config.macros.remove(name);
        Ok(())
    }

    /// Export configuration to different format (for sharing/backup)
    pub fn export_config(&self, format: ExportFormat) -> Result<String, ConfigError> {
        match format {
            ExportFormat::Toml => toml::to_string_pretty(&self.current_config)
                .map_err(|e| ConfigError::SerializeError(e.to_string())),
            ExportFormat::Json => serde_json::to_string_pretty(&self.current_config)
                .map_err(|e| ConfigError::SerializeError(e.to_string())),
            ExportFormat::Yaml => serde_yaml::to_string(&self.current_config)
                .map_err(|e| ConfigError::SerializeError(e.to_string())),
        }
    }

    /// Import configuration from string
    pub fn import_config(&mut self, config_str: &str, format: ExportFormat) -> Result<(), ConfigError> {
        let new_config = match format {
            ExportFormat::Toml => toml::from_str(config_str)
                .map_err(|e| ConfigError::ParseError(e.to_string()))?,
            ExportFormat::Json => serde_json::from_str(config_str)
                .map_err(|e| ConfigError::ParseError(e.to_string()))?,
            ExportFormat::Yaml => serde_yaml::from_str(config_str)
                .map_err(|e| ConfigError::ParseError(e.to_string()))?,
        };

        self.current_config = new_config;
        self.validate_config()?;
        Ok(())
    }

    /// Reset configuration to defaults
    pub fn reset_to_defaults(&mut self) {
        self.current_config = PrinterConfig::default();
    }

    /// Generate sample configuration for different printer types
    pub fn generate_sample_config(printer_type: PrinterType) -> PrinterConfig {
        let mut config = PrinterConfig::default();
        
        match printer_type {
            PrinterType::Cartesian => {
                config.kinematics.kinematics_type = "cartesian".to_string();
                // Keep default motor setup
            }
            PrinterType::CoreXY => {
                config.kinematics.kinematics_type = "corexy".to_string();
                // Update motors for CoreXY
                config.motors.clear();
                config.motors.insert("stepper_a".to_string(), MotorConfig {
                    function: MotorFunction::CoreXY_A(0),
                    motor_type: MotorType::Stepper,
                    enable_pin: "PA1".to_string(),
                    step_pin: "PA2".to_string(),
                    dir_pin: "PA3".to_string(),
                    steps_per_mm: 80.0,
                    max_velocity: 300.0,
                    max_acceleration: 3000.0,
                    microsteps: 16,
                    current: 0.8,
                    hold_current: 0.5,
                    interpolate: true,
                    stealthchop_threshold: Some(999999),
                });
                config.motors.insert("stepper_b".to_string(), MotorConfig {
                    function: MotorFunction::CoreXY_B(0),
                    motor_type: MotorType::Stepper,
                    enable_pin: "PB1".to_string(),
                    step_pin: "PB2".to_string(),
                    dir_pin: "PB3".to_string(),
                    steps_per_mm: 80.0,
                    max_velocity: 300.0,
                    max_acceleration: 3000.0,
                    microsteps: 16,
                    current: 0.8,
                    hold_current: 0.5,
                    interpolate: true,
                    stealthchop_threshold: Some(999999),
                });
            }
            PrinterType::Delta => {
                config.kinematics.kinematics_type = "delta".to_string();
                // Configure delta motors
                config.motors.clear();
                for (i, tower) in ["a", "b", "c"].iter().enumerate() {
                    let function = match i {
                        0 => MotorFunction::DeltaA(0),
                        1 => MotorFunction::DeltaB(0), 
                        2 => MotorFunction::DeltaC(0),
                        _ => unreachable!(),
                    };
                    config.motors.insert(format!("stepper_{}", tower), MotorConfig {
                        function,
                        motor_type: MotorType::Stepper,
                        enable_pin: format!("P{}{}", ('A' as u8 + i as u8) as char, 1),
                        step_pin: format!("P{}{}", ('A' as u8 + i as u8) as char, 2),
                        dir_pin: format!("P{}{}", ('A' as u8 + i as u8) as char, 3),
                        steps_per_mm: 80.0,
                        max_velocity: 300.0,
                        max_acceleration: 3000.0,
                        microsteps: 16,
                        current: 0.8,
                        hold_current: 0.5,
                        interpolate: true,
                        stealthchop_threshold: Some(999999),
                    });
                }
            }
        }
        
        config
    }
}

#[derive(Debug, Clone)]
pub enum ConfigError {
    IoError(String),
    ParseError(String),
    SerializeError(String),
    ValidationError(String),
}

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigError::IoError(msg) => write!(f, "IO Error: {}", msg),
            ConfigError::ParseError(msg) => write!(f, "Parse Error: {}", msg),
            ConfigError::SerializeError(msg) => write!(f, "Serialize Error: {}", msg),
            ConfigError::ValidationError(msg) => write!(f, "Validation Error: {}", msg),
        }
    }
}

impl std::error::Error for ConfigError {}

#[derive(Debug, Clone)]
pub enum ExportFormat {
    Toml,
    Json,
    Yaml,
}

#[derive(Debug, Clone)]
pub enum PrinterType {
    Cartesian,
    CoreXY,
    Delta,
}