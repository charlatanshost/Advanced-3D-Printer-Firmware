use serde::{Deserialize, Serialize};
use heapless::{String, Vec};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

const MAX_CONFIG_PROFILES: usize = 8;
const MAX_STRING_LENGTH: usize = 64;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrinterConfiguration {
    pub profile_name: String<MAX_STRING_LENGTH>,
    pub version: u32,
    pub created_timestamp: u64,
    pub modified_timestamp: u64,

    // Motion settings
    pub motion: MotionConfig,
    
    // Temperature settings
    pub temperature: TemperatureConfig,
    
    // Extruder settings
    pub extruder: ExtruderConfig,
    
    // Bed settings
    pub bed: BedConfig,
    
    // Network settings
    pub network: NetworkConfig,
    
    // Safety limits
    pub safety: SafetyConfig,
    
    // Feature flags
    pub features: FeatureConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotionConfig {
    pub max_feedrate: [f32; 4], // X, Y, Z, E mm/min
    pub max_acceleration: [f32; 4], // X, Y, Z, E mm/s²
    pub max_jerk: [f32; 4], // X, Y, Z, E mm/s
    pub steps_per_mm: [f32; 4], // X, Y, Z, E steps/mm
    pub homing_feedrate: [f32; 3], // X, Y, Z mm/min
    pub homing_direction: [i8; 3], // X, Y, Z (-1 or 1)
    pub build_volume: [[f32; 2]; 3], // [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
    pub z_probe_offset: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TemperatureConfig {
    pub hotend_max_temp: f32,
    pub hotend_min_temp: f32,
    pub bed_max_temp: f32,
    pub bed_min_temp: f32,
    pub thermal_runaway_protection: bool,
    pub pid_hotend: PIDConfig,
    pub pid_bed: PIDConfig,
    pub temp_sensor_types: [u8; 2], // Hotend, Bed
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PIDConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub enabled: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExtruderConfig {
    pub filament_diameter: f32,
    pub nozzle_diameter: f32,
    pub retract_length: f32,
    pub retract_speed: f32,
    pub unretract_speed: f32,
    pub linear_advance_factor: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BedConfig {
    pub bed_size: [f32; 2], // X, Y mm
    pub auto_bed_leveling: bool,
    pub mesh_bed_leveling: bool,
    pub probe_points: Vec<[f32; 2], 16>, // X, Y coordinates
    pub z_fade_height: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NetworkConfig {
    pub wifi_ssid: String<MAX_STRING_LENGTH>,
    pub wifi_password: String<MAX_STRING_LENGTH>,
    pub static_ip: bool,
    pub ip_address: [u8; 4],
    pub subnet_mask: [u8; 4],
    pub gateway: [u8; 4],
    pub dns_server: [u8; 4],
    pub web_port: u16,
    pub websocket_port: u16,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetyConfig {
    pub min_software_endstops: bool,
    pub max_software_endstops: bool,
    pub endstop_inverting: [bool; 3], // X, Y, Z
    pub emergency_stop_enabled: bool,
    pub power_loss_recovery_enabled: bool,
    pub filament_runout_sensor: bool,
    pub thermal_protection_enabled: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeatureConfig {
    pub power_loss_recovery: bool,
    pub exclude_objects: bool,
    pub auto_bed_leveling: bool,
    pub linear_advance: bool,
    pub junction_deviation: bool,
    pub s_curve_acceleration: bool,
    pub resume_print: bool,
}

impl Default for MotionConfig {
    fn default() -> Self {
        Self {
            max_feedrate: [300.0, 300.0, 5.0, 25.0], // Typical Ender-3 values
            max_acceleration: [500.0, 500.0, 100.0, 1000.0],
            max_jerk: [10.0, 10.0, 0.3, 5.0],
            steps_per_mm: [80.0, 80.0, 400.0, 93.0],
            homing_feedrate: [50.0, 50.0, 4.0],
            homing_direction: [-1, -1, -1],
            build_volume: [[0.0, 220.0], [0.0, 220.0], [0.0, 250.0]],
            z_probe_offset: 0.0,
        }
    }
}

impl Default for PIDConfig {
    fn default() -> Self {
        Self {
            kp: 22.2,
            ki: 1.08,
            kd: 114.0,
            enabled: true,
        }
    }
}

impl Default for TemperatureConfig {
    fn default() -> Self {
        Self {
            hotend_max_temp: 275.0,
            hotend_min_temp: 5.0,
            bed_max_temp: 150.0,
            bed_min_temp: 5.0,
            thermal_runaway_protection: true,
            pid_hotend: PIDConfig::default(),
            pid_bed: PIDConfig {
                kp: 54.0,
                ki: 0.41,
                kd: 1766.0,
                enabled: true,
            },
            temp_sensor_types: [1, 1], // Generic thermistor
        }
    }
}

impl Default for ExtruderConfig {
    fn default() -> Self {
        Self {
            filament_diameter: 1.75,
            nozzle_diameter: 0.4,
            retract_length: 5.0,
            retract_speed: 40.0,
            unretract_speed: 40.0,
            linear_advance_factor: 0.0,
        }
    }
}

impl Default for BedConfig {
    fn default() -> Self {
        Self {
            bed_size: [220.0, 220.0],
            auto_bed_leveling: false,
            mesh_bed_leveling: false,
            probe_points: Vec::new(),
            z_fade_height: 10.0,
        }
    }
}

impl Default for NetworkConfig {
    fn default() -> Self {
        Self {
            wifi_ssid: String::new(),
            wifi_password: String::new(),
            static_ip: false,
            ip_address: [192, 168, 1, 100],
            subnet_mask: [255, 255, 255, 0],
            gateway: [192, 168, 1, 1],
            dns_server: [8, 8, 8, 8],
            web_port: 80,
            websocket_port: 8080,
        }
    }
}

impl Default for SafetyConfig {
    fn default() -> Self {
        Self {
            min_software_endstops: true,
            max_software_endstops: true,
            endstop_inverting: [false, false, false],
            emergency_stop_enabled: true,
            power_loss_recovery_enabled: true,
            filament_runout_sensor: false,
            thermal_protection_enabled: true,
        }
    }
}

impl Default for FeatureConfig {
    fn default() -> Self {
        Self {
            power_loss_recovery: true,
            exclude_objects: true,
            auto_bed_leveling: false,
            linear_advance: false,
            junction_deviation: false,
            s_curve_acceleration: false,
            resume_print: true,
        }
    }
}

impl Default for PrinterConfiguration {
    fn default() -> Self {
        Self {
            profile_name: String::try_from("Default").unwrap_or_default(),
            version: 1,
            created_timestamp: 0,
            modified_timestamp: 0,
            motion: MotionConfig::default(),
            temperature: TemperatureConfig::default(),
            extruder: ExtruderConfig::default(),
            bed: BedConfig::default(),
            network: NetworkConfig::default(),
            safety: SafetyConfig::default(),
            features: FeatureConfig::default(),
        }
    }
}

pub struct ConfigManager {
    current_config: Mutex<CriticalSectionRawMutex, PrinterConfiguration>,
    saved_profiles: Mutex<CriticalSectionRawMutex, Vec<PrinterConfiguration, MAX_CONFIG_PROFILES>>,
}

impl ConfigManager {
    pub fn new() -> Self {
        Self {
            current_config: Mutex::new(PrinterConfiguration::default()),
            saved_profiles: Mutex::new(Vec::new()),
        }
    }

    pub async fn get_current_config(&self) -> PrinterConfiguration {
        let config = self.current_config.lock().await;
        config.clone()
    }

    pub async fn update_config(&self, config: PrinterConfiguration) -> Result<(), ConfigError> {
        self.validate_config(&config)?;
        
        let mut current = self.current_config.lock().await;
        *current = config;
        current.modified_timestamp = embassy_time::Instant::now().as_millis();
        
        // Save to persistent storage
        self.save_to_storage(&*current).await?;
        
        Ok(())
    }

    pub async fn save_profile(&self, profile_name: &str) -> Result<(), ConfigError> {
        let current_config = self.get_current_config().await;
        let mut new_profile = current_config.clone();
        
        new_profile.profile_name = String::try_from(profile_name)
            .map_err(|_| ConfigError::InvalidProfileName)?;
        new_profile.created_timestamp = embassy_time::Instant::now().as_millis();
        new_profile.modified_timestamp = new_profile.created_timestamp;
        
        let mut profiles = self.saved_profiles.lock().await;
        
        // Remove existing profile with same name
        profiles.retain(|p| p.profile_name.as_str() != profile_name);
        
        // Add new profile
        profiles.push(new_profile).map_err(|_| ConfigError::TooManyProfiles)?;
        
        Ok(())
    }

    pub async fn load_profile(&self, profile_name: &str) -> Result<(), ConfigError> {
        let profile = {
            let profiles = self.saved_profiles.lock().await;
            
            profiles
                .iter()
                .find(|p| p.profile_name.as_str() == profile_name)
                .ok_or(ConfigError::ProfileNotFound)?
                .clone()
        };
        
        self.update_config(profile).await?;
        
        Ok(())
    }

    pub async fn delete_profile(&self, profile_name: &str) -> Result<(), ConfigError> {
        let mut profiles = self.saved_profiles.lock().await;
        
        let initial_len = profiles.len();
        profiles.retain(|p| p.profile_name.as_str() != profile_name);
        
        if profiles.len() == initial_len {
            return Err(ConfigError::ProfileNotFound);
        }
        
        Ok(())
    }

    pub async fn list_profiles(&self) -> Vec<String<MAX_STRING_LENGTH>, MAX_CONFIG_PROFILES> {
        let profiles = self.saved_profiles.lock().await;
        let mut profile_names = Vec::new();
        
        for profile in profiles.iter() {
            if profile_names.push(profile.profile_name.clone()).is_err() {
                break;
            }
        }
        
        profile_names
    }

    pub async fn reset_to_defaults(&self) -> Result<(), ConfigError> {
        let default_config = PrinterConfiguration::default();
        self.update_config(default_config).await
    }

    pub async fn import_config(&self, json_data: &str) -> Result<(), ConfigError> {
        let config: PrinterConfiguration = postcard::from_bytes(json_data.as_bytes())
            .map_err(|_| ConfigError::InvalidConfigFormat)?;
            
        self.update_config(config).await
    }

    pub async fn export_config(&self) -> Result<(), ConfigError> {
        let config = self.get_current_config().await;
        // In a real implementation, this would serialize the config and save it to storage
        // For now, just validate it
        self.validate_config(&config)?;
        Ok(())
    }

    fn validate_config(&self, config: &PrinterConfiguration) -> Result<(), ConfigError> {
        // Validate motion settings
        if config.motion.max_feedrate.iter().any(|&f| f <= 0.0 || f > 1000.0) {
            return Err(ConfigError::InvalidMotionConfig);
        }
        
        if config.motion.max_acceleration.iter().any(|&a| a <= 0.0 || a > 10000.0) {
            return Err(ConfigError::InvalidMotionConfig);
        }
        
        if config.motion.steps_per_mm.iter().any(|&s| s <= 0.0 || s > 10000.0) {
            return Err(ConfigError::InvalidMotionConfig);
        }

        // Validate temperature settings
        if config.temperature.hotend_max_temp < config.temperature.hotend_min_temp ||
           config.temperature.hotend_max_temp > 500.0 {
            return Err(ConfigError::InvalidTemperatureConfig);
        }

        if config.temperature.bed_max_temp < config.temperature.bed_min_temp ||
           config.temperature.bed_max_temp > 200.0 {
            return Err(ConfigError::InvalidTemperatureConfig);
        }

        // Validate extruder settings
        if config.extruder.filament_diameter <= 0.0 || config.extruder.filament_diameter > 5.0 {
            return Err(ConfigError::InvalidExtruderConfig);
        }

        if config.extruder.nozzle_diameter <= 0.0 || config.extruder.nozzle_diameter > 2.0 {
            return Err(ConfigError::InvalidExtruderConfig);
        }

        // Validate network settings
        if config.network.web_port == 0 || config.network.websocket_port == 0 {
            return Err(ConfigError::InvalidNetworkConfig);
        }

        Ok(())
    }

    async fn save_to_storage(&self, config: &PrinterConfiguration) -> Result<(), ConfigError> {
        // Implementation would save to EEPROM/Flash
        // For now, this is a placeholder
        Ok(())
    }

    pub async fn load_from_storage(&self) -> Result<PrinterConfiguration, ConfigError> {
        // Implementation would load from EEPROM/Flash
        // For now, return default config
        Ok(PrinterConfiguration::default())
    }

    pub async fn get_config_field(&self, field_path: &str) -> Option<ConfigValue> {
        let config = self.current_config.lock().await;
        self.extract_config_value(&*config, field_path)
    }

    pub async fn set_config_field(&self, field_path: &str, value: ConfigValue) -> Result<(), ConfigError> {
        let mut config = self.current_config.lock().await;
        self.update_config_value(&mut *config, field_path, value)?;
        config.modified_timestamp = embassy_time::Instant::now().as_millis();
        Ok(())
    }

    fn extract_config_value(&self, config: &PrinterConfiguration, field_path: &str) -> Option<ConfigValue> {
        match field_path {
            "motion.max_feedrate.x" => Some(ConfigValue::Float(config.motion.max_feedrate[0])),
            "motion.max_feedrate.y" => Some(ConfigValue::Float(config.motion.max_feedrate[1])),
            "motion.max_feedrate.z" => Some(ConfigValue::Float(config.motion.max_feedrate[2])),
            "motion.max_feedrate.e" => Some(ConfigValue::Float(config.motion.max_feedrate[3])),
            "temperature.hotend_max_temp" => Some(ConfigValue::Float(config.temperature.hotend_max_temp)),
            "temperature.bed_max_temp" => Some(ConfigValue::Float(config.temperature.bed_max_temp)),
            "extruder.filament_diameter" => Some(ConfigValue::Float(config.extruder.filament_diameter)),
            "extruder.nozzle_diameter" => Some(ConfigValue::Float(config.extruder.nozzle_diameter)),
            "network.web_port" => Some(ConfigValue::Int(config.network.web_port as i32)),
            "safety.thermal_protection_enabled" => Some(ConfigValue::Bool(config.safety.thermal_protection_enabled)),
            "features.power_loss_recovery" => Some(ConfigValue::Bool(config.features.power_loss_recovery)),
            _ => None,
        }
    }

    fn update_config_value(&self, config: &mut PrinterConfiguration, field_path: &str, value: ConfigValue) -> Result<(), ConfigError> {
        match (field_path, value) {
            ("motion.max_feedrate.x", ConfigValue::Float(v)) => config.motion.max_feedrate[0] = v,
            ("motion.max_feedrate.y", ConfigValue::Float(v)) => config.motion.max_feedrate[1] = v,
            ("motion.max_feedrate.z", ConfigValue::Float(v)) => config.motion.max_feedrate[2] = v,
            ("motion.max_feedrate.e", ConfigValue::Float(v)) => config.motion.max_feedrate[3] = v,
            ("temperature.hotend_max_temp", ConfigValue::Float(v)) => config.temperature.hotend_max_temp = v,
            ("temperature.bed_max_temp", ConfigValue::Float(v)) => config.temperature.bed_max_temp = v,
            ("extruder.filament_diameter", ConfigValue::Float(v)) => config.extruder.filament_diameter = v,
            ("extruder.nozzle_diameter", ConfigValue::Float(v)) => config.extruder.nozzle_diameter = v,
            ("network.web_port", ConfigValue::Int(v)) => config.network.web_port = v as u16,
            ("safety.thermal_protection_enabled", ConfigValue::Bool(v)) => config.safety.thermal_protection_enabled = v,
            ("features.power_loss_recovery", ConfigValue::Bool(v)) => config.features.power_loss_recovery = v,
            _ => return Err(ConfigError::InvalidFieldPath),
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ConfigValue {
    Float(f32),
    Int(i32),
    Bool(bool),
    String(String<MAX_STRING_LENGTH>),
}

#[derive(Debug)]
pub enum ConfigError {
    InvalidConfigFormat,
    InvalidMotionConfig,
    InvalidTemperatureConfig,
    InvalidExtruderConfig,
    InvalidNetworkConfig,
    InvalidProfileName,
    ProfileNotFound,
    TooManyProfiles,
    SerializationError,
    StorageError,
    InvalidFieldPath,
}

pub static CONFIG_MANAGER: Mutex<CriticalSectionRawMutex, Option<ConfigManager>> = Mutex::new(None);

pub async fn init_config_manager() -> &'static Mutex<CriticalSectionRawMutex, Option<ConfigManager>> {
    let mut manager = ConfigManager::new();
    
    // Try to load configuration from storage
    if let Ok(saved_config) = manager.load_from_storage().await {
        let _ = manager.update_config(saved_config).await;
    }
    
    {
        let mut guard = CONFIG_MANAGER.lock().await;
        *guard = Some(manager);
    }
    
    &CONFIG_MANAGER
}