use serde::{Deserialize, Serialize};
use heapless::{String, Vec};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use crate::config_manager::{PrinterConfiguration, ConfigError, ConfigValue};

const MAX_CONFIG_HIERARCHY_DEPTH: usize = 4;
const MAX_CONFIG_FILES: usize = 16;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfigHierarchy {
    pub machine_config: Option<String<64>>,
    pub toolhead_config: Option<String<64>>,
    pub nozzle_config: Option<String<64>>,
    pub material_config: Option<String<64>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolheadConfig {
    pub name: String<64>,
    pub extruder: ExtruderSettings,
    pub hotend: HotendSettings,
    pub cooling: CoolingSettings,
    pub probe: Option<ProbeSettings>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExtruderSettings {
    pub steps_per_mm: f32,
    pub max_feedrate: f32,
    pub max_acceleration: f32,
    pub pressure_advance: f32,
    pub retract_length: f32,
    pub retract_speed: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HotendSettings {
    pub thermistor_type: u8,
    pub max_temp: f32,
    pub min_temp: f32,
    pub pid_kp: f32,
    pub pid_ki: f32,
    pub pid_kd: f32,
    pub thermal_protection: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoolingSettings {
    pub part_cooling_pwm_freq: u16,
    pub hotend_cooling_pwm_freq: u16,
    pub max_part_cooling_speed: f32,
    pub min_part_cooling_speed: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProbeSettings {
    pub probe_type: String<32>,
    pub x_offset: f32,
    pub y_offset: f32,
    pub z_offset: f32,
    pub speed: f32,
    pub samples: u8,
    pub sample_retract_dist: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NozzleConfig {
    pub name: String<64>,
    pub diameter: f32,
    pub flow_rate_multiplier: f32,
    pub linear_advance_factor: f32,
    pub max_volumetric_speed: f32,
    pub temperature_offset: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MaterialConfig {
    pub name: String<64>,
    pub density: f32,
    pub hotend_temp: f32,
    pub bed_temp: f32,
    pub chamber_temp: Option<f32>,
    pub fan_speed: f32,
    pub retract_length: f32,
    pub retract_speed: f32,
    pub linear_advance: f32,
    pub pressure_advance: f32,
    pub flow_ratio: f32,
}

pub struct HierarchicalConfigManager {
    base_config: Mutex<CriticalSectionRawMutex, PrinterConfiguration>,
    config_hierarchy: Mutex<CriticalSectionRawMutex, ConfigHierarchy>,
    toolhead_configs: Mutex<CriticalSectionRawMutex, Vec<ToolheadConfig, MAX_CONFIG_FILES>>,
    nozzle_configs: Mutex<CriticalSectionRawMutex, Vec<NozzleConfig, MAX_CONFIG_FILES>>,
    material_configs: Mutex<CriticalSectionRawMutex, Vec<MaterialConfig, MAX_CONFIG_FILES>>,
    current_config: Mutex<CriticalSectionRawMutex, PrinterConfiguration>,
}

impl HierarchicalConfigManager {
    pub fn new() -> Self {
        Self {
            base_config: Mutex::new(PrinterConfiguration::default()),
            config_hierarchy: Mutex::new(ConfigHierarchy {
                machine_config: None,
                toolhead_config: None,
                nozzle_config: None,
                material_config: None,
            }),
            toolhead_configs: Mutex::new(Vec::new()),
            nozzle_configs: Mutex::new(Vec::new()),
            material_configs: Mutex::new(Vec::new()),
            current_config: Mutex::new(PrinterConfiguration::default()),
        }
    }

    pub async fn load_machine_config(&self, config_name: &str) -> Result<(), ConfigError> {
        // Load base machine configuration
        let machine_config = self.load_config_from_storage(config_name).await?;
        
        {
            let mut base = self.base_config.lock().await;
            *base = machine_config;
        }
        
        {
            let mut hierarchy = self.config_hierarchy.lock().await;
            hierarchy.machine_config = String::try_from(config_name)
                .map_err(|_| ConfigError::InvalidProfileName)?
                .into();
        }
        
        self.rebuild_configuration().await
    }

    pub async fn set_toolhead(&self, toolhead_name: &str) -> Result<(), ConfigError> {
        let toolhead_config = {
            let toolheads = self.toolhead_configs.lock().await;
            toolheads.iter()
                .find(|t| t.name.as_str() == toolhead_name)
                .ok_or(ConfigError::ProfileNotFound)?
                .clone()
        };

        {
            let mut hierarchy = self.config_hierarchy.lock().await;
            hierarchy.toolhead_config = String::try_from(toolhead_name)
                .map_err(|_| ConfigError::InvalidProfileName)?
                .into();
        }

        self.rebuild_configuration().await
    }

    pub async fn set_nozzle(&self, nozzle_name: &str) -> Result<(), ConfigError> {
        let nozzle_config = {
            let nozzles = self.nozzle_configs.lock().await;
            nozzles.iter()
                .find(|n| n.name.as_str() == nozzle_name)
                .ok_or(ConfigError::ProfileNotFound)?
                .clone()
        };

        {
            let mut hierarchy = self.config_hierarchy.lock().await;
            hierarchy.nozzle_config = String::try_from(nozzle_name)
                .map_err(|_| ConfigError::InvalidProfileName)?
                .into();
        }

        self.rebuild_configuration().await
    }

    pub async fn set_material(&self, material_name: &str) -> Result<(), ConfigError> {
        let material_config = {
            let materials = self.material_configs.lock().await;
            materials.iter()
                .find(|m| m.name.as_str() == material_name)
                .ok_or(ConfigError::ProfileNotFound)?
                .clone()
        };

        {
            let mut hierarchy = self.config_hierarchy.lock().await;
            hierarchy.material_config = String::try_from(material_name)
                .map_err(|_| ConfigError::InvalidProfileName)?
                .into();
        }

        self.rebuild_configuration().await
    }

    async fn rebuild_configuration(&self) -> Result<(), ConfigError> {
        let base_config = self.base_config.lock().await.clone();
        let hierarchy = self.config_hierarchy.lock().await;
        
        let mut final_config = base_config;
        
        // Apply toolhead overrides
        if let Some(ref toolhead_name) = hierarchy.toolhead_config {
            let toolheads = self.toolhead_configs.lock().await;
            if let Some(toolhead) = toolheads.iter().find(|t| t.name.as_str() == toolhead_name.as_str()) {
                self.apply_toolhead_config(&mut final_config, toolhead).await;
            }
        }
        
        // Apply nozzle overrides
        if let Some(ref nozzle_name) = hierarchy.nozzle_config {
            let nozzles = self.nozzle_configs.lock().await;
            if let Some(nozzle) = nozzles.iter().find(|n| n.name.as_str() == nozzle_name.as_str()) {
                self.apply_nozzle_config(&mut final_config, nozzle).await;
            }
        }
        
        // Apply material overrides
        if let Some(ref material_name) = hierarchy.material_config {
            let materials = self.material_configs.lock().await;
            if let Some(material) = materials.iter().find(|m| m.name.as_str() == material_name.as_str()) {
                self.apply_material_config(&mut final_config, material).await;
            }
        }
        
        {
            let mut current = self.current_config.lock().await;
            *current = final_config;
        }
        
        Ok(())
    }

    async fn apply_toolhead_config(&self, config: &mut PrinterConfiguration, toolhead: &ToolheadConfig) {
        // Apply extruder settings
        config.motion.steps_per_mm[3] = toolhead.extruder.steps_per_mm;
        config.motion.max_feedrate[3] = toolhead.extruder.max_feedrate;
        config.motion.max_acceleration[3] = toolhead.extruder.max_acceleration;
        config.extruder.linear_advance_factor = toolhead.extruder.pressure_advance;
        config.extruder.retract_length = toolhead.extruder.retract_length;
        config.extruder.retract_speed = toolhead.extruder.retract_speed;
        
        // Apply hotend settings
        config.temperature.hotend_max_temp = toolhead.hotend.max_temp;
        config.temperature.hotend_min_temp = toolhead.hotend.min_temp;
        config.temperature.pid_hotend.kp = toolhead.hotend.pid_kp;
        config.temperature.pid_hotend.ki = toolhead.hotend.pid_ki;
        config.temperature.pid_hotend.kd = toolhead.hotend.pid_kd;
        config.temperature.thermal_runaway_protection = toolhead.hotend.thermal_protection;
        config.temperature.temp_sensor_types[0] = toolhead.hotend.thermistor_type;
    }

    async fn apply_nozzle_config(&self, config: &mut PrinterConfiguration, nozzle: &NozzleConfig) {
        config.extruder.nozzle_diameter = nozzle.diameter;
        config.extruder.linear_advance_factor = nozzle.linear_advance_factor;
        
        // Adjust flow rate and temperature based on nozzle
        // These would be applied to the G-code processor or slicer interface
    }

    async fn apply_material_config(&self, config: &mut PrinterConfiguration, material: &MaterialConfig) {
        // Material configs would typically be used by the slicer or G-code processor
        // but we can set some basic temperature defaults
        config.extruder.retract_length = material.retract_length;
        config.extruder.retract_speed = material.retract_speed;
        config.extruder.linear_advance_factor = material.linear_advance;
    }

    pub async fn get_current_config(&self) -> PrinterConfiguration {
        let current = self.current_config.lock().await;
        current.clone()
    }

    pub async fn add_toolhead_config(&self, config: ToolheadConfig) -> Result<(), ConfigError> {
        let mut toolheads = self.toolhead_configs.lock().await;
        
        // Remove existing config with same name
        toolheads.retain(|t| t.name != config.name);
        
        // Add new config
        toolheads.push(config).map_err(|_| ConfigError::TooManyProfiles)?;
        
        Ok(())
    }

    pub async fn add_nozzle_config(&self, config: NozzleConfig) -> Result<(), ConfigError> {
        let mut nozzles = self.nozzle_configs.lock().await;
        
        // Remove existing config with same name
        nozzles.retain(|n| n.name != config.name);
        
        // Add new config
        nozzles.push(config).map_err(|_| ConfigError::TooManyProfiles)?;
        
        Ok(())
    }

    pub async fn add_material_config(&self, config: MaterialConfig) -> Result<(), ConfigError> {
        let mut materials = self.material_configs.lock().await;
        
        // Remove existing config with same name
        materials.retain(|m| m.name != config.name);
        
        // Add new config
        materials.push(config).map_err(|_| ConfigError::TooManyProfiles)?;
        
        Ok(())
    }

    pub async fn list_toolheads(&self) -> Vec<String<64>, MAX_CONFIG_FILES> {
        let toolheads = self.toolhead_configs.lock().await;
        let mut names = Vec::new();
        
        for toolhead in toolheads.iter() {
            if names.push(toolhead.name.clone()).is_err() {
                break;
            }
        }
        
        names
    }

    pub async fn list_nozzles(&self) -> Vec<String<64>, MAX_CONFIG_FILES> {
        let nozzles = self.nozzle_configs.lock().await;
        let mut names = Vec::new();
        
        for nozzle in nozzles.iter() {
            if names.push(nozzle.name.clone()).is_err() {
                break;
            }
        }
        
        names
    }

    pub async fn list_materials(&self) -> Vec<String<64>, MAX_CONFIG_FILES> {
        let materials = self.material_configs.lock().await;
        let mut names = Vec::new();
        
        for material in materials.iter() {
            if names.push(material.name.clone()).is_err() {
                break;
            }
        }
        
        names
    }

    pub async fn get_current_hierarchy(&self) -> ConfigHierarchy {
        let hierarchy = self.config_hierarchy.lock().await;
        hierarchy.clone()
    }

    async fn load_config_from_storage(&self, config_name: &str) -> Result<PrinterConfiguration, ConfigError> {
        // In a real implementation, this would load from file system or EEPROM
        // For now, return a default configuration
        Ok(PrinterConfiguration::default())
    }

    pub async fn save_hierarchy_to_storage(&self) -> Result<(), ConfigError> {
        // Save the current hierarchy configuration to persistent storage
        // This would include all the config files and their relationships
        Ok(())
    }

    pub async fn validate_hierarchy(&self) -> Result<(), ConfigError> {
        let hierarchy = self.config_hierarchy.lock().await;
        
        // Validate that all referenced configs exist
        if let Some(ref toolhead_name) = hierarchy.toolhead_config {
            let toolheads = self.toolhead_configs.lock().await;
            if !toolheads.iter().any(|t| t.name.as_str() == toolhead_name.as_str()) {
                return Err(ConfigError::ProfileNotFound);
            }
        }
        
        if let Some(ref nozzle_name) = hierarchy.nozzle_config {
            let nozzles = self.nozzle_configs.lock().await;
            if !nozzles.iter().any(|n| n.name.as_str() == nozzle_name.as_str()) {
                return Err(ConfigError::ProfileNotFound);
            }
        }
        
        if let Some(ref material_name) = hierarchy.material_config {
            let materials = self.material_configs.lock().await;
            if !materials.iter().any(|m| m.name.as_str() == material_name.as_str()) {
                return Err(ConfigError::ProfileNotFound);
            }
        }
        
        Ok(())
    }
}

impl Default for ToolheadConfig {
    fn default() -> Self {
        Self {
            name: String::try_from("Default E3D V6").unwrap_or_default(),
            extruder: ExtruderSettings {
                steps_per_mm: 93.0,
                max_feedrate: 25.0,
                max_acceleration: 1000.0,
                pressure_advance: 0.0,
                retract_length: 5.0,
                retract_speed: 40.0,
            },
            hotend: HotendSettings {
                thermistor_type: 1,
                max_temp: 275.0,
                min_temp: 5.0,
                pid_kp: 22.2,
                pid_ki: 1.08,
                pid_kd: 114.0,
                thermal_protection: true,
            },
            cooling: CoolingSettings {
                part_cooling_pwm_freq: 100,
                hotend_cooling_pwm_freq: 100,
                max_part_cooling_speed: 255.0,
                min_part_cooling_speed: 0.0,
            },
            probe: None,
        }
    }
}

impl Default for NozzleConfig {
    fn default() -> Self {
        Self {
            name: String::try_from("0.4mm Standard").unwrap_or_default(),
            diameter: 0.4,
            flow_rate_multiplier: 1.0,
            linear_advance_factor: 0.0,
            max_volumetric_speed: 15.0,
            temperature_offset: 0.0,
        }
    }
}

impl Default for MaterialConfig {
    fn default() -> Self {
        Self {
            name: String::try_from("PLA").unwrap_or_default(),
            density: 1.24,
            hotend_temp: 210.0,
            bed_temp: 60.0,
            chamber_temp: None,
            fan_speed: 100.0,
            retract_length: 5.0,
            retract_speed: 40.0,
            linear_advance: 0.0,
            pressure_advance: 0.0,
            flow_ratio: 1.0,
        }
    }
}