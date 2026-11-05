/// Slicer integration for 5-axis printing and machining
/// This module provides interfaces to work with existing slicers and CAM software
/// Compatible with PrusaSlicer, SuperSlicer, Cura, and CAM packages

use crate::five_axis::*;
use crate::practical_configs::*;
use nalgebra::{Point3, Vector3};
use serde::{Serialize, Deserialize};
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

/// Slicer integration manager
pub struct SlicerIntegration {
    kinematics: PracticalKinematics,
    active_profile: SlicerProfile,
    post_processors: HashMap<String, Box<dyn PostProcessor>>,
}

/// Slicer profile configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SlicerProfile {
    pub name: String,
    pub slicer_type: SlicerType,
    pub machine_config: MachineProfile,
    pub print_settings: PrintSettings,
    pub material_settings: MaterialSettings,
    pub advanced_settings: AdvancedSettings,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SlicerType {
    PrusaSlicer,
    SuperSlicer,
    Cura,
    IdeaMaker,
    Simplify3D,
    Custom(String),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MachineProfile {
    pub bed_size: (f64, f64, f64),           // X, Y, Z limits
    pub nozzle_diameter: f64,
    pub filament_diameter: f64,
    pub max_feedrates: HashMap<String, f64>,  // Axis -> mm/min
    pub accelerations: HashMap<String, f64>,  // Axis -> mm/s²
    pub rotary_axes: Vec<RotaryAxisConfig>,
    pub tool_offsets: HashMap<u32, Vector3<f64>>, // Tool ID -> offset
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RotaryAxisConfig {
    pub name: String,                        // A, B, C
    pub min_angle: f64,                      // degrees
    pub max_angle: f64,                      // degrees
    pub resolution: f64,                     // degrees per step
    pub home_position: f64,                  // degrees
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrintSettings {
    pub layer_height: f64,                   // mm
    pub first_layer_height: f64,             // mm
    pub perimeters: u32,
    pub infill_percentage: f64,              // 0-100%
    pub support_enabled: bool,
    pub non_planar_enabled: bool,
    pub adaptive_layer_height: bool,
    pub surface_following: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MaterialSettings {
    pub material_type: String,               // PLA, ABS, PETG, etc.
    pub nozzle_temperature: f64,             // °C
    pub bed_temperature: f64,                // °C
    pub chamber_temperature: Option<f64>,    // °C
    pub cooling_settings: CoolingSettings,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoolingSettings {
    pub fan_speed: f64,                      // 0-100%
    pub bridge_fan_speed: f64,               // 0-100%
    pub overhang_fan_speed: f64,             // 0-100%
    pub disable_fan_first_layers: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdvancedSettings {
    pub pressure_advance: f64,
    pub linear_advance: f64,
    pub input_shaping: bool,
    pub junction_deviation: f64,             // mm
    pub max_acceleration_change: f64,        // mm/s³
    pub orientation_strategy: String,        // Name of orientation strategy
}

/// G-code post-processor trait for different machine types
pub trait PostProcessor: Send + Sync {
    fn process_gcode(&self, input: &str, config: &SlicerProfile) -> Result<String, ProcessingError>;
    fn get_name(&self) -> &str;
    fn supports_machine_type(&self, machine_type: &PracticalMachineType) -> bool;
}

#[derive(Debug)]
pub enum ProcessingError {
    ParseError(String),
    KinematicsError(KinematicsError),
    UnsupportedOperation(String),
    FileError(std::io::Error),
}

impl From<KinematicsError> for ProcessingError {
    fn from(error: KinematicsError) -> Self {
        ProcessingError::KinematicsError(error)
    }
}

impl From<std::io::Error> for ProcessingError {
    fn from(error: std::io::Error) -> Self {
        ProcessingError::FileError(error)
    }
}

/// RepRapFirmware post-processor for 5-axis machines
pub struct RepRapFirmwarePostProcessor {
    machine_type: PracticalMachineType,
}

/// Marlin post-processor with 5-axis extensions
pub struct MarlinPostProcessor {
    machine_type: PracticalMachineType,
}

/// Klipper post-processor with custom kinematics
pub struct KlipperPostProcessor {
    machine_type: PracticalMachineType,
}

impl SlicerIntegration {
    pub fn new(kinematics: PracticalKinematics, profile: SlicerProfile) -> Self {
        let mut post_processors: HashMap<String, Box<dyn PostProcessor>> = HashMap::new();
        
        // Add default post-processors
        post_processors.insert(
            "RepRapFirmware".to_string(), 
            Box::new(RepRapFirmwarePostProcessor::new(PracticalMachineType::penta_axis_style()))
        );
        post_processors.insert(
            "Marlin".to_string(),
            Box::new(MarlinPostProcessor::new(PracticalMachineType::penta_axis_style()))
        );
        post_processors.insert(
            "Klipper".to_string(),
            Box::new(KlipperPostProcessor::new(PracticalMachineType::penta_axis_style()))
        );
        
        Self {
            kinematics,
            active_profile: profile,
            post_processors,
        }
    }
    
    /// Generate machine configuration for slicer
    pub fn generate_slicer_config(&self, output_path: &Path) -> Result<(), ProcessingError> {
        match self.active_profile.slicer_type {
            SlicerType::PrusaSlicer | SlicerType::SuperSlicer => {
                self.generate_prusa_config(output_path)
            },
            SlicerType::Cura => {
                self.generate_cura_config(output_path)
            },
            SlicerType::IdeaMaker => {
                self.generate_ideamaker_config(output_path)
            },
            _ => {
                self.generate_generic_config(output_path)
            },
        }
    }
    
    fn generate_prusa_config(&self, output_path: &Path) -> Result<(), ProcessingError> {
        let mut file = BufWriter::new(File::create(output_path)?);
        
        // Generate PrusaSlicer printer configuration
        writeln!(file, "# PrusaSlicer 5-axis printer configuration")?;
        writeln!(file, "# Generated automatically for {}", self.active_profile.name)?;
        writeln!(file)?;
        
        // Basic printer settings
        writeln!(file, "[printer_model:{}]", self.active_profile.name)?;
        writeln!(file, "name = {}", self.active_profile.name)?;
        writeln!(file, "technology = FFF")?;
        writeln!(file, "family = 5-axis")?;
        writeln!(file)?;
        
        // Bed and build volume
        let bed_size = &self.active_profile.machine_config.bed_size;
        writeln!(file, "bed_shape = 0x0,{}x0,{}x{},0x{}", 
            bed_size.0, bed_size.0, bed_size.1, bed_size.1)?;
        writeln!(file, "max_print_height = {}", bed_size.2)?;
        
        // Extruder configuration
        writeln!(file, "extruder_count = 1")?;
        writeln!(file, "nozzle_diameter = {}", self.active_profile.machine_config.nozzle_diameter)?;
        writeln!(file, "filament_diameter = {}", self.active_profile.machine_config.filament_diameter)?;
        
        // 5-axis specific settings
        writeln!(file)?;
        writeln!(file, "# 5-axis configuration")?;
        for axis in &self.active_profile.machine_config.rotary_axes {
            writeln!(file, "{}_axis_min = {}", axis.name.to_lowercase(), axis.min_angle)?;
            writeln!(file, "{}_axis_max = {}", axis.name.to_lowercase(), axis.max_angle)?;
            writeln!(file, "{}_axis_resolution = {}", axis.name.to_lowercase(), axis.resolution)?;
        }
        
        // Motion settings
        writeln!(file)?;
        writeln!(file, "# Motion settings")?;
        for (axis, feedrate) in &self.active_profile.machine_config.max_feedrates {
            writeln!(file, "max_feedrate_{} = {}", axis.to_lowercase(), feedrate)?;
        }
        
        for (axis, accel) in &self.active_profile.machine_config.accelerations {
            writeln!(file, "max_acceleration_{} = {}", axis.to_lowercase(), accel)?;
        }
        
        // Custom G-code
        writeln!(file)?;
        writeln!(file, "# Custom G-code for 5-axis operations")?;
        writeln!(file, "start_gcode = G21 ; metric units\\nG90 ; absolute positioning\\nG28 ; home all axes\\nM109 S[first_layer_temperature] ; wait for temperature")?;
        writeln!(file, "end_gcode = G91 ; relative positioning\\nG1 Z10 ; lift Z\\nG28 A0 B0 ; reset rotary axes\\nG90 ; absolute positioning\\nM104 S0 ; turn off hotend\\nM140 S0 ; turn off bed\\nM84 ; disable motors")?;
        
        file.flush()?;
        Ok(())
    }
    
    fn generate_cura_config(&self, output_path: &Path) -> Result<(), ProcessingError> {
        let mut file = BufWriter::new(File::create(output_path)?);
        
        // Generate Cura printer definition
        writeln!(file, "{{")?;
        writeln!(file, "    \"version\": 2,")?;
        writeln!(file, "    \"name\": \"{}\",", self.active_profile.name)?;
        writeln!(file, "    \"inherits\": \"fdmprinter\",")?;
        writeln!(file, "    \"metadata\": {{")?;
        writeln!(file, "        \"visible\": true,")?;
        writeln!(file, "        \"author\": \"5-axis firmware generator\",")?;
        writeln!(file, "        \"manufacturer\": \"Custom\",")?;
        writeln!(file, "        \"file_formats\": \"text/x-gcode\",")?;
        writeln!(file, "        \"platform\": \"custom_5axis_platform.3mf\"")?;
        writeln!(file, "    }},")?;
        
        // Override settings for 5-axis
        writeln!(file, "    \"overrides\": {{")?;
        
        let bed_size = &self.active_profile.machine_config.bed_size;
        writeln!(file, "        \"machine_width\": {{ \"default_value\": {} }},", bed_size.0)?;
        writeln!(file, "        \"machine_depth\": {{ \"default_value\": {} }},", bed_size.1)?;
        writeln!(file, "        \"machine_height\": {{ \"default_value\": {} }},", bed_size.2)?;
        
        writeln!(file, "        \"machine_nozzle_size\": {{ \"default_value\": {} }},", 
            self.active_profile.machine_config.nozzle_diameter)?;
        
        // Add custom 5-axis settings
        writeln!(file, "        \"machine_start_gcode\": {{ \"default_value\": \"G21\\\\nG90\\\\nG28\\\\nM109 S{{material_print_temperature}}\" }},")?;
        writeln!(file, "        \"machine_end_gcode\": {{ \"default_value\": \"G91\\\\nG1 Z10\\\\nG28 A0 B0\\\\nG90\\\\nM104 S0\\\\nM140 S0\\\\nM84\" }}")?;
        
        writeln!(file, "    }}")?;
        writeln!(file, "}}")?;
        
        file.flush()?;
        Ok(())
    }
    
    fn generate_ideamaker_config(&self, output_path: &Path) -> Result<(), ProcessingError> {
        // IdeaMaker uses XML configuration files
        let mut file = BufWriter::new(File::create(output_path)?);
        
        writeln!(file, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>")?;
        writeln!(file, "<printer>")?;
        writeln!(file, "  <name>{}</name>", self.active_profile.name)?;
        writeln!(file, "  <type>5-axis</type>")?;
        
        let bed_size = &self.active_profile.machine_config.bed_size;
        writeln!(file, "  <buildVolume>")?;
        writeln!(file, "    <width>{}</width>", bed_size.0)?;
        writeln!(file, "    <depth>{}</depth>", bed_size.1)?;
        writeln!(file, "    <height>{}</height>", bed_size.2)?;
        writeln!(file, "  </buildVolume>")?;
        
        writeln!(file, "  <extruders>")?;
        writeln!(file, "    <extruder id=\"0\">")?;
        writeln!(file, "      <nozzleDiameter>{}</nozzleDiameter>", 
            self.active_profile.machine_config.nozzle_diameter)?;
        writeln!(file, "      <filamentDiameter>{}</filamentDiameter>", 
            self.active_profile.machine_config.filament_diameter)?;
        writeln!(file, "    </extruder>")?;
        writeln!(file, "  </extruders>")?;
        
        // Rotary axes configuration
        writeln!(file, "  <rotaryAxes>")?;
        for axis in &self.active_profile.machine_config.rotary_axes {
            writeln!(file, "    <axis name=\"{}\" min=\"{}\" max=\"{}\" resolution=\"{}\"/>", 
                axis.name, axis.min_angle, axis.max_angle, axis.resolution)?;
        }
        writeln!(file, "  </rotaryAxes>")?;
        
        writeln!(file, "</printer>")?;
        
        file.flush()?;
        Ok(())
    }
    
    fn generate_generic_config(&self, output_path: &Path) -> Result<(), ProcessingError> {
        // Generate JSON configuration for custom slicers
        let config_json = serde_json::to_string_pretty(&self.active_profile)
            .map_err(|e| ProcessingError::ParseError(e.to_string()))?;
        
        std::fs::write(output_path, config_json)?;
        Ok(())
    }
    
    /// Post-process G-code for 5-axis operation
    pub fn post_process_gcode(&self, input_path: &Path, output_path: &Path, processor_name: &str) -> Result<(), ProcessingError> {
        let processor = self.post_processors.get(processor_name)
            .ok_or_else(|| ProcessingError::ParseError(format!("Unknown post-processor: {}", processor_name)))?;
        
        let input_gcode = std::fs::read_to_string(input_path)?;
        let output_gcode = processor.process_gcode(&input_gcode, &self.active_profile)?;
        
        std::fs::write(output_path, output_gcode)?;
        Ok(())
    }
    
    /// Convert 3-axis G-code to 5-axis with surface following
    pub fn convert_to_surface_following(&self, input_gcode: &str, surface_strategy: &str) -> Result<String, ProcessingError> {
        let mut output_lines = Vec::new();
        let mut current_position = Point3::origin();
        
        for line in input_gcode.lines() {
            let trimmed = line.trim();
            
            // The code part only. A comment must not be read as words, and exactly G0/G1
            // are moves: G10, G17 and so on merely start the same way.
            let code = trimmed.split(';').next().unwrap_or("").trim();
            let is_move = matches!(code.split_whitespace().next(), Some("G0" | "G1" | "G00" | "G01"));

            if is_move {
                // Parse move command
                let mut x = current_position.x;
                let mut y = current_position.y;
                let mut z = current_position.z;
                let mut feedrate: Option<f64> = None;
                let mut extrusion: Option<&str> = None;

                let parts: Vec<&str> = code.split_whitespace().collect();
                for part in parts.iter().skip(1) {
                    if let Some(value_str) = part.strip_prefix('X') {
                        x = value_str.parse().unwrap_or(x);
                    } else if let Some(value_str) = part.strip_prefix('Y') {
                        y = value_str.parse().unwrap_or(y);
                    } else if let Some(value_str) = part.strip_prefix('Z') {
                        z = value_str.parse().unwrap_or(z);
                    } else if let Some(value_str) = part.strip_prefix('F') {
                        feedrate = value_str.parse().ok();
                    } else if let Some(value_str) = part.strip_prefix('E') {
                        extrusion = Some(value_str);
                    }
                }

                // A move with no position (E or F only) stays as it is
                let has_position = parts.iter().skip(1).any(|p| p.starts_with(['X', 'Y', 'Z']));
                if !has_position {
                    output_lines.push(trimmed.to_string());
                    continue;
                }
                
                let target_position = Point3::new(x, y, z);
                
                // Calculate tool orientation based on surface strategy
                let orientation_angles = self.calculate_surface_orientation(&target_position, surface_strategy)?;
                
                // Generate 5-axis G-code
                let mut output_line = format!("{} X{:.3} Y{:.3} Z{:.3} A{:.3} B{:.3}", 
                    &parts[0], x, y, z, orientation_angles.0, orientation_angles.1);

                // Without its E word a move prints nothing
                if let Some(e) = extrusion {
                    output_line.push_str(&format!(" E{}", e));
                }

                if let Some(f) = feedrate {
                    output_line.push_str(&format!(" F{}", f));
                }
                
                output_lines.push(output_line);
                current_position = target_position;
            } else {
                // Pass through non-move commands
                output_lines.push(trimmed.to_string());
            }
        }
        
        Ok(output_lines.join("\n"))
    }
    
    fn calculate_surface_orientation(&self, position: &Point3<f64>, strategy: &str) -> Result<(f64, f64), ProcessingError> {
        // Simplified surface orientation calculation
        // In practice, this would use the full ToolOrientationController
        
        match strategy {
            "vertical" => Ok((0.0, 0.0)),
            "surface_normal" => {
                // Calculate based on surface normal (placeholder)
                let normal_angle = (position.x / 100.0).atan().to_degrees();
                Ok((normal_angle, 0.0))
            },
            "adaptive" => {
                // Adaptive orientation based on local geometry. Upright at bed level, where
                // these angles are undefined (0/0 gave NaN, which came out as a hard tilt).
                if position.z.abs() < 1e-6 {
                    return Ok((0.0, 0.0));
                }
                let a_angle = (position.y / position.z).atan().to_degrees().max(-45.0).min(45.0);
                let b_angle = (position.x / position.z).atan().to_degrees().max(-30.0).min(30.0);
                Ok((a_angle, b_angle))
            },
            _ => Ok((0.0, 0.0)),
        }
    }
}

impl RepRapFirmwarePostProcessor {
    pub fn new(machine_type: PracticalMachineType) -> Self {
        Self { machine_type }
    }
}

impl PostProcessor for RepRapFirmwarePostProcessor {
    fn process_gcode(&self, input: &str, _config: &SlicerProfile) -> Result<String, ProcessingError> {
        let mut output_lines = Vec::new();
        
        // Add RRF-specific initialization
        // Kinematics are the machine's configuration. A print file must not change them.
        output_lines.push("; 5-axis G-code processed for RepRapFirmware".to_string());
        
        // Process each line
        for line in input.lines() {
            let trimmed = line.trim();
            
            if trimmed.starts_with("G1") || trimmed.starts_with("G0") {
                // Add segmentation for smooth 5-axis motion
                let segmented_line = format!("{} ; RRF 5-axis", trimmed);
                output_lines.push(segmented_line);
            } else {
                output_lines.push(trimmed.to_string());
            }
        }
        
        Ok(output_lines.join("\n"))
    }
    
    fn get_name(&self) -> &str {
        "RepRapFirmware"
    }
    
    fn supports_machine_type(&self, _machine_type: &PracticalMachineType) -> bool {
        true // RRF supports custom kinematics
    }
}

impl MarlinPostProcessor {
    pub fn new(machine_type: PracticalMachineType) -> Self {
        Self { machine_type }
    }
}

impl PostProcessor for MarlinPostProcessor {
    fn process_gcode(&self, input: &str, _config: &SlicerProfile) -> Result<String, ProcessingError> {
        let mut output_lines = Vec::new();
        
        output_lines.push("; 5-axis G-code processed for Marlin".to_string());
        
        // Marlin requires specific axis mapping
        for line in input.lines() {
            let trimmed = line.trim();
            
            // Only in the code part of moves: not in comments, nor in text like M117 messages
            let (code, comment) = match trimmed.find(';') {
                Some(index) => trimmed.split_at(index),
                None => (trimmed, ""),
            };
            let is_move = matches!(code.split_whitespace().next(), Some("G0" | "G1" | "G00" | "G01"));
            if is_move && (code.contains(" A") || code.contains(" B")) {
                // Convert A/B to appropriate Marlin axis names (often I/J)
                let converted = code.trim_end().replace(" A", " I").replace(" B", " J");
                output_lines.push(format!("{} {}", converted, if comment.is_empty() { "; Marlin 5-axis" } else { comment }));
            } else {
                output_lines.push(trimmed.to_string());
            }
        }
        
        Ok(output_lines.join("\n"))
    }
    
    fn get_name(&self) -> &str {
        "Marlin"
    }
    
    fn supports_machine_type(&self, machine_type: &PracticalMachineType) -> bool {
        match machine_type {
            PracticalMachineType::ToolheadRotary { .. } => true,
            PracticalMachineType::CoreXYRotary { .. } => true,
            _ => false, // Marlin has limited 5-axis support
        }
    }
}

impl KlipperPostProcessor {
    pub fn new(machine_type: PracticalMachineType) -> Self {
        Self { machine_type }
    }
}

impl PostProcessor for KlipperPostProcessor {
    fn process_gcode(&self, input: &str, _config: &SlicerProfile) -> Result<String, ProcessingError> {
        let mut output_lines = Vec::new();
        
        // No SET_KINEMATIC_POSITION here: it tells Klipper the machine is homed wherever it
        // happens to be, which defeats the protection against moving when not homed.
        output_lines.push("; 5-axis G-code processed for Klipper".to_string());
        
        for line in input.lines() {
            let trimmed = line.trim();
            
            if trimmed.starts_with("G1") || trimmed.starts_with("G0") {
                // Klipper can handle A/B axes directly with custom kinematics
                output_lines.push(format!("{} ; Klipper 5-axis", trimmed));
            } else {
                output_lines.push(trimmed.to_string());
            }
        }
        
        Ok(output_lines.join("\n"))
    }
    
    fn get_name(&self) -> &str {
        "Klipper"
    }
    
    fn supports_machine_type(&self, _machine_type: &PracticalMachineType) -> bool {
        true // Klipper supports custom kinematics modules
    }
}

#[cfg(test)]
mod repro_tests {
    use super::*;

    fn integration() -> SlicerIntegration {
        SlicerIntegration::new(PracticalKinematics::new(PracticalMachineType::penta_axis_style()), SlicerProfile::penta_axis_prusa_profile())
    }

    #[test]
    fn surface_following_keeps_extrusion_and_only_rewrites_moves() {
        let output = integration().convert_to_surface_following(
            "G1 X10 Y20 Z5 E1.5 F600 ; Z99 comment\nG10\nG1 E-1 F2400\nM104 S200\nG1 X0 Y0 Z0 E2",
            "adaptive",
        ).unwrap();
        let lines: Vec<&str> = output.lines().collect();
        assert!(lines[0].contains("E1.5") && lines[0].contains("Z5.000"), "{}", lines[0]);
        assert_eq!(lines[1], "G10");
        assert_eq!(lines[2], "G1 E-1 F2400");
        assert_eq!(lines[3], "M104 S200");
        // Bed level is upright, not a hard tilt
        assert!(lines[4].contains("A0.000 B0.000"), "{}", lines[4]);
    }

    #[test]
    fn post_processors_do_not_touch_machine_state_or_text() {
        let profile = SlicerProfile::penta_axis_prusa_profile();
        let input = "M117 A Big Print\nG1 X1 A5 B6 ; A comment\n";
        let marlin = MarlinPostProcessor::new(PracticalMachineType::penta_axis_style()).process_gcode(input, &profile).unwrap();
        assert!(marlin.contains("M117 A Big Print"), "{}", marlin);
        assert!(marlin.contains("G1 X1 I5 J6 ; A comment"), "{}", marlin);
        let klipper = KlipperPostProcessor::new(PracticalMachineType::penta_axis_style()).process_gcode(input, &profile).unwrap();
        assert!(!klipper.contains("SET_KINEMATIC_POSITION"));
        let rrf = RepRapFirmwarePostProcessor::new(PracticalMachineType::penta_axis_style()).process_gcode(input, &profile).unwrap();
        assert!(!rrf.contains("M669"));
    }
}

/// Factory functions for common slicer profiles
impl SlicerProfile {
    pub fn penta_axis_prusa_profile() -> Self {
        Self {
            name: "PentaAxis PrusaSlicer".to_string(),
            slicer_type: SlicerType::PrusaSlicer,
            machine_config: MachineProfile {
                bed_size: (220.0, 220.0, 250.0),
                nozzle_diameter: 0.4,
                filament_diameter: 1.75,
                max_feedrates: {
                    let mut map = HashMap::new();
                    map.insert("X".to_string(), 9000.0);
                    map.insert("Y".to_string(), 9000.0);
                    map.insert("Z".to_string(), 720.0);
                    map.insert("A".to_string(), 600.0);
                    map.insert("B".to_string(), 1800.0);
                    map
                },
                accelerations: {
                    let mut map = HashMap::new();
                    map.insert("X".to_string(), 1000.0);
                    map.insert("Y".to_string(), 1000.0);
                    map.insert("Z".to_string(), 100.0);
                    map.insert("A".to_string(), 100.0);
                    map.insert("B".to_string(), 300.0);
                    map
                },
                rotary_axes: vec![
                    RotaryAxisConfig {
                        name: "A".to_string(),
                        min_angle: -45.0,
                        max_angle: 45.0,
                        resolution: 0.09,  // degrees per step
                        home_position: 0.0,
                    },
                    RotaryAxisConfig {
                        name: "B".to_string(),
                        min_angle: -180.0,
                        max_angle: 180.0,
                        resolution: 0.15,  // degrees per step
                        home_position: 0.0,
                    },
                ],
                tool_offsets: {
                    let mut map = HashMap::new();
                    map.insert(1, Vector3::new(0.0, 0.0, 45.0));
                    map
                },
            },
            print_settings: PrintSettings {
                layer_height: 0.2,
                first_layer_height: 0.25,
                perimeters: 3,
                infill_percentage: 20.0,
                support_enabled: true,
                non_planar_enabled: true,
                adaptive_layer_height: true,
                surface_following: true,
            },
            material_settings: MaterialSettings {
                material_type: "PLA".to_string(),
                nozzle_temperature: 210.0,
                bed_temperature: 60.0,
                chamber_temperature: None,
                cooling_settings: CoolingSettings {
                    fan_speed: 100.0,
                    bridge_fan_speed: 100.0,
                    overhang_fan_speed: 100.0,
                    disable_fan_first_layers: 2,
                },
            },
            advanced_settings: AdvancedSettings {
                pressure_advance: 0.04,
                linear_advance: 0.0,
                input_shaping: true,
                junction_deviation: 0.05,
                max_acceleration_change: 1500.0,
                orientation_strategy: "surface_normal".to_string(),
            },
        }
    }
}