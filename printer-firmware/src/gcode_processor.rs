use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use heapless::{String, Vec};

use crate::exclude_object::{EXCLUDE_OBJECT_MANAGER, ObjectError};

const MAX_GCODE_LINE_LENGTH: usize = 256;
const MAX_GCODE_BUFFER: usize = 1024;

pub struct GcodeProcessor {
    current_line: u32,
    current_layer: u32,
    skipping_object: bool,
    position: [f32; 4], // X, Y, Z, E
}

impl GcodeProcessor {
    pub fn new() -> Self {
        Self {
            current_line: 0,
            current_layer: 0,
            skipping_object: false,
            position: [0.0; 4],
        }
    }

    pub async fn process_gcode_line(&mut self, line: &str) -> Result<GcodeAction, GcodeError> {
        self.current_line += 1;
        
        // Check if we should skip this line due to excluded objects
        let should_skip = {
            if let Ok(manager_guard) = EXCLUDE_OBJECT_MANAGER.try_lock() {
                if let Some(ref manager) = *manager_guard {
                    manager.should_skip_line(self.current_line).await
                } else {
                    false
                }
            } else {
                false
            }
        };

        if should_skip {
            return Ok(GcodeAction::Skip);
        }

        // Parse the line
        let clean_line = self.clean_gcode_line(line);
        
        // Check for layer changes
        if let Some(layer) = self.parse_layer_comment(&clean_line) {
            self.current_layer = layer;
            self.update_object_tracking().await;
            return Ok(GcodeAction::LayerChange(layer));
        }

        // Check for object markers
        if let Some(object_name) = self.parse_object_start(&clean_line) {
            self.handle_object_start(&object_name).await;
            return Ok(GcodeAction::ObjectStart(object_name));
        }

        if clean_line.contains("; end object") {
            self.handle_object_end().await;
            return Ok(GcodeAction::ObjectEnd);
        }

        // Parse G-code commands
        if let Some(gcode) = self.parse_gcode_command(&clean_line) {
            match gcode {
                GcodeCommand::Move { x, y, z, e, feed_rate } => {
                    self.update_position(x, y, z, e);
                    return Ok(GcodeAction::Execute(GcodeCommand::Move { x, y, z, e, feed_rate }));
                }
                GcodeCommand::SetTemperature { tool, temp } => {
                    return Ok(GcodeAction::Execute(GcodeCommand::SetTemperature { tool, temp }));
                }
                GcodeCommand::SetBedTemperature { temp } => {
                    return Ok(GcodeAction::Execute(GcodeCommand::SetBedTemperature { temp }));
                }
                GcodeCommand::SetFanSpeed { fan, speed } => {
                    return Ok(GcodeAction::Execute(GcodeCommand::SetFanSpeed { fan, speed }));
                }
                GcodeCommand::Home { axes } => {
                    return Ok(GcodeAction::Execute(GcodeCommand::Home { axes }));
                }
                _ => {
                    return Ok(GcodeAction::Execute(gcode));
                }
            }
        }

        // Default: execute the line as-is if it's not a comment
        if !clean_line.starts_with(';') && !clean_line.is_empty() {
            return Ok(GcodeAction::ExecuteRaw(clean_line));
        }

        Ok(GcodeAction::Skip)
    }

    fn clean_gcode_line(&self, line: &str) -> String<MAX_GCODE_LINE_LENGTH> {
        let trimmed = line.trim();
        String::try_from(trimmed).unwrap_or_default()
    }

    fn parse_layer_comment(&self, line: &str) -> Option<u32> {
        if let Some(start) = line.find(";LAYER:") {
            let layer_str = &line[start + 7..].trim();
            layer_str.parse().ok()
        } else if let Some(start) = line.find("; layer ") {
            let layer_str = &line[start + 8..].trim();
            layer_str.parse().ok()
        } else {
            None
        }
    }

    fn parse_object_start(&self, line: &str) -> Option<String<64>> {
        if let Some(start) = line.find("; object:") {
            let name = &line[start + 9..].trim();
            String::try_from(*name).ok()
        } else {
            None
        }
    }

    fn parse_gcode_command(&self, line: &str) -> Option<GcodeCommand> {
        let parts: Vec<&str, 16> = line.split_whitespace().collect();
        if parts.is_empty() {
            return None;
        }

        match parts[0] {
            "G0" | "G1" => {
                let mut x = None;
                let mut y = None;
                let mut z = None;
                let mut e = None;
                let mut feed_rate = None;

                for part in &parts[1..] {
                    if let Some(value_str) = part.strip_prefix('X') {
                        x = value_str.parse().ok();
                    } else if let Some(value_str) = part.strip_prefix('Y') {
                        y = value_str.parse().ok();
                    } else if let Some(value_str) = part.strip_prefix('Z') {
                        z = value_str.parse().ok();
                    } else if let Some(value_str) = part.strip_prefix('E') {
                        e = value_str.parse().ok();
                    } else if let Some(value_str) = part.strip_prefix('F') {
                        feed_rate = value_str.parse().ok();
                    }
                }

                Some(GcodeCommand::Move { x, y, z, e, feed_rate })
            }
            "G28" => {
                let mut axes = String::new();
                for part in &parts[1..] {
                    if part.len() == 1 && "XYZ".contains(part) {
                        axes.push_str(part).ok();
                    }
                }
                Some(GcodeCommand::Home { axes })
            }
            "M104" => {
                let mut tool = 0u8;
                let mut temp = 0f32;
                
                for part in &parts[1..] {
                    if let Some(value_str) = part.strip_prefix('T') {
                        tool = value_str.parse().unwrap_or(0);
                    } else if let Some(value_str) = part.strip_prefix('S') {
                        temp = value_str.parse().unwrap_or(0.0);
                    }
                }
                
                Some(GcodeCommand::SetTemperature { tool, temp })
            }
            "M140" => {
                let mut temp = 0f32;
                for part in &parts[1..] {
                    if let Some(value_str) = part.strip_prefix('S') {
                        temp = value_str.parse().unwrap_or(0.0);
                    }
                }
                Some(GcodeCommand::SetBedTemperature { temp })
            }
            "M106" => {
                let mut fan = 0u8;
                let mut speed = 255u8;
                
                for part in &parts[1..] {
                    if let Some(value_str) = part.strip_prefix('P') {
                        fan = value_str.parse().unwrap_or(0);
                    } else if let Some(value_str) = part.strip_prefix('S') {
                        speed = value_str.parse().unwrap_or(255);
                    }
                }
                
                Some(GcodeCommand::SetFanSpeed { fan, speed })
            }
            _ => Some(GcodeCommand::Other { command: String::try_from(line).unwrap_or_default() })
        }
    }

    fn update_position(&mut self, x: Option<f32>, y: Option<f32>, z: Option<f32>, e: Option<f32>) {
        if let Some(x_val) = x { self.position[0] = x_val; }
        if let Some(y_val) = y { self.position[1] = y_val; }
        if let Some(z_val) = z { self.position[2] = z_val; }
        if let Some(e_val) = e { self.position[3] = e_val; }
    }

    async fn update_object_tracking(&self) {
        if let Ok(mut manager_guard) = EXCLUDE_OBJECT_MANAGER.try_lock() {
            if let Some(ref mut manager) = *manager_guard {
                let _ = manager.update_current_object(self.current_line).await;
            }
        }
    }

    async fn handle_object_start(&mut self, object_name: &String<64>) {
        if let Ok(mut manager_guard) = EXCLUDE_OBJECT_MANAGER.try_lock() {
            if let Some(ref mut manager) = *manager_guard {
                self.skipping_object = manager.is_object_excluded(object_name.as_str()).await;
            }
        }
    }

    async fn handle_object_end(&mut self) {
        if let Ok(mut manager_guard) = EXCLUDE_OBJECT_MANAGER.try_lock() {
            if let Some(ref mut manager) = *manager_guard {
                let _ = manager.complete_current_object().await;
            }
        }
        self.skipping_object = false;
    }

    pub fn get_current_position(&self) -> [f32; 4] {
        self.position
    }

    pub fn get_current_line(&self) -> u32 {
        self.current_line
    }

    pub fn get_current_layer(&self) -> u32 {
        self.current_layer
    }
}

#[derive(Debug, Clone)]
pub enum GcodeCommand {
    Move { 
        x: Option<f32>, 
        y: Option<f32>, 
        z: Option<f32>, 
        e: Option<f32>, 
        feed_rate: Option<f32> 
    },
    Home { axes: String<8> },
    SetTemperature { tool: u8, temp: f32 },
    SetBedTemperature { temp: f32 },
    SetFanSpeed { fan: u8, speed: u8 },
    Dwell { time: f32 },
    Other { command: String<MAX_GCODE_LINE_LENGTH> },
}

#[derive(Debug)]
pub enum GcodeAction {
    Execute(GcodeCommand),
    ExecuteRaw(String<MAX_GCODE_LINE_LENGTH>),
    Skip,
    LayerChange(u32),
    ObjectStart(String<64>),
    ObjectEnd,
}

#[derive(Debug)]
pub enum GcodeError {
    InvalidCommand,
    ParseError,
    BufferFull,
}

pub static GCODE_PROCESSOR: Mutex<CriticalSectionRawMutex, Option<GcodeProcessor>> = Mutex::new(None);

pub async fn init_gcode_processor() -> &'static Mutex<CriticalSectionRawMutex, Option<GcodeProcessor>> {
    let processor = GcodeProcessor::new();
    
    {
        let mut guard = GCODE_PROCESSOR.lock().await;
        *guard = Some(processor);
    }
    
    &GCODE_PROCESSOR
}

// Print progress tracking
pub struct PrintProgress {
    pub total_lines: u32,
    pub processed_lines: u32,
    pub total_objects: usize,
    pub completed_objects: usize,
    pub excluded_objects: usize,
    pub current_layer: u32,
    pub estimated_layers: u32,
}

impl PrintProgress {
    pub fn progress_percentage(&self) -> f32 {
        if self.total_lines > 0 {
            (self.processed_lines as f32 / self.total_lines as f32) * 100.0
        } else {
            0.0
        }
    }

    pub fn layer_progress_percentage(&self) -> f32 {
        if self.estimated_layers > 0 {
            (self.current_layer as f32 / self.estimated_layers as f32) * 100.0
        } else {
            0.0
        }
    }

    pub fn objects_progress_percentage(&self) -> f32 {
        let active_objects = self.total_objects - self.excluded_objects;
        if active_objects > 0 {
            (self.completed_objects as f32 / active_objects as f32) * 100.0
        } else {
            0.0
        }
    }
}