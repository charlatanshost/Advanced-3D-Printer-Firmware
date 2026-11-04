use serde::{Deserialize, Serialize};
use heapless::{String, Vec, FnvIndexMap};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

const MAX_OBJECTS: usize = 32;
const MAX_OBJECT_NAME_LENGTH: usize = 32;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum ObjectStatus {
    Pending,
    Printing,
    Completed,
    Excluded,
    Failed,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrintObject {
    pub name: String<MAX_OBJECT_NAME_LENGTH>,
    pub status: ObjectStatus,
    pub start_line: u32,
    pub end_line: u32,
    pub layer_start: u32,
    pub layer_end: u32,
    pub estimated_time: u32, // seconds
    pub progress: f32, // 0.0 to 100.0
    pub center: [f32; 2], // X, Y center coordinates
    pub bounds: [[f32; 2]; 2], // [[min_x, min_y], [max_x, max_y]]
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExcludeObjectState {
    pub objects: Vec<PrintObject, MAX_OBJECTS>,
    pub current_object: Option<String<MAX_OBJECT_NAME_LENGTH>>,
    pub excluded_objects: Vec<String<MAX_OBJECT_NAME_LENGTH>, MAX_OBJECTS>,
    pub total_objects: usize,
    pub completed_objects: usize,
    pub failed_objects: usize,
    pub print_active: bool,
}

impl Default for ObjectStatus {
    fn default() -> Self {
        ObjectStatus::Pending
    }
}

impl Default for PrintObject {
    fn default() -> Self {
        Self {
            name: String::new(),
            status: ObjectStatus::default(),
            start_line: 0,
            end_line: 0,
            layer_start: 0,
            layer_end: 0,
            estimated_time: 0,
            progress: 0.0,
            center: [0.0, 0.0],
            bounds: [[0.0, 0.0], [0.0, 0.0]],
        }
    }
}

impl Default for ExcludeObjectState {
    fn default() -> Self {
        Self {
            objects: Vec::new(),
            current_object: None,
            excluded_objects: Vec::new(),
            total_objects: 0,
            completed_objects: 0,
            failed_objects: 0,
            print_active: false,
        }
    }
}

pub struct ExcludeObjectManager {
    state: Mutex<CriticalSectionRawMutex, ExcludeObjectState>,
    gcode_processor: GcodeProcessor,
}

impl ExcludeObjectManager {
    pub fn new() -> Self {
        Self {
            state: Mutex::new(ExcludeObjectState::default()),
            gcode_processor: GcodeProcessor::new(),
        }
    }

    pub async fn parse_gcode_objects(&mut self, gcode_content: &str) -> Result<(), ObjectError> {
        let objects = self.gcode_processor.extract_objects(gcode_content).await?;
        
        let mut state = self.state.lock().await;
        state.objects.clear();
        
        for obj in objects.iter() {
            if state.objects.push(obj.clone()).is_err() {
                return Err(ObjectError::TooManyObjects);
            }
        }
        
        state.total_objects = state.objects.len();
        state.completed_objects = 0;
        state.failed_objects = 0;
        state.print_active = false;
        
        Ok(())
    }

    pub async fn exclude_object(&mut self, object_name: &str) -> Result<(), ObjectError> {
        let mut state = self.state.lock().await;
        
        // Find object and mark as excluded
        let mut found = false;
        for obj in state.objects.iter_mut() {
            if obj.name.as_str() == object_name {
                if obj.status == ObjectStatus::Printing {
                    return Err(ObjectError::ObjectCurrentlyPrinting);
                }
                obj.status = ObjectStatus::Excluded;
                found = true;
                break;
            }
        }
        
        if !found {
            return Err(ObjectError::ObjectNotFound);
        }
        
        // Add to excluded list
        let excluded_name = String::try_from(object_name).map_err(|_| ObjectError::InvalidName)?;
        if state.excluded_objects.iter().find(|&x| x == &excluded_name).is_none() {
            state.excluded_objects.push(excluded_name).map_err(|_| ObjectError::TooManyObjects)?;
        }
        
        Ok(())
    }

    pub async fn include_object(&mut self, object_name: &str) -> Result<(), ObjectError> {
        let mut state = self.state.lock().await;
        
        // Find object and mark as pending
        let mut found = false;
        for obj in state.objects.iter_mut() {
            if obj.name.as_str() == object_name {
                if obj.status == ObjectStatus::Printing {
                    return Err(ObjectError::ObjectCurrentlyPrinting);
                }
                if obj.status == ObjectStatus::Completed {
                    return Err(ObjectError::ObjectAlreadyCompleted);
                }
                obj.status = ObjectStatus::Pending;
                found = true;
                break;
            }
        }
        
        if !found {
            return Err(ObjectError::ObjectNotFound);
        }
        
        // Remove from excluded list
        if let Some(pos) = state.excluded_objects.iter().position(|x| x.as_str() == object_name) {
            state.excluded_objects.swap_remove(pos);
        }
        
        Ok(())
    }

    pub async fn should_skip_line(&self, line_number: u32) -> bool {
        let state = self.state.lock().await;
        
        for excluded_name in &state.excluded_objects {
            if let Some(obj) = state.objects.iter().find(|o| &o.name == excluded_name) {
                if line_number >= obj.start_line && line_number <= obj.end_line {
                    return true;
                }
            }
        }
        
        false
    }

    pub async fn update_current_object(&mut self, line_number: u32) -> Option<String<MAX_OBJECT_NAME_LENGTH>> {
        let mut state = self.state.lock().await;
        
        // First, find the object name that should be current
        let mut new_current: Option<String<MAX_OBJECT_NAME_LENGTH>> = None;
        
        for obj in state.objects.iter() {
            if line_number >= obj.start_line && line_number <= obj.end_line && obj.status != ObjectStatus::Excluded {
                if state.current_object.as_ref() != Some(&obj.name) {
                    new_current = Some(obj.name.clone());
                    break;
                }
            }
        }
        
        // If we found a new current object, update the state
        if let Some(ref new_name) = new_current {
            for obj in state.objects.iter_mut() {
                if &obj.name == new_name {
                    obj.status = ObjectStatus::Printing;
                    break;
                }
            }
            state.current_object = new_current.clone();
        }
        
        new_current
    }

    pub async fn complete_current_object(&mut self) -> Result<(), ObjectError> {
        let mut state = self.state.lock().await;
        
        if let Some(current_name) = state.current_object.clone() {
            for obj in state.objects.iter_mut() {
                if obj.name == current_name {
                    obj.status = ObjectStatus::Completed;
                    obj.progress = 100.0;
                    state.completed_objects += 1;
                    break;
                }
            }
            state.current_object = None;
        }
        
        Ok(())
    }

    pub async fn fail_current_object(&mut self) -> Result<(), ObjectError> {
        let mut state = self.state.lock().await;
        
        if let Some(current_name) = state.current_object.clone() {
            for obj in state.objects.iter_mut() {
                if obj.name == current_name {
                    obj.status = ObjectStatus::Failed;
                    state.failed_objects += 1;
                    break;
                }
            }
            state.current_object = None;
        }
        
        Ok(())
    }

    pub async fn get_object_list(&self) -> Vec<PrintObject, MAX_OBJECTS> {
        let state = self.state.lock().await;
        state.objects.clone()
    }

    pub async fn get_print_status(&self) -> (usize, usize, usize) {
        let state = self.state.lock().await;
        (state.total_objects, state.completed_objects, state.failed_objects)
    }

    pub async fn is_object_excluded(&self, object_name: &str) -> bool {
        let state = self.state.lock().await;
        state.excluded_objects.iter().any(|name| name.as_str() == object_name)
    }

    pub async fn get_current_object(&self) -> Option<String<MAX_OBJECT_NAME_LENGTH>> {
        let state = self.state.lock().await;
        state.current_object.clone()
    }

    pub async fn update_object_progress(&mut self, object_name: &str, progress: f32) {
        let mut state = self.state.lock().await;
        
        if let Some(obj) = state.objects.iter_mut().find(|o| o.name.as_str() == object_name) {
            obj.progress = progress.clamp(0.0, 100.0);
        }
    }

    pub async fn auto_exclude_failed_objects(&mut self, failure_threshold: f32) -> Vec<String<MAX_OBJECT_NAME_LENGTH>, 8> {
        let mut state = self.state.lock().await;
        let mut auto_excluded = Vec::new();
        
        // Collect names first to avoid borrow issues
        let mut objects_to_exclude = Vec::<String<MAX_OBJECT_NAME_LENGTH>, 8>::new();
        
        for obj in state.objects.iter() {
            if obj.status == ObjectStatus::Printing && obj.progress < failure_threshold {
                if objects_to_exclude.push(obj.name.clone()).is_err() {
                    break; // Vector full
                }
            }
        }
        
        // Now update the objects
        for exclude_name in &objects_to_exclude {
            for obj in state.objects.iter_mut() {
                if obj.name == *exclude_name {
                    obj.status = ObjectStatus::Excluded;
                    if auto_excluded.push(obj.name.clone()).is_err() {
                        break;
                    }
                    break;
                }
            }
        }
        
        // Add to excluded objects list separately
        for name in &objects_to_exclude {
            if state.excluded_objects.push(name.clone()).is_err() {
                break;
            }
        }
        
        auto_excluded
    }
}

pub struct GcodeProcessor {
    object_pattern_start: &'static str,
    object_pattern_end: &'static str,
}

impl GcodeProcessor {
    pub fn new() -> Self {
        Self {
            object_pattern_start: "; object:",
            object_pattern_end: "; end object",
        }
    }

    pub async fn extract_objects(&self, gcode_content: &str) -> Result<Vec<PrintObject, MAX_OBJECTS>, ObjectError> {
        let mut objects = Vec::new();
        let mut current_object: Option<PrintObject> = None;
        let mut line_number = 0u32;
        let mut layer_number = 0u32;
        
        for line in gcode_content.lines() {
            line_number += 1;
            
            // Track layer changes
            if line.starts_with(";LAYER:") || line.starts_with("; layer ") {
                layer_number += 1;
            }
            
            // Check for object start
            if let Some(object_name) = self.parse_object_start(line) {
                let mut obj = PrintObject::default();
                obj.name = object_name;
                obj.start_line = line_number;
                obj.layer_start = layer_number;
                obj.status = ObjectStatus::Pending;
                
                // Extract object metadata if available
                self.parse_object_metadata(&mut obj, line);
                
                current_object = Some(obj);
            }
            
            // Check for object end
            if line.contains(self.object_pattern_end) {
                if let Some(mut obj) = current_object.take() {
                    obj.end_line = line_number;
                    obj.layer_end = layer_number;
                    
                    if objects.push(obj).is_err() {
                        return Err(ObjectError::TooManyObjects);
                    }
                }
            }
            
            // Update current object with movement data for bounds calculation
            if let Some(ref mut obj) = current_object {
                self.update_object_bounds(obj, line);
            }
        }
        
        // Handle case where object end marker is missing
        if let Some(mut obj) = current_object {
            obj.end_line = line_number;
            obj.layer_end = layer_number;
            objects.push(obj).map_err(|_| ObjectError::TooManyObjects)?;
        }
        
        Ok(objects)
    }

    fn parse_object_start(&self, line: &str) -> Option<String<MAX_OBJECT_NAME_LENGTH>> {
        if let Some(start_pos) = line.find(self.object_pattern_start) {
            let name_start = start_pos + self.object_pattern_start.len();
            let name = line[name_start..].trim();
            if !name.is_empty() {
                return String::try_from(name).ok();
            }
        }
        None
    }

    fn parse_object_metadata(&self, obj: &mut PrintObject, line: &str) {
        // Parse object center coordinates if available
        if let Some(center_data) = line.find("center:") {
            let coords_str = &line[center_data + 7..];
            if let Ok(coords) = self.parse_coordinates(coords_str) {
                obj.center = coords;
            }
        }
        
        // Parse estimated time if available
        if let Some(time_data) = line.find("time:") {
            let time_str = &line[time_data + 5..].trim();
            if let Ok(time) = time_str.parse::<u32>() {
                obj.estimated_time = time;
            }
        }
    }

    fn update_object_bounds(&self, obj: &mut PrintObject, line: &str) {
        // Parse G1/G0 moves to calculate object bounds
        if line.starts_with("G1 ") || line.starts_with("G0 ") {
            if let (Some(x), Some(y)) = (self.extract_coordinate(line, 'X'), self.extract_coordinate(line, 'Y')) {
                // Update bounds
                if obj.bounds[0][0] == 0.0 && obj.bounds[0][1] == 0.0 && 
                   obj.bounds[1][0] == 0.0 && obj.bounds[1][1] == 0.0 {
                    // First coordinates - initialize bounds
                    obj.bounds[0] = [x, y]; // min
                    obj.bounds[1] = [x, y]; // max
                } else {
                    // Update bounds
                    obj.bounds[0][0] = obj.bounds[0][0].min(x); // min_x
                    obj.bounds[0][1] = obj.bounds[0][1].min(y); // min_y
                    obj.bounds[1][0] = obj.bounds[1][0].max(x); // max_x
                    obj.bounds[1][1] = obj.bounds[1][1].max(y); // max_y
                }
                
                // Update center
                obj.center[0] = (obj.bounds[0][0] + obj.bounds[1][0]) / 2.0;
                obj.center[1] = (obj.bounds[0][1] + obj.bounds[1][1]) / 2.0;
            }
        }
    }

    fn extract_coordinate(&self, line: &str, axis: char) -> Option<f32> {
        if let Some(start) = line.find(axis) {
            let coord_str = &line[start + 1..];
            if let Some(end) = coord_str.find(' ') {
                coord_str[..end].parse().ok()
            } else {
                coord_str.trim().parse().ok()
            }
        } else {
            None
        }
    }

    fn parse_coordinates(&self, coords_str: &str) -> Result<[f32; 2], ObjectError> {
        let parts: Vec<&str, 4> = coords_str.split(',').collect();
        if parts.len() >= 2 {
            let x = parts[0].trim().parse().map_err(|_| ObjectError::InvalidCoordinates)?;
            let y = parts[1].trim().parse().map_err(|_| ObjectError::InvalidCoordinates)?;
            Ok([x, y])
        } else {
            Err(ObjectError::InvalidCoordinates)
        }
    }
}

#[derive(Debug)]
pub enum ObjectError {
    TooManyObjects,
    ObjectNotFound,
    ObjectCurrentlyPrinting,
    ObjectAlreadyCompleted,
    InvalidName,
    InvalidCoordinates,
    ParseError,
}

pub static EXCLUDE_OBJECT_MANAGER: Mutex<CriticalSectionRawMutex, Option<ExcludeObjectManager>> = Mutex::new(None);

pub async fn init_exclude_object_manager() -> &'static Mutex<CriticalSectionRawMutex, Option<ExcludeObjectManager>> {
    let manager = ExcludeObjectManager::new();
    
    {
        let mut guard = EXCLUDE_OBJECT_MANAGER.lock().await;
        *guard = Some(manager);
    }
    
    &EXCLUDE_OBJECT_MANAGER
}