use serde::{Deserialize, Serialize};
use heapless::{String, Vec};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

const MAX_OBJECTS: usize = 32;
const MAX_OBJECT_NAME_LENGTH: usize = 32;
const OBJECT_START_MARKER: &str = "; object:";
pub const OBJECT_END_MARKER: &str = "; end object";

/// The object name from a `; object: NAME [center:x,y] [time:n]` marker line. There is one
/// definition of this, so names always match wherever they are read.
pub fn parse_object_marker(line: &str) -> Option<String<MAX_OBJECT_NAME_LENGTH>> {
    let start = line.find(OBJECT_START_MARKER)? + OBJECT_START_MARKER.len();
    let mut name = line[start..].trim();
    for metadata in [" center:", " time:"] {
        if let Some(end) = name.find(metadata) {
            name = name[..end].trim_end();
        }
    }
    if name.is_empty() {
        return None;
    }
    String::try_from(name).ok()
}

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
    pub has_bounds: bool, // A move at exactly (0, 0) must not look like "no bounds yet"
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
            has_bounds: false,
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
        // A new print starts clean: an object named "A" excluded from the last print
        // must not silently disappear from this one.
        state.excluded_objects.clear();
        state.current_object = None;
        
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
        let Some(obj) = state.objects.iter_mut().find(|o| o.name.as_str() == object_name) else {
            return Err(ObjectError::ObjectNotFound);
        };
        match obj.status {
            ObjectStatus::Printing => return Err(ObjectError::ObjectCurrentlyPrinting),
            ObjectStatus::Completed => return Err(ObjectError::ObjectAlreadyCompleted),
            _ => {}
        }
        if obj.status == ObjectStatus::Failed {
            state.failed_objects = state.failed_objects.saturating_sub(1);
        }
        let Some(obj) = state.objects.iter_mut().find(|o| o.name.as_str() == object_name) else {
            return Err(ObjectError::ObjectNotFound);
        };
        obj.status = ObjectStatus::Excluded;
        
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
        let mut was_failed = false;
        for obj in state.objects.iter_mut() {
            if obj.name.as_str() == object_name {
                if obj.status == ObjectStatus::Printing {
                    return Err(ObjectError::ObjectCurrentlyPrinting);
                }
                if obj.status == ObjectStatus::Completed {
                    return Err(ObjectError::ObjectAlreadyCompleted);
                }
                was_failed = obj.status == ObjectStatus::Failed;
                obj.status = ObjectStatus::Pending;
                found = true;
                break;
            }
        }
        
        if !found {
            return Err(ObjectError::ObjectNotFound);
        }
        if was_failed {
            state.failed_objects = state.failed_objects.saturating_sub(1);
        }
        
        // Remove from excluded list
        if let Some(pos) = state.excluded_objects.iter().position(|x| x.as_str() == object_name) {
            state.excluded_objects.swap_remove(pos);
        }
        
        Ok(())
    }

    /// An object's section has started in the print. Returns true if it must be skipped.
    /// Objects usually have one section per layer, so this runs many times for each.
    pub async fn object_started(&mut self, object_name: &str, _line_number: u32) -> bool {
        let mut state = self.state.lock().await;
        if state.excluded_objects.iter().any(|name| name.as_str() == object_name) {
            return true;
        }
        let name = state.objects.iter_mut().find(|o| o.name.as_str() == object_name).map(|obj| {
            obj.status = ObjectStatus::Printing;
            obj.name.clone()
        });
        if name.is_some() {
            state.current_object = name;
        }
        false
    }

    /// The current object's section has ended. Only its last section completes it. In
    /// between it goes back to Pending, so it can still be excluded on a later layer.
    pub async fn object_finished(&mut self, line_number: u32) {
        let mut state = self.state.lock().await;
        let Some(current_name) = state.current_object.take() else {
            return;
        };
        let mut completed = false;
        if let Some(obj) = state.objects.iter_mut().find(|o| o.name == current_name) {
            if line_number >= obj.end_line {
                obj.status = ObjectStatus::Completed;
                obj.progress = 100.0;
                completed = true;
            } else {
                obj.status = ObjectStatus::Pending;
            }
        }
        if completed {
            state.completed_objects += 1;
        }
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
        
        // Only objects already marked as failed: one that is just starting has low progress
        // too, and that does not make it a failure.
        for obj in state.objects.iter() {
            if obj.status == ObjectStatus::Failed && obj.progress < failure_threshold {
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
            if !state.excluded_objects.iter().any(|existing| existing == name)
                && state.excluded_objects.push(name.clone()).is_err()
            {
                break;
            }
        }
        
        auto_excluded
    }
}

pub struct GcodeProcessor {
    object_pattern_end: &'static str,
}

impl GcodeProcessor {
    pub fn new() -> Self {
        Self {
            object_pattern_end: OBJECT_END_MARKER,
        }
    }

    pub async fn extract_objects(&self, gcode_content: &str) -> Result<Vec<PrintObject, MAX_OBJECTS>, ObjectError> {
        // One entry per object, however many sections (usually one per layer) it spans.
        let mut objects: Vec<PrintObject, MAX_OBJECTS> = Vec::new();
        let mut current_object: Option<usize> = None;
        let mut line_number = 0u32;
        let mut layer_number = 0u32;
        
        for line in gcode_content.lines() {
            line_number += 1;
            
            // Track layer changes
            if line.starts_with(";LAYER:") || line.starts_with("; layer ") {
                layer_number += 1;
            }
            
            // Check for object start
            if let Some(object_name) = parse_object_marker(line) {
                let index = match objects.iter().position(|o| o.name == object_name) {
                    Some(index) => index,
                    None => {
                        let mut obj = PrintObject::default();
                        obj.name = object_name;
                        obj.start_line = line_number;
                        obj.layer_start = layer_number;
                        obj.status = ObjectStatus::Pending;

                        // Extract object metadata if available
                        self.parse_object_metadata(&mut obj, line);

                        objects.push(obj).map_err(|_| ObjectError::TooManyObjects)?;
                        objects.len() - 1
                    }
                };
                objects[index].end_line = line_number;
                objects[index].layer_end = layer_number;
                current_object = Some(index);
            }

            // Check for object end
            if line.contains(self.object_pattern_end) {
                if let Some(index) = current_object.take() {
                    objects[index].end_line = line_number;
                    objects[index].layer_end = layer_number;
                }
            }

            // Update current object with movement data for bounds calculation
            if let Some(index) = current_object {
                self.update_object_bounds(&mut objects[index], line);
                // Also covers a missing end marker: the section runs to the last line seen
                objects[index].end_line = line_number;
                objects[index].layer_end = layer_number;
            }
        }

        Ok(objects)
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
                if !obj.has_bounds {
                    // First coordinates - initialize bounds
                    obj.has_bounds = true;
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
        // Up to the next space, as more metadata can follow. No fixed-size collection here:
        // the firmware must not panic on the contents of a file.
        let coords = coords_str.trim().split_whitespace().next().unwrap_or("");
        let mut parts = coords.split(',');
        let (Some(x), Some(y)) = (parts.next(), parts.next()) else {
            return Err(ObjectError::InvalidCoordinates);
        };
        let x = x.trim().parse().map_err(|_| ObjectError::InvalidCoordinates)?;
        let y = y.trim().parse().map_err(|_| ObjectError::InvalidCoordinates)?;
        Ok([x, y])
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