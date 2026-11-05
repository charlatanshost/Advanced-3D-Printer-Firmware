// Advanced Motion Planning and Control Systems
// This module implements sophisticated motion control features including:
// - Volumetric speed control
// - Advanced lookahead and trajectory planning
// - Closed-loop servo control
// - Multi-tool management
// - Active resonance cancellation
// - Dynamic torque and force control

use std::collections::{VecDeque, HashMap};
use std::time::{Duration, Instant};
use serde::{Deserialize, Serialize};

// Volumetric Speed Control System
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VolumetricSettings {
    pub max_volumetric_rate: f32,      // mm³/s - maximum filament flow rate
    pub filament_diameter: f32,        // mm - filament diameter
    pub nozzle_diameter: f32,          // mm - nozzle diameter
    pub layer_height: f32,             // mm - current layer height
    pub line_width: f32,               // mm - current line width
    pub density_factor: f32,           // 0.0-1.0 - fill density
    pub material_flow_coefficient: f32, // Material-specific flow adjustment
}

impl Default for VolumetricSettings {
    fn default() -> Self {
        Self {
            max_volumetric_rate: 24.0,  // mm³/s - typical for PLA at 200°C
            filament_diameter: 1.75,    // mm
            nozzle_diameter: 0.4,       // mm
            layer_height: 0.2,          // mm
            line_width: 0.4,            // mm
            density_factor: 1.0,        // 100% density
            material_flow_coefficient: 1.0,
        }
    }
}

pub struct VolumetricController {
    settings: VolumetricSettings,
    current_flow_rate: f32,            // mm³/s
    target_flow_rate: f32,             // mm³/s
    flow_history: VecDeque<(Instant, f32)>,
    flow_smoothing_window: Duration,
}

impl VolumetricController {
    pub fn new(settings: VolumetricSettings) -> Self {
        Self {
            settings,
            current_flow_rate: 0.0,
            target_flow_rate: 0.0,
            flow_history: VecDeque::new(),
            flow_smoothing_window: Duration::from_millis(100),
        }
    }
    
    pub fn calculate_volumetric_rate(&self, extrusion_rate: f32, line_width: f32, layer_height: f32) -> f32 {
        // Calculate cross-sectional area of extruded material
        let cross_section = line_width * layer_height * self.settings.density_factor;
        
        // Volume = cross-section × speed
        cross_section * extrusion_rate * self.settings.material_flow_coefficient
    }
    
    pub fn calculate_max_speed(&self, line_width: f32, layer_height: f32) -> f32 {
        let cross_section = line_width * layer_height * self.settings.density_factor;
        
        if cross_section > 0.0 {
            self.settings.max_volumetric_rate / cross_section / self.settings.material_flow_coefficient
        } else {
            f32::INFINITY
        }
    }
    
    pub fn limit_feedrate(&self, requested_speed: f32, line_width: f32, layer_height: f32) -> f32 {
        let max_speed = self.calculate_max_speed(line_width, layer_height);
        requested_speed.min(max_speed)
    }
    
    pub fn update_flow_rate(&mut self, current_speed: f32, line_width: f32, layer_height: f32) {
        let now = Instant::now();
        let volumetric_rate = self.calculate_volumetric_rate(current_speed, line_width, layer_height);
        
        // Add to history
        self.flow_history.push_back((now, volumetric_rate));
        
        // Remove old entries
        while let Some(&(timestamp, _)) = self.flow_history.front() {
            if now.duration_since(timestamp) > self.flow_smoothing_window {
                self.flow_history.pop_front();
            } else {
                break;
            }
        }
        
        // Calculate smoothed flow rate
        if !self.flow_history.is_empty() {
            let sum: f32 = self.flow_history.iter().map(|(_, rate)| rate).sum();
            self.current_flow_rate = sum / self.flow_history.len() as f32;
        }
        
        self.target_flow_rate = volumetric_rate;
    }
    
    pub fn get_flow_utilization(&self) -> f32 {
        if self.settings.max_volumetric_rate > 0.0 {
            (self.current_flow_rate / self.settings.max_volumetric_rate).clamp(0.0, 1.0)
        } else {
            0.0
        }
    }
    
    pub fn update_settings(&mut self, settings: VolumetricSettings) {
        self.settings = settings;
    }
}

// Advanced Lookahead and Trajectory Planning
#[derive(Debug, Clone)]
pub struct MotionSegment {
    pub start_pos: [f32; 5],           // X, Y, Z, A, B positions
    pub end_pos: [f32; 5],
    pub start_velocity: f32,
    pub end_velocity: f32,
    pub max_velocity: f32,
    pub acceleration: f32,
    pub deceleration: f32,
    pub distance: f32,
    pub duration: f32,
    pub extrusion_rate: f32,
    pub line_width: f32,
    pub layer_height: f32,
    pub segment_type: SegmentType,
}

#[derive(Debug, Clone, PartialEq)]
pub enum SegmentType {
    Travel,      // Non-extruding move
    Extrude,     // Extruding move
    Retract,     // Retraction move
    Prime,       // Prime/unretract move
}

pub struct TrajectoryPlanner {
    segments: VecDeque<MotionSegment>,
    lookahead_buffer_size: usize,
    max_acceleration: f32,
    max_velocity: f32,
    junction_deviation: f32,
    minimum_planner_speed: f32,
    volumetric_controller: VolumetricController,
}

impl TrajectoryPlanner {
    pub fn new(volumetric_settings: VolumetricSettings) -> Self {
        Self {
            segments: VecDeque::new(),
            lookahead_buffer_size: 64,
            max_acceleration: 3000.0,      // mm/s²
            max_velocity: 300.0,           // mm/s
            junction_deviation: 0.02,      // mm
            minimum_planner_speed: 0.0,    // mm/s
            volumetric_controller: VolumetricController::new(volumetric_settings),
        }
    }
    
    /// Queue a move. Returns false if the buffer is full and the move was not accepted, so
    /// the caller must retry after taking segments. Dropping queued motion to make room would
    /// silently skip part of the print.
    #[must_use]
    pub fn add_segment(&mut self, mut segment: MotionSegment) -> bool {
        if self.segments.len() >= self.lookahead_buffer_size {
            return false;
        }

        // Apply volumetric speed limiting
        if segment.segment_type == SegmentType::Extrude {
            let volumetric_limit = self.volumetric_controller.calculate_max_speed(
                segment.line_width, 
                segment.layer_height
            );
            segment.max_velocity = segment.max_velocity.min(volumetric_limit);
        }
        
        self.segments.push_back(segment);

        // Perform lookahead planning
        self.recalculate_velocities();
        true
    }
    
    fn recalculate_velocities(&mut self) {
        let count = self.segments.len();
        if count == 0 {
            return;
        }

        // Worked out from scratch each time, so adding a move can raise a speed that was
        // low only because the path used to end there. velocity[i] is the speed at the start
        // of segment i. The first is fixed, and the last must be able to stop.
        let mut velocity = vec![0.0f32; count + 1];
        velocity[0] = self.segments[0].start_velocity;
        for i in 1..count {
            velocity[i] = self.calculate_junction_velocity(&self.segments[i - 1], &self.segments[i]);
        }

        // Backward pass: slow enough to be able to brake for what comes next
        for i in (1..count).rev() {
            velocity[i] = velocity[i].min(self.calculate_max_entry_velocity(&self.segments[i], velocity[i + 1]));
        }

        // Forward pass: no faster than can be reached from what came before
        for i in 0..count {
            let segment = &self.segments[i];
            let reachable = (velocity[i].powi(2) + 2.0 * segment.acceleration * segment.distance).sqrt();
            velocity[i + 1] = velocity[i + 1].min(reachable);
        }

        for (i, segment) in self.segments.iter_mut().enumerate() {
            segment.start_velocity = velocity[i];
            segment.end_velocity = velocity[i + 1];
        }

        // Recalculate segment durations based on new velocities
        let segments = &mut self.segments;
        for segment in segments.iter_mut() {
            TrajectoryPlanner::calculate_segment_timing_static(segment);
        }
    }
    
    fn calculate_junction_velocity(&self, prev: &MotionSegment, current: &MotionSegment) -> f32 {
        // Calculate angle between segments
        let prev_dir = self.normalize_direction(
            [current.start_pos[0] - prev.start_pos[0],
             current.start_pos[1] - prev.start_pos[1],
             current.start_pos[2] - prev.start_pos[2]]
        );
        
        let current_dir = self.normalize_direction(
            [current.end_pos[0] - current.start_pos[0],
             current.end_pos[1] - current.start_pos[1],
             current.end_pos[2] - current.start_pos[2]]
        );
        
        let dot_product = prev_dir[0] * current_dir[0] + 
                         prev_dir[1] * current_dir[1] + 
                         prev_dir[2] * current_dir[2];
        
        // Junction deviation. With θ the interior angle (180 degrees when going straight on,
        // 0 for a full reversal), the fillet radius is δ·sin(θ/2) / (1 - sin(θ/2)).
        let sin_half_interior = ((1.0 + dot_product.clamp(-1.0, 1.0)) / 2.0).sqrt();
        let junction_velocity = if sin_half_interior > 0.9999 {
            self.max_velocity
        } else {
            let radius = self.junction_deviation * sin_half_interior / (1.0 - sin_half_interior);
            (self.max_acceleration * radius).sqrt()
        };
        
        junction_velocity.min(prev.max_velocity).min(current.max_velocity)
    }
    
    fn normalize_direction(&self, dir: [f32; 3]) -> [f32; 3] {
        let magnitude = (dir[0].powi(2) + dir[1].powi(2) + dir[2].powi(2)).sqrt();
        if magnitude > 0.001 {
            [dir[0] / magnitude, dir[1] / magnitude, dir[2] / magnitude]
        } else {
            [0.0, 0.0, 0.0]
        }
    }
    
    fn calculate_max_entry_velocity(&self, current: &MotionSegment, exit_velocity: f32) -> f32 {
        // Calculate maximum entry velocity based on acceleration limits and exit velocity
        let acceleration_distance = current.distance;
        let max_velocity_squared = exit_velocity.powi(2) + 2.0 * current.acceleration * acceleration_distance;
        
        if max_velocity_squared > 0.0 {
            max_velocity_squared.sqrt().min(current.max_velocity)
        } else {
            0.0
        }
    }
    
    fn calculate_segment_timing(&self, segment: &mut MotionSegment) {
        TrajectoryPlanner::calculate_segment_timing_static(segment);
    }
    
    fn calculate_segment_timing_static(segment: &mut MotionSegment) {
        // Timing for a trapezoidal velocity profile. Averaging only the end speeds would
        // make a move from rest to rest take zero time.
        let (v0, v1, a) = (segment.start_velocity, segment.end_velocity, segment.acceleration);
        if segment.distance <= 0.0 {
            segment.duration = 0.0;
        } else if a <= 0.0 {
            // No acceleration figure: assume a steady move at the speed limit
            segment.duration = segment.distance / segment.max_velocity.max(1e-6);
        } else {
            let peak = ((2.0 * a * segment.distance + v0 * v0 + v1 * v1) / 2.0).sqrt();
            let cruise = peak.min(segment.max_velocity).max(v0).max(v1).max(1e-6);
            let ramp_up = (cruise - v0) / a;
            let ramp_down = (cruise - v1) / a;
            let ramps = (v0 + cruise) / 2.0 * ramp_up + (cruise + v1) / 2.0 * ramp_down;
            segment.duration = ramp_up + ramp_down + (segment.distance - ramps).max(0.0) / cruise;
        }
    }
    
    pub fn get_next_segment(&mut self) -> Option<MotionSegment> {
        self.segments.pop_front()
    }
    
    pub fn get_buffer_size(&self) -> usize {
        self.segments.len()
    }
    
    pub fn update_settings(&mut self, max_accel: f32, max_vel: f32, junction_dev: f32) {
        self.max_acceleration = max_accel;
        self.max_velocity = max_vel;
        self.junction_deviation = junction_dev;
    }
}

// Closed-Loop Servo Control with Encoder Feedback
#[derive(Debug, Clone)]
pub struct ServoAxis {
    pub axis_id: char,
    pub current_position: f32,        // Current encoder position (mm)
    pub target_position: f32,         // Target position (mm)
    pub current_velocity: f32,        // Current velocity (mm/s)
    pub target_velocity: f32,         // Target velocity (mm/s)
    pub position_error: f32,          // Position error (mm)
    pub velocity_error: f32,          // Velocity error (mm/s)
    pub encoder_counts: i64,          // Raw encoder counts
    pub encoder_resolution: f32,      // Counts per mm
    pub max_following_error: f32,     // Maximum allowable position error (mm)
    pub last_update: Instant,
}

pub struct ServoController {
    axes: HashMap<char, ServoAxis>,
    position_kp: f32,                 // Position loop proportional gain
    position_ki: f32,                 // Position loop integral gain
    position_kd: f32,                 // Position loop derivative gain
    velocity_kp: f32,                 // Velocity loop proportional gain
    velocity_ki: f32,                 // Velocity loop integral gain
    velocity_ff: f32,                 // Velocity feedforward gain
    integral_limits: f32,             // Integral windup limit
    position_integrals: HashMap<char, f32>,
    velocity_integrals: HashMap<char, f32>,
    previous_errors: HashMap<char, (f32, f32)>, // (position_error, velocity_error)
}

impl ServoController {
    pub fn new() -> Self {
        Self {
            axes: HashMap::new(),
            position_kp: 10.0,           // Position loop gains
            position_ki: 0.1,
            position_kd: 0.5,
            velocity_kp: 0.5,            // Velocity loop gains
            velocity_ki: 0.05,
            velocity_ff: 1.0,
            integral_limits: 1000.0,
            position_integrals: HashMap::new(),
            velocity_integrals: HashMap::new(),
            previous_errors: HashMap::new(),
        }
    }
    
    pub fn add_axis(&mut self, axis_id: char, encoder_resolution: f32, max_following_error: f32) {
        let servo_axis = ServoAxis {
            axis_id,
            current_position: 0.0,
            target_position: 0.0,
            current_velocity: 0.0,
            target_velocity: 0.0,
            position_error: 0.0,
            velocity_error: 0.0,
            encoder_counts: 0,
            encoder_resolution,
            max_following_error,
            last_update: Instant::now(),
        };
        
        self.axes.insert(axis_id, servo_axis);
        self.position_integrals.insert(axis_id, 0.0);
        self.velocity_integrals.insert(axis_id, 0.0);
        self.previous_errors.insert(axis_id, (0.0, 0.0));
    }
    
    pub fn update_encoder_position(&mut self, axis_id: char, encoder_counts: i64) -> Result<(), String> {
        if let Some(axis) = self.axes.get_mut(&axis_id) {
            // Calculate velocity from position change. Only take a new sample when the
            // velocity updates, or the position would change without its time being counted.
            let now = Instant::now();
            let dt = now.duration_since(axis.last_update).as_secs_f32();
            let previous_position = axis.current_position;
            let position = encoder_counts as f32 / axis.encoder_resolution;
            axis.encoder_counts = encoder_counts;
            axis.current_position = position;
            if dt > 0.001 { // Minimum 1ms between updates
                axis.current_velocity = (position - previous_position) / dt;
                axis.last_update = now;
            }
            
            Ok(())
        } else {
            Err(format!("Axis {} not found", axis_id))
        }
    }
    
    pub fn set_target(&mut self, axis_id: char, target_pos: f32, target_vel: f32) -> Result<(), String> {
        if let Some(axis) = self.axes.get_mut(&axis_id) {
            axis.target_position = target_pos;
            axis.target_velocity = target_vel;
            Ok(())
        } else {
            Err(format!("Axis {} not found", axis_id))
        }
    }
    
    pub fn calculate_servo_output(&mut self, axis_id: char, dt: f32) -> Result<f32, String> {
        if !(dt > 0.0) {
            return Err(format!("Invalid time step on axis {}: {}", axis_id, dt));
        }
        if let Some(axis) = self.axes.get_mut(&axis_id) {
            // Calculate errors
            axis.position_error = axis.target_position - axis.current_position;
            axis.velocity_error = axis.target_velocity - axis.current_velocity;
            
            // Check following error
            if axis.position_error.abs() > axis.max_following_error {
                return Err(format!("Following error exceeded on axis {}: {:.3}mm", 
                    axis_id, axis.position_error));
            }
            
            // Position loop PID
            let position_integral = self.position_integrals.get_mut(&axis_id).unwrap();
            *position_integral += axis.position_error * dt;
            *position_integral = position_integral.clamp(-self.integral_limits, self.integral_limits);
            
            let (prev_pos_error, _) = *self.previous_errors.get(&axis_id).unwrap_or(&(0.0, 0.0));
            let position_derivative = (axis.position_error - prev_pos_error) / dt;
            
            let position_output = self.position_kp * axis.position_error +
                                self.position_ki * *position_integral +
                                self.position_kd * position_derivative;
            
            // Velocity loop PID with feedforward
            let velocity_setpoint = axis.target_velocity + position_output;
            let velocity_error_loop = velocity_setpoint - axis.current_velocity;
            
            let velocity_integral = self.velocity_integrals.get_mut(&axis_id).unwrap();
            *velocity_integral += velocity_error_loop * dt;
            *velocity_integral = velocity_integral.clamp(-self.integral_limits, self.integral_limits);
            
            let (_, prev_vel_error) = *self.previous_errors.get(&axis_id).unwrap_or(&(0.0, 0.0));
            let velocity_derivative = (velocity_error_loop - prev_vel_error) / dt;
            
            let velocity_output = self.velocity_kp * velocity_error_loop +
                                self.velocity_ki * *velocity_integral +
                                self.velocity_ff * velocity_setpoint;
            
            // Store previous errors
            self.previous_errors.insert(axis_id, (axis.position_error, velocity_error_loop));
            
            Ok(velocity_output)
        } else {
            Err(format!("Axis {} not found", axis_id))
        }
    }
    
    pub fn get_axis_status(&self, axis_id: char) -> Option<&ServoAxis> {
        self.axes.get(&axis_id)
    }
    
    pub fn tune_position_loop(&mut self, kp: f32, ki: f32, kd: f32) {
        self.position_kp = kp;
        self.position_ki = ki;
        self.position_kd = kd;
    }
    
    pub fn tune_velocity_loop(&mut self, kp: f32, ki: f32, ff: f32) {
        self.velocity_kp = kp;
        self.velocity_ki = ki;
        self.velocity_ff = ff;
    }
    
    pub fn reset_integrals(&mut self, axis_id: Option<char>) {
        match axis_id {
            Some(id) => {
                self.position_integrals.insert(id, 0.0);
                self.velocity_integrals.insert(id, 0.0);
            }
            None => {
                for integral in self.position_integrals.values_mut() {
                    *integral = 0.0;
                }
                for integral in self.velocity_integrals.values_mut() {
                    *integral = 0.0;
                }
            }
        }
    }
}

// Multi-Tool and Multi-Material Management System
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolOffset {
    pub x: f32,                       // X offset from reference tool (mm)
    pub y: f32,                       // Y offset (mm)
    pub z: f32,                       // Z offset (mm)
    pub a: f32,                       // A axis offset (degrees)
    pub b: f32,                       // B axis offset (degrees)
}

impl Default for ToolOffset {
    fn default() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0, a: 0.0, b: 0.0 }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MaterialSettings {
    pub name: String,
    pub nozzle_temperature: f32,      // °C
    pub bed_temperature: f32,         // °C
    pub flow_rate: f32,               // Flow multiplier (0.8-1.2)
    pub retraction_distance: f32,     // mm
    pub retraction_speed: f32,        // mm/s
    pub print_speed: f32,             // mm/s
    pub first_layer_speed: f32,       // mm/s
    pub volumetric_rate: f32,         // mm³/s
    pub linear_advance_k: f32,        // Linear advance K factor
    pub pressure_advance: f32,        // Pressure advance value
}

impl Default for MaterialSettings {
    fn default() -> Self {
        Self {
            name: "PLA".to_string(),
            nozzle_temperature: 200.0,
            bed_temperature: 60.0,
            flow_rate: 1.0,
            retraction_distance: 0.8,
            retraction_speed: 35.0,
            print_speed: 60.0,
            first_layer_speed: 30.0,
            volumetric_rate: 24.0,
            linear_advance_k: 0.0,
            pressure_advance: 0.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolHead {
    pub tool_id: u8,
    pub name: String,
    pub offset: ToolOffset,
    pub material: MaterialSettings,
    pub nozzle_diameter: f32,         // mm
    pub heater_id: u8,                // Heater channel
    pub fan_id: u8,                   // Fan channel  
    pub extruder_steps_per_mm: f32,   // Steps/mm for this extruder
    pub max_temp: f32,                // Maximum safe temperature
    pub can_uuid: Option<u32>,        // CAN bus UUID if using CAN toolheads
    pub is_active: bool,              // Currently selected tool
    pub is_calibrated: bool,          // Tool offset calibrated
    #[serde(skip)]
    pub last_used: Option<Instant>,   // Last usage timestamp
}

impl ToolHead {
    pub fn new(tool_id: u8, name: String) -> Self {
        Self {
            tool_id,
            name,
            offset: ToolOffset::default(),
            material: MaterialSettings::default(),
            nozzle_diameter: 0.4,
            heater_id: tool_id,
            fan_id: tool_id,
            extruder_steps_per_mm: 93.0,
            max_temp: 300.0,
            can_uuid: None,
            is_active: false,
            is_calibrated: false,
            last_used: None,
        }
    }
    
    pub fn update_offset(&mut self, offset: ToolOffset) {
        self.offset = offset;
        self.is_calibrated = true;
    }
    
    pub fn update_material(&mut self, material: MaterialSettings) {
        self.material = material;
    }
}

pub struct MultiToolManager {
    tools: HashMap<u8, ToolHead>,
    active_tool: Option<u8>,
    reference_tool: u8,               // Tool 0 is usually reference
    tool_change_position: [f32; 3],   // X, Y, Z position for tool changes
    tool_change_retraction: f32,      // Retraction distance during tool change
    purge_volume: f32,                // mm³ - purge volume when changing materials
    wipe_sequence: Vec<[f32; 3]>,     // Wipe tower positions
    current_z: f32,                   // Current print height, so tool changes can clear the part
    mixing_ratios: HashMap<String, Vec<f32>>, // For mixing extruders
}

impl MultiToolManager {
    pub fn new() -> Self {
        let mut manager = Self {
            tools: HashMap::new(),
            active_tool: None,
            reference_tool: 0,
            tool_change_position: [150.0, 150.0, 50.0], // Safe tool change position
            tool_change_retraction: 2.0,
            purge_volume: 100.0,
            wipe_sequence: Vec::new(),
            current_z: 0.0,
            mixing_ratios: HashMap::new(),
        };
        
        // Add default tools
        manager.add_tool(ToolHead::new(0, "Primary Extruder".to_string()));
        manager.add_tool(ToolHead::new(1, "Secondary Extruder".to_string()));
        
        // Set up default wipe tower sequence
        manager.wipe_sequence = vec![
            [200.0, 200.0, 0.5],  // Wipe start
            [200.0, 205.0, 0.5],  // Wipe line
            [205.0, 205.0, 0.5],  // Wipe corner
            [205.0, 200.0, 0.5],  // Wipe return
        ];
        
        manager
    }
    
    pub fn add_tool(&mut self, tool: ToolHead) {
        let tool_id = tool.tool_id;
        self.tools.insert(tool_id, tool);
        
        // Set first tool as active if none selected
        if self.active_tool.is_none() {
            self.active_tool = Some(tool_id);
            if let Some(tool) = self.tools.get_mut(&tool_id) {
                tool.is_active = true;
            }
        }
    }
    
    /// Keep this up to date during a print. Tool changes use it to stay above the part.
    pub fn set_current_z(&mut self, z: f32) {
        self.current_z = z;
    }

    pub fn select_tool(&mut self, tool_id: u8) -> Result<Vec<ToolChangeCommand>, String> {
        if !self.tools.contains_key(&tool_id) {
            return Err(format!("Tool {} not found", tool_id));
        }
        
        if Some(tool_id) == self.active_tool {
            return Ok(Vec::new()); // Already active
        }
        
        let mut commands = Vec::new();
        
        // Deactivate current tool
        if let Some(current_id) = self.active_tool {
            if let Some(current_tool) = self.tools.get_mut(&current_id) {
                current_tool.is_active = false;
                current_tool.last_used = Some(Instant::now());
                
                // Add retraction command
                commands.push(ToolChangeCommand::Retract {
                    distance: self.tool_change_retraction,
                    speed: current_tool.material.retraction_speed,
                });
                
                // Set heater to standby
                commands.push(ToolChangeCommand::SetTemperature {
                    heater_id: current_tool.heater_id,
                    temperature: (current_tool.material.nozzle_temperature - 50.0).max(0.0), // Standby temp
                });
            }
        }
        
        // Move to tool change position, staying above the print. A fixed height would drive
        // straight through any part taller than it.
        const TOOL_CHANGE_CLEARANCE: f32 = 5.0;
        let safe_z = self.tool_change_position[2].max(self.current_z + TOOL_CHANGE_CLEARANCE);
        commands.push(ToolChangeCommand::MoveTo {
            position: [self.tool_change_position[0], self.tool_change_position[1], safe_z],
            feedrate: 6000.0, // 100mm/s
        });
        
        // Check if purge is needed (before mutable borrow)
        let needs_purge = if let Some(current_id) = self.active_tool {
            if let (Some(current_tool), Some(new_tool)) = (self.tools.get(&current_id), self.tools.get(&tool_id)) {
                current_tool.material.name != new_tool.material.name
            } else {
                false
            }
        } else {
            false
        };
        
        // Activate new tool
        if let Some(new_tool) = self.tools.get_mut(&tool_id) {
            new_tool.is_active = true;
            
            // Heat new tool, within what this hotend can safely do
            let temperature = new_tool.material.nozzle_temperature.min(new_tool.max_temp);
            commands.push(ToolChangeCommand::SetTemperature {
                heater_id: new_tool.heater_id,
                temperature,
            });

            // Wait for temperature
            commands.push(ToolChangeCommand::WaitForTemperature {
                heater_id: new_tool.heater_id,
                temperature,
                timeout: Duration::from_secs(300),
            });
            
            // Apply tool offset
            commands.push(ToolChangeCommand::ApplyOffset {
                offset: new_tool.offset.clone(),
            });
            
            // Prime new tool
            commands.push(ToolChangeCommand::Prime {
                distance: self.tool_change_retraction + 1.0, // Extra for priming
                speed: new_tool.material.retraction_speed / 2.0,
            });
            
            // Purge if materials are different
            if needs_purge {
                // Not below the current height either: the tower grows with the print
                let current_z = self.current_z;
                commands.push(ToolChangeCommand::PurgeMaterial {
                    volume: self.purge_volume,
                    wipe_sequence: self.wipe_sequence.iter().map(|p| [p[0], p[1], p[2].max(current_z)]).collect(),
                });
            }
        }
        
        self.active_tool = Some(tool_id);
        Ok(commands)
    }
    
    pub fn calibrate_tool_offset(&mut self, tool_id: u8, measured_offset: ToolOffset) -> Result<(), String> {
        if let Some(tool) = self.tools.get_mut(&tool_id) {
            tool.update_offset(measured_offset);
            Ok(())
        } else {
            Err(format!("Tool {} not found", tool_id))
        }
    }
    
    pub fn auto_calibrate_tool(&mut self, tool_id: u8) -> Result<Vec<CalibrationCommand>, String> {
        if !self.tools.contains_key(&tool_id) {
            return Err(format!("Tool {} not found", tool_id));
        }
        
        let mut commands = Vec::new();
        
        // Probe sequence for tool offset calibration
        commands.push(CalibrationCommand::ProbeBed {
            tool_id: self.reference_tool,
            probe_points: vec![
                [100.0, 100.0], [150.0, 100.0], [200.0, 100.0],
                [100.0, 150.0], [150.0, 150.0], [200.0, 150.0],
                [100.0, 200.0], [150.0, 200.0], [200.0, 200.0],
            ],
        });
        
        commands.push(CalibrationCommand::SwitchTool { tool_id });
        
        commands.push(CalibrationCommand::ProbeBed {
            tool_id,
            probe_points: vec![
                [100.0, 100.0], [150.0, 100.0], [200.0, 100.0],
                [100.0, 150.0], [150.0, 150.0], [200.0, 150.0],
                [100.0, 200.0], [150.0, 200.0], [200.0, 200.0],
            ],
        });
        
        commands.push(CalibrationCommand::CalculateOffset {
            reference_tool: self.reference_tool,
            target_tool: tool_id,
        });
        
        Ok(commands)
    }
    
    pub fn get_active_tool(&self) -> Option<&ToolHead> {
        if let Some(tool_id) = self.active_tool {
            self.tools.get(&tool_id)
        } else {
            None
        }
    }
    
    pub fn get_tool(&self, tool_id: u8) -> Option<&ToolHead> {
        self.tools.get(&tool_id)
    }
    
    pub fn get_tool_mut(&mut self, tool_id: u8) -> Option<&mut ToolHead> {
        self.tools.get_mut(&tool_id)
    }
    
    pub fn list_tools(&self) -> Vec<&ToolHead> {
        self.tools.values().collect()
    }
    
    pub fn set_mixing_ratio(&mut self, recipe_name: String, ratios: Vec<f32>) {
        // Normalize ratios to sum to 1.0
        let sum: f32 = ratios.iter().sum();
        if sum > 0.0 {
            let normalized: Vec<f32> = ratios.iter().map(|&r| r / sum).collect();
            self.mixing_ratios.insert(recipe_name, normalized);
        }
    }
    
    pub fn get_mixing_ratio(&self, recipe_name: &str) -> Option<&Vec<f32>> {
        self.mixing_ratios.get(recipe_name)
    }
    
    pub fn update_tool_settings(&mut self, tool_id: u8, material: MaterialSettings) -> Result<(), String> {
        if let Some(tool) = self.tools.get_mut(&tool_id) {
            tool.update_material(material);
            Ok(())
        } else {
            Err(format!("Tool {} not found", tool_id))
        }
    }
}

#[derive(Debug, Clone)]
pub enum ToolChangeCommand {
    MoveTo { position: [f32; 3], feedrate: f32 },
    Retract { distance: f32, speed: f32 },
    Prime { distance: f32, speed: f32 },
    SetTemperature { heater_id: u8, temperature: f32 },
    WaitForTemperature { heater_id: u8, temperature: f32, timeout: Duration },
    ApplyOffset { offset: ToolOffset },
    PurgeMaterial { volume: f32, wipe_sequence: Vec<[f32; 3]> },
}

#[derive(Debug, Clone)]
pub enum CalibrationCommand {
    SwitchTool { tool_id: u8 },
    ProbeBed { tool_id: u8, probe_points: Vec<[f32; 2]> },
    CalculateOffset { reference_tool: u8, target_tool: u8 },
}