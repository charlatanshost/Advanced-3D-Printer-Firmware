// LinuxCNC-Inspired Continuous Control System
// This module implements continuous analog-like control for servo motors to eliminate VFAs
// Based on LinuxCNC's approach of using continuous signals instead of discrete step/dir

use std::collections::HashMap;
use std::time::{Duration, Instant};
use std::sync::{Arc, Mutex};
use serde::{Serialize, Deserialize};
use crate::motor_assignment::MotorAssignmentManager;
use crate::closed_loop_control::{ClosedLoopManager, PIDController};

// Continuous control output types (similar to LinuxCNC HAL)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ControlOutputType {
    AnalogVelocity,    // Velocity command via DAC/PWM (V/Hz)
    AnalogPosition,    // Position command via DAC
    DigitalVelocity,   // Digital velocity command over bus
    DigitalPosition,   // Digital position command over bus
    TorqueCommand,     // Torque/current command
}

// Advanced control parameters inspired by LinuxCNC
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AxisControlParameters {
    pub max_velocity: f64,          // mm/s or deg/s
    pub max_acceleration: f64,      // mm/s² or deg/s²
    pub max_jerk: f64,             // mm/s³ or deg/s³ (jerk limiting)
    pub following_error_limit: f64, // Maximum following error before fault
    pub ferror_limit: f64,         // Following error limit during motion
    pub min_ferror: f64,           // Minimum following error limit
    pub deadband: f64,             // Position deadband (ignore small errors)
    pub bias: f64,                 // Output bias/offset
    pub scale: f64,                // Position scaling factor
    pub output_scale: f64,         // Output command scaling
    pub output_offset: f64,        // Output command offset
    pub backlash_compensation: f64, // Backlash compensation amount
    pub screw_compensation: bool,   // Enable leadscrew error compensation
}

impl Default for AxisControlParameters {
    fn default() -> Self {
        Self {
            max_velocity: 300.0,
            max_acceleration: 3000.0,
            max_jerk: 30000.0,
            following_error_limit: 1.0,
            ferror_limit: 0.5,
            min_ferror: 0.01,
            deadband: 0.001,
            bias: 0.0,
            scale: 1.0,
            output_scale: 1.0,
            output_offset: 0.0,
            backlash_compensation: 0.0,
            screw_compensation: false,
        }
    }
}

// Trajectory interpolation state (inspired by LinuxCNC motion controller)
#[derive(Debug, Clone)]
pub struct TrajectoryState {
    pub commanded_position: f64,    // Commanded position
    pub commanded_velocity: f64,    // Commanded velocity  
    pub commanded_acceleration: f64, // Commanded acceleration
    pub actual_position: f64,       // Actual position from feedback
    pub actual_velocity: f64,       // Actual velocity from feedback
    pub position_error: f64,        // Following error
    pub velocity_error: f64,        // Velocity error
    pub output_command: f64,        // Output to motor drive
    pub in_position: bool,          // Within deadband
    pub enabled: bool,              // Axis enabled
    pub fault: bool,               // Axis fault state
    pub homed: bool,               // Axis homed
}

impl Default for TrajectoryState {
    fn default() -> Self {
        Self {
            commanded_position: 0.0,
            commanded_velocity: 0.0,
            commanded_acceleration: 0.0,
            actual_position: 0.0,
            actual_velocity: 0.0,
            position_error: 0.0,
            velocity_error: 0.0,
            output_command: 0.0,
            in_position: true,
            enabled: false,
            fault: false,
            homed: false,
        }
    }
}

// S-curve trajectory generator (like LinuxCNC's trajectory planner)
pub struct SCurveTrajectoryGenerator {
    current_position: f64,
    current_velocity: f64,
    current_acceleration: f64,
    target_position: f64,
    target_velocity: f64,
    max_velocity: f64,
    max_acceleration: f64,
    max_jerk: f64,
    trajectory_time: Duration,
    start_time: Instant,
    profile_complete: bool,
}

impl SCurveTrajectoryGenerator {
    pub fn new(max_vel: f64, max_accel: f64, max_jerk: f64) -> Self {
        Self {
            current_position: 0.0,
            current_velocity: 0.0,
            current_acceleration: 0.0,
            target_position: 0.0,
            target_velocity: 0.0,
            max_velocity: max_vel,
            max_acceleration: max_accel,
            max_jerk: max_jerk,
            trajectory_time: Duration::ZERO,
            start_time: Instant::now(),
            profile_complete: true,
        }
    }
    
    pub fn plan_move(&mut self, start_pos: f64, start_vel: f64, end_pos: f64, end_vel: f64) {
        self.current_position = start_pos;
        self.current_velocity = start_vel;
        self.current_acceleration = 0.0;
        self.target_position = end_pos;
        self.target_velocity = end_vel;
        self.start_time = Instant::now();
        self.profile_complete = false;
        
        // Calculate trajectory time using S-curve profile
        let distance = (end_pos - start_pos).abs();
        let velocity_change = (end_vel - start_vel).abs();
        
        // Simplified S-curve calculation - in practice this would be more complex
        let accel_time = self.max_velocity / self.max_acceleration;
        let jerk_time = self.max_acceleration / self.max_jerk;
        
        let total_time = if distance / self.max_velocity > accel_time {
            // Long move with cruising phase
            distance / self.max_velocity + accel_time + jerk_time
        } else {
            // Short move, triangular or S-curve profile
            2.0 * (distance / self.max_acceleration).sqrt() + jerk_time
        };
        
        self.trajectory_time = Duration::from_secs_f64(total_time);
    }
    
    pub fn update(&mut self) -> (f64, f64, f64) {
        if self.profile_complete {
            return (self.target_position, self.target_velocity, 0.0);
        }
        
        let elapsed = self.start_time.elapsed();
        let t = elapsed.as_secs_f64();
        let total_time = self.trajectory_time.as_secs_f64();
        
        if elapsed >= self.trajectory_time {
            self.profile_complete = true;
            self.current_position = self.target_position;
            self.current_velocity = self.target_velocity;
            self.current_acceleration = 0.0;
            return (self.current_position, self.current_velocity, self.current_acceleration);
        }
        
        // S-curve trajectory calculation
        let distance = self.target_position - self.current_position;
        let progress = t / total_time;
        
        // Smooth S-curve using cubic interpolation (simplified)
        let s = if progress < 0.5 {
            2.0 * progress * progress
        } else {
            1.0 - 2.0 * (1.0 - progress) * (1.0 - progress)
        };
        
        self.current_position = self.current_position + distance * s;
        
        // Calculate velocity and acceleration from position derivative
        let dt = 0.001; // 1ms time step
        let s_dt = if (progress + dt / total_time) < 0.5 {
            2.0 * (progress + dt / total_time) * (progress + dt / total_time)
        } else {
            1.0 - 2.0 * (1.0 - (progress + dt / total_time)) * (1.0 - (progress + dt / total_time))
        };
        
        self.current_velocity = distance * (s_dt - s) / dt;
        
        (self.current_position, self.current_velocity, self.current_acceleration)
    }
    
    pub fn is_complete(&self) -> bool {
        self.profile_complete
    }
}

// Main continuous control system
pub struct LinuxCNCController {
    axes: HashMap<String, AxisControl>,
    motor_manager: Arc<Mutex<MotorAssignmentManager>>,
    closed_loop_manager: Arc<Mutex<ClosedLoopManager>>,
    update_rate: f64,           // Control loop frequency (Hz)
    last_update: Instant,
    enabled: bool,
    emergency_stop: bool,
}

pub struct AxisControl {
    pub axis_name: String,
    pub parameters: AxisControlParameters,
    pub trajectory_generator: SCurveTrajectoryGenerator,
    pub pid_controller: PIDController,
    pub state: TrajectoryState,
    pub output_type: ControlOutputType,
    pub motor_ids: Vec<String>,
    pub backlash_direction: i8,  // -1, 0, 1 for backlash compensation
    pub last_command_direction: i8,
}

impl LinuxCNCController {
    pub fn new(motor_manager: Arc<Mutex<MotorAssignmentManager>>, 
               closed_loop_manager: Arc<Mutex<ClosedLoopManager>>, 
               update_rate: f64) -> Self {
        Self {
            axes: HashMap::new(),
            motor_manager,
            closed_loop_manager,
            update_rate,
            last_update: Instant::now(),
            enabled: false,
            emergency_stop: false,
        }
    }
    
    pub fn add_axis(&mut self, axis_name: String, parameters: AxisControlParameters, 
                    output_type: ControlOutputType, motor_ids: Vec<String>) -> Result<(), String> {
        
        // Validate motor IDs exist
        let manager = self.motor_manager.lock().unwrap();
        for motor_id in &motor_ids {
            if manager.get_motor_config(motor_id).is_none() {
                return Err(format!("Motor {} not found", motor_id));
            }
        }
        drop(manager);
        
        let trajectory_generator = SCurveTrajectoryGenerator::new(
            parameters.max_velocity,
            parameters.max_acceleration, 
            parameters.max_jerk
        );
        
        let pid_controller = PIDController::new(10.0, 0.1, 0.01); // Default PID gains
        
        let axis_control = AxisControl {
            axis_name: axis_name.clone(),
            parameters,
            trajectory_generator,
            pid_controller,
            state: TrajectoryState::default(),
            output_type,
            motor_ids,
            backlash_direction: 0,
            last_command_direction: 0,
        };
        
        self.axes.insert(axis_name, axis_control);
        Ok(())
    }
    
    pub fn command_position(&mut self, axis_name: &str, position: f64, velocity: f64) -> Result<(), String> {
        if let Some(axis) = self.axes.get_mut(axis_name) {
            axis.trajectory_generator.plan_move(
                axis.state.commanded_position,
                axis.state.commanded_velocity,
                position,
                velocity
            );
            Ok(())
        } else {
            Err(format!("Axis {} not found", axis_name))
        }
    }
    
    pub fn update_control_loop(&mut self) -> Result<HashMap<String, f64>, String> {
        let now = Instant::now();
        let dt = now.duration_since(self.last_update).as_secs_f64();
        let expected_dt = 1.0 / self.update_rate;
        
        if dt < expected_dt * 0.9 {
            return Ok(HashMap::new()); // Not time for update yet
        }
        
        self.last_update = now;
        let mut commands = HashMap::new();
        
        if !self.enabled || self.emergency_stop {
            return Ok(commands);
        }
        
        // Update each axis
        for (axis_name, axis) in self.axes.iter_mut() {
            // Update trajectory generation
            let (cmd_pos, cmd_vel, cmd_acc) = axis.trajectory_generator.update();
            axis.state.commanded_position = cmd_pos;
            axis.state.commanded_velocity = cmd_vel;
            axis.state.commanded_acceleration = cmd_acc;
            
            // Get actual position from encoders
            if let Ok(closed_loop) = self.closed_loop_manager.lock() {
                if let Some(diagnostics) = closed_loop.get_axis_diagnostics(axis_name) {
                    axis.state.actual_position = diagnostics.current_position;
                    axis.state.actual_velocity = diagnostics.velocity;
                }
            }
            
            // Calculate position error
            axis.state.position_error = axis.state.commanded_position - axis.state.actual_position;
            axis.state.velocity_error = axis.state.commanded_velocity - axis.state.actual_velocity;
            
            // Check deadband
            axis.state.in_position = axis.state.position_error.abs() <= axis.parameters.deadband;
            
            // Check following error limits
            if axis.state.position_error.abs() > axis.parameters.following_error_limit {
                axis.state.fault = true;
                return Err(format!("Following error limit exceeded on axis {}", axis_name));
            }
            
            // Backlash compensation
            let mut compensated_position = axis.state.commanded_position;
            let command_direction = if axis.state.commanded_velocity > 0.0 { 1 } 
                                  else if axis.state.commanded_velocity < 0.0 { -1 } 
                                  else { axis.last_command_direction };
            
            if command_direction != axis.backlash_direction && axis.parameters.backlash_compensation != 0.0 {
                compensated_position += axis.parameters.backlash_compensation * command_direction as f64;
                axis.backlash_direction = command_direction;
            }
            axis.last_command_direction = command_direction;
            
            // PID control calculation with feedforward
            let pid_output = axis.pid_controller.update(compensated_position, axis.state.actual_position);
            let feedforward = axis.state.commanded_velocity * 0.1; // Simple velocity feedforward
            
            // Calculate final output command
            axis.state.output_command = (pid_output + feedforward + axis.parameters.bias) 
                                      * axis.parameters.output_scale + axis.parameters.output_offset;
            
            // Apply output limits
            axis.state.output_command = axis.state.output_command.clamp(-100.0, 100.0);
            
            // Send commands to all motors for this axis
            for motor_id in &axis.motor_ids {
                commands.insert(format!("{}_{}", axis_name, motor_id), axis.state.output_command);
            }
        }
        
        Ok(commands)
    }
    
    pub fn enable_axis(&mut self, axis_name: &str) -> Result<(), String> {
        if let Some(axis) = self.axes.get_mut(axis_name) {
            axis.state.enabled = true;
            axis.state.fault = false;
            axis.pid_controller.reset();
            Ok(())
        } else {
            Err(format!("Axis {} not found", axis_name))
        }
    }
    
    pub fn disable_axis(&mut self, axis_name: &str) -> Result<(), String> {
        if let Some(axis) = self.axes.get_mut(axis_name) {
            axis.state.enabled = false;
            axis.state.output_command = 0.0;
            Ok(())
        } else {
            Err(format!("Axis {} not found", axis_name))
        }
    }
    
    pub fn home_axis(&mut self, axis_name: &str) -> Result<(), String> {
        if let Some(axis) = self.axes.get_mut(axis_name) {
            // Reset position to zero (home position)
            axis.state.commanded_position = 0.0;
            axis.state.actual_position = 0.0;
            axis.state.position_error = 0.0;
            axis.state.homed = true;
            axis.pid_controller.reset();
            Ok(())
        } else {
            Err(format!("Axis {} not found", axis_name))
        }
    }
    
    pub fn set_pid_gains(&mut self, axis_name: &str, kp: f64, ki: f64, kd: f64) -> Result<(), String> {
        if let Some(axis) = self.axes.get_mut(axis_name) {
            axis.pid_controller.set_gains(kp, ki, kd);
            Ok(())
        } else {
            Err(format!("Axis {} not found", axis_name))
        }
    }
    
    pub fn enable_global(&mut self) {
        self.enabled = true;
        self.emergency_stop = false;
    }
    
    pub fn disable_global(&mut self) {
        self.enabled = false;
        // Set all axis outputs to zero
        for axis in self.axes.values_mut() {
            axis.state.output_command = 0.0;
            axis.state.enabled = false;
        }
    }
    
    pub fn emergency_stop(&mut self) {
        self.emergency_stop = true;
        self.enabled = false;
        // Immediately stop all axes
        for axis in self.axes.values_mut() {
            axis.state.output_command = 0.0;
            axis.state.enabled = false;
            axis.state.commanded_velocity = 0.0;
            axis.state.commanded_acceleration = 0.0;
        }
    }
    
    pub fn get_axis_state(&self, axis_name: &str) -> Option<&TrajectoryState> {
        self.axes.get(axis_name).map(|axis| &axis.state)
    }
    
    pub fn get_all_axis_states(&self) -> HashMap<String, &TrajectoryState> {
        self.axes.iter().map(|(name, axis)| (name.clone(), &axis.state)).collect()
    }
    
    // Multi-axis coordinated moves (like LinuxCNC coordinated motion)
    pub fn coordinate_linear_move(&mut self, target_positions: HashMap<String, f64>, 
                                 feedrate: f64) -> Result<(), String> {
        // Calculate the move time based on the longest axis move
        let mut max_distance: f64 = 0.0;
        let mut axis_distances = HashMap::new();
        
        for (axis_name, target_pos) in &target_positions {
            if let Some(axis) = self.axes.get(axis_name) {
                let distance = (target_pos - axis.state.commanded_position).abs();
                axis_distances.insert(axis_name.clone(), distance);
                max_distance = max_distance.max(distance);
            }
        }
        
        if max_distance == 0.0 {
            return Ok(()); // No movement needed
        }
        
        let move_time = max_distance / feedrate;
        
        // Plan synchronized moves for all axes
        for (axis_name, target_pos) in target_positions {
            if let Some(axis) = self.axes.get_mut(&axis_name) {
                let distance = axis_distances[&axis_name];
                let axis_velocity = if move_time > 0.0 { distance / move_time } else { 0.0 };
                
                axis.trajectory_generator.plan_move(
                    axis.state.commanded_position,
                    0.0, // Start from rest
                    target_pos,
                    axis_velocity.min(axis.parameters.max_velocity)
                );
            }
        }
        
        Ok(())
    }
}

// Integration with existing systems
impl LinuxCNCController {
    pub fn sync_with_motor_assignments(&mut self) -> Result<(), String> {
        // Collect data first to avoid borrowing conflicts
        let axis_data = {
            let manager = self.motor_manager.lock().unwrap();
            let mut data = Vec::new();
            
            for (group_name, axis_group) in manager.get_all_axis_groups() {
                if !self.axes.contains_key(group_name) {
                    let params = AxisControlParameters::default();
                    let output_type = match axis_group.motor_ids.first() {
                        Some(motor_id) => {
                            if let Some(config) = manager.get_motor_config(motor_id) {
                                if config.capabilities.has_encoder {
                                    ControlOutputType::AnalogVelocity
                                } else {
                                    ControlOutputType::DigitalPosition
                                }
                            } else {
                                ControlOutputType::DigitalPosition
                            }
                        }
                        None => ControlOutputType::DigitalPosition,
                    };
                    
                    data.push((group_name.clone(), params, output_type, axis_group.motor_ids.clone()));
                }
            }
            data
        }; // Manager lock is dropped here
        
        // Now add axes without holding the lock
        for (group_name, params, output_type, motor_ids) in axis_data {
            self.add_axis(group_name, params, output_type, motor_ids)?;
        }
        
        Ok(())
    }
}