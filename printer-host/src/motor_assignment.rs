// Motor Assignment System for 3D Printer Firmware
// Provides flexible mapping of motors to axes, joints, extruders, and other functions

use std::collections::HashMap;
use std::time::Instant;
use serde::{Serialize, Deserialize};

// Motor types and capabilities
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MotorType {
    Stepper,
    Servo,
    BrushlessDC,
    BrushedDC,
    LinearActuator,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MotorFunction {
    // Primary motion axes (now supporting multiple motors per axis)
    XAxis(u8),    // X axis motor ID (0, 1, 2, etc. for multi-motor)
    YAxis(u8),    // Y axis motor ID
    ZAxis(u8),    // Z axis motor ID (supporting Z1, Z2, Z3, Z4+ for bed leveling)
    AAxis(u8),    // Rotational A axis motor ID
    BAxis(u8),    // Rotational B axis motor ID
    CAxis(u8),    // Rotational C axis motor ID
    
    // Additional axes for complex kinematics
    UAxis(u8),    // U axis (linear or rotational)
    VAxis(u8),    // V axis (linear or rotational)
    WAxis(u8),    // W axis (linear or rotational)
    
    // CoreXY/CoreXZ special designations
    CoreXY_A(u8), // CoreXY A motor (X+Y)
    CoreXY_B(u8), // CoreXY B motor (X-Y)
    CoreXZ_A(u8), // CoreXZ A motor (X+Z)
    CoreXZ_B(u8), // CoreXZ B motor (X-Z)
    
    // Delta printer motors
    DeltaA(u8),   // Delta tower A motor
    DeltaB(u8),   // Delta tower B motor
    DeltaC(u8),   // Delta tower C motor
    
    // Extruders
    Extruder(u8),  // Extruder ID
    
    // Tool changers and multi-tool
    ToolChanger,
    ToolLock(u8),  // Tool lock mechanism for tool ID
    
    // Bed and chamber
    BedLeveling(u8),  // Bed leveling motor/servo ID
    BedTilting,
    ChamberDoor,
    
    // Filament handling
    FilamentFeeder(u8),
    FilamentCutter,
    FilamentSensor,
    
    // Cooling and ventilation
    PartFan(u8),
    HotendFan(u8), 
    ChamberFan,
    ExhaustFan,
    
    // Probing and measurement
    ProbeRetract,
    ProbeExtend,
    
    // Custom functions
    Custom(String),
    
    // Unused/unassigned
    Unassigned,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorCapabilities {
    pub max_speed: f32,           // Maximum speed (steps/sec or RPM)
    pub max_acceleration: f32,    // Maximum acceleration
    pub steps_per_unit: f32,      // Steps per mm or degrees
    pub max_current: f32,         // Maximum current (A)
    pub min_current: f32,         // Minimum current (A)
    pub has_encoder: bool,        // Closed-loop capability
    pub encoder_resolution: Option<u32>, // Encoder counts per revolution
    pub has_endstops: bool,       // Connected endstops
    pub can_reverse: bool,        // Bidirectional capability
    pub thermal_protection: bool, // Overtemperature shutdown
    pub stall_detection: bool,    // TMC-style stall detection
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorConfiguration {
    pub motor_id: String,         // Unique identifier (e.g., "stepper_x", "servo_1")
    pub motor_type: MotorType,
    pub function: MotorFunction,
    pub capabilities: MotorCapabilities,
    pub physical_connection: MotorConnection,
    pub enabled: bool,
    pub inverted: bool,           // Direction inversion
    pub microstepping: Option<u8>, // For steppers (1, 2, 4, 8, 16, 32, 64, 128, 256)
    pub current_setting: f32,     // Operating current (A)
    pub hold_current: f32,        // Hold current percentage (0.0-1.0)
    pub interpolation: bool,      // Step interpolation (TMC feature)
    #[serde(skip, default = "Instant::now")]
    pub created_at: Instant,
    #[serde(skip, default = "Instant::now")]
    pub last_modified: Instant,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MotorConnection {
    // Direct MCU connections
    StepDir {
        step_pin: String,
        dir_pin: String,
        enable_pin: Option<String>,
    },
    
    // SPI-connected drivers (TMC series)
    SpiDriver {
        cs_pin: String,
        spi_bus: String,
        driver_type: String, // "TMC2209", "TMC5160", etc.
    },
    
    // UART-connected drivers
    UartDriver {
        uart_port: String,
        address: u8,
        driver_type: String,
    },
    
    // CAN bus motors (like Klipper CAN toolheads)
    CanBus {
        can_id: u32,
        node_name: String,
    },
    
    // EtherCAT servo drives
    EtherCat {
        slave_position: u16,
        vendor_id: u32,
        product_code: u32,
    },
    
    // Direct PWM (for simple motors)
    PwmOutput {
        pwm_pin: String,
        dir_pin: Option<String>,
    },
}

#[derive(Debug, Clone)]
pub struct MotorState {
    pub motor_id: String,
    pub position: f64,            // Current position (steps, mm, or degrees)
    pub velocity: f64,            // Current velocity
    pub target_position: f64,     // Target position
    pub target_velocity: f64,     // Target velocity
    pub current_actual: f32,      // Actual current draw
    pub temperature: Option<f32>, // Driver temperature
    pub fault_flags: u32,         // Error/fault flags
    pub enabled: bool,
    pub homed: bool,              // Has been homed
    pub last_update: Instant,
    pub encoder_position: Option<i64>, // Raw encoder counts
    pub load: Option<f32>,        // Load percentage (0-100%)
}

#[derive(Clone)]
pub struct MotorAssignmentManager {
    motors: HashMap<String, MotorConfiguration>,
    motor_states: HashMap<String, MotorState>,
    function_assignments: HashMap<MotorFunction, Vec<String>>, // Function -> Multiple Motor IDs
    axis_groups: HashMap<String, AxisGroup>, // Axis name -> group of motors
    next_motor_id: u32,
    assignment_history: Vec<AssignmentEvent>,
}

// Multi-motor axis group for coordinated control
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AxisGroup {
    pub axis_name: String,
    pub motor_ids: Vec<String>,
    pub coordination_mode: CoordinationMode,
    pub sync_tolerance: f32,     // Maximum position difference between motors (mm)
    pub load_balancing: bool,    // Enable load balancing between motors
    pub redundancy_level: u8,    // Number of motors required to be operational
    pub master_motor: Option<String>, // Primary motor for this axis (if any)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum CoordinationMode {
    Synchronized,    // All motors move together in lockstep
    Independent,     // Motors can move independently
    LoadSharing,     // Motors share load proportionally
    Redundant,       // Motors provide redundancy (N+1 configuration)
    Differential,    // Motors move differentially (like bed leveling)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssignmentEvent {
    #[serde(skip, default = "Instant::now")]
    pub timestamp: Instant,
    pub motor_id: String,
    pub old_function: MotorFunction,
    pub new_function: MotorFunction,
    pub reason: String,
}

impl MotorAssignmentManager {
    pub fn new() -> Self {
        Self {
            motors: HashMap::new(),
            motor_states: HashMap::new(),
            function_assignments: HashMap::new(),
            axis_groups: HashMap::new(),
            next_motor_id: 0,
            assignment_history: Vec::new(),
        }
    }
    
    // Motor registration and configuration
    pub fn register_motor(&mut self, mut config: MotorConfiguration) -> Result<String, String> {
        // Validate configuration
        if config.motor_id.is_empty() {
            config.motor_id = format!("motor_{}", self.next_motor_id);
            self.next_motor_id += 1;
        }
        
        if self.motors.contains_key(&config.motor_id) {
            return Err(format!("Motor {} already registered", config.motor_id));
        }
        
        let motor_id = config.motor_id.clone();
        
        // Register motor with multi-motor support
        if config.function != MotorFunction::Unassigned {
            self.function_assignments.entry(config.function.clone())
                .or_insert_with(Vec::new)
                .push(motor_id.clone());
        }
        
        config.created_at = Instant::now();
        config.last_modified = Instant::now();
        
        let motor_id = config.motor_id.clone();
        
        // Create initial state
        let state = MotorState {
            motor_id: motor_id.clone(),
            position: 0.0,
            velocity: 0.0,
            target_position: 0.0,
            target_velocity: 0.0,
            current_actual: 0.0,
            temperature: None,
            fault_flags: 0,
            enabled: config.enabled,
            homed: false,
            last_update: Instant::now(),
            encoder_position: None,
            load: None,
        };
        
        // This section is now handled above
        
        let function = config.function.clone();
        self.motors.insert(motor_id.clone(), config);
        self.motor_states.insert(motor_id.clone(), state);

        if function != MotorFunction::Unassigned {
            self.record_assignment_event(&motor_id, MotorFunction::Unassigned, function, "Motor added");
        }

        Ok(motor_id)
    }
    
    pub fn assign_function(&mut self, motor_id: &str, function: MotorFunction, reason: &str) -> Result<(), String> {
        // Check if motor exists first
        if !self.motors.contains_key(motor_id) {
            return Err(format!("Motor {} not found", motor_id));
        }
        
        let old_function = self.motors.get(motor_id).unwrap().function.clone();
        
        // Update the target motor
        let motor = self.motors.get_mut(motor_id).unwrap();
        motor.function = function.clone();
        motor.last_modified = Instant::now();
        
        // Record the assignment change
        self.record_assignment_event(motor_id, old_function.clone(), function.clone(), reason);
        
        // Remove from old function assignment
        if old_function != MotorFunction::Unassigned {
            if let Some(motor_list) = self.function_assignments.get_mut(&old_function) {
                motor_list.retain(|id| id != motor_id);
                if motor_list.is_empty() {
                    self.function_assignments.remove(&old_function);
                }
            }
        }
        
        // Add to new function assignment
        if function != MotorFunction::Unassigned {
            self.function_assignments.entry(function)
                .or_insert_with(Vec::new)
                .push(motor_id.to_string());
        }
        
        Ok(())
    }
    
    pub fn get_motors_for_function(&self, function: &MotorFunction) -> Option<&Vec<String>> {
        self.function_assignments.get(function)
    }
    
    pub fn get_primary_motor_for_function(&self, function: &MotorFunction) -> Option<&String> {
        self.function_assignments.get(function)?.first()
    }
    
    pub fn get_motor_config(&self, motor_id: &str) -> Option<&MotorConfiguration> {
        self.motors.get(motor_id)
    }
    
    pub fn get_motor_state(&self, motor_id: &str) -> Option<&MotorState> {
        self.motor_states.get(motor_id)
    }
    
    pub fn get_all_motors(&self) -> &HashMap<String, MotorConfiguration> {
        &self.motors
    }
    
    pub fn get_motors_by_type(&self, motor_type: &MotorType) -> Vec<&MotorConfiguration> {
        self.motors.values().filter(|m| m.motor_type == *motor_type).collect()
    }
    
    pub fn get_motors_by_function(&self, function: &MotorFunction) -> Vec<&MotorConfiguration> {
        self.motors.values().filter(|m| m.function == *function).collect()
    }
    
    pub fn update_motor_state(&mut self, motor_id: &str, state_update: MotorStateUpdate) -> Result<(), String> {
        let state = self.motor_states.get_mut(motor_id).ok_or_else(|| format!("Motor {} not found", motor_id))?;
        
        if let Some(position) = state_update.position {
            state.position = position;
        }
        if let Some(velocity) = state_update.velocity {
            state.velocity = velocity;
        }
        if let Some(target_position) = state_update.target_position {
            state.target_position = target_position;
        }
        if let Some(target_velocity) = state_update.target_velocity {
            state.target_velocity = target_velocity;
        }
        if let Some(current) = state_update.current_actual {
            state.current_actual = current;
        }
        if let Some(temp) = state_update.temperature {
            state.temperature = Some(temp);
        }
        if let Some(flags) = state_update.fault_flags {
            state.fault_flags = flags;
        }
        if let Some(enabled) = state_update.enabled {
            state.enabled = enabled;
        }
        if let Some(homed) = state_update.homed {
            state.homed = homed;
        }
        if let Some(encoder_pos) = state_update.encoder_position {
            state.encoder_position = Some(encoder_pos);
        }
        if let Some(load) = state_update.load {
            state.load = Some(load);
        }
        
        state.last_update = Instant::now();
        Ok(())
    }
    
    pub fn remove_motor(&mut self, motor_id: &str) -> Result<MotorConfiguration, String> {
        let config = self.motors.remove(motor_id).ok_or_else(|| format!("Motor {} not found", motor_id))?;
        self.motor_states.remove(motor_id);
        
        // Remove just this motor from its function. Others may share it, as dual Z motors do.
        if config.function != MotorFunction::Unassigned {
            if let Some(motor_list) = self.function_assignments.get_mut(&config.function) {
                motor_list.retain(|id| id != motor_id);
                if motor_list.is_empty() {
                    self.function_assignments.remove(&config.function);
                }
            }
        }

        // And from any axis group, which must not go on commanding a motor that is gone
        for group in self.axis_groups.values_mut() {
            group.motor_ids.retain(|id| id != motor_id);
            if group.master_motor.as_deref() == Some(motor_id) {
                group.master_motor = group.motor_ids.first().cloned();
            }
        }
        // A group with no motors left has nothing to coordinate
        self.axis_groups.retain(|_, group| !group.motor_ids.is_empty());
        
        self.record_assignment_event(motor_id, config.function.clone(), MotorFunction::Unassigned, "Motor removed");
        
        Ok(config)
    }
    
    pub fn get_unassigned_motors(&self) -> Vec<&MotorConfiguration> {
        self.motors.values().filter(|m| m.function == MotorFunction::Unassigned).collect()
    }
    
    pub fn get_assignment_history(&self) -> &Vec<AssignmentEvent> {
        &self.assignment_history
    }
    
    // Axis group management for multi-motor coordination
    pub fn create_axis_group(&mut self, axis_name: String, motor_ids: Vec<String>, coordination_mode: CoordinationMode) -> Result<(), String> {
        // Validate that all motors exist
        for motor_id in &motor_ids {
            if !self.motors.contains_key(motor_id) {
                return Err(format!("Motor {} not found", motor_id));
            }
        }
        
        if motor_ids.is_empty() {
            return Err("Cannot create axis group with no motors".to_string());
        }
        
        let axis_group = AxisGroup {
            axis_name: axis_name.clone(),
            motor_ids: motor_ids.clone(),
            coordination_mode,
            sync_tolerance: 0.1, // 0.1mm default tolerance
            load_balancing: true,
            redundancy_level: 1, // At least 1 motor must be operational
            master_motor: motor_ids.first().cloned(), // First motor is master by default
        };
        
        self.axis_groups.insert(axis_name.clone(), axis_group);
        Ok(())
    }
    
    pub fn add_motor_to_axis_group(&mut self, axis_name: &str, motor_id: String) -> Result<(), String> {
        if !self.motors.contains_key(&motor_id) {
            return Err(format!("Motor {} not found", motor_id));
        }
        
        if let Some(axis_group) = self.axis_groups.get_mut(axis_name) {
            if !axis_group.motor_ids.contains(&motor_id) {
                axis_group.motor_ids.push(motor_id);
            }
            Ok(())
        } else {
            Err(format!("Axis group {} not found", axis_name))
        }
    }
    
    pub fn remove_motor_from_axis_group(&mut self, axis_name: &str, motor_id: &str) -> Result<(), String> {
        if let Some(axis_group) = self.axis_groups.get_mut(axis_name) {
            // Check first, so a refusal leaves the group as it was
            let remaining = axis_group.motor_ids.iter().filter(|id| id.as_str() != motor_id).count();
            if remaining < axis_group.redundancy_level as usize {
                return Err(format!("Removing motor {} would leave insufficient motors for axis group {}", motor_id, axis_name));
            }

            axis_group.motor_ids.retain(|id| id != motor_id);

            // Update master motor if it was removed
            if axis_group.master_motor.as_ref() == Some(&motor_id.to_string()) {
                axis_group.master_motor = axis_group.motor_ids.first().cloned();
            }

            Ok(())
        } else {
            Err(format!("Axis group {} not found", axis_name))
        }
    }
    
    pub fn get_axis_group(&self, axis_name: &str) -> Option<&AxisGroup> {
        self.axis_groups.get(axis_name)
    }
    
    pub fn get_all_axis_groups(&self) -> &HashMap<String, AxisGroup> {
        &self.axis_groups
    }
    
    pub fn set_axis_coordination_mode(&mut self, axis_name: &str, mode: CoordinationMode) -> Result<(), String> {
        if let Some(axis_group) = self.axis_groups.get_mut(axis_name) {
            axis_group.coordination_mode = mode;
            Ok(())
        } else {
            Err(format!("Axis group {} not found", axis_name))
        }
    }
    
    pub fn set_axis_sync_tolerance(&mut self, axis_name: &str, tolerance: f32) -> Result<(), String> {
        if let Some(axis_group) = self.axis_groups.get_mut(axis_name) {
            axis_group.sync_tolerance = tolerance.abs(); // Ensure positive value
            Ok(())
        } else {
            Err(format!("Axis group {} not found", axis_name))
        }
    }
    
    pub fn check_axis_synchronization(&self, axis_name: &str) -> Result<bool, String> {
        if let Some(axis_group) = self.axis_groups.get(axis_name) {
            if axis_group.motor_ids.len() < 2 {
                return Ok(true); // Single motor is always synchronized
            }
            // These modes are meant to have motors at different positions
            if matches!(axis_group.coordination_mode, CoordinationMode::Independent | CoordinationMode::Differential) {
                return Ok(true);
            }
            
            // Get positions of all motors in the group
            let mut positions = Vec::new();
            for motor_id in &axis_group.motor_ids {
                if let Some(state) = self.motor_states.get(motor_id) {
                    positions.push(state.position);
                }
            }
            
            if positions.len() < 2 {
                return Ok(true);
            }
            
            // Check if all positions are within tolerance
            let min_pos = positions.iter().cloned().fold(f64::INFINITY, f64::min);
            let max_pos = positions.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            let position_spread = max_pos - min_pos;
            
            Ok(position_spread <= axis_group.sync_tolerance as f64)
        } else {
            Err(format!("Axis group {} not found", axis_name))
        }
    }
    
    // Bulk operations for machine configuration
    pub fn create_standard_cartesian_setup(&mut self) -> Result<(), String> {
        self.all_or_nothing(Self::build_cartesian_setup)
    }

    pub fn create_corexy_setup(&mut self) -> Result<(), String> {
        self.all_or_nothing(Self::build_corexy_setup)
    }

    pub fn create_multi_servo_cartesian_setup(&mut self) -> Result<(), String> {
        self.all_or_nothing(Self::build_multi_servo_setup)
    }

    /// Apply a change only if all of it succeeds, so a failure can't leave half a machine behind.
    fn all_or_nothing(&mut self, build: fn(&mut Self) -> Result<(), String>) -> Result<(), String> {
        let mut trial = self.clone();
        build(&mut trial)?;
        *self = trial;
        Ok(())
    }

    fn build_cartesian_setup(&mut self) -> Result<(), String> {
        // Standard Cartesian printer setup with multi-motor support
        let configs = vec![
            self.create_stepper_config("x_motor", MotorFunction::XAxis(0), "X1", "X2", Some("X_EN")),
            self.create_stepper_config("y_motor", MotorFunction::YAxis(0), "Y1", "Y2", Some("Y_EN")),
            self.create_stepper_config("z_motor_left", MotorFunction::ZAxis(0), "Z1", "Z2", Some("Z_EN_L")),
            self.create_stepper_config("z_motor_right", MotorFunction::ZAxis(1), "Z3", "Z4", Some("Z_EN_R")),
            self.create_stepper_config("extruder_motor", MotorFunction::Extruder(0), "E1", "E2", Some("E_EN")),
        ];
        
        for config in configs {
            self.register_motor(config)?;
        }
        
        // Create Z-axis group for dual Z motors
        self.create_axis_group(
            "Z".to_string(), 
            vec!["z_motor_left".to_string(), "z_motor_right".to_string()], 
            CoordinationMode::Synchronized
        )?;
        
        Ok(())
    }
    
    fn build_corexy_setup(&mut self) -> Result<(), String> {
        // CoreXY printer setup
        let configs = vec![
            self.create_stepper_config("a_motor", MotorFunction::CoreXY_A(0), "X1", "X2", Some("X_EN")), // A motor (X+Y)
            self.create_stepper_config("b_motor", MotorFunction::CoreXY_B(0), "Y1", "Y2", Some("Y_EN")), // B motor (X-Y)
            self.create_stepper_config("z_motor_left", MotorFunction::ZAxis(0), "Z1", "Z2", Some("Z_EN_L")),
            self.create_stepper_config("z_motor_right", MotorFunction::ZAxis(1), "Z3", "Z4", Some("Z_EN_R")),
            self.create_stepper_config("extruder_motor", MotorFunction::Extruder(0), "E1", "E2", Some("E_EN")),
        ];
        
        for config in configs {
            self.register_motor(config)?;
        }
        
        // Create CoreXY group
        self.create_axis_group(
            "CoreXY".to_string(),
            vec!["a_motor".to_string(), "b_motor".to_string()],
            CoordinationMode::Synchronized
        )?;
        
        // Create Z-axis group for dual Z motors
        self.create_axis_group(
            "Z".to_string(), 
            vec!["z_motor_left".to_string(), "z_motor_right".to_string()], 
            CoordinationMode::Synchronized
        )?;
        
        Ok(())
    }
    
    // High-performance servo setup with multiple motors per axis
    fn build_multi_servo_setup(&mut self) -> Result<(), String> {
        let configs = vec![
            self.create_servo_config("x_servo_1", MotorFunction::XAxis(0), 1000.0),
            self.create_servo_config("x_servo_2", MotorFunction::XAxis(1), 1000.0),
            self.create_servo_config("y_servo_1", MotorFunction::YAxis(0), 1000.0),
            self.create_servo_config("y_servo_2", MotorFunction::YAxis(1), 1000.0),
            self.create_servo_config("z_servo_1", MotorFunction::ZAxis(0), 500.0),
            self.create_servo_config("z_servo_2", MotorFunction::ZAxis(1), 500.0),
            self.create_servo_config("z_servo_3", MotorFunction::ZAxis(2), 500.0),
            self.create_servo_config("z_servo_4", MotorFunction::ZAxis(3), 500.0),
        ];
        
        // Each servo drive is its own slave on the bus
        for (position, mut config) in configs.into_iter().enumerate() {
            config.physical_connection = MotorConnection::EtherCat {
                slave_position: position as u16,
                vendor_id: 0x0000,
                product_code: 0x0000,
            };
            self.register_motor(config)?;
        }

        // Create axis groups for coordinated control
        self.create_axis_group(
            "X".to_string(),
            vec!["x_servo_1".to_string(), "x_servo_2".to_string()],
            CoordinationMode::LoadSharing
        )?;
        
        self.create_axis_group(
            "Y".to_string(),
            vec!["y_servo_1".to_string(), "y_servo_2".to_string()],
            CoordinationMode::LoadSharing
        )?;
        
        self.create_axis_group(
            "Z".to_string(),
            vec!["z_servo_1".to_string(), "z_servo_2".to_string(), 
                 "z_servo_3".to_string(), "z_servo_4".to_string()],
            CoordinationMode::Differential // For bed leveling
        )?;
        
        Ok(())
    }
    
    /// Remove every motor and axis group. The history is kept.
    pub fn clear_all_motors(&mut self) -> usize {
        let ids: Vec<String> = self.motors.keys().cloned().collect();
        let count = ids.len();
        for id in ids {
            let _ = self.remove_motor(&id);
        }
        count
    }

    /// Replace everything with a standard setup. If it fails, the current motors stay.
    pub fn replace_with_setup(&mut self, build: fn(&mut Self) -> Result<(), String>) -> Result<(), String> {
        let mut trial = self.clone();
        trial.clear_all_motors();
        build(&mut trial)?;
        *self = trial;
        Ok(())
    }

    /// Record whether a motor should be enabled. This changes the machine's configuration: no
    /// driver is told, since nothing connects this manager to hardware yet.
    pub fn set_motor_enabled(&mut self, motor_id: &str, enabled: bool) -> Result<(), String> {
        let config = self.motors.get_mut(motor_id).ok_or_else(|| format!("Motor {} not found", motor_id))?;
        config.enabled = enabled;
        config.last_modified = Instant::now();
        if let Some(state) = self.motor_states.get_mut(motor_id) {
            state.enabled = enabled;
            state.last_update = Instant::now();
        }
        Ok(())
    }

    pub fn set_all_motors_enabled(&mut self, enabled: bool) -> usize {
        let ids: Vec<String> = self.motors.keys().cloned().collect();
        for id in &ids {
            let _ = self.set_motor_enabled(id, enabled);
        }
        ids.len()
    }

    /// Save edited settings for a motor. Its ID and function change through their own methods,
    /// which keep the assignment tables in step.
    pub fn update_motor_settings(&mut self, edited: MotorConfiguration) -> Result<(), String> {
        if !(edited.capabilities.steps_per_unit > 0.0) {
            return Err("Steps per unit must be above zero".to_string());
        }
        if !(edited.current_setting > 0.0) || edited.current_setting > edited.capabilities.max_current {
            return Err(format!("Current must be above 0 and no more than {:.2} A", edited.capabilities.max_current));
        }
        if !(0.0..=1.0).contains(&edited.hold_current) {
            return Err("Hold current must be between 0% and 100%".to_string());
        }
        let config = self.motors.get_mut(&edited.motor_id).ok_or_else(|| format!("Motor {} not found", edited.motor_id))?;
        let function = config.function.clone();
        let enabled = config.enabled;
        let created_at = config.created_at;
        *config = edited;
        config.function = function;
        config.enabled = enabled;
        config.created_at = created_at;
        config.last_modified = Instant::now();
        Ok(())
    }

    /// Motor IDs in a fixed order, so lists on screen don't change order between launches.
    pub fn sorted_motor_ids(&self) -> Vec<String> {
        let mut ids: Vec<String> = self.motors.keys().cloned().collect();
        ids.sort();
        ids
    }

    /// Motors doing this kind of job, whichever number they have: ZAxis(0) and ZAxis(1) are both Z.
    pub fn motors_in_family(&self, template: &MotorFunction) -> Vec<&MotorConfiguration> {
        let mut found: Vec<&MotorConfiguration> = self.motors.values()
            .filter(|m| std::mem::discriminant(&m.function) == std::mem::discriminant(template))
            .collect();
        found.sort_by_key(|m| (m.function.index(), m.motor_id.clone()));
        found
    }

    /// The first free slot of this kind: the second motor given to Z becomes ZAxis(1).
    pub fn next_free_function(&self, template: &MotorFunction) -> MotorFunction {
        let used: Vec<Option<u8>> = self.motors_in_family(template).iter().map(|m| m.function.index()).collect();
        (0..=u8::MAX)
            .find_map(|i| {
                let candidate = template.with_index(i)?;
                (!used.contains(&candidate.index())).then_some(candidate)
            })
            .unwrap_or_else(|| template.clone())
    }

    /// Give a motor a kind of job, using the next free slot. Does nothing if it already has that job.
    pub fn assign_to_family(&mut self, motor_id: &str, template: &MotorFunction, reason: &str) -> Result<(), String> {
        let current = &self.motors.get(motor_id).ok_or_else(|| format!("Motor {} not found", motor_id))?.function;
        if std::mem::discriminant(current) == std::mem::discriminant(template) {
            return Ok(());
        }
        let function = self.next_free_function(template);
        self.assign_function(motor_id, function, reason)
    }

    /// A copy of a motor under a new name, with no job and its own pins left blank to fill in.
    pub fn clone_motor(&mut self, motor_id: &str) -> Result<String, String> {
        let mut copy = self.motors.get(motor_id).ok_or_else(|| format!("Motor {} not found", motor_id))?.clone();
        let base = format!("{}_clone", motor_id);
        let mut new_id = base.clone();
        let mut n = 2;
        while self.motors.contains_key(&new_id) {
            new_id = format!("{}{}", base, n);
            n += 1;
        }
        copy.motor_id = new_id;
        copy.function = MotorFunction::Unassigned;
        self.register_motor(copy)
    }

    /// Which motor already uses this pin, if any.
    pub fn pin_owner(&self, pin: &str) -> Option<&str> {
        let mut ids: Vec<&String> = self.motors.keys().collect();
        ids.sort();
        ids.into_iter()
            .find(|id| match &self.motors[*id].physical_connection {
                MotorConnection::StepDir { step_pin, dir_pin, enable_pin } => {
                    step_pin == pin || dir_pin == pin || enable_pin.as_deref() == Some(pin)
                }
                MotorConnection::PwmOutput { pwm_pin, dir_pin } => pwm_pin == pin || dir_pin.as_deref() == Some(pin),
                MotorConnection::SpiDriver { cs_pin, .. } => cs_pin == pin,
                _ => false,
            })
            .map(|id| id.as_str())
    }

    /// A motor ID that isn't taken yet.
    pub fn suggest_motor_id(&self) -> String {
        (1..).map(|n| format!("motor_{}", n)).find(|id| !self.motors.contains_key(id)).unwrap()
    }

    pub fn to_json(&self) -> Result<String, String> {
        let mut motors: Vec<MotorConfiguration> = self.motors.values().cloned().collect();
        motors.sort_by(|a, b| a.motor_id.cmp(&b.motor_id));
        let mut axis_groups: Vec<AxisGroup> = self.axis_groups.values().cloned().collect();
        axis_groups.sort_by(|a, b| a.axis_name.cmp(&b.axis_name));
        serde_json::to_string_pretty(&SavedMotors { motors, axis_groups }).map_err(|e| e.to_string())
    }

    pub fn from_json(json: &str) -> Result<Self, String> {
        let saved: SavedMotors = serde_json::from_str(json).map_err(|e| e.to_string())?;
        let mut manager = Self::new();
        for motor in saved.motors {
            manager.register_motor(motor)?;
        }
        for mut group in saved.axis_groups {
            group.motor_ids.retain(|id| manager.motors.contains_key(id));
            if !group.motor_ids.is_empty() {
                manager.axis_groups.insert(group.axis_name.clone(), group);
            }
        }
        // Reading the file isn't a change anyone made
        manager.assignment_history.clear();
        Ok(manager)
    }

    /// Write the file in one step, so a crash part way can't leave it damaged.
    pub fn save_to_file(&self, path: &std::path::Path) -> Result<(), String> {
        let json = self.to_json()?;
        let temp = path.with_extension("json.tmp");
        std::fs::write(&temp, json).map_err(|e| e.to_string())?;
        std::fs::rename(&temp, path).map_err(|e| e.to_string())
    }

    /// The saved motors. None if nothing has been saved yet.
    pub fn load_from_file(path: &std::path::Path) -> Result<Option<Self>, String> {
        match std::fs::read_to_string(path) {
            Ok(json) => Self::from_json(&json).map(Some),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
            Err(e) => Err(e.to_string()),
        }
    }

    fn create_stepper_config(&self, id: &str, function: MotorFunction, step_pin: &str, dir_pin: &str, enable_pin: Option<&str>) -> MotorConfiguration {
        // Typical values: belts for X/Y, a leadscrew for Z (5x as many steps per mm), and a
        // geared extruder. One number for all of them would move Z and E the wrong distance.
        let steps_per_unit = match function {
            MotorFunction::ZAxis(_) => 400.0,
            MotorFunction::Extruder(_) => 93.0,
            _ => 80.0,
        };
        MotorConfiguration {
            motor_id: id.to_string(),
            motor_type: MotorType::Stepper,
            function,
            capabilities: MotorCapabilities {
                max_speed: 1000.0,
                max_acceleration: 3000.0,
                steps_per_unit,
                max_current: 1.5,
                min_current: 0.1,
                has_encoder: false,
                encoder_resolution: None,
                has_endstops: true,
                can_reverse: true,
                thermal_protection: true,
                stall_detection: false,
            },
            physical_connection: MotorConnection::StepDir {
                step_pin: step_pin.to_string(),
                dir_pin: dir_pin.to_string(),
                enable_pin: enable_pin.map(|s| s.to_string()),
            },
            enabled: true,
            inverted: false,
            microstepping: Some(16),
            current_setting: 1.0,
            hold_current: 0.5,
            interpolation: true,
            created_at: Instant::now(),
            last_modified: Instant::now(),
        }
    }
    
    fn create_servo_config(&self, id: &str, function: MotorFunction, encoder_resolution: f32) -> MotorConfiguration {
        MotorConfiguration {
            motor_id: id.to_string(),
            motor_type: MotorType::Servo,
            function,
            capabilities: MotorCapabilities {
                max_speed: 5000.0,            // High speed capability for servos
                max_acceleration: 10000.0,    // High acceleration for servos
                steps_per_unit: encoder_resolution,
                max_current: 3.0,
                min_current: 0.1,
                has_encoder: true,           // Servos have encoders
                encoder_resolution: Some(encoder_resolution as u32),
                has_endstops: false,         // Servos don't typically need endstops
                can_reverse: true,
                thermal_protection: true,
                stall_detection: true,       // Advanced servo drives detect stalls
            },
            physical_connection: MotorConnection::EtherCat {
                slave_position: 0,
                vendor_id: 0x0000,
                product_code: 0x0000,
            },
            enabled: true,
            inverted: false,
            microstepping: None,             // Not applicable for servos
            current_setting: 2.0,
            hold_current: 0.0,              // Servos don't need hold current
            interpolation: false,           // Servo drives handle this internally
            created_at: Instant::now(),
            last_modified: Instant::now(),
        }
    }
    
    fn record_assignment_event(&mut self, motor_id: &str, old_function: MotorFunction, new_function: MotorFunction, reason: &str) {
        let event = AssignmentEvent {
            timestamp: Instant::now(),
            motor_id: motor_id.to_string(),
            old_function,
            new_function,
            reason: reason.to_string(),
        };
        
        self.assignment_history.push(event);
        
        // Keep history size reasonable
        if self.assignment_history.len() > 1000 {
            self.assignment_history.remove(0);
        }
    }
}

#[derive(Serialize, Deserialize)]
struct SavedMotors {
    motors: Vec<MotorConfiguration>,
    axis_groups: Vec<AxisGroup>,
}

impl MotorFunction {
    /// Which of several motors on the same job this is. None for jobs with no number.
    pub fn index(&self) -> Option<u8> {
        use MotorFunction::*;
        match self {
            XAxis(i) | YAxis(i) | ZAxis(i) | AAxis(i) | BAxis(i) | CAxis(i) | UAxis(i) | VAxis(i) | WAxis(i)
            | CoreXY_A(i) | CoreXY_B(i) | CoreXZ_A(i) | CoreXZ_B(i) | DeltaA(i) | DeltaB(i) | DeltaC(i)
            | Extruder(i) | ToolLock(i) | BedLeveling(i) | FilamentFeeder(i) | PartFan(i) | HotendFan(i) => Some(*i),
            _ => None,
        }
    }

    /// This job with another number. For jobs with no number, the job itself once.
    fn with_index(&self, n: u8) -> Option<MotorFunction> {
        use MotorFunction::*;
        Some(match self {
            XAxis(_) => XAxis(n),
            YAxis(_) => YAxis(n),
            ZAxis(_) => ZAxis(n),
            AAxis(_) => AAxis(n),
            BAxis(_) => BAxis(n),
            CAxis(_) => CAxis(n),
            UAxis(_) => UAxis(n),
            VAxis(_) => VAxis(n),
            WAxis(_) => WAxis(n),
            CoreXY_A(_) => CoreXY_A(n),
            CoreXY_B(_) => CoreXY_B(n),
            CoreXZ_A(_) => CoreXZ_A(n),
            CoreXZ_B(_) => CoreXZ_B(n),
            DeltaA(_) => DeltaA(n),
            DeltaB(_) => DeltaB(n),
            DeltaC(_) => DeltaC(n),
            Extruder(_) => Extruder(n),
            ToolLock(_) => ToolLock(n),
            BedLeveling(_) => BedLeveling(n),
            FilamentFeeder(_) => FilamentFeeder(n),
            PartFan(_) => PartFan(n),
            HotendFan(_) => HotendFan(n),
            other => {
                if n > 0 {
                    return None;
                }
                other.clone()
            }
        })
    }

    /// The jobs offered in menus, with their names.
    pub fn assignable() -> Vec<(&'static str, MotorFunction)> {
        use MotorFunction::*;
        vec![
            ("X Axis", XAxis(0)),
            ("Y Axis", YAxis(0)),
            ("Z Axis", ZAxis(0)),
            ("A Axis", AAxis(0)),
            ("B Axis", BAxis(0)),
            ("C Axis", CAxis(0)),
            ("CoreXY A", CoreXY_A(0)),
            ("CoreXY B", CoreXY_B(0)),
            ("Delta A", DeltaA(0)),
            ("Delta B", DeltaB(0)),
            ("Delta C", DeltaC(0)),
            ("Extruder", Extruder(0)),
            ("Tool Changer", ToolChanger),
            ("Bed Leveling", BedLeveling(0)),
        ]
    }

    /// A readable name: the second Z motor is "Z Axis 2".
    pub fn display_name(&self) -> String {
        if *self == MotorFunction::Unassigned {
            return "Unassigned".to_string();
        }
        let name = Self::assignable()
            .into_iter()
            .find(|(_, f)| std::mem::discriminant(f) == std::mem::discriminant(self))
            .map(|(name, _)| name.to_string())
            .unwrap_or_else(|| format!("{:?}", self));
        match self.index() {
            Some(i) if i > 0 => format!("{} {}", name, i + 1),
            _ => name,
        }
    }
}

// Helper struct for state updates
#[derive(Debug, Default)]
pub struct MotorStateUpdate {
    pub position: Option<f64>,
    pub velocity: Option<f64>,
    pub target_position: Option<f64>,
    pub target_velocity: Option<f64>,
    pub current_actual: Option<f32>,
    pub temperature: Option<f32>,
    pub fault_flags: Option<u32>,
    pub enabled: Option<bool>,
    pub homed: Option<bool>,
    pub encoder_position: Option<i64>,
    pub load: Option<f32>,
}

// Trait for motor driver implementations
pub trait MotorDriver: Send + Sync {
    fn initialize(&mut self, config: &MotorConfiguration) -> Result<(), String>;
    fn enable(&mut self) -> Result<(), String>;
    fn disable(&mut self) -> Result<(), String>;
    fn move_to_position(&mut self, position: f64, velocity: f64) -> Result<(), String>;
    fn move_at_velocity(&mut self, velocity: f64) -> Result<(), String>;
    fn stop(&mut self) -> Result<(), String>;
    fn home(&mut self) -> Result<(), String>;
    fn get_position(&self) -> f64;
    fn get_velocity(&self) -> f64;
    fn is_moving(&self) -> bool;
    fn has_fault(&self) -> bool;
    fn get_fault_flags(&self) -> u32;
    fn clear_faults(&mut self) -> Result<(), String>;
    fn set_current(&mut self, current: f32) -> Result<(), String>;
    fn get_temperature(&self) -> Option<f32>;
}

impl Default for MotorAssignmentManager {
    fn default() -> Self {
        Self::new()
    }
}