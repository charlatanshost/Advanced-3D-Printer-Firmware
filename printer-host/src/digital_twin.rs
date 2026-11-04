use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use std::thread;
use std::sync::mpsc::{self, Receiver, Sender};

#[derive(Debug, Clone)]
pub struct PhysicsParams {
    pub mass: f64,                    // kg
    pub friction_coefficient: f64,    // Static friction
    pub inertia: f64,                // kg⋅m²
    pub stiffness: f64,              // N/m
    pub damping: f64,                // N⋅s/m
    pub gravity: f64,                // m/s²
}

#[derive(Debug, Clone)]
pub struct ThermalParams {
    pub thermal_mass: f64,           // J/K
    pub thermal_conductivity: f64,   // W/(m⋅K)
    pub heat_capacity: f64,          // J/(kg⋅K)
    pub ambient_temp: f64,           // °C
    pub surface_area: f64,           // m²
    pub emissivity: f64,             // Stefan-Boltzmann
}

#[derive(Debug, Clone)]
pub struct AxisState {
    pub position: f64,               // mm
    pub velocity: f64,               // mm/s
    pub acceleration: f64,           // mm/s²
    pub force: f64,                  // N
    pub temperature: f64,            // °C
    pub encoder_position: i64,       // encoder counts
    pub motor_current: f64,          // A
    pub backlash: f64,              // mm
}

#[derive(Debug, Clone)]
pub struct ExtruderState {
    pub position: f64,               // mm of filament
    pub velocity: f64,               // mm/s
    pub pressure: f64,               // Pa
    pub temperature: f64,            // °C
    pub flow_rate: f64,              // mm³/s
    pub back_pressure: f64,          // Pa
}

#[derive(Debug, Clone)]
pub struct HotendState {
    pub temperature: f64,            // °C
    pub target_temperature: f64,     // °C
    pub heater_power: f64,           // W
    pub thermistor_reading: f64,     // ADC counts
    pub thermal_mass: f64,           // J/K
    pub heat_loss_coefficient: f64,  // W/K
}

#[derive(Debug, Clone)]
pub struct PrinterState {
    pub axes: HashMap<String, AxisState>,
    pub extruders: Vec<ExtruderState>,
    pub hotends: Vec<HotendState>,
    pub bed_temperature: f64,        // °C
    pub chamber_temperature: f64,    // °C
    pub print_time: Duration,
    pub layer_height: f64,           // mm
    pub print_speed: f64,            // mm/s
    pub fan_speeds: Vec<f64>,        // 0-100%
}

pub struct DigitalTwin {
    state: Arc<Mutex<PrinterState>>,
    physics_params: HashMap<String, PhysicsParams>,
    thermal_params: HashMap<String, ThermalParams>,
    simulation_running: Arc<Mutex<bool>>,
    command_sender: Sender<TwinCommand>,
    command_receiver: Arc<Mutex<Receiver<TwinCommand>>>,
    update_rate: Duration,
    start_time: Instant,
}

#[derive(Debug, Clone)]
pub enum TwinCommand {
    MoveAxis { axis: String, position: f64, feedrate: f64 },
    SetTemperature { zone: String, temperature: f64 },
    SetHeaterPower { heater: String, power: f64 },
    SetFanSpeed { fan: u8, speed: f64 },
    ExtrudeFilament { extruder: u8, amount: f64, rate: f64 },
    SetPrintSpeed { speed: f64 },
    EnableMotors,
    DisableMotors,
    HomingSequence { axes: Vec<String> },
    ProbeBed { x: f64, y: f64 },
    StartPrint,
    PausePrint,
    StopPrint,
    EmergencyStop,
}

#[derive(Debug, Clone)]
pub struct SimulationMetrics {
    pub position_accuracy: f64,      // mm RMS error
    pub temperature_accuracy: f64,   // °C RMS error
    pub timing_accuracy: f64,        // ms RMS error
    pub computation_load: f64,       // 0-1 CPU usage
    pub memory_usage: u64,           // bytes
    pub simulation_speed: f64,       // real-time ratio
}

impl DigitalTwin {
    pub fn new(update_rate_hz: f64) -> Self {
        let (command_sender, command_receiver) = mpsc::channel();
        
        let mut initial_state = PrinterState {
            axes: HashMap::new(),
            extruders: Vec::new(),
            hotends: Vec::new(),
            bed_temperature: 20.0,
            chamber_temperature: 20.0,
            print_time: Duration::new(0, 0),
            layer_height: 0.2,
            print_speed: 50.0,
            fan_speeds: vec![0.0; 4],
        };

        // Initialize standard axes
        for axis_name in ["X", "Y", "Z", "E"] {
            initial_state.axes.insert(axis_name.to_string(), AxisState {
                position: 0.0,
                velocity: 0.0,
                acceleration: 0.0,
                force: 0.0,
                temperature: 20.0,
                encoder_position: 0,
                motor_current: 0.0,
                backlash: 0.01, // 0.01mm default backlash
            });
        }

        // Initialize extruder and hotend
        initial_state.extruders.push(ExtruderState {
            position: 0.0,
            velocity: 0.0,
            pressure: 0.0,
            temperature: 20.0,
            flow_rate: 0.0,
            back_pressure: 0.0,
        });

        initial_state.hotends.push(HotendState {
            temperature: 20.0,
            target_temperature: 20.0,
            heater_power: 0.0,
            thermistor_reading: 0.0,
            thermal_mass: 50.0,  // J/K
            heat_loss_coefficient: 5.0,  // W/K
        });

        Self {
            state: Arc::new(Mutex::new(initial_state)),
            physics_params: Self::default_physics_params(),
            thermal_params: Self::default_thermal_params(),
            simulation_running: Arc::new(Mutex::new(false)),
            command_sender,
            command_receiver: Arc::new(Mutex::new(command_receiver)),
            update_rate: Duration::from_secs_f64(1.0 / update_rate_hz),
            start_time: Instant::now(),
        }
    }

    fn default_physics_params() -> HashMap<String, PhysicsParams> {
        let mut params = HashMap::new();
        
        // X/Y axes (typical belt-driven)
        for axis in ["X", "Y"] {
            params.insert(axis.to_string(), PhysicsParams {
                mass: 0.5,  // kg (printhead + carriage)
                friction_coefficient: 0.1,
                inertia: 0.001,  // kg⋅m²
                stiffness: 50000.0,  // N/m
                damping: 10.0,  // N⋅s/m
                gravity: 9.81,
            });
        }
        
        // Z axis (lead screw driven)
        params.insert("Z".to_string(), PhysicsParams {
            mass: 2.0,  // kg (bed + support)
            friction_coefficient: 0.15,
            inertia: 0.01,
            stiffness: 100000.0,
            damping: 50.0,
            gravity: 9.81,
        });
        
        // Extruder
        params.insert("E".to_string(), PhysicsParams {
            mass: 0.01,  // kg (filament segment)
            friction_coefficient: 0.3,
            inertia: 0.00001,
            stiffness: 1000.0,
            damping: 1.0,
            gravity: 0.0,  // Not affected by gravity
        });
        
        params
    }

    fn default_thermal_params() -> HashMap<String, ThermalParams> {
        let mut params = HashMap::new();
        
        // Hotend thermal characteristics
        params.insert("hotend".to_string(), ThermalParams {
            thermal_mass: 50.0,     // J/K
            thermal_conductivity: 16.0,  // W/(m⋅K) - aluminum
            heat_capacity: 900.0,   // J/(kg⋅K)
            ambient_temp: 20.0,
            surface_area: 0.001,    // m²
            emissivity: 0.8,
        });
        
        // Heated bed
        params.insert("bed".to_string(), ThermalParams {
            thermal_mass: 2000.0,   // J/K
            thermal_conductivity: 400.0,  // W/(m⋅K) - aluminum
            heat_capacity: 900.0,
            ambient_temp: 20.0,
            surface_area: 0.04,     // m² (200x200mm)
            emissivity: 0.9,
        });
        
        params
    }

    pub fn start_simulation(&self) {
        let running = Arc::clone(&self.simulation_running);
        let state = Arc::clone(&self.state);
        let command_receiver = Arc::clone(&self.command_receiver);
        let physics_params = self.physics_params.clone();
        let thermal_params = self.thermal_params.clone();
        let update_rate = self.update_rate;

        {
            let mut running_guard = running.lock().unwrap();
            *running_guard = true;
        }

        thread::spawn(move || {
            let mut last_update = Instant::now();
            
            while *running.lock().unwrap() {
                let now = Instant::now();
                let dt = now.duration_since(last_update).as_secs_f64();
                
                // Process commands
                if let Ok(receiver) = command_receiver.try_lock() {
                    while let Ok(command) = receiver.try_recv() {
                        Self::process_command(&state, &command);
                    }
                }
                
                // Update physics simulation
                Self::update_physics(&state, &physics_params, dt);
                
                // Update thermal simulation
                Self::update_thermal(&state, &thermal_params, dt);
                
                last_update = now;
                thread::sleep(update_rate);
            }
        });
    }

    pub fn stop_simulation(&self) {
        let mut running = self.simulation_running.lock().unwrap();
        *running = false;
    }

    pub fn send_command(&self, command: TwinCommand) -> Result<(), mpsc::SendError<TwinCommand>> {
        self.command_sender.send(command)
    }

    pub fn get_state(&self) -> PrinterState {
        let state = self.state.lock().unwrap();
        state.clone()
    }

    fn process_command(state: &Arc<Mutex<PrinterState>>, command: &TwinCommand) {
        let mut state_guard = state.lock().unwrap();
        
        match command {
            TwinCommand::MoveAxis { axis, position, feedrate } => {
                if let Some(axis_state) = state_guard.axes.get_mut(axis) {
                    // Calculate move time and set target velocity
                    let distance = (position - axis_state.position).abs();
                    let move_time = distance / feedrate * 60.0; // feedrate is in mm/min
                    axis_state.velocity = if move_time > 0.0 { distance / move_time } else { 0.0 };
                    
                    // Set target position (actual movement will be simulated in physics update)
                    axis_state.position = *position;
                }
            },
            
            TwinCommand::SetTemperature { zone, temperature } => {
                match zone.as_str() {
                    "hotend" | "hotend0" => {
                        if let Some(hotend) = state_guard.hotends.get_mut(0) {
                            hotend.target_temperature = *temperature;
                        }
                    },
                    "bed" => {
                        // For bed, we'd need a separate bed heater state
                        // For now, just set the temperature directly in simplified simulation
                    },
                    _ => {},
                }
            },
            
            TwinCommand::SetHeaterPower { heater, power } => {
                if heater == "hotend" || heater == "hotend0" {
                    if let Some(hotend) = state_guard.hotends.get_mut(0) {
                        hotend.heater_power = *power;
                    }
                }
            },
            
            TwinCommand::SetFanSpeed { fan, speed } => {
                if (*fan as usize) < state_guard.fan_speeds.len() {
                    state_guard.fan_speeds[*fan as usize] = *speed;
                }
            },
            
            TwinCommand::ExtrudeFilament { extruder, amount, rate } => {
                if (*extruder as usize) < state_guard.extruders.len() {
                    let extruder_state = &mut state_guard.extruders[*extruder as usize];
                    extruder_state.position += amount;
                    extruder_state.velocity = *rate;
                    
                    // Calculate pressure based on flow rate and nozzle characteristics
                    let nozzle_area = std::f64::consts::PI * (0.4f64 / 2.0f64).powi(2); // 0.4mm nozzle
                    extruder_state.flow_rate = rate * nozzle_area;
                    extruder_state.pressure = rate * 1000.0; // Simplified pressure model
                }
            },
            
            TwinCommand::EmergencyStop => {
                // Stop all motion
                for axis_state in state_guard.axes.values_mut() {
                    axis_state.velocity = 0.0;
                    axis_state.acceleration = 0.0;
                }
                
                // Turn off all heaters
                for hotend in &mut state_guard.hotends {
                    hotend.target_temperature = 0.0;
                    hotend.heater_power = 0.0;
                }
            },
            
            _ => {
                // Handle other commands as needed
            }
        }
    }

    fn update_physics(state: &Arc<Mutex<PrinterState>>, physics_params: &HashMap<String, PhysicsParams>, dt: f64) {
        let mut state_guard = state.lock().unwrap();
        
        for (axis_name, axis_state) in &mut state_guard.axes {
            if let Some(params) = physics_params.get(axis_name) {
                // Simple physics integration using Euler method
                // In practice, you'd use more sophisticated integrators like Runge-Kutta
                
                // Calculate forces
                let friction_force = -params.friction_coefficient * params.mass * params.gravity * axis_state.velocity.signum();
                let spring_force = -params.stiffness * axis_state.position * 0.001; // Convert mm to m
                let damping_force = -params.damping * axis_state.velocity * 0.001;
                
                let total_force = axis_state.force + friction_force + spring_force + damping_force;
                
                // Update acceleration (F = ma)
                axis_state.acceleration = total_force / params.mass * 1000.0; // Convert back to mm/s²
                
                // Update velocity
                axis_state.velocity += axis_state.acceleration * dt;
                
                // Update position
                axis_state.position += axis_state.velocity * dt;
                
                // Update encoder position (assuming 4000 steps/mm)
                axis_state.encoder_position = (axis_state.position * 4000.0) as i64;
                
                // Simple motor current model based on acceleration and velocity
                axis_state.motor_current = (axis_state.acceleration.abs() * 0.01 + axis_state.velocity.abs() * 0.001).min(2.0);
                
                // Temperature rise due to motor current
                axis_state.temperature += axis_state.motor_current.powi(2) * 0.1 * dt;
                
                // Cooling back to ambient
                axis_state.temperature += (20.0 - axis_state.temperature) * 0.01 * dt;
            }
        }
    }

    fn update_thermal(state: &Arc<Mutex<PrinterState>>, thermal_params: &HashMap<String, ThermalParams>, dt: f64) {
        let mut state_guard = state.lock().unwrap();
        
        // Update hotend temperatures
        for hotend in &mut state_guard.hotends {
            if let Some(params) = thermal_params.get("hotend") {
                // PID-like behavior for heater control
                let temp_error = hotend.target_temperature - hotend.temperature;
                let required_power = temp_error * 10.0; // Simple proportional control
                hotend.heater_power = required_power.max(0.0).min(100.0); // Limit to 0-100W
                
                // Heat input from heater
                let heat_input = hotend.heater_power * dt; // Joules
                
                // Heat loss to environment (Newton's law of cooling)
                let temp_diff = hotend.temperature - params.ambient_temp;
                let heat_loss = params.surface_area * params.emissivity * 5.67e-8 * 
                    ((hotend.temperature + 273.15).powi(4) - (params.ambient_temp + 273.15).powi(4)) * dt;
                
                // Convective cooling
                let convective_loss = params.surface_area * 10.0 * temp_diff * dt; // Simplified
                
                // Update temperature
                let net_heat = heat_input - heat_loss - convective_loss;
                hotend.temperature += net_heat / params.thermal_mass;
                
                // Simulate thermistor reading with noise
                hotend.thermistor_reading = hotend.temperature * 10.0 + (rand::random::<f64>() - 0.5) * 0.1;
            }
        }
    }

    pub fn get_metrics(&self) -> SimulationMetrics {
        let state = self.state.lock().unwrap();
        
        // Calculate various metrics
        // In a real implementation, these would be computed over time
        SimulationMetrics {
            position_accuracy: 0.01,  // mm RMS
            temperature_accuracy: 0.5, // °C RMS
            timing_accuracy: 1.0,     // ms RMS
            computation_load: 0.1,    // 10% CPU
            memory_usage: 1024 * 1024, // 1MB
            simulation_speed: 1.0,    // Real-time
        }
    }

    pub fn reset_simulation(&self) {
        let mut state = self.state.lock().unwrap();
        
        // Reset all axes to home position
        for axis_state in state.axes.values_mut() {
            axis_state.position = 0.0;
            axis_state.velocity = 0.0;
            axis_state.acceleration = 0.0;
            axis_state.force = 0.0;
            axis_state.temperature = 20.0;
            axis_state.encoder_position = 0;
            axis_state.motor_current = 0.0;
        }
        
        // Reset thermal states
        for hotend in &mut state.hotends {
            hotend.temperature = 20.0;
            hotend.target_temperature = 20.0;
            hotend.heater_power = 0.0;
        }
        
        state.bed_temperature = 20.0;
        state.chamber_temperature = 20.0;
        state.print_time = Duration::new(0, 0);
    }

    pub fn export_state_history(&self, filename: &str) -> Result<(), std::io::Error> {
        // In a real implementation, this would export the complete state history
        // for analysis and validation purposes
        Ok(())
    }

    pub fn load_real_world_calibration(&self, calibration_file: &str) -> Result<(), std::io::Error> {
        // Load calibration data from real printer to match behavior
        Ok(())
    }
}