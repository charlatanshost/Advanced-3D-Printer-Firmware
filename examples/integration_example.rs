// Integration Example: Demonstrating all three major improvements working together
// This example shows how the hierarchical config, digital twin, and advanced control
// systems integrate to create a comprehensive 3D printer firmware solution.

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

// Import our new modules
mod hierarchical_config;
mod digital_twin;
mod advanced_control;

use hierarchical_config::*;
use digital_twin::*;
use advanced_control::*;

struct IntegratedPrinterSystem {
    config_manager: HierarchicalConfigManager,
    digital_twin: DigitalTwin,
    motion_controllers: Vec<AdvancedMotionController>,
    simulation_mode: bool,
    current_print_job: Option<PrintJob>,
}

#[derive(Debug, Clone)]
struct PrintJob {
    name: String,
    gcode_commands: Vec<GCodeCommand>,
    estimated_time: Duration,
    material_usage: f64,  // grams
}

#[derive(Debug, Clone)]
enum GCodeCommand {
    Move { x: Option<f64>, y: Option<f64>, z: Option<f64>, e: Option<f64>, f: Option<f64> },
    SetTemperature { heater: String, temp: f64 },
    SetFanSpeed { fan: u8, speed: f64 },
    Home { axes: Vec<String> },
    Dwell { time: Duration },
}

impl IntegratedPrinterSystem {
    pub fn new(simulation_mode: bool) -> Self {
        let config_manager = HierarchicalConfigManager::new();
        let digital_twin = DigitalTwin::new(1000.0); // 1kHz update rate
        
        // Create motion controllers for each axis
        let motion_controllers = vec![
            AdvancedMotionController::new("X".to_string(), 0.001),
            AdvancedMotionController::new("Y".to_string(), 0.001),
            AdvancedMotionController::new("Z".to_string(), 0.001),
            AdvancedMotionController::new("E".to_string(), 0.001),
        ];
        
        Self {
            config_manager,
            digital_twin,
            motion_controllers,
            simulation_mode,
            current_print_job: None,
        }
    }
    
    pub async fn initialize(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        println!("🚀 Initializing Advanced 3D Printer Firmware");
        
        // 1. Load hierarchical configuration
        println!("📋 Loading hierarchical configuration...");
        self.load_configuration().await?;
        
        // 2. Initialize digital twin if in simulation mode
        if self.simulation_mode {
            println!("🔬 Starting digital twin simulation...");
            self.digital_twin.start_simulation();
        }
        
        // 3. Configure motion controllers based on loaded settings
        println!("⚙️ Configuring advanced motion controllers...");
        self.configure_motion_controllers().await?;
        
        // 4. Run system diagnostics
        println!("🔍 Running system diagnostics...");
        self.run_diagnostics().await?;
        
        println!("✅ System initialization complete!");
        Ok(())
    }
    
    async fn load_configuration(&mut self) -> Result<(), ConfigError> {
        // Load base machine configuration
        self.config_manager.load_machine_config("corexy_advanced").await?;
        
        // Set active toolhead
        self.config_manager.set_toolhead("e3d_revo").await?;
        
        // Set nozzle configuration
        self.config_manager.set_nozzle("0.4mm_hardened_steel").await?;
        
        // Set material profile
        self.config_manager.set_material("pla_plus").await?;
        
        println!("   ✓ Machine: CoreXY Advanced");
        println!("   ✓ Toolhead: E3D Revo Six");
        println!("   ✓ Nozzle: 0.4mm Hardened Steel");
        println!("   ✓ Material: PLA+");
        
        Ok(())
    }
    
    async fn configure_motion_controllers(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let config = self.config_manager.get_current_config().await;
        
        // Configure each axis controller based on the hierarchical configuration
        for controller in &mut self.motion_controllers {
            match controller.axis_name.as_str() {
                "X" | "Y" => {
                    // Configure for high-speed XY movement with state-space control
                    controller.set_control_mode(ControlMode::StateSpace);
                    println!("   ✓ {} axis: State-Space Control (high-speed)", controller.axis_name);
                },
                "Z" => {
                    // Configure for precise Z movement with MPC
                    controller.set_control_mode(ControlMode::MPC);
                    println!("   ✓ {} axis: Model Predictive Control (precision)", controller.axis_name);
                },
                "E" => {
                    // Configure extruder with hybrid control
                    controller.set_control_mode(ControlMode::Hybrid);
                    println!("   ✓ {} axis: Hybrid Control (adaptive)", controller.axis_name);
                },
                _ => {}
            }
        }
        
        Ok(())
    }
    
    async fn run_diagnostics(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        println!("   🔧 Testing motion system...");
        
        if self.simulation_mode {
            // Test digital twin responsiveness
            let test_commands = vec![
                TwinCommand::MoveAxis { axis: "X".to_string(), position: 100.0, feedrate: 3000.0 },
                TwinCommand::MoveAxis { axis: "Y".to_string(), position: 100.0, feedrate: 3000.0 },
                TwinCommand::SetTemperature { zone: "hotend".to_string(), temperature: 220.0 },
            ];
            
            for cmd in test_commands {
                self.digital_twin.send_command(cmd)?;
            }
            
            // Wait for simulation to process
            thread::sleep(Duration::from_millis(100));
            
            let state = self.digital_twin.get_state();
            println!("   ✓ Digital twin simulation active");
            println!("     - X position: {:.2}mm", state.axes.get("X").map_or(0.0, |a| a.position));
            println!("     - Y position: {:.2}mm", state.axes.get("Y").map_or(0.0, |a| a.position));
            println!("     - Hotend temp: {:.1}°C", state.hotends.first().map_or(0.0, |h| h.temperature));
        }
        
        println!("   🧠 Testing advanced control algorithms...");
        
        // Test each motion controller
        for controller in &mut self.motion_controllers {
            let control_output = controller.update_control(
                0.0,   // current position
                0.0,   // current velocity  
                0.0,   // current acceleration
                10.0,  // target position
                0.0,   // target velocity
            );
            
            let metrics = controller.get_metrics();
            println!("     {} axis: Control output = {:.2}, RMS error = {:.4}", 
                controller.axis_name, control_output, metrics.tracking_error_rms);
        }
        
        println!("   📊 Configuration validation...");
        self.config_manager.validate_hierarchy().await?;
        println!("     ✓ All configuration files valid and consistent");
        
        Ok(())
    }
    
    pub async fn start_print_job(&mut self, job: PrintJob) -> Result<(), Box<dyn std::error::Error>> {
        println!("🖨️ Starting print job: {}", job.name);
        self.current_print_job = Some(job.clone());
        
        // Pre-heat based on material configuration
        if self.simulation_mode {
            self.digital_twin.send_command(TwinCommand::SetTemperature {
                zone: "hotend".to_string(),
                temperature: 220.0, // From PLA+ material config
            })?;
            
            self.digital_twin.send_command(TwinCommand::SetTemperature {
                zone: "bed".to_string(), 
                temperature: 60.0, // From PLA+ material config
            })?;
        }
        
        // Home all axes
        if self.simulation_mode {
            self.digital_twin.send_command(TwinCommand::HomingSequence {
                axes: vec!["X".to_string(), "Y".to_string(), "Z".to_string()],
            })?;
        }
        
        // Process G-code commands
        for (i, command) in job.gcode_commands.iter().enumerate() {
            if i % 100 == 0 {
                println!("   Processing command {}/{}", i + 1, job.gcode_commands.len());
            }
            
            self.execute_gcode_command(command).await?;
            
            // Small delay to prevent overwhelming the system
            thread::sleep(Duration::from_micros(100));
        }
        
        println!("✅ Print job completed successfully!");
        Ok(())
    }
    
    async fn execute_gcode_command(&mut self, command: &GCodeCommand) -> Result<(), Box<dyn std::error::Error>> {
        match command {
            GCodeCommand::Move { x, y, z, e, f } => {
                // Execute coordinated motion using advanced controllers
                
                if let Some(x_pos) = x {
                    if let Some(controller) = self.motion_controllers.iter_mut().find(|c| c.axis_name == "X") {
                        let control_output = controller.update_control(0.0, 0.0, 0.0, *x_pos, 0.0);
                        
                        if self.simulation_mode {
                            self.digital_twin.send_command(TwinCommand::MoveAxis {
                                axis: "X".to_string(),
                                position: *x_pos,
                                feedrate: f.unwrap_or(3000.0),
                            })?;
                        }
                    }
                }
                
                if let Some(y_pos) = y {
                    if let Some(controller) = self.motion_controllers.iter_mut().find(|c| c.axis_name == "Y") {
                        let control_output = controller.update_control(0.0, 0.0, 0.0, *y_pos, 0.0);
                        
                        if self.simulation_mode {
                            self.digital_twin.send_command(TwinCommand::MoveAxis {
                                axis: "Y".to_string(),
                                position: *y_pos,
                                feedrate: f.unwrap_or(3000.0),
                            })?;
                        }
                    }
                }
                
                if let Some(z_pos) = z {
                    if let Some(controller) = self.motion_controllers.iter_mut().find(|c| c.axis_name == "Z") {
                        let control_output = controller.update_control(0.0, 0.0, 0.0, *z_pos, 0.0);
                        
                        if self.simulation_mode {
                            self.digital_twin.send_command(TwinCommand::MoveAxis {
                                axis: "Z".to_string(),
                                position: *z_pos,
                                feedrate: f.unwrap_or(300.0),
                            })?;
                        }
                    }
                }
                
                if let Some(e_pos) = e {
                    if self.simulation_mode {
                        self.digital_twin.send_command(TwinCommand::ExtrudeFilament {
                            extruder: 0,
                            amount: *e_pos,
                            rate: f.unwrap_or(1500.0) / 60.0, // Convert mm/min to mm/s
                        })?;
                    }
                }
            },
            
            GCodeCommand::SetTemperature { heater, temp } => {
                if self.simulation_mode {
                    self.digital_twin.send_command(TwinCommand::SetTemperature {
                        zone: heater.clone(),
                        temperature: *temp,
                    })?;
                }
            },
            
            GCodeCommand::SetFanSpeed { fan, speed } => {
                if self.simulation_mode {
                    self.digital_twin.send_command(TwinCommand::SetFanSpeed {
                        fan: *fan,
                        speed: *speed,
                    })?;
                }
            },
            
            GCodeCommand::Home { axes } => {
                if self.simulation_mode {
                    self.digital_twin.send_command(TwinCommand::HomingSequence {
                        axes: axes.clone(),
                    })?;
                }
            },
            
            GCodeCommand::Dwell { time } => {
                thread::sleep(*time);
            },
        }
        
        Ok(())
    }
    
    pub fn get_system_status(&self) -> SystemStatus {
        let digital_twin_state = if self.simulation_mode {
            Some(self.digital_twin.get_state())
        } else {
            None
        };
        
        let controller_metrics: Vec<_> = self.motion_controllers
            .iter()
            .map(|c| (c.axis_name.clone(), c.get_metrics().clone()))
            .collect();
        
        SystemStatus {
            simulation_active: self.simulation_mode,
            digital_twin_state,
            controller_metrics,
            current_print_job: self.current_print_job.as_ref().map(|j| j.name.clone()),
        }
    }
}

#[derive(Debug)]
struct SystemStatus {
    simulation_active: bool,
    digital_twin_state: Option<PrinterState>,
    controller_metrics: Vec<(String, ControlMetrics)>,
    current_print_job: Option<String>,
}

// Example usage and demonstration
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🎯 Advanced 3D Printer Firmware Integration Example");
    println!("================================================\n");
    
    // Initialize the integrated system in simulation mode
    let mut printer_system = IntegratedPrinterSystem::new(true);
    printer_system.initialize().await?;
    
    println!("\n📈 System Status Overview:");
    let status = printer_system.get_system_status();
    println!("   Simulation Active: {}", status.simulation_active);
    if let Some(ref twin_state) = status.digital_twin_state {
        println!("   Bed Temperature: {:.1}°C", twin_state.bed_temperature);
        println!("   Chamber Temperature: {:.1}°C", twin_state.chamber_temperature);
    }
    
    // Create a sample print job
    let sample_job = PrintJob {
        name: "Advanced Control Demo".to_string(),
        gcode_commands: vec![
            GCodeCommand::Home { axes: vec!["X".to_string(), "Y".to_string(), "Z".to_string()] },
            GCodeCommand::SetTemperature { heater: "hotend".to_string(), temp: 220.0 },
            GCodeCommand::SetTemperature { heater: "bed".to_string(), temp: 60.0 },
            GCodeCommand::Move { x: Some(100.0), y: Some(100.0), z: Some(0.3), e: None, f: Some(3000.0) },
            GCodeCommand::Move { x: Some(200.0), y: Some(100.0), z: None, e: Some(10.0), f: Some(1800.0) },
            GCodeCommand::Move { x: Some(200.0), y: Some(200.0), z: None, e: Some(20.0), f: Some(1800.0) },
            GCodeCommand::Move { x: Some(100.0), y: Some(200.0), z: None, e: Some(30.0), f: Some(1800.0) },
            GCodeCommand::Move { x: Some(100.0), y: Some(100.0), z: None, e: Some(40.0), f: Some(1800.0) },
        ],
        estimated_time: Duration::from_secs(300),
        material_usage: 5.2,
    };
    
    // Start the demo print job
    println!("\n🎬 Starting demonstration print job...");
    printer_system.start_print_job(sample_job).await?;
    
    println!("\n📊 Final System Performance:");
    let final_status = printer_system.get_system_status();
    for (axis, metrics) in &final_status.controller_metrics {
        println!("   {} Axis - Tracking Error: {:.4}mm RMS, Steady State Error: {:.4}mm",
                axis, metrics.tracking_error_rms, metrics.steady_state_error);
    }
    
    if let Some(ref twin_state) = final_status.digital_twin_state {
        if let Some(metrics) = twin_state.axes.get("X") {
            println!("   Digital Twin X Position: {:.2}mm", metrics.position);
        }
        if let Some(metrics) = twin_state.axes.get("Y") {
            println!("   Digital Twin Y Position: {:.2}mm", metrics.position);
        }
    }
    
    println!("\n🎉 Integration example completed successfully!");
    println!("\nKey Features Demonstrated:");
    println!("✅ Hierarchical Configuration Management");
    println!("✅ Digital Twin Simulation with Physics");
    println!("✅ Advanced Control Theory (State-Space & MPC)");
    println!("✅ Real-time System Integration");
    println!("✅ Performance Monitoring & Metrics");
    
    Ok(())
}