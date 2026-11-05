
pub mod advanced_control;
pub mod advanced_motion;
pub mod auto_input_shaper;
pub mod closed_loop_control;
pub mod concurrent_processor;
pub mod digital_twin;
pub mod fast_gcode_parser;
pub mod input_shaping;
pub mod integration_test;
pub mod klipper_can;
pub mod linuxcnc_control;
pub mod motor_assignment;
pub mod optimized_3d_viewer;
pub mod plugin_system;
pub mod power_loss_recovery;
pub mod pressure_advance;
pub mod protocols;
pub mod resonance_cancellation;
pub mod runtime_config;
pub mod sensor_fusion;
pub mod printer_settings;
pub mod simulator;
pub mod smooth_trajectory;
pub mod temperature_control;
pub mod test_runner;
pub mod torque_control;
pub mod volumetric_compensation;

// Platform-specific EtherCAT implementations
#[cfg(windows)]
pub mod ethercat_windows;
