//! The machine's settings from the Printer page, kept in printer_settings.toml.

use serde::{Deserialize, Serialize};
use std::path::Path;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum MotorType {
    Stepper,
    Servo,
}

/// Settings missing from an older file take their default, so adding one doesn't lose the rest.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PrinterSettings {
    // Temperature settings
    pub extruder_max_temp: f32,
    pub bed_max_temp: f32,
    pub extruder_min_extrude_temp: f32,

    // Movement settings
    pub max_velocity: f32,
    pub max_accel: f32,
    pub max_accel_to_decel: f32,
    pub square_corner_velocity: f32,

    // Homing settings
    pub homing_speed: f32,
    pub homing_retract_dist: f32,
    pub second_homing_speed: f32,

    // Safety settings
    pub position_min: [f32; 3], // X, Y, Z minimum
    pub position_max: [f32; 3], // X, Y, Z maximum

    // Motor configuration (per axis: X, Y, Z, E0, E1)
    pub motor_types: [MotorType; 5],

    // Stepper settings
    pub hold_current: f32,
    pub run_current: f32,
    pub microsteps: u16,

    // Servo settings
    pub servo_min_pulse: u16, // microseconds
    pub servo_max_pulse: u16, // microseconds
    pub servo_max_angle: f32, // degrees

    // Firmware retraction
    pub retract_length: f32,
    pub retract_speed: f32,
    pub unretract_extra_length: f32,
    pub unretract_speed: f32,

    // Pressure advance settings
    pub pressure_advance: f32,
    pub pressure_advance_smooth_time: f32,

    // Z offset and probe settings
    pub z_offset: f32,
    pub probe_z_offset: f32,
    pub probe_xy_offset: [f32; 2], // X, Y offset of probe from nozzle
    pub probe_speed: f32,
    pub probe_lift_speed: f32,
    pub probe_samples: u8,
    pub probe_sample_retract_dist: f32,

    // 5-axis control
    pub enable_5axis_mode: bool,
    pub tool_point_control: bool, // TCP (Tool Center Point) control
    pub rotary_axis_max_velocity: [f32; 2], // A, B axis max velocities
    pub rotary_axis_acceleration: [f32; 2], // A, B axis accelerations
}

impl Default for PrinterSettings {
    fn default() -> Self {
        Self {
            extruder_max_temp: 300.0,
            bed_max_temp: 120.0,
            extruder_min_extrude_temp: 170.0,
            max_velocity: 300.0,
            max_accel: 3000.0,
            max_accel_to_decel: 1500.0,
            square_corner_velocity: 5.0,
            homing_speed: 50.0,
            homing_retract_dist: 5.0,
            second_homing_speed: 10.0,
            position_min: [0.0, 0.0, 0.0],
            position_max: [200.0, 200.0, 200.0],
            motor_types: [MotorType::Stepper, MotorType::Stepper, MotorType::Stepper, MotorType::Stepper, MotorType::Stepper],
            hold_current: 0.5,
            run_current: 0.8,
            microsteps: 16,
            servo_min_pulse: 500,
            servo_max_pulse: 2500,
            servo_max_angle: 180.0,
            retract_length: 0.5,
            retract_speed: 35.0,
            unretract_extra_length: 0.0,
            unretract_speed: 35.0,
            pressure_advance: 0.0,
            pressure_advance_smooth_time: 0.04,
            z_offset: 0.0,
            probe_z_offset: 0.0,
            probe_xy_offset: [0.0, 0.0],
            probe_speed: 5.0,
            probe_lift_speed: 15.0,
            probe_samples: 3,
            probe_sample_retract_dist: 2.0,
            enable_5axis_mode: false,
            tool_point_control: false,
            rotary_axis_max_velocity: [120.0, 120.0],
            rotary_axis_acceleration: [1800.0, 1800.0],
        }
    }
}

impl PrinterSettings {
    /// What's wrong with these settings, in plain words. Empty if they can be used.
    /// Comparisons are written as "must be true" so that a NaN fails them too.
    pub fn problems(&self) -> Vec<String> {
        let mut problems = Vec::new();
        for (axis, name) in ["X", "Y", "Z"].iter().enumerate() {
            if !(self.position_min[axis] < self.position_max[axis]) {
                problems.push(format!("{} minimum must be below {} maximum", name, name));
            }
        }
        if !(self.extruder_min_extrude_temp < self.extruder_max_temp) {
            problems.push("Min extrude temperature must be below the max extruder temperature".to_string());
        }
        if !(self.bed_max_temp > 0.0) {
            problems.push("Max bed temperature must be above zero".to_string());
        }
        if !(self.max_velocity > 0.0) || !(self.max_accel > 0.0) {
            problems.push("Max velocity and acceleration must be above zero".to_string());
        }
        if !(self.max_accel_to_decel <= self.max_accel) {
            problems.push("Max accel to decel can't be above max acceleration".to_string());
        }
        if !(self.second_homing_speed <= self.homing_speed) {
            problems.push("Second homing speed can't be above homing speed".to_string());
        }
        if !(self.hold_current <= self.run_current) {
            problems.push("Hold current can't be above run current".to_string());
        }
        if !(self.servo_min_pulse < self.servo_max_pulse) {
            problems.push("Servo min pulse must be below max pulse".to_string());
        }
        problems
    }

    /// The saved settings. None if nothing has been saved yet.
    pub fn load_from_file(path: &Path) -> Result<Option<Self>, String> {
        match std::fs::read_to_string(path) {
            Ok(text) => toml::from_str(&text).map(Some).map_err(|e| e.to_string()),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
            Err(e) => Err(e.to_string()),
        }
    }

    /// Write the file in one step, so a crash part way can't leave it damaged.
    pub fn save_to_file(&self, path: &Path) -> Result<(), String> {
        let text = toml::to_string_pretty(self).map_err(|e| e.to_string())?;
        let temp = path.with_extension("toml.tmp");
        std::fs::write(&temp, text).map_err(|e| e.to_string())?;
        std::fs::rename(&temp, path).map_err(|e| e.to_string())
    }
}
