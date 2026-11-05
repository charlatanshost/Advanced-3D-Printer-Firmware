// Virtual printer: runs real G-code through the real parser, trajectory planner, kinematics,
// heater safety logic and power-loss recovery, with simulated physics in place of hardware.
//
// It runs much faster than real time on a simulated clock. It models what the control code
// sees and does: heater and sensor physics, where the nozzle goes, what it runs into, and how
// much filament it pushes. It does not model the plastic, belts or vibration, and passing
// here does not show a real machine will behave, only that the logic does the right thing.

use crate::fast_gcode_parser::{CommandType, GCodeCommand, Parameters, StreamingGCodeParser};
use crate::power_loss_recovery::{PowerLossRecoveryManager, PrintStateUpdate, RecoveryConfig};
use crate::smooth_trajectory::{MotionConstraints, MotionProfile, SmoothTrajectoryPlanner};
use crate::temperature_control::{HeaterConfig, SensorConfig, TemperatureController, TemperatureZone};
use printer_kinematics::{CartesianKinematics, Kinematics};
use std::collections::{BTreeMap, HashMap, VecDeque};
use std::io::Cursor;
use std::path::Path;
use std::time::{Duration, Instant};

const THERMAL_STEP: f64 = 0.1; // Seconds. The heater loop's 10 Hz
const LOOKAHEAD_MOVES: usize = 32;
const HOMING_SPEED: f64 = 50.0; // mm/s
const MAX_EXTRUDER_SPEED: f64 = 100.0; // mm/s
const TEMPERATURE_TOLERANCE: f64 = 2.0; // How close counts as at temperature, in °C
const TEMPERATURE_SETTLE: f64 = 3.0; // How long it must stay there, in seconds
const TEMPERATURE_WAIT_LIMIT: f64 = 900.0; // Longest wait for M109/M190, in seconds
const HEIGHT_MAP_CELL: f64 = 0.25; // mm
// How far under already-printed material counts as a crash. About a layer: non-planar prints
// legitimately slope down across a cell, while a real crash is far deeper than that.
const COLLISION_TOLERANCE: f64 = 0.35; // mm

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Heater {
    Hotend,
    Bed,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Fault {
    /// Thermistor wire broken
    SensorOpen(Heater),
    /// Thermistor wires touching
    SensorShort(Heater),
    /// Sensor fallen out of the block: it reads a steady value whatever really happens
    SensorFrozen(Heater),
    /// Burnt-out heater or broken wire
    HeaterDead(Heater),
    /// Failed MOSFET: full power however the software commands it
    HeaterStuckOn(Heater),
    /// Mains gone with no warning
    PowerCut,
    /// Mains failing, with time to run the emergency sequence
    PowerLossDetected,
    /// Stop button
    EmergencyStop,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FaultTrigger {
    /// Seconds after the simulation started
    AtTime(f64),
    /// When this line of the file comes up
    AtLine(u32),
}

/// A first-order thermal model: a mass heated by a heater and losing heat to the room.
#[derive(Debug, Clone)]
pub struct ThermalPlant {
    pub temperature: f64,
    pub ambient: f64,
    pub heat_capacity: f64, // J/K
    pub heat_loss: f64,     // W/K
    pub heater_power: f64,  // W
}

impl ThermalPlant {
    fn step(&mut self, duty: f64, dt: f64) {
        let net_power = self.heater_power * duty - self.heat_loss * (self.temperature - self.ambient);
        self.temperature += net_power * dt / self.heat_capacity;
    }

    /// Cool with the power off
    fn cool_for(&mut self, seconds: f64) {
        let decay = (-self.heat_loss / self.heat_capacity * seconds).exp();
        self.temperature = self.ambient + (self.temperature - self.ambient) * decay;
    }
}

#[derive(Debug, Clone)]
pub struct SimConfig {
    pub build_volume: [f64; 3],
    pub start_position: [f64; 3],
    pub min_extrude_temp: f64,
    pub hotend: ThermalPlant,
    pub bed: ThermalPlant,
    pub hotend_max_temp: f64,
    pub bed_max_temp: f64,
    pub constraints: MotionConstraints,
    pub profile: MotionProfile,
    /// Points per second of motion. Lower runs faster and checks less often.
    pub interpolation_hz: f64,
    /// Where to keep recovery data. None turns recovery off.
    pub recovery: Option<RecoveryConfig>,
    /// How often print state is saved for recovery, in simulated seconds
    pub recovery_save_interval: f64,
    pub faults: Vec<(FaultTrigger, Fault)>,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            build_volume: [220.0, 220.0, 250.0],
            start_position: [110.0, 110.0, 20.0],
            min_extrude_temp: 170.0,
            // 40 W cartridge in a typical block: about 2.5 °C/s from cold
            hotend: ThermalPlant { temperature: 25.0, ambient: 25.0, heat_capacity: 15.0, heat_loss: 0.09, heater_power: 40.0 },
            // 220 x 220 aluminium bed on 200 W: about 0.5 °C/s from cold
            bed: ThermalPlant { temperature: 25.0, ambient: 25.0, heat_capacity: 400.0, heat_loss: 1.5, heater_power: 200.0 },
            hotend_max_temp: 300.0,
            bed_max_temp: 150.0,
            constraints: MotionConstraints::default(),
            profile: MotionProfile::Trapezoidal,
            interpolation_hz: 200.0,
            recovery: None,
            recovery_save_interval: 1.0,
            faults: Vec::new(),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Outcome {
    Running,
    Completed,
    /// The machine stopped itself for this reason
    Halted(String),
    /// Power went away part way through
    PowerLost,
}

#[derive(Debug, Clone)]
pub struct SimReport {
    pub outcome: Outcome,
    pub seconds: f64,
    pub lines_run: usize,
    pub moves: usize,
    /// Filament pushed forward, in mm. Counts everything, including what was retracted again.
    pub filament_extruded: f64,
    /// Forward minus retraction: what actually left the nozzle, about
    pub filament_net: f64,
    pub final_position: [f64; 3],
    pub final_motors: [f32; 5],
    pub final_e: f64,
    pub peak_hotend: f64,
    pub peak_bed: f64,
    /// Furthest above target once at temperature, in °C
    pub hotend_overshoot: f64,
    pub bed_overshoot: f64,
    pub peak_speed: f64,
    /// Samples where the nozzle went through something already printed
    pub collisions: usize,
    pub first_collision: Option<[f64; 3]>,
    /// Extrusion refused because the hotend was too cold
    pub cold_extrusions: usize,
    /// Commands this simulator does not model
    pub ignored_commands: BTreeMap<String, usize>,
    /// What happened and when: (seconds, description)
    pub events: Vec<(f64, String)>,
    /// Problems software cannot solve on its own
    pub warnings: Vec<String>,
}

impl SimReport {
    /// Finished the file, with nothing physically wrong along the way
    pub fn is_clean(&self) -> bool {
        self.outcome == Outcome::Completed && self.collisions == 0 && self.cold_extrusions == 0
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum SensorState {
    Normal,
    Open,
    Short,
    Frozen(u16),
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum HeaterState {
    Normal,
    Dead,
    StuckOn,
}

struct Zone {
    plant: ThermalPlant,
    controller: TemperatureController,
    sensor: SensorState,
    heater: HeaterState,
    max_temp: f64,
    peak: f64,
    overshoot: f64,
    reached_target: bool,
}

impl Zone {
    fn new(zone: TemperatureZone, plant: ThermalPlant, max_temp: f64, heater_power: f64) -> Self {
        let sensor_config = SensorConfig { max_temp, ..SensorConfig::default() };
        let heater_config = HeaterConfig { max_power: heater_power, ..HeaterConfig::default() };
        let mut controller = TemperatureController::new(zone, sensor_config, Some(heater_config));
        controller.enable();
        let peak = plant.temperature;
        Self { plant, controller, sensor: SensorState::Normal, heater: HeaterState::Normal, max_temp, peak, overshoot: 0.0, reached_target: false }
    }

    /// What the ADC would read for a temperature, with the given noise
    fn adc_reading(&self, noise: i32) -> u16 {
        match self.sensor {
            SensorState::Open => 4095,
            SensorState::Short => 0,
            SensorState::Frozen(raw) => raw,
            SensorState::Normal => (temperature_to_adc(self.plant.temperature) as i32 + noise).clamp(1, 4094) as u16,
        }
    }

    fn set_target(&mut self, target: f64) {
        self.controller.set_target_temperature(target);
        self.reached_target = false;
    }
}

/// The inverse of the controller's conversion: the ADC value a real thermistor would produce
fn temperature_to_adc(temperature: f64) -> u16 {
    let config = SensorConfig::default();
    let inverse_reference = 1.0 / (config.t0 + 273.15);
    let resistance = config.r25 * (config.beta * (1.0 / (temperature + 273.15) - inverse_reference)).exp();
    let ratio = resistance / (resistance + config.series_resistor);
    (ratio * 4095.0).round().clamp(0.0, 4095.0) as u16
}

/// A planned move still being executed, and the state to record once it is done
#[derive(Debug, Clone)]
struct PendingMove {
    e_delta: f64,
    e_applied: f64,
    line: u32,
    end_logical: [f64; 3],
    end_e: f64,
    feed: f64,
    relative_e: bool,
}

pub struct Simulator {
    config: SimConfig,
    kinematics: Box<dyn Kinematics>,
    planner: SmoothTrajectoryPlanner,
    pending: VecDeque<PendingMove>,
    last_point_time: f64,
    hotend: Zone,
    bed: Zone,
    clock_start: Instant,
    clock: f64,
    next_thermal: f64,
    noise_state: u32,

    // Where the machine physically is, and where motion has been planned to
    position: [f64; 3],
    planned_end: [f64; 3],
    // logical = physical + offset (G92 moves the origin, not the machine)
    offset: [f64; 3],
    axis_known: [bool; 3],
    relative_xyz: bool,
    relative_e: bool,
    e_counter: f64,
    feed: f64,
    fan: f64,
    current_line: u32,
    motors: [f32; 5],

    height_map: HashMap<(i32, i32), f64>,
    recovery: Option<PowerLossRecoveryManager>,
    last_save: f64,
    last_z_save: f64,
    completed: Option<PendingMove>,
    report: SimReport,
    faults: Vec<(FaultTrigger, Fault)>,
}

impl Simulator {
    pub fn new(config: SimConfig) -> Self {
        Self::with_kinematics(config, Box::new(CartesianKinematics))
    }

    pub fn with_kinematics(config: SimConfig, kinematics: Box<dyn Kinematics>) -> Self {
        let mut planner = SmoothTrajectoryPlanner::new(config.constraints.clone(), config.profile.clone());
        planner.set_interpolation_rate(config.interpolation_hz);
        let clock_start = Instant::now();
        let hotend = Zone::new(TemperatureZone::Hotend(0), config.hotend.clone(), config.hotend_max_temp, config.hotend.heater_power);
        let bed = Zone::new(TemperatureZone::Bed, config.bed.clone(), config.bed_max_temp, config.bed.heater_power);
        let position = config.start_position;
        let faults = config.faults.clone();
        let recovery = config.recovery.clone().and_then(|c| PowerLossRecoveryManager::new(c).ok());
        let mut simulator = Self {
            kinematics,
            planner,
            pending: VecDeque::new(),
            last_point_time: 0.0,
            hotend,
            bed,
            clock_start,
            clock: 0.0,
            next_thermal: 0.0,
            noise_state: 0x1234_5678,
            position,
            planned_end: position,
            offset: [0.0; 3],
            axis_known: [false; 3],
            relative_xyz: false,
            relative_e: false,
            e_counter: 0.0,
            feed: 30.0,
            fan: 0.0,
            current_line: 0,
            motors: [0.0; 5],
            height_map: HashMap::new(),
            recovery,
            last_save: 0.0,
            last_z_save: -1.0,
            completed: None,
            report: SimReport {
                outcome: Outcome::Running,
                seconds: 0.0,
                lines_run: 0,
                moves: 0,
                filament_extruded: 0.0,
                filament_net: 0.0,
                final_position: position,
                final_motors: [0.0; 5],
                final_e: 0.0,
                peak_hotend: 0.0,
                peak_bed: 0.0,
                hotend_overshoot: 0.0,
                bed_overshoot: 0.0,
                peak_speed: 0.0,
                collisions: 0,
                first_collision: None,
                cold_extrusions: 0,
                ignored_commands: BTreeMap::new(),
                events: Vec::new(),
                warnings: Vec::new(),
            },
            faults,
            config,
        };
        simulator.sync_clock();
        simulator
    }

    fn sim_now(&self) -> Instant {
        self.clock_start + Duration::from_secs_f64(self.clock)
    }

    fn sync_clock(&mut self) {
        let now = self.sim_now();
        self.hotend.controller.set_clock(now);
        self.bed.controller.set_clock(now);
    }

    fn log(&mut self, message: String) {
        self.report.events.push((self.clock, message));
    }

    fn stopped(&self) -> bool {
        self.report.outcome != Outcome::Running
    }

    /// Stop the machine: heaters off, motion abandoned. A stuck MOSFET does not listen.
    fn halt(&mut self, reason: String) {
        if self.stopped() {
            return;
        }
        self.log(format!("HALT: {}", reason));
        self.report.outcome = Outcome::Halted(reason);
        self.planner.clear_buffer();
        self.pending.clear();
        self.last_point_time = 0.0;
        self.hotend.set_target(0.0);
        self.bed.set_target(0.0);
        self.hotend.controller.disable();
        self.bed.controller.disable();
    }

    // ---------- Time and heaters ----------

    /// Let time pass, running the heater loop as it comes due
    fn advance(&mut self, seconds: f64) {
        self.clock += seconds;
        while self.next_thermal <= self.clock {
            self.next_thermal += THERMAL_STEP;
            self.thermal_step();
            self.apply_time_faults();
        }
    }

    fn next_noise(&mut self) -> i32 {
        // Xorshift: deterministic, so a failure always reproduces
        self.noise_state ^= self.noise_state << 13;
        self.noise_state ^= self.noise_state >> 17;
        self.noise_state ^= self.noise_state << 5;
        (self.noise_state % 3) as i32 - 1
    }

    fn thermal_step(&mut self) {
        let now = self.clock_start + Duration::from_secs_f64(self.next_thermal - THERMAL_STEP);
        let hotend_noise = self.next_noise();
        let bed_noise = self.next_noise();
        Self::step_zone(&mut self.hotend, now, hotend_noise);
        Self::step_zone(&mut self.bed, now, bed_noise);

        // The heater logic deciding to stop is what stops the print
        if !self.stopped() {
            let tripped = [&self.hotend, &self.bed].into_iter()
                .find(|zone| zone.controller.safety_shutdown)
                .map(|zone| zone.controller.last_error.clone().unwrap_or_else(|| "Heater safety shutdown".to_string()));
            if let Some(reason) = tripped {
                self.halt(reason);
            }
        }
    }

    fn step_zone(zone: &mut Zone, now: Instant, noise: i32) {
        zone.controller.set_clock(now);
        let raw = zone.adc_reading(noise);
        let _ = zone.controller.update_reading(raw); // A fault latches inside the controller
        let commanded = zone.controller.calculate_output();
        let duty = match zone.heater {
            HeaterState::Normal => commanded,
            HeaterState::Dead => 0.0,
            HeaterState::StuckOn => 1.0,
        };
        zone.plant.step(duty, THERMAL_STEP);
        zone.peak = zone.peak.max(zone.plant.temperature);

        let target = zone.controller.target_temperature;
        if target > 0.0 {
            if zone.plant.temperature >= target - TEMPERATURE_TOLERANCE {
                zone.reached_target = true;
            }
            if zone.reached_target {
                zone.overshoot = zone.overshoot.max(zone.plant.temperature - target);
            }
        }
    }

    fn zone_mut(&mut self, heater: Heater) -> &mut Zone {
        match heater {
            Heater::Hotend => &mut self.hotend,
            Heater::Bed => &mut self.bed,
        }
    }

    fn apply_time_faults(&mut self) {
        let clock = self.clock;
        let mut due = Vec::new();
        self.faults.retain(|(trigger, fault)| match trigger {
            FaultTrigger::AtTime(t) if *t <= clock => {
                due.push(*fault);
                false
            }
            _ => true,
        });
        for fault in due {
            self.apply_fault(fault);
        }
    }

    fn apply_line_faults(&mut self, line: u32) {
        let mut due = Vec::new();
        self.faults.retain(|(trigger, fault)| match trigger {
            FaultTrigger::AtLine(l) if *l <= line => {
                due.push(*fault);
                false
            }
            _ => true,
        });
        for fault in due {
            self.apply_fault(fault);
        }
    }

    fn apply_fault(&mut self, fault: Fault) {
        self.log(format!("FAULT: {:?}", fault));
        match fault {
            Fault::SensorOpen(heater) => self.zone_mut(heater).sensor = SensorState::Open,
            Fault::SensorShort(heater) => self.zone_mut(heater).sensor = SensorState::Short,
            Fault::SensorFrozen(heater) => {
                let zone = self.zone_mut(heater);
                zone.sensor = SensorState::Frozen(temperature_to_adc(zone.plant.temperature));
            }
            Fault::HeaterDead(heater) => self.zone_mut(heater).heater = HeaterState::Dead,
            Fault::HeaterStuckOn(heater) => self.zone_mut(heater).heater = HeaterState::StuckOn,
            Fault::PowerCut => self.lose_power(),
            Fault::PowerLossDetected => self.power_loss_detected(),
            Fault::EmergencyStop => self.halt("Emergency stop".to_string()),
        }
    }

    // ---------- Power loss ----------

    fn lose_power(&mut self) {
        if self.stopped() {
            return;
        }
        self.log("Power lost".to_string());
        self.report.outcome = Outcome::PowerLost;
        self.planner.clear_buffer();
        self.pending.clear();
        self.last_point_time = 0.0;
        self.planned_end = self.position;
        self.hotend.heater = HeaterState::Dead;
        self.bed.heater = HeaterState::Dead;
    }

    /// Failing mains, with time left: run the same sequence a real machine would, then die.
    fn power_loss_detected(&mut self) {
        if self.stopped() {
            return;
        }
        if let Some(manager) = &mut self.recovery {
            // Below any threshold, however often it has been sampled
            let _ = manager.monitor_power(0.0);
            let lift = manager.get_config().z_lift_on_power_loss;
            let retract = manager.get_config().recovery_retraction;
            self.position[2] = (self.position[2] + lift).min(self.config.build_volume[2]);
            self.report.filament_net -= retract;
            let confirmed = manager.confirm_emergency_lift();
            let message = format!("Emergency lift of {} mm and retract of {} mm ({:?})", lift, retract, confirmed);
            self.log(message);
        }
        self.lose_power();
    }

    /// Everything that survives a power cut: the part, and the axes where they stopped.
    /// Everything the controller knew is gone. Heaters have cooled for `off_seconds`.
    pub fn restart_after_power_loss(mut self, off_seconds: f64) -> Self {
        let mut config = self.config.clone();
        config.hotend = self.hotend.plant.clone();
        config.bed = self.bed.plant.clone();
        config.hotend.cool_for(off_seconds);
        config.bed.cool_for(off_seconds);
        config.start_position = self.position;
        config.faults.clear();
        let height_map = std::mem::take(&mut self.height_map);
        let previous = self.report.clone();

        let mut restarted = Self::with_kinematics(config, self.kinematics);
        restarted.height_map = height_map;
        restarted.report.filament_extruded = previous.filament_extruded;
        restarted.report.filament_net = previous.filament_net;
        restarted.report.collisions = previous.collisions;
        restarted.report.first_collision = previous.first_collision;
        restarted.report.cold_extrusions = previous.cold_extrusions;
        restarted
    }

    /// Load the recovery file and carry on with the rest of the G-code. Returns where it resumed.
    pub fn resume(&mut self, commands: &[GCodeCommand]) -> Result<u32, String> {
        let manager = self.recovery.as_mut().ok_or("Recovery is not configured")?;
        let state = manager.load_recovery_data()?.ok_or("No recovery data")?;
        let resume_line = state.line_number;
        let plan = manager.start_recovery(state)?;
        self.current_line = resume_line;
        self.log(format!("Resuming after line {}", resume_line));

        for step in &plan.steps {
            for text in &step.gcode_commands {
                let parsed = parse_gcode(text).map_err(|e| format!("Recovery step {:?}: {}", text, e))?;
                for command in &parsed {
                    self.execute(command);
                    if self.stopped() {
                        return Err(format!("Recovery stopped during {:?}", text));
                    }
                }
            }
        }
        self.drain();

        self.run_commands(commands.iter().filter(|c| c.line_number > resume_line));
        self.finish();
        if let Some(manager) = self.recovery.as_mut() {
            if self.report.outcome == Outcome::Completed {
                let _ = manager.complete_recovery(true);
            }
        }
        Ok(resume_line)
    }

    // ---------- Running a print ----------

    pub fn run(&mut self, commands: &[GCodeCommand]) {
        if let Some(manager) = self.recovery.as_mut() {
            let _ = manager.start_print_tracking("simulated.gcode".to_string(), 0);
        }
        self.run_commands(commands.iter());
        self.finish();
        if self.report.outcome == Outcome::Completed {
            if let Some(manager) = self.recovery.as_mut() {
                let _ = manager.stop_print_tracking();
            }
        }
    }

    fn run_commands<'a>(&mut self, commands: impl Iterator<Item = &'a GCodeCommand>) {
        for command in commands {
            self.current_line = command.line_number;
            self.apply_line_faults(command.line_number);
            if self.stopped() {
                return;
            }
            self.execute(command);
            if self.stopped() {
                return;
            }
            self.report.lines_run += 1;
        }
        self.drain();
    }

    fn finish(&mut self) {
        if self.report.outcome == Outcome::Running {
            self.report.outcome = Outcome::Completed;
            self.log("Print finished".to_string());
        }

        // After a failure, watch what the heaters do, since it may not be over.
        if matches!(self.report.outcome, Outcome::Halted(_)) {
            for _ in 0..(180.0 / THERMAL_STEP) as usize {
                self.advance(THERMAL_STEP);
            }
            for (name, zone) in [("Hotend", &self.hotend), ("Bed", &self.bed)] {
                if zone.heater == HeaterState::StuckOn && zone.peak > zone.max_temp {
                    self.report.warnings.push(format!(
                        "{} reached {:.0}°C with its heater failed on, after the software shut down. Only hardware can stop this: a thermal fuse or a relay the software can open.",
                        name, zone.peak
                    ));
                }
            }
        }

        self.report.seconds = self.clock;
        self.report.final_position = self.position;
        self.report.final_motors = self.motors;
        self.report.final_e = self.e_counter;
        self.report.peak_hotend = self.hotend.peak;
        self.report.peak_bed = self.bed.peak;
        self.report.hotend_overshoot = self.hotend.overshoot;
        self.report.bed_overshoot = self.bed.overshoot;
    }

    pub fn report(&self) -> &SimReport {
        &self.report
    }

    /// Highest material printed within half a millimetre of a position, if any
    pub fn printed_height(&self, x: f64, y: f64) -> Option<f64> {
        let (cx, cy) = Self::cell(x, y);
        let reach = (0.5 / HEIGHT_MAP_CELL).ceil() as i32;
        let mut highest = None;
        for dx in -reach..=reach {
            for dy in -reach..=reach {
                if let Some(height) = self.height_map.get(&(cx + dx, cy + dy)) {
                    highest = Some(highest.map_or(*height, |h: f64| h.max(*height)));
                }
            }
        }
        highest
    }
    
    /// How many cells of the height map have been printed on
    pub fn printed_cells(&self) -> usize {
        self.height_map.len()
    }

    fn cell(x: f64, y: f64) -> (i32, i32) {
        ((x / HEIGHT_MAP_CELL).floor() as i32, (y / HEIGHT_MAP_CELL).floor() as i32)
    }

    fn execute(&mut self, command: &GCodeCommand) {
        let p = &command.parameters;
        match &command.command_type {
            CommandType::G(0) | CommandType::G(1) => self.linear_move(p),
            CommandType::G(2) => self.arc_move(p, true),
            CommandType::G(3) => self.arc_move(p, false),
            CommandType::G(4) => {
                self.drain();
                let seconds = p.s.unwrap_or(0.0) as f64 + p.p.unwrap_or(0.0) as f64 / 1000.0;
                self.wait(seconds);
            }
            CommandType::G(28) => self.home(p),
            CommandType::G(90) => {
                self.drain();
                self.relative_xyz = false;
                self.relative_e = false;
            }
            CommandType::G(91) => {
                self.drain();
                self.relative_xyz = true;
                self.relative_e = true;
            }
            CommandType::G(92) => self.set_position(p),
            CommandType::M(82) => {
                self.drain();
                self.relative_e = false;
            }
            CommandType::M(83) => {
                self.drain();
                self.relative_e = true;
            }
            CommandType::M(104) => self.set_temperature(Heater::Hotend, p, false),
            CommandType::M(109) => self.set_temperature(Heater::Hotend, p, true),
            CommandType::M(140) => self.set_temperature(Heater::Bed, p, false),
            CommandType::M(190) => self.set_temperature(Heater::Bed, p, true),
            CommandType::M(106) => {
                self.drain();
                self.fan = (p.s.unwrap_or(255.0) as f64 / 255.0).clamp(0.0, 1.0);
            }
            CommandType::M(107) => {
                self.drain();
                self.fan = 0.0;
            }
            // Comments, messages, and settings with no effect on motion or heat
            CommandType::G(17 | 20 | 21 | 29) | CommandType::M(84 | 105 | 114 | 117 | 118 | 400) => {}
            CommandType::M(0 | 1 | 2 | 30) => self.drain(),
            other => {
                let name = match other {
                    CommandType::G(n) => format!("G{}", n),
                    CommandType::M(n) => format!("M{}", n),
                    CommandType::T(n) => format!("T{}", n),
                    CommandType::Other(text) => text.clone(),
                };
                *self.report.ignored_commands.entry(name).or_insert(0) += 1;
            }
        }
    }

    fn logical(&self, physical: &[f64; 3]) -> [f64; 3] {
        [physical[0] + self.offset[0], physical[1] + self.offset[1], physical[2] + self.offset[2]]
    }

    /// Work out the physical target of a move, refusing anything the machine must not do
    fn resolve_target(&mut self, words: [Option<f32>; 3]) -> Option<[f64; 3]> {
        let current = self.logical(&self.planned_end);
        let mut logical = current;
        for axis in 0..3 {
            if let Some(value) = words[axis] {
                logical[axis] = if self.relative_xyz { current[axis] + value as f64 } else { value as f64 };
            }
        }
        for axis in 0..3 {
            let moves = (logical[axis] - current[axis]).abs() > 1e-9;
            if moves && !self.axis_known[axis] {
                self.halt(format!("Line {}: moved {} before it was homed", self.current_line, ["X", "Y", "Z"][axis]));
                return None;
            }
        }
        let target = [logical[0] - self.offset[0], logical[1] - self.offset[1], logical[2] - self.offset[2]];
        for axis in 0..3 {
            if target[axis] < -0.001 || target[axis] > self.config.build_volume[axis] + 0.001 {
                self.halt(format!("Line {}: {} = {:.3} is outside the build volume", self.current_line, ["X", "Y", "Z"][axis], target[axis]));
                return None;
            }
        }
        if !self.kinematics.is_reachable(target[0] as f32, target[1] as f32, target[2] as f32, 0.0, 0.0) {
            self.halt(format!("Line {}: {:?} is out of reach", self.current_line, target));
            return None;
        }
        Some(target)
    }

    fn update_feed(&mut self, p: &Parameters) {
        if let Some(f) = p.f {
            if f > 0.0 {
                self.feed = f as f64 / 60.0;
            }
        }
    }

    fn extrusion_delta(&mut self, p: &Parameters) -> f64 {
        match p.e {
            Some(e) => {
                let new = if self.relative_e { self.e_counter + e as f64 } else { e as f64 };
                let delta = new - self.e_counter;
                self.e_counter = new;
                delta
            }
            None => 0.0,
        }
    }

    fn linear_move(&mut self, p: &Parameters) {
        self.update_feed(p);
        let Some(target) = self.resolve_target([p.x, p.y, p.z]) else { return };
        let e_delta = self.extrusion_delta(p);
        let queued = self.planner.queued_segments();
        self.planner.add_linear_move(self.planned_end, target, self.feed);
        self.after_planning(queued, target, e_delta);
    }

    fn arc_move(&mut self, p: &Parameters, clockwise: bool) {
        self.update_feed(p);
        let Some(target) = self.resolve_target([p.x, p.y, p.z]) else { return };
        let e_delta = self.extrusion_delta(p);
        let start = self.planned_end;
        let queued = self.planner.queued_segments();
        match (p.i, p.j) {
            (None, None) => {
                // The radius form is not modelled: the end point is still right
                *self.report.ignored_commands.entry("G2/G3 R form (drawn straight)".to_string()).or_insert(0) += 1;
                self.planner.add_linear_move(start, target, self.feed);
            }
            (i, j) => {
                let center = [start[0] + i.unwrap_or(0.0) as f64, start[1] + j.unwrap_or(0.0) as f64, start[2]];
                self.planner.add_arc_move(start, target, center, clockwise, self.feed);
            }
        }
        self.after_planning(queued, target, e_delta);
    }

    /// Queue what was just given to the planner, or for an extruder-only move, run it now
    fn after_planning(&mut self, queued_before: usize, target: [f64; 3], e_delta: f64) {
        let pending = PendingMove {
            e_delta,
            e_applied: 0.0,
            line: self.current_line,
            end_logical: self.logical(&target),
            end_e: self.e_counter,
            feed: self.feed,
            relative_e: self.relative_e,
        };
        self.report.moves += 1;

        if self.planner.queued_segments() > queued_before {
            self.planned_end = target;
            self.pending.push_back(pending);
            while self.pending.len() > LOOKAHEAD_MOVES && !self.stopped() {
                if !self.step_motion() {
                    break;
                }
            }
        } else {
            // No XYZ motion: wait for the head to arrive, then run the extruder
            self.drain();
            if !self.stopped() && e_delta != 0.0 {
                let seconds = e_delta.abs() / self.feed.min(MAX_EXTRUDER_SPEED);
                self.apply_extrusion(e_delta);
                self.wait(seconds);
            }
            if !self.stopped() {
                self.move_completed(pending);
            }
        }
    }

    fn apply_extrusion(&mut self, delta: f64) -> bool {
        if delta > 0.0 && self.hotend.plant.temperature < self.config.min_extrude_temp {
            self.report.cold_extrusions += 1;
            return false;
        }
        self.report.filament_net += delta;
        if delta > 0.0 {
            self.report.filament_extruded += delta;
        }
        delta > 0.0
    }

    /// One point of motion. Returns false when nothing more is planned.
    fn step_motion(&mut self) -> bool {
        let Some(point) = self.planner.get_next_point() else {
            return false;
        };
        let dt = (point.time - self.last_point_time).max(0.0);
        self.last_point_time = point.time;
        self.position = point.position;
        self.report.peak_speed = self.report.peak_speed.max(point.actual_feedrate);

        // Physically, the nozzle has moved first. So check for what it hit before depositing.
        self.check_collision();

        let mut extruding = false;
        if let Some(front) = self.pending.front_mut() {
            let total = front.e_delta * point.segment_fraction;
            let delta = total - front.e_applied;
            front.e_applied = total;
            extruding = self.apply_extrusion(delta);
        }
        if extruding {
            let cell = Self::cell(point.position[0], point.position[1]);
            let height = self.height_map.entry(cell).or_insert(0.0);
            *height = height.max(point.position[2]);
        }

        self.motors = self.kinematics.to_motor_positions(point.position[0] as f32, point.position[1] as f32, point.position[2] as f32, 0.0, 0.0);
        self.advance(dt);

        if point.segment_fraction >= 1.0 {
            if let Some(done) = self.pending.pop_front() {
                self.move_completed(done);
            }
        }
        true
    }

    fn check_collision(&mut self) {
        let [x, y, z] = self.position;
        if let Some(height) = self.height_map.get(&Self::cell(x, y)) {
            if z < height - COLLISION_TOLERANCE {
                self.report.collisions += 1;
                if self.report.first_collision.is_none() {
                    self.report.first_collision = Some(self.position);
                    let message = format!("COLLISION: nozzle at Z{:.3} is inside the print (height {:.3}) at X{:.1} Y{:.1}", z, height, x, y);
                    self.log(message);
                }
            }
        }
    }

    /// Only what has actually been executed is saved, not what has merely been planned. If
    /// a resume started from the planned position it would skip the moves in between.
    fn move_completed(&mut self, done: PendingMove) {
        let z_changed = self.completed.as_ref().map_or(true, |previous| previous.end_logical[2] != done.end_logical[2]);
        self.completed = Some(done);
        // Z changes save at once, at most once a simulated second, as on the real thing
        let z_save_due = z_changed && self.clock - self.last_z_save >= 1.0;
        if z_save_due {
            self.last_z_save = self.clock;
        }
        if z_save_due || self.clock - self.last_save >= self.config.recovery_save_interval {
            self.save_recovery_state();
        }
    }

    fn save_recovery_state(&mut self) {
        self.last_save = self.clock;
        let Some(done) = self.completed.clone() else { return };
        let Some(manager) = self.recovery.as_mut() else { return };
        let update = PrintStateUpdate {
            position: Some([done.end_logical[0], done.end_logical[1], done.end_logical[2], 0.0, 0.0, 0.0]),
            line_number: Some(done.line),
            hotend_temperatures: Some(HashMap::from([(0u8, self.hotend.controller.target_temperature)])),
            bed_temperature: Some(self.bed.controller.target_temperature),
            print_time: Some(Duration::from_secs_f64(self.clock)),
            extruder_position: Some(done.end_e),
            relative_extrusion: Some(done.relative_e),
            feedrate: Some(done.feed * 60.0),
            fan_speeds: Some(HashMap::from([(0u8, self.fan)])),
            ..Default::default()
        };
        if let Err(e) = manager.update_print_state(update) {
            self.report.warnings.push(format!("Could not save recovery state: {}", e));
        }
    }

    /// Finish all planned motion
    fn drain(&mut self) {
        while !self.stopped() && self.step_motion() {}
    }

    fn wait(&mut self, seconds: f64) {
        let mut remaining = seconds;
        while remaining > 0.0 && !self.stopped() {
            let step = remaining.min(THERMAL_STEP);
            self.advance(step);
            remaining -= step;
        }
    }

    fn home(&mut self, p: &Parameters) {
        self.drain();
        let named = |letter: char, word: Option<f32>| word.is_some() || p.flags.contains(&letter);
        let (x, y, z) = (named('X', p.x), named('Y', p.y), named('Z', p.z));
        let all = !x && !y && !z;
        // Order matters to what the nozzle hits on the way, as on a real machine.
        for (axis, wanted) in [(0, x || all), (1, y || all), (2, z || all)] {
            if !wanted {
                continue;
            }
            while self.position[axis] > 0.0 && !self.stopped() {
                let step = (HOMING_SPEED * 0.02).min(self.position[axis]);
                self.position[axis] -= step;
                self.check_collision();
                self.advance(0.02);
            }
            self.position[axis] = 0.0;
            self.offset[axis] = 0.0;
            self.axis_known[axis] = true;
        }
        self.planned_end = self.position;
    }

    fn set_position(&mut self, p: &Parameters) {
        self.drain();
        for (axis, word) in [p.x, p.y, p.z].into_iter().enumerate() {
            if let Some(value) = word {
                self.offset[axis] = value as f64 - self.position[axis];
                // Declaring a position is what recovery relies on, since it cannot home Z
                self.axis_known[axis] = true;
            }
        }
        if let Some(e) = p.e {
            self.e_counter = e as f64;
        }
        self.planned_end = self.position;
    }

    fn set_temperature(&mut self, heater: Heater, p: &Parameters, wait: bool) {
        self.drain();
        let limit = match heater {
            Heater::Hotend => self.config.hotend_max_temp,
            Heater::Bed => self.config.bed_max_temp,
        };
        if let Some(target) = p.s.or(p.r) {
            let target = target as f64;
            if !(0.0..=limit).contains(&target) {
                self.halt(format!("Line {}: temperature {} is outside 0..{}", self.current_line, target, limit));
                return;
            }
            self.zone_mut(heater).set_target(target);
            self.log(format!("{:?} target {} °C", heater, target));
        }
        if wait {
            self.wait_for_temperature(heater);
        }
    }

    fn wait_for_temperature(&mut self, heater: Heater) {
        let started = self.clock;
        let mut settled_since: Option<f64> = None;
        while !self.stopped() {
            let zone = self.zone_mut(heater);
            let target = zone.controller.target_temperature;
            if target <= 0.0 {
                return;
            }
            let reading = zone.controller.current_reading.filtered_temperature;
            if (reading - target).abs() <= TEMPERATURE_TOLERANCE {
                let since = *settled_since.get_or_insert(self.clock);
                if self.clock - since >= TEMPERATURE_SETTLE {
                    let message = format!("{:?} at temperature after {:.0} s", heater, self.clock - started);
                    self.log(message);
                    return;
                }
            } else {
                settled_since = None;
            }
            if self.clock - started > TEMPERATURE_WAIT_LIMIT {
                self.halt(format!("{:?} did not reach temperature in {} s", heater, TEMPERATURE_WAIT_LIMIT));
                return;
            }
            self.advance(THERMAL_STEP);
        }
    }
}

/// Parse G-code text with the same parser the application uses
pub fn parse_gcode(text: &str) -> Result<Vec<GCodeCommand>, String> {
    StreamingGCodeParser::new()
        .parse_file(Cursor::new(text.as_bytes().to_vec()))
        .map_err(|e| e.to_string())
}

/// Parse a fault description such as `hotend-sensor-open@120` (at 120 s) or `power-cut@line:500`
pub fn parse_fault(text: &str) -> Result<(FaultTrigger, Fault), String> {
    let (name, when) = text.split_once('@').ok_or_else(|| format!("{}: expected NAME@SECONDS or NAME@line:N", text))?;
    let trigger = if let Some(line) = when.strip_prefix("line:") {
        FaultTrigger::AtLine(line.parse().map_err(|_| format!("Bad line number: {}", line))?)
    } else {
        FaultTrigger::AtTime(when.parse().map_err(|_| format!("Bad time: {}", when))?)
    };
    let fault = match name {
        "hotend-sensor-open" => Fault::SensorOpen(Heater::Hotend),
        "hotend-sensor-short" => Fault::SensorShort(Heater::Hotend),
        "hotend-sensor-frozen" => Fault::SensorFrozen(Heater::Hotend),
        "hotend-heater-dead" => Fault::HeaterDead(Heater::Hotend),
        "hotend-heater-stuck" => Fault::HeaterStuckOn(Heater::Hotend),
        "bed-sensor-open" => Fault::SensorOpen(Heater::Bed),
        "bed-sensor-short" => Fault::SensorShort(Heater::Bed),
        "bed-sensor-frozen" => Fault::SensorFrozen(Heater::Bed),
        "bed-heater-dead" => Fault::HeaterDead(Heater::Bed),
        "bed-heater-stuck" => Fault::HeaterStuckOn(Heater::Bed),
        "power-cut" => Fault::PowerCut,
        "power-loss-detected" => Fault::PowerLossDetected,
        "estop" => Fault::EmergencyStop,
        other => return Err(format!("Unknown fault: {}", other)),
    };
    Ok((trigger, fault))
}

fn print_report(report: &SimReport) {
    println!("Outcome:          {:?}", report.outcome);
    println!("Simulated time:   {:.1} min", report.seconds / 60.0);
    println!("Lines run:        {}", report.lines_run);
    println!("Moves:            {}", report.moves);
    println!("Filament:         {:.1} mm extruded, {:.1} mm net", report.filament_extruded, report.filament_net);
    println!("Final position:   X{:.3} Y{:.3} Z{:.3}", report.final_position[0], report.final_position[1], report.final_position[2]);
    println!("Peak temperature: hotend {:.1} °C, bed {:.1} °C", report.peak_hotend, report.peak_bed);
    println!("Overshoot:        hotend {:.1} °C, bed {:.1} °C", report.hotend_overshoot, report.bed_overshoot);
    println!("Peak speed:       {:.1} mm/s", report.peak_speed);
    println!("Collisions:       {}", report.collisions);
    println!("Cold extrusions:  {}", report.cold_extrusions);
    if !report.ignored_commands.is_empty() {
        println!("Not simulated:    {:?}", report.ignored_commands);
    }
    println!("Events:");
    for (time, event) in &report.events {
        println!("  {:>9.1} s  {}", time, event);
    }
    for warning in &report.warnings {
        println!("WARNING: {}", warning);
    }
}

/// `printer-host --simulate FILE [--fault NAME@WHEN]... [--resume-after SECONDS]`
/// Returns the process exit code: 0 if the print ran clean.
pub fn run_cli(args: &[String]) -> i32 {
    let usage = "Usage: printer-host --simulate FILE [--fault NAME@SECONDS|NAME@line:N]... [--resume-after SECONDS]";
    let Some(path) = args.first() else {
        eprintln!("{}", usage);
        return 2;
    };

    let mut config = SimConfig::default();
    let mut resume_after = None;
    let mut rest = args[1..].iter();
    while let Some(arg) = rest.next() {
        match arg.as_str() {
            "--fault" => match rest.next().map(|text| parse_fault(text)) {
                Some(Ok(fault)) => config.faults.push(fault),
                Some(Err(e)) => {
                    eprintln!("{}", e);
                    return 2;
                }
                None => {
                    eprintln!("{}", usage);
                    return 2;
                }
            },
            "--resume-after" => match rest.next().and_then(|text| text.parse::<f64>().ok()) {
                Some(seconds) => resume_after = Some(seconds),
                None => {
                    eprintln!("{}", usage);
                    return 2;
                }
            },
            _ => {
                eprintln!("{}", usage);
                return 2;
            }
        }
    }

    let text = match std::fs::read_to_string(Path::new(path)) {
        Ok(text) => text,
        Err(e) => {
            eprintln!("Cannot read {}: {}", path, e);
            return 2;
        }
    };
    let commands = match parse_gcode(&text) {
        Ok(commands) => commands,
        Err(e) => {
            eprintln!("Cannot parse {}: {}", path, e);
            return 2;
        }
    };

    let recovery_path = std::env::temp_dir().join(format!("printer_sim_{}.plr", std::process::id()));
    if resume_after.is_some() || config.faults.iter().any(|(_, f)| matches!(f, Fault::PowerCut | Fault::PowerLossDetected)) {
        config.recovery = Some(RecoveryConfig {
            recovery_file_path: recovery_path.clone(),
            save_interval_ms: 0,
            recovery_timeout_hours: 1e6,
            ..RecoveryConfig::default()
        });
    }

    let mut simulator = Simulator::new(config);
    simulator.run(&commands);
    print_report(simulator.report());

    let mut clean = simulator.report().is_clean();
    if let (Some(off_seconds), Outcome::PowerLost) = (resume_after, simulator.report().outcome.clone()) {
        println!("\n--- Power back after {} s, resuming ---\n", off_seconds);
        let mut restarted = simulator.restart_after_power_loss(off_seconds);
        if let Err(e) = restarted.resume(&commands) {
            println!("Resume failed: {}", e);
        }
        print_report(restarted.report());
        clean = restarted.report().is_clean();
    }

    for extension in ["plr", "bak", "tmp", "stats"] {
        let _ = std::fs::remove_file(recovery_path.with_extension(extension));
    }
    if clean { 0 } else { 1 }
}
