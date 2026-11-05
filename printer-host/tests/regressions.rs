// Regression tests for safety-critical behaviour.
use printer_host::closed_loop_control::*;
use printer_host::input_shaping::*;
use printer_host::klipper_can::*;
use printer_host::power_loss_recovery::*;
use printer_host::pressure_advance::*;
use printer_host::smooth_trajectory::{MotionConstraints, MotionProfile, SmoothTrajectoryPlanner};
use printer_host::temperature_control::*;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

// ---------- Temperature ----------

fn heating_controller() -> TemperatureController {
    let mut controller = TemperatureController::new(
        TemperatureZone::Hotend(0),
        SensorConfig::default(),
        Some(HeaterConfig::default()),
    );
    controller.enable();
    controller.set_target_temperature(200.0);
    controller.update_reading(3900).unwrap(); // About 26C
    std::thread::sleep(Duration::from_millis(5));
    assert!(controller.calculate_output() > 0.0, "should be heating");
    controller
}

#[test]
fn sensor_faults_turn_the_heater_off_and_latch() {
    for raw in [4095u16, 0, 4094] {
        let mut controller = heating_controller();
        let _ = controller.update_reading(raw);
        std::thread::sleep(Duration::from_millis(2));
        assert_eq!(controller.calculate_output(), 0.0, "raw {raw}");
        assert!(controller.safety_shutdown, "raw {raw}");
        assert!(controller.last_error.is_some(), "raw {raw}");

        // A good reading must not restart the heater by itself
        controller.update_reading(3900).unwrap();
        std::thread::sleep(Duration::from_millis(2));
        assert_eq!(controller.calculate_output(), 0.0, "raw {raw}: latch released");
    }
}

#[test]
fn over_temperature_latches() {
    let mut controller = heating_controller();
    controller.update_reading(20).unwrap(); // Very low resistance: far above 300C
    assert!(controller.safety_shutdown);
    assert_eq!(controller.calculate_output(), 0.0);
}

#[test]
fn stale_readings_shut_the_heater_off() {
    let mut controller = heating_controller();
    controller.heater_config.as_mut().unwrap().safety_timeout = Duration::from_millis(10);
    std::thread::sleep(Duration::from_millis(30));
    assert_eq!(controller.calculate_output(), 0.0);
    assert!(controller.safety_shutdown);
}

#[test]
fn heater_that_does_not_heat_is_caught() {
    let mut controller = heating_controller();
    controller.runaway_config.watch_period = Duration::from_millis(20);
    controller.calculate_output();
    for _ in 0..5 {
        std::thread::sleep(Duration::from_millis(10));
        controller.update_reading(3900).unwrap(); // Temperature never changes
        controller.calculate_output();
    }
    assert!(controller.safety_shutdown);
    assert!(controller.last_error.as_deref().unwrap().contains("Heating failed"), "{:?}", controller.last_error);
}

#[test]
fn heater_rising_normally_is_not_a_fault() {
    let mut controller = heating_controller();
    controller.runaway_config.watch_period = Duration::from_millis(60);
    for raw in [3700u16, 3400, 3100, 2800, 2500, 2200] {
        std::thread::sleep(Duration::from_millis(10));
        controller.update_reading(raw).unwrap();
        controller.calculate_output();
    }
    assert!(!controller.safety_shutdown, "{:?}", controller.last_error);
}

#[test]
fn losing_temperature_at_target_is_caught() {
    let mut controller = heating_controller();
    controller.runaway_config.hold_period = Duration::from_millis(20);
    controller.current_reading.filtered_temperature = 200.0;
    controller.current_reading.timestamp = Instant::now();
    controller.calculate_output(); // Reaches target
    for _ in 0..4 {
        std::thread::sleep(Duration::from_millis(10));
        controller.current_reading.filtered_temperature = 150.0; // Fell off, and stays off
        controller.current_reading.timestamp = Instant::now();
        controller.calculate_output();
    }
    assert!(controller.safety_shutdown);
    assert!(controller.last_error.as_deref().unwrap().contains("Thermal runaway"), "{:?}", controller.last_error);
}

#[test]
fn pid_does_not_hold_the_heater_on_above_target() {
    let mut controller = heating_controller();
    controller.feedforward_model.enabled = false;
    controller.pid_state.integral = 1000.0; // However wound up it got
    controller.current_reading.filtered_temperature = 210.0;
    controller.current_reading.timestamp = Instant::now();
    controller.pid_state.initialized = false;
    std::thread::sleep(Duration::from_millis(5));
    let duty = controller.calculate_output();
    assert!(duty < 1.0, "duty {duty} at 10C over target");
}

#[test]
fn target_changes_reset_pid_and_are_limited() {
    let mut controller = heating_controller();
    controller.pid_state.integral = 77.0;
    controller.set_target_temperature(50.0);
    assert_eq!(controller.pid_state.integral, 0.0);

    controller.set_target_temperature(9999.0);
    assert!(controller.target_temperature < controller.sensor_config.max_temp);
    controller.set_target_temperature(f64::NAN);
    assert_eq!(controller.target_temperature, 0.0);

    let mut limited = heating_controller();
    limited.heater_config.as_mut().unwrap().max_pwm_duty = 0.3;
    std::thread::sleep(Duration::from_millis(2));
    assert!(limited.calculate_output() <= 0.3);
}

#[test]
fn emergency_stop_commands_zero_and_stays_off() {
    let zone = TemperatureZone::Hotend(0);
    let mut manager = TemperatureManager::new(1000);
    manager.add_controller(zone.clone(), SensorConfig::default(), Some(HeaterConfig::default())).unwrap();
    manager.enable_global();
    manager.enable_zone(&zone).unwrap();
    manager.set_target_temperature(&zone, 200.0).unwrap();
    manager.update_reading(&zone, 3900).unwrap();
    std::thread::sleep(Duration::from_millis(5));
    assert!(manager.process_update().unwrap()[&zone] > 0.0);

    manager.emergency_stop();
    // Immediately, with no waiting for the next update tick
    assert_eq!(manager.process_update().unwrap()[&zone], 0.0);

    manager.clear_emergency_stop();
    std::thread::sleep(Duration::from_millis(5));
    assert_eq!(manager.process_update().unwrap()[&zone], 0.0, "must not restart on its own");
}

#[test]
fn mock_sensor_reads_back_what_it_simulates() {
    let mut sensor = MockTemperatureSensor::new(20.0);
    sensor.add_noise(0.0);
    let mut controller = TemperatureController::new(TemperatureZone::Bed, SensorConfig::default(), None);
    controller.update_reading(sensor.read_raw().unwrap()).unwrap();
    assert!((controller.current_reading.temperature - 20.0).abs() < 1.0, "{}", controller.current_reading.temperature);
}

// ---------- Power loss recovery ----------

fn recovery_config(name: &str) -> RecoveryConfig {
    let mut config = RecoveryConfig::default();
    let path = std::env::temp_dir().join(format!("regression_{name}_{}.plr", std::process::id()));
    for extension in ["plr", "bak", "tmp", "stats"] {
        let _ = std::fs::remove_file(path.with_extension(extension));
    }
    config.recovery_file_path = path;
    config.save_interval_ms = 0;
    config
}

fn tracked_print(config: &RecoveryConfig) -> PowerLossRecoveryManager {
    let mut manager = PowerLossRecoveryManager::new(config.clone()).unwrap();
    manager.start_print_tracking("x.gcode".into(), 1000).unwrap();
    manager.update_print_state(PrintStateUpdate {
        position: Some([50.0, 60.0, 10.0, 0.0, 0.0, 0.0]),
        hotend_temperatures: Some(HashMap::from([(0u8, 210.0)])),
        bed_temperature: Some(60.0),
        extruder_position: Some(1234.5),
        fan_speeds: Some(HashMap::from([(0u8, 0.5)])),
        ..Default::default()
    }).unwrap();
    manager
}

fn plan_after_restart(config: &RecoveryConfig) -> RecoveryPlan {
    let mut manager = PowerLossRecoveryManager::new(config.clone()).unwrap();
    let state = manager.load_recovery_data().unwrap().unwrap();
    manager.start_recovery(state).unwrap()
}

fn all_commands(plan: &RecoveryPlan) -> Vec<String> {
    plan.steps.iter().flat_map(|step| step.gcode_commands.clone()).collect()
}

fn position_of(commands: &[String], needle: &str) -> usize {
    commands.iter().position(|c| c.contains(needle)).unwrap_or_else(|| panic!("{needle} not in {commands:?}"))
}

fn check_safe_order(commands: &[String]) {
    // Reheat, raise Z, and only then move XY. Back down last.
    let wait_bed = position_of(commands, "M190 S60");
    let wait_hotend = position_of(commands, "M109 S210 T0");
    let raise = position_of(commands, "G1 Z");
    let move_xy = position_of(commands, "G1 X50.000 Y60.000");
    let lower = position_of(commands, "G1 Z10.000");
    assert!(wait_bed < wait_hotend && wait_hotend < raise, "{commands:?}");
    assert!(raise < move_xy && move_xy < lower, "{commands:?}");
    assert!(!commands.iter().any(|c| c == "G28"), "Z must not be homed into the print: {commands:?}");
    // E must be restored before printing resumes
    assert!(position_of(commands, "G92 E1234.5") < move_xy, "{commands:?}");
    assert!(position_of(commands, "M106 P0 S128") > 0);
}

#[test]
fn detected_power_loss_keeps_what_is_needed_to_resume() {
    let config = recovery_config("detected");
    let mut manager = tracked_print(&config);
    std::thread::sleep(Duration::from_millis(30));
    assert!(manager.monitor_power(8.0).unwrap());

    // Later updates do not overwrite the saved moment
    manager.update_print_state(PrintStateUpdate { line_number: Some(999), ..Default::default() }).unwrap();

    let commands = all_commands(&plan_after_restart(&config));
    check_safe_order(&commands);
    assert_eq!(position_of(&commands, "G92 Z10.000"), position_of(&commands, "G92 Z10"), "{commands:?}");
}

#[test]
fn sudden_power_cut_never_drives_into_the_print() {
    let config = recovery_config("sudden");
    drop(tracked_print(&config));
    let commands = all_commands(&plan_after_restart(&config));
    check_safe_order(&commands);
    // The lowest Z ever commanded is the height of the print
    for command in commands.iter().filter(|c| c.starts_with("G1 Z")) {
        let z: f64 = command[4..].split_whitespace().next().unwrap().parse().unwrap();
        assert!(z >= 10.0, "{command}");
    }
}

#[test]
fn confirmed_lift_is_accounted_for() {
    let config = recovery_config("lift");
    let mut manager = tracked_print(&config);
    std::thread::sleep(Duration::from_millis(30));
    manager.monitor_power(8.0).unwrap();
    assert!(manager.emergency_gcode().iter().any(|c| c.contains("G1 Z5.000")));
    manager.confirm_emergency_lift().unwrap();
    let commands = all_commands(&plan_after_restart(&config));
    assert!(commands.iter().any(|c| c == "G92 Z15.000"), "{commands:?}");
    assert!(commands.iter().any(|c| c.starts_with("G1 Z10.000")), "{commands:?}");
}

#[test]
fn recovery_data_loads_reliably() {
    for i in 0..300u32 {
        let config = recovery_config(&format!("load{i}"));
        let mut manager = PowerLossRecoveryManager::new(config.clone()).unwrap();
        manager.start_print_tracking("x.gcode".into(), 1000).unwrap();
        let f = 1.0 + (i as f64) * 0.123456789123;
        manager.update_print_state(PrintStateUpdate {
            position: Some([f, f * 1.7, f / 3.0, 0.0, 0.0, 0.0]),
            hotend_temperatures: Some(HashMap::from([(0u8, 210.0), (1u8, 220.0), (2u8, 230.0), (3u8, 240.0)])),
            ..Default::default()
        }).unwrap();
        let mut restarted = PowerLossRecoveryManager::new(config.clone()).unwrap();
        restarted.load_recovery_data().unwrap_or_else(|e| panic!("{i}: {e}")).unwrap();
        let _ = std::fs::remove_file(&config.recovery_file_path);
        let _ = std::fs::remove_file(config.recovery_file_path.with_extension("bak"));
    }
}

#[test]
fn corrupted_recovery_data_is_refused() {
    let config = recovery_config("corrupt");
    drop(tracked_print(&config));
    let mut bytes = std::fs::read(&config.recovery_file_path).unwrap();
    let last = bytes.len() - 5;
    bytes[last] ^= 0x01;
    std::fs::write(&config.recovery_file_path, bytes).unwrap();
    let mut manager = PowerLossRecoveryManager::new(config).unwrap();
    assert!(manager.load_recovery_data().is_err());
}

#[test]
fn finished_print_leaves_nothing_to_resume() {
    let config = recovery_config("finished");
    let mut manager = tracked_print(&config);
    assert!(manager.has_recovery_data());
    manager.stop_print_tracking().unwrap();
    assert!(!manager.has_recovery_data());
    assert!(!config.recovery_file_path.with_extension("bak").exists());
}

#[test]
fn recovery_attempts_are_limited() {
    let config = recovery_config("attempts");
    drop(tracked_print(&config));
    for attempt in 0..config.max_recovery_attempts {
        plan_after_restart(&config);
        assert!(attempt < config.max_recovery_attempts);
    }
    let mut manager = PowerLossRecoveryManager::new(config.clone()).unwrap();
    let state = manager.load_recovery_data().unwrap().unwrap();
    assert!(manager.start_recovery(state).is_err());
}

// ---------- Input shaping ----------

fn step_response(shaper_type: ShaperType) -> (InputShaper, Vec<(f64, f64)>) {
    let mut shaper = InputShaper::new(InputShaperConfig { shaper_type, frequency: 40.0, damping_ratio: 0.1, ..Default::default() });
    let mut output = Vec::new();
    for ms in 0..200 {
        let t = ms as f64 / 1000.0;
        let position = if ms < 20 { 0.0 } else { 100.0 };
        let shaped = shaper.shape_command(MotionCommand { timestamp: t, position, velocity: 0.0, acceleration: 0.0, axis: 'X' });
        assert_eq!(shaped.len(), 1);
        output.push((t, shaped[0].position));
    }
    (shaper, output)
}

#[test]
fn shapers_end_where_commanded_and_stay_in_range() {
    for shaper_type in [ShaperType::ZV, ShaperType::MZV, ShaperType::ZVD, ShaperType::EI, ShaperType::EI2, ShaperType::EI3, ShaperType::TwoHumpEI, ShaperType::ThreeHumpEI] {
        let (shaper, output) = step_response(shaper_type);
        let total: f64 = shaper.impulses().iter().map(|i| i.0).sum();
        assert!((total - 1.0).abs() < 1e-9, "{shaper_type:?} weights sum to {total}");
        assert_eq!(shaper.get_config().shaper_type, shaper_type, "must not change the user's setting");
        assert!((output.last().unwrap().1 - 100.0).abs() < 1e-9, "{shaper_type:?} ended at {:?}", output.last());
        assert_eq!(output[0].1, 0.0);
        assert!(output.iter().all(|(_, p)| *p >= -1e-9 && *p <= 100.0 + 1e-9), "{shaper_type:?} overshoots");
    }
}

#[test]
fn shapers_cancel_vibration_at_the_design_frequency() {
    let (frequency, damping): (f64, f64) = (40.0, 0.1);
    let omega = 2.0 * std::f64::consts::PI * frequency;
    let omega_d = omega * (1.0 - damping * damping).sqrt();
    for (shaper_type, limit) in [(ShaperType::ZV, 0.001), (ShaperType::ZVD, 0.001), (ShaperType::MZV, 0.10), (ShaperType::EI, 0.06), (ShaperType::TwoHumpEI, 0.06), (ShaperType::ThreeHumpEI, 0.06)] {
        let shaper = InputShaper::new(InputShaperConfig { shaper_type, frequency, damping_ratio: damping, ..Default::default() });
        let impulses = shaper.impulses();
        let end = impulses.iter().map(|i| i.1).fold(0.0, f64::max);
        let (mut re, mut im) = (0.0, 0.0);
        for (weight, time) in &impulses {
            let decay = (-damping * omega * (end - time)).exp();
            re += weight * decay * (omega_d * time).cos();
            im += weight * decay * (omega_d * time).sin();
        }
        let residual = (re * re + im * im).sqrt();
        assert!(residual < limit, "{shaper_type:?} leaves {:.1}%", residual * 100.0);
    }
}

#[test]
fn position_path_used_by_the_app_is_not_scaled() {
    let mut manager = InputShaperManager::new();
    manager.add_axis('X', InputShaperConfig::default());
    for _ in 0..5 {
        let shaped = manager.process_command('X', 100.0).unwrap();
        assert!((shaped - 100.0).abs() < 1e-3, "{shaped}");
        std::thread::sleep(Duration::from_millis(10));
    }
}

#[test]
fn smoothing_filter_converges() {
    let mut shaper = InputShaper::new(InputShaperConfig { shaper_type: ShaperType::Smoothing, smoothing_time: 0.01, ..Default::default() });
    let mut last = 0.0;
    shaper.shape_command(MotionCommand { timestamp: 0.0, position: 0.0, velocity: 0.0, acceleration: 0.0, axis: 'X' });
    for ms in 1..200 {
        last = shaper.shape_command(MotionCommand { timestamp: ms as f64 / 1000.0, position: 50.0, velocity: 0.0, acceleration: 0.0, axis: 'X' })[0].position;
    }
    assert!((last - 50.0).abs() < 1e-3, "{last}");
}

#[test]
fn invalid_shaper_settings_pass_through() {
    for (frequency, damping_ratio) in [(0.0, 0.1), (f64::NAN, 0.1), (40.0, 1.0), (40.0, -0.1)] {
        let shaper = InputShaper::new(InputShaperConfig { frequency, damping_ratio, ..Default::default() });
        assert_eq!(shaper.impulses(), vec![(1.0, 0.0)]);
    }
}

// ---------- Trajectory planning ----------

fn drain(planner: &mut SmoothTrajectoryPlanner) -> Vec<printer_host::smooth_trajectory::MotionPoint> {
    let mut points = Vec::new();
    while let Some(point) = planner.get_next_point() {
        points.push(point);
    }
    points
}

fn check_limits(points: &[printer_host::smooth_trajectory::MotionPoint], constraints: &MotionConstraints, feedrate: f64) {
    let dt = points[1].time - points[0].time;
    assert!(dt > 0.0);
    for pair in points.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        assert!(b.time > a.time, "time went backwards");
        let step: f64 = (0..3).map(|i| (b.position[i] - a.position[i]).powi(2)).sum::<f64>().sqrt();
        let speed = step / (b.time - a.time);
        assert!(speed <= feedrate * 1.02, "moved at {speed}, limit {feedrate}");
        assert!(b.actual_feedrate <= feedrate * 1.001);
        let accel = (b.actual_feedrate - a.actual_feedrate).abs() / (b.time - a.time);
        assert!(accel <= constraints.max_acceleration * 1.05, "accelerated at {accel}");
    }
}

#[test]
fn single_move_runs_with_real_acceleration() {
    for profile in [MotionProfile::Trapezoidal, MotionProfile::SCurve, MotionProfile::BSpline] {
        let constraints = MotionConstraints::default();
        let mut planner = SmoothTrajectoryPlanner::new(constraints.clone(), profile);
        planner.add_linear_move([0.0; 3], [100.0, 0.0, 0.0], 100.0);
        assert!(planner.has_planned_points());
        let points = drain(&mut planner);
        check_limits(&points, &constraints, 100.0);
        let last = points.last().unwrap();
        assert_eq!(last.position, [100.0, 0.0, 0.0]);
        assert_eq!(last.actual_feedrate, 0.0);
        assert!(points.iter().any(|p| (p.actual_feedrate - 100.0).abs() < 1e-6), "never reached feedrate");
        assert!(!planner.has_planned_points());
    }
}

#[test]
fn short_moves_stay_within_limits() {
    for profile in [MotionProfile::Trapezoidal, MotionProfile::SCurve] {
        let constraints = MotionConstraints::default();
        let mut planner = SmoothTrajectoryPlanner::new(constraints.clone(), profile);
        planner.add_linear_move([0.0; 3], [1.0, 0.0, 0.0], 100.0);
        planner.add_linear_move([1.0, 0.0, 0.0], [2.0, 0.0, 0.0], 100.0);
        planner.add_linear_move([2.0, 0.0, 0.0], [3.0, 0.0, 0.0], 100.0);
        let points = drain(&mut planner);
        check_limits(&points, &constraints, 100.0);
        // Collinear moves flow through the junctions without stopping
        let at_junction = points.iter().find(|p| p.position[0] >= 1.0).unwrap();
        assert!(at_junction.actual_feedrate > 10.0, "{}", at_junction.actual_feedrate);
        assert_eq!(points.last().unwrap().position, [3.0, 0.0, 0.0]);
    }
}

#[test]
fn nothing_is_replayed_when_moves_are_added() {
    let mut planner = SmoothTrajectoryPlanner::new(MotionConstraints::default(), MotionProfile::Trapezoidal);
    planner.add_linear_move([0.0; 3], [10.0, 0.0, 0.0], 100.0);
    planner.add_linear_move([10.0, 0.0, 0.0], [20.0, 0.0, 0.0], 100.0);
    let mut last = planner.get_next_point().unwrap();
    for _ in 0..50 {
        last = planner.get_next_point().unwrap();
    }
    planner.add_linear_move([20.0, 0.0, 0.0], [30.0, 0.0, 0.0], 100.0);
    let mut x = last.position[0];
    let mut t = last.time;
    while let Some(point) = planner.get_next_point() {
        assert!(point.position[0] >= x - 1e-9, "went back from {x} to {}", point.position[0]);
        assert!(point.time > t);
        x = point.position[0];
        t = point.time;
    }
    assert_eq!(x, 30.0);
}

#[test]
fn corners_are_taken_slowly_and_reversals_stop() {
    let constraints = MotionConstraints::default();

    // Full reversal
    let mut planner = SmoothTrajectoryPlanner::new(constraints.clone(), MotionProfile::Trapezoidal);
    planner.add_linear_move([0.0; 3], [50.0, 0.0, 0.0], 200.0);
    planner.add_linear_move([50.0, 0.0, 0.0], [0.0, 0.0, 0.0], 200.0);
    let points = drain(&mut planner);
    check_limits(&points, &constraints, 200.0);
    let tip = points.iter().min_by(|a, b| (a.position[0] - 50.0).abs().total_cmp(&(b.position[0] - 50.0).abs())).unwrap();
    assert!(tip.actual_feedrate < 1.0, "{}", tip.actual_feedrate);

    // Right angle: between stopped and full speed
    let mut planner = SmoothTrajectoryPlanner::new(constraints.clone(), MotionProfile::Trapezoidal);
    planner.add_linear_move([0.0; 3], [50.0, 0.0, 0.0], 200.0);
    planner.add_linear_move([50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 200.0);
    let points = drain(&mut planner);
    check_limits(&points, &constraints, 200.0);
    let corner = points.iter().min_by(|a, b| {
        let d = |p: &printer_host::smooth_trajectory::MotionPoint| (p.position[0] - 50.0).abs() + p.position[1].abs();
        d(a).total_cmp(&d(b))
    }).unwrap();
    assert!(corner.actual_feedrate > 1.0 && corner.actual_feedrate < 50.0, "{}", corner.actual_feedrate);
}

#[test]
fn arcs_follow_the_circle() {
    let constraints = MotionConstraints::default();

    // Quarter circle, with the rounding real G-code has. Counter-clockwise goes straight there.
    let mut planner = SmoothTrajectoryPlanner::new(constraints.clone(), MotionProfile::SCurve);
    planner.add_arc_move([10.0, 0.0, 0.0], [0.001, 10.002, 0.0], [0.0, 0.0, 0.0], false, 50.0);
    let points = drain(&mut planner);
    assert!(points.len() > 100);
    for point in &points {
        let radius = point.position[0].hypot(point.position[1]);
        assert!((radius - 10.0).abs() < 0.01, "radius {radius}");
        assert!(point.position[0] >= -0.01 && point.position[1] >= -0.01);
    }

    // Clockwise between the same points is the long way round
    let mut planner = SmoothTrajectoryPlanner::new(constraints.clone(), MotionProfile::Trapezoidal);
    planner.add_arc_move([10.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, 0.0, 0.0], true, 50.0);
    let points = drain(&mut planner);
    assert!(points.iter().any(|p| p.position[0] < -5.0 && p.position[1] < -5.0));
    let length: f64 = points.windows(2).map(|w| (w[1].position[0] - w[0].position[0]).hypot(w[1].position[1] - w[0].position[1])).sum();
    assert!((length - 47.12).abs() < 1.0, "{length}");

    // Full circle
    let mut planner = SmoothTrajectoryPlanner::new(constraints.clone(), MotionProfile::Trapezoidal);
    planner.add_arc_move([10.0, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 0.0, 0.0], false, 50.0);
    let points = drain(&mut planner);
    assert!(points.iter().all(|p| p.position.iter().all(|v| v.is_finite())));
    let length: f64 = points.windows(2).map(|w| (w[1].position[0] - w[0].position[0]).hypot(w[1].position[1] - w[0].position[1])).sum();
    assert!((length - 62.83).abs() < 1.0, "{length}");
}

#[test]
fn bad_feedrate_is_ignored() {
    let mut planner = SmoothTrajectoryPlanner::new(MotionConstraints::default(), MotionProfile::Trapezoidal);
    planner.add_linear_move([0.0; 3], [10.0, 0.0, 0.0], 0.0);
    planner.add_linear_move([0.0; 3], [10.0, 0.0, 0.0], f64::NAN);
    planner.add_arc_move([10.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0; 3], false, -1.0);
    assert!(!planner.has_planned_points());
}

// ---------- Pressure advance ----------

fn segment(extrude_velocity: f64) -> ExtrusionSegment {
    ExtrusionSegment {
        start_pos: [0.0; 4],
        end_pos: [1.0, 0.0, 0.0, 0.1],
        start_velocity: 50.0,
        end_velocity: 50.0,
        extrude_velocity,
        segment_time: 0.02,
        pressure_advance: 0.05,
        timestamp: Instant::now(),
    }
}

#[test]
fn advance_is_proportional_to_flow_and_follows_it() {
    let mut controller = PressureAdvanceController::new(PressureAdvanceConfig { pressure_advance: 0.05, ..Default::default() });
    for _ in 0..50 {
        controller.add_segment(segment(5.0));
    }
    assert!((controller.get_adjusted_extruder_position() - 0.25).abs() < 1e-3);
    assert_eq!(controller.get_statistics().segments_processed, 50, "each segment counted once");

    for _ in 0..50 {
        controller.add_segment(segment(20.0));
    }
    assert!((controller.get_adjusted_extruder_position() - 1.0).abs() < 1e-3);

    // Retraction builds no pressure
    let mut retract = segment(5.0);
    retract.end_pos[3] = -1.0;
    for _ in 0..50 {
        controller.add_segment(retract.clone());
    }
    assert!(controller.get_adjusted_extruder_position().abs() < 1e-3);

    assert!(controller.auto_calibrate(&[segment(5.0)]).is_err());
}

#[test]
fn extruders_default_to_no_advance() {
    let mut multi = MultiExtruderPressureAdvance::new(2);
    assert_eq!(multi.get_controller(1).unwrap().get_config().pressure_advance, 0.0);
    let mut none = MultiExtruderPressureAdvance::new(0);
    none.get_active_controller();
    assert!(none.set_active_extruder(5).is_err());
}

// ---------- CAN ----------

struct CapturingInterface(Arc<Mutex<Vec<(u32, Vec<u8>)>>>, Arc<Mutex<Vec<(u32, Vec<u8>)>>>);

impl CanBusInterface for CapturingInterface {
    fn send_frame(&mut self, id: u32, data: &[u8]) -> Result<(), String> {
        self.0.lock().unwrap().push((id, data.to_vec()));
        Ok(())
    }
    fn receive_frame(&mut self) -> Option<(u32, Vec<u8>)> {
        let mut queue = self.1.lock().unwrap();
        if queue.is_empty() { None } else { Some(queue.remove(0)) }
    }
    fn set_bitrate(&mut self, _: u32) -> Result<(), String> { Ok(()) }
    fn set_filter(&mut self, _: u32, _: u32) -> Result<(), String> { Ok(()) }
    fn get_error_count(&self) -> (u32, u32) { (0, 0) }
}

/// A manager with one discovered toolhead, reporting the given temperature
fn can_manager(temperature: f32) -> (KlipperCanManager, Arc<Mutex<Vec<(u32, Vec<u8>)>>>, u32) {
    let uuid = 0x123456;
    let sent = Arc::new(Mutex::new(Vec::new()));
    let incoming = Arc::new(Mutex::new(Vec::new()));
    let mut manager = KlipperCanManager::new(Box::new(CapturingInterface(sent.clone(), incoming.clone())));
    let mut temperature_reply = vec![0x80];
    temperature_reply.extend_from_slice(&temperature.to_le_bytes());
    incoming.lock().unwrap().push(((uuid << 5) | 1, vec![0x80]));
    assert_eq!(manager.discover_toolheads().unwrap(), vec![uuid]);
    incoming.lock().unwrap().push(((uuid << 5) | 1, temperature_reply));
    manager.process_messages();
    sent.lock().unwrap().clear();
    (manager, sent, uuid)
}

#[test]
fn can_frames_are_valid_and_decodable() {
    let (mut manager, sent, uuid) = can_manager(220.0);
    manager.emergency_stop(uuid).unwrap();
    manager.set_temperature(uuid, 200.0).unwrap();
    manager.move_extruder(uuid, 5.0, 40.0).unwrap();
    let frames = sent.lock().unwrap().clone();
    assert_eq!(frames.len(), 3);
    for (id, data) in &frames {
        assert!(*id <= 0x1FFF_FFFF, "0x{id:X} does not fit in a 29-bit CAN ID");
        assert_eq!((id >> 5) & 0xFFFFFF, uuid);
        assert!(data.len() <= 8);
    }
    let extrude = &frames[2].1;
    assert_eq!(f32::from_le_bytes([extrude[1], extrude[2], extrude[3], extrude[4]]), 5.0);
    assert_eq!(u16::from_le_bytes([extrude[5], extrude[6]]), 4000, "40 mm/s in 0.01 mm/s");
}

#[test]
fn can_commands_respect_toolhead_limits() {
    let (mut manager, sent, uuid) = can_manager(100.0);
    assert!(manager.set_temperature(uuid, 9999.0).is_err());
    assert!(manager.set_temperature(uuid, f32::NAN).is_err());
    assert!(manager.move_extruder(uuid, 5.0, 40.0).is_err(), "100C is too cold to extrude");
    assert!(manager.move_extruder(uuid, -1.0, 40.0).is_ok(), "retracting is fine");
    assert!(manager.move_extruder(uuid, -1.0, 9999.0).is_err());
    assert!(manager.send_toolhead_command(uuid, KlipperCanCmd::Move, &[0u8; 8]).is_err());
    assert_eq!(sent.lock().unwrap().len(), 1);
}

// ---------- Closed loop ----------

fn encoder(invert: bool, threshold: f64) -> EncoderConfig {
    EncoderConfig {
        encoder_type: EncoderType::Quadrature,
        resolution: 4000,
        interface: "GPIO".into(),
        pin_config: EncoderPinConfig::default(),
        invert_direction: invert,
        counts_per_mm: 100.0,
        max_error_threshold: threshold,
        filter_samples: 1,
        update_rate_hz: 1000,
    }
}

#[test]
fn velocity_is_measured_over_one_sample() {
    let mut axis = ClosedLoopAxis::new("X".into(), encoder(false, 1000.0));
    let mut measured = Vec::new();
    for i in 1..=6 {
        std::thread::sleep(Duration::from_millis(50));
        axis.update_encoder_position(i * 100).unwrap();
        measured.push(axis.encoder_state.velocity);
    }
    for velocity in &measured[1..] {
        assert!(*velocity > 1000.0 && *velocity < 2100.0, "{velocity} (100 counts per 50ms is 2000/s)");
    }
}

#[test]
fn encoder_direction_setting_is_used() {
    let mut axis = ClosedLoopAxis::new("X".into(), encoder(true, 1000.0));
    std::thread::sleep(Duration::from_millis(2));
    axis.update_encoder_position(500).unwrap();
    assert_eq!(axis.current_position, -5.0);
}

#[test]
fn homing_survives_the_next_reading() {
    let mut axis = ClosedLoopAxis::new("X".into(), encoder(false, 1.0));
    std::thread::sleep(Duration::from_millis(2));
    let _ = axis.update_encoder_position(5000);
    axis.home_axis().unwrap();
    std::thread::sleep(Duration::from_millis(2));
    axis.update_encoder_position(5000).unwrap();
    assert_eq!(axis.current_position, 0.0);
    std::thread::sleep(Duration::from_millis(2));
    axis.update_encoder_position(5050).unwrap();
    assert_eq!(axis.current_position, 0.5);
}

#[test]
fn no_correction_without_a_live_encoder() {
    let mut axis = ClosedLoopAxis::new("X".into(), encoder(false, 1000.0));
    axis.home_axis().unwrap();
    axis.enable_closed_loop();
    axis.set_target_position(5.0);
    std::thread::sleep(Duration::from_millis(2));
    assert_eq!(axis.calculate_correction(), 0.0);
    axis.update_encoder_position(0).unwrap();
    std::thread::sleep(Duration::from_millis(2));
    assert!(axis.calculate_correction() > 0.0);
}

#[test]
fn plugins_in_the_allowed_directory_validate() {
    let unique = format!("plugins/regression_{}", std::process::id());
    std::fs::create_dir_all(&unique).unwrap();
    let security = printer_host::plugin_system::PluginSecurityManager::new();
    let inside = security.validate_plugin_path(std::path::Path::new(&unique));
    let outside = security.validate_plugin_path(std::path::Path::new("src"));
    let _ = std::fs::remove_dir(&unique);
    let _ = std::fs::remove_dir("plugins");
    assert!(inside.is_ok());
    assert!(outside.is_err());

    // A manifest cannot grant itself control of the machine
    use printer_host::plugin_system::Permission;
    let requested = vec![Permission::ReadConfig, Permission::AccessMotion, Permission::SystemControl];
    assert_eq!(security.permissions_to_grant("p", &requested), vec![Permission::ReadConfig]);
    security.approve_plugin("p");
    assert_eq!(security.permissions_to_grant("p", &requested), requested);
}
