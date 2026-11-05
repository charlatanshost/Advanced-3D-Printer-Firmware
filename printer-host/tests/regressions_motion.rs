// Regression tests for the motion, motor, torque and protocol modules.
use printer_host::advanced_motion::*;
use printer_host::closed_loop_control::ClosedLoopManager;
use printer_host::linuxcnc_control::*;
use printer_host::motor_assignment::*;
use printer_host::runtime_config::*;
use printer_host::torque_control::*;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

// ---------- Lookahead planner ----------

fn segment(start: [f32; 2], end: [f32; 2], speed: f32) -> MotionSegment {
    let distance = (end[0] - start[0]).hypot(end[1] - start[1]);
    MotionSegment {
        start_pos: [start[0], start[1], 0.0, 0.0, 0.0],
        end_pos: [end[0], end[1], 0.0, 0.0, 0.0],
        start_velocity: 0.0,
        end_velocity: 0.0,
        max_velocity: speed,
        acceleration: 3000.0,
        deceleration: 3000.0,
        distance,
        duration: 0.0,
        extrusion_rate: 0.0,
        line_width: 0.4,
        layer_height: 0.2,
        segment_type: SegmentType::Travel,
    }
}

fn planner_with(segments: &[([f32; 2], [f32; 2])]) -> TrajectoryPlanner {
    let mut planner = TrajectoryPlanner::new(VolumetricSettings::default());
    for (start, end) in segments {
        assert!(planner.add_segment(segment(*start, *end, 100.0)));
    }
    planner
}

#[test]
fn junction_speeds_join_up_and_the_path_ends_stopped() {
    let mut planner = planner_with(&[([0.0, 0.0], [50.0, 0.0]), ([50.0, 0.0], [100.0, 0.0]), ([100.0, 0.0], [100.0, 50.0])]);
    let a = planner.get_next_segment().unwrap();
    let b = planner.get_next_segment().unwrap();
    let c = planner.get_next_segment().unwrap();

    assert_eq!(a.start_velocity, 0.0);
    assert_eq!(a.end_velocity, b.start_velocity);
    assert_eq!(b.end_velocity, c.start_velocity);
    assert_eq!(c.end_velocity, 0.0);

    // Straight through at speed, but the corner is slower. So a segment must be able to slow
    // down along its length, which the planner used to forbid.
    assert_eq!(a.end_velocity, 100.0);
    assert!(b.end_velocity < 20.0, "{}", b.end_velocity);
    assert!(b.start_velocity > b.end_velocity);
}

#[test]
fn speed_is_limited_by_what_can_be_braked_for() {
    // A long run into a very short final move: there is only 1mm to stop in, so that
    // limits the speed at the start of it, and so the end of the move before.
    let mut planner = planner_with(&[([0.0, 0.0], [100.0, 0.0]), ([100.0, 0.0], [101.0, 0.0])]);
    let long = planner.get_next_segment().unwrap();
    let short = planner.get_next_segment().unwrap();
    let can_brake_from = (2.0f32 * 3000.0 * 1.0).sqrt();
    assert_eq!(short.end_velocity, 0.0);
    assert!(short.start_velocity <= can_brake_from + 0.01, "{}", short.start_velocity);
    assert_eq!(long.end_velocity, short.start_velocity);
    assert!(long.end_velocity > 50.0, "{}", long.end_velocity);
}

#[test]
fn moves_take_time_even_from_rest_to_rest() {
    let mut planner = planner_with(&[([0.0, 0.0], [100.0, 0.0])]);
    let only = planner.get_next_segment().unwrap();
    // 100mm at 100mm/s, plus ramps at 3000mm/s²: 1s + 100/3000
    assert!((only.duration - 1.0333).abs() < 0.01, "{}", only.duration);
}

#[test]
fn full_buffer_refuses_instead_of_dropping_motion() {
    let mut planner = TrajectoryPlanner::new(VolumetricSettings::default());
    let mut accepted = 0;
    for i in 0..200 {
        if planner.add_segment(segment([i as f32, 0.0], [i as f32 + 1.0, 0.0], 100.0)) {
            accepted += 1;
        }
    }
    assert!(accepted < 200);
    assert_eq!(planner.get_buffer_size(), accepted);
    let first = planner.get_next_segment().unwrap();
    assert_eq!(first.start_pos[0], 0.0, "the first move must still be there");
}

#[test]
fn planner_survives_its_ring_buffer_wrapping() {
    let mut planner = TrajectoryPlanner::new(VolumetricSettings::default());
    for i in 0..500 {
        let x = i as f32;
        while !planner.add_segment(segment([x, 0.0], [x + 1.0, 0.0], 100.0)) {
            planner.get_next_segment();
        }
        if i % 3 == 0 {
            planner.get_next_segment();
        }
    }
}

// ---------- Servo control ----------

#[test]
fn servo_velocity_is_measured() {
    let mut servo = ServoController::new();
    servo.add_axis('X', 100.0, 1000.0);
    std::thread::sleep(Duration::from_millis(5));
    servo.update_encoder_position('X', 0).unwrap();
    std::thread::sleep(Duration::from_millis(50));
    servo.update_encoder_position('X', 100).unwrap(); // 1mm in about 50ms
    let velocity = servo.get_axis_status('X').unwrap().current_velocity;
    assert!(velocity > 10.0 && velocity < 25.0, "{velocity}");
    assert!(servo.calculate_servo_output('X', 0.0).is_err());
}

// ---------- Tool changes ----------

#[test]
fn tool_change_stays_above_a_tall_print() {
    let mut tools = MultiToolManager::new();
    tools.set_current_z(120.0);
    let commands = tools.select_tool(1).unwrap();
    let z = commands.iter().find_map(|c| match c {
        ToolChangeCommand::MoveTo { position, .. } => Some(position[2]),
        _ => None,
    }).unwrap();
    assert!(z > 120.0, "{z}");

    // Standby follows the nozzle's temperature, and heating respects the tool's limit
    let standby = commands.iter().find_map(|c| match c {
        ToolChangeCommand::SetTemperature { temperature, .. } => Some(*temperature),
        _ => None,
    }).unwrap();
    assert_eq!(standby, 150.0);

    let mut hot = MaterialSettings::default();
    hot.nozzle_temperature = 999.0;
    tools.update_tool_settings(0, hot).unwrap();
    let commands = tools.select_tool(0).unwrap();
    assert!(commands.iter().all(|c| !matches!(c, ToolChangeCommand::WaitForTemperature { temperature, .. } if *temperature > 300.0)));
}

// ---------- Motor assignment ----------

fn dual_z() -> MotorAssignmentManager {
    let mut manager = MotorAssignmentManager::new();
    manager.create_standard_cartesian_setup().unwrap();
    manager
}

#[test]
fn removing_one_motor_keeps_the_others_on_that_axis() {
    let mut manager = dual_z();
    manager.assign_function("z_motor_right", MotorFunction::ZAxis(0), "share").unwrap();
    assert_eq!(manager.get_motors_for_function(&MotorFunction::ZAxis(0)).unwrap().len(), 2);
    manager.remove_motor("z_motor_left").unwrap();
    assert_eq!(manager.get_motors_for_function(&MotorFunction::ZAxis(0)).unwrap(), &vec!["z_motor_right".to_string()]);
    let group = manager.get_axis_group("Z").unwrap();
    assert_eq!(group.motor_ids, vec!["z_motor_right".to_string()]);
    assert_eq!(group.master_motor.as_deref(), Some("z_motor_right"));
}

#[test]
fn refused_removal_changes_nothing() {
    let mut manager = dual_z();
    manager.remove_motor_from_axis_group("Z", "z_motor_right").unwrap();
    assert!(manager.remove_motor_from_axis_group("Z", "z_motor_left").is_err());
    assert_eq!(manager.get_axis_group("Z").unwrap().motor_ids, vec!["z_motor_left".to_string()]);
}

#[test]
fn default_steps_suit_each_axis_and_servos_have_their_own_slave() {
    let manager = dual_z();
    assert_eq!(manager.get_motor_config("x_motor").unwrap().capabilities.steps_per_unit, 80.0);
    assert_eq!(manager.get_motor_config("z_motor_left").unwrap().capabilities.steps_per_unit, 400.0);
    assert_eq!(manager.get_motor_config("extruder_motor").unwrap().capabilities.steps_per_unit, 93.0);

    let mut servos = MotorAssignmentManager::new();
    servos.create_multi_servo_cartesian_setup().unwrap();
    let mut positions: Vec<u16> = servos.get_all_motors().values().map(|m| match m.physical_connection {
        MotorConnection::EtherCat { slave_position, .. } => slave_position,
        _ => panic!("expected EtherCAT"),
    }).collect();
    positions.sort();
    positions.dedup();
    assert_eq!(positions.len(), 8);
}

// ---------- Trajectory generator and control loop ----------

#[test]
fn s_curve_follows_the_move_and_ends_on_target() {
    let mut generator = SCurveTrajectoryGenerator::new(100.0, 1000.0, 10000.0);
    generator.plan_move(10.0, 0.0, 20.0, 0.0);
    let mut last = 10.0;
    let mut samples = 0;
    while !generator.is_complete() {
        let (position, velocity, _) = generator.update();
        assert!(position >= last - 1e-9 && position <= 20.0 + 1e-9, "{position}");
        assert!(velocity.abs() < 1000.0, "{velocity}");
        last = position;
        samples += 1;
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(generator.update().0, 20.0);
    assert!(samples > 5, "ran too fast to test: {samples}");
}

fn controller() -> (LinuxCNCController, String) {
    let mut motors = MotorAssignmentManager::new();
    motors.create_standard_cartesian_setup().unwrap();
    let mut controller = LinuxCNCController::new(
        Arc::new(Mutex::new(motors)),
        Arc::new(Mutex::new(ClosedLoopManager::new(1000))),
        1000.0,
    );
    let mut parameters = AxisControlParameters::default();
    parameters.backlash_compensation = 0.0;
    controller.add_axis("X".to_string(), parameters.clone(), ControlOutputType::DigitalPosition, vec!["x_motor".to_string()]).unwrap();
    controller.add_axis("Y".to_string(), parameters, ControlOutputType::DigitalPosition, vec!["y_motor".to_string()]).unwrap();
    controller.enable_global();
    controller.enable_axis("X").unwrap();
    controller.enable_axis("Y").unwrap();
    (controller, "X_x_motor".to_string())
}

fn step(controller: &mut LinuxCNCController) -> HashMap<String, f64> {
    std::thread::sleep(Duration::from_millis(2));
    controller.update_control_loop().unwrap()
}

#[test]
fn stopped_controller_commands_zero_and_stays_stopped() {
    let (mut controller, x) = controller();
    controller.command_position("X", 5.0, 0.0).unwrap();
    step(&mut controller);

    controller.emergency_stop();
    assert_eq!(step(&mut controller)[&x], 0.0, "an explicit zero, not silence");

    controller.enable_global();
    assert_eq!(step(&mut controller)[&x], 0.0, "enable alone must not clear a stop");

    controller.reset_emergency_stop();
    controller.enable_global();
    assert!(step(&mut controller).contains_key(&x));
}

#[test]
fn disabled_axis_gets_no_output() {
    let (mut controller, x) = controller();
    controller.command_position("X", 5.0, 0.0).unwrap();
    controller.disable_axis("X").unwrap();
    assert_eq!(step(&mut controller)[&x], 0.0);
}

#[test]
fn open_loop_axis_does_not_fault() {
    let (mut controller, _) = controller();
    controller.coordinate_linear_move(HashMap::from([("X".to_string(), 500.0)]), 500.0).unwrap();
    for _ in 0..20 {
        step(&mut controller);
    }
    assert!(!controller.get_axis_state("X").unwrap().fault);
}

#[test]
fn homing_stops_a_move_in_progress() {
    let (mut controller, _) = controller();
    controller.command_position("X", 50.0, 0.0).unwrap();
    step(&mut controller);
    controller.home_axis("X").unwrap();
    step(&mut controller);
    assert_eq!(controller.get_axis_state("X").unwrap().commanded_position, 0.0);
}

#[test]
fn coordinated_axes_finish_together() {
    let (mut controller, _) = controller();
    controller.coordinate_linear_move(HashMap::from([("X".to_string(), 3.0), ("Y".to_string(), 1.0)]), 100.0).unwrap();
    let mut ratios = Vec::new();
    for _ in 0..30 {
        step(&mut controller);
        let x = controller.get_axis_state("X").unwrap().commanded_position;
        let y = controller.get_axis_state("Y").unwrap().commanded_position;
        if x > 0.1 && y > 0.0 && x < 2.9 {
            ratios.push(x / y);
        }
    }
    assert!(!ratios.is_empty());
    for ratio in ratios {
        assert!((ratio - 3.0).abs() < 0.2, "path is not straight: X/Y = {ratio}");
    }
}

// ---------- Torque monitoring ----------

struct FixedSensor(f32, bool);

impl TorqueSensor for FixedSensor {
    fn read_torque(&self) -> Result<f32, String> {
        if self.1 { Err("disconnected".to_string()) } else { Ok(self.0) }
    }
    fn get_max_torque(&self) -> f32 { 5.0 }
    fn calibrate(&mut self) -> Result<(), String> { Ok(()) }
    fn get_sensor_id(&self) -> String { "test".to_string() }
}

fn motor(torque: f32) -> ServoMotorWithTorque {
    ServoMotorWithTorque::new('E', Box::new(FixedSensor(torque, false)))
}

#[test]
fn blocked_extruder_is_detected_as_a_clog() {
    let config = TorqueControlConfig::default();
    let high_torque = config.extrusion_threshold + 0.9 * (config.collision_threshold - config.extrusion_threshold);
    let mut extruder = AdaptiveExtruder::new(0, motor(high_torque));
    assert!(matches!(extruder.update(&config, 5.0, 200.0).unwrap(), ExtrusionEvent::ClogDetected { .. }));
    assert!(extruder.actual_flow_rate < 1.0, "{}", extruder.actual_flow_rate);
}

#[test]
fn healthy_extruder_keeps_its_flow() {
    let config = TorqueControlConfig::default();
    let mut extruder = AdaptiveExtruder::new(0, motor(0.0));
    for _ in 0..500 {
        extruder.update(&config, 5.0, 200.0).unwrap();
    }
    assert_eq!(extruder.adaptive_multiplier, 1.0, "must not creep toward over-extrusion");
    assert_eq!(extruder.actual_flow_rate, 5.0);
}

#[test]
fn torque_failures_stop_the_machine() {
    let config = TorqueControlConfig::default();
    let mut nan_motor = motor(f32::NAN);
    assert!(matches!(nan_motor.update_torque(&config).unwrap(), TorqueEvent::OverTorque { .. }));

    futures::executor::block_on(async {
        let mut controller = DynamicForceController::new(config);
        controller.add_motor('X', Box::new(FixedSensor(0.0, true)));
        assert!(matches!(controller.update_all().await.unwrap(), SystemEvent::EmergencyStop { .. }), "losing the sensor is not a warning");
    });

    futures::executor::block_on(async {
        let mut controller = DynamicForceController::new(TorqueControlConfig::default());
        controller.add_motor('X', Box::new(FixedSensor(0.0, false)));
        assert!(matches!(controller.update_all().await.unwrap(), SystemEvent::Normal { .. }));
        controller.emergency_stop();
        assert!(matches!(controller.update_all().await.unwrap(), SystemEvent::EmergencyStop { .. }), "stays stopped");
        controller.reset_safety();
        assert!(matches!(controller.update_all().await.unwrap(), SystemEvent::Normal { .. }));
    });
}

#[test]
fn unclog_never_overheats() {
    let mut extruder = AdaptiveExtruder::new(0, motor(0.0));
    extruder.temperature = 275.0;
    for command in extruder.force_unclog_sequence() {
        if let UnclogCommand::SetTemperature { temperature, .. } = command {
            assert!(temperature <= 280.0, "{temperature}");
        }
    }
}

// ---------- Configuration ----------

fn config_path(name: &str) -> std::path::PathBuf {
    std::env::temp_dir().join(format!("regression_config_{name}_{}.toml", std::process::id()))
}

#[test]
fn bad_config_is_not_taken_and_saves_are_clean() {
    let path = config_path("bad");
    let _ = std::fs::remove_file(&path);
    let mut manager = ConfigManager::new(&path);
    manager.save_config().unwrap();
    assert!(!path.with_extension("toml.tmp").exists());
    manager.load_config().unwrap();
    let good_velocity = manager.get_config().motion.max_velocity;

    let mut bad = manager.get_config().clone();
    bad.motion.max_velocity = f64::NAN;
    let text = serde_json::to_string(&bad).unwrap();
    assert!(manager.import_config(&text, ExportFormat::Json).is_err());
    assert_eq!(manager.get_config().motion.max_velocity, good_velocity);

    let mut too_hot = manager.get_config().clone();
    too_hot.motion.max_velocity = -1.0;
    let text = serde_json::to_string(&too_hot).unwrap();
    assert!(manager.import_config(&text, ExportFormat::Json).is_err());

    let _ = std::fs::remove_file(&path);
    let _ = std::fs::remove_file(path.with_extension("toml.backup"));
}

#[test]
fn corexy_sample_keeps_the_motors_it_does_not_replace() {
    let default_count = ConfigManager::generate_sample_config(PrinterType::Cartesian).motors.len();
    let corexy = ConfigManager::generate_sample_config(PrinterType::CoreXY);
    assert!(corexy.motors.contains_key("stepper_a") && corexy.motors.contains_key("stepper_b"));
    assert!(corexy.motors.values().any(|m| matches!(m.function, MotorFunction::ZAxis(_))) || default_count <= 2, "CoreXY needs its Z motor");
}

// ---------- EtherCAT frames ----------

#[cfg(windows)]
#[test]
fn ethercat_frames_follow_the_wire_format() {
    use printer_host::ethercat_windows::WindowsEtherCATMaster;
    let frame = WindowsEtherCATMaster::build_read_frame(&[1, 2, 3, 4, 5, 6], 0x07, 0x0130, 8);
    assert_eq!(&frame[12..14], &[0x88, 0xA4]);
    assert_eq!(frame.len(), 60, "padded to Ethernet's minimum");

    let header = u16::from_le_bytes([frame[14], frame[15]]);
    assert_eq!(header >> 12, 1, "type: EtherCAT commands");
    assert_eq!(header & 0x07FF, 20, "10 byte header + 8 data + 2 working counter");

    assert_eq!(frame[16], 0x07, "command");
    assert_eq!(u16::from_le_bytes([frame[20], frame[21]]), 0x0130, "register");
    assert_eq!(u16::from_le_bytes([frame[22], frame[23]]), 8, "data length");

    // The working counter is found by the datagram's length, despite padding after it
    let mut reply = frame.clone();
    reply[34] = 3;
    assert_eq!(WindowsEtherCATMaster::parse_working_counter(&reply), Some(3));
    assert_eq!(WindowsEtherCATMaster::parse_working_counter(&frame), Some(0));
    assert_eq!(WindowsEtherCATMaster::parse_working_counter(&[0u8; 60]), None, "not EtherCAT");
}

#[cfg(windows)]
#[test]
fn ethercat_does_not_claim_slaves_are_operational() {
    use printer_host::ethercat_windows::WindowsEtherCATMaster;
    let mut master = WindowsEtherCATMaster::new();
    assert!(futures::executor::block_on(master.configure_slave(1)).is_err());
}
