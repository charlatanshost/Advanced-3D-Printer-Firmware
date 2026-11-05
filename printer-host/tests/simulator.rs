// End-to-end tests: G-code in, simulated machine out, using the real control code throughout.
use printer_host::power_loss_recovery::RecoveryConfig;
use printer_host::simulator::*;
use printer_kinematics::{CoreXYConfig, CoreXYKinematics, DeltaConfig, DeltaKinematics};
use std::fmt::Write;
use std::path::PathBuf;

const START: &str = "M140 S60\nM104 S200\nM190 S60\nM109 S200\nG28\nG90\nM82\nG92 E0\n";

/// A hollow 20 mm square: the kind of absolute-extrusion G-code a slicer writes
fn square_tower(layers: usize) -> String {
    let mut text = String::from(START);
    let mut e = 0.0f64;
    writeln!(text, "M106 S128").unwrap();
    for layer in 1..=layers {
        let z = layer as f64 * 0.2;
        writeln!(text, "G1 Z{:.3} F1200", z).unwrap();
        writeln!(text, "G1 X100 Y100 F6000").unwrap();
        for (x, y) in [(120.0, 100.0), (120.0, 120.0), (100.0, 120.0), (100.0, 100.0)] {
            e += 0.8;
            writeln!(text, "G1 X{:.3} Y{:.3} E{:.5} F1800", x, y, e).unwrap();
        }
    }
    writeln!(text, "G1 Z{:.3} F600\nM104 S0\nM140 S0\nM107", layers as f64 * 0.2 + 10.0).unwrap();
    text
}

fn simulate(gcode: &str, config: SimConfig) -> Simulator {
    let commands = parse_gcode(gcode).unwrap();
    let mut simulator = Simulator::new(config);
    simulator.run(&commands);
    simulator
}

fn with_faults(faults: Vec<(FaultTrigger, Fault)>) -> SimConfig {
    SimConfig { faults, ..SimConfig::default() }
}

fn halted_reason(simulator: &Simulator) -> String {
    match &simulator.report().outcome {
        Outcome::Halted(reason) => reason.clone(),
        other => panic!("expected a halt, got {:?}", other),
    }
}

// ---------- Normal printing ----------

#[test]
fn a_normal_print_runs_clean() {
    let simulator = simulate(&square_tower(10), SimConfig::default());
    let report = simulator.report();
    assert!(report.is_clean(), "{:#?}", report);
    assert!((report.final_e - 32.0).abs() < 1e-3, "{}", report.final_e);
    assert!((report.filament_net - 32.0).abs() < 1e-3, "{}", report.filament_net);
    assert!((report.final_position[2] - 12.0).abs() < 1e-3);
    assert!(report.peak_speed <= 100.0 + 1e-6, "{}", report.peak_speed);
    assert!(report.ignored_commands.is_empty(), "{:?}", report.ignored_commands);

    // The part really is where the file put it, and not anywhere else
    assert!((simulator.printed_height(110.5, 100.5).unwrap() - 2.0).abs() < 1e-3);
    assert!(simulator.printed_height(110.5, 110.5).is_none());
}

#[test]
fn heaters_reach_temperature_without_much_overshoot() {
    let simulator = simulate(&format!("{}G4 S120\n", START), SimConfig::default());
    let report = simulator.report();
    assert!(report.is_clean(), "{:#?}", report);
    assert!(report.seconds < 400.0, "took {} s to heat", report.seconds);
    assert!(report.hotend_overshoot < 15.0, "hotend overshot by {}", report.hotend_overshoot);
    assert!(report.bed_overshoot < 15.0, "bed overshot by {}", report.bed_overshoot);
}

#[test]
fn smooth_profile_prints_the_same_thing() {
    let plain = simulate(&square_tower(3), SimConfig::default());
    let smooth = simulate(&square_tower(3), SimConfig { profile: printer_host::smooth_trajectory::MotionProfile::SCurve, ..SimConfig::default() });
    assert!(smooth.report().is_clean(), "{:#?}", smooth.report());
    assert!((plain.report().final_e - smooth.report().final_e).abs() < 1e-6);
    assert_eq!(plain.printed_cells(), smooth.printed_cells());
}

#[test]
fn arcs_extrude_along_the_curve() {
    let gcode = format!("{}G1 Z0.2 F600\nG1 X120 Y100 F3000\nG3 X100 Y120 I-20 J0 E5 F1200\n", START);
    let simulator = simulate(&gcode, SimConfig::default());
    assert!(simulator.report().is_clean(), "{:#?}", simulator.report());
    // On the circle, not the straight line between the ends (which would be at about 110, 110)
    let on_circle = 100.0 + 20.0 * std::f64::consts::FRAC_1_SQRT_2;
    assert!(simulator.printed_height(on_circle, on_circle).is_some());
    assert!(simulator.printed_height(110.0, 110.0).is_none());
}

#[test]
fn relative_modes_and_coordinate_shifts_are_followed() {
    let gcode = format!("{}M83\nG1 Z0.2 F600\nG1 X10 Y10 E1 F1800\nG1 X20 E1\nG92 X0 Y0 E0\nG91\nG1 X5 E0.5\nG90\nG1 X0\n", START);
    let report_sim = simulate(&gcode, SimConfig::default());
    let report = report_sim.report();
    assert!(report.is_clean(), "{:#?}", report);
    // X0 after G92 is back at physical X20, not at the end of the bed
    assert!((report.final_position[0] - 20.0).abs() < 1e-3, "{:?}", report.final_position);
    assert!((report.filament_net - 2.5).abs() < 1e-3, "{}", report.filament_net);
}

#[test]
fn motors_follow_the_kinematics() {
    let commands = parse_gcode(&format!("{}G1 X30 Y10 F3000\n", START)).unwrap();
    let mut corexy = Simulator::with_kinematics(SimConfig::default(), Box::new(CoreXYKinematics::new(CoreXYConfig { belt_angle: 45.0 })));
    corexy.run(&commands);
    let motors = corexy.report().final_motors;
    assert_eq!((motors[0], motors[1]), (40.0, 20.0), "A = X+Y, B = X-Y");
}

// ---------- Refusing to do damage ----------

#[test]
fn moving_outside_the_machine_stops_the_print() {
    let simulator = simulate(&format!("{}G1 X500\nG1 X10\n", START), SimConfig::default());
    assert!(halted_reason(&simulator).contains("outside the build volume"));
    assert!(simulator.report().final_position[0] <= 220.0);
}

#[test]
fn delta_stops_at_what_it_cannot_reach() {
    let commands = parse_gcode("G28\nG1 X200 Y200\n").unwrap();
    let config = DeltaConfig { arm_length: 100.0, rod_length: 200.0, bed_radius: 100.0 };
    let mut delta = Simulator::with_kinematics(SimConfig::default(), Box::new(DeltaKinematics::new(config)));
    delta.run(&commands);
    assert!(halted_reason(&delta).contains("out of reach"));
}

#[test]
fn moving_before_homing_stops_the_print() {
    let simulator = simulate("G1 X10 F600\n", SimConfig::default());
    assert!(halted_reason(&simulator).contains("before it was homed"));
}

#[test]
fn cold_extrusion_is_refused() {
    let simulator = simulate("G28\nG1 X10 Y10 E5 F600\n", SimConfig::default());
    assert!(simulator.report().cold_extrusions > 0);
    assert_eq!(simulator.report().filament_net, 0.0, "the filament must not have moved");
    assert!(!simulator.report().is_clean());
}

#[test]
fn absurd_temperature_stops_the_print() {
    let simulator = simulate("M104 S9999\nG28\n", SimConfig::default());
    assert!(halted_reason(&simulator).contains("temperature"));
    assert!(simulator.report().peak_hotend < 30.0);
}

#[test]
fn printing_through_the_part_is_detected() {
    let gcode = format!("{}G1 Z1 F600\nG1 X100 Y100 F3000\nG1 X120 E2 F1200\nG1 Z10 F600\nG1 X110 F3000\nG1 Z0.2 F600\n", START);
    let report = simulate(&gcode, SimConfig::default()).report().clone();
    assert!(report.collisions > 0, "{:#?}", report);
    assert!(!report.is_clean());
}

// ---------- Hardware faults ----------

const HOLD: &str = "G4 S300\n";

fn heated_hold() -> String {
    format!("{}{}", START, HOLD)
}

#[test]
fn broken_sensors_stop_the_print_and_the_heat() {
    for fault in [Fault::SensorOpen(Heater::Hotend), Fault::SensorShort(Heater::Hotend), Fault::SensorOpen(Heater::Bed), Fault::SensorShort(Heater::Bed)] {
        let simulator = simulate(&heated_hold(), with_faults(vec![(FaultTrigger::AtTime(250.0), fault)]));
        let report = simulator.report();
        assert!(halted_reason(&simulator).contains("Sensor fault"), "{:?}: {:#?}", fault, report);
        assert!(report.seconds < 250.0 + 181.0, "{:?} carried on until {}", fault, report.seconds);
        assert!(report.peak_hotend < 225.0, "{:?} reached {}", fault, report.peak_hotend);
        assert!(report.warnings.is_empty(), "{:?}: {:?}", fault, report.warnings);
    }
}

#[test]
fn heater_failing_while_heating_up_is_caught() {
    let simulator = simulate(&heated_hold(), with_faults(vec![(FaultTrigger::AtTime(5.0), Fault::HeaterDead(Heater::Hotend))]));
    assert!(halted_reason(&simulator).contains("Heating failed"), "{:#?}", simulator.report());
}

#[test]
fn heater_failing_at_temperature_is_caught() {
    let simulator = simulate(&heated_hold(), with_faults(vec![(FaultTrigger::AtTime(200.0), Fault::HeaterDead(Heater::Hotend))]));
    assert!(halted_reason(&simulator).contains("Thermal runaway"), "{:#?}", simulator.report());
}

#[test]
fn sensor_slipping_out_during_heat_up_is_caught_before_it_gets_dangerous() {
    let simulator = simulate(&heated_hold(), with_faults(vec![(FaultTrigger::AtTime(10.0), Fault::SensorFrozen(Heater::Hotend))]));
    let report = simulator.report();
    assert!(halted_reason(&simulator).contains("Heating failed"), "{:#?}", report);
    assert!(report.peak_hotend < 300.0, "really reached {}", report.peak_hotend);
}

#[test]
fn stuck_heater_is_detected_but_needs_hardware_to_stop() {
    let simulator = simulate(&heated_hold(), with_faults(vec![(FaultTrigger::AtTime(200.0), Fault::HeaterStuckOn(Heater::Hotend))]));
    let report = simulator.report();
    assert!(halted_reason(&simulator).contains("Over temperature"), "{:#?}", report);
    // This is the honest limit of any software: it keeps heating after the shutdown.
    assert!(report.peak_hotend > 300.0);
    assert!(report.warnings.iter().any(|w| w.contains("thermal fuse")), "{:?}", report.warnings);
}

#[test]
fn emergency_stop_stops_everything() {
    let simulator = simulate(&square_tower(50), with_faults(vec![(FaultTrigger::AtTime(200.0), Fault::EmergencyStop)]));
    let report = simulator.report();
    assert_eq!(halted_reason(&simulator), "Emergency stop");
    assert!(report.final_position[2] < 10.0, "{:?}", report.final_position);
    assert!(report.final_e < 200.0, "{}", report.final_e);
    assert!(report.lines_run < parse_gcode(&square_tower(50)).unwrap().len());
}

// ---------- Power loss, the whole way through ----------

fn recovery_config(name: &str) -> RecoveryConfig {
    let path: PathBuf = std::env::temp_dir().join(format!("sim_{name}_{}.plr", std::process::id()));
    for extension in ["plr", "bak", "tmp", "stats"] {
        let _ = std::fs::remove_file(path.with_extension(extension));
    }
    RecoveryConfig { recovery_file_path: path, save_interval_ms: 0, ..RecoveryConfig::default() }
}

/// Print with a power failure part way, come back, resume. Returns the reference print too.
fn print_through_power_loss(name: &str, fault: Fault, line: u32, off_seconds: f64) -> (Simulator, Simulator) {
    let gcode = square_tower(20);
    let commands = parse_gcode(&gcode).unwrap();

    let reference = simulate(&gcode, SimConfig::default());
    assert!(reference.report().is_clean());

    let config = SimConfig {
        recovery: Some(recovery_config(name)),
        faults: vec![(FaultTrigger::AtLine(line), fault)],
        ..SimConfig::default()
    };
    let mut first = Simulator::new(config);
    first.run(&commands);
    assert_eq!(first.report().outcome, Outcome::PowerLost);

    let mut second = first.restart_after_power_loss(off_seconds);
    let resumed = second.resume(&commands);
    let resumed_from = resumed.unwrap_or_else(|e| panic!("{}: {:#?}", e, second.report()));
    assert!(resumed_from > 8, "resumed from line {}", resumed_from);
    (reference, second)
}

fn check_recovered_print(reference: &Simulator, recovered: &Simulator) {
    let (expected, report) = (reference.report(), recovered.report());
    assert!(report.is_clean(), "{:#?}", report);

    // Finished where it should, with the extruder counter where it should be
    for axis in 0..3 {
        assert!((report.final_position[axis] - expected.final_position[axis]).abs() < 1e-3, "{:?} vs {:?}", report.final_position, expected.final_position);
    }
    assert!((report.final_e - expected.final_e).abs() < 1e-3, "E {} vs {}", report.final_e, expected.final_e);

    // The same part: same outline, same final height
    assert_eq!(recovered.printed_cells(), reference.printed_cells());
    for (x, y) in [(110.5, 100.5), (120.5, 110.5), (110.5, 120.5), (100.5, 110.5)] {
        let (a, b) = (recovered.printed_height(x, y).unwrap(), reference.printed_height(x, y).unwrap());
        assert!((a - b).abs() < 1e-3, "height at {},{}: {} vs {}", x, y, a, b);
    }

    // Nothing skipped, and not much printed twice: at most what ran since the last save, plus
    // the prime. A second's worth at this speed is under 1.2 mm of filament.
    let extra = report.filament_extruded - expected.filament_extruded;
    assert!(extra > -1e-3, "{} mm of filament went missing", -extra);
    assert!(extra < 6.0, "{} mm extra filament", extra);

    // Reheated before extruding
    assert_eq!(report.cold_extrusions, 0);
}

#[test]
fn sudden_power_cut_resumes_to_the_same_part() {
    let (reference, recovered) = print_through_power_loss("cut", Fault::PowerCut, 100, 60.0);
    check_recovered_print(&reference, &recovered);
}

#[test]
fn power_cut_with_a_cold_machine_reheats_first() {
    let (reference, recovered) = print_through_power_loss("cold", Fault::PowerCut, 100, 3600.0);
    check_recovered_print(&reference, &recovered);
    let events: Vec<&str> = recovered.report().events.iter().map(|e| e.1.as_str()).collect();
    let at_temperature = events.iter().position(|e| e.contains("Hotend at temperature")).expect("waited for the hotend");
    assert!(at_temperature < events.len());
    assert!(recovered.report().seconds > 60.0, "must have spent time heating");
}

#[test]
fn power_loss_with_warning_lifts_and_resumes_to_the_same_part() {
    let (reference, recovered) = print_through_power_loss("detected", Fault::PowerLossDetected, 100, 60.0);
    check_recovered_print(&reference, &recovered);
}

#[test]
fn power_cuts_all_through_a_print_recover() {
    // Layer changes and everything in between
    for line in [20, 37, 55, 73, 90, 118] {
        let name = format!("sweep{}", line);
        let (reference, recovered) = print_through_power_loss(&name, Fault::PowerCut, line, 30.0);
        check_recovered_print(&reference, &recovered);
    }
}

// ---------- The real sample file ----------

#[test]
fn real_slicer_output_runs_clean() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../gcode/example.gcode");
    let text = std::fs::read_to_string(path).unwrap();
    // The first few thousand lines: heating, first layers, plenty of retractions and travels.
    let excerpt: String = text.lines().take(4000).map(|line| format!("{line}\n")).collect();
    let simulator = simulate(&excerpt, SimConfig { interpolation_hz: 100.0, ..SimConfig::default() });
    let report = simulator.report();
    assert!(report.is_clean(), "{:?} collisions {} {:?} cold {}\n{:?}", report.outcome, report.collisions, report.first_collision, report.cold_extrusions, report.events);
    assert!(report.moves > 3000, "{}", report.moves);
    assert!(simulator.printed_cells() > 100);
}

#[test]
fn fault_descriptions_parse() {
    assert_eq!(parse_fault("hotend-sensor-open@120.5").unwrap(), (FaultTrigger::AtTime(120.5), Fault::SensorOpen(Heater::Hotend)));
    assert_eq!(parse_fault("power-cut@line:500").unwrap(), (FaultTrigger::AtLine(500), Fault::PowerCut));
    assert!(parse_fault("nonsense@1").is_err());
    assert!(parse_fault("power-cut").is_err());
}
