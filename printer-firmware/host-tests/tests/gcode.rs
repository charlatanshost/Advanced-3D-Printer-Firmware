// Each test here is run in order on a single global exclude-object manager, so they share one
// test function per global state where it matters.
use firmware_host_tests::exclude_object::{parse_object_marker, ExcludeObjectManager, ObjectStatus, EXCLUDE_OBJECT_MANAGER};
use firmware_host_tests::gcode_processor::*;
use futures::executor::block_on;

fn run(line: &str) -> Result<GcodeAction, GcodeError> {
    block_on(GcodeProcessor::new().process_gcode_line(line))
}

fn expect_move(line: &str) -> (Option<f32>, Option<f32>, Option<f32>, Option<f32>, Option<f32>) {
    match run(line) {
        Ok(GcodeAction::Execute(GcodeCommand::Move { x, y, z, e, feed_rate })) => (x, y, z, e, feed_rate),
        other => panic!("{line:?} -> {other:?}"),
    }
}

#[test]
fn comments_are_never_commands() {
    assert!(matches!(run("; a comment"), Ok(GcodeAction::Skip)));
    assert_eq!(expect_move("G1 X10 Y10 ; Z5 F60 lift"), (Some(10.0), Some(10.0), None, None, None));
    assert_eq!(expect_move("G1 X10 ; E5 prime"), (Some(10.0), None, None, None, None));
}

#[test]
fn long_lines_do_not_panic() {
    assert!(matches!(run(&format!("; {}", "word ".repeat(100))), Ok(GcodeAction::Skip)));
    let many_words = format!("G1 {}", "X1 ".repeat(30));
    assert!(run(&many_words).is_ok());
    assert!(matches!(run(&format!("G1 {}", "X1 ".repeat(200))), Err(GcodeError::LineTooLong)));
}

#[test]
fn common_formats_parse() {
    assert_eq!(expect_move("G01 X5").0, Some(5.0));
    assert_eq!(expect_move("g1 x5").0, Some(5.0));
    assert_eq!(expect_move("N10 G1 X5*33").0, Some(5.0));
    assert_eq!(expect_move("G1X10Y-2.5").1, Some(-2.5));
}

#[test]
fn bad_values_are_errors_not_dropped_axes() {
    for line in ["G1 X1O", "G1 X", "G1 X1 F0", "M104", "M140", "M104 Sabc"] {
        assert!(matches!(run(line), Err(GcodeError::ParseError)), "{line}: {:?}", run(line));
    }
}

#[test]
fn temperatures_are_limited() {
    assert!(matches!(run("M104 S200"), Ok(GcodeAction::Execute(GcodeCommand::SetTemperature { temp, .. })) if temp == 200.0));
    for line in ["M104 S9999", "M104 S-5", "M140 S500", "M109 S9999", "M190 S500", "M109 R9999"] {
        assert!(matches!(run(line), Err(GcodeError::TemperatureOutOfRange)), "{line}: {:?}", run(line));
    }
}

#[test]
fn fan_speed_fractions() {
    let speed = |line: &str| match run(line) {
        Ok(GcodeAction::Execute(GcodeCommand::SetFanSpeed { speed, .. })) => speed,
        other => panic!("{line}: {other:?}"),
    };
    assert_eq!(speed("M106 S127.5"), 128);
    assert_eq!(speed("M106"), 255);
    assert_eq!(speed("M106 S999"), 255);
    assert_eq!(speed("M107"), 0);
}

#[test]
fn position_tracking_follows_modes() {
    let mut p = GcodeProcessor::new();
    block_on(async {
        for line in ["G1 X10 E5", "G91", "G1 X5 E1", "G1 X5", "G90", "G1 X1", "M83", "G1 E2", "G92 E0", "G92 X100"] {
            p.process_gcode_line(line).await.unwrap();
        }
    });
    assert_eq!(p.get_current_position(), [100.0, 0.0, 0.0, 0.0]);
    let mut p = GcodeProcessor::new();
    block_on(async {
        for line in ["G91", "G1 X5", "G1 X5", "M83"] {
            p.process_gcode_line(line).await.unwrap();
        }
    });
    assert_eq!(p.get_current_position()[0], 10.0);
}

#[test]
fn object_marker_names() {
    assert_eq!(parse_object_marker("; object: A").unwrap().as_str(), "A");
    assert_eq!(parse_object_marker("; object: Part 1 center:1,2 time:30").unwrap().as_str(), "Part 1");
    assert!(parse_object_marker("; object:").is_none());
}

const TWO_LAYERS: &str = ";LAYER:0\n; object: A\nG1 X1 Y1 E1\n; end object\n; object: B\nG1 X2 Y2 E2\n; end object\n;LAYER:1\n; object: A\nG1 X1 Y1 E3\n; end object\n; object: B\nG1 X2 Y2 E4\n; end object\nG1 X9 Y9 E5\n";

#[test]
fn objects_are_one_entry_however_many_layers() {
    block_on(async {
        let mut m = ExcludeObjectManager::new();
        m.parse_gcode_objects(TWO_LAYERS).await.unwrap();
        let list = m.get_object_list().await;
        assert_eq!(list.len(), 2);
        assert_eq!((list[0].start_line, list[0].end_line), (2, 11));
        assert_eq!((list[1].start_line, list[1].end_line), (5, 14));
        assert_eq!(m.get_print_status().await.0, 2);

        // Metadata with too many values must not panic, and a move at 0,0 counts for bounds
        m.parse_gcode_objects("; object: C center:1,2,3,4,5,6\nG1 X0 Y0\nG1 X5 Y5\n; end object\n").await.unwrap();
        let list = m.get_object_list().await;
        assert_eq!(list[0].center, [2.5, 2.5]);
    });
}

#[test]
fn new_print_forgets_old_exclusions() {
    block_on(async {
        let mut m = ExcludeObjectManager::new();
        m.parse_gcode_objects(TWO_LAYERS).await.unwrap();
        m.exclude_object("A").await.unwrap();
        assert!(m.is_object_excluded("A").await);
        m.parse_gcode_objects(TWO_LAYERS).await.unwrap();
        assert!(!m.is_object_excluded("A").await);
    });
}

/// Uses the global manager, so everything that needs it lives in this one test.
#[test]
fn printing_with_objects() {
    block_on(async {
        // A normal print: both objects complete, and only after their last layer
        {
            let mut manager = ExcludeObjectManager::new();
            manager.parse_gcode_objects(TWO_LAYERS).await.unwrap();
            *EXCLUDE_OBJECT_MANAGER.lock().await = Some(manager);
        }
        let mut p = GcodeProcessor::new();
        for (i, line) in TWO_LAYERS.lines().enumerate() {
            p.process_gcode_line(line).await.unwrap();
            if i == 3 {
                let guard = EXCLUDE_OBJECT_MANAGER.lock().await;
                let list = guard.as_ref().unwrap().get_object_list().await;
                assert_eq!(list[0].status, ObjectStatus::Pending, "A has another layer to go");
            }
        }
        {
            let guard = EXCLUDE_OBJECT_MANAGER.lock().await;
            let manager = guard.as_ref().unwrap();
            assert_eq!(manager.get_print_status().await, (2, 2, 0));
            assert!(manager.get_object_list().await.iter().all(|o| o.status == ObjectStatus::Completed));
        }

        // Excluding an object skips every layer of it, resynchronises E, and keeps other commands
        let file = "; object: A\nG1 X1 E100\nM106 S50\nG1 X2 E200\n; end object\n; object: B\nG1 X3 E201\n; end object\n; object: A\nG1 X4 E300\n; end object\nG1 X5 E301\n";
        {
            let mut manager = ExcludeObjectManager::new();
            manager.parse_gcode_objects(file).await.unwrap();
            manager.exclude_object("A").await.unwrap();
            *EXCLUDE_OBJECT_MANAGER.lock().await = Some(manager);
        }
        let mut p = GcodeProcessor::new();
        let mut actions = Vec::new();
        for line in file.lines() {
            actions.push(p.process_gcode_line(line).await.unwrap());
        }
        assert!(matches!(actions[1], GcodeAction::Skip));
        assert!(matches!(actions[2], GcodeAction::Execute(GcodeCommand::SetFanSpeed { speed: 50, .. })));
        assert!(matches!(actions[3], GcodeAction::Skip));
        // First move back: E goes to where it was just before this move (200), so this move
        // extrudes 1mm rather than 201mm
        match &actions[6] {
            GcodeAction::ResyncExtruderThenExecute { e, command: GcodeCommand::Move { e: Some(move_e), .. } } => {
                assert_eq!(*e, 200.0);
                assert_eq!(*move_e, 201.0);
            }
            other => panic!("{other:?}"),
        }
        assert!(matches!(actions[9], GcodeAction::Skip), "A's second section is skipped too");
        assert!(matches!(actions[11], GcodeAction::ResyncExtruderThenExecute { e, .. } if e == 300.0));
        assert_eq!(p.get_current_position()[3], 301.0);

        *EXCLUDE_OBJECT_MANAGER.lock().await = None;
    });
}

#[test]
fn progress_percentages_survive_odd_counts() {
    let p = PrintProgress { total_lines: 0, processed_lines: 0, total_objects: 1, completed_objects: 0, excluded_objects: 2, current_layer: 0, estimated_layers: 0 };
    assert_eq!(p.objects_progress_percentage(), 0.0);
}
