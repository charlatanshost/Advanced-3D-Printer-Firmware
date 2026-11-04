// Integration test for the power loss recovery module.

use printer_host::power_loss_recovery::{
    PowerLossRecoveryManager, RecoveryConfig, PrintStateUpdate,
};
use std::fs;
use std::time::Duration;

#[test]
fn test_save_and_load_recovery_state() {
    // 1. Setup a unique temporary file for this test.
    let temp_dir = std::env::temp_dir();
    // Use a unique name to prevent conflicts between test runs.
    let recovery_file_path = temp_dir.join(format!("test_recovery_{}.plr", std::process::id()));

    // Ensure the file is clean before starting.
    let _ = fs::remove_file(&recovery_file_path);

    let gcode_file = "test.gcode".to_string();
    let file_size = 2048;
    let updated_line_number = 123;
    let updated_layer = 5;
    let updated_file_pos = 1024;

    {
        // --- This block simulates the first run of the application ---
        let mut config = RecoveryConfig::default();
        config.recovery_file_path = recovery_file_path.clone();
        // Set a short save interval for testing purposes.
        config.save_interval_ms = 20;

        let mut manager = PowerLossRecoveryManager::new(config.clone()).expect("Failed to create manager");

        // 2. Start tracking a print. This performs an initial save of the state.
        manager.start_print_tracking(gcode_file.clone(), file_size).expect("Failed to start tracking");

        // 3. Update the print state with new values.
        let update = PrintStateUpdate {
            line_number: Some(updated_line_number),
            current_layer: Some(updated_layer),
            file_position: Some(updated_file_pos),
            ..Default::default()
        };
        manager.update_print_state(update).expect("Failed to update state");

        // 4. Wait for the save interval to pass, then trigger another update.
        // This ensures the check for saving the state is performed and the updated state is written to disk.
        std::thread::sleep(Duration::from_millis(30));
        manager.update_print_state(Default::default()).expect("Failed to trigger save");
    }

    {
        // --- This block simulates restarting the application after a power loss ---
        let mut config = RecoveryConfig::default();
        config.recovery_file_path = recovery_file_path.clone();

        let mut new_manager = PowerLossRecoveryManager::new(config).expect("Failed to create new manager");

        // 5. Verify that recovery data exists and then load it.
        assert!(new_manager.has_recovery_data(), "Recovery data should exist on the new manager.");
        let loaded_state_opt = new_manager.load_recovery_data().expect("Failed to load recovery data");

        // 6. Assert that the loaded data is present and correct.
        let loaded_state = loaded_state_opt.expect("Failed to load any recovery state.");

        assert_eq!(loaded_state.gcode_file_path, gcode_file);
        assert_eq!(loaded_state.file_size, file_size);
        assert_eq!(loaded_state.line_number, updated_line_number);
        assert_eq!(loaded_state.current_layer, updated_layer);
        assert_eq!(loaded_state.file_position, updated_file_pos);

        // 7. Clean up the recovery file by stopping the print tracking.
        new_manager.stop_print_tracking().expect("Failed to stop tracking and clean up");
        assert!(!new_manager.has_recovery_data(), "Recovery file was not cleaned up after stopping.");
    }

    // Final check to ensure the file is gone from the filesystem.
    assert!(!recovery_file_path.exists(), "Recovery file was not cleaned up from the filesystem.");
}
