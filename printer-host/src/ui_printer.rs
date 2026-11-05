//! The Printer page. Settings save themselves whenever they change, wherever in the app they
//! were changed from, as long as they make sense together.

use super::{ui_shell, MotorType, MyApp, PrinterSettings};
use eframe::egui;

pub const SETTINGS_FILE: &str = "printer_settings.toml";

fn stored_only_note(ui: &mut egui::Ui) {
    ui.colored_label(ui_shell::muted(), "Stored only: nothing uses this until a printer is connected.");
}

fn used_note(ui: &mut egui::Ui, text: &str) {
    ui.colored_label(ui_shell::info(), text);
}

impl MyApp {
    /// Read the saved settings at startup. A file we can't read is left alone and reported.
    pub(super) fn load_saved_settings(&mut self) {
        match PrinterSettings::load_from_file(&self.settings_path) {
            Ok(Some(saved)) => {
                self.printer_settings = saved.clone();
                self.last_saved_settings = saved;
            }
            Ok(None) => {}
            Err(e) => {
                self.settings_save_blocked = true;
                self.settings_message = Some((
                    format!("Could not read {}: {}. Nothing will be saved over it until you fix or move that file.", self.settings_path.display(), e),
                    false,
                ));
            }
        }
    }

    /// Save once a change is finished. Settings that don't make sense aren't saved: the Printer
    /// page says what's wrong, and the file keeps the last good ones.
    pub(super) fn save_settings_if_changed(&mut self, ctx: &egui::Context) {
        if self.printer_settings == self.last_saved_settings || self.settings_save_blocked || ctx.dragged_id().is_some() {
            return;
        }
        if !self.printer_settings.problems().is_empty() {
            return;
        }
        match self.printer_settings.save_to_file(&self.settings_path) {
            Ok(()) => {
                self.last_saved_settings = self.printer_settings.clone();
                self.settings_message = Some(("Saved".to_string(), true));
                ctx.request_repaint(); // Saving happens after drawing, so show the result now
            }
            Err(e) => self.settings_message = Some((format!("Could not save {}: {}", self.settings_path.display(), e), false)),
        }
    }

    pub(super) fn show_printer_page(&mut self, ui: &mut egui::Ui) {
        ui.heading("Printer Configuration");
        ui.label("Changes save automatically. Each section says what uses it.");

        // This line is always there, so a message appearing can't move the sliders mid-drag.
        let problems = self.printer_settings.problems();
        if problems.is_empty() {
            match &self.settings_message {
                Some((message, ok)) => {
                    ui.colored_label(if *ok { ui_shell::good() } else { ui_shell::danger() }, message);
                }
                None => {
                    ui.label(" ");
                }
            }
        } else {
            ui.colored_label(ui_shell::danger(), format!("Not saved: {}", problems.join(". ")));
        }

        ui.horizontal_wrapped(|ui| {
            if self.confirm_settings_reset {
                ui.colored_label(ui_shell::warning(), "Reset every setting on this page?");
                if ui.button("Yes, reset").clicked() {
                    self.printer_settings = PrinterSettings::default();
                    self.confirm_settings_reset = false;
                }
                if ui.button("Cancel").clicked() {
                    self.confirm_settings_reset = false;
                }
            } else if ui.button("🔄 Reset to Defaults").clicked() {
                self.confirm_settings_reset = true;
            }
        });

        egui::ScrollArea::vertical().id_salt("printer_config_scroll").show(ui, |ui| {
            ui.group(|ui| {
                ui.label("🌡 Temperature Settings");
                used_note(ui, "Limits heaters on the Temperature page and in the simulator.");
                ui.add(egui::Slider::new(&mut self.printer_settings.extruder_max_temp, 180.0..=400.0).text("Max Extruder Temp (°C)"));
                ui.add(egui::Slider::new(&mut self.printer_settings.bed_max_temp, 60.0..=150.0).text("Max Bed Temp (°C)"));
                ui.add(egui::Slider::new(&mut self.printer_settings.extruder_min_extrude_temp, 160.0..=200.0).text("Min Extrude Temp (°C)"));
            });

            ui.separator();

            ui.group(|ui| {
                ui.label("🏃 Movement Settings");
                used_note(ui, "Max velocity and acceleration limit the simulator. The other two are stored only.");
                ui.add(egui::Slider::new(&mut self.printer_settings.max_velocity, 50.0..=500.0).text("Max Velocity (mm/s)"));
                ui.add(egui::Slider::new(&mut self.printer_settings.max_accel, 500.0..=10000.0).text("Max Acceleration (mm/s²)"));
                ui.add(egui::Slider::new(&mut self.printer_settings.max_accel_to_decel, 250.0..=5000.0).text("Max Accel to Decel (mm/s²)"));
                ui.add(egui::Slider::new(&mut self.printer_settings.square_corner_velocity, 1.0..=20.0).text("Square Corner Velocity (mm/s)"));
            });

            ui.separator();

            ui.group(|ui| {
                ui.label("🏠 Homing Settings");
                stored_only_note(ui);
                ui.add(egui::Slider::new(&mut self.printer_settings.homing_speed, 10.0..=100.0).text("Homing Speed (mm/s)"));
                ui.add(egui::Slider::new(&mut self.printer_settings.homing_retract_dist, 1.0..=20.0).text("Homing Retract Distance (mm)"));
                ui.add(egui::Slider::new(&mut self.printer_settings.second_homing_speed, 5.0..=50.0).text("Second Homing Speed (mm/s)"));
            });

            ui.separator();

            ui.group(|ui| {
                ui.label("📏 Print Area Limits");
                used_note(ui, "Limits the position sliders on the Kinematics page, and the simulator.");
                ui.horizontal_wrapped(|ui| {
                    ui.label("X min/max:");
                    ui.add(egui::DragValue::new(&mut self.printer_settings.position_min[0]).speed(1.0));
                    ui.add(egui::DragValue::new(&mut self.printer_settings.position_max[0]).speed(1.0));
                });
                ui.horizontal_wrapped(|ui| {
                    ui.label("Y min/max:");
                    ui.add(egui::DragValue::new(&mut self.printer_settings.position_min[1]).speed(1.0));
                    ui.add(egui::DragValue::new(&mut self.printer_settings.position_max[1]).speed(1.0));
                });
                ui.horizontal_wrapped(|ui| {
                    ui.label("Z min/max:");
                    ui.add(egui::DragValue::new(&mut self.printer_settings.position_min[2]).speed(1.0));
                    ui.add(egui::DragValue::new(&mut self.printer_settings.position_max[2]).speed(1.0));
                });
            });

            ui.separator();

            ui.group(|ui| {
                ui.label("⚙ Motor Configuration");
                stored_only_note(ui);
                let motor_labels = ["X-Axis", "Y-Axis", "Z-Axis", "Extruder 0", "Extruder 1"];

                for (i, label) in motor_labels.iter().enumerate() {
                    ui.horizontal_wrapped(|ui| {
                        ui.label(format!("{}:", label));
                        egui::ComboBox::from_id_salt(format!("motor_{}", i))
                            .selected_text(format!("{:?}", self.printer_settings.motor_types[i]))
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut self.printer_settings.motor_types[i], MotorType::Stepper, "Stepper");
                                ui.selectable_value(&mut self.printer_settings.motor_types[i], MotorType::Servo, "Servo");
                            });
                    });
                }
            });

            ui.separator();

            ui.group(|ui| {
                ui.label("⚡ Stepper Motor Settings");
                stored_only_note(ui);
                ui.add(egui::Slider::new(&mut self.printer_settings.hold_current, 0.1..=2.0).text("Hold Current (A)"));
                ui.add(egui::Slider::new(&mut self.printer_settings.run_current, 0.2..=2.5).text("Run Current (A)"));
                egui::ComboBox::from_label("Microsteps")
                    .selected_text(format!("{}", self.printer_settings.microsteps))
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.printer_settings.microsteps, 16, "16");
                        ui.selectable_value(&mut self.printer_settings.microsteps, 32, "32");
                        ui.selectable_value(&mut self.printer_settings.microsteps, 64, "64");
                        ui.selectable_value(&mut self.printer_settings.microsteps, 128, "128");
                        ui.selectable_value(&mut self.printer_settings.microsteps, 256, "256");
                    });
            });

            ui.separator();

            ui.group(|ui| {
                ui.label("🔧 Servo Motor Settings");
                stored_only_note(ui);
                ui.add(egui::Slider::new(&mut self.printer_settings.servo_min_pulse, 500..=1500).text("Min Pulse Width (µs)"));
                ui.add(egui::Slider::new(&mut self.printer_settings.servo_max_pulse, 1500..=3000).text("Max Pulse Width (µs)"));
                ui.add(egui::Slider::new(&mut self.printer_settings.servo_max_angle, 90.0..=360.0).text("Max Angle (degrees)"));
                ui.label("Common servo types:");
                ui.horizontal_wrapped(|ui| {
                    if ui.button("Standard (500-2500µs)").clicked() {
                        self.printer_settings.servo_min_pulse = 500;
                        self.printer_settings.servo_max_pulse = 2500;
                        self.printer_settings.servo_max_angle = 180.0;
                    }
                    if ui.button("Digital (600-2400µs)").clicked() {
                        self.printer_settings.servo_min_pulse = 600;
                        self.printer_settings.servo_max_pulse = 2400;
                        self.printer_settings.servo_max_angle = 180.0;
                    }
                    if ui.button("Continuous (1000-2000µs)").clicked() {
                        self.printer_settings.servo_min_pulse = 1000;
                        self.printer_settings.servo_max_pulse = 2000;
                        self.printer_settings.servo_max_angle = 360.0;
                    }
                });
            });

            ui.separator();

            ui.group(|ui| {
                ui.label("🌊 Pressure Advance");
                stored_only_note(ui);
                ui.add(egui::Slider::new(&mut self.printer_settings.pressure_advance, 0.0..=0.2).text("Pressure Advance"));
                ui.add(egui::Slider::new(&mut self.printer_settings.pressure_advance_smooth_time, 0.01..=0.1).text("Smooth Time (s)"));
                ui.label("⚠ Start with small values (0.01-0.05) and increase gradually");
            });

            ui.separator();

            ui.group(|ui| {
                ui.label("🔍 Z Probe Settings");
                stored_only_note(ui);
                ui.add(egui::Slider::new(&mut self.printer_settings.probe_z_offset, -5.0..=5.0).text("Probe Z Offset (mm)"));
                ui.horizontal_wrapped(|ui| {
                    ui.label("Probe XY Offset:");
                    ui.add(egui::DragValue::new(&mut self.printer_settings.probe_xy_offset[0]).prefix("X:").speed(0.1));
                    ui.add(egui::DragValue::new(&mut self.printer_settings.probe_xy_offset[1]).prefix("Y:").speed(0.1));
                });
                ui.add(egui::Slider::new(&mut self.printer_settings.probe_speed, 1.0..=20.0).text("Probe Speed (mm/s)"));
                ui.add(egui::Slider::new(&mut self.printer_settings.probe_lift_speed, 5.0..=50.0).text("Lift Speed (mm/s)"));
                ui.add(egui::Slider::new(&mut self.printer_settings.probe_samples, 1..=10).text("Samples per Point"));
                ui.add(egui::Slider::new(&mut self.printer_settings.probe_sample_retract_dist, 0.5..=10.0).text("Sample Retract Distance (mm)"));
            });

            ui.separator();

            ui.group(|ui| {
                ui.label("🔄 5-Axis Control");
                ui.checkbox(&mut self.printer_settings.enable_5axis_mode, "Enable 5-Axis Mode");
                if self.printer_settings.enable_5axis_mode {
                    ui.checkbox(&mut self.printer_settings.tool_point_control, "Tool Center Point (TCP) Control");
                    stored_only_note(ui);
                    ui.horizontal_wrapped(|ui| {
                        ui.label("A-Axis Max Velocity:");
                        ui.add(egui::DragValue::new(&mut self.printer_settings.rotary_axis_max_velocity[0]).suffix("°/s").speed(1.0));
                    });
                    ui.horizontal_wrapped(|ui| {
                        ui.label("B-Axis Max Velocity:");
                        ui.add(egui::DragValue::new(&mut self.printer_settings.rotary_axis_max_velocity[1]).suffix("°/s").speed(1.0));
                    });
                    ui.horizontal_wrapped(|ui| {
                        ui.label("A-Axis Acceleration:");
                        ui.add(egui::DragValue::new(&mut self.printer_settings.rotary_axis_acceleration[0]).suffix("°/s²").speed(10.0));
                    });
                    ui.horizontal_wrapped(|ui| {
                        ui.label("B-Axis Acceleration:");
                        ui.add(egui::DragValue::new(&mut self.printer_settings.rotary_axis_acceleration[1]).suffix("°/s²").speed(10.0));
                    });
                    ui.label("⚠ TCP control maintains tool tip position during rotary movement");
                }
            });

            ui.separator();

            ui.group(|ui| {
                ui.label("🔄 Firmware Retraction");
                stored_only_note(ui);
                ui.add(egui::Slider::new(&mut self.printer_settings.retract_length, 0.0..=10.0).text("Retract Length (mm)"));
                ui.add(egui::Slider::new(&mut self.printer_settings.retract_speed, 10.0..=100.0).text("Retract Speed (mm/s)"));
                ui.add(egui::Slider::new(&mut self.printer_settings.unretract_extra_length, 0.0..=2.0).text("Unretract Extra Length (mm)"));
                ui.add(egui::Slider::new(&mut self.printer_settings.unretract_speed, 10.0..=100.0).text("Unretract Speed (mm/s)"));
            });
        });
    }
}

#[cfg(all(test, not(target_os = "linux"), not(all(windows, feature = "windows-ethercat"))))]
mod tests {
    use super::*;
    use egui_kittest::kittest::Queryable;
    use egui_kittest::Harness;
    use std::path::PathBuf;

    /// A test's own folder, removed afterwards, so nothing touches real settings.
    struct Folder(PathBuf);
    impl Folder {
        fn new(name: &str) -> Self {
            let dir = std::env::temp_dir().join(format!("printer_page_{}_{}", name, std::process::id()));
            std::fs::create_dir_all(&dir).unwrap();
            Self(dir)
        }
        fn file(&self) -> PathBuf {
            self.0.join("printer_settings.toml")
        }
    }
    impl Drop for Folder {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    fn harness(folder: &Folder) -> Harness<'static, MyApp> {
        let mut app = crate::test_app();
        app.settings_path = folder.file();
        app.load_saved_settings();
        // Tall enough to show the whole page without scrolling
        Harness::builder().with_size(egui::vec2(1000.0, 2400.0)).build_state(
            |ctx, app: &mut MyApp| {
                egui::CentralPanel::default().show(ctx, |ui| app.show_printer_page(ui));
                app.save_settings_if_changed(ctx);
            },
            app,
        )
    }

    #[test]
    fn nothing_is_written_until_something_changes() {
        let folder = Folder::new("idle");
        let mut h = harness(&folder);
        h.run();
        assert!(!folder.file().exists(), "just opening the page must not write a file");
    }

    #[test]
    fn a_change_is_saved_and_comes_back_after_a_restart() {
        let folder = Folder::new("restart");
        let mut h = harness(&folder);
        h.state_mut().printer_settings.max_velocity = 123.5;
        h.state_mut().printer_settings.position_max = [150.0, 160.0, 170.0];
        h.state_mut().printer_settings.motor_types[2] = MotorType::Servo;
        h.run();
        h.get_by_label("Saved");

        let reopened = harness(&folder);
        let settings = &reopened.state().printer_settings;
        assert_eq!(settings.max_velocity, 123.5);
        assert_eq!(settings.position_max, [150.0, 160.0, 170.0]);
        assert_eq!(settings.motor_types[2], MotorType::Servo);
        assert_eq!(reopened.state().last_saved_settings, *settings, "a reload isn't a change to save");
    }

    #[test]
    fn the_z_offset_buttons_on_the_side_panel_save_too() {
        let folder = Folder::new("zoffset");
        let mut h = harness(&folder);
        h.state_mut().adjust_z_offset(0.05);
        h.run();
        let saved = PrinterSettings::load_from_file(&folder.file()).unwrap().unwrap();
        assert!((saved.z_offset - 0.05).abs() < 1e-6);
    }

    #[test]
    fn settings_that_make_no_sense_are_explained_and_not_saved() {
        let folder = Folder::new("invalid");
        let mut h = harness(&folder);
        h.state_mut().printer_settings.max_velocity = 100.0;
        h.run();

        h.state_mut().printer_settings.position_min[0] = 500.0;
        h.run();
        h.get_by_label("Not saved: X minimum must be below X maximum");
        let saved = PrinterSettings::load_from_file(&folder.file()).unwrap().unwrap();
        assert_eq!(saved.position_min[0], 0.0, "the file keeps the last good settings");
        assert_eq!(saved.max_velocity, 100.0);

        // Fixing it saves again, including what was changed in the meantime
        h.state_mut().printer_settings.max_velocity = 90.0;
        h.state_mut().printer_settings.position_min[0] = 5.0;
        h.run();
        let saved = PrinterSettings::load_from_file(&folder.file()).unwrap().unwrap();
        assert_eq!((saved.position_min[0], saved.max_velocity), (5.0, 90.0));
    }

    #[test]
    fn a_damaged_file_is_reported_and_left_alone() {
        let folder = Folder::new("damaged");
        std::fs::write(folder.file(), "this is = = not toml").unwrap();
        let mut h = harness(&folder);
        h.state_mut().printer_settings.max_velocity = 100.0;
        h.run();
        assert_eq!(std::fs::read_to_string(folder.file()).unwrap(), "this is = = not toml");
        assert!(h.state().settings_save_blocked);
    }

    #[test]
    fn an_older_file_missing_some_settings_keeps_the_rest() {
        let folder = Folder::new("older");
        std::fs::write(folder.file(), "max_velocity = 111.0\n").unwrap();
        let h = harness(&folder);
        assert_eq!(h.state().printer_settings.max_velocity, 111.0);
        assert_eq!(h.state().printer_settings.bed_max_temp, PrinterSettings::default().bed_max_temp);
    }

    #[test]
    fn reset_asks_first_and_restores_the_defaults() {
        let folder = Folder::new("reset");
        let mut h = harness(&folder);
        h.state_mut().printer_settings.max_velocity = 100.0;
        h.run();

        h.get_by_label("🔄 Reset to Defaults").click();
        h.run();
        assert_eq!(h.state().printer_settings.max_velocity, 100.0, "nothing changes before confirming");
        h.get_by_label("Cancel").click();
        h.run();
        assert_eq!(h.state().printer_settings.max_velocity, 100.0);

        h.get_by_label("🔄 Reset to Defaults").click();
        h.run();
        h.get_by_label("Yes, reset").click();
        h.run();
        assert_eq!(h.state().printer_settings, PrinterSettings::default());
        let saved = PrinterSettings::load_from_file(&folder.file()).unwrap().unwrap();
        assert_eq!(saved, PrinterSettings::default());
    }

    #[test]
    fn the_servo_preset_buttons_set_their_values() {
        let folder = Folder::new("servo");
        let mut h = harness(&folder);
        h.get_by_label("Digital (600-2400µs)").click();
        h.run();
        let settings = &h.state().printer_settings;
        assert_eq!((settings.servo_min_pulse, settings.servo_max_pulse), (600, 2400));
    }

    #[test]
    fn the_five_axis_checkbox_reveals_its_settings() {
        let folder = Folder::new("five_axis");
        let mut h = harness(&folder);
        assert!(h.query_by_label("Tool Center Point (TCP) Control").is_none());
        h.get_by_label("Enable 5-Axis Mode").click();
        h.run();
        assert!(h.state().printer_settings.enable_5axis_mode);
        h.get_by_label("Tool Center Point (TCP) Control");
    }

    #[test]
    fn heater_limits_come_from_the_settings() {
        let mut settings = PrinterSettings::default();
        settings.extruder_max_temp = 260.0;
        settings.bed_max_temp = 90.0;
        assert_eq!(crate::zone_temperature_limit(&settings, &crate::TemperatureZone::Hotend(0)), 260.0);
        assert_eq!(crate::zone_temperature_limit(&settings, &crate::TemperatureZone::Bed), 90.0);
    }
}
