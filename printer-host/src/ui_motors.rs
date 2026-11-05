//! The Motors page and its Add Motor dialog.
//!
//! This edits the machine's motor configuration and keeps it in motors.json. Nothing connects it
//! to real drivers yet, so anything that needs hardware is greyed out and says so.

use super::{ui_shell, MyApp};
use eframe::egui;
use printer_host::motor_assignment::{
    MotorAssignmentManager, MotorCapabilities, MotorConfiguration, MotorConnection, MotorFunction, MotorType,
};
use std::path::Path;

const MOTORS_FILE: &str = "motors.json";
/// Tall enough to show every job without scrolling inside the menu.
const JOB_MENU_HEIGHT: f32 = 480.0;

/// A choice that replaces what's there, waiting for the user to confirm it.
#[derive(Clone, Copy, PartialEq)]
pub enum PendingMotorChange {
    Cartesian,
    CoreXY,
    ClearAll,
}

impl PendingMotorChange {
    fn question(self, count: usize) -> String {
        match self {
            PendingMotorChange::Cartesian => format!("Replace your {} motors with the Cartesian setup?", count),
            PendingMotorChange::CoreXY => format!("Replace your {} motors with the CoreXY setup?", count),
            PendingMotorChange::ClearAll => format!("Remove all {} motors?", count),
        }
    }
}

enum Action {
    Request(PendingMotorChange),
    Confirm(PendingMotorChange),
    Cancel,
    OpenAddDialog,
    Select(String),
    Remove(String),
    Clone(String),
    SetEnabled(String, bool),
    Assign(String, MotorFunction),
    Update(MotorConfiguration),
}

const MOTOR_TYPES: [(&str, MotorType); 5] = [
    ("Stepper Motor", MotorType::Stepper),
    ("Servo Motor", MotorType::Servo),
    ("Brushless DC Motor", MotorType::BrushlessDC),
    ("Brushed DC Motor", MotorType::BrushedDC),
    ("Linear Actuator", MotorType::LinearActuator),
];

fn type_name(motor_type: &MotorType) -> &'static str {
    MOTOR_TYPES.iter().find(|(_, t)| t == motor_type).map(|(name, _)| *name).unwrap_or("Motor")
}

impl MyApp {
    /// Read the saved motors at startup. A file we can't read is left alone and reported.
    pub(super) fn load_saved_motors(&mut self) {
        match MotorAssignmentManager::load_from_file(Path::new(MOTORS_FILE)) {
            Ok(Some(saved)) => self.motor_manager = saved,
            Ok(None) => {}
            Err(e) => {
                self.motors_save_blocked = true;
                self.motor_message = Some((
                    format!("Could not read {}: {}. Nothing will be saved over it until you fix or move that file.", MOTORS_FILE, e),
                    false,
                ));
            }
        }
    }

    fn report_motors(&mut self, result: Result<String, String>) {
        match result {
            Ok(message) => {
                self.motors_unsaved = true;
                self.motor_message = Some((message, true));
            }
            Err(e) => self.motor_message = Some((e, false)),
        }
    }

    /// Change the whole setup. Also the handler for the buttons that need no confirmation.
    fn apply_motor_change(&mut self, change: PendingMotorChange) {
        let result = match change {
            PendingMotorChange::Cartesian => self
                .motor_manager
                .replace_with_setup(MotorAssignmentManager::create_standard_cartesian_setup)
                .map(|_| "Cartesian setup applied".to_string()),
            PendingMotorChange::CoreXY => self
                .motor_manager
                .replace_with_setup(MotorAssignmentManager::create_corexy_setup)
                .map(|_| "CoreXY setup applied".to_string()),
            PendingMotorChange::ClearAll => Ok(format!("Removed {} motors", self.motor_manager.clear_all_motors())),
        };
        self.selected_motor_id = None;
        self.report_motors(result);
    }

    fn run_motor_action(&mut self, action: Action) {
        match action {
            Action::Request(change) => {
                if self.motor_manager.get_all_motors().is_empty() {
                    self.apply_motor_change(change);
                } else {
                    self.pending_motor_change = Some(change);
                }
            }
            Action::Confirm(change) => {
                self.pending_motor_change = None;
                self.apply_motor_change(change);
            }
            Action::Cancel => self.pending_motor_change = None,
            Action::OpenAddDialog => {
                self.new_motor_id = self.motor_manager.suggest_motor_id();
                self.new_motor_step_pin.clear();
                self.new_motor_dir_pin.clear();
                self.new_motor_enable_pin.clear();
                self.add_motor_error = None;
                self.show_add_motor_dialog = true;
            }
            Action::Select(id) => self.selected_motor_id = Some(id),
            Action::Remove(id) => {
                let result = self.motor_manager.remove_motor(&id).map(|_| format!("Removed {}", id));
                self.selected_motor_id = None;
                self.report_motors(result);
            }
            Action::Clone(id) => {
                let result = self.motor_manager.clone_motor(&id);
                if let Ok(new_id) = &result {
                    self.selected_motor_id = Some(new_id.clone());
                }
                self.report_motors(result.map(|new_id| format!("Added {} as a copy of {}. It has no job yet.", new_id, id)));
            }
            Action::SetEnabled(id, enabled) => {
                let result = self.motor_manager.set_motor_enabled(&id, enabled).map(|_| {
                    format!("{} marked {}. No driver was told: no printer is connected.", id, if enabled { "enabled" } else { "disabled" })
                });
                self.report_motors(result);
            }
            Action::Assign(id, template) => {
                let result = self
                    .motor_manager
                    .assign_to_family(&id, &template, "Assigned on the Motors page")
                    .map(|_| match self.motor_manager.get_motor_config(&id) {
                        Some(config) => format!("{} is now: {}", id, config.function.display_name()),
                        None => id.clone(),
                    });
                self.report_motors(result);
            }
            Action::Update(config) => {
                match self.motor_manager.update_motor_settings(config) {
                    Ok(()) => {
                        self.motors_unsaved = true;
                        self.motor_message = None;
                    }
                    Err(e) => self.motor_message = Some((e, false)),
                }
            }
        }
    }

    /// Write changes to disk, but not on every frame of a drag.
    pub(super) fn save_motors_if_changed(&mut self, ctx: &egui::Context) {
        if !self.motors_unsaved || self.motors_save_blocked || ctx.dragged_id().is_some() {
            return;
        }
        self.motors_unsaved = false;
        if let Err(e) = self.motor_manager.save_to_file(Path::new(MOTORS_FILE)) {
            self.motor_message = Some((format!("Could not save {}: {}", MOTORS_FILE, e), false));
        }
    }

    pub(super) fn show_motors_page(&mut self, ui: &mut egui::Ui) {
        ui.heading("🔧 Motor Assignment & Configuration");
        ui.label("Set up which motor does which job. Your changes are saved automatically.");

        // This line is always there, so a message appearing can't move a button out from under
        // someone about to click it.
        match &self.motor_message {
            Some((message, ok)) => {
                let color = if *ok { ui_shell::good() } else { ui_shell::danger() };
                ui.colored_label(color, message);
            }
            None => {
                ui.label(" ");
            }
        }

        let mut action = None;
        egui::ScrollArea::vertical().id_salt("motor_assignment_main_scroll").show(ui, |ui| {
            ui.columns(2, |columns| {
                columns[0].group(|ui| self.motor_list_column(ui, &mut action));
                columns[1].group(|ui| self.motor_jobs_column(ui, &mut action));
            });
        });
        if let Some(action) = action {
            self.run_motor_action(action);
        }
    }

    fn motor_list_column(&self, ui: &mut egui::Ui, action: &mut Option<Action>) {
        ui.label("🎯 Motors");
        let count = self.motor_manager.get_all_motors().len();

        if let Some(pending) = self.pending_motor_change {
            ui.colored_label(ui_shell::warning(), pending.question(count));
            ui.horizontal_wrapped(|ui| {
                let confirm = egui::Button::new(egui::RichText::new("Yes, replace").color(egui::Color32::WHITE)).fill(ui_shell::ESTOP);
                if ui.add(confirm).clicked() {
                    *action = Some(Action::Confirm(pending));
                }
                if ui.button("Cancel").clicked() {
                    *action = Some(Action::Cancel);
                }
            });
        } else {
            ui.horizontal_wrapped(|ui| {
                if ui.button("📐 Cartesian Setup").on_hover_text("X, Y, two Z motors and an extruder").clicked() {
                    *action = Some(Action::Request(PendingMotorChange::Cartesian));
                }
                if ui.button("🔄 CoreXY Setup").on_hover_text("A and B belts, two Z motors and an extruder").clicked() {
                    *action = Some(Action::Request(PendingMotorChange::CoreXY));
                }
                if ui.button("➕ Add Motor").clicked() {
                    *action = Some(Action::OpenAddDialog);
                }
                let has_selection = self.selected_motor_id.is_some();
                let remove = ui.add_enabled(has_selection, egui::Button::new("🗑 Remove Selected"));
                if remove.on_disabled_hover_text("Select a motor first").clicked() {
                    if let Some(id) = &self.selected_motor_id {
                        *action = Some(Action::Remove(id.clone()));
                    }
                }
                if ui.add_enabled(count > 0, egui::Button::new("🧹 Clear All Motors")).clicked() {
                    *action = Some(Action::Request(PendingMotorChange::ClearAll));
                }
            });
        }

        ui.separator();
        ui.label(format!("🔩 Registered Motors ({})", count));
        if count == 0 {
            ui.colored_label(ui_shell::muted(), "No motors yet. Pick a setup above, or add one.");
        }
        egui::ScrollArea::vertical()
            .id_salt("motor_list_scroll")
            .max_height(260.0)
            .auto_shrink([false, true])
            .show(ui, |ui| {
                for id in self.motor_manager.sorted_motor_ids() {
                    let Some(config) = self.motor_manager.get_motor_config(&id) else { continue };
                    ui.horizontal_wrapped(|ui| {
                        if ui.selectable_label(self.selected_motor_id.as_ref() == Some(&id), &id).clicked() {
                            *action = Some(Action::Select(id.clone()));
                        }
                        let color = if config.function == MotorFunction::Unassigned {
                            ui_shell::warning()
                        } else if config.enabled {
                            ui_shell::good()
                        } else {
                            ui_shell::muted()
                        };
                        ui.colored_label(color, config.function.display_name());
                        if !config.enabled {
                            ui.colored_label(ui_shell::muted(), "(disabled)");
                        }
                    });
                }
            });

        let selected = self.selected_motor_id.as_ref().and_then(|id| self.motor_manager.get_motor_config(id));
        if let Some(config) = selected {
            ui.separator();
            self.motor_details(ui, config, action);
        }
    }

    fn motor_details(&self, ui: &mut egui::Ui, config: &MotorConfiguration, action: &mut Option<Action>) {
        ui.label(format!("🔧 {} ({})", config.motor_id, type_name(&config.motor_type)));
        if let MotorConnection::StepDir { step_pin, dir_pin, enable_pin } = &config.physical_connection {
            ui.colored_label(
                ui_shell::muted(),
                format!("Pins: step {}, dir {}, enable {}", step_pin, dir_pin, enable_pin.as_deref().unwrap_or("none")),
            );
        }

        let mut edited = config.clone();
        let mut changed = false;
        egui::Grid::new("motor_settings").num_columns(2).show(ui, |ui| {
            ui.label("Steps per unit");
            changed |= ui
                .add(egui::DragValue::new(&mut edited.capabilities.steps_per_unit).range(0.1..=100_000.0).speed(1.0))
                .changed();
            ui.end_row();

            ui.label("Max speed");
            changed |= ui.add(egui::DragValue::new(&mut edited.capabilities.max_speed).range(1.0..=100_000.0).speed(10.0)).changed();
            ui.end_row();

            ui.label("Current");
            let max_current = edited.capabilities.max_current.max(0.01);
            changed |= ui
                .add(egui::DragValue::new(&mut edited.current_setting).range(0.01..=max_current).speed(0.01).suffix(" A"))
                .on_hover_text(format!("Up to this motor's rating of {:.2} A", max_current))
                .changed();
            ui.end_row();

            ui.label("Hold current");
            let mut hold_percent = edited.hold_current * 100.0;
            if ui.add(egui::DragValue::new(&mut hold_percent).range(0.0..=100.0).suffix(" %")).changed() {
                edited.hold_current = hold_percent / 100.0;
                changed = true;
            }
            ui.end_row();

            if let Some(microsteps) = edited.microstepping {
                ui.label("Microstepping");
                egui::ComboBox::from_id_salt("motor_microstepping")
                    .selected_text(microsteps.to_string())
                    .show_ui(ui, |ui| {
                        for value in [1u8, 2, 4, 8, 16, 32, 64, 128] {
                            if ui.selectable_label(microsteps == value, value.to_string()).clicked() {
                                edited.microstepping = Some(value);
                                changed = true;
                            }
                        }
                    });
                ui.end_row();
            }

            ui.label("Direction");
            changed |= ui.checkbox(&mut edited.inverted, "Reversed").changed();
            ui.end_row();
        });
        if changed {
            *action = Some(Action::Update(edited));
        }
        if config.capabilities.has_encoder {
            ui.colored_label(ui_shell::good(), "🔄 Encoder Available");
        }
        if config.capabilities.stall_detection {
            ui.colored_label(ui_shell::info(), "⚠ Stall Detection");
        }
    }

    fn motor_jobs_column(&self, ui: &mut egui::Ui, action: &mut Option<Action>) {
        ui.label("🎯 Jobs");
        let manager = &self.motor_manager;
        // On CoreXY, belts A and B drive X and Y together, so X and Y aren't missing a motor.
        let core_xy = !manager.motors_in_family(&MotorFunction::CoreXY_A(0)).is_empty()
            || !manager.motors_in_family(&MotorFunction::CoreXY_B(0)).is_empty();

        egui::Grid::new("function_grid").num_columns(3).show(ui, |ui| {
            ui.strong("Job");
            ui.strong("Motor");
            ui.strong("Status");
            ui.end_row();

            for (name, template) in MotorFunction::assignable() {
                let motors = manager.motors_in_family(&template);
                let required = matches!(template, MotorFunction::XAxis(_) | MotorFunction::YAxis(_) | MotorFunction::ZAxis(_) | MotorFunction::Extruder(_));
                // Keep the table short: rows for optional jobs appear once they're in use
                if motors.is_empty() && !required {
                    continue;
                }
                ui.label(name);
                if motors.is_empty() {
                    if core_xy && matches!(template, MotorFunction::XAxis(_) | MotorFunction::YAxis(_)) {
                        ui.colored_label(ui_shell::good(), "CoreXY A + B");
                        ui.colored_label(ui_shell::muted(), "Via belts");
                    } else {
                        ui.colored_label(ui_shell::danger(), "Unassigned");
                        ui.colored_label(ui_shell::danger(), "Needs a motor");
                    }
                } else {
                    let names: Vec<&str> = motors.iter().map(|m| m.motor_id.as_str()).collect();
                    ui.label(names.join(", "));
                    let all_enabled = motors.iter().all(|m| m.enabled);
                    if all_enabled {
                        ui.colored_label(ui_shell::good(), "Enabled");
                    } else {
                        ui.colored_label(ui_shell::warning(), "Has a disabled motor");
                    }
                }
                ui.end_row();
            }
        });

        ui.separator();

        if let Some(id) = self.selected_motor_id.clone() {
            if let Some(config) = manager.get_motor_config(&id) {
                ui.label(format!("🔧 Selected: {}", id));
                ui.horizontal_wrapped(|ui| {
                    ui.label("Job:");
                    egui::ComboBox::from_id_salt("motor_assign_job")
                        .selected_text(config.function.display_name())
                        .height(JOB_MENU_HEIGHT)
                        .show_ui(ui, |ui| {
                            let same = |t: &MotorFunction| std::mem::discriminant(t) == std::mem::discriminant(&config.function);
                            for (name, template) in MotorFunction::assignable() {
                                if ui.selectable_label(same(&template), name).clicked() {
                                    *action = Some(Action::Assign(id.clone(), template));
                                }
                            }
                            if ui.selectable_label(config.function == MotorFunction::Unassigned, "Unassigned").clicked() {
                                *action = Some(Action::Assign(id.clone(), MotorFunction::Unassigned));
                            }
                        });
                });
                ui.horizontal_wrapped(|ui| {
                    let can_enable = !config.enabled && !self.emergency_stop_triggered;
                    let enable = ui.add_enabled(can_enable, egui::Button::new("▶ Enable"));
                    if enable
                        .on_disabled_hover_text(if self.emergency_stop_triggered { "Reset the emergency stop first" } else { "Already enabled" })
                        .clicked()
                    {
                        *action = Some(Action::SetEnabled(id.clone(), true));
                    }
                    let disable = ui.add_enabled(config.enabled, egui::Button::new("⏸ Disable"));
                    if disable.on_disabled_hover_text("Already disabled").clicked() {
                        *action = Some(Action::SetEnabled(id.clone(), false));
                    }
                    if ui.button("➕ Clone").clicked() {
                        *action = Some(Action::Clone(id.clone()));
                    }
                    ui_shell::needs_printer_button(ui, "🏠 Home");
                });
            }
        } else {
            ui.colored_label(ui_shell::muted(), "Select a motor to change its job or settings.");
        }

        ui.separator();
        ui.label("📜 Recent Changes");
        egui::ScrollArea::vertical()
            .id_salt("motor_assignment_history_scroll")
            .max_height(180.0)
            .auto_shrink([false, true])
            .show(ui, |ui| {
                let history = manager.get_assignment_history();
                if history.is_empty() {
                    ui.colored_label(ui_shell::muted(), "Nothing yet");
                }
                for event in history.iter().rev().take(10) {
                    ui.horizontal_wrapped(|ui| {
                        ui.label(&event.motor_id);
                        ui.label("→");
                        ui.colored_label(ui_shell::good(), event.new_function.display_name());
                        ui.colored_label(ui_shell::muted(), &event.reason);
                    });
                }
            });
    }

    /// Why this new motor can't be added, if there's a reason.
    fn new_motor_problem(&self) -> Option<String> {
        let id = self.new_motor_id.trim();
        if id.is_empty() {
            return Some("Give the motor a name".to_string());
        }
        if self.motor_manager.get_motor_config(id).is_some() {
            return Some(format!("There is already a motor called {}", id));
        }
        let step = self.new_motor_step_pin.trim();
        let dir = self.new_motor_dir_pin.trim();
        let enable = self.new_motor_enable_pin.trim();
        if step.is_empty() || dir.is_empty() {
            return Some("Step and direction pins are required".to_string());
        }
        if step == dir || (!enable.is_empty() && (enable == step || enable == dir)) {
            return Some("Each pin can only be used once".to_string());
        }
        for pin in [step, dir, enable] {
            if !pin.is_empty() {
                if let Some(owner) = self.motor_manager.pin_owner(pin) {
                    return Some(format!("Pin {} is already used by {}", pin, owner));
                }
            }
        }
        None
    }

    pub(super) fn show_add_motor_window(&mut self, ctx: &egui::Context) {
        if !self.show_add_motor_dialog {
            return;
        }
        egui::Window::new("Add Motor").collapsible(false).resizable(false).show(ctx, |ui| {
            egui::Grid::new("add_motor_form").num_columns(2).show(ui, |ui| {
                ui.label("Name");
                ui.text_edit_singleline(&mut self.new_motor_id);
                ui.end_row();

                ui.label("Job");
                egui::ComboBox::from_id_salt("add_motor_job")
                    .selected_text(self.new_motor_function.display_name())
                    .height(JOB_MENU_HEIGHT)
                    .show_ui(ui, |ui| {
                        for (name, template) in MotorFunction::assignable() {
                            ui.selectable_value(&mut self.new_motor_function, template, name);
                        }
                        ui.selectable_value(&mut self.new_motor_function, MotorFunction::Unassigned, "Unassigned");
                    });
                ui.end_row();

                ui.label("Type");
                egui::ComboBox::from_id_salt("add_motor_type")
                    .selected_text(type_name(&self.new_motor_type))
                    .show_ui(ui, |ui| {
                        for (name, motor_type) in MOTOR_TYPES {
                            ui.selectable_value(&mut self.new_motor_type, motor_type, name);
                        }
                    });
                ui.end_row();

                ui.label("Step pin");
                ui.text_edit_singleline(&mut self.new_motor_step_pin);
                ui.end_row();
                ui.label("Direction pin");
                ui.text_edit_singleline(&mut self.new_motor_dir_pin);
                ui.end_row();
                ui.label("Enable pin");
                ui.add(egui::TextEdit::singleline(&mut self.new_motor_enable_pin).hint_text("Optional"));
                ui.end_row();
            });

            let problem = self.new_motor_problem();
            if let Some(problem) = problem.as_ref().or(self.add_motor_error.as_ref()) {
                ui.colored_label(ui_shell::warning(), problem);
            }
            ui.horizontal_wrapped(|ui| {
                if ui.add_enabled(problem.is_none(), egui::Button::new("✅ Add Motor")).clicked() {
                    self.add_new_motor();
                }
                if ui.button("Cancel").clicked() {
                    self.show_add_motor_dialog = false;
                }
            });
        });
    }

    fn add_new_motor(&mut self) {
        let enable = self.new_motor_enable_pin.trim();
        let function = self.motor_manager.next_free_function(&self.new_motor_function);
        let config = MotorConfiguration {
            motor_id: self.new_motor_id.trim().to_string(),
            function,
            motor_type: self.new_motor_type.clone(),
            physical_connection: MotorConnection::StepDir {
                step_pin: self.new_motor_step_pin.trim().to_string(),
                dir_pin: self.new_motor_dir_pin.trim().to_string(),
                enable_pin: (!enable.is_empty()).then(|| enable.to_string()),
            },
            capabilities: MotorCapabilities {
                max_speed: 1000.0,
                max_acceleration: 3000.0,
                steps_per_unit: 80.0,
                max_current: 1.2,
                min_current: 0.1,
                has_encoder: false,
                encoder_resolution: None,
                has_endstops: true,
                can_reverse: true,
                thermal_protection: true,
                stall_detection: false,
            },
            enabled: true,
            inverted: false,
            microstepping: Some(16),
            current_setting: 0.8,
            hold_current: 0.5,
            interpolation: true,
            created_at: std::time::Instant::now(),
            last_modified: std::time::Instant::now(),
        };
        match self.motor_manager.register_motor(config) {
            Ok(id) => {
                self.selected_motor_id = Some(id.clone());
                self.show_add_motor_dialog = false;
                self.report_motors(Ok(format!("Added {}. Check its steps per unit and current.", id)));
            }
            Err(e) => self.add_motor_error = Some(e),
        }
    }
}

/// These run the real page without a window: find a control by what it says, use it, and check
/// that the machine's configuration changed the way the control promises.
#[cfg(all(test, not(target_os = "linux"), not(all(windows, feature = "windows-ethercat"))))]
mod tests {
    use super::*;
    use egui_kittest::kittest::{NodeT, Queryable};
    use egui_kittest::Harness;

    fn harness() -> Harness<'static, MyApp> {
        Harness::new_state(
            |ctx, app: &mut MyApp| {
                egui::CentralPanel::default().show(ctx, |ui| app.show_motors_page(ui));
                app.show_add_motor_window(ctx);
            },
            crate::test_app(),
        )
    }

    /// Click the first control with this text. A motor's name shows in the list first, then the table.
    fn click(harness: &mut Harness<'static, MyApp>, label: &str) {
        harness.get_all_by_label(label).next().unwrap_or_else(|| panic!("no control says {}", label)).click();
        harness.run();
    }

    fn with_cartesian() -> Harness<'static, MyApp> {
        let mut h = harness();
        click(&mut h, "📐 Cartesian Setup");
        h
    }

    #[test]
    fn a_setup_on_an_empty_machine_applies_straight_away() {
        let h = with_cartesian();
        assert_eq!(h.state().motor_manager.get_all_motors().len(), 5);
        assert!(h.state().motors_unsaved, "the change must be queued for saving");
    }

    #[test]
    fn replacing_motors_asks_first_and_cancel_keeps_them() {
        let mut h = with_cartesian();

        click(&mut h, "🔄 CoreXY Setup");
        assert!(h.state().motor_manager.get_motor_config("x_motor").is_some(), "nothing changes before confirming");
        h.get_by_label("Replace your 5 motors with the CoreXY setup?");

        click(&mut h, "Cancel");
        assert!(h.state().motor_manager.get_motor_config("x_motor").is_some());

        click(&mut h, "🔄 CoreXY Setup");
        click(&mut h, "Yes, replace");
        assert!(h.state().motor_manager.get_motor_config("x_motor").is_none());
        assert!(h.state().motor_manager.get_motor_config("a_motor").is_some());
    }

    #[test]
    fn clear_all_removes_everything_after_confirming() {
        let mut h = with_cartesian();
        click(&mut h, "🧹 Clear All Motors");
        assert_eq!(h.state().motor_manager.get_all_motors().len(), 5);
        click(&mut h, "Yes, replace");
        assert!(h.state().motor_manager.get_all_motors().is_empty());
        assert!(h.state().motor_manager.get_all_axis_groups().is_empty());
    }

    #[test]
    fn selecting_a_motor_then_disable_enable_clone_and_remove() {
        let mut h = with_cartesian();
        click(&mut h, "x_motor");
        assert_eq!(h.state().selected_motor_id.as_deref(), Some("x_motor"));

        click(&mut h, "⏸ Disable");
        assert!(!h.state().motor_manager.get_motor_config("x_motor").unwrap().enabled);
        click(&mut h, "▶ Enable");
        assert!(h.state().motor_manager.get_motor_config("x_motor").unwrap().enabled);

        click(&mut h, "➕ Clone");
        assert!(h.state().motor_manager.get_motor_config("x_motor_clone").is_some());
        assert_eq!(h.state().selected_motor_id.as_deref(), Some("x_motor_clone"));

        click(&mut h, "🗑 Remove Selected");
        assert!(h.state().motor_manager.get_motor_config("x_motor_clone").is_none());
        assert!(h.state().selected_motor_id.is_none());
        assert_eq!(h.state().motor_manager.get_all_motors().len(), 5);
    }

    #[test]
    fn choosing_a_job_from_the_menu_assigns_it() {
        let mut h = with_cartesian();
        click(&mut h, "x_motor");
        // The job menu is the last one on the page, after the motor's settings
        h.get_all_by_role(egui::accesskit::Role::ComboBox).last().unwrap().click();
        h.run();
        h.get_by_label("Tool Changer").hover();
        h.run();
        h.get_by_label("Tool Changer").click();
        h.run();
        assert_eq!(h.state().motor_manager.get_motor_config("x_motor").unwrap().function, MotorFunction::ToolChanger);
    }

    #[test]
    fn the_add_motor_form_refuses_bad_input_and_adds_good_input() {
        let mut h = harness();
        click(&mut h, "➕ Add Motor");
        assert!(h.state().show_add_motor_dialog);
        h.get_by_label("Step and direction pins are required");
        assert!(h.get_by_label("✅ Add Motor").accesskit_node().is_disabled(), "no pins yet");

        h.state_mut().new_motor_step_pin = "PA2".into();
        h.state_mut().new_motor_dir_pin = "PA2".into();
        h.run();
        h.get_by_label("Each pin can only be used once");

        h.state_mut().new_motor_dir_pin = "PA3".into();
        h.run();
        click(&mut h, "✅ Add Motor");
        assert!(!h.state().show_add_motor_dialog);
        let id = h.state().selected_motor_id.clone().unwrap();
        assert_eq!(h.state().motor_manager.get_motor_config(&id).unwrap().function, MotorFunction::XAxis(0));

        // A second motor can't reuse those pins, and takes the next X slot when it has its own
        click(&mut h, "➕ Add Motor");
        h.state_mut().new_motor_step_pin = "PA2".into();
        h.state_mut().new_motor_dir_pin = "PA5".into();
        h.run();
        h.get_by_label(&format!("Pin PA2 is already used by {}", id));
        h.state_mut().new_motor_step_pin = "PA4".into();
        h.run();
        click(&mut h, "✅ Add Motor");
        let second = h.state().selected_motor_id.clone().unwrap();
        assert_eq!(h.state().motor_manager.get_motor_config(&second).unwrap().function, MotorFunction::XAxis(1));
    }

    #[test]
    fn the_two_menus_in_the_add_motor_form_open_separately() {
        let mut h = harness();
        click(&mut h, "➕ Add Motor");
        // Job comes first in the form, then Type
        h.get_all_by_role(egui::accesskit::Role::ComboBox).last().unwrap().click();
        h.run();
        h.get_by_label("Linear Actuator").hover();
        h.run();
        h.get_by_label("Linear Actuator").click();
        h.run();
        assert_eq!(h.state().new_motor_type, MotorType::LinearActuator);
        assert_eq!(h.state().new_motor_function, MotorFunction::XAxis(0), "the other menu must not change");
    }

    #[test]
    fn controls_that_need_a_printer_are_not_clickable() {
        let mut h = with_cartesian();
        click(&mut h, "x_motor");
        assert!(h.get_by_label("🏠 Home").accesskit_node().is_disabled());
    }

    #[test]
    fn remove_is_unavailable_until_something_is_selected() {
        let h = with_cartesian();
        assert!(h.get_by_label("🗑 Remove Selected").accesskit_node().is_disabled());
    }
}
