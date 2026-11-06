//! The real-hardware card on the Fieldbus page: choose an adapter, see what's on it, and watch its
//! inputs live. Nothing goes on the wire until a button is pressed, and no adapter is pre-selected,
//! because EtherCAT frames don't belong on an office network.

use super::ui_shell;
use eframe::egui;
use printer_host::ethercat_check::{Activity, CheckState, Found, HardwareCheck};
use printer_host::ethercat_coupler::InputsView;
use printer_host::ethercat_probe::{self, Adapter};

/// The adapters worth offering. No numbers: they can change from one run to the next.
fn usable_adapters() -> Result<Vec<Adapter>, String> {
    let mut adapters = ethercat_probe::list_adapters()?;
    adapters.retain(Adapter::could_be_ethercat);
    Ok(adapters)
}

#[derive(Default)]
pub struct EthercatPage {
    pub check: HardwareCheck,
    /// Read when the card is first shown, and again on Refresh
    pub adapters: Option<Result<Vec<Adapter>, String>>,
    pub chosen: Option<usize>,
}

fn adapter_label(adapter: &Adapter) -> String {
    format!(
        "{}{}",
        if adapter.description.is_empty() { &adapter.name } else { &adapter.description },
        if adapter.connected { "  (link up)" } else { "" }
    )
}

fn input_light(ui: &mut egui::Ui, on: bool) {
    let (spot, _) = ui.allocate_exact_size(egui::vec2(14.0, 14.0), egui::Sense::hover());
    let visuals = ui.visuals();
    let fill = if on { ui_shell::good() } else { visuals.extreme_bg_color };
    let outline = visuals.widgets.noninteractive.bg_stroke.color;
    ui.painter().circle(spot.center(), 6.0, fill, egui::Stroke::new(1.0, outline));
}

fn show_modules(ui: &mut egui::Ui, found: &Found, live: Option<&InputsView>) {
    for (index, inputs) in found.module_inputs.iter().enumerate() {
        let values = live.and_then(|view| view.modules.get(index));
        ui.horizontal(|ui| {
            ui.add_sized([80.0, 18.0], egui::Label::new(format!("Module {}", index + 1)));
            ui.colored_label(ui_shell::muted(), format!("{} inputs", inputs));
            for channel in 0..*inputs {
                input_light(ui, values.and_then(|v| v.get(channel)).copied().unwrap_or(false));
            }
            let text: String = match values {
                Some(values) => values.iter().map(|on| if *on { '1' } else { '0' }).collect(),
                None => "-".repeat(*inputs),
            };
            ui.monospace(text);
        });
    }
}

impl EthercatPage {
    /// The selected adapter, if the list still has it.
    fn chosen_adapter(&self) -> Option<Adapter> {
        let adapters = self.adapters.as_ref()?.as_ref().ok()?;
        adapters.get(self.chosen?).cloned()
    }

    fn show_controls(&mut self, ui: &mut egui::Ui, state: &CheckState) {
        let idle = state.activity == Activity::Idle;
        ui.horizontal_wrapped(|ui| {
            ui.add_enabled_ui(idle, |ui| {
                let selected = match (self.adapters.as_ref(), self.chosen) {
                    (Some(Ok(list)), Some(index)) if index < list.len() => adapter_label(&list[index]),
                    _ => "Choose an adapter".to_string(),
                };
                egui::ComboBox::from_id_salt("ethercat_adapter").selected_text(selected).width(340.0).show_ui(ui, |ui| {
                    if let Some(Ok(list)) = &self.adapters {
                        for (index, adapter) in list.iter().enumerate() {
                            ui.selectable_value(&mut self.chosen, Some(index), adapter_label(adapter));
                        }
                    }
                });
                if ui.button("Refresh").clicked() {
                    self.adapters = Some(usable_adapters());
                    self.chosen = None;
                }
            });

            let adapter = self.chosen_adapter();
            if ui.add_enabled(idle && adapter.is_some(), egui::Button::new("Find modules")).clicked() {
                if let Some(adapter) = &adapter {
                    self.check.find(adapter);
                }
            }
            if state.activity == Activity::Watching {
                if ui.button("Stop watching").clicked() {
                    self.check.stop();
                }
            } else {
                let can_watch = idle && adapter.is_some() && state.found.as_ref().is_some_and(|found| found.can_watch);
                let watch = ui
                    .add_enabled(can_watch, egui::Button::new("Watch inputs"))
                    .on_disabled_hover_text("Find modules first. This needs a BK1120 with inputs and no outputs.");
                if watch.clicked() {
                    if let Some(adapter) = &adapter {
                        self.check.watch(adapter);
                    }
                }
            }
            if !idle {
                ui.spinner();
            }
        });
    }

    pub fn show(&mut self, ui: &mut egui::Ui) {
        if self.adapters.is_none() {
            self.adapters = Some(usable_adapters());
        }
        let state = self.check.state();

        ui_shell::card(ui, |ui| {
            ui_shell::section_title(ui, "ETHERCAT HARDWARE CHECK");
            ui.label("Finds the real devices on the adapter you choose and shows their inputs. It only reads: outputs are never driven.");
            ui.colored_label(ui_shell::muted(), "Choose the adapter wired to the coupler. EtherCAT frames don't belong on an office network.");
            ui.add_space(6.0);

            self.show_controls(ui, &state);

            if let Some(Err(why)) = &self.adapters {
                ui.colored_label(ui_shell::danger(), why);
            }
            if let Some(error) = &state.error {
                ui.colored_label(ui_shell::danger(), error);
            }

            let Some(found) = &state.found else {
                return;
            };
            ui.add_space(8.0);
            for device in &found.devices {
                ui.horizontal_wrapped(|ui| {
                    ui_shell::pill(ui, &device.name, ui_shell::good());
                    ui.colored_label(ui_shell::muted(), device.describe());
                });
            }
            if let Some(note) = &found.note {
                ui.colored_label(ui_shell::warning(), note);
            }
            if found.other_output_bits > 0 {
                ui.colored_label(
                    ui_shell::warning(),
                    format!("It also has {} output bits, so watching stays off: this check never runs a device with outputs.", found.other_output_bits),
                );
            }
            ui.add_space(4.0);
            show_modules(ui, found, state.live.as_ref());
            match &state.live {
                Some(live) => {
                    ui.add_space(4.0);
                    let status = live.status.map_or("no status word".to_string(), |word| format!("coupler status {:#06x}", word));
                    ui.colored_label(ui_shell::good(), format!("Live: {} changes, {}", state.changes, status));
                    ui.label("Put 24 V on an input and its light should come on.");
                }
                None if found.can_watch => {
                    ui.colored_label(ui_shell::muted(), "Not being read. Press Watch inputs to see them live.");
                }
                None => {}
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui_kittest::kittest::Queryable;
    use egui_kittest::Harness;
    use printer_host::ethercat_probe::{DeviceInfo, RawLink};
    use std::sync::Arc;

    fn fake_adapter() -> Adapter {
        Adapter { name: "fake".to_string(), description: "Test adapter".to_string(), connected: true }
    }

    fn page(open_error: &'static str) -> EthercatPage {
        // A lock of its own, so tests can't collide with a real run or each other
        let lock = std::env::temp_dir().join(format!("ui_ethercat_{}_{:?}.lock", std::process::id(), std::thread::current().id()));
        EthercatPage {
            check: HardwareCheck::with(Arc::new(move |_| Err::<Box<dyn RawLink>, _>(open_error.to_string())), lock),
            adapters: Some(Ok(vec![fake_adapter()])),
            chosen: None,
        }
    }

    fn found() -> Found {
        Found {
            devices: vec![DeviceInfo { position: 0, name: "BK1120".to_string(), vendor_id: 2, product_id: 0x0460_2C22, revision: 0, serial: 0 }],
            module_inputs: vec![4, 4],
            other_output_bits: 0,
            can_watch: true,
            note: None,
        }
    }

    fn harness(page: EthercatPage) -> Harness<'static, EthercatPage> {
        Harness::builder().with_size(egui::vec2(1100.0, 700.0)).build_state(
            |ctx, page: &mut EthercatPage| {
                egui::CentralPanel::default().show(ctx, |ui| page.show(ui));
            },
            page,
        )
    }

    fn step_until_idle(h: &mut Harness<'static, EthercatPage>) {
        for _ in 0..500 {
            h.step();
            if h.state().check.state().activity == Activity::Idle {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        assert_eq!(h.state().check.state().activity, Activity::Idle);
        h.run();
    }

    #[test]
    fn nothing_is_sent_until_an_adapter_is_chosen() {
        let mut h = harness(page("should not open"));
        h.run();
        h.get_by_label("Find modules").click();
        h.run();
        let state = h.state().check.state();
        assert_eq!(state.error, None, "pressing it with no adapter does nothing");
        assert_eq!(state.found, None);
    }

    #[test]
    fn find_modules_uses_the_chosen_adapter_and_shows_what_went_wrong() {
        let mut page = page("Couldn't open the adapter");
        page.chosen = Some(0);
        let mut h = harness(page);
        h.run();
        h.get_by_label("Find modules").click();
        step_until_idle(&mut h);
        h.get_by_label("Couldn't open the adapter");
    }

    #[test]
    fn found_modules_are_listed_and_marked_as_not_live() {
        let mut page = page("unused");
        page.chosen = Some(0);
        page.check.set_state(CheckState { found: Some(found()), ..CheckState::default() });
        let mut h = harness(page);
        h.run();
        h.get_by_label("BK1120");
        h.get_by_label("Module 1");
        h.get_by_label("Module 2");
        assert_eq!(h.get_all_by_label("----").count(), 2, "with no fake 0000 readings");
        h.get_by_label("Not being read. Press Watch inputs to see them live.");
    }

    #[test]
    fn a_live_reading_shows_each_module_and_offers_stop() {
        let mut page = page("unused");
        page.chosen = Some(0);
        page.check.set_state(CheckState {
            activity: Activity::Watching,
            found: Some(found()),
            live: Some(InputsView { status: Some(0x0080), modules: vec![vec![true, false, false, false], vec![false; 4]] }),
            changes: 3,
            error: None,
        });
        let mut h = harness(page);
        h.step();
        h.get_by_label("1000");
        h.get_by_label("0000");
        h.get_by_label("Live: 3 changes, coupler status 0x0080");
        h.get_by_label("Stop watching");
        assert!(h.query_by_label("Watch inputs").is_none());
    }

    #[test]
    fn a_device_with_real_outputs_is_called_out() {
        let mut page = page("unused");
        page.check.set_state(CheckState { found: Some(Found { other_output_bits: 8, can_watch: false, ..found() }), ..CheckState::default() });
        let mut h = harness(page);
        h.run();
        h.get_by_label("It also has 8 output bits, so watching stays off: this check never runs a device with outputs.");
    }

    #[test]
    fn a_missing_npcap_is_explained() {
        let mut page = page("unused");
        page.adapters = Some(Err("Npcap isn't installed.".to_string()));
        let mut h = harness(page);
        h.run();
        h.get_by_label("Npcap isn't installed.");
    }

    /// Drives the page against the real coupler. Run it by hand, with nothing else on the bus:
    /// set REAL_ETHERCAT_ADAPTER to the adapter's full description, then
    /// cargo test --bin printer-host real_coupler -- --ignored --nocapture
    #[test]
    #[ignore = "needs the real coupler"]
    fn real_coupler_through_the_page() {
        let wanted = std::env::var("REAL_ETHERCAT_ADAPTER").expect("set REAL_ETHERCAT_ADAPTER");
        let adapters = usable_adapters().unwrap();
        let chosen = adapters.iter().position(|adapter| adapter.description == wanted).expect("no adapter with that description");
        let page = EthercatPage { chosen: Some(chosen), adapters: Some(Ok(adapters)), ..EthercatPage::default() };
        let mut h = harness(page);
        h.run();
        h.get_by_label("Find modules").click();
        step_until_idle(&mut h);
        let state = h.state().check.state();
        println!("found: {:#?}\nerror: {:?}", state.found, state.error);
        assert!(state.found.as_ref().is_some_and(|found| found.can_watch), "{:?}", state.error);

        h.get_by_label("Watch inputs").click();
        for _ in 0..1500 {
            h.step();
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        let state = h.state().check.state();
        println!("live: {:?}  changes: {}", state.live, state.changes);
        assert_eq!(state.activity, Activity::Watching, "{:?}", state.error);

        h.get_by_label("Stop watching").click();
        step_until_idle(&mut h);
        let state = h.state().check.state();
        assert_eq!(state.error, None);
    }
}
