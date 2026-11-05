//! The app's look and its navigation: colours, fonts, the sidebar and small shared widgets.
//! Page contents stay in main.rs.

use super::Tab;
use eframe::egui::{
    self, Align2, Color32, CornerRadius, FontData, FontDefinitions, FontFamily, FontId, Frame, Key,
    Modifiers, RichText, Sense, Stroke, StrokeKind, TextStyle, Theme, Visuals,
};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

// ---------------------------------------------------------------------------------------------
// Colours
// ---------------------------------------------------------------------------------------------

pub struct Palette {
    pub background: Color32,
    pub surface: Color32,
    pub raised: Color32,
    pub hover: Color32,
    pub border: Color32,
    pub text: Color32,
    pub muted: Color32,
    pub accent: Color32,
    pub accent_soft: Color32,
    pub good: Color32,
    pub warning: Color32,
    pub danger: Color32,
    pub info: Color32,
}

const DARK: Palette = Palette {
    background: Color32::from_rgb(16, 19, 24),
    surface: Color32::from_rgb(23, 27, 34),
    raised: Color32::from_rgb(34, 40, 51),
    hover: Color32::from_rgb(44, 52, 65),
    border: Color32::from_rgb(42, 49, 61),
    text: Color32::from_rgb(230, 233, 239),
    muted: Color32::from_rgb(152, 162, 179),
    accent: Color32::from_rgb(76, 141, 255),
    accent_soft: Color32::from_rgb(30, 47, 77),
    good: Color32::from_rgb(74, 222, 128),
    warning: Color32::from_rgb(251, 191, 36),
    danger: Color32::from_rgb(248, 113, 113),
    info: Color32::from_rgb(96, 165, 250),
};

const LIGHT: Palette = Palette {
    background: Color32::from_rgb(244, 246, 249),
    surface: Color32::from_rgb(255, 255, 255),
    raised: Color32::from_rgb(238, 241, 245),
    hover: Color32::from_rgb(226, 231, 238),
    border: Color32::from_rgb(216, 222, 231),
    text: Color32::from_rgb(27, 34, 48),
    muted: Color32::from_rgb(91, 101, 118),
    accent: Color32::from_rgb(31, 102, 224),
    accent_soft: Color32::from_rgb(220, 232, 252),
    good: Color32::from_rgb(21, 128, 61),
    warning: Color32::from_rgb(180, 83, 9),
    danger: Color32::from_rgb(198, 40, 40),
    info: Color32::from_rgb(29, 95, 209),
};

/// Emergency stop stays this colour in both themes.
pub const ESTOP: Color32 = Color32::from_rgb(217, 45, 32);
pub const ESTOP_LATCHED: Color32 = Color32::from_rgb(127, 29, 29);

static DARK_ACTIVE: AtomicBool = AtomicBool::new(true);

pub fn palette() -> &'static Palette {
    if DARK_ACTIVE.load(Ordering::Relaxed) { &DARK } else { &LIGHT }
}

// Status colours that stay readable on either theme.
pub fn good() -> Color32 { palette().good }
pub fn warning() -> Color32 { palette().warning }
pub fn danger() -> Color32 { palette().danger }
pub fn info() -> Color32 { palette().info }
pub fn muted() -> Color32 { palette().muted }

fn build_style(p: &Palette, dark: bool) -> egui::Style {
    let mut style = egui::Style::default();
    let family = FontFamily::Proportional;
    style.text_styles.insert(TextStyle::Body, FontId::new(14.0, family.clone()));
    style.text_styles.insert(TextStyle::Button, FontId::new(14.0, family.clone()));
    style.text_styles.insert(TextStyle::Small, FontId::new(12.0, family));
    style.text_styles.insert(TextStyle::Monospace, FontId::new(13.0, FontFamily::Monospace));
    style.text_styles.insert(TextStyle::Heading, FontId::new(22.0, FontFamily::Name(SEMIBOLD.into())));

    style.spacing.item_spacing = egui::vec2(8.0, 8.0);
    style.spacing.button_padding = egui::vec2(10.0, 5.0);
    style.spacing.interact_size.y = 26.0;
    style.spacing.slider_width = 200.0;
    style.spacing.window_margin = egui::Margin::same(14);
    style.spacing.menu_margin = egui::Margin::same(8);

    let mut v = if dark { Visuals::dark() } else { Visuals::light() };
    v.override_text_color = None;
    v.panel_fill = p.background;
    v.window_fill = p.surface;
    v.window_stroke = Stroke::new(1.0, p.border);
    v.window_corner_radius = CornerRadius::same(12);
    v.menu_corner_radius = CornerRadius::same(8);
    v.extreme_bg_color = p.raised;
    v.text_edit_bg_color = Some(p.raised);
    v.faint_bg_color = p.raised;
    v.code_bg_color = p.raised;
    v.hyperlink_color = p.accent;
    v.warn_fg_color = p.warning;
    v.error_fg_color = p.danger;
    v.slider_trailing_fill = true;
    v.selection.bg_fill = p.accent;
    v.selection.stroke = Stroke::new(1.0, Color32::WHITE);

    let radius = CornerRadius::same(7);
    let w = &mut v.widgets;
    w.noninteractive.bg_fill = p.surface;
    w.noninteractive.weak_bg_fill = p.surface;
    w.noninteractive.bg_stroke = Stroke::new(1.0, p.border);
    w.noninteractive.fg_stroke = Stroke::new(1.0, p.text);
    w.noninteractive.corner_radius = CornerRadius::same(10);
    for (state, fill) in [
        (&mut w.inactive, p.raised),
        (&mut w.hovered, p.hover),
        (&mut w.active, p.hover),
        (&mut w.open, p.hover),
    ] {
        state.bg_fill = fill;
        state.weak_bg_fill = fill;
        state.bg_stroke = Stroke::NONE;
        state.fg_stroke = Stroke::new(1.0, p.text);
        state.corner_radius = radius;
        state.expansion = 0.0;
    }
    w.hovered.bg_stroke = Stroke::new(1.0, p.border);
    w.active.bg_stroke = Stroke::new(1.0, p.accent);

    style.visuals = v;
    style
}

/// Follow the app's own theme setting. Call once per frame.
pub fn apply_theme(ctx: &egui::Context, dark: bool) {
    DARK_ACTIVE.store(dark, Ordering::Relaxed);
    ctx.set_theme(if dark { Theme::Dark } else { Theme::Light });
}

// ---------------------------------------------------------------------------------------------
// Fonts
// ---------------------------------------------------------------------------------------------

const SEMIBOLD: &str = "semibold";

fn add_system_font(fonts: &mut FontDefinitions, name: &str, path: &str) -> bool {
    match std::fs::read(path) {
        Ok(bytes) => {
            fonts.font_data.insert(name.to_owned(), Arc::new(FontData::from_owned(bytes)));
            true
        }
        Err(_) => false,
    }
}

fn build_fonts() -> FontDefinitions {
    let mut fonts = FontDefinitions::default();
    fonts.font_data.insert(
        "arial".to_owned(),
        Arc::new(FontData::from_static(include_bytes!("resources/arial.ttf"))),
    );

    // Use the system UI font where it exists. The fonts built into the app are the fallback,
    // so other systems still work.
    let has_ui_font = add_system_font(&mut fonts, "ui", r"C:\Windows\Fonts\segoeui.ttf");
    let has_semibold = add_system_font(&mut fonts, SEMIBOLD, r"C:\Windows\Fonts\seguisb.ttf");
    let has_symbols = add_system_font(&mut fonts, "symbols", r"C:\Windows\Fonts\seguisym.ttf");
    let has_emoji = add_system_font(&mut fonts, "emoji", r"C:\Windows\Fonts\seguiemj.ttf");

    let proportional = fonts.families.entry(FontFamily::Proportional).or_default();
    if has_ui_font {
        proportional.insert(0, "ui".to_owned());
    }
    proportional.push("arial".to_owned());
    if has_symbols {
        proportional.push("symbols".to_owned());
    }
    if has_emoji {
        proportional.push("emoji".to_owned());
    }
    let mut heading = proportional.clone();
    if has_semibold {
        heading.insert(0, SEMIBOLD.to_owned());
    }

    let monospace = fonts.families.entry(FontFamily::Monospace).or_default();
    monospace.push("arial".to_owned());
    if has_symbols {
        monospace.push("symbols".to_owned());
    }
    if has_emoji {
        monospace.push("emoji".to_owned());
    }

    fonts.families.insert(FontFamily::Name(SEMIBOLD.into()), heading);
    fonts
}

/// Set up fonts and both themes. Call once at startup.
pub fn install(ctx: &egui::Context) {
    ctx.set_fonts(build_fonts());
    ctx.set_style_of(Theme::Dark, build_style(&DARK, true));
    ctx.set_style_of(Theme::Light, build_style(&LIGHT, false));
}

// ---------------------------------------------------------------------------------------------
// Navigation
// ---------------------------------------------------------------------------------------------

pub struct NavItem {
    pub tab: Tab,
    pub icon: &'static str,
    pub label: &'static str,
    /// Other words someone might search for to find this page.
    pub keywords: &'static str,
}

pub struct NavGroup {
    pub title: &'static str,
    pub open_by_default: bool,
    pub items: &'static [NavItem],
}

const fn item(tab: Tab, icon: &'static str, label: &'static str, keywords: &'static str) -> NavItem {
    NavItem { tab, icon, label, keywords }
}

pub const NAV: &[NavGroup] = &[
    NavGroup {
        title: "PRINT",
        open_by_default: true,
        items: &[
            item(Tab::Overview, "🏠", "Overview", "home dashboard status start"),
            item(Tab::Gcode, "📄", "G-code", "print job run start pause toolpath preview file"),
            item(Tab::GcodeImport, "📁", "Import", "open load file gcode"),
            item(Tab::Macros, "📜", "Macros", "scripts commands automation"),
            item(Tab::Terminal, "💻", "Terminal", "console commands send gcode"),
            item(Tab::Webcam, "📷", "Webcam", "camera video monitor timelapse"),
        ],
    },
    NavGroup {
        title: "CONTROL",
        open_by_default: true,
        items: &[
            item(Tab::Kinematics, "🎯", "Kinematics", "position move jog axis xyz 5-axis"),
            item(Tab::MotionControl, "🔄", "Motion Control", "5-axis rotary machine type trunnion"),
            item(Tab::Temperature, "🌡", "Temperature", "heater hotend bed pid thermal"),
            item(Tab::FanControl, "🌀", "Fans", "cooling part fan speed"),
        ],
    },
    NavGroup {
        title: "CALIBRATE",
        open_by_default: true,
        items: &[
            item(Tab::BedMesh, "📐", "Bed Mesh", "leveling probe z offset adaptive"),
            item(Tab::InputShaping, "📊", "Input Shaping", "vibration resonance ringing accelerometer"),
            item(Tab::PressureAdvance, "💨", "Pressure Advance", "extruder linear advance corners"),
        ],
    },
    NavGroup {
        title: "MACHINE SETUP",
        open_by_default: true,
        items: &[
            item(Tab::PrinterConfig, "🖨", "Printer", "build volume steps limits configuration"),
            item(Tab::MotorAssignment, "🔩", "Motors", "drivers stepper servo assignment axis"),
            item(Tab::ClosedLoop, "🔁", "Closed Loop", "encoder feedback servo tuning"),
            item(Tab::PowerLoss, "⚡", "Power Loss", "recovery resume outage"),
        ],
    },
    NavGroup {
        title: "CONNECTIONS",
        open_by_default: true,
        items: &[
            item(Tab::FieldbusConfig, "🌐", "Fieldbus", "ethercat modbus canopen profibus serial protocol network"),
            item(Tab::CanToolheads, "🔌", "CAN Toolheads", "can bus toolhead klipper"),
        ],
    },
    NavGroup {
        title: "APP",
        open_by_default: true,
        items: &[item(Tab::Settings, "🔧", "Settings", "theme appearance dark light mode")],
    },
    NavGroup {
        title: "NOT BUILT YET",
        open_by_default: false,
        items: &[
            item(Tab::AutoInputShaper, "🎯", "Auto Calibration", "input shaper resonance automatic"),
            item(Tab::VolumetricCompensation, "📐", "Volumetric Comp.", "flow compensation"),
            item(Tab::SensorFusion, "🔗", "Sensor Fusion", "imu encoder sensors"),
            item(Tab::StateSpaceControl, "📈", "State-Space Control", "advanced control"),
            item(Tab::ModelPredictiveControl, "🔮", "Predictive Control", "mpc advanced control"),
            item(Tab::RuntimeConfig, "⚙", "Runtime Config", "klipper settings live parameters"),
            item(Tab::Plugins, "🧩", "Plugins", "extensions"),
        ],
    },
];

/// True if every word typed appears in the page's name, its keywords or its group's title.
pub fn matches_filter(group: &NavGroup, item: &NavItem, filter: &str) -> bool {
    let haystack = format!("{} {} {}", item.label, item.keywords, group.title).to_lowercase();
    filter.to_lowercase().split_whitespace().all(|word| haystack.contains(word))
}

fn normalized(name: &str) -> String {
    name.chars().filter(|c| c.is_alphanumeric()).collect::<String>().to_lowercase()
}

/// The page with this name, for the --page option. Case, spaces and dashes don't matter.
pub fn tab_from_name(name: &str) -> Option<Tab> {
    let wanted = normalized(name);
    NAV.iter()
        .flat_map(|group| group.items)
        .find(|item| normalized(item.label) == wanted)
        .map(|item| item.tab)
}

fn nav_row(ui: &mut egui::Ui, item: &NavItem, selected: bool) -> egui::Response {
    let p = palette();
    let size = egui::vec2(ui.available_width(), 30.0);
    let (rect, response) = ui.allocate_exact_size(size, Sense::click());
    response.widget_info(|| {
        egui::WidgetInfo::selected(egui::WidgetType::SelectableLabel, true, selected, item.label)
    });

    let painter = ui.painter();
    let radius = CornerRadius::same(7);
    if selected {
        painter.rect_filled(rect, radius, p.accent_soft);
        let bar = egui::Rect::from_min_size(rect.min + egui::vec2(0.0, 7.0), egui::vec2(3.0, rect.height() - 14.0));
        painter.rect_filled(bar, CornerRadius::same(2), p.accent);
    } else if response.hovered() {
        painter.rect_filled(rect, radius, p.raised);
    }
    if response.has_focus() {
        painter.rect_stroke(rect, radius, Stroke::new(1.0, p.accent), StrokeKind::Inside);
    }

    let text_color = if selected || response.hovered() { p.text } else { p.muted };
    let center_y = rect.center().y;
    painter.text(
        egui::pos2(rect.left() + 22.0, center_y),
        Align2::CENTER_CENTER,
        item.icon,
        FontId::proportional(14.0),
        text_color,
    );
    painter.text(
        egui::pos2(rect.left() + 40.0, center_y),
        Align2::LEFT_CENTER,
        item.label,
        FontId::proportional(14.0),
        text_color,
    );
    response.on_hover_cursor(egui::CursorIcon::PointingHand)
}

pub fn sidebar(ctx: &egui::Context, selected: &mut Tab, filter: &mut String) {
    let p = palette();
    egui::SidePanel::left("navigation")
        .exact_width(232.0)
        .resizable(false)
        .frame(Frame::new().fill(p.surface).stroke(Stroke::new(1.0, p.border)).inner_margin(12))
        .show(ctx, |ui| {
            ui.add_space(2.0);
            ui.label(RichText::new("3D Printer Host").font(FontId::new(17.0, FontFamily::Name(SEMIBOLD.into()))));
            ui.add_space(4.0);

            let search = ui.add(
                egui::TextEdit::singleline(filter)
                    .hint_text("🔍  Find a page  (Ctrl+K)")
                    .desired_width(f32::INFINITY)
                    .margin(egui::vec2(8.0, 6.0)),
            );
            if ctx.input_mut(|i| i.consume_key(Modifiers::COMMAND, Key::K)) {
                search.request_focus();
            }
            if search.lost_focus() && ctx.input(|i| i.key_pressed(Key::Escape)) {
                filter.clear();
            }
            let open_first = search.lost_focus() && ctx.input(|i| i.key_pressed(Key::Enter));
            ui.add_space(4.0);

            let filtering = !filter.trim().is_empty();
            let mut any_match = false;
            egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                ui.spacing_mut().item_spacing.y = 2.0;
                for group in NAV {
                    let visible: Vec<&NavItem> = group
                        .items
                        .iter()
                        .filter(|item| matches_filter(group, item, filter))
                        .collect();
                    if visible.is_empty() {
                        continue;
                    }
                    if filtering && open_first && !any_match {
                        *selected = visible[0].tab;
                    }
                    any_match = true;
                    egui::CollapsingHeader::new(RichText::new(group.title).size(11.0).color(p.muted).strong())
                        .id_salt(group.title)
                        .default_open(group.open_by_default)
                        .open(filtering.then_some(true))
                        .show_unindented(ui, |ui| {
                            for item in visible {
                                if nav_row(ui, item, *selected == item.tab).clicked() {
                                    *selected = item.tab;
                                }
                            }
                        });
                    ui.add_space(4.0);
                }
                if !any_match {
                    ui.add_space(8.0);
                    ui.label(RichText::new("No pages match").color(p.muted));
                }
            });
        });
}

// ---------------------------------------------------------------------------------------------
// Small shared widgets
// ---------------------------------------------------------------------------------------------

/// A small rounded status label.
pub fn pill(ui: &mut egui::Ui, text: &str, color: Color32) {
    Frame::new()
        .fill(palette().raised)
        .corner_radius(CornerRadius::same(12))
        .inner_margin(egui::Margin::symmetric(10, 4))
        .show(ui, |ui| {
            ui.spacing_mut().item_spacing.x = 6.0;
            ui.horizontal(|ui| {
                let (dot, _) = ui.allocate_exact_size(egui::vec2(8.0, 8.0), Sense::hover());
                ui.painter().circle_filled(dot.center(), 4.0, color);
                ui.label(RichText::new(text).size(13.0));
            });
        });
}

/// A control that can't work yet: greyed out, and it says why when you hover over it.
pub fn unavailable_button(ui: &mut egui::Ui, label: &str, why: &str) {
    ui.add_enabled(false, egui::Button::new(label)).on_disabled_hover_text(why);
}

/// A control that needs a machine to act on.
pub fn needs_printer_button(ui: &mut egui::Ui, label: &str) {
    unavailable_button(ui, label, "Needs a connected printer. Connecting to one isn't supported yet.");
}

/// A bordered section of a page.
pub fn card<R>(ui: &mut egui::Ui, add_contents: impl FnOnce(&mut egui::Ui) -> R) -> R {
    let p = palette();
    Frame::new()
        .fill(p.surface)
        .stroke(Stroke::new(1.0, p.border))
        .corner_radius(CornerRadius::same(10))
        .inner_margin(16)
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            add_contents(ui)
        })
        .inner
}

/// A small heading inside a card or panel.
pub fn section_title(ui: &mut egui::Ui, text: &str) {
    ui.label(RichText::new(text).size(12.0).strong().color(palette().muted));
}

/// A clickable card that takes you to a page.
pub fn tile(ui: &mut egui::Ui, icon: &str, title: &str, subtitle: &str) -> egui::Response {
    let p = palette();
    let size = egui::vec2(ui.available_width(), 72.0);
    let (rect, response) = ui.allocate_exact_size(size, Sense::click());
    response.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Button, true, title));
    let painter = ui.painter();
    let radius = CornerRadius::same(10);
    painter.rect_filled(rect, radius, if response.hovered() { p.raised } else { p.surface });
    let border = if response.hovered() || response.has_focus() { p.accent } else { p.border };
    painter.rect_stroke(rect, radius, Stroke::new(1.0, border), StrokeKind::Inside);
    painter.text(
        egui::pos2(rect.left() + 28.0, rect.center().y),
        Align2::CENTER_CENTER,
        icon,
        FontId::proportional(20.0),
        p.accent,
    );
    painter.text(
        egui::pos2(rect.left() + 52.0, rect.center().y - 10.0),
        Align2::LEFT_CENTER,
        title,
        FontId::new(14.0, FontFamily::Name(SEMIBOLD.into())),
        p.text,
    );
    painter.text(
        egui::pos2(rect.left() + 52.0, rect.center().y + 10.0),
        Align2::LEFT_CENTER,
        subtitle,
        FontId::proportional(12.0),
        p.muted,
    );
    response.on_hover_cursor(egui::CursorIcon::PointingHand)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn all_items() -> impl Iterator<Item = (&'static NavGroup, &'static NavItem)> {
        NAV.iter().flat_map(|group| group.items.iter().map(move |item| (group, item)))
    }

    fn results(filter: &str) -> Vec<&'static str> {
        all_items().filter(|(g, i)| matches_filter(g, i, filter)).map(|(_, i)| i.label).collect()
    }

    #[test]
    fn every_page_is_listed_exactly_once() {
        let items: Vec<_> = all_items().collect();
        for (i, (_, a)) in items.iter().enumerate() {
            assert!(!items[i + 1..].iter().any(|(_, b)| b.tab == a.tab), "{} is listed twice", a.label);
        }
        assert_eq!(items.len(), 27);
    }

    #[test]
    fn search_finds_pages_by_the_words_people_use() {
        assert_eq!(results("heater"), ["Temperature"]);
        assert_eq!(results("EtherCAT"), ["Fieldbus"]);
        assert_eq!(results("  fans "), ["Fans"]);
        assert!(results("no such page").is_empty());
        assert_eq!(results("").len(), 27);
        assert_eq!(results("calibrate pressure"), ["Pressure Advance"]);
    }

    #[test]
    fn page_names_resolve_for_the_command_line() {
        assert!(tab_from_name("bed mesh") == Some(Tab::BedMesh));
        assert!(tab_from_name("Bed-Mesh") == Some(Tab::BedMesh));
        assert!(tab_from_name("g-code") == Some(Tab::Gcode));
        assert!(tab_from_name("nonsense").is_none());
    }

    /// A button whose handler is empty, a comment or only a print looks like it works and doesn't.
    /// If it can't work yet, use unavailable_button or needs_printer_button, which say so.
    fn dead_buttons(source: &str) -> Vec<String> {
        // Code inside /* */ isn't live
        let mut in_comment = false;
        let lines: Vec<&str> = source
            .lines()
            .map(|line| {
                let trimmed = line.trim_start();
                if trimmed.starts_with("/*") {
                    in_comment = true;
                }
                let kept = if in_comment { "" } else { line };
                if line.contains("*/") {
                    in_comment = false;
                }
                kept
            })
            .collect();
        let indent = |line: &str| line.len() - line.trim_start().len();
        let mut dead = Vec::new();
        for (i, line) in lines.iter().enumerate() {
            if !line.trim_start().starts_with("if ") || !line.trim_end().ends_with(".clicked() {") {
                continue;
            }
            let does_something = lines[i + 1..]
                .iter()
                .take_while(|next| next.trim().is_empty() || indent(next) > indent(line))
                .map(|next| next.trim())
                .any(|next| !next.is_empty() && next != "}" && !next.starts_with("//") && !next.starts_with("println!") && !next.starts_with("eprintln!"));
            if !does_something {
                dead.push(format!("line {}: {}", i + 1, line.trim()));
            }
        }
        dead
    }

    #[test]
    fn every_button_does_something_or_says_why_it_cannot() {
        for source in [include_str!("main.rs"), include_str!("ui_motors.rs")] {
            let dead = dead_buttons(source);
            assert!(dead.is_empty(), "buttons that do nothing:\n{}", dead.join("\n"));
        }
    }

    #[test]
    fn the_dead_button_check_can_tell_the_difference() {
        let dead = "        if ui.button(\"A\").clicked() {\n            // later\n        }\n\
                    if ui.button(\"B\").clicked() {\n            println!(\"b\");\n        }\n";
        assert_eq!(dead_buttons(dead).len(), 2);
        let live = "        if ui.button(\"C\").clicked() {\n            self.x = 1;\n        }\n";
        assert!(dead_buttons(live).is_empty());
    }

    /// A symbol missing from the fonts shows up as an empty box.
    #[test]
    fn every_symbol_in_the_interface_can_be_drawn() {
        let ctx = egui::Context::default();
        ctx.set_fonts(build_fonts());
        let mut missing = std::collections::BTreeSet::new();
        let _ = ctx.run(Default::default(), |ctx| {
            ctx.fonts(|fonts| {
                let font = FontId::proportional(14.0);
                let nav: String = all_items().map(|(_, i)| i.icon).collect();
                for source in [nav.as_str(), include_str!("main.rs")] {
                    for c in source.chars().filter(|c| !c.is_ascii()) {
                        if !fonts.has_glyph(&font, c) {
                            missing.insert(c);
                        }
                    }
                }
            });
        });
        assert!(missing.is_empty(), "no glyph for: {:?}", missing);
    }
}
