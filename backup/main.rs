use eframe::egui::{self, FontDefinitions, FontData, Color32, RichText, Visuals};
// use egui_plot::{Line, Plot, PlotPoints, Points}; // Commented out for now
use printer_kinematics::{create_kinematics, Config, Kinematics, Scene};
use std::fs;
use std::sync::Arc;
#[cfg(target_os = "linux")]
use ethercrab::{Client, ClientConfig, SlaveGroup, std::tx_rx_task, std::ethercat_now, Timeouts, PduStorage};
#[cfg(target_os = "linux")]
use tokio::sync::mpsc;
// use gcode_parser::GCodeFile; // Commented out for now
use serialport::{self, SerialPort};
// use rfd::FileDialog; // Commented out for now

#[cfg(target_os = "linux")]
const MAX_SUBDEVICES: usize = 16;
#[cfg(target_os = "linux")]
const MAX_PDU_DATA: usize = 1100;
#[cfg(target_os = "linux")]
const MAX_FRAMES: usize = 16;
#[cfg(target_os = "linux")]
const PDI_LEN: usize = 64;

#[cfg(target_os = "linux")]
static PDU_STORAGE: PduStorage<MAX_FRAMES, MAX_PDU_DATA> = PduStorage::new();

#[derive(Debug, PartialEq)]
enum CommunicationMode {
    EtherCAT,
    Serial,
}

#[tokio::main]
async fn main() -> eframe::Result<()> {
    let config_str = fs::read_to_string("printer.toml").unwrap_or_else(|e| panic!("Failed to read printer.toml: {}", e));
    let config: Config = toml::from_str(&config_str).unwrap_or_else(|e| panic!("Failed to parse printer.toml: {}", e));
    let kinematics = create_kinematics(&config);

    #[cfg(target_os = "linux")]
    let (group, tx, _pdu_task) = {
        let interface = std::env::args().nth(1).expect("Provide network interface as first argument, e.g., '\\Device\\NPF_{...}'");
        let (tx, rx, pdu_loop) = PDU_STORAGE.try_split().expect("Can only split once");
        let client = Client::new(pdu_loop, Timeouts::default(), ClientConfig::default());
        let _pdu_task = tokio::spawn(tx_rx_task(&interface, tx, rx).expect("Spawn TX/RX task"));

        println!("Initializing EtherCAT master...");
        let group = Arc::new(client.init_single_group::<MAX_SUBDEVICES, PDI_LEN>(ethercat_now).await.unwrap_or_else(|e| panic!("EtherCAT init failed: {}", e)));
        println!("Discovered {} EtherCAT slaves", group.len());

        let (tx, rx) = mpsc::channel::<[f32; 5]>(64);
        let group_clone: Arc<SlaveGroup<MAX_SUBDEVICES, PDI_LEN>> = Arc::clone(&group);
        tokio::spawn(async move {
            while let Some(positions) = rx.recv().await {
                for (i, subdevice) in group_clone.iter().enumerate() {
                    if let Some(pos) = positions.get(i) {
                        if let Err(e) = subdevice.write_slice(&pos.to_le_bytes()).await {
                            eprintln!("Failed to write to slave {}: {}", subdevice.configured_address(), e);
                        }
                    }
                }
            }
        });

        (group, tx, _pdu_task)
    };

    #[cfg(not(target_os = "linux"))]
    let (group, tx) = {
        println!("EtherCAT not supported on this platform");
        (Arc::new(DummySlaveGroup), tokio::sync::mpsc::channel::<[f32; 5]>(64).0)
    };

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size(egui::vec2(1280.0, 720.0)),
        renderer: eframe::Renderer::Wgpu, // Using wgpu renderer
        ..Default::default()
    };

    let gcode = fs::read_to_string("gcode/example.gcode").unwrap_or_else(|_| "No G-code file found".to_string());
    // let parsed_gcode = GCodeFile::parse(&gcode).unwrap_or_else(|| GCodeFile { commands: vec![] }); // Commented out
    // let plot_points = kinematics.generate_path(&parsed_gcode); // Commented out

    eframe::run_native(
        "3D Printer Host",
        native_options,
        Box::new(move |cc| {
            // Load fallback font (Arial) with error handling
            let mut fonts = FontDefinitions::default();
            match FontData::from_static(include_bytes!("resources/arial.ttf")) {
                font_data => {
                    fonts.font_data.insert("arial".to_owned(), font_data.into()); // Fixed with .into()
                    fonts.families.get_mut(&egui::FontFamily::Proportional).unwrap().insert(0, "arial".to_owned());
                    fonts.families.get_mut(&egui::FontFamily::Monospace).unwrap().insert(0, "arial".to_owned());
                    cc.egui_ctx.set_fonts(fonts);
                    println!("Loaded Arial font successfully");
                }
                _ => {
                    println!("Arial font file not found, using default fonts");
                    // Fallback to default fonts if file is missing
                }
            }

            // Set dark theme with high-contrast text
            cc.egui_ctx.set_visuals(Visuals {
                dark_mode: true,
                override_text_color: Some(Color32::WHITE),
                panel_fill: Color32::from_rgb(30, 30, 30), // Dark gray background
                ..Visuals::default()
            });
            cc.egui_ctx.set_debug_on_hover(true); // Enable debug rendering
            println!("UI initialized with wgpu renderer");
            Ok(Box::new(MyApp::new(kinematics, group, tx, gcode, vec![]))) // Empty plot_points for now
        }),
    )
}

// Dummy struct for non-Linux platforms
#[cfg(not(target_os = "linux"))]
struct DummySlaveGroup;

#[cfg(not(target_os = "linux"))]
impl DummySlaveGroup {
    fn is_empty(&self) -> bool {
        true
    }

    fn len(&self) -> usize {
        0
    }
}

#[derive(Debug, PartialEq)]
enum Tab {
    Kinematics,
    Gcode,
    Settings,
    GcodeImport,
}

struct MyApp {
    kinematics: Box<dyn Kinematics>,
    scene: Scene,
    x: f32,
    y: f32,
    z: f32,
    a: f32,
    b: f32,
    gcode: String,
    // parsed_gcode: GCodeFile, // Commented out
    plot_points: Vec<[f64; 2]>,
    selected_tab: Tab,
    show_status_panel: bool,
    #[cfg(target_os = "linux")]
    group: Arc<SlaveGroup<MAX_SUBDEVICES, PDI_LEN>>,
    #[cfg(not(target_os = "linux"))]
    group: Arc<DummySlaveGroup>,
    ethercat_tx: tokio::sync::mpsc::Sender<[f32; 5]>,
    ethercat_initialized: bool,
    communication_mode: CommunicationMode,
    serial_port: Option<Box<dyn SerialPort>>,
    serial_port_name: String,
    serial_baud_rate: u32,
    last_file_error: Option<String>,
}

impl MyApp {
    fn new(
        kinematics: Box<dyn Kinematics>,
        #[cfg(target_os = "linux")]
        group: Arc<SlaveGroup<MAX_SUBDEVICES, PDI_LEN>>,
        #[cfg(not(target_os = "linux"))]
        group: Arc<DummySlaveGroup>,
        ethercat_tx: tokio::sync::mpsc::Sender<[f32; 5]>,
        gcode: String,
        plot_points: Vec<[f64; 2]>,
    ) -> Self {
        Self {
            kinematics,
            scene: Scene::new(),
            x: 0.0,
            y: 0.0,
            z: 0.0,
            a: 0.0,
            b: 0.0,
            gcode,
            // parsed_gcode: GCodeFile { commands: vec![] }, // Commented out
            plot_points,
            selected_tab: Tab::Kinematics,
            show_status_panel: true,
            group,
            ethercat_tx,
            ethercat_initialized: false,
            communication_mode: CommunicationMode::EtherCAT,
            serial_port: None,
            serial_port_name: "COM3".to_string(),
            serial_baud_rate: 115200,
            last_file_error: None,
        }
    }

    // fn load_gcode(&mut self) { ... } // Commented out for now
    // fn open_serial_port(&mut self) { ... } // Commented out for now
    // fn send_serial_positions(&mut self, positions: [f32; 5]) { ... } // Commented out for now
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.set_visuals(Visuals {
            dark_mode: true,
            override_text_color: Some(Color32::WHITE),
            panel_fill: Color32::from_rgb(30, 30, 30), // Dark gray background
            ..Visuals::default()
        });
        ctx.set_pixels_per_point(2.0); // Increase font size

        egui::SidePanel::right("status_panel").show(ctx, |ui| {
            ui.heading("Printer Status");
            ui.separator();
            ui.label(format!("Status: {}", if self.group.is_empty() { "Disconnected" } else { "Ready" }));
            ui.label(format!("Position: X:{:.1}, Y:{:.1}, Z:{:.1}", self.x, self.y, self.z));
            ui.label(format!("Communication: {:?}", self.communication_mode));
        });

        let positions = self.kinematics.to_motor_positions(self.x, self.y, self.z, self.a, self.b);
        match self.communication_mode {
            CommunicationMode::EtherCAT => {
                let tx = self.ethercat_tx.clone();
                tokio::spawn(async move {
                    if let Err(e) = tx.send(positions).await {
                        eprintln!("Failed to send positions to EtherCAT task: {}", e);
                    }
                });
            }
            CommunicationMode::Serial => {
                // self.send_serial_positions(positions); // Commented out for now
            }
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            // Simplified tab bar to test text rendering
            ui.horizontal(|ui| {
                ui.style_mut().visuals.selection.bg_fill = Color32::DARK_BLUE;
                ui.style_mut().visuals.selection.stroke.color = Color32::WHITE;
                ui.style_mut().spacing.button_padding = egui::vec2(10.0, 5.0);

                println!("Constructing tab bar");
                ui.label(RichText::new("Test Tab Bar").color(Color32::YELLOW)); // Test text rendering
            });
            ui.separator();

            // Default to Kinematics tab content for now
            ui.heading("Printer Kinematics Control");
            ui.add(egui::Slider::new(&mut self.x, -100.0..=100.0).text("X"));
            ui.add(egui::Slider::new(&mut self.y, -100.0..=100.0).text("Y"));
            ui.add(egui::Slider::new(&mut self.z, -100.0..=100.0).text("Z"));
            ui.add(egui::Slider::new(&mut self.a, -180.0..=180.0).text("A (degrees)"));
            ui.add(egui::Slider::new(&mut self.b, -180.0..=180.0).text("B (degrees)"));

            let positions = self.kinematics.to_motor_positions(self.x, self.y, self.z, self.a, self.b);
            ui.label(format!("Motor Positions: {:?}", positions));

            self.kinematics.visualize(&mut self.scene, self.a, self.b);
            ui.label(format!("Scene Entities: {}", self.scene.entities.len()));
        });
    }
}