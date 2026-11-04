use serde::Deserialize;
use nalgebra::{Rotation3, Vector3};

// Export new 5-axis modules
pub mod five_axis;
pub mod motion_planning;
pub mod tool_orientation;
pub mod practical_configs;
pub mod slicer_integration;
pub mod embedded_optimized;

// Re-export five_axis items
pub use five_axis::{FiveAxisConfiguration, Pose6DOF, JointConfiguration, FiveAxisKinematics, KinematicsError as FiveAxisKinematicsError};

// Re-export motion_planning items
pub use motion_planning::*;

// Re-export tool_orientation items  
pub use tool_orientation::*;

// Re-export practical_configs items
pub use practical_configs::*;

// Re-export slicer_integration items
pub use slicer_integration::*;

// Re-export embedded_optimized items (but use alias for KinematicsError)
pub use embedded_optimized::{EmbeddedKinematics, KinematicsError as EmbeddedKinematicsError};

// Simple Scene struct for visualization data
#[derive(Default)]
pub struct Scene {
    pub entities: Vec<Entity>,
}

#[derive(Clone)]
pub struct Entity {
    pub position: (f32, f32, f32),
    pub rotation: (f32, f32, f32), // Euler angles (roll, pitch, yaw) in degrees
    pub mesh: Mesh,
}

#[derive(Clone)]
pub enum Mesh {
    Cube(f32, f32, f32), // width, depth, height
    Sphere(f32),         // radius
    Cylinder(f32, f32),  // radius, height
}

impl Scene {
    pub fn new() -> Self { Self { entities: Vec::new() } }
    pub fn add_entity(&mut self, entity: Entity) { self.entities.push(entity); }
}

fn safe_sqrt(value: f32) -> f32 {
    if value >= 0.0 { value.sqrt() } else { 0.0 }
}

// Config structs
#[derive(Clone, Deserialize)]
pub struct DeltaConfig {
    pub arm_length: f32,
    pub rod_length: f32,
    pub bed_radius: f32,
}

#[derive(Clone, Deserialize)]
pub struct DualDeltaConfig {
    pub primary: DeltaConfig,
    pub secondary: DeltaConfig,
}

#[derive(Clone, Deserialize)]
pub struct CoreXYConfig {
    pub belt_angle: f32,
}

#[derive(Clone, Deserialize)]
pub struct HBotConfig {
    pub belt_angle: f32,
}

#[derive(Clone, Deserialize)]
pub struct InverseDeltaConfig {
    pub arm_length: f32,
    pub rod_length: f32,
    pub bed_radius: f32,
}

#[derive(Clone, Deserialize)]
pub struct PolarConfig {
    pub max_radius: f32,
}

#[derive(Clone, Deserialize)]
pub struct FiveAxisTrunnionConfig {
    pub table_length: f32,
}

#[derive(Clone, Deserialize)]
pub struct FiveAxisBedConfig {
    pub bed_length: f32,
}

#[derive(Clone, Deserialize)]
pub struct SphericalParallelConfig {
    pub platform_radius: f32,
    pub base_radius: f32,
}

#[derive(Deserialize)]
pub struct PrinterConfig {
    pub kinematics: String,
    pub max_velocity: f32,
    pub custom_kinematics_path: Option<String>,
    pub delta: Option<DeltaConfig>,
    pub dual_delta: Option<DualDeltaConfig>,
    pub corexy: Option<CoreXYConfig>,
    pub hbot: Option<HBotConfig>,
    pub inverse_delta: Option<InverseDeltaConfig>,
    pub polar: Option<PolarConfig>,
    pub five_axis_trunnion: Option<FiveAxisTrunnionConfig>,
    pub five_axis_bed: Option<FiveAxisBedConfig>,
    pub spherical_parallel: Option<SphericalParallelConfig>,
}

#[derive(Deserialize)]
pub struct CommConfig {
    pub protocol: String,
    pub device: String,
}

#[derive(Deserialize)]
pub struct Config {
    pub printer: PrinterConfig,
    pub communication: CommConfig,
}

pub trait Kinematics {
    fn to_motor_positions(&self, x: f32, y: f32, z: f32, a: f32, b: f32) -> [f32; 5];
    fn visualize(&self, scene: &mut Scene, a: f32, b: f32);
    fn generate_path(&self, gcode: &gcode_parser::GCodeFile) -> Vec<[f64; 2]>;
}

pub struct CartesianKinematics;
impl Kinematics for CartesianKinematics {
    fn to_motor_positions(&self, x: f32, y: f32, z: f32, a: f32, b: f32) -> [f32; 5] {
        [x, y, z, a, b]
    }
    fn visualize(&self, scene: &mut Scene, _a: f32, _b: f32) {
        let bed = Mesh::Cube(200.0, 200.0, 5.0);
        scene.add_entity(Entity { position: (0.0, 0.0, 0.0), rotation: (0.0, 0.0, 0.0), mesh: bed });
    }
    fn generate_path(&self, gcode: &gcode_parser::GCodeFile) -> Vec<[f64; 2]> {
        let mut points = Vec::new();
        for cmd in &gcode.commands {
            if cmd.starts_with("G1") {
                let parts: Vec<&str> = cmd.split_whitespace().collect();
                let mut x = 0.0;
                let mut y = 0.0;
                for part in parts {
                    if part.starts_with("X") {
                        x = part[1..].parse::<f64>().unwrap_or(0.0);
                    } else if part.starts_with("Y") {
                        y = part[1..].parse::<f64>().unwrap_or(0.0);
                    }
                }
                points.push([x, y]);
            }
        }
        points
    }
}

pub struct DeltaKinematics {
    pub config: DeltaConfig,
}
impl DeltaKinematics {
    pub fn new(config: DeltaConfig) -> Self { Self { config } }
}
impl Kinematics for DeltaKinematics {
    fn to_motor_positions(&self, x: f32, y: f32, z: f32, _a: f32, _b: f32) -> [f32; 5] {
        let a = (self.config.bed_radius * 2.0 / 3.0, 0.0);
        let b = (-self.config.bed_radius / 3.0, self.config.bed_radius * 3.0f32.sqrt() / 3.0);
        let c = (-self.config.bed_radius / 3.0, -self.config.bed_radius * 3.0f32.sqrt() / 3.0);
        let za = safe_sqrt(self.config.arm_length.powi(2) - (x - a.0).powi(2) - (y - a.1).powi(2)) + z;
        let zb = safe_sqrt(self.config.arm_length.powi(2) - (x - b.0).powi(2) - (y - b.1).powi(2)) + z;
        let zc = safe_sqrt(self.config.arm_length.powi(2) - (x - c.0).powi(2) - (y - c.1).powi(2)) + z;
        [za, zb, zc, 0.0, 0.0]
    }
    fn visualize(&self, scene: &mut Scene, _a: f32, _b: f32) {
        let bed_size = self.config.bed_radius * 2.0;
        let tower_height = self.config.arm_length * 2.0;
        let bed = Mesh::Cube(bed_size, bed_size, 5.0);
        scene.add_entity(Entity { position: (0.0, 0.0, 0.0), rotation: (0.0, 0.0, 0.0), mesh: bed });
        let tower = Mesh::Cube(10.0, 10.0, tower_height);
        scene.add_entity(Entity { position: (self.config.bed_radius * 2.0 / 3.0, 0.0, tower_height / 2.0), rotation: (0.0, 0.0, 0.0), mesh: tower.clone() });
        scene.add_entity(Entity { position: (-self.config.bed_radius / 3.0, self.config.bed_radius * 3.0f32.sqrt() / 3.0, tower_height / 2.0), rotation: (0.0, 0.0, 0.0), mesh: tower.clone() });
        scene.add_entity(Entity { position: (-self.config.bed_radius / 3.0, -self.config.bed_radius * 3.0f32.sqrt() / 3.0, tower_height / 2.0), rotation: (0.0, 0.0, 0.0), mesh: tower });
    }
    fn generate_path(&self, gcode: &gcode_parser::GCodeFile) -> Vec<[f64; 2]> {
        let mut points = Vec::new();
        let mut last_x = 0.0;
        let mut last_y = 0.0;
        for cmd in &gcode.commands {
            if cmd.starts_with("G1") {
                let parts: Vec<&str> = cmd.split_whitespace().collect();
                let mut x = last_x;
                let mut y = last_y;
                for part in parts {
                    if part.starts_with("X") {
                        x = part[1..].parse::<f64>().unwrap_or(last_x);
                    } else if part.starts_with("Y") {
                        y = part[1..].parse::<f64>().unwrap_or(last_y);
                    }
                }
                points.push([x, y]);
                last_x = x;
                last_y = y;
            }
        }
        points
    }
}

pub struct DualDeltaKinematics {
    pub primary: DeltaConfig,
    pub secondary: DeltaConfig,
}
impl DualDeltaKinematics {
    pub fn new(config: DualDeltaConfig) -> Self { Self { primary: config.primary, secondary: config.secondary } }
}
impl Kinematics for DualDeltaKinematics {
    fn to_motor_positions(&self, x: f32, y: f32, z: f32, _a: f32, _b: f32) -> [f32; 5] {
        let mut pos = DeltaKinematics { config: self.primary.clone() }.to_motor_positions(x, y, z, 0.0, 0.0);
        let secondary = DeltaKinematics { config: self.secondary.clone() }.to_motor_positions(x, y, z, 0.0, 0.0);
        pos[3] = secondary[0]; // D
        pos[4] = secondary[1]; // E
        pos
    }
    fn visualize(&self, scene: &mut Scene, _a: f32, _b: f32) {
        DeltaKinematics { config: self.primary.clone() }.visualize(scene, 0.0, 0.0);
        let bed = Mesh::Cube(self.secondary.bed_radius * 2.0, self.secondary.bed_radius * 2.0, 5.0);
        scene.add_entity(Entity { position: (300.0, 0.0, 0.0), rotation: (0.0, 0.0, 0.0), mesh: bed });
    }
    fn generate_path(&self, gcode: &gcode_parser::GCodeFile) -> Vec<[f64; 2]> {
        DeltaKinematics { config: self.primary.clone() }.generate_path(gcode)
    }
}

pub struct CoreXYKinematics {
    pub config: CoreXYConfig,
}
impl CoreXYKinematics {
    pub fn new(config: CoreXYConfig) -> Self { Self { config } }
}
impl Kinematics for CoreXYKinematics {
    fn to_motor_positions(&self, x: f32, y: f32, z: f32, a: f32, b: f32) -> [f32; 5] {
        let a_pos = x + y;
        let b_pos = x - y;
        [a_pos, b_pos, z, a, b]
    }
    fn visualize(&self, scene: &mut Scene, _a: f32, _b: f32) {
        let bed = Mesh::Cube(200.0, 200.0, 5.0);
        scene.add_entity(Entity { position: (0.0, 0.0, 0.0), rotation: (0.0, 0.0, 0.0), mesh: bed });
        let belt = Mesh::Cube(200.0, 2.0, 2.0);
        scene.add_entity(Entity { position: (0.0, 100.0, 10.0), rotation: (0.0, 0.0, self.config.belt_angle.to_radians()), mesh: belt });
    }
    fn generate_path(&self, gcode: &gcode_parser::GCodeFile) -> Vec<[f64; 2]> {
        CartesianKinematics.generate_path(gcode)
    }
}

pub struct HBotKinematics {
    pub config: HBotConfig,
}
impl HBotKinematics {
    pub fn new(config: HBotConfig) -> Self { Self { config } }
}
impl Kinematics for HBotKinematics {
    fn to_motor_positions(&self, x: f32, y: f32, z: f32, a: f32, b: f32) -> [f32; 5] {
        let a_pos = x + y;
        let b_pos = x - y;
        [a_pos, b_pos, z, a, b]
    }
    fn visualize(&self, scene: &mut Scene, _a: f32, _b: f32) {
        let bed = Mesh::Cube(200.0, 200.0, 5.0);
        scene.add_entity(Entity { position: (0.0, 0.0, 0.0), rotation: (0.0, 0.0, 0.0), mesh: bed });
        let belt = Mesh::Cube(200.0, 2.0, 2.0);
        scene.add_entity(Entity { position: (0.0, 100.0, 10.0), rotation: (0.0, 0.0, self.config.belt_angle.to_radians()), mesh: belt });
    }
    fn generate_path(&self, gcode: &gcode_parser::GCodeFile) -> Vec<[f64; 2]> {
        CartesianKinematics.generate_path(gcode)
    }
}

pub struct InverseDeltaKinematics {
    pub config: InverseDeltaConfig,
}
impl InverseDeltaKinematics {
    pub fn new(config: InverseDeltaConfig) -> Self { Self { config } }
}
impl Kinematics for InverseDeltaKinematics {
    fn to_motor_positions(&self, x: f32, y: f32, z: f32, _a: f32, _b: f32) -> [f32; 5] {
        let a = (self.config.bed_radius * 2.0 / 3.0, 0.0);
        let b = (-self.config.bed_radius / 3.0, self.config.bed_radius * 3.0f32.sqrt() / 3.0);
        let c = (-self.config.bed_radius / 3.0, -self.config.bed_radius * 3.0f32.sqrt() / 3.0);
        let za = safe_sqrt(self.config.arm_length.powi(2) - (x - a.0).powi(2) - (y - a.1).powi(2)) + z;
        let zb = safe_sqrt(self.config.arm_length.powi(2) - (x - b.0).powi(2) - (y - b.1).powi(2)) + z;
        let zc = safe_sqrt(self.config.arm_length.powi(2) - (x - c.0).powi(2) - (y - c.1).powi(2)) + z;
        [za, zb, zc, 0.0, 0.0]
    }
    fn visualize(&self, scene: &mut Scene, _a: f32, _b: f32) {
        let bed_size = self.config.bed_radius * 2.0;
        let tower_height = self.config.arm_length * 2.0;
        let bed = Mesh::Cube(bed_size, bed_size, 5.0);
        scene.add_entity(Entity { position: (0.0, 0.0, 0.0), rotation: (0.0, 0.0, 0.0), mesh: bed });
        let tower = Mesh::Cube(10.0, 10.0, tower_height);
        scene.add_entity(Entity { position: (self.config.bed_radius * 2.0 / 3.0, 0.0, tower_height / 2.0), rotation: (0.0, 0.0, 0.0), mesh: tower.clone() });
        scene.add_entity(Entity { position: (-self.config.bed_radius / 3.0, self.config.bed_radius * 3.0f32.sqrt() / 3.0, tower_height / 2.0), rotation: (0.0, 0.0, 0.0), mesh: tower.clone() });
        scene.add_entity(Entity { position: (-self.config.bed_radius / 3.0, -self.config.bed_radius * 3.0f32.sqrt() / 3.0, tower_height / 2.0), rotation: (0.0, 0.0, 0.0), mesh: tower });
    }
    fn generate_path(&self, gcode: &gcode_parser::GCodeFile) -> Vec<[f64; 2]> {
        DeltaKinematics { config: DeltaConfig {
            arm_length: self.config.arm_length,
            rod_length: self.config.rod_length,
            bed_radius: self.config.bed_radius,
        }}.generate_path(gcode)
    }
}

pub struct PolarKinematics {
    pub config: PolarConfig,
}
impl PolarKinematics {
    pub fn new(config: PolarConfig) -> Self { Self { config } }
}
impl Kinematics for PolarKinematics {
    fn to_motor_positions(&self, x: f32, y: f32, z: f32, a: f32, b: f32) -> [f32; 5] {
        let r = (x.powi(2) + y.powi(2)).sqrt();
        let theta = y.atan2(x).to_degrees();
        [r, theta, z, a, b]
    }
    fn visualize(&self, scene: &mut Scene, _a: f32, _b: f32) {
        let bed = Mesh::Cylinder(self.config.max_radius, 5.0);
        scene.add_entity(Entity { position: (0.0, 0.0, 0.0), rotation: (0.0, 0.0, 0.0), mesh: bed });
        let arm = Mesh::Cube(self.config.max_radius, 10.0, 10.0);
        scene.add_entity(Entity { position: (self.config.max_radius / 2.0, 0.0, 10.0), rotation: (0.0, 0.0, 0.0), mesh: arm });
    }
    fn generate_path(&self, gcode: &gcode_parser::GCodeFile) -> Vec<[f64; 2]> {
        let mut points = Vec::new();
        let mut last_r = 0.0;
        let mut last_theta = 0.0;
        for cmd in &gcode.commands {
            if cmd.starts_with("G1") {
                let parts: Vec<&str> = cmd.split_whitespace().collect();
                let mut r = last_r;
                let mut theta = last_theta;
                for part in parts {
                    if part.starts_with("R") {
                        r = part[1..].parse::<f64>().unwrap_or(last_r);
                    } else if part.starts_with("T") {
                        theta = part[1..].parse::<f64>().unwrap_or(last_theta);
                    }
                }
                points.push([r, theta]);
                last_r = r;
                last_theta = theta;
            }
        }
        points
    }
}

pub struct FiveAxisTrunnionKinematics {
    pub config: FiveAxisTrunnionConfig,
}
impl FiveAxisTrunnionKinematics {
    pub fn new(config: FiveAxisTrunnionConfig) -> Self { Self { config } }
}
impl Kinematics for FiveAxisTrunnionKinematics {
    fn to_motor_positions(&self, x: f32, y: f32, z: f32, a: f32, b: f32) -> [f32; 5] {
        let rot_a = Rotation3::from_axis_angle(&Vector3::x_axis(), a.to_radians());
        let rot_b = Rotation3::from_axis_angle(&Vector3::y_axis(), b.to_radians());
        let pos = (rot_b * rot_a).matrix() * Vector3::new(x, y, z);
        [pos.x, pos.y, pos.z, a, b]
    }
    fn visualize(&self, scene: &mut Scene, a: f32, b: f32) {
        let bed = Mesh::Cube(self.config.table_length, self.config.table_length, 5.0);
        scene.add_entity(Entity { position: (0.0, 0.0, 0.0), rotation: (a, b, 0.0), mesh: bed });
    }
    fn generate_path(&self, gcode: &gcode_parser::GCodeFile) -> Vec<[f64; 2]> {
        CartesianKinematics.generate_path(gcode)
    }
}

pub struct FiveAxisBedKinematics {
    pub config: FiveAxisBedConfig,
}
impl FiveAxisBedKinematics {
    pub fn new(config: FiveAxisBedConfig) -> Self { Self { config } }
}
impl Kinematics for FiveAxisBedKinematics {
    fn to_motor_positions(&self, x: f32, y: f32, z: f32, a: f32, b: f32) -> [f32; 5] {
        let rot_a = Rotation3::from_axis_angle(&Vector3::x_axis(), a.to_radians());
        let rot_b = Rotation3::from_axis_angle(&Vector3::y_axis(), b.to_radians());
        let pos = (rot_b * rot_a).matrix() * Vector3::new(x, y, z);
        [pos.x, pos.y, pos.z, a, b]
    }
    fn visualize(&self, scene: &mut Scene, a: f32, b: f32) {
        let bed = Mesh::Cube(self.config.bed_length, self.config.bed_length, 5.0);
        scene.add_entity(Entity { position: (0.0, 0.0, 0.0), rotation: (a, b, 0.0), mesh: bed });
    }
    fn generate_path(&self, gcode: &gcode_parser::GCodeFile) -> Vec<[f64; 2]> {
        CartesianKinematics.generate_path(gcode)
    }
}

pub struct SphericalParallelKinematics {
    pub config: SphericalParallelConfig,
}
impl SphericalParallelKinematics {
    pub fn new(config: SphericalParallelConfig) -> Self { Self { config } }
}
impl Kinematics for SphericalParallelKinematics {
    fn to_motor_positions(&self, x: f32, y: f32, z: f32, _a: f32, _b: f32) -> [f32; 5] {
        [x, y, z, 0.0, 0.0]
    }
    fn visualize(&self, scene: &mut Scene, _a: f32, _b: f32) {
        let platform = Mesh::Cylinder(self.config.platform_radius, 5.0);
        scene.add_entity(Entity { position: (0.0, 0.0, 100.0), rotation: (0.0, 0.0, 0.0), mesh: platform });
        let base = Mesh::Cylinder(self.config.base_radius, 5.0);
        scene.add_entity(Entity { position: (0.0, 0.0, 0.0), rotation: (0.0, 0.0, 0.0), mesh: base });
    }
    fn generate_path(&self, gcode: &gcode_parser::GCodeFile) -> Vec<[f64; 2]> {
        CartesianKinematics.generate_path(gcode)
    }
}

pub fn create_kinematics(config: &Config) -> Box<dyn Kinematics> {
    match config.printer.kinematics.as_str() {
        "cartesian" => Box::new(CartesianKinematics),
        "delta" => Box::new(DeltaKinematics::new(config.printer.delta.as_ref().expect("Delta config required").clone())),
        "dual_delta" => Box::new(DualDeltaKinematics::new(config.printer.dual_delta.as_ref().expect("Dual Delta config required").clone())),
        "corexy" => Box::new(CoreXYKinematics::new(config.printer.corexy.as_ref().expect("CoreXY config required").clone())),
        "hbot" => Box::new(HBotKinematics::new(config.printer.hbot.as_ref().expect("HBot config required").clone())),
        "inverse_delta" => Box::new(InverseDeltaKinematics::new(config.printer.inverse_delta.as_ref().expect("Inverse Delta config required").clone())),
        "polar" => Box::new(PolarKinematics::new(config.printer.polar.as_ref().expect("Polar config required").clone())),
        "five_axis_trunnion" => Box::new(FiveAxisTrunnionKinematics::new(config.printer.five_axis_trunnion.as_ref().expect("5-axis Trunnion config required").clone())),
        "five_axis_bed" => Box::new(FiveAxisBedKinematics::new(config.printer.five_axis_bed.as_ref().expect("5-axis Bed config required").clone())),
        "spherical_parallel" => Box::new(SphericalParallelKinematics::new(config.printer.spherical_parallel.as_ref().expect("SPM config required").clone())),
        _ => panic!("Unsupported kinematics"),
    }
}

#[cfg(feature = "dynamic")]
pub fn load_custom_kinematics(path: &str) -> Box<dyn Kinematics> {
    use libloading::{Library, Symbol};
    unsafe {
        let lib = Library::new(path).expect("Failed to load library");
        let constructor: Symbol<unsafe extern "C" fn() -> Box<dyn Kinematics>> = lib.get(b"create_kinematics").expect("Failed to load symbol");
        constructor()
    }
}