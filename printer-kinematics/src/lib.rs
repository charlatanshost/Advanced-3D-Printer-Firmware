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

/// Parse the XY path from G0/G1 moves. Omitted words keep their last value, moves with
/// no X or Y (E-only, Z-only, F-only) add no point, and G10/G17 are not mistaken for G1.
fn xy_path(gcode: &gcode_parser::GCodeFile) -> Vec<[f64; 2]> {
    let mut points = Vec::new();
    let (mut x, mut y) = (0.0, 0.0);
    for cmd in &gcode.commands {
        let mut words = cmd.split_whitespace();
        if !matches!(words.next(), Some("G0" | "G1" | "G00" | "G01" | "g0" | "g1")) {
            continue;
        }
        let mut has_xy = false;
        for word in words {
            let (axis, value) = word.split_at(word.chars().next().map_or(0, char::len_utf8));
            let Ok(value) = value.parse::<f64>() else { continue };
            match axis {
                "X" | "x" => { x = value; has_xy = true; }
                "Y" | "y" => { y = value; has_xy = true; }
                _ => {}
            }
        }
        if has_xy {
            points.push([x, y]);
        }
    }
    points
}

/// Is this tower position reachable on a delta? (Rod must reach the carriage.)
fn delta_reachable(arm_length: f32, bed_radius: f32, x: f32, y: f32) -> bool {
    let towers = [
        (bed_radius * 2.0 / 3.0, 0.0),
        (-bed_radius / 3.0, bed_radius * 3.0f32.sqrt() / 3.0),
        (-bed_radius / 3.0, -bed_radius * 3.0f32.sqrt() / 3.0),
    ];
    towers.iter().all(|t| arm_length.powi(2) - (x - t.0).powi(2) - (y - t.1).powi(2) >= 0.0)
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
    /// False if the machine cannot physically reach this position. Motor positions
    /// computed for an unreachable point are meaningless and must not be sent.
    fn is_reachable(&self, _x: f32, _y: f32, _z: f32, _a: f32, _b: f32) -> bool {
        true
    }
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
        xy_path(gcode)
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
        xy_path(gcode)
    }
    fn is_reachable(&self, x: f32, y: f32, _z: f32, _a: f32, _b: f32) -> bool {
        delta_reachable(self.config.arm_length, self.config.bed_radius, x, y)
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
    fn is_reachable(&self, x: f32, y: f32, _z: f32, _a: f32, _b: f32) -> bool {
        delta_reachable(self.primary.arm_length, self.primary.bed_radius, x, y)
            && delta_reachable(self.secondary.arm_length, self.secondary.bed_radius, x, y)
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
    fn is_reachable(&self, x: f32, y: f32, _z: f32, _a: f32, _b: f32) -> bool {
        delta_reachable(self.config.arm_length, self.config.bed_radius, x, y)
    }
}

pub struct PolarKinematics {
    pub config: PolarConfig,
    /// Last commanded motor angle. Lets the angle keep counting past +/-180 degrees, so
    /// crossing the -X axis is a small move rather than a full turn the other way.
    last_theta: std::cell::Cell<f32>,
}
impl PolarKinematics {
    pub fn new(config: PolarConfig) -> Self { Self { config, last_theta: std::cell::Cell::new(0.0) } }
}
impl Kinematics for PolarKinematics {
    fn to_motor_positions(&self, x: f32, y: f32, z: f32, a: f32, b: f32) -> [f32; 5] {
        let r = (x.powi(2) + y.powi(2)).sqrt();
        let last = self.last_theta.get();
        // At the centre the angle is undefined: stay where the motor is.
        let theta = if r < 1e-6 {
            last
        } else {
            let raw = y.atan2(x).to_degrees();
            last + (raw - last + 180.0).rem_euclid(360.0) - 180.0
        };
        self.last_theta.set(theta);
        [r, theta, z, a, b]
    }
    fn visualize(&self, scene: &mut Scene, _a: f32, _b: f32) {
        let bed = Mesh::Cylinder(self.config.max_radius, 5.0);
        scene.add_entity(Entity { position: (0.0, 0.0, 0.0), rotation: (0.0, 0.0, 0.0), mesh: bed });
        let arm = Mesh::Cube(self.config.max_radius, 10.0, 10.0);
        scene.add_entity(Entity { position: (self.config.max_radius / 2.0, 0.0, 10.0), rotation: (0.0, 0.0, 0.0), mesh: arm });
    }
    fn generate_path(&self, gcode: &gcode_parser::GCodeFile) -> Vec<[f64; 2]> {
        // Slicers emit Cartesian X/Y even for polar machines (to_motor_positions takes X/Y too)
        xy_path(gcode)
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

#[cfg(test)]
mod tests {
    use super::*;

    fn gcode(commands: &[&str]) -> gcode_parser::GCodeFile {
        gcode_parser::GCodeFile { commands: commands.iter().map(|c| c.to_string()).collect() }
    }

    #[test]
    fn paths_keep_position_and_skip_non_moves() {
        let path = CartesianKinematics.generate_path(&gcode(&["G1 X10 Y10", "G1 E-1", "G1 Y20", "G0 X5", "G10", "G17", "G1 F100", "M104 S200"]));
        assert_eq!(path, vec![[10.0, 10.0], [10.0, 20.0], [5.0, 20.0]]);
    }

    #[test]
    fn polar_path_reads_xy_like_the_slicer_writes() {
        let polar = PolarKinematics::new(PolarConfig { max_radius: 100.0 });
        assert_eq!(polar.generate_path(&gcode(&["G1 X10 Y10"])), vec![[10.0, 10.0]]);
    }

    #[test]
    fn polar_does_not_spin_round_when_crossing_the_negative_x_axis() {
        let polar = PolarKinematics::new(PolarConfig { max_radius: 100.0 });
        let before = polar.to_motor_positions(-10.0, 0.01, 0.0, 0.0, 0.0)[1];
        let after = polar.to_motor_positions(-10.0, -0.01, 0.0, 0.0, 0.0)[1];
        assert!((after - before).abs() < 1.0, "{} then {}", before, after);
        // And the centre leaves the motor alone
        assert_eq!(polar.to_motor_positions(0.0, 0.0, 0.0, 0.0, 0.0)[1], after);
    }

    #[test]
    fn delta_knows_what_it_cannot_reach() {
        let delta = DeltaKinematics::new(DeltaConfig { arm_length: 100.0, rod_length: 200.0, bed_radius: 100.0 });
        assert!(delta.is_reachable(0.0, 0.0, 10.0, 0.0, 0.0));
        assert!(!delta.is_reachable(500.0, 0.0, 10.0, 0.0, 0.0));
        assert!(CartesianKinematics.is_reachable(500.0, 0.0, 10.0, 0.0, 0.0));
    }

    #[test]
    fn embedded_tool_reaches_the_bed_and_limits_use_axis_units() {
        use embedded_optimized::*;
        let penta = EmbeddedKinematics::new(EmbeddedMachineConfig::PENTA_AXIS);
        let at_bed = Pose5D { x: 0.0, y: 0.0, z: 0.2, a: 0.0, b_or_c: 0.0 };
        let joints = penta.inverse_kinematics(at_bed).expect("a nozzle must be able to reach the bed");
        assert!((joints[2] - 45.2).abs() < 1e-3, "{:?}", joints);

        let mut config = EmbeddedMachineConfig::PENTA_AXIS;
        config.machine_type = MachineType::CoreXYRotary;
        config.tool_offset = 0.0;
        let corexy = EmbeddedKinematics::new(config);
        let corner = Pose5D { x: 100.0, y: 100.0, z: 50.0, a: 0.0, b_or_c: 0.0 };
        assert_eq!(corexy.inverse_kinematics(corner).unwrap()[0], 200.0, "motor A is X+Y");

        let table = lookup_tables::generate_sin_table();
        assert!((table[900] - 1.0).abs() < 1e-5, "sin 90 degrees");
    }
}

#[cfg(feature = "dynamic")]
pub fn load_custom_kinematics(path: &str) -> Box<dyn Kinematics> {
    use libloading::{Library, Symbol};
    unsafe {
        let lib = Library::new(path).expect("Failed to load library");
        let constructor: Symbol<unsafe extern "C" fn() -> Box<dyn Kinematics>> = lib.get(b"create_kinematics").expect("Failed to load symbol");
        let kinematics = constructor();
        // The object's code and vtable live in the library, so it must stay loaded for
        // the rest of the program. Dropping `lib` here would leave them dangling.
        drop(constructor);
        std::mem::forget(lib);
        kinematics
    }
}