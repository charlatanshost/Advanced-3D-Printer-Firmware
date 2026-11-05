use crate::five_axis::{Pose6DOF, FiveAxisKinematics, KinematicsError};
use crate::motion_planning::{MotionSegment, ToolOrientationMode};
use nalgebra::{Vector3, Point3, Unit, Rotation3, UnitQuaternion, Matrix3};
use std::collections::HashMap;

/// Advanced tool orientation control for 5-axis machining and non-planar printing
pub struct ToolOrientationController {
    kinematics: FiveAxisKinematics,
    active_tool: ToolDefinition,
    workpiece_coordinate_system: WorkpieceCS,
    orientation_strategies: HashMap<String, OrientationStrategy>,
}

#[derive(Debug, Clone)]
pub struct ToolDefinition {
    pub tool_id: u32,
    pub tool_type: ToolType,
    pub length: f64,                    // Tool length (mm)
    pub diameter: f64,                  // Tool diameter (mm)
    pub tip_radius: f64,                // Tool tip radius (mm)
    pub cutting_edge_angle: f64,        // For V-bits and chamfer tools (degrees)
    pub flute_count: u32,               // Number of cutting edges
    pub recommended_speeds: SpeedRange,  // Recommended cutting speeds
    pub material_properties: ToolMaterial,
}

#[derive(Debug, Clone)]
pub enum ToolType {
    /// 3D printer extruder nozzle
    Extruder {
        nozzle_diameter: f64,
        flow_rate: f64,
        temperature: f64,
    },
    /// End mill for machining
    EndMill {
        flute_count: u32,
        helix_angle: f64,
        coating: String,
    },
    /// Ball nose end mill
    BallNose {
        corner_radius: f64,
    },
    /// V-bit for engraving
    VBit {
        included_angle: f64,
        flat_tip_diameter: f64,
    },
    /// Chamfer tool
    Chamfer {
        angle: f64,
    },
    /// Probe for measurement
    Probe {
        trigger_force: f64,
        repeatability: f64,
    },
}

#[derive(Debug, Clone)]
pub struct SpeedRange {
    pub min_spindle_rpm: f64,
    pub max_spindle_rpm: f64,
    pub min_feedrate: f64,               // mm/min
    pub max_feedrate: f64,               // mm/min
}

#[derive(Debug, Clone)]
pub struct ToolMaterial {
    pub material: String,                // HSS, Carbide, Diamond, etc.
    pub hardness: f64,                   // HRC or equivalent
    pub thermal_conductivity: f64,       // W/m·K
    pub wear_resistance: f64,            // Relative scale 0-1
}

/// Workpiece coordinate system for part setup
#[derive(Debug, Clone)]
pub struct WorkpieceCS {
    pub origin: Point3<f64>,             // Workpiece origin in machine coordinates
    pub rotation: UnitQuaternion<f64>,   // Workpiece orientation
    pub name: String,                    // G54, G55, etc.
    pub setup_description: String,       // Human-readable setup description
}

/// Strategic tool orientation control
#[derive(Debug, Clone)]
pub enum OrientationStrategy {
    /// Maintain constant tool orientation
    Constant {
        orientation: UnitQuaternion<f64>,
    },
    
    /// Follow surface normal with specified lead/lag and tilt angles
    SurfaceNormal {
        lead_angle: f64,                 // Lead angle in degrees (positive = leading)
        tilt_angle: f64,                 // Tilt angle in degrees (for side clearance)
        surface_normal_source: SurfaceSource,
    },
    
    /// Optimize for best surface finish
    SurfaceFinish {
        preferred_cutting_direction: Vector3<f64>,
        climb_milling: bool,             // Prefer climb milling over conventional
        stepover_optimization: bool,     // Adjust tool angle for optimal stepover
    },
    
    /// Collision avoidance strategy
    CollisionAvoidance {
        avoidance_angle: f64,            // Degrees to tilt away from obstacles
        priority_axis: String,           // Which axis to prefer for avoidance
        max_deviation: f64,              // Maximum allowed deviation from ideal
    },
    
    /// Tool accessibility optimization
    Accessibility {
        prefer_positive_angles: bool,    // Prefer positive A/B angles
        minimize_joint_motion: bool,     // Minimize rotary axis movement
        avoid_singularities: bool,       // Stay away from kinematic singularities
    },
    
    /// Custom orientation control
    Custom {
        control_function: fn(&Point3<f64>, &Vector3<f64>) -> UnitQuaternion<f64>,
    },
}

#[derive(Debug, Clone)]
pub enum SurfaceSource {
    /// Surface defined by CAD model
    CADModel {
        model_file: String,
        tolerance: f64,
    },
    /// Surface defined by point cloud
    PointCloud {
        points: Vec<Point3<f64>>,
        normals: Vec<Unit<Vector3<f64>>>,
    },
    /// Analytical surface (plane, cylinder, sphere, etc.)
    Analytical {
        surface_type: AnalyticalSurface,
    },
    /// Surface extracted from STL mesh
    STLMesh {
        vertices: Vec<Point3<f64>>,
        triangles: Vec<[usize; 3]>,
        face_normals: Vec<Unit<Vector3<f64>>>,
    },
}

#[derive(Debug, Clone)]
pub enum AnalyticalSurface {
    Plane {
        point: Point3<f64>,
        normal: Unit<Vector3<f64>>,
    },
    Cylinder {
        axis_point: Point3<f64>,
        axis_direction: Unit<Vector3<f64>>,
        radius: f64,
    },
    Sphere {
        center: Point3<f64>,
        radius: f64,
    },
    Cone {
        apex: Point3<f64>,
        axis_direction: Unit<Vector3<f64>>,
        half_angle: f64,
    },
}

/// G-code interpreter for 5-axis operations
pub struct FiveAxisGCodeProcessor {
    orientation_controller: ToolOrientationController,
    current_position: Pose6DOF,
    modal_state: ModalState,
    coordinate_systems: HashMap<String, WorkpieceCS>,
    tool_library: HashMap<u32, ToolDefinition>,
}

#[derive(Debug, Clone)]
pub struct ModalState {
    pub motion_mode: MotionMode,         // G0, G1, G2, G3, etc.
    pub coordinate_system: String,       // G54, G55, etc.
    pub plane_selection: PlaneSelection, // G17, G18, G19
    pub feedrate: f64,                   // F word value
    pub spindle_speed: f64,              // S word value
    pub tool_number: u32,                // T word value
    pub coolant_state: CoolantState,     // M7, M8, M9
    pub units: Units,                    // G20, G21
    pub tool_length_compensation: bool,  // G43/G49
    pub tool_radius_compensation: CompensationMode, // G40, G41, G42
}

#[derive(Debug, Clone)]
pub enum MotionMode {
    Rapid,                               // G0
    Linear,                              // G1
    CircularCW,                          // G2
    CircularCCW,                         // G3
    Dwell,                               // G4
    ProbeToward,                         // G38.2
    ProbeAway,                           // G38.3
}

#[derive(Debug, Clone)]
pub enum PlaneSelection {
    XY,                                  // G17
    XZ,                                  // G18
    YZ,                                  // G19
}

#[derive(Debug, Clone)]
pub enum CoolantState {
    Off,                                 // M9
    Mist,                               // M7
    Flood,                              // M8
    Both,                               // M7 + M8
}

#[derive(Debug, Clone)]
pub enum Units {
    Inches,                             // G20
    Millimeters,                        // G21
}

#[derive(Debug, Clone)]
pub enum CompensationMode {
    Off,                                // G40
    Left,                               // G41
    Right,                              // G42
}

/// Parsed 5-axis G-code command
#[derive(Debug, Clone)]
pub struct FiveAxisCommand {
    pub gcode: String,                   // Original G-code line
    pub motion_type: MotionMode,
    pub target_pose: Option<Pose6DOF>,   // Target position and orientation
    pub feedrate: Option<f64>,
    pub spindle_speed: Option<f64>,
    pub tool_change: Option<u32>,
    pub orientation_mode: Option<ToolOrientationMode>,
    pub additional_axes: HashMap<String, f64>, // U, V, W, etc.
}

impl ToolOrientationController {
    pub fn new(kinematics: FiveAxisKinematics) -> Self {
        let active_tool = ToolDefinition::default_extruder();
        let workpiece_cs = WorkpieceCS::default();
        let orientation_strategies = HashMap::new();
        
        Self {
            kinematics,
            active_tool,
            workpiece_coordinate_system: workpiece_cs,
            orientation_strategies,
        }
    }
    
    pub fn set_active_tool(&mut self, tool: ToolDefinition) {
        self.active_tool = tool;
    }
    
    pub fn set_workpiece_coordinate_system(&mut self, cs: WorkpieceCS) {
        self.workpiece_coordinate_system = cs;
    }
    
    pub fn add_orientation_strategy(&mut self, name: String, strategy: OrientationStrategy) {
        self.orientation_strategies.insert(name, strategy);
    }
    
    /// Calculate optimal tool orientation for given position and strategy
    pub fn calculate_tool_orientation(&self, position: &Point3<f64>, strategy: &OrientationStrategy) -> Result<UnitQuaternion<f64>, KinematicsError> {
        match strategy {
            OrientationStrategy::Constant { orientation } => {
                Ok(*orientation)
            },
            
            OrientationStrategy::SurfaceNormal { lead_angle, tilt_angle, surface_normal_source } => {
                let surface_normal = self.get_surface_normal_at_point(position, surface_normal_source)?;
                self.apply_lead_and_tilt(surface_normal, *lead_angle, *tilt_angle)
            },
            
            OrientationStrategy::SurfaceFinish { preferred_cutting_direction, climb_milling, .. } => {
                let surface_normal = self.estimate_surface_normal(position)?;
                self.optimize_for_surface_finish(surface_normal, preferred_cutting_direction, *climb_milling)
            },
            
            OrientationStrategy::CollisionAvoidance { avoidance_angle, priority_axis, .. } => {
                self.calculate_collision_avoiding_orientation(position, *avoidance_angle, priority_axis)
            },
            
            OrientationStrategy::Accessibility { prefer_positive_angles, minimize_joint_motion, .. } => {
                self.optimize_for_accessibility(position, *prefer_positive_angles, *minimize_joint_motion)
            },
            
            OrientationStrategy::Custom { control_function } => {
                let surface_normal = self.estimate_surface_normal(position)?;
                Ok(control_function(position, &surface_normal.into_inner()))
            },
        }
    }
    
    fn get_surface_normal_at_point(&self, position: &Point3<f64>, source: &SurfaceSource) -> Result<Unit<Vector3<f64>>, KinematicsError> {
        match source {
            SurfaceSource::Analytical { surface_type } => {
                self.calculate_analytical_surface_normal(position, surface_type)
            },
            
            SurfaceSource::PointCloud { points, normals } => {
                self.interpolate_normal_from_point_cloud(position, points, normals)
            },
            
            SurfaceSource::STLMesh { vertices, triangles, face_normals } => {
                self.calculate_mesh_normal_at_point(position, vertices, triangles, face_normals)
            },
            
            SurfaceSource::CADModel { model_file: _, tolerance: _ } => {
                // In practice, this would interface with a CAD kernel
                // For now, return a default vertical normal
                Ok(Unit::new_normalize(Vector3::z()))
            },
        }
    }
    
    fn calculate_analytical_surface_normal(&self, position: &Point3<f64>, surface: &AnalyticalSurface) -> Result<Unit<Vector3<f64>>, KinematicsError> {
        match surface {
            AnalyticalSurface::Plane { normal, .. } => {
                Ok(*normal)
            },
            
            AnalyticalSurface::Cylinder { axis_point, axis_direction, radius: _ } => {
                let to_point = position - axis_point;
                let axis_component = to_point.dot(&axis_direction.into_inner()) * axis_direction.into_inner();
                let radial_component = to_point - axis_component;
                // On the axis itself no direction is "outward"
                Unit::try_new(radial_component, 1e-9).ok_or(KinematicsError::SolutionNotFound)
            },
            
            AnalyticalSurface::Sphere { center, .. } => {
                let radial_vector = position - center;
                Unit::try_new(radial_vector, 1e-9).ok_or(KinematicsError::SolutionNotFound)
            },
            
            AnalyticalSurface::Cone { apex, axis_direction, half_angle } => {
                let to_point = position - apex;
                let axis_component = to_point.dot(&axis_direction.into_inner());
                let radial_component = to_point - axis_component * axis_direction.into_inner();
                let radial = radial_component.try_normalize(1e-9).ok_or(KinematicsError::SolutionNotFound)?;

                // Calculate cone surface normal
                let cone_angle_rad = half_angle.to_radians();
                let normal_direction = radial * cone_angle_rad.cos() +
                                     axis_direction.into_inner() * cone_angle_rad.sin();
                Ok(Unit::new_normalize(normal_direction))
            },
        }
    }
    
    fn interpolate_normal_from_point_cloud(&self, position: &Point3<f64>, points: &[Point3<f64>], normals: &[Unit<Vector3<f64>>]) -> Result<Unit<Vector3<f64>>, KinematicsError> {
        if points.len() != normals.len() {
            return Err(KinematicsError::InvalidConfiguration);
        }

        // Find closest points and interpolate normal
        let mut closest_indices = Vec::new();
        let mut distances = Vec::new();
        
        for (i, point) in points.iter().enumerate() {
            let distance = (position - point).norm();
            closest_indices.push(i);
            distances.push(distance);
        }
        
        // Sort by distance and take closest 3 points
        let mut indexed_distances: Vec<(usize, f64)> = closest_indices.iter().zip(distances.iter()).map(|(&i, &d)| (i, d)).collect();
        indexed_distances.sort_by(|a, b| a.1.total_cmp(&b.1));
        
        if indexed_distances.len() >= 3 {
            // Weighted average of closest normals
            let mut weighted_normal = Vector3::zeros();
            let mut total_weight = 0.0;
            
            for i in 0..3.min(indexed_distances.len()) {
                let (index, distance) = indexed_distances[i];
                let weight = 1.0 / (distance + 1e-6); // Avoid division by zero
                weighted_normal += weight * normals[index].into_inner();
                total_weight += weight;
            }
            
            // Opposing normals can cancel out
            Unit::try_new(weighted_normal / total_weight, 1e-9).ok_or(KinematicsError::SolutionNotFound)
        } else {
            Ok(Unit::new_normalize(Vector3::z())) // Default to Z-up
        }
    }
    
    fn calculate_mesh_normal_at_point(&self, position: &Point3<f64>, vertices: &[Point3<f64>], triangles: &[[usize; 3]], face_normals: &[Unit<Vector3<f64>>]) -> Result<Unit<Vector3<f64>>, KinematicsError> {
        // Find the triangle containing the point (simplified approach)
        let mut closest_triangle = 0;
        let mut min_distance = f64::INFINITY;
        
        for (i, triangle) in triangles.iter().enumerate() {
            let v0 = vertices[triangle[0]];
            let v1 = vertices[triangle[1]];
            let v2 = vertices[triangle[2]];
            
            // Calculate distance to triangle (simplified - distance to centroid)
            let centroid = (v0 + v1.coords + v2.coords) / 3.0;
            let distance = (position - centroid).norm();
            
            if distance < min_distance {
                min_distance = distance;
                closest_triangle = i;
            }
        }
        
        face_normals.get(closest_triangle).copied().ok_or(KinematicsError::InvalidConfiguration)
    }
    
    fn apply_lead_and_tilt(&self, surface_normal: Unit<Vector3<f64>>, lead_angle: f64, tilt_angle: f64) -> Result<UnitQuaternion<f64>, KinematicsError> {
        // Apply lead angle (rotation around surface tangent)
        let lead_rotation = Rotation3::from_axis_angle(&Vector3::y_axis(), lead_angle.to_radians());
        
        // Apply tilt angle (rotation around other tangent)
        let tilt_rotation = Rotation3::from_axis_angle(&Vector3::x_axis(), tilt_angle.to_radians());
        
        // Combine rotations
        let combined_rotation = tilt_rotation * lead_rotation;
        
        // Apply to surface normal to get tool orientation
        let tool_direction = combined_rotation * surface_normal.into_inner();
        
        // Convert to quaternion (tool pointing along Z-axis)
        let rotation_to_z = Rotation3::rotation_between(&Vector3::z(), &tool_direction)
            .unwrap_or_else(|| Rotation3::identity());
        
        Ok(UnitQuaternion::from_rotation_matrix(&rotation_to_z))
    }
    
    fn optimize_for_surface_finish(&self, surface_normal: Unit<Vector3<f64>>, cutting_direction: &Vector3<f64>, climb_milling: bool) -> Result<UnitQuaternion<f64>, KinematicsError> {
        // Optimize tool orientation for best surface finish
        let tool_axis = if climb_milling {
            -surface_normal.into_inner()
        } else {
            surface_normal.into_inner()
        };
        
        // Align tool with preferred cutting direction
        // Cutting along the tool axis leaves no side direction to work with
        let side_vector = cutting_direction.cross(&tool_axis).try_normalize(1e-9).ok_or(KinematicsError::SolutionNotFound)?;
        let adjusted_cutting_direction = tool_axis.cross(&side_vector);
        
        // Create rotation matrix
        let rotation_matrix = Matrix3::from_columns(&[
            side_vector,
            adjusted_cutting_direction,
            tool_axis,
        ]);
        
        Ok(UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation_matrix)))
    }
    
    fn calculate_collision_avoiding_orientation(&self, _position: &Point3<f64>, avoidance_angle: f64, priority_axis: &str) -> Result<UnitQuaternion<f64>, KinematicsError> {
        // Simple collision avoidance - tilt tool away from obstacles
        let tilt_rotation = match priority_axis {
            "A" | "a" => Rotation3::from_axis_angle(&Vector3::x_axis(), avoidance_angle.to_radians()),
            "B" | "b" => Rotation3::from_axis_angle(&Vector3::y_axis(), avoidance_angle.to_radians()),
            "C" | "c" => Rotation3::from_axis_angle(&Vector3::z_axis(), avoidance_angle.to_radians()),
            _ => Rotation3::identity(),
        };
        
        Ok(UnitQuaternion::from_rotation_matrix(&tilt_rotation))
    }
    
    fn optimize_for_accessibility(&self, _position: &Point3<f64>, prefer_positive: bool, minimize_motion: bool) -> Result<UnitQuaternion<f64>, KinematicsError> {
        // Choose tool orientation that minimizes joint angles and avoids limits
        
        if minimize_motion {
            // Try to maintain current orientation
            Ok(UnitQuaternion::identity())
        } else if prefer_positive {
            // Prefer positive rotary angles
            let rotation = Rotation3::from_euler_angles(0.1, 0.1, 0.0);
            Ok(UnitQuaternion::from_rotation_matrix(&rotation))
        } else {
            Ok(UnitQuaternion::identity())
        }
    }
    
    fn estimate_surface_normal(&self, _position: &Point3<f64>) -> Result<Unit<Vector3<f64>>, KinematicsError> {
        // Default implementation - assumes flat surface
        Ok(Unit::new_normalize(Vector3::z()))
    }
    
    /// Generate optimized 5-axis tool path
    pub fn generate_tool_path(&self, positions: &[Point3<f64>], strategy_name: &str) -> Result<Vec<Pose6DOF>, KinematicsError> {
        let strategy = self.orientation_strategies.get(strategy_name)
            .ok_or(KinematicsError::InvalidConfiguration)?;
        
        let mut path = Vec::new();
        
        for position in positions {
            let orientation = self.calculate_tool_orientation(position, strategy)?;
            let pose = Pose6DOF {
                position: *position,
                orientation,
            };
            path.push(pose);
        }
        
        Ok(path)
    }
}

impl FiveAxisGCodeProcessor {
    pub fn new(orientation_controller: ToolOrientationController) -> Self {
        let current_position = Pose6DOF::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let modal_state = ModalState::default();
        let coordinate_systems = HashMap::new();
        let tool_library = HashMap::new();
        
        Self {
            orientation_controller,
            current_position,
            modal_state,
            coordinate_systems,
            tool_library,
        }
    }
    
    pub fn process_gcode_line(&mut self, line: &str) -> Result<Option<MotionSegment>, KinematicsError> {
        let command = self.parse_gcode_line(line)?;

        // G0/G1/G2/G3 are modal: `X10` on its own line continues the last one
        self.modal_state.motion_mode = command.motion_type.clone();

        match command.motion_type {
            MotionMode::Linear | MotionMode::Rapid => {
                if let Some(target_pose) = command.target_pose {
                    let orientation_mode = command.orientation_mode.unwrap_or(ToolOrientationMode::Interpolated);
                    let feedrate = command.feedrate.unwrap_or(self.modal_state.feedrate);
                    
                    // Update modal state
                    if let Some(f) = command.feedrate {
                        self.modal_state.feedrate = f;
                    }
                    
                    // Plan motion segment
                    let planner = crate::motion_planning::FiveAxisMotionPlanner::new(self.orientation_controller.kinematics.clone());
                    let segment = planner.plan_motion(&self.current_position, &target_pose, feedrate, orientation_mode)?;
                    
                    self.current_position = target_pose;
                    return Ok(Some(segment));
                }
            },
            
            MotionMode::CircularCW | MotionMode::CircularCCW => {
                // Implement circular interpolation with tool orientation control
                // This would require arc generation with continuous tool orientation
            },
            
            _ => {
                // Handle other motion modes
            },
        }
        
        Ok(None)
    }
    
    fn parse_gcode_line(&self, line: &str) -> Result<FiveAxisCommand, KinematicsError> {
        // Comments must not be read as words: `G1 X10 ; Z5` is not a Z move
        let cleaned_line = line.split(';').next().unwrap_or("").trim().to_uppercase();
        
        // Initialize command with current modal state
        let mut command = FiveAxisCommand {
            gcode: line.to_string(),
            motion_type: self.modal_state.motion_mode.clone(),
            target_pose: None,
            feedrate: None,
            spindle_speed: None,
            tool_change: None,
            orientation_mode: None,
            additional_axes: HashMap::new(),
        };
        
        // Parse individual words
        let words: Vec<&str> = cleaned_line.split_whitespace().collect();
        let mut x = None;
        let mut y = None;
        let mut z = None;
        let mut a = None;
        let mut b = None;
        let mut c = None;
        
        for word in words {
            if word.is_empty() {
                continue;
            }
            
            // Not byte 1: the first character may be more than one byte long
            let letter = word.chars().next().unwrap();
            let value_str = &word[letter.len_utf8()..];
            
            match letter {
                'G' => {
                    if let Ok(gcode) = value_str.parse::<u32>() {
                        command.motion_type = match gcode {
                            0 => MotionMode::Rapid,
                            1 => MotionMode::Linear,
                            2 => MotionMode::CircularCW,
                            3 => MotionMode::CircularCCW,
                            4 => MotionMode::Dwell,
                            _ => command.motion_type,
                        };
                    }
                },
                'X' => x = value_str.parse().ok(),
                'Y' => y = value_str.parse().ok(),
                'Z' => z = value_str.parse().ok(),
                'A' => a = value_str.parse().ok(),
                'B' => b = value_str.parse().ok(),
                'C' => c = value_str.parse().ok(),
                'F' => command.feedrate = value_str.parse().ok(),
                'S' => command.spindle_speed = value_str.parse().ok(),
                'T' => command.tool_change = value_str.parse().ok(),
                'U' | 'V' | 'W' => {
                    if let Ok(value) = value_str.parse::<f64>() {
                        command.additional_axes.insert(letter.to_string(), value);
                    }
                },
                _ => {}, // Ignore unrecognized letters
            }
        }
        
        // Build target pose if position coordinates are specified
        if x.is_some() || y.is_some() || z.is_some() || a.is_some() || b.is_some() || c.is_some() {
            let target_x = x.unwrap_or(self.current_position.position.x);
            let target_y = y.unwrap_or(self.current_position.position.y);
            let target_z = z.unwrap_or(self.current_position.position.z);
            
            let (current_roll, current_pitch, current_yaw) = self.current_position.euler_angles();
            let target_roll = a.map(|deg: f64| deg.to_radians()).unwrap_or(current_roll);
            let target_pitch = b.map(|deg: f64| deg.to_radians()).unwrap_or(current_pitch);
            let target_yaw = c.map(|deg: f64| deg.to_radians()).unwrap_or(current_yaw);
            
            command.target_pose = Some(Pose6DOF::new(
                target_x, target_y, target_z,
                target_roll, target_pitch, target_yaw,
            ));
        }
        
        Ok(command)
    }
}

#[cfg(test)]
mod gcode_tests {
    use super::*;
    use crate::five_axis::FiveAxisConfiguration;

    fn processor() -> FiveAxisGCodeProcessor {
        let kinematics = FiveAxisKinematics::new(FiveAxisConfiguration::RotaryTable {
            table_offset: Vector3::zeros(),
            a_axis_position: Vector3::zeros(),
            b_axis_position: Vector3::zeros(),
            spindle_offset: Vector3::zeros(),
            max_a_angle: 90.0,
            max_b_angle: 180.0,
            table_diameter: 300.0,
        });
        FiveAxisGCodeProcessor::new(ToolOrientationController::new(kinematics))
    }

    #[test]
    fn comments_are_not_commands_and_do_not_crash_it() {
        let mut p = processor();
        let segment = p.process_gcode_line("G1 X10 Y10 Z10 F600 ; Z99 at 5°C °°").unwrap().unwrap();
        assert_eq!(segment.end_pose.position.z, 10.0);
        // A non-ASCII first character in a word must not panic
        assert!(p.process_gcode_line("°").is_ok());
    }

    #[test]
    fn motion_mode_is_modal() {
        let mut p = processor();
        p.process_gcode_line("G1 X10 Y10 Z10 F600").unwrap();
        // No G word: continues as G1
        assert!(p.process_gcode_line("X20").unwrap().is_some());
    }
}

impl ToolDefinition {
    pub fn default_extruder() -> Self {
        Self {
            tool_id: 1,
            tool_type: ToolType::Extruder {
                nozzle_diameter: 0.4,
                flow_rate: 1.0,
                temperature: 210.0,
            },
            length: 50.0,
            diameter: 0.4,
            tip_radius: 0.0,
            cutting_edge_angle: 0.0,
            flute_count: 1,
            recommended_speeds: SpeedRange {
                min_spindle_rpm: 0.0,
                max_spindle_rpm: 0.0,
                min_feedrate: 300.0,
                max_feedrate: 6000.0,
            },
            material_properties: ToolMaterial {
                material: "Hardened Steel".to_string(),
                hardness: 60.0,
                thermal_conductivity: 50.0,
                wear_resistance: 0.8,
            },
        }
    }
    
    pub fn carbide_end_mill(diameter: f64, length: f64) -> Self {
        Self {
            tool_id: 2,
            tool_type: ToolType::EndMill {
                flute_count: 4,
                helix_angle: 30.0,
                coating: "TiAlN".to_string(),
            },
            length,
            diameter,
            tip_radius: 0.0,
            cutting_edge_angle: 0.0,
            flute_count: 4,
            recommended_speeds: SpeedRange {
                min_spindle_rpm: 5000.0,
                max_spindle_rpm: 20000.0,
                min_feedrate: 500.0,
                max_feedrate: 3000.0,
            },
            material_properties: ToolMaterial {
                material: "Carbide".to_string(),
                hardness: 85.0,
                thermal_conductivity: 100.0,
                wear_resistance: 0.95,
            },
        }
    }
}

impl WorkpieceCS {
    pub fn default() -> Self {
        Self {
            origin: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            name: "G54".to_string(),
            setup_description: "Default coordinate system".to_string(),
        }
    }
}

impl Default for ModalState {
    fn default() -> Self {
        Self {
            motion_mode: MotionMode::Rapid,
            coordinate_system: "G54".to_string(),
            plane_selection: PlaneSelection::XY,
            feedrate: 100.0,
            spindle_speed: 0.0,
            tool_number: 1,
            coolant_state: CoolantState::Off,
            units: Units::Millimeters,
            tool_length_compensation: false,
            tool_radius_compensation: CompensationMode::Off,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::five_axis::FiveAxisConfiguration;
    
    #[test]
    fn test_tool_orientation_controller() {
        let config = FiveAxisConfiguration::RotaryTable {
            table_offset: nalgebra::Vector3::zeros(),
            a_axis_position: nalgebra::Vector3::zeros(),
            b_axis_position: nalgebra::Vector3::zeros(),
            spindle_offset: nalgebra::Vector3::zeros(),
            max_a_angle: 90.0,
            max_b_angle: 180.0,
            table_diameter: 300.0,
        };
        
        let kinematics = FiveAxisKinematics::new(config);
        let controller = ToolOrientationController::new(kinematics);
        
        let position = Point3::new(100.0, 100.0, 200.0);
        let strategy = OrientationStrategy::Constant {
            orientation: UnitQuaternion::identity(),
        };
        
        let orientation = controller.calculate_tool_orientation(&position, &strategy);
        assert!(orientation.is_ok());
    }
    
    #[test]
    fn test_gcode_parsing() {
        let config = FiveAxisConfiguration::RotaryTable {
            table_offset: nalgebra::Vector3::zeros(),
            a_axis_position: nalgebra::Vector3::zeros(),
            b_axis_position: nalgebra::Vector3::zeros(),
            spindle_offset: nalgebra::Vector3::zeros(),
            max_a_angle: 90.0,
            max_b_angle: 180.0,
            table_diameter: 300.0,
        };
        
        let kinematics = FiveAxisKinematics::new(config);
        let controller = ToolOrientationController::new(kinematics);
        let mut processor = FiveAxisGCodeProcessor::new(controller);
        
        let result = processor.parse_gcode_line("G1 X100 Y50 Z200 A45 B90 F3000");
        assert!(result.is_ok());
        
        let command = result.unwrap();
        assert!(matches!(command.motion_type, MotionMode::Linear));
        assert!(command.target_pose.is_some());
        assert_eq!(command.feedrate, Some(3000.0));
    }
}