/// Practical 5-axis machine configurations based on real-world implementations
/// This module provides simplified, proven kinematic models for common 5-axis setups
/// Referenced from successful projects like Penta Axis (PAX), Open5X, and RepRapFirmware

use crate::five_axis::*;
use nalgebra::{Vector3, Point3, UnitQuaternion, Matrix4, Rotation3, Translation3, Isometry3};
use serde::{Deserialize, Serialize};

/// Common 5-axis machine types with proven kinematics
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PracticalMachineType {
    /// Toolhead-based: Rotary axes on the extruder/head (popular for non-planar printing)
    /// Examples: Penta Axis, Open5X, modified Prusa/Voron
    ToolheadRotary {
        /// Distance from rotary pivot to tool tip (typically 45mm)
        tool_offset: f64,
        /// Tilt axis limits (A-axis around X)
        tilt_limits: (f64, f64),
        /// Rotation axis limits (B/C-axis around Z)  
        rotation_limits: (f64, f64),
        /// Gear ratios for rotary axes
        gear_ratios: (f64, f64),
        /// Steps per degree for rotary motors
        steps_per_degree: (f64, f64),
    },
    
    /// Table-based: Rotary axes on the build plate (trunnion table)
    /// Examples: Traditional CNC 5-axis, some large format printers
    TableRotary {
        /// Table center offset from machine origin
        table_center: Vector3<f64>,
        /// Table diameter for collision checking
        table_diameter: f64,
        /// A-axis (tilt) configuration
        a_axis_config: AxisConfig,
        /// C-axis (rotation) configuration
        c_axis_config: AxisConfig,
    },
    
    /// Hybrid: CoreXY linear + rotary orientation (CoreXYAC)
    /// Examples: Modified Voron with rotary extruder
    CoreXYRotary {
        /// CoreXY belt configuration
        belt_ratio: f64,
        /// Rotary axes on toolhead
        rotary_offset: Vector3<f64>,
        /// A and C axis configurations
        axis_configs: (AxisConfig, AxisConfig),
    },
    
    /// 5-axis robot arm (experimental for pick-and-place or multi-tool)
    RobotArm {
        /// Link lengths for each segment
        link_lengths: Vec<f64>,
        /// Joint limits for each axis
        joint_limits: Vec<(f64, f64)>,
        /// End effector offset
        end_effector_offset: f64,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AxisConfig {
    pub min_angle: f64,         // degrees
    pub max_angle: f64,         // degrees
    pub steps_per_degree: f64,  // motor steps per degree
    pub gear_ratio: f64,        // reduction ratio
    pub homing_speed: f64,      // degrees/minute
    pub max_speed: f64,         // degrees/minute
    pub acceleration: f64,      // degrees/second²
}

/// Optimized kinematics solver for common configurations
pub struct PracticalKinematics {
    machine_type: PracticalMachineType,
    /// Cached transformation matrices for performance
    base_transform: Matrix4<f64>,
    tool_transform: Matrix4<f64>,
    /// Optimization flags
    use_lookup_tables: bool,
    use_approximations: bool,
}

impl PracticalKinematics {
    pub fn new(machine_type: PracticalMachineType) -> Self {
        let base_transform = Matrix4::identity();
        let tool_transform = Matrix4::identity();
        
        Self {
            machine_type,
            base_transform,
            tool_transform,
            use_lookup_tables: true,
            use_approximations: false,
        }
    }
    
    /// Fast forward kinematics using optimized methods for each configuration
    pub fn forward_kinematics_fast(&self, joints: &JointConfiguration) -> Result<Pose6DOF, KinematicsError> {
        match &self.machine_type {
            PracticalMachineType::ToolheadRotary { tool_offset, .. } => {
                self.forward_toolhead_rotary(joints, *tool_offset)
            },
            PracticalMachineType::TableRotary { table_center, .. } => {
                self.forward_table_rotary(joints, table_center)
            },
            PracticalMachineType::CoreXYRotary { belt_ratio, rotary_offset, .. } => {
                self.forward_corexy_rotary(joints, *belt_ratio, rotary_offset)
            },
            PracticalMachineType::RobotArm { link_lengths, .. } => {
                self.forward_robot_arm(joints, link_lengths)
            },
        }
    }
    
    /// Simplified inverse kinematics using analytical methods where possible
    pub fn inverse_kinematics_analytical(&self, target_pose: &Pose6DOF) -> Result<Vec<JointConfiguration>, KinematicsError> {
        match &self.machine_type {
            PracticalMachineType::ToolheadRotary { tool_offset, tilt_limits, rotation_limits, .. } => {
                self.inverse_toolhead_rotary(target_pose, *tool_offset, *tilt_limits, *rotation_limits)
            },
            PracticalMachineType::TableRotary { table_center, .. } => {
                self.inverse_table_rotary(target_pose, table_center)
            },
            PracticalMachineType::CoreXYRotary { belt_ratio, rotary_offset, .. } => {
                self.inverse_corexy_rotary(target_pose, *belt_ratio, rotary_offset)
            },
            PracticalMachineType::RobotArm { link_lengths, joint_limits, .. } => {
                self.inverse_robot_arm(target_pose, link_lengths, joint_limits)
            },
        }
    }
    
    /// A and B/C (degrees) that aim a head rotating as Rz(b) * Rx(a) along the pose's tool
    /// direction, which works out as (sin a sin b, -sin a cos b, cos a). Euler angles would
    /// not do: this order has no "pitch", and using it would lose the tilt.
    fn head_angles_from_direction(pose: &Pose6DOF) -> (f64, f64) {
        let d = pose.tool_direction();
        let tilt = d.z.clamp(-1.0, 1.0).acos();
        let spin = if tilt < 1e-9 { 0.0 } else { d.x.atan2(-d.y) };
        (tilt.to_degrees(), spin.to_degrees())
    }

    /// A and C (degrees) for a table rotating as Rx(a) * Rz(c). The tool's direction relative
    /// to the table is then (sin a sin c, sin a cos c, cos a).
    fn table_angles_from_direction(pose: &Pose6DOF) -> (f64, f64) {
        let d = pose.tool_direction();
        let tilt = d.z.clamp(-1.0, 1.0).acos();
        let spin = if tilt < 1e-9 { 0.0 } else { d.x.atan2(d.y) };
        (tilt.to_degrees(), spin.to_degrees())
    }

    // Toolhead rotary kinematics (most common for 3D printing)
    fn forward_toolhead_rotary(&self, joints: &JointConfiguration, tool_offset: f64) -> Result<Pose6DOF, KinematicsError> {
        // Linear position (X, Y, Z are direct)
        let linear_pos = Vector3::new(joints.x, joints.y, joints.z);
        
        // Rotary transformations
        let a_rad = joints.a.to_radians();
        let b_rad = joints.b_or_c.to_radians();
        
        // Tool offset accounting for rotations
        // This is the "handshake" method mentioned by Grok - visually intuitive
        // The tool tip hangs below the pivot
        let tool_offset_vector = Vector3::new(0.0, 0.0, -tool_offset);
        
        // Apply A-axis rotation (tilt around X)
        let a_rotation = Rotation3::from_axis_angle(&Vector3::x_axis(), a_rad);
        let rotated_offset = a_rotation * tool_offset_vector;
        
        // Apply B-axis rotation (rotate around Z)
        let b_rotation = Rotation3::from_axis_angle(&Vector3::z_axis(), b_rad);
        let final_offset = b_rotation * rotated_offset;
        
        // Final tool position
        let tool_position = Point3::from(linear_pos + final_offset);
        
        // Tool orientation (composition of rotations)
        let combined_rotation = b_rotation * a_rotation;
        let orientation = UnitQuaternion::from_rotation_matrix(&combined_rotation);
        
        Ok(Pose6DOF {
            position: tool_position,
            orientation,
        })
    }
    
    fn inverse_toolhead_rotary(&self, target_pose: &Pose6DOF, tool_offset: f64, tilt_limits: (f64, f64), rotation_limits: (f64, f64)) -> Result<Vec<JointConfiguration>, KinematicsError> {
        // Forward kinematics is Rz(b) * Rx(a), so A tilts and B spins
        let (a_deg, b_deg) = Self::head_angles_from_direction(target_pose);

        // Check limits
        if a_deg < tilt_limits.0 || a_deg > tilt_limits.1 {
            return Err(KinematicsError::JointLimitsExceeded);
        }
        if b_deg < rotation_limits.0 || b_deg > rotation_limits.1 {
            return Err(KinematicsError::JointLimitsExceeded);
        }
        
        // Calculate required linear position by backing out tool offset
        let a_rad = a_deg.to_radians();
        let b_rad = b_deg.to_radians();
        
        let tool_offset_vector = Vector3::new(0.0, 0.0, -tool_offset);
        let a_rotation = Rotation3::from_axis_angle(&Vector3::x_axis(), a_rad);
        let b_rotation = Rotation3::from_axis_angle(&Vector3::z_axis(), b_rad);
        let offset_after_rotation = b_rotation * (a_rotation * tool_offset_vector);
        
        let linear_position = target_pose.position.coords - offset_after_rotation;
        
        let solution = JointConfiguration::new(
            linear_position.x,
            linear_position.y,
            linear_position.z,
            a_deg,
            b_deg,
        );
        
        Ok(vec![solution])
    }
    
    fn forward_table_rotary(&self, joints: &JointConfiguration, table_center: &Vector3<f64>) -> Result<Pose6DOF, KinematicsError> {
        // For table rotary, the workpiece rotates while tool stays fixed
        // More complex because workpiece coordinate system changes
        
        let spindle_pos = Vector3::new(joints.x, joints.y, joints.z);
        
        // Table rotations affect workpiece, not tool directly
        let a_rad = joints.a.to_radians();
        let c_rad = joints.b_or_c.to_radians();
        
        // Transform from workpiece coordinates to machine coordinates
        let a_rotation = Rotation3::from_axis_angle(&Vector3::x_axis(), a_rad);
        let c_rotation = Rotation3::from_axis_angle(&Vector3::z_axis(), c_rad);
        
        // The tool is at the spindle position. Its orientation relative to the part is the
        // inverse of the table's rotation. (Following the part with X/Y/Z, which is RTCP and
        // what table_center would be for, is not done here.)
        let _ = table_center;
        let table_rotation = a_rotation * c_rotation;
        let tool_pose = Pose6DOF {
            position: Point3::from(spindle_pos),
            orientation: UnitQuaternion::from_rotation_matrix(&table_rotation.inverse()),
        };
        
        Ok(tool_pose)
    }
    
    fn inverse_table_rotary(&self, target_pose: &Pose6DOF, _table_center: &Vector3<f64>) -> Result<Vec<JointConfiguration>, KinematicsError> {
        // For table rotary, inverse is about positioning the workpiece
        // Tool position directly gives X, Y, Z
        let linear_pos = target_pose.position.coords;
        
        // Orientation determines table angles
        let (a_deg, c_deg) = Self::table_angles_from_direction(target_pose);

        let solution = JointConfiguration::new(
            linear_pos.x,
            linear_pos.y,
            linear_pos.z,
            a_deg,
            c_deg,
        );
        
        Ok(vec![solution])
    }
    
    fn forward_corexy_rotary(&self, joints: &JointConfiguration, belt_ratio: f64, rotary_offset: &Vector3<f64>) -> Result<Pose6DOF, KinematicsError> {
        // CoreXY transformation for X, Y. The inverse of motor_a = (x + y) * ratio and
        // motor_b = (x - y) * ratio, which needs the factor of 2.
        let corexy_x = (joints.x + joints.y) / (2.0 * belt_ratio);
        let corexy_y = (joints.x - joints.y) / (2.0 * belt_ratio);
        
        // Z is direct
        let linear_pos = Vector3::new(corexy_x, corexy_y, joints.z);
        
        // Apply rotary transformations similar to toolhead rotary
        let a_rad = joints.a.to_radians();
        let c_rad = joints.b_or_c.to_radians();
        
        let a_rotation = Rotation3::from_axis_angle(&Vector3::x_axis(), a_rad);
        let c_rotation = Rotation3::from_axis_angle(&Vector3::z_axis(), c_rad);
        
        let rotated_offset = c_rotation * (a_rotation * rotary_offset);
        let tool_position = Point3::from(linear_pos + rotated_offset);
        
        let combined_rotation = c_rotation * a_rotation;
        let orientation = UnitQuaternion::from_rotation_matrix(&combined_rotation);
        
        Ok(Pose6DOF {
            position: tool_position,
            orientation,
        })
    }
    
    fn inverse_corexy_rotary(&self, target_pose: &Pose6DOF, belt_ratio: f64, rotary_offset: &Vector3<f64>) -> Result<Vec<JointConfiguration>, KinematicsError> {
        // Extract orientation
        let (a_deg, c_deg) = Self::head_angles_from_direction(target_pose);

        // Back out rotary offset
        let a_rad = a_deg.to_radians();
        let c_rad = c_deg.to_radians();
        
        let a_rotation = Rotation3::from_axis_angle(&Vector3::x_axis(), a_rad);
        let c_rotation = Rotation3::from_axis_angle(&Vector3::z_axis(), c_rad);
        let rotated_offset = c_rotation * (a_rotation * rotary_offset);
        
        let linear_position = target_pose.position.coords - rotated_offset;
        
        // CoreXY inverse transformation
        let motor_a = (linear_position.x + linear_position.y) * belt_ratio;
        let motor_b = (linear_position.x - linear_position.y) * belt_ratio;
        
        let solution = JointConfiguration::new(
            motor_a,
            motor_b,
            linear_position.z,
            a_deg,
            c_deg,
        );
        
        Ok(vec![solution])
    }
    
    fn forward_robot_arm(&self, joints: &JointConfiguration, link_lengths: &[f64]) -> Result<Pose6DOF, KinematicsError> {
        // Standard DH parameter forward kinematics for robot arm
        let joint_angles = [joints.x, joints.y, joints.z, joints.a, joints.b_or_c];
        
        let mut transform = Matrix4::identity();
        
        for (i, &length) in link_lengths.iter().enumerate() {
            if i >= joint_angles.len() {
                break;
            }
            
            let angle = joint_angles[i].to_radians();
            
            // Simplified DH transformation (assuming standard configuration)
            let cos_angle = angle.cos();
            let sin_angle = angle.sin();
            
            let link_transform = Matrix4::new(
                cos_angle, -sin_angle, 0.0, length * cos_angle,
                sin_angle,  cos_angle, 0.0, length * sin_angle,
                0.0,        0.0,       1.0, 0.0,
                0.0,        0.0,       0.0, 1.0,
            );
            
            transform = transform * link_transform;
        }
        
        // Extract rotation and translation from transform matrix
        let rotation = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(transform.fixed_view::<3, 3>(0, 0).into()));
        let translation = Translation3::from(Vector3::new(transform[(0, 3)], transform[(1, 3)], transform[(2, 3)]));
        let isometry = Isometry3::from_parts(translation, rotation);
        Ok(Pose6DOF::from_transform(&isometry))
    }
    
    fn inverse_robot_arm(&self, target_pose: &Pose6DOF, link_lengths: &[f64], joint_limits: &[(f64, f64)]) -> Result<Vec<JointConfiguration>, KinematicsError> {
        // For robot arm, use numerical inverse kinematics
        // This is a simplified placeholder - would use Jacobian method in practice
        
        // Start with a reasonable initial guess
        let mut current_joints = JointConfiguration::new(0.0, 0.0, 0.0, 0.0, 0.0);
        
        const MAX_ITERATIONS: usize = 50;
        const TOLERANCE: f64 = 0.001;
        
        for _iteration in 0..MAX_ITERATIONS {
            let current_pose = self.forward_robot_arm(&current_joints, link_lengths)?;
            
            let position_error = (target_pose.position - current_pose.position).norm();
            if position_error < TOLERANCE {
                return Ok(vec![current_joints]);
            }
            
            // Simple gradient descent (would use proper Jacobian in practice)
            let error_vector = target_pose.position - current_pose.position;
            let step_size = 0.01;
            
            current_joints.x += error_vector.x * step_size;
            current_joints.y += error_vector.y * step_size;
            current_joints.z += error_vector.z * step_size;
            
            // Check joint limits
            for (i, &(min_val, max_val)) in joint_limits.iter().enumerate() {
                let joint_value = match i {
                    0 => &mut current_joints.x,
                    1 => &mut current_joints.y,
                    2 => &mut current_joints.z,
                    3 => &mut current_joints.a,
                    4 => &mut current_joints.b_or_c,
                    _ => continue,
                };
                
                *joint_value = joint_value.max(min_val).min(max_val);
            }
        }
        
        Err(KinematicsError::SolutionNotFound)
    }
    
    /// Calculate steps per degree for rotary axes based on hardware configuration
    pub fn calculate_steps_per_degree(full_steps_per_rev: u32, gear_ratio: f64, microstepping: u32) -> f64 {
        // Formula from Grok: full steps per revolution × gear ratio × microstepping / 360
        (full_steps_per_rev as f64) * gear_ratio * (microstepping as f64) / 360.0
    }
    
    /// Move Z up relative to wherever it is, for clearance before anything is homed
    fn raise_z_commands() -> Vec<String> {
        vec!["G91".to_string(), "G1 Z10 F600".to_string(), "G90".to_string()]
    }

    /// Generate homing sequence that avoids collisions
    pub fn generate_safe_homing_sequence(&self) -> Vec<String> {
        let mut sequence = Vec::new();
        
        match &self.machine_type {
            PracticalMachineType::ToolheadRotary { .. } => {
                // Raise first: a tilted head can swing into the bed while its rotaries home
                sequence.extend(Self::raise_z_commands());
                sequence.push("G28 A0".to_string());      // Tilt to neutral
                sequence.push("G28 B0".to_string());      // Rotation to neutral
                sequence.push("G28 X Y".to_string());     // X, Y, with the head upright
                sequence.push("G28 Z".to_string());       // Z last, now over a known spot
            },
            
            PracticalMachineType::TableRotary { .. } => {
                // Raise first, as the table needs room to turn. G28 Z itself may go down.
                sequence.extend(Self::raise_z_commands());
                sequence.push("G28 A0".to_string());      // Table tilt to neutral
                sequence.push("G28 C0".to_string());      // Table rotation to neutral
                sequence.push("G28 X Y".to_string());     // Linear axes
                sequence.push("G28 Z".to_string());       // Z last, with the table level
            },
            
            PracticalMachineType::CoreXYRotary { .. } => {
                // Similar to toolhead rotary
                sequence.extend(Self::raise_z_commands());
                sequence.push("G28 A0".to_string());
                sequence.push("G28 C0".to_string());
                sequence.push("G28 X Y".to_string());
                sequence.push("G28 Z".to_string());
            },
            
            PracticalMachineType::RobotArm { .. } => {
                // Robot arm should home in sequence from base to end effector
                sequence.push("G28 X".to_string());       // Base rotation
                sequence.push("G28 Y".to_string());       // Shoulder
                sequence.push("G28 Z".to_string());       // Elbow
                sequence.push("G28 A".to_string());       // Wrist tilt
                sequence.push("G28 B".to_string());       // Wrist rotation
            },
        }
        
        sequence
    }
}

/// Factory functions for common machine configurations
impl PracticalMachineType {
    /// Create configuration for Penta Axis style printer
    pub fn penta_axis_style() -> Self {
        Self::ToolheadRotary {
            tool_offset: 45.0,               // 45mm typical offset
            tilt_limits: (-45.0, 45.0),      // ±45° tilt range
            rotation_limits: (-180.0, 180.0), // Full rotation
            gear_ratios: (50.0, 30.0),       // Worm gear reductions
            steps_per_degree: (11.11, 6.67), // Calculated for NEMA17 + gearing
        }
    }
    
    /// Create configuration for Open5X style printer
    pub fn open5x_style() -> Self {
        Self::CoreXYRotary {
            belt_ratio: 1.0,                 // Standard CoreXY
            rotary_offset: Vector3::new(0.0, 0.0, 50.0), // Tool offset
            axis_configs: (
                AxisConfig {
                    min_angle: -30.0,
                    max_angle: 30.0,
                    steps_per_degree: 8.89,  // NEMA17 with 40:1 reduction
                    gear_ratio: 40.0,
                    homing_speed: 10.0,
                    max_speed: 600.0,
                    acceleration: 100.0,
                },
                AxisConfig {
                    min_angle: -180.0,
                    max_angle: 180.0,
                    steps_per_degree: 4.44,  // NEMA17 with 20:1 reduction
                    gear_ratio: 20.0,
                    homing_speed: 30.0,
                    max_speed: 1800.0,
                    acceleration: 300.0,
                },
            ),
        }
    }
    
    /// Create configuration for CNC trunnion table
    pub fn cnc_trunnion_table() -> Self {
        Self::TableRotary {
            table_center: Vector3::new(0.0, 0.0, 100.0),
            table_diameter: 300.0,
            a_axis_config: AxisConfig {
                min_angle: -110.0,
                max_angle: 110.0,
                steps_per_degree: 22.22,     // High precision worm gear
                gear_ratio: 80.0,
                homing_speed: 5.0,
                max_speed: 300.0,
                acceleration: 50.0,
            },
            c_axis_config: AxisConfig {
                min_angle: -360.0,
                max_angle: 360.0,
                steps_per_degree: 11.11,
                gear_ratio: 40.0,
                homing_speed: 10.0,
                max_speed: 600.0,
                acceleration: 100.0,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_penta_axis_kinematics() {
        let machine = PracticalMachineType::penta_axis_style();
        let kinematics = PracticalKinematics::new(machine);
        
        let joints = JointConfiguration::new(100.0, 100.0, 200.0, 30.0, 45.0);
        let pose = kinematics.forward_kinematics_fast(&joints);
        
        assert!(pose.is_ok());
        let p = pose.unwrap();
        assert!(p.position.x > 0.0);
        assert!(p.position.y > 0.0);
        assert!(p.position.z > 0.0);
    }
    
    #[test]
    fn test_steps_per_degree_calculation() {
        // NEMA17 with 50:1 worm gear, 16x microstepping
        let steps = PracticalKinematics::calculate_steps_per_degree(200, 50.0, 16);
        assert!((steps - 444.44).abs() < 0.1);
    }
    
    #[test]
    fn test_inverse_kinematics_accuracy() {
        let machine = PracticalMachineType::penta_axis_style();
        let kinematics = PracticalKinematics::new(machine);
        
        let target_pose = Pose6DOF::new(150.0, 100.0, 250.0, 0.2, 0.3, 0.1);
        let solutions = kinematics.inverse_kinematics_analytical(&target_pose);
        
        assert!(solutions.is_ok());
        let sols = solutions.unwrap();
        assert!(!sols.is_empty());
        
        // Verify solution by forward kinematics
        let computed_pose = kinematics.forward_kinematics_fast(&sols[0]);
        assert!(computed_pose.is_ok());
        let computed_pose = computed_pose.unwrap();

        let error = (computed_pose.position - target_pose.position).norm();
        assert!(error < 0.1); // Less than 0.1mm error

        // And the tool must point the right way
        let direction_error = (computed_pose.tool_direction().into_inner() - target_pose.tool_direction().into_inner()).norm();
        assert!(direction_error < 1e-6, "tool direction off by {}", direction_error);
    }

    #[test]
    fn test_corexy_round_trip() {
        let axis = AxisConfig { min_angle: -90.0, max_angle: 90.0, steps_per_degree: 10.0, gear_ratio: 1.0, homing_speed: 100.0, max_speed: 1000.0, acceleration: 100.0 };
        let machine = PracticalMachineType::CoreXYRotary {
            belt_ratio: 1.0,
            rotary_offset: nalgebra::Vector3::new(0.0, 0.0, -30.0),
            axis_configs: (axis.clone(), axis),
        };
        let kinematics = PracticalKinematics::new(machine);
        let target = Pose6DOF::new(50.0, 20.0, 100.0, 0.3, 0.0, 0.0);
        let joints = kinematics.inverse_kinematics_analytical(&target).unwrap();
        let computed = kinematics.forward_kinematics_fast(&joints[0]).unwrap();
        assert!((computed.position - target.position).norm() < 1e-9, "{:?}", computed.position);
    }

    #[test]
    fn test_homing_raises_z_before_anything_swings() {
        let kinematics = PracticalKinematics::new(PracticalMachineType::penta_axis_style());
        let sequence = kinematics.generate_safe_homing_sequence();
        let position = |text: &str| sequence.iter().position(|c| c.contains(text)).unwrap();
        assert!(position("G1 Z10") < position("G28 A0"));
        assert!(position("G28 X Y") < position("G28 Z"));
    }
}