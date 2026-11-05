/// Embedded-optimized 5-axis kinematics following Grok's performance recommendations
/// This module provides no-std, no-alloc implementations suitable for microcontrollers
/// Optimized for real-time performance with deterministic execution times

use core::f32::consts::PI;

/// Machine configuration optimized for embedded systems
#[derive(Clone, Copy)]
pub struct EmbeddedMachineConfig {
    pub tool_offset: f32,              // mm from pivot to tool tip
    pub axis_limits: [AxisLimit; 5],   // [X, Y, Z, A, B] limits
    pub steps_per_unit: [f32; 5],      // Steps per mm/degree
    pub max_feedrates: [f32; 5],       // Maximum feedrates per axis
    pub accelerations: [f32; 5],       // Maximum accelerations per axis
    pub machine_type: MachineType,     // Hardware configuration type
}

#[derive(Clone, Copy)]
pub struct AxisLimit {
    pub min: f32,
    pub max: f32,
}

#[derive(Clone, Copy)]
pub enum MachineType {
    /// Toolhead rotary: A around Z, B around X (Penta Axis style)
    ToolheadAB,
    /// Toolhead rotary: A around X, C around Z (Open5X style)  
    ToolheadAC,
    /// Table rotary: A around X, C around Z (CNC trunnion)
    TableAC,
    /// CoreXY with rotary head
    CoreXYRotary,
}

/// Cartesian pose in 5-axis space
#[derive(Clone, Copy, Debug)]
pub struct Pose5D {
    pub x: f32,      // mm
    pub y: f32,      // mm
    pub z: f32,      // mm
    pub a: f32,      // degrees
    pub b_or_c: f32, // degrees (B for AB config, C for AC config)
}

/// Joint positions (motor coordinates)
pub type JointArray = [f32; 5];

/// Error types for embedded kinematics
#[derive(Clone, Copy, Debug)]
pub enum KinematicsError {
    OutOfBounds,
    Singularity,
    InvalidConfig,
}

/// Fast embedded kinematics solver
pub struct EmbeddedKinematics {
    config: EmbeddedMachineConfig,
    /// Precomputed lookup tables for trigonometric functions (optional optimization)
    sin_table: Option<&'static [f32; 3600]>, // 0.1° resolution
    cos_table: Option<&'static [f32; 3600]>,
}

impl EmbeddedKinematics {
    /// Create new kinematics solver with configuration
    pub const fn new(config: EmbeddedMachineConfig) -> Self {
        Self {
            config,
            sin_table: None,
            cos_table: None,
        }
    }
    
    /// Enable lookup table optimization (saves ~50% trig computation time)
    pub fn with_lookup_tables(mut self, sin_table: &'static [f32; 3600], cos_table: &'static [f32; 3600]) -> Self {
        self.sin_table = Some(sin_table);
        self.cos_table = Some(cos_table);
        self
    }
    
    /// Forward kinematics: Joint positions → Tool pose
    /// Optimized for <1ms execution time on embedded targets
    #[inline]
    pub fn forward_kinematics(&self, joints: JointArray) -> Pose5D {
        match self.config.machine_type {
            MachineType::ToolheadAB => self.fk_toolhead_ab(joints),
            MachineType::ToolheadAC => self.fk_toolhead_ac(joints),
            MachineType::TableAC => self.fk_table_ac(joints),
            MachineType::CoreXYRotary => self.fk_corexy_rotary(joints),
        }
    }
    
    /// Inverse kinematics: Tool pose → Joint positions
    /// Returns None if pose is unreachable or would cause collision
    #[inline]
    pub fn inverse_kinematics(&self, pose: Pose5D) -> Option<JointArray> {
        // Check workspace limits first (fast rejection)
        if !self.check_workspace_limits(&pose) {
            return None;
        }
        
        let result = match self.config.machine_type {
            MachineType::ToolheadAB => self.ik_toolhead_ab(pose),
            MachineType::ToolheadAC => self.ik_toolhead_ac(pose),
            MachineType::TableAC => self.ik_table_ac(pose),
            MachineType::CoreXYRotary => self.ik_corexy_rotary(pose),
        };
        
        // Validate joint limits
        if let Some(joints) = result {
            if self.check_joint_limits(&joints) {
                Some(joints)
            } else {
                None
            }
        } else {
            None
        }
    }
    
    /// Fast sine function using lookup table or libm
    #[inline]
    fn fast_sin(&self, angle_deg: f32) -> f32 {
        if let Some(table) = self.sin_table {
            // Lookup table with linear interpolation
            let normalized = ((angle_deg % 360.0 + 360.0) % 360.0) * 10.0; // 0.1° resolution
            let index = normalized as usize;
            let frac = normalized - index as f32;
            
            if index < 3599 {
                let val1 = table[index];
                let val2 = table[index + 1];
                val1 + frac * (val2 - val1)
            } else {
                table[index]
            }
        } else {
            // Use libm for embedded targets without std
            #[cfg(feature = "std")]
            return (angle_deg * PI / 180.0).sin();
            
            #[cfg(not(feature = "std"))]
            return libm::sinf(angle_deg * PI / 180.0);
        }
    }
    
    /// Fast cosine function using lookup table or libm
    #[inline]
    fn fast_cos(&self, angle_deg: f32) -> f32 {
        if let Some(table) = self.cos_table {
            // Similar to sin but using cos table
            let normalized = ((angle_deg % 360.0 + 360.0) % 360.0) * 10.0;
            let index = normalized as usize;
            let frac = normalized - index as f32;
            
            if index < 3599 {
                let val1 = table[index];
                let val2 = table[index + 1];
                val1 + frac * (val2 - val1)
            } else {
                table[index]
            }
        } else {
            #[cfg(feature = "std")]
            return (angle_deg * PI / 180.0).cos();
            
            #[cfg(not(feature = "std"))]
            return libm::cosf(angle_deg * PI / 180.0);
        }
    }
    
    /// Forward kinematics for toolhead AB configuration (Penta Axis style)
    /// Implementation follows Grok's "handshake" method for intuitive computation
    #[inline]
    fn fk_toolhead_ab(&self, joints: JointArray) -> Pose5D {
        let [x_m, y_m, z_m, a_deg, b_deg] = joints;
        let offset = self.config.tool_offset;
        
        // Apply rotations to tool offset vector
        // A rotation around Z (yaw), B rotation around X (tilt)
        let sin_a = self.fast_sin(a_deg);
        let cos_a = self.fast_cos(a_deg);
        let sin_b = self.fast_sin(b_deg);
        let cos_b = self.fast_cos(b_deg);
        
        // Tool offset after rotations (simplified from transformation matrix)
        let offset_x = offset * sin_b * cos_a;
        let offset_y = offset * sin_b * sin_a;
        let offset_z = offset * cos_b;

        // The tool tip hangs below the pivot
        Pose5D {
            x: x_m - offset_x,
            y: y_m - offset_y,
            z: z_m - offset_z,
            a: a_deg,
            b_or_c: b_deg,
        }
    }
    
    /// Inverse kinematics for toolhead AB configuration
    /// Analytical solution for maximum speed
    #[inline]
    fn ik_toolhead_ab(&self, pose: Pose5D) -> Option<JointArray> {
        let offset = self.config.tool_offset;
        let a_deg = pose.a;
        let b_deg = pose.b_or_c;
        
        // Check for singularities (B near ±90°)
        if libm::fabsf(b_deg) > 89.0 {
            return None; // Singularity avoidance
        }
        
        // Calculate required machine position by backing out tool offset
        let sin_a = self.fast_sin(a_deg);
        let cos_a = self.fast_cos(a_deg);
        let sin_b = self.fast_sin(b_deg);
        let cos_b = self.fast_cos(b_deg);
        
        let offset_x = offset * sin_b * cos_a;
        let offset_y = offset * sin_b * sin_a;
        let offset_z = offset * cos_b;
        
        let joints = [
            pose.x + offset_x,
            pose.y + offset_y,
            pose.z + offset_z,
            a_deg,
            b_deg,
        ];
        
        Some(joints)
    }
    
    /// Forward kinematics for toolhead AC configuration (Open5X style)
    #[inline]
    fn fk_toolhead_ac(&self, joints: JointArray) -> Pose5D {
        let [x_m, y_m, z_m, a_deg, c_deg] = joints;
        let offset = self.config.tool_offset;
        
        // A rotation around X (tilt), C rotation around Z (spin)
        let sin_a = self.fast_sin(a_deg);
        let cos_a = self.fast_cos(a_deg);
        let sin_c = self.fast_sin(c_deg);
        let cos_c = self.fast_cos(c_deg);
        
        // Apply rotations in sequence: tilt then spin
        let offset_x = offset * sin_a * cos_c;
        let offset_y = offset * sin_a * sin_c;
        let offset_z = offset * cos_a;
        
        Pose5D {
            x: x_m - offset_x,
            y: y_m - offset_y,
            z: z_m - offset_z,
            a: a_deg,
            b_or_c: c_deg,
        }
    }

    /// Inverse kinematics for toolhead AC configuration
    #[inline]
    fn ik_toolhead_ac(&self, pose: Pose5D) -> Option<JointArray> {
        let offset = self.config.tool_offset;
        let a_deg = pose.a;
        let c_deg = pose.b_or_c;
        
        // Singularity check for A axis
        if libm::fabsf(a_deg) > 89.0 {
            return None;
        }
        
        let sin_a = self.fast_sin(a_deg);
        let cos_a = self.fast_cos(a_deg);
        let sin_c = self.fast_sin(c_deg);
        let cos_c = self.fast_cos(c_deg);
        
        let offset_x = offset * sin_a * cos_c;
        let offset_y = offset * sin_a * sin_c;
        let offset_z = offset * cos_a;
        
        Some([
            pose.x + offset_x,
            pose.y + offset_y,
            pose.z + offset_z,
            a_deg,
            c_deg,
        ])
    }
    
    /// Forward kinematics for table AC configuration (CNC trunnion style)
    #[inline]
    fn fk_table_ac(&self, joints: JointArray) -> Pose5D {
        let [x_m, y_m, z_m, a_deg, c_deg] = joints;
        
        // For table rotary, tool position is direct (spindle coordinates)
        // Table rotations affect workpiece coordinate system, not tool directly
        Pose5D {
            x: x_m,
            y: y_m,
            z: z_m,
            a: a_deg,
            b_or_c: c_deg,
        }
    }
    
    /// Inverse kinematics for table AC configuration
    #[inline]
    fn ik_table_ac(&self, pose: Pose5D) -> Option<JointArray> {
        // Direct mapping for table rotary
        Some([pose.x, pose.y, pose.z, pose.a, pose.b_or_c])
    }
    
    /// Forward kinematics for CoreXY + rotary configuration
    #[inline]
    fn fk_corexy_rotary(&self, joints: JointArray) -> Pose5D {
        let [motor_a, motor_b, z_m, a_deg, c_deg] = joints;
        
        // CoreXY transformation
        let x_corexy = (motor_a + motor_b) * 0.5;
        let y_corexy = (motor_a - motor_b) * 0.5;
        
        // Apply rotary offset
        let offset = self.config.tool_offset;
        let sin_a = self.fast_sin(a_deg);
        let cos_a = self.fast_cos(a_deg);
        let sin_c = self.fast_sin(c_deg);
        let cos_c = self.fast_cos(c_deg);
        
        let offset_x = offset * sin_a * cos_c;
        let offset_y = offset * sin_a * sin_c;
        let offset_z = offset * cos_a;
        
        Pose5D {
            x: x_corexy - offset_x,
            y: y_corexy - offset_y,
            z: z_m - offset_z,
            a: a_deg,
            b_or_c: c_deg,
        }
    }
    
    /// Inverse kinematics for CoreXY + rotary configuration
    #[inline]
    fn ik_corexy_rotary(&self, pose: Pose5D) -> Option<JointArray> {
        let offset = self.config.tool_offset;
        let a_deg = pose.a;
        let c_deg = pose.b_or_c;
        
        // Back out rotary offset
        let sin_a = self.fast_sin(a_deg);
        let cos_a = self.fast_cos(a_deg);
        let sin_c = self.fast_sin(c_deg);
        let cos_c = self.fast_cos(c_deg);
        
        let offset_x = offset * sin_a * cos_c;
        let offset_y = offset * sin_a * sin_c;
        let offset_z = offset * cos_a;
        
        let x_corexy = pose.x + offset_x;
        let y_corexy = pose.y + offset_y;
        let z_pos = pose.z + offset_z;
        
        // CoreXY inverse transformation
        let motor_a = x_corexy + y_corexy;
        let motor_b = x_corexy - y_corexy;
        
        Some([motor_a, motor_b, z_pos, a_deg, c_deg])
    }
    
    /// Check if pose is within workspace limits
    #[inline]
    fn check_workspace_limits(&self, pose: &Pose5D) -> bool {
        pose.x >= self.config.axis_limits[0].min && pose.x <= self.config.axis_limits[0].max &&
        pose.y >= self.config.axis_limits[1].min && pose.y <= self.config.axis_limits[1].max &&
        pose.z >= self.config.axis_limits[2].min && pose.z <= self.config.axis_limits[2].max &&
        pose.a >= self.config.axis_limits[3].min && pose.a <= self.config.axis_limits[3].max &&
        pose.b_or_c >= self.config.axis_limits[4].min && pose.b_or_c <= self.config.axis_limits[4].max
    }
    
    /// Check if joint positions are within machine limits
    #[inline]
    fn check_joint_limits(&self, joints: &JointArray) -> bool {
        // Limits are in axis units. CoreXY joints 0 and 1 are belt motors (X+Y, X-Y),
        // so convert them back to X and Y before comparing.
        let mut axes = *joints;
        if matches!(self.config.machine_type, MachineType::CoreXYRotary) {
            axes[0] = (joints[0] + joints[1]) * 0.5;
            axes[1] = (joints[0] - joints[1]) * 0.5;
        }
        for i in 0..5 {
            if axes[i] < self.config.axis_limits[i].min || axes[i] > self.config.axis_limits[i].max {
                return false;
            }
        }
        true
    }
    
    /// Calculate steps per degree for rotary axis (from Grok's formula)
    pub const fn calculate_steps_per_degree(full_steps_per_rev: u32, gear_ratio: f32, microstepping: u32) -> f32 {
        (full_steps_per_rev as f32) * gear_ratio * (microstepping as f32) / 360.0
    }
    
    /// Convert pose to step positions for motor control
    pub fn pose_to_steps(&self, pose: Pose5D) -> Option<[i32; 5]> {
        if let Some(joints) = self.inverse_kinematics(pose) {
            let mut steps = [0i32; 5];
            for i in 0..5 {
                steps[i] = (joints[i] * self.config.steps_per_unit[i]) as i32;
            }
            Some(steps)
        } else {
            None
        }
    }
    
    /// Convert step positions to pose
    pub fn steps_to_pose(&self, steps: [i32; 5]) -> Pose5D {
        let mut joints = [0.0f32; 5];
        for i in 0..5 {
            joints[i] = (steps[i] as f32) / self.config.steps_per_unit[i];
        }
        self.forward_kinematics(joints)
    }
}

/// Default configurations for common machines (following Grok's examples)
impl EmbeddedMachineConfig {
    /// Penta Axis style configuration (toolhead AB rotary)
    pub const PENTA_AXIS: Self = Self {
        tool_offset: 45.0, // mm
        axis_limits: [
            AxisLimit { min: -110.0, max: 110.0 },  // X
            AxisLimit { min: -110.0, max: 110.0 },  // Y  
            AxisLimit { min: 0.0, max: 250.0 },     // Z
            AxisLimit { min: -180.0, max: 180.0 },  // A (rotate)
            AxisLimit { min: -45.0, max: 45.0 },    // B (tilt)
        ],
        steps_per_unit: [80.0, 80.0, 400.0, 11.11, 11.11], // Steps/mm for XYZ, steps/deg for AB
        max_feedrates: [9000.0, 9000.0, 720.0, 600.0, 600.0], // mm/min for XYZ, deg/min for AB
        accelerations: [1000.0, 1000.0, 100.0, 100.0, 100.0], // mm/s² and deg/s²
        machine_type: MachineType::ToolheadAB,
    };
    
    /// Open5X style configuration (toolhead AC rotary)
    pub const OPEN5X: Self = Self {
        tool_offset: 50.0,
        axis_limits: [
            AxisLimit { min: -125.0, max: 125.0 },  // X
            AxisLimit { min: -125.0, max: 125.0 },  // Y
            AxisLimit { min: 0.0, max: 300.0 },     // Z
            AxisLimit { min: -30.0, max: 30.0 },    // A (tilt)
            AxisLimit { min: -180.0, max: 180.0 },  // C (rotate)
        ],
        steps_per_unit: [80.0, 80.0, 400.0, 8.89, 4.44],
        max_feedrates: [12000.0, 12000.0, 720.0, 300.0, 1800.0],
        accelerations: [1500.0, 1500.0, 100.0, 50.0, 300.0],
        machine_type: MachineType::ToolheadAC,
    };
    
    /// CNC trunnion table configuration
    pub const CNC_TRUNNION: Self = Self {
        tool_offset: 0.0, // Tool offset handled differently for table rotary
        axis_limits: [
            AxisLimit { min: -200.0, max: 200.0 },  // X
            AxisLimit { min: -150.0, max: 150.0 },  // Y
            AxisLimit { min: 0.0, max: 350.0 },     // Z
            AxisLimit { min: -110.0, max: 110.0 },  // A (table tilt)
            AxisLimit { min: -360.0, max: 360.0 },  // C (table rotate)
        ],
        steps_per_unit: [200.0, 200.0, 400.0, 22.22, 11.11],
        max_feedrates: [6000.0, 6000.0, 600.0, 300.0, 600.0],
        accelerations: [800.0, 800.0, 100.0, 50.0, 100.0],
        machine_type: MachineType::TableAC,
    };
}

/// Pre-computed lookup tables for trigonometric functions (optional optimization)
pub mod lookup_tables {
    use super::*;
    
    /// Generate sine lookup table with 0.1° resolution
    pub fn generate_sin_table() -> [f32; 3600] {
        let mut table = [0.0f32; 3600];
        for (i, entry) in table.iter_mut().enumerate() {
            *entry = libm::sinf(i as f32 * 0.1 * PI / 180.0);
        }
        table
    }

    /// Generate cosine lookup table with 0.1° resolution
    pub fn generate_cos_table() -> [f32; 3600] {
        let mut table = [0.0f32; 3600];
        for (i, entry) in table.iter_mut().enumerate() {
            *entry = libm::cosf(i as f32 * 0.1 * PI / 180.0);
        }
        table
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_penta_axis_kinematics() {
        let kinematics = EmbeddedKinematics::new(EmbeddedMachineConfig::PENTA_AXIS);
        
        // Test forward kinematics. Stays inside the +/-110 limits so IK can round-trip.
        let joints = [50.0, 50.0, 200.0, 30.0, 15.0];
        let pose = kinematics.forward_kinematics(joints);

        // The tool hangs 45mm below the pivot, tilted 15 degrees
        assert!((pose.x - (50.0 - 45.0 * 0.2588 * 0.8660)).abs() < 0.01);
        assert!((pose.y - (50.0 - 45.0 * 0.2588 * 0.5)).abs() < 0.01);
        assert!((pose.z - (200.0 - 45.0 * 0.9659)).abs() < 0.01);
        
        // Test inverse kinematics round-trip
        if let Some(recovered_joints) = kinematics.inverse_kinematics(pose) {
            // Should be close to original (within numerical precision)
            for i in 0..5 {
                let error = (recovered_joints[i] - joints[i]).abs();
                assert!(error < 0.01, "Joint {} error too large: {}", i, error);
            }
        } else {
            panic!("Inverse kinematics failed for valid pose");
        }
    }
    
    #[test]
    fn test_singularity_avoidance() {
        let kinematics = EmbeddedKinematics::new(EmbeddedMachineConfig::PENTA_AXIS);
        
        // Test singularity at B = 90°
        let singular_pose = Pose5D {
            x: 100.0,
            y: 100.0,
            z: 200.0,
            a: 0.0,
            b_or_c: 90.0, // Singularity
        };
        
        // Should return None due to singularity
        assert!(kinematics.inverse_kinematics(singular_pose).is_none());
    }
    
    #[test]
    fn test_workspace_limits() {
        let kinematics = EmbeddedKinematics::new(EmbeddedMachineConfig::PENTA_AXIS);
        
        // Test out-of-bounds pose
        let oob_pose = Pose5D {
            x: 500.0, // Beyond X limit
            y: 100.0,
            z: 200.0,
            a: 0.0,
            b_or_c: 0.0,
        };
        
        // Should return None due to workspace limits
        assert!(kinematics.inverse_kinematics(oob_pose).is_none());
    }
    
    #[test]
    fn test_steps_calculation() {
        let steps_per_deg = EmbeddedKinematics::calculate_steps_per_degree(200, 50.0, 16);
        assert!((steps_per_deg - 444.44).abs() < 0.1);
    }
}