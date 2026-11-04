use nalgebra::{Matrix4, Vector3, Point3, Rotation3, Translation3, Isometry3, Unit, UnitQuaternion};
use serde::{Deserialize, Serialize};

/// 5-Axis machine configurations - defines the physical layout and parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum FiveAxisConfiguration {
    /// A/B rotary table configuration (A rotates around X, B rotates around Z)
    RotaryTable {
        table_offset: Vector3<f64>,      // Offset from machine origin to table center
        a_axis_position: Vector3<f64>,   // Position of A-axis rotation center
        b_axis_position: Vector3<f64>,   // Position of B-axis rotation center
        spindle_offset: Vector3<f64>,    // Tool tip offset from spindle center
        max_a_angle: f64,                // Maximum A-axis rotation (degrees)
        max_b_angle: f64,                // Maximum B-axis rotation (degrees)
        table_diameter: f64,             // Physical table size for collision detection
    },
    
    /// Trunnion table configuration (A rotates around X, C rotates around Z on the table)
    Trunnion {
        trunnion_center: Vector3<f64>,   // Center of trunnion rotation
        c_axis_height: f64,              // Height of C-axis above A-axis
        spindle_offset: Vector3<f64>,
        max_a_angle: f64,
        max_c_angle: f64,
        table_size: (f64, f64),          // Table dimensions (width, depth)
    },
    
    /// Tilting head configuration (B/C axes on the spindle head)
    TiltingHead {
        head_pivot_point: Vector3<f64>,  // Point around which head tilts
        b_axis_offset: f64,              // Distance from pivot to B-axis center
        c_axis_offset: f64,              // Distance from B-axis to C-axis center
        tool_length_offset: f64,         // Tool length compensation
        max_b_angle: f64,
        max_c_angle: f64,
        head_clearance_radius: f64,      // Head envelope for collision detection
    },
    
    /// Custom configuration with DH parameters
    Custom {
        dh_parameters: Vec<DHParameter>, // Denavit-Hartenberg parameters
        joint_limits: Vec<(f64, f64)>,   // Min/max for each joint
        link_geometries: Vec<LinkGeometry>, // For collision detection
    },
}

/// Denavit-Hartenberg parameters for custom kinematics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DHParameter {
    pub a: f64,      // Link length
    pub alpha: f64,  // Link twist (radians)
    pub d: f64,      // Joint offset
    pub theta: f64,  // Joint angle (radians) - variable for revolute joints
    pub joint_type: JointType,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum JointType {
    Revolute,   // Rotating joint
    Prismatic,  // Linear joint
    Fixed,      // Fixed transformation
}

/// Link geometry for collision detection
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LinkGeometry {
    pub shape: CollisionShape,
    pub transform: Isometry3<f64>,  // Transform relative to link frame
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum CollisionShape {
    Box { width: f64, depth: f64, height: f64 },
    Cylinder { radius: f64, height: f64 },
    Sphere { radius: f64 },
    ConvexHull { vertices: Vec<Point3<f64>> },
}

/// Complete pose in 6DOF (position + orientation)
#[derive(Debug, Clone)]
pub struct Pose6DOF {
    pub position: Point3<f64>,       // X, Y, Z position
    pub orientation: Unit<nalgebra::Quaternion<f64>>, // Tool orientation as quaternion
}

impl Pose6DOF {
    pub fn new(x: f64, y: f64, z: f64, roll: f64, pitch: f64, yaw: f64) -> Self {
        let position = Point3::new(x, y, z);
        let rotation = Rotation3::from_euler_angles(roll, pitch, yaw);
        let orientation = UnitQuaternion::from_rotation_matrix(&rotation);
        
        Self { position, orientation }
    }
    
    pub fn from_transform(transform: &Isometry3<f64>) -> Self {
        Self {
            position: transform.translation.vector.into(),
            orientation: transform.rotation,
        }
    }
    
    pub fn to_transform(&self) -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::from(self.position.coords),
            self.orientation.into(),
        )
    }
    
    /// Get Euler angles from orientation (roll, pitch, yaw)
    pub fn euler_angles(&self) -> (f64, f64, f64) {
        let rotation: Rotation3<f64> = self.orientation.into();
        rotation.euler_angles()
    }
    
    /// Get tool direction vector (Z-axis of tool frame)
    pub fn tool_direction(&self) -> Unit<Vector3<f64>> {
        let rotation: Rotation3<f64> = self.orientation.into();
        Unit::new_normalize(rotation * Vector3::z())
    }
}

/// Joint configuration for 5-axis machine
#[derive(Debug, Clone)]
pub struct JointConfiguration {
    pub x: f64,      // Linear X position (mm)
    pub y: f64,      // Linear Y position (mm)  
    pub z: f64,      // Linear Z position (mm)
    pub a: f64,      // Rotary A angle (degrees)
    pub b_or_c: f64, // Rotary B or C angle (degrees)
}

impl JointConfiguration {
    pub fn new(x: f64, y: f64, z: f64, a: f64, b_or_c: f64) -> Self {
        Self { x, y, z, a, b_or_c }
    }
    
    pub fn to_array(&self) -> [f64; 5] {
        [self.x, self.y, self.z, self.a, self.b_or_c]
    }
    
    pub fn from_array(values: [f64; 5]) -> Self {
        Self::new(values[0], values[1], values[2], values[3], values[4])
    }
}

/// 5-axis kinematics solver
#[derive(Debug, Clone)]
pub struct FiveAxisKinematics {
    pub config: FiveAxisConfiguration,
    pub workspace_limits: WorkspaceLimits,
    pub collision_checker: CollisionChecker,
}

#[derive(Debug, Clone)]
pub struct WorkspaceLimits {
    pub x_limits: (f64, f64),
    pub y_limits: (f64, f64),
    pub z_limits: (f64, f64),
    pub a_limits: (f64, f64),
    pub b_or_c_limits: (f64, f64),
    pub max_tool_length: f64,
    pub safety_margin: f64,
}

/// Collision detection system
#[derive(Debug, Clone)]
pub struct CollisionChecker {
    machine_geometry: Vec<LinkGeometry>,
    table_geometry: Vec<LinkGeometry>,
    tool_geometry: Option<LinkGeometry>,
    check_enabled: bool,
}

#[derive(Debug, Clone)]
pub enum KinematicsError {
    SolutionNotFound,
    OutOfWorkspace,
    SingularConfiguration,
    CollisionDetected,
    InvalidConfiguration,
    JointLimitsExceeded,
}

impl FiveAxisKinematics {
    pub fn new(config: FiveAxisConfiguration) -> Self {
        let workspace_limits = Self::default_workspace_limits(&config);
        let collision_checker = CollisionChecker::new();
        
        Self {
            config,
            workspace_limits,
            collision_checker,
        }
    }
    
    fn default_workspace_limits(config: &FiveAxisConfiguration) -> WorkspaceLimits {
        match config {
            FiveAxisConfiguration::RotaryTable { max_a_angle, max_b_angle, .. } => {
                WorkspaceLimits {
                    x_limits: (-500.0, 500.0),
                    y_limits: (-500.0, 500.0),
                    z_limits: (0.0, 400.0),
                    a_limits: (-*max_a_angle, *max_a_angle),
                    b_or_c_limits: (-*max_b_angle, *max_b_angle),
                    max_tool_length: 200.0,
                    safety_margin: 10.0,
                }
            },
            FiveAxisConfiguration::Trunnion { max_a_angle, max_c_angle, .. } => {
                WorkspaceLimits {
                    x_limits: (-400.0, 400.0),
                    y_limits: (-300.0, 300.0),
                    z_limits: (0.0, 350.0),
                    a_limits: (-*max_a_angle, *max_a_angle),
                    b_or_c_limits: (-*max_c_angle, *max_c_angle),
                    max_tool_length: 150.0,
                    safety_margin: 15.0,
                }
            },
            FiveAxisConfiguration::TiltingHead { max_b_angle, max_c_angle, .. } => {
                WorkspaceLimits {
                    x_limits: (-600.0, 600.0),
                    y_limits: (-400.0, 400.0),
                    z_limits: (0.0, 500.0),
                    a_limits: (0.0, 0.0), // No A-axis in tilting head
                    b_or_c_limits: (-(*max_b_angle).max(*max_c_angle), (*max_b_angle).max(*max_c_angle)),
                    max_tool_length: 250.0,
                    safety_margin: 20.0,
                }
            },
            FiveAxisConfiguration::Custom { joint_limits, .. } => {
                WorkspaceLimits {
                    x_limits: joint_limits.get(0).unwrap_or(&(-500.0, 500.0)).clone(),
                    y_limits: joint_limits.get(1).unwrap_or(&(-500.0, 500.0)).clone(),
                    z_limits: joint_limits.get(2).unwrap_or(&(0.0, 400.0)).clone(),
                    a_limits: joint_limits.get(3).unwrap_or(&(-180.0, 180.0)).clone(),
                    b_or_c_limits: joint_limits.get(4).unwrap_or(&(-180.0, 180.0)).clone(),
                    max_tool_length: 200.0,
                    safety_margin: 10.0,
                }
            },
        }
    }
    
    /// Forward kinematics: Joint angles → Tool pose
    pub fn forward_kinematics(&self, joints: &JointConfiguration) -> Result<Pose6DOF, KinematicsError> {
        // Check joint limits
        self.check_joint_limits(joints)?;
        
        match &self.config {
            FiveAxisConfiguration::RotaryTable { 
                table_offset, 
                a_axis_position, 
                b_axis_position, 
                spindle_offset, 
                .. 
            } => {
                self.forward_kinematics_rotary_table(joints, table_offset, a_axis_position, b_axis_position, spindle_offset)
            },
            
            FiveAxisConfiguration::Trunnion { 
                trunnion_center, 
                c_axis_height, 
                spindle_offset, 
                .. 
            } => {
                self.forward_kinematics_trunnion(joints, trunnion_center, *c_axis_height, spindle_offset)
            },
            
            FiveAxisConfiguration::TiltingHead { 
                head_pivot_point, 
                b_axis_offset, 
                c_axis_offset, 
                tool_length_offset, 
                .. 
            } => {
                self.forward_kinematics_tilting_head(joints, head_pivot_point, *b_axis_offset, *c_axis_offset, *tool_length_offset)
            },
            
            FiveAxisConfiguration::Custom { dh_parameters, .. } => {
                self.forward_kinematics_custom(joints, dh_parameters)
            },
        }
    }
    
    fn forward_kinematics_rotary_table(
        &self,
        joints: &JointConfiguration,
        table_offset: &Vector3<f64>,
        a_axis_position: &Vector3<f64>,
        b_axis_position: &Vector3<f64>,
        spindle_offset: &Vector3<f64>,
    ) -> Result<Pose6DOF, KinematicsError> {
        // Machine coordinate system transformations
        
        // 1. Start with machine base coordinates
        let machine_base = Isometry3::identity();
        
        // 2. Apply linear X, Y, Z motions
        let linear_transform = Isometry3::from_parts(
            Translation3::new(joints.x, joints.y, joints.z),
            nalgebra::UnitQuaternion::identity(),
        );
        
        // 3. Transform to table coordinate system
        let table_transform = Isometry3::from_parts(
            Translation3::from(*table_offset),
            nalgebra::UnitQuaternion::identity(),
        );
        
        // 4. Apply A-axis rotation (around X-axis)
        let a_rotation = Rotation3::from_axis_angle(
            &Vector3::x_axis(),
            joints.a.to_radians(),
        );
        let a_transform = Isometry3::from_parts(
            Translation3::from(*a_axis_position),
            nalgebra::UnitQuaternion::from_rotation_matrix(&a_rotation),
        );
        
        // 5. Apply B-axis rotation (around Z-axis)
        let b_rotation = Rotation3::from_axis_angle(
            &Vector3::z_axis(),
            joints.b_or_c.to_radians(),
        );
        let b_transform = Isometry3::from_parts(
            Translation3::from(*b_axis_position),
            nalgebra::UnitQuaternion::from_rotation_matrix(&b_rotation),
        );
        
        // 6. Apply spindle offset to get tool tip position
        let spindle_transform = Isometry3::from_parts(
            Translation3::from(*spindle_offset),
            nalgebra::UnitQuaternion::identity(),
        );
        
        // Chain all transformations
        let final_transform = machine_base * linear_transform * table_transform * a_transform * b_transform * spindle_transform;
        
        Ok(Pose6DOF::from_transform(&final_transform))
    }
    
    fn forward_kinematics_trunnion(
        &self,
        joints: &JointConfiguration,
        trunnion_center: &Vector3<f64>,
        c_axis_height: f64,
        spindle_offset: &Vector3<f64>,
    ) -> Result<Pose6DOF, KinematicsError> {
        // Linear motion
        let linear_transform = Isometry3::from_parts(
            Translation3::new(joints.x, joints.y, joints.z),
            nalgebra::UnitQuaternion::identity(),
        );
        
        // Move to trunnion center
        let trunnion_transform = Isometry3::from_parts(
            Translation3::from(*trunnion_center),
            nalgebra::UnitQuaternion::identity(),
        );
        
        // A-axis rotation (trunnion tilt)
        let a_rotation = Rotation3::from_axis_angle(
            &Vector3::x_axis(),
            joints.a.to_radians(),
        );
        let a_transform = Isometry3::from_parts(
            Translation3::identity(),
            nalgebra::UnitQuaternion::from_rotation_matrix(&a_rotation),
        );
        
        // Move to C-axis
        let c_axis_transform = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, c_axis_height),
            nalgebra::UnitQuaternion::identity(),
        );
        
        // C-axis rotation (table rotation)
        let c_rotation = Rotation3::from_axis_angle(
            &Vector3::z_axis(),
            joints.b_or_c.to_radians(),
        );
        let c_transform = Isometry3::from_parts(
            Translation3::identity(),
            nalgebra::UnitQuaternion::from_rotation_matrix(&c_rotation),
        );
        
        // Spindle offset
        let spindle_transform = Isometry3::from_parts(
            Translation3::from(*spindle_offset),
            nalgebra::UnitQuaternion::identity(),
        );
        
        let final_transform = linear_transform * trunnion_transform * a_transform * c_axis_transform * c_transform * spindle_transform;
        
        Ok(Pose6DOF::from_transform(&final_transform))
    }
    
    fn forward_kinematics_tilting_head(
        &self,
        joints: &JointConfiguration,
        head_pivot_point: &Vector3<f64>,
        b_axis_offset: f64,
        c_axis_offset: f64,
        tool_length_offset: f64,
    ) -> Result<Pose6DOF, KinematicsError> {
        // Linear motion
        let linear_transform = Isometry3::from_parts(
            Translation3::new(joints.x, joints.y, joints.z),
            nalgebra::UnitQuaternion::identity(),
        );
        
        // Move to head pivot point
        let head_transform = Isometry3::from_parts(
            Translation3::from(*head_pivot_point),
            nalgebra::UnitQuaternion::identity(),
        );
        
        // B-axis rotation (head tilt around Y)
        let b_rotation = Rotation3::from_axis_angle(
            &Vector3::y_axis(),
            joints.a.to_radians(), // A maps to B in tilting head
        );
        let b_transform = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, b_axis_offset),
            nalgebra::UnitQuaternion::from_rotation_matrix(&b_rotation),
        );
        
        // C-axis rotation (spindle rotation around Z)
        let c_rotation = Rotation3::from_axis_angle(
            &Vector3::z_axis(),
            joints.b_or_c.to_radians(),
        );
        let c_transform = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, c_axis_offset),
            nalgebra::UnitQuaternion::from_rotation_matrix(&c_rotation),
        );
        
        // Tool length offset
        let tool_transform = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, tool_length_offset),
            nalgebra::UnitQuaternion::identity(),
        );
        
        let final_transform = linear_transform * head_transform * b_transform * c_transform * tool_transform;
        
        Ok(Pose6DOF::from_transform(&final_transform))
    }
    
    fn forward_kinematics_custom(
        &self,
        joints: &JointConfiguration,
        dh_parameters: &[DHParameter],
    ) -> Result<Pose6DOF, KinematicsError> {
        let joint_values = [joints.x, joints.y, joints.z, joints.a, joints.b_or_c];
        let mut transform = Isometry3::identity();
        
        for (i, dh) in dh_parameters.iter().enumerate() {
            let joint_value = joint_values.get(i).unwrap_or(&0.0);
            
            let (theta, d) = match dh.joint_type {
                JointType::Revolute => (*joint_value + dh.theta, dh.d),
                JointType::Prismatic => (dh.theta, *joint_value + dh.d),
                JointType::Fixed => (dh.theta, dh.d),
            };
            
            // DH transformation matrix
            let cos_theta = theta.cos();
            let sin_theta = theta.sin();
            let cos_alpha = dh.alpha.cos();
            let sin_alpha = dh.alpha.sin();
            
            let dh_matrix = Matrix4::new(
                cos_theta, -sin_theta * cos_alpha,  sin_theta * sin_alpha, dh.a * cos_theta,
                sin_theta,  cos_theta * cos_alpha, -cos_theta * sin_alpha, dh.a * sin_theta,
                0.0,        sin_alpha,              cos_alpha,             d,
                0.0,        0.0,                    0.0,                   1.0,
            );
            
            // Extract rotation and translation from DH matrix
            let rotation = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(dh_matrix.fixed_view::<3, 3>(0, 0).into()));
            let translation = Translation3::from(Vector3::new(dh_matrix[(0, 3)], dh_matrix[(1, 3)], dh_matrix[(2, 3)]));
            let link_transform = Isometry3::from_parts(translation, rotation);
            transform = transform * link_transform;
        }
        
        Ok(Pose6DOF::from_transform(&transform))
    }
    
    /// Inverse kinematics: Tool pose → Joint angles
    pub fn inverse_kinematics(&self, target_pose: &Pose6DOF) -> Result<Vec<JointConfiguration>, KinematicsError> {
        // Check if target is in workspace
        self.check_workspace_limits(target_pose)?;
        
        match &self.config {
            FiveAxisConfiguration::RotaryTable { .. } => {
                self.inverse_kinematics_rotary_table(target_pose)
            },
            FiveAxisConfiguration::Trunnion { .. } => {
                self.inverse_kinematics_trunnion(target_pose)
            },
            FiveAxisConfiguration::TiltingHead { .. } => {
                self.inverse_kinematics_tilting_head(target_pose)
            },
            FiveAxisConfiguration::Custom { .. } => {
                self.inverse_kinematics_numerical(target_pose)
            },
        }
    }
    
    fn inverse_kinematics_rotary_table(&self, target_pose: &Pose6DOF) -> Result<Vec<JointConfiguration>, KinematicsError> {
        let mut solutions = Vec::new();
        
        if let FiveAxisConfiguration::RotaryTable { 
            table_offset, 
            a_axis_position: _, 
            b_axis_position: _, 
            spindle_offset: _, 
            .. 
        } = &self.config {
            
            // Get target position and orientation
            let target_pos = target_pose.position;
            let (_roll, _pitch, _yaw) = target_pose.euler_angles();
            
            // For rotary table, we need to solve for the table orientation
            // that positions the workpiece correctly relative to the tool
            
            // Try different A and B angle combinations
            let a_step = 5.0; // degrees
            let b_step = 5.0; // degrees
            
            for a_deg in (-90..=90).step_by(a_step as usize) {
                for b_deg in (-180..=180).step_by(b_step as usize) {
                    let _a_rad = (a_deg as f64).to_radians();
                    let _b_rad = (b_deg as f64).to_radians();
                    
                    // Calculate required linear position
                    // This is a simplified approach - in practice, you'd use
                    // the full inverse transformation chain
                    
                    let joints = JointConfiguration::new(
                        target_pos.x - table_offset.x,
                        target_pos.y - table_offset.y,
                        target_pos.z - table_offset.z,
                        a_deg as f64,
                        b_deg as f64,
                    );
                    
                    // Verify solution by forward kinematics
                    if let Ok(computed_pose) = self.forward_kinematics(&joints) {
                        let position_error = (computed_pose.position - target_pose.position).norm();
                        if position_error < 0.1 { // 0.1mm tolerance
                            solutions.push(joints);
                        }
                    }
                }
            }
        }
        
        if solutions.is_empty() {
            Err(KinematicsError::SolutionNotFound)
        } else {
            Ok(solutions)
        }
    }
    
    fn inverse_kinematics_trunnion(&self, target_pose: &Pose6DOF) -> Result<Vec<JointConfiguration>, KinematicsError> {
        // Similar approach to rotary table but with trunnion geometry
        // This would implement the specific inverse kinematics for trunnion configuration
        
        // Placeholder implementation
        let joints = JointConfiguration::new(
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z,
            0.0,
            0.0,
        );
        
        Ok(vec![joints])
    }
    
    fn inverse_kinematics_tilting_head(&self, target_pose: &Pose6DOF) -> Result<Vec<JointConfiguration>, KinematicsError> {
        // Analytical solution for tilting head configuration
        // The linear axes directly control the tool tip position
        
        if let FiveAxisConfiguration::TiltingHead { 
            head_pivot_point, 
            tool_length_offset, 
            .. 
        } = &self.config {
            
            // Calculate linear positions
            let x = target_pose.position.x - head_pivot_point.x;
            let y = target_pose.position.y - head_pivot_point.y;
            let z = target_pose.position.z - head_pivot_point.z - tool_length_offset;
            
            // Calculate required head orientation
            let _tool_direction = target_pose.tool_direction();
            let (_roll, pitch, yaw) = target_pose.euler_angles();
            
            // Convert orientation to B and C angles
            let b_angle = pitch.to_degrees();
            let c_angle = yaw.to_degrees();
            
            let joints = JointConfiguration::new(x, y, z, b_angle, c_angle);
            
            Ok(vec![joints])
        } else {
            Err(KinematicsError::InvalidConfiguration)
        }
    }
    
    fn inverse_kinematics_numerical(&self, target_pose: &Pose6DOF) -> Result<Vec<JointConfiguration>, KinematicsError> {
        // Numerical inverse kinematics using Newton-Raphson method
        // This is a general solution for any configuration
        
        const MAX_ITERATIONS: usize = 100;
        const TOLERANCE: f64 = 1e-6;
        
        // Initial guess
        let mut joints = JointConfiguration::new(
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z,
            0.0,
            0.0,
        );
        
        for _iteration in 0..MAX_ITERATIONS {
            // Compute forward kinematics
            let current_pose = self.forward_kinematics(&joints)?;
            
            // Calculate error
            let position_error = target_pose.position - current_pose.position;
            let orientation_error = target_pose.orientation.quaternion() - current_pose.orientation.quaternion();
            
            use nalgebra::Vector5;
            let error = Vector5::new(
                position_error.x,
                position_error.y, 
                position_error.z,
                orientation_error.w,
                orientation_error.w, // Using w component for both A and B/C axis error
            );
            
            if error.norm() < TOLERANCE {
                return Ok(vec![joints]);
            }
            
            // Compute Jacobian (numerical differentiation)
            let jacobian = self.compute_jacobian(&joints)?;
            
            // Newton-Raphson update
            if let Some(jacobian_inv) = jacobian.try_inverse() {
                let delta = jacobian_inv * error;
                
                joints.x -= delta[0];
                joints.y -= delta[1];
                joints.z -= delta[2];
                joints.a -= delta[3].to_degrees();
                joints.b_or_c -= delta[4].to_degrees();
                
                // Check joint limits
                if self.check_joint_limits(&joints).is_err() {
                    break;
                }
            } else {
                return Err(KinematicsError::SingularConfiguration);
            }
        }
        
        Err(KinematicsError::SolutionNotFound)
    }
    
    fn compute_jacobian(&self, joints: &JointConfiguration) -> Result<nalgebra::Matrix5<f64>, KinematicsError> {
        const EPSILON: f64 = 1e-6;
        
        let base_pose = self.forward_kinematics(joints)?;
        let mut jacobian = nalgebra::Matrix5::zeros();
        
        let joint_array = joints.to_array();
        
        for i in 0..5 {
            let mut perturbed_joints = joint_array;
            perturbed_joints[i] += EPSILON;
            
            let perturbed_config = JointConfiguration::from_array(perturbed_joints);
            let perturbed_pose = self.forward_kinematics(&perturbed_config)?;
            
            let position_diff = (perturbed_pose.position - base_pose.position) / EPSILON;
            let orientation_diff = (perturbed_pose.orientation.quaternion() - base_pose.orientation.quaternion()) / EPSILON;
            
            jacobian[(0, i)] = position_diff.x;
            jacobian[(1, i)] = position_diff.y;
            jacobian[(2, i)] = position_diff.z;
            jacobian[(3, i)] = orientation_diff.w;
            jacobian[(4, i)] = orientation_diff.i;
        }
        
        Ok(jacobian)
    }
    
    fn check_joint_limits(&self, joints: &JointConfiguration) -> Result<(), KinematicsError> {
        let limits = &self.workspace_limits;
        
        if joints.x < limits.x_limits.0 || joints.x > limits.x_limits.1 ||
           joints.y < limits.y_limits.0 || joints.y > limits.y_limits.1 ||
           joints.z < limits.z_limits.0 || joints.z > limits.z_limits.1 ||
           joints.a < limits.a_limits.0 || joints.a > limits.a_limits.1 ||
           joints.b_or_c < limits.b_or_c_limits.0 || joints.b_or_c > limits.b_or_c_limits.1 {
            return Err(KinematicsError::JointLimitsExceeded);
        }
        
        Ok(())
    }
    
    fn check_workspace_limits(&self, pose: &Pose6DOF) -> Result<(), KinematicsError> {
        let limits = &self.workspace_limits;
        
        if pose.position.x < limits.x_limits.0 || pose.position.x > limits.x_limits.1 ||
           pose.position.y < limits.y_limits.0 || pose.position.y > limits.y_limits.1 ||
           pose.position.z < limits.z_limits.0 || pose.position.z > limits.z_limits.1 {
            return Err(KinematicsError::OutOfWorkspace);
        }
        
        Ok(())
    }
    
    /// Check for collisions between machine components
    pub fn check_collisions(&self, joints: &JointConfiguration) -> Result<(), KinematicsError> {
        if !self.collision_checker.check_enabled {
            return Ok(());
        }
        
        // Simplified collision checking - in practice this would be much more sophisticated
        // Check table vs. spindle clearance for rotary table
        if let FiveAxisConfiguration::RotaryTable { table_diameter, spindle_offset, .. } = &self.config {
            let table_center_distance = (joints.x.powi(2) + joints.y.powi(2)).sqrt();
            let min_clearance = table_diameter / 2.0 + self.workspace_limits.safety_margin;
            
            if table_center_distance < min_clearance && joints.z < spindle_offset.z + 50.0 {
                return Err(KinematicsError::CollisionDetected);
            }
        }
        
        Ok(())
    }
    
    /// Generate optimized path for 5-axis motion
    pub fn plan_motion_path(&self, start_pose: &Pose6DOF, end_pose: &Pose6DOF, num_segments: usize) -> Result<Vec<Pose6DOF>, KinematicsError> {
        let mut path = Vec::with_capacity(num_segments + 1);
        
        for i in 0..=num_segments {
            let t = i as f64 / num_segments as f64;
            
            // Interpolate position linearly
            let position = start_pose.position + t * (end_pose.position - start_pose.position);
            
            // Interpolate orientation using spherical linear interpolation (SLERP)
            let orientation = start_pose.orientation.slerp(&end_pose.orientation, t);
            
            let interpolated_pose = Pose6DOF { position, orientation };
            
            // Verify the pose is achievable
            if self.inverse_kinematics(&interpolated_pose).is_ok() {
                path.push(interpolated_pose);
            } else {
                return Err(KinematicsError::OutOfWorkspace);
            }
        }
        
        Ok(path)
    }
}

impl CollisionChecker {
    pub fn new() -> Self {
        Self {
            machine_geometry: vec![],
            table_geometry: vec![],
            tool_geometry: None,
            check_enabled: false,
        }
    }
    
    pub fn add_machine_geometry(&mut self, geometry: LinkGeometry) {
        self.machine_geometry.push(geometry);
    }
    
    pub fn set_tool_geometry(&mut self, geometry: LinkGeometry) {
        self.tool_geometry = Some(geometry);
    }
    
    pub fn enable_checking(&mut self, enabled: bool) {
        self.check_enabled = enabled;
    }
}

impl Default for WorkspaceLimits {
    fn default() -> Self {
        Self {
            x_limits: (-500.0, 500.0),
            y_limits: (-500.0, 500.0),
            z_limits: (0.0, 400.0),
            a_limits: (-90.0, 90.0),
            b_or_c_limits: (-180.0, 180.0),
            max_tool_length: 200.0,
            safety_margin: 10.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_rotary_table_forward_kinematics() {
        let config = FiveAxisConfiguration::RotaryTable {
            table_offset: Vector3::new(0.0, 0.0, 100.0),
            a_axis_position: Vector3::new(0.0, 0.0, 0.0),
            b_axis_position: Vector3::new(0.0, 0.0, 0.0),
            spindle_offset: Vector3::new(0.0, 0.0, 200.0),
            max_a_angle: 90.0,
            max_b_angle: 180.0,
            table_diameter: 300.0,
        };
        
        let kinematics = FiveAxisKinematics::new(config);
        let joints = JointConfiguration::new(100.0, 50.0, 200.0, 45.0, 90.0);
        
        let result = kinematics.forward_kinematics(&joints);
        assert!(result.is_ok());
        
        let pose = result.unwrap();
        assert!(pose.position.x > 0.0);
        assert!(pose.position.y > 0.0);
        assert!(pose.position.z > 0.0);
    }
    
    #[test]
    fn test_inverse_kinematics_simple() {
        let config = FiveAxisConfiguration::TiltingHead {
            head_pivot_point: Vector3::new(0.0, 0.0, 300.0),
            b_axis_offset: 50.0,
            c_axis_offset: 100.0,
            tool_length_offset: 150.0,
            max_b_angle: 90.0,
            max_c_angle: 360.0,
            head_clearance_radius: 100.0,
        };
        
        let kinematics = FiveAxisKinematics::new(config);
        let target_pose = Pose6DOF::new(100.0, 100.0, 200.0, 0.0, 0.0, 0.0);
        
        let result = kinematics.inverse_kinematics(&target_pose);
        assert!(result.is_ok());
        
        let solutions = result.unwrap();
        assert!(!solutions.is_empty());
    }
    
    #[test]
    fn test_joint_limits_checking() {
        let config = FiveAxisConfiguration::RotaryTable {
            table_offset: Vector3::zeros(),
            a_axis_position: Vector3::zeros(),
            b_axis_position: Vector3::zeros(),
            spindle_offset: Vector3::zeros(),
            max_a_angle: 90.0,
            max_b_angle: 180.0,
            table_diameter: 300.0,
        };
        
        let kinematics = FiveAxisKinematics::new(config);
        
        // Valid joints
        let valid_joints = JointConfiguration::new(100.0, 100.0, 100.0, 45.0, 90.0);
        assert!(kinematics.check_joint_limits(&valid_joints).is_ok());
        
        // Invalid joints (exceeding limits)
        let invalid_joints = JointConfiguration::new(1000.0, 100.0, 100.0, 45.0, 90.0);
        assert!(kinematics.check_joint_limits(&invalid_joints).is_err());
    }
}