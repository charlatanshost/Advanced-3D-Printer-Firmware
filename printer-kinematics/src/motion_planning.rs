use crate::five_axis::{FiveAxisKinematics, Pose6DOF, JointConfiguration, KinematicsError};
use nalgebra::{Vector3, Point3, Unit, Vector6};
use std::collections::VecDeque;
use std::time::Duration;
use std::f64::consts::PI;

/// Advanced motion planning for 5-axis simultaneous movement
pub struct FiveAxisMotionPlanner {
    kinematics: FiveAxisKinematics,
    constraints: MotionConstraints,
    planner_config: PlannerConfiguration,
}

#[derive(Debug, Clone)]
pub struct MotionConstraints {
    pub max_linear_velocity: Vector3<f64>,     // mm/s for X, Y, Z
    pub max_angular_velocity: Vector3<f64>,    // rad/s for A, B/C
    pub max_linear_acceleration: Vector3<f64>, // mm/s² for X, Y, Z
    pub max_angular_acceleration: Vector3<f64>, // rad/s² for A, B/C
    pub max_jerk: Vector6<f64>,                // Combined linear and angular jerk limits
    pub corner_deviation: f64,                 // Maximum deviation for corner rounding (mm)
    pub minimum_segment_time: Duration,        // Minimum time per motion segment
}

#[derive(Debug, Clone)]
pub struct PlannerConfiguration {
    pub lookahead_segments: usize,             // Number of segments to look ahead
    pub acceleration_algorithm: AccelerationAlgorithm,
    pub corner_rounding: bool,                 // Enable automatic corner rounding
    pub adaptive_feedrate: bool,               // Dynamically adjust feedrate based on curvature
    pub collision_checking: bool,              // Enable real-time collision checking
    pub singularity_avoidance: bool,           // Avoid kinematic singularities
}

#[derive(Debug, Clone)]
pub enum AccelerationAlgorithm {
    Trapezoidal,    // Classic trapezoidal velocity profile
    SCurve,         // S-curve acceleration for smoother motion
    JerkLimited,    // Jerk-limited motion planning
    Adaptive,       // Adaptive algorithm based on geometry
}

/// Planned motion segment with complete kinematic information
#[derive(Debug, Clone)]
pub struct MotionSegment {
    pub start_pose: Pose6DOF,
    pub end_pose: Pose6DOF,
    pub start_joints: JointConfiguration,
    pub end_joints: JointConfiguration,
    pub duration: Duration,
    pub max_velocity: Vector6<f64>,            // Linear + angular velocities
    pub acceleration_profile: AccelerationProfile,
    pub tool_orientation_mode: ToolOrientationMode,
}

#[derive(Debug, Clone)]
pub enum ToolOrientationMode {
    /// Tool orientation remains constant in machine coordinates
    MachineConstant,
    /// Tool orientation remains constant relative to workpiece
    WorkpieceConstant,
    /// Tool follows surface normal (for machining applications)
    SurfaceNormal { lead_angle: f64, tilt_angle: f64 },
    /// Tool orientation interpolated smoothly between start and end
    Interpolated,
    /// Custom orientation control function
    Custom { control_function: fn(f64) -> (f64, f64, f64) },
}

#[derive(Debug, Clone)]
pub struct AccelerationProfile {
    pub acceleration_distance: f64,           // Distance over which to accelerate
    pub cruise_distance: f64,                 // Distance at constant velocity
    pub deceleration_distance: f64,           // Distance over which to decelerate
    pub entry_velocity: f64,                  // Velocity at segment start
    pub cruise_velocity: f64,                 // Maximum velocity during segment
    pub exit_velocity: f64,                   // Velocity at segment end
}

/// Surface definition for non-planar printing or machining
#[derive(Debug, Clone)]
pub struct Surface {
    pub points: Vec<Point3<f64>>,
    pub normals: Vec<Unit<Vector3<f64>>>,
    pub triangles: Vec<[usize; 3]>,           // Triangle indices into points array
}

/// Tool path with surface conformance for non-planar operations
#[derive(Debug, Clone)]
pub struct NonPlanarPath {
    pub waypoints: Vec<Pose6DOF>,
    pub surface: Option<Surface>,
    pub layer_height_variation: bool,         // Allow variable layer height
    pub adaptive_slicing: bool,               // Adapt slice thickness to surface curvature
}

impl FiveAxisMotionPlanner {
    pub fn new(kinematics: FiveAxisKinematics) -> Self {
        let constraints = MotionConstraints::default();
        let planner_config = PlannerConfiguration::default();
        
        Self {
            kinematics,
            constraints,
            planner_config,
        }
    }
    
    pub fn with_constraints(mut self, constraints: MotionConstraints) -> Self {
        self.constraints = constraints;
        self
    }
    
    pub fn with_config(mut self, config: PlannerConfiguration) -> Self {
        self.planner_config = config;
        self
    }
    
    /// Plan motion between two poses with specified tool orientation mode
    pub fn plan_motion(&self, start_pose: &Pose6DOF, end_pose: &Pose6DOF, feedrate: f64, orientation_mode: ToolOrientationMode) -> Result<MotionSegment, KinematicsError> {
        // Solve inverse kinematics for both poses
        let start_solutions = self.kinematics.inverse_kinematics(start_pose)?;
        let end_solutions = self.kinematics.inverse_kinematics(end_pose)?;
        
        // Select optimal joint configurations (minimize joint motion)
        let (start_joints, end_joints) = self.select_optimal_joint_configurations(&start_solutions, &end_solutions)?;
        
        // Check for collisions and singularities along the path
        if self.planner_config.collision_checking {
            self.check_path_validity(&start_joints, &end_joints)?;
        }
        
        // Calculate motion distance and time
        let linear_distance = (end_pose.position - start_pose.position).norm();
        let angular_distance = self.calculate_angular_distance(&start_pose.orientation, &end_pose.orientation);
        
        // Determine limiting factor (linear or angular motion)
        let time_for_linear = linear_distance / feedrate;
        let time_for_angular = angular_distance / self.constraints.max_angular_velocity.x.min(self.constraints.max_angular_velocity.y);
        let segment_duration = Duration::from_secs_f64(time_for_linear.max(time_for_angular));
        
        // Calculate velocity profile
        let max_velocity = self.calculate_velocity_limits(&start_joints, &end_joints, feedrate);
        let acceleration_profile = self.generate_acceleration_profile(&start_joints, &end_joints, feedrate)?;
        
        Ok(MotionSegment {
            start_pose: start_pose.clone(),
            end_pose: end_pose.clone(),
            start_joints,
            end_joints,
            duration: segment_duration,
            max_velocity,
            acceleration_profile,
            tool_orientation_mode: orientation_mode,
        })
    }
    
    /// Plan complex path with multiple waypoints and corner optimization
    pub fn plan_path(&self, waypoints: &[Pose6DOF], feedrates: &[f64], orientation_mode: ToolOrientationMode) -> Result<Vec<MotionSegment>, KinematicsError> {
        if waypoints.len() < 2 {
            return Err(KinematicsError::InvalidConfiguration);
        }
        
        let mut segments = Vec::new();
        let mut lookahead_queue = VecDeque::new();
        
        // Generate initial segments
        for i in 0..waypoints.len() - 1 {
            let feedrate = feedrates.get(i).unwrap_or(&feedrates[0]);
            let segment = self.plan_motion(&waypoints[i], &waypoints[i + 1], *feedrate, orientation_mode.clone())?;
            lookahead_queue.push_back(segment);
            
            // Process segments with lookahead optimization
            if lookahead_queue.len() >= self.planner_config.lookahead_segments {
                if let Some(optimized_segment) = lookahead_queue.pop_front() {
                    let optimized = self.optimize_segment_with_lookahead(&optimized_segment, &lookahead_queue)?;
                    segments.push(optimized);
                }
            }
        }
        
        // Process remaining segments
        while let Some(segment) = lookahead_queue.pop_front() {
            let optimized = self.optimize_segment_with_lookahead(&segment, &lookahead_queue)?;
            segments.push(optimized);
        }
        
        Ok(segments)
    }
    
    /// Plan non-planar printing path with surface conformance
    pub fn plan_non_planar_path(&self, path: &NonPlanarPath, base_feedrate: f64) -> Result<Vec<MotionSegment>, KinematicsError> {
        let mut segments = Vec::new();
        
        for i in 0..path.waypoints.len() - 1 {
            let start_pose = &path.waypoints[i];
            let end_pose = &path.waypoints[i + 1];
            
            // Calculate surface-adaptive feedrate
            let adaptive_feedrate = if self.planner_config.adaptive_feedrate {
                self.calculate_surface_adaptive_feedrate(start_pose, end_pose, &path.surface, base_feedrate)
            } else {
                base_feedrate
            };
            
            // Determine tool orientation based on surface normal
            let orientation_mode = if let Some(ref surface) = path.surface {
                self.calculate_surface_orientation_mode(start_pose, end_pose, surface)
            } else {
                ToolOrientationMode::Interpolated
            };
            
            let segment = self.plan_motion(start_pose, end_pose, adaptive_feedrate, orientation_mode)?;
            segments.push(segment);
        }
        
        // Post-process for smooth acceleration between segments
        self.smooth_segment_transitions(&mut segments)?;
        
        Ok(segments)
    }
    
    fn select_optimal_joint_configurations(&self, start_solutions: &[JointConfiguration], end_solutions: &[JointConfiguration]) -> Result<(JointConfiguration, JointConfiguration), KinematicsError> {
        let mut best_start = start_solutions[0].clone();
        let mut best_end = end_solutions[0].clone();
        let mut min_joint_motion = f64::INFINITY;
        
        // Try all combinations and select the one with minimum joint motion
        for start_config in start_solutions {
            for end_config in end_solutions {
                let joint_motion = self.calculate_joint_motion_distance(start_config, end_config);
                
                if joint_motion < min_joint_motion {
                    min_joint_motion = joint_motion;
                    best_start = start_config.clone();
                    best_end = end_config.clone();
                }
            }
        }
        
        Ok((best_start, best_end))
    }
    
    fn calculate_joint_motion_distance(&self, start: &JointConfiguration, end: &JointConfiguration) -> f64 {
        let linear_distance = ((end.x - start.x).powi(2) + (end.y - start.y).powi(2) + (end.z - start.z).powi(2)).sqrt();
        let angular_distance = ((end.a - start.a).powi(2) + (end.b_or_c - start.b_or_c).powi(2)).sqrt();
        
        // Weight angular motion more heavily to prefer solutions with less rotation
        linear_distance + angular_distance * 2.0
    }
    
    fn calculate_angular_distance(&self, start_orientation: &Unit<nalgebra::Quaternion<f64>>, end_orientation: &Unit<nalgebra::Quaternion<f64>>) -> f64 {
        let dot_product = start_orientation.quaternion().dot(end_orientation.quaternion());
        let angle = 2.0 * dot_product.abs().min(1.0).acos();
        angle
    }
    
    fn calculate_velocity_limits(&self, start_joints: &JointConfiguration, end_joints: &JointConfiguration, feedrate: f64) -> Vector6<f64> {
        // Calculate required velocities for each axis
        let linear_velocity = Vector3::new(
            (end_joints.x - start_joints.x).abs(),
            (end_joints.y - start_joints.y).abs(),
            (end_joints.z - start_joints.z).abs(),
        ) * feedrate / 60.0; // Convert mm/min to mm/s
        
        let angular_velocity = Vector3::new(
            (end_joints.a - start_joints.a).abs().to_radians(),
            (end_joints.b_or_c - start_joints.b_or_c).abs().to_radians(),
            0.0,
        ) * feedrate / 60.0;
        
        // Combine into 6DOF velocity vector
        Vector6::new(
            linear_velocity.x.min(self.constraints.max_linear_velocity.x),
            linear_velocity.y.min(self.constraints.max_linear_velocity.y),
            linear_velocity.z.min(self.constraints.max_linear_velocity.z),
            angular_velocity.x.min(self.constraints.max_angular_velocity.x),
            angular_velocity.y.min(self.constraints.max_angular_velocity.y),
            0.0,
        )
    }
    
    fn generate_acceleration_profile(&self, start_joints: &JointConfiguration, end_joints: &JointConfiguration, feedrate: f64) -> Result<AccelerationProfile, KinematicsError> {
        let total_distance = self.calculate_joint_motion_distance(start_joints, end_joints);
        
        match self.planner_config.acceleration_algorithm {
            AccelerationAlgorithm::Trapezoidal => {
                self.generate_trapezoidal_profile(total_distance, feedrate)
            },
            AccelerationAlgorithm::SCurve => {
                self.generate_scurve_profile(total_distance, feedrate)
            },
            AccelerationAlgorithm::JerkLimited => {
                self.generate_jerk_limited_profile(total_distance, feedrate)
            },
            AccelerationAlgorithm::Adaptive => {
                self.generate_adaptive_profile(start_joints, end_joints, total_distance, feedrate)
            },
        }
    }
    
    fn generate_trapezoidal_profile(&self, distance: f64, feedrate: f64) -> Result<AccelerationProfile, KinematicsError> {
        let max_velocity = feedrate / 60.0; // Convert to mm/s
        let max_acceleration = self.constraints.max_linear_acceleration.x; // Use X-axis as reference
        
        // Calculate acceleration distance
        let acceleration_time = max_velocity / max_acceleration;
        let acceleration_distance = 0.5 * max_acceleration * acceleration_time.powi(2);
        
        // Check if we can reach full speed
        if 2.0 * acceleration_distance <= distance {
            // Trapezoidal profile with cruise phase
            Ok(AccelerationProfile {
                acceleration_distance,
                cruise_distance: distance - 2.0 * acceleration_distance,
                deceleration_distance: acceleration_distance,
                entry_velocity: 0.0,
                cruise_velocity: max_velocity,
                exit_velocity: 0.0,
            })
        } else {
            // Triangular profile (no cruise phase)
            let peak_velocity = (max_acceleration * distance).sqrt();
            let accel_distance = distance / 2.0;
            
            Ok(AccelerationProfile {
                acceleration_distance: accel_distance,
                cruise_distance: 0.0,
                deceleration_distance: accel_distance,
                entry_velocity: 0.0,
                cruise_velocity: peak_velocity,
                exit_velocity: 0.0,
            })
        }
    }
    
    fn generate_scurve_profile(&self, distance: f64, feedrate: f64) -> Result<AccelerationProfile, KinematicsError> {
        // S-curve profile with smooth acceleration transitions
        // This is a simplified implementation - full S-curve requires more complex calculations
        let mut profile = self.generate_trapezoidal_profile(distance, feedrate)?;
        
        // Modify for smoother transitions (simplified)
        profile.acceleration_distance *= 1.2;
        profile.deceleration_distance *= 1.2;
        profile.cruise_distance = (distance - profile.acceleration_distance - profile.deceleration_distance).max(0.0);
        
        Ok(profile)
    }
    
    fn generate_jerk_limited_profile(&self, distance: f64, feedrate: f64) -> Result<AccelerationProfile, KinematicsError> {
        let max_velocity = feedrate / 60.0;
        let max_acceleration = self.constraints.max_linear_acceleration.x;
        let max_jerk = self.constraints.max_jerk[0]; // Use first component as reference
        
        // Calculate jerk-limited profile parameters
        let jerk_time = max_acceleration / max_jerk;
        let accel_time = max_velocity / max_acceleration;
        
        let jerk_distance = max_jerk * jerk_time.powi(3) / 6.0;
        let accel_distance = max_velocity * accel_time - 0.5 * max_acceleration * accel_time.powi(2);
        
        let total_accel_distance = 2.0 * jerk_distance + accel_distance;
        
        Ok(AccelerationProfile {
            acceleration_distance: total_accel_distance,
            cruise_distance: (distance - 2.0 * total_accel_distance).max(0.0),
            deceleration_distance: total_accel_distance,
            entry_velocity: 0.0,
            cruise_velocity: max_velocity,
            exit_velocity: 0.0,
        })
    }
    
    fn generate_adaptive_profile(&self, start_joints: &JointConfiguration, end_joints: &JointConfiguration, distance: f64, feedrate: f64) -> Result<AccelerationProfile, KinematicsError> {
        // Adaptive profile based on joint motion characteristics
        let linear_motion = ((end_joints.x - start_joints.x).powi(2) + (end_joints.y - start_joints.y).powi(2) + (end_joints.z - start_joints.z).powi(2)).sqrt();
        let angular_motion = ((end_joints.a - start_joints.a).powi(2) + (end_joints.b_or_c - start_joints.b_or_c).powi(2)).sqrt();
        
        // Use different acceleration algorithms based on motion type
        if angular_motion > linear_motion * 0.1 {
            // Significant angular motion - use jerk-limited for smoothness
            self.generate_jerk_limited_profile(distance, feedrate)
        } else {
            // Primarily linear motion - use trapezoidal for efficiency
            self.generate_trapezoidal_profile(distance, feedrate)
        }
    }
    
    fn check_path_validity(&self, start_joints: &JointConfiguration, end_joints: &JointConfiguration) -> Result<(), KinematicsError> {
        // Check multiple intermediate points along the path
        const CHECK_POINTS: usize = 10;
        
        for i in 0..=CHECK_POINTS {
            let t = i as f64 / CHECK_POINTS as f64;
            
            let intermediate_joints = JointConfiguration::new(
                start_joints.x + t * (end_joints.x - start_joints.x),
                start_joints.y + t * (end_joints.y - start_joints.y),
                start_joints.z + t * (end_joints.z - start_joints.z),
                start_joints.a + t * (end_joints.a - start_joints.a),
                start_joints.b_or_c + t * (end_joints.b_or_c - start_joints.b_or_c),
            );
            
            // Check collisions
            self.kinematics.check_collisions(&intermediate_joints)?;
            
            // Check for singularities (simplified check)
            if self.planner_config.singularity_avoidance {
                self.check_singularity(&intermediate_joints)?;
            }
        }
        
        Ok(())
    }
    
    fn check_singularity(&self, joints: &JointConfiguration) -> Result<(), KinematicsError> {
        // Simplified singularity check - in practice this would be more sophisticated
        
        // Check for wrist singularities (when axes align)
        if (joints.a % 180.0).abs() < 1.0 {
            return Err(KinematicsError::SingularConfiguration);
        }
        
        // Check for shoulder singularities (when arm is fully extended or retracted)
        let reach = (joints.x.powi(2) + joints.y.powi(2) + joints.z.powi(2)).sqrt();
        if reach < 50.0 || reach > 800.0 { // Arbitrary limits for this example
            return Err(KinematicsError::SingularConfiguration);
        }
        
        Ok(())
    }
    
    fn optimize_segment_with_lookahead(&self, segment: &MotionSegment, lookahead_segments: &VecDeque<MotionSegment>) -> Result<MotionSegment, KinematicsError> {
        let mut optimized = segment.clone();
        
        // Corner velocity optimization
        if self.planner_config.corner_rounding && !lookahead_segments.is_empty() {
            let next_segment = &lookahead_segments[0];
            
            // Calculate corner angle
            let current_direction = (segment.end_pose.position - segment.start_pose.position).normalize();
            let next_direction = (next_segment.end_pose.position - next_segment.start_pose.position).normalize();
            let corner_angle = current_direction.dot(&next_direction).acos();
            
            // Adjust exit velocity based on corner angle
            let corner_velocity_factor = (corner_angle / PI).min(1.0);
            optimized.acceleration_profile.exit_velocity *= corner_velocity_factor;
        }
        
        Ok(optimized)
    }
    
    fn calculate_surface_adaptive_feedrate(&self, start_pose: &Pose6DOF, end_pose: &Pose6DOF, surface: &Option<Surface>, base_feedrate: f64) -> f64 {
        if let Some(_surf) = surface {
            // Calculate surface curvature and adjust feedrate accordingly
            // This is a simplified implementation
            let _path_length = (end_pose.position - start_pose.position).norm();
            
            // Assume higher curvature requires lower feedrate
            let curvature_factor = 1.0; // Would be calculated from surface geometry
            base_feedrate * curvature_factor
        } else {
            base_feedrate
        }
    }
    
    fn calculate_surface_orientation_mode(&self, _start_pose: &Pose6DOF, _end_pose: &Pose6DOF, _surface: &Surface) -> ToolOrientationMode {
        // Calculate appropriate tool orientation for surface following
        // This would involve finding the surface normal at the tool position
        
        ToolOrientationMode::SurfaceNormal {
            lead_angle: 0.0,   // Angle ahead of surface normal
            tilt_angle: 0.0,   // Side tilt for tool clearance
        }
    }
    
    fn smooth_segment_transitions(&self, segments: &mut [MotionSegment]) -> Result<(), KinematicsError> {
        // Smooth velocity transitions between segments
        for i in 0..segments.len() - 1 {
            let current_exit_velocity = segments[i].acceleration_profile.exit_velocity;
            let next_entry_velocity = segments[i + 1].acceleration_profile.entry_velocity;
            
            // Average the velocities for smoother transition
            let smooth_velocity = (current_exit_velocity + next_entry_velocity) / 2.0;
            
            segments[i].acceleration_profile.exit_velocity = smooth_velocity;
            segments[i + 1].acceleration_profile.entry_velocity = smooth_velocity;
        }
        
        Ok(())
    }
}

impl Default for MotionConstraints {
    fn default() -> Self {
        Self {
            max_linear_velocity: Vector3::new(100.0, 100.0, 50.0),    // mm/s
            max_angular_velocity: Vector3::new(PI, PI, 2.0 * PI),     // rad/s
            max_linear_acceleration: Vector3::new(1000.0, 1000.0, 500.0), // mm/s²
            max_angular_acceleration: Vector3::new(10.0, 10.0, 20.0), // rad/s²
            max_jerk: Vector6::new(5000.0, 5000.0, 2500.0, 50.0, 50.0, 100.0), // mm/s³ and rad/s³
            corner_deviation: 0.1,                                    // mm
            minimum_segment_time: Duration::from_millis(1),           // 1ms minimum
        }
    }
}

impl Default for PlannerConfiguration {
    fn default() -> Self {
        Self {
            lookahead_segments: 5,
            acceleration_algorithm: AccelerationAlgorithm::SCurve,
            corner_rounding: true,
            adaptive_feedrate: true,
            collision_checking: true,
            singularity_avoidance: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::five_axis::FiveAxisConfiguration;
    
    #[test]
    fn test_motion_planning() {
        let config = FiveAxisConfiguration::RotaryTable {
            table_offset: nalgebra::Vector3::zeros(),
            a_axis_position: nalgebra::Vector3::zeros(),
            b_axis_position: nalgebra::Vector3::zeros(),
            spindle_offset: nalgebra::Vector3::new(0.0, 0.0, 200.0),
            max_a_angle: 90.0,
            max_b_angle: 180.0,
            table_diameter: 300.0,
        };
        
        let kinematics = FiveAxisKinematics::new(config);
        let planner = FiveAxisMotionPlanner::new(kinematics);
        
        let start_pose = Pose6DOF::new(100.0, 100.0, 200.0, 0.0, 0.0, 0.0);
        let end_pose = Pose6DOF::new(200.0, 150.0, 250.0, 0.1, 0.1, 0.0);
        
        let segment = planner.plan_motion(&start_pose, &end_pose, 3000.0, ToolOrientationMode::Interpolated);
        assert!(segment.is_ok());
    }
    
    #[test]
    fn test_acceleration_profiles() {
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
        let planner = FiveAxisMotionPlanner::new(kinematics);
        
        let profile = planner.generate_trapezoidal_profile(100.0, 3000.0);
        assert!(profile.is_ok());
        
        let prof = profile.unwrap();
        assert!(prof.acceleration_distance > 0.0);
        assert!(prof.cruise_velocity > 0.0);
    }
}