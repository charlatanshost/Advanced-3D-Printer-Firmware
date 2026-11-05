use crate::five_axis::{FiveAxisKinematics, Pose6DOF, JointConfiguration, KinematicsError};
use nalgebra::{Vector3, Point3, Unit, Vector6};
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
        if !(feedrate > 0.0) || !feedrate.is_finite() {
            return Err(KinematicsError::InvalidConfiguration);
        }

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
        
        // The slowest of: the commanded feedrate (mm/min), the rotation limit, and each
        // axis's own speed limit decides how long this move takes.
        let time_for_linear = linear_distance / (feedrate / 60.0);
        let time_for_angular = angular_distance / self.constraints.max_angular_velocity.x.min(self.constraints.max_angular_velocity.y);
        let time_for_axes = self.time_for_axis_limits(&start_joints, &end_joints);
        let duration_secs = time_for_linear.max(time_for_angular).max(time_for_axes);
        let segment_duration = Duration::from_secs_f64(duration_secs);

        // Calculate velocity profile
        let max_velocity = self.calculate_velocity_limits(&start_joints, &end_joints, duration_secs);
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
        if waypoints.len() < 2 || feedrates.is_empty() {
            return Err(KinematicsError::InvalidConfiguration);
        }

        // Generate initial segments
        let mut segments = Vec::with_capacity(waypoints.len() - 1);
        for i in 0..waypoints.len() - 1 {
            let feedrate = feedrates.get(i).unwrap_or(&feedrates[0]);
            segments.push(self.plan_motion(&waypoints[i], &waypoints[i + 1], *feedrate, orientation_mode.clone())?);
        }

        self.plan_junction_velocities(&mut segments);
        Ok(segments)
    }

    /// How fast may the path go through the corner between two segments? Full speed when
    /// straight, falling to zero for a complete reversal.
    fn junction_velocity(&self, current: &MotionSegment, next: &MotionSegment) -> f64 {
        let cruise = current.acceleration_profile.cruise_velocity.min(next.acceleration_profile.cruise_velocity);
        let current_direction = (current.end_pose.position - current.start_pose.position).try_normalize(1e-9);
        let next_direction = (next.end_pose.position - next.start_pose.position).try_normalize(1e-9);
        match (current_direction, next_direction) {
            (Some(a), Some(b)) => cruise * ((1.0 + a.dot(&b).clamp(-1.0, 1.0)) / 2.0),
            _ => 0.0, // Pure rotation on one side: stop at the junction
        }
    }

    /// Velocity lookahead: set each junction speed, then limit it to what the move lengths
    /// and acceleration allow in both directions, and rebuild each segment's profile.
    fn plan_junction_velocities(&self, segments: &mut [MotionSegment]) {
        if segments.is_empty() {
            return;
        }
        let accel = self.effective_acceleration();
        let lengths: Vec<f64> = segments.iter()
            .map(|s| { let p = &s.acceleration_profile; p.acceleration_distance + p.cruise_distance + p.deceleration_distance })
            .collect();

        // velocities[i] is the speed at the start of segment i. The path starts and ends stopped.
        let mut velocities = vec![0.0; segments.len() + 1];
        for i in 1..segments.len() {
            velocities[i] = self.junction_velocity(&segments[i - 1], &segments[i]);
        }
        for i in (0..segments.len()).rev() {
            velocities[i] = velocities[i].min((velocities[i + 1].powi(2) + 2.0 * accel * lengths[i]).sqrt());
        }
        for i in 0..segments.len() {
            velocities[i + 1] = velocities[i + 1].min((velocities[i].powi(2) + 2.0 * accel * lengths[i]).sqrt());
        }

        for (i, segment) in segments.iter_mut().enumerate() {
            let limit = segment.acceleration_profile.cruise_velocity;
            segment.acceleration_profile = Self::profile_between(lengths[i], velocities[i], velocities[i + 1], limit, accel);
        }
    }

    /// Trapezoid or triangle over `distance`, entering and leaving at the given speeds.
    fn profile_between(distance: f64, entry: f64, exit: f64, limit: f64, accel: f64) -> AccelerationProfile {
        let peak = ((2.0 * accel * distance + entry * entry + exit * exit) / 2.0).sqrt();
        let cruise = peak.min(limit).max(entry).max(exit);
        let acceleration_distance = ((cruise * cruise - entry * entry) / (2.0 * accel)).max(0.0);
        let deceleration_distance = ((cruise * cruise - exit * exit) / (2.0 * accel)).max(0.0);
        AccelerationProfile {
            acceleration_distance,
            cruise_distance: (distance - acceleration_distance - deceleration_distance).max(0.0),
            deceleration_distance,
            entry_velocity: entry,
            cruise_velocity: cruise,
            exit_velocity: exit,
        }
    }

    /// The acceleration the planner may assume. A smooth (S-curve) ramp peaks at 1.5x its
    /// average, so its average must be lower to keep the peak inside the limit.
    fn effective_acceleration(&self) -> f64 {
        let max = self.constraints.max_linear_acceleration.x;
        match self.planner_config.acceleration_algorithm {
            AccelerationAlgorithm::Trapezoidal => max,
            _ => max / 1.5,
        }
    }
    
    /// Plan non-planar printing path with surface conformance
    pub fn plan_non_planar_path(&self, path: &NonPlanarPath, base_feedrate: f64) -> Result<Vec<MotionSegment>, KinematicsError> {
        if path.waypoints.len() < 2 {
            return Err(KinematicsError::InvalidConfiguration);
        }

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
        self.plan_junction_velocities(&mut segments);
        
        Ok(segments)
    }
    
    fn select_optimal_joint_configurations(&self, start_solutions: &[JointConfiguration], end_solutions: &[JointConfiguration]) -> Result<(JointConfiguration, JointConfiguration), KinematicsError> {
        let (Some(first_start), Some(first_end)) = (start_solutions.first(), end_solutions.first()) else {
            return Err(KinematicsError::SolutionNotFound);
        };
        let mut best_start = first_start.clone();
        let mut best_end = first_end.clone();
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
        let angular_distance = (Self::shortest_angle(end.a - start.a).powi(2) + Self::shortest_angle(end.b_or_c - start.b_or_c).powi(2)).sqrt();

        // Weight angular motion more heavily to prefer solutions with less rotation
        linear_distance + angular_distance * 2.0
    }
    
    /// Wrap a difference in degrees into -180..180: 359 to 1 is 2 degrees, not 358.
    fn shortest_angle(delta_deg: f64) -> f64 {
        (delta_deg + 180.0).rem_euclid(360.0) - 180.0
    }

    fn calculate_angular_distance(&self, start_orientation: &Unit<nalgebra::Quaternion<f64>>, end_orientation: &Unit<nalgebra::Quaternion<f64>>) -> f64 {
        let dot_product = start_orientation.quaternion().dot(end_orientation.quaternion());
        let angle = 2.0 * dot_product.abs().min(1.0).acos();
        angle
    }
    
    /// Shortest time that respects every axis's own speed limit
    fn time_for_axis_limits(&self, start: &JointConfiguration, end: &JointConfiguration) -> f64 {
        let c = &self.constraints;
        [
            (end.x - start.x).abs() / c.max_linear_velocity.x,
            (end.y - start.y).abs() / c.max_linear_velocity.y,
            (end.z - start.z).abs() / c.max_linear_velocity.z,
            Self::shortest_angle(end.a - start.a).abs().to_radians() / c.max_angular_velocity.x,
            Self::shortest_angle(end.b_or_c - start.b_or_c).abs().to_radians() / c.max_angular_velocity.y,
        ].into_iter().fold(0.0, f64::max)
    }

    fn calculate_velocity_limits(&self, start_joints: &JointConfiguration, end_joints: &JointConfiguration, duration_secs: f64) -> Vector6<f64> {
        // Each axis's average speed is its travel divided by the move's duration
        let duration = duration_secs.max(1e-9);
        let linear_velocity = Vector3::new(
            (end_joints.x - start_joints.x).abs(),
            (end_joints.y - start_joints.y).abs(),
            (end_joints.z - start_joints.z).abs(),
        ) / duration;

        let angular_velocity = Vector3::new(
            Self::shortest_angle(end_joints.a - start_joints.a).abs().to_radians(),
            Self::shortest_angle(end_joints.b_or_c - start_joints.b_or_c).abs().to_radians(),
            0.0,
        ) / duration;

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
        // Profile distance is path length in mm. Pure rotations use their angle instead.
        let linear = ((end_joints.x - start_joints.x).powi(2) + (end_joints.y - start_joints.y).powi(2) + (end_joints.z - start_joints.z).powi(2)).sqrt();
        let angular = (Self::shortest_angle(end_joints.a - start_joints.a).powi(2) + Self::shortest_angle(end_joints.b_or_c - start_joints.b_or_c).powi(2)).sqrt();
        let total_distance = if linear > 1e-9 { linear } else { angular };

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
        // A smooth ramp with peak acceleration A covers the same distance as a linear
        // ramp at A / 1.5, so this always fits within `distance`.
        let accel = self.constraints.max_linear_acceleration.x / 1.5;
        Ok(Self::profile_between(distance, 0.0, 0.0, feedrate / 60.0, accel))
    }
    
    fn generate_jerk_limited_profile(&self, distance: f64, feedrate: f64) -> Result<AccelerationProfile, KinematicsError> {
        let max_velocity = feedrate / 60.0;
        let max_acceleration = self.constraints.max_linear_acceleration.x;
        let max_jerk = self.constraints.max_jerk[0]; // Use first component as reference
        
        // Getting up to speed v takes v/a + a/j seconds (ramp time plus jerk time) at an
        // average of v/2, so it covers v * (v/a + a/j) / 2.
        let ramp_distance = |v: f64| v * (v / max_acceleration + max_acceleration / max_jerk) / 2.0;

        // Short moves cannot reach full speed: find the speed where both ramps just fit.
        let mut cruise = max_velocity;
        if 2.0 * ramp_distance(cruise) > distance {
            let (mut low, mut high) = (0.0, max_velocity);
            for _ in 0..50 {
                let mid = (low + high) / 2.0;
                if 2.0 * ramp_distance(mid) > distance { high = mid; } else { low = mid; }
            }
            cruise = low;
        }
        let total_accel_distance = ramp_distance(cruise);

        Ok(AccelerationProfile {
            acceleration_distance: total_accel_distance,
            cruise_distance: (distance - 2.0 * total_accel_distance).max(0.0),
            deceleration_distance: total_accel_distance,
            entry_velocity: 0.0,
            cruise_velocity: cruise,
            exit_velocity: 0.0,
        })
    }
    
    fn generate_adaptive_profile(&self, start_joints: &JointConfiguration, end_joints: &JointConfiguration, distance: f64, feedrate: f64) -> Result<AccelerationProfile, KinematicsError> {
        // Adaptive profile based on joint motion characteristics
        let linear_motion = ((end_joints.x - start_joints.x).powi(2) + (end_joints.y - start_joints.y).powi(2) + (end_joints.z - start_joints.z).powi(2)).sqrt();
        let angular_motion = (Self::shortest_angle(end_joints.a - start_joints.a).powi(2) + Self::shortest_angle(end_joints.b_or_c - start_joints.b_or_c).powi(2)).sqrt();

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
                self.check_singularity(&intermediate_joints, start_joints, end_joints)?;
            }
        }

        Ok(())
    }

    /// With the tilt near zero the spinning axis lines up with the tool, so any change in
    /// it is a spin on the spot that needs unbounded speed. Sitting upright, or starting or
    /// ending there, is normal. Only a move passing through there while spinning is a problem.
    fn check_singularity(&self, joints: &JointConfiguration, start: &JointConfiguration, end: &JointConfiguration) -> Result<(), KinematicsError> {
        let tilting = (end.a - start.a).abs() > 1e-6;
        let spinning = Self::shortest_angle(end.b_or_c - start.b_or_c).abs() > 1.0;
        let passes_through = start.a.abs() >= 1.0 && end.a.abs() >= 1.0;
        if tilting && spinning && passes_through && joints.a.abs() < 1.0 {
            return Err(KinematicsError::SingularConfiguration);
        }
        Ok(())
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
        assert!(segment.is_ok(), "{:?}", segment.err());
    }

    fn simple_planner() -> FiveAxisMotionPlanner {
        FiveAxisMotionPlanner::new(FiveAxisKinematics::new(FiveAxisConfiguration::RotaryTable {
            table_offset: nalgebra::Vector3::zeros(),
            a_axis_position: nalgebra::Vector3::zeros(),
            b_axis_position: nalgebra::Vector3::zeros(),
            spindle_offset: nalgebra::Vector3::zeros(),
            max_a_angle: 90.0,
            max_b_angle: 180.0,
            table_diameter: 300.0,
        }))
    }

    fn level(x: f64, y: f64) -> Pose6DOF {
        Pose6DOF::new(x, y, 100.0, 0.0, 0.0, 0.0)
    }

    #[test]
    fn test_feedrate_is_mm_per_minute() {
        let segment = simple_planner().plan_motion(&level(100.0, 100.0), &level(200.0, 100.0), 3000.0, ToolOrientationMode::Interpolated).unwrap();
        assert!((segment.duration.as_secs_f64() - 2.0).abs() < 1e-6, "{:?}", segment.duration);
        assert!((segment.max_velocity[0] - 50.0).abs() < 1e-6, "{}", segment.max_velocity[0]);
        assert_eq!(segment.start_joints.a, 0.0, "a level pose stays level");
    }

    #[test]
    fn test_profiles_fit_the_move() {
        let planner = simple_planner();
        for profile in [planner.generate_scurve_profile(1.0, 6000.0).unwrap(), planner.generate_jerk_limited_profile(1.0, 6000.0).unwrap(), planner.generate_trapezoidal_profile(1.0, 6000.0).unwrap()] {
            let total = profile.acceleration_distance + profile.cruise_distance + profile.deceleration_distance;
            assert!((total - 1.0).abs() < 1e-6, "{:?}", profile);
        }
    }

    #[test]
    fn test_corners_slow_down_and_straights_do_not() {
        let planner = simple_planner();
        let plan = |points: &[(f64, f64)]| {
            let poses: Vec<_> = points.iter().map(|p| level(p.0, p.1)).collect();
            planner.plan_path(&poses, &[3000.0], ToolOrientationMode::Interpolated).unwrap()
        };
        let straight = plan(&[(100.0, 100.0), (200.0, 100.0), (300.0, 100.0)]);
        let reversal = plan(&[(100.0, 100.0), (200.0, 100.0), (100.0, 100.0)]);
        let square = plan(&[(100.0, 100.0), (200.0, 100.0), (200.0, 200.0)]);
        let junction = |s: &[MotionSegment]| s[0].acceleration_profile.exit_velocity;
        assert!((junction(&straight) - 50.0).abs() < 1e-6, "{}", junction(&straight));
        assert_eq!(junction(&reversal), 0.0);
        assert!(junction(&square) > 0.0 && junction(&square) < 50.0);
        for segs in [&straight, &reversal, &square] {
            assert_eq!(segs[0].acceleration_profile.entry_velocity, 0.0);
            assert_eq!(segs[1].acceleration_profile.exit_velocity, 0.0);
            assert_eq!(segs[0].acceleration_profile.exit_velocity, segs[1].acceleration_profile.entry_velocity);
        }
    }

    #[test]
    fn test_upright_is_not_singular_and_bad_input_is_an_error() {
        let planner = simple_planner();
        let path = NonPlanarPath { waypoints: vec![level(1.0, 1.0)], surface: None, layer_height_variation: false, adaptive_slicing: false };
        assert!(planner.plan_non_planar_path(&path, 100.0).is_err());
        assert!(planner.plan_motion(&level(1.0, 1.0), &level(2.0, 1.0), 0.0, ToolOrientationMode::Interpolated).is_err());
        assert!(planner.plan_path(&[level(1.0, 1.0), level(2.0, 1.0)], &[], ToolOrientationMode::Interpolated).is_err());
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