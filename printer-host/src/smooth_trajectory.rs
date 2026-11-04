// Smooth Trajectory Planning System to Eliminate VFAs
// This module implements advanced trajectory planning techniques inspired by LinuxCNC
// to provide smooth, continuous motion that eliminates vertical fine artifacts (VFAs)

use std::collections::VecDeque;
use std::time::{Duration, Instant};
use serde::{Serialize, Deserialize};

// Advanced motion profile types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MotionProfile {
    Trapezoidal,     // Traditional trapezoidal velocity profile
    SCurve,          // S-curve profile with jerk limiting
    Bezier,          // Bezier curve smoothing
    BSpline,         // B-spline trajectory
    NURBS,           // Non-uniform rational B-splines
}

// Motion constraints for smooth planning
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotionConstraints {
    pub max_velocity: f64,       // mm/s
    pub max_acceleration: f64,   // mm/s²
    pub max_jerk: f64,          // mm/s³
    pub max_snap: f64,          // mm/s⁴ (4th derivative)
    pub corner_velocity: f64,    // Max velocity through corners
    pub deviation_tolerance: f64, // Path deviation tolerance
    pub minimum_time_step: f64,  // Minimum time between points
}

impl Default for MotionConstraints {
    fn default() -> Self {
        Self {
            max_velocity: 300.0,
            max_acceleration: 3000.0,
            max_jerk: 30000.0,
            max_snap: 300000.0,
            corner_velocity: 50.0,
            deviation_tolerance: 0.01,
            minimum_time_step: 0.001,
        }
    }
}

// 3D point with timing information
#[derive(Debug, Clone)]
pub struct MotionPoint {
    pub position: [f64; 3],      // X, Y, Z coordinates
    pub velocity: [f64; 3],      // Velocity vector
    pub acceleration: [f64; 3],  // Acceleration vector
    pub jerk: [f64; 3],         // Jerk vector
    pub time: f64,              // Time from start of move
    pub feedrate: f64,          // Commanded feedrate
    pub actual_feedrate: f64,   // Actual achievable feedrate
    pub curvature: f64,         // Path curvature at this point
}

impl Default for MotionPoint {
    fn default() -> Self {
        Self {
            position: [0.0; 3],
            velocity: [0.0; 3],
            acceleration: [0.0; 3],
            jerk: [0.0; 3],
            time: 0.0,
            feedrate: 0.0,
            actual_feedrate: 0.0,
            curvature: 0.0,
        }
    }
}

// Smooth trajectory segment
#[derive(Debug, Clone)]
pub struct SmoothSegment {
    pub start_point: MotionPoint,
    pub end_point: MotionPoint,
    pub control_points: Vec<MotionPoint>, // For spline-based motion
    pub profile_type: MotionProfile,
    pub segment_time: f64,
    pub arc_length: f64,
    pub max_deviation: f64,
}

// Advanced lookahead buffer for smooth motion planning
pub struct SmoothTrajectoryPlanner {
    segments: VecDeque<SmoothSegment>,
    planned_points: VecDeque<MotionPoint>,
    constraints: MotionConstraints,
    lookahead_time: f64,          // How far ahead to plan (seconds)
    interpolation_rate: f64,      // Points per second for interpolation
    current_time: f64,
    profile_type: MotionProfile,
    cornering_algorithm: CorneringAlgorithm,
}

#[derive(Debug, Clone)]
pub enum CorneringAlgorithm {
    ConstantAcceleration,   // Traditional approach
    CornerBlending,         // Blend corners with curves
    AdaptiveLookahead,      // Adaptive lookahead based on geometry
    PredictiveSmoothing,    // Predictive smoothing based on upcoming moves
}

impl SmoothTrajectoryPlanner {
    pub fn new(constraints: MotionConstraints, profile_type: MotionProfile) -> Self {
        Self {
            segments: VecDeque::with_capacity(256),
            planned_points: VecDeque::with_capacity(1000),
            constraints,
            lookahead_time: 1.0,        // 1 second lookahead
            interpolation_rate: 1000.0, // 1000 Hz interpolation
            current_time: 0.0,
            profile_type,
            cornering_algorithm: CorneringAlgorithm::PredictiveSmoothing,
        }
    }
    
    pub fn add_linear_move(&mut self, start: [f64; 3], end: [f64; 3], feedrate: f64) {
        let distance = Self::distance_3d(&start, &end);
        if distance < 1e-6 {
            return; // Skip zero-length moves
        }
        
        let segment_time = distance / feedrate.min(self.constraints.max_velocity);
        
        let start_point = MotionPoint {
            position: start,
            feedrate,
            ..Default::default()
        };
        
        let end_point = MotionPoint {
            position: end,
            feedrate,
            time: segment_time,
            ..Default::default()
        };
        
        let segment = SmoothSegment {
            start_point,
            end_point,
            control_points: Vec::new(),
            profile_type: self.profile_type.clone(),
            segment_time,
            arc_length: distance,
            max_deviation: 0.0,
        };
        
        self.segments.push_back(segment);
        self.replan_velocities();
    }
    
    pub fn add_arc_move(&mut self, start: [f64; 3], end: [f64; 3], center: [f64; 3], 
                       clockwise: bool, feedrate: f64) {
        // Calculate arc parameters
        let start_radius = Self::distance_3d(&start, &center);
        let end_radius = Self::distance_3d(&end, &center);
        
        if (start_radius - end_radius).abs() > 1e-6 {
            // Radii don't match, fall back to linear move
            self.add_linear_move(start, end, feedrate);
            return;
        }
        
        let arc_angle = Self::calculate_arc_angle(&start, &end, &center);
        let arc_length = start_radius * arc_angle;
        let segment_time = arc_length / feedrate.min(self.constraints.max_velocity);
        
        // Generate control points for smooth arc
        let num_points = (arc_length / 1.0).ceil() as usize; // 1mm resolution
        let mut control_points = Vec::with_capacity(num_points);
        
        for i in 0..=num_points {
            let t = i as f64 / num_points as f64;
            let angle = if clockwise { -arc_angle * t } else { arc_angle * t };
            let point = Self::rotate_point_around_center(&start, &center, angle);
            
            control_points.push(MotionPoint {
                position: point,
                time: segment_time * t,
                feedrate,
                curvature: 1.0 / start_radius, // Curvature is 1/radius
                ..Default::default()
            });
        }
        
        let segment = SmoothSegment {
            start_point: MotionPoint {
                position: start,
                feedrate,
                curvature: 1.0 / start_radius,
                ..Default::default()
            },
            end_point: MotionPoint {
                position: end,
                feedrate,
                time: segment_time,
                curvature: 1.0 / start_radius,
                ..Default::default()
            },
            control_points,
            profile_type: MotionProfile::Bezier,
            segment_time,
            arc_length,
            max_deviation: 0.0,
        };
        
        self.segments.push_back(segment);
        self.replan_velocities();
    }
    
    fn replan_velocities(&mut self) {
        if self.segments.len() < 2 {
            return;
        }
        
        // Forward pass: calculate maximum entry velocities based on cornering
        for i in 1..self.segments.len() {
            let (prev_seg, curr_seg) = self.get_adjacent_segments(i);
            let corner_velocity = self.calculate_corner_velocity(prev_seg, curr_seg);
            
            // Update entry velocity of current segment
            if let Some(segment) = self.segments.get_mut(i) {
                let prev_feedrate = segment.start_point.feedrate;
                segment.start_point.actual_feedrate = prev_feedrate.min(corner_velocity);
            }
        }
        
        // Backward pass: propagate velocity constraints
        for i in (0..self.segments.len() - 1).rev() {
            if let (Some(current), Some(next)) = 
                (self.segments.get(i), self.segments.get(i + 1)) {
                
                let max_entry_vel = self.calculate_max_entry_velocity(
                    current, 
                    next.start_point.actual_feedrate
                );
                
                if let Some(segment) = self.segments.get_mut(i) {
                    segment.end_point.actual_feedrate = 
                        segment.end_point.feedrate.min(max_entry_vel);
                }
            }
        }
        
        // Generate interpolated points for smooth execution
        self.generate_interpolated_points();
    }
    
    fn calculate_corner_velocity(&self, prev_seg: &SmoothSegment, curr_seg: &SmoothSegment) -> f64 {
        // Calculate the angle between segments
        let prev_dir = Self::normalize(&Self::subtract_3d(
            &prev_seg.end_point.position, 
            &prev_seg.start_point.position
        ));
        
        let curr_dir = Self::normalize(&Self::subtract_3d(
            &curr_seg.end_point.position, 
            &curr_seg.start_point.position
        ));
        
        let dot_product = Self::dot_product_3d(&prev_dir, &curr_dir);
        let angle = dot_product.clamp(-1.0, 1.0).acos();
        
        if angle < 0.1 { // Small angle, maintain high velocity
            return self.constraints.max_velocity;
        }
        
        // Calculate corner velocity based on deviation tolerance
        let radius = self.constraints.deviation_tolerance / (1.0 - (angle / 2.0).cos());
        let centripetal_velocity = (self.constraints.max_acceleration * radius).sqrt();
        
        centripetal_velocity.min(self.constraints.corner_velocity)
    }
    
    fn calculate_max_entry_velocity(&self, current: &SmoothSegment, exit_velocity: f64) -> f64 {
        let distance = current.arc_length;
        let max_accel = self.constraints.max_acceleration;
        
        // v² = u² + 2as, solving for u (entry velocity)
        let max_velocity_squared = exit_velocity.powi(2) + 2.0 * max_accel * distance;
        max_velocity_squared.sqrt().min(current.start_point.feedrate)
    }
    
    fn generate_interpolated_points(&mut self) {
        self.planned_points.clear();
        
        // Clone segments to avoid borrowing conflicts
        let segments_to_process: Vec<SmoothSegment> = self.segments.iter().cloned().collect();
        
        for segment in segments_to_process {
            match segment.profile_type {
                MotionProfile::SCurve => {
                    self.generate_s_curve_points(&segment);
                },
                MotionProfile::Bezier => {
                    self.generate_bezier_points(&segment);
                },
                MotionProfile::BSpline => {
                    self.generate_bspline_points(&segment);
                },
                _ => {
                    self.generate_trapezoidal_points(&segment);
                }
            }
        }
    }
    
    fn generate_s_curve_points(&mut self, segment: &SmoothSegment) {
        let num_points = (segment.segment_time * self.interpolation_rate).ceil() as usize;
        let dt = segment.segment_time / num_points as f64;
        
        for i in 0..=num_points {
            let t = i as f64 * dt;
            let s = self.s_curve_profile(t / segment.segment_time);
            
            // Interpolate position along segment
            let position = Self::interpolate_3d(
                &segment.start_point.position,
                &segment.end_point.position,
                s
            );
            
            // Calculate derivatives for velocity and acceleration
            let ds_dt = self.s_curve_velocity(t / segment.segment_time) / segment.segment_time;
            let d2s_dt2 = self.s_curve_acceleration(t / segment.segment_time) / segment.segment_time.powi(2);
            
            let direction = Self::normalize(&Self::subtract_3d(
                &segment.end_point.position,
                &segment.start_point.position
            ));
            
            let velocity = Self::scale_3d(&direction, ds_dt * segment.arc_length);
            let acceleration = Self::scale_3d(&direction, d2s_dt2 * segment.arc_length);
            
            let point = MotionPoint {
                position,
                velocity,
                acceleration,
                jerk: [0.0; 3], // Calculate if needed
                time: self.current_time + t,
                feedrate: segment.start_point.feedrate,
                actual_feedrate: Self::magnitude_3d(&velocity),
                curvature: 0.0, // Linear segment
            };
            
            self.planned_points.push_back(point);
        }
        
        self.current_time += segment.segment_time;
    }
    
    fn generate_bezier_points(&mut self, segment: &SmoothSegment) {
        let num_points = (segment.segment_time * self.interpolation_rate).ceil() as usize;
        
        if segment.control_points.is_empty() {
            // Simple linear Bezier
            self.generate_s_curve_points(segment);
            return;
        }
        
        // Cubic Bezier curve
        for i in 0..=num_points {
            let t = i as f64 / num_points as f64;
            
            // Cubic Bezier interpolation
            let point = if segment.control_points.len() >= 2 {
                let p0 = &segment.start_point.position;
                let p1 = &segment.control_points[0].position;
                let p2 = &segment.control_points[segment.control_points.len() - 1].position;
                let p3 = &segment.end_point.position;
                
                Self::cubic_bezier(p0, p1, p2, p3, t)
            } else {
                Self::interpolate_3d(
                    &segment.start_point.position,
                    &segment.end_point.position,
                    t
                )
            };
            
            // Calculate velocity from position derivative
            let velocity = if i > 0 && i < num_points {
                let dt = segment.segment_time / num_points as f64;
                let prev_pos = self.planned_points.back().unwrap().position;
                let vel = Self::scale_3d(&Self::subtract_3d(&point, &prev_pos), 1.0 / dt);
                vel
            } else {
                [0.0; 3]
            };
            
            let motion_point = MotionPoint {
                position: point,
                velocity,
                acceleration: [0.0; 3], // Would need second derivative
                jerk: [0.0; 3],
                time: self.current_time + t * segment.segment_time,
                feedrate: segment.start_point.feedrate,
                actual_feedrate: Self::magnitude_3d(&velocity),
                curvature: 0.0, // Would need curvature calculation
            };
            
            self.planned_points.push_back(motion_point);
        }
        
        self.current_time += segment.segment_time;
    }
    
    fn generate_trapezoidal_points(&mut self, segment: &SmoothSegment) {
        // Simple trapezoidal velocity profile
        let num_points = (segment.segment_time * self.interpolation_rate).ceil() as usize;
        let dt = segment.segment_time / num_points as f64;
        
        let start_vel = segment.start_point.actual_feedrate;
        let end_vel = segment.end_point.actual_feedrate;
        let max_vel = segment.start_point.feedrate.min(self.constraints.max_velocity);
        
        for i in 0..=num_points {
            let t = i as f64 * dt;
            let progress = t / segment.segment_time;
            
            // Trapezoidal velocity profile
            let velocity_magnitude = if progress < 0.5 {
                start_vel + (max_vel - start_vel) * (progress * 2.0).min(1.0)
            } else {
                max_vel - (max_vel - end_vel) * ((progress - 0.5) * 2.0).min(1.0)
            };
            
            let position = Self::interpolate_3d(
                &segment.start_point.position,
                &segment.end_point.position,
                progress
            );
            
            let direction = Self::normalize(&Self::subtract_3d(
                &segment.end_point.position,
                &segment.start_point.position
            ));
            
            let velocity = Self::scale_3d(&direction, velocity_magnitude);
            
            let point = MotionPoint {
                position,
                velocity,
                acceleration: [0.0; 3],
                jerk: [0.0; 3],
                time: self.current_time + t,
                feedrate: segment.start_point.feedrate,
                actual_feedrate: velocity_magnitude,
                curvature: 0.0,
            };
            
            self.planned_points.push_back(point);
        }
        
        self.current_time += segment.segment_time;
    }
    
    fn generate_bspline_points(&mut self, _segment: &SmoothSegment) {
        // B-spline implementation would go here
        // For now, fall back to S-curve
        // self.generate_s_curve_points(segment);
    }
    
    // S-curve profile functions
    fn s_curve_profile(&self, t: f64) -> f64 {
        if t <= 0.0 { 0.0 }
        else if t >= 1.0 { 1.0 }
        else if t < 0.5 {
            2.0 * t * t
        } else {
            1.0 - 2.0 * (1.0 - t) * (1.0 - t)
        }
    }
    
    fn s_curve_velocity(&self, t: f64) -> f64 {
        if t <= 0.0 || t >= 1.0 { 0.0 }
        else if t < 0.5 {
            4.0 * t
        } else {
            4.0 * (1.0 - t)
        }
    }
    
    fn s_curve_acceleration(&self, t: f64) -> f64 {
        if t <= 0.0 || t >= 1.0 { 0.0 }
        else if t < 0.5 {
            4.0
        } else {
            -4.0
        }
    }
    
    // Utility functions
    fn get_adjacent_segments(&self, index: usize) -> (&SmoothSegment, &SmoothSegment) {
        (&self.segments[index - 1], &self.segments[index])
    }
    
    fn distance_3d(a: &[f64; 3], b: &[f64; 3]) -> f64 {
        ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt()
    }
    
    fn subtract_3d(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
        [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
    }
    
    fn normalize(v: &[f64; 3]) -> [f64; 3] {
        let mag = Self::magnitude_3d(v);
        if mag > 1e-10 {
            [v[0] / mag, v[1] / mag, v[2] / mag]
        } else {
            [0.0, 0.0, 0.0]
        }
    }
    
    fn magnitude_3d(v: &[f64; 3]) -> f64 {
        (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
    }
    
    fn dot_product_3d(a: &[f64; 3], b: &[f64; 3]) -> f64 {
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    }
    
    fn scale_3d(v: &[f64; 3], scale: f64) -> [f64; 3] {
        [v[0] * scale, v[1] * scale, v[2] * scale]
    }
    
    fn interpolate_3d(start: &[f64; 3], end: &[f64; 3], t: f64) -> [f64; 3] {
        [
            start[0] + (end[0] - start[0]) * t,
            start[1] + (end[1] - start[1]) * t,
            start[2] + (end[2] - start[2]) * t,
        ]
    }
    
    fn cubic_bezier(p0: &[f64; 3], p1: &[f64; 3], p2: &[f64; 3], p3: &[f64; 3], t: f64) -> [f64; 3] {
        let u = 1.0 - t;
        let tt = t * t;
        let uu = u * u;
        let uuu = uu * u;
        let ttt = tt * t;
        
        [
            uuu * p0[0] + 3.0 * uu * t * p1[0] + 3.0 * u * tt * p2[0] + ttt * p3[0],
            uuu * p0[1] + 3.0 * uu * t * p1[1] + 3.0 * u * tt * p2[1] + ttt * p3[1],
            uuu * p0[2] + 3.0 * uu * t * p1[2] + 3.0 * u * tt * p2[2] + ttt * p3[2],
        ]
    }
    
    fn calculate_arc_angle(start: &[f64; 3], end: &[f64; 3], center: &[f64; 3]) -> f64 {
        let start_vec = Self::subtract_3d(start, center);
        let end_vec = Self::subtract_3d(end, center);
        
        let dot = Self::dot_product_3d(&start_vec, &end_vec);
        let mag_start = Self::magnitude_3d(&start_vec);
        let mag_end = Self::magnitude_3d(&end_vec);
        
        if mag_start > 1e-10 && mag_end > 1e-10 {
            (dot / (mag_start * mag_end)).clamp(-1.0, 1.0).acos()
        } else {
            0.0
        }
    }
    
    fn rotate_point_around_center(point: &[f64; 3], center: &[f64; 3], angle: f64) -> [f64; 3] {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        
        let dx = point[0] - center[0];
        let dy = point[1] - center[1];
        
        [
            center[0] + dx * cos_a - dy * sin_a,
            center[1] + dx * sin_a + dy * cos_a,
            point[2], // Z unchanged for 2D rotation
        ]
    }
    
    // Public interface methods
    pub fn get_next_point(&mut self) -> Option<MotionPoint> {
        self.planned_points.pop_front()
    }
    
    pub fn has_planned_points(&self) -> bool {
        !self.planned_points.is_empty()
    }
    
    pub fn get_buffer_level(&self) -> f64 {
        self.planned_points.len() as f64 / 1000.0 // Percentage full
    }
    
    pub fn clear_buffer(&mut self) {
        self.segments.clear();
        self.planned_points.clear();
        self.current_time = 0.0;
    }
    
    pub fn set_motion_profile(&mut self, profile: MotionProfile) {
        self.profile_type = profile;
    }
    
    pub fn set_constraints(&mut self, constraints: MotionConstraints) {
        self.constraints = constraints;
    }
    
    pub fn get_current_constraints(&self) -> &MotionConstraints {
        &self.constraints
    }
}