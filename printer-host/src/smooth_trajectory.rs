// Smooth Trajectory Planning System to Eliminate VFAs
// This module implements advanced trajectory planning techniques inspired by LinuxCNC
// to provide smooth, continuous motion that eliminates vertical fine artifacts (VFAs)
//
// How it works:
// - Moves wait in a queue. Each new move re-plans the junction speeds of the queue.
// - Points are generated one segment at a time, only when the previous segment has been
//   consumed. Anything already handed out is never planned again.
// - Each segment follows a real speed profile that respects the velocity, acceleration
//   and junction limits, and position is the integral of that speed.

use std::collections::VecDeque;
use serde::{Serialize, Deserialize};

// Advanced motion profile types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MotionProfile {
    Trapezoidal,     // Traditional trapezoidal velocity profile
    SCurve,          // S-curve profile with jerk limiting
    Bezier,          // Currently uses the same smooth speed ramps as SCurve
    BSpline,         // Currently uses the same smooth speed ramps as SCurve
    NURBS,           // Currently uses the same smooth speed ramps as SCurve
}

impl MotionProfile {
    fn is_smooth(&self) -> bool {
        !matches!(self, MotionProfile::Trapezoidal)
    }
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

/// Exact geometry of an arc in the XY plane (Z may change, giving a helix)
#[derive(Debug, Clone)]
struct ArcGeometry {
    center: [f64; 2],
    radius: f64,
    start_angle: f64,
    /// Signed angle swept, in radians. Positive is counter-clockwise.
    sweep: f64,
    start_z: f64,
    end_z: f64,
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
    arc: Option<ArcGeometry>,
}

impl SmoothSegment {
    /// Position after travelling `s` mm along this segment
    fn position_at(&self, s: f64) -> [f64; 3] {
        let t = if self.arc_length > 0.0 { (s / self.arc_length).clamp(0.0, 1.0) } else { 1.0 };
        match &self.arc {
            Some(arc) => {
                let angle = arc.start_angle + arc.sweep * t;
                [
                    arc.center[0] + arc.radius * angle.cos(),
                    arc.center[1] + arc.radius * angle.sin(),
                    arc.start_z + (arc.end_z - arc.start_z) * t,
                ]
            }
            None => SmoothTrajectoryPlanner::interpolate_3d(&self.start_point.position, &self.end_point.position, t),
        }
    }

    /// Unit direction of travel after `s` mm along this segment
    fn direction_at(&self, s: f64) -> [f64; 3] {
        match &self.arc {
            Some(arc) => {
                let t = if self.arc_length > 0.0 { (s / self.arc_length).clamp(0.0, 1.0) } else { 1.0 };
                let angle = arc.start_angle + arc.sweep * t;
                let turning = arc.sweep.signum() * arc.radius * arc.sweep.abs();
                SmoothTrajectoryPlanner::normalize(&[-angle.sin() * turning, angle.cos() * turning, arc.end_z - arc.start_z])
            }
            None => SmoothTrajectoryPlanner::normalize(&SmoothTrajectoryPlanner::subtract_3d(
                &self.end_point.position,
                &self.start_point.position,
            )),
        }
    }

    fn curvature(&self) -> f64 {
        self.arc.as_ref().map_or(0.0, |arc| 1.0 / arc.radius)
    }

    /// Speed limit for this segment: its feedrate, the machine's, and for arcs the
    /// speed at which cornering stays inside the acceleration limit.
    fn speed_limit(&self, constraints: &MotionConstraints) -> f64 {
        let mut limit = self.start_point.feedrate.min(constraints.max_velocity);
        if let Some(arc) = &self.arc {
            limit = limit.min((constraints.max_acceleration * arc.radius).sqrt());
        }
        limit
    }
}

/// Speed over time for one segment: ramp from entry to cruise, hold, ramp to exit.
#[derive(Debug, Clone, Copy)]
struct SpeedProfile {
    entry: f64,
    cruise: f64,
    exit: f64,
    accel_time: f64,
    cruise_time: f64,
    decel_time: f64,
    smooth: bool,
}

impl SpeedProfile {
    fn total_time(&self) -> f64 {
        self.accel_time + self.cruise_time + self.decel_time
    }

    /// One ramp from v0 to v1 lasting `duration`: (distance so far, speed, acceleration) at time t
    fn ramp(v0: f64, v1: f64, duration: f64, t: f64, smooth: bool) -> (f64, f64, f64) {
        if duration <= 0.0 {
            return (0.0, v1, 0.0);
        }
        let u = (t / duration).clamp(0.0, 1.0);
        let dv = v1 - v0;
        if smooth {
            // Smoothstep: speed eases in and out, peak acceleration is 1.5x the average
            let speed = v0 + dv * (3.0 * u * u - 2.0 * u * u * u);
            let distance = duration * (v0 * u + dv * (u * u * u - 0.5 * u * u * u * u));
            let accel = dv * 6.0 * u * (1.0 - u) / duration;
            (distance, speed, accel)
        } else {
            (duration * (v0 * u + 0.5 * dv * u * u), v0 + dv * u, dv / duration)
        }
    }

    /// (distance, speed, acceleration) at time t
    fn sample(&self, t: f64) -> (f64, f64, f64) {
        if t < self.accel_time {
            return Self::ramp(self.entry, self.cruise, self.accel_time, t, self.smooth);
        }
        let accel_distance = (self.entry + self.cruise) / 2.0 * self.accel_time;
        let t = t - self.accel_time;
        if t < self.cruise_time {
            return (accel_distance + self.cruise * t, self.cruise, 0.0);
        }
        let cruise_distance = self.cruise * self.cruise_time;
        let t = t - self.cruise_time;
        let (d, v, a) = Self::ramp(self.cruise, self.exit, self.decel_time, t, self.smooth);
        (accel_distance + cruise_distance + d, v, a)
    }
}

// Advanced lookahead buffer for smooth motion planning
pub struct SmoothTrajectoryPlanner {
    segments: VecDeque<SmoothSegment>,
    planned_points: VecDeque<MotionPoint>,
    constraints: MotionConstraints,
    interpolation_rate: f64,      // Points per second for interpolation
    current_time: f64,
    profile_type: MotionProfile,
    /// Speed at the end of the last segment handed out, which the next must start from
    current_speed: f64,
}

impl SmoothTrajectoryPlanner {
    pub fn new(constraints: MotionConstraints, profile_type: MotionProfile) -> Self {
        Self {
            segments: VecDeque::with_capacity(256),
            planned_points: VecDeque::with_capacity(1000),
            constraints,
            interpolation_rate: 1000.0, // 1000 Hz interpolation
            current_time: 0.0,
            profile_type,
            current_speed: 0.0,
        }
    }

    pub fn add_linear_move(&mut self, start: [f64; 3], end: [f64; 3], feedrate: f64) {
        let distance = Self::distance_3d(&start, &end);
        if distance < 1e-6 {
            return; // Skip zero-length moves
        }
        if !(feedrate > 0.0) || !feedrate.is_finite() {
            log::warn!("Ignoring move with invalid feedrate {}", feedrate);
            return;
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
            arc: None,
        };

        self.segments.push_back(segment);
        self.replan_velocities();
    }

    /// Add an arc in the XY plane. If `start` and `end` are the same point this is a full circle.
    pub fn add_arc_move(&mut self, start: [f64; 3], end: [f64; 3], center: [f64; 3],
                       clockwise: bool, feedrate: f64) {
        if !(feedrate > 0.0) || !feedrate.is_finite() {
            log::warn!("Ignoring arc with invalid feedrate {}", feedrate);
            return;
        }

        let start_radius = (start[0] - center[0]).hypot(start[1] - center[1]);
        let end_radius = (end[0] - center[0]).hypot(end[1] - center[1]);

        // G-code coordinates are rounded (usually to 0.001mm), so the two radii never match
        // exactly. Only treat them as inconsistent if they differ by more than rounding could.
        let tolerance = (start_radius * 0.001).max(0.05);
        if start_radius < 1e-6 || (start_radius - end_radius).abs() > tolerance {
            log::warn!("Inconsistent arc (radius {} vs {}), using a straight line", start_radius, end_radius);
            self.add_linear_move(start, end, feedrate);
            return;
        }

        let sweep = Self::calculate_arc_sweep(&start, &end, &center, clockwise);
        let planar_length = start_radius * sweep.abs();
        let arc_length = planar_length.hypot(end[2] - start[2]);
        let segment_time = arc_length / feedrate.min(self.constraints.max_velocity);
        let curvature = 1.0 / start_radius;

        let segment = SmoothSegment {
            start_point: MotionPoint {
                position: start,
                feedrate,
                curvature,
                ..Default::default()
            },
            end_point: MotionPoint {
                position: end,
                feedrate,
                time: segment_time,
                curvature,
                ..Default::default()
            },
            control_points: Vec::new(),
            profile_type: self.profile_type.clone(),
            segment_time,
            arc_length,
            max_deviation: 0.0,
            arc: Some(ArcGeometry {
                center: [center[0], center[1]],
                radius: start_radius,
                start_angle: (start[1] - center[1]).atan2(start[0] - center[0]),
                sweep,
                start_z: start[2],
                end_z: end[2],
            }),
        };

        self.segments.push_back(segment);
        self.replan_velocities();
    }

    /// The acceleration planning may assume. A smooth ramp peaks at 1.5x its average,
    /// so its average has to be lower for the peak to stay inside the limit.
    fn planning_acceleration(&self, profile: &MotionProfile) -> f64 {
        if profile.is_smooth() {
            self.constraints.max_acceleration / 1.5
        } else {
            self.constraints.max_acceleration
        }
    }

    /// Set the entry and exit speed of every waiting segment
    fn replan_velocities(&mut self) {
        let count = self.segments.len();
        if count == 0 {
            return;
        }

        // Upper limit on the speed at each junction. junction[i] is the start of segment i.
        // The last move must be able to stop, as nothing is known to follow it.
        let mut junction = vec![0.0f64; count + 1];
        junction[0] = self.current_speed;
        for i in 1..count {
            let (previous, current) = (&self.segments[i - 1], &self.segments[i]);
            let corner = self.calculate_corner_velocity(previous, current);
            junction[i] = corner
                .min(previous.speed_limit(&self.constraints))
                .min(current.speed_limit(&self.constraints));
        }

        // Backward pass: slow enough to be able to brake for what comes next
        for i in (0..count).rev() {
            let segment = &self.segments[i];
            let accel = self.planning_acceleration(&segment.profile_type);
            let reachable = (junction[i + 1].powi(2) + 2.0 * accel * segment.arc_length).sqrt();
            if i > 0 {
                junction[i] = junction[i].min(reachable);
            }
        }

        // Forward pass: no faster than can be reached from what came before
        for i in 0..count {
            let segment = &self.segments[i];
            let accel = self.planning_acceleration(&segment.profile_type);
            let reachable = (junction[i].powi(2) + 2.0 * accel * segment.arc_length).sqrt();
            junction[i + 1] = junction[i + 1].min(reachable);
        }

        for (i, segment) in self.segments.iter_mut().enumerate() {
            segment.start_point.actual_feedrate = junction[i];
            segment.end_point.actual_feedrate = junction[i + 1];
        }
    }

    /// Fastest speed through the corner that stays within `deviation_tolerance` of the path
    fn calculate_corner_velocity(&self, prev_seg: &SmoothSegment, curr_seg: &SmoothSegment) -> f64 {
        let prev_dir = prev_seg.direction_at(prev_seg.arc_length);
        let curr_dir = curr_seg.direction_at(0.0);

        let dot_product = Self::dot_product_3d(&prev_dir, &curr_dir).clamp(-1.0, 1.0);
        let angle = dot_product.acos();

        if angle < 0.1 { // Nearly straight, maintain high velocity
            return self.constraints.max_velocity;
        }

        // Junction deviation. The fillet radius is δ·sin(θ/2) / (1 - sin(θ/2)), where θ is the
        // interior angle: 180 degrees when going straight on, 0 for a full reversal.
        let sin_half_interior = ((1.0 + dot_product) / 2.0).clamp(0.0, 1.0).sqrt();
        if sin_half_interior <= 0.0 {
            return 0.0; // Full reversal: must stop
        }
        let radius = self.constraints.deviation_tolerance * sin_half_interior / (1.0 - sin_half_interior).max(1e-9);
        let centripetal_velocity = (self.constraints.max_acceleration * radius).sqrt();

        centripetal_velocity.min(self.constraints.corner_velocity)
    }

    /// Work out the speed profile for a segment going from `entry` to `exit`
    fn build_profile(&self, segment: &SmoothSegment, entry: f64, exit: f64) -> SpeedProfile {
        let smooth = segment.profile_type.is_smooth();
        let accel = self.constraints.max_acceleration;
        let jerk = self.constraints.max_jerk;
        let distance = segment.arc_length;
        let limit = segment.speed_limit(&self.constraints).max(entry).max(exit);

        // How long a ramp between two speeds takes
        let ramp_time = |from: f64, to: f64| -> f64 {
            let dv = (to - from).abs();
            if smooth {
                // Smoothstep: peak acceleration 1.5 dv/T, peak jerk 6 dv/T²
                (1.5 * dv / accel).max((6.0 * dv / jerk).sqrt())
            } else {
                dv / accel
            }
        };
        let ramps_distance = |cruise: f64| -> f64 {
            (entry + cruise) / 2.0 * ramp_time(entry, cruise) + (cruise + exit) / 2.0 * ramp_time(cruise, exit)
        };

        // Highest cruise speed whose two ramps fit in this segment
        let floor = entry.max(exit);
        let cruise = if ramps_distance(limit) <= distance {
            limit
        } else {
            let (mut low, mut high) = (floor, limit);
            for _ in 0..40 {
                let mid = (low + high) / 2.0;
                if ramps_distance(mid) <= distance { low = mid; } else { high = mid; }
            }
            low
        };

        let mut accel_time = ramp_time(entry, cruise);
        let mut decel_time = ramp_time(cruise, exit);
        let used = (entry + cruise) / 2.0 * accel_time + (cruise + exit) / 2.0 * decel_time;

        let cruise_time = if used <= distance {
            if cruise > 1e-9 { (distance - used) / cruise } else { 0.0 }
        } else {
            // Only the jerk limit can get here, as planning already allowed for the acceleration
            // limit. Stretch to the exact fit: slower ramps can only reduce acceleration.
            let scale = distance / used.max(1e-12);
            accel_time *= scale;
            decel_time *= scale;
            0.0
        };

        SpeedProfile { entry, cruise, exit, accel_time, cruise_time, decel_time, smooth }
    }

    /// Turn the next waiting segment into points
    fn generate_next_segment(&mut self) -> bool {
        let Some(mut segment) = self.segments.pop_front() else {
            return false;
        };

        let entry = self.current_speed;
        let exit = segment.end_point.actual_feedrate;
        let mut profile = self.build_profile(&segment, entry, exit);

        // A move starting and ending stopped with no time would never get anywhere
        if profile.total_time() <= 0.0 {
            profile.cruise = segment.speed_limit(&self.constraints);
            profile.cruise_time = segment.arc_length / profile.cruise.max(1e-9);
        }

        let total_time = profile.total_time();
        segment.segment_time = total_time;
        let steps = ((total_time * self.interpolation_rate).ceil() as usize).max(1);
        let dt = total_time / steps as f64;
        let feedrate = segment.start_point.feedrate;
        let curvature = segment.curvature();

        for i in 1..=steps {
            let t = i as f64 * dt;
            let (mut distance, mut speed, mut accel) = profile.sample(t);
            if i == steps {
                // Finish exactly on the end point despite rounding
                distance = segment.arc_length;
                speed = exit;
                accel = 0.0;
            }
            let direction = segment.direction_at(distance);
            self.planned_points.push_back(MotionPoint {
                position: if i == steps { segment.end_point.position } else { segment.position_at(distance) },
                velocity: Self::scale_3d(&direction, speed),
                acceleration: Self::scale_3d(&direction, accel),
                jerk: [0.0; 3],
                time: self.current_time + t,
                feedrate,
                actual_feedrate: speed,
                curvature,
            });
        }

        self.current_time += total_time;
        self.current_speed = exit;
        true
    }

    // Utility functions
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

    /// Signed angle from start to end around center. The same point twice is a full circle.
    fn calculate_arc_sweep(start: &[f64; 3], end: &[f64; 3], center: &[f64; 3], clockwise: bool) -> f64 {
        use std::f64::consts::TAU;
        let start_angle = (start[1] - center[1]).atan2(start[0] - center[0]);
        let end_angle = (end[1] - center[1]).atan2(end[0] - center[0]);
        let same_point = (start[0] - end[0]).hypot(start[1] - end[1]) < 1e-6;
        let ccw = if same_point { TAU } else { (end_angle - start_angle).rem_euclid(TAU) };
        if clockwise {
            if same_point { -TAU } else { ccw - TAU }
        } else {
            ccw
        }
    }

    // Public interface methods
    pub fn get_next_point(&mut self) -> Option<MotionPoint> {
        if self.planned_points.is_empty() {
            self.generate_next_segment();
        }
        self.planned_points.pop_front()
    }

    pub fn has_planned_points(&self) -> bool {
        !self.planned_points.is_empty() || !self.segments.is_empty()
    }

    pub fn get_buffer_level(&self) -> f64 {
        self.planned_points.len() as f64 / 1000.0 // Percentage full
    }

    pub fn clear_buffer(&mut self) {
        self.segments.clear();
        self.planned_points.clear();
        self.current_time = 0.0;
        self.current_speed = 0.0;
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
