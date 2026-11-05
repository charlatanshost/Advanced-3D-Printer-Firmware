/// Volumetric Compensation System
///
/// This module implements advanced volumetric compensation that goes beyond simple
/// backlash compensation. It creates detailed compensation maps using probing routines
/// to measure and correct for geometric inaccuracies across the entire build volume.

use std::collections::HashMap;
use std::time::{Duration, Instant};
use serde::{Deserialize, Serialize};
use nalgebra::{Vector3, Point3, Matrix3, Matrix4};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VolumetricCompensationConfig {
    pub probe_resolution: f32,           // Probe point spacing in mm
    pub build_volume: [f32; 3],          // X, Y, Z build volume limits
    pub probe_speed: f32,                // Probing feedrate in mm/min
    pub probe_retract_distance: f32,     // Distance to retract after probe
    pub probe_samples: u8,               // Number of samples per probe point
    pub max_deviation: f32,              // Maximum allowed deviation in mm
    pub temperature_compensation: bool,   // Enable thermal compensation
    pub reference_temperature: f32,      // Reference temperature for measurements
}

impl Default for VolumetricCompensationConfig {
    fn default() -> Self {
        Self {
            probe_resolution: 25.0,          // 25mm grid
            build_volume: [300.0, 300.0, 400.0],
            probe_speed: 300.0,              // 5mm/s
            probe_retract_distance: 5.0,
            probe_samples: 3,
            max_deviation: 2.0,              // 2mm max deviation
            temperature_compensation: true,
            reference_temperature: 20.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProbePoint {
    pub commanded_position: Point3<f32>, // Where we commanded the machine to go
    pub actual_position: Point3<f32>,    // Where it actually went (measured)
    pub error_vector: Vector3<f32>,      // Difference vector
    pub temperature: f32,                // Temperature at time of measurement
    pub timestamp: u64,                  // Unix timestamp of measurement
    pub confidence: f32,                 // Measurement confidence (0-1)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CompensationMap {
    pub grid_resolution: f32,
    pub build_volume: [f32; 3],
    pub probe_points: Vec<ProbePoint>,
    pub grid_points: HashMap<(i32, i32, i32), Vector3<f32>>, // Grid index -> correction vector
    pub temperature_coefficients: Option<ThermalCompensationCoeffs>,
    pub creation_timestamp: u64,
    pub calibration_id: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThermalCompensationCoeffs {
    pub linear_expansion: [f32; 3],      // Linear expansion coefficients per axis (mm/°C/mm)
    pub angular_distortion: [f32; 3],   // Angular distortion coefficients (rad/°C)
    pub frame_expansion: Matrix3<f32>,   // Frame expansion matrix
    pub hotend_offset_temp_coeff: Vector3<f32>, // Hotend thermal offset
}

#[derive(Debug, Clone)]
pub struct VolumetricCompensator {
    config: VolumetricCompensationConfig,
    active_map: Option<CompensationMap>,
    probe_sequence: Option<ProbeSequence>,
    current_temperature: f32,
    machine_state: MachineGeometry,
}

#[derive(Debug, Clone)]
pub struct MachineGeometry {
    pub frame_squareness: Matrix3<f32>,  // Frame squareness correction matrix
    pub axis_scaling: Vector3<f32>,      // Steps per mm scaling corrections
    pub backlash_vectors: Vector3<f32>,  // Mechanical backlash per axis
    pub leadscrew_pitch_error: HashMap<char, Vec<f32>>, // Leadscrew pitch errors
}

impl Default for MachineGeometry {
    fn default() -> Self {
        Self {
            frame_squareness: Matrix3::identity(),
            axis_scaling: Vector3::new(1.0, 1.0, 1.0),
            backlash_vectors: Vector3::zeros(),
            leadscrew_pitch_error: HashMap::new(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct ProbeSequence {
    pub probe_points: Vec<Point3<f32>>,
    pub current_index: usize,
    pub estimated_duration: Duration,
    pub actual_start_time: Option<Instant>,
}

impl VolumetricCompensator {
    pub fn new(config: VolumetricCompensationConfig) -> Self {
        Self {
            config,
            active_map: None,
            probe_sequence: None,
            current_temperature: 20.0,
            machine_state: MachineGeometry::default(),
        }
    }

    /// Generate comprehensive probing sequence for volumetric calibration
    pub fn generate_probe_sequence(&self) -> Result<ProbeSequence, CompensationError> {
        let mut probe_points = Vec::new();

        // Calculate grid dimensions
        let x_points = (self.config.build_volume[0] / self.config.probe_resolution) as i32 + 1;
        let y_points = (self.config.build_volume[1] / self.config.probe_resolution) as i32 + 1;
        let z_points = (self.config.build_volume[2] / self.config.probe_resolution) as i32 + 1;

        println!("🗺️ Generating volumetric probe sequence: {}x{}x{} grid", x_points, y_points, z_points);

        // Generate 3D grid of probe points
        for z_idx in 0..z_points {
            for y_idx in 0..y_points {
                for x_idx in 0..x_points {
                    let x = x_idx as f32 * self.config.probe_resolution;
                    let y = y_idx as f32 * self.config.probe_resolution;
                    let z = z_idx as f32 * self.config.probe_resolution;

                    // Skip points outside build volume
                    if x <= self.config.build_volume[0] &&
                       y <= self.config.build_volume[1] &&
                       z <= self.config.build_volume[2] {
                        probe_points.push(Point3::new(x, y, z));
                    }
                }
            }
        }

        // Optimize probe order to minimize travel time
        probe_points = self.optimize_probe_order(probe_points);

        // Calculate estimated duration
        let total_distance = self.calculate_total_probe_distance(&probe_points);
        let probe_time_per_point = Duration::from_secs_f32(
            (self.config.probe_retract_distance * 2.0) / (self.config.probe_speed / 60.0) +
            self.config.probe_samples as f32 * 0.5
        );
        let estimated_duration = Duration::from_secs_f32(total_distance / (self.config.probe_speed / 60.0)) +
                                probe_time_per_point * probe_points.len() as u32;

        println!("📊 Probe sequence: {} points, estimated time: {:.1} minutes",
                probe_points.len(), estimated_duration.as_secs_f32() / 60.0);

        Ok(ProbeSequence {
            probe_points,
            current_index: 0,
            estimated_duration,
            actual_start_time: None,
        })
    }

    fn optimize_probe_order(&self, mut points: Vec<Point3<f32>>) -> Vec<Point3<f32>> {
        if points.len() <= 1 {
            return points;
        }

        // Use nearest neighbor algorithm with some optimizations
        let mut optimized = Vec::with_capacity(points.len());
        let mut current_pos = Point3::new(0.0, 0.0, 0.0); // Start from home

        while !points.is_empty() {
            let mut nearest_idx = 0;
            let mut nearest_dist = f32::INFINITY;

            for (i, point) in points.iter().enumerate() {
                let dist = (point - current_pos).norm();
                if dist < nearest_dist {
                    nearest_dist = dist;
                    nearest_idx = i;
                }
            }

            current_pos = points.remove(nearest_idx);
            optimized.push(current_pos);
        }

        optimized
    }

    fn calculate_total_probe_distance(&self, points: &[Point3<f32>]) -> f32 {
        if points.len() <= 1 {
            return 0.0;
        }

        let mut total_distance = (points[0] - Point3::new(0.0, 0.0, 0.0)).norm(); // From home to first point

        for i in 1..points.len() {
            total_distance += (points[i] - points[i-1]).norm();
        }

        total_distance
    }

    /// Start volumetric calibration sequence
    pub fn start_calibration(&mut self) -> Result<Vec<String>, CompensationError> {
        let sequence = self.generate_probe_sequence()?;
        // Measure the raw machine. Moves corrected by an old map would measure that map's
        // errors, and new results must not merge into it.
        self.active_map = None;
        let mut gcode_commands = Vec::new();

        // Initialization commands
        gcode_commands.push("G21".to_string());    // Metric units
        gcode_commands.push("G90".to_string());    // Absolute positioning
        gcode_commands.push("G28".to_string());    // Home all axes
        gcode_commands.push(format!("G1 F{:.0}", self.config.probe_speed)); // Set probe feedrate

        // Temperature stabilization if enabled
        if self.config.temperature_compensation {
            gcode_commands.push("M105".to_string()); // Report temperatures
            gcode_commands.push("M400".to_string()); // Wait for moves to complete
        }

        // Generate probe commands for each point
        for (i, point) in sequence.probe_points.iter().enumerate() {
            // Move to XY position above probe point
            gcode_commands.push(format!("G1 X{:.3} Y{:.3} Z{:.3}",
                                      point.x, point.y, point.z + self.config.probe_retract_distance));

            // Probe downward with multiple samples
            for sample in 0..self.config.probe_samples {
                gcode_commands.push(format!("G30 S{} ; Probe point {} sample {}",
                                          i * self.config.probe_samples as usize + sample as usize,
                                          i, sample));
                gcode_commands.push(format!("G1 Z{:.3}", point.z + self.config.probe_retract_distance)); // Retract
            }

            // Progress update every 10 points
            if i % 10 == 0 {
                gcode_commands.push(format!("M117 Probing {}/{}", i + 1, sequence.probe_points.len()));
            }
        }

        // Return to home position
        gcode_commands.push("G28".to_string());
        gcode_commands.push("M117 Volumetric calibration complete".to_string());

        self.probe_sequence = Some(sequence);
        Ok(gcode_commands)
    }

    /// Process probe result and build compensation map
    pub fn process_probe_result(&mut self, commanded_pos: Point3<f32>, actual_pos: Point3<f32>, temperature: f32) -> Result<(), CompensationError> {
        let error_vector = actual_pos - commanded_pos;

        // Check if deviation is within acceptable limits
        if error_vector.norm() > self.config.max_deviation {
            return Err(CompensationError::ExcessiveDeviation {
                point: commanded_pos,
                deviation: error_vector.norm(),
                max_allowed: self.config.max_deviation,
            });
        }

        let probe_point = ProbePoint {
            commanded_position: commanded_pos,
            actual_position: actual_pos,
            error_vector,
            temperature,
            timestamp: std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_secs(),
            confidence: self.calculate_measurement_confidence(&error_vector),
        };

        // Add to active map or create new one
        if let Some(ref mut map) = self.active_map {
            map.probe_points.push(probe_point);
        } else {
            let mut new_map = CompensationMap {
                grid_resolution: self.config.probe_resolution,
                build_volume: self.config.build_volume,
                probe_points: vec![probe_point],
                grid_points: HashMap::new(),
                temperature_coefficients: None,
                creation_timestamp: std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_secs(),
                calibration_id: format!("vol_cal_{}", chrono::Utc::now().format("%Y%m%d_%H%M%S")),
            };

            self.active_map = Some(new_map);
        }

        Ok(())
    }

    fn calculate_measurement_confidence(&self, error_vector: &Vector3<f32>) -> f32 {
        // Confidence decreases with larger errors
        let error_magnitude = error_vector.norm();
        let max_error = self.config.max_deviation;

        if error_magnitude <= max_error * 0.1 {
            1.0 // High confidence for small errors
        } else if error_magnitude <= max_error * 0.5 {
            0.8 // Medium confidence
        } else {
            0.5 // Low confidence for large errors
        }
    }

    /// Finalize calibration and generate compensation grid
    pub fn finalize_calibration(&mut self) -> Result<CompensationMap, CompensationError> {
        // Work on a copy. A failure here must not throw away hours of probing.
        let map = self.active_map.clone().ok_or(CompensationError::NoCalibrationData)?;

        if map.probe_points.len() < 8 {
            return Err(CompensationError::InsufficientData);
        }

        println!("🧮 Finalizing volumetric compensation with {} probe points", map.probe_points.len());

        let mut finalized_map = map;

        // Generate interpolated grid
        self.generate_compensation_grid(&mut finalized_map)?;

        // Calculate thermal compensation coefficients if enabled
        // All probing at one temperature is normal: it just means no thermal model.
        if self.config.temperature_compensation {
            finalized_map.temperature_coefficients = match self.calculate_thermal_coefficients(&finalized_map.probe_points) {
                Ok(coefficients) => Some(coefficients),
                Err(CompensationError::InsufficientThermalData) => {
                    println!("⚠️ Not enough temperature variation for thermal compensation, leaving it off");
                    None
                }
                Err(e) => return Err(e),
            };
        }

        // Validate the compensation map
        self.validate_compensation_map(&finalized_map)?;

        self.active_map = Some(finalized_map.clone());

        println!("✅ Volumetric compensation map generated with {} grid points", finalized_map.grid_points.len());

        Ok(finalized_map)
    }

    fn generate_compensation_grid(&self, map: &mut CompensationMap) -> Result<(), CompensationError> {
        // Create a 3D grid and interpolate correction vectors
        let x_steps = (self.config.build_volume[0] / self.config.probe_resolution) as i32 + 1;
        let y_steps = (self.config.build_volume[1] / self.config.probe_resolution) as i32 + 1;
        let z_steps = (self.config.build_volume[2] / self.config.probe_resolution) as i32 + 1;

        for x_idx in 0..x_steps {
            for y_idx in 0..y_steps {
                for z_idx in 0..z_steps {
                    let grid_pos = Point3::new(
                        x_idx as f32 * self.config.probe_resolution,
                        y_idx as f32 * self.config.probe_resolution,
                        z_idx as f32 * self.config.probe_resolution,
                    );

                    // Interpolate correction vector using inverse distance weighting
                    let correction = self.interpolate_correction(&map.probe_points, grid_pos);
                    map.grid_points.insert((x_idx, y_idx, z_idx), correction);
                }
            }
        }

        Ok(())
    }

    fn interpolate_correction(&self, probe_points: &[ProbePoint], target_pos: Point3<f32>) -> Vector3<f32> {
        // Inverse distance weighting interpolation
        let mut weighted_sum = Vector3::zeros();
        let mut weight_sum = 0.0f32;

        for probe_point in probe_points {
            let distance = (probe_point.commanded_position - target_pos).norm();

            if distance < 0.001 {
                // Very close point, use its value directly
                return probe_point.error_vector;
            }

            let weight = probe_point.confidence / (distance * distance);
            weighted_sum += probe_point.error_vector * weight;
            weight_sum += weight;
        }

        if weight_sum > 0.0 {
            weighted_sum / weight_sum
        } else {
            Vector3::zeros()
        }
    }

    fn calculate_thermal_coefficients(&self, probe_points: &[ProbePoint]) -> Result<ThermalCompensationCoeffs, CompensationError> {
        // Group measurements by temperature
        let mut temp_groups: HashMap<i32, Vec<&ProbePoint>> = HashMap::new();

        for point in probe_points {
            // Group by whole degrees. Sensor noise would make 0.1°C groups meaningless.
            let temp_key = point.temperature.round() as i32 * 10;
            temp_groups.entry(temp_key).or_insert_with(Vec::new).push(point);
        }

        if temp_groups.len() < 2 {
            return Err(CompensationError::InsufficientThermalData);
        }

        // Calculate linear expansion coefficients by analyzing position errors vs temperature
        let mut linear_expansion = [0.0f32; 3];

        // For each axis, find correlation between temperature and position error
        for axis in 0..3 {
            let mut temp_error_pairs = Vec::new();

            for (temp_key, points) in &temp_groups {
                let temp = *temp_key as f32 / 10.0;
                let avg_error: f32 = points.iter().map(|p| p.error_vector[axis]).sum::<f32>() / points.len() as f32;
                temp_error_pairs.push((temp, avg_error));
            }

            // Simple linear regression to find temperature coefficient
            if temp_error_pairs.len() >= 2 {
                let (slope, _intercept) = self.linear_regression(&temp_error_pairs);
                linear_expansion[axis] = slope;
            }
        }

        Ok(ThermalCompensationCoeffs {
            linear_expansion,
            angular_distortion: [0.0; 3], // Would require more sophisticated analysis
            frame_expansion: Matrix3::identity(),
            hotend_offset_temp_coeff: Vector3::zeros(),
        })
    }

    fn linear_regression(&self, data: &[(f32, f32)]) -> (f32, f32) {
        let n = data.len() as f32;
        let sum_x: f32 = data.iter().map(|(x, _)| x).sum();
        let sum_y: f32 = data.iter().map(|(_, y)| y).sum();
        let sum_xy: f32 = data.iter().map(|(x, y)| x * y).sum();
        let sum_x2: f32 = data.iter().map(|(x, _)| x * x).sum();

        let slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
        let intercept = (sum_y - slope * sum_x) / n;

        (slope, intercept)
    }

    fn validate_compensation_map(&self, map: &CompensationMap) -> Result<(), CompensationError> {
        // Check for excessive corrections that might indicate measurement errors
        let max_correction = map.grid_points.values()
            .map(|v| v.norm())
            .fold(0.0f32, f32::max);

        if max_correction > self.config.max_deviation {
            return Err(CompensationError::ExcessiveDeviation {
                point: Point3::origin(),
                deviation: max_correction,
                max_allowed: self.config.max_deviation,
            });
        }

        // Check for sufficient coverage
        if map.grid_points.len() < 8 {
            return Err(CompensationError::InsufficientData);
        }

        println!("✓ Compensation map validated: max correction {:.3}mm", max_correction);
        Ok(())
    }

    /// Apply volumetric compensation to a commanded position
    pub fn apply_compensation(&self, commanded_pos: Point3<f32>) -> Point3<f32> {
        let Some(ref map) = self.active_map else {
            return commanded_pos; // No compensation if no map loaded
        };

        // Still collecting probe points: no finished map to use, and correcting while
        // measuring would corrupt the measurements.
        if map.grid_points.is_empty() {
            return commanded_pos;
        }

        // Expected error here, blended smoothly between grid points. Snapping to the nearest
        // point would put a visible step in the print at every cell boundary.
        let error = self.interpolate_grid(map, commanded_pos);

        // Temperature drift: the regression slope is already mm of error per °C
        let thermal_error = if let Some(ref thermal) = map.temperature_coefficients {
            let temp_diff = self.current_temperature - self.config.reference_temperature;
            Vector3::new(
                thermal.linear_expansion[0] * temp_diff,
                thermal.linear_expansion[1] * temp_diff,
                thermal.linear_expansion[2] * temp_diff,
            )
        } else {
            Vector3::zeros()
        };

        // The error is where the machine went minus where it was told. To land on target it
        // must be told the opposite. Adding it would double every error instead of removing it.
        commanded_pos - error - thermal_error
    }

    /// Trilinear interpolation of the correction grid, holding the edge values outside it
    fn interpolate_grid(&self, map: &CompensationMap, position: Point3<f32>) -> Vector3<f32> {
        let cell = |value: f32, size: f32| -> (i32, f32) {
            let scaled = (value / map.grid_resolution).clamp(0.0, size / map.grid_resolution);
            let index = scaled.floor();
            (index as i32, scaled - index)
        };
        let (x0, fx) = cell(position.x, map.build_volume[0]);
        let (y0, fy) = cell(position.y, map.build_volume[1]);
        let (z0, fz) = cell(position.z, map.build_volume[2]);

        let mut result = Vector3::zeros();
        let mut total_weight = 0.0;
        for (dx, wx) in [(0, 1.0 - fx), (1, fx)] {
            for (dy, wy) in [(0, 1.0 - fy), (1, fy)] {
                for (dz, wz) in [(0, 1.0 - fz), (1, fz)] {
                    let weight = wx * wy * wz;
                    if weight > 0.0 {
                        if let Some(error) = map.grid_points.get(&(x0 + dx, y0 + dy, z0 + dz)) {
                            result += *error * weight;
                            total_weight += weight;
                        }
                    }
                }
            }
        }
        if total_weight > 0.0 {
            result / total_weight
        } else {
            // A map with gaps: fall back to the raw measurements
            self.interpolate_correction(&map.probe_points, position)
        }
    }

    pub fn set_current_temperature(&mut self, temperature: f32) {
        self.current_temperature = temperature;
    }

    pub fn load_compensation_map(&mut self, map: CompensationMap) -> Result<(), CompensationError> {
        self.validate_compensation_map(&map)?;
        self.active_map = Some(map);
        println!("📋 Volumetric compensation map loaded");
        Ok(())
    }

    pub fn get_compensation_status(&self) -> CompensationStatus {
        CompensationStatus {
            is_active: self.active_map.is_some(),
            current_temperature: self.current_temperature,
            map_info: self.active_map.as_ref().map(|m| CompensationMapInfo {
                calibration_id: m.calibration_id.clone(),
                probe_points: m.probe_points.len(),
                grid_points: m.grid_points.len(),
                has_thermal_compensation: m.temperature_coefficients.is_some(),
                creation_timestamp: m.creation_timestamp,
            }),
        }
    }
}

#[derive(Debug, Clone)]
pub struct CompensationStatus {
    pub is_active: bool,
    pub current_temperature: f32,
    pub map_info: Option<CompensationMapInfo>,
}

#[derive(Debug, Clone)]
pub struct CompensationMapInfo {
    pub calibration_id: String,
    pub probe_points: usize,
    pub grid_points: usize,
    pub has_thermal_compensation: bool,
    pub creation_timestamp: u64,
}

#[derive(Debug, Clone)]
pub enum CompensationError {
    NoCalibrationData,
    InsufficientData,
    InsufficientThermalData,
    ExcessiveDeviation {
        point: Point3<f32>,
        deviation: f32,
        max_allowed: f32,
    },
    InvalidConfiguration,
    ProbeError(String),
}

impl std::fmt::Display for CompensationError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            CompensationError::NoCalibrationData => write!(f, "No calibration data available"),
            CompensationError::InsufficientData => write!(f, "Insufficient probe data for compensation"),
            CompensationError::InsufficientThermalData => write!(f, "Insufficient thermal data for temperature compensation"),
            CompensationError::ExcessiveDeviation { point, deviation, max_allowed } => {
                write!(f, "Excessive deviation at {:?}: {:.3}mm (max: {:.3}mm)", point, deviation, max_allowed)
            },
            CompensationError::InvalidConfiguration => write!(f, "Invalid compensation configuration"),
            CompensationError::ProbeError(msg) => write!(f, "Probe error: {}", msg),
        }
    }
}

impl std::error::Error for CompensationError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compensation_interpolation() {
        let mut compensator = VolumetricCompensator::new(VolumetricCompensationConfig {
            build_volume: [100.0, 100.0, 100.0],
            probe_resolution: 100.0,
            temperature_compensation: false,
            ..VolumetricCompensationConfig::default()
        });

        // The machine overshoots by +0.1 in X everywhere, so it must be told 0.1 less.
        for x in [0.0f32, 100.0] {
            for y in [0.0f32, 100.0] {
                for z in [0.0f32, 100.0] {
                    let commanded = Point3::new(x, y, z);
                    compensator.process_probe_result(commanded, commanded + Vector3::new(0.1, 0.0, 0.0), 20.0).unwrap();
                }
            }
        }
        let mid = Point3::new(50.0, 50.0, 50.0);
        assert_eq!(compensator.apply_compensation(mid), mid, "no correction until finalized");
        compensator.finalize_calibration().unwrap();
        let corrected = compensator.apply_compensation(mid);
        assert!((corrected.x - 49.9).abs() < 1e-3, "{:?}", corrected);
        assert!((corrected.y - 50.0).abs() < 1e-3, "{:?}", corrected);

        // Too little data keeps what was probed
        let mut short = VolumetricCompensator::new(VolumetricCompensationConfig::default());
        short.process_probe_result(mid, mid, 20.0).unwrap();
        assert!(short.finalize_calibration().is_err());
        assert!(short.finalize_calibration().is_err(), "second attempt sees the same data, not none");
        assert!(matches!(short.finalize_calibration(), Err(CompensationError::InsufficientData)));

        let probe_points = vec![
            ProbePoint {
                commanded_position: Point3::new(0.0, 0.0, 0.0),
                actual_position: Point3::new(0.1, 0.0, 0.0),
                error_vector: Vector3::new(0.1, 0.0, 0.0),
                temperature: 20.0,
                timestamp: 0,
                confidence: 1.0,
            },
            ProbePoint {
                commanded_position: Point3::new(10.0, 0.0, 0.0),
                actual_position: Point3::new(10.2, 0.0, 0.0),
                error_vector: Vector3::new(0.2, 0.0, 0.0),
                temperature: 20.0,
                timestamp: 0,
                confidence: 1.0,
            },
        ];

        let interpolated = compensator.interpolate_correction(&probe_points, Point3::new(5.0, 0.0, 0.0));

        // Should be somewhere between 0.1 and 0.2
        assert!(interpolated.x > 0.1 && interpolated.x < 0.2);
    }

    #[test]
    fn test_probe_sequence_generation() {
        let config = VolumetricCompensationConfig {
            probe_resolution: 50.0,
            build_volume: [100.0, 100.0, 100.0],
            ..Default::default()
        };

        let compensator = VolumetricCompensator::new(config);
        let sequence = compensator.generate_probe_sequence().unwrap();

        // Should have 3x3x3 = 27 points for 100mm cube with 50mm resolution
        assert_eq!(sequence.probe_points.len(), 27);
        assert!(sequence.estimated_duration.as_secs() > 0);
    }
}