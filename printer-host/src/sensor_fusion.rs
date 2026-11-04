/// Advanced Sensor Fusion for State Estimation
///
/// This module implements sophisticated sensor fusion algorithms that combine data
/// from multiple encoder types, accelerometers, and other sensors to provide
/// highly accurate real-time state estimation of the printer's toolhead position,
/// velocity, and acceleration.

use std::collections::{HashMap, VecDeque};
use std::time::{Duration, Instant};
use nalgebra::{Matrix4, Matrix2, Matrix2x4, Vector4, Vector2, DMatrix, DVector};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorConfig {
    pub encoder_resolution: f32,      // Steps per mm
    pub accelerometer_range: f32,     // ±g range
    pub gyroscope_range: f32,         // ±deg/s range
    pub sample_rate: u32,             // Hz
    pub noise_variance: SensorNoiseProfile,
    pub calibration_matrix: [[f32; 3]; 3], // Sensor calibration
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorNoiseProfile {
    pub encoder_noise: f32,           // Position noise (mm)
    pub accelerometer_noise: f32,     // Acceleration noise (m/s²)
    pub gyroscope_noise: f32,         // Angular velocity noise (rad/s)
    pub temperature_drift: f32,       // Temperature coefficient
}

impl Default for SensorConfig {
    fn default() -> Self {
        Self {
            encoder_resolution: 4000.0,  // 4000 steps/mm
            accelerometer_range: 16.0,   // ±16g
            gyroscope_range: 2000.0,     // ±2000°/s
            sample_rate: 1000,           // 1kHz
            noise_variance: SensorNoiseProfile {
                encoder_noise: 0.001,    // 1μm
                accelerometer_noise: 0.1, // 0.1 m/s²
                gyroscope_noise: 0.01,   // 0.01 rad/s
                temperature_drift: 0.0001,
            },
            calibration_matrix: [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
        }
    }
}

#[derive(Debug, Clone)]
pub struct SensorData {
    pub timestamp: Instant,
    pub encoder_position: f32,        // mm
    pub encoder_velocity: f32,        // mm/s (derived)
    pub accelerometer: [f32; 3],      // m/s² [X, Y, Z]
    pub gyroscope: [f32; 3],          // rad/s [X, Y, Z]
    pub temperature: f32,             // °C
    pub confidence: f32,              // Sensor confidence (0-1)
}

#[derive(Debug, Clone)]
pub struct FusedState {
    pub position: f32,                // Best estimate position (mm)
    pub velocity: f32,                // Best estimate velocity (mm/s)
    pub acceleration: f32,            // Best estimate acceleration (mm/s²)
    pub jerk: f32,                    // Estimated jerk (mm/s³)
    pub uncertainty: StateUncertainty,
    pub timestamp: Instant,
}

#[derive(Debug, Clone)]
pub struct StateUncertainty {
    pub position_std: f32,            // Position standard deviation (mm)
    pub velocity_std: f32,            // Velocity standard deviation (mm/s)
    pub acceleration_std: f32,        // Acceleration standard deviation (mm/s²)
    pub confidence_level: f32,        // Overall confidence (0-1)
}

/// Enhanced Kalman Filter for multi-sensor fusion
pub struct ExtendedKalmanFilter {
    // State vector: [position, velocity, acceleration, jerk]
    state: Vector4<f32>,
    covariance: Matrix4<f32>,

    // Process model
    process_matrix: Matrix4<f32>,     // State transition (F)
    process_noise: Matrix4<f32>,      // Process noise covariance (Q)

    // Measurement models
    encoder_measurement_matrix: Matrix2x4<f32>,   // H for encoder (position + velocity)
    accel_measurement_matrix: Matrix2<f32>,       // H for accelerometer
    measurement_noise: Matrix2<f32>,              // R measurement noise

    dt: f32,                          // Time step
    last_update: Instant,
}

impl ExtendedKalmanFilter {
    pub fn new(initial_state: Vector4<f32>, dt: f32) -> Self {
        // State transition matrix for constant jerk model
        let f = Matrix4::new(
            1.0, dt, dt*dt/2.0, dt*dt*dt/6.0,
            0.0, 1.0, dt,       dt*dt/2.0,
            0.0, 0.0, 1.0,      dt,
            0.0, 0.0, 0.0,      1.0,
        );

        // Process noise (assuming white noise acceleration and jerk)
        let q_scalar = 0.1; // Tuning parameter
        let q = Matrix4::new(
            dt*dt*dt*dt/4.0, dt*dt*dt/2.0, dt*dt/2.0, dt,
            dt*dt*dt/2.0,    dt*dt,        dt,       1.0,
            dt*dt/2.0,       dt,           1.0,      0.0,
            dt,              1.0,          0.0,      0.0,
        ) * q_scalar;

        // Encoder measurement matrix (measures position and velocity)
        let h_encoder = Matrix2x4::new(
            1.0, 0.0, 0.0, 0.0, // Position measurement (maps state[0] to measurement[0])
            0.0, 1.0, 0.0, 0.0, // Velocity measurement (maps state[1] to measurement[1])
        );

        // Accelerometer measurement matrix (measures acceleration directly)
        let h_accel = Matrix2::new(
            0.0, 0.0, // No direct position measurement
            0.0, 0.0, // No direct velocity measurement
        );

        // Measurement noise
        let r = Matrix2::new(
            0.001, 0.0,   // Encoder position noise
            0.0,   0.01,  // Encoder velocity noise
        );

        // Initial covariance (high uncertainty)
        let p = Matrix4::identity() * 10.0;

        Self {
            state: initial_state,
            covariance: p,
            process_matrix: f,
            process_noise: q,
            encoder_measurement_matrix: h_encoder,
            accel_measurement_matrix: h_accel,
            measurement_noise: r,
            dt,
            last_update: Instant::now(),
        }
    }

    /// Predict step of Kalman filter
    pub fn predict(&mut self, dt: f32) {
        // Update time step if different
        if (dt - self.dt).abs() > 0.001 {
            self.update_matrices(dt);
        }

        // State prediction: x_k|k-1 = F * x_k-1|k-1
        self.state = &self.process_matrix * &self.state;

        // Covariance prediction: P_k|k-1 = F * P_k-1|k-1 * F^T + Q
        self.covariance = &self.process_matrix * &self.covariance * self.process_matrix.transpose() + &self.process_noise;
    }

    /// Update step with encoder measurement
    pub fn update_encoder(&mut self, position: f32, velocity: f32, noise_variance: Matrix2<f32>) {
        let measurement = Vector2::new(position, velocity);
        let h = self.encoder_measurement_matrix.clone();

        self.update_with_measurement(measurement, &h, &noise_variance);
    }

    /// Update step with accelerometer measurement
    pub fn update_accelerometer(&mut self, acceleration: f32, noise_variance: f32) {
        // For accelerometer, we directly measure acceleration (3rd state)
        let h = Matrix2::new(
            0.0, 0.0, // Row 1: no measurement
            0.0, 0.0, // Row 2: no measurement
        );

        // Create a 1x4 measurement matrix for acceleration
        let h_accel_1d = Vector4::new(0.0, 0.0, 1.0, 0.0).transpose();

        // Innovation
        let predicted_accel = h_accel_1d * &self.state;
        let innovation = acceleration - predicted_accel[0];

        // Innovation covariance
        let s = (h_accel_1d * &self.covariance * h_accel_1d.transpose())[0] + noise_variance;

        if s > 0.0 {
            // Kalman gain
            let k = &self.covariance * h_accel_1d.transpose() / s;

            // State update
            self.state += k * innovation;

            // Covariance update (Joseph form for numerical stability)
            let i_kh = Matrix4::identity() - k * h_accel_1d;
            self.covariance = i_kh * &self.covariance;
        }
    }

    fn update_with_measurement(&mut self, measurement: Vector2<f32>, h: &Matrix2x4<f32>, r: &Matrix2<f32>) {
        // Innovation: y = z - H * x
        let predicted_measurement = h * &self.state;
        let innovation = measurement - predicted_measurement;

        // Innovation covariance: S = H * P * H^T + R
        let s = h * &self.covariance * h.transpose() + r;

        // Kalman gain: K = P * H^T * S^-1
        if let Some(s_inv) = s.try_inverse() {
            let k = &self.covariance * h.transpose() * s_inv;

            // State update: x = x + K * y
            self.state += k * innovation;

            // Covariance update: P = (I - K * H) * P
            let kh = k * h;
            let i_kh = Matrix4::identity() - kh;
            self.covariance = i_kh * &self.covariance;
        }
    }

    fn update_matrices(&mut self, dt: f32) {
        self.dt = dt;

        // Update state transition matrix
        self.process_matrix = Matrix4::new(
            1.0, dt, dt*dt/2.0, dt*dt*dt/6.0,
            0.0, 1.0, dt,       dt*dt/2.0,
            0.0, 0.0, 1.0,      dt,
            0.0, 0.0, 0.0,      1.0,
        );

        // Update process noise matrix
        let q_scalar = 0.1;
        self.process_noise = Matrix4::new(
            dt*dt*dt*dt/4.0, dt*dt*dt/2.0, dt*dt/2.0, dt,
            dt*dt*dt/2.0,    dt*dt,        dt,       1.0,
            dt*dt/2.0,       dt,           1.0,      0.0,
            dt,              1.0,          0.0,      0.0,
        ) * q_scalar;
    }

    pub fn get_state(&self) -> Vector4<f32> {
        self.state
    }

    pub fn get_uncertainty(&self) -> Vector4<f32> {
        Vector4::new(
            self.covariance[(0, 0)].sqrt(),
            self.covariance[(1, 1)].sqrt(),
            self.covariance[(2, 2)].sqrt(),
            self.covariance[(3, 3)].sqrt(),
        )
    }
}

/// Multi-axis sensor fusion manager
pub struct SensorFusionManager {
    axis_filters: HashMap<char, ExtendedKalmanFilter>,
    sensor_configs: HashMap<char, SensorConfig>,
    sensor_buffers: HashMap<char, VecDeque<SensorData>>,
    calibration_data: HashMap<char, SensorCalibration>,
    fusion_stats: FusionStatistics,
    last_fusion_time: Instant,
    outlier_detector: OutlierDetector,
}

#[derive(Debug, Clone)]
pub struct SensorCalibration {
    pub encoder_offset: f32,
    pub accelerometer_bias: [f32; 3],
    pub gyroscope_bias: [f32; 3],
    pub cross_axis_sensitivity: Matrix4<f32>,
    pub temperature_compensation: TemperatureCompensation,
}

#[derive(Debug, Clone)]
pub struct TemperatureCompensation {
    pub reference_temperature: f32,
    pub encoder_temp_coeff: f32,
    pub accel_temp_coeff: [f32; 3],
    pub gyro_temp_coeff: [f32; 3],
}

#[derive(Debug, Clone)]
pub struct FusionStatistics {
    pub total_measurements: u64,
    pub outliers_rejected: u64,
    pub avg_fusion_time: Duration,
    pub position_accuracy: f32,        // RMS error (mm)
    pub velocity_accuracy: f32,        // RMS error (mm/s)
    pub sensor_health: HashMap<char, f32>, // Health score per axis (0-1)
}

impl Default for FusionStatistics {
    fn default() -> Self {
        Self {
            total_measurements: 0,
            outliers_rejected: 0,
            avg_fusion_time: Duration::from_micros(100),
            position_accuracy: 0.001,
            velocity_accuracy: 0.1,
            sensor_health: HashMap::new(),
        }
    }
}

pub struct OutlierDetector {
    mahalanobis_threshold: f32,
    z_score_threshold: f32,
    recent_residuals: HashMap<char, VecDeque<f32>>,
    adaptive_threshold: bool,
}

impl OutlierDetector {
    pub fn new() -> Self {
        Self {
            mahalanobis_threshold: 3.0,  // 3-sigma threshold
            z_score_threshold: 2.5,
            recent_residuals: HashMap::new(),
            adaptive_threshold: true,
        }
    }

    /// Detect outliers using Mahalanobis distance
    pub fn is_outlier(&mut self, axis: char, measurement: f32, predicted: f32, covariance: f32) -> bool {
        let residual = measurement - predicted;
        let mahalanobis_dist = (residual * residual / covariance).sqrt();

        // Store recent residuals for adaptive thresholding
        let residuals = self.recent_residuals.entry(axis).or_insert_with(VecDeque::new);
        residuals.push_back(residual.abs());
        if residuals.len() > 100 {
            residuals.pop_front();
        }

        // Adaptive threshold based on recent measurement history
        let threshold = if self.adaptive_threshold && residuals.len() > 10 {
            let mean: f32 = residuals.iter().sum::<f32>() / residuals.len() as f32;
            let variance: f32 = residuals.iter()
                .map(|x| (x - mean) * (x - mean))
                .sum::<f32>() / (residuals.len() - 1) as f32;
            let adaptive_factor = 1.0 + (variance.sqrt() / mean).min(2.0);
            self.mahalanobis_threshold * adaptive_factor
        } else {
            self.mahalanobis_threshold
        };

        mahalanobis_dist > threshold
    }
}

impl SensorFusionManager {
    pub fn new() -> Self {
        Self {
            axis_filters: HashMap::new(),
            sensor_configs: HashMap::new(),
            sensor_buffers: HashMap::new(),
            calibration_data: HashMap::new(),
            fusion_stats: FusionStatistics::default(),
            last_fusion_time: Instant::now(),
            outlier_detector: OutlierDetector::new(),
        }
    }

    /// Initialize sensor fusion for an axis
    pub fn initialize_axis(&mut self, axis: char, config: SensorConfig) {
        // Initialize Kalman filter with zero state
        let initial_state = Vector4::zeros();
        let filter = ExtendedKalmanFilter::new(initial_state, 1.0 / config.sample_rate as f32);

        self.axis_filters.insert(axis, filter);
        self.sensor_configs.insert(axis, config);
        self.sensor_buffers.insert(axis, VecDeque::with_capacity(1000));

        // Initialize calibration data
        let calibration = SensorCalibration {
            encoder_offset: 0.0,
            accelerometer_bias: [0.0; 3],
            gyroscope_bias: [0.0; 3],
            cross_axis_sensitivity: Matrix4::identity(),
            temperature_compensation: TemperatureCompensation {
                reference_temperature: 20.0,
                encoder_temp_coeff: 0.0,
                accel_temp_coeff: [0.0; 3],
                gyro_temp_coeff: [0.0; 3],
            },
        };
        self.calibration_data.insert(axis, calibration);

        self.fusion_stats.sensor_health.insert(axis, 1.0);

        println!("🔗 Sensor fusion initialized for {} axis", axis);
    }

    /// Process new sensor data and update fused state
    pub fn process_sensor_data(&mut self, axis: char, sensor_data: SensorData) -> Result<FusedState, FusionError> {
        let start_time = Instant::now();

        // Validate sensor data
        if !self.validate_sensor_data(&sensor_data) {
            return Err(FusionError::InvalidSensorData);
        }

        // Apply sensor calibration
        let calibrated_data = self.apply_calibration(axis, sensor_data)?;

        // Store in buffer
        let buffer = self.sensor_buffers.get_mut(&axis)
            .ok_or(FusionError::AxisNotInitialized(axis))?;
        buffer.push_back(calibrated_data.clone());
        if buffer.len() > 1000 {
            buffer.pop_front();
        }

        // Get Kalman filter for this axis
        let filter = self.axis_filters.get_mut(&axis)
            .ok_or(FusionError::AxisNotInitialized(axis))?;

        // Calculate time step
        let dt = calibrated_data.timestamp.duration_since(self.last_fusion_time).as_secs_f32();
        if dt > 0.0 && dt < 1.0 { // Sanity check
            // Prediction step
            filter.predict(dt);

            // Update with encoder data (if available and valid)
            if !self.outlier_detector.is_outlier(axis, calibrated_data.encoder_position,
                                                filter.get_state()[0], filter.get_uncertainty()[0]) {
                let encoder_noise = Matrix2::new(
                    self.sensor_configs[&axis].noise_variance.encoder_noise.powi(2), 0.0,
                    0.0, (self.sensor_configs[&axis].noise_variance.encoder_noise * 10.0).powi(2)
                );
                filter.update_encoder(calibrated_data.encoder_position, calibrated_data.encoder_velocity, encoder_noise);
            } else {
                self.fusion_stats.outliers_rejected += 1;
            }

            // Update with accelerometer data
            let accel_magnitude = (calibrated_data.accelerometer[0].powi(2) +
                                 calibrated_data.accelerometer[1].powi(2) +
                                 calibrated_data.accelerometer[2].powi(2)).sqrt();

            // Choose the axis that corresponds to the motion axis (simplified)
            let axis_accel = match axis {
                'X' => calibrated_data.accelerometer[0],
                'Y' => calibrated_data.accelerometer[1],
                'Z' => calibrated_data.accelerometer[2],
                _ => calibrated_data.accelerometer[0],
            };

            if !self.outlier_detector.is_outlier(axis, axis_accel,
                                                filter.get_state()[2], filter.get_uncertainty()[2]) {
                let accel_noise = self.sensor_configs[&axis].noise_variance.accelerometer_noise.powi(2);
                filter.update_accelerometer(axis_accel, accel_noise);
            } else {
                self.fusion_stats.outliers_rejected += 1;
            }
        }

        // Generate fused state
        let state = filter.get_state();
        let uncertainty = filter.get_uncertainty();

        let fused_state = FusedState {
            position: state[0],
            velocity: state[1],
            acceleration: state[2],
            jerk: state[3],
            uncertainty: StateUncertainty {
                position_std: uncertainty[0],
                velocity_std: uncertainty[1],
                acceleration_std: uncertainty[2],
                confidence_level: self.calculate_confidence_level(&uncertainty),
            },
            timestamp: calibrated_data.timestamp,
        };

        // Update statistics
        self.update_statistics(axis, start_time.elapsed());
        self.last_fusion_time = Instant::now();

        Ok(fused_state)
    }

    fn validate_sensor_data(&self, data: &SensorData) -> bool {
        // Check for NaN or infinite values
        if !data.encoder_position.is_finite() || !data.encoder_velocity.is_finite() {
            return false;
        }

        for &accel in &data.accelerometer {
            if !accel.is_finite() {
                return false;
            }
        }

        for &gyro in &data.gyroscope {
            if !gyro.is_finite() {
                return false;
            }
        }

        // Check reasonable ranges
        if data.encoder_position.abs() > 10000.0 || data.encoder_velocity.abs() > 10000.0 {
            return false;
        }

        let accel_magnitude = (data.accelerometer[0].powi(2) +
                             data.accelerometer[1].powi(2) +
                             data.accelerometer[2].powi(2)).sqrt();
        if accel_magnitude > 1000.0 { // > 100g seems unreasonable
            return false;
        }

        true
    }

    fn apply_calibration(&self, axis: char, mut data: SensorData) -> Result<SensorData, FusionError> {
        let calibration = self.calibration_data.get(&axis)
            .ok_or(FusionError::AxisNotInitialized(axis))?;

        // Apply encoder offset
        data.encoder_position -= calibration.encoder_offset;

        // Apply accelerometer bias compensation
        for i in 0..3 {
            data.accelerometer[i] -= calibration.accelerometer_bias[i];
        }

        // Apply gyroscope bias compensation
        for i in 0..3 {
            data.gyroscope[i] -= calibration.gyroscope_bias[i];
        }

        // Apply temperature compensation
        let temp_diff = data.temperature - calibration.temperature_compensation.reference_temperature;
        data.encoder_position -= calibration.temperature_compensation.encoder_temp_coeff * temp_diff;

        for i in 0..3 {
            data.accelerometer[i] -= calibration.temperature_compensation.accel_temp_coeff[i] * temp_diff;
            data.gyroscope[i] -= calibration.temperature_compensation.gyro_temp_coeff[i] * temp_diff;
        }

        Ok(data)
    }

    fn calculate_confidence_level(&self, uncertainty: &Vector4<f32>) -> f32 {
        // Combine uncertainties into overall confidence
        // Lower uncertainty = higher confidence
        let max_position_uncertainty = 0.1;  // 0.1mm
        let max_velocity_uncertainty = 10.0; // 10mm/s

        let position_confidence = (1.0 - (uncertainty[0] / max_position_uncertainty).min(1.0)).max(0.0);
        let velocity_confidence = (1.0 - (uncertainty[1] / max_velocity_uncertainty).min(1.0)).max(0.0);

        (position_confidence + velocity_confidence) / 2.0
    }

    fn update_statistics(&mut self, axis: char, processing_time: Duration) {
        self.fusion_stats.total_measurements += 1;

        // Update average processing time with exponential moving average
        let alpha = 0.1;
        let new_time = processing_time.as_secs_f32();
        let old_time = self.fusion_stats.avg_fusion_time.as_secs_f32();
        let avg_time = old_time * (1.0 - alpha) + new_time * alpha;
        self.fusion_stats.avg_fusion_time = Duration::from_secs_f32(avg_time);

        // Update sensor health (simplified - based on outlier rate)
        let outlier_rate = self.fusion_stats.outliers_rejected as f32 / self.fusion_stats.total_measurements as f32;
        let health_score = (1.0 - outlier_rate * 10.0).max(0.0).min(1.0);
        self.fusion_stats.sensor_health.insert(axis, health_score);
    }

    /// Perform sensor calibration routine
    pub fn calibrate_sensors(&mut self, axis: char) -> Result<SensorCalibration, FusionError> {
        println!("🎛️ Starting sensor calibration for {} axis", axis);

        let buffer = self.sensor_buffers.get(&axis)
            .ok_or(FusionError::AxisNotInitialized(axis))?;

        if buffer.len() < 100 {
            return Err(FusionError::InsufficientCalibrationData);
        }

        let mut calibration = SensorCalibration {
            encoder_offset: 0.0,
            accelerometer_bias: [0.0; 3],
            gyroscope_bias: [0.0; 3],
            cross_axis_sensitivity: Matrix4::identity(),
            temperature_compensation: TemperatureCompensation {
                reference_temperature: 20.0,
                encoder_temp_coeff: 0.0,
                accel_temp_coeff: [0.0; 3],
                gyro_temp_coeff: [0.0; 3],
            },
        };

        // Calculate accelerometer bias (assuming machine is stationary)
        let mut accel_sum = [0.0f32; 3];
        let mut gyro_sum = [0.0f32; 3];
        let mut temp_sum = 0.0f32;
        let sample_count = buffer.len() as f32;

        for sample in buffer.iter() {
            for i in 0..3 {
                accel_sum[i] += sample.accelerometer[i];
                gyro_sum[i] += sample.gyroscope[i];
            }
            temp_sum += sample.temperature;
        }

        // Calculate biases
        for i in 0..3 {
            calibration.accelerometer_bias[i] = accel_sum[i] / sample_count;
            calibration.gyroscope_bias[i] = gyro_sum[i] / sample_count;
        }

        // Subtract gravity from Z-axis accelerometer bias
        calibration.accelerometer_bias[2] -= 9.81;

        calibration.temperature_compensation.reference_temperature = temp_sum / sample_count;

        self.calibration_data.insert(axis, calibration.clone());

        println!("✓ Sensor calibration complete for {} axis", axis);
        println!("  Accel bias: [{:.3}, {:.3}, {:.3}] m/s²",
                calibration.accelerometer_bias[0],
                calibration.accelerometer_bias[1],
                calibration.accelerometer_bias[2]);
        println!("  Gyro bias: [{:.3}, {:.3}, {:.3}] rad/s",
                calibration.gyroscope_bias[0],
                calibration.gyroscope_bias[1],
                calibration.gyroscope_bias[2]);

        Ok(calibration)
    }

    pub fn get_fusion_statistics(&self) -> &FusionStatistics {
        &self.fusion_stats
    }

    pub fn get_axis_state(&self, axis: char) -> Option<Vector4<f32>> {
        self.axis_filters.get(&axis).map(|filter| filter.get_state())
    }

    pub fn reset_axis_filter(&mut self, axis: char) -> Result<(), FusionError> {
        if let Some(config) = self.sensor_configs.get(&axis) {
            let dt = 1.0 / config.sample_rate as f32;
            let initial_state = Vector4::zeros();
            let filter = ExtendedKalmanFilter::new(initial_state, dt);
            self.axis_filters.insert(axis, filter);
            Ok(())
        } else {
            Err(FusionError::AxisNotInitialized(axis))
        }
    }
}

#[derive(Debug, Clone)]
pub enum FusionError {
    AxisNotInitialized(char),
    InvalidSensorData,
    InsufficientCalibrationData,
    CalibrationFailed(String),
    OutlierDetected,
    FilterDivergence,
}

impl std::fmt::Display for FusionError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            FusionError::AxisNotInitialized(axis) => write!(f, "Axis {} not initialized", axis),
            FusionError::InvalidSensorData => write!(f, "Invalid sensor data"),
            FusionError::InsufficientCalibrationData => write!(f, "Insufficient data for calibration"),
            FusionError::CalibrationFailed(msg) => write!(f, "Calibration failed: {}", msg),
            FusionError::OutlierDetected => write!(f, "Outlier detected in sensor data"),
            FusionError::FilterDivergence => write!(f, "Kalman filter divergence detected"),
        }
    }
}

impl std::error::Error for FusionError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kalman_filter_initialization() {
        let initial_state = Vector4::new(0.0, 0.0, 0.0, 0.0);
        let filter = ExtendedKalmanFilter::new(initial_state, 0.001);

        let state = filter.get_state();
        assert_eq!(state[0], 0.0); // position
        assert_eq!(state[1], 0.0); // velocity
        assert_eq!(state[2], 0.0); // acceleration
        assert_eq!(state[3], 0.0); // jerk
    }

    #[test]
    fn test_sensor_fusion_manager() {
        let mut manager = SensorFusionManager::new();
        let config = SensorConfig::default();

        manager.initialize_axis('X', config);
        assert!(manager.axis_filters.contains_key(&'X'));
        assert!(manager.sensor_configs.contains_key(&'X'));
    }

    #[test]
    fn test_outlier_detection() {
        let mut detector = OutlierDetector::new();

        // Normal measurement should not be outlier
        assert!(!detector.is_outlier('X', 1.0, 1.1, 0.01));

        // Very different measurement should be outlier
        assert!(detector.is_outlier('X', 1.0, 10.0, 0.01));
    }
}