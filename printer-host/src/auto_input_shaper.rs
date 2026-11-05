/// Automated Input Shaper Calibration System
///
/// This module provides fully automated resonance testing and input shaper configuration.
/// It analyzes accelerometer data to determine optimal shaper parameters and presents
/// results to the user for confirmation before applying.

use std::collections::HashMap;
use std::time::{Duration, Instant};
use serde::{Deserialize, Serialize};

// Optional dependencies for advanced features
#[cfg(feature = "input-shaping")]
use rustfft::{FftPlanner, num_complex::Complex};

// UI dependencies (conditional for when building as library vs binary)
#[cfg(feature = "default")]
use eframe::egui;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResonanceTestConfig {
    pub test_axis: char,              // X, Y, Z axis to test
    pub frequency_range: (f32, f32),  // Min, Max frequency in Hz
    pub frequency_step: f32,          // Step size in Hz
    pub amplitude: f32,               // Test amplitude in mm
    pub duration_per_freq: Duration,  // Time to oscillate at each frequency
    pub settle_time: Duration,        // Time to wait between frequencies
    pub sample_rate: u32,             // Accelerometer sample rate in Hz
}

impl Default for ResonanceTestConfig {
    fn default() -> Self {
        Self {
            test_axis: 'X',
            frequency_range: (5.0, 200.0),
            frequency_step: 1.0,
            amplitude: 1.0,
            duration_per_freq: Duration::from_millis(2000),
            settle_time: Duration::from_millis(500),
            sample_rate: 1000,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InputShaperType {
    pub name: String,
    pub coefficients: Vec<f32>,       // Shaper impulse response coefficients
    pub delays: Vec<f32>,             // Time delays for each coefficient (seconds)
    pub vibration_tolerance: f32,     // Remaining vibration as fraction (0-1)
    pub smoothing_factor: f32,        // Smoothing applied to motion (0-1)
}

impl InputShaperType {
    /// Damping assumed when it has not been measured
    const TYPICAL_DAMPING: f32 = 0.1;

    /// Built from the one implementation of the impulse math in `input_shaping`, so
    /// calibration recommends the same shapers that actually run.
    fn from_shaper(name: &str, shaper_type: crate::input_shaping::ShaperType, frequency: f32, damping: f32, vibration_tolerance: f32, smoothing_factor: f32) -> Self {
        let shaper = crate::input_shaping::InputShaper::new(crate::input_shaping::InputShaperConfig {
            shaper_type,
            frequency: frequency as f64,
            damping_ratio: damping.clamp(0.0, 0.9) as f64,
            ..Default::default()
        });
        let impulses = shaper.impulses();
        Self {
            name: name.to_string(),
            coefficients: impulses.iter().map(|i| i.0 as f32).collect(),
            delays: impulses.iter().map(|i| i.1 as f32).collect(),
            vibration_tolerance,
            smoothing_factor,
        }
    }

    pub fn zv(frequency: f32) -> Self {
        Self::zv_damped(frequency, Self::TYPICAL_DAMPING)
    }

    pub fn zvd(frequency: f32) -> Self {
        Self::zvd_damped(frequency, Self::TYPICAL_DAMPING)
    }

    pub fn mzv(frequency: f32) -> Self {
        Self::mzv_damped(frequency, Self::TYPICAL_DAMPING)
    }

    pub fn ei(frequency: f32) -> Self {
        Self::ei_damped(frequency, Self::TYPICAL_DAMPING)
    }

    pub fn zv_damped(frequency: f32, damping: f32) -> Self {
        Self::from_shaper("ZV", crate::input_shaping::ShaperType::ZV, frequency, damping, 0.2, 0.1)
    }

    pub fn zvd_damped(frequency: f32, damping: f32) -> Self {
        Self::from_shaper("ZVD", crate::input_shaping::ShaperType::ZVD, frequency, damping, 0.05, 0.25)
    }

    pub fn mzv_damped(frequency: f32, damping: f32) -> Self {
        Self::from_shaper("MZV", crate::input_shaping::ShaperType::MZV, frequency, damping, 0.05, 0.15)
    }

    pub fn ei_damped(frequency: f32, damping: f32) -> Self {
        Self::from_shaper("EI", crate::input_shaping::ShaperType::EI, frequency, damping, 0.05, 0.25)
    }
}

/// The hardest a resonance test may shake the machine. At a fixed amplitude, acceleration
/// grows with frequency squared: 1mm at 200Hz would be 1,600,000 mm/s².
const MAX_TEST_ACCELERATION: f32 = 5000.0;

#[derive(Debug, Clone)]
pub struct ResonancePeak {
    pub frequency: f32,               // Peak frequency in Hz
    pub magnitude: f32,               // Peak magnitude
    pub q_factor: f32,                // Quality factor (sharpness)
    pub damping_ratio: f32,           // Damping ratio
    pub confidence: f32,              // Detection confidence (0-1)
}

#[derive(Debug, Clone)]
pub struct CalibrationResults {
    pub test_axis: char,
    pub resonance_peaks: Vec<ResonancePeak>,
    pub recommended_shapers: Vec<(InputShaperType, f32)>, // (shaper, score)
    pub frequency_response: Vec<(f32, f32)>,              // (freq, magnitude)
    pub raw_accelerometer_data: Vec<f32>,
    pub test_timestamp: Instant,
    pub test_duration: Duration,
}

#[derive(Debug, Clone)]
pub struct AutoInputShaperCalibrator {
    current_config: ResonanceTestConfig,
    accelerometer_buffer: Vec<f32>,
    frequency_response_cache: HashMap<char, Vec<(f32, f32)>>,
    last_calibration: HashMap<char, CalibrationResults>,
}

impl AutoInputShaperCalibrator {
    pub fn new() -> Self {
        Self {
            current_config: ResonanceTestConfig::default(),
            accelerometer_buffer: Vec::new(),
            frequency_response_cache: HashMap::new(),
            last_calibration: HashMap::new(),
        }
    }

    pub fn set_test_config(&mut self, config: ResonanceTestConfig) {
        self.current_config = config;
    }

    /// Start automated resonance test for specified axis
    pub async fn start_calibration(&mut self, axis: char) -> Result<CalibrationTestSequence, CalibrationError> {
        println!("🔬 Starting automated input shaper calibration for {} axis", axis);

        let mut config = self.current_config.clone();
        config.test_axis = axis;

        // A zero step would loop forever below
        if !(config.frequency_step > 0.0) || !(config.frequency_range.0 > 0.0) || config.frequency_range.0 > config.frequency_range.1 {
            return Err(CalibrationError::InvalidConfig);
        }

        // Generate test sequence
        let mut test_sequence = CalibrationTestSequence::new(config.clone());

        // Calculate number of test frequencies
        let freq_count = ((config.frequency_range.1 - config.frequency_range.0) / config.frequency_step) as usize;
        println!("📊 Testing {} frequencies from {:.1} to {:.1} Hz",
                freq_count, config.frequency_range.0, config.frequency_range.1);

        // Pre-allocate accelerometer buffer
        let expected_samples = freq_count * config.duration_per_freq.as_millis() as usize * config.sample_rate as usize / 1000;
        self.accelerometer_buffer.clear();
        self.accelerometer_buffer.reserve(expected_samples);

        // Generate test movements
        let mut current_freq = config.frequency_range.0;
        while current_freq <= config.frequency_range.1 {
            test_sequence.add_frequency_test(current_freq, config.amplitude);
            current_freq += config.frequency_step;
        }

        test_sequence.finalize();
        Ok(test_sequence)
    }

    /// Process accelerometer data during test
    pub fn process_accelerometer_sample(&mut self, sample: f32, timestamp: Instant) {
        self.accelerometer_buffer.push(sample);
    }

    /// Analyze collected data and generate calibration results
    pub fn analyze_results(&mut self, axis: char) -> Result<CalibrationResults, CalibrationError> {
        let start_time = Instant::now();

        // One sample has no spectrum, and the window below would divide by zero
        if self.accelerometer_buffer.len() < 2 {
            return Err(CalibrationError::NoData);
        }

        println!("🧮 Analyzing {} accelerometer samples", self.accelerometer_buffer.len());

        // Perform FFT analysis
        let frequency_response = self.compute_frequency_response(&self.accelerometer_buffer)?;
        self.frequency_response_cache.insert(axis, frequency_response.clone());

        // Detect resonance peaks
        let resonance_peaks = self.detect_resonance_peaks(&frequency_response)?;
        println!("📈 Detected {} resonance peaks", resonance_peaks.len());

        // Generate recommended input shapers
        let recommended_shapers = self.generate_shaper_recommendations(&resonance_peaks)?;

        let results = CalibrationResults {
            test_axis: axis,
            resonance_peaks,
            recommended_shapers,
            frequency_response,
            raw_accelerometer_data: self.accelerometer_buffer.clone(),
            test_timestamp: start_time,
            test_duration: start_time.elapsed(),
        };

        self.last_calibration.insert(axis, results.clone());

        println!("✅ Calibration analysis complete in {:.2}s", start_time.elapsed().as_secs_f32());
        Ok(results)
    }

    fn compute_frequency_response(&self, data: &[f32]) -> Result<Vec<(f32, f32)>, CalibrationError> {
        let mut planner = FftPlanner::new();
        let fft = planner.plan_fft_forward(data.len());

        // Convert real data to complex
        let mut complex_data: Vec<Complex<f32>> = data.iter()
            .map(|&x| Complex::new(x, 0.0))
            .collect();

        // Apply window function (Hann window)
        for (i, sample) in complex_data.iter_mut().enumerate() {
            let window = 0.5 * (1.0 - (2.0 * std::f32::consts::PI * i as f32 / (data.len() - 1) as f32).cos());
            sample.re *= window;
        }

        // Perform FFT
        fft.process(&mut complex_data);

        // Calculate magnitude spectrum
        let sample_rate = self.current_config.sample_rate as f32;
        let mut frequency_response = Vec::new();

        let half_len = complex_data.len() / 2;
        for i in 0..half_len {
            let frequency = i as f32 * sample_rate / complex_data.len() as f32;
            let magnitude = (complex_data[i].re.powi(2) + complex_data[i].im.powi(2)).sqrt();
            frequency_response.push((frequency, magnitude));
        }

        Ok(frequency_response)
    }

    fn detect_resonance_peaks(&self, frequency_response: &[(f32, f32)]) -> Result<Vec<ResonancePeak>, CalibrationError> {
        let mut peaks = Vec::new();

        if frequency_response.len() < 3 {
            return Ok(peaks);
        }

        // Find local maxima
        let mut peak_candidates = Vec::new();
        for i in 1..frequency_response.len() - 1 {
            let (freq, mag) = frequency_response[i];
            let prev_mag = frequency_response[i - 1].1;
            let next_mag = frequency_response[i + 1].1;

            if mag > prev_mag && mag > next_mag {
                peak_candidates.push((i, freq, mag));
            }
        }

        // Filter peaks by prominence and frequency range
        let test_range = self.current_config.frequency_range;
        let mean_magnitude: f32 = frequency_response.iter().map(|(_, mag)| mag).sum::<f32>() / frequency_response.len() as f32;
        let magnitude_threshold = mean_magnitude * 2.0; // Peaks must be 2x average

        for (index, freq, mag) in peak_candidates {
            if freq >= test_range.0 && freq <= test_range.1 && mag > magnitude_threshold {
                // Calculate Q factor by measuring peak width at half maximum
                let half_max = mag * 0.707; // -3dB point

                let mut left_index = index;
                while left_index > 0 && frequency_response[left_index].1 > half_max {
                    left_index -= 1;
                }

                let mut right_index = index;
                while right_index < frequency_response.len() - 1 && frequency_response[right_index].1 > half_max {
                    right_index += 1;
                }

                let bandwidth = frequency_response[right_index].0 - frequency_response[left_index].0;
                let q_factor = if bandwidth > 0.0 { freq / bandwidth } else { 1.0 };
                let damping_ratio = 1.0 / (2.0 * q_factor);

                // Calculate confidence based on peak prominence and Q factor
                let prominence = mag / mean_magnitude;
                let confidence = (prominence * q_factor / 10.0).min(1.0);

                peaks.push(ResonancePeak {
                    frequency: freq,
                    magnitude: mag,
                    q_factor,
                    damping_ratio,
                    confidence,
                });
            }
        }

        // Sort by magnitude (strongest peaks first)
        peaks.sort_by(|a, b| b.magnitude.total_cmp(&a.magnitude));

        Ok(peaks)
    }

    fn generate_shaper_recommendations(&self, peaks: &[ResonancePeak]) -> Result<Vec<(InputShaperType, f32)>, CalibrationError> {
        let mut recommendations = Vec::new();

        // No resonance found means nothing to recommend. Inventing a frequency would look
        // like a measured result.
        if peaks.is_empty() {
            return Ok(recommendations);
        }

        // For each significant peak, generate shaper options
        for peak in peaks.iter().take(3) { // Consider top 3 peaks
            if peak.confidence < 0.3 {
                continue; // Skip low-confidence peaks
            }

            let freq = peak.frequency;

            // Generate different shaper types for this frequency
            let damping = peak.damping_ratio;
            let shapers = vec![
                InputShaperType::zv_damped(freq, damping),
                InputShaperType::zvd_damped(freq, damping),
                InputShaperType::mzv_damped(freq, damping),
                InputShaperType::ei_damped(freq, damping),
            ];

            for shaper in shapers {
                // Score shaper based on vibration reduction vs smoothing trade-off
                let vibration_reduction = 1.0 - shaper.vibration_tolerance;
                let smoothing_penalty = shaper.smoothing_factor * 0.5;
                let frequency_penalty = if freq > 100.0 { (freq - 100.0) / 100.0 * 0.2 } else { 0.0 };

                let score = (vibration_reduction - smoothing_penalty - frequency_penalty).max(0.0);

                recommendations.push((shaper, score * peak.confidence));
            }
        }

        // Sort by score (best first)
        recommendations.sort_by(|a, b| b.1.total_cmp(&a.1));

        // Remove duplicates and keep top 5
        recommendations.truncate(5);

        Ok(recommendations)
    }

    pub fn get_last_calibration(&self, axis: char) -> Option<&CalibrationResults> {
        self.last_calibration.get(&axis)
    }

    pub fn clear_calibration_data(&mut self, axis: Option<char>) {
        match axis {
            Some(ax) => {
                self.last_calibration.remove(&ax);
                self.frequency_response_cache.remove(&ax);
            }
            None => {
                self.last_calibration.clear();
                self.frequency_response_cache.clear();
            }
        }
        self.accelerometer_buffer.clear();
    }
}

#[derive(Debug, Clone)]
pub struct CalibrationTestSequence {
    pub config: ResonanceTestConfig,
    pub test_moves: Vec<TestMove>,
    pub total_duration: Duration,
    pub current_step: usize,
    pub is_finalized: bool,
}

#[derive(Debug, Clone)]
pub struct TestMove {
    pub frequency: f32,
    pub amplitude: f32,
    pub duration: Duration,
    pub settle_time: Duration,
    pub gcode_commands: Vec<String>,
}

impl CalibrationTestSequence {
    pub fn new(config: ResonanceTestConfig) -> Self {
        Self {
            config,
            test_moves: Vec::new(),
            total_duration: Duration::new(0, 0),
            current_step: 0,
            is_finalized: false,
        }
    }

    pub fn add_frequency_test(&mut self, frequency: f32, amplitude: f32) {
        let axis = self.config.test_axis;
        let duration = self.config.duration_per_freq;
        let settle_time = self.config.settle_time;

        // Generate G-code for sinusoidal motion
        let mut gcode_commands = Vec::new();

        // Keep the peak acceleration (angular frequency squared * amplitude) inside the limit
        let omega = 2.0 * std::f32::consts::PI * frequency;
        let amplitude = amplitude.min(MAX_TEST_ACCELERATION / (omega * omega));

        // Set feedrate based on frequency and amplitude
        let peak_speed = omega * amplitude; // mm/s
        let feedrate = peak_speed * 60.0; // mm/min

        // Generate sinusoidal motion commands
        let steps_per_cycle = 20;
        let cycles = (duration.as_secs_f32() * frequency) as u32;
        let total_steps = cycles * steps_per_cycle;

        gcode_commands.push(format!("G21")); // Metric units
        // Relative: the test shakes around wherever the axis is. Absolute would shake around
        // machine zero, driving into the endstop at the start of the axis.
        gcode_commands.push(format!("G91"));
        gcode_commands.push(format!("G1 F{:.0}", feedrate)); // Set feedrate

        let mut previous = 0.0f32;
        for step in 0..total_steps {
            let t = step as f32 / (steps_per_cycle as f32 * frequency);
            let position = amplitude * (2.0 * std::f32::consts::PI * frequency * t).sin();

            gcode_commands.push(format!("G1 {}{:.4}", axis, position - previous));
            previous = position;
        }

        // Return to center
        gcode_commands.push(format!("G1 {}{:.4}", axis, -previous));
        gcode_commands.push(format!("G90")); // Back to absolute positioning
        gcode_commands.push(format!("G4 P{}", settle_time.as_millis())); // Dwell

        let test_move = TestMove {
            frequency,
            amplitude,
            duration,
            settle_time,
            gcode_commands,
        };

        self.total_duration += duration + settle_time;
        self.test_moves.push(test_move);
    }

    pub fn finalize(&mut self) {
        self.is_finalized = true;
        println!("📝 Test sequence finalized: {} moves, estimated duration: {:.1}s",
                self.test_moves.len(), self.total_duration.as_secs_f32());
    }

    pub fn get_next_move(&mut self) -> Option<&TestMove> {
        if self.current_step < self.test_moves.len() {
            let move_ref = &self.test_moves[self.current_step];
            self.current_step += 1;
            Some(move_ref)
        } else {
            None
        }
    }

    pub fn progress(&self) -> f32 {
        if self.test_moves.is_empty() {
            0.0
        } else {
            self.current_step as f32 / self.test_moves.len() as f32
        }
    }
}

#[derive(Debug, Clone)]
pub enum CalibrationError {
    NoData,
    InvalidConfig,
    AnalysisError(String),
    HardwareError(String),
    UserCancelled,
}

impl std::fmt::Display for CalibrationError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            CalibrationError::NoData => write!(f, "No accelerometer data available for analysis"),
            CalibrationError::InvalidConfig => write!(f, "Invalid calibration configuration"),
            CalibrationError::AnalysisError(msg) => write!(f, "Analysis error: {}", msg),
            CalibrationError::HardwareError(msg) => write!(f, "Hardware error: {}", msg),
            CalibrationError::UserCancelled => write!(f, "Calibration cancelled by user"),
        }
    }
}

impl std::error::Error for CalibrationError {}

/// User interface for calibration results and confirmation
#[derive(Debug, Clone)]
pub struct CalibrationUI {
    pub results: Option<CalibrationResults>,
    pub selected_shaper: Option<InputShaperType>,
    pub user_confirmed: bool,
    pub show_advanced_options: bool,
}

impl CalibrationUI {
    pub fn new() -> Self {
        Self {
            results: None,
            selected_shaper: None,
            user_confirmed: false,
            show_advanced_options: false,
        }
    }

    pub fn set_results(&mut self, results: CalibrationResults) {
        self.results = Some(results);
        self.user_confirmed = false;

        // Auto-select best recommendation
        if let Some(ref results) = self.results {
            if let Some((best_shaper, _)) = results.recommended_shapers.first() {
                self.selected_shaper = Some(best_shaper.clone());
            }
        }
    }

    pub fn render_results_ui(&mut self, ui: &mut egui::Ui) -> Option<InputShaperType> {
        if let Some(ref results) = self.results {
            ui.heading("🎯 Input Shaper Calibration Results");

            ui.separator();

            // Test summary
            ui.label(format!("Axis: {}", results.test_axis));
            ui.label(format!("Test duration: {:.1}s", results.test_duration.as_secs_f32()));
            ui.label(format!("Resonance peaks detected: {}", results.resonance_peaks.len()));

            ui.separator();

            // Resonance peaks
            if !results.resonance_peaks.is_empty() {
                ui.heading("📈 Detected Resonances");
                for (i, peak) in results.resonance_peaks.iter().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("Peak {}: {:.1} Hz", i + 1, peak.frequency));
                        ui.label(format!("Q: {:.1}", peak.q_factor));
                        ui.label(format!("Confidence: {:.0}%", peak.confidence * 100.0));
                    });
                }
                ui.separator();
            }

            // Shaper recommendations
            ui.heading("🛠 Recommended Input Shapers");

            for (shaper, score) in &results.recommended_shapers {
                let is_selected = self.selected_shaper.as_ref()
                    .map(|s| s.name == shaper.name)
                    .unwrap_or(false);

                if ui.radio(is_selected, &shaper.name).clicked() {
                    self.selected_shaper = Some(shaper.clone());
                }

                ui.indent("shaper_details", |ui| {
                    ui.label(format!("Score: {:.2}", score));
                    ui.label(format!("Vibration reduction: {:.0}%", (1.0 - shaper.vibration_tolerance) * 100.0));
                    ui.label(format!("Smoothing: {:.0}%", shaper.smoothing_factor * 100.0));
                });

                ui.separator();
            }

            // Advanced options
            ui.collapsing("Advanced Options", |ui| {
                if let Some(ref mut selected) = self.selected_shaper {
                    ui.label("Fine-tune selected shaper:");

                    // Allow manual adjustment of shaper parameters
                    ui.horizontal(|ui| {
                        ui.label("Vibration tolerance:");
                        ui.add(egui::Slider::new(&mut selected.vibration_tolerance, 0.0..=0.5)
                               .text(""));
                    });

                    ui.horizontal(|ui| {
                        ui.label("Smoothing factor:");
                        ui.add(egui::Slider::new(&mut selected.smoothing_factor, 0.0..=0.5)
                               .text(""));
                    });
                }
            });

            ui.separator();

            // Confirmation buttons
            let button_response = ui.horizontal(|ui| {
                if ui.button("✅ Apply Settings").clicked() {
                    self.user_confirmed = true;
                    return Some(self.selected_shaper.clone());
                }

                if ui.button("🔄 Retest").clicked() {
                    self.results = None;
                    self.selected_shaper = None;
                    self.user_confirmed = false;
                    return Some(None);
                }

                if ui.button("❌ Cancel").clicked() {
                    self.results = None;
                    self.selected_shaper = None;
                    self.user_confirmed = false;
                    return Some(None);
                }
                None
            });

            if let Some(result) = button_response.inner {
                return result;
            }
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_input_shaper_generation() {
        let zv = InputShaperType::zv(50.0);
        assert_eq!(zv.name, "ZV");
        assert_eq!(zv.coefficients.len(), 2);
        assert_eq!(zv.delays.len(), 2);

        let zvd = InputShaperType::zvd(50.0);
        assert_eq!(zvd.name, "ZVD");
        assert_eq!(zvd.coefficients.len(), 3);
        assert_eq!(zvd.delays.len(), 3);

        // Weights sum to 1 for all of them, or the axis overshoots
        for shaper in [InputShaperType::zv(50.0), zvd, InputShaperType::mzv(50.0), InputShaperType::ei(50.0)] {
            let total: f32 = shaper.coefficients.iter().sum();
            assert!((total - 1.0).abs() < 1e-5, "{} sums to {}", shaper.name, total);
        }
    }

    #[test]
    fn test_sequences_stay_gentle_and_near_the_start_point() {
        let mut sequence = CalibrationTestSequence::new(ResonanceTestConfig::default());
        sequence.add_frequency_test(200.0, 1.0);
        let commands = &sequence.test_moves[0].gcode_commands;
        assert!(commands.iter().any(|c| c == "G91"));

        // Follow the relative moves: always close to the start, and back there at the end
        let mut position = 0.0f32;
        let mut furthest = 0.0f32;
        for command in commands.iter().filter(|c| c.starts_with("G1 X")) {
            position += command[4..].parse::<f32>().unwrap();
            furthest = furthest.max(position.abs());
        }
        assert!(position.abs() < 0.001, "ended {} from the start", position);
        let omega = 2.0 * std::f32::consts::PI * 200.0;
        // Tolerance: at this frequency the amplitude is only a few steps of G-code's 0.0001mm
        assert!(furthest * omega * omega <= MAX_TEST_ACCELERATION * 1.15, "shakes at {} mm/s²", furthest * omega * omega);
    }

    #[tokio::test]
    async fn test_bad_config_is_refused_rather_than_hanging() {
        let mut calibrator = AutoInputShaperCalibrator::new();
        calibrator.set_test_config(ResonanceTestConfig { frequency_step: 0.0, ..Default::default() });
        assert!(calibrator.start_calibration('X').await.is_err());
    }

    #[test]
    fn test_test_sequence_generation() {
        let config = ResonanceTestConfig {
            test_axis: 'X',
            frequency_range: (10.0, 20.0),
            frequency_step: 5.0,
            ..Default::default()
        };

        let mut sequence = CalibrationTestSequence::new(config);
        sequence.add_frequency_test(10.0, 1.0);
        sequence.add_frequency_test(15.0, 1.0);
        sequence.add_frequency_test(20.0, 1.0);
        sequence.finalize();

        assert_eq!(sequence.test_moves.len(), 3);
        assert!(sequence.total_duration.as_secs() > 0);
    }
}