// Active Resonance Cancellation System
// Advanced vibration control using real-time accelerometer feedback

use std::collections::{VecDeque, HashMap};
use std::time::{Duration, Instant};
use serde::{Deserialize, Serialize};
use crate::input_shaping::{Accelerometer, ShaperType};

// Real-time resonance cancellation using accelerometer feedback
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResonanceCancellationConfig {
    pub enabled: bool,
    pub sample_rate: u32,             // Hz - accelerometer sampling rate
    pub window_size: usize,           // Sample window for FFT analysis
    pub update_interval: Duration,     // How often to update cancellation
    pub gain: f32,                    // Overall cancellation gain (0.0-1.0)
    pub frequency_range: (f32, f32),  // Hz - frequency range to monitor
    pub threshold_amplitude: f32,      // Minimum vibration to trigger cancellation
    pub max_correction: f32,          // Maximum correction amplitude (mm/s²)
    pub learning_rate: f32,           // Adaptive learning rate for algorithm
}

impl Default for ResonanceCancellationConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            sample_rate: 1000,           // 1kHz sampling
            window_size: 512,            // ~0.5s window at 1kHz
            update_interval: Duration::from_millis(50), // 20Hz update rate
            gain: 0.5,
            frequency_range: (5.0, 200.0),
            threshold_amplitude: 0.1,    // 0.1 m/s² threshold
            max_correction: 2.0,         // 2 m/s² max correction
            learning_rate: 0.01,
        }
    }
}

#[derive(Debug, Clone)]
pub struct FrequencyBin {
    pub frequency: f32,              // Hz
    pub magnitude: f32,              // Current magnitude
    pub phase: f32,                  // Current phase (radians)
    pub peak_history: VecDeque<f32>, // Historical peak magnitudes
    pub is_resonant: bool,           // Is this a problematic frequency?
}

pub struct ActiveResonanceCanceller {
    config: ResonanceCancellationConfig,
    accelerometer: Box<dyn Accelerometer + Send>,
    sample_buffer: VecDeque<(f32, f32, f32)>, // (ax, ay, az) samples
    frequency_bins: Vec<FrequencyBin>,
    correction_filters: HashMap<char, AdaptiveFilter>, // Per-axis filters
    last_update: Instant,
    fft_buffer: Vec<f32>,           // Working buffer for FFT
    cancellation_signals: HashMap<char, VecDeque<f32>>, // Generated correction signals
}

impl ActiveResonanceCanceller {
    pub fn new(config: ResonanceCancellationConfig, accelerometer: Box<dyn Accelerometer + Send>) -> Self {
        let num_bins = config.window_size / 2;
        let frequency_bins = (0..num_bins).map(|i| {
            let frequency = (i as f32 * config.sample_rate as f32) / config.window_size as f32;
            FrequencyBin {
                frequency,
                magnitude: 0.0,
                phase: 0.0,
                peak_history: VecDeque::with_capacity(100),
                is_resonant: frequency >= config.frequency_range.0 && 
                           frequency <= config.frequency_range.1,
            }
        }).collect();
        
        let mut correction_filters = HashMap::new();
        for &axis in &['X', 'Y', 'Z', 'A', 'B'] {
            correction_filters.insert(axis, AdaptiveFilter::new());
        }
        
        let mut cancellation_signals = HashMap::new();
        for &axis in &['X', 'Y', 'Z', 'A', 'B'] {
            cancellation_signals.insert(axis, VecDeque::with_capacity(1000));
        }
        
        let window_size = config.window_size;
        Self {
            config,
            accelerometer,
            sample_buffer: VecDeque::with_capacity(window_size * 2),
            frequency_bins,
            correction_filters,
            last_update: Instant::now(),
            fft_buffer: vec![0.0; window_size],
            cancellation_signals,
        }
    }
    
    pub async fn update(&mut self) -> Result<HashMap<char, f32>, String> {
        if !self.config.enabled {
            return Ok(HashMap::new());
        }
        
        // Sample accelerometer at configured rate
        let (ax, ay, az) = self.accelerometer.read_acceleration()?;
        self.sample_buffer.push_back((ax as f32, ay as f32, az as f32));
        
        // Maintain buffer size
        while self.sample_buffer.len() > self.config.window_size {
            self.sample_buffer.pop_front();
        }
        
        let mut corrections = HashMap::new();
        
        // Update cancellation at configured interval
        let now = Instant::now();
        if now.duration_since(self.last_update) >= self.config.update_interval {
            if self.sample_buffer.len() >= self.config.window_size {
                // Analyze vibrations for each axis
                corrections.insert('X', self.analyze_and_correct_axis(0).await?);
                corrections.insert('Y', self.analyze_and_correct_axis(1).await?);
                corrections.insert('Z', self.analyze_and_correct_axis(2).await?);
                
                self.last_update = now;
            }
        }
        
        Ok(corrections)
    }
    
    async fn analyze_and_correct_axis(&mut self, axis_index: usize) -> Result<f32, String> {
        // Extract axis data from sample buffer
        for (i, sample) in self.sample_buffer.iter().enumerate() {
            self.fft_buffer[i] = match axis_index {
                0 => sample.0, // X axis
                1 => sample.1, // Y axis
                2 => sample.2, // Z axis
                _ => 0.0,
            };
        }
        
        // Perform FFT analysis
        let spectrum = self.perform_fft(&self.fft_buffer);
        
        // Update frequency bins with new data
        for (i, bin) in self.frequency_bins.iter_mut().enumerate() {
            if i < spectrum.len() && bin.is_resonant {
                bin.magnitude = spectrum[i].0; // Real part as magnitude
                bin.phase = spectrum[i].1;     // Imaginary part as phase
                
                // Update peak history
                bin.peak_history.push_back(bin.magnitude);
                if bin.peak_history.len() > 100 {
                    bin.peak_history.pop_front();
                }
            }
        }
        
        // Identify problematic frequencies
        let problematic_frequencies = self.identify_resonances();
        
        // Generate correction signal
        let axis_char = match axis_index {
            0 => 'X',
            1 => 'Y',
            2 => 'Z',
            _ => return Ok(0.0),
        };
        
        let correction = self.generate_correction_signal(axis_char, &problematic_frequencies);
        
        Ok(correction)
    }
    
    #[cfg(feature = "rustfft")]
    fn perform_fft(&self, input: &[f32]) -> Vec<(f32, f32)> {
        use rustfft::{FftPlanner, num_complex::Complex};
        
        let mut planner = FftPlanner::new();
        let fft = planner.plan_fft_forward(input.len());
        
        let mut buffer: Vec<Complex<f32>> = input.iter()
            .map(|&x| Complex::new(x, 0.0))
            .collect();
        
        fft.process(&mut buffer);
        
        buffer.iter().take(buffer.len() / 2)
            .map(|c| (c.re, c.im))
            .collect()
    }
    
    #[cfg(not(feature = "rustfft"))]
    fn perform_fft(&self, input: &[f32]) -> Vec<(f32, f32)> {
        // Simplified FFT implementation when rustfft is not available
        let n = input.len();
        let mut result = Vec::with_capacity(n / 2);
        
        for k in 0..n/2 {
            let mut real_sum = 0.0;
            let mut imag_sum = 0.0;
            
            for i in 0..n {
                let angle = -2.0 * std::f32::consts::PI * (k as f32) * (i as f32) / (n as f32);
                real_sum += input[i] * angle.cos();
                imag_sum += input[i] * angle.sin();
            }
            
            result.push((real_sum, imag_sum));
        }
        
        result
    }
    
    fn identify_resonances(&self) -> Vec<(f32, f32)> {
        // Identify frequencies that exceed threshold and show resonant behavior
        let mut resonances = Vec::new();
        
        for bin in &self.frequency_bins {
            if bin.magnitude > self.config.threshold_amplitude {
                // Check if this frequency has consistent peaks (resonance indicator)
                if bin.peak_history.len() > 10 {
                    let avg_magnitude: f32 = bin.peak_history.iter().sum::<f32>() / bin.peak_history.len() as f32;
                    let variance: f32 = bin.peak_history.iter()
                        .map(|&x| (x - avg_magnitude).powi(2))
                        .sum::<f32>() / bin.peak_history.len() as f32;
                    
                    // Low variance indicates consistent resonance
                    if variance < avg_magnitude * 0.1 && avg_magnitude > self.config.threshold_amplitude {
                        resonances.push((bin.frequency, bin.magnitude));
                    }
                }
            }
        }
        
        resonances
    }
    
    fn generate_correction_signal(&mut self, axis: char, resonances: &[(f32, f32)]) -> f32 {
        if resonances.is_empty() {
            return 0.0;
        }
        
        let now_secs = Instant::now().elapsed().as_secs_f32();
        let mut correction = 0.0;
        
        // Generate anti-phase signals for each identified resonance
        for &(frequency, magnitude) in resonances {
            // Calculate correction amplitude (proportional to detected vibration)
            let correction_amplitude = (magnitude * self.config.gain).min(self.config.max_correction);
            
            // Generate anti-phase signal (180° phase shift)
            let anti_phase_signal = -correction_amplitude * (2.0 * std::f32::consts::PI * frequency * now_secs).sin();
            correction += anti_phase_signal;
        }
        
        // Apply adaptive filtering to improve correction over time
        if let Some(filter) = self.correction_filters.get_mut(&axis) {
            correction = filter.process(correction);
        }
        
        // Store correction signal for visualization/debugging
        if let Some(signal_buffer) = self.cancellation_signals.get_mut(&axis) {
            signal_buffer.push_back(correction);
            if signal_buffer.len() > 1000 {
                signal_buffer.pop_front();
            }
        }
        
        correction.clamp(-self.config.max_correction, self.config.max_correction)
    }
    
    pub fn get_vibration_spectrum(&self) -> Vec<(f32, f32)> {
        // Return current frequency spectrum for visualization
        self.frequency_bins.iter()
            .filter(|bin| bin.is_resonant)
            .map(|bin| (bin.frequency, bin.magnitude))
            .collect()
    }
    
    pub fn get_cancellation_effectiveness(&self) -> HashMap<char, f32> {
        // Calculate effectiveness of cancellation for each axis
        let mut effectiveness = HashMap::new();
        
        for (&axis, signal) in &self.cancellation_signals {
            if signal.len() > 100 {
                // Compare recent vibrations to historical average
                let recent_avg: f32 = signal.iter().rev().take(50).sum::<f32>() / 50.0;
                let historical_avg: f32 = signal.iter().sum::<f32>() / signal.len() as f32;
                
                let reduction = 1.0 - (recent_avg.abs() / historical_avg.abs().max(0.001));
                effectiveness.insert(axis, reduction.clamp(0.0, 1.0));
            } else {
                effectiveness.insert(axis, 0.0);
            }
        }
        
        effectiveness
    }
    
    pub fn tune_parameters(&mut self, gain: f32, threshold: f32, learning_rate: f32) {
        self.config.gain = gain.clamp(0.0, 1.0);
        self.config.threshold_amplitude = threshold.max(0.0);
        self.config.learning_rate = learning_rate.clamp(0.001, 0.1);
        
        // Update adaptive filters with new learning rate
        for filter in self.correction_filters.values_mut() {
            filter.set_learning_rate(learning_rate);
        }
    }
    
    pub fn enable(&mut self, enabled: bool) {
        self.config.enabled = enabled;
        if !enabled {
            // Clear correction signals when disabling
            for signal in self.cancellation_signals.values_mut() {
                signal.clear();
            }
        }
    }
}

// Adaptive filter for learning optimal correction signals
pub struct AdaptiveFilter {
    coefficients: Vec<f32>,           // Filter coefficients
    input_history: VecDeque<f32>,     // Input signal history
    learning_rate: f32,               // Adaptation speed
    filter_length: usize,             // Number of filter taps
}

impl AdaptiveFilter {
    pub fn new() -> Self {
        let filter_length = 32;
        Self {
            coefficients: vec![0.0; filter_length],
            input_history: VecDeque::with_capacity(filter_length),
            learning_rate: 0.01,
            filter_length,
        }
    }
    
    pub fn process(&mut self, input: f32) -> f32 {
        // Add input to history
        self.input_history.push_back(input);
        if self.input_history.len() > self.filter_length {
            self.input_history.pop_front();
        }
        
        // Calculate filter output
        let mut output = 0.0;
        for (i, &coeff) in self.coefficients.iter().enumerate() {
            if let Some(&hist_input) = self.input_history.get(self.input_history.len().saturating_sub(i + 1)) {
                output += coeff * hist_input;
            }
        }
        
        // Adaptive update (simplified LMS algorithm)
        let error = input - output; // Error between desired and actual
        for (i, coeff) in self.coefficients.iter_mut().enumerate() {
            if let Some(&hist_input) = self.input_history.get(self.input_history.len().saturating_sub(i + 1)) {
                *coeff += self.learning_rate * error * hist_input;
                // Prevent coefficient explosion
                *coeff = coeff.clamp(-1.0, 1.0);
            }
        }
        
        output
    }
    
    pub fn set_learning_rate(&mut self, rate: f32) {
        self.learning_rate = rate.clamp(0.001, 0.1);
    }
    
    pub fn reset(&mut self) {
        self.coefficients.fill(0.0);
        self.input_history.clear();
    }
}

// Integration with motion system for real-time correction
pub struct MotionCorrectionInterface {
    canceller: ActiveResonanceCanceller,
    correction_history: HashMap<char, VecDeque<f32>>,
    motion_commands: VecDeque<MotionCorrection>,
}

#[derive(Debug, Clone)]
pub struct MotionCorrection {
    pub axis: char,
    pub correction: f32,              // mm/s² correction
    pub timestamp: Instant,
    pub duration: Duration,           // How long to apply correction
}

impl MotionCorrectionInterface {
    pub fn new(canceller: ActiveResonanceCanceller) -> Self {
        let mut correction_history = HashMap::new();
        for &axis in &['X', 'Y', 'Z', 'A', 'B'] {
            correction_history.insert(axis, VecDeque::with_capacity(1000));
        }
        
        Self {
            canceller,
            correction_history,
            motion_commands: VecDeque::with_capacity(100),
        }
    }
    
    pub async fn update_motion_corrections(&mut self) -> Result<Vec<MotionCorrection>, String> {
        // Get latest corrections from resonance canceller
        let corrections = self.canceller.update().await?;
        
        let mut motion_corrections = Vec::new();
        let now = Instant::now();
        
        for (axis, correction) in corrections {
            if correction.abs() > 0.001 { // Only apply significant corrections
                let motion_correction = MotionCorrection {
                    axis,
                    correction,
                    timestamp: now,
                    duration: Duration::from_millis(50), // Short correction duration
                };
                
                motion_corrections.push(motion_correction.clone());
                self.motion_commands.push_back(motion_correction);
                
                // Store in history
                if let Some(history) = self.correction_history.get_mut(&axis) {
                    history.push_back(correction);
                    if history.len() > 1000 {
                        history.pop_front();
                    }
                }
            }
        }
        
        // Remove expired motion commands
        let cutoff_time = now - Duration::from_millis(100);
        while let Some(cmd) = self.motion_commands.front() {
            if cmd.timestamp < cutoff_time {
                self.motion_commands.pop_front();
            } else {
                break;
            }
        }
        
        Ok(motion_corrections)
    }
    
    pub fn get_active_corrections(&self) -> Vec<&MotionCorrection> {
        let now = Instant::now();
        self.motion_commands.iter()
            .filter(|cmd| now.duration_since(cmd.timestamp) < cmd.duration)
            .collect()
    }
    
    pub fn get_correction_statistics(&self) -> HashMap<char, (f32, f32, f32)> {
        // Return (mean, std_dev, max) for each axis
        let mut stats = HashMap::new();
        
        for (axis, history) in &self.correction_history {
            if !history.is_empty() {
                let mean: f32 = history.iter().sum::<f32>() / history.len() as f32;
                let variance: f32 = history.iter()
                    .map(|&x| (x - mean).powi(2))
                    .sum::<f32>() / history.len() as f32;
                let std_dev = variance.sqrt();
                let max_correction = history.iter().fold(0.0f32, |acc, &x| acc.max(x.abs()));
                
                stats.insert(*axis, (mean, std_dev, max_correction));
            }
        }
        
        stats
    }
}