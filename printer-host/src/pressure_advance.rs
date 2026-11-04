// Pressure Advance Implementation for Improved Extrusion Quality
// This implements Klipper-style pressure advance to reduce oozing and improve corner quality

use std::collections::VecDeque;
use std::time::Instant;

/// Pressure advance configuration parameters
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PressureAdvanceConfig {
    /// Pressure advance coefficient (typically 0.0 - 0.1)
    pub pressure_advance: f64,
    /// Smooth time for pressure advance (typically 0.04s)
    pub smooth_time: f64,
    /// Maximum lookahead distance for pressure prediction
    pub max_lookahead_distance: f64,
    /// Minimum segment time to apply pressure advance
    pub min_segment_time: f64,
}

impl Default for PressureAdvanceConfig {
    fn default() -> Self {
        Self {
            pressure_advance: 0.0,
            smooth_time: 0.04,
            max_lookahead_distance: 5.0, // mm
            min_segment_time: 0.001,      // 1ms
        }
    }
}

/// Represents a motion segment with extrusion
#[derive(Debug, Clone)]
pub struct ExtrusionSegment {
    pub start_pos: [f64; 4],      // X, Y, Z, E
    pub end_pos: [f64; 4],        // X, Y, Z, E
    pub start_velocity: f64,      // mm/s
    pub end_velocity: f64,        // mm/s  
    pub extrude_velocity: f64,    // mm/s extrusion rate
    pub segment_time: f64,        // Duration in seconds
    pub pressure_advance: f64,    // PA coefficient for this segment
    pub timestamp: Instant,
}

impl ExtrusionSegment {
    /// Calculate the extrusion distance for this segment
    pub fn extrusion_distance(&self) -> f64 {
        self.end_pos[3] - self.start_pos[3]
    }
    
    /// Calculate the travel distance for this segment
    pub fn travel_distance(&self) -> f64 {
        let dx = self.end_pos[0] - self.start_pos[0];
        let dy = self.end_pos[1] - self.start_pos[1];
        let dz = self.end_pos[2] - self.start_pos[2];
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
    
    /// Check if this is an extrusion move (E axis movement)
    pub fn is_extrusion(&self) -> bool {
        self.extrusion_distance().abs() > 0.001
    }
}

/// Pressure advance controller
pub struct PressureAdvanceController {
    config: PressureAdvanceConfig,
    segment_buffer: VecDeque<ExtrusionSegment>,
    extruder_position: f64,
    extruder_velocity: f64,
    last_update_time: Instant,
    pressure_buffer: VecDeque<f64>,
    statistics: PressureAdvanceStats,
}

#[derive(Debug, Default)]
pub struct PressureAdvanceStats {
    pub segments_processed: u64,
    pub total_advance_applied: f64,
    pub max_advance_applied: f64,
    pub avg_pressure_change: f64,
    pub pressure_reversals: u64,
}

impl PressureAdvanceController {
    pub fn new(config: PressureAdvanceConfig) -> Self {
        Self {
            config,
            segment_buffer: VecDeque::new(),
            extruder_position: 0.0,
            extruder_velocity: 0.0,
            last_update_time: Instant::now(),
            pressure_buffer: VecDeque::new(),
            statistics: PressureAdvanceStats::default(),
        }
    }
    
    /// Add a new extrusion segment to the lookahead buffer
    pub fn add_segment(&mut self, segment: ExtrusionSegment) {
        self.segment_buffer.push_back(segment);
        
        // Limit buffer size to prevent excessive memory usage
        while self.segment_buffer.len() > 100 {
            self.segment_buffer.pop_front();
        }
        
        self.process_segments();
    }
    
    /// Calculate pressure advance for upcoming segments
    fn process_segments(&mut self) {
        if self.segment_buffer.is_empty() {
            return;
        }
        
        let current_time = Instant::now();
        let lookahead_time = self.config.smooth_time;
        
        // Process segments within lookahead window
        let mut lookahead_segments = Vec::new();
        let mut total_lookahead_time = 0.0;
        
        for segment in &self.segment_buffer {
            if total_lookahead_time >= lookahead_time {
                break;
            }
            
            if segment.is_extrusion() && segment.segment_time >= self.config.min_segment_time {
                lookahead_segments.push(segment.clone());
                total_lookahead_time += segment.segment_time;
            }
        }
        
        // Calculate pressure advance adjustments
        for (i, segment) in lookahead_segments.iter().enumerate() {
            let pressure_adjustment = self.calculate_pressure_adjustment(segment, &lookahead_segments, i);
            self.apply_pressure_advance(segment, pressure_adjustment);
        }
        
        self.last_update_time = current_time;
    }
    
    /// Calculate pressure adjustment for a specific segment
    fn calculate_pressure_adjustment(&self, current_segment: &ExtrusionSegment, 
                                   lookahead_segments: &[ExtrusionSegment], 
                                   current_index: usize) -> f64 {
        if !current_segment.is_extrusion() {
            return 0.0;
        }
        
        let current_extrude_rate = current_segment.extrude_velocity;
        
        // Look ahead to next segment to predict velocity change
        let next_extrude_rate = if current_index + 1 < lookahead_segments.len() {
            lookahead_segments[current_index + 1].extrude_velocity
        } else {
            0.0 // Assume stop at end
        };
        
        // Calculate velocity change
        let velocity_change = next_extrude_rate - current_extrude_rate;
        
        // Apply pressure advance formula: 
        // advance = pressure_advance * velocity_change
        let pressure_adjustment = self.config.pressure_advance * velocity_change;
        
        // Apply smoothing to prevent abrupt changes
        self.apply_smoothing(pressure_adjustment)
    }
    
    /// Apply smoothing to pressure advance to prevent jerky motion
    fn apply_smoothing(&self, raw_adjustment: f64) -> f64 {
        // Simple exponential smoothing
        let alpha = 0.3; // Smoothing factor
        
        if let Some(&last_adjustment) = self.pressure_buffer.back() {
            alpha * raw_adjustment + (1.0 - alpha) * last_adjustment
        } else {
            raw_adjustment
        }
    }
    
    /// Apply the calculated pressure advance to extruder motion
    fn apply_pressure_advance(&mut self, segment: &ExtrusionSegment, adjustment: f64) {
        // Update statistics
        self.statistics.segments_processed += 1;
        self.statistics.total_advance_applied += adjustment.abs();
        self.statistics.max_advance_applied = self.statistics.max_advance_applied.max(adjustment.abs());
        
        // Track pressure reversals (sign changes)
        if let Some(&last_pressure) = self.pressure_buffer.back() {
            if (adjustment > 0.0) != (last_pressure > 0.0) {
                self.statistics.pressure_reversals += 1;
            }
        }
        
        // Add to pressure buffer for smoothing
        self.pressure_buffer.push_back(adjustment);
        if self.pressure_buffer.len() > 20 {
            self.pressure_buffer.pop_front();
        }
        
        // Calculate average pressure change
        if self.statistics.segments_processed > 0 {
            self.statistics.avg_pressure_change = 
                self.statistics.total_advance_applied / self.statistics.segments_processed as f64;
        }
    }
    
    /// Get the current extruder position with pressure advance applied
    pub fn get_adjusted_extruder_position(&self) -> f64 {
        let current_pressure = self.pressure_buffer.back().unwrap_or(&0.0);
        self.extruder_position + current_pressure
    }
    
    /// Update extruder position
    pub fn update_extruder_position(&mut self, position: f64) {
        self.extruder_position = position;
    }
    
    /// Update extruder velocity
    pub fn update_extruder_velocity(&mut self, velocity: f64) {
        self.extruder_velocity = velocity;
    }
    
    /// Get current pressure advance configuration
    pub fn get_config(&self) -> &PressureAdvanceConfig {
        &self.config
    }
    
    /// Update pressure advance configuration
    pub fn update_config(&mut self, config: PressureAdvanceConfig) {
        self.config = config;
    }
    
    /// Get pressure advance statistics
    pub fn get_statistics(&self) -> &PressureAdvanceStats {
        &self.statistics
    }
    
    /// Reset statistics
    pub fn reset_statistics(&mut self) {
        self.statistics = PressureAdvanceStats::default();
    }
    
    /// Clear all buffers (useful for print start/stop)
    pub fn clear_buffers(&mut self) {
        self.segment_buffer.clear();
        self.pressure_buffer.clear();
        self.extruder_position = 0.0;
        self.extruder_velocity = 0.0;
    }
    
    /// Auto-calibrate pressure advance based on print characteristics
    pub fn auto_calibrate(&mut self, test_segments: &[ExtrusionSegment]) -> Result<f64, String> {
        if test_segments.is_empty() {
            return Err("No test segments provided".to_string());
        }
        
        // Analyze test segments to find optimal pressure advance
        let mut optimal_pa = 0.0;
        let mut min_error = f64::INFINITY;
        
        // Test different PA values from 0.0 to 0.1 in steps of 0.005
        for pa_test in (0..=20).map(|i| i as f64 * 0.005) {
            let mut test_controller = self.clone();
            test_controller.config.pressure_advance = pa_test;
            
            // Simulate the test segments
            let error = test_controller.simulate_and_measure_error(test_segments);
            
            if error < min_error {
                min_error = error;
                optimal_pa = pa_test;
            }
        }
        
        // Apply the optimal pressure advance
        self.config.pressure_advance = optimal_pa;
        
        Ok(optimal_pa)
    }
    
    /// Simulate segments and measure quality error
    fn simulate_and_measure_error(&mut self, test_segments: &[ExtrusionSegment]) -> f64 {
        self.clear_buffers();
        
        let mut total_error = 0.0;
        let mut corner_count = 0;
        
        for segment in test_segments {
            self.add_segment(segment.clone());
            
            // Measure error at corners (direction changes)
            if self.is_corner_segment(segment, test_segments) {
                let expected_flow = segment.extrude_velocity;
                let actual_flow = self.get_adjusted_extruder_position();
                let error = (expected_flow - actual_flow).abs();
                total_error += error;
                corner_count += 1;
            }
        }
        
        if corner_count > 0 {
            total_error / corner_count as f64
        } else {
            0.0
        }
    }
    
    /// Check if a segment represents a corner (direction change)
    fn is_corner_segment(&self, segment: &ExtrusionSegment, all_segments: &[ExtrusionSegment]) -> bool {
        // Simple heuristic: significant velocity change indicates corner
        (segment.end_velocity - segment.start_velocity).abs() > 10.0 // mm/s
    }
}

impl Clone for PressureAdvanceController {
    fn clone(&self) -> Self {
        Self {
            config: self.config.clone(),
            segment_buffer: self.segment_buffer.clone(),
            extruder_position: self.extruder_position,
            extruder_velocity: self.extruder_velocity,
            last_update_time: Instant::now(), // Reset timestamp for clone
            pressure_buffer: self.pressure_buffer.clone(),
            statistics: PressureAdvanceStats::default(), // Reset stats for clone
        }
    }
}

/// Pressure advance integration for multi-extruder systems
pub struct MultiExtruderPressureAdvance {
    controllers: Vec<PressureAdvanceController>,
    active_extruder: usize,
}

impl MultiExtruderPressureAdvance {
    pub fn new(num_extruders: usize) -> Self {
        let mut controllers = Vec::new();
        
        for i in 0..num_extruders {
            let config = PressureAdvanceConfig {
                pressure_advance: 0.05 * (i + 1) as f64 / num_extruders as f64, // Different defaults
                ..Default::default()
            };
            controllers.push(PressureAdvanceController::new(config));
        }
        
        Self {
            controllers,
            active_extruder: 0,
        }
    }
    
    pub fn set_active_extruder(&mut self, extruder: usize) -> Result<(), String> {
        if extruder >= self.controllers.len() {
            return Err(format!("Extruder {} not available (max: {})", extruder, self.controllers.len() - 1));
        }
        
        self.active_extruder = extruder;
        Ok(())
    }
    
    pub fn get_active_controller(&mut self) -> &mut PressureAdvanceController {
        &mut self.controllers[self.active_extruder]
    }
    
    pub fn get_controller(&mut self, extruder: usize) -> Option<&mut PressureAdvanceController> {
        self.controllers.get_mut(extruder)
    }
    
    pub fn add_segment(&mut self, segment: ExtrusionSegment) {
        self.get_active_controller().add_segment(segment);
    }
    
    /// Configure pressure advance for a specific extruder
    pub fn configure_extruder(&mut self, extruder: usize, config: PressureAdvanceConfig) -> Result<(), String> {
        if let Some(controller) = self.controllers.get_mut(extruder) {
            controller.update_config(config);
            Ok(())
        } else {
            Err(format!("Extruder {} not found", extruder))
        }
    }
}