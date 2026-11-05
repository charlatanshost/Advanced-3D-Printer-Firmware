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
    
    /// Update the advance for the newest segment. Each segment is applied exactly once.
    fn process_segments(&mut self) {
        let Some(segment) = self.segment_buffer.back().cloned() else {
            return;
        };
        if segment.segment_time < self.config.min_segment_time {
            return;
        }

        let target = self.calculate_pressure_adjustment(&segment);
        let smoothed = self.apply_smoothing(target, segment.segment_time);
        self.apply_pressure_advance(&segment, smoothed);
        self.last_update_time = Instant::now();
    }

    /// How far ahead the extruder should be during this segment. Nozzle pressure follows
    /// flow rate, so the advance is proportional to the extrusion velocity itself: it holds
    /// while cruising and bleeds off when slowing. Only forward extrusion builds pressure,
    /// so travels and retractions have no advance.
    fn calculate_pressure_adjustment(&self, segment: &ExtrusionSegment) -> f64 {
        if !segment.is_extrusion() || segment.extrusion_distance() <= 0.0 {
            return 0.0;
        }
        self.config.pressure_advance * segment.extrude_velocity
    }

    /// Move toward the target with a time constant of `smooth_time`
    fn apply_smoothing(&self, target: f64, dt: f64) -> f64 {
        let last = self.pressure_buffer.back().copied().unwrap_or(0.0);
        let alpha = if self.config.smooth_time > 0.0 {
            1.0 - (-dt / self.config.smooth_time).exp()
        } else {
            1.0
        };
        last + alpha * (target - last)
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
    
    /// Pressure advance cannot be found from the commanded motion alone. It needs a
    /// measurement of what the plastic did (a test print or a pressure sensor). The old
    /// search compared mm/s to mm and returned an arbitrary value, so this refuses instead.
    pub fn auto_calibrate(&mut self, _test_segments: &[ExtrusionSegment]) -> Result<f64, String> {
        Err("Pressure advance calibration needs measured data, which is not available yet. Tune it with a test print.".to_string())
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

        // Always at least one, so there is always an active controller. PA stays off
        // until configured: a value that suits one filament can ruin prints with another.
        for _ in 0..num_extruders.max(1) {
            controllers.push(PressureAdvanceController::new(PressureAdvanceConfig::default()));
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