// Concurrent G-Code Processing and Rendering System
// Separates parsing, geometry generation, and rendering into different threads
// for optimal performance on both SBCs and high-end PCs

use std::sync::{Arc, Mutex, mpsc};
use std::thread::{self, JoinHandle};
use std::path::Path;
use std::fs::File;
use std::time::{Duration, Instant};
use std::collections::VecDeque;

use crate::fast_gcode_parser::{
    StreamingGCodeParser, GCodeCommand, ToolpathGeometry, GeometryGenerator
};
use crate::optimized_3d_viewer::{Optimized3DViewer, LODConfig, RenderStatistics};

// Message types for inter-thread communication
#[derive(Debug)]
pub enum ProcessorMessage {
    LoadFile(String),           // Request to load a g-code file
    UpdateProgress(f32),        // Progress update (0.0 to 1.0)
    RenderFrame,               // Request frame render
    Shutdown,                  // Shutdown request
}

#[derive(Debug)]
pub enum ProcessorResponse {
    LoadStarted,
    LoadCompleted { 
        line_count: u32, 
        geometry_stats: GeometryStats,
        parsing_time: Duration,
    },
    LoadError(String),
    ProgressUpdate(f32),
    FrameRendered(RenderStatistics),
    ShutdownComplete,
    GeometryReady(ToolpathGeometry), // New geometry available
}

#[derive(Debug, Clone)]
pub struct GeometryStats {
    pub total_lines: u32,
    pub extrude_lines: u32,
    pub travel_lines: u32,
    pub total_distance: f32,
    pub print_time_estimate: f32,
    pub bounds: [f32; 6], // min_x, max_x, min_y, max_y, min_z, max_z
    pub layer_count: u32,
}

// Configuration for the concurrent processor
#[derive(Debug, Clone)]
pub struct ProcessorConfig {
    pub parsing_threads: usize,
    pub geometry_cache_size: usize,
    pub max_frame_rate: f32,
    pub enable_background_loading: bool,
    pub lod_config: LODConfig,
    pub buffer_size: usize,
}

impl Default for ProcessorConfig {
    fn default() -> Self {
        Self {
            parsing_threads: num_cpus::get(),
            geometry_cache_size: 10, // Cache up to 10 files
            max_frame_rate: 120.0,
            enable_background_loading: true,
            lod_config: LODConfig::default(),
            buffer_size: 1024 * 1024, // 1MB buffer
        }
    }
}

// Main concurrent processor
pub struct ConcurrentGCodeProcessor {
    config: ProcessorConfig,
    
    // Thread handles
    parsing_thread: Option<JoinHandle<()>>,
    geometry_thread: Option<JoinHandle<()>>,
    
    // Communication channels
    command_sender: mpsc::Sender<ProcessorMessage>,
    response_receiver: mpsc::Receiver<ProcessorResponse>,
    
    // Internal channels
    parse_sender: mpsc::Sender<ParsingTask>,
    geometry_sender: mpsc::Sender<GeometryTask>,
    
    // State
    current_file: Option<String>,
    is_loading: bool,
    geometry_cache: Arc<Mutex<GeometryCache>>,
    
    // Performance monitoring
    performance_stats: PerformanceStats,
}

#[derive(Debug)]
struct ParsingTask {
    file_path: String,
    task_id: u64,
    priority: u8, // 0 = highest priority
}

#[derive(Debug)]
struct GeometryTask {
    commands: Vec<GCodeCommand>,
    task_id: u64,
    response_sender: mpsc::Sender<ProcessorResponse>,
}

#[derive(Debug, Default)]
struct GeometryCache {
    entries: VecDeque<CacheEntry>,
    max_size: usize,
    total_memory: usize,
}

#[derive(Debug)]
struct CacheEntry {
    file_path: String,
    geometry: ToolpathGeometry,
    last_accessed: Instant,
    memory_size: usize,
}

#[derive(Debug, Default)]
struct PerformanceStats {
    pub total_files_processed: u64,
    pub average_parsing_time: Duration,
    pub average_geometry_time: Duration,
    pub cache_hit_rate: f32,
    pub memory_usage: usize,
}

impl ConcurrentGCodeProcessor {
    pub fn new(config: ProcessorConfig) -> Result<Self, ProcessorError> {
        // Create communication channels
        let (command_sender, command_receiver) = mpsc::channel();
        let (response_sender, response_receiver) = mpsc::channel();
        let (parse_sender, parse_receiver) = mpsc::channel();
        let (geometry_sender, geometry_receiver) = mpsc::channel();
        
        // Create shared geometry cache
        let geometry_cache = Arc::new(Mutex::new(GeometryCache {
            entries: VecDeque::new(),
            max_size: config.geometry_cache_size,
            total_memory: 0,
        }));
        
        let mut processor = Self {
            config: config.clone(),
            parsing_thread: None,
            geometry_thread: None,
            command_sender,
            response_receiver,
            parse_sender,
            geometry_sender,
            current_file: None,
            is_loading: false,
            geometry_cache: geometry_cache.clone(),
            performance_stats: PerformanceStats::default(),
        };
        
        // Start worker threads
        processor.start_threads(
            command_receiver,
            response_sender,
            parse_receiver,
            geometry_receiver,
            geometry_cache,
        )?;
        
        Ok(processor)
    }
    
    fn start_threads(
        &mut self,
        command_receiver: mpsc::Receiver<ProcessorMessage>,
        response_sender: mpsc::Sender<ProcessorResponse>,
        parse_receiver: mpsc::Receiver<ParsingTask>,
        geometry_receiver: mpsc::Receiver<GeometryTask>,
        geometry_cache: Arc<Mutex<GeometryCache>>,
    ) -> Result<(), ProcessorError> {
        
        let config_for_parser = self.config.clone();
        let geometry_sender = self.geometry_sender.clone();
        let response_sender_clone = response_sender.clone();
        self.parsing_thread = Some(thread::spawn(move || {
            Self::parsing_thread_worker(
                parse_receiver,
                geometry_sender,
                response_sender_clone,
                config_for_parser,
            );
        }));
        
        // Start geometry processing thread
        let response_sender_clone = response_sender.clone();
        let cache_clone = geometry_cache.clone();
        
        let config_for_geometry = self.config.clone();
        self.geometry_thread = Some(thread::spawn(move || {
            Self::geometry_thread_worker(
                geometry_receiver,
                response_sender_clone,
                cache_clone,
                config_for_geometry,
            );
        }));
        
        // Start main command processing thread
        let parse_sender_clone = self.parse_sender.clone();
        let cache_clone = geometry_cache;
        
        thread::spawn(move || {
            Self::command_thread_worker(
                command_receiver,
                response_sender,
                parse_sender_clone,
                cache_clone,
            );
        });
        
        Ok(())
    }
    
    // Main command processing thread
    fn command_thread_worker(
        command_receiver: mpsc::Receiver<ProcessorMessage>,
        response_sender: mpsc::Sender<ProcessorResponse>,
        parse_sender: mpsc::Sender<ParsingTask>,
        geometry_cache: Arc<Mutex<GeometryCache>>,
    ) {
        let mut task_id = 0u64;
        
        while let Ok(message) = command_receiver.recv() {
            match message {
                ProcessorMessage::LoadFile(file_path) => {
                    // Check cache first
                    {
                        let mut cache = geometry_cache.lock().unwrap();
                        if let Some(entry) = cache.find_entry(&file_path) {
                            // Cache hit - return geometry immediately
                            let geometry = entry.geometry.clone();
                            entry.last_accessed = Instant::now();
                            
                            let _ = response_sender.send(ProcessorResponse::GeometryReady(geometry));
                            let _ = response_sender.send(ProcessorResponse::LoadCompleted {
                                line_count: entry.geometry.line_segments.len() as u32,
                                geometry_stats: Self::calculate_geometry_stats(&entry.geometry),
                                parsing_time: Duration::from_millis(0), // From cache
                            });
                            continue;
                        }
                    }
                    
                    // Cache miss - start parsing
                    let _ = response_sender.send(ProcessorResponse::LoadStarted);
                    
                    let parsing_task = ParsingTask {
                        file_path,
                        task_id,
                        priority: 0,
                    };
                    
                    if parse_sender.send(parsing_task).is_err() {
                        let _ = response_sender.send(ProcessorResponse::LoadError(
                            "Failed to queue parsing task".to_string()
                        ));
                    }
                    
                    task_id += 1;
                }
                
                ProcessorMessage::Shutdown => {
                    let _ = response_sender.send(ProcessorResponse::ShutdownComplete);
                    break;
                }
                
                _ => {
                    // Other messages handled elsewhere
                }
            }
        }
    }
    
    // Parsing thread worker
    fn parsing_thread_worker(
        parse_receiver: mpsc::Receiver<ParsingTask>,
        geometry_sender: mpsc::Sender<GeometryTask>,
        response_sender: mpsc::Sender<ProcessorResponse>,
        config: ProcessorConfig,
    ) {
        while let Ok(task) = parse_receiver.recv() {
            let start_time = Instant::now();
            
            // Parse the file
            match Self::parse_file_async(&task.file_path, &config) {
                Ok(commands) => {
                    let parsing_time = start_time.elapsed();
                    
                    // Send parsed commands to geometry processing
                    let geometry_task = GeometryTask {
                        commands,
                        task_id: task.task_id,
                        response_sender: response_sender.clone(),
                    };
                    
                    if geometry_sender.send(geometry_task).is_err() {
                        let _ = response_sender.send(ProcessorResponse::LoadError(
                            "Failed to queue geometry generation".to_string()
                        ));
                    }
                }
                Err(e) => {
                    let _ = response_sender.send(ProcessorResponse::LoadError(
                        format!("Parsing failed: {}", e)
                    ));
                }
            }
        }
    }
    
    // Geometry processing thread worker
    fn geometry_thread_worker(
        geometry_receiver: mpsc::Receiver<GeometryTask>,
        response_sender: mpsc::Sender<ProcessorResponse>,
        geometry_cache: Arc<Mutex<GeometryCache>>,
        config: ProcessorConfig,
    ) {
        while let Ok(task) = geometry_receiver.recv() {
            let start_time = Instant::now();
            
            // Generate geometry from commands with optimizations
            let mut generator = GeometryGenerator::new();
            
            // Pre-process geometry for different LOD levels
            let base_geometry = generator.generate_geometry(&task.commands);
            let mut processed_geometry = Self::preprocess_geometry(base_geometry, &config);
            
            // Generate simplified versions for LOD
            processed_geometry = Self::generate_lod_geometry(processed_geometry, &config.lod_config);
            
            let geometry_time = start_time.elapsed();
            
            // Cache the processed geometry
            let memory_size = Self::estimate_geometry_memory_size(&processed_geometry);
            {
                let mut cache = geometry_cache.lock().unwrap();
                let cache_entry = CacheEntry {
                    file_path: format!("task_{}", task.task_id), // Use task ID as temporary key
                    geometry: processed_geometry.clone(),
                    last_accessed: Instant::now(),
                    memory_size,
                };
                cache.add_entry(cache_entry);
            }
            
            // Send geometry back
            let stats = Self::calculate_geometry_stats(&processed_geometry);
            let _ = task.response_sender.send(ProcessorResponse::GeometryReady(processed_geometry));
            let _ = task.response_sender.send(ProcessorResponse::LoadCompleted {
                line_count: task.commands.len() as u32,
                geometry_stats: stats,
                parsing_time: geometry_time,
            });
        }
    }
    
    // Pre-process geometry for optimal rendering performance
    fn preprocess_geometry(mut geometry: ToolpathGeometry, config: &ProcessorConfig) -> ToolpathGeometry {
        // Sort line segments by type for better rendering batching
        geometry.line_segments.sort_by_key(|segment| segment.line_type as u8);
        
        // Pre-calculate vertex buffer optimizations
        geometry = Self::optimize_vertex_order(geometry);
        
        // Remove duplicate vertices and merge adjacent identical segments
        geometry = Self::merge_adjacent_segments(geometry);
        
        // Pre-calculate bounding volumes for frustum culling
        geometry = Self::calculate_detailed_bounds(geometry);
        
        geometry
    }
    
    // Generate Level of Detail versions of the geometry
    fn generate_lod_geometry(mut geometry: ToolpathGeometry, lod_config: &LODConfig) -> ToolpathGeometry {
        // Create simplified versions for different distances
        let original_segment_count = geometry.line_segments.len();
        
        // For very large files, create simplified representations
        if original_segment_count > 100_000 {
            // Simplify travel moves more aggressively
            geometry.line_segments = geometry.line_segments
                .into_iter()
                .enumerate()
                .filter(|(i, segment)| {
                    match segment.line_type {
                        crate::fast_gcode_parser::LineType::Travel => {
                            // Keep every 4th travel move for large files
                            i % 4 == 0
                        },
                        _ => true, // Keep all extrusion moves
                    }
                })
                .map(|(_, segment)| segment)
                .collect();
        }
        
        geometry
    }
    
    // Optimize vertex order for better GPU cache performance
    fn optimize_vertex_order(mut geometry: ToolpathGeometry) -> ToolpathGeometry {
        // Simple optimization: ensure vertices are ordered to minimize cache misses
        // This is a simplified version - real implementations would use more sophisticated algorithms
        
        let mut optimized_vertices = Vec::new();
        let mut optimized_colors = Vec::new();
        let mut optimized_indices = Vec::new();
        let mut vertex_map = std::collections::HashMap::new();
        
        let mut next_index = 0;
        
        for &index in &geometry.indices {
            if let Some(&new_index) = vertex_map.get(&index) {
                optimized_indices.push(new_index);
            } else {
                vertex_map.insert(index, next_index);
                optimized_indices.push(next_index);
                
                if let Some(vertex) = geometry.vertices.get(index as usize) {
                    optimized_vertices.push(*vertex);
                }
                if let Some(color) = geometry.colors.get(index as usize) {
                    optimized_colors.push(*color);
                }
                
                next_index += 1;
            }
        }
        
        geometry.vertices = optimized_vertices;
        geometry.colors = optimized_colors;
        geometry.indices = optimized_indices;
        
        geometry
    }
    
    // Merge adjacent line segments with identical properties
    fn merge_adjacent_segments(mut geometry: ToolpathGeometry) -> ToolpathGeometry {
        let mut merged_segments = Vec::new();
        
        for segment in geometry.line_segments {
            if let Some(last_segment) = merged_segments.last_mut() {
                // Check if segments can be merged (same color, type, and connected)
                if Self::can_merge_segments(last_segment, &segment) {
                    last_segment.end = segment.end;
                    continue;
                }
            }
            merged_segments.push(segment);
        }
        
        geometry.line_segments = merged_segments;
        geometry
    }
    
    fn can_merge_segments(seg1: &crate::fast_gcode_parser::LineSegment, seg2: &crate::fast_gcode_parser::LineSegment) -> bool {
        // Check if segments are connected and have same properties
        const EPSILON: f32 = 1e-6;
        
        (seg1.end[0] - seg2.start[0]).abs() < EPSILON &&
        (seg1.end[1] - seg2.start[1]).abs() < EPSILON &&
        (seg1.end[2] - seg2.start[2]).abs() < EPSILON &&
        seg1.line_type == seg2.line_type &&
        (seg1.color[0] - seg2.color[0]).abs() < EPSILON &&
        (seg1.color[1] - seg2.color[1]).abs() < EPSILON &&
        (seg1.color[2] - seg2.color[2]).abs() < EPSILON &&
        (seg1.color[3] - seg2.color[3]).abs() < EPSILON
    }
    
    // Calculate detailed bounding information for rendering optimizations
    fn calculate_detailed_bounds(mut geometry: ToolpathGeometry) -> ToolpathGeometry {
        if geometry.line_segments.is_empty() {
            return geometry;
        }
        
        let mut min_x = f32::INFINITY;
        let mut max_x = f32::NEG_INFINITY;
        let mut min_y = f32::INFINITY;
        let mut max_y = f32::NEG_INFINITY;
        let mut min_z = f32::INFINITY;
        let mut max_z = f32::NEG_INFINITY;
        
        for segment in &geometry.line_segments {
            // Check start point
            min_x = min_x.min(segment.start[0]);
            max_x = max_x.max(segment.start[0]);
            min_y = min_y.min(segment.start[1]);
            max_y = max_y.max(segment.start[1]);
            min_z = min_z.min(segment.start[2]);
            max_z = max_z.max(segment.start[2]);
            
            // Check end point
            min_x = min_x.min(segment.end[0]);
            max_x = max_x.max(segment.end[0]);
            min_y = min_y.min(segment.end[1]);
            max_y = max_y.max(segment.end[1]);
            min_z = min_z.min(segment.end[2]);
            max_z = max_z.max(segment.end[2]);
        }
        
        geometry.bounds = crate::fast_gcode_parser::BoundingBox {
            min: [min_x, min_y, min_z],
            max: [max_x, max_y, max_z],
        };
        
        geometry
    }
    
    fn estimate_geometry_memory_size(geometry: &ToolpathGeometry) -> usize {
        let vertices_size = geometry.vertices.len() * std::mem::size_of::<[f32; 3]>();
        let colors_size = geometry.colors.len() * std::mem::size_of::<[f32; 4]>();
        let indices_size = geometry.indices.len() * std::mem::size_of::<u32>();
        let segments_size = geometry.line_segments.len() * std::mem::size_of::<crate::fast_gcode_parser::LineSegment>();
        
        vertices_size + colors_size + indices_size + segments_size
    }
    
    fn parse_file_async(file_path: &str, config: &ProcessorConfig) -> Result<Vec<GCodeCommand>, ProcessorError> {
        let file = File::open(file_path)?;
        let mut parser = StreamingGCodeParser::new()
            .with_buffer_size(config.buffer_size)
            .with_parallel_chunks(config.parsing_threads);
        
        parser.parse_file(file).map_err(ProcessorError::ParseError)
    }
    
    fn calculate_geometry_stats(geometry: &ToolpathGeometry) -> GeometryStats {
        let mut stats = GeometryStats {
            total_lines: geometry.line_segments.len() as u32,
            extrude_lines: 0,
            travel_lines: 0,
            total_distance: 0.0,
            print_time_estimate: 0.0,
            bounds: [
                geometry.bounds.min[0], geometry.bounds.max[0],
                geometry.bounds.min[1], geometry.bounds.max[1],
                geometry.bounds.min[2], geometry.bounds.max[2],
            ],
            layer_count: 0,
        };
        
        let mut last_z = f32::NEG_INFINITY;
        
        for segment in &geometry.line_segments {
            match segment.line_type {
                crate::fast_gcode_parser::LineType::Extrude => stats.extrude_lines += 1,
                crate::fast_gcode_parser::LineType::Travel => stats.travel_lines += 1,
                _ => {}
            }
            
            // Calculate distance
            let dx = segment.end[0] - segment.start[0];
            let dy = segment.end[1] - segment.start[1];
            let dz = segment.end[2] - segment.start[2];
            stats.total_distance += (dx * dx + dy * dy + dz * dz).sqrt();
            
            // Count layers (approximate)
            if (segment.start[2] - last_z).abs() > 0.1 {
                stats.layer_count += 1;
                last_z = segment.start[2];
            }
        }
        
        // Estimate print time (very rough)
        stats.print_time_estimate = stats.total_distance / 60.0; // Assume 60mm/s average
        
        stats
    }
    
    // Public API methods
    pub fn load_file<P: AsRef<Path>>(&mut self, file_path: P) -> Result<(), ProcessorError> {
        let path_str = file_path.as_ref().to_string_lossy().to_string();
        
        self.command_sender
            .send(ProcessorMessage::LoadFile(path_str.clone()))
            .map_err(|_| ProcessorError::ThreadCommunication)?;
        
        self.current_file = Some(path_str);
        self.is_loading = true;
        
        Ok(())
    }
    
    pub fn is_loading(&self) -> bool {
        self.is_loading
    }
    
    pub fn get_current_file(&self) -> Option<&String> {
        self.current_file.as_ref()
    }
    
    pub fn poll_responses(&mut self) -> Vec<ProcessorResponse> {
        let mut responses = Vec::new();
        
        while let Ok(response) = self.response_receiver.try_recv() {
            match &response {
                ProcessorResponse::LoadCompleted { .. } => {
                    self.is_loading = false;
                    self.performance_stats.total_files_processed += 1;
                }
                ProcessorResponse::LoadError(_) => {
                    self.is_loading = false;
                }
                _ => {}
            }
            
            responses.push(response);
        }
        
        responses
    }
    
    pub fn get_cache_info(&self) -> Option<Vec<(String, usize, Duration)>> {
        self.geometry_cache.lock().ok().map(|cache| {
            cache.entries.iter().map(|entry| {
                (
                    entry.file_path.clone(),
                    entry.memory_size,
                    entry.last_accessed.elapsed(),
                )
            }).collect()
        })
    }
    
    pub fn clear_cache(&mut self) {
        if let Ok(mut cache) = self.geometry_cache.lock() {
            cache.entries.clear();
            cache.total_memory = 0;
        }
    }
    
    pub fn get_performance_stats(&self) -> &PerformanceStats {
        &self.performance_stats
    }
    
    pub fn shutdown(&mut self) -> Result<(), ProcessorError> {
        // Send shutdown message
        self.command_sender
            .send(ProcessorMessage::Shutdown)
            .map_err(|_| ProcessorError::ThreadCommunication)?;
        
        // Wait for threads to complete
        if let Some(thread) = self.parsing_thread.take() {
            let _ = thread.join();
        }
        
        if let Some(thread) = self.geometry_thread.take() {
            let _ = thread.join();
        }
        
        Ok(())
    }
}

impl GeometryCache {
    fn find_entry(&mut self, file_path: &str) -> Option<&mut CacheEntry> {
        self.entries
            .iter_mut()
            .find(|entry| entry.file_path == file_path)
    }
    
    fn add_entry(&mut self, entry: CacheEntry) {
        // Remove oldest entries if cache is full
        while self.entries.len() >= self.max_size && !self.entries.is_empty() {
            if let Some(old_entry) = self.entries.pop_back() {
                self.total_memory -= old_entry.memory_size;
            }
        }
        
        self.total_memory += entry.memory_size;
        self.entries.push_front(entry);
    }
}

impl Drop for ConcurrentGCodeProcessor {
    fn drop(&mut self) {
        let _ = self.shutdown();
    }
}

#[derive(Debug, thiserror::Error)]
pub enum ProcessorError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    
    #[error("Parse error: {0}")]
    ParseError(#[from] crate::fast_gcode_parser::ParseError),
    
    #[error("Thread communication error")]
    ThreadCommunication,
    
    #[error("Configuration error: {0}")]
    Configuration(String),
}

// Integration helper for easy use with existing code
pub struct PerformanceOptimizedGCodeLoader {
    processor: ConcurrentGCodeProcessor,
    viewer: Option<Optimized3DViewer>,
    last_update: Instant,
    update_interval: Duration,
}

impl PerformanceOptimizedGCodeLoader {
    pub fn new(config: ProcessorConfig) -> Result<Self, ProcessorError> {
        let processor = ConcurrentGCodeProcessor::new(config)?;
        
        Ok(Self {
            processor,
            viewer: None,
            last_update: Instant::now(),
            update_interval: Duration::from_millis(16), // ~60 FPS
        })
    }
    
    pub fn set_viewer(&mut self, viewer: Optimized3DViewer) {
        self.viewer = Some(viewer);
    }
    
    pub fn load_file_async<P: AsRef<Path>>(&mut self, file_path: P) -> Result<(), ProcessorError> {
        self.processor.load_file(file_path)
    }
    
    pub fn update(&mut self) -> Vec<ProcessorResponse> {
        let now = Instant::now();
        if now.duration_since(self.last_update) < self.update_interval {
            return Vec::new();
        }
        
        self.last_update = now;
        
        let responses = self.processor.poll_responses();
        
        // Handle geometry updates
        for response in &responses {
            if let ProcessorResponse::GeometryReady(geometry) = response {
                if let Some(viewer) = &mut self.viewer {
                    let _ = viewer.load_geometry(geometry.clone());
                }
            }
        }
        
        responses
    }
    
    pub fn is_loading(&self) -> bool {
        self.processor.is_loading()
    }
    
    pub fn get_viewer_mut(&mut self) -> Option<&mut Optimized3DViewer> {
        self.viewer.as_mut()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    
    #[test]
    fn test_concurrent_processor_creation() {
        let config = ProcessorConfig::default();
        let processor = ConcurrentGCodeProcessor::new(config);
        assert!(processor.is_ok());
    }
    
    #[test]
    fn test_geometry_stats_calculation() {
        // Create mock geometry for testing
        let geometry = crate::fast_gcode_parser::ToolpathGeometry {
            vertices: vec![[0.0, 0.0, 0.0], [10.0, 10.0, 1.0]],
            colors: vec![[1.0, 0.0, 0.0, 1.0], [1.0, 0.0, 0.0, 1.0]],
            indices: vec![0, 1],
            line_segments: vec![crate::fast_gcode_parser::LineSegment {
                start: [0.0, 0.0, 0.0],
                end: [10.0, 10.0, 1.0],
                color: [1.0, 0.0, 0.0, 1.0],
                line_type: crate::fast_gcode_parser::LineType::Extrude,
            }],
            bounds: crate::fast_gcode_parser::BoundingBox {
                min: [0.0, 0.0, 0.0],
                max: [10.0, 10.0, 1.0],
            },
        };
        
        let stats = ConcurrentGCodeProcessor::calculate_geometry_stats(&geometry);
        
        assert_eq!(stats.total_lines, 1);
        assert_eq!(stats.extrude_lines, 1);
        assert_eq!(stats.travel_lines, 0);
        assert!(stats.total_distance > 0.0);
    }
}