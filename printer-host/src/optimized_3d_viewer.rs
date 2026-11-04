// High-Performance 3D Viewer with Hardware Acceleration and LOD
// Uses GPU-accelerated rendering with Level of Detail for smooth interaction
// Designed for large g-code files and real-time visualization

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Instant;
use wgpu::util::DeviceExt;
use crate::fast_gcode_parser::{ToolpathGeometry, LineSegment, LineType, BoundingBox};

// Vertex structure for GPU rendering
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    position: [f32; 3],
    color: [f32; 4],
}

impl Vertex {
    fn desc() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                // Position
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },
                // Color
                wgpu::VertexAttribute {
                    offset: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x4,
                },
            ],
        }
    }
}

// Camera uniform buffer for GPU
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CameraUniform {
    view_proj: [[f32; 4]; 4],
}

// Level of Detail configuration
#[derive(Debug, Clone)]
pub struct LODConfig {
    pub level_count: usize,
    pub distance_thresholds: Vec<f32>,
    pub simplification_ratios: Vec<f32>,
    pub min_line_length: f32,
}

impl Default for LODConfig {
    fn default() -> Self {
        Self {
            level_count: 4,
            distance_thresholds: vec![10.0, 50.0, 200.0, 1000.0],
            simplification_ratios: vec![1.0, 0.75, 0.5, 0.25],
            min_line_length: 0.1,
        }
    }
}

// LOD geometry level
#[derive(Debug, Clone)]
pub struct LODLevel {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub line_count: usize,
    pub vertex_buffer: Option<wgpu::Buffer>,
    pub index_buffer: Option<wgpu::Buffer>,
}

// High-performance 3D viewer
pub struct Optimized3DViewer {
    device: Arc<wgpu::Device>,
    queue: Arc<wgpu::Queue>,
    render_pipeline: wgpu::RenderPipeline,
    camera_buffer: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,
    
    // Geometry data
    original_geometry: Option<ToolpathGeometry>,
    lod_levels: Vec<LODLevel>,
    lod_config: LODConfig,
    current_lod_level: usize,
    
    // Camera and view state
    camera_position: [f32; 3],
    camera_target: [f32; 3],
    camera_up: [f32; 3],
    projection_matrix: nalgebra::Matrix4<f32>,
    view_matrix: nalgebra::Matrix4<f32>,
    
    // Performance monitoring
    frame_times: Vec<f32>,
    render_stats: RenderStatistics,
    
    // Culling and optimization
    frustum_culling: bool,
    back_face_culling: bool,
    auto_lod: bool,
    target_fps: f32,
}

#[derive(Debug, Default)]
pub struct RenderStatistics {
    pub frame_time_ms: f32,
    pub vertices_rendered: u32,
    pub triangles_rendered: u32,
    pub draw_calls: u32,
    pub current_lod_level: usize,
    pub culled_objects: u32,
    pub fps: f32,
}

impl Optimized3DViewer {
    pub async fn new(
        device: Arc<wgpu::Device>,
        queue: Arc<wgpu::Queue>,
        surface_format: wgpu::TextureFormat,
    ) -> Result<Self, ViewerError> {
        // Create shader modules
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("3D Viewer Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/viewer.wgsl").into()),
        });
        
        // Create camera uniform buffer
        let camera_uniform = CameraUniform {
            view_proj: nalgebra::Matrix4::identity().into(),
        };
        
        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Camera Buffer"),
            contents: bytemuck::cast_slice(&[camera_uniform]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        
        // Create bind group layout for camera
        let camera_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
                label: Some("Camera Bind Group Layout"),
            });
        
        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &camera_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
            label: Some("Camera Bind Group"),
        });
        
        // Create render pipeline layout
        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Render Pipeline Layout"),
                bind_group_layouts: &[&camera_bind_group_layout],
                push_constant_ranges: &[],
            });
        
        // Create render pipeline
        let render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Render Pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[Vertex::desc()],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: surface_format,
                    blend: Some(wgpu::BlendState {
                        color: wgpu::BlendComponent::REPLACE,
                        alpha: wgpu::BlendComponent::REPLACE,
                    }),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList, // For line rendering
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            multiview: None,
            cache: None,
        });
        
        // Initialize camera
        let camera_position = [0.0, 0.0, 100.0];
        let camera_target = [0.0, 0.0, 0.0];
        let camera_up = [0.0, 1.0, 0.0];
        
        let projection_matrix = nalgebra::Perspective3::new(16.0 / 9.0, 45.0_f32.to_radians(), 0.1, 1000.0)
            .to_homogeneous();
        let view_matrix = nalgebra::Isometry3::look_at_rh(
            &nalgebra::Point3::new(camera_position[0], camera_position[1], camera_position[2]),
            &nalgebra::Point3::new(camera_target[0], camera_target[1], camera_target[2]),
            &nalgebra::Vector3::new(camera_up[0], camera_up[1], camera_up[2]),
        ).to_homogeneous();
        
        Ok(Self {
            device,
            queue,
            render_pipeline,
            camera_buffer,
            camera_bind_group,
            original_geometry: None,
            lod_levels: Vec::new(),
            lod_config: LODConfig::default(),
            current_lod_level: 0,
            camera_position,
            camera_target,
            camera_up,
            projection_matrix,
            view_matrix,
            frame_times: Vec::with_capacity(100),
            render_stats: RenderStatistics::default(),
            frustum_culling: true,
            back_face_culling: true,
            auto_lod: true,
            target_fps: 60.0,
        })
    }
    
    /// Load geometry and generate LOD levels
    pub fn load_geometry(&mut self, geometry: ToolpathGeometry) -> Result<(), ViewerError> {
        println!("Loading geometry with {} line segments", geometry.line_segments.len());
        
        // Store original geometry
        self.original_geometry = Some(geometry.clone());
        
        // Generate LOD levels
        self.generate_lod_levels(&geometry)?;
        
        // Create GPU buffers for all LOD levels
        self.create_gpu_buffers()?;
        
        // Set up initial camera position based on bounds
        self.setup_camera_for_bounds(&geometry.bounds);
        
        println!("Generated {} LOD levels", self.lod_levels.len());
        Ok(())
    }
    
    fn generate_lod_levels(&mut self, geometry: &ToolpathGeometry) -> Result<(), ViewerError> {
        self.lod_levels.clear();
        
        for (level, &simplification_ratio) in self.lod_config.simplification_ratios.iter().enumerate() {
            let lod_level = self.create_lod_level(geometry, simplification_ratio, level)?;
            self.lod_levels.push(lod_level);
        }
        
        Ok(())
    }
    
    fn create_lod_level(
        &self,
        geometry: &ToolpathGeometry,
        simplification_ratio: f32,
        level: usize,
    ) -> Result<LODLevel, ViewerError> {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        
        // Calculate how many segments to keep based on simplification ratio
        let segments_to_keep = ((geometry.line_segments.len() as f32) * simplification_ratio) as usize;
        let step = if segments_to_keep > 0 {
            geometry.line_segments.len() / segments_to_keep
        } else {
            1
        }.max(1);
        
        let mut vertex_count = 0u32;
        let mut line_count = 0;
        
        // Process line segments with LOD-appropriate simplification
        for (i, segment) in geometry.line_segments.iter().enumerate() {
            // Skip segments based on LOD level
            if level > 0 && i % step != 0 {
                // For higher LOD levels, skip some segments
                continue;
            }
            
            // Skip very short lines at higher LOD levels
            if level > 1 {
                let length = self.calculate_line_length(segment);
                if length < self.lod_config.min_line_length * (level as f32) {
                    continue;
                }
            }
            
            // Add vertices for this line segment
            let start_vertex = Vertex {
                position: segment.start,
                color: segment.color,
            };
            let end_vertex = Vertex {
                position: segment.end,
                color: segment.color,
            };
            
            vertices.push(start_vertex);
            vertices.push(end_vertex);
            
            // Add indices for line rendering
            indices.push(vertex_count);
            indices.push(vertex_count + 1);
            
            vertex_count += 2;
            line_count += 1;
        }
        
        println!("LOD Level {}: {} lines, {} vertices", level, line_count, vertices.len());
        
        Ok(LODLevel {
            vertices,
            indices,
            line_count,
            vertex_buffer: None,
            index_buffer: None,
        })
    }
    
    fn create_gpu_buffers(&mut self) -> Result<(), ViewerError> {
        for lod_level in &mut self.lod_levels {
            if !lod_level.vertices.is_empty() {
                // Create vertex buffer
                let vertex_buffer = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("LOD Vertex Buffer"),
                    contents: bytemuck::cast_slice(&lod_level.vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                });
                
                // Create index buffer
                let index_buffer = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("LOD Index Buffer"),
                    contents: bytemuck::cast_slice(&lod_level.indices),
                    usage: wgpu::BufferUsages::INDEX,
                });
                
                lod_level.vertex_buffer = Some(vertex_buffer);
                lod_level.index_buffer = Some(index_buffer);
            }
        }
        
        Ok(())
    }
    
    fn calculate_line_length(&self, segment: &LineSegment) -> f32 {
        let dx = segment.end[0] - segment.start[0];
        let dy = segment.end[1] - segment.start[1];
        let dz = segment.end[2] - segment.start[2];
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
    
    fn setup_camera_for_bounds(&mut self, bounds: &BoundingBox) {
        let center = bounds.center();
        let size = bounds.size();
        let max_dimension = size[0].max(size[1]).max(size[2]);
        
        // Position camera to view the entire model
        let distance = max_dimension * 1.5;
        self.camera_position = [center[0], center[1], center[2] + distance];
        self.camera_target = center;
        
        self.update_view_matrix();
    }
    
    fn update_view_matrix(&mut self) {
        self.view_matrix = nalgebra::Isometry3::look_at_rh(
            &nalgebra::Point3::new(self.camera_position[0], self.camera_position[1], self.camera_position[2]),
            &nalgebra::Point3::new(self.camera_target[0], self.camera_target[1], self.camera_target[2]),
            &nalgebra::Vector3::new(self.camera_up[0], self.camera_up[1], self.camera_up[2]),
        ).to_homogeneous();
    }
    
    /// Update LOD level based on camera distance and performance
    pub fn update_lod(&mut self, camera_distance: f32) {
        if !self.auto_lod || self.lod_levels.is_empty() {
            return;
        }
        
        // Determine LOD level based on distance and performance
        let mut target_level = 0;
        
        for (i, &threshold) in self.lod_config.distance_thresholds.iter().enumerate() {
            if camera_distance > threshold {
                target_level = (i + 1).min(self.lod_levels.len() - 1);
            }
        }
        
        // Adjust for performance if FPS is too low
        if self.render_stats.fps < self.target_fps * 0.8 && target_level < self.lod_levels.len() - 1 {
            target_level += 1;
        } else if self.render_stats.fps > self.target_fps * 1.2 && target_level > 0 {
            target_level -= 1;
        }
        
        if target_level != self.current_lod_level {
            self.current_lod_level = target_level;
            println!("Switched to LOD level {} (distance: {:.1}, fps: {:.1})", 
                     target_level, camera_distance, self.render_stats.fps);
        }
    }
    
    /// Render the 3D scene
    pub fn render(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        color_view: &wgpu::TextureView,
        depth_view: &wgpu::TextureView,
    ) -> Result<(), ViewerError> {
        let frame_start = Instant::now();
        
        // Update camera uniform buffer
        let view_proj = self.projection_matrix * self.view_matrix;
        let camera_uniform = CameraUniform {
            view_proj: view_proj.into(),
        };
        
        self.queue.write_buffer(
            &self.camera_buffer,
            0,
            bytemuck::cast_slice(&[camera_uniform]),
        );
        
        // Begin render pass
        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("3D Viewer Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: color_view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.1,
                            g: 0.1,
                            b: 0.1,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            
            render_pass.set_pipeline(&self.render_pipeline);
            render_pass.set_bind_group(0, &self.camera_bind_group, &[]);
            
            // Render current LOD level
            if let Some(lod_level) = self.lod_levels.get(self.current_lod_level) {
                if let (Some(vertex_buffer), Some(index_buffer)) = 
                    (&lod_level.vertex_buffer, &lod_level.index_buffer) {
                    
                    render_pass.set_vertex_buffer(0, vertex_buffer.slice(..));
                    render_pass.set_index_buffer(index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    render_pass.draw_indexed(0..lod_level.indices.len() as u32, 0, 0..1);
                    
                    // Update render statistics
                    self.render_stats.vertices_rendered = lod_level.vertices.len() as u32;
                    self.render_stats.triangles_rendered = (lod_level.indices.len() / 2) as u32; // Lines, not triangles
                    self.render_stats.draw_calls = 1;
                    self.render_stats.current_lod_level = self.current_lod_level;
                }
            }
        }
        
        // Update performance statistics
        let frame_time = frame_start.elapsed();
        self.render_stats.frame_time_ms = frame_time.as_secs_f32() * 1000.0;
        
        // Update FPS calculation (rolling average)
        self.frame_times.push(frame_time.as_secs_f32());
        if self.frame_times.len() > 100 {
            self.frame_times.remove(0);
        }
        
        if !self.frame_times.is_empty() {
            let avg_frame_time = self.frame_times.iter().sum::<f32>() / self.frame_times.len() as f32;
            self.render_stats.fps = if avg_frame_time > 0.0 { 1.0 / avg_frame_time } else { 0.0 };
        }
        
        Ok(())
    }
    
    /// Update camera position (for mouse/keyboard controls)
    pub fn update_camera(&mut self, position: [f32; 3], target: [f32; 3]) {
        self.camera_position = position;
        self.camera_target = target;
        self.update_view_matrix();
        
        // Update LOD based on new camera distance
        let distance = {
            let dx = position[0] - target[0];
            let dy = position[1] - target[1];
            let dz = position[2] - target[2];
            (dx * dx + dy * dy + dz * dz).sqrt()
        };
        
        self.update_lod(distance);
    }
    
    /// Get current render statistics
    pub fn get_render_stats(&self) -> &RenderStatistics {
        &self.render_stats
    }
    
    /// Configure LOD system
    pub fn set_lod_config(&mut self, config: LODConfig) {
        self.lod_config = config;
        
        // Regenerate LOD levels if geometry is loaded
        if let Some(ref geometry) = self.original_geometry.clone() {
            let _ = self.generate_lod_levels(geometry);
            let _ = self.create_gpu_buffers();
        }
    }
    
    /// Enable/disable automatic LOD
    pub fn set_auto_lod(&mut self, enabled: bool) {
        self.auto_lod = enabled;
    }
    
    /// Set target FPS for performance-based LOD adjustment
    pub fn set_target_fps(&mut self, fps: f32) {
        self.target_fps = fps;
    }
    
    /// Manually set LOD level
    pub fn set_lod_level(&mut self, level: usize) {
        if level < self.lod_levels.len() {
            self.current_lod_level = level;
            self.auto_lod = false; // Disable auto LOD when manually set
        }
    }
    
    /// Get available LOD levels info
    pub fn get_lod_info(&self) -> Vec<(usize, usize, usize)> {
        self.lod_levels
            .iter()
            .enumerate()
            .map(|(i, level)| (i, level.line_count, level.vertices.len()))
            .collect()
    }
}

#[derive(Debug, thiserror::Error)]
pub enum ViewerError {
    #[error("GPU error: {0}")]
    Gpu(String),
    
    #[error("Geometry error: {0}")]
    Geometry(String),
    
    #[error("Shader compilation error: {0}")]
    Shader(String),
}

impl From<wgpu::RequestDeviceError> for ViewerError {
    fn from(err: wgpu::RequestDeviceError) -> Self {
        ViewerError::Gpu(format!("Device request failed: {}", err))
    }
}

impl From<wgpu::CreateSurfaceError> for ViewerError {
    fn from(err: wgpu::CreateSurfaceError) -> Self {
        ViewerError::Gpu(format!("Surface creation failed: {}", err))
    }
}