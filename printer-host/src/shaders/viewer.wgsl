// High-Performance 3D Viewer Shader for G-Code Visualization
// Optimized for line rendering with hardware acceleration

// Camera uniform buffer
struct CameraUniform {
    view_proj: mat4x4<f32>,
}

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

// Vertex input
struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) color: vec4<f32>,
}

// Vertex output / Fragment input
struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) world_position: vec3<f32>,
}

// Vertex shader
@vertex
fn vs_main(model: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    
    // Transform vertex position to clip space
    out.clip_position = camera.view_proj * vec4<f32>(model.position, 1.0);
    
    // Pass through color and world position
    out.color = model.color;
    out.world_position = model.position;
    
    return out;
}

// Fragment shader
@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    var color = in.color;
    
    // Add depth-based fog for better depth perception
    let fog_start = 100.0;
    let fog_end = 1000.0;
    let fog_color = vec3<f32>(0.1, 0.1, 0.1);
    
    // Calculate distance from camera (approximate)
    let depth = in.clip_position.z / in.clip_position.w;
    let fog_factor = clamp((fog_end - depth) / (fog_end - fog_start), 0.0, 1.0);
    
    // Mix with fog color
    color = vec4<f32>(
        mix(fog_color, color.rgb, fog_factor),
        color.a
    );
    
    // Add slight brightness adjustment based on line type
    // Travel moves (green) get slight transparency
    if (color.g > 0.8 && color.r < 0.2 && color.b < 0.2) {
        color.a *= 0.6; // Make travel moves more transparent
    }
    
    return color;
}