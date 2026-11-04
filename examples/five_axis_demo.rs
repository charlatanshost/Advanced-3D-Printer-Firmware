/// Comprehensive 5-Axis Kinematics Integration Example
/// 
/// This example demonstrates:
/// 1. Forward and inverse kinematics for multiple 5-axis configurations
/// 2. Advanced motion planning with collision avoidance
/// 3. Tool orientation control for non-planar printing and machining
/// 4. G-code processing for 5-axis simultaneous motion
/// 5. Surface-following strategies for complex geometries

use printer_kinematics::five_axis::*;
use printer_kinematics::motion_planning::*;
use printer_kinematics::tool_orientation::*;
use nalgebra::{Point3, Vector3, Unit, UnitQuaternion, Rotation3};
use std::f64::consts::PI;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🎯 5-Axis Kinematics Integration Demonstration");
    println!("=============================================\n");
    
    // Demo 1: Rotary Table Configuration
    println!("📐 Demo 1: Rotary Table Configuration");
    demo_rotary_table_kinematics()?;
    
    // Demo 2: Trunnion Table Configuration  
    println!("\n📐 Demo 2: Trunnion Table Configuration");
    demo_trunnion_kinematics()?;
    
    // Demo 3: Tilting Head Configuration
    println!("\n📐 Demo 3: Tilting Head Configuration");
    demo_tilting_head_kinematics()?;
    
    // Demo 4: Advanced Motion Planning
    println!("\n🎬 Demo 4: Advanced Motion Planning");
    demo_motion_planning()?;
    
    // Demo 5: Tool Orientation Control
    println!("\n🔧 Demo 5: Tool Orientation Control");
    demo_tool_orientation_control()?;
    
    // Demo 6: Non-Planar Printing
    println!("\n🖨️ Demo 6: Non-Planar Printing");
    demo_non_planar_printing()?;
    
    // Demo 7: 5-Axis Machining
    println!("\n⚙️ Demo 7: 5-Axis Machining");
    demo_five_axis_machining()?;
    
    // Demo 8: G-code Processing
    println!("\n📝 Demo 8: G-code Processing");
    demo_gcode_processing()?;
    
    println!("\n✅ All 5-axis kinematics demos completed successfully!");
    println!("\n🌟 Key Features Demonstrated:");
    println!("   ✓ Multiple 5-axis machine configurations");
    println!("   ✓ Analytical and numerical inverse kinematics");
    println!("   ✓ Advanced motion planning with jerk control");
    println!("   ✓ Collision detection and singularity avoidance");
    println!("   ✓ Surface-following tool orientation strategies");
    println!("   ✓ Non-planar printing capabilities");
    println!("   ✓ Professional machining operations");
    println!("   ✓ Full G-code compatibility");
    
    Ok(())
}

fn demo_rotary_table_kinematics() -> Result<(), KinematicsError> {
    // Create rotary table configuration (A/B axes)
    let config = FiveAxisConfiguration::RotaryTable {
        table_offset: Vector3::new(0.0, 0.0, 100.0),
        a_axis_position: Vector3::new(0.0, 0.0, 100.0),
        b_axis_position: Vector3::new(0.0, 0.0, 120.0),
        spindle_offset: Vector3::new(0.0, 0.0, 300.0),
        max_a_angle: 120.0,
        max_b_angle: 360.0,
        table_diameter: 400.0,
    };
    
    let kinematics = FiveAxisKinematics::new(config);
    
    // Test forward kinematics
    let joint_config = JointConfiguration::new(200.0, 150.0, 250.0, 45.0, 90.0);
    let tool_pose = kinematics.forward_kinematics(&joint_config)?;
    
    println!("   Joint Configuration: {:?}", joint_config.to_array());
    println!("   Tool Position: [{:.2}, {:.2}, {:.2}]", 
        tool_pose.position.x, tool_pose.position.y, tool_pose.position.z);
    
    let (roll, pitch, yaw) = tool_pose.euler_angles();
    println!("   Tool Orientation: Roll={:.1}°, Pitch={:.1}°, Yaw={:.1}°",
        roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees());
    
    // Test inverse kinematics
    let target_pose = Pose6DOF::new(300.0, 200.0, 350.0, 0.2, 0.3, 0.1);
    let solutions = kinematics.inverse_kinematics(&target_pose)?;
    
    println!("   Target Pose: [{:.2}, {:.2}, {:.2}]", 
        target_pose.position.x, target_pose.position.y, target_pose.position.z);
    println!("   Found {} inverse kinematic solutions", solutions.len());
    
    if !solutions.is_empty() {
        println!("   Best Solution: {:?}", solutions[0].to_array());
        
        // Verify solution accuracy
        let verification = kinematics.forward_kinematics(&solutions[0])?;
        let position_error = (verification.position - target_pose.position).norm();
        println!("   Position Error: {:.4} mm", position_error);
    }
    
    Ok(())
}

fn demo_trunnion_kinematics() -> Result<(), KinematicsError> {
    let config = FiveAxisConfiguration::Trunnion {
        trunnion_center: Vector3::new(0.0, 0.0, 150.0),
        c_axis_height: 80.0,
        spindle_offset: Vector3::new(0.0, 0.0, 250.0),
        max_a_angle: 210.0,
        max_c_angle: 360.0,
        table_size: (300.0, 200.0),
    };
    
    let kinematics = FiveAxisKinematics::new(config);
    
    // Test complex orientation
    let joint_config = JointConfiguration::new(100.0, 75.0, 200.0, 30.0, 180.0);
    let tool_pose = kinematics.forward_kinematics(&joint_config)?;
    
    println!("   Trunnion Joint Config: {:?}", joint_config.to_array());
    println!("   Resulting Tool Pose: [{:.2}, {:.2}, {:.2}]", 
        tool_pose.position.x, tool_pose.position.y, tool_pose.position.z);
    
    // Test workspace limits
    let workspace_test_pose = Pose6DOF::new(500.0, 400.0, 100.0, 0.0, 0.0, 0.0);
    match kinematics.inverse_kinematics(&workspace_test_pose) {
        Ok(_) => println!("   Workspace test pose: REACHABLE"),
        Err(KinematicsError::OutOfWorkspace) => println!("   Workspace test pose: OUT OF WORKSPACE"),
        Err(e) => println!("   Workspace test error: {:?}", e),
    }
    
    Ok(())
}

fn demo_tilting_head_kinematics() -> Result<(), KinematicsError> {
    let config = FiveAxisConfiguration::TiltingHead {
        head_pivot_point: Vector3::new(0.0, 0.0, 400.0),
        b_axis_offset: 100.0,
        c_axis_offset: 150.0,
        tool_length_offset: 200.0,
        max_b_angle: 120.0,
        max_c_angle: 360.0,
        head_clearance_radius: 80.0,
    };
    
    let kinematics = FiveAxisKinematics::new(config);
    
    // Test tilting head for complex orientations
    let orientations = [
        (0.0, 0.0),     // Vertical
        (30.0, 45.0),   // Tilted
        (60.0, 90.0),   // High tilt
        (45.0, 180.0),  // Rotated
    ];
    
    for (b_angle, c_angle) in &orientations {
        let joint_config = JointConfiguration::new(150.0, 100.0, 200.0, *b_angle, *c_angle);
        let tool_pose = kinematics.forward_kinematics(&joint_config)?;
        
        println!("   B={:.0}°, C={:.0}° → Tool Position: [{:.1}, {:.1}, {:.1}]", 
            b_angle, c_angle, tool_pose.position.x, tool_pose.position.y, tool_pose.position.z);
    }
    
    Ok(())
}

fn demo_motion_planning() -> Result<(), KinematicsError> {
    let config = FiveAxisConfiguration::RotaryTable {
        table_offset: Vector3::zeros(),
        a_axis_position: Vector3::zeros(),
        b_axis_position: Vector3::zeros(),
        spindle_offset: Vector3::new(0.0, 0.0, 200.0),
        max_a_angle: 90.0,
        max_b_angle: 180.0,
        table_diameter: 300.0,
    };
    
    let kinematics = FiveAxisKinematics::new(config);
    let motion_planner = FiveAxisMotionPlanner::new(kinematics);
    
    // Define complex path waypoints
    let waypoints = vec![
        Pose6DOF::new(100.0, 100.0, 200.0, 0.0, 0.0, 0.0),
        Pose6DOF::new(200.0, 150.0, 220.0, 0.2, 0.1, 0.3),
        Pose6DOF::new(300.0, 100.0, 250.0, 0.4, 0.2, 0.6),
        Pose6DOF::new(250.0, 50.0, 230.0, 0.1, 0.3, 0.2),
        Pose6DOF::new(150.0, 75.0, 210.0, 0.0, 0.0, 0.0),
    ];
    
    let feedrates = vec![3000.0, 2500.0, 2000.0, 2500.0, 3000.0];
    
    // Plan complex path with different acceleration algorithms
    let algorithms = [
        ("Trapezoidal", AccelerationAlgorithm::Trapezoidal),
        ("S-Curve", AccelerationAlgorithm::SCurve),
        ("Jerk-Limited", AccelerationAlgorithm::JerkLimited),
        ("Adaptive", AccelerationAlgorithm::Adaptive),
    ];
    
    for (name, algorithm) in &algorithms {
        let config = PlannerConfiguration {
            acceleration_algorithm: algorithm.clone(),
            lookahead_segments: 3,
            corner_rounding: true,
            adaptive_feedrate: true,
            collision_checking: true,
            singularity_avoidance: true,
        };
        
        let planner = motion_planner.clone().with_config(config);
        let segments = planner.plan_path(&waypoints, &feedrates, ToolOrientationMode::Interpolated)?;
        
        let total_time: Duration = segments.iter().map(|s| s.duration).sum();
        let total_distance: f64 = segments.iter()
            .map(|s| (s.end_pose.position - s.start_pose.position).norm())
            .sum();
        
        println!("   {} Algorithm:", name);
        println!("     Segments: {}, Total Time: {:.2}s, Distance: {:.1}mm", 
            segments.len(), total_time.as_secs_f64(), total_distance);
        
        if !segments.is_empty() {
            println!("     Max Velocity: {:.1} mm/s", segments[0].max_velocity[0]);
            println!("     Acceleration Distance: {:.1} mm", segments[0].acceleration_profile.acceleration_distance);
        }
    }
    
    Ok(())
}

fn demo_tool_orientation_control() -> Result<(), KinematicsError> {
    let config = FiveAxisConfiguration::RotaryTable {
        table_offset: Vector3::zeros(),
        a_axis_position: Vector3::zeros(),
        b_axis_position: Vector3::zeros(),
        spindle_offset: Vector3::new(0.0, 0.0, 200.0),
        max_a_angle: 90.0,
        max_b_angle: 180.0,
        table_diameter: 300.0,
    };
    
    let kinematics = FiveAxisKinematics::new(config);
    let mut controller = ToolOrientationController::new(kinematics);
    
    // Define different orientation strategies
    let strategies = vec![
        ("Constant", OrientationStrategy::Constant {
            orientation: UnitQuaternion::identity(),
        }),
        ("Surface Normal", OrientationStrategy::SurfaceNormal {
            lead_angle: 15.0,
            tilt_angle: 5.0,
            surface_normal_source: SurfaceSource::Analytical {
                surface_type: AnalyticalSurface::Plane {
                    point: Point3::new(0.0, 0.0, 200.0),
                    normal: Unit::new_normalize(Vector3::new(0.0, 0.1, 1.0)),
                },
            },
        }),
        ("Surface Finish", OrientationStrategy::SurfaceFinish {
            preferred_cutting_direction: Vector3::new(1.0, 0.0, 0.0),
            climb_milling: true,
            stepover_optimization: true,
        }),
        ("Collision Avoidance", OrientationStrategy::CollisionAvoidance {
            avoidance_angle: 20.0,
            priority_axis: "A".to_string(),
            max_deviation: 45.0,
        }),
        ("Accessibility", OrientationStrategy::Accessibility {
            prefer_positive_angles: true,
            minimize_joint_motion: true,
            avoid_singularities: true,
        }),
    ];
    
    let test_position = Point3::new(200.0, 150.0, 250.0);
    
    for (name, strategy) in strategies {
        controller.add_orientation_strategy(name.to_string(), strategy.clone());
        let orientation = controller.calculate_tool_orientation(&test_position, &strategy)?;
        
        let (roll, pitch, yaw) = orientation.euler_angles();
        println!("   {} Strategy:", name);
        println!("     Tool Orientation: Roll={:.1}°, Pitch={:.1}°, Yaw={:.1}°",
            roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees());
        
        // Generate tool path with this strategy
        let positions = vec![
            Point3::new(200.0, 150.0, 250.0),
            Point3::new(220.0, 160.0, 255.0),
            Point3::new(240.0, 170.0, 260.0),
        ];
        
        let path = controller.generate_tool_path(&positions, name)?;
        println!("     Generated path with {} waypoints", path.len());
    }
    
    Ok(())
}

fn demo_non_planar_printing() -> Result<(), KinematicsError> {
    println!("   Setting up non-planar printing scenario...");
    
    // Create surface for non-planar printing (curved dome)
    let mut surface_points = Vec::new();
    let mut surface_normals = Vec::new();
    
    // Generate dome surface
    for theta in 0..=36 {
        for phi in 0..=18 {
            let theta_rad = (theta as f64) * PI / 18.0;
            let phi_rad = (phi as f64) * PI / 18.0;
            
            let x = 100.0 * phi_rad.sin() * theta_rad.cos();
            let y = 100.0 * phi_rad.sin() * theta_rad.sin();
            let z = 200.0 + 50.0 * phi_rad.cos();
            
            surface_points.push(Point3::new(x, y, z));
            
            // Calculate surface normal
            let normal = Vector3::new(
                phi_rad.sin() * theta_rad.cos(),
                phi_rad.sin() * theta_rad.sin(),
                phi_rad.cos(),
            );
            surface_normals.push(Unit::new_normalize(normal));
        }
    }
    
    let surface = Surface {
        points: surface_points.clone(),
        normals: surface_normals,
        triangles: Vec::new(), // Simplified for demo
    };
    
    // Create non-planar path following the surface
    let non_planar_path = NonPlanarPath {
        waypoints: surface_points.iter()
            .step_by(10)
            .map(|p| Pose6DOF::new(p.x, p.y, p.z, 0.0, 0.0, 0.0))
            .collect(),
        surface: Some(surface),
        layer_height_variation: true,
        adaptive_slicing: true,
    };
    
    // Plan non-planar motion
    let config = FiveAxisConfiguration::TiltingHead {
        head_pivot_point: Vector3::new(0.0, 0.0, 400.0),
        b_axis_offset: 100.0,
        c_axis_offset: 150.0,
        tool_length_offset: 50.0,
        max_b_angle: 90.0,
        max_c_angle: 360.0,
        head_clearance_radius: 80.0,
    };
    
    let kinematics = FiveAxisKinematics::new(config);
    let motion_planner = FiveAxisMotionPlanner::new(kinematics);
    
    let segments = motion_planner.plan_non_planar_path(&non_planar_path, 1500.0)?;
    
    println!("   Non-planar printing path:");
    println!("     Surface points: {}", surface_points.len());
    println!("     Print segments: {}", segments.len());
    println!("     Adaptive layer heights: enabled");
    
    let total_print_time: Duration = segments.iter().map(|s| s.duration).sum();
    println!("     Estimated print time: {:.1} minutes", total_print_time.as_secs_f64() / 60.0);
    
    Ok(())
}

fn demo_five_axis_machining() -> Result<(), KinematicsError> {
    println!("   Setting up 5-axis machining scenario...");
    
    // Configure for machining with rotary table
    let config = FiveAxisConfiguration::RotaryTable {
        table_offset: Vector3::new(0.0, 0.0, 100.0),
        a_axis_position: Vector3::new(0.0, 0.0, 100.0),
        b_axis_position: Vector3::new(0.0, 0.0, 120.0),
        spindle_offset: Vector3::new(0.0, 0.0, 350.0),
        max_a_angle: 120.0,
        max_b_angle: 360.0,
        table_diameter: 300.0,
    };
    
    let kinematics = FiveAxisKinematics::new(config);
    let mut controller = ToolOrientationController::new(kinematics);
    
    // Set up machining tool
    let end_mill = ToolDefinition::carbide_end_mill(6.0, 75.0);
    controller.set_active_tool(end_mill);
    
    // Define workpiece coordinate system
    let workpiece_cs = WorkpieceCS {
        origin: Point3::new(0.0, 0.0, 150.0),
        rotation: UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
        name: "G54".to_string(),
        setup_description: "Part setup for 5-axis machining".to_string(),
    };
    controller.set_workpiece_coordinate_system(workpiece_cs);
    
    // Define machining operations
    let operations = vec![
        ("Face Milling", OrientationStrategy::SurfaceFinish {
            preferred_cutting_direction: Vector3::new(1.0, 0.0, 0.0),
            climb_milling: true,
            stepover_optimization: true,
        }),
        ("Profiling", OrientationStrategy::SurfaceNormal {
            lead_angle: 5.0,
            tilt_angle: 2.0,
            surface_normal_source: SurfaceSource::Analytical {
                surface_type: AnalyticalSurface::Cylinder {
                    axis_point: Point3::new(0.0, 0.0, 150.0),
                    axis_direction: Unit::new_normalize(Vector3::z()),
                    radius: 50.0,
                },
            },
        }),
        ("Finishing", OrientationStrategy::Accessibility {
            prefer_positive_angles: true,
            minimize_joint_motion: true,
            avoid_singularities: true,
        }),
    ];
    
    for (operation_name, strategy) in operations {
        println!("   {} Operation:", operation_name);
        
        // Generate toolpath for operation
        let toolpath_points = match operation_name {
            "Face Milling" => generate_face_milling_path(),
            "Profiling" => generate_profiling_path(),
            "Finishing" => generate_finishing_path(),
            _ => Vec::new(),
        };
        
        let path = controller.generate_tool_path(&toolpath_points, "operation")?;
        
        println!("     Toolpath points: {}", toolpath_points.len());
        println!("     Tool orientations calculated: {}", path.len());
        
        // Calculate machining time estimate
        let total_distance: f64 = path.windows(2)
            .map(|w| (w[1].position - w[0].position).norm())
            .sum();
        
        let feedrate = 800.0; // mm/min
        let machining_time = total_distance / feedrate * 60.0; // seconds
        
        println!("     Distance: {:.1} mm, Time: {:.1} seconds", total_distance, machining_time);
    }
    
    Ok(())
}

fn demo_gcode_processing() -> Result<(), KinematicsError> {
    println!("   Processing 5-axis G-code commands...");
    
    let config = FiveAxisConfiguration::RotaryTable {
        table_offset: Vector3::zeros(),
        a_axis_position: Vector3::zeros(),
        b_axis_position: Vector3::zeros(),
        spindle_offset: Vector3::new(0.0, 0.0, 200.0),
        max_a_angle: 90.0,
        max_b_angle: 180.0,
        table_diameter: 300.0,
    };
    
    let kinematics = FiveAxisKinematics::new(config);
    let controller = ToolOrientationController::new(kinematics);
    let mut processor = FiveAxisGCodeProcessor::new(controller);
    
    // Sample 5-axis G-code program
    let gcode_program = vec![
        "G54",                                    // Select coordinate system
        "G17",                                    // XY plane selection
        "G21",                                    // Metric units
        "G94",                                    // Feed per minute
        "T1 M6",                                 // Tool change
        "G43 H1",                                // Tool length compensation
        "S12000 M3",                             // Spindle on
        "G0 X100 Y100 Z200 A0 B0",               // Rapid positioning
        "G1 X150 Y120 Z190 A15 B30 F1200",       // Linear interpolation with rotation
        "G1 X200 Y150 Z185 A30 B60 F1000",       // Continue with orientation change
        "G1 X180 Y140 Z180 A45 B45 F800",        // Complex 5-axis motion
        "G0 Z250 A0 B0",                         // Retract and reset orientation
        "M30",                                   // Program end
    ];
    
    let mut total_segments = 0;
    let mut total_time = Duration::new(0, 0);
    
    for (line_num, gcode_line) in gcode_program.iter().enumerate() {
        match processor.process_gcode_line(gcode_line) {
            Ok(Some(segment)) => {
                total_segments += 1;
                total_time += segment.duration;
                
                println!("   Line {}: {} → Segment planned ({:.2}s)", 
                    line_num + 1, gcode_line, segment.duration.as_secs_f64());
                
                // Show joint motion for complex moves
                if gcode_line.contains("A") || gcode_line.contains("B") {
                    println!("     Start Joints: {:?}", segment.start_joints.to_array());
                    println!("     End Joints: {:?}", segment.end_joints.to_array());
                }
            },
            Ok(None) => {
                println!("   Line {}: {} → Modal command", line_num + 1, gcode_line);
            },
            Err(e) => {
                println!("   Line {}: {} → Error: {:?}", line_num + 1, gcode_line, e);
            },
        }
    }
    
    println!("   G-code Processing Summary:");
    println!("     Total lines: {}", gcode_program.len());
    println!("     Motion segments: {}", total_segments);
    println!("     Total motion time: {:.2} seconds", total_time.as_secs_f64());
    
    Ok(())
}

// Helper functions for generating machining toolpaths

fn generate_face_milling_path() -> Vec<Point3<f64>> {
    let mut points = Vec::new();
    
    // Generate zigzag pattern for face milling
    for y in (0..=100).step_by(10) {
        if y % 20 == 0 {
            // Left to right
            for x in (0..=100).step_by(5) {
                points.push(Point3::new(x as f64, y as f64, 200.0));
            }
        } else {
            // Right to left
            for x in (0..=100).step_by(5).rev() {
                points.push(Point3::new(x as f64, y as f64, 200.0));
            }
        }
    }
    
    points
}

fn generate_profiling_path() -> Vec<Point3<f64>> {
    let mut points = Vec::new();
    
    // Generate circular profile
    for angle in (0..=360).step_by(5) {
        let angle_rad = (angle as f64).to_radians();
        let radius = 50.0;
        
        let x = radius * angle_rad.cos();
        let y = radius * angle_rad.sin();
        let z = 200.0;
        
        points.push(Point3::new(x, y, z));
    }
    
    points
}

fn generate_finishing_path() -> Vec<Point3<f64>> {
    let mut points = Vec::new();
    
    // Generate spiral finishing path
    for i in 0..=200 {
        let t = i as f64 / 200.0;
        let angle = t * 4.0 * PI;
        let radius = 40.0 * (1.0 - t);
        
        let x = radius * angle.cos();
        let y = radius * angle.sin();
        let z = 200.0 + t * 20.0;
        
        points.push(Point3::new(x, y, z));
    }
    
    points
}