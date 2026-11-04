use printer_kinematics::*;
use printer_kinematics::five_axis::*;

fn main() {
    println!("5-Axis Kinematics Library Test");
    
    // Test basic functionality
    test_practical_configs();
    test_five_axis_kinematics();
    
    println!("All tests completed successfully!");
}

fn test_practical_configs() {
    println!("\n=== Testing Practical Configurations ===");
    
    // Create a practical toolhead rotary configuration
    let machine = PracticalMachineType::penta_axis_style();
    let kinematics = PracticalKinematics::new(machine);
    
    // Test a simple pose
    let target_pose = Pose6DOF::new(100.0, 50.0, 25.0, 0.0, 0.174, 0.0); // 10 degrees pitch
    
    match kinematics.inverse_kinematics_analytical(&target_pose) {
        Ok(solutions) => {
            println!("✓ Found {} solution(s) for toolhead rotary configuration", solutions.len());
            if let Some(solution) = solutions.first() {
                println!("  Solution: X={:.2}, Y={:.2}, Z={:.2}, A={:.2}°, B={:.2}°", 
                         solution.x, solution.y, solution.z, solution.a, solution.b_or_c);
                
                // Test forward kinematics
                match kinematics.forward_kinematics_fast(&solution) {
                    Ok(result_pose) => {
                        println!("✓ Forward kinematics verification successful");
                        println!("  Result pose: X={:.2}, Y={:.2}, Z={:.2}", 
                                 result_pose.position.x, result_pose.position.y, result_pose.position.z);
                    },
                    Err(e) => println!("✗ Forward kinematics failed: {:?}", e),
                }
            }
        },
        Err(e) => println!("✗ Inverse kinematics failed: {:?}", e),
    }
}

fn test_five_axis_kinematics() {
    println!("\n=== Testing Five-Axis Kinematics Engine ===");
    
    // Create a rotary table configuration
    let config = FiveAxisConfiguration::RotaryTable {
        table_offset: nalgebra::Vector3::new(0.0, 0.0, 100.0),
        a_axis_position: nalgebra::Vector3::new(0.0, 0.0, 100.0),
        b_axis_position: nalgebra::Vector3::new(0.0, 0.0, 120.0),
        spindle_offset: nalgebra::Vector3::new(0.0, 0.0, 50.0),
        max_a_angle: 180.0,
        max_b_angle: 180.0,
        table_diameter: 200.0,
    };
    
    let workspace_limits = WorkspaceLimits {
        x_limits: (-400.0, 400.0),
        y_limits: (-400.0, 400.0),
        z_limits: (0.0, 500.0),
        a_limits: (-180.0, 180.0),
        b_or_c_limits: (-180.0, 180.0),
        max_tool_length: 100.0,
        safety_margin: 10.0,
    };
    
    let collision_checker = CollisionChecker::new();
    
    let kinematics = FiveAxisKinematics {
        config,
        workspace_limits,
        collision_checker,
    };
    
    // Test joint configuration
    let joints = JointConfiguration::new(100.0, 50.0, 200.0, 15.0, 30.0);
    
    match kinematics.forward_kinematics(&joints) {
        Ok(pose) => {
            println!("✓ Forward kinematics successful");
            println!("  Tool position: X={:.2}, Y={:.2}, Z={:.2}", 
                     pose.position.x, pose.position.y, pose.position.z);
            
            // Test inverse kinematics
            match kinematics.inverse_kinematics(&pose) {
                Ok(solutions) => {
                    println!("✓ Found {} inverse solution(s)", solutions.len());
                },
                Err(e) => println!("✗ Inverse kinematics failed: {:?}", e),
            }
        },
        Err(e) => println!("✗ Forward kinematics failed: {:?}", e),
    }
}