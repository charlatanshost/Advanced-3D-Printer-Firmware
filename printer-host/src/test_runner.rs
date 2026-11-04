// Test Runner for Multi-Motor Integration Test

use crate::integration_test;

pub fn run_integration_test() {
    println!("Starting Multi-Motor 3D Printer Integration Test...");
    println!("====================================================");
    
    match integration_test::run_full_integration_test() {
        Ok(()) => {
            println!("\n🎉 SUCCESS: All integration tests passed!");
            println!("The multi-motor LinuxCNC-style control system is working properly.");
        }
        Err(e) => {
            println!("\n❌ FAILED: Integration test failed with error: {}", e);
        }
    }
}