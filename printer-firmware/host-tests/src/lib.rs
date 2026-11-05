// The firmware's own source files, compiled for the PC.
#![allow(dead_code, unused_imports, unused_variables)]

#[path = "../../src/config_manager.rs"]
pub mod config_manager;
#[path = "../../src/exclude_object.rs"]
pub mod exclude_object;
#[path = "../../src/gcode_processor.rs"]
pub mod gcode_processor;
#[path = "../../src/power_recovery.rs"]
pub mod power_recovery;
#[path = "../../src/safety.rs"]
pub mod safety;
#[path = "../../src/web_server.rs"]
pub mod web_server;
