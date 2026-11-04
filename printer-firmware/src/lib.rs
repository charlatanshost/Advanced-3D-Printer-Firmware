#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use embedded_hal::pwm::SetDutyCycle;
use panic_halt as _;

mod power_recovery;
mod exclude_object;
mod gcode_processor;
mod config_manager;
mod web_server;

use power_recovery::{init_power_recovery, POWER_RECOVERY, RecoveryError};
use exclude_object::{init_exclude_object_manager, EXCLUDE_OBJECT_MANAGER, ObjectError, ObjectStatus};
use gcode_processor::{init_gcode_processor, GCODE_PROCESSOR, GcodeAction, GcodeError};
use config_manager::{init_config_manager, CONFIG_MANAGER, ConfigError, ConfigValue};
use web_server::{init_web_server, WEB_SERVER, WebServerError};

enum Command {
    Move { axis: char, position: f32 },
    SetServo { pin: u8, angle: u16 },
    ResumePrint,
    CheckPowerRecovery,
    EmergencyStop,
    ExcludeObject { name: [u8; 32] },
    IncludeObject { name: [u8; 32] },
    ListObjects,
    GetObjectStatus { name: [u8; 32] },
    ParseGcode { length: usize },
}

enum Status {
    Ok,
    Position { axis: char, pos: f32 },
    PowerRecoveryAvailable { timestamp: u64, progress: f32 },
    RecoveryError(RecoveryError),
    ObjectStatus { name: [u8; 32], status: ObjectStatus, progress: f32 },
    ObjectList { count: usize },
    ObjectError(ObjectError),
}

static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, Command, 10> = Channel::new();

#[cfg(feature = "rp2040")]
use rp2040_hal::{entry as _, pac, pwm::{Channel as PwmChannel, Slices}, gpio::{Pins, Pin, FunctionPwm}};

#[entry]
fn main() -> ! {
    #[cfg(feature = "rp2040")]
    {
        let mut p = pac::Peripherals::take().unwrap();
        let core = pac::CorePeripherals::take().unwrap();
        
        let sio = rp2040_hal::Sio::new(p.SIO);
        let mut pwm_slices = Slices::new(p.PWM, &mut p.RESETS);

        let mut pwm = pwm_slices.pwm0;
        pwm.set_ph_correct();
        pwm.enable();
        let mut channel: PwmChannel<_, _> = pwm.channel_a;
        let pins = Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);
        let pwm_pin: Pin<_, FunctionPwm, _> = pins.gpio0.into_function();

        // Initialize power recovery system would be done differently
        // For now, keep a simpler approach without the async tasks

        loop {
            if let Ok(cmd) = COMMAND_CHANNEL.try_receive() {
                match cmd {
                    Command::SetServo { pin: _, angle } => {
                        let duty = (angle as u32 * 1000 / 180 + 1000) as u16;
                        channel.set_duty_cycle(duty).ok();
                    }
                    Command::ResumePrint => {
                        // Handle resume print - simplified for now
                    }
                    Command::CheckPowerRecovery => {
                        // Check power recovery status - simplified for now  
                    }
                    Command::EmergencyStop => {
                        // Handle emergency stop - simplified for now
                    }
                    Command::ExcludeObject { name } => {
                        // Handle exclude object command
                    }
                    Command::IncludeObject { name } => {
                        // Handle include object command
                    }
                    Command::ListObjects => {
                        // Handle list objects command
                    }
                    Command::GetObjectStatus { name } => {
                        // Handle get object status command
                    }
                    Command::ParseGcode { length: _ } => {
                        // Handle parse G-code command
                    }
                    _ => {}
                }
            }
        }
    }

    #[cfg(not(feature = "rp2040"))]
    loop {}
}

// Async task implementations would go here when full embassy executor is set up
// For now, the power recovery system is available as a module that can be integrated