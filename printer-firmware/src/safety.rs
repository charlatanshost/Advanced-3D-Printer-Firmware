//! Machine-wide safety state, readable from any task or interrupt without locking.

use core::sync::atomic::{AtomicBool, Ordering};

static EMERGENCY_STOP: AtomicBool = AtomicBool::new(false);

/// Latch the emergency stop. It stays set until the board is reset, so that a stop can
/// never be undone by accident or over the network.
pub fn trigger_emergency_stop() {
    EMERGENCY_STOP.store(true, Ordering::SeqCst);
}

pub fn emergency_stop_active() -> bool {
    EMERGENCY_STOP.load(Ordering::SeqCst)
}
