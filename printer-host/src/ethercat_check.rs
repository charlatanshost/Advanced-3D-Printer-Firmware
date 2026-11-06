//! The hardware check behind the Fieldbus page: find what's on an adapter and watch its inputs,
//! on a background thread so the page never freezes waiting for the bus.
//!
//! It stays as careful as the command line: it holds the same lock, never goes to OP and never
//! writes process data. Each job leaves the coupler back in INIT.

use crate::ethercat_coupler::{self, InputsView};
use crate::ethercat_probe::{self, Adapter, DeviceInfo, RawLink};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

/// What's on the adapter.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Found {
    pub devices: Vec<DeviceInfo>,
    /// How many input bits each module has, in the order the coupler reports them
    pub module_inputs: Vec<usize>,
    /// Output bits other than the coupler's own control word. Watching refuses to run with any.
    pub other_output_bits: usize,
    pub can_watch: bool,
    /// Why modules couldn't be shown, when they couldn't
    pub note: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Activity {
    #[default]
    Idle,
    Finding,
    Watching,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct CheckState {
    pub activity: Activity,
    pub found: Option<Found>,
    /// The inputs right now. Only there during a watch: a stale reading must not look live.
    pub live: Option<InputsView>,
    /// How many times the inputs changed during this watch
    pub changes: u64,
    pub error: Option<String>,
}

type Opener = Arc<dyn Fn(&Adapter) -> Result<Box<dyn RawLink>, String> + Send + Sync>;

pub struct HardwareCheck {
    state: Arc<Mutex<CheckState>>,
    stop: Arc<AtomicBool>,
    open: Opener,
    lock_file: PathBuf,
    worker: Option<JoinHandle<()>>,
}

fn find_modules(link: Box<dyn RawLink>) -> Result<Found, String> {
    let (devices, link) = ethercat_probe::scan_keeping_link(link)?;
    if devices.is_empty() {
        return Err("No EtherCAT devices answered on this adapter. Check the cable goes to the coupler's IN port.".to_string());
    }
    if !only_a_bk1120(&devices) {
        return Ok(Found {
            devices,
            note: Some("Modules can only be listed when a BK1120 is the only device on the bus.".to_string()),
            ..Found::default()
        });
    }
    let (inputs, outputs, _) = match ethercat_coupler::read_process_data_layout(link) {
        Ok(layout) => layout,
        Err(e) => return Ok(Found { devices, note: Some(format!("The coupler answered, but its modules couldn't be read: {}", e)), ..Found::default() }),
    };
    let module_inputs: Vec<usize> = ethercat_coupler::read_inputs(&inputs, &[]).modules.iter().map(Vec::len).collect();
    let other_output_bits = ethercat_coupler::other_output_bits(&outputs);
    let note = module_inputs
        .is_empty()
        .then(|| "The coupler reports no modules. Check they're seated and the end terminal is fitted.".to_string());
    Ok(Found { devices, can_watch: !module_inputs.is_empty() && other_output_bits == 0, module_inputs, other_output_bits, note })
}

fn only_a_bk1120(devices: &[DeviceInfo]) -> bool {
    matches!(devices, [only] if only.is_bk1120())
}

impl HardwareCheck {
    pub fn new() -> Self {
        Self::with(Arc::new(|adapter| ethercat_probe::open_link(adapter)), ethercat_probe::bus_lock_file())
    }

    /// With a stand-in for the adapter and its own lock, for tests.
    pub fn with(open: Opener, lock_file: PathBuf) -> Self {
        Self { state: Default::default(), stop: Default::default(), open, lock_file, worker: None }
    }

    /// The current state. A job that died can't leave the page waiting for ever.
    pub fn state(&self) -> CheckState {
        let mut state = self.state.lock().unwrap();
        if state.activity != Activity::Idle && self.worker.as_ref().is_some_and(JoinHandle::is_finished) {
            state.activity = Activity::Idle;
            state.live = None;
            state.error.get_or_insert_with(|| "The EtherCAT check stopped unexpectedly.".to_string());
        }
        state.clone()
    }

    /// Show this state, for tests of how it looks.
    pub fn set_state(&mut self, state: CheckState) {
        *self.state.lock().unwrap() = state;
    }

    /// Find what's on this adapter. Ignored if the check is already busy.
    pub fn find(&mut self, adapter: &Adapter) {
        self.start(adapter, Activity::Finding, |link, state, _| {
            let found = find_modules(link)?;
            state.lock().unwrap().found = Some(found);
            Ok(())
        });
    }

    /// Watch the inputs until stopped. Ignored if the check is already busy.
    pub fn watch(&mut self, adapter: &Adapter) {
        self.start(adapter, Activity::Watching, |link, state, stop| {
            // Checked again: this could be a different device from the one found earlier
            let (devices, link) = ethercat_probe::scan_keeping_link(link)?;
            if !only_a_bk1120(&devices) {
                return Err("Inputs can only be watched when a BK1120 is the only device on the bus.".to_string());
            }
            ethercat_coupler::watch_while(
                link,
                |_| !stop.load(Ordering::Relaxed),
                |layout, inputs| {
                    let mut state = state.lock().unwrap();
                    state.changes += u64::from(state.live.is_some());
                    state.live = Some(ethercat_coupler::read_inputs(layout, inputs));
                },
            )
            .map(|_| ())
        });
    }

    /// Stop a watch. It finishes by putting the coupler back in INIT.
    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }

    fn start(
        &mut self,
        adapter: &Adapter,
        activity: Activity,
        job: impl FnOnce(Box<dyn RawLink>, &Mutex<CheckState>, &AtomicBool) -> Result<(), String> + Send + 'static,
    ) {
        if self.state().activity != Activity::Idle {
            return;
        }
        if let Some(previous) = self.worker.take() {
            let _ = previous.join();
        }
        {
            let mut state = self.state.lock().unwrap();
            state.activity = activity;
            state.live = None;
            state.changes = 0;
            state.error = None;
            if activity == Activity::Finding {
                state.found = None;
            }
        }
        self.stop.store(false, Ordering::Relaxed);
        let (state, stop, open, lock_file, adapter) = (self.state.clone(), self.stop.clone(), self.open.clone(), self.lock_file.clone(), adapter.clone());
        self.worker = Some(std::thread::spawn(move || {
            let result = ethercat_probe::claim_bus(&lock_file).and_then(|_bus| job(open(&adapter)?, &state, &stop));
            let mut state = state.lock().unwrap();
            state.activity = Activity::Idle;
            state.live = None;
            state.error = result.err();
        }));
    }
}

impl Default for HardwareCheck {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for HardwareCheck {
    /// Closing the app shouldn't leave the coupler running.
    fn drop(&mut self) {
        self.stop();
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ethercat_coupler::fake::{bk1120, real_layout, FakeCoupler};
    use std::time::{Duration, Instant};

    const STATE_INIT: u8 = 1;
    const STATE_PRE_OP: u8 = 2;
    const STATE_OP: u8 = 8;

    struct Scratch(PathBuf);
    impl Scratch {
        fn new(name: &str) -> Self {
            let dir = std::env::temp_dir().join(format!("ethercat_check_{}_{}", name, std::process::id()));
            std::fs::create_dir_all(&dir).unwrap();
            Self(dir)
        }
        fn lock(&self) -> PathBuf {
            self.0.join("bus.lock")
        }
    }
    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    fn adapter() -> Adapter {
        Adapter { name: "fake".to_string(), description: "Fake".to_string(), connected: true }
    }

    fn check_on(coupler: FakeCoupler, scratch: &Scratch) -> HardwareCheck {
        let coupler = Mutex::new(Some(coupler));
        HardwareCheck::with(
            Arc::new(move |_| coupler.lock().unwrap().take().map(|c| Box::new(c) as Box<dyn RawLink>).ok_or_else(|| "no second link".to_string())),
            scratch.lock(),
        )
    }

    fn wait_for(check: &HardwareCheck, what: impl Fn(&CheckState) -> bool) -> CheckState {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            let state = check.state();
            if what(&state) {
                return state;
            }
            assert!(Instant::now() < deadline, "timed out, at {:?}", state);
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    #[test]
    fn the_coupler_and_all_five_modules_are_found_and_it_is_left_in_init() {
        let scratch = Scratch::new("find");
        let coupler = real_layout();
        let seen = coupler.seen.clone();
        let mut check = check_on(coupler, &scratch);

        check.find(&adapter());
        let state = wait_for(&check, |s| s.activity == Activity::Idle);

        let found = state.found.unwrap();
        assert_eq!(found.devices[0].name, "BK1120");
        assert_eq!(found.module_inputs, vec![4, 4, 4, 4, 4]);
        assert!(found.can_watch);
        let seen = seen.lock().unwrap();
        assert_eq!(seen.states_requested.last(), Some(&STATE_INIT));
        assert_eq!(seen.logical_writes, 0);
    }

    #[test]
    fn a_live_input_change_shows_on_the_right_module_and_stop_leaves_it_in_init() {
        let scratch = Scratch::new("watch");
        let mut coupler = real_layout();
        coupler.input_byte = 2; // First module, after the status word
        coupler.input_sequence = [0b0000, 0b0000, 0b0001].into_iter().collect();
        let seen = coupler.seen.clone();
        let mut check = check_on(coupler, &scratch);

        check.watch(&adapter());
        let state = wait_for(&check, |s| s.live.as_ref().is_some_and(|live| live.modules[0][0]));

        assert_eq!(state.activity, Activity::Watching);
        assert_eq!(state.changes, 1, "the first reading isn't a change");
        assert_eq!(state.live.as_ref().unwrap().modules[0], vec![true, false, false, false]);
        assert!(!state.live.unwrap().modules[1][0], "and only that module");

        check.stop();
        let state = wait_for(&check, |s| s.activity == Activity::Idle);
        assert_eq!(state.live, None, "a stopped watch mustn't look live");
        assert_eq!(state.error, None);
        let seen = seen.lock().unwrap();
        assert_eq!(seen.states_requested.last(), Some(&STATE_INIT));
        assert!(!seen.states_requested.contains(&STATE_OP), "{:?}", seen.states_requested);
        assert_eq!(seen.logical_writes, 0);
    }

    #[test]
    fn a_coupler_dropping_out_mid_watch_shows_the_reason() {
        let scratch = Scratch::new("drop");
        let mut coupler = real_layout();
        coupler.fall_after_reads = Some(3);
        let mut check = check_on(coupler, &scratch);

        check.watch(&adapter());
        let state = wait_for(&check, |s| s.activity == Activity::Idle);

        let error = state.error.unwrap();
        assert!(error.contains("stopped answering"), "{}", error);
        assert!(error.contains("PRE-OP"), "{}", error);
        assert_eq!(state.live, None);
    }

    #[test]
    fn another_run_on_the_bus_blocks_the_check_and_says_so() {
        let scratch = Scratch::new("busy");
        let _other_run = ethercat_probe::claim_bus(&scratch.lock()).unwrap();
        let coupler = real_layout();
        let seen = coupler.seen.clone();
        let mut check = check_on(coupler, &scratch);

        check.find(&adapter());
        let state = wait_for(&check, |s| s.activity == Activity::Idle);

        assert!(state.error.unwrap().contains("already using EtherCAT"));
        assert!(seen.lock().unwrap().states_requested.is_empty(), "nothing was sent");
    }

    #[test]
    fn a_device_it_does_not_know_is_listed_but_not_started() {
        let scratch = Scratch::new("unknown");
        let mut coupler = bk1120(4, 0);
        coupler.other_product = true;
        let seen = coupler.seen.clone();
        let mut check = check_on(coupler, &scratch);

        check.find(&adapter());
        let found = wait_for(&check, |s| s.activity == Activity::Idle).found.unwrap();

        assert_eq!(found.devices.len(), 1);
        assert!(!found.can_watch);
        assert!(found.note.unwrap().contains("BK1120"));
        assert!(seen.lock().unwrap().states_requested.is_empty(), "an unknown device is never started");
    }

    #[test]
    fn a_coupler_that_cant_be_started_is_still_listed() {
        let scratch = Scratch::new("refuse");
        let mut coupler = real_layout();
        coupler.refuse_state = Some(STATE_PRE_OP);
        let mut check = check_on(coupler, &scratch);

        check.find(&adapter());
        let found = wait_for(&check, |s| s.activity == Activity::Idle).found.unwrap();

        assert_eq!(found.devices[0].name, "BK1120");
        assert!(!found.can_watch);
        assert!(found.note.unwrap().contains("refused PRE-OP"));
    }

    #[test]
    fn real_outputs_rule_out_watching() {
        let scratch = Scratch::new("outputs");
        let mut check = check_on(bk1120(4, 8), &scratch);

        check.find(&adapter());
        let found = wait_for(&check, |s| s.activity == Activity::Idle).found.unwrap();

        assert_eq!(found.other_output_bits, 8);
        assert!(!found.can_watch);
    }

    #[test]
    fn an_adapter_that_wont_open_is_reported() {
        let scratch = Scratch::new("open");
        let mut check = HardwareCheck::with(Arc::new(|_| Err("Npcap isn't installed".to_string())), scratch.lock());

        check.find(&adapter());
        let state = wait_for(&check, |s| s.activity == Activity::Idle);

        assert_eq!(state.error.as_deref(), Some("Npcap isn't installed"));
    }

    #[test]
    fn closing_during_a_watch_stops_it_and_leaves_the_coupler_in_init() {
        let scratch = Scratch::new("close");
        let coupler = real_layout();
        let seen = coupler.seen.clone();
        let mut check = check_on(coupler, &scratch);

        check.watch(&adapter());
        wait_for(&check, |s| s.live.is_some());
        drop(check);

        assert_eq!(seen.lock().unwrap().states_requested.last(), Some(&STATE_INIT));
    }
}
