//! Starting a Beckhoff BK1120 coupler by hand, and watching its inputs.
//!
//! The BK1120's EEPROM has its identity and mailbox addresses but no configuration categories, so
//! ethercrab can't set up its mailboxes and the coupler refuses PRE-OP. Its start-up is standard
//! and short, though, so it's done directly here.
//!
//! It only ever reads. Inputs are read in SAFE-OP, where outputs stay in their safe state, and it
//! never goes to OP. The one output it accepts is the coupler's own control word (object 0xF200,
//! device control): with any other output present it stops in PRE-OP.

use crate::ethercat_probe::{RawLink, RegisterReader};
use std::time::{Duration, Instant};

pub const BECKHOFF_VENDOR_ID: u32 = 2;
pub const BK1120_PRODUCT_CODE: u32 = 0x0460_2C22;

const REG_AL_CONTROL: u16 = 0x0120;
const REG_AL_STATUS: u16 = 0x0130;
const REG_FMMU0: u16 = 0x0600;
const REG_SM0: u16 = 0x0800;
const REG_SM1_STATUS: u16 = 0x080D;

const STATE_INIT: u8 = 1;
const STATE_PRE_OP: u8 = 2;
const STATE_SAFE_OP: u8 = 4;
#[cfg(test)]
const STATE_OP: u8 = 8;
const ERROR_FLAG: u8 = 0x10;

// Where the coupler's EEPROM says its mailboxes are
const MAILBOX_LENGTH: u16 = 0x88;
const MAILBOX_OUT: u16 = 0x1C00;
const MAILBOX_IN: u16 = 0x1E00;
// Where Beckhoff couplers keep process data: the same addresses as its bootstrap mailboxes
const INPUTS_ADDRESS: u16 = 0x1800;
const OUTPUTS_ADDRESS: u16 = 0x1000;

const COE_MAILBOX_TYPE: u8 = 3;
const COE_SDO_REQUEST: u8 = 2;
const COE_SDO_RESPONSE: u8 = 3;
const OBJECT_INPUT_ASSIGN: u16 = 0x1C13;
const OBJECT_OUTPUT_ASSIGN: u16 = 0x1C12;
/// The coupler's own status and control words. Neither is a terminal's input or output.
const OBJECT_DEVICE_STATUS: u16 = 0xF100;
const OBJECT_DEVICE_CONTROL: u16 = 0xF200;

fn state_name(state: u8) -> &'static str {
    match state & 0x0F {
        STATE_INIT => "INIT",
        STATE_PRE_OP => "PRE-OP",
        3 => "BOOT",
        STATE_SAFE_OP => "SAFE-OP",
        8 => "OP",
        _ => "an unknown state",
    }
}

/// What a watch saw. Updates are the input bytes each time they changed, the first being how
/// they started.
#[derive(Debug, Clone, PartialEq)]
pub struct WatchResult {
    pub input_bits: usize,
    pub layout: Vec<DataEntry>,
    pub updates: Vec<Vec<u8>>,
}

struct Coupler {
    device: RegisterReader,
    mailbox_counter: u8,
}

impl Coupler {
    fn write_register(&mut self, register: u16, data: &[u8], what: &str) -> Result<(), String> {
        if self.device.write(register, data)? {
            Ok(())
        } else {
            Err(format!("The coupler didn't accept {}", what))
        }
    }

    fn status(&mut self) -> Result<u8, String> {
        Ok(self.device.read(REG_AL_STATUS, 2)?.ok_or("The coupler's state wasn't readable")?[0])
    }

    /// Ask for a state and wait for it. A refusal is reported instead of waiting for a timeout.
    fn go_to_state(&mut self, state: u8) -> Result<(), String> {
        self.write_register(REG_AL_CONTROL, &[state, 0], "a state change")?;
        let deadline = Instant::now() + Duration::from_secs(3);
        loop {
            let status = self.status()?;
            if status & ERROR_FLAG != 0 {
                let code = match self.device.read(0x0134, 2)? {
                    Some(code) => format!(" Status code {:#06x}.", u16::from_le_bytes([code[0], code[1]])),
                    None => String::new(),
                };
                return Err(format!("The coupler refused {} and stayed in {}.{}", state_name(state), state_name(status), code));
            }
            if status & 0x0F == state {
                return Ok(());
            }
            if Instant::now() > deadline {
                return Err(format!("The coupler didn't reach {}: still in {}", state_name(state), state_name(status)));
            }
        }
    }

    /// What state the coupler says it's in, in words. For explaining a failure, so it can't fail.
    fn describe_state(&mut self) -> String {
        let status = match self.status() {
            Ok(status) => status,
            Err(e) => return format!("Its state couldn't be read: {}", e),
        };
        if status & ERROR_FLAG == 0 {
            return format!("It reports {}, with no error.", state_name(status));
        }
        let code = match self.device.read(0x0134, 2) {
            Ok(Some(code)) => format!(", status code {:#06x}", u16::from_le_bytes([code[0], code[1]])),
            _ => ", and its status code wasn't readable".to_string(),
        };
        format!("It reports {} with an error{}.", state_name(status), code)
    }

    /// Back to a clean INIT, clearing any error left from before.
    fn reset(&mut self) -> Result<(), String> {
        self.write_register(REG_AL_CONTROL, &[STATE_INIT | ERROR_FLAG, 0], "a reset")?;
        let deadline = Instant::now() + Duration::from_secs(3);
        while Instant::now() < deadline {
            let status = self.status()?;
            if status & 0x0F == STATE_INIT && status & ERROR_FLAG == 0 {
                return Ok(());
            }
        }
        Err("The coupler wouldn't reset to INIT".to_string())
    }

    fn set_sync_manager(&mut self, index: u16, start: u16, length: u16, control: u8) -> Result<(), String> {
        let mut data = Vec::with_capacity(8);
        data.extend_from_slice(&start.to_le_bytes());
        data.extend_from_slice(&length.to_le_bytes());
        data.extend_from_slice(&[control, 0, 1, 0]);
        self.write_register(REG_SM0 + index * 8, &data, "a sync manager setting")
    }

    /// Read one value from the coupler's object dictionary. Returns its bytes.
    fn read_object(&mut self, index: u16, sub_index: u8) -> Result<Vec<u8>, String> {
        self.mailbox_counter = if self.mailbox_counter >= 7 { 1 } else { self.mailbox_counter + 1 };
        let mut request = vec![0u8; MAILBOX_LENGTH as usize];
        request[0] = 10; // Length of what follows the mailbox header
        request[5] = COE_MAILBOX_TYPE | (self.mailbox_counter << 4);
        request[7] = COE_SDO_REQUEST << 4;
        request[8] = 0x40; // Upload request
        request[9..11].copy_from_slice(&index.to_le_bytes());
        request[11] = sub_index;

        let what = format!("object {:#06x}:{}", index, sub_index);
        let deadline = Instant::now() + Duration::from_secs(1);
        // The write only counts once the coupler has room for it
        while !self.device.write(MAILBOX_OUT, &request)? {
            if Instant::now() > deadline {
                return Err(format!("The coupler's mailbox wouldn't accept a request for {}", what));
            }
        }
        loop {
            if Instant::now() > deadline {
                return Err(format!("No mailbox answer for {}", what));
            }
            let full = self.device.read(REG_SM1_STATUS, 1)?.map_or(false, |status| status[0] & 0x08 != 0);
            if !full {
                continue;
            }
            let reply = self.device.read(MAILBOX_IN, MAILBOX_LENGTH as usize)?.ok_or("The mailbox wasn't readable")?;
            // Anything but the answer, such as an emergency message, gets skipped
            if reply[5] & 0x0F != COE_MAILBOX_TYPE || reply[7] >> 4 != COE_SDO_RESPONSE {
                continue;
            }
            let command = reply[8];
            if command == 0x80 {
                let code = u32::from_le_bytes([reply[12], reply[13], reply[14], reply[15]]);
                return Err(format!("The coupler has no {} (abort code {:#010x})", what, code));
            }
            if command & 0x02 != 0 {
                // Short enough to fit in the reply itself
                let unused = if command & 0x01 != 0 { ((command >> 2) & 0x03) as usize } else { 0 };
                return Ok(reply[12..16 - unused].to_vec());
            }
            let size = u32::from_le_bytes([reply[12], reply[13], reply[14], reply[15]]) as usize;
            let end = (16 + size).min(reply.len());
            return Ok(reply[16..end].to_vec());
        }
    }

    fn read_u8(&mut self, index: u16, sub_index: u8) -> Result<u8, String> {
        self.read_object(index, sub_index)?.first().copied().ok_or_else(|| format!("Object {:#06x} was empty", index))
    }

    /// The items of process data assigned to this direction.
    fn assigned_entries(&mut self, assign_object: u16) -> Result<Vec<DataEntry>, String> {
        let mut entries = Vec::new();
        for pdo_slot in 1..=self.read_u8(assign_object, 0)? {
            let pdo = self.read_object(assign_object, pdo_slot)?;
            let pdo = u16::from_le_bytes([*pdo.first().unwrap_or(&0), *pdo.get(1).unwrap_or(&0)]);
            for entry_slot in 1..=self.read_u8(pdo, 0)? {
                let entry = self.read_object(pdo, entry_slot)?;
                let byte = |i: usize| *entry.get(i).unwrap_or(&0);
                entries.push(DataEntry {
                    pdo,
                    bits: byte(0),
                    sub_index: byte(1),
                    object: u16::from_le_bytes([byte(2), byte(3)]),
                });
            }
        }
        Ok(entries)
    }

    /// How many bits of process data the PDOs assigned to this direction carry.
    fn assigned_bits(&mut self, assign_object: u16) -> Result<usize, String> {
        Ok(self.assigned_entries(assign_object)?.iter().map(|entry| entry.bits as usize).sum())
    }

    fn map_inputs(&mut self, input_bytes: u16) -> Result<(), String> {
        let mut fmmu = Vec::with_capacity(16);
        fmmu.extend_from_slice(&0u32.to_le_bytes()); // Logical address 0
        fmmu.extend_from_slice(&input_bytes.to_le_bytes());
        fmmu.extend_from_slice(&[0, 7]); // Whole bytes
        fmmu.extend_from_slice(&INPUTS_ADDRESS.to_le_bytes());
        fmmu.extend_from_slice(&[0, 1, 1, 0, 0, 0]); // Physical bit 0, read, active
        self.write_register(REG_FMMU0, &fmmu, "the input mapping")
    }
}

/// Input bytes sorted out: the coupler's status word, and each module's bits.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct InputsView {
    pub status: Option<u32>,
    pub modules: Vec<Vec<bool>>,
}

/// Sort input bytes by the layout. Bytes that are missing read as off, so an empty slice gives the
/// shape of the modules.
pub fn read_inputs(layout: &[DataEntry], inputs: &[u8]) -> InputsView {
    let bit = |at: usize| inputs.get(at / 8).map_or(false, |byte| byte >> (at % 8) & 1 == 1);
    let mut view = InputsView::default();
    let mut current_module: Option<u16> = None;
    let mut at = 0usize;
    for entry in layout {
        let bits = entry.bits as usize;
        if entry.object == OBJECT_DEVICE_STATUS {
            view.status = Some((0..bits).fold(0u32, |word, i| word | (u32::from(bit(at + i)) << i)));
            current_module = None;
        } else if entry.object != 0 {
            if current_module != Some(entry.object) {
                current_module = Some(entry.object);
                view.modules.push(Vec::new());
            }
            if let Some(module) = view.modules.last_mut() {
                module.extend((0..bits).map(|i| bit(at + i)));
            }
        }
        at += bits;
    }
    view
}

/// Output bits other than the coupler's own control word.
pub fn other_output_bits(outputs: &[DataEntry]) -> usize {
    outputs.iter().filter(|entry| entry.object != OBJECT_DEVICE_CONTROL && entry.object != 0).map(|entry| entry.bits as usize).sum()
}

/// One update in words: the coupler's status word, then each module's bits in order.
pub fn describe_inputs(layout: &[DataEntry], inputs: &[u8]) -> String {
    let view = read_inputs(layout, inputs);
    let status = view.status.map(|word| format!("status {:#06x}", word));
    let modules = view.modules.iter().enumerate().map(|(index, bits)| {
        let text: String = bits.iter().map(|on| if *on { '1' } else { '0' }).collect();
        format!("module {}: {}", index + 1, text)
    });
    status.into_iter().chain(modules).collect::<Vec<_>>().join("  ")
}

/// Bring the coupler up and report its inputs as they change. `on_update` hears each change.
pub fn watch_inputs(link: Box<dyn RawLink>, how_long: Duration, on_update: impl FnMut(&[DataEntry], &[u8])) -> Result<WatchResult, String> {
    watch_while(link, |watched| watched < how_long, on_update)
}

/// watch_inputs for as long as `keep_going` says, given how long it has been watching.
pub fn watch_while(
    link: Box<dyn RawLink>,
    mut keep_going: impl FnMut(Duration) -> bool,
    mut on_update: impl FnMut(&[DataEntry], &[u8]),
) -> Result<WatchResult, String> {
    let mut coupler = Coupler { device: RegisterReader::new(link), mailbox_counter: 0 };

    coupler.reset()?;
    coupler.set_sync_manager(0, MAILBOX_OUT, MAILBOX_LENGTH, 0x26)?;
    coupler.set_sync_manager(1, MAILBOX_IN, MAILBOX_LENGTH, 0x22)?;
    coupler.go_to_state(STATE_PRE_OP)?;
    println!("  Reached PRE-OP");

    let outputs = coupler.assigned_entries(OBJECT_OUTPUT_ASSIGN)?;
    let layout = coupler.assigned_entries(OBJECT_INPUT_ASSIGN)?;
    let output_bits: usize = outputs.iter().map(|entry| entry.bits as usize).sum();
    let input_bits: usize = layout.iter().map(|entry| entry.bits as usize).sum();
    println!("  The coupler reports {} input bits and {} output bits", input_bits, output_bits);
    if let Some(real_output) = outputs.iter().find(|entry| entry.object != OBJECT_DEVICE_CONTROL) {
        let object = real_output.object;
        let _ = coupler.reset();
        return Err(format!("The coupler has outputs at object {:#06x}. This tool only reads, so it stopped before SAFE-OP.", object));
    }
    if !layout.iter().any(|entry| entry.object != 0 && entry.object != OBJECT_DEVICE_STATUS) {
        let _ = coupler.reset();
        return Err("The coupler reports no inputs. Check the terminals are seated and the end terminal is fitted.".to_string());
    }

    let input_bytes = input_bits.div_ceil(8) as u16;
    if output_bits > 0 {
        // Required for SAFE-OP. Nothing maps it or writes to it, and it has no effect in SAFE-OP.
        coupler.set_sync_manager(2, OUTPUTS_ADDRESS, output_bits.div_ceil(8) as u16, 0x24)?;
    }
    coupler.set_sync_manager(3, INPUTS_ADDRESS, input_bytes, 0x20)?;
    coupler.map_inputs(input_bytes)?;
    coupler.go_to_state(STATE_SAFE_OP)?;
    println!("  Reached SAFE-OP: inputs run, outputs stay in their safe state. Watching inputs.");

    let started = Instant::now();
    let mut updates: Vec<Vec<u8>> = Vec::new();
    let mut failure = None;
    let mut good_reads = 0u64;
    while keep_going(started.elapsed()) {
        match coupler.device.read_logical(0, input_bytes as usize) {
            Ok(Some(inputs)) => {
                good_reads += 1;
                if updates.last() != Some(&inputs) {
                    on_update(&layout, &inputs);
                    updates.push(inputs);
                }
            }
            Ok(None) => {
                // Ask now: the reset below wipes the evidence
                failure = Some(format!(
                    "The coupler stopped answering input reads after {:.2} s and {} good reads. {}",
                    started.elapsed().as_secs_f64(),
                    good_reads,
                    coupler.describe_state()
                ));
                break;
            }
            Err(e) => {
                failure = Some(e);
                break;
            }
        }
        std::thread::sleep(Duration::from_millis(5));
    }

    // Leave it as we found it, whatever happened
    let _ = coupler.reset();
    match failure {
        Some(e) => Err(e),
        None => Ok(WatchResult { input_bits, layout, updates }),
    }
}

/// One item of process data, as the coupler describes it.
#[derive(Debug, Clone, PartialEq)]
pub struct DataEntry {
    pub pdo: u16,
    pub object: u16,
    pub sub_index: u8,
    pub bits: u8,
}

/// What the coupler says it exchanges, and what it says about itself. This goes to PRE-OP to use
/// the mailbox and back to INIT. Process data isn't running in either, so nothing is driven.
pub fn read_process_data_layout(link: Box<dyn RawLink>) -> Result<(Vec<DataEntry>, Vec<DataEntry>, Vec<(String, String)>), String> {
    let mut coupler = Coupler { device: RegisterReader::new(link), mailbox_counter: 0 };
    coupler.reset()?;
    coupler.set_sync_manager(0, MAILBOX_OUT, MAILBOX_LENGTH, 0x26)?;
    coupler.set_sync_manager(1, MAILBOX_IN, MAILBOX_LENGTH, 0x22)?;
    coupler.go_to_state(STATE_PRE_OP)?;

    let result = (|| {
        let inputs = coupler.assigned_entries(OBJECT_INPUT_ASSIGN)?;
        let outputs = coupler.assigned_entries(OBJECT_OUTPUT_ASSIGN)?;
        let mut about = Vec::new();
        for (label, object) in [("Device name", 0x1008u16), ("Hardware version", 0x1009), ("Software version", 0x100A)] {
            if let Ok(value) = coupler.read_object(object, 0) {
                about.push((label.to_string(), String::from_utf8_lossy(&value).trim_end_matches('\0').to_string()));
            }
        }
        Ok((inputs, outputs, about))
    })();
    let _ = coupler.reset();
    result
}

/// read_process_data_layout for the command line.
pub fn run_layout(link: Box<dyn RawLink>) -> Result<(), String> {
    let (inputs, outputs, about) = read_process_data_layout(link)?;
    for (label, value) in about {
        println!("  {}: {}", label, value);
    }
    for (title, entries) in [("Inputs", &inputs), ("Outputs", &outputs)] {
        println!("  {}: {} bits", title, entries.iter().map(|e| e.bits as usize).sum::<usize>());
        for entry in entries {
            println!("    PDO {:#06x}: object {:#06x}:{}, {} bits", entry.pdo, entry.object, entry.sub_index, entry.bits);
        }
    }
    println!("Nothing was driven: the coupler is back in INIT.");
    Ok(())
}

/// watch_inputs for the command line.
pub fn run_watch(link: Box<dyn RawLink>, seconds: u64) -> Result<(), String> {
    let started = Instant::now();
    let result = watch_inputs(link, Duration::from_secs(seconds), |layout, inputs| {
        println!("  {:7.2} s  {}", started.elapsed().as_secs_f64(), describe_inputs(layout, inputs));
    })?;
    println!("Done: {} input bits, {} changes after the starting state.", result.input_bits, result.updates.len().saturating_sub(1));
    Ok(())
}

#[cfg(test)]
pub(crate) mod fake {
    //! A stand-in BK1120: registers, EEPROM, state machine, mailbox and process data, laid out as
    //! the real one reported through `--ethercat inspect`.
    use super::*;
    use std::collections::{HashMap, VecDeque};
    use std::sync::{Arc, Mutex};

    #[derive(Default)]
    pub struct Seen {
        pub states_requested: Vec<u8>,
        pub logical_writes: usize,
    }

    pub struct FakeCoupler {
        pub registers: Vec<u8>,
        pub state: u8,
        pub error: bool,
        pub objects: HashMap<(u16, u8), Vec<u8>>,
        pub input_sequence: VecDeque<u8>,
        pub input_now: u8,
        pub input_byte: usize,
        pub refuse_state: Option<u8>,
        /// After this many input reads, drop to PRE-OP with an error, as a faulting coupler does
        pub fall_after_reads: Option<usize>,
        /// Don't acknowledge this input read, but carry on
        pub miss_read: Option<usize>,
        pub status_code: Option<u16>,
        pub input_reads: usize,
        /// Report a product code that isn't a BK1120
        pub other_product: bool,
        pub seen: Arc<Mutex<Seen>>,
        replies: VecDeque<Vec<u8>>,
        mailbox_reply: Option<Vec<u8>>,
    }

    /// Four KL1104 input bits by default, and no outputs.
    pub fn bk1120(input_bits: u8, output_bits: u8) -> FakeCoupler {
        let mut registers = vec![0u8; 0x2000];
        registers[0x0000..0x0004].copy_from_slice(&[2, 0xFD, 0x0C, 0]);
        registers[0x0004..0x0008].copy_from_slice(&[2, 4, 4, 0]);
        let mut objects = HashMap::new();
        objects.insert((0x1C12, 0), vec![u8::from(output_bits > 0)]);
        objects.insert((0x1C13, 0), vec![u8::from(input_bits > 0)]);
        if output_bits > 0 {
            objects.insert((0x1C12, 1), vec![0x00, 0x16]);
            objects.insert((0x1600, 0), vec![1]);
            objects.insert((0x1600, 1), vec![output_bits, 1, 0x00, 0x70]);
        }
        if input_bits > 0 {
            objects.insert((0x1C13, 1), vec![0x00, 0x1A]);
            objects.insert((0x1A00, 0), vec![1]);
            objects.insert((0x1A00, 1), vec![input_bits, 1, 0x00, 0x60]);
        }
        FakeCoupler {
            registers,
            state: STATE_INIT,
            error: false,
            objects,
            input_sequence: VecDeque::new(),
            input_now: 0,
            input_byte: 0,
            refuse_state: None,
            fall_after_reads: None,
            miss_read: None,
            status_code: None,
            input_reads: 0,
            other_product: false,
            seen: Default::default(),
            replies: VecDeque::new(),
            mailbox_reply: None,
        }
    }

    /// As the real one: its status word and five KL1104s in, and only its control word out.
    pub fn real_layout() -> FakeCoupler {
        let mut coupler = bk1120(0, 0);
        let objects = &mut coupler.objects;
        objects.insert((0x1C13, 0), vec![6]);
        objects.insert((0x1C13, 1), vec![0xFF, 0x1A]);
        objects.insert((0x1AFF, 0), vec![1]);
        objects.insert((0x1AFF, 1), vec![16, 1, 0x00, 0xF1]);
        for module in 0..5u8 {
            objects.insert((0x1C13, 2 + module), vec![module, 0x1A]);
            objects.insert((0x1A00 + module as u16, 0), vec![4]);
            for channel in 1..=4u8 {
                objects.insert((0x1A00 + module as u16, channel), vec![1, channel, module * 0x10, 0x60]);
            }
        }
        objects.insert((0x1C13, 7), vec![0x01, 0x1B]);
        objects.insert((0x1B01, 0), vec![1]);
        objects.insert((0x1B01, 1), vec![12, 0, 0, 0]);
        objects.insert((0x1C13, 0), vec![7]);
        objects.insert((0x1C12, 0), vec![1]);
        objects.insert((0x1C12, 1), vec![0xFF, 0x16]);
        objects.insert((0x16FF, 0), vec![1]);
        objects.insert((0x16FF, 1), vec![16, 1, 0x00, 0xF2]);
        coupler
    }

    fn eeprom(word: u16) -> [u8; 4] {
        match word {
            0x08 => [2, 0, 0, 0],
            0x0A => [0x22, 0x2C, 0x60, 0x04],
            0x0C => [0, 0, 1, 0],
            0x18 => [0x00, 0x1C, 0x88, 0x00],
            0x1A => [0x00, 0x1E, 0x88, 0x00],
            0x1C => [0x0D, 0, 0, 0],
            w if w >= 0x40 => [0xFF; 4],
            _ => [0; 4],
        }
    }

    impl FakeCoupler {
        fn control_write(&mut self, value: u8) {
            let wanted = value & 0x0F;
            self.seen.lock().unwrap().states_requested.push(wanted);
            if value & ERROR_FLAG != 0 {
                self.error = false;
            }
            let sm_length = |registers: &[u8], index: usize| u16::from_le_bytes([registers[0x0802 + index * 8], registers[0x0803 + index * 8]]);
            let allowed = match wanted {
                STATE_INIT => true,
                STATE_PRE_OP => sm_length(&self.registers, 0) == MAILBOX_LENGTH && sm_length(&self.registers, 1) == MAILBOX_LENGTH,
                STATE_SAFE_OP => {
                    self.state == STATE_PRE_OP
                        && sm_length(&self.registers, 3) > 0
                        && u16::from_le_bytes([self.registers[0x0818], self.registers[0x0819]]) == INPUTS_ADDRESS
                        && self.registers[0x060C] == 1
                        // With a control word in the layout, it needs somewhere to go
                        && (!self.objects.contains_key(&(0x16FF, 0)) || sm_length(&self.registers, 2) > 0)
                }
                STATE_OP => self.state == STATE_SAFE_OP,
                _ => false,
            } && self.refuse_state != Some(wanted);
            if allowed {
                self.state = wanted;
            } else {
                self.error = true;
            }
        }

        fn mailbox_write(&mut self, request: &[u8]) {
            let index = u16::from_le_bytes([request[9], request[10]]);
            let sub_index = request[11];
            let mut reply = vec![0u8; MAILBOX_LENGTH as usize];
            reply[0] = 10;
            reply[5] = COE_MAILBOX_TYPE;
            reply[7] = COE_SDO_RESPONSE << 4;
            reply[9..11].copy_from_slice(&index.to_le_bytes());
            reply[11] = sub_index;
            match self.objects.get(&(index, sub_index)) {
                Some(value) => {
                    let unused = 4 - value.len().min(4) as u8;
                    reply[8] = 0x43 | (unused << 2);
                    reply[12..12 + value.len().min(4)].copy_from_slice(&value[..value.len().min(4)]);
                }
                None => {
                    reply[8] = 0x80;
                    reply[12..16].copy_from_slice(&0x0602_0000u32.to_le_bytes());
                }
            }
            self.mailbox_reply = Some(reply);
        }

        fn handle(&mut self, command: u8, register: u16, data: &mut [u8]) -> u16 {
            let start = register as usize;
            match command {
                7 => 1, // Broadcast read, used to count devices
                1 | 10 => {
                    if command == 10 {
                        if let Some(next) = self.input_sequence.pop_front() {
                            self.input_now = next;
                        }
                        self.input_reads += 1;
                        if self.fall_after_reads == Some(self.input_reads) {
                            self.state = STATE_PRE_OP;
                            self.error = true;
                        }
                        if self.state != STATE_OP && self.state != STATE_SAFE_OP || self.miss_read == Some(self.input_reads) {
                            return 0;
                        }
                        data.fill(0);
                        data[self.input_byte.min(data.len() - 1)] = self.input_now;
                        return 1;
                    }
                    match register {
                        // The real coupler has no such registers, and doesn't acknowledge them
                        0x0134 if self.status_code.is_some() => {
                            data[..2].copy_from_slice(&self.status_code.unwrap_or(0).to_le_bytes());
                            return 1;
                        }
                        0x0012 | 0x0134 => return 0,
                        0x0130 => {
                            data[0] = self.state | if self.error { ERROR_FLAG } else { 0 };
                            return 1;
                        }
                        0x080D => {
                            data[0] = if self.mailbox_reply.is_some() { 0x08 } else { 0 };
                            return 1;
                        }
                        MAILBOX_IN => match self.mailbox_reply.take() {
                            Some(reply) => {
                                data.copy_from_slice(&reply[..data.len()]);
                                return 1;
                            }
                            None => return 0,
                        },
                        _ => {}
                    }
                    if start + data.len() <= self.registers.len() {
                        let source = &self.registers[start..start + data.len()];
                        data.copy_from_slice(source);
                        1
                    } else {
                        0
                    }
                }
                2 => {
                    match register {
                        REG_AL_CONTROL => {
                            self.control_write(data[0]);
                            return 1;
                        }
                        0x0502 => {
                            let word = u16::from_le_bytes([data[2], data[3]]);
                            self.registers[0x0508..0x050C].copy_from_slice(&eeprom(word));
                            if self.other_product && word == 0x0A {
                                self.registers[0x0508] = 0x99;
                            }
                            self.registers[0x0502..0x0504].copy_from_slice(&[0, 0]);
                            return 1;
                        }
                        MAILBOX_OUT => {
                            self.mailbox_write(data);
                            return 1;
                        }
                        _ => {}
                    }
                    self.registers[start..start + data.len()].copy_from_slice(data);
                    1
                }
                11 | 12 => {
                    self.seen.lock().unwrap().logical_writes += 1;
                    0
                }
                _ => 0,
            }
        }
    }

    impl RawLink for FakeCoupler {
        fn send(&mut self, frame: &[u8]) -> Result<(), String> {
            let mut reply = frame.to_vec();
            reply[6] |= 0x02;
            let command = reply[16];
            let register = u16::from_le_bytes([reply[20], reply[21]]);
            let length = (u16::from_le_bytes([reply[22], reply[23]]) & 0x07FF) as usize;
            // Writes by position land on the device whose position counts down to zero
            let position = u16::from_le_bytes([reply[18], reply[19]]);
            let addressed = !matches!(command, 1 | 2) || position == 0;
            let mut data = reply[26..26 + length].to_vec();
            let counter = if addressed { self.handle(command, register, &mut data) } else { 0 };
            reply[26..26 + length].copy_from_slice(&data);
            reply[26 + length..28 + length].copy_from_slice(&counter.to_le_bytes());
            self.replies.push_back(reply);
            Ok(())
        }
        fn receive(&mut self) -> Result<Option<Vec<u8>>, String> {
            Ok(self.replies.pop_front())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::fake::*;
    use super::*;

    fn watch(coupler: FakeCoupler) -> Result<WatchResult, String> {
        watch_inputs(Box::new(coupler), Duration::from_millis(150), |_, _| {})
    }

    #[test]
    fn the_coupler_starts_and_its_inputs_are_seen_changing() {
        let mut coupler = bk1120(4, 0);
        coupler.input_sequence = [0b0000, 0b0000, 0b0001, 0b0001, 0b0101].into_iter().collect();
        let seen = coupler.seen.clone();

        let result = watch(coupler).unwrap();

        assert_eq!(result.input_bits, 4);
        assert_eq!(result.updates, vec![vec![0b0000], vec![0b0001], vec![0b0101]]);
        let states = seen.lock().unwrap().states_requested.clone();
        assert_eq!(states, vec![STATE_INIT, STATE_PRE_OP, STATE_SAFE_OP, STATE_INIT], "and it's left back in INIT");
    }

    #[test]
    fn the_real_layout_is_watched_in_safe_op_and_never_goes_to_op() {
        let mut coupler = real_layout();
        coupler.input_byte = 2; // First module, after the two status bytes
        coupler.input_sequence = [0b0000, 0b0100].into_iter().collect();
        let seen = coupler.seen.clone();

        let result = watch(coupler).unwrap();

        assert_eq!(result.input_bits, 48);
        assert_eq!(result.updates.last().unwrap()[2], 0b0100);
        let seen = seen.lock().unwrap();
        assert!(!seen.states_requested.contains(&STATE_OP), "{:?}", seen.states_requested);
        assert_eq!(seen.logical_writes, 0);
    }

    #[test]
    fn any_output_except_the_control_word_stops_it_in_pre_op() {
        let coupler = bk1120(4, 8); // Mapped to object 0x7000: a real output terminal
        let seen = coupler.seen.clone();

        let error = watch(coupler).unwrap_err();

        assert!(error.contains("0x7000"), "{}", error);
        let seen = seen.lock().unwrap();
        assert!(!seen.states_requested.contains(&STATE_SAFE_OP), "{:?}", seen.states_requested);
        assert!(!seen.states_requested.contains(&STATE_OP), "{:?}", seen.states_requested);
        assert_eq!(seen.logical_writes, 0);
    }

    #[test]
    fn a_control_word_alongside_a_real_output_still_stops_it() {
        let mut coupler = real_layout();
        coupler.objects.insert((0x1C12, 0), vec![2]);
        coupler.objects.insert((0x1C12, 2), vec![0x00, 0x16]);
        coupler.objects.insert((0x1600, 0), vec![1]);
        coupler.objects.insert((0x1600, 1), vec![4, 1, 0x00, 0x70]);
        let error = watch(coupler).unwrap_err();
        assert!(error.contains("0x7000"), "{}", error);
    }

    #[test]
    fn a_coupler_that_drops_out_is_asked_why_before_it_gets_reset() {
        let mut coupler = real_layout();
        coupler.fall_after_reads = Some(4);
        coupler.status_code = Some(0x001B);
        let seen = coupler.seen.clone();

        let error = watch(coupler).unwrap_err();

        assert!(error.contains("3 good reads"), "{}", error);
        assert!(error.contains("PRE-OP with an error"), "{}", error);
        assert!(error.contains("0x001b"), "{}", error);
        assert_eq!(seen.lock().unwrap().states_requested.last(), Some(&STATE_INIT), "and it's still left in INIT");
    }

    #[test]
    fn a_missed_read_with_the_coupler_still_running_says_so() {
        let mut coupler = real_layout();
        coupler.miss_read = Some(2);
        let error = watch(coupler).unwrap_err();
        assert!(error.contains("1 good reads"), "{}", error);
        assert!(error.contains("SAFE-OP, with no error"), "{}", error);
    }

    #[test]
    fn an_unreadable_status_code_doesnt_hide_the_state() {
        let mut coupler = real_layout();
        coupler.fall_after_reads = Some(1);
        let error = watch(coupler).unwrap_err();
        assert!(error.contains("PRE-OP with an error"), "{}", error);
        assert!(error.contains("status code wasn't readable"), "{}", error);
    }

    #[test]
    fn inputs_are_described_per_module() {
        let layout = [
            DataEntry { pdo: 0x1AFF, object: OBJECT_DEVICE_STATUS, sub_index: 1, bits: 16 },
            DataEntry { pdo: 0x1A00, object: 0x6000, sub_index: 1, bits: 1 },
            DataEntry { pdo: 0x1A00, object: 0x6000, sub_index: 2, bits: 1 },
            DataEntry { pdo: 0x1A01, object: 0x6010, sub_index: 1, bits: 1 },
            DataEntry { pdo: 0x1A01, object: 0x6010, sub_index: 2, bits: 1 },
            DataEntry { pdo: 0x1B01, object: 0, sub_index: 0, bits: 4 },
        ];
        assert_eq!(describe_inputs(&layout, &[0x34, 0x12, 0b0000_1001]), "status 0x1234  module 1: 10  module 2: 01");
    }

    #[test]
    fn nothing_is_ever_written_to_process_data() {
        let coupler = real_layout();
        let seen = coupler.seen.clone();
        watch(coupler).unwrap();
        assert_eq!(seen.lock().unwrap().logical_writes, 0);
    }

    #[test]
    fn no_inputs_gets_advice_about_the_terminals() {
        // The status word alone doesn't count as having inputs
        let mut coupler = real_layout();
        coupler.objects.insert((0x1C13, 0), vec![1]);
        let error = watch(coupler).unwrap_err();
        assert!(error.contains("no inputs"), "{}", error);
    }

    #[test]
    fn a_refused_state_is_named_and_the_coupler_left_in_init() {
        let mut coupler = bk1120(4, 0);
        coupler.refuse_state = Some(STATE_SAFE_OP);
        let error = watch(coupler).unwrap_err();
        assert!(error.contains("refused SAFE-OP"), "{}", error);
        assert!(error.contains("stayed in PRE-OP"), "{}", error);
    }

    #[test]
    fn an_error_left_by_an_earlier_attempt_is_cleared_first() {
        // As the real coupler was found after ethercrab's attempt: INIT, with an error flagged
        let mut coupler = bk1120(4, 0);
        coupler.error = true;
        assert!(watch(coupler).is_ok());
    }

    #[test]
    fn a_missing_object_is_reported_with_its_number() {
        let mut coupler = bk1120(4, 0);
        coupler.objects.remove(&(0x1C13, 0));
        let error = watch(coupler).unwrap_err();
        assert!(error.contains("0x1c13"), "{}", error);
    }

    #[test]
    fn the_layout_is_read_without_leaving_pre_op() {
        let coupler = bk1120(4, 8);
        let seen = coupler.seen.clone();

        let (inputs, outputs, _) = read_process_data_layout(Box::new(coupler)).unwrap();

        assert_eq!(inputs, vec![DataEntry { pdo: 0x1A00, object: 0x6000, sub_index: 1, bits: 4 }]);
        assert_eq!(outputs, vec![DataEntry { pdo: 0x1600, object: 0x7000, sub_index: 1, bits: 8 }]);
        let seen = seen.lock().unwrap();
        assert_eq!(seen.states_requested, vec![STATE_INIT, STATE_PRE_OP, STATE_INIT]);
        assert_eq!(seen.logical_writes, 0);
    }

    #[test]
    fn inputs_over_a_byte_are_sized_properly() {
        let mut coupler = bk1120(12, 0);
        coupler.input_sequence = [0xAB].into_iter().collect();
        let result = watch(coupler).unwrap();
        assert_eq!(result.input_bits, 12);
        assert_eq!(result.updates.last().unwrap().len(), 2);
    }
}
