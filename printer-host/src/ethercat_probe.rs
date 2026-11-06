//! A read-only EtherCAT check: find the devices on a network adapter and watch their inputs.
//!
//! This is real EtherCAT, using the ethercrab master over Npcap on Windows. It is deliberately
//! limited, because it is the first thing here to touch real hardware:
//! - it only uses the adapter you name, since these frames don't belong on an office network
//! - it never writes outputs, and won't go operational if a device has any
//! - scan and inspect don't change what a device is doing. Watch has to start the device up
//!
//! Npcap is loaded when needed instead of linked, so the app builds and runs without it.

use crate::ethercat_coupler::{self, BECKHOFF_VENDOR_ID, BK1120_PRODUCT_CODE};
use ethercrab::{MainDevice, MainDeviceConfig, PduRx, PduStorage, PduTx, Timeouts};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

const MAX_SUBDEVICES: usize = 16;
const MAX_FRAMES: usize = 16;
const MAX_PDU_DATA: usize = PduStorage::element_size(1100);
const PDI_LEN: usize = 256;
const ETHERCAT_ETHERTYPE: [u8; 2] = [0x88, 0xA4];

/// Nanoseconds since the EtherCAT epoch, 2000-01-01.
fn ethercat_now() -> u64 {
    const SECONDS_FROM_1970_TO_2000: u64 = 946_684_800;
    let since_1970 = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default();
    since_1970.as_nanos().saturating_sub(SECONDS_FROM_1970_TO_2000 as u128 * 1_000_000_000) as u64
}

/// A way to send and receive raw Ethernet frames. Npcap in real use, a stand-in in tests.
pub trait RawLink: Send {
    fn send(&mut self, frame: &[u8]) -> Result<(), String>;
    /// The next received frame, if one is waiting. Must not wait for one.
    fn receive(&mut self) -> Result<Option<Vec<u8>>, String>;
}

#[derive(Debug, Clone, PartialEq)]
pub struct Adapter {
    pub name: String,
    pub description: String,
    pub connected: bool,
}

/// Find the adapter the user asked for: its number from the list, or part of its name.
/// Guessing could put EtherCAT frames on the wrong network, so anything unclear is refused.
pub fn pick_adapter<'a>(adapters: &'a [Adapter], wanted: &str) -> Result<&'a Adapter, String> {
    if let Ok(number) = wanted.parse::<usize>() {
        return number
            .checked_sub(1)
            .and_then(|index| adapters.get(index))
            .ok_or_else(|| format!("There is no adapter number {}", number));
    }
    let wanted_lower = wanted.to_lowercase();
    if let Some(exact) = adapters.iter().find(|a| a.description.to_lowercase() == wanted_lower) {
        return Ok(exact);
    }
    let matches: Vec<&Adapter> = adapters
        .iter()
        .filter(|a| a.name.to_lowercase().contains(&wanted_lower) || a.description.to_lowercase().contains(&wanted_lower))
        .collect();
    match matches.as_slice() {
        [one] => Ok(one),
        [] => Err(format!("No adapter matches \"{}\"", wanted)),
        _ => Err(format!("\"{}\" matches {} adapters: use its number from the list", wanted, matches.len())),
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct DeviceInfo {
    pub position: u16,
    pub name: String,
    pub vendor_id: u32,
    pub product_id: u32,
    pub revision: u32,
    pub serial: u32,
}

/// A name for devices this tool knows. Others are shown by their numbers.
fn known_name(vendor_id: u32, product_id: u32) -> String {
    if vendor_id == BECKHOFF_VENDOR_ID && product_id == BK1120_PRODUCT_CODE {
        "BK1120".to_string()
    } else {
        "EtherCAT device".to_string()
    }
}

impl DeviceInfo {
    fn describe(&self) -> String {
        format!(
            "{} at position {}: vendor {:#x}{}, product {:#010x}, revision {:#x}, serial {}",
            self.name,
            self.position,
            self.vendor_id,
            if self.vendor_id == BECKHOFF_VENDOR_ID { " (Beckhoff)" } else { "" },
            self.product_id,
            self.revision,
            self.serial
        )
    }

    /// BK couplers run a K-bus of their own, whose terminals aren't EtherCAT devices.
    fn has_kbus(&self) -> bool {
        self.vendor_id == BECKHOFF_VENDOR_ID && self.name.to_uppercase().starts_with("BK")
    }
}

/// Input bytes as bits, lowest first in each byte: 0b0000_0101 reads "1010 0000".
pub fn format_bits(bytes: &[u8]) -> String {
    if bytes.is_empty() {
        return "(none)".to_string();
    }
    bytes
        .iter()
        .map(|byte| {
            let bit = |i: u8| if byte >> i & 1 == 1 { '1' } else { '0' };
            format!("{}{}{}{} {}{}{}{}", bit(0), bit(1), bit(2), bit(3), bit(4), bit(5), bit(6), bit(7))
        })
        .collect::<Vec<_>>()
        .join(" | ")
}

/// One frame in words, for --trace: who sent it, and each EtherCAT command in it.
pub fn describe_frame(frame: &[u8]) -> String {
    if frame.len() < 16 {
        return format!("short frame, {} bytes", frame.len());
    }
    let source = frame[6..12].iter().map(|b| format!("{:02x}", b)).collect::<Vec<_>>().join(":");
    let mut text = format!("from {} {} bytes", source, frame.len());
    if frame[12..14] != ETHERCAT_ETHERTYPE {
        text.push_str(&format!(", not EtherCAT (type {:02x}{:02x})", frame[12], frame[13]));
        return text;
    }
    let mut at = 16;
    // Each command: type, index, address 4 bytes, length and flags 2, interrupt 2, data, counter 2
    while let Some(header) = frame.get(at..at + 10) {
        let command = match header[0] {
            0 => "NOP".to_string(),
            1 => "APRD".to_string(),
            2 => "APWR".to_string(),
            3 => "APRW".to_string(),
            4 => "FPRD".to_string(),
            5 => "FPWR".to_string(),
            6 => "FPRW".to_string(),
            7 => "BRD".to_string(),
            8 => "BWR".to_string(),
            9 => "BRW".to_string(),
            10 => "LRD".to_string(),
            11 => "LWR".to_string(),
            12 => "LRW".to_string(),
            13 => "ARMW".to_string(),
            14 => "FRMW".to_string(),
            other => format!("command {}", other),
        };
        let position = u16::from_le_bytes([header[2], header[3]]);
        let register = u16::from_le_bytes([header[4], header[5]]);
        let flags = u16::from_le_bytes([header[6], header[7]]);
        let length = (flags & 0x07FF) as usize;
        let Some(data) = frame.get(at + 10..at + 10 + length) else {
            text.push_str(&format!(" | {} cut short", command));
            break;
        };
        let Some(counter) = frame.get(at + 10 + length..at + 12 + length) else {
            text.push_str(&format!(" | {} cut short", command));
            break;
        };
        let shown: Vec<String> = data.iter().take(8).map(|b| format!("{:02x}", b)).collect();
        text.push_str(&format!(
            " | {} device {:#06x} register {:#06x} data [{}{}] counter {}",
            command,
            position,
            register,
            shown.join(" "),
            if length > 8 { " ..." } else { "" },
            u16::from_le_bytes([counter[0], counter[1]])
        ));
        if flags & 0x8000 == 0 {
            break;
        }
        at += 12 + length;
    }
    text
}

/// Prints every frame as it passes, around another link.
pub struct TracingLink(pub Box<dyn RawLink>);
impl RawLink for TracingLink {
    fn send(&mut self, frame: &[u8]) -> Result<(), String> {
        println!("  sent {}", describe_frame(frame));
        self.0.send(frame)
    }
    fn receive(&mut self) -> Result<Option<Vec<u8>>, String> {
        let received = self.0.receive()?;
        if let Some(frame) = &received {
            println!("  got  {}", describe_frame(frame));
        }
        Ok(received)
    }
}

/// Move frames between the master and the link until told to stop.
fn run_link(mut link: Box<dyn RawLink>, mut tx: PduTx<'static>, mut rx: PduRx<'static>, stop: Arc<AtomicBool>, fault: Arc<std::sync::Mutex<Option<String>>>) {
    while !stop.load(Ordering::Relaxed) {
        while let Some(frame) = tx.next_sendable_frame() {
            let result = frame.send_blocking(|bytes| match link.send(bytes) {
                Ok(()) => Ok(bytes.len()),
                Err(e) => {
                    *fault.lock().unwrap() = Some(e);
                    Err(ethercrab::error::Error::Internal)
                }
            });
            if result.is_err() {
                return;
            }
        }
        loop {
            match link.receive() {
                Ok(Some(packet)) => {
                    // Other traffic on the adapter isn't ours to interpret
                    if packet.get(12..14) == Some(&ETHERCAT_ETHERTYPE[..]) {
                        let _ = rx.receive_frame(&packet);
                    }
                }
                Ok(None) => break,
                Err(e) => {
                    *fault.lock().unwrap() = Some(e);
                    return;
                }
            }
        }
        std::thread::yield_now();
    }
}

const NO_REPLY: &str = "No reply on this adapter. Check it's the right adapter, that the cable goes to the coupler's IN port, and that the coupler has power.";

/// Talks to the first device with plain register reads, without the master. This shows what a
/// device itself reports, which is what's needed when the master can't get it going.
pub struct RegisterReader {
    link: Box<dyn RawLink>,
    index: u8,
    /// Which device on the bus, counting from 0 at the master
    position: u16,
}

const APRD: u8 = 1;
const APWR: u8 = 2;
const BRD: u8 = 7;
const LRD: u8 = 10;

impl RegisterReader {
    pub fn new(link: Box<dyn RawLink>) -> Self {
        Self { link, index: 0, position: 0 }
    }

    pub fn into_link(self) -> Box<dyn RawLink> {
        self.link
    }

    /// How many devices are on the bus: each adds one to the counter as the frame passes.
    pub fn count_devices(&mut self) -> Result<u16, String> {
        Ok(self.exchange(BRD, 0, &[0])?.1)
    }

    /// True if the device acknowledged the write.
    pub fn write(&mut self, register: u16, data: &[u8]) -> Result<bool, String> {
        Ok(self.exchange(APWR, register, data)?.1 > 0)
    }

    /// Mapped process data, or None if nothing acknowledged the read.
    pub fn read_logical(&mut self, address: u16, length: usize) -> Result<Option<Vec<u8>>, String> {
        let saved = std::mem::replace(&mut self.position, 0);
        let result = self.exchange(LRD, address, &vec![0; length]);
        self.position = saved;
        let (data, counter) = result?;
        Ok((counter > 0).then_some(data))
    }

    /// The identity from a device's EEPROM.
    pub fn identity(&mut self, position: u16) -> Result<DeviceInfo, String> {
        self.position = position;
        let result = (|| {
            let vendor_id = u32::from_le_bytes(self.read_eeprom(0x08)?);
            let product_id = u32::from_le_bytes(self.read_eeprom(0x0A)?);
            let revision = u32::from_le_bytes(self.read_eeprom(0x0C)?);
            let serial = u32::from_le_bytes(self.read_eeprom(0x0E)?);
            Ok(DeviceInfo { position, name: known_name(vendor_id, product_id), vendor_id, product_id, revision, serial })
        })();
        self.position = 0;
        result
    }

    fn exchange(&mut self, command: u8, register: u16, payload: &[u8]) -> Result<(Vec<u8>, u16), String> {
        self.index = self.index.wrapping_add(1);
        let mut frame = vec![0xFF; 6];
        frame.extend_from_slice(&[0x10; 6]);
        frame.extend_from_slice(&ETHERCAT_ETHERTYPE);
        let pdu_length = (10 + payload.len() + 2) as u16;
        frame.extend_from_slice(&(pdu_length | 0x1000).to_le_bytes());
        // By position, the address counts down: the device that sees zero is the one addressed
        frame.extend_from_slice(&[command, self.index]);
        frame.extend_from_slice(&0u16.wrapping_sub(self.position).to_le_bytes());
        frame.extend_from_slice(&register.to_le_bytes());
        frame.extend_from_slice(&(payload.len() as u16).to_le_bytes());
        frame.extend_from_slice(&[0, 0]);
        frame.extend_from_slice(payload);
        frame.extend_from_slice(&[0, 0]);
        self.link.send(&frame)?;

        let deadline = Instant::now() + Duration::from_millis(200);
        while Instant::now() < deadline {
            let Some(reply) = self.link.receive()? else {
                std::thread::yield_now();
                continue;
            };
            let ours = reply.get(12..14) == Some(&ETHERCAT_ETHERTYPE[..])
                && reply.get(6).map_or(false, |first| first & 0x02 != 0)
                && reply.get(17) == Some(&self.index);
            if !ours {
                continue;
            }
            let data = reply.get(26..26 + payload.len()).ok_or("reply cut short")?.to_vec();
            let counter = reply.get(26 + payload.len()..28 + payload.len()).ok_or("reply cut short")?;
            return Ok((data, u16::from_le_bytes([counter[0], counter[1]])));
        }
        Err(NO_REPLY.to_string())
    }

    /// A register's bytes, or None if the device didn't acknowledge the read.
    pub fn read(&mut self, register: u16, length: usize) -> Result<Option<Vec<u8>>, String> {
        let (data, counter) = self.exchange(APRD, register, &vec![0; length])?;
        Ok((counter > 0).then_some(data))
    }

    /// Two words from the EEPROM. Writing the address is how an EEPROM read is asked for.
    pub fn read_eeprom(&mut self, word: u16) -> Result<[u8; 4], String> {
        let mut request = vec![0x00, 0x01];
        request.extend_from_slice(&word.to_le_bytes());
        request.extend_from_slice(&[0, 0]);
        let (_, counter) = self.exchange(APWR, 0x0502, &request)?;
        if counter == 0 {
            return Err(format!("EEPROM read of word {:#06x} wasn't accepted", word));
        }
        let deadline = Instant::now() + Duration::from_millis(100);
        loop {
            let status = self.read(0x0502, 2)?.ok_or("EEPROM status wasn't readable")?;
            if status[1] & 0x80 == 0 {
                break;
            }
            if Instant::now() > deadline {
                return Err("EEPROM stayed busy".to_string());
            }
        }
        let data = self.read(0x0508, 4)?.ok_or("EEPROM data wasn't readable")?;
        Ok([data[0], data[1], data[2], data[3]])
    }
}

/// What a device's status code means, for the common ones during start-up.
pub fn al_status_meaning(code: u16) -> &'static str {
    match code {
        0x0000 => "no error",
        0x0001 => "unspecified error",
        0x0011 => "invalid requested state change",
        0x0012 => "unknown requested state",
        0x0013 => "bootstrap not supported",
        0x0014 => "no valid firmware",
        0x0015 => "invalid mailbox configuration (bootstrap)",
        0x0016 => "invalid mailbox configuration (PRE-OP)",
        0x0017 => "invalid sync manager configuration",
        0x0018 => "no valid inputs available",
        0x0019 => "no valid outputs",
        0x001A => "synchronization error",
        0x001B => "sync manager watchdog",
        0x001D => "invalid output configuration",
        0x001E => "invalid input configuration",
        0x001F => "invalid watchdog configuration",
        0x0020 => "device needs a cold start",
        0x0021 => "device needs to be in INIT",
        0x0022 => "device needs to be in PRE-OP",
        0x0023 => "device needs to be in SAFE-OP",
        _ => "not a code this tool knows",
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{:02x}", b)).collect::<Vec<_>>().join(" ")
}

/// Print what the first device reports about itself. Changes nothing it does.
pub fn inspect(link: Box<dyn RawLink>) -> Result<(), String> {
    let mut device = RegisterReader::new(link);
    let shown = |value: Option<Vec<u8>>| value.map_or("not acknowledged".to_string(), |bytes| hex(&bytes));

    println!("Device type, revision, build: {}", shown(device.read(0x0000, 4)?));
    if let Some(sizes) = device.read(0x0004, 4)? {
        println!("FMMUs: {}, sync managers: {}, RAM: {} KB", sizes[0], sizes[1], sizes[2]);
    }
    println!("Station address: {}", shown(device.read(0x0010, 2)?));
    println!("Link status: {}", shown(device.read(0x0110, 2)?));
    println!("Requested state: {}", shown(device.read(0x0120, 2)?));
    if let Some(status) = device.read(0x0130, 2)? {
        let state = match status[0] & 0x0F {
            1 => "INIT",
            2 => "PRE-OP",
            3 => "BOOT",
            4 => "SAFE-OP",
            8 => "OP",
            _ => "unknown",
        };
        println!("State: {}{}", state, if status[0] & 0x10 != 0 { ", with an error flagged" } else { "" });
    }
    if let Some(code) = device.read(0x0134, 2)? {
        let code = u16::from_le_bytes([code[0], code[1]]);
        println!("Status code: {:#06x}: {}", code, al_status_meaning(code));
    }
    println!("Sync managers (start, length, control, status, activate, PDI):");
    for manager in 0..4u16 {
        println!("  SM{}: {}", manager, shown(device.read(0x0800 + manager * 8, 8)?));
    }
    println!("EEPROM:");
    for word in (0..0x48u16).step_by(2) {
        let data = device.read_eeprom(word)?;
        println!("  {:#06x}: {}", word, hex(&data));
    }
    Ok(())
}

struct Session {
    main_device: &'static MainDevice<'static>,
    stop: Arc<AtomicBool>,
    fault: Arc<std::sync::Mutex<Option<String>>>,
}

impl Session {
    fn start(link: Box<dyn RawLink>, timeouts: Timeouts) -> Result<Self, String> {
        // One run per process in normal use. Kept for the life of the program, as ethercrab needs.
        let storage: &'static PduStorage<MAX_FRAMES, MAX_PDU_DATA> = Box::leak(Box::new(PduStorage::new()));
        let (tx, rx, pdu_loop) = storage.try_split().map_err(|e| format!("{:?}", e))?;
        let main_device: &'static MainDevice<'static> =
            Box::leak(Box::new(MainDevice::new(pdu_loop, timeouts, MainDeviceConfig::default())));

        let stop = Arc::new(AtomicBool::new(false));
        let fault = Arc::new(std::sync::Mutex::new(None));
        let (thread_stop, thread_fault) = (stop.clone(), fault.clone());
        std::thread::Builder::new()
            .name("ethercat-link".to_string())
            .spawn(move || run_link(link, tx, rx, thread_stop, thread_fault))
            .map_err(|e| e.to_string())?;
        Ok(Self { main_device, stop, fault })
    }

    /// Why a step failed, in terms someone at the bench can act on.
    fn explain(&self, step: &str, error: ethercrab::error::Error) -> String {
        if let Some(fault) = self.fault.lock().unwrap().take() {
            return format!("{}: the network adapter failed: {}", step, fault);
        }
        match error {
            ethercrab::error::Error::Timeout => format!(
                "{}: no reply on this adapter. Check it's the right adapter, that the cable goes to the coupler's IN port, and that the coupler has power.",
                step
            ),
            other => format!("{}: {}", step, other),
        }
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

fn default_timeouts() -> Timeouts {
    Timeouts { wait_loop_delay: Duration::ZERO, eeprom: Duration::from_millis(50), ..Timeouts::default() }
}

/// Find the devices on the link. This only reads: it doesn't reset them or change their state.
pub fn scan(link: Box<dyn RawLink>) -> Result<Vec<DeviceInfo>, String> {
    let mut reader = RegisterReader::new(link);
    let count = reader.count_devices()?;
    (0..count).map(|position| reader.identity(position)).collect()
}

/// Watch inputs, using whichever start-up the device on the bus needs.
pub async fn watch(link: Box<dyn RawLink>, seconds: u64) -> Result<(), String> {
    let mut reader = RegisterReader::new(link);
    let count = reader.count_devices()?;
    if count == 0 {
        return Err("No EtherCAT devices answered on this adapter.".to_string());
    }
    let first = reader.identity(0)?;
    let is_bk1120 = first.vendor_id == BECKHOFF_VENDOR_ID && first.product_id == BK1120_PRODUCT_CODE;
    let link = reader.into_link();
    if is_bk1120 {
        if count > 1 {
            return Err(format!("Found {} devices. Watching a BK1120 only works when it's the only one on the bus so far.", count));
        }
        return ethercat_coupler::run_watch(link, seconds);
    }
    watch_inputs(link, default_timeouts(), seconds).await
}

/// Bring the devices up and print their inputs whenever they change.
/// Stops before going operational if any device has outputs, since this tool never drives them.
pub async fn watch_inputs(link: Box<dyn RawLink>, timeouts: Timeouts, seconds: u64) -> Result<(), String> {
    let session = Session::start(link, timeouts)?;
    let md = session.main_device;
    let group = md
        .init_single_group::<MAX_SUBDEVICES, PDI_LEN>(ethercat_now)
        .await
        .map_err(|e| session.explain("Looking for devices", e))?;
    if group.len() == 0 {
        return Err("No EtherCAT devices answered on this adapter.".to_string());
    }

    let group = group.into_safe_op(md).await.map_err(|e| session.explain("Going to SAFE-OP", e))?;
    let mut has_outputs = false;
    for device in group.iter(md) {
        let io = device.io_raw();
        println!("  {}: {} input bytes, {} output bytes", device.name(), io.inputs().len(), io.outputs().len());
        has_outputs |= !io.outputs().is_empty();
    }
    if has_outputs {
        return Err("A device has outputs. This tool only reads, so it stopped before going operational.".to_string());
    }

    let mut group = group.into_op(md).await.map_err(|e| session.explain("Going to OP", e))?;
    println!("Operational. Watching inputs for {} seconds: change one to see it here.", seconds);

    let started = Instant::now();
    let mut last: Vec<Vec<u8>> = Vec::new();
    let mut changes = 0usize;
    while started.elapsed() < Duration::from_secs(seconds) {
        group.tx_rx(md).await.map_err(|e| session.explain("Exchanging data", e))?;
        let now: Vec<Vec<u8>> = group.iter(md).map(|device| device.io_raw().inputs().to_vec()).collect();
        if now != last {
            for (device, bytes) in group.iter(md).zip(&now) {
                println!("  {:7.2} s  {}: {}", started.elapsed().as_secs_f64(), device.name(), format_bits(bytes));
            }
            changes += 1;
            last = now;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    println!("Done: {} updates seen, the first being the starting state.", changes);

    // Hand the devices back in a safe state. If this fails their own watchdogs take over.
    if let Ok(group) = group.into_safe_op(md).await {
        let _ = group.into_pre_op(md).await;
    }
    Ok(())
}

#[cfg(windows)]
mod npcap {
    use super::{Adapter, RawLink};
    use libloading::os::windows::{Library, Symbol, LOAD_WITH_ALTERED_SEARCH_PATH};
    use std::ffi::{c_char, c_int, c_uchar, c_uint, c_void, CStr, CString};
    use std::sync::Arc;

    const ERRBUF_SIZE: usize = 256;
    const IF_LOOPBACK: c_uint = 0x1;
    const IF_CONNECTION_STATUS: c_uint = 0x30;
    const IF_DISCONNECTED: c_uint = 0x20;

    #[repr(C)]
    struct PcapIf {
        next: *mut PcapIf,
        name: *mut c_char,
        description: *mut c_char,
        addresses: *mut c_void,
        flags: c_uint,
    }

    #[repr(C)]
    struct PacketHeader {
        seconds: i32,
        microseconds: i32,
        captured_length: c_uint,
        length: c_uint,
    }

    pub const NOT_INSTALLED: &str = "Npcap isn't installed. Windows needs it to send EtherCAT frames. Get the installer from https://npcap.com, then try again.";

    fn load() -> Result<Arc<Library>, String> {
        let system = std::env::var("SystemRoot").unwrap_or_else(|_| r"C:\Windows".to_string());
        let path = format!(r"{}\System32\Npcap\wpcap.dll", system);
        // Altered search path, so it finds Npcap's own Packet.dll next to it
        unsafe { Library::load_with_flags(&path, LOAD_WITH_ALTERED_SEARCH_PATH) }
            .map(Arc::new)
            .map_err(|_| NOT_INSTALLED.to_string())
    }

    fn symbol<'a, T>(library: &'a Library, name: &[u8]) -> Result<Symbol<T>, String> {
        unsafe { library.get(name) }.map_err(|e| format!("Npcap is missing {}: {}", String::from_utf8_lossy(name), e))
    }

    fn text(pointer: *const c_char) -> String {
        if pointer.is_null() {
            String::new()
        } else {
            unsafe { CStr::from_ptr(pointer) }.to_string_lossy().into_owned()
        }
    }

    pub fn adapters() -> Result<Vec<Adapter>, String> {
        let library = load()?;
        let find: Symbol<unsafe extern "C" fn(*mut *mut PcapIf, *mut c_char) -> c_int> = symbol(&library, b"pcap_findalldevs\0")?;
        let free: Symbol<unsafe extern "C" fn(*mut PcapIf)> = symbol(&library, b"pcap_freealldevs\0")?;
        let mut errors = [0 as c_char; ERRBUF_SIZE];
        let mut first: *mut PcapIf = std::ptr::null_mut();
        if unsafe { find(&mut first, errors.as_mut_ptr()) } != 0 {
            return Err(format!("Couldn't list adapters: {}", text(errors.as_ptr())));
        }
        let mut found = Vec::new();
        let mut current = first;
        while !current.is_null() {
            let device = unsafe { &*current };
            if device.flags & IF_LOOPBACK == 0 {
                found.push(Adapter {
                    name: text(device.name),
                    description: text(device.description),
                    connected: device.flags & IF_CONNECTION_STATUS != IF_DISCONNECTED,
                });
            }
            current = device.next;
        }
        if !first.is_null() {
            unsafe { free(first) };
        }
        Ok(found)
    }

    pub struct NpcapLink {
        // Declared first so the capture closes before the library unloads
        handle: *mut c_void,
        send: unsafe extern "C" fn(*mut c_void, *const c_uchar, c_int) -> c_int,
        next: unsafe extern "C" fn(*mut c_void, *mut *mut PacketHeader, *mut *const c_uchar) -> c_int,
        geterr: unsafe extern "C" fn(*mut c_void) -> *const c_char,
        close: unsafe extern "C" fn(*mut c_void),
        _library: Arc<Library>,
    }

    // The handle is only ever used from the one thread that owns the link.
    unsafe impl Send for NpcapLink {}

    impl NpcapLink {
        pub fn open(device: &str) -> Result<Self, String> {
            let library = load()?;
            let name = CString::new(device).map_err(|e| e.to_string())?;
            let create: Symbol<unsafe extern "C" fn(*const c_char, *mut c_char) -> *mut c_void> = symbol(&library, b"pcap_create\0")?;
            let set_int: Symbol<unsafe extern "C" fn(*mut c_void, c_int) -> c_int> = symbol(&library, b"pcap_set_promisc\0")?;
            let set_snaplen: Symbol<unsafe extern "C" fn(*mut c_void, c_int) -> c_int> = symbol(&library, b"pcap_set_snaplen\0")?;
            let set_immediate: Symbol<unsafe extern "C" fn(*mut c_void, c_int) -> c_int> = symbol(&library, b"pcap_set_immediate_mode\0")?;
            let activate: Symbol<unsafe extern "C" fn(*mut c_void) -> c_int> = symbol(&library, b"pcap_activate\0")?;
            let set_nonblock: Symbol<unsafe extern "C" fn(*mut c_void, c_int, *mut c_char) -> c_int> = symbol(&library, b"pcap_setnonblock\0")?;
            let send: Symbol<unsafe extern "C" fn(*mut c_void, *const c_uchar, c_int) -> c_int> = symbol(&library, b"pcap_sendpacket\0")?;
            let next: Symbol<unsafe extern "C" fn(*mut c_void, *mut *mut PacketHeader, *mut *const c_uchar) -> c_int> = symbol(&library, b"pcap_next_ex\0")?;
            let geterr: Symbol<unsafe extern "C" fn(*mut c_void) -> *const c_char> = symbol(&library, b"pcap_geterr\0")?;
            let close: Symbol<unsafe extern "C" fn(*mut c_void)> = symbol(&library, b"pcap_close\0")?;

            let mut errors = [0 as c_char; ERRBUF_SIZE];
            let handle = unsafe { create(name.as_ptr(), errors.as_mut_ptr()) };
            if handle.is_null() {
                return Err(format!("Couldn't open the adapter: {}", text(errors.as_ptr())));
            }
            let opened = unsafe {
                set_int(handle, 1);
                set_snaplen(handle, 65535);
                set_immediate(handle, 1);
                activate(handle)
            };
            if opened < 0 {
                let why = text(unsafe { geterr(handle) });
                unsafe { close(handle) };
                return Err(format!("Couldn't open the adapter: {}", why));
            }
            if unsafe { set_nonblock(handle, 1, errors.as_mut_ptr()) } != 0 {
                unsafe { close(handle) };
                return Err(format!("Couldn't set up the adapter: {}", text(errors.as_ptr())));
            }
            let (send, next, geterr, close) = (*send, *next, *geterr, *close);
            Ok(Self { handle, send, next, geterr, close, _library: library })
        }
    }

    impl RawLink for NpcapLink {
        fn send(&mut self, frame: &[u8]) -> Result<(), String> {
            if unsafe { (self.send)(self.handle, frame.as_ptr(), frame.len() as c_int) } != 0 {
                return Err(text(unsafe { (self.geterr)(self.handle) }));
            }
            Ok(())
        }

        fn receive(&mut self) -> Result<Option<Vec<u8>>, String> {
            let mut header: *mut PacketHeader = std::ptr::null_mut();
            let mut data: *const c_uchar = std::ptr::null();
            match unsafe { (self.next)(self.handle, &mut header, &mut data) } {
                1 => {
                    let length = unsafe { (*header).captured_length } as usize;
                    Ok(Some(unsafe { std::slice::from_raw_parts(data, length) }.to_vec()))
                }
                0 => Ok(None),
                _ => Err(text(unsafe { (self.geterr)(self.handle) })),
            }
        }
    }

    impl Drop for NpcapLink {
        fn drop(&mut self) {
            unsafe { (self.close)(self.handle) };
        }
    }
}

#[cfg(windows)]
fn list_adapters() -> Result<Vec<Adapter>, String> {
    npcap::adapters()
}

#[cfg(windows)]
fn open_link(adapter: &Adapter) -> Result<Box<dyn RawLink>, String> {
    Ok(Box::new(npcap::NpcapLink::open(&adapter.name)?))
}

#[cfg(not(windows))]
fn list_adapters() -> Result<Vec<Adapter>, String> {
    Err("This check only supports Windows so far.".to_string())
}

#[cfg(not(windows))]
fn open_link(_adapter: &Adapter) -> Result<Box<dyn RawLink>, String> {
    Err("This check only supports Windows so far.".to_string())
}

fn print_adapters(adapters: &[Adapter]) {
    println!("Network adapters:");
    for (i, adapter) in adapters.iter().enumerate() {
        println!(
            "  {}  {}{}",
            i + 1,
            if adapter.description.is_empty() { &adapter.name } else { &adapter.description },
            if adapter.connected { "  (link up)" } else { "" }
        );
    }
}

const USAGE: &str = "Add --trace to scan or watch to print every frame.
Usage:
  printer-host --ethercat list              Show network adapters
  printer-host --ethercat scan ADAPTER      Find devices. ADAPTER is a number from the list
  printer-host --ethercat inspect ADAPTER   Show what the first device reports about itself
  printer-host --ethercat layout ADAPTER    Show a BK1120's process data. Drives nothing
  printer-host --ethercat watch ADAPTER [SECONDS]
                                            Watch inputs. Never drives an output, and stops if a
                                            device has any beyond a coupler's control word";

/// The command line for this check. Returns the exit code: 0 worked, 1 didn't, 2 couldn't try.
/// Two copies on one bus wreck each other: each one's reset stops the other's run. The system
/// lets go of this lock when the program ends, however it ends.
fn claim_bus(lock_file: &std::path::Path) -> Result<std::fs::File, String> {
    let file = std::fs::OpenOptions::new()
        .create(true)
        .truncate(false)
        .write(true)
        .open(lock_file)
        .map_err(|e| format!("Couldn't check whether EtherCAT is in use: {}", e))?;
    match file.try_lock() {
        Ok(()) => Ok(file),
        Err(std::fs::TryLockError::WouldBlock) => {
            Err("Another printer-host is already using EtherCAT. Wait for it to finish, or stop it, and try again.".to_string())
        }
        Err(std::fs::TryLockError::Error(e)) => Err(format!("Couldn't check whether EtherCAT is in use: {}", e)),
    }
}

pub async fn run_cli(args: &[String]) -> i32 {
    let adapters = |action: &str| -> Result<Vec<Adapter>, i32> {
        list_adapters().map_err(|e| {
            eprintln!("Can't {}: {}", action, e);
            2
        })
    };
    match args.first().map(String::as_str) {
        Some("list") => match adapters("list adapters") {
            Ok(found) => {
                print_adapters(&found);
                0
            }
            Err(code) => code,
        },
        Some(action @ ("scan" | "watch" | "inspect" | "layout")) => {
            let Some(wanted) = args.get(1) else {
                eprintln!("{}", USAGE);
                return 2;
            };
            let _bus = match claim_bus(&std::env::temp_dir().join("printer-host-ethercat.lock")) {
                Ok(bus) => bus,
                Err(e) => {
                    eprintln!("{}", e);
                    return 2;
                }
            };
            let found = match adapters("open an adapter") {
                Ok(found) => found,
                Err(code) => return code,
            };
            let adapter = match pick_adapter(&found, wanted) {
                Ok(adapter) => adapter,
                Err(e) => {
                    eprintln!("{}", e);
                    print_adapters(&found);
                    return 2;
                }
            };
            println!("Using {}", if adapter.description.is_empty() { &adapter.name } else { &adapter.description });
            let link = match open_link(adapter) {
                Ok(link) if args.iter().any(|a| a == "--trace") => Box::new(TracingLink(link)) as Box<dyn RawLink>,
                Ok(link) => link,
                Err(e) => {
                    eprintln!("{}", e);
                    return 2;
                }
            };
            if action == "inspect" || action == "layout" {
                let result = if action == "inspect" { inspect(link) } else { ethercat_coupler::run_layout(link) };
                return match result {
                    Ok(()) => 0,
                    Err(e) => {
                        eprintln!("{}", e);
                        1
                    }
                };
            }
            let result = if action == "scan" {
                scan(link).map(|devices| report_scan(&devices))
            } else {
                let seconds = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(30);
                watch(link, seconds).await.map(|_| 0)
            };
            match result {
                Ok(code) => code,
                Err(e) => {
                    eprintln!("{}", e);
                    1
                }
            }
        }
        _ => {
            eprintln!("{}", USAGE);
            2
        }
    }
}

fn report_scan(devices: &[DeviceInfo]) -> i32 {
    if devices.is_empty() {
        eprintln!("Frames came back, but no EtherCAT device counted them. Check the cable goes to the coupler's IN port.");
        return 1;
    }
    println!("Found {} device(s):", devices.len());
    for device in devices {
        println!("  {}", device.describe());
    }
    if devices.iter().any(DeviceInfo::has_kbus) {
        println!("K-bus terminals such as a KL1104 aren't EtherCAT devices, so they aren't listed. Their inputs are in the coupler's data: use watch to see them.");
    }
    0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn adapter(name: &str, description: &str) -> Adapter {
        Adapter { name: name.to_string(), description: description.to_string(), connected: true }
    }

    fn sample() -> Vec<Adapter> {
        vec![
            adapter(r"\Device\NPF_{AAA}", "Intel Wi-Fi 6E"),
            adapter(r"\Device\NPF_{0ED096E0}", "Intel X710 10GBASE-T"),
            adapter(r"\Device\NPF_{BBB}", "Intel X710 10GBASE-T #2"),
        ]
    }

    #[test]
    fn adapters_are_picked_by_number_or_clear_name() {
        let list = sample();
        assert_eq!(pick_adapter(&list, "2").unwrap().name, list[1].name);
        assert_eq!(pick_adapter(&list, "wi-fi").unwrap().name, list[0].name);
        assert_eq!(pick_adapter(&list, "0ed096e0").unwrap().name, list[1].name);
        // A name that is also the start of another still means the exact one
        assert_eq!(pick_adapter(&list, "Intel X710 10GBASE-T").unwrap().name, list[1].name);
    }

    #[test]
    fn anything_unclear_is_refused_rather_than_guessed() {
        let list = sample();
        assert!(pick_adapter(&list, "x710").unwrap_err().contains("matches 2"));
        assert!(pick_adapter(&list, "intel").unwrap_err().contains("matches 3"));
        assert!(pick_adapter(&list, "printer").is_err());
        assert!(pick_adapter(&list, "0").is_err());
        assert!(pick_adapter(&list, "4").is_err());
    }

    #[test]
    fn bits_read_in_input_order() {
        assert_eq!(format_bits(&[0b0000_0101]), "1010 0000");
        assert_eq!(format_bits(&[0x0F, 0x80]), "1111 0000 | 0000 0001");
        assert_eq!(format_bits(&[]), "(none)");
    }

    #[test]
    fn a_coupler_with_a_kbus_is_recognised() {
        let device = |vendor, name: &str| DeviceInfo { position: 0, name: name.to_string(), vendor_id: vendor, product_id: 0, revision: 0, serial: 0 };
        assert!(device(2, "BK1120").has_kbus());
        assert!(!device(2, "EK1100").has_kbus());
        assert!(!device(9, "BK1120").has_kbus());
    }

    #[test]
    fn frames_are_described_in_words() {
        let mut frame = vec![0u8; 14];
        frame[6..12].copy_from_slice(&[0x12, 0x10, 0x10, 0x10, 0x10, 0x10]);
        frame[12..14].copy_from_slice(&ETHERCAT_ETHERTYPE);
        frame.extend_from_slice(&[0, 0x10]); // EtherCAT header
        frame.extend_from_slice(&[7, 1, 0, 0, 0x00, 0x00, 1, 0, 0, 0, 0x11, 1, 0]); // BRD, 1 byte, counter 1
        let text = describe_frame(&frame);
        assert!(text.contains("from 12:10:10:10:10:10"), "{}", text);
        assert!(text.contains("BRD device 0x0000 register 0x0000 data [11] counter 1"), "{}", text);

        frame[12] = 0x08;
        frame[13] = 0x00;
        assert!(describe_frame(&frame).contains("not EtherCAT"));
        assert!(describe_frame(&[1, 2, 3]).contains("short frame"));

        // A frame that claims more data than it has must not panic
        let mut cut = vec![0u8; 14];
        cut[12..14].copy_from_slice(&ETHERCAT_ETHERTYPE);
        cut.extend_from_slice(&[0, 0x10, 7, 1, 0, 0, 0, 0, 0xFF, 0x07, 0, 0, 1]);
        assert!(describe_frame(&cut).contains("cut short"));
    }

    /// Nothing on the wire: frames go out and nothing ever comes back.
    struct SilentLink;
    impl RawLink for SilentLink {
        fn send(&mut self, _frame: &[u8]) -> Result<(), String> {
            Ok(())
        }
        fn receive(&mut self) -> Result<Option<Vec<u8>>, String> {
            Ok(None)
        }
    }

    /// A cable with nothing on it that counts: each frame returns the way a passive loop would,
    /// marked as having been through hardware but with no device having touched it.
    struct LoopbackLink(std::collections::VecDeque<Vec<u8>>);
    impl RawLink for LoopbackLink {
        fn send(&mut self, frame: &[u8]) -> Result<(), String> {
            let mut returned = frame.to_vec();
            returned[6] |= 0x02;
            self.0.push_back(returned);
            Ok(())
        }
        fn receive(&mut self) -> Result<Option<Vec<u8>>, String> {
            Ok(self.0.pop_front())
        }
    }

    struct BrokenLink;
    impl RawLink for BrokenLink {
        fn send(&mut self, _frame: &[u8]) -> Result<(), String> {
            Err("the cable fell out".to_string())
        }
        fn receive(&mut self) -> Result<Option<Vec<u8>>, String> {
            Ok(None)
        }
    }

    #[test]
    fn silence_on_the_wire_gets_advice_instead_of_a_hang() {
        let error = scan(Box::new(SilentLink)).unwrap_err();
        assert!(error.contains("No reply"), "{}", error);
        assert!(error.contains("IN port"), "{}", error);
    }

    #[test]
    fn a_bus_with_no_devices_is_reported_as_that() {
        let devices = scan(Box::new(LoopbackLink(Default::default()))).unwrap();
        assert!(devices.is_empty());
        assert_eq!(report_scan(&devices), 1);
    }

    #[tokio::test]
    async fn watching_a_bus_with_no_devices_stops_cleanly() {
        let error = watch(Box::new(LoopbackLink(Default::default())), 1).await.unwrap_err();
        assert!(error.contains("No EtherCAT devices"), "{}", error);
    }

    #[test]
    fn only_one_run_can_hold_the_bus_at_a_time() {
        let folder = std::env::temp_dir().join(format!("printer-host-bus-test-{}", std::process::id()));
        std::fs::create_dir_all(&folder).unwrap();
        let lock_file = folder.join("bus.lock");

        let first = claim_bus(&lock_file).unwrap();
        let error = claim_bus(&lock_file).unwrap_err();
        assert!(error.contains("already using EtherCAT"), "{}", error);

        drop(first);
        assert!(claim_bus(&lock_file).is_ok(), "free again once the first run has gone");
        let _ = std::fs::remove_dir_all(&folder);
    }

    #[test]
    fn a_failing_adapter_is_named_as_the_cause() {
        let error = scan(Box::new(BrokenLink)).unwrap_err();
        assert!(error.contains("the cable fell out"), "{}", error);
    }

    #[test]
    fn a_bk1120_is_identified_from_its_eeprom() {
        let devices = scan(Box::new(ethercat_coupler::fake::bk1120(4, 0))).unwrap();
        assert_eq!(devices.len(), 1);
        assert_eq!(devices[0].name, "BK1120");
        assert_eq!(devices[0].product_id, BK1120_PRODUCT_CODE);
        assert!(devices[0].has_kbus());
    }

    #[test]
    fn scanning_never_asks_a_device_to_change_state() {
        let coupler = ethercat_coupler::fake::bk1120(4, 0);
        let seen = coupler.seen.clone();
        scan(Box::new(coupler)).unwrap();
        assert!(seen.lock().unwrap().states_requested.is_empty());
    }

    /// An office network: plenty of traffic, none of it EtherCAT, including frames shaped to look
    /// like a reply in every way except their type.
    struct BusyNetwork(std::collections::VecDeque<Vec<u8>>);
    impl RawLink for BusyNetwork {
        fn send(&mut self, frame: &[u8]) -> Result<(), String> {
            let mut lookalike = frame.to_vec();
            lookalike[6] |= 0x02;
            lookalike[12] = 0x08; // IPv4
            lookalike[13] = 0x00;
            self.0.push_back(lookalike);
            Ok(())
        }
        fn receive(&mut self) -> Result<Option<Vec<u8>>, String> {
            Ok(self.0.pop_front())
        }
    }

    #[test]
    fn other_traffic_on_the_adapter_is_not_taken_for_a_reply() {
        let error = scan(Box::new(BusyNetwork(Default::default()))).unwrap_err();
        assert!(error.contains("No reply"), "{}", error);
    }
}
