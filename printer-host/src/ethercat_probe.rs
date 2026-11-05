//! A read-only EtherCAT check: find the devices on a network adapter and watch their inputs.
//!
//! This is real EtherCAT, using the ethercrab master over Npcap on Windows. It is deliberately
//! limited, because it is the first thing here to touch real hardware:
//! - it only uses the adapter you name, since these frames don't belong on an office network
//! - it never writes outputs, and won't go operational if a device has any
//!
//! Npcap is loaded when needed instead of linked, so the app builds and runs without it.

use ethercrab::{MainDevice, MainDeviceConfig, PduRx, PduStorage, PduTx, Timeouts};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

const MAX_SUBDEVICES: usize = 16;
const MAX_FRAMES: usize = 16;
const MAX_PDU_DATA: usize = PduStorage::element_size(1100);
const PDI_LEN: usize = 256;
const ETHERCAT_ETHERTYPE: [u8; 2] = [0x88, 0xA4];
const BECKHOFF_VENDOR_ID: u32 = 2;

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
    pub address: u16,
    pub name: String,
    pub vendor_id: u32,
    pub product_id: u32,
    pub revision: u32,
    pub serial: u32,
}

impl DeviceInfo {
    fn describe(&self) -> String {
        format!(
            "{} at address {:#06x}: vendor {:#x}{}, product {:#010x}, revision {:#x}, serial {}",
            self.name,
            self.address,
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

/// Find the devices on the link. This reads their identity and changes nothing they do.
pub async fn scan(link: Box<dyn RawLink>, timeouts: Timeouts) -> Result<Vec<DeviceInfo>, String> {
    let session = Session::start(link, timeouts)?;
    let group = session
        .main_device
        .init_single_group::<MAX_SUBDEVICES, PDI_LEN>(ethercat_now)
        .await
        .map_err(|e| session.explain("Looking for devices", e))?;

    Ok(group
        .iter(session.main_device)
        .map(|device| {
            let identity = device.identity();
            DeviceInfo {
                address: device.configured_address(),
                name: device.name().to_string(),
                vendor_id: identity.vendor_id,
                product_id: identity.product_id,
                revision: identity.revision,
                serial: identity.serial,
            }
        })
        .collect())
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

const USAGE: &str = "Usage:
  printer-host --ethercat list              Show network adapters
  printer-host --ethercat scan ADAPTER      Find devices. ADAPTER is a number from the list
  printer-host --ethercat watch ADAPTER [SECONDS]
                                            Watch inputs. Refuses if a device has outputs";

/// The command line for this check. Returns the exit code: 0 worked, 1 didn't, 2 couldn't try.
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
        Some(action @ ("scan" | "watch")) => {
            let Some(wanted) = args.get(1) else {
                eprintln!("{}", USAGE);
                return 2;
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
                Ok(link) => link,
                Err(e) => {
                    eprintln!("{}", e);
                    return 2;
                }
            };
            let result = if action == "scan" {
                scan(link, default_timeouts()).await.map(|devices| report_scan(&devices))
            } else {
                let seconds = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(30);
                watch_inputs(link, default_timeouts(), seconds).await.map(|_| 0)
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
        eprintln!("The adapter works, but no EtherCAT devices answered. Check the cable goes to the coupler's IN port.");
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
        let device = |vendor, name: &str| DeviceInfo { address: 0x1000, name: name.to_string(), vendor_id: vendor, product_id: 0, revision: 0, serial: 0 };
        assert!(device(2, "BK1120").has_kbus());
        assert!(!device(2, "EK1100").has_kbus());
        assert!(!device(9, "BK1120").has_kbus());
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

    fn quick() -> Timeouts {
        Timeouts { pdu: Duration::from_millis(20), ..default_timeouts() }
    }

    #[tokio::test]
    async fn silence_on_the_wire_gets_advice_instead_of_a_hang() {
        let error = scan(Box::new(SilentLink), quick()).await.unwrap_err();
        assert!(error.contains("no reply"), "{}", error);
        assert!(error.contains("IN port"), "{}", error);
    }

    #[tokio::test]
    async fn a_bus_with_no_devices_is_reported_as_that() {
        let devices = scan(Box::new(LoopbackLink(Default::default())), quick()).await.unwrap();
        assert!(devices.is_empty());
        assert_eq!(report_scan(&devices), 1);
    }

    #[tokio::test]
    async fn watching_a_bus_with_no_devices_stops_cleanly() {
        let error = watch_inputs(Box::new(LoopbackLink(Default::default())), quick(), 1).await.unwrap_err();
        assert!(error.contains("No EtherCAT devices"), "{}", error);
    }

    #[tokio::test]
    async fn a_failing_adapter_is_named_as_the_cause() {
        let error = scan(Box::new(BrokenLink), quick()).await.unwrap_err();
        assert!(error.contains("the cable fell out"), "{}", error);
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

    #[tokio::test]
    async fn other_traffic_on_the_adapter_is_not_taken_for_a_reply() {
        let error = scan(Box::new(BusyNetwork(Default::default())), quick()).await.unwrap_err();
        assert!(error.contains("no reply"), "{}", error);
    }
}
