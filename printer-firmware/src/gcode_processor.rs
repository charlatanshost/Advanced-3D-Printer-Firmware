use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use heapless::String;

use crate::exclude_object::{parse_object_marker, EXCLUDE_OBJECT_MANAGER, OBJECT_END_MARKER};
use crate::safety::emergency_stop_active;

const MAX_GCODE_LINE_LENGTH: usize = 256;
const DEFAULT_MAX_HOTEND_TEMP: f32 = 275.0;
const DEFAULT_MAX_BED_TEMP: f32 = 150.0;

pub struct GcodeProcessor {
    current_line: u32,
    current_layer: u32,
    skipping_object: bool,
    /// Moves were skipped, so the machine's E counter is behind the file's
    extruder_resync_pending: bool,
    relative_xyz: bool,
    relative_e: bool,
    /// E before the most recent move, which is what a resync has to set the counter to
    last_e_before_move: f32,
    position: [f32; 4], // X, Y, Z, E as the file sees them, including skipped moves
    max_hotend_temp: f32,
    max_bed_temp: f32,
}

impl GcodeProcessor {
    pub fn new() -> Self {
        Self {
            current_line: 0,
            current_layer: 0,
            skipping_object: false,
            extruder_resync_pending: false,
            relative_xyz: false,
            relative_e: false,
            last_e_before_move: 0.0,
            position: [0.0; 4],
            max_hotend_temp: DEFAULT_MAX_HOTEND_TEMP,
            max_bed_temp: DEFAULT_MAX_BED_TEMP,
        }
    }

    /// Set the highest temperatures a G-code file is allowed to request
    pub fn set_temperature_limits(&mut self, hotend: f32, bed: f32) {
        self.max_hotend_temp = hotend;
        self.max_bed_temp = bed;
    }

    pub async fn process_gcode_line(&mut self, line: &str) -> Result<GcodeAction, GcodeError> {
        if emergency_stop_active() {
            return Err(GcodeError::EmergencyStopActive);
        }

        self.current_line += 1;
        let trimmed = line.trim();

        // Markers live in comments, so look for them before comments are removed.
        if let Some(layer) = Self::parse_layer_comment(trimmed) {
            self.current_layer = layer;
            return Ok(GcodeAction::LayerChange(layer));
        }
        if let Some(object_name) = parse_object_marker(trimmed) {
            let name = object_name.clone();
            self.handle_object_start(&name).await;
            return Ok(GcodeAction::ObjectStart(object_name));
        }
        if trimmed.contains(OBJECT_END_MARKER) {
            self.handle_object_end().await;
            return Ok(GcodeAction::ObjectEnd);
        }

        // Everything after a semicolon is a comment, and must never be read as parameters.
        let code = trimmed.split(';').next().unwrap_or("").trim();
        if code.is_empty() {
            return Ok(GcodeAction::Skip);
        }
        if code.len() > MAX_GCODE_LINE_LENGTH {
            return Err(GcodeError::LineTooLong);
        }

        let command = self.parse_gcode_command(code)?;
        self.track_modes_and_position(&command);

        // While skipping an object only motion is dropped. Fan, temperature and other
        // commands in its section still apply to the rest of the print.
        if self.skipping_object && matches!(command, GcodeCommand::Move { .. } | GcodeCommand::Arc { .. }) {
            self.extruder_resync_pending = !self.relative_e;
            return Ok(GcodeAction::Skip);
        }

        Ok(self.next_action(command))
    }

    /// In absolute extrusion, a skipped move leaves the machine's E counter behind the
    /// file's. The next move would then try to extrude all the skipped filament in one go,
    /// so tell the executor to bring the counter up to date first.
    fn next_action(&mut self, command: GcodeCommand) -> GcodeAction {
        if let GcodeCommand::Move { e, .. } | GcodeCommand::Arc { e, .. } = &command {
            if self.extruder_resync_pending {
                self.extruder_resync_pending = false;
                // Counter value just before this move
                let before_move = if e.is_some() { self.last_e_before_move } else { self.position[3] };
                return GcodeAction::ResyncExtruderThenExecute { e: before_move, command };
            }
        }
        GcodeAction::Execute(command)
    }

    fn parse_layer_comment(line: &str) -> Option<u32> {
        let rest = line.strip_prefix(";LAYER:").or_else(|| line.strip_prefix("; layer "))?;
        rest.trim().parse().ok()
    }

    fn parse_gcode_command(&self, code: &str) -> Result<GcodeCommand, GcodeError> {
        // Strip any checksum. Letter case is handled per word.
        let code = code.split('*').next().unwrap_or("").trim();
        let mut words = Words::new(code);

        let mut first = words.next().ok_or(GcodeError::InvalidCommand)?;
        if first.letter == 'N' {
            first = words.next().ok_or(GcodeError::InvalidCommand)?; // Line number
        }

        // Whole numbers only: G1.5 is not G1
        let number = first.value.filter(|v| *v >= 0.0 && (*v as u32) as f32 == *v).map(|v| v as u32);
        match (first.letter, number) {
            ('G', Some(0 | 1)) => {
                let (x, y, z, e, feed_rate) = Self::parse_motion_words(words)?;
                Ok(GcodeCommand::Move { x, y, z, e, feed_rate })
            }
            ('G', Some(2 | 3)) => {
                let mut x = None;
                let mut y = None;
                let mut z = None;
                let mut e = None;
                for word in words {
                    let value = word.value;
                    match word.letter {
                        'X' => x = value,
                        'Y' => y = value,
                        'Z' => z = value,
                        'E' => e = value,
                        _ => {}
                    }
                }
                Ok(GcodeCommand::Arc { x, y, z, e, command: Self::raw_command(code) })
            }
            ('G', Some(28)) => {
                let mut axes = String::new();
                for word in words {
                    if matches!(word.letter, 'X' | 'Y' | 'Z') {
                        axes.push(word.letter).map_err(|_| GcodeError::ParseError)?;
                    }
                }
                Ok(GcodeCommand::Home { axes })
            }
            ('G', Some(92)) => {
                let (x, y, z, e, _) = Self::parse_motion_words(words)?;
                Ok(GcodeCommand::SetPosition { x, y, z, e })
            }
            ('M', Some(104)) => {
                let mut tool = 0u8;
                let mut temp = None;
                for word in words {
                    match word.letter {
                        'T' => tool = word.required()? as u8,
                        'S' => temp = Some(word.required()?),
                        _ => {}
                    }
                }
                // No S is not "turn it off". A typo must not change the temperature.
                let temp = temp.ok_or(GcodeError::ParseError)?;
                Self::check_temperature(temp, self.max_hotend_temp)?;
                Ok(GcodeCommand::SetTemperature { tool, temp })
            }
            ('M', Some(140)) => {
                let mut temp = None;
                for word in words {
                    if word.letter == 'S' {
                        temp = Some(word.required()?);
                    }
                }
                let temp = temp.ok_or(GcodeError::ParseError)?;
                Self::check_temperature(temp, self.max_bed_temp)?;
                Ok(GcodeCommand::SetBedTemperature { temp })
            }
            // The wait-for-temperature forms also set the target, so they need the same limit
            ('M', Some(n @ (109 | 190))) => {
                let limit = if n == 109 { self.max_hotend_temp } else { self.max_bed_temp };
                for word in words {
                    if matches!(word.letter, 'S' | 'R') {
                        Self::check_temperature(word.required()?, limit)?;
                    }
                }
                Ok(GcodeCommand::Other { command: Self::raw_command(code) })
            }
            ('M', Some(106)) => {
                let mut fan = 0u8;
                let mut speed = 255.0f32;
                for word in words {
                    match word.letter {
                        'P' => fan = word.required()? as u8,
                        // Slicers send fractions such as S127.5
                        'S' => speed = word.required()?,
                        _ => {}
                    }
                }
                Ok(GcodeCommand::SetFanSpeed { fan, speed: fan_speed_to_u8(speed) })
            }
            ('M', Some(107)) => {
                let mut fan = 0u8;
                for word in words {
                    if word.letter == 'P' {
                        fan = word.required()? as u8;
                    }
                }
                Ok(GcodeCommand::SetFanSpeed { fan, speed: 0 })
            }
            _ => Ok(GcodeCommand::Other { command: Self::raw_command(code) }),
        }
    }

    fn raw_command(code: &str) -> String<MAX_GCODE_LINE_LENGTH> {
        // Length was checked by the caller
        String::try_from(code).unwrap_or_default()
    }

    /// Parse X/Y/Z/E/F. A malformed value is an error, not a dropped axis: running
    /// `G1 X1O Y5` as just `G1 Y5` would quietly print the wrong thing.
    #[allow(clippy::type_complexity)]
    fn parse_motion_words(words: Words<'_>) -> Result<(Option<f32>, Option<f32>, Option<f32>, Option<f32>, Option<f32>), GcodeError> {
        let (mut x, mut y, mut z, mut e, mut f) = (None, None, None, None, None);
        for word in words {
            let value = Some(word.required()?);
            match word.letter {
                'X' => x = value,
                'Y' => y = value,
                'Z' => z = value,
                'E' => e = value,
                'F' => {
                    if word.required()? <= 0.0 {
                        return Err(GcodeError::ParseError);
                    }
                    f = value;
                }
                _ => return Err(GcodeError::ParseError),
            }
        }
        Ok((x, y, z, e, f))
    }

    fn check_temperature(temp: f32, limit: f32) -> Result<(), GcodeError> {
        // Written so that NaN fails too
        if temp >= 0.0 && temp <= limit {
            Ok(())
        } else {
            Err(GcodeError::TemperatureOutOfRange)
        }
    }

    /// Follow G90/G91/M82/M83 and keep the file's position up to date, including for moves
    /// that end up skipped.
    fn track_modes_and_position(&mut self, command: &GcodeCommand) {
        match command {
            GcodeCommand::Move { x, y, z, e, .. } | GcodeCommand::Arc { x, y, z, e, .. } => {
                self.last_e_before_move = self.position[3];
                let relative = self.relative_xyz;
                for (i, value) in [x, y, z].into_iter().enumerate() {
                    if let Some(v) = value {
                        self.position[i] = if relative { self.position[i] + v } else { *v };
                    }
                }
                if let Some(v) = e {
                    self.position[3] = if self.relative_e { self.position[3] + v } else { *v };
                }
            }
            GcodeCommand::SetPosition { x, y, z, e } => {
                for (i, value) in [x, y, z, e].into_iter().enumerate() {
                    if let Some(v) = value {
                        self.position[i] = *v;
                    }
                }
                // Setting E re-bases the counter, on the machine too once this runs
                if e.is_some() {
                    self.extruder_resync_pending = false;
                }
            }
            GcodeCommand::Other { command } => {
                let mut words = Words::new(command);
                if let Some(first) = words.next() {
                    match (first.letter, first.value.map(|v| v as u32)) {
                        ('G', Some(90)) => {
                            self.relative_xyz = false;
                            self.relative_e = false;
                        }
                        ('G', Some(91)) => {
                            self.relative_xyz = true;
                            self.relative_e = true;
                        }
                        ('M', Some(82)) => self.relative_e = false,
                        ('M', Some(83)) => self.relative_e = true,
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }

    async fn handle_object_start(&mut self, object_name: &str) {
        let line = self.current_line;
        let mut manager_guard = EXCLUDE_OBJECT_MANAGER.lock().await;
        if let Some(ref mut manager) = *manager_guard {
            self.skipping_object = manager.object_started(object_name, line).await;
        }
    }

    async fn handle_object_end(&mut self) {
        if self.skipping_object {
            self.skipping_object = false;
            return;
        }
        let line = self.current_line;
        let mut manager_guard = EXCLUDE_OBJECT_MANAGER.lock().await;
        if let Some(ref mut manager) = *manager_guard {
            manager.object_finished(line).await;
        }
    }

    /// Get ready for a new print
    pub fn reset(&mut self) {
        let (hotend, bed) = (self.max_hotend_temp, self.max_bed_temp);
        *self = Self::new();
        self.set_temperature_limits(hotend, bed);
    }

    pub fn get_current_position(&self) -> [f32; 4] {
        self.position
    }

    pub fn get_current_line(&self) -> u32 {
        self.current_line
    }

    pub fn get_current_layer(&self) -> u32 {
        self.current_layer
    }
}

fn fan_speed_to_u8(speed: f32) -> u8 {
    // Out of range clamps, NaN becomes 0. `as` is saturating, so add 0.5 to round.
    (speed + 0.5) as u8
}

/// One G-code word: a letter and its number
#[derive(Debug, Clone, Copy)]
struct Word {
    letter: char,
    value: Option<f32>,
}

impl Word {
    fn required(&self) -> Result<f32, GcodeError> {
        self.value.filter(|v| v.is_finite()).ok_or(GcodeError::ParseError)
    }
}

/// Splits `G1X10 y-2.5` into words. No fixed-size buffer: a long line cannot overflow it,
/// and the firmware must never panic on the contents of a file.
struct Words<'a> {
    rest: &'a str,
}

impl<'a> Words<'a> {
    fn new(code: &'a str) -> Self {
        Self { rest: code }
    }
}

impl Iterator for Words<'_> {
    type Item = Word;

    fn next(&mut self) -> Option<Word> {
        self.rest = self.rest.trim_start();
        let first = self.rest.chars().next()?;
        let after_letter = &self.rest[first.len_utf8()..];
        let number_length = after_letter
            .find(|c: char| !(c.is_ascii_digit() || c == '.' || c == '-' || c == '+'))
            .unwrap_or(after_letter.len());
        let (number, rest) = after_letter.split_at(number_length);
        self.rest = rest;
        Some(Word {
            letter: first.to_ascii_uppercase(),
            value: number.parse().ok(),
        })
    }
}

#[derive(Debug, Clone)]
pub enum GcodeCommand {
    Move {
        x: Option<f32>,
        y: Option<f32>,
        z: Option<f32>,
        e: Option<f32>,
        feed_rate: Option<f32>
    },
    /// G2/G3. The end point is tracked here, and the full command is kept for the executor.
    Arc {
        x: Option<f32>,
        y: Option<f32>,
        z: Option<f32>,
        e: Option<f32>,
        command: String<MAX_GCODE_LINE_LENGTH>,
    },
    Home { axes: String<8> },
    SetPosition { x: Option<f32>, y: Option<f32>, z: Option<f32>, e: Option<f32> },
    SetTemperature { tool: u8, temp: f32 },
    SetBedTemperature { temp: f32 },
    SetFanSpeed { fan: u8, speed: u8 },
    Dwell { time: f32 },
    Other { command: String<MAX_GCODE_LINE_LENGTH> },
}

#[derive(Debug)]
pub enum GcodeAction {
    Execute(GcodeCommand),
    /// Set the machine's E counter to `e` (like G92 E), then run the command
    ResyncExtruderThenExecute { e: f32, command: GcodeCommand },
    ExecuteRaw(String<MAX_GCODE_LINE_LENGTH>),
    Skip,
    LayerChange(u32),
    ObjectStart(String<32>),
    ObjectEnd,
}

#[derive(Debug)]
pub enum GcodeError {
    InvalidCommand,
    ParseError,
    BufferFull,
    LineTooLong,
    TemperatureOutOfRange,
    EmergencyStopActive,
}

pub static GCODE_PROCESSOR: Mutex<CriticalSectionRawMutex, Option<GcodeProcessor>> = Mutex::new(None);

pub async fn init_gcode_processor() -> &'static Mutex<CriticalSectionRawMutex, Option<GcodeProcessor>> {
    let processor = GcodeProcessor::new();

    {
        let mut guard = GCODE_PROCESSOR.lock().await;
        *guard = Some(processor);
    }

    &GCODE_PROCESSOR
}

// Print progress tracking
pub struct PrintProgress {
    pub total_lines: u32,
    pub processed_lines: u32,
    pub total_objects: usize,
    pub completed_objects: usize,
    pub excluded_objects: usize,
    pub current_layer: u32,
    pub estimated_layers: u32,
}

impl PrintProgress {
    pub fn progress_percentage(&self) -> f32 {
        if self.total_lines > 0 {
            (self.processed_lines as f32 / self.total_lines as f32) * 100.0
        } else {
            0.0
        }
    }

    pub fn layer_progress_percentage(&self) -> f32 {
        if self.estimated_layers > 0 {
            (self.current_layer as f32 / self.estimated_layers as f32) * 100.0
        } else {
            0.0
        }
    }

    pub fn objects_progress_percentage(&self) -> f32 {
        let active_objects = self.total_objects.saturating_sub(self.excluded_objects);
        if active_objects > 0 {
            (self.completed_objects as f32 / active_objects as f32) * 100.0
        } else {
            0.0
        }
    }
}
