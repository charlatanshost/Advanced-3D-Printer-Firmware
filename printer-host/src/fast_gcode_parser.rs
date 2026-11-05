// High-Performance Streaming G-Code Parser
// Uses zero-copy parsing and state machines for maximum speed
// Designed to handle large files efficiently on both SBCs and PCs

use std::io::{BufReader, Read};
use std::collections::HashMap;
use std::time::Instant;
use rayon::prelude::*;

#[derive(Debug, Clone, PartialEq)]
pub struct GCodeCommand {
    pub command_type: CommandType,
    pub line_number: u32,
    pub parameters: Parameters,
    pub comment: Option<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum CommandType {
    G(u16),  // G-commands (G0, G1, G2, etc.)
    M(u16),  // M-commands (M104, M109, M600, etc.)
    T(u16),  // Tool commands (T0, T1, etc.)
    Other(String), // Other commands (F, S, Klipper macros like START_PRINT, etc.)
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct Parameters {
    pub x: Option<f32>,
    pub y: Option<f32>,
    pub z: Option<f32>,
    pub e: Option<f32>,
    pub f: Option<f32>,  // Feedrate
    pub s: Option<f32>,  // Spindle speed
    pub p: Option<f32>,  // Parameter
    pub i: Option<f32>,  // Arc center I
    pub j: Option<f32>,  // Arc center J
    pub r: Option<f32>,  // Arc radius
    pub other: HashMap<char, f32>, // Other parameters
    pub flags: Vec<char>,          // Letters with no value, as in `G28 X Y`
    pub text: Option<String>,      // Free text, as in `M117 Hello` or `MACRO NAME=value`
}

#[derive(Debug)]
pub struct ParseStatistics {
    pub total_lines: u32,
    pub parsed_commands: u32,
    pub parsing_time: std::time::Duration,
    pub lines_per_second: f64,
    pub memory_used: usize,
}

pub struct StreamingGCodeParser {
    line_number: u32,
    buffer_size: usize,
    parallel_chunks: usize,
    statistics: ParseStatistics,
}

impl StreamingGCodeParser {
    pub fn new() -> Self {
        Self {
            line_number: 0,
            buffer_size: 64 * 1024, // 64KB buffer
            parallel_chunks: num_cpus::get(),
            statistics: ParseStatistics {
                total_lines: 0,
                parsed_commands: 0,
                parsing_time: std::time::Duration::ZERO,
                lines_per_second: 0.0,
                memory_used: 0,
            },
        }
    }
    
    pub fn with_buffer_size(mut self, size: usize) -> Self {
        self.buffer_size = size;
        self
    }
    
    pub fn with_parallel_chunks(mut self, chunks: usize) -> Self {
        self.parallel_chunks = chunks;
        self
    }
    
    /// Parse a g-code file using streaming approach
    pub fn parse_file<R: Read>(&mut self, reader: R) -> Result<Vec<GCodeCommand>, ParseError> {
        let start_time = Instant::now();
        let mut buf_reader = BufReader::with_capacity(self.buffer_size, reader);
        let mut commands = Vec::with_capacity(10000); // Pre-allocate for typical file sizes
        
        // Read file in chunks for parallel processing if file is large enough
        let mut buffer = Vec::with_capacity(self.buffer_size);
        buf_reader.read_to_end(&mut buffer)?;
        
        if buffer.len() > 1024 * 1024 { // 1MB threshold for parallel processing
            commands = self.parse_parallel(&buffer)?;
        } else {
            commands = self.parse_sequential(&buffer)?;
        }
        
        self.statistics.parsing_time = start_time.elapsed();
        self.statistics.lines_per_second = 
            self.statistics.total_lines as f64 / self.statistics.parsing_time.as_secs_f64();
        self.statistics.memory_used = commands.capacity() * std::mem::size_of::<GCodeCommand>();
        
        Ok(commands)
    }
    
    /// Parse using parallel processing for large files
    fn parse_parallel(&mut self, buffer: &[u8]) -> Result<Vec<GCodeCommand>, ParseError> {
        // Split buffer into chunks, being careful about line boundaries
        let chunk_count = self.parallel_chunks.max(1);
        let chunk_size = buffer.len() / chunk_count;
        let mut chunks = Vec::new();
        let mut start = 0;

        for _ in 0..chunk_count - 1 {
            if start >= buffer.len() {
                break;
            }
            let mut end = (start + chunk_size).min(buffer.len());

            // Find the next newline to avoid splitting lines
            while end < buffer.len() && buffer[end] != b'\n' {
                end += 1;
            }
            if end < buffer.len() {
                end += 1; // Include the newline
            }
            
            chunks.push((start, end));
            start = end;
        }
        
        // Add the final chunk
        if start < buffer.len() {
            chunks.push((start, buffer.len()));
        }

        // Each chunk's first line number is the number of lines before it. Chunks can hold
        // any number of lines, so this has to be counted, not assumed.
        let mut first_lines = Vec::with_capacity(chunks.len());
        let mut lines_so_far = 0u32;
        for &(start, end) in &chunks {
            first_lines.push(lines_so_far);
            lines_so_far += count_lines(&buffer[start..end]);
        }
        self.statistics.total_lines = lines_so_far;

        // Parse chunks in parallel. collect() on an indexed parallel iterator keeps chunk
        // order, so the results are already in file order.
        let results: Result<Vec<_>, _> = chunks
            .into_par_iter()
            .zip(first_lines.into_par_iter())
            .map(|((start, end), first_line)| {
                let chunk = &buffer[start..end];
                let mut parser = FastLineParser::new(first_line);
                parser.parse_chunk(chunk)
            })
            .collect();

        let mut all_commands = Vec::new();
        for chunk_commands in results? {
            self.statistics.parsed_commands += chunk_commands.len() as u32;
            all_commands.extend(chunk_commands);
        }

        Ok(all_commands)
    }
    
    /// Parse using sequential processing for smaller files
    fn parse_sequential(&mut self, buffer: &[u8]) -> Result<Vec<GCodeCommand>, ParseError> {
        let mut parser = FastLineParser::new(0);
        let commands = parser.parse_chunk(buffer)?;
        self.statistics.parsed_commands = commands.len() as u32;
        self.statistics.total_lines = count_lines(buffer);
        Ok(commands)
    }
    
    pub fn get_statistics(&self) -> &ParseStatistics {
        &self.statistics
    }
}

/// Number of lines in a buffer, counting a last line with no newline
fn count_lines(buffer: &[u8]) -> u32 {
    let newlines = buffer.iter().filter(|&&b| b == b'\n').count() as u32;
    newlines + u32::from(buffer.last().map_or(false, |&b| b != b'\n'))
}

/// Fast line-by-line parser using state machine approach
struct FastLineParser {
    base_line_number: u32,
}

impl FastLineParser {
    fn new(base_line_number: u32) -> Self {
        Self { base_line_number }
    }
    
    fn parse_chunk(&mut self, buffer: &[u8]) -> Result<Vec<GCodeCommand>, ParseError> {
        let mut commands = Vec::with_capacity(1000);
        let mut line_start = 0;
        let mut line_number = self.base_line_number;
        
        for i in 0..buffer.len() {
            if buffer[i] == b'\n' {
                let line = &buffer[line_start..i];
                
                if let Some(command) = self.parse_line_fast(line, line_number)? {
                    commands.push(command);
                }
                
                line_start = i + 1;
                line_number += 1;
            }
        }
        
        // Handle the last line if it doesn't end with a newline
        if line_start < buffer.len() {
            let line = &buffer[line_start..];
            if let Some(command) = self.parse_line_fast(line, line_number)? {
                commands.push(command);
            }
        }
        
        Ok(commands)
    }
    
    /// Ultra-fast line parsing using zero-copy techniques
    fn parse_line_fast(&self, line: &[u8], line_number: u32) -> Result<Option<GCodeCommand>, ParseError> {
        if line.is_empty() {
            return Ok(None);
        }
        
        // Trim whitespace
        let mut start = 0;
        let mut end = line.len();
        
        while start < end && line[start].is_ascii_whitespace() {
            start += 1;
        }
        
        while end > start && line[end - 1].is_ascii_whitespace() {
            end -= 1;
        }
        
        if start >= end {
            return Ok(None);
        }
        
        let trimmed = &line[start..end];
        
        // Skip comments and empty lines
        if trimmed.is_empty() || trimmed[0] == b';' || trimmed[0] == b'(' {
            return Ok(None);
        }
        
        let mut parser_state = LineParserState::new(trimmed);
        let command = parser_state.parse(line_number)?;
        
        Ok(Some(command))
    }
}

/// State machine for parsing individual g-code lines
struct LineParserState<'a> {
    input: &'a [u8],
    pos: usize,
}

impl<'a> LineParserState<'a> {
    fn new(input: &'a [u8]) -> Self {
        Self { input, pos: 0 }
    }
    
    fn parse(&mut self, line_number: u32) -> Result<GCodeCommand, ParseError> {
        let mut parameters = Parameters::default();
        let mut comment = None;

        // Extended commands (START_PRINT, EXCLUDE_OBJECT_DEFINE NAME=a): a word of letters
        // with free-form arguments that must not be read as G-code parameters.
        let word_end = self.input.iter()
            .position(|b| b.is_ascii_whitespace() || *b == b';')
            .unwrap_or(self.input.len());
        let is_extended = word_end >= 2
            && self.input[0].is_ascii_alphabetic()
            && self.input[1].is_ascii_alphabetic();
        if is_extended {
            let name = String::from_utf8_lossy(&self.input[..word_end]).into_owned();
            self.pos = word_end;
            parameters.text = self.parse_free_text(&mut comment);
            return Ok(GCodeCommand { command_type: CommandType::Other(name), line_number, parameters, comment });
        }

        let command_type = self.parse_command_type()?;

        // These take a free-text argument (a message or file name), not parameters.
        if matches!(command_type, CommandType::M(23 | 28 | 29 | 30 | 32 | 117 | 118)) {
            parameters.text = self.parse_free_text(&mut comment);
            return Ok(GCodeCommand { command_type, line_number, parameters, comment });
        }

        // Parse parameters
        while self.pos < self.input.len() {
            self.skip_whitespace();
            
            if self.pos >= self.input.len() {
                break;
            }
            
            let ch = self.input[self.pos] as char;
            
            // Handle comments
            if ch == ';' || ch == '(' {
                comment = Some(self.parse_comment());
                break;
            }
            
            // Parse parameter
            if ch.is_ascii_alphabetic() {
                self.parse_parameter(ch, &mut parameters)?;
            } else {
                self.pos += 1;
            }
        }
        
        Ok(GCodeCommand {
            command_type,
            line_number,
            parameters,
            comment,
        })
    }
    
    fn parse_command_type(&mut self) -> Result<CommandType, ParseError> {
        if self.pos >= self.input.len() {
            return Err(ParseError::UnexpectedEnd);
        }
        
        let cmd_char = self.input[self.pos] as char;
        self.pos += 1;
        
        match cmd_char {
            'G' | 'g' => Ok(self.parse_number_u16().map_or_else(|_| CommandType::Other("G".to_string()), CommandType::G)),
            'M' | 'm' => Ok(self.parse_number_u16().map_or_else(|_| CommandType::Other("M".to_string()), CommandType::M)),
            'T' | 't' => Ok(self.parse_number_u16().map_or_else(|_| CommandType::Other("T".to_string()), CommandType::T)),
            _ => Ok(CommandType::Other(cmd_char.to_string())),
        }
    }
    
    /// The rest of the line as text, stopping at a comment
    fn parse_free_text(&mut self, comment: &mut Option<String>) -> Option<String> {
        self.skip_whitespace();
        let rest = &self.input[self.pos..];
        let end = rest.iter().position(|&b| b == b';').unwrap_or(rest.len());
        let text = String::from_utf8_lossy(&rest[..end]).trim().to_string();
        self.pos += end;
        if self.pos < self.input.len() {
            *comment = Some(self.parse_comment());
        }
        if text.is_empty() { None } else { Some(text) }
    }

    fn parse_parameter(&mut self, param_char: char, parameters: &mut Parameters) -> Result<(), ParseError> {
        self.pos += 1; // Skip the parameter character
        // A letter with no number is valid G-code (`G28 X`), so it is a flag and not an error.
        let saved = self.pos;
        let Ok(value) = self.parse_number_f32() else {
            self.pos = saved;
            parameters.flags.push(param_char.to_ascii_uppercase());
            return Ok(());
        };

        match param_char.to_ascii_uppercase() {
            'X' => parameters.x = Some(value),
            'Y' => parameters.y = Some(value),
            'Z' => parameters.z = Some(value),
            'E' => parameters.e = Some(value),
            'F' => parameters.f = Some(value),
            'S' => parameters.s = Some(value),
            'P' => parameters.p = Some(value),
            'I' => parameters.i = Some(value),
            'J' => parameters.j = Some(value),
            'R' => parameters.r = Some(value),
            _ => {
                parameters.other.insert(param_char.to_ascii_uppercase(), value);
            }
        }
        
        Ok(())
    }
    
    /// Fast integer parsing without allocation
    fn parse_number_u16(&mut self) -> Result<u16, ParseError> {
        self.skip_whitespace();

        let start = self.pos;
        let mut result = 0u16;

        while self.pos < self.input.len() {
            let ch = self.input[self.pos];
            if ch.is_ascii_digit() {
                let digit = (ch - b'0') as u16;
                result = result.saturating_mul(10).saturating_add(digit);
                self.pos += 1;
            } else {
                break;
            }
        }
        
        if self.pos == start {
            return Err(ParseError::InvalidNumber);
        }
        
        Ok(result)
    }
    
    /// Fast float parsing optimized for g-code values
    fn parse_number_f32(&mut self) -> Result<f32, ParseError> {
        self.skip_whitespace();
        
        let start = self.pos;
        let mut negative = false;
        
        // Handle sign
        if self.pos < self.input.len() && (self.input[self.pos] == b'-' || self.input[self.pos] == b'+') {
            negative = self.input[self.pos] == b'-';
            self.pos += 1;
        }
        
        let mut integer_part = 0i64;
        let mut decimal_part = 0i64;
        let mut decimal_places = 0;
        let mut has_decimal = false;
        
        // Parse integer part
        while self.pos < self.input.len() && self.input[self.pos].is_ascii_digit() {
            integer_part = integer_part.saturating_mul(10).saturating_add((self.input[self.pos] - b'0') as i64);
            self.pos += 1;
        }
        
        // Parse decimal part
        if self.pos < self.input.len() && self.input[self.pos] == b'.' {
            has_decimal = true;
            self.pos += 1;
            
            while self.pos < self.input.len() && self.input[self.pos].is_ascii_digit() && decimal_places < 6 {
                decimal_part = decimal_part * 10 + (self.input[self.pos] - b'0') as i64;
                decimal_places += 1;
                self.pos += 1;
            }
            // Precision past 6 places is dropped, and must not be read as a new word
            while self.pos < self.input.len() && self.input[self.pos].is_ascii_digit() {
                self.pos += 1;
            }
        }
        
        if self.pos == start || (self.pos == start + 1 && (self.input[start] == b'-' || self.input[start] == b'+')) {
            return Err(ParseError::InvalidNumber);
        }
        
        // Convert to float
        let mut result = integer_part as f32;
        if has_decimal && decimal_places > 0 {
            let divisor = 10f32.powi(decimal_places as i32);
            result += decimal_part as f32 / divisor;
        }
        
        // Scientific notation, lowercase only: an uppercase E after a number is the extruder
        // word (`X1E5` is X=1, E=5), but `1e-2` is one value and not X=1 and a 2mm retract.
        if self.pos < self.input.len() && self.input[self.pos] == b'e' {
            let mut p = self.pos + 1;
            let exponent_negative = p < self.input.len() && self.input[p] == b'-';
            if p < self.input.len() && (self.input[p] == b'-' || self.input[p] == b'+') {
                p += 1;
            }
            let digits_start = p;
            let mut exponent = 0i32;
            while p < self.input.len() && self.input[p].is_ascii_digit() {
                exponent = exponent.saturating_mul(10).saturating_add((self.input[p] - b'0') as i32);
                p += 1;
            }
            if p > digits_start {
                result *= 10f32.powi(if exponent_negative { -exponent } else { exponent });
                self.pos = p;
            }
        }

        if negative {
            result = -result;
        }

        Ok(result)
    }
    
    fn parse_comment(&mut self) -> String {
        let start = self.pos;
        
        if self.input[self.pos] == b'(' {
            // Find closing parenthesis
            self.pos += 1;
            while self.pos < self.input.len() && self.input[self.pos] != b')' {
                self.pos += 1;
            }
            if self.pos < self.input.len() {
                self.pos += 1; // Skip closing parenthesis
            }
        } else {
            // Semicolon comment - rest of line
            self.pos = self.input.len();
        }
        
        String::from_utf8_lossy(&self.input[start..self.pos]).into_owned()
    }
    
    fn skip_whitespace(&mut self) {
        while self.pos < self.input.len() && self.input[self.pos].is_ascii_whitespace() {
            self.pos += 1;
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum ParseError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    
    #[error("Invalid number format")]
    InvalidNumber,
    
    #[error("Unexpected end of input")]
    UnexpectedEnd,
    
    #[error("Invalid command format")]
    InvalidCommand,
}

// Optimized geometry generation for 3D rendering
#[derive(Debug, Clone)]
pub struct ToolpathGeometry {
    pub vertices: Vec<[f32; 3]>,      // 3D positions
    pub colors: Vec<[f32; 4]>,        // RGBA colors  
    pub indices: Vec<u32>,            // Triangle indices for GPU rendering
    pub line_segments: Vec<LineSegment>, // For line rendering
    pub bounds: BoundingBox,
}

#[derive(Debug, Clone)]
pub struct LineSegment {
    pub start: [f32; 3],
    pub end: [f32; 3],
    pub color: [f32; 4],
    pub line_type: LineType,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LineType {
    Travel,      // Non-extruding moves
    Extrude,     // Extruding moves
    Retract,     // Retraction moves
    ZMove,       // Z-only moves
}

#[derive(Debug, Clone)]
pub struct BoundingBox {
    pub min: [f32; 3],
    pub max: [f32; 3],
}

impl BoundingBox {
    fn new() -> Self {
        Self {
            min: [f32::INFINITY; 3],
            max: [f32::NEG_INFINITY; 3],
        }
    }
    
    fn expand(&mut self, point: [f32; 3]) {
        for i in 0..3 {
            self.min[i] = self.min[i].min(point[i]);
            self.max[i] = self.max[i].max(point[i]);
        }
    }
    
    pub fn center(&self) -> [f32; 3] {
        [
            (self.min[0] + self.max[0]) * 0.5,
            (self.min[1] + self.max[1]) * 0.5,
            (self.min[2] + self.max[2]) * 0.5,
        ]
    }
    
    pub fn size(&self) -> [f32; 3] {
        [
            self.max[0] - self.min[0],
            self.max[1] - self.min[1],
            self.max[2] - self.min[2],
        ]
    }
}

/// High-performance geometry generator for 3D visualization
pub struct GeometryGenerator {
    current_position: [f32; 3],
    current_extrude_pos: f32,
    previous_extrude_pos: f32,
    layer_height: f32,
    extrude_width: f32,
    relative_xyz: bool,    // G91 / G90
    relative_e: bool,      // M83 / M82, and G91 / G90 which set it too
}

impl GeometryGenerator {
    pub fn new() -> Self {
        Self {
            current_position: [0.0, 0.0, 0.0],
            current_extrude_pos: 0.0,
            previous_extrude_pos: 0.0,
            layer_height: 0.2,
            extrude_width: 0.4,
            relative_xyz: false,
            relative_e: false,
        }
    }
    
    /// Generate optimized geometry from parsed g-code commands
    pub fn generate_geometry(&mut self, commands: &[GCodeCommand]) -> ToolpathGeometry {
        let mut geometry = ToolpathGeometry {
            vertices: Vec::with_capacity(commands.len() * 2), // Pre-allocate
            colors: Vec::with_capacity(commands.len() * 2),
            indices: Vec::with_capacity(commands.len() * 6),
            line_segments: Vec::with_capacity(commands.len()),
            bounds: BoundingBox::new(),
        };
        
        // Process commands in batches for better cache performance
        const BATCH_SIZE: usize = 1000;
        for batch in commands.chunks(BATCH_SIZE) {
            self.process_command_batch(batch, &mut geometry);
        }
        
        geometry
    }
    
    fn process_command_batch(&mut self, commands: &[GCodeCommand], geometry: &mut ToolpathGeometry) {
        for command in commands {
            match &command.command_type {
                CommandType::G(0) => {
                    // G0 - Rapid move (travel)
                    self.handle_move(command, LineType::Travel, geometry);
                }
                CommandType::G(1) => {
                    // G1 - Linear move
                    let line_type = if self.is_extruding(&command.parameters) {
                        LineType::Extrude
                    } else {
                        LineType::Travel
                    };
                    self.handle_move(command, line_type, geometry);
                }
                CommandType::G(2) | CommandType::G(3) => {
                    // G2/G3 - Arc moves
                    self.handle_arc(command, geometry);
                }
                CommandType::G(90) => {
                    self.relative_xyz = false;
                    self.relative_e = false;
                }
                CommandType::G(91) => {
                    self.relative_xyz = true;
                    self.relative_e = true;
                }
                CommandType::M(82) => self.relative_e = false,
                CommandType::M(83) => self.relative_e = true,
                CommandType::G(92) => {
                    // Set position, without moving
                    let p = &command.parameters;
                    self.current_position = [
                        p.x.unwrap_or(self.current_position[0]),
                        p.y.unwrap_or(self.current_position[1]),
                        p.z.unwrap_or(self.current_position[2]),
                    ];
                    if let Some(e) = p.e {
                        self.previous_extrude_pos = self.current_extrude_pos;
                        self.current_extrude_pos = e;
                    }
                }
                _ => {
                    // Nothing else moves the toolhead. In particular the E in M92, M201 or
                    // M203 is a setting, not an extruder position.
                }
            }
        }
    }
    
    fn handle_move(&mut self, command: &GCodeCommand, line_type: LineType, geometry: &mut ToolpathGeometry) {
        let new_position = self.calculate_new_position(&command.parameters);
        
        // Only add geometry if there's actual movement
        if self.has_significant_movement(new_position) {
            let color = self.get_line_color(&line_type, &command.parameters);
            
            let segment = LineSegment {
                start: self.current_position,
                end: new_position,
                color,
                line_type,
            };
            
            self.add_line_segment(segment, geometry);
        }
        
        self.update_position(&command.parameters);
        self.current_position = new_position;
    }
    
    fn handle_arc(&mut self, command: &GCodeCommand, geometry: &mut ToolpathGeometry) {
        // Generate arc as series of line segments
        let clockwise = matches!(command.command_type, CommandType::G(2));
        let segments = self.generate_arc_segments(&command.parameters, clockwise);
        
        let line_type = if self.is_extruding(&command.parameters) {
            LineType::Extrude
        } else {
            LineType::Travel
        };
        
        for (start, end) in segments {
            let color = self.get_line_color(&line_type, &command.parameters);
            
            let segment = LineSegment {
                start,
                end,
                color,
                line_type: line_type.clone(),
            };
            
            self.add_line_segment(segment, geometry);
            self.current_position = end;
        }
        
        self.update_position(&command.parameters);
    }
    
    fn add_line_segment(&self, segment: LineSegment, geometry: &mut ToolpathGeometry) {
        let start_idx = geometry.vertices.len() as u32;
        
        // Add vertices
        geometry.vertices.push(segment.start);
        geometry.vertices.push(segment.end);
        
        // Add colors
        geometry.colors.push(segment.color);
        geometry.colors.push(segment.color);
        
        // Add line indices (for line rendering)
        geometry.indices.push(start_idx);
        geometry.indices.push(start_idx + 1);
        
        // Update bounds
        geometry.bounds.expand(segment.start);
        geometry.bounds.expand(segment.end);
        
        // Store line segment for detailed rendering
        geometry.line_segments.push(segment);
    }
    
    fn calculate_new_position(&self, params: &Parameters) -> [f32; 3] {
        let current = self.current_position;
        if self.relative_xyz {
            [
                current[0] + params.x.unwrap_or(0.0),
                current[1] + params.y.unwrap_or(0.0),
                current[2] + params.z.unwrap_or(0.0),
            ]
        } else {
            [
                params.x.unwrap_or(current[0]),
                params.y.unwrap_or(current[1]),
                params.z.unwrap_or(current[2]),
            ]
        }
    }

    fn update_position(&mut self, params: &Parameters) {
        if let Some(e) = params.e {
            self.previous_extrude_pos = self.current_extrude_pos;
            self.current_extrude_pos = if self.relative_e { self.current_extrude_pos + e } else { e };
        }
    }

    fn is_extruding(&self, params: &Parameters) -> bool {
        match params.e {
            Some(e) if self.relative_e => e > 0.0,
            Some(e) => e > self.current_extrude_pos,
            None => false,
        }
    }
    
    fn has_significant_movement(&self, new_pos: [f32; 3]) -> bool {
        const MIN_MOVEMENT: f32 = 0.001; // 1 micron
        
        let dx = new_pos[0] - self.current_position[0];
        let dy = new_pos[1] - self.current_position[1];  
        let dz = new_pos[2] - self.current_position[2];
        
        (dx * dx + dy * dy + dz * dz).sqrt() > MIN_MOVEMENT
    }
    
    fn get_line_color(&self, line_type: &LineType, params: &Parameters) -> [f32; 4] {
        match line_type {
            LineType::Travel => [0.0, 1.0, 0.0, 0.6], // Semi-transparent green
            LineType::Extrude => {
                // Color by layer height or speed
                if let Some(z) = params.z {
                    let height_ratio = (z / 10.0).clamp(0.0, 1.0);
                    [1.0 - height_ratio, 0.5, height_ratio, 1.0]
                } else {
                    [1.0, 0.5, 0.0, 1.0] // Orange
                }
            },
            LineType::Retract => [1.0, 0.0, 0.0, 0.8], // Red
            LineType::ZMove => [0.0, 0.0, 1.0, 0.4],   // Blue
        }
    }
    
    fn generate_arc_segments(&self, params: &Parameters, clockwise: bool) -> Vec<([f32; 3], [f32; 3])> {
        use std::f32::consts::TAU;
        let start = self.current_position;
        let end = self.calculate_new_position(params);
        let straight = vec![(start, end)];

        let center = match (params.i, params.j, params.r) {
            (i, j, _) if i.is_some() || j.is_some() => {
                [start[0] + i.unwrap_or(0.0), start[1] + j.unwrap_or(0.0)]
            }
            (_, _, Some(r)) => {
                // Radius form: the center lies on the perpendicular bisector of the chord.
                let (dx, dy) = (end[0] - start[0], end[1] - start[1]);
                let chord = dx.hypot(dy);
                if chord < 1e-6 || r.abs() < chord / 2.0 {
                    return straight;
                }
                let height = (r * r - chord * chord / 4.0).sqrt();
                // Negative R picks the long way round, which swaps the side for the same direction
                let side = if clockwise == (r > 0.0) { 1.0 } else { -1.0 };
                [
                    (start[0] + end[0]) / 2.0 + side * height * dy / chord,
                    (start[1] + end[1]) / 2.0 - side * height * dx / chord,
                ]
            }
            _ => return straight,
        };

        let radius = (start[0] - center[0]).hypot(start[1] - center[1]);
        if radius < 1e-6 {
            return straight;
        }
        let start_angle = (start[1] - center[1]).atan2(start[0] - center[0]);
        let end_angle = (end[1] - center[1]).atan2(end[0] - center[0]);
        let same_point = (start[0] - end[0]).hypot(start[1] - end[1]) < 1e-5;
        let ccw_sweep = if same_point { TAU } else { (end_angle - start_angle).rem_euclid(TAU) };
        let sweep = if clockwise { ccw_sweep - TAU } else { ccw_sweep };

        // About one segment per 5 degrees
        let segments = ((sweep.abs() / 5f32.to_radians()).ceil() as usize).max(1);
        let mut result = Vec::with_capacity(segments);
        let mut previous = start;
        for n in 1..=segments {
            let t = n as f32 / segments as f32;
            let angle = start_angle + sweep * t;
            let point = if n == segments {
                end
            } else {
                [
                    center[0] + radius * angle.cos(),
                    center[1] + radius * angle.sin(),
                    start[2] + (end[2] - start[2]) * t,
                ]
            };
            result.push((previous, point));
            previous = point;
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;
    
    #[test]
    fn test_fast_number_parsing() {
        let mut parser = LineParserState::new(b"123.456");
        assert_eq!(parser.parse_number_f32().unwrap(), 123.456);
        
        parser = LineParserState::new(b"-42");
        parser.pos = 0;
        assert_eq!(parser.parse_number_f32().unwrap(), -42.0);
    }
    
    #[test]
    fn test_gcode_parsing() {
        let gcode = "G1 X10.5 Y20.3 Z0.2 F1500\nG0 X0 Y0\n; comment line\nM104 S200";
        let cursor = Cursor::new(gcode);
        
        let mut parser = StreamingGCodeParser::new();
        let commands = parser.parse_file(cursor).unwrap();
        
        assert_eq!(commands.len(), 3);
        assert!(matches!(commands[0].command_type, CommandType::G(1)));
        assert!(matches!(commands[1].command_type, CommandType::G(0)));
        assert!(matches!(commands[2].command_type, CommandType::M(104)));
    }
    
    #[test]
    fn test_geometry_generation() {
        let commands = vec![
            GCodeCommand {
                command_type: CommandType::G(1),
                line_number: 1,
                parameters: Parameters {
                    x: Some(10.0),
                    y: Some(10.0),
                    z: Some(0.2),
                    e: Some(1.0),
                    ..Default::default()
                },
                comment: None,
            }
        ];
        
        let mut generator = GeometryGenerator::new();
        let geometry = generator.generate_geometry(&commands);
        
        assert!(!geometry.line_segments.is_empty());
        assert_eq!(geometry.vertices.len(), geometry.colors.len());
    }

    fn parse(text: &str) -> Vec<GCodeCommand> {
        StreamingGCodeParser::new().parse_file(Cursor::new(text)).unwrap()
    }

    #[test]
    fn test_valid_gcode_does_not_fail_the_file() {
        for line in ["G28 X", "G28 X Y", "M84 X Y", "M117 Hello World", "EXCLUDE_OBJECT_DEFINE NAME=a", "G1 X", "M", "G1 X1O"] {
            let commands = parse(&format!("G1 X1\n{}\nG1 X2\n", line));
            assert_eq!(commands.len(), 3, "{}", line);
        }
        let commands = parse("G28 X Y\nM117 Hello\nSTART_PRINT BED=60 ; go\n");
        assert_eq!(commands[0].parameters.flags, vec!['X', 'Y']);
        assert_eq!(commands[1].parameters.text.as_deref(), Some("Hello"));
        assert_eq!(commands[2].command_type, CommandType::Other("START_PRINT".to_string()));
        assert_eq!(commands[2].parameters.text.as_deref(), Some("BED=60"));
    }

    #[test]
    fn test_large_m_codes_stay_distinct() {
        let commands = parse("M300 S1\nM400\nM600\nM900 K0.05\n");
        let types: Vec<_> = commands.iter().map(|c| c.command_type.clone()).collect();
        assert_eq!(types, vec![CommandType::M(300), CommandType::M(400), CommandType::M(600), CommandType::M(900)]);
    }

    #[test]
    fn test_scientific_notation_vs_extruder_word() {
        let commands = parse("G1 X1e-2\nG1 X1E5\n");
        assert!((commands[0].parameters.x.unwrap() - 0.01).abs() < 1e-6);
        assert_eq!(commands[0].parameters.e, None);
        assert_eq!(commands[1].parameters.x, Some(1.0));
        assert_eq!(commands[1].parameters.e, Some(5.0));
    }

    #[test]
    fn test_parallel_parse_keeps_file_order() {
        let mut text = String::new();
        for i in 0..200_000u32 {
            text.push_str(&format!("G1 X{}\n", i));
        }
        let mut parser = StreamingGCodeParser::new().with_parallel_chunks(8);
        let commands = parser.parse_file(Cursor::new(text)).unwrap();
        assert_eq!(commands.len(), 200_000);
        assert_eq!(parser.get_statistics().total_lines, 200_000);
        for (i, command) in commands.iter().enumerate() {
            assert_eq!(command.line_number, i as u32);
        }
        assert!(commands.windows(2).all(|w| w[0].parameters.x < w[1].parameters.x));
    }

    fn line_types(text: &str) -> Vec<LineType> {
        GeometryGenerator::new().generate_geometry(&parse(text)).line_segments.iter().map(|s| s.line_type).collect()
    }

    #[test]
    fn test_extrusion_classification() {
        // E in a settings command is not an extruder position
        assert_eq!(line_types("M201 X500 E5000\nG1 X10 E1\n"), vec![LineType::Extrude]);
        // Relative extrusion
        assert_eq!(line_types("M83\nG1 X10 E0.5\nG1 X20 E0.5\n"), vec![LineType::Extrude, LineType::Extrude]);
        // G92 resets the counter
        assert_eq!(line_types("G1 X10 E5\nG92 E0\nG1 X20 E1\n"), vec![LineType::Extrude, LineType::Extrude]);
    }

    #[test]
    fn test_arcs_follow_the_circle() {
        // Quarter circle of radius 10 around the origin, counter-clockwise
        let geometry = GeometryGenerator::new().generate_geometry(&parse("G1 X10 Y0\nG3 X0 Y10 I-10 J0\n"));
        let arc = &geometry.line_segments[1..];
        assert!(arc.len() > 5);
        for segment in arc {
            assert!((segment.end[0].hypot(segment.end[1]) - 10.0).abs() < 0.01, "{:?}", segment.end);
            assert!(segment.end[0] >= -0.01 && segment.end[1] >= -0.01, "wrong way round: {:?}", segment.end);
        }
        // Same end points, clockwise, goes the long way through the other three quadrants
        let geometry = GeometryGenerator::new().generate_geometry(&parse("G1 X10 Y0\nG2 X0 Y10 I-10 J0\n"));
        assert!(geometry.line_segments.iter().any(|s| s.end[0] < -5.0 && s.end[1] < -5.0));
        // Radius form, same quarter circle
        let geometry = GeometryGenerator::new().generate_geometry(&parse("G1 X10 Y0\nG3 X0 Y10 R10\n"));
        for segment in &geometry.line_segments[1..] {
            assert!((segment.end[0].hypot(segment.end[1]) - 10.0).abs() < 0.01, "{:?}", segment.end);
        }
    }
}