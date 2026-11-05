pub struct GCodeFile {
    pub commands: Vec<String>,
}

impl GCodeFile {
    /// Split G-code into commands: comments and blank lines removed, whitespace trimmed.
    pub fn parse(input: &str) -> Option<Self> {
        let commands = input
            .lines()
            .map(|line| line.split(';').next().unwrap_or("").trim())
            .filter(|line| !line.is_empty())
            .map(str::to_string)
            .collect();
        Some(GCodeFile { commands })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keeps_commands_and_drops_comments() {
        let parsed = GCodeFile::parse("; header\nG1 X1 ; move\n\n  M104 S200\r\n").unwrap();
        assert_eq!(parsed.commands, vec!["G1 X1", "M104 S200"]);
    }
}
