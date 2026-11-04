pub struct GCodeFile {
    pub commands: Vec<String>,
}

impl GCodeFile {
    pub fn parse(_input: &str) -> Option<Self> {
        Some(GCodeFile { commands: vec![] })
    }
}