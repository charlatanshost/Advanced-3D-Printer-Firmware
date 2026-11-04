/// Plugin System Architecture
///
/// This module provides a comprehensive plugin system that allows dynamic loading
/// of plugins for extending printer functionality. Supports both native Rust plugins
/// and scripted plugins via Lua.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, Instant};
use serde::{Deserialize, Serialize};
use serde_json::Value as JsonValue;

// Optional plugin runtime dependencies
#[cfg(feature = "plugins")]
pub use libloading::{Library, Symbol};

// Lua scripting support (optional)
#[cfg(feature = "plugins")]
use mlua;

// WebAssembly runtime support (optional)
#[cfg(feature = "plugins")]
use wasmtime;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PluginManifest {
    pub name: String,
    pub version: String,
    pub description: String,
    pub author: String,
    pub license: String,
    pub plugin_type: PluginType,
    pub entry_point: String,          // Main file/function
    pub dependencies: Vec<String>,    // Other plugins this depends on
    pub permissions: Vec<Permission>, // What the plugin can access
    pub min_api_version: String,      // Minimum API version required
    pub max_api_version: Option<String>,
    pub metadata: HashMap<String, JsonValue>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PluginType {
    Native,        // Compiled Rust plugin (.dll/.so/.dylib)
    Script,        // Lua script
    WebAssembly,   // WASM module
    Python,        // Python script (future)
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Permission {
    ReadConfig,
    WriteConfig,
    AccessMotion,
    AccessTemperature,
    AccessProbes,
    AccessFileSystem,
    AccessNetwork,
    AccessSerial,
    AccessCAN,
    AccessGCode,
    AccessUI,
    SystemControl,
}

#[derive(Debug, Clone)]
pub struct PluginInfo {
    pub manifest: PluginManifest,
    pub plugin_path: PathBuf,
    pub load_time: Instant,
    pub last_error: Option<String>,
    pub is_enabled: bool,
    pub execution_stats: PluginStats,
}

#[derive(Debug, Clone, Default)]
pub struct PluginStats {
    pub calls_made: u64,
    pub total_execution_time: Duration,
    pub last_execution_time: Duration,
    pub errors_count: u64,
    pub memory_usage: usize,
}

pub struct PluginManager {
    plugins: RwLock<HashMap<String, Arc<Mutex<LoadedPlugin>>>>,
    plugin_info: RwLock<HashMap<String, PluginInfo>>,
    plugin_directories: Vec<PathBuf>,
    api_registry: Arc<PluginApiRegistry>,
    event_bus: Arc<PluginEventBus>,
    security_manager: PluginSecurityManager,
}

pub enum LoadedPlugin {
    Native(NativePlugin),
    Script(ScriptPlugin),
    WebAssembly(WasmPlugin),
}

#[cfg(feature = "plugins")]
pub struct NativePlugin {
    library: libloading::Library,
    interface: Box<dyn PluginInterface + Send + Sync>,
}

#[cfg(not(feature = "plugins"))]
pub struct NativePlugin {
    _placeholder: (),
}

#[cfg(feature = "plugins")]
pub struct ScriptPlugin {
    source_code: String,
    lua_context: Option<mlua::Lua>,
}

#[cfg(not(feature = "plugins"))]
pub struct ScriptPlugin {
    _placeholder: (),
}

#[cfg(feature = "plugins")]
pub struct WasmPlugin {
    module: Vec<u8>, // WASM bytecode
    instance: Option<wasmtime::Instance>,
}

#[cfg(not(feature = "plugins"))]
pub struct WasmPlugin {
    _placeholder: (),
}

/// Core plugin interface that all plugins must implement
pub trait PluginInterface {
    fn initialize(&mut self, api: &PluginApiRegistry) -> Result<(), PluginError>;
    fn get_manifest(&self) -> &PluginManifest;
    fn execute_command(&mut self, command: &str, args: &[String]) -> Result<JsonValue, PluginError>;
    fn handle_event(&mut self, event: &PluginEvent) -> Result<(), PluginError>;
    fn shutdown(&mut self) -> Result<(), PluginError>;
    fn get_stats(&self) -> PluginStats;
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PluginEvent {
    PrintStarted { filename: String },
    PrintFinished { success: bool, duration: Duration },
    PrintPaused,
    PrintResumed,
    LayerChanged { layer: u32 },
    ToolChanged { from_tool: u8, to_tool: u8 },
    TemperatureChanged { zone: String, temperature: f32 },
    PositionChanged { x: f32, y: f32, z: f32 },
    ErrorOccurred { error_code: u32, message: String },
    ConfigChanged { section: String },
    CustomEvent { name: String, data: JsonValue },
}

pub struct PluginApiRegistry {
    motion_api: Arc<dyn MotionApi + Send + Sync>,
    temperature_api: Arc<dyn TemperatureApi + Send + Sync>,
    gcode_api: Arc<dyn GCodeApi + Send + Sync>,
    ui_api: Arc<dyn UIApi + Send + Sync>,
    config_api: Arc<dyn ConfigApi + Send + Sync>,
    file_api: Arc<dyn FileApi + Send + Sync>,
}

/// Motion control API for plugins
pub trait MotionApi {
    fn move_to(&self, x: f32, y: f32, z: f32, feedrate: f32) -> Result<(), PluginError>;
    fn home_axes(&self, axes: &str) -> Result<(), PluginError>;
    fn get_position(&self) -> Result<(f32, f32, f32), PluginError>;
    fn set_acceleration(&self, axis: char, accel: f32) -> Result<(), PluginError>;
    fn enable_motors(&self, enable: bool) -> Result<(), PluginError>;
    fn probe_point(&self, x: f32, y: f32) -> Result<f32, PluginError>;
}

/// Temperature control API for plugins
pub trait TemperatureApi {
    fn set_temperature(&self, heater: &str, temp: f32) -> Result<(), PluginError>;
    fn get_temperature(&self, heater: &str) -> Result<f32, PluginError>;
    fn set_fan_speed(&self, fan: u8, speed: f32) -> Result<(), PluginError>;
    fn wait_for_temperature(&self, heater: &str, temp: f32, timeout: Duration) -> Result<(), PluginError>;
}

/// G-code processing API for plugins
pub trait GCodeApi {
    fn execute_gcode(&self, gcode: &str) -> Result<String, PluginError>;
    fn queue_gcode(&self, gcode: &str) -> Result<(), PluginError>;
    fn register_gcode_handler(&self, gcode: &str, handler: Box<dyn Fn(&str) -> Result<String, PluginError>>) -> Result<(), PluginError>;
    fn get_print_progress(&self) -> Result<f32, PluginError>;
}

/// User interface API for plugins
pub trait UIApi {
    fn add_menu_item(&self, menu: &str, item: &str, callback: Box<dyn Fn()>) -> Result<(), PluginError>;
    fn show_notification(&self, message: &str, level: NotificationLevel) -> Result<(), PluginError>;
    fn create_custom_panel(&self, name: &str, content: UIElement) -> Result<(), PluginError>;
    fn update_status_bar(&self, message: &str) -> Result<(), PluginError>;
}

#[derive(Debug, Clone)]
pub enum NotificationLevel {
    Info,
    Warning,
    Error,
    Success,
}

#[derive(Debug, Clone)]
pub enum UIElement {
    Button { text: String, callback: String },
    Slider { min: f32, max: f32, value: f32, callback: String },
    TextInput { placeholder: String, callback: String },
    Label { text: String },
    Group { title: String, children: Vec<UIElement> },
}

/// Configuration API for plugins
pub trait ConfigApi {
    fn get_config_value(&self, key: &str) -> Result<JsonValue, PluginError>;
    fn set_config_value(&self, key: &str, value: &JsonValue) -> Result<(), PluginError>;
    fn save_config(&self) -> Result<(), PluginError>;
    fn create_config_section(&self, section: &str) -> Result<(), PluginError>;
}

/// File system API for plugins
pub trait FileApi {
    fn read_file(&self, path: &str) -> Result<String, PluginError>;
    fn write_file(&self, path: &str, content: &str) -> Result<(), PluginError>;
    fn list_files(&self, directory: &str) -> Result<Vec<String>, PluginError>;
    fn delete_file(&self, path: &str) -> Result<(), PluginError>;
    fn file_exists(&self, path: &str) -> Result<bool, PluginError>;
}

pub struct PluginEventBus {
    subscribers: RwLock<HashMap<String, Vec<String>>>, // event_type -> plugin_names
}

impl PluginEventBus {
    pub fn new() -> Self {
        Self {
            subscribers: RwLock::new(HashMap::new()),
        }
    }

    pub fn subscribe(&self, plugin_name: &str, event_type: &str) {
        let mut subscribers = self.subscribers.write().unwrap();
        subscribers.entry(event_type.to_string())
                  .or_insert_with(Vec::new)
                  .push(plugin_name.to_string());
    }

    pub fn unsubscribe(&self, plugin_name: &str, event_type: &str) {
        let mut subscribers = self.subscribers.write().unwrap();
        if let Some(plugin_list) = subscribers.get_mut(event_type) {
            plugin_list.retain(|name| name != plugin_name);
        }
    }

    pub fn publish(&self, event: &PluginEvent) -> Vec<String> {
        let event_type = self.get_event_type(event);
        let subscribers = self.subscribers.read().unwrap();

        subscribers.get(&event_type)
                  .map(|plugins| plugins.clone())
                  .unwrap_or_default()
    }

    fn get_event_type(&self, event: &PluginEvent) -> String {
        match event {
            PluginEvent::PrintStarted { .. } => "print_started".to_string(),
            PluginEvent::PrintFinished { .. } => "print_finished".to_string(),
            PluginEvent::PrintPaused => "print_paused".to_string(),
            PluginEvent::PrintResumed => "print_resumed".to_string(),
            PluginEvent::LayerChanged { .. } => "layer_changed".to_string(),
            PluginEvent::ToolChanged { .. } => "tool_changed".to_string(),
            PluginEvent::TemperatureChanged { .. } => "temperature_changed".to_string(),
            PluginEvent::PositionChanged { .. } => "position_changed".to_string(),
            PluginEvent::ErrorOccurred { .. } => "error_occurred".to_string(),
            PluginEvent::ConfigChanged { .. } => "config_changed".to_string(),
            PluginEvent::CustomEvent { name, .. } => name.clone(),
        }
    }
}

pub struct PluginSecurityManager {
    allowed_paths: Vec<PathBuf>,
    banned_plugins: Vec<String>,
    permission_cache: RwLock<HashMap<String, Vec<Permission>>>,
}

impl PluginSecurityManager {
    pub fn new() -> Self {
        Self {
            allowed_paths: vec![
                PathBuf::from("plugins/"),
                PathBuf::from("scripts/"),
                PathBuf::from("data/plugins/"),
            ],
            banned_plugins: Vec::new(),
            permission_cache: RwLock::new(HashMap::new()),
        }
    }

    pub fn validate_plugin_path(&self, path: &Path) -> Result<(), PluginError> {
        let canonical_path = path.canonicalize()
            .map_err(|_| PluginError::SecurityViolation("Invalid plugin path".to_string()))?;

        for allowed_path in &self.allowed_paths {
            if canonical_path.starts_with(allowed_path) {
                return Ok(());
            }
        }

        Err(PluginError::SecurityViolation("Plugin path not in allowed directories".to_string()))
    }

    pub fn check_permission(&self, plugin_name: &str, permission: &Permission) -> bool {
        let cache = self.permission_cache.read().unwrap();
        if let Some(permissions) = cache.get(plugin_name) {
            permissions.contains(permission)
        } else {
            false
        }
    }

    pub fn grant_permissions(&self, plugin_name: &str, permissions: Vec<Permission>) {
        let mut cache = self.permission_cache.write().unwrap();
        cache.insert(plugin_name.to_string(), permissions);
    }

    pub fn is_plugin_banned(&self, plugin_name: &str) -> bool {
        self.banned_plugins.contains(&plugin_name.to_string())
    }
}

impl PluginManager {
    pub fn new() -> Self {
        Self {
            plugins: RwLock::new(HashMap::new()),
            plugin_info: RwLock::new(HashMap::new()),
            plugin_directories: vec![
                PathBuf::from("plugins/native/"),
                PathBuf::from("plugins/scripts/"),
                PathBuf::from("plugins/wasm/"),
            ],
            api_registry: Arc::new(PluginApiRegistry::new()),
            event_bus: Arc::new(PluginEventBus::new()),
            security_manager: PluginSecurityManager::new(),
        }
    }

    /// Scan plugin directories and load all valid plugins
    pub fn scan_and_load_plugins(&self) -> Result<Vec<String>, PluginError> {
        let mut loaded_plugins = Vec::new();

        for plugin_dir in &self.plugin_directories {
            if !plugin_dir.exists() {
                std::fs::create_dir_all(plugin_dir)
                    .map_err(|e| PluginError::IoError(e.to_string()))?;
                continue;
            }

            for entry in std::fs::read_dir(plugin_dir)
                .map_err(|e| PluginError::IoError(e.to_string()))?
            {
                let entry = entry.map_err(|e| PluginError::IoError(e.to_string()))?;
                let path = entry.path();

                if path.is_dir() {
                    // Look for manifest file
                    let manifest_path = path.join("plugin.toml");
                    if manifest_path.exists() {
                        match self.load_plugin_from_directory(&path) {
                            Ok(plugin_name) => loaded_plugins.push(plugin_name),
                            Err(e) => eprintln!("Failed to load plugin from {:?}: {}", path, e),
                        }
                    }
                }
            }
        }

        println!("🔌 Loaded {} plugins", loaded_plugins.len());
        Ok(loaded_plugins)
    }

    fn load_plugin_from_directory(&self, plugin_dir: &Path) -> Result<String, PluginError> {
        self.security_manager.validate_plugin_path(plugin_dir)?;

        // Read manifest
        let manifest_path = plugin_dir.join("plugin.toml");
        let manifest_content = std::fs::read_to_string(&manifest_path)
            .map_err(|e| PluginError::IoError(e.to_string()))?;

        let manifest: PluginManifest = toml::from_str(&manifest_content)
            .map_err(|e| PluginError::ManifestError(e.to_string()))?;

        // Check if plugin is banned
        if self.security_manager.is_plugin_banned(&manifest.name) {
            return Err(PluginError::SecurityViolation("Plugin is banned".to_string()));
        }

        // Load plugin based on type
        let loaded_plugin = match manifest.plugin_type {
            PluginType::Native => self.load_native_plugin(plugin_dir, &manifest)?,
            PluginType::Script => self.load_script_plugin(plugin_dir, &manifest)?,
            PluginType::WebAssembly => self.load_wasm_plugin(plugin_dir, &manifest)?,
            PluginType::Python => return Err(PluginError::UnsupportedType("Python plugins not yet implemented".to_string())),
        };

        // Grant permissions
        self.security_manager.grant_permissions(&manifest.name, manifest.permissions.clone());

        // Store plugin info
        let plugin_info = PluginInfo {
            manifest: manifest.clone(),
            plugin_path: plugin_dir.to_path_buf(),
            load_time: Instant::now(),
            last_error: None,
            is_enabled: true,
            execution_stats: PluginStats::default(),
        };

        let mut plugins = self.plugins.write().unwrap();
        let mut plugin_info_map = self.plugin_info.write().unwrap();

        plugins.insert(manifest.name.clone(), Arc::new(Mutex::new(loaded_plugin)));
        plugin_info_map.insert(manifest.name.clone(), plugin_info);

        println!("✓ Loaded plugin: {} v{}", manifest.name, manifest.version);

        Ok(manifest.name)
    }

    #[cfg(feature = "plugins")]
    fn load_native_plugin(&self, plugin_dir: &Path, manifest: &PluginManifest) -> Result<LoadedPlugin, PluginError> {
        // Determine library file extension based on platform
        let lib_extension = if cfg!(target_os = "windows") {
            "dll"
        } else if cfg!(target_os = "macos") {
            "dylib"
        } else {
            "so"
        };

        let lib_path = plugin_dir.join(format!("{}.{}", manifest.entry_point, lib_extension));

        if !lib_path.exists() {
            return Err(PluginError::NotFound(format!("Plugin library not found: {:?}", lib_path)));
        }

        // Load library
        let library = unsafe {
            Library::new(&lib_path)
                .map_err(|e| PluginError::LoadError(e.to_string()))?
        };

        // Get plugin interface factory function
        let create_plugin: Symbol<unsafe extern "C" fn() -> *mut dyn PluginInterface> = unsafe {
            library.get(b"create_plugin")
                .map_err(|e| PluginError::LoadError(format!("Missing create_plugin function: {}", e)))?
        };

        // Create plugin instance
        let plugin_ptr = unsafe { create_plugin() };
        let plugin_interface = unsafe { Box::from_raw(plugin_ptr) };

        Ok(LoadedPlugin::Native(NativePlugin {
            library,
            interface: plugin_interface,
        }))
    }

    #[cfg(feature = "plugins")]
    fn load_script_plugin(&self, plugin_dir: &Path, manifest: &PluginManifest) -> Result<LoadedPlugin, PluginError> {
        let script_path = plugin_dir.join(&manifest.entry_point);

        if !script_path.exists() {
            return Err(PluginError::NotFound(format!("Script file not found: {:?}", script_path)));
        }

        let source_code = std::fs::read_to_string(&script_path)
            .map_err(|e| PluginError::IoError(e.to_string()))?;

        // Initialize Lua context
        let lua = mlua::Lua::new();

        // Set up plugin API bindings
        self.setup_lua_api(&lua)?;

        // Load and execute the script
        lua.load(&source_code)
           .exec()
           .map_err(|e| PluginError::ScriptError(e.to_string()))?;

        Ok(LoadedPlugin::Script(ScriptPlugin {
            source_code,
            lua_context: Some(lua),
        }))
    }

    #[cfg(feature = "plugins")]
    fn load_wasm_plugin(&self, plugin_dir: &Path, manifest: &PluginManifest) -> Result<LoadedPlugin, PluginError> {
        let wasm_path = plugin_dir.join(&manifest.entry_point);

        if !wasm_path.exists() {
            return Err(PluginError::NotFound(format!("WASM file not found: {:?}", wasm_path)));
        }

        let wasm_bytes = std::fs::read(&wasm_path)
            .map_err(|e| PluginError::IoError(e.to_string()))?;

        // For now, just store the bytes. Full WASM runtime integration would go here.
        Ok(LoadedPlugin::WebAssembly(WasmPlugin {
            module: wasm_bytes,
            instance: None,
        }))
    }

    // Stub implementations for when plugins are disabled
    #[cfg(not(feature = "plugins"))]
    fn load_native_plugin(&self, _plugin_dir: &Path, _manifest: &PluginManifest) -> Result<LoadedPlugin, PluginError> {
        Err(PluginError::LoadError("Plugin support not compiled in".to_string()))
    }

    #[cfg(not(feature = "plugins"))]
    fn load_script_plugin(&self, _plugin_dir: &Path, _manifest: &PluginManifest) -> Result<LoadedPlugin, PluginError> {
        Err(PluginError::LoadError("Plugin support not compiled in".to_string()))
    }

    #[cfg(not(feature = "plugins"))]
    fn load_wasm_plugin(&self, _plugin_dir: &Path, _manifest: &PluginManifest) -> Result<LoadedPlugin, PluginError> {
        Err(PluginError::LoadError("Plugin support not compiled in".to_string()))
    }

    #[cfg(feature = "plugins")]
    fn setup_lua_api(&self, lua: &mlua::Lua) -> Result<(), PluginError> {
        // Create API tables for Lua scripts
        let globals = lua.globals();

        // Motion API
        let motion = lua.create_table()?;
        motion.set("move_to", lua.create_function(|_, (x, y, z, feedrate): (f32, f32, f32, f32)| {
            // Implementation would call actual motion API
            println!("Lua plugin called move_to: ({}, {}, {}) at {}", x, y, z, feedrate);
            Ok(())
        })?)?;

        motion.set("home", lua.create_function(|_, axes: String| {
            println!("Lua plugin called home: {}", axes);
            Ok(())
        })?)?;

        globals.set("motion", motion)?;

        // Temperature API
        let temperature = lua.create_table()?;
        temperature.set("set_temp", lua.create_function(|_, (heater, temp): (String, f32)| {
            println!("Lua plugin set temperature: {} to {}°C", heater, temp);
            Ok(())
        })?)?;

        globals.set("temperature", temperature)?;

        // Utility functions
        let utils = lua.create_table()?;
        utils.set("log", lua.create_function(|_, message: String| {
            println!("Plugin log: {}", message);
            Ok(())
        })?)?;

        globals.set("utils", utils)?;

        Ok(())
    }

    /// Execute a command on a specific plugin
    #[cfg(feature = "plugins")]
    pub fn execute_plugin_command(&self, plugin_name: &str, command: &str, args: &[String]) -> Result<JsonValue, PluginError> {
        let plugins = self.plugins.read().unwrap();
        let plugin = plugins.get(plugin_name)
            .ok_or_else(|| PluginError::NotFound(plugin_name.to_string()))?;

        let mut plugin_guard = plugin.lock().unwrap();

        // Update execution stats
        let start_time = Instant::now();

        let result = match &mut *plugin_guard {
            LoadedPlugin::Native(native) => {
                native.interface.execute_command(command, args)
            },
            LoadedPlugin::Script(script) => {
                self.execute_lua_command(script, command, args)
            },
            LoadedPlugin::WebAssembly(_wasm) => {
                Err(PluginError::UnsupportedType("WASM execution not implemented".to_string()))
            },
        };

        let execution_time = start_time.elapsed();

        // Update plugin stats
        {
            let mut plugin_info = self.plugin_info.write().unwrap();
            if let Some(info) = plugin_info.get_mut(plugin_name) {
                info.execution_stats.calls_made += 1;
                info.execution_stats.total_execution_time += execution_time;
                info.execution_stats.last_execution_time = execution_time;

                if result.is_err() {
                    info.execution_stats.errors_count += 1;
                    info.last_error = Some(format!("{:?}", result));
                }
            }
        }

        result
    }

    /// Stub implementation when plugins are disabled
    #[cfg(not(feature = "plugins"))]
    pub fn execute_plugin_command(&self, _plugin_name: &str, _command: &str, _args: &[String]) -> Result<JsonValue, PluginError> {
        Err(PluginError::LoadError("Plugin support not compiled in".to_string()))
    }

    #[cfg(feature = "plugins")]
    fn execute_lua_command(&self, script: &mut ScriptPlugin, command: &str, args: &[String]) -> Result<JsonValue, PluginError> {
        let lua = script.lua_context.as_ref()
            .ok_or_else(|| PluginError::ScriptError("Lua context not initialized".to_string()))?;

        // Convert args to Lua table
        let lua_args = lua.create_table()
            .map_err(|e| PluginError::ScriptError(e.to_string()))?;

        for (i, arg) in args.iter().enumerate() {
            lua_args.set(i + 1, arg.clone())
                .map_err(|e| PluginError::ScriptError(e.to_string()))?;
        }

        // Call the command function
        let globals = lua.globals();
        let result: mlua::Value = globals.get::<_, mlua::Function>(command)
            .map_err(|e| PluginError::ScriptError(format!("Command '{}' not found: {}", command, e)))?
            .call(lua_args)
            .map_err(|e| PluginError::ScriptError(e.to_string()))?;

        // Convert Lua result to JSON
        self.lua_value_to_json(result)
    }

    #[cfg(feature = "plugins")]
    fn lua_value_to_json(&self, value: mlua::Value) -> Result<JsonValue, PluginError> {
        match value {
            mlua::Value::Nil => Ok(JsonValue::Null),
            mlua::Value::Boolean(b) => Ok(JsonValue::Bool(b)),
            mlua::Value::Number(n) => Ok(JsonValue::Number(serde_json::Number::from_f64(n).unwrap())),
            mlua::Value::String(s) => Ok(JsonValue::String(s.to_str().unwrap().to_string())),
            mlua::Value::Table(t) => {
                let mut map = serde_json::Map::new();
                for pair in t.pairs::<mlua::Value, mlua::Value>() {
                    let (key, value) = pair.map_err(|e| PluginError::ScriptError(e.to_string()))?;
                    let key_str = match key {
                        mlua::Value::String(s) => s.to_str().unwrap().to_string(),
                        mlua::Value::Number(n) => n.to_string(),
                        _ => return Err(PluginError::ScriptError("Invalid table key".to_string())),
                    };
                    map.insert(key_str, self.lua_value_to_json(value)?);
                }
                Ok(JsonValue::Object(map))
            },
            _ => Err(PluginError::ScriptError("Unsupported Lua value type".to_string())),
        }
    }

    /// Broadcast an event to all subscribed plugins
    pub fn broadcast_event(&self, event: &PluginEvent) {
        let subscribers = self.event_bus.publish(event);

        for plugin_name in subscribers {
            if let Err(e) = self.send_event_to_plugin(&plugin_name, event) {
                eprintln!("Failed to send event to plugin {}: {}", plugin_name, e);
            }
        }
    }

    #[cfg(feature = "plugins")]
    fn send_event_to_plugin(&self, plugin_name: &str, event: &PluginEvent) -> Result<(), PluginError> {
        let plugins = self.plugins.read().unwrap();
        let plugin = plugins.get(plugin_name)
            .ok_or_else(|| PluginError::NotFound(plugin_name.to_string()))?;

        let mut plugin_guard = plugin.lock().unwrap();

        match &mut *plugin_guard {
            LoadedPlugin::Native(native) => {
                native.interface.handle_event(event)
            },
            LoadedPlugin::Script(_script) => {
                // Could implement Lua event handling here
                Ok(())
            },
            LoadedPlugin::WebAssembly(_wasm) => {
                Ok(())
            },
        }
    }

    #[cfg(not(feature = "plugins"))]
    fn send_event_to_plugin(&self, _plugin_name: &str, _event: &PluginEvent) -> Result<(), PluginError> {
        Err(PluginError::LoadError("Plugin support not compiled in".to_string()))
    }

    /// Get information about all loaded plugins
    pub fn list_plugins(&self) -> Vec<PluginInfo> {
        let plugin_info = self.plugin_info.read().unwrap();
        plugin_info.values().cloned().collect()
    }

    /// Enable or disable a plugin
    pub fn set_plugin_enabled(&self, plugin_name: &str, enabled: bool) -> Result<(), PluginError> {
        let mut plugin_info = self.plugin_info.write().unwrap();
        let info = plugin_info.get_mut(plugin_name)
            .ok_or_else(|| PluginError::NotFound(plugin_name.to_string()))?;

        info.is_enabled = enabled;
        println!("Plugin {} {}", plugin_name, if enabled { "enabled" } else { "disabled" });

        Ok(())
    }

    /// Unload a plugin
    #[cfg(feature = "plugins")]
    pub fn unload_plugin(&self, plugin_name: &str) -> Result<(), PluginError> {
        let mut plugins = self.plugins.write().unwrap();
        let mut plugin_info = self.plugin_info.write().unwrap();

        // Shutdown plugin if it's loaded
        if let Some(plugin) = plugins.get(plugin_name) {
            let mut plugin_guard = plugin.lock().unwrap();
            if let LoadedPlugin::Native(native) = &mut *plugin_guard {
                let _ = native.interface.shutdown();
            }
        }

        plugins.remove(plugin_name);
        plugin_info.remove(plugin_name);

        println!("Plugin {} unloaded", plugin_name);
        Ok(())
    }

    /// Stub implementation when plugins are disabled
    #[cfg(not(feature = "plugins"))]
    pub fn unload_plugin(&self, _plugin_name: &str) -> Result<(), PluginError> {
        Err(PluginError::LoadError("Plugin support not compiled in".to_string()))
    }
}

// Mock implementations for development/testing
#[derive(Debug)]
struct MockMotionApi;

impl MotionApi for MockMotionApi {
    fn move_to(&self, _x: f32, _y: f32, _z: f32, _feedrate: f32) -> Result<(), PluginError> {
        println!("MockMotionApi: move_to called");
        Ok(())
    }

    fn home_axes(&self, _axes: &str) -> Result<(), PluginError> {
        println!("MockMotionApi: home_axes called");
        Ok(())
    }

    fn get_position(&self) -> Result<(f32, f32, f32), PluginError> {
        Ok((0.0, 0.0, 0.0))
    }

    fn set_acceleration(&self, _axis: char, _accel: f32) -> Result<(), PluginError> {
        println!("MockMotionApi: set_acceleration called");
        Ok(())
    }

    fn enable_motors(&self, _enable: bool) -> Result<(), PluginError> {
        println!("MockMotionApi: enable_motors called");
        Ok(())
    }

    fn probe_point(&self, _x: f32, _y: f32) -> Result<f32, PluginError> {
        Ok(0.0)
    }
}

#[derive(Debug)]
struct MockTemperatureApi;

impl TemperatureApi for MockTemperatureApi {
    fn set_temperature(&self, _heater: &str, _temp: f32) -> Result<(), PluginError> {
        println!("MockTemperatureApi: set_temperature called");
        Ok(())
    }

    fn get_temperature(&self, _heater: &str) -> Result<f32, PluginError> {
        Ok(20.0)
    }

    fn set_fan_speed(&self, _fan: u8, _speed: f32) -> Result<(), PluginError> {
        println!("MockTemperatureApi: set_fan_speed called");
        Ok(())
    }

    fn wait_for_temperature(&self, _heater: &str, _temp: f32, _timeout: Duration) -> Result<(), PluginError> {
        println!("MockTemperatureApi: wait_for_temperature called");
        Ok(())
    }
}

#[derive(Debug)]
struct MockGCodeApi;

impl GCodeApi for MockGCodeApi {
    fn execute_gcode(&self, _gcode: &str) -> Result<String, PluginError> {
        println!("MockGCodeApi: execute_gcode called");
        Ok("ok".to_string())
    }

    fn queue_gcode(&self, _gcode: &str) -> Result<(), PluginError> {
        println!("MockGCodeApi: queue_gcode called");
        Ok(())
    }

    fn register_gcode_handler(&self, _gcode: &str, _handler: Box<dyn Fn(&str) -> Result<String, PluginError>>) -> Result<(), PluginError> {
        println!("MockGCodeApi: register_gcode_handler called");
        Ok(())
    }

    fn get_print_progress(&self) -> Result<f32, PluginError> {
        Ok(0.0)
    }
}

#[derive(Debug)]
struct MockUIApi;

impl UIApi for MockUIApi {
    fn add_menu_item(&self, _menu: &str, _item: &str, _callback: Box<dyn Fn()>) -> Result<(), PluginError> {
        println!("MockUIApi: add_menu_item called");
        Ok(())
    }

    fn show_notification(&self, _message: &str, _level: NotificationLevel) -> Result<(), PluginError> {
        println!("MockUIApi: show_notification called");
        Ok(())
    }

    fn create_custom_panel(&self, _name: &str, _content: UIElement) -> Result<(), PluginError> {
        println!("MockUIApi: create_custom_panel called");
        Ok(())
    }

    fn update_status_bar(&self, _message: &str) -> Result<(), PluginError> {
        println!("MockUIApi: update_status_bar called");
        Ok(())
    }
}

#[derive(Debug)]
struct MockConfigApi;

impl ConfigApi for MockConfigApi {
    fn get_config_value(&self, _key: &str) -> Result<JsonValue, PluginError> {
        Ok(JsonValue::Null)
    }

    fn set_config_value(&self, _key: &str, _value: &JsonValue) -> Result<(), PluginError> {
        println!("MockConfigApi: set_config_value called");
        Ok(())
    }

    fn save_config(&self) -> Result<(), PluginError> {
        println!("MockConfigApi: save_config called");
        Ok(())
    }

    fn create_config_section(&self, _section: &str) -> Result<(), PluginError> {
        println!("MockConfigApi: create_config_section called");
        Ok(())
    }
}

#[derive(Debug)]
struct MockFileApi;

impl FileApi for MockFileApi {
    fn read_file(&self, _path: &str) -> Result<String, PluginError> {
        Ok("mock file content".to_string())
    }

    fn write_file(&self, _path: &str, _content: &str) -> Result<(), PluginError> {
        println!("MockFileApi: write_file called");
        Ok(())
    }

    fn list_files(&self, _directory: &str) -> Result<Vec<String>, PluginError> {
        Ok(vec!["mock_file.txt".to_string()])
    }

    fn delete_file(&self, _path: &str) -> Result<(), PluginError> {
        println!("MockFileApi: delete_file called");
        Ok(())
    }

    fn file_exists(&self, _path: &str) -> Result<bool, PluginError> {
        Ok(false)
    }
}

impl PluginApiRegistry {
    pub fn new() -> Self {
        // Create mock implementations for development/testing
        Self {
            motion_api: Arc::new(MockMotionApi),
            temperature_api: Arc::new(MockTemperatureApi),
            gcode_api: Arc::new(MockGCodeApi),
            ui_api: Arc::new(MockUIApi),
            config_api: Arc::new(MockConfigApi),
            file_api: Arc::new(MockFileApi),
        }
    }
}

#[derive(Debug, Clone)]
pub enum PluginError {
    NotFound(String),
    LoadError(String),
    ManifestError(String),
    SecurityViolation(String),
    IoError(String),
    ScriptError(String),
    UnsupportedType(String),
    PermissionDenied(String),
    ApiError(String),
}

impl std::fmt::Display for PluginError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            PluginError::NotFound(name) => write!(f, "Plugin not found: {}", name),
            PluginError::LoadError(msg) => write!(f, "Plugin load error: {}", msg),
            PluginError::ManifestError(msg) => write!(f, "Plugin manifest error: {}", msg),
            PluginError::SecurityViolation(msg) => write!(f, "Security violation: {}", msg),
            PluginError::IoError(msg) => write!(f, "IO error: {}", msg),
            PluginError::ScriptError(msg) => write!(f, "Script error: {}", msg),
            PluginError::UnsupportedType(msg) => write!(f, "Unsupported plugin type: {}", msg),
            PluginError::PermissionDenied(msg) => write!(f, "Permission denied: {}", msg),
            PluginError::ApiError(msg) => write!(f, "API error: {}", msg),
        }
    }
}

impl std::error::Error for PluginError {}

// Convenience macros for plugin development
#[macro_export]
macro_rules! declare_plugin {
    ($plugin_type:ty) => {
        #[no_mangle]
        pub unsafe extern "C" fn create_plugin() -> *mut dyn $crate::plugin_system::PluginInterface {
            let plugin = <$plugin_type>::new();
            Box::into_raw(Box::new(plugin))
        }
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_plugin_manager_creation() {
        let manager = PluginManager::new();
        assert_eq!(manager.list_plugins().len(), 0);
    }

    #[test]
    fn test_event_bus() {
        let event_bus = PluginEventBus::new();
        event_bus.subscribe("test_plugin", "print_started");

        let event = PluginEvent::PrintStarted {
            filename: "test.gcode".to_string()
        };

        let subscribers = event_bus.publish(&event);
        assert_eq!(subscribers.len(), 1);
        assert_eq!(subscribers[0], "test_plugin");
    }

    #[test]
    fn test_security_manager() {
        let security_manager = PluginSecurityManager::new();
        assert!(!security_manager.is_plugin_banned("test_plugin"));
        assert!(!security_manager.check_permission("test_plugin", &Permission::ReadConfig));
    }
}