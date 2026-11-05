use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use heapless::{String, Vec};
use serde::{Deserialize, Serialize};

use crate::config_manager::{ConfigError, ConfigValue, PIDConfig, CONFIG_MANAGER};
use crate::exclude_object::EXCLUDE_OBJECT_MANAGER;
use crate::gcode_processor::GCODE_PROCESSOR;
use crate::power_recovery::POWER_RECOVERY;
use crate::safety::{emergency_stop_active, trigger_emergency_stop};

const MAX_HTTP_RESPONSE_SIZE: usize = 8192;
const MAX_HTTP_REQUEST_SIZE: usize = 2048;
const MAX_URL_LENGTH: usize = 256;
const MAX_WEBSOCKET_CLIENTS: usize = 4;
const MAX_API_TOKEN_LENGTH: usize = 64;

static WEB_INTERFACE: &str = include_str!("web_interface.html");

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApiResponse<T> {
    pub success: bool,
    pub data: Option<T>,
    pub error: Option<String<128>>,
    pub timestamp: u64,
}

impl<T> ApiResponse<T> {
    pub fn success(data: T) -> Self {
        Self {
            success: true,
            data: Some(data),
            error: None,
            timestamp: embassy_time::Instant::now().as_millis(),
        }
    }

    pub fn error(message: &str) -> Self {
        Self {
            success: false,
            data: None,
            error: String::try_from(message).ok(),
            timestamp: embassy_time::Instant::now().as_millis(),
        }
    }
}

/// Values not known yet are null, so the page can show "--". A made-up reading like 25°C
/// could look like a safe, cold hotend when it is not.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrinterStatus {
    pub is_printing: bool,
    pub emergency_stop: bool,
    pub current_temp_hotend: Option<f32>,
    pub target_temp_hotend: Option<f32>,
    pub current_temp_bed: Option<f32>,
    pub target_temp_bed: Option<f32>,
    pub print_progress: Option<f32>,
    pub current_layer: u32,
    pub position: [f32; 4], // X, Y, Z, E
    pub fan_speed: Option<u8>,
    pub power_recovery_available: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WebSocketMessage {
    pub message_type: String<32>,
    pub data: String<512>,
    pub timestamp: u64,
}

#[derive(Deserialize)]
struct ObjectRequest<'a> {
    object_name: &'a str,
}

#[derive(Deserialize)]
struct ProfileRequest<'a> {
    profile_name: &'a str,
}

#[derive(Deserialize)]
struct FloatValue {
    value: f32,
}

#[derive(Deserialize)]
struct BoolValue {
    value: bool,
}

/// What the settings form on the page sends. Only these fields change.
#[derive(Deserialize)]
struct ConfigForm {
    motion: MotionForm,
    temperature: TemperatureForm,
    extruder: ExtruderForm,
    safety: SafetyForm,
    features: FeaturesForm,
}

#[derive(Deserialize)]
struct MotionForm {
    max_feedrate: [f32; 4],
    steps_per_mm: [f32; 4],
}

#[derive(Deserialize)]
struct TemperatureForm {
    hotend_max_temp: f32,
    bed_max_temp: f32,
    pid_hotend: PIDConfig,
}

#[derive(Deserialize)]
struct ExtruderForm {
    filament_diameter: f32,
    nozzle_diameter: f32,
    retract_length: f32,
}

#[derive(Deserialize)]
struct SafetyForm {
    thermal_protection_enabled: bool,
}

#[derive(Deserialize)]
struct FeaturesForm {
    power_loss_recovery: bool,
    exclude_objects: bool,
}

pub struct WebServer {
    port: u16,
    websocket_port: u16,
    active_connections: Vec<WebSocketClient, MAX_WEBSOCKET_CLIENTS>,
    running: bool,
    api_token: String<MAX_API_TOKEN_LENGTH>,
}

#[derive(Debug, Clone)]
struct WebSocketClient {
    id: u32,
    connected: bool,
    last_ping: u64,
}

fn config_error_message(error: &ConfigError) -> &'static str {
    match error {
        ConfigError::InvalidMotionConfig => "Invalid motion settings",
        ConfigError::InvalidTemperatureConfig => "Invalid temperature settings",
        ConfigError::InvalidExtruderConfig => "Invalid extruder settings",
        ConfigError::InvalidNetworkConfig => "Invalid network settings",
        ConfigError::ThermalProtectionRequired => "Thermal protection cannot be turned off",
        ConfigError::InvalidFieldPath => "Unknown setting",
        ConfigError::ProfileNotFound => "Profile not found",
        ConfigError::TooManyProfiles => "Too many profiles",
        ConfigError::InvalidProfileName => "Invalid profile name",
        _ => "Configuration error",
    }
}

fn constant_time_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    a.iter().zip(b.iter()).fold(0u8, |diff, (x, y)| diff | (x ^ y)) == 0
}

impl WebServer {
    /// With an empty token every protected request is refused. There is no default password.
    pub fn new(port: u16, websocket_port: u16, api_token: &str) -> Self {
        Self {
            port,
            websocket_port,
            active_connections: Vec::new(),
            running: false,
            api_token: String::try_from(api_token).unwrap_or_default(),
        }
    }

    pub async fn start(&mut self) -> Result<(), WebServerError> {
        // This would initialize the actual network stack and HTTP server
        // For embedded systems, this might use embassy-net or similar
        // Until then nothing is listening, and starting must not claim otherwise.
        Err(WebServerError::NetworkError)
    }

    pub async fn stop(&mut self) {
        self.running = false;
        self.active_connections.clear();
    }

    /// Reading status is open, as is the emergency stop: nobody should need a password to
    /// stop a machine. Anything else that changes or reveals settings needs the token.
    fn request_allowed(&self, request: &HttpRequest) -> bool {
        let path = request.path.as_str();
        let open = path == "/api/emergency_stop"
            || (request.method.as_str() == "GET"
                && matches!(path, "/" | "/api/status" | "/api/objects" | "/api/recovery"));
        if open {
            return true;
        }
        match (&request.authorization, self.api_token.is_empty()) {
            (Some(header), false) => {
                let presented = header.as_str().strip_prefix("Bearer ").unwrap_or("");
                constant_time_eq(presented.as_bytes(), self.api_token.as_bytes())
            }
            _ => false,
        }
    }

    pub async fn handle_http_request(&self, request: &HttpRequest) -> HttpResponse {
        if !self.request_allowed(request) {
            return HttpResponse::with_status(401, &ApiResponse::<()>::error("Authentication required"));
        }
        match request.method.as_str() {
            "GET" => self.handle_get_request(request).await,
            "POST" => self.handle_post_request(request).await,
            "PUT" => self.handle_put_request(request).await,
            "DELETE" => self.handle_delete_request(request).await,
            _ => HttpResponse::method_not_allowed(),
        }
    }

    async fn handle_get_request(&self, request: &HttpRequest) -> HttpResponse {
        match request.path.as_str() {
            "/" => HttpResponse::static_page(WEB_INTERFACE.as_bytes(), "text/html"),
            "/api/status" => self.get_printer_status().await,
            "/api/config" => self.get_configuration().await,
            "/api/profiles" => self.list_config_profiles().await,
            "/api/objects" => self.list_print_objects().await,
            "/api/recovery" => self.get_recovery_info().await,
            path if path.starts_with("/api/config/") => {
                let field = &path["/api/config/".len()..];
                self.get_config_field(field).await
            }
            _ => HttpResponse::not_found(),
        }
    }

    async fn handle_post_request(&self, request: &HttpRequest) -> HttpResponse {
        match request.path.as_str() {
            "/api/config/save" => self.save_config_profile(request).await,
            "/api/config/load" => self.load_config_profile(request).await,
            "/api/config/reset" => self.reset_configuration().await,
            "/api/config/import" => HttpResponse::not_implemented(),
            "/api/objects/exclude" => self.change_object(request, true).await,
            "/api/objects/include" => self.change_object(request, false).await,
            "/api/recovery/resume" => self.resume_print().await,
            "/api/emergency_stop" => self.emergency_stop(),
            _ => HttpResponse::not_found(),
        }
    }

    async fn handle_put_request(&self, request: &HttpRequest) -> HttpResponse {
        match request.path.as_str() {
            "/api/config" => self.update_configuration(request).await,
            path if path.starts_with("/api/config/") => {
                let field = &path["/api/config/".len()..];
                self.update_config_field(field, request).await
            }
            _ => HttpResponse::not_found(),
        }
    }

    async fn handle_delete_request(&self, request: &HttpRequest) -> HttpResponse {
        match request.path.as_str() {
            path if path.starts_with("/api/profiles/") => {
                let profile_name = &path["/api/profiles/".len()..];
                self.delete_config_profile(profile_name).await
            }
            _ => HttpResponse::not_found(),
        }
    }

    async fn get_printer_status(&self) -> HttpResponse {
        // Only what can actually be known so far. See PrinterStatus.
        let (position, current_layer) = match GCODE_PROCESSOR.try_lock() {
            Ok(guard) => guard.as_ref().map_or(([0.0; 4], 0), |p| (p.get_current_position(), p.get_current_layer())),
            Err(_) => ([0.0; 4], 0),
        };
        let power_recovery_available = match POWER_RECOVERY.try_lock() {
            Ok(guard) => match guard.as_ref() {
                Some(manager) => manager.get_recovery_info().await.is_some(),
                None => false,
            },
            Err(_) => false,
        };

        HttpResponse::json(&ApiResponse::success(PrinterStatus {
            is_printing: false,
            emergency_stop: emergency_stop_active(),
            current_temp_hotend: None,
            target_temp_hotend: None,
            current_temp_bed: None,
            target_temp_bed: None,
            print_progress: None,
            current_layer,
            position,
            fan_speed: None,
            power_recovery_available,
        }))
    }

    async fn get_configuration(&self) -> HttpResponse {
        let guard = CONFIG_MANAGER.lock().await;
        let Some(manager) = guard.as_ref() else {
            return HttpResponse::unavailable();
        };
        let mut config = manager.get_current_config().await;
        // Stored secrets never leave the printer, even for an authenticated client
        config.network.wifi_password.clear();
        HttpResponse::json(&ApiResponse::success(config))
    }

    async fn update_configuration(&self, request: &HttpRequest) -> HttpResponse {
        let Ok((form, _)) = serde_json_core::from_slice::<ConfigForm>(&request.body) else {
            return HttpResponse::bad_json("Invalid configuration format");
        };
        let guard = CONFIG_MANAGER.lock().await;
        let Some(manager) = guard.as_ref() else {
            return HttpResponse::unavailable();
        };
        let mut config = manager.get_current_config().await;
        config.motion.max_feedrate = form.motion.max_feedrate;
        config.motion.steps_per_mm = form.motion.steps_per_mm;
        config.temperature.hotend_max_temp = form.temperature.hotend_max_temp;
        config.temperature.bed_max_temp = form.temperature.bed_max_temp;
        config.temperature.pid_hotend = form.temperature.pid_hotend;
        config.extruder.filament_diameter = form.extruder.filament_diameter;
        config.extruder.nozzle_diameter = form.extruder.nozzle_diameter;
        config.extruder.retract_length = form.extruder.retract_length;
        config.safety.thermal_protection_enabled = form.safety.thermal_protection_enabled;
        config.features.power_loss_recovery = form.features.power_loss_recovery;
        config.features.exclude_objects = form.features.exclude_objects;
        Self::config_result(manager.update_config(config).await)
    }

    fn config_result(result: Result<(), ConfigError>) -> HttpResponse {
        match result {
            Ok(()) => HttpResponse::json(&ApiResponse::<()>::success(())),
            Err(error) => HttpResponse::bad_json(config_error_message(&error)),
        }
    }

    async fn get_config_field(&self, field_path: &str) -> HttpResponse {
        let guard = CONFIG_MANAGER.lock().await;
        let Some(manager) = guard.as_ref() else {
            return HttpResponse::unavailable();
        };
        match manager.get_config_field(field_path).await {
            Some(value) => HttpResponse::json(&ApiResponse::success(value)),
            None => HttpResponse::not_found(),
        }
    }

    async fn update_config_field(&self, field_path: &str, request: &HttpRequest) -> HttpResponse {
        let value = if let Ok((parsed, _)) = serde_json_core::from_slice::<BoolValue>(&request.body) {
            ConfigValue::Bool(parsed.value)
        } else if let Ok((parsed, _)) = serde_json_core::from_slice::<FloatValue>(&request.body) {
            // Integer settings. Out-of-range values saturate here and fail validation.
            if field_path == "network.web_port" {
                ConfigValue::Int(parsed.value as i32)
            } else {
                ConfigValue::Float(parsed.value)
            }
        } else {
            return HttpResponse::bad_json("Expected {\"value\": ...}");
        };
        let guard = CONFIG_MANAGER.lock().await;
        let Some(manager) = guard.as_ref() else {
            return HttpResponse::unavailable();
        };
        Self::config_result(manager.set_config_field(field_path, value).await)
    }

    async fn save_config_profile(&self, request: &HttpRequest) -> HttpResponse {
        let Ok((body, _)) = serde_json_core::from_slice::<ProfileRequest>(&request.body) else {
            return HttpResponse::bad_json("Expected a profile_name");
        };
        let guard = CONFIG_MANAGER.lock().await;
        let Some(manager) = guard.as_ref() else {
            return HttpResponse::unavailable();
        };
        Self::config_result(manager.save_profile(body.profile_name).await)
    }

    async fn load_config_profile(&self, request: &HttpRequest) -> HttpResponse {
        let Ok((body, _)) = serde_json_core::from_slice::<ProfileRequest>(&request.body) else {
            return HttpResponse::bad_json("Expected a profile_name");
        };
        let guard = CONFIG_MANAGER.lock().await;
        let Some(manager) = guard.as_ref() else {
            return HttpResponse::unavailable();
        };
        Self::config_result(manager.load_profile(body.profile_name).await)
    }

    async fn list_config_profiles(&self) -> HttpResponse {
        let guard = CONFIG_MANAGER.lock().await;
        let Some(manager) = guard.as_ref() else {
            return HttpResponse::unavailable();
        };
        HttpResponse::json(&ApiResponse::success(manager.list_profiles().await))
    }

    async fn delete_config_profile(&self, profile_name: &str) -> HttpResponse {
        let guard = CONFIG_MANAGER.lock().await;
        let Some(manager) = guard.as_ref() else {
            return HttpResponse::unavailable();
        };
        Self::config_result(manager.delete_profile(profile_name).await)
    }

    async fn list_print_objects(&self) -> HttpResponse {
        let guard = EXCLUDE_OBJECT_MANAGER.lock().await;
        let Some(manager) = guard.as_ref() else {
            return HttpResponse::unavailable();
        };
        HttpResponse::json(&ApiResponse::success(manager.get_object_list().await))
    }

    async fn change_object(&self, request: &HttpRequest, exclude: bool) -> HttpResponse {
        let Ok((body, _)) = serde_json_core::from_slice::<ObjectRequest>(&request.body) else {
            return HttpResponse::bad_json("Expected an object_name");
        };
        let mut guard = EXCLUDE_OBJECT_MANAGER.lock().await;
        let Some(manager) = guard.as_mut() else {
            return HttpResponse::unavailable();
        };
        let result = if exclude {
            manager.exclude_object(body.object_name).await
        } else {
            manager.include_object(body.object_name).await
        };
        match result {
            Ok(()) => HttpResponse::json(&ApiResponse::<()>::success(())),
            Err(_) => HttpResponse::bad_json(if exclude { "Failed to exclude object" } else { "Failed to include object" }),
        }
    }

    async fn get_recovery_info(&self) -> HttpResponse {
        let guard = POWER_RECOVERY.lock().await;
        if let Some(manager) = guard.as_ref() {
            if let Some(info) = manager.get_recovery_info().await {
                return HttpResponse::json(&ApiResponse::success(info));
            }
        }
        HttpResponse::json(&ApiResponse::<()>::error("No recovery data available"))
    }

    async fn resume_print(&self) -> HttpResponse {
        let mut guard = POWER_RECOVERY.lock().await;
        let Some(manager) = guard.as_mut() else {
            return HttpResponse::unavailable();
        };
        match manager.resume_print().await {
            Ok(()) => HttpResponse::json(&ApiResponse::<()>::success(())),
            Err(_) => HttpResponse::bad_json("Failed to resume print"),
        }
    }

    async fn reset_configuration(&self) -> HttpResponse {
        let guard = CONFIG_MANAGER.lock().await;
        let Some(manager) = guard.as_ref() else {
            return HttpResponse::unavailable();
        };
        Self::config_result(manager.reset_to_defaults().await)
    }

    /// Takes no locks, so a busy subsystem can never delay or block a stop.
    fn emergency_stop(&self) -> HttpResponse {
        trigger_emergency_stop();
        HttpResponse::json(&ApiResponse::<()>::success(()))
    }

    pub async fn broadcast_websocket_message(&mut self, _message: &WebSocketMessage) {
        // Remove disconnected clients
        self.active_connections.retain(|client| client.connected);

        // Broadcast to all connected clients
        for client in &mut self.active_connections {
            // Implementation would send the message to each WebSocket client
            client.last_ping = embassy_time::Instant::now().as_millis();
        }
    }
}

#[derive(Debug, Clone)]
pub struct HttpRequest {
    pub method: String<8>,
    pub path: String<MAX_URL_LENGTH>,
    /// The Authorization header, if the request had one
    pub authorization: Option<String<96>>,
    pub body: Vec<u8, MAX_HTTP_REQUEST_SIZE>,
}

#[derive(Debug, Clone)]
pub struct HttpResponse {
    pub status_code: u16,
    pub headers: Vec<(String<32>, String<64>), 8>,
    pub body: Vec<u8, MAX_HTTP_RESPONSE_SIZE>,
    /// Large constant content such as the web page, sent from flash without being copied
    pub static_body: Option<&'static [u8]>,
}

impl HttpResponse {
    fn empty(status_code: u16) -> Self {
        Self { status_code, headers: Vec::new(), body: Vec::new(), static_body: None }
    }

    fn with_content_type(mut self, content_type: &str) -> Self {
        self.headers.push((
            String::try_from("Content-Type").unwrap_or_default(),
            String::try_from(content_type).unwrap_or_default()
        )).ok();
        self
    }

    pub fn with_status<T: Serialize>(status_code: u16, value: &T) -> Self {
        match serde_json_core::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(value) {
            Ok(body) => {
                let mut response = Self::empty(status_code).with_content_type("application/json");
                response.body = body;
                response
            }
            // Too big or unserializable. An error, never a truncated 200.
            Err(_) => Self::empty(500),
        }
    }

    pub fn json<T: Serialize>(value: &T) -> Self {
        Self::with_status(200, value)
    }

    pub fn bad_json(message: &str) -> Self {
        Self::with_status(400, &ApiResponse::<()>::error(message))
    }

    pub fn static_page(body: &'static [u8], content_type: &str) -> Self {
        let mut response = Self::empty(200).with_content_type(content_type);
        response.static_body = Some(body);
        response
    }

    pub fn not_found() -> Self {
        Self::empty(404)
    }

    pub fn method_not_allowed() -> Self {
        Self::empty(405)
    }

    pub fn not_implemented() -> Self {
        Self::with_status(501, &ApiResponse::<()>::error("Not implemented"))
    }

    pub fn unavailable() -> Self {
        Self::with_status(503, &ApiResponse::<()>::error("Not ready"))
    }
}

#[derive(Debug)]
pub enum WebServerError {
    NetworkError,
    ConfigurationError,
    InternalError,
}

pub static WEB_SERVER: Mutex<CriticalSectionRawMutex, Option<WebServer>> = Mutex::new(None);

pub async fn init_web_server(port: u16, websocket_port: u16, api_token: &str) -> &'static Mutex<CriticalSectionRawMutex, Option<WebServer>> {
    let mut server = WebServer::new(port, websocket_port, api_token);
    let _ = server.start().await;

    {
        let mut guard = WEB_SERVER.lock().await;
        *guard = Some(server);
    }

    &WEB_SERVER
}
