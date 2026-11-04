use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use heapless::{String, Vec};
use serde::{Deserialize, Serialize};

use crate::config_manager::{CONFIG_MANAGER, ConfigValue, ConfigError};
use crate::exclude_object::EXCLUDE_OBJECT_MANAGER;
use crate::power_recovery::POWER_RECOVERY;

const MAX_HTTP_RESPONSE_SIZE: usize = 4096;
const MAX_HTTP_REQUEST_SIZE: usize = 2048;
const MAX_URL_LENGTH: usize = 256;
const MAX_WEBSOCKET_CLIENTS: usize = 4;

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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrinterStatus {
    pub is_printing: bool,
    pub current_temp_hotend: f32,
    pub target_temp_hotend: f32,
    pub current_temp_bed: f32,
    pub target_temp_bed: f32,
    pub print_progress: f32,
    pub current_layer: u32,
    pub position: [f32; 4], // X, Y, Z, E
    pub fan_speed: u8,
    pub power_recovery_available: bool,
    pub excluded_objects: Vec<String<32>, 16>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WebSocketMessage {
    pub message_type: String<32>,
    pub data: String<512>,
    pub timestamp: u64,
}

pub struct WebServer {
    port: u16,
    websocket_port: u16,
    active_connections: Vec<WebSocketClient, MAX_WEBSOCKET_CLIENTS>,
    running: bool,
}

#[derive(Debug, Clone)]
struct WebSocketClient {
    id: u32,
    connected: bool,
    last_ping: u64,
}

impl WebServer {
    pub fn new(port: u16, websocket_port: u16) -> Self {
        Self {
            port,
            websocket_port,
            active_connections: Vec::new(),
            running: false,
        }
    }

    pub async fn start(&mut self) -> Result<(), WebServerError> {
        self.running = true;
        
        // This would initialize the actual network stack and HTTP server
        // For embedded systems, this might use embassy-net or similar
        
        Ok(())
    }

    pub async fn stop(&mut self) {
        self.running = false;
        self.active_connections.clear();
    }

    pub async fn handle_http_request(&self, request: &HttpRequest) -> HttpResponse {
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
            "/" => self.serve_web_interface().await,
            "/api/status" => self.get_printer_status().await,
            "/api/config" => self.get_configuration().await,
            "/api/profiles" => self.list_config_profiles().await,
            "/api/objects" => self.list_print_objects().await,
            "/api/recovery" => self.get_recovery_info().await,
            path if path.starts_with("/api/config/") => {
                let field = &path[12..]; // Remove "/api/config/"
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
            "/api/config/import" => self.import_configuration(request).await,
            "/api/objects/exclude" => self.exclude_object(request).await,
            "/api/objects/include" => self.include_object(request).await,
            "/api/recovery/resume" => self.resume_print().await,
            "/api/emergency_stop" => self.emergency_stop().await,
            _ => HttpResponse::not_found(),
        }
    }

    async fn handle_put_request(&self, request: &HttpRequest) -> HttpResponse {
        match request.path.as_str() {
            "/api/config" => self.update_configuration(request).await,
            path if path.starts_with("/api/config/") => {
                let field = &path[12..]; // Remove "/api/config/"
                self.update_config_field(field, request).await
            }
            _ => HttpResponse::not_found(),
        }
    }

    async fn handle_delete_request(&self, request: &HttpRequest) -> HttpResponse {
        match request.path.as_str() {
            path if path.starts_with("/api/profiles/") => {
                let profile_name = &path[14..]; // Remove "/api/profiles/"
                self.delete_config_profile(profile_name).await
            }
            _ => HttpResponse::not_found(),
        }
    }

    async fn serve_web_interface(&self) -> HttpResponse {
        let html = include_str!("web_interface.html");
        HttpResponse::ok(html.as_bytes(), "text/html")
    }

    async fn get_printer_status(&self) -> HttpResponse {
        // Collect status from various subsystems
        let status = PrinterStatus {
            is_printing: false, // Would check actual printing state
            current_temp_hotend: 25.0, // Would read from sensors
            target_temp_hotend: 0.0,
            current_temp_bed: 25.0,
            target_temp_bed: 0.0,
            print_progress: 0.0,
            current_layer: 0,
            position: [0.0; 4],
            fan_speed: 0,
            power_recovery_available: false,
            excluded_objects: Vec::new(),
        };

        let response = ApiResponse::success(status);
        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
            HttpResponse::ok(&json, "application/json")
        } else {
            HttpResponse::internal_error()
        }
    }

    async fn get_configuration(&self) -> HttpResponse {
        if let Ok(manager_guard) = CONFIG_MANAGER.try_lock() {
            if let Some(ref manager) = *manager_guard {
                let config = manager.get_current_config().await;
                let response = ApiResponse::success(config);
                
                if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                    return HttpResponse::ok(&json, "application/json");
                }
            }
        }
        HttpResponse::internal_error()
    }

    async fn update_configuration(&self, _request: &HttpRequest) -> HttpResponse {
        // Parse the request body to get the new configuration
        if let Ok(_manager_guard) = CONFIG_MANAGER.try_lock() {
            if let Some(ref _manager) = *_manager_guard {
                // This would parse the JSON body and update the configuration
                let response = ApiResponse::<()>::success(());
                
                if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                    return HttpResponse::ok(&json, "application/json");
                }
            }
        }
        HttpResponse::internal_error()
    }

    async fn get_config_field(&self, field_path: &str) -> HttpResponse {
        if let Ok(manager_guard) = CONFIG_MANAGER.try_lock() {
            if let Some(ref manager) = *manager_guard {
                if let Some(value) = manager.get_config_field(field_path).await {
                    let response = ApiResponse::success(value);
                    
                    if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                        return HttpResponse::ok(&json, "application/json");
                    }
                }
            }
        }
        HttpResponse::not_found()
    }

    async fn update_config_field(&self, _field_path: &str, _request: &HttpRequest) -> HttpResponse {
        // This would parse the request body to extract the new value
        // and update the specific configuration field
        let response = ApiResponse::<()>::success(());
        
        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
            HttpResponse::ok(&json, "application/json")
        } else {
            HttpResponse::internal_error()
        }
    }

    async fn save_config_profile(&self, _request: &HttpRequest) -> HttpResponse {
        // Parse profile name from request body
        let profile_name = "Default"; // Would parse from request
        
        if let Ok(manager_guard) = CONFIG_MANAGER.try_lock() {
            if let Some(ref manager) = *manager_guard {
                match manager.save_profile(profile_name).await {
                    Ok(()) => {
                        let response = ApiResponse::<()>::success(());
                        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                            return HttpResponse::ok(&json, "application/json");
                        }
                    }
                    Err(_) => {
                        let response = ApiResponse::<()>::error("Failed to save profile");
                        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                            return HttpResponse::bad_request(&json, "application/json");
                        }
                    }
                }
            }
        }
        HttpResponse::internal_error()
    }

    async fn load_config_profile(&self, _request: &HttpRequest) -> HttpResponse {
        let profile_name = "Default"; // Would parse from request
        
        if let Ok(manager_guard) = CONFIG_MANAGER.try_lock() {
            if let Some(ref manager) = *manager_guard {
                match manager.load_profile(profile_name).await {
                    Ok(()) => {
                        let response = ApiResponse::<()>::success(());
                        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                            return HttpResponse::ok(&json, "application/json");
                        }
                    }
                    Err(_) => {
                        let response = ApiResponse::<()>::error("Failed to load profile");
                        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                            return HttpResponse::bad_request(&json, "application/json");
                        }
                    }
                }
            }
        }
        HttpResponse::internal_error()
    }

    async fn list_config_profiles(&self) -> HttpResponse {
        if let Ok(manager_guard) = CONFIG_MANAGER.try_lock() {
            if let Some(ref manager) = *manager_guard {
                let profiles = manager.list_profiles().await;
                let response = ApiResponse::success(profiles);
                
                if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                    return HttpResponse::ok(&json, "application/json");
                }
            }
        }
        HttpResponse::internal_error()
    }

    async fn delete_config_profile(&self, profile_name: &str) -> HttpResponse {
        if let Ok(manager_guard) = CONFIG_MANAGER.try_lock() {
            if let Some(ref manager) = *manager_guard {
                match manager.delete_profile(profile_name).await {
                    Ok(()) => {
                        let response = ApiResponse::<()>::success(());
                        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                            return HttpResponse::ok(&json, "application/json");
                        }
                    }
                    Err(_) => {
                        let response = ApiResponse::<()>::error("Failed to delete profile");
                        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                            return HttpResponse::bad_request(&json, "application/json");
                        }
                    }
                }
            }
        }
        HttpResponse::internal_error()
    }

    async fn list_print_objects(&self) -> HttpResponse {
        if let Ok(manager_guard) = EXCLUDE_OBJECT_MANAGER.try_lock() {
            if let Some(ref manager) = *manager_guard {
                let objects = manager.get_object_list().await;
                let response = ApiResponse::success(objects);
                
                if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                    return HttpResponse::ok(&json, "application/json");
                }
            }
        }
        HttpResponse::internal_error()
    }

    async fn exclude_object(&self, _request: &HttpRequest) -> HttpResponse {
        let object_name = "object1"; // Would parse from request body
        
        if let Ok(mut manager_guard) = EXCLUDE_OBJECT_MANAGER.try_lock() {
            if let Some(ref mut manager) = *manager_guard {
                match manager.exclude_object(object_name).await {
                    Ok(()) => {
                        let response = ApiResponse::<()>::success(());
                        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                            return HttpResponse::ok(&json, "application/json");
                        }
                    }
                    Err(_) => {
                        let response = ApiResponse::<()>::error("Failed to exclude object");
                        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                            return HttpResponse::bad_request(&json, "application/json");
                        }
                    }
                }
            }
        }
        HttpResponse::internal_error()
    }

    async fn include_object(&self, _request: &HttpRequest) -> HttpResponse {
        let object_name = "object1"; // Would parse from request body
        
        if let Ok(mut manager_guard) = EXCLUDE_OBJECT_MANAGER.try_lock() {
            if let Some(ref mut manager) = *manager_guard {
                match manager.include_object(object_name).await {
                    Ok(()) => {
                        let response = ApiResponse::<()>::success(());
                        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                            return HttpResponse::ok(&json, "application/json");
                        }
                    }
                    Err(_) => {
                        let response = ApiResponse::<()>::error("Failed to include object");
                        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                            return HttpResponse::bad_request(&json, "application/json");
                        }
                    }
                }
            }
        }
        HttpResponse::internal_error()
    }

    async fn get_recovery_info(&self) -> HttpResponse {
        if let Ok(manager_guard) = POWER_RECOVERY.try_lock() {
            if let Some(ref manager) = *manager_guard {
                if let Some((timestamp, progress)) = manager.get_recovery_info().await {
                    let recovery_info = (timestamp, progress);
                    let response = ApiResponse::success(recovery_info);
                    
                    if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                        return HttpResponse::ok(&json, "application/json");
                    }
                }
            }
        }
        
        let response = ApiResponse::<()>::error("No recovery data available");
        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
            HttpResponse::ok(&json, "application/json")
        } else {
            HttpResponse::internal_error()
        }
    }

    async fn resume_print(&self) -> HttpResponse {
        if let Ok(mut manager_guard) = POWER_RECOVERY.try_lock() {
            if let Some(ref mut manager) = *manager_guard {
                match manager.resume_print().await {
                    Ok(()) => {
                        let response = ApiResponse::<()>::success(());
                        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                            return HttpResponse::ok(&json, "application/json");
                        }
                    }
                    Err(_) => {
                        let response = ApiResponse::<()>::error("Failed to resume print");
                        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                            return HttpResponse::bad_request(&json, "application/json");
                        }
                    }
                }
            }
        }
        HttpResponse::internal_error()
    }

    async fn reset_configuration(&self) -> HttpResponse {
        if let Ok(manager_guard) = CONFIG_MANAGER.try_lock() {
            if let Some(ref manager) = *manager_guard {
                match manager.reset_to_defaults().await {
                    Ok(()) => {
                        let response = ApiResponse::<()>::success(());
                        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                            return HttpResponse::ok(&json, "application/json");
                        }
                    }
                    Err(_) => {
                        let response = ApiResponse::<()>::error("Failed to reset configuration");
                        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
                            return HttpResponse::bad_request(&json, "application/json");
                        }
                    }
                }
            }
        }
        HttpResponse::internal_error()
    }

    async fn import_configuration(&self, _request: &HttpRequest) -> HttpResponse {
        // Would parse JSON from request body
        let response = ApiResponse::<()>::success(());
        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
            HttpResponse::ok(&json, "application/json")
        } else {
            HttpResponse::internal_error()
        }
    }

    async fn emergency_stop(&self) -> HttpResponse {
        // Implement emergency stop logic
        let response = ApiResponse::<()>::success(());
        if let Ok(json) = postcard::to_vec::<_, MAX_HTTP_RESPONSE_SIZE>(&response) {
            HttpResponse::ok(&json, "application/json")
        } else {
            HttpResponse::internal_error()
        }
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
    pub body: Vec<u8, MAX_HTTP_REQUEST_SIZE>,
}

#[derive(Debug, Clone)]
pub struct HttpResponse {
    pub status_code: u16,
    pub headers: Vec<(String<32>, String<64>), 8>,
    pub body: Vec<u8, MAX_HTTP_RESPONSE_SIZE>,
}

impl HttpResponse {
    pub fn ok(body: &[u8], content_type: &str) -> Self {
        let mut response = Self {
            status_code: 200,
            headers: Vec::new(),
            body: Vec::new(),
        };
        
        response.headers.push((
            String::try_from("Content-Type").unwrap_or_default(),
            String::try_from(content_type).unwrap_or_default()
        )).ok();
        
        response.body.extend_from_slice(body).ok();
        response
    }

    pub fn bad_request(body: &[u8], content_type: &str) -> Self {
        let mut response = Self {
            status_code: 400,
            headers: Vec::new(),
            body: Vec::new(),
        };
        
        response.headers.push((
            String::try_from("Content-Type").unwrap_or_default(),
            String::try_from(content_type).unwrap_or_default()
        )).ok();
        
        response.body.extend_from_slice(body).ok();
        response
    }

    pub fn not_found() -> Self {
        Self {
            status_code: 404,
            headers: Vec::new(),
            body: Vec::new(),
        }
    }

    pub fn method_not_allowed() -> Self {
        Self {
            status_code: 405,
            headers: Vec::new(),
            body: Vec::new(),
        }
    }

    pub fn internal_error() -> Self {
        Self {
            status_code: 500,
            headers: Vec::new(),
            body: Vec::new(),
        }
    }
}

#[derive(Debug)]
pub enum WebServerError {
    NetworkError,
    ConfigurationError,
    InternalError,
}

pub static WEB_SERVER: Mutex<CriticalSectionRawMutex, Option<WebServer>> = Mutex::new(None);

pub async fn init_web_server(port: u16, websocket_port: u16) -> &'static Mutex<CriticalSectionRawMutex, Option<WebServer>> {
    let mut server = WebServer::new(port, websocket_port);
    let _ = server.start().await;
    
    {
        let mut guard = WEB_SERVER.lock().await;
        *guard = Some(server);
    }
    
    &WEB_SERVER
}