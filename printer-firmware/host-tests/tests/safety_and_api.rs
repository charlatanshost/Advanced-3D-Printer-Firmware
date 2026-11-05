use firmware_host_tests::config_manager::*;
use firmware_host_tests::power_recovery::*;
use firmware_host_tests::safety::*;
use firmware_host_tests::web_server::*;
use futures::executor::block_on;
use heapless::{String, Vec};

fn request(method: &str, path: &str, token: Option<&str>, body: &str) -> HttpRequest {
    HttpRequest {
        method: String::try_from(method).unwrap(),
        path: String::try_from(path).unwrap(),
        authorization: token.map(|t| {
            let mut header: String<96> = String::try_from("Bearer ").unwrap();
            header.push_str(t).unwrap();
            header
        }),
        body: Vec::from_slice(body.as_bytes()).unwrap(),
    }
}

fn json(response: &HttpResponse) -> serde_json::Value {
    serde_json::from_slice(&response.body).unwrap_or_else(|e| panic!("not JSON ({e}): {:?}", std::str::from_utf8(&response.body)))
}

/// All API checks share the global managers and the emergency stop latch, so they run in one test.
#[test]
fn web_api() {
    block_on(async {
        init_config_manager().await;
        let server = WebServer::new(80, 8080, "secret");
        let call = |method: &'static str, path: &'static str, token: Option<&'static str>, body: &'static str| {
            let server = &server;
            async move { server.handle_http_request(&request(method, path, token, body)).await }
        };

        // The page is delivered whole
        let page = call("GET", "/", None, "").await;
        assert_eq!(page.status_code, 200);
        assert!(page.static_body.unwrap().len() > 30000);

        // Authentication
        assert_eq!(call("GET", "/api/config", None, "").await.status_code, 401);
        assert_eq!(call("GET", "/api/config", Some("wrong"), "").await.status_code, 401);
        assert_eq!(call("POST", "/api/config/reset", None, "").await.status_code, 401);
        assert_eq!(call("GET", "/api/status", None, "").await.status_code, 200);
        // An empty token protects, it does not open
        let no_token = WebServer::new(80, 8080, "");
        assert_eq!(no_token.handle_http_request(&request("GET", "/api/config", Some(""), "")).await.status_code, 401);

        // Status is real JSON, with null for what is not known
        let status = json(&call("GET", "/api/status", None, "").await);
        assert_eq!(status["success"], true);
        assert!(status["data"]["current_temp_hotend"].is_null());
        assert_eq!(status["data"]["emergency_stop"], false);

        // The stored Wi-Fi password never comes back
        {
            let guard = CONFIG_MANAGER.lock().await;
            let manager = guard.as_ref().unwrap();
            let mut config = manager.get_current_config().await;
            config.network.wifi_password = String::try_from("hunter2").unwrap();
            manager.update_config(config).await.unwrap();
        }
        let config_response = call("GET", "/api/config", Some("secret"), "").await;
        assert_eq!(config_response.status_code, 200);
        let config = json(&config_response);
        assert_eq!(config["data"]["network"]["wifi_password"], "");
        assert!(!std::str::from_utf8(&config_response.body).unwrap().contains("hunter2"));

        // The settings form applies, and only what it covers
        const FORM: &str = r#"{"motion":{"max_feedrate":[100,100,5,25],"steps_per_mm":[81,80,400,93]},"temperature":{"hotend_max_temp":260,"bed_max_temp":110,"pid_hotend":{"kp":20,"ki":1,"kd":114,"enabled":true}},"extruder":{"filament_diameter":1.75,"nozzle_diameter":0.6,"retract_length":3},"safety":{"thermal_protection_enabled":true},"features":{"power_loss_recovery":true,"exclude_objects":false}}"#;
        let saved = call("PUT", "/api/config", Some("secret"), FORM).await;
        assert_eq!(saved.status_code, 200, "{:?}", std::str::from_utf8(&saved.body));
        let config = json(&call("GET", "/api/config", Some("secret"), "").await);
        assert_eq!(config["data"]["temperature"]["hotend_max_temp"], 260.0);
        assert_eq!(config["data"]["features"]["exclude_objects"], false);
        assert_eq!(config["data"]["motion"]["max_acceleration"][0], 500.0);

        // Refusals over the API
        const NO_PROTECTION: &str = r#"{"motion":{"max_feedrate":[100,100,5,25],"steps_per_mm":[81,80,400,93]},"temperature":{"hotend_max_temp":260,"bed_max_temp":110,"pid_hotend":{"kp":20,"ki":1,"kd":114,"enabled":true}},"extruder":{"filament_diameter":1.75,"nozzle_diameter":0.6,"retract_length":3},"safety":{"thermal_protection_enabled":false},"features":{"power_loss_recovery":true,"exclude_objects":false}}"#;
        let refused = call("PUT", "/api/config", Some("secret"), NO_PROTECTION).await;
        assert_eq!(refused.status_code, 400);
        assert_eq!(json(&refused)["error"], "Thermal protection cannot be turned off");
        assert_eq!(call("PUT", "/api/config", Some("secret"), "not json").await.status_code, 400);
        assert_eq!(call("PUT", "/api/config/temperature.hotend_max_temp", Some("secret"), r#"{"value":9999}"#).await.status_code, 400);
        assert_eq!(call("PUT", "/api/config/safety.thermal_protection_enabled", Some("secret"), r#"{"value":false}"#).await.status_code, 400);
        assert_eq!(call("PUT", "/api/config/network.web_port", Some("secret"), r#"{"value":70000}"#).await.status_code, 400);
        assert_eq!(call("PUT", "/api/config/temperature.hotend_max_temp", Some("secret"), r#"{"value":270}"#).await.status_code, 200);
        assert_eq!(call("POST", "/api/config/import", Some("secret"), "").await.status_code, 501);

        // Profiles use the name that was sent
        assert_eq!(call("POST", "/api/config/save", Some("secret"), r#"{"profile_name":"PETG"}"#).await.status_code, 200);
        let profiles = json(&call("GET", "/api/profiles", Some("secret"), "").await);
        assert_eq!(profiles["data"], serde_json::json!(["PETG"]));
        assert_eq!(call("POST", "/api/config/load", Some("secret"), r#"{"profile_name":"Nope"}"#).await.status_code, 400);
        assert_eq!(call("DELETE", "/api/profiles/PETG", Some("secret"), "").await.status_code, 200);

        // Reset keeps the network settings
        assert_eq!(call("POST", "/api/config/reset", Some("secret"), "").await.status_code, 200);
        let guard = CONFIG_MANAGER.lock().await;
        assert_eq!(guard.as_ref().unwrap().get_current_config().await.network.wifi_password.as_str(), "hunter2");
        drop(guard);

        // Emergency stop needs no token, works with subsystems locked, and latches
        assert!(!emergency_stop_active());
        let held = CONFIG_MANAGER.lock().await;
        let stop = call("POST", "/api/emergency_stop", None, "").await;
        drop(held);
        assert_eq!(stop.status_code, 200);
        assert!(emergency_stop_active());
        let status = json(&call("GET", "/api/status", None, "").await);
        assert_eq!(status["data"]["emergency_stop"], true);
    });
}

#[test]
fn config_validation() {
    block_on(async {
        let manager = ConfigManager::new();
        let base = PrinterConfiguration::default();
        assert!(manager.update_config(base.clone()).await.is_ok());

        let mut nan_feedrate = base.clone();
        nan_feedrate.motion.max_feedrate[0] = f32::NAN;
        assert!(matches!(manager.update_config(nan_feedrate).await, Err(ConfigError::InvalidMotionConfig)));

        let mut nan_temp = base.clone();
        nan_temp.temperature.hotend_max_temp = f32::NAN;
        assert!(matches!(manager.update_config(nan_temp).await, Err(ConfigError::InvalidTemperatureConfig)));

        let mut bad_volume = base.clone();
        bad_volume.motion.build_volume[0] = [220.0, 0.0];
        assert!(matches!(manager.update_config(bad_volume).await, Err(ConfigError::InvalidMotionConfig)));

        // A rejected single field leaves the configuration unchanged
        let before = manager.get_current_config().await.temperature.hotend_max_temp;
        assert!(manager.set_config_field("temperature.hotend_max_temp", ConfigValue::Float(f32::NAN)).await.is_err());
        assert_eq!(manager.get_current_config().await.temperature.hotend_max_temp, before);
    });
}

#[test]
fn power_recovery_behaviour() {
    block_on(async {
        let mut manager = PowerRecoveryManager::new();

        // Dips are ignored, and a real loss fires once however long it lasts
        assert!(!manager.process_voltage_sample(10.0).await);
        assert!(!manager.process_voltage_sample(12.0).await);
        assert!(!manager.process_voltage_sample(10.0).await);
        assert!(!manager.process_voltage_sample(10.0).await);
        assert!(manager.process_voltage_sample(10.0).await);
        for _ in 0..10 {
            assert!(!manager.process_voltage_sample(10.0).await);
        }
        // And it re-arms when power returns
        assert!(!manager.process_voltage_sample(12.0).await);
        assert!(!manager.process_voltage_sample(10.0).await);
        assert!(!manager.process_voltage_sample(10.0).await);
        assert!(manager.process_voltage_sample(10.0).await);

        // Resuming must not claim success when it cannot do anything
        assert!(matches!(manager.resume_print().await, Err(RecoveryError::NoRecoveryData)));
    });

    // Checksums catch damage
    let mut data = RecoveryData {
        magic: 0xDEADBEEF,
        timestamp: 1,
        printer_state: PrinterState::default(),
        gcode_filename: String::try_from("a.gcode").unwrap(),
        recovery_position: 5,
        checksum: 0,
    };
    assert!(!data.is_valid(), "a checksum of zero is not valid");
    data.checksum = data.signed_checksum().unwrap();
    assert!(data.is_valid());
    data.recovery_position = 6;
    assert!(!data.is_valid());
}
