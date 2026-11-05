// New tab implementations for the advanced features
// This will be manually inserted into main.rs

                Tab::AutoInputShaper => {
                    ui.heading("🎯 Automated Input Shaper Calibration");
                    ui.label("Fully automated resonance testing and input shaper configuration with user confirmation");

                    egui::ScrollArea::vertical().id_salt("auto_input_shaper_scroll").show(ui, |ui| {
                        // Global calibration controls
                        ui.group(|ui| {
                            ui.label("🛠 Automated Calibration");

                            ui.horizontal(|ui| {
                                if !self.calibration_running {
                                    if ui.button("🚀 Start Auto-Calibration").clicked() {
                                        // Start automated calibration
                                        if let Ok(test_sequence) = self.auto_input_shaper.start_calibration(self.selected_shaper_axis).await {
                                            self.calibration_running = true;
                                            println!("Started automated input shaper calibration for {} axis", self.selected_shaper_axis);
                                        }
                                    }
                                } else {
                                    ui.colored_label(Color32::YELLOW, "🔬 Calibration in progress...");
                                    if ui.button("⏹ Stop Calibration").clicked() {
                                        self.calibration_running = false;
                                        self.auto_input_shaper.clear_calibration_data(Some(self.selected_shaper_axis));
                                    }
                                }
                            });

                            ui.horizontal(|ui| {
                                ui.label("Test axis:");
                                egui::ComboBox::from_label("Axis")
                                    .selected_text(format!("{}", self.selected_shaper_axis))
                                    .show_ui(ui, |ui| {
                                        ui.selectable_value(&mut self.selected_shaper_axis, 'X', "X Axis");
                                        ui.selectable_value(&mut self.selected_shaper_axis, 'Y', "Y Axis");
                                        ui.selectable_value(&mut self.selected_shaper_axis, 'Z', "Z Axis");
                                    });
                            });
                        });

                        ui.separator();

                        // Show calibration results and user confirmation
                        if let Some(applied_shaper) = self.calibration_ui.render_results_ui(ui) {
                            // User confirmed to apply settings
                            if let Some(config) = self.input_shaper_manager.get_axis_config(self.selected_shaper_axis) {
                                let mut new_config = config.clone();
                                new_config.frequency = applied_shaper.delays[1] * 2.0; // Convert from delay to frequency
                                new_config.shaper_type = match applied_shaper.name.as_str() {
                                    "ZV" => ShaperType::ZV,
                                    "ZVD" => ShaperType::MZV, // Map to closest available type
                                    "MZV" => ShaperType::MZV,
                                    "EI" => ShaperType::EI,
                                    _ => ShaperType::MZV,
                                };
                                self.input_shaper_manager.update_axis_config(self.selected_shaper_axis, new_config);
                                println!("✅ Applied calibrated input shaper settings for {} axis", self.selected_shaper_axis);
                            }
                            self.calibration_running = false;
                        }

                        ui.separator();

                        // Calibration history
                        ui.group(|ui| {
                            ui.label("📊 Calibration History");
                            if let Some(last_cal) = self.auto_input_shaper.get_last_calibration(self.selected_shaper_axis) {
                                ui.label(format!("Last calibration: {} peaks detected", last_cal.resonance_peaks.len()));
                                ui.label(format!("Test duration: {:.1}s", last_cal.test_duration.as_secs_f32()));

                                if !last_cal.resonance_peaks.is_empty() {
                                    ui.collapsing("Peak Details", |ui| {
                                        for (i, peak) in last_cal.resonance_peaks.iter().enumerate() {
                                            ui.label(format!("Peak {}: {:.1} Hz (confidence: {:.0}%)",
                                                    i + 1, peak.frequency, peak.confidence * 100.0));
                                        }
                                    });
                                }
                            } else {
                                ui.label("No calibration history available");
                            }
                        });
                    });
                }

                Tab::VolumetricCompensation => {
                    ui.heading("📐 Volumetric Compensation");
                    ui.label("Advanced volumetric compensation using probing for geometric accuracy across the build volume");

                    egui::ScrollArea::vertical().id_salt("volumetric_compensation_scroll").show(ui, |ui| {
                        // Compensation status
                        ui.group(|ui| {
                            ui.label("🎯 Compensation Status");

                            let status = self.volumetric_compensator.get_compensation_status();
                            ui.horizontal(|ui| {
                                ui.checkbox(&mut self.compensation_enabled, "Enable Volumetric Compensation");
                                if status.is_active && self.compensation_enabled {
                                    ui.colored_label(Color32::GREEN, "✓ Active");
                                } else {
                                    ui.colored_label(Color32::RED, "⚠ Disabled");
                                }
                            });

                            if let Some(map_info) = status.map_info {
                                ui.label(format!("Calibration ID: {}", map_info.calibration_id));
                                ui.label(format!("Probe points: {}", map_info.probe_points));
                                ui.label(format!("Grid points: {}", map_info.grid_points));
                                if map_info.has_thermal_compensation {
                                    ui.colored_label(Color32::BLUE, "🌡 Thermal compensation enabled");
                                }
                            }
                        });

                        ui.separator();

                        // Calibration controls
                        ui.group(|ui| {
                            ui.label("🔧 Calibration Controls");

                            if !self.compensation_calibration_running {
                                if ui.button("🚀 Start Volumetric Calibration").clicked() {
                                    match self.volumetric_compensator.start_calibration() {
                                        Ok(gcode_commands) => {
                                            self.compensation_calibration_running = true;
                                            println!("Started volumetric compensation calibration with {} commands", gcode_commands.len());
                                            // TODO: Execute G-code commands
                                        }
                                        Err(e) => {
                                            println!("Failed to start volumetric calibration: {}", e);
                                        }
                                    }
                                }
                            } else {
                                ui.colored_label(Color32::YELLOW, "🔬 Calibration in progress...");
                                if ui.button("⏹ Stop Calibration").clicked() {
                                    self.compensation_calibration_running = false;
                                }
                            }

                            ui.horizontal(|ui| {
                                ui.label("Current temperature:");
                                ui.label(format!("{:.1}°C", self.volumetric_compensator.get_compensation_status().current_temperature));
                            });
                        });

                        ui.separator();

                        // Configuration settings
                        ui.group(|ui| {
                            ui.label("⚙ Configuration");
                            ui.label("Probe resolution: 25mm grid");
                            ui.label("Build volume: 300×300×400mm");
                            ui.label("Max deviation: 2.0mm");
                        });
                    });
                }

                Tab::SensorFusion => {
                    ui.heading("🔗 Advanced Sensor Fusion");
                    ui.label("Multi-sensor state estimation combining encoders, accelerometers, and other sensors");

                    egui::ScrollArea::vertical().id_salt("sensor_fusion_scroll").show(ui, |ui| {
                        // Fusion status
                        ui.group(|ui| {
                            ui.label("🎯 Sensor Fusion Status");

                            ui.horizontal(|ui| {
                                ui.checkbox(&mut self.fusion_enabled, "Enable Sensor Fusion");
                                if self.fusion_enabled {
                                    ui.colored_label(Color32::GREEN, "✓ Active");
                                } else {
                                    ui.colored_label(Color32::RED, "⚠ Disabled");
                                }
                            });
                        });

                        ui.separator();

                        // Per-axis fusion status
                        ui.group(|ui| {
                            ui.label("📊 Axis Status");

                            for axis in ['X', 'Y', 'Z'] {
                                ui.horizontal(|ui| {
                                    ui.label(format!("{} Axis:", axis));

                                    if let Some(state) = self.sensor_fusion_manager.get_axis_state(axis) {
                                        ui.label(format!("Pos: {:.3}mm", state[0]));
                                        ui.label(format!("Vel: {:.1}mm/s", state[1]));
                                        ui.label(format!("Acc: {:.1}mm/s²", state[2]));

                                        // Health indicator
                                        let health = self.sensor_fusion_manager.get_fusion_statistics()
                                            .sensor_health.get(&axis).copied().unwrap_or(0.0);

                                        let health_color = if health > 0.8 {
                                            Color32::GREEN
                                        } else if health > 0.5 {
                                            Color32::YELLOW
                                        } else {
                                            Color32::RED
                                        };

                                        ui.colored_label(health_color, format!("Health: {:.0}%", health * 100.0));
                                    } else {
                                        ui.colored_label(Color32::RED, "Not initialized");
                                    }
                                });
                            }
                        });

                        ui.separator();

                        // Fusion statistics
                        ui.group(|ui| {
                            ui.label("📈 Fusion Statistics");

                            let stats = self.sensor_fusion_manager.get_fusion_statistics();
                            ui.label(format!("Total measurements: {}", stats.total_measurements));
                            ui.label(format!("Outliers rejected: {}", stats.outliers_rejected));
                            ui.label(format!("Avg fusion time: {:.1}μs", stats.avg_fusion_time.as_micros()));
                            ui.label(format!("Position accuracy: {:.3}mm RMS", stats.position_accuracy));
                            ui.label(format!("Velocity accuracy: {:.1}mm/s RMS", stats.velocity_accuracy));
                        });

                        ui.separator();

                        // Calibration controls
                        ui.group(|ui| {
                            ui.label("🔧 Sensor Calibration");

                            ui.horizontal(|ui| {
                                if ui.button("📊 Calibrate X Axis").clicked() {
                                    match self.sensor_fusion_manager.calibrate_sensors('X') {
                                        Ok(calibration) => {
                                            println!("✅ X axis sensor calibration completed");
                                            println!("  Accel bias: {:?}", calibration.accelerometer_bias);
                                        }
                                        Err(e) => {
                                            println!("❌ X axis calibration failed: {}", e);
                                        }
                                    }
                                }

                                if ui.button("📊 Calibrate Y Axis").clicked() {
                                    match self.sensor_fusion_manager.calibrate_sensors('Y') {
                                        Ok(calibration) => {
                                            println!("✅ Y axis sensor calibration completed");
                                        }
                                        Err(e) => {
                                            println!("❌ Y axis calibration failed: {}", e);
                                        }
                                    }
                                }

                                if ui.button("📊 Calibrate Z Axis").clicked() {
                                    match self.sensor_fusion_manager.calibrate_sensors('Z') {
                                        Ok(calibration) => {
                                            println!("✅ Z axis sensor calibration completed");
                                        }
                                        Err(e) => {
                                            println!("❌ Z axis calibration failed: {}", e);
                                        }
                                    }
                                }
                            });

                            ui.label("Ensure machine is stationary before calibrating sensors");
                        });
                    });
                }

                Tab::Plugins => {
                    ui.heading("🔌 Plugin System");
                    ui.label("Manage and configure plugins to extend printer functionality");

                    egui::ScrollArea::vertical().id_salt("plugins_scroll").show(ui, |ui| {
                        // Plugin management
                        ui.group(|ui| {
                            ui.label("🛠 Plugin Management");

                            ui.horizontal(|ui| {
                                if ui.button("🔄 Refresh Plugins").clicked() {
                                    match self.plugin_manager.scan_and_load_plugins() {
                                        Ok(loaded) => {
                                            println!("✅ Loaded {} plugins", loaded.len());
                                        }
                                        Err(e) => {
                                            println!("❌ Failed to scan plugins: {}", e);
                                        }
                                    }
                                }

                                if ui.button("📂 Open Plugin Directory").clicked() {
                                    // TODO: Open plugin directory in file manager
                                    println!("Opening plugin directory...");
                                }
                            });
                        });

                        ui.separator();

                        // Plugin list
                        ui.group(|ui| {
                            ui.label("📋 Installed Plugins");

                            let plugins = self.plugin_manager.list_plugins();

                            if plugins.is_empty() {
                                ui.label("No plugins installed");
                            } else {
                                for plugin_info in plugins {
                                    ui.horizontal(|ui| {
                                        let is_selected = self.selected_plugin.as_ref() == Some(&plugin_info.manifest.name);

                                        if ui.selectable_label(is_selected, &plugin_info.manifest.name).clicked() {
                                            self.selected_plugin = Some(plugin_info.manifest.name.clone());
                                        }

                                        // Status indicator
                                        if plugin_info.is_enabled {
                                            ui.colored_label(Color32::GREEN, "✓ Enabled");
                                        } else {
                                            ui.colored_label(Color32::RED, "⚠ Disabled");
                                        }

                                        // Plugin type
                                        let type_text = match plugin_info.manifest.plugin_type {
                                            PluginType::Native => "📦 Native",
                                            PluginType::Script => "📜 Script",
                                            PluginType::WebAssembly => "🌐 WASM",
                                            PluginType::Python => "🐍 Python",
                                        };
                                        ui.label(type_text);
                                    });
                                }
                            }
                        });

                        ui.separator();

                        // Selected plugin details
                        if let Some(ref selected_plugin_name) = self.selected_plugin {
                            let plugins = self.plugin_manager.list_plugins();
                            if let Some(plugin_info) = plugins.iter().find(|p| p.manifest.name == *selected_plugin_name) {
                                ui.group(|ui| {
                                    ui.label(format!("🔍 Plugin Details: {}", plugin_info.manifest.name));

                                    ui.label(format!("Version: {}", plugin_info.manifest.version));
                                    ui.label(format!("Author: {}", plugin_info.manifest.author));
                                    ui.label(format!("Description: {}", plugin_info.manifest.description));

                                    ui.separator();

                                    ui.horizontal(|ui| {
                                        if plugin_info.is_enabled {
                                            if ui.button("⏸ Disable Plugin").clicked() {
                                                if let Err(e) = self.plugin_manager.set_plugin_enabled(&plugin_info.manifest.name, false) {
                                                    println!("Failed to disable plugin: {}", e);
                                                }
                                            }
                                        } else {
                                            if ui.button("▶ Enable Plugin").clicked() {
                                                if let Err(e) = self.plugin_manager.set_plugin_enabled(&plugin_info.manifest.name, true) {
                                                    println!("Failed to enable plugin: {}", e);
                                                }
                                            }
                                        }

                                        if ui.button("🗑 Unload Plugin").clicked() {
                                            if let Err(e) = self.plugin_manager.unload_plugin(&plugin_info.manifest.name) {
                                                println!("Failed to unload plugin: {}", e);
                                            } else {
                                                self.selected_plugin = None;
                                            }
                                        }
                                    });

                                    // Plugin statistics
                                    ui.separator();
                                    ui.label("📊 Statistics:");
                                    ui.label(format!("Calls made: {}", plugin_info.execution_stats.calls_made));
                                    ui.label(format!("Errors: {}", plugin_info.execution_stats.errors_count));
                                    ui.label(format!("Last execution: {:.1}ms", plugin_info.execution_stats.last_execution_time.as_millis()));
                                });
                            }
                        }
                    });
                }

                Tab::StateSpaceControl => {
                    ui.heading("🎚 State-Space Control & LQR");
                    ui.label("Modern control theory for multi-axis coordination and optimal performance");

                    egui::ScrollArea::vertical().id_salt("state_space_control_scroll").show(ui, |ui| {
                        // Control system status
                        ui.group(|ui| {
                            ui.label("🎯 Control System Status");

                            ui.horizontal(|ui| {
                                ui.checkbox(&mut self.state_space_enabled, "Enable State-Space Control");
                                if self.state_space_enabled {
                                    ui.colored_label(Color32::GREEN, "✓ Active");
                                } else {
                                    ui.colored_label(Color32::RED, "⚠ Using PID Control");
                                }
                            });

                            if self.state_space_enabled {
                                let controller_status = self.state_space_controller.get_status();
                                ui.label(format!("Controller type: {}", controller_status.controller_type));
                                ui.label(format!("Update rate: {:.0} Hz", controller_status.update_rate));
                                ui.label(format!("Computational load: {:.1}%", controller_status.cpu_usage * 100.0));
                            }
                        });

                        ui.separator();

                        // System identification
                        ui.group(|ui| {
                            ui.label("🔬 System Identification");

                            ui.horizontal(|ui| {
                                if !self.system_id_running {
                                    if ui.button("🚀 Start System ID").clicked() {
                                        match self.state_space_controller.start_system_identification() {
                                            Ok(_) => {
                                                self.system_id_running = true;
                                                println!("Started automatic system identification");
                                            }
                                            Err(e) => {
                                                println!("Failed to start system ID: {}", e);
                                            }
                                        }
                                    }
                                } else {
                                    ui.colored_label(Color32::YELLOW, "🔬 Identification in progress...");
                                    if ui.button("⏹ Stop System ID").clicked() {
                                        self.system_id_running = false;
                                    }
                                }
                            });

                            if let Some(system_params) = self.state_space_controller.get_identified_parameters() {
                                ui.collapsing("System Parameters", |ui| {
                                    ui.label(format!("Natural frequency: {:.1} Hz", system_params.natural_frequency));
                                    ui.label(format!("Damping ratio: {:.3}", system_params.damping_ratio));
                                    ui.label(format!("Time constant: {:.3} s", system_params.time_constant));
                                    ui.label(format!("Identification confidence: {:.0}%", system_params.confidence * 100.0));
                                });
                            }
                        });

                        ui.separator();

                        // LQR Controller Design
                        ui.group(|ui| {
                            ui.label("⚙ LQR Controller Design");

                            ui.horizontal(|ui| {
                                ui.label("Position weight:");
                                ui.add(egui::Slider::new(&mut self.lqr_position_weight, 1.0..=10000.0)
                                    .logarithmic(true)
                                    .text("Q_pos"));
                            });

                            ui.horizontal(|ui| {
                                ui.label("Velocity weight:");
                                ui.add(egui::Slider::new(&mut self.lqr_velocity_weight, 0.1..=1000.0)
                                    .logarithmic(true)
                                    .text("Q_vel"));
                            });

                            ui.horizontal(|ui| {
                                ui.label("Control effort weight:");
                                ui.add(egui::Slider::new(&mut self.lqr_control_weight, 0.01..=100.0)
                                    .logarithmic(true)
                                    .text("R"));
                            });

                            if ui.button("🔄 Redesign Controller").clicked() {
                                let lqr_config = LQRConfig {
                                    position_weight: self.lqr_position_weight,
                                    velocity_weight: self.lqr_velocity_weight,
                                    control_weight: self.lqr_control_weight,
                                };

                                match self.state_space_controller.redesign_lqr_controller(lqr_config) {
                                    Ok(gains) => {
                                        println!("✅ LQR controller redesigned successfully");
                                        println!("  Position gain: {:.3}", gains.position_gain);
                                        println!("  Velocity gain: {:.3}", gains.velocity_gain);
                                    }
                                    Err(e) => {
                                        println!("❌ LQR redesign failed: {}", e);
                                    }
                                }
                            }
                        });

                        ui.separator();

                        // Performance monitoring
                        ui.group(|ui| {
                            ui.label("📊 Performance Monitoring");

                            if let Some(performance) = self.state_space_controller.get_performance_metrics() {
                                ui.horizontal(|ui| {
                                    ui.label("Position error RMS:");
                                    ui.label(format!("{:.4} mm", performance.position_error_rms));
                                });

                                ui.horizontal(|ui| {
                                    ui.label("Control effort RMS:");
                                    ui.label(format!("{:.2}%", performance.control_effort_rms * 100.0));
                                });

                                ui.horizontal(|ui| {
                                    ui.label("Settling time:");
                                    ui.label(format!("{:.1} ms", performance.settling_time.as_millis()));
                                });

                                ui.horizontal(|ui| {
                                    ui.label("Overshoot:");
                                    ui.label(format!("{:.1}%", performance.overshoot_percent));
                                });
                            } else {
                                ui.label("No performance data available");
                            }
                        });

                        ui.separator();

                        // Observer status
                        ui.group(|ui| {
                            ui.label("👁 State Observer (Kalman Filter)");

                            for axis in ['X', 'Y', 'Z'] {
                                if let Some(observer_state) = self.state_space_controller.get_observer_state(axis) {
                                    ui.horizontal(|ui| {
                                        ui.label(format!("{} Axis:", axis));
                                        ui.label(format!("Est. Pos: {:.3}mm", observer_state.estimated_position));
                                        ui.label(format!("Est. Vel: {:.1}mm/s", observer_state.estimated_velocity));
                                        ui.label(format!("Confidence: {:.0}%", observer_state.confidence * 100.0));
                                    });
                                }
                            }

                            ui.horizontal(|ui| {
                                ui.label("Process noise:");
                                ui.add(egui::Slider::new(&mut self.kalman_process_noise, 0.001..=1.0)
                                    .logarithmic(true)
                                    .text("Q"));
                            });

                            ui.horizontal(|ui| {
                                ui.label("Measurement noise:");
                                ui.add(egui::Slider::new(&mut self.kalman_measurement_noise, 0.001..=1.0)
                                    .logarithmic(true)
                                    .text("R"));
                            });

                            if ui.button("🔄 Update Observer").clicked() {
                                let observer_config = KalmanFilterConfig {
                                    process_noise: self.kalman_process_noise,
                                    measurement_noise: self.kalman_measurement_noise,
                                };

                                if let Err(e) = self.state_space_controller.update_observer_config(observer_config) {
                                    println!("Failed to update observer: {}", e);
                                }
                            }
                        });
                    });
                }

                Tab::ModelPredictiveControl => {
                    ui.heading("🔮 Model Predictive Control (MPC)");
                    ui.label("Advanced predictive control with lookahead optimization for maximum performance");

                    egui::ScrollArea::vertical().id_salt("mpc_scroll").show(ui, |ui| {
                        // MPC status
                        ui.group(|ui| {
                            ui.label("🎯 MPC Status");

                            ui.horizontal(|ui| {
                                ui.checkbox(&mut self.mpc_enabled, "Enable Model Predictive Control");
                                if self.mpc_enabled {
                                    ui.colored_label(Color32::GREEN, "✓ Active");
                                } else {
                                    ui.colored_label(Color32::RED, "⚠ Using Fallback Control");
                                }
                            });

                            if self.mpc_enabled {
                                let mpc_status = self.mpc_controller.get_status();
                                ui.label(format!("Prediction horizon: {:.0}ms", mpc_status.prediction_horizon.as_millis()));
                                ui.label(format!("Control horizon: {:.0}ms", mpc_status.control_horizon.as_millis()));
                                ui.label(format!("Solver: {}", mpc_status.solver_type));
                                ui.label(format!("Solve time: {:.1}μs avg", mpc_status.avg_solve_time.as_micros()));
                            }
                        });

                        ui.separator();

                        // Horizon configuration
                        ui.group(|ui| {
                            ui.label("⏰ Prediction Horizons");

                            ui.horizontal(|ui| {
                                ui.label("Prediction horizon:");
                                ui.add(egui::Slider::new(&mut self.prediction_horizon_ms, 10.0..=200.0)
                                    .suffix(" ms")
                                    .text("N"));
                            });

                            ui.horizontal(|ui| {
                                ui.label("Control horizon:");
                                ui.add(egui::Slider::new(&mut self.control_horizon_ms, 5.0..=100.0)
                                    .suffix(" ms")
                                    .text("M"));
                            });

                            ui.horizontal(|ui| {
                                ui.label("Lookahead distance:");
                                ui.add(egui::Slider::new(&mut self.lookahead_distance_mm, 1.0..=50.0)
                                    .suffix(" mm")
                                    .text("Path"));
                            });

                            if ui.button("🔄 Update Horizons").clicked() {
                                let horizon_config = MPCHorizonConfig {
                                    prediction_horizon: Duration::from_millis(self.prediction_horizon_ms as u64),
                                    control_horizon: Duration::from_millis(self.control_horizon_ms as u64),
                                    lookahead_distance: self.lookahead_distance_mm,
                                };

                                if let Err(e) = self.mpc_controller.update_horizons(horizon_config) {
                                    println!("Failed to update MPC horizons: {}", e);
                                }
                            }
                        });

                        ui.separator();

                        // Optimization weights
                        ui.group(|ui| {
                            ui.label("⚖ Optimization Weights");

                            ui.horizontal(|ui| {
                                ui.label("Tracking weight:");
                                ui.add(egui::Slider::new(&mut self.mpc_tracking_weight, 1.0..=1000.0)
                                    .logarithmic(true)
                                    .text("Q"));
                            });

                            ui.horizontal(|ui| {
                                ui.label("Control effort weight:");
                                ui.add(egui::Slider::new(&mut self.mpc_control_weight, 0.01..=100.0)
                                    .logarithmic(true)
                                    .text("R"));
                            });

                            ui.horizontal(|ui| {
                                ui.label("Smoothness weight:");
                                ui.add(egui::Slider::new(&mut self.mpc_smoothness_weight, 0.1..=100.0)
                                    .logarithmic(true)
                                    .text("S"));
                            });

                            if ui.button("🔄 Update Weights").clicked() {
                                let weights_config = MPCWeightsConfig {
                                    tracking_weight: self.mpc_tracking_weight,
                                    control_effort_weight: self.mpc_control_weight,
                                    smoothness_weight: self.mpc_smoothness_weight,
                                };

                                if let Err(e) = self.mpc_controller.update_weights(weights_config) {
                                    println!("Failed to update MPC weights: {}", e);
                                }
                            }
                        });

                        ui.separator();

                        // Constraints
                        ui.group(|ui| {
                            ui.label("🚧 System Constraints");

                            ui.collapsing("Position Limits", |ui| {
                                ui.horizontal(|ui| {
                                    ui.label("X limits:");
                                    ui.add(egui::DragValue::new(&mut self.mpc_x_min).speed(1.0).suffix(" mm"));
                                    ui.label("to");
                                    ui.add(egui::DragValue::new(&mut self.mpc_x_max).speed(1.0).suffix(" mm"));
                                });

                                ui.horizontal(|ui| {
                                    ui.label("Y limits:");
                                    ui.add(egui::DragValue::new(&mut self.mpc_y_min).speed(1.0).suffix(" mm"));
                                    ui.label("to");
                                    ui.add(egui::DragValue::new(&mut self.mpc_y_max).speed(1.0).suffix(" mm"));
                                });

                                ui.horizontal(|ui| {
                                    ui.label("Z limits:");
                                    ui.add(egui::DragValue::new(&mut self.mpc_z_min).speed(1.0).suffix(" mm"));
                                    ui.label("to");
                                    ui.add(egui::DragValue::new(&mut self.mpc_z_max).speed(1.0).suffix(" mm"));
                                });
                            });

                            ui.collapsing("Velocity Limits", |ui| {
                                ui.horizontal(|ui| {
                                    ui.label("Max velocity X:");
                                    ui.add(egui::DragValue::new(&mut self.mpc_max_vel_x).speed(1.0).suffix(" mm/s"));
                                });

                                ui.horizontal(|ui| {
                                    ui.label("Max velocity Y:");
                                    ui.add(egui::DragValue::new(&mut self.mpc_max_vel_y).speed(1.0).suffix(" mm/s"));
                                });

                                ui.horizontal(|ui| {
                                    ui.label("Max velocity Z:");
                                    ui.add(egui::DragValue::new(&mut self.mpc_max_vel_z).speed(1.0).suffix(" mm/s"));
                                });
                            });

                            ui.collapsing("Acceleration Limits", |ui| {
                                ui.horizontal(|ui| {
                                    ui.label("Max acceleration X:");
                                    ui.add(egui::DragValue::new(&mut self.mpc_max_accel_x).speed(100.0).suffix(" mm/s²"));
                                });

                                ui.horizontal(|ui| {
                                    ui.label("Max acceleration Y:");
                                    ui.add(egui::DragValue::new(&mut self.mpc_max_accel_y).speed(100.0).suffix(" mm/s²"));
                                });

                                ui.horizontal(|ui| {
                                    ui.label("Max acceleration Z:");
                                    ui.add(egui::DragValue::new(&mut self.mpc_max_accel_z).speed(100.0).suffix(" mm/s²"));
                                });
                            });

                            if ui.button("🔄 Update Constraints").clicked() {
                                let constraints_config = MPCConstraintsConfig {
                                    position_bounds: [
                                        (self.mpc_x_min, self.mpc_x_max),
                                        (self.mpc_y_min, self.mpc_y_max),
                                        (self.mpc_z_min, self.mpc_z_max),
                                    ],
                                    velocity_bounds: [
                                        (-self.mpc_max_vel_x, self.mpc_max_vel_x),
                                        (-self.mpc_max_vel_y, self.mpc_max_vel_y),
                                        (-self.mpc_max_vel_z, self.mpc_max_vel_z),
                                    ],
                                    acceleration_bounds: [
                                        (-self.mpc_max_accel_x, self.mpc_max_accel_x),
                                        (-self.mpc_max_accel_y, self.mpc_max_accel_y),
                                        (-self.mpc_max_accel_z, self.mpc_max_accel_z),
                                    ],
                                };

                                if let Err(e) = self.mpc_controller.update_constraints(constraints_config) {
                                    println!("Failed to update MPC constraints: {}", e);
                                }
                            }
                        });

                        ui.separator();

                        // Solver configuration
                        ui.group(|ui| {
                            ui.label("🧮 Solver Configuration");

                            ui.horizontal(|ui| {
                                ui.label("Solver type:");
                                egui::ComboBox::from_label("Solver")
                                    .selected_text(format!("{:?}", self.selected_mpc_solver))
                                    .show_ui(ui, |ui| {
                                        ui.selectable_value(&mut self.selected_mpc_solver, MPCSolverType::OSQP, "OSQP (Recommended)");
                                        ui.selectable_value(&mut self.selected_mpc_solver, MPCSolverType::ClarabelRs, "Clarabel.rs");
                                        ui.selectable_value(&mut self.selected_mpc_solver, MPCSolverType::SCS, "SCS");
                                    });
                            });

                            ui.horizontal(|ui| {
                                ui.label("Max iterations:");
                                ui.add(egui::DragValue::new(&mut self.mpc_max_iterations).speed(1.0).clamp_range(10..=1000));
                            });

                            ui.horizontal(|ui| {
                                ui.label("Tolerance:");
                                ui.add(egui::DragValue::new(&mut self.mpc_tolerance).speed(1e-8).clamp_range(1e-10..=1e-3));
                            });

                            ui.horizontal(|ui| {
                                ui.checkbox(&mut self.mpc_warm_start, "Warm start");
                                ui.checkbox(&mut self.mpc_polish, "Polish solution");
                            });

                            if ui.button("🔄 Update Solver").clicked() {
                                let solver_config = MPCSolverConfig {
                                    solver_type: self.selected_mpc_solver.clone(),
                                    max_iterations: self.mpc_max_iterations,
                                    tolerance: self.mpc_tolerance,
                                    warm_start: self.mpc_warm_start,
                                    polish: self.mpc_polish,
                                };

                                if let Err(e) = self.mpc_controller.update_solver_config(solver_config) {
                                    println!("Failed to update MPC solver: {}", e);
                                }
                            }
                        });

                        ui.separator();

                        // Performance monitoring
                        ui.group(|ui| {
                            ui.label("📊 MPC Performance");

                            if let Some(performance) = self.mpc_controller.get_performance_metrics() {
                                ui.horizontal(|ui| {
                                    ui.label("Avg solve time:");
                                    ui.label(format!("{:.1} μs", performance.avg_solve_time.as_micros()));
                                    if performance.avg_solve_time.as_micros() > 800 {
                                        ui.colored_label(Color32::RED, "⚠ Slow");
                                    } else {
                                        ui.colored_label(Color32::GREEN, "✓ Fast");
                                    }
                                });

                                ui.horizontal(|ui| {
                                    ui.label("Max solve time:");
                                    ui.label(format!("{:.1} μs", performance.max_solve_time.as_micros()));
                                });

                                ui.horizontal(|ui| {
                                    ui.label("Solver success rate:");
                                    ui.label(format!("{:.1}%", performance.solver_success_rate * 100.0));
                                });

                                ui.horizontal(|ui| {
                                    ui.label("Path following error:");
                                    ui.label(format!("{:.4} mm RMS", performance.path_following_error));
                                });

                                ui.horizontal(|ui| {
                                    ui.label("Predicted speedup:");
                                    ui.label(format!("{:.0}%", performance.predicted_speedup * 100.0));
                                });
                            } else {
                                ui.label("No performance data available");
                            }
                        });

                        ui.separator();

                        // Advanced features
                        ui.group(|ui| {
                            ui.label("🚀 Advanced Features");

                            ui.horizontal(|ui| {
                                ui.checkbox(&mut self.mpc_adaptive_learning, "Adaptive model learning");
                                ui.checkbox(&mut self.mpc_thermal_compensation, "Thermal compensation");
                            });

                            ui.horizontal(|ui| {
                                ui.checkbox(&mut self.mpc_vibration_prediction, "Vibration prediction");
                                ui.checkbox(&mut self.mpc_corner_optimization, "Corner optimization");
                            });

                            if ui.button("🔄 Update Advanced Features").clicked() {
                                let advanced_config = MPCAdvancedConfig {
                                    adaptive_learning: self.mpc_adaptive_learning,
                                    thermal_compensation: self.mpc_thermal_compensation,
                                    vibration_prediction: self.mpc_vibration_prediction,
                                    corner_optimization: self.mpc_corner_optimization,
                                };

                                if let Err(e) = self.mpc_controller.update_advanced_features(advanced_config) {
                                    println!("Failed to update MPC advanced features: {}", e);
                                }
                            }
                        });
                    });
                }