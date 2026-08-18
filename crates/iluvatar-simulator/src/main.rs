use bevy::{asset::AssetPlugin, prelude::*};
use iluvatar_simulator::{RenderSimulatorPlugin, SimulatorPlugin, SimulatorTomlConfig};
use std::path::Path;

fn main() {
    // Parse command-line args
    let args: Vec<String> = std::env::args().collect();
    let use_render_mode = args.iter().any(|a| a == "--render" || a == "-r");

    // Find --config / -c value
    let config_path = args
        .windows(2)
        .find(|w| w[0] == "--config" || w[0] == "-c")
        .map(|w| w[1].clone());

    let mut app = App::new();

    app.add_plugins(
        DefaultPlugins
            .set(WindowPlugin {
                primary_window: Some(Window {
                    title: if use_render_mode {
                        "Iluvatar Simulator (Render Mode)".to_string()
                    } else {
                        "Iluvatar Simulator (Geometric Mode)".to_string()
                    },
                    resolution: (1280u32, 720u32).into(),
                    ..default()
                }),
                ..default()
            })
            .set(AssetPlugin {
                // Bevy resolves relative asset paths from the executable, which puts
                // debug builds under target/. Anchor this development simulator to
                // its crate instead so `cargo run` works from any directory.
                file_path: format!("{}/assets", env!("CARGO_MANIFEST_DIR")),
                ..default()
            }),
    );

    // Load simulator TOML config if provided
    if let Some(path) = &config_path {
        match SimulatorTomlConfig::load(Path::new(path)) {
            Ok(config) => {
                println!(
                    "Loaded simulator config from {} ({} cameras)",
                    path,
                    config.cameras.len()
                );
                app.insert_resource(config);
            }
            Err(e) => {
                eprintln!("Failed to load config from {}: {}", path, e);
                std::process::exit(1);
            }
        }
    } else {
        // Try default path
        let default_path = "config/simulator.toml";
        if Path::new(default_path).exists() {
            match SimulatorTomlConfig::load(Path::new(default_path)) {
                Ok(config) => {
                    println!(
                        "Loaded simulator config from {} ({} cameras)",
                        default_path,
                        config.cameras.len()
                    );
                    app.insert_resource(config);
                }
                Err(e) => {
                    println!(
                        "Warning: failed to load {}: {}. Using defaults.",
                        default_path, e
                    );
                }
            }
        } else {
            println!("No config file specified, using default camera layout.");
        }
    }

    if use_render_mode {
        println!("=== Running in RENDER mode ===");
        println!("Cameras actually render the scene and detect motion via pixel changes.");
        println!("This is more realistic but slower.");
        app.add_plugins(RenderSimulatorPlugin);
    } else {
        println!("=== Running in GEOMETRIC mode ===");
        println!("Cameras use geometric projection (faster, idealized).");
        println!("Use --render or -r flag for render-based detection.");
        app.add_plugins(SimulatorPlugin);
    }

    app.run();
}
