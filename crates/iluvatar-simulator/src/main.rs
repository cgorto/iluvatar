use bevy::prelude::*;
use iluvatar_simulator::{RenderSimulatorPlugin, SimulatorPlugin};

fn main() {
    // Check command-line args to determine which mode to run
    let args: Vec<String> = std::env::args().collect();
    let use_render_mode = args.iter().any(|a| a == "--render" || a == "-r");

    let mut app = App::new();

    app.add_plugins(DefaultPlugins.set(WindowPlugin {
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
    }));

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
