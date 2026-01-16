use bevy::prelude::*;
use iluvatar_simulator::{ActiveScenario, SimulatorPlugin, TestScenario};

fn main() {
    // Parse command line for scenario selection
    let args: Vec<String> = std::env::args().collect();
    let scenario = parse_scenario(&args);

    println!("Starting Iluvatar Simulator");
    println!("Scenario: {}", scenario.name);
    println!("Targets: {}", scenario.targets.len());
    println!("Duration: {}s", scenario.duration);
    println!();
    println!("Available scenarios:");
    for s in TestScenario::all() {
        println!("  - {}", s.name);
    }
    println!();
    println!("Use --scenario <name> to select a different scenario");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: format!("Iluvatar Simulator - {}", scenario.name),
                resolution: (1280u32, 720u32).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ActiveScenario {
            scenario,
            start_time: 0.0,
        })
        .add_plugins(SimulatorPlugin)
        .run();
}

fn parse_scenario(args: &[String]) -> TestScenario {
    // Look for --scenario <name> argument
    for i in 0..args.len() {
        if args[i] == "--scenario" || args[i] == "-s" {
            if let Some(name) = args.get(i + 1) {
                if let Some(scenario) = TestScenario::by_name(name) {
                    return scenario;
                } else {
                    eprintln!("Unknown scenario: {}", name);
                    eprintln!("Available scenarios:");
                    for s in TestScenario::all() {
                        eprintln!("  - {}", s.name);
                    }
                    std::process::exit(1);
                }
            }
        }
    }

    // Default scenario
    TestScenario::crossing_paths()
}
