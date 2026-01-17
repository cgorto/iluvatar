use bevy::prelude::*;
use iluvatar_simulator::SimulatorPlugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Iluvatar Simulator".to_string(),
                resolution: (1280u32, 720u32).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(SimulatorPlugin)
        .run();
}
