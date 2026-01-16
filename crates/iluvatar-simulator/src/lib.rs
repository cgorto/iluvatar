pub mod cameras;
pub mod capture;
pub mod scene;
pub mod targets;
pub mod validation;

use bevy::prelude::*;

// Re-export commonly used types
pub use targets::{
    ActiveScenario, BezierPath, LoopMode, MotionSpec, PathFollower, SimulatedTarget, TargetSpec,
    TestScenario,
};
pub use validation::ValidationMetrics;

pub struct SimulatorPlugin;

impl Plugin for SimulatorPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(scene::ScenePlugin)
            .add_plugins(targets::TargetsPlugin)
            .add_plugins(cameras::CamerasPlugin)
            .add_plugins(capture::CapturePlugin)
            .add_plugins(validation::ValidationPlugin);
    }
}
