pub mod components;
pub mod config;
pub mod extraction;
pub mod gpu_readback;
pub mod pipeline;

use bevy::prelude::*;

pub use components::{CaptureCamera, CaptureState, RaymarcherInstances};
pub use config::{CaptureConfig, SimulatorOrigin};
pub use extraction::ExtractedFrame;
pub use gpu_readback::{CameraReadback, ReadbackData, ReadbackFrames};
pub use pipeline::CapturedFrames;

pub struct CapturePlugin;

impl Plugin for CapturePlugin {
    fn build(&self, app: &mut App) {
        app
            // Resources
            .init_resource::<CaptureConfig>()
            .init_resource::<SimulatorOrigin>()
            .init_resource::<CapturedFrames>()
            .init_resource::<RaymarcherInstances>()
            .init_resource::<ReadbackFrames>()
            // Observer for GPU readback completion events
            .add_observer(gpu_readback::on_readback_complete)
            // Startup: Create capture infrastructure after cameras exist
            .add_systems(
                Startup,
                components::setup_capture_cameras.after(crate::cameras::spawn_simulated_cameras),
            )
            // Update: Main capture pipeline
            .add_systems(
                Update,
                (
                    // Rate-limited capture marking
                    extraction::mark_cameras_for_capture,
                    // Process readback frames through difference detection
                    extraction::process_readback_frames,
                    // Difference detection + raymarching
                    pipeline::process_captured_frames,
                )
                    .chain(),
            )
            // PostUpdate: Integration with validation
            .add_systems(
                PostUpdate,
                (
                    integrate_with_validation.after(crate::validation::collect_ground_truth),
                    pipeline::clear_captured_frames,
                    gpu_readback::clear_readback_frames,
                )
                    .chain(),
            );
    }
}

/// Integrate captured frames with validation metrics
fn integrate_with_validation(
    captured_frames: Res<CapturedFrames>,
    config: Res<CaptureConfig>,
    mut metrics: ResMut<crate::validation::ValidationMetrics>,
) {
    for frame in &captured_frames.frames {
        metrics.record_captured_frame(frame, &config);
    }
}
