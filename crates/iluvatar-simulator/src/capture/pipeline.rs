use bevy::prelude::*;
use iluvatar_camera::capture::GrayscaleFrame;
use iluvatar_core::CameraFrame;

use super::components::{CaptureState, RaymarcherInstances};
use super::config::CaptureConfig;
use super::extraction::ExtractedFrame;
use crate::cameras::SimulatedCamera;

/// Output queue for processed camera frames
#[derive(Resource, Default)]
pub struct CapturedFrames {
    pub frames: Vec<CameraFrame>,
}

/// Process extracted frames through FrameProcessor and Raymarcher
pub fn process_captured_frames(
    mut commands: Commands,
    config: Res<CaptureConfig>,
    raymarchers: Res<RaymarcherInstances>,
    mut captured_frames: ResMut<CapturedFrames>,
    mut query: Query<(Entity, &SimulatedCamera, &mut CaptureState, &ExtractedFrame)>,
) {
    for (entity, sim_camera, mut state, extracted) in query.iter_mut() {
        // Clone the grayscale frame (FrameProcessor takes ownership)
        let grayscale = GrayscaleFrame {
            width: extracted.frame.width,
            height: extracted.frame.height,
            data: extracted.frame.data.clone(),
        };

        // Compute difference mask
        let diff_result = state.frame_processor.compute_difference(grayscale);

        if diff_result.is_none() {
            tracing::debug!(
                "Camera {}: compute_difference returned None (first frame, no previous)",
                sim_camera.camera_id
            );
        }

        if let Some(mask) = diff_result {
            let motion_count = mask.motion_count();

            tracing::debug!(
                "Camera {}: motion_count={}, threshold={}",
                sim_camera.camera_id,
                motion_count,
                config.motion_threshold_pixels
            );

            // Only process if sufficient motion detected
            if motion_count as u32 >= config.motion_threshold_pixels {
                // Get raymarcher for this camera
                if let Some(raymarcher) = raymarchers.instances.get(&sim_camera.camera_id) {
                    // Generate voxel contributions
                    let contributions = raymarcher.raymarch(&extracted.pose, &mask);

                    // Build CameraFrame
                    let frame = CameraFrame {
                        camera_id: sim_camera.camera_id,
                        sequence: state.sequence,
                        timestamp: extracted.timestamp,
                        pose: extracted.pose,
                        contributions,
                    };

                    tracing::debug!(
                        "Camera {} frame {}: {} motion pixels, {} contributions",
                        sim_camera.camera_id,
                        state.sequence,
                        motion_count,
                        frame.contributions.len()
                    );

                    captured_frames.frames.push(frame);
                    state.sequence += 1;
                }
            }
        }

        // Clean up extracted frame component
        commands.entity(entity).remove::<ExtractedFrame>();
    }
}

/// Clear the captured frames buffer at the end of each frame
pub fn clear_captured_frames(mut captured_frames: ResMut<CapturedFrames>) {
    captured_frames.frames.clear();
}
