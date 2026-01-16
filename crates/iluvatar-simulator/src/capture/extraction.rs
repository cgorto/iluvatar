use bevy::prelude::*;
use iluvatar_camera::capture::GrayscaleFrame;
use iluvatar_core::{CameraPose, GeoPosition, LocalizationStatus, PoseUncertainty};
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

use super::components::{CaptureCamera, CaptureState};
use super::config::{CaptureConfig, SimulatorOrigin};
use super::gpu_readback::ReadbackFrames;
use crate::cameras::SimulatedCamera;

/// Convert Bevy's Quat to glam's Quat
fn bevy_to_glam_quat(q: bevy::prelude::Quat) -> glam::Quat {
    glam::Quat::from_xyzw(q.x, q.y, q.z, q.w)
}

/// Marker for cameras ready for pixel extraction this frame
#[derive(Component)]
pub struct PendingExtraction;

/// Temporary storage for extracted frame data
#[derive(Component)]
pub struct ExtractedFrame {
    pub frame: GrayscaleFrame,
    pub pose: CameraPose,
    pub timestamp: u64,
}

/// Get current time in microseconds since UNIX epoch
fn now_micros() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_micros() as u64
}

/// Convert Bevy Transform to CameraPose
///
/// Bevy uses Y-up coordinate system, while ENU uses Z-up:
/// - Bevy X -> ENU East (X)
/// - Bevy Z -> ENU North (Y)
/// - Bevy Y -> ENU Up (Z)
fn transform_to_pose(transform: &Transform, origin: &GeoPosition, timestamp: u64) -> CameraPose {
    // Convert Bevy Y-up to ENU Z-up
    let enu_position = glam::Vec3::new(
        transform.translation.x, // East
        transform.translation.z, // North (Bevy Z)
        transform.translation.y, // Up (Bevy Y)
    );

    let geo_position = GeoPosition::from_local_enu(enu_position, origin);

    CameraPose {
        position: geo_position,
        orientation: bevy_to_glam_quat(transform.rotation),
        timestamp,
        uncertainty: PoseUncertainty::default(),
        status: LocalizationStatus::Nominal,
    }
}

/// Mark cameras that should capture this frame (rate limiting)
pub fn mark_cameras_for_capture(
    mut commands: Commands,
    time: Res<Time>,
    config: Res<CaptureConfig>,
    cameras: Query<(Entity, &CaptureState), With<CaptureCamera>>,
) {
    let current_time = time.elapsed_secs_f64();
    let capture_interval = 1.0 / config.capture_rate as f64;

    for (entity, state) in cameras.iter() {
        if current_time - state.last_capture_time >= capture_interval {
            commands.entity(entity).insert(PendingExtraction);
        }
    }
}

/// Process GPU readback frames and convert to grayscale ExtractedFrames
pub fn process_readback_frames(
    mut commands: Commands,
    time: Res<Time>,
    readback_frames: Res<ReadbackFrames>,
    origin: Res<SimulatorOrigin>,
    mut cameras: Query<
        (Entity, &SimulatedCamera, &Transform, &mut CaptureState),
        With<PendingExtraction>,
    >,
) {
    let timestamp = now_micros();
    let current_time = time.elapsed_secs_f64();

    // Build a map of camera_id -> readback data for quick lookup
    let readback_map: HashMap<u32, _> = readback_frames
        .frames
        .iter()
        .map(|f| (f.camera_id, f))
        .collect();

    for (entity, sim_camera, transform, mut state) in cameras.iter_mut() {
        let camera_id = sim_camera.camera_id as u32;

        // Find readback data for this camera
        let Some(readback) = readback_map.get(&camera_id) else {
            // No readback data yet for this camera - happens on first frame
            continue;
        };

        // Debug: check if the image has any non-zero content
        let non_zero_pixels = readback
            .data
            .chunks_exact(4)
            .filter(|c| c[0] > 0 || c[1] > 0 || c[2] > 0)
            .count();

        tracing::debug!(
            "Camera {}: readback {}x{} image, {} non-zero pixels out of {}",
            camera_id,
            readback.width,
            readback.height,
            non_zero_pixels,
            readback.data.len() / 4
        );

        // Convert RGBA to grayscale using ITU-R BT.601 luminance formula
        let mut grayscale = GrayscaleFrame::new(readback.width, readback.height);
        for (i, chunk) in readback.data.chunks_exact(4).enumerate() {
            let r = chunk[0] as f32;
            let g = chunk[1] as f32;
            let b = chunk[2] as f32;
            let luma = (0.299 * r + 0.587 * g + 0.114 * b) as u8;
            grayscale.data[i] = luma;
        }

        // Build CameraPose from Transform
        let pose = transform_to_pose(transform, &origin.geo_position, timestamp);

        // Store extracted frame
        commands.entity(entity).insert(ExtractedFrame {
            frame: grayscale,
            pose,
            timestamp,
        });

        // Update timing
        state.last_capture_time = current_time;

        // Remove pending marker
        commands.entity(entity).remove::<PendingExtraction>();
    }
}
