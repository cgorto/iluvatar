//! GPU-to-CPU image readback system using Bevy's built-in Readback component
//!
//! This module provides infrastructure for reading rendered images back from
//! the GPU to CPU memory for processing. It uses Bevy's `Readback` component
//! which handles all the buffer management internally.

use bevy::{
    prelude::*,
    render::gpu_readback::{Readback, ReadbackComplete},
};

/// Data received from GPU readback
#[derive(Debug, Clone)]
pub struct ReadbackData {
    pub camera_id: u32,
    pub width: u32,
    pub height: u32,
    pub data: Vec<u8>,
}

/// Resource storing frames received from GPU readback
#[derive(Resource, Default)]
pub struct ReadbackFrames {
    pub frames: Vec<ReadbackData>,
}

/// Links a Readback entity to its camera
#[derive(Component)]
pub struct CameraReadback {
    pub camera_id: u32,
    pub width: u32,
    pub height: u32,
}

/// Observer that handles ReadbackComplete events and stores the data
pub fn on_readback_complete(
    event: On<ReadbackComplete>,
    query: Query<&CameraReadback>,
    mut readback_frames: ResMut<ReadbackFrames>,
) {
    let entity = event.entity;

    let Ok(camera_readback) = query.get(entity) else {
        return;
    };

    // Get the raw bytes from the readback event data
    let data = event.data.clone();

    tracing::debug!(
        "Received readback for camera {}: {}x{}, {} bytes",
        camera_readback.camera_id,
        camera_readback.width,
        camera_readback.height,
        data.len()
    );

    readback_frames.frames.push(ReadbackData {
        camera_id: camera_readback.camera_id,
        width: camera_readback.width,
        height: camera_readback.height,
        data,
    });
}

/// Clear readback frames at end of frame
pub fn clear_readback_frames(mut readback_frames: ResMut<ReadbackFrames>) {
    readback_frames.frames.clear();
}

/// Spawn a readback entity for a camera's render target
pub fn spawn_camera_readback(
    commands: &mut Commands,
    camera_id: u32,
    image_handle: Handle<Image>,
    width: u32,
    height: u32,
) -> Entity {
    commands
        .spawn((
            Readback::texture(image_handle),
            CameraReadback {
                camera_id,
                width,
                height,
            },
        ))
        .id()
}
