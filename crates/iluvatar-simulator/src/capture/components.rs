use bevy::{
    prelude::*,
    render::render_resource::{TextureFormat, TextureUsages},
};
use iluvatar_camera::{difference::FrameProcessor, raymarch::Raymarcher};
use iluvatar_core::CameraId;
use std::collections::HashMap;

use super::config::CaptureConfig;
use super::gpu_readback::spawn_camera_readback;
use crate::cameras::SimulatedCamera;

/// Links a SimulatedCamera to its render infrastructure
#[derive(Component)]
pub struct CaptureCamera {
    pub render_target: Handle<Image>,
    pub readback_entity: Entity,
}

/// Marks the Camera3d entity that renders for a SimulatedCamera
#[derive(Component)]
pub struct SimulatedCameraRenderer;

/// Per-camera processing state
#[derive(Component)]
pub struct CaptureState {
    pub frame_processor: FrameProcessor,
    pub sequence: u64,
    pub last_capture_time: f64,
}

/// Per-camera Raymarcher instances (stored separately due to size)
#[derive(Resource, Default)]
pub struct RaymarcherInstances {
    pub instances: HashMap<CameraId, Raymarcher>,
}

/// Set up capture cameras for all simulated cameras
pub fn setup_capture_cameras(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    config: Res<CaptureConfig>,
    origin: Res<super::config::SimulatorOrigin>,
    simulated_cameras: Query<(Entity, &SimulatedCamera)>,
    mut raymarchers: ResMut<RaymarcherInstances>,
) {
    for (entity, sim_camera) in simulated_cameras.iter() {
        // Create render target image with COPY_SRC for GPU readback
        let mut image = Image::new_target_texture(
            config.render_width,
            config.render_height,
            TextureFormat::Rgba8Unorm,
            Some(TextureFormat::Rgba8UnormSrgb),
        );
        // Add COPY_SRC so Bevy's Readback can copy to a buffer for CPU readback
        image.texture_descriptor.usage |= TextureUsages::COPY_SRC;

        let image_handle = images.add(image);

        // Spawn Readback entity for GPU-to-CPU readback using Bevy's built-in system
        let readback_entity = spawn_camera_readback(
            &mut commands,
            sim_camera.camera_id as u32,
            image_handle.clone(),
            config.render_width,
            config.render_height,
        );

        // Add capture components and child render camera to simulated camera
        // The render camera is parented so it automatically inherits the transform
        commands.entity(entity).insert((
            CaptureCamera {
                render_target: image_handle.clone(),
                readback_entity,
            },
            CaptureState {
                frame_processor: FrameProcessor::new(config.difference_threshold),
                sequence: 0,
                last_capture_time: 0.0,
            },
            children![(
                // Child render camera - inherits parent transform automatically
                Camera3d::default(),
                Camera {
                    order: -1, // Render before main camera
                    ..default()
                },
                bevy::camera::RenderTarget::Image(image_handle.into()),
                SimulatedCameraRenderer,
            )],
        ));

        // Create raymarcher for this camera
        let raymarcher = Raymarcher::new(
            sim_camera.intrinsics,
            config.raymarch_config.clone(),
            config.grid_bounds,
            config.voxel_size,
            origin.geo_position,
        );
        raymarchers
            .instances
            .insert(sim_camera.camera_id, raymarcher);

        tracing::info!(
            "Set up capture for camera {} with {}x{} render target (using Bevy Readback)",
            sim_camera.camera_id,
            config.render_width,
            config.render_height
        );
    }
}
