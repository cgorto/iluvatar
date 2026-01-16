use bevy::prelude::*;
use glam::Vec3;
use iluvatar_core::{BoundingBox, GeoPosition, RaymarchConfig};

/// Global capture configuration
#[derive(Resource)]
pub struct CaptureConfig {
    /// Resolution for render targets (can be lower than camera intrinsics for performance)
    pub render_width: u32,
    pub render_height: u32,

    /// Difference threshold for motion detection (pixel intensity change)
    pub difference_threshold: u8,

    /// Minimum motion pixels to generate contributions
    pub motion_threshold_pixels: u32,

    /// Grid bounds for raymarching (local ENU coordinates)
    pub grid_bounds: BoundingBox,

    /// Voxel size for raymarch grid
    pub voxel_size: f32,

    /// Raymarch configuration
    pub raymarch_config: RaymarchConfig,

    /// Target capture rate (Hz) - independent of render framerate
    pub capture_rate: f32,
}

impl Default for CaptureConfig {
    fn default() -> Self {
        Self {
            render_width: 640,
            render_height: 360,
            difference_threshold: 5,     // Lowered for debugging (was 20)
            motion_threshold_pixels: 10, // Lowered for debugging (was 100)
            grid_bounds: BoundingBox::new(
                Vec3::new(-200.0, -200.0, 0.0), // ENU: East, North, Up
                Vec3::new(200.0, 200.0, 150.0),
            ),
            voxel_size: 1.0,
            raymarch_config: RaymarchConfig::default(),
            capture_rate: 10.0,
        }
    }
}

/// Geo origin for coordinate conversion (local ENU -> GeoPosition)
#[derive(Resource)]
pub struct SimulatorOrigin {
    pub geo_position: GeoPosition,
}

impl Default for SimulatorOrigin {
    fn default() -> Self {
        Self {
            // Default to Seattle coordinates
            geo_position: GeoPosition::new(47.6062, -122.3321, 0.0),
        }
    }
}
