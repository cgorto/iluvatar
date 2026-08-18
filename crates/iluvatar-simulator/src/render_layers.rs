//! Render layer definitions for the simulator
//!
//! This module defines the render layers used to separate different types of
//! rendered content, allowing render cameras to see only targets (not debug gizmos)
//! while the debug/free camera can see everything.
//!
//! ## Layer Assignments
//!
//! | Layer | Contents | Seen By |
//! |-------|----------|---------|
//! | 0 | Default - ground plane, static scene geometry, camera markers | All cameras |
//! | 1 | Targets - moving objects to be detected | Render cameras, Debug camera |
//! | 2 | Debug gizmos - voxels, trails, coordinate axes | Debug camera only |
//!
//! ## Why This Matters
//!
//! Without layer separation, the frame differencing algorithm detects ALL pixel changes,
//! including debug visualizations like voxel cubes and track trails. This causes
//! 71,000+ false motion pixels instead of ~700 real target pixels.

use bevy::camera::visibility::RenderLayers;

/// Layer 0: Default scene geometry (ground plane, lights, camera markers)
pub const LAYER_DEFAULT: usize = 0;

/// Layer 1: Moving targets that should be detected by cameras
pub const LAYER_TARGETS: usize = 1;

/// Layer 2: Debug visualizations (voxels, trails, gizmos)
pub const LAYER_DEBUG: usize = 2;

/// Render layers for target entities - visible to render cameras
pub fn target_layers() -> RenderLayers {
    RenderLayers::layer(LAYER_TARGETS)
}

/// Render layers for debug gizmos - NOT visible to render cameras
pub fn debug_layers() -> RenderLayers {
    RenderLayers::layer(LAYER_DEBUG)
}

/// Render layers for scene geometry (ground, lights) - visible to all cameras
pub fn scene_layers() -> RenderLayers {
    RenderLayers::layer(LAYER_DEFAULT)
}

/// Render layers for the debug/free camera - sees everything
pub fn debug_camera_layers() -> RenderLayers {
    RenderLayers::from_layers(&[LAYER_DEFAULT, LAYER_TARGETS, LAYER_DEBUG])
}

/// Render layers for render cameras - see scene and targets, NOT debug gizmos
pub fn render_camera_layers() -> RenderLayers {
    RenderLayers::from_layers(&[LAYER_DEFAULT, LAYER_TARGETS])
}

/// Render layers for lights - must illuminate both scene geometry and targets
pub fn light_layers() -> RenderLayers {
    RenderLayers::layer(LAYER_DEFAULT).with(LAYER_TARGETS)
}
