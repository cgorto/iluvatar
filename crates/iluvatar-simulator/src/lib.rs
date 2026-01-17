//! Iluvatar Simulator - Integration testing using real modules
//!
//! This simulator uses the real iluvatar modules:
//! - `iluvatar_core`: `Ray`, `VoxelContribution`, `CameraIntrinsics`, `BoundingBox`
//! - `iluvatar_server::grid::SparseVoxelGrid`: Real voxel storage with DashMap and camera bitmasks
//! - 3D-DDA raymarching: Amanatides & Woo's algorithm for efficient voxel traversal
//!
//! The simulator projects known target positions to cameras, casts rays using
//! mathematically correct DDA traversal, and accumulates contributions in the
//! real sparse voxel grid. Multi-camera triangulation is demonstrated by the
//! camera bitmask tracking in each voxel.

mod camera;
mod scene;
mod targets;
mod voxels;

use bevy::{camera_controller::free_camera::FreeCameraPlugin, prelude::*};

// Re-export simulator types
pub use camera::CaptureCamera;
pub use targets::{Target, TargetPath};
pub use voxels::{SimulatorConfig, SimulatorRaymarcher, VoxelGridResource};

// Re-export core types used by the simulator
pub use iluvatar_core::{
    BoundingBox, CameraIntrinsics, Fov, Ray, RaymarchConfig, VoxelContribution,
};
pub use iluvatar_server::grid::SparseVoxelGrid;

pub struct SimulatorPlugin;

impl Plugin for SimulatorPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(scene::ScenePlugin)
            .add_plugins(camera::CameraPlugin)
            .add_plugins(targets::TargetsPlugin)
            .add_plugins(voxels::VoxelsPlugin)
            .add_plugins(FreeCameraPlugin);
    }
}
