//! Iluvatar Simulator - Integration testing using real modules
//!
//! This simulator uses the real iluvatar modules:
//! - `iluvatar_core`: `Ray`, `VoxelContribution`, `CameraIntrinsics`, `BoundingBox`
//! - `iluvatar_server::grid::SparseVoxelGrid`: Real voxel storage with DashMap and camera bitmasks
//! - `iluvatar_server::detector::ObjectDetector`: DBSCAN clustering for object detection
//! - `iluvatar_server::tracker::ObjectTracker`: Multi-object tracking with Kalman filters
//! - 3D-DDA raymarching: Amanatides & Woo's algorithm for efficient voxel traversal
//!
//! ## Pipeline
//!
//! 1. **Targets**: Moving objects in the scene (ground truth)
//! 2. **Cameras**: Project targets to image space, generate rays for detected "motion"
//! 3. **Raymarching**: DDA traversal accumulates voxel contributions along each ray
//! 4. **Detection**: Extract multi-camera consensus voxels, cluster with DBSCAN
//! 5. **Tracking**: Associate detections with tracks, update Kalman-filtered state
//!
//! The Kalman filter maintains a belief about each object's position and velocity,
//! propagating uncertainty through time and fusing in noisy measurements. The
//! cross-covariance between position and velocity allows the filter to learn
//! velocity purely from position observations.

mod camera;
mod debug_ui;
mod scene;
mod targets;
mod tracking;
mod voxels;

use bevy::{camera_controller::free_camera::FreeCameraPlugin, prelude::*};

// Re-export simulator types
pub use camera::CaptureCamera;
pub use debug_ui::VisualizationConfig;
pub use targets::{Target, TargetPath};
pub use tracking::{TrackingConfig, TrackingMetrics, TrackingState};
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
            .add_plugins(tracking::TrackingPlugin)
            .add_plugins(debug_ui::DebugUiPlugin)
            .add_plugins(FreeCameraPlugin);
    }
}
