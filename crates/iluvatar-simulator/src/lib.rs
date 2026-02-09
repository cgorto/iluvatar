//! Iluvatar Simulator - Integration testing using real modules
//!
//! This simulator uses the real iluvatar modules:
//! - `iluvatar_core`: `Ray`, `VoxelContribution`, `CameraIntrinsics`, `BoundingBox`
//! - `iluvatar_server::grid::SparseVoxelGrid`: Real voxel storage with DashMap and camera bitmasks
//! - `iluvatar_server::detector::ObjectDetector`: DBSCAN clustering for object detection
//! - `iluvatar_server::tracker::ObjectTracker`: Multi-object tracking with Kalman filters
//! - 3D-DDA raymarching: Amanatides & Woo's algorithm for efficient voxel traversal
//!
//! ## Pipeline Modes
//!
//! ### Geometric Mode (original)
//! 1. **Targets**: Moving objects in the scene (ground truth)
//! 2. **Cameras**: Project targets to image space geometrically
//! 3. **Raymarching**: Cast rays at projected target positions
//! 4. **Detection**: Extract multi-camera consensus voxels, cluster with DBSCAN
//! 5. **Tracking**: Associate detections with tracks, update Kalman-filtered state
//!
//! ### Render Mode (new - realistic)
//! 1. **Targets**: Moving objects with visible meshes
//! 2. **Cameras**: Actually render the scene to textures
//! 3. **Frame Differencing**: Detect motion via pixel changes
//! 4. **Raymarching**: Cast rays for each motion pixel
//! 5. **Detection/Tracking**: Same as geometric mode
//!
//! Render mode is more realistic - it can simulate:
//! - Partial visibility (edge of frame, occlusion)
//! - Size-dependent detection (small targets = few pixels)
//! - Environmental noise, lighting conditions
//!
//! ## Usage
//!
//! ```rust,ignore
//! // Geometric mode (fast, idealized)
//! app.add_plugins(SimulatorPlugin);
//!
//! // Render mode (realistic, uses actual rendering)
//! app.add_plugins(RenderSimulatorPlugin);
//! ```
//!
//! ## Headless Testing
//!
//! The `harness` module provides a headless simulation runner for integration tests:
//!
//! ```rust,ignore
//! use iluvatar_simulator::harness::{ScenarioBuilder, TargetSpec, run_scenario};
//! use std::time::Duration;
//! use glam::Vec3;
//!
//! let scenario = ScenarioBuilder::new()
//!     .camera(/* camera spec */)
//!     .target(TargetSpec::stationary(1, Vec3::new(0.0, 20.0, 0.0)))
//!     .duration(Duration::from_secs(5))
//!     .build();
//!
//! let result = run_scenario(scenario);
//! result.assert_position_error_mean(10.0);
//! result.assert_detection_rate(0.9);
//! ```

pub(crate) mod camera;
mod debug_ui;
mod frame_server;
pub mod gpu_pipeline;
pub mod harness;
mod motion_raymarch;
mod render_camera;
pub mod render_layers;
mod scene;
pub mod sim_config;
pub(crate) mod targets;
mod tracking;
pub(crate) mod voxels;

use bevy::{camera_controller::free_camera::FreeCameraPlugin, prelude::*};

// Re-export simulator types
pub use camera::CaptureCamera;
pub use debug_ui::VisualizationConfig;
pub use gpu_pipeline::{GpuPipelineMetrics, GpuRay, RayBufferResource};
pub use render_camera::{RenderCamera, RenderCameraConfig, RenderCameraPlugin};
pub use sim_config::SimulatorTomlConfig;
pub use targets::{Target, TargetPath};
pub use tracking::{TrackingConfig, TrackingMetrics, TrackingState};
pub use voxels::{SimulatorConfig, SimulatorRaymarcher, VoxelGridResource};

// Re-export core types used by the simulator
pub use iluvatar_core::{
    BoundingBox, CameraIntrinsics, Fov, Ray, RaymarchConfig, VoxelContribution,
};
pub use iluvatar_server::grid::SparseVoxelGrid;

/// Original simulator plugin using geometric projection
///
/// This is faster but idealized - targets are detected if they're
/// geometrically within a camera's FOV, regardless of size, occlusion,
/// or lighting conditions.
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

/// Render-based simulator plugin using actual frame differencing
///
/// This is more realistic - cameras actually render the scene and
/// detect motion via pixel changes. This can simulate:
/// - Partial visibility at frame edges
/// - Size-dependent detection (small targets may not trigger enough pixels)
/// - Occlusion between targets
/// - (Future) Lighting conditions, noise, etc.
pub struct RenderSimulatorPlugin;

impl Plugin for RenderSimulatorPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(scene::ScenePlugin)
            // Targets with visible meshes (already has meshes in spawn_targets)
            .add_plugins(targets::TargetsPlugin)
            // Render cameras instead of geometric projection cameras
            .add_plugins(render_camera::RenderCameraPlugin)
            // GPU motion detection pipeline (frame diff + ray gen on GPU)
            .add_plugins(gpu_pipeline::GpuMotionPipelinePlugin)
            // Motion-based raymarching (consumes rays from GPU buffer)
            .add_plugins(motion_raymarch::MotionRaymarchPlugin)
            // Voxel grid without the project_and_raymarch system
            .add_plugins(voxels::VoxelsPluginWithoutRaymarch)
            // Tracking still works the same way
            .add_plugins(tracking::TrackingPluginForRenderMode)
            // TCP frame server for streaming to camera processes
            .add_plugins(frame_server::FrameServerPlugin)
            .add_plugins(debug_ui::DebugUiPlugin)
            .add_plugins(FreeCameraPlugin);
    }
}
