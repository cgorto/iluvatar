//! Detection system using DBSCAN clustering from iluvatar-server
//!
//! This module integrates the real `ObjectDetector` from iluvatar-server into
//! the Bevy simulator. After raymarching accumulates voxel intensities, we
//! extract high-confidence points and cluster them with DBSCAN to find objects.
//!
//! The flow:
//! 1. `SparseVoxelGrid::extract_points()` → `Vec<DetectedPoint>`
//! 2. `ObjectDetector::detect()` → `Vec<TrackedObject>` (with id=0, to be assigned by tracker)
//!
//! These "anonymous" detections become input for the tracking system.

use bevy::prelude::*;

use iluvatar_core::{DetectedPoint, DetectionConfig, TrackedObject};
use iluvatar_server::detector::ObjectDetector;

use crate::camera::CaptureCamera;
use crate::voxels::VoxelGridResource;

pub struct DetectionPlugin;

impl Plugin for DetectionPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DetectorResource>()
            .init_resource::<DetectionOutput>()
            .add_systems(
                Update,
                run_detection.after(crate::voxels::project_and_raymarch),
            );
    }
}

/// Configuration for detection in the simulator
#[derive(Resource, Clone)]
pub struct DetectorConfig {
    /// Intensity threshold for extracting voxels as points
    pub intensity_threshold: f32,
    /// Minimum number of cameras that must see a voxel
    pub min_contributors: u8,
    /// DBSCAN epsilon - maximum distance between points in same cluster (meters)
    pub cluster_epsilon: f32,
    /// DBSCAN min_points - minimum points to form a cluster
    pub cluster_min_points: usize,
}

impl Default for DetectorConfig {
    fn default() -> Self {
        // Tuned for our simulator scale:
        // - 2m voxels, targets are ~3m radius spheres
        // - 4 cameras for triangulation
        // - epsilon of 6m allows nearby voxels to cluster
        // - min 3 points prevents noise clusters
        Self {
            intensity_threshold: 5.0, // Lower than default since we're in a controlled sim
            min_contributors: 2,      // Require at least 2 cameras for triangulation
            cluster_epsilon: 6.0,     // ~3 voxels at 2m size
            cluster_min_points: 3,    // Need at least 3 high-intensity voxels
        }
    }
}

/// Wraps `ObjectDetector` as a Bevy Resource
#[derive(Resource)]
pub struct DetectorResource {
    pub detector: ObjectDetector,
    pub config: DetectorConfig,
}

impl Default for DetectorResource {
    fn default() -> Self {
        let config = DetectorConfig::default();
        let detection_config = DetectionConfig {
            intensity_threshold: config.intensity_threshold,
            min_contributors: config.min_contributors,
            cluster_epsilon: config.cluster_epsilon,
            cluster_min_points: config.cluster_min_points,
        };
        Self {
            detector: ObjectDetector::new(detection_config),
            config,
        }
    }
}

/// Output from detection system - available to tracking and visualization
#[derive(Resource, Default)]
pub struct DetectionOutput {
    /// Raw detected points extracted from voxel grid
    pub points: Vec<DetectedPoint>,
    /// Clustered objects (id=0, to be assigned by tracker)
    pub detections: Vec<TrackedObject>,
    /// Stats for debugging
    pub points_extracted: usize,
    pub clusters_found: usize,
}

/// Run detection on the voxel grid
///
/// This system:
/// 1. Extracts high-intensity, multi-camera voxels as DetectedPoints
/// 2. Runs DBSCAN clustering to find object clusters
/// 3. Outputs anonymous TrackedObjects for the tracker
pub fn run_detection(
    grid_res: Res<VoxelGridResource>,
    mut detector_res: ResMut<DetectorResource>,
    mut output: ResMut<DetectionOutput>,
    cameras: Query<&CaptureCamera>,
) {
    // Count active cameras for confidence calculation
    let active_cameras = cameras.iter().count() as u8;

    // Build DetectionConfig from our config
    let detection_config = DetectionConfig {
        intensity_threshold: detector_res.config.intensity_threshold,
        min_contributors: detector_res.config.min_contributors,
        cluster_epsilon: detector_res.config.cluster_epsilon,
        cluster_min_points: detector_res.config.cluster_min_points,
    };

    // Extract points from the voxel grid
    // This filters to voxels above intensity threshold with enough camera contributors
    let points = grid_res
        .grid
        .extract_points_with_camera_count(&detection_config, active_cameras);

    output.points_extracted = points.len();

    // Run DBSCAN clustering
    let detections = detector_res.detector.detect(&points);
    output.clusters_found = detections.len();

    // Store outputs
    output.points = points;
    output.detections = detections;
}
