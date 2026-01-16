use bevy::prelude::*;
use iluvatar_core::{CameraFrame, DetectionConfig, GeoPosition};
use iluvatar_server::{
    detector::{ObjectDetector, ObjectIdGenerator},
    grid::SparseVoxelGrid,
    tracker::ObjectTracker,
};
use std::sync::Arc;

use crate::capture::{CaptureConfig, SimulatorOrigin};
use crate::targets::SimulatedTarget;

pub struct ValidationPlugin;

impl Plugin for ValidationPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ValidationMetrics>()
            .add_systems(Startup, setup_tracker_simulation)
            .add_systems(
                Update,
                (
                    collect_ground_truth,
                    update_tracker_simulation,
                    log_validation_metrics,
                ),
            );
    }
}

/// Resource that runs a local instance of the server's tracking pipeline
#[derive(Resource)]
pub struct TrackerSimulation {
    pub grid: Arc<SparseVoxelGrid>,
    pub detector: ObjectDetector,
    pub tracker: ObjectTracker,
    pub last_update: f64,
    pub detected_tracks: Vec<TrackedObjectInfo>,
}

#[derive(Clone, Debug)]
pub struct TrackedObjectInfo {
    pub id: u64,
    pub position: Vec3,
    pub velocity: Vec3,
}

fn setup_tracker_simulation(mut commands: Commands, origin: Res<SimulatorOrigin>) {
    // Default configuration for the simulated tracker
    let grid_origin = GeoPosition::new(
        origin.geo_position.latitude,
        origin.geo_position.longitude,
        origin.geo_position.altitude,
    );
    let voxel_size = 1.0;

    // Create grid with 500m radius (1000m dim)
    let grid = Arc::new(SparseVoxelGrid::new(
        grid_origin,
        glam::UVec3::new(1000, 1000, 200),
        voxel_size,
        0.5, // Decay rate
    ));

    let id_generator = Arc::new(ObjectIdGenerator::new());

    let detection_config = DetectionConfig {
        intensity_threshold: 5.0, // Lower threshold for simulation
        min_contributors: 1,      // Allow single camera detection for simple tests
        cluster_epsilon: 5.0,
        cluster_min_points: 2,
    };

    let detector = ObjectDetector::new(detection_config, id_generator.clone());

    let tracker = ObjectTracker::new(
        id_generator,
        15.0, // Association threshold
        60,   // Max missing frames (generous for sim)
        60.0, // Frame rate
    );

    commands.insert_resource(TrackerSimulation {
        grid,
        detector,
        tracker,
        last_update: 0.0,
        detected_tracks: Vec::new(),
    });
}

fn update_tracker_simulation(
    time: Res<Time>,
    mut tracker_sim: ResMut<TrackerSimulation>,
    capture_config: Res<CaptureConfig>,
) {
    let now = time.elapsed_secs_f64();

    // Run detection/tracking at 10Hz
    if now - tracker_sim.last_update >= 0.1 {
        // 1. Decay
        tracker_sim.grid.apply_decay();

        // 2. Detect
        let detection_config = DetectionConfig {
            intensity_threshold: 5.0,
            min_contributors: 1,
            cluster_epsilon: 5.0,
            cluster_min_points: 2,
        };
        let points = tracker_sim.grid.extract_points(&detection_config);
        let detections = tracker_sim.detector.detect(&points);

        // 3. Track
        let tracks = tracker_sim.tracker.update(detections);

        // 4. Convert tracks back to Bevy coordinates for validation
        tracker_sim.detected_tracks = tracks
            .into_iter()
            .map(|t| {
                // t.centroid is in local grid coordinates (offset from grid min)
                // ENU = grid_min + centroid
                let enu = capture_config.grid_bounds.min + t.centroid;

                TrackedObjectInfo {
                    id: t.id,
                    position: Vec3::new(enu.x, enu.z, enu.y),
                    velocity: Vec3::ZERO,
                }
            })
            .collect();

        tracker_sim.last_update = now;
    }
}

/// Ground truth position for a target at a point in time
#[derive(Debug, Clone)]
pub struct GroundTruthSample {
    pub target_id: u32,
    pub position: Vec3,
    pub velocity: Vec3,
    pub timestamp: f64,
}

/// Validation metrics and ground truth collection
#[derive(Resource, Default)]
pub struct ValidationMetrics {
    pub ground_truth: Vec<GroundTruthSample>,
    pub detection_errors: Vec<f32>,
    pub false_positives: u32,
    pub false_negatives: u32,
    pub total_detections: u32,
    pub id_switches: u32,
    /// Maps tracker_id -> ground_truth_id for consistency checking
    pub id_mappings: std::collections::HashMap<u64, u32>,
}

impl ValidationMetrics {
    /// Record a detection with tracker ID and compare to ground truth
    ///
    /// This version tracks ID consistency - use for tracker output validation
    pub fn record_detection(
        &mut self,
        tracker_id: u64,
        detected_pos: Vec3,
        timestamp: f64,
    ) -> Option<f32> {
        if let Some((error, gt_id)) = self.find_closest_ground_truth(detected_pos, timestamp) {
            self.detection_errors.push(error);
            self.total_detections += 1;

            // Check for ID consistency
            if let Some(previous_gt_id) = self.id_mappings.get(&tracker_id) {
                if *previous_gt_id != gt_id {
                    self.id_switches += 1;
                    tracing::warn!(
                        "ID Switch! Tracker {} switched from GT {} to GT {}",
                        tracker_id,
                        previous_gt_id,
                        gt_id
                    );
                }
            }
            self.id_mappings.insert(tracker_id, gt_id);

            Some(error)
        } else {
            self.false_positives += 1;
            None
        }
    }

    /// Record a raw detection (no tracker ID) and compare to ground truth
    ///
    /// Use this for frame-level validation before tracker assignment
    pub fn record_raw_detection(&mut self, detected_pos: Vec3, timestamp: f64) -> Option<f32> {
        if let Some((error, _gt_id)) = self.find_closest_ground_truth(detected_pos, timestamp) {
            self.detection_errors.push(error);
            self.total_detections += 1;
            Some(error)
        } else {
            self.false_positives += 1;
            None
        }
    }

    /// Find closest ground truth sample to a detected position
    fn find_closest_ground_truth(&self, detected_pos: Vec3, timestamp: f64) -> Option<(f32, u32)> {
        self.ground_truth
            .iter()
            .filter(|gt| (gt.timestamp - timestamp).abs() < 0.1)
            .min_by(|a, b| {
                let dist_a = a.position.distance(detected_pos);
                let dist_b = b.position.distance(detected_pos);
                dist_a
                    .partial_cmp(&dist_b)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|gt| (gt.position.distance(detected_pos), gt.target_id))
    }

    /// Calculate mean detection error
    pub fn mean_error(&self) -> Option<f32> {
        if self.detection_errors.is_empty() {
            None
        } else {
            Some(self.detection_errors.iter().sum::<f32>() / self.detection_errors.len() as f32)
        }
    }

    /// Calculate detection rate
    pub fn detection_rate(&self) -> f32 {
        let total = self.total_detections + self.false_negatives;
        if total == 0 {
            0.0
        } else {
            self.total_detections as f32 / total as f32
        }
    }

    /// Record a captured camera frame for validation
    ///
    /// This method analyzes the voxel contributions from a camera frame
    /// to identify potential object detections and validate against ground truth.
    pub fn record_captured_frame(&mut self, frame: &CameraFrame, config: &CaptureConfig) {
        // Convert timestamp from microseconds to seconds
        let timestamp_secs = frame.timestamp as f64 / 1_000_000.0;

        if !frame.contributions.is_empty() {
            // Calculate centroid of all contributions as a rough detection position
            let mut centroid = Vec3::ZERO;
            let mut total_weight = 0.0f32;

            for contrib in &frame.contributions {
                // Convert voxel index to world coordinates (ENU)
                // Voxel (0,0,0) corresponds to grid_bounds.min
                let enu_pos = Vec3::new(
                    config.grid_bounds.min.x + (contrib.index.x as f32 + 0.5) * config.voxel_size,
                    config.grid_bounds.min.y + (contrib.index.y as f32 + 0.5) * config.voxel_size,
                    config.grid_bounds.min.z + (contrib.index.z as f32 + 0.5) * config.voxel_size,
                );

                // Convert ENU (East, North, Up) to Bevy (X-right, Y-up, Z-back)
                // ENU X (East) -> Bevy X
                // ENU Y (North) -> Bevy Z (but negated for forward)
                // ENU Z (Up) -> Bevy Y
                let bevy_pos = Vec3::new(enu_pos.x, enu_pos.z, enu_pos.y);

                let weight = contrib.intensity as f32;
                centroid += bevy_pos * weight;
                total_weight += weight;
            }

            if total_weight > 0.0 {
                centroid /= total_weight;

                // Record this as a raw detection (no tracker ID yet)
                if let Some(error) = self.record_raw_detection(centroid, timestamp_secs) {
                    tracing::debug!(
                        "Detection at {:?}, error: {:.2}m ({} contributions)",
                        centroid,
                        error,
                        frame.contributions.len()
                    );
                }
            }
        }
    }
}

/// Collect ground truth positions from simulated targets
pub fn collect_ground_truth(
    time: Res<Time>,
    targets: Query<(&Transform, &SimulatedTarget)>,
    mut metrics: ResMut<ValidationMetrics>,
) {
    let timestamp = time.elapsed_secs_f64();

    for (transform, target) in targets.iter() {
        metrics.ground_truth.push(GroundTruthSample {
            target_id: target.ground_truth_id,
            position: transform.translation,
            velocity: target.velocity,
            timestamp,
        });
    }

    // Keep only recent ground truth (last 10 seconds)
    let cutoff = timestamp - 10.0;
    metrics.ground_truth.retain(|gt| gt.timestamp > cutoff);
}

/// Log validation metrics periodically
fn log_validation_metrics(
    time: Res<Time>,
    metrics: Res<ValidationMetrics>,
    tracker_sim: Option<Res<TrackerSimulation>>,
    mut last_log: Local<f64>,
) {
    let now = time.elapsed_secs_f64();

    // Log every 5 seconds
    if now - *last_log >= 5.0 {
        *last_log = now;

        let tracker_count = tracker_sim
            .as_ref()
            .map(|t| t.detected_tracks.len())
            .unwrap_or(0);

        tracing::info!(
            "Validation Stats | Detections: {} | False Positives: {} | Mean Error: {:.2}m | Active Tracks: {}",
            metrics.total_detections,
            metrics.false_positives,
            metrics.mean_error().unwrap_or(0.0),
            tracker_count
        );

        // Log current ground truth positions
        let unique_targets: std::collections::HashSet<u32> = metrics
            .ground_truth
            .iter()
            .filter(|gt| (gt.timestamp - now).abs() < 0.2)
            .map(|gt| gt.target_id)
            .collect();

        if !unique_targets.is_empty() {
            tracing::debug!("Active targets: {:?}", unique_targets);
        }
    }
}
