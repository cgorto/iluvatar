use bevy::prelude::*;
use iluvatar_core::{CameraFrame, DetectionConfig, GeoPosition};
use iluvatar_server::{detector::ObjectDetector, grid::SparseVoxelGrid, tracker::ObjectTracker};
use std::sync::Arc;

use crate::capture::{CaptureConfig, SimulatorOrigin};
use crate::targets::SimulatedTarget;

/// Configuration for voxel grid visualization
#[derive(Resource)]
pub struct VoxelVisualizationConfig {
    /// Minimum intensity threshold for drawing voxels (skip dim ones for performance)
    pub intensity_threshold: f32,
    /// Maximum number of voxels to draw per frame (performance limit)
    pub max_voxels: usize,
    /// Size of each voxel cube (slightly smaller than actual voxel for visibility)
    pub cube_size: f32,
    /// Enable/disable visualization
    pub enabled: bool,
}

impl Default for VoxelVisualizationConfig {
    fn default() -> Self {
        Self {
            intensity_threshold: 0.5,
            max_voxels: 5000,
            cube_size: 0.8,
            enabled: true,
        }
    }
}

pub struct ValidationPlugin;

impl Plugin for ValidationPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ValidationMetrics>()
            .init_resource::<VoxelVisualizationConfig>()
            .add_systems(Startup, setup_tracker_simulation)
            .add_systems(
                Update,
                (
                    collect_ground_truth,
                    update_tracker_simulation,
                    log_validation_metrics,
                    draw_debug_visualization,
                    draw_voxel_grid_heatmap,
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

    let detection_config = DetectionConfig {
        intensity_threshold: 5.0, // Lower threshold for simulation
        min_contributors: 1,      // Allow single camera detection for simple tests
        cluster_epsilon: 5.0,
        cluster_min_points: 2,
    };

    // Detector produces anonymous detections (id=0), tracker assigns identity
    let detector = ObjectDetector::new(detection_config);

    let tracker = ObjectTracker::new(
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
                // t.centroid is in voxel grid coordinates (offset from grid min in voxels)
                // ENU = grid_min + centroid * voxel_size
                let enu = capture_config.grid_bounds.min + t.centroid * capture_config.voxel_size;

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

/// Draw debug visualization using gizmos
///
/// - Detected tracks: Red spheres with yellow velocity vectors
/// - Ground truth targets: Green wireframe spheres
fn draw_debug_visualization(
    mut gizmos: Gizmos,
    tracker_sim: Option<Res<TrackerSimulation>>,
    targets: Query<(&Transform, &SimulatedTarget)>,
) {
    // Draw ground truth targets (green wireframe spheres)
    for (transform, target) in targets.iter() {
        let pos = transform.translation;

        // Green sphere for ground truth
        gizmos.sphere(
            Isometry3d::from_translation(pos),
            3.0,
            Color::srgb(0.2, 1.0, 0.2), // Bright green
        );

        // Draw ground truth velocity vector (cyan)
        if target.velocity.length_squared() > 0.01 {
            let velocity_end = pos + target.velocity.normalize() * 5.0;
            gizmos.arrow(pos, velocity_end, Color::srgb(0.2, 1.0, 1.0));
        }

        // Label with ground truth ID (small cross marker)
        let label_offset = Vec3::Y * 4.0;
        gizmos.cross(
            Isometry3d::from_translation(pos + label_offset),
            1.0,
            Color::srgb(0.2, 1.0, 0.2),
        );
    }

    // Draw detected tracks from the tracker simulation
    if let Some(tracker) = tracker_sim {
        for track in &tracker.detected_tracks {
            let pos = track.position;

            // Red sphere for detected objects
            gizmos.sphere(
                Isometry3d::from_translation(pos),
                2.5,
                Color::srgb(1.0, 0.2, 0.2), // Bright red
            );

            // Yellow velocity vector
            if track.velocity.length_squared() > 0.01 {
                let velocity_end = pos + track.velocity.normalize() * 4.0;
                gizmos.arrow(pos, velocity_end, Color::srgb(1.0, 1.0, 0.2));
            }

            // Draw track ID indicator (orange cross above the sphere)
            let id_offset = Vec3::Y * 5.0;
            gizmos.cross(
                Isometry3d::from_translation(pos + id_offset),
                0.8,
                Color::srgb(1.0, 0.6, 0.0), // Orange
            );
        }
    }
}

/// Draw voxel grid heatmap visualization using gizmos
///
/// Visualizes active voxels in the sparse voxel grid as colored cubes:
/// - Low intensity: Blue
/// - Medium intensity: Yellow  
/// - High intensity: Red
///
/// The grid is in ENU coordinates, which must be converted to Bevy's Y-up system.
fn draw_voxel_grid_heatmap(
    mut gizmos: Gizmos,
    tracker_sim: Option<Res<TrackerSimulation>>,
    config: Res<VoxelVisualizationConfig>,
    capture_config: Res<CaptureConfig>,
) {
    if !config.enabled {
        return;
    }

    let Some(tracker) = tracker_sim else {
        return;
    };

    // Get max intensity for normalization (with a minimum to avoid division by zero)
    let max_intensity = tracker.grid.max_intensity().max(1.0);

    // Iterate over active voxels
    let voxels = tracker
        .grid
        .iter_voxels_for_visualization(config.intensity_threshold, config.max_voxels);

    for (grid_pos, intensity, camera_count) in voxels {
        // grid_pos is in grid-local coordinates (meters from grid origin at 0,0,0)
        // The grid origin corresponds to (0,0,0) in the grid, but the actual ENU
        // position needs to account for grid_bounds.min offset
        //
        // grid_pos already contains the world position from voxel_to_world(),
        // but it's relative to the grid's (0,0,0). We need to offset by grid_bounds.min
        // to get the ENU position, then convert to Bevy coordinates.
        //
        // ENU (East, North, Up) -> Bevy (X-right, Y-up, Z-forward)
        // ENU.x (East)  -> Bevy.x
        // ENU.y (North) -> Bevy.z
        // ENU.z (Up)    -> Bevy.y

        // The grid's voxel_to_world returns position in grid space (0,0,0 at grid origin)
        // We need to offset by the capture config's grid_bounds.min to get ENU coordinates
        let enu_pos = capture_config.grid_bounds.min + grid_pos;
        let bevy_pos = Vec3::new(enu_pos.x, enu_pos.z, enu_pos.y);

        // Compute normalized intensity (0.0 to 1.0)
        let t = (intensity / max_intensity).clamp(0.0, 1.0);

        // Heatmap color: blue (cold) -> yellow -> red (hot)
        let color = heatmap_color(t, camera_count);

        // Draw a small cuboid at the voxel position
        gizmos.cube(
            Transform::from_translation(bevy_pos).with_scale(Vec3::splat(config.cube_size)),
            color,
        );
    }
}

/// Convert normalized intensity (0.0-1.0) to a heatmap color.
/// Also considers camera_count for additional visual distinction.
///
/// Color scheme:
/// - t=0.0: Blue (cold/dim)
/// - t=0.5: Yellow (medium)  
/// - t=1.0: Red (hot/bright)
/// - Multiple cameras: Brighter/more saturated
fn heatmap_color(t: f32, camera_count: u8) -> Color {
    // Base heatmap interpolation
    let (r, g, b) = if t < 0.5 {
        // Blue to Yellow (0.0 -> 0.5)
        let t2 = t * 2.0;
        (
            t2,       // 0 -> 1
            t2,       // 0 -> 1
            1.0 - t2, // 1 -> 0
        )
    } else {
        // Yellow to Red (0.5 -> 1.0)
        let t2 = (t - 0.5) * 2.0;
        (
            1.0,      // stays 1
            1.0 - t2, // 1 -> 0
            0.0,      // stays 0
        )
    };

    // Boost brightness/alpha based on camera count (multi-camera corroboration)
    // More cameras = more visible/brighter
    let alpha = 0.4 + (camera_count as f32 / 5.0).min(0.6);

    Color::srgba(r, g, b, alpha)
}
