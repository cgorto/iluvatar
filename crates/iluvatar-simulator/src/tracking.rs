//! Detection and tracking integration for the simulator
//!
//! This module wires up the real detection and tracking pipeline from iluvatar-server:
//! - `ObjectDetector`: DBSCAN clustering on the voxel point cloud
//! - `ObjectTracker`: Multi-object tracking with Kalman-filtered state estimation
//!
//! The flow:
//! 1. Extract points from voxel grid (voxels with sufficient intensity + multi-camera consensus)
//! 2. Cluster points into discrete objects via DBSCAN
//! 3. Associate detections with existing tracks (Hungarian-ish assignment)
//! 4. Update Kalman filters for matched tracks
//! 5. Spawn new tracks for unmatched detections
//! 6. Remove stale tracks
//!
//! The Kalman filter is the mathematical heart of tracking. It maintains a belief
//! about each object's state (position + velocity) as a Gaussian distribution.
//! The covariance matrix encodes our uncertainty - and watching it evolve as
//! measurements flow in is genuinely beautiful. The filter naturally trades off
//! between trusting the motion model (prediction) and trusting the measurements,
//! based on their relative uncertainties.

use bevy::prelude::*;
use glam::Vec3;
use std::collections::VecDeque;

use iluvatar_core::{DetectionConfig, ObjectId, TrackedObject};
use iluvatar_server::detector::ObjectDetector;
use iluvatar_server::tracker::ObjectTracker;

use crate::targets::{Target, TargetPath};
use crate::voxels::{SimulatorConfig, VoxelGridResource};

pub struct TrackingPlugin;

impl Plugin for TrackingPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TrackingConfig>()
            .init_resource::<TrackingState>()
            .init_resource::<TrackingMetrics>()
            .add_systems(
                Update,
                (
                    run_detection_and_tracking,
                    visualize_tracked_objects,
                    compute_tracking_metrics,
                    print_tracking_stats,
                )
                    .chain()
                    .after(crate::voxels::project_and_raymarch),
            );
    }
}

/// Configuration for the tracking system
#[derive(Resource, Clone)]
pub struct TrackingConfig {
    /// Detection configuration (thresholds, DBSCAN params)
    pub detection: DetectionConfig,
    /// Maximum distance to associate a detection with an existing track (meters)
    pub association_threshold: f32,
    /// How many frames a track can go without detection before being removed
    pub max_missing_frames: u32,
    /// Assumed frame rate for dt calculation
    pub frame_rate: f32,
    /// How many positions to keep in track history for visualization
    pub trail_length: usize,
    /// Minimum confidence for visualization (0-1, based on camera consensus)
    pub min_visualization_confidence: f32,
    /// Use percentile-based extraction instead of fixed threshold
    pub use_percentile_extraction: bool,
    /// Percentile threshold (0.9 = keep top 10%)
    pub extraction_percentile: f32,
    /// Clear the voxel grid each frame (vs. relying on decay)
    pub clear_grid_each_frame: bool,
}

impl Default for TrackingConfig {
    fn default() -> Self {
        Self {
            detection: DetectionConfig {
                intensity_threshold: 5.0, // Used as fallback if percentile disabled
                min_contributors: 2,      // Require at least 2 cameras (triangulation!)
                cluster_epsilon: 5.0,     // 5m radius - catches nearby voxels from same target
                cluster_min_points: 1,    // Single high-confidence point = valid detection
            },
            association_threshold: 10.0, // Tighter association - 10m instead of 15m
            max_missing_frames: 30,      // ~0.5s at 60fps before track dies
            frame_rate: 60.0,
            trail_length: 60, // 1 second of history
            min_visualization_confidence: 0.3,
            use_percentile_extraction: true, // NEW: use percentile-based extraction
            extraction_percentile: 0.80,     // Keep top 20% - more forgiving for sparse data
            clear_grid_each_frame: true,     // NEW: fresh slate each frame
        }
    }
}

/// Runtime state for the tracking system
#[derive(Resource)]
pub struct TrackingState {
    /// The DBSCAN-based detector
    detector: ObjectDetector,
    /// The multi-object tracker with Kalman filters
    tracker: ObjectTracker,
    /// Current tracked objects (updated each frame)
    pub tracked_objects: Vec<TrackedObject>,
    /// Position history for each track (for trail visualization)
    pub track_histories: std::collections::HashMap<ObjectId, VecDeque<Vec3>>,
}

impl FromWorld for TrackingState {
    fn from_world(world: &mut World) -> Self {
        let config = world
            .get_resource::<TrackingConfig>()
            .cloned()
            .unwrap_or_default();

        Self {
            detector: ObjectDetector::new(config.detection),
            tracker: ObjectTracker::new(
                config.association_threshold,
                config.max_missing_frames,
                config.frame_rate,
            ),
            tracked_objects: Vec::new(),
            track_histories: std::collections::HashMap::new(),
        }
    }
}

/// Metrics comparing tracked objects to ground truth
#[derive(Resource, Default)]
pub struct TrackingMetrics {
    /// Number of ground truth targets
    pub ground_truth_count: usize,
    /// Number of tracked objects
    pub tracked_count: usize,
    /// Average position error for matched track-target pairs (meters)
    pub avg_position_error: f32,
    /// Average velocity error for matched pairs (m/s)
    pub avg_velocity_error: f32,
    /// Number of tracks matched to targets
    pub matched_count: usize,
    /// Number of detected points (before clustering)
    pub detected_points: usize,
    /// Number of clusters found
    pub cluster_count: usize,
}

/// The main detection and tracking system
///
/// This is where the magic happens:
/// 1. Extract high-confidence voxels from the grid (using percentile filtering!)
/// 2. Cluster them into discrete objects
/// 3. Update the tracker with the detections
fn run_detection_and_tracking(
    time: Res<Time>,
    grid_res: Res<VoxelGridResource>,
    config: Res<TrackingConfig>,
    mut state: ResMut<TrackingState>,
    mut metrics: ResMut<TrackingMetrics>,
) {
    // Extract points from the voxel grid
    // The key insight: use percentile-based extraction to find the TRUE hotspots
    // where rays from multiple cameras intersect, not just any voxel above a threshold
    let points = if config.use_percentile_extraction {
        // Percentile-based: keep only the top N% of voxels by intensity
        // This naturally finds the intersection peaks without needing to tune
        // an absolute threshold that depends on ray count, attenuation, etc.
        grid_res.grid.extract_points_percentile(
            config.extraction_percentile,
            config.detection.min_contributors,
            4, // We have 4 cameras in the simulator
        )
    } else {
        // Fallback: fixed intensity threshold
        grid_res.grid.extract_points_with_camera_count(
            &config.detection,
            4, // We have 4 cameras in the simulator
        )
    };

    metrics.detected_points = points.len();

    // Run DBSCAN clustering to find discrete objects
    let detections = state.detector.detect(&points);
    metrics.cluster_count = detections.len();

    // Update the tracker with new detections
    // This handles:
    // - Association (matching detections to existing tracks)
    // - Kalman filter predict/update cycle
    // - Track lifecycle (birth, death)
    let tracked = state.tracker.update(detections, time.delta_secs());

    metrics.tracked_count = tracked.len();

    // Update track histories for trail visualization
    let trail_len = config.trail_length;
    for obj in &tracked {
        let history = state.track_histories.entry(obj.id).or_default();
        history.push_back(obj.centroid);
        while history.len() > trail_len {
            history.pop_front();
        }
    }

    // Remove histories for dead tracks
    let active_ids: std::collections::HashSet<ObjectId> = tracked.iter().map(|o| o.id).collect();
    state
        .track_histories
        .retain(|id, _| active_ids.contains(id));

    state.tracked_objects = tracked;
}

/// Visualize tracked objects with gizmos
fn visualize_tracked_objects(
    mut gizmos: Gizmos,
    state: Res<TrackingState>,
    sim_config: Res<SimulatorConfig>,
    config: Res<TrackingConfig>,
    vis_config: Option<Res<crate::debug_ui::VisualizationConfig>>,
) {
    let grid_origin = sim_config.grid_origin;

    let show_velocity = vis_config.as_ref().is_none_or(|c| c.show_velocity_vectors);
    let show_trails = vis_config.as_ref().is_none_or(|c| c.show_trails);
    let show_bboxes = vis_config.as_ref().is_none_or(|c| c.show_bounding_boxes);

    for obj in &state.tracked_objects {
        // Skip low-confidence tracks
        if obj.confidence < config.min_visualization_confidence {
            continue;
        }

        // Tracked object position (adjust for grid origin)
        let pos = obj.centroid + grid_origin;

        // Color based on track ID (consistent color per track)
        let hue = (obj.id as f32 * 0.618034) % 1.0; // Golden ratio for nice spread
        let color = Color::hsl(hue * 360.0, 0.8, 0.6);

        // Draw the tracked object as a larger sphere
        gizmos.sphere(Isometry3d::from_translation(pos), 5.0, color);

        // Draw bounding box
        if show_bboxes {
            let bb_center = obj.bounding_box.center() + grid_origin;
            let bb_size = obj.bounding_box.size();
            gizmos.cube(
                Transform::from_translation(bb_center).with_scale(bb_size),
                color.with_alpha(0.3),
            );
        }

        // Draw velocity vector if available
        if show_velocity {
            if let Some(vel) = obj.velocity {
                if vel.length() > 0.1 {
                    // Scale velocity for visualization (1 second lookahead)
                    let vel_end = pos + vel;
                    gizmos.arrow(pos, vel_end, Color::WHITE);
                }
            }
        }

        // Draw track history (trail)
        if show_trails {
            if let Some(history) = state.track_histories.get(&obj.id) {
                let points: Vec<Vec3> = history.iter().map(|p| *p + grid_origin).collect();
                if points.len() >= 2 {
                    for i in 0..points.len() - 1 {
                        let alpha = i as f32 / points.len() as f32;
                        let trail_color = color.with_alpha(alpha * 0.5);
                        gizmos.line(points[i], points[i + 1], trail_color);
                    }
                }
            }
        }

        // Draw track ID label position (slightly above the object)
        // Note: Bevy gizmos don't support text, but we position a small marker
        let label_pos = pos + Vec3::Y * 8.0;
        gizmos.sphere(Isometry3d::from_translation(label_pos), 1.0, Color::WHITE);
    }
}

/// Compute tracking metrics by comparing to ground truth
fn compute_tracking_metrics(
    time: Res<Time>,
    state: Res<TrackingState>,
    sim_config: Res<SimulatorConfig>,
    targets: Query<(&Transform, &TargetPath), With<Target>>,
    mut metrics: ResMut<TrackingMetrics>,
) {
    let grid_origin = sim_config.grid_origin;
    let t = time.elapsed_secs();

    // Collect ground truth positions and velocities
    let ground_truth: Vec<(Vec3, Vec3)> = targets
        .iter()
        .map(|(transform, path)| (transform.translation, path.current_velocity(t)))
        .collect();
    metrics.ground_truth_count = ground_truth.len();

    if state.tracked_objects.is_empty() || ground_truth.is_empty() {
        metrics.matched_count = 0;
        metrics.avg_position_error = 0.0;
        metrics.avg_velocity_error = 0.0;
        return;
    }

    // Simple greedy matching: for each ground truth, find closest track
    let mut total_pos_error = 0.0;
    let mut total_vel_error = 0.0;
    let mut matched = 0;

    for (gt_pos, gt_vel) in &ground_truth {
        let mut best_dist = f32::MAX;
        let mut best_track: Option<&TrackedObject> = None;

        for track in &state.tracked_objects {
            let track_pos = track.centroid + grid_origin;
            let dist = track_pos.distance(*gt_pos);
            if dist < best_dist {
                best_dist = dist;
                best_track = Some(track);
            }
        }

        // Only count as matched if within reasonable distance (20m)
        if best_dist < 20.0 {
            matched += 1;
            total_pos_error += best_dist;

            // Velocity error: compare tracked velocity to ground truth velocity
            if let Some(track) = best_track {
                if let Some(vel) = track.velocity {
                    // Compute velocity error as vector difference
                    let vel_error = (vel - *gt_vel).length();
                    total_vel_error += vel_error;
                }
            }
        }
    }

    metrics.matched_count = matched;
    metrics.avg_position_error = if matched > 0 {
        total_pos_error / matched as f32
    } else {
        0.0
    };
    metrics.avg_velocity_error = if matched > 0 {
        total_vel_error / matched as f32
    } else {
        0.0
    };
}

/// Print tracking statistics periodically
fn print_tracking_stats(
    time: Res<Time>,
    state: Res<TrackingState>,
    sim_config: Res<SimulatorConfig>,
    targets: Query<(&Transform, &TargetPath, &Target)>,
    metrics: Res<TrackingMetrics>,
    mut last_print: Local<f32>,
) {
    let now = time.elapsed_secs();
    if now - *last_print > 2.0 {
        *last_print = now;

        let grid_origin = sim_config.grid_origin;

        println!("\n=== Tracking Stats (t={:.1}s) ===", now);
        println!(
            "Pipeline: {} detected pts -> {} clusters -> {} tracks",
            metrics.detected_points, metrics.cluster_count, metrics.tracked_count
        );
        println!(
            "Ground truth: {} targets | Matched: {}/{} | Pos err: {:.2}m | Vel err: {:.2}m/s",
            metrics.ground_truth_count,
            metrics.matched_count,
            metrics.ground_truth_count,
            metrics.avg_position_error,
            metrics.avg_velocity_error
        );

        // Collect ground truth for matching
        let ground_truth: Vec<(Vec3, Vec3, u32)> = targets
            .iter()
            .map(|(t, path, target)| (t.translation, path.current_velocity(now), target.id))
            .collect();

        for obj in &state.tracked_objects {
            // Convert grid-local centroid to world coordinates for display
            let world_pos = obj.centroid + grid_origin;

            // Find closest ground truth target
            let closest_gt = ground_truth.iter().min_by(|a, b| {
                let da = world_pos.distance(a.0);
                let db = world_pos.distance(b.0);
                da.partial_cmp(&db).unwrap()
            });

            let (vel_str, gt_info) = if let Some(vel) = obj.velocity {
                let vel_str = format!(
                    "vel=({:.1},{:.1},{:.1}) |v|={:.1}m/s",
                    vel.x,
                    vel.y,
                    vel.z,
                    vel.length()
                );
                let gt_info = if let Some((gt_pos, gt_vel, gt_id)) = closest_gt {
                    let pos_err = world_pos.distance(*gt_pos);
                    let vel_err = (vel - *gt_vel).length();
                    format!(
                        " <-> Target {} (pos_err={:.1}m, vel_err={:.1}m/s, gt_vel=({:.1},{:.1},{:.1}))",
                        gt_id, pos_err, vel_err, gt_vel.x, gt_vel.y, gt_vel.z
                    )
                } else {
                    String::new()
                };
                (vel_str, gt_info)
            } else {
                ("no velocity".to_string(), String::new())
            };

            println!(
                "  Track {}: pos=({:.1},{:.1},{:.1}) {} conf={:.2}{}",
                obj.id, world_pos.x, world_pos.y, world_pos.z, vel_str, obj.confidence, gt_info
            );
        }
    }
}
