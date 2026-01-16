use bevy::prelude::*;
use iluvatar_core::CameraFrame;

use crate::capture::CaptureConfig;
use crate::targets::SimulatedTarget;

pub struct ValidationPlugin;

impl Plugin for ValidationPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ValidationMetrics>()
            .add_systems(Update, (collect_ground_truth, log_validation_metrics));
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
}

impl ValidationMetrics {
    /// Record a detection and compare to ground truth
    pub fn record_detection(&mut self, detected_pos: Vec3, timestamp: f64) -> Option<f32> {
        // Find closest ground truth at this timestamp
        let closest = self
            .ground_truth
            .iter()
            .filter(|gt| (gt.timestamp - timestamp).abs() < 0.1)
            .min_by(|a, b| {
                let dist_a = a.position.distance(detected_pos);
                let dist_b = b.position.distance(detected_pos);
                dist_a.partial_cmp(&dist_b).unwrap()
            });

        if let Some(gt) = closest {
            let error = gt.position.distance(detected_pos);
            self.detection_errors.push(error);
            self.total_detections += 1;
            Some(error)
        } else {
            self.false_positives += 1;
            None
        }
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

                // Record this as a detection
                if let Some(error) = self.record_detection(centroid, timestamp_secs) {
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
    mut last_log: Local<f64>,
) {
    let now = time.elapsed_secs_f64();

    // Log every 5 seconds
    if now - *last_log >= 5.0 {
        *last_log = now;

        tracing::info!(
            "Validation Stats | Detections: {} | False Positives: {} | Mean Error: {:.2}m",
            metrics.total_detections,
            metrics.false_positives,
            metrics.mean_error().unwrap_or(0.0)
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
