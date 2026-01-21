//! Headless simulation harness for integration testing
//!
//! This module provides a way to run simulations programmatically without
//! a window, enabling automated integration tests. The key insight is that
//! we can extract the pure logic (targets, cameras, raymarching, detection,
//! tracking) from the rendering concerns.
//!
//! # Architecture
//!
//! ```text
//! SimulationScenario (config)
//!       │
//!       ▼
//! HeadlessSimulator (logic runner)
//!       │
//!       ├── SimulatedWorld (state)
//!       │     ├── Targets with ground truth
//!       │     ├── Cameras with geometry
//!       │     └── VoxelGrid + Tracker
//!       │
//!       ▼
//! SimulationResult (assertions)
//! ```

use glam::{UVec3, Vec3};
use std::collections::HashMap;
use std::time::Duration;

use iluvatar_core::{
    AttenuationConfig, BoundingBox, DetectionConfig, DistortionModel, GeoPosition, Ray,
    RaymarchConfig, TrackedObject, VoxelContribution,
};
use iluvatar_server::detector::ObjectDetector;
use iluvatar_server::grid::SparseVoxelGrid;
use iluvatar_server::time::Clock;
use iluvatar_server::tracker::ObjectTracker;

use crate::camera::CaptureCamera;
use crate::voxels::SimulatorRaymarcher;

/// Specification for a camera in the test scenario
#[derive(Debug, Clone)]
pub struct CameraSpec {
    /// Camera position in world coordinates
    pub position: Vec3,
    /// Point the camera looks at
    pub look_at: Vec3,
    /// Horizontal field of view in degrees
    pub fov_degrees: f32,
}

impl CameraSpec {
    pub fn new(position: Vec3, look_at: Vec3, fov_degrees: f32) -> Self {
        Self {
            position,
            look_at,
            fov_degrees,
        }
    }
}

/// Specification for a target in the test scenario
#[derive(Debug, Clone)]
pub struct TargetSpec {
    /// Unique target ID
    pub id: u32,
    /// Initial position
    pub start: Vec3,
    /// Movement pattern
    pub pattern: TargetPattern,
}

/// Simplified movement patterns for tests
#[derive(Debug, Clone)]
pub enum TargetPattern {
    /// Stationary target
    Static,
    /// Linear motion with constant velocity (m/s)
    Linear { velocity: Vec3 },
    /// Oscillating between start and end
    Oscillate { end: Vec3, period_secs: f32 },
    /// Circular motion
    Circle { radius: f32, period_secs: f32 },
}

impl TargetSpec {
    pub fn stationary(id: u32, position: Vec3) -> Self {
        Self {
            id,
            start: position,
            pattern: TargetPattern::Static,
        }
    }

    pub fn linear(id: u32, start: Vec3, velocity: Vec3) -> Self {
        Self {
            id,
            start,
            pattern: TargetPattern::Linear { velocity },
        }
    }

    pub fn oscillate(id: u32, start: Vec3, end: Vec3, period_secs: f32) -> Self {
        Self {
            id,
            start,
            pattern: TargetPattern::Oscillate { end, period_secs },
        }
    }

    /// Compute target position at time t
    pub fn position_at(&self, t: f32) -> Vec3 {
        match &self.pattern {
            TargetPattern::Static => self.start,
            TargetPattern::Linear { velocity } => self.start + *velocity * t,
            TargetPattern::Oscillate { end, period_secs } => {
                let progress = (t * std::f32::consts::PI / period_secs).sin().abs();
                self.start.lerp(*end, progress)
            }
            TargetPattern::Circle {
                radius,
                period_secs,
            } => {
                let angle = t * std::f32::consts::TAU / period_secs;
                self.start + Vec3::new(angle.cos() * radius, 0.0, angle.sin() * radius)
            }
        }
    }

    /// Compute target velocity at time t
    pub fn velocity_at(&self, t: f32) -> Vec3 {
        match &self.pattern {
            TargetPattern::Static => Vec3::ZERO,
            TargetPattern::Linear { velocity } => *velocity,
            TargetPattern::Oscillate { end, period_secs } => {
                // Derivative of lerp with sin progress
                let omega = std::f32::consts::PI / period_secs;
                let direction = *end - self.start;
                let scale = omega * (t * omega).sin().signum() * (t * omega).cos();
                direction * scale
            }
            TargetPattern::Circle {
                radius,
                period_secs,
            } => {
                let omega = std::f32::consts::TAU / period_secs;
                let angle = t * omega;
                Vec3::new(
                    -angle.sin() * radius * omega,
                    0.0,
                    angle.cos() * radius * omega,
                )
            }
        }
    }
}

/// Configuration for a simulation scenario
#[derive(Debug, Clone)]
pub struct SimulationScenario {
    /// Grid origin position (local coordinates, typically around 0)
    pub grid_origin: Vec3,
    /// Grid size in voxels
    pub grid_dimensions: UVec3,
    /// Size of each voxel in meters
    pub voxel_size: f32,
    /// Camera specifications
    pub cameras: Vec<CameraSpec>,
    /// Target specifications
    pub targets: Vec<TargetSpec>,
    /// Simulation duration
    pub duration: Duration,
    /// Time step for simulation (frame interval)
    pub time_step: Duration,
    /// Decay rate for voxel intensities
    pub decay_rate: f32,
    /// Ray intensity for detected motion
    pub ray_intensity: f32,
    /// Detection configuration
    pub detection: DetectionConfig,
    /// Tracking association threshold
    pub association_threshold: f32,
    /// Max frames a track can be missing before removal
    pub max_missing_frames: u32,
    /// Use percentile-based extraction
    pub use_percentile_extraction: bool,
    /// Percentile threshold (0.0-1.0)
    pub extraction_percentile: f32,
    /// Clear grid each frame
    pub clear_grid_each_frame: bool,
    /// Random seed for deterministic behavior (if applicable)
    pub seed: u64,
}

impl Default for SimulationScenario {
    fn default() -> Self {
        Self {
            grid_origin: Vec3::new(-100.0, 0.0, -100.0),
            grid_dimensions: UVec3::new(100, 50, 100), // 200m x 100m x 200m at 2m voxels
            voxel_size: 2.0,
            cameras: vec![],
            targets: vec![],
            duration: Duration::from_secs(5),
            time_step: Duration::from_millis(16), // ~60 fps
            decay_rate: 5.0,
            ray_intensity: 10.0,
            detection: DetectionConfig {
                intensity_threshold: 5.0,
                min_contributors: 2,
                cluster_epsilon: 5.0,
                cluster_min_points: 1,
            },
            association_threshold: 10.0,
            max_missing_frames: 30,
            use_percentile_extraction: true,
            extraction_percentile: 0.80,
            clear_grid_each_frame: true,
            seed: 42,
        }
    }
}

/// Builder for constructing simulation scenarios
pub struct ScenarioBuilder {
    scenario: SimulationScenario,
}

impl ScenarioBuilder {
    pub fn new() -> Self {
        Self {
            scenario: SimulationScenario::default(),
        }
    }

    /// Set the grid origin (local coordinates)
    pub fn grid_origin(mut self, x: f32, y: f32, z: f32) -> Self {
        self.scenario.grid_origin = Vec3::new(x, y, z);
        self
    }

    /// Set the grid dimensions in voxels
    pub fn grid_dimensions(mut self, x: u32, y: u32, z: u32) -> Self {
        self.scenario.grid_dimensions = UVec3::new(x, y, z);
        self
    }

    /// Set voxel size in meters
    pub fn voxel_size(mut self, size: f32) -> Self {
        self.scenario.voxel_size = size;
        self
    }

    /// Add a camera
    pub fn camera(mut self, spec: CameraSpec) -> Self {
        self.scenario.cameras.push(spec);
        self
    }

    /// Add a target
    pub fn target(mut self, spec: TargetSpec) -> Self {
        self.scenario.targets.push(spec);
        self
    }

    /// Set simulation duration
    pub fn duration(mut self, duration: Duration) -> Self {
        self.scenario.duration = duration;
        self
    }

    /// Set time step (frame interval)
    pub fn time_step(mut self, step: Duration) -> Self {
        self.scenario.time_step = step;
        self
    }

    /// Set detection configuration
    pub fn detection_config(mut self, config: DetectionConfig) -> Self {
        self.scenario.detection = config;
        self
    }

    /// Set tracking association threshold
    pub fn association_threshold(mut self, threshold: f32) -> Self {
        self.scenario.association_threshold = threshold;
        self
    }

    /// Set maximum missing frames for tracking
    pub fn max_missing_frames(mut self, frames: u32) -> Self {
        self.scenario.max_missing_frames = frames;
        self
    }

    /// Configure percentile-based extraction
    pub fn percentile_extraction(mut self, enabled: bool, percentile: f32) -> Self {
        self.scenario.use_percentile_extraction = enabled;
        self.scenario.extraction_percentile = percentile;
        self
    }

    /// Set random seed for determinism
    pub fn seed(mut self, seed: u64) -> Self {
        self.scenario.seed = seed;
        self
    }

    /// Build the scenario
    pub fn build(self) -> SimulationScenario {
        self.scenario
    }
}

impl Default for ScenarioBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// A simulated camera with its transform and intrinsics
struct SimCamera {
    /// Camera component with intrinsics
    capture: CaptureCamera,
    /// Camera position in world space
    position: Vec3,
    /// Camera rotation (stored as forward/up vectors for ray generation)
    rotation: glam::Quat,
}

impl SimCamera {
    fn new(spec: &CameraSpec, camera_id: u32) -> Self {
        use glam::{UVec2, Vec2};
        use iluvatar_core::{CameraIntrinsics, Fov};

        let fov_h = spec.fov_degrees.to_radians();
        let fov_v = fov_h * 0.75; // Assume 4:3 aspect ratio

        let intrinsics = CameraIntrinsics {
            focal_length: Vec2::new(500.0, 500.0),
            principal_point: Vec2::new(640.0, 360.0),
            resolution: UVec2::new(1280, 720),
            fov: Fov {
                horizontal: fov_h,
                vertical: fov_v,
            },
            distortion: DistortionModel::None,
        };

        // Compute rotation from look_at
        let forward = (spec.look_at - spec.position).normalize();
        let rotation = glam::Quat::from_rotation_arc(Vec3::NEG_Z, forward);

        Self {
            capture: CaptureCamera::new(intrinsics, camera_id),
            position: spec.position,
            rotation,
        }
    }

    /// Get a Transform-like struct for projection
    fn transform(&self) -> SimTransform {
        SimTransform {
            translation: self.position,
            rotation: self.rotation,
        }
    }
}

/// Minimal transform for camera projection (avoids Bevy dependency in core logic)
struct SimTransform {
    translation: Vec3,
    rotation: glam::Quat,
}

// Implement the methods that CaptureCamera expects
impl SimTransform {
    #[allow(dead_code)]
    fn forward(&self) -> Vec3 {
        self.rotation * Vec3::NEG_Z
    }
}

// We need to adapt the CaptureCamera's project_point to work with SimTransform
// Since CaptureCamera expects a Bevy Transform, we'll duplicate the projection logic

fn project_point_sim(
    camera: &CaptureCamera,
    cam_transform: &SimTransform,
    world_point: Vec3,
) -> Option<(f32, f32)> {
    // Vector from camera to point
    let to_point = world_point - cam_transform.translation;

    // Transform to camera local space
    let local = cam_transform.rotation.inverse() * to_point;

    // In camera space: -Z is forward, X is right, Y is up
    // Point must be in front of camera (negative Z in local space)
    if local.z >= 0.0 {
        return None; // Behind camera
    }

    // Project to normalized device coordinates using FOV
    let half_fov_h = camera.intrinsics.fov.horizontal / 2.0;
    let half_fov_v = camera.intrinsics.fov.vertical / 2.0;

    // Angle from forward axis
    let angle_h = (local.x / -local.z).atan();
    let angle_v = (local.y / -local.z).atan();

    // Check if within FOV
    if angle_h.abs() > half_fov_h || angle_v.abs() > half_fov_v {
        return None; // Outside field of view
    }

    // Normalize to [-1, 1]
    let u = angle_h / half_fov_h;
    let v = angle_v / half_fov_v;

    Some((u, v))
}

fn ray_direction_sim(camera: &CaptureCamera, cam_transform: &SimTransform, u: f32, v: f32) -> Vec3 {
    let half_fov_h = camera.intrinsics.fov.horizontal / 2.0;
    let half_fov_v = camera.intrinsics.fov.vertical / 2.0;

    // Convert normalized coords to angles
    let angle_h = u * half_fov_h;
    let angle_v = v * half_fov_v;

    // Direction in camera local space (-Z is forward)
    let local_dir = Vec3::new(angle_h.tan(), angle_v.tan(), -1.0).normalize();

    // Transform to world space
    cam_transform.rotation * local_dir
}

/// Results from a single simulation frame
#[derive(Debug, Clone)]
pub struct FrameResult {
    /// Simulation time
    pub time: f32,
    /// Ground truth target states
    pub ground_truth: Vec<GroundTruth>,
    /// Tracked objects from the pipeline
    pub tracked_objects: Vec<TrackedObject>,
    /// Number of detected voxel points
    pub detected_points: usize,
    /// Number of clusters found
    pub cluster_count: usize,
}

/// Ground truth for a single target
#[derive(Debug, Clone)]
pub struct GroundTruth {
    pub id: u32,
    pub position: Vec3,
    pub velocity: Vec3,
}

/// Aggregated results from a complete simulation run
#[derive(Debug, Clone)]
pub struct SimulationResult {
    /// All frame results
    pub frames: Vec<FrameResult>,
    /// Computed metrics
    pub metrics: SimulationMetrics,
}

/// Computed metrics from the simulation
#[derive(Debug, Clone, Default)]
pub struct SimulationMetrics {
    /// Mean position error across all matched track-target pairs (meters)
    pub position_error_mean: f32,
    /// 95th percentile position error
    pub position_error_p95: f32,
    /// Maximum position error
    pub position_error_max: f32,
    /// Mean velocity error (m/s)
    pub velocity_error_mean: f32,
    /// 95th percentile velocity error
    pub velocity_error_p95: f32,
    /// Detection rate: fraction of frames where all targets were detected
    pub detection_rate: f32,
    /// Number of track ID switches (should be 0 for single targets)
    pub track_switches: usize,
    /// Total frames simulated
    pub total_frames: usize,
    /// Frames where at least one target was matched
    pub frames_with_detections: usize,
}

impl SimulationResult {
    /// Assert that position error is within tolerance
    pub fn assert_position_error_mean(&self, max: f32) {
        assert!(
            self.metrics.position_error_mean <= max,
            "Position error mean {:.2}m exceeds max {:.2}m",
            self.metrics.position_error_mean,
            max
        );
    }

    /// Assert that position error 95th percentile is within tolerance
    pub fn assert_position_error_p95(&self, max: f32) {
        assert!(
            self.metrics.position_error_p95 <= max,
            "Position error P95 {:.2}m exceeds max {:.2}m",
            self.metrics.position_error_p95,
            max
        );
    }

    /// Assert that velocity error is within tolerance
    pub fn assert_velocity_error_mean(&self, max: f32) {
        assert!(
            self.metrics.velocity_error_mean <= max,
            "Velocity error mean {:.2}m/s exceeds max {:.2}m/s",
            self.metrics.velocity_error_mean,
            max
        );
    }

    /// Assert that detection rate is above threshold
    pub fn assert_detection_rate(&self, min: f32) {
        assert!(
            self.metrics.detection_rate >= min,
            "Detection rate {:.2}% below minimum {:.2}%",
            self.metrics.detection_rate * 100.0,
            min * 100.0
        );
    }

    /// Assert no track ID switches occurred
    pub fn assert_no_track_switches(&self) {
        assert!(
            self.metrics.track_switches == 0,
            "Expected no track switches, got {}",
            self.metrics.track_switches
        );
    }
}

/// The headless simulation runner
pub struct HeadlessSimulator {
    scenario: SimulationScenario,
}

impl HeadlessSimulator {
    pub fn new(scenario: SimulationScenario) -> Self {
        Self { scenario }
    }

    /// Run the simulation and collect results
    pub fn run(&self) -> SimulationResult {
        // Set up the voxel grid
        let clock = Clock::new();
        let grid_size = Vec3::new(
            self.scenario.grid_dimensions.x as f32 * self.scenario.voxel_size,
            self.scenario.grid_dimensions.y as f32 * self.scenario.voxel_size,
            self.scenario.grid_dimensions.z as f32 * self.scenario.voxel_size,
        );
        let bounds = BoundingBox::new(
            self.scenario.grid_origin,
            self.scenario.grid_origin + grid_size,
        );

        let grid = SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            self.scenario.grid_dimensions,
            self.scenario.voxel_size,
            self.scenario.decay_rate,
            clock,
        );

        // Set up raymarching config
        let raymarch_config = RaymarchConfig {
            max_distance: 300.0,
            step_size: 0.5,
            attenuation: AttenuationConfig::Linear {
                max_distance: 300.0,
            },
        };

        // Set up cameras
        let cameras: Vec<SimCamera> = self
            .scenario
            .cameras
            .iter()
            .enumerate()
            .map(|(i, spec)| SimCamera::new(spec, i as u32))
            .collect();

        // Set up detector and tracker
        let mut detector = ObjectDetector::new(self.scenario.detection.clone());
        let mut tracker = ObjectTracker::new(
            self.scenario.association_threshold,
            self.scenario.max_missing_frames,
            60.0, // Assume 60 fps for dt calculation
        );

        // Run simulation
        let mut frames = Vec::new();
        let dt = self.scenario.time_step.as_secs_f32();
        let total_time = self.scenario.duration.as_secs_f32();
        let mut t = 0.0;

        while t <= total_time {
            // Clear grid if configured
            if self.scenario.clear_grid_each_frame {
                grid.clear();
            }

            // Collect ground truth
            let ground_truth: Vec<GroundTruth> = self
                .scenario
                .targets
                .iter()
                .map(|target| GroundTruth {
                    id: target.id,
                    position: target.position_at(t),
                    velocity: target.velocity_at(t),
                })
                .collect();

            // For each camera, project targets and raymarch
            for (camera_idx, camera) in cameras.iter().enumerate() {
                let camera_id = camera_idx as u64;
                let cam_transform = camera.transform();
                let raymarcher =
                    SimulatorRaymarcher::new(&raymarch_config, &bounds, self.scenario.voxel_size);

                let mut contributions: HashMap<(u32, u32, u32), f32> = HashMap::new();

                for gt in &ground_truth {
                    // Check if target is visible to this camera
                    if let Some((u, v)) =
                        project_point_sim(&camera.capture, &cam_transform, gt.position)
                    {
                        // Target is visible! Create a ray toward it
                        let ray_dir = ray_direction_sim(&camera.capture, &cam_transform, u, v);
                        let ray = Ray::new(
                            cam_transform.translation,
                            ray_dir,
                            self.scenario.ray_intensity,
                        );

                        // March the ray using 3D-DDA
                        raymarcher.march_ray(
                            &ray,
                            self.scenario.grid_dimensions,
                            &mut contributions,
                        );
                    }
                }

                // Convert to VoxelContribution and add to grid
                let voxel_contributions: Vec<VoxelContribution> = contributions
                    .into_iter()
                    .map(|((x, y, z), intensity)| VoxelContribution {
                        index: UVec3::new(x, y, z),
                        intensity,
                    })
                    .collect();

                grid.add_camera_contributions(camera_id, &voxel_contributions);
            }

            // Extract points from grid
            let active_cameras = cameras.len() as u8;
            let points = if self.scenario.use_percentile_extraction {
                grid.extract_points_percentile(
                    self.scenario.extraction_percentile,
                    self.scenario.detection.min_contributors,
                    active_cameras,
                )
            } else {
                grid.extract_points_with_camera_count(&self.scenario.detection, active_cameras)
            };

            let detected_points = points.len();

            // Run DBSCAN clustering
            let detections = detector.detect(&points);
            let cluster_count = detections.len();

            // Update tracker
            let tracked_objects = tracker.update(detections, dt);

            // Store frame result
            frames.push(FrameResult {
                time: t,
                ground_truth,
                tracked_objects,
                detected_points,
                cluster_count,
            });

            // Apply decay (for non-clear mode)
            if !self.scenario.clear_grid_each_frame {
                grid.apply_decay();
            }

            t += dt;
        }

        // Compute metrics
        let metrics = self.compute_metrics(&frames);

        SimulationResult { frames, metrics }
    }

    fn compute_metrics(&self, frames: &[FrameResult]) -> SimulationMetrics {
        let mut position_errors: Vec<f32> = Vec::new();
        let mut velocity_errors: Vec<f32> = Vec::new();
        let mut frames_with_detections = 0;
        let grid_origin = self.scenario.grid_origin;

        // Track ID consistency for switch detection
        let mut last_assignments: HashMap<u32, u64> = HashMap::new(); // target_id -> track_id
        let mut track_switches = 0;

        for frame in frames {
            let mut frame_has_detection = false;

            for gt in &frame.ground_truth {
                // Find closest track to this ground truth
                let mut best_dist = f32::MAX;
                let mut best_track: Option<&TrackedObject> = None;

                for track in &frame.tracked_objects {
                    // Track centroid is in grid-local coordinates
                    let track_world = track.centroid + grid_origin;
                    let dist = track_world.distance(gt.position);
                    if dist < best_dist && dist < 25.0 {
                        // 25m max for matching
                        best_dist = dist;
                        best_track = Some(track);
                    }
                }

                if let Some(track) = best_track {
                    frame_has_detection = true;
                    position_errors.push(best_dist);

                    // Check for track ID switch
                    if let Some(&prev_track_id) = last_assignments.get(&gt.id)
                        && prev_track_id != track.id
                    {
                        track_switches += 1;
                    }
                    last_assignments.insert(gt.id, track.id);

                    // Velocity error
                    if let Some(track_vel) = track.velocity {
                        let vel_err = (track_vel - gt.velocity).length();
                        velocity_errors.push(vel_err);
                    }
                }
            }

            if frame_has_detection {
                frames_with_detections += 1;
            }
        }

        // Compute statistics
        let total_frames = frames.len();

        // Position errors
        let position_error_mean = if position_errors.is_empty() {
            f32::INFINITY
        } else {
            position_errors.iter().sum::<f32>() / position_errors.len() as f32
        };

        let position_error_max = position_errors.iter().cloned().fold(0.0f32, f32::max);

        let position_error_p95 = if position_errors.is_empty() {
            f32::INFINITY
        } else {
            let mut sorted = position_errors.clone();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let idx = ((sorted.len() - 1) as f32 * 0.95).round() as usize;
            sorted[idx.min(sorted.len() - 1)]
        };

        // Velocity errors
        let velocity_error_mean = if velocity_errors.is_empty() {
            f32::INFINITY
        } else {
            velocity_errors.iter().sum::<f32>() / velocity_errors.len() as f32
        };

        let velocity_error_p95 = if velocity_errors.is_empty() {
            f32::INFINITY
        } else {
            let mut sorted = velocity_errors.clone();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let idx = ((sorted.len() - 1) as f32 * 0.95).round() as usize;
            sorted[idx.min(sorted.len() - 1)]
        };

        // Detection rate: frames where all targets were matched
        let expected_detections = frames.len() * self.scenario.targets.len();
        let actual_detections = position_errors.len();
        let detection_rate = if expected_detections > 0 {
            actual_detections as f32 / expected_detections as f32
        } else {
            1.0
        };

        SimulationMetrics {
            position_error_mean,
            position_error_p95,
            position_error_max,
            velocity_error_mean,
            velocity_error_p95,
            detection_rate,
            track_switches,
            total_frames,
            frames_with_detections,
        }
    }
}

/// Convenience function to run a scenario and get results
pub fn run_scenario(scenario: SimulationScenario) -> SimulationResult {
    HeadlessSimulator::new(scenario).run()
}

/// Create a standard 4-camera setup around a central area
pub fn standard_camera_setup() -> Vec<CameraSpec> {
    vec![
        // Front-left
        CameraSpec::new(
            Vec3::new(-50.0, 30.0, -60.0),
            Vec3::new(0.0, 15.0, 0.0),
            90.0,
        ),
        // Front-right
        CameraSpec::new(
            Vec3::new(50.0, 30.0, -60.0),
            Vec3::new(0.0, 15.0, 0.0),
            90.0,
        ),
        // Back-center
        CameraSpec::new(Vec3::new(0.0, 40.0, 70.0), Vec3::new(0.0, 15.0, 0.0), 90.0),
        // Overhead
        CameraSpec::new(Vec3::new(0.0, 80.0, 0.0), Vec3::new(0.0, 15.0, 0.0), 90.0),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_target_spec_static() {
        let target = TargetSpec::stationary(1, Vec3::new(10.0, 20.0, 30.0));
        assert_eq!(target.position_at(0.0), Vec3::new(10.0, 20.0, 30.0));
        assert_eq!(target.position_at(100.0), Vec3::new(10.0, 20.0, 30.0));
        assert_eq!(target.velocity_at(0.0), Vec3::ZERO);
    }

    #[test]
    fn test_target_spec_linear() {
        let target = TargetSpec::linear(1, Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0));
        assert_eq!(target.position_at(0.0), Vec3::ZERO);
        assert_eq!(target.position_at(1.0), Vec3::new(10.0, 0.0, 0.0));
        assert_eq!(target.position_at(2.0), Vec3::new(20.0, 0.0, 0.0));
        assert_eq!(target.velocity_at(0.0), Vec3::new(10.0, 0.0, 0.0));
    }

    #[test]
    fn test_scenario_builder() {
        let scenario = ScenarioBuilder::new()
            .grid_origin(-50.0, 0.0, -50.0)
            .grid_dimensions(50, 25, 50)
            .voxel_size(1.0)
            .camera(CameraSpec::new(
                Vec3::new(0.0, 30.0, -50.0),
                Vec3::ZERO,
                90.0,
            ))
            .target(TargetSpec::stationary(1, Vec3::new(0.0, 15.0, 0.0)))
            .duration(Duration::from_secs(1))
            .build();

        assert_eq!(scenario.cameras.len(), 1);
        assert_eq!(scenario.targets.len(), 1);
        assert_eq!(scenario.voxel_size, 1.0);
    }
}
