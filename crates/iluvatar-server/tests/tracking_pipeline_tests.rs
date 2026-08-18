//! Integration tests for the tracking pipeline composition.
//!
//! These tests exercise the full server-side detection and tracking pipeline
//! over multiple frames with moving objects:
//!
//!   VoxelContribution[] → FlatVoxelGrid → extract_points() →
//!   ObjectDetector::detect() → ObjectTracker::update()
//!
//! Where `motion_pipeline_tests.rs` validates the raymarching path (pixel →
//! voxel), these tests validate the tracking path (voxel → tracked object)
//! across time. The key question: does the Kalman predictor's velocity
//! estimate help the Hungarian algorithm maintain correct track identity
//! when objects cross paths?

use glam::UVec3;
use iluvatar_core::{DetectionConfig, GeoPosition, ObjectId, VoxelContribution};
use iluvatar_server::detector::ObjectDetector;
use iluvatar_server::flat_grid::FlatVoxelGrid;
use iluvatar_server::time::Clock;
use iluvatar_server::tracker::ObjectTracker;
use std::sync::Arc;

// ============================================================================
// Test fixtures
// ============================================================================

/// Frame interval: 10 FPS = 100ms = 100_000 µs.
const FRAME_INTERVAL_MICROS: u64 = 100_000;

/// Time step in seconds (1 / 10 FPS).
const DT: f32 = 0.1;

/// Base timestamp (1 second into simulated time).
const BASE_TIME_MICROS: u64 = 1_000_000;

/// Create a test grid with simulated clock.
///
/// Grid: 100x100x100, 1m voxels, moderate decay (rate=2.0).
/// The epoch-based camera mask ensures only current-frame contributions
/// pass the min_contributors filter, so decay rate only affects how
/// quickly dead voxels are garbage-collected from the table.
fn test_grid_with_clock() -> (Arc<Clock>, FlatVoxelGrid) {
    let clock = Clock::new();
    clock.set_simulated_time(BASE_TIME_MICROS);
    let grid = FlatVoxelGrid::with_max_voxels(
        GeoPosition::new(0.0, 0.0, 0.0),
        UVec3::new(100, 100, 100),
        1.0, // voxel_size
        2.0, // decay_rate
        clock.clone(),
        100_000, // max_voxels
    );
    (clock, grid)
}

/// Detection config for tracking tests.
///
/// min_contributors=2 requires both cameras to see the voxel this frame.
/// cluster_epsilon=3.0 with cluster_min_points=2 means each object needs
/// at least 2 voxels within 3m to form a cluster.
fn test_detection_config() -> DetectionConfig {
    DetectionConfig {
        intensity_threshold: 1.0,
        min_contributors: 2,
        cluster_epsilon: 3.0,
        cluster_min_points: 2,
    }
}

/// Create VoxelContributions for a 3-voxel line along the X axis.
///
/// Returns voxels at (cx-1, cy, cz), (cx, cy, cz), (cx+1, cy, cz),
/// each with the given intensity. The 3-voxel cluster ensures DBSCAN
/// groups them (all within 1m < epsilon=3m) and satisfies min_points=2.
fn object_contributions(cx: u32, cy: u32, cz: u32, intensity: f32) -> Vec<VoxelContribution> {
    assert!(cx > 0);
    assert!(cx < 99);
    vec![
        VoxelContribution {
            index: UVec3::new(cx - 1, cy, cz),
            intensity,
        },
        VoxelContribution {
            index: UVec3::new(cx, cy, cz),
            intensity,
        },
        VoxelContribution {
            index: UVec3::new(cx + 1, cy, cz),
            intensity,
        },
    ]
}

// ============================================================================
// T1: Crossing Paths Maintain Identity
// ============================================================================

/// Two objects cross paths over 30 frames. The Hungarian algorithm + Kalman
/// prediction must maintain correct track identity through the crossing.
///
/// Object A: starts at (20, 45, 50), moves +2 voxels/frame in X (20 m/s).
/// Object B: starts at (80, 55, 50), moves -2 voxels/frame in X (-20 m/s).
///
/// Y separation is 10m, well above cluster_epsilon (3m), so DBSCAN always
/// produces two distinct clusters. The challenge is at frame 15 when both
/// objects share the same X coordinate — without Kalman-predicted positions,
/// the tracker could swap identities.
///
/// Timeline:
///   Frames 0-4:   Velocity convergence (Kalman learns ~20 m/s).
///   Frame 5:      Record track IDs by Y position.
///   Frames 5-29:  Verify both IDs persist every frame.
///   Frame 25:     Verify final positions match expected trajectories.
#[test]
fn test_crossing_paths_maintain_identity() {
    let (clock, mut grid) = test_grid_with_clock();
    let extract_config = test_detection_config();
    let mut detector = ObjectDetector::new(test_detection_config());
    let mut tracker = ObjectTracker::new(
        15.0, // association_threshold (meters)
        5,    // max_missing_frames
        10.0, // frame_rate (Hz)
    );

    // Object trajectories.
    let a_start_x: u32 = 20;
    let a_y: u32 = 45;
    let b_start_x: u32 = 80;
    let b_y: u32 = 55;
    let z: u32 = 50;
    let intensity = 10.0f32;

    // Track identity recorded after warmup.
    let mut track_id_a: Option<ObjectId> = None;
    let mut track_id_b: Option<ObjectId> = None;

    for frame in 0..30u32 {
        // Object positions this frame.
        let a_x = a_start_x + frame * 2;
        let b_x = b_start_x - frame * 2;

        // Advance simulated time.
        clock.set_simulated_time(BASE_TIME_MICROS + (frame as u64 + 1) * FRAME_INTERVAL_MICROS);

        // Pipeline step 1: reset camera masks (epoch advance).
        grid.reset_camera_masks();

        // Pipeline step 2: both cameras contribute to both objects.
        let mut contributions = Vec::with_capacity(12);
        contributions.extend(object_contributions(a_x, a_y, z, intensity));
        contributions.extend(object_contributions(b_x, b_y, z, intensity));

        grid.add_camera_contributions(0, &contributions);
        grid.add_camera_contributions(1, &contributions);

        // Pipeline step 3: decay.
        grid.apply_decay();

        // Pipeline step 4: extract → detect → track.
        let points = grid.extract_points(&extract_config);
        let detections = detector.detect(&points);
        let tracked = tracker.update(detections, DT);

        // After warmup (frame 5), record and verify track IDs.
        if frame == 5 {
            assert_eq!(
                tracked.len(),
                2,
                "Frame {}: expected 2 tracks, got {}",
                frame,
                tracked.len(),
            );

            // Identify tracks by Y position. Object A has y~45.5, B has y~55.5.
            for obj in &tracked {
                if obj.centroid.y < 50.0 {
                    track_id_a = Some(obj.id);
                } else {
                    track_id_b = Some(obj.id);
                }
            }

            assert!(track_id_a.is_some(), "Track A (y<50) not found at frame 5.");
            assert!(track_id_b.is_some(), "Track B (y>50) not found at frame 5.");
            assert_ne!(
                track_id_a.unwrap(),
                track_id_b.unwrap(),
                "Tracks A and B must have distinct IDs.",
            );
        }

        // From frame 5 onward, verify both track IDs persist.
        if frame >= 5 {
            let id_a = track_id_a.unwrap();
            let id_b = track_id_b.unwrap();

            let has_a = tracked.iter().any(|o| o.id == id_a);
            let has_b = tracked.iter().any(|o| o.id == id_b);

            assert!(
                has_a,
                "Frame {}: Track A (id={}) lost. Active tracks: {:?}",
                frame,
                id_a,
                tracked
                    .iter()
                    .map(|o| (o.id, o.centroid))
                    .collect::<Vec<_>>(),
            );
            assert!(
                has_b,
                "Frame {}: Track B (id={}) lost. Active tracks: {:?}",
                frame,
                id_b,
                tracked
                    .iter()
                    .map(|o| (o.id, o.centroid))
                    .collect::<Vec<_>>(),
            );
        }

        // Post-crossing verification at frame 25.
        // A started at x=20 moving right, now at x=20+50=70. World x = 70.5.
        // B started at x=80 moving left, now at x=80-50=30. World x = 30.5.
        if frame == 25 {
            let id_a = track_id_a.unwrap();
            let id_b = track_id_b.unwrap();

            let obj_a = tracked.iter().find(|o| o.id == id_a).unwrap();
            let obj_b = tracked.iter().find(|o| o.id == id_b).unwrap();

            // Track A (originally at low X) should now be at high X.
            assert!(
                obj_a.centroid.x > 55.0,
                "Frame 25: Track A should be at high X (>55), got x={:.1}",
                obj_a.centroid.x,
            );

            // Track B (originally at high X) should now be at low X.
            assert!(
                obj_b.centroid.x < 45.0,
                "Frame 25: Track B should be at low X (<45), got x={:.1}",
                obj_b.centroid.x,
            );

            // Velocity check: A should have positive X velocity, B negative.
            if let Some(vel_a) = obj_a.velocity {
                assert!(
                    vel_a.x > 0.0,
                    "Track A should have positive X velocity, got {:.1}",
                    vel_a.x,
                );
            }
            if let Some(vel_b) = obj_b.velocity {
                assert!(
                    vel_b.x < 0.0,
                    "Track B should have negative X velocity, got {:.1}",
                    vel_b.x,
                );
            }
        }
    }
}

// ============================================================================
// T2: Velocity Convergence Through Full Pipeline
// ============================================================================

/// Single object moving at constant velocity through the full pipeline.
///
/// Verifies that the Kalman filter learns velocity correctly when driven
/// through grid → DBSCAN → tracker (not just in isolation). The tracker's
/// unit tests prove the Kalman filter works with direct TrackedObject input;
/// this test proves it works when the input comes from the messy reality
/// of voxel accumulation, decay, and DBSCAN centroid estimation.
///
/// Object: starts at (10, 50, 50), moves +2 voxels/frame in X (20 m/s).
/// After 15 frames, velocity estimate should converge to ~20 m/s.
#[test]
fn test_pipeline_velocity_convergence() {
    let (clock, mut grid) = test_grid_with_clock();
    let extract_config = test_detection_config();
    let mut detector = ObjectDetector::new(test_detection_config());
    let mut tracker = ObjectTracker::new(
        15.0, // association_threshold
        5,    // max_missing_frames
        10.0, // frame_rate
    );

    let start_x: u32 = 10;
    let y: u32 = 50;
    let z: u32 = 50;
    let velocity_voxels_per_frame: u32 = 2; // 20 m/s at 1m voxels, 10 FPS.
    let intensity = 10.0f32;

    let mut first_track_id: Option<ObjectId> = None;

    for frame in 0..15u32 {
        let cx = start_x + frame * velocity_voxels_per_frame;

        // Advance time.
        clock.set_simulated_time(BASE_TIME_MICROS + (frame as u64 + 1) * FRAME_INTERVAL_MICROS);

        // Pipeline: reset → contribute → decay → extract → detect → track.
        grid.reset_camera_masks();

        let contributions = object_contributions(cx, y, z, intensity);
        grid.add_camera_contributions(0, &contributions);
        grid.add_camera_contributions(1, &contributions);

        grid.apply_decay();

        let points = grid.extract_points(&extract_config);
        let detections = detector.detect(&points);
        let tracked = tracker.update(detections, DT);

        assert_eq!(
            tracked.len(),
            1,
            "Frame {}: expected 1 track, got {}",
            frame,
            tracked.len(),
        );

        let obj = &tracked[0];

        // Track ID must remain stable.
        match first_track_id {
            None => first_track_id = Some(obj.id),
            Some(id) => assert_eq!(
                obj.id, id,
                "Frame {}: track ID changed from {} to {}",
                frame, id, obj.id,
            ),
        }

        // After 10 frames, velocity should have converged.
        // True velocity: 2 voxels/frame × 1m/voxel × 10 FPS = 20 m/s in X.
        if frame >= 10 {
            let vel = obj
                .velocity
                .expect("Velocity should be present after 10 frames.");
            let vel_error = (vel.x - 20.0).abs();
            assert!(
                vel_error < 4.0,
                "Frame {}: X velocity should be ~20 m/s, got {:.1} (error {:.1})",
                frame,
                vel.x,
                vel_error,
            );
            // Y and Z velocity should be near zero.
            assert!(
                vel.y.abs() < 2.0,
                "Frame {}: Y velocity should be ~0, got {:.1}",
                frame,
                vel.y,
            );
            assert!(
                vel.z.abs() < 2.0,
                "Frame {}: Z velocity should be ~0, got {:.1}",
                frame,
                vel.z,
            );
        }
    }
}

// ============================================================================
// T3: Object Appears, Persists, Disappears
// ============================================================================

/// Object appears at frame 5, persists through frame 15, then vanishes.
/// The tracker should create a track on appearance, maintain it while
/// detections arrive, and drop it after max_missing_frames (5 frames)
/// of no detections.
#[test]
fn test_object_lifecycle() {
    let (clock, mut grid) = test_grid_with_clock();
    let extract_config = test_detection_config();
    let mut detector = ObjectDetector::new(test_detection_config());
    let mut tracker = ObjectTracker::new(
        15.0, // association_threshold
        5,    // max_missing_frames
        10.0, // frame_rate
    );

    let cx: u32 = 50;
    let cy: u32 = 50;
    let cz: u32 = 50;
    let intensity = 10.0f32;

    let mut track_id: Option<ObjectId> = None;

    for frame in 0..25u32 {
        clock.set_simulated_time(BASE_TIME_MICROS + (frame as u64 + 1) * FRAME_INTERVAL_MICROS);

        grid.reset_camera_masks();

        // Object present only during frames 5-15.
        let object_present = (5..=15).contains(&frame);
        if object_present {
            let contributions = object_contributions(cx, cy, cz, intensity);
            grid.add_camera_contributions(0, &contributions);
            grid.add_camera_contributions(1, &contributions);
        }

        grid.apply_decay();

        let points = grid.extract_points(&extract_config);
        let detections = detector.detect(&points);
        let tracked = tracker.update(detections, DT);

        // Before appearance: no tracks.
        if frame < 5 {
            assert_eq!(
                tracked.len(),
                0,
                "Frame {}: no tracks expected before object appears.",
                frame,
            );
        }

        // During presence: exactly 1 track.
        if (5..=15).contains(&frame) {
            assert_eq!(
                tracked.len(),
                1,
                "Frame {}: expected 1 track while object present, got {}.",
                frame,
                tracked.len(),
            );
            match track_id {
                None => track_id = Some(tracked[0].id),
                Some(id) => assert_eq!(
                    tracked[0].id, id,
                    "Frame {}: track ID should be stable.",
                    frame,
                ),
            }
        }

        // After max_missing_frames (5) without detection, track is dropped.
        // Object vanishes after frame 15. Track should persist through frame 20
        // (5 missing frames), then be gone by frame 21.
        if frame >= 21 {
            assert_eq!(
                tracker.track_count(),
                0,
                "Frame {}: track should be dropped after {} missing frames.",
                frame,
                frame - 15,
            );
        }
    }
}
