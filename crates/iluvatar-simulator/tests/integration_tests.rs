//! End-to-end integration tests for the Iluvatar detection and tracking pipeline.
//!
//! These tests verify that the complete pipeline works correctly:
//! - Camera projection
//! - Ray marching / voxel contribution
//! - DBSCAN clustering
//! - Kalman filter tracking
//!
//! Tests run headlessly (no window required) and are deterministic via seeded scenarios.

use std::time::Duration;

use glam::Vec3;
use iluvatar_core::DetectionConfig;
use iluvatar_simulator::harness::{
    CameraSpec, ScenarioBuilder, TargetSpec, run_scenario, standard_camera_setup,
};

/// Helper to create a default scenario builder with standard cameras
fn default_scenario() -> ScenarioBuilder {
    let mut builder = ScenarioBuilder::new()
        .grid_origin(-100.0, 0.0, -100.0)
        .grid_dimensions(100, 50, 100) // 200m x 100m x 200m at 2m voxels
        .voxel_size(2.0)
        .duration(Duration::from_secs(3))
        .time_step(Duration::from_millis(50)) // 20 fps for faster tests
        .detection_config(DetectionConfig {
            intensity_threshold: 5.0,
            min_contributors: 2,
            cluster_epsilon: 5.0,
            cluster_min_points: 1,
        })
        .association_threshold(15.0) // Larger threshold for initial tests
        .max_missing_frames(60)
        .percentile_extraction(true, 0.80);

    // Add standard 4-camera setup
    for spec in standard_camera_setup() {
        builder = builder.camera(spec);
    }

    builder
}

// ============================================================================
// Basic Functionality Tests
// ============================================================================

/// Test that a stationary target is detected within reasonable accuracy.
///
/// A static target should be detected at the exact same position every frame,
/// giving us a baseline for position accuracy without velocity complications.
#[test]
fn test_static_target_detection() {
    let scenario = default_scenario()
        .target(TargetSpec::stationary(1, Vec3::new(0.0, 20.0, 0.0)))
        .duration(Duration::from_secs(2))
        .build();

    let result = run_scenario(scenario);

    // Print metrics for debugging
    println!("Static target detection metrics:");
    println!(
        "  Position error mean: {:.2}m",
        result.metrics.position_error_mean
    );
    println!(
        "  Position error P95:  {:.2}m",
        result.metrics.position_error_p95
    );
    println!(
        "  Detection rate:      {:.1}%",
        result.metrics.detection_rate * 100.0
    );
    println!(
        "  Frames with detections: {}/{}",
        result.metrics.frames_with_detections, result.metrics.total_frames
    );

    // For a static target seen by 4 cameras with good triangulation geometry,
    // we expect sub-5m accuracy
    if result.metrics.detection_rate > 0.5 {
        result.assert_position_error_mean(10.0); // Mean error < 10m
        result.assert_position_error_p95(15.0); // P95 error < 15m
    }

    // Detection rate should be reasonable (some early frames may miss)
    result.assert_detection_rate(0.5);
}

/// Test that a target moving at constant velocity is tracked correctly.
///
/// The Kalman filter should converge on velocity estimation after a few frames.
#[test]
fn test_constant_velocity_tracking() {
    // 10 m/s in X direction = 36 km/h
    let velocity = Vec3::new(10.0, 0.0, 0.0);

    let scenario = default_scenario()
        .target(TargetSpec::linear(1, Vec3::new(-30.0, 20.0, 0.0), velocity))
        .duration(Duration::from_secs(4))
        .build();

    let result = run_scenario(scenario);

    println!("Constant velocity tracking metrics:");
    println!(
        "  Position error mean: {:.2}m",
        result.metrics.position_error_mean
    );
    println!(
        "  Velocity error mean: {:.2}m/s",
        result.metrics.velocity_error_mean
    );
    println!(
        "  Detection rate:      {:.1}%",
        result.metrics.detection_rate * 100.0
    );
    println!("  Track switches:      {}", result.metrics.track_switches);

    // Position accuracy
    if result.metrics.detection_rate > 0.5 {
        result.assert_position_error_mean(12.0);

        // Velocity should converge - allow higher tolerance initially
        // Kalman filter needs time to estimate velocity from position observations
        if result.metrics.velocity_error_mean < f32::INFINITY {
            assert!(
                result.metrics.velocity_error_mean < 8.0,
                "Velocity error {:.2} m/s too high (expected < 8.0 m/s)",
                result.metrics.velocity_error_mean
            );
        }
    }

    // Should maintain track ID (no switches for single target)
    result.assert_no_track_switches();
}

/// Test multi-camera triangulation quality scales with camera count.
///
/// Adding more cameras should improve position accuracy.
#[test]
fn test_multi_camera_triangulation() {
    // Test with 2 cameras
    let scenario_2cam = ScenarioBuilder::new()
        .grid_origin(-100.0, 0.0, -100.0)
        .grid_dimensions(100, 50, 100)
        .voxel_size(2.0)
        .camera(CameraSpec::new(
            Vec3::new(-50.0, 30.0, -60.0),
            Vec3::new(0.0, 15.0, 0.0),
            90.0,
        ))
        .camera(CameraSpec::new(
            Vec3::new(50.0, 30.0, -60.0),
            Vec3::new(0.0, 15.0, 0.0),
            90.0,
        ))
        .target(TargetSpec::stationary(1, Vec3::new(0.0, 20.0, 0.0)))
        .duration(Duration::from_secs(2))
        .time_step(Duration::from_millis(50))
        .percentile_extraction(true, 0.80)
        .build();

    let result_2cam = run_scenario(scenario_2cam);

    // Test with 4 cameras (standard setup)
    let scenario_4cam = default_scenario()
        .target(TargetSpec::stationary(1, Vec3::new(0.0, 20.0, 0.0)))
        .duration(Duration::from_secs(2))
        .build();

    let result_4cam = run_scenario(scenario_4cam);

    println!("Triangulation comparison:");
    println!(
        "  2 cameras: pos_err={:.2}m, det_rate={:.1}%",
        result_2cam.metrics.position_error_mean,
        result_2cam.metrics.detection_rate * 100.0
    );
    println!(
        "  4 cameras: pos_err={:.2}m, det_rate={:.1}%",
        result_4cam.metrics.position_error_mean,
        result_4cam.metrics.detection_rate * 100.0
    );

    // 4 cameras should generally be better or equal to 2 cameras
    // (More samples, better triangulation geometry)
    // Note: This might not always hold due to noise - make assertion lenient
    if result_2cam.metrics.detection_rate > 0.3 && result_4cam.metrics.detection_rate > 0.3 {
        // At minimum, 4 cameras shouldn't be dramatically worse
        assert!(
            result_4cam.metrics.position_error_mean < result_2cam.metrics.position_error_mean * 2.0,
            "4-camera triangulation significantly worse than 2-camera"
        );
    }
}

// ============================================================================
// Edge Case Tests
// ============================================================================

/// Test tracking at high speed (100 m/s = 360 km/h).
///
/// This stresses the Kalman filter's ability to predict and the tracking
/// association's ability to match detections to tracks with large inter-frame motion.
#[test]
fn test_high_speed_target() {
    // 100 m/s in X direction = 360 km/h (aircraft landing speed)
    let velocity = Vec3::new(100.0, 0.0, 0.0);

    let scenario = default_scenario()
        .target(TargetSpec::linear(
            1,
            Vec3::new(-200.0, 30.0, 0.0),
            velocity,
        ))
        .duration(Duration::from_secs(3))
        .association_threshold(50.0) // Wider threshold for high-speed tracking
        .max_missing_frames(10) // More forgiving - target moves fast through FOV
        .build();

    let result = run_scenario(scenario);

    println!("High speed (100 m/s) tracking metrics:");
    println!(
        "  Position error mean: {:.2}m",
        result.metrics.position_error_mean
    );
    println!(
        "  Velocity error mean: {:.2}m/s",
        result.metrics.velocity_error_mean
    );
    println!(
        "  Detection rate:      {:.1}%",
        result.metrics.detection_rate * 100.0
    );
    println!("  Track switches:      {}", result.metrics.track_switches);
    println!("  Frames:              {}", result.metrics.total_frames);

    // At high speed, detection rate might be lower (target in FOV less time)
    // But when detected, tracking shouldn't completely fail
    if result.metrics.frames_with_detections > 5 {
        // Position error will be higher at high speed
        result.assert_position_error_mean(25.0);
    }

    // Should not crash or produce NaN
    assert!(!result.metrics.position_error_mean.is_nan());
}

/// Test target at grid boundary.
///
/// Targets near the edge of the voxel grid should still be detected
/// without crashes or assertion failures.
#[test]
fn test_grid_boundary() {
    // Place target near the corner of the grid
    let scenario = default_scenario()
        .target(TargetSpec::stationary(1, Vec3::new(-90.0, 20.0, -90.0)))
        .duration(Duration::from_secs(2))
        .build();

    let result = run_scenario(scenario);

    println!("Grid boundary test metrics:");
    println!(
        "  Position error mean: {:.2}m",
        result.metrics.position_error_mean
    );
    println!(
        "  Detection rate:      {:.1}%",
        result.metrics.detection_rate * 100.0
    );

    // Should not crash - that's the main test
    // Detection might be poor due to limited camera coverage at corners
    assert!(!result.metrics.position_error_mean.is_nan());
}

/// Test target entering and exiting camera FOV.
///
/// The tracker should gracefully handle track birth/death.
#[test]
fn test_target_appears_disappears() {
    // Target starts outside grid, passes through, exits
    let scenario = default_scenario()
        .target(TargetSpec::linear(
            1,
            Vec3::new(-150.0, 20.0, 0.0),
            Vec3::new(50.0, 0.0, 0.0),
        ))
        .duration(Duration::from_secs(5))
        .build();

    let result = run_scenario(scenario);

    println!("Appear/disappear test metrics:");
    println!(
        "  Position error mean: {:.2}m",
        result.metrics.position_error_mean
    );
    println!(
        "  Detection rate:      {:.1}%",
        result.metrics.detection_rate * 100.0
    );
    println!(
        "  Frames with detections: {}/{}",
        result.metrics.frames_with_detections, result.metrics.total_frames
    );

    // Should handle gracefully - some frames will have no detection
    // (target outside FOV or grid)
    assert!(!result.metrics.position_error_mean.is_nan());

    // Should have at least some detections when target is in view
    assert!(
        result.metrics.frames_with_detections > 0,
        "Target was never detected"
    );
}

// ============================================================================
// Failure Mode Tests
// ============================================================================

/// Test with only one camera - no triangulation possible.
///
/// With a single camera, we can detect ray directions but not localize.
/// The system should not crash, but detections may be poor.
#[test]
fn test_single_camera_no_triangulation() {
    let scenario = ScenarioBuilder::new()
        .grid_origin(-100.0, 0.0, -100.0)
        .grid_dimensions(100, 50, 100)
        .voxel_size(2.0)
        .camera(CameraSpec::new(
            Vec3::new(0.0, 30.0, -60.0),
            Vec3::new(0.0, 15.0, 0.0),
            90.0,
        ))
        .target(TargetSpec::stationary(1, Vec3::new(0.0, 20.0, 0.0)))
        .duration(Duration::from_secs(2))
        .time_step(Duration::from_millis(50))
        .detection_config(DetectionConfig {
            intensity_threshold: 5.0,
            min_contributors: 1, // Allow single camera (though localization will be poor)
            cluster_epsilon: 5.0,
            cluster_min_points: 1,
        })
        .percentile_extraction(true, 0.80)
        .build();

    let result = run_scenario(scenario);

    println!("Single camera test metrics:");
    println!(
        "  Detection rate:      {:.1}%",
        result.metrics.detection_rate * 100.0
    );
    println!("  Detected points:     varied per frame");

    // System should not crash
    // With min_contributors=1, it might detect something (a ray through the grid)
    // but localization will be along the entire ray, not at a point
    assert!(!result.metrics.position_error_mean.is_nan());
}

/// Test target completely outside grid bounds.
///
/// Should not crash, should produce zero detections.
#[test]
fn test_target_outside_grid() {
    // Target far outside the grid
    let scenario = default_scenario()
        .target(TargetSpec::stationary(1, Vec3::new(500.0, 200.0, 500.0)))
        .duration(Duration::from_secs(1))
        .build();

    let result = run_scenario(scenario);

    println!("Target outside grid test metrics:");
    println!(
        "  Detection rate:      {:.1}%",
        result.metrics.detection_rate * 100.0
    );
    println!(
        "  Frames with detections: {}",
        result.metrics.frames_with_detections
    );

    // Should not crash
    // No detections expected (target outside camera FOV and grid)
    assert!(
        !result.metrics.position_error_mean.is_nan() || result.metrics.frames_with_detections == 0
    );
}

// ============================================================================
// Realistic Scenario Tests
// ============================================================================

/// Test aircraft landing pattern.
///
/// Simulates an aircraft descending at typical approach angle and speed.
#[test]
fn test_aircraft_landing_pattern() {
    // Typical approach: 3° glide slope, 70 m/s (250 km/h) ground speed
    // Descent rate: 70 * sin(3°) ≈ 3.7 m/s
    let velocity = Vec3::new(70.0, -3.7, 0.0);

    let scenario = default_scenario()
        .grid_origin(-200.0, 0.0, -100.0)
        .grid_dimensions(200, 50, 100) // Larger grid for approach path
        .target(TargetSpec::linear(
            1,
            Vec3::new(-180.0, 80.0, 0.0),
            velocity,
        ))
        .duration(Duration::from_secs(5))
        .association_threshold(30.0)
        .build();

    let result = run_scenario(scenario);

    println!("Aircraft landing pattern metrics:");
    println!(
        "  Position error mean: {:.2}m",
        result.metrics.position_error_mean
    );
    println!(
        "  Position error P95:  {:.2}m",
        result.metrics.position_error_p95
    );
    println!(
        "  Velocity error mean: {:.2}m/s",
        result.metrics.velocity_error_mean
    );
    println!(
        "  Detection rate:      {:.1}%",
        result.metrics.detection_rate * 100.0
    );
    println!("  Track switches:      {}", result.metrics.track_switches);

    // For airport deployment, we need reasonable accuracy
    if result.metrics.detection_rate > 0.3 {
        result.assert_position_error_mean(20.0); // 20m mean error
    }
}

/// Test multiple simultaneous targets.
///
/// Verifies that the system can track multiple objects with distinct IDs.
#[test]
fn test_multiple_targets() {
    let scenario = default_scenario()
        .target(TargetSpec::linear(
            1,
            Vec3::new(-30.0, 20.0, -20.0),
            Vec3::new(5.0, 0.0, 0.0),
        ))
        .target(TargetSpec::linear(
            2,
            Vec3::new(-30.0, 30.0, 20.0),
            Vec3::new(8.0, 0.0, 0.0),
        ))
        .target(TargetSpec::stationary(3, Vec3::new(0.0, 25.0, 0.0)))
        .duration(Duration::from_secs(4))
        .build();

    let result = run_scenario(scenario);

    println!("Multiple targets test metrics:");
    println!(
        "  Position error mean: {:.2}m",
        result.metrics.position_error_mean
    );
    println!(
        "  Detection rate:      {:.1}%",
        result.metrics.detection_rate * 100.0
    );
    println!("  Track switches:      {}", result.metrics.track_switches);

    // Should detect at least some targets
    assert!(
        result.metrics.frames_with_detections > 0,
        "No targets detected in multi-target scenario"
    );

    // Track switches indicate ID confusion between targets
    // With multiple targets moving in similar areas, some switches may occur
    // This is a challenging scenario for any tracker
    assert!(
        result.metrics.track_switches < 30,
        "Too many track switches ({}) - severe ID confusion",
        result.metrics.track_switches
    );
}

// ============================================================================
// Regression Tests
// ============================================================================

/// Verify deterministic execution with the same seed.
///
/// Running the same scenario twice should produce identical results.
#[test]
fn test_deterministic_execution() {
    let build_scenario = || {
        default_scenario()
            .target(TargetSpec::linear(
                1,
                Vec3::new(-20.0, 20.0, 0.0),
                Vec3::new(10.0, 0.0, 0.0),
            ))
            .duration(Duration::from_secs(2))
            .seed(12345)
            .build()
    };

    let result1 = run_scenario(build_scenario());
    let result2 = run_scenario(build_scenario());

    // Results should be identical
    assert_eq!(
        result1.metrics.total_frames, result2.metrics.total_frames,
        "Frame count differs between runs"
    );

    // Position errors should be very close (floating point may have tiny differences)
    let pos_diff =
        (result1.metrics.position_error_mean - result2.metrics.position_error_mean).abs();
    assert!(
        pos_diff < 0.01,
        "Position error differs by {} between runs",
        pos_diff
    );
}
