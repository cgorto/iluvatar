//! Integration tests for the server-side raymarching pipeline.
//!
//! Validates the full motion frame processing path that the K230 hardware uses:
//! MotionPixel[] -> Raymarcher::raymarch_into() -> FlatVoxelGrid::accumulate()
//! -> extract_points() -> ObjectDetector::detect() -> ObjectTracker::update()

use glam::{Quat, UVec2, UVec3, Vec2, Vec3};
use iluvatar_core::raymarch::Raymarcher;
use iluvatar_core::{
    BoundingBox, CameraIntrinsics, CameraPose, CoordinateMode, DetectedPoint, DetectionConfig,
    DistortionModel, Fov, GeoPosition, LocalizationStatus, MAX_CONTRIBUTIONS_PER_FRAME,
    MotionPixel, PoseUncertainty, RaymarchConfig, Timestamp,
};
use iluvatar_server::detector::ObjectDetector;
use iluvatar_server::flat_grid::FlatVoxelGrid;
use iluvatar_server::time::Clock;
use iluvatar_server::tracker::ObjectTracker;

// ============================================================================
// Test Fixtures
// ============================================================================

/// 1920x1080, focal_length=500, 90 deg HFOV, no distortion.
/// Matches the core raymarch test intrinsics.
fn test_intrinsics() -> CameraIntrinsics {
    CameraIntrinsics {
        focal_length: Vec2::new(500.0, 500.0),
        principal_point: Vec2::new(960.0, 540.0),
        resolution: UVec2::new(1920, 1080),
        fov: Fov {
            horizontal: std::f32::consts::FRAC_PI_2,
            vertical: std::f32::consts::FRAC_PI_4,
        },
        distortion: DistortionModel::None,
    }
}

/// Create a camera pose at the given altitude and orientation.
/// Position is always at lat=0, lon=0 (matching the grid world origin).
fn test_pose(altitude: f64, orientation: Quat) -> CameraPose {
    CameraPose {
        position: GeoPosition::new(0.0, 0.0, altitude),
        orientation,
        timestamp: 0 as Timestamp,
        uncertainty: PoseUncertainty::default(),
        status: LocalizationStatus::Nominal,
    }
}

/// Create a Raymarcher matching the 100x100x100 grid geometry.
/// Grid bounds: (0,0,0) to (100,100,100), 1m voxels.
fn test_raymarcher(intrinsics: &CameraIntrinsics) -> Raymarcher {
    Raymarcher::new(
        *intrinsics,
        RaymarchConfig::default(),
        BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
        1.0,
        GeoPosition::new(0.0, 0.0, 0.0),
        CoordinateMode::Gps,
        MAX_CONTRIBUTIONS_PER_FRAME,
    )
}

/// Create a FlatVoxelGrid matching the Raymarcher geometry.
/// 100x100x100 grid, 1m voxels, origin at (0,0,0), 100K max voxels.
fn test_flat_grid() -> FlatVoxelGrid {
    let clock = Clock::new();
    FlatVoxelGrid::with_max_voxels(
        GeoPosition::new(0.0, 0.0, 0.0),
        UVec3::new(100, 100, 100),
        1.0,
        0.5,
        clock,
        100_000,
    )
}

/// Detection config tuned for test scenarios.
/// Low threshold so raymarched intensities pass, min_contributors=2 for
/// intersection tests, min_points=1 so single-voxel clusters are detected.
fn test_detection_config() -> DetectionConfig {
    DetectionConfig {
        intensity_threshold: 1.0,
        min_contributors: 2,
        cluster_epsilon: 5.0,
        cluster_min_points: 1,
    }
}

/// Generate a block of motion pixels centered at (cx, cy) with the given
/// half-width, half-height, and uniform intensity.
fn pixel_block(cx: u16, cy: u16, half_w: u16, half_h: u16, intensity: u8) -> Vec<MotionPixel> {
    let mut pixels = Vec::new();
    let x_min = cx.saturating_sub(half_w);
    let x_max = cx.saturating_add(half_w);
    let y_min = cy.saturating_sub(half_h);
    let y_max = cy.saturating_add(half_h);
    for y in y_min..=y_max {
        for x in x_min..=x_max {
            pixels.push(MotionPixel::new(x, y, intensity));
        }
    }
    pixels
}

// ============================================================================
// B1: Single Camera Populates Grid
// ============================================================================

#[test]
fn test_single_camera_populates_grid() {
    // One camera at altitude 100m (identity orientation = looking North in ENU).
    // Feed center pixels. Assert: active voxels > 0, all intensities finite
    // and positive, count bounded by contribution limit.
    let intrinsics = test_intrinsics();
    let mut raymarcher = test_raymarcher(&intrinsics);
    let mut grid = test_flat_grid();
    let pose = test_pose(100.0, Quat::IDENTITY);

    // Block of pixels around image center.
    let pixels = pixel_block(960, 540, 5, 5, 128);
    assert!(!pixels.is_empty());

    let camera_bit = 1u64 << 0;
    raymarcher.raymarch_into(&pose, &pixels, &mut |key, intensity| {
        grid.accumulate(key, intensity, camera_bit);
    });

    let active = grid.active_count();
    assert!(
        active > 0,
        "Center pixels from a camera above the grid must produce voxels",
    );
    assert!(
        active <= MAX_CONTRIBUTIONS_PER_FRAME,
        "Active voxels ({}) must not exceed contribution limit ({})",
        active,
        MAX_CONTRIBUTIONS_PER_FRAME,
    );

    // Verify intensities via extraction (threshold=0 to get everything).
    let extract_config = DetectionConfig {
        intensity_threshold: 0.01,
        min_contributors: 1,
        cluster_epsilon: 5.0,
        cluster_min_points: 1,
    };
    let points = grid.extract_points(&extract_config);
    assert!(!points.is_empty());

    for point in &points {
        assert!(point.intensity.is_finite());
        assert!(point.intensity > 0.0);
    }
}

// ============================================================================
// B2: Two Camera Intersection
// ============================================================================

#[test]
fn test_two_camera_intersection() {
    // Two cameras at the same position (0,0,50) inside the grid, facing
    // orthogonal directions. Camera 0 looks North (+Y in ENU), Camera 1
    // looks East (+X in ENU). Their center rays cross at the camera position,
    // producing overlapping voxels. Extract with min_contributors=2 to find
    // only the intersection region.
    let intrinsics = test_intrinsics();
    let mut raymarcher_0 = test_raymarcher(&intrinsics);
    let mut raymarcher_1 = test_raymarcher(&intrinsics);
    let mut grid = test_flat_grid();

    // Camera 0: looking North (Quat::IDENTITY).
    let pose_0 = test_pose(50.0, Quat::IDENTITY);

    // Camera 1: looking East. Rotate -90 deg around Bevy Y so that the
    // Bevy -Z (forward) direction maps to +X in Bevy, which becomes +X (East)
    // in ENU after the Bevy-to-ENU transform.
    let pose_1 = test_pose(50.0, Quat::from_rotation_y(-std::f32::consts::FRAC_PI_2));

    // Feed center pixel blocks from both cameras.
    let pixels = pixel_block(960, 540, 3, 3, 200);

    let camera_bit_0 = 1u64 << 0;
    raymarcher_0.raymarch_into(&pose_0, &pixels, &mut |key, intensity| {
        grid.accumulate(key, intensity, camera_bit_0);
    });

    let camera_bit_1 = 1u64 << 1;
    raymarcher_1.raymarch_into(&pose_1, &pixels, &mut |key, intensity| {
        grid.accumulate(key, intensity, camera_bit_1);
    });

    // Extract only voxels seen by both cameras.
    let config = DetectionConfig {
        intensity_threshold: 1.0,
        min_contributors: 2,
        cluster_epsilon: 10.0,
        cluster_min_points: 1,
    };
    let points = grid.extract_points(&config);

    assert!(
        !points.is_empty(),
        "Two orthogonal cameras at the same position must produce overlapping voxels",
    );

    // At lat=0, lon=0 the ENU rotation maps altitude to the Y axis.
    // Camera altitude=50 → ENU position (0, 50, 0). Both cameras' rays
    // start at y=50, so their intersection voxels have y≈50.
    for point in &points {
        assert!(
            point.position.y > 40.0 && point.position.y < 60.0,
            "Intersection point y={} should be near camera altitude (50m)",
            point.position.y,
        );
    }
}

// ============================================================================
// B3: Full Pipeline — Detect and Track
// ============================================================================

#[test]
fn test_full_pipeline_detect_and_track() {
    // Two cameras, 5 frames of a stationary target region. Each frame:
    // reset masks -> raymarch both cameras -> extract -> detect -> track.
    // After convergence, expect at least 1 tracked object near the expected
    // position.
    let intrinsics = test_intrinsics();
    let mut raymarcher_0 = test_raymarcher(&intrinsics);
    let mut raymarcher_1 = test_raymarcher(&intrinsics);
    let mut grid = test_flat_grid();
    let mut detector = ObjectDetector::new(test_detection_config());
    let mut tracker = ObjectTracker::new(
        20.0, // association_threshold
        10,   // max_missing_frames
        60.0, // frame_rate
    );

    let pose_0 = test_pose(50.0, Quat::IDENTITY);
    let pose_1 = test_pose(50.0, Quat::from_rotation_y(-std::f32::consts::FRAC_PI_2));
    let pixels = pixel_block(960, 540, 3, 3, 200);
    let dt = 1.0 / 60.0;

    let mut last_tracked = Vec::new();

    for _frame in 0..5 {
        // Reset camera masks so min_contributors is evaluated per frame.
        grid.reset_camera_masks();

        // Raymarch Camera 0.
        let camera_bit_0 = 1u64 << 0;
        raymarcher_0.raymarch_into(&pose_0, &pixels, &mut |key, intensity| {
            grid.accumulate(key, intensity, camera_bit_0);
        });

        // Raymarch Camera 1.
        let camera_bit_1 = 1u64 << 1;
        raymarcher_1.raymarch_into(&pose_1, &pixels, &mut |key, intensity| {
            grid.accumulate(key, intensity, camera_bit_1);
        });

        // Extract -> Detect -> Track.
        let points = grid.extract_points(&test_detection_config());
        let detections = detector.detect(&points);
        last_tracked = tracker.update(detections, dt);
    }

    // After 5 frames of consistent input, the tracker should have converged.
    assert!(
        !last_tracked.is_empty(),
        "5 frames of consistent two-camera input must produce tracked objects",
    );

    // At lat=0, lon=0 the ENU rotation maps altitude to the Y axis.
    // Camera altitude=50 → ENU y=50. The tracked object should be near y=50.
    let obj = &last_tracked[0];
    assert!(
        obj.centroid.y > 30.0 && obj.centroid.y < 70.0,
        "Tracked object centroid y={} should be near the intersection region (~50m)",
        obj.centroid.y,
    );
    assert!(obj.point_count > 0);
    assert!(obj.confidence > 0.0);
}

// ============================================================================
// B4: Empty Motion Frame
// ============================================================================

#[test]
fn test_empty_motion_frame() {
    // Empty pixel slice must leave the grid unchanged.
    let intrinsics = test_intrinsics();
    let mut raymarcher = test_raymarcher(&intrinsics);
    let mut grid = test_flat_grid();
    let pose = test_pose(100.0, Quat::IDENTITY);

    assert_eq!(grid.active_count(), 0);

    let camera_bit = 1u64 << 0;
    raymarcher.raymarch_into(&pose, &[], &mut |key, intensity| {
        grid.accumulate(key, intensity, camera_bit);
    });

    assert_eq!(
        grid.active_count(),
        0,
        "Empty motion pixel input must not change the grid",
    );
}

// ============================================================================
// B5: Camera Looking Away From Grid
// ============================================================================

#[test]
fn test_camera_looking_away_from_grid() {
    // Camera at altitude 200m (above grid max z=100), looking straight up
    // (+Z in ENU). All rays go away from the grid, so no voxels are populated.
    let intrinsics = test_intrinsics();
    let mut raymarcher = test_raymarcher(&intrinsics);
    let mut grid = test_flat_grid();

    // Rotate +90 deg around Bevy X so center pixel (0,0,-1) in Bevy maps to
    // (0,1,0) in Bevy, which becomes (0,0,1) = Up in ENU.
    let looking_up = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
    let pose = test_pose(200.0, looking_up);

    let pixels = pixel_block(960, 540, 10, 10, 255);
    assert!(!pixels.is_empty());

    let camera_bit = 1u64 << 0;
    raymarcher.raymarch_into(&pose, &pixels, &mut |key, intensity| {
        grid.accumulate(key, intensity, camera_bit);
    });

    assert_eq!(
        grid.active_count(),
        0,
        "Camera at z=200 looking up should not hit grid (z=0..100)",
    );
}

// ============================================================================
// B6: Camera Bit Tracking
// ============================================================================

#[test]
fn test_camera_bit_tracking() {
    // Two cameras contribute to overlapping voxels. Extract with
    // min_contributors=2. Points should exist only where both cameras
    // contributed (their rays intersected).
    let intrinsics = test_intrinsics();
    let mut raymarcher_0 = test_raymarcher(&intrinsics);
    let mut raymarcher_1 = test_raymarcher(&intrinsics);
    let mut grid = test_flat_grid();

    let pose_0 = test_pose(50.0, Quat::IDENTITY);
    let pose_1 = test_pose(50.0, Quat::from_rotation_y(-std::f32::consts::FRAC_PI_2));

    let pixels = pixel_block(960, 540, 3, 3, 200);

    // Camera 0 only.
    let camera_bit_0 = 1u64 << 0;
    raymarcher_0.raymarch_into(&pose_0, &pixels, &mut |key, intensity| {
        grid.accumulate(key, intensity, camera_bit_0);
    });

    // With min_contributors=2, camera 0 alone should not produce points.
    let config_two_cameras = DetectionConfig {
        intensity_threshold: 1.0,
        min_contributors: 2,
        cluster_epsilon: 10.0,
        cluster_min_points: 1,
    };
    let points_single = grid.extract_points(&config_two_cameras);
    assert!(
        points_single.is_empty(),
        "Single camera should not produce points when min_contributors=2 (got {} points)",
        points_single.len(),
    );

    // With min_contributors=1, camera 0 alone should produce points.
    let config_one_camera = DetectionConfig {
        intensity_threshold: 1.0,
        min_contributors: 1,
        cluster_epsilon: 10.0,
        cluster_min_points: 1,
    };
    let points_one = grid.extract_points(&config_one_camera);
    assert!(
        !points_one.is_empty(),
        "Single camera with min_contributors=1 must produce points",
    );

    // Now add Camera 1.
    let camera_bit_1 = 1u64 << 1;
    raymarcher_1.raymarch_into(&pose_1, &pixels, &mut |key, intensity| {
        grid.accumulate(key, intensity, camera_bit_1);
    });

    // With min_contributors=2, intersection voxels should now appear.
    let points_two = grid.extract_points(&config_two_cameras);
    assert!(
        !points_two.is_empty(),
        "Two overlapping cameras must produce points with min_contributors=2",
    );

    // Intersection points must be a strict subset of total active voxels.
    let total_active = grid.active_count();
    assert!(
        points_two.len() < total_active,
        "Intersection points ({}) should be fewer than total active voxels ({})",
        points_two.len(),
        total_active,
    );
}

// ============================================================================
// B7: Server-Side vs Camera-Side Equivalence
// ============================================================================

#[test]
fn test_server_side_vs_camera_side_equivalence() {
    // Compare two processing paths for identical inputs:
    //
    // Path A (server): raymarch_into -> grid_a.accumulate()
    // Path B (camera): raymarch_motion_pixels -> grid_b.add_camera_contributions()
    //
    // Both grids should have identical active_count and matching voxel positions.
    let intrinsics = test_intrinsics();
    let mut raymarcher_a = Raymarcher::new(
        intrinsics,
        RaymarchConfig::default(),
        BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
        1.0,
        GeoPosition::new(0.0, 0.0, 0.0),
        CoordinateMode::Gps,
        MAX_CONTRIBUTIONS_PER_FRAME,
    );
    let mut raymarcher_b = Raymarcher::new(
        intrinsics,
        RaymarchConfig::default(),
        BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
        1.0,
        GeoPosition::new(0.0, 0.0, 0.0),
        CoordinateMode::Gps,
        MAX_CONTRIBUTIONS_PER_FRAME,
    );

    let clock_a = Clock::new();
    let clock_b = Clock::new();
    let mut grid_a = FlatVoxelGrid::with_max_voxels(
        GeoPosition::new(0.0, 0.0, 0.0),
        UVec3::new(100, 100, 100),
        1.0,
        0.5,
        clock_a,
        100_000,
    );
    let mut grid_b = FlatVoxelGrid::with_max_voxels(
        GeoPosition::new(0.0, 0.0, 0.0),
        UVec3::new(100, 100, 100),
        1.0,
        0.5,
        clock_b,
        100_000,
    );

    let pose = test_pose(50.0, Quat::IDENTITY);
    let pixels = pixel_block(960, 540, 5, 5, 150);
    let camera_id: u64 = 0;
    let camera_bit = 1u64 << camera_id;

    // Path A: server-side (raymarch_into -> accumulate).
    raymarcher_a.raymarch_into(&pose, &pixels, &mut |key, intensity| {
        grid_a.accumulate(key, intensity, camera_bit);
    });

    // Path B: camera-side (raymarch_motion_pixels -> add_camera_contributions).
    let contributions = raymarcher_b.raymarch_motion_pixels(&pose, &pixels);
    grid_b.add_camera_contributions(camera_id, &contributions);

    // Both grids should have the same number of active voxels.
    assert_eq!(
        grid_a.active_count(),
        grid_b.active_count(),
        "Server path ({}) and camera path ({}) must produce same active voxel count",
        grid_a.active_count(),
        grid_b.active_count(),
    );
    assert!(
        grid_a.active_count() > 0,
        "Both paths should produce non-zero voxels",
    );

    // Extract all points from both grids and compare positions and intensities.
    let extract_config = DetectionConfig {
        intensity_threshold: 0.01,
        min_contributors: 1,
        cluster_epsilon: 5.0,
        cluster_min_points: 1,
    };

    let mut points_a = grid_a.extract_points(&extract_config);
    let mut points_b = grid_b.extract_points(&extract_config);

    assert_eq!(
        points_a.len(),
        points_b.len(),
        "Extracted point counts must match: server={}, camera={}",
        points_a.len(),
        points_b.len(),
    );

    // Sort by position for deterministic comparison.
    let sort_key = |p: &DetectedPoint| {
        (
            (p.position.x * 1000.0) as i64,
            (p.position.y * 1000.0) as i64,
            (p.position.z * 1000.0) as i64,
        )
    };
    points_a.sort_by_key(sort_key);
    points_b.sort_by_key(sort_key);

    for (pa, pb) in points_a.iter().zip(points_b.iter()) {
        let pos_diff = (pa.position - pb.position).length();
        assert!(
            pos_diff < 0.01,
            "Position mismatch: server={:?}, camera={:?}",
            pa.position,
            pb.position,
        );

        let intensity_diff = (pa.intensity - pb.intensity).abs();
        assert!(
            intensity_diff < 0.01,
            "Intensity mismatch at {:?}: server={}, camera={}",
            pa.position,
            pa.intensity,
            pb.intensity,
        );
    }
}
