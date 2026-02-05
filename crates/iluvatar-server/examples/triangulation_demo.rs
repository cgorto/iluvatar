//! Integration test demonstrating multi-camera triangulation.
//!
//! This example bypasses the network layer entirely to prove the core
//! algorithms work together: multiple cameras detect motion, cast rays
//! into a shared voxel grid, and the intersection of rays triangulates
//! the actual object position.
//!
//! Run with: cargo run -p iluvatar-server --example triangulation_demo

use glam::{Quat, UVec2, UVec3, Vec2, Vec3};
use iluvatar_core::{
    CameraIntrinsics, CameraPose, DetectionConfig, DistortionModel, Fov, GeoPosition,
    LocalizationStatus, PoseUncertainty, RaymarchConfig, VoxelContribution,
};
use iluvatar_server::{
    detector::ObjectDetector, grid::SparseVoxelGrid, time::Clock, tracker::ObjectTracker,
};
use std::f32::consts::PI;

/// A simulated camera that can render a target at a known world position
#[allow(dead_code)]
struct SimCamera {
    id: u64,
    pose: CameraPose,
    intrinsics: CameraIntrinsics,
    resolution: (u32, u32),
}

impl SimCamera {
    fn new(id: u64, position: Vec3, look_at: Vec3, resolution: (u32, u32)) -> Self {
        // Compute orientation quaternion to look at target
        // Camera looks along +Z in its local space
        let forward = (look_at - position).normalize();
        let world_up = Vec3::Z;
        // Standard look-at: right = up × forward, then recompute up = forward × right
        let right = world_up.cross(forward).normalize();
        let up = forward.cross(right);

        // Build rotation matrix: columns are the camera's local axes in world space
        // We want +Z to be forward (toward look_at), +Y to be up, +X to be right
        let rotation = glam::Mat3::from_cols(right, up, forward);
        let orientation = Quat::from_mat3(&rotation);

        let pose = CameraPose {
            position: GeoPosition::new(0.0, 0.0, position.z as f64),
            orientation,
            timestamp: 0,
            uncertainty: PoseUncertainty::default(),
            status: LocalizationStatus::Nominal,
        };

        // Store local position for ray calculations
        let intrinsics = CameraIntrinsics {
            focal_length: Vec2::new(500.0, 500.0),
            principal_point: Vec2::new(resolution.0 as f32 / 2.0, resolution.1 as f32 / 2.0),
            resolution: UVec2::new(resolution.0, resolution.1),
            fov: Fov {
                horizontal: PI / 2.0, // 90 degree FOV
                vertical: PI / 2.0 * (resolution.1 as f32 / resolution.0 as f32),
            },
            distortion: DistortionModel::None,
        };

        Self {
            id,
            pose,
            intrinsics,
            resolution,
        }
    }

    /// Project a world point to pixel coordinates
    fn project(&self, world_pos: Vec3, camera_local_pos: Vec3) -> Option<(u32, u32)> {
        // Vector from camera to point in world space
        let to_point = world_pos - camera_local_pos;

        // Transform to camera space (inverse of orientation)
        let cam_space = self.pose.orientation.inverse() * to_point;

        // Point must be in front of camera (positive Z in camera space)
        if cam_space.z < 0.1 {
            return None;
        }

        // Project to normalized device coordinates
        let nx = cam_space.x / (cam_space.z * (self.intrinsics.fov.horizontal / 2.0).tan());
        let ny = -cam_space.y / (cam_space.z * (self.intrinsics.fov.vertical / 2.0).tan());

        // Check if within view frustum
        if nx.abs() > 1.0 || ny.abs() > 1.0 {
            return None;
        }

        // Convert to pixel coordinates
        let px = ((nx + 1.0) * 0.5 * self.resolution.0 as f32) as u32;
        let py = ((ny + 1.0) * 0.5 * self.resolution.1 as f32) as u32;

        Some((px.min(self.resolution.0 - 1), py.min(self.resolution.1 - 1)))
    }

    /// Generate voxel contributions for a target at the given world position.
    /// Simulates: frame capture → difference detection → ray marching
    fn observe_target(
        &self,
        target_pos: Vec3,
        camera_local_pos: Vec3,
        grid: &GridSetup,
    ) -> Vec<VoxelContribution> {
        // Project target to pixel coordinates
        let Some((px, py)) = self.project(target_pos, camera_local_pos) else {
            return Vec::new();
        };

        // Simulate a blob of motion pixels around the projected point
        let blob_radius = 2u32;
        let mut motion_pixels = Vec::new();

        for dy in 0..=blob_radius * 2 {
            for dx in 0..=blob_radius * 2 {
                let x = px.saturating_sub(blob_radius).saturating_add(dx);
                let y = py.saturating_sub(blob_radius).saturating_add(dy);

                if x < self.resolution.0 && y < self.resolution.1 {
                    // Intensity falls off from center
                    let dist = ((dx as f32 - blob_radius as f32).powi(2)
                        + (dy as f32 - blob_radius as f32).powi(2))
                    .sqrt();
                    let intensity = (1.0 - dist / blob_radius as f32).max(0.0) * 200.0;
                    if intensity > 10.0 {
                        motion_pixels.push((x, y, intensity));
                    }
                }
            }
        }

        // March rays and accumulate voxel contributions
        self.raymarch_pixels(&motion_pixels, camera_local_pos, grid)
    }

    /// Convert motion pixels to voxel contributions via ray marching
    fn raymarch_pixels(
        &self,
        pixels: &[(u32, u32, f32)],
        camera_local_pos: Vec3,
        grid: &GridSetup,
    ) -> Vec<VoxelContribution> {
        use std::collections::HashMap;

        let config = RaymarchConfig {
            max_distance: 1500.,
            step_size: 0.5,
            ..Default::default()
        };

        let mut contributions: HashMap<(u32, u32, u32), f32> = HashMap::new();

        for &(px, py, intensity) in pixels {
            // Convert pixel to normalized device coordinates
            let nx =
                (px as f32 - self.intrinsics.principal_point.x) / (self.resolution.0 as f32 / 2.0);
            let ny =
                (py as f32 - self.intrinsics.principal_point.y) / (self.resolution.1 as f32 / 2.0);

            // Direction in camera space
            let dir_camera = Vec3::new(
                nx * (self.intrinsics.fov.horizontal / 2.0).tan(),
                -ny * (self.intrinsics.fov.vertical / 2.0).tan(),
                1.0,
            )
            .normalize();

            // Transform to world space
            let dir_world = self.pose.orientation * dir_camera;

            // March the ray
            let mut t = 1.0; // Start slightly in front of camera
            while t < config.max_distance {
                let point = camera_local_pos + dir_world * t;

                if let Some(voxel_idx) = grid.world_to_voxel(point) {
                    let key = (voxel_idx.x, voxel_idx.y, voxel_idx.z);
                    *contributions.entry(key).or_insert(0.0) += intensity;
                }

                t += config.step_size;
            }
        }

        contributions
            .into_iter()
            .map(|((x, y, z), intensity)| VoxelContribution {
                index: UVec3::new(x, y, z),
                intensity,
            })
            .collect()
    }
}

/// Grid configuration helper
struct GridSetup {
    origin: Vec3,
    dimensions: UVec3,
    voxel_size: f32,
}

impl GridSetup {
    fn new(origin: Vec3, dimensions: UVec3, voxel_size: f32) -> Self {
        Self {
            origin,
            dimensions,
            voxel_size,
        }
    }

    fn world_to_voxel(&self, pos: Vec3) -> Option<UVec3> {
        let local = pos - self.origin;
        if local.x < 0.0 || local.y < 0.0 || local.z < 0.0 {
            return None;
        }

        let vx = (local.x / self.voxel_size) as u32;
        let vy = (local.y / self.voxel_size) as u32;
        let vz = (local.z / self.voxel_size) as u32;

        if vx >= self.dimensions.x || vy >= self.dimensions.y || vz >= self.dimensions.z {
            return None;
        }

        Some(UVec3::new(vx, vy, vz))
    }
}

fn main() {
    println!("=== Iluvatar Multi-Camera Triangulation Demo ===\n");

    // Set up a 100x100x50 meter grid (voxel size = 1m)
    let grid_setup = GridSetup::new(Vec3::ZERO, UVec3::new(1000, 1000, 1000), 1.0);

    // Create the server's voxel grid
    let clock = Clock::new();
    let grid = SparseVoxelGrid::new(
        GeoPosition::new(0.0, 0.0, 0.0),
        grid_setup.dimensions,
        grid_setup.voxel_size,
        0.5, // decay rate
        clock.clone(),
    );

    // Position 4 cameras around the perimeter, all looking at center

    let center = Vec3::new(500.0, 500.0, 200.0);
    let camera_positions = [
        Vec3::new(100.0, 500.0, 50.0), // West
        Vec3::new(900.0, 500.0, 50.0), // East
        Vec3::new(500.0, 100.0, 50.0), // South
        Vec3::new(500.0, 900.0, 50.0), // North
    ];
    let cameras: Vec<SimCamera> = camera_positions
        .iter()
        .enumerate()
        .map(|(i, &pos)| SimCamera::new(i as u64, pos, center, (640, 480)))
        .collect();

    println!(
        "Grid: {}x{}x{} voxels at {}m resolution",
        grid_setup.dimensions.x,
        grid_setup.dimensions.y,
        grid_setup.dimensions.z,
        grid_setup.voxel_size
    );
    println!("Cameras: {} positioned around perimeter\n", cameras.len());

    // Ground truth: target position
    let target_pos = Vec3::new(550.0, 480.0, 100.0);
    println!(
        "Ground truth target position: ({:.1}, {:.1}, {:.1})\n",
        target_pos.x, target_pos.y, target_pos.z
    );

    // Each camera observes the target and generates voxel contributions
    println!("Camera observations:");
    let mut total_contributions = 0;
    let num_cameras = cameras.len();

    for (i, camera) in cameras.iter().enumerate() {
        let contributions = camera.observe_target(target_pos, camera_positions[i], &grid_setup);
        println!(
            "  Camera {}: {} voxel contributions",
            i,
            contributions.len()
        );
        total_contributions += contributions.len();
        // Use camera index as camera_id for proper multi-camera tracking
        grid.add_camera_contributions(i as u64, &contributions);
    }

    println!("\nTotal contributions: {}", total_contributions);
    println!("Active voxels in grid: {}\n", grid.active_count());

    // Run detection pipeline
    let detection_config = DetectionConfig {
        intensity_threshold: 100.0,
        min_contributors: 2, // Require at least 2 cameras to see a voxel (triangulation!)
        cluster_epsilon: 5.0, // 5m cluster radius
        cluster_min_points: 3,
    };

    let points = grid.extract_points_with_camera_count(&detection_config, num_cameras as u8);
    println!("Detected {} points above threshold\n", points.len());

    // Run DBSCAN clustering
    let mut detector = ObjectDetector::new(detection_config.clone());
    let detected_objects = detector.detect(&points);

    println!("DBSCAN found {} clusters\n", detected_objects.len());

    // Run tracker (single frame, so mostly just assigns IDs)
    let mut tracker = ObjectTracker::new(10.0, 30, 60.0);
    let dt = 1.0 / 60.0; // 60 FPS
    let tracked_objects = tracker.update(detected_objects, dt);

    // Report results
    println!("=== Detection Results ===\n");

    if tracked_objects.is_empty() {
        println!("No objects detected!");
        println!("\nDiagnostics:");
        println!("  - Check if cameras can see the target");
        println!("  - Intensity threshold may be too high");
        println!("  - Grid bounds may not contain the target");
    } else {
        for obj in &tracked_objects {
            let error = (obj.centroid - target_pos).length();
            println!(
                "Object {} detected at ({:.1}, {:.1}, {:.1})",
                obj.id, obj.centroid.x, obj.centroid.y, obj.centroid.z
            );
            println!(
                "  Bounding box: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})",
                obj.bounding_box.min.x,
                obj.bounding_box.min.y,
                obj.bounding_box.min.z,
                obj.bounding_box.max.x,
                obj.bounding_box.max.y,
                obj.bounding_box.max.z
            );
            println!("  Points in cluster: {}", obj.point_count);
            println!("  Total intensity: {:.1}", obj.total_intensity);
            println!("  Confidence: {:.2}", obj.confidence);
            println!("  Position error: {:.2}m from ground truth\n", error);
        }

        let best = tracked_objects
            .iter()
            .min_by(|a, b| {
                let err_a = (a.centroid - target_pos).length();
                let err_b = (b.centroid - target_pos).length();
                err_a.partial_cmp(&err_b).unwrap()
            })
            .unwrap();

        let error = (best.centroid - target_pos).length();
        println!("=== Summary ===");
        println!("Best detection error: {:.2}m", error);

        if error < 5.0 {
            println!("Result: GOOD - Target localized within 5m accuracy");
        } else if error < 10.0 {
            println!("Result: ACCEPTABLE - Target localized within 10m accuracy");
        } else {
            println!("Result: POOR - Target localization error > 10m");
        }
    }

    // Part 2: Moving target with velocity tracking
    println!("\n\n=== Part 2: Moving Target Tracking ===\n");

    let mut tracker2 = ObjectTracker::new(5.0, 30, 60.0);

    // Target moves from (30, 30, 10) toward (70, 70, 10) at 2m/frame
    let start_pos = Vec3::new(300.0, 300.0, 300.0);
    let velocity = Vec3::new(20.0, 20.0, 0.0); // 2m/frame in X and Y

    println!(
        "Target path: ({:.0}, {:.0}, {:.0}) → ({:.0}, {:.0}, {:.0})",
        start_pos.x,
        start_pos.y,
        start_pos.z,
        start_pos.x + velocity.x * 10.0,
        start_pos.y + velocity.y * 10.0,
        start_pos.z
    );
    println!(
        "Ground truth velocity: ({:.1}, {:.1}, {:.1}) m/frame\n",
        velocity.x, velocity.y, velocity.z
    );

    let dt_ms: u64 = 16;

    for frame in 0..10 {
        let current_pos = start_pos + velocity * frame as f32;
        let _current_time = frame as u64 * dt_ms;

        // Create fresh grid for each frame (simulating decay clearing old data)
        let frame_clock = Clock::new();
        let frame_grid = SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            grid_setup.dimensions,
            grid_setup.voxel_size,
            0.5,
            frame_clock,
        );

        // All cameras observe the target
        for (i, camera) in cameras.iter().enumerate() {
            let contributions =
                camera.observe_target(current_pos, camera_positions[i], &grid_setup);
            frame_grid.add_camera_contributions(i as u64, &contributions);
        }

        // Detect objects
        let points =
            frame_grid.extract_points_with_camera_count(&detection_config, num_cameras as u8);
        let mut detector2 = ObjectDetector::new(detection_config.clone());
        let detected = detector2.detect(&points);

        // Track
        let frame_dt = dt_ms as f32 / 1000.0; // Convert ms to seconds
        let tracked = tracker2.update(detected, frame_dt);

        if let Some(obj) = tracked.first() {
            let pos_error = (obj.centroid - current_pos).length();
            // Velocity is in m/s, convert to m/frame by dividing by frame rate
            let vel_str = if let Some(v) = obj.velocity {
                format!("({:.1}, {:.1}, {:.1})", v.x / 60.0, v.y / 60.0, v.z / 60.0)
            } else {
                "N/A".to_string()
            };

            println!(
                "Frame {:2}: pos ({:5.1}, {:5.1}, {:5.1}) | error: {:4.1}m | vel: {} m/frame",
                frame, obj.centroid.x, obj.centroid.y, obj.centroid.z, pos_error, vel_str
            );
        }
    }

    println!(
        "\nTracking complete - velocity should converge to ~({:.1}, {:.1}, {:.1}) m/frame",
        velocity.x, velocity.y, velocity.z
    );
}
