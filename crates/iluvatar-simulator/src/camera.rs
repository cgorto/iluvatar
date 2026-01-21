//! Capture cameras using real iluvatar-core types
//!
//! This module provides simulated capture cameras that use the real
//! `CameraIntrinsics` and `Fov` types from iluvatar-core.

use bevy::prelude::*;
use glam::{UVec2, Vec2};

use iluvatar_core::{CameraIntrinsics, DistortionModel, Fov};

pub struct CameraPlugin;

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_capture_cameras);
    }
}

/// A simulated capture camera that projects targets mathematically
///
/// Uses real `CameraIntrinsics` from iluvatar-core for camera parameters.
#[derive(Component)]
pub struct CaptureCamera {
    /// Camera intrinsics (focal length, principal point, resolution, FOV)
    pub intrinsics: CameraIntrinsics,
    /// Unique camera ID (0-63 for bitmask tracking)
    pub camera_id: u32,
}

impl CaptureCamera {
    /// Create a new capture camera with the given intrinsics and ID
    pub fn new(intrinsics: CameraIntrinsics, camera_id: u32) -> Self {
        Self {
            intrinsics,
            camera_id,
        }
    }

    /// Create a default camera with typical settings
    pub fn with_id(camera_id: u32) -> Self {
        Self {
            intrinsics: default_intrinsics(),
            camera_id,
        }
    }

    /// Check if a world point is visible to this camera
    /// Returns Some((u, v)) normalized coordinates if visible, None otherwise
    pub fn project_point(
        &self,
        camera_transform: &Transform,
        world_point: Vec3,
    ) -> Option<(f32, f32)> {
        // Vector from camera to point
        let to_point = world_point - camera_transform.translation;

        // Transform to camera local space
        let local = camera_transform.rotation.inverse() * to_point;

        // In camera space: -Z is forward, X is right, Y is up
        // Point must be in front of camera (negative Z in local space)
        if local.z >= 0.0 {
            return None; // Behind camera
        }

        // Project to normalized device coordinates using FOV
        let half_fov_h = self.intrinsics.fov.horizontal / 2.0;
        let half_fov_v = self.intrinsics.fov.vertical / 2.0;

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

    /// Generate a ray direction for a given normalized coordinate
    /// u, v are in [-1, 1] range
    pub fn ray_direction(&self, camera_transform: &Transform, u: f32, v: f32) -> Vec3 {
        let half_fov_h = self.intrinsics.fov.horizontal / 2.0;
        let half_fov_v = self.intrinsics.fov.vertical / 2.0;

        // Convert normalized coords to angles
        let angle_h = u * half_fov_h;
        let angle_v = v * half_fov_v;

        // Direction in camera local space (-Z is forward)
        let local_dir = Vec3::new(angle_h.tan(), angle_v.tan(), -1.0).normalize();

        // Transform to world space
        camera_transform.rotation * local_dir
    }
}

/// Default camera intrinsics for simulation
fn default_intrinsics() -> CameraIntrinsics {
    CameraIntrinsics {
        focal_length: Vec2::new(500.0, 500.0),
        principal_point: Vec2::new(640.0, 360.0),
        resolution: UVec2::new(1280, 720),
        fov: Fov {
            horizontal: std::f32::consts::FRAC_PI_2, // 90 degrees
            vertical: std::f32::consts::FRAC_PI_3,   // 60 degrees
        },
        distortion: DistortionModel::None,
    }
}

/// Camera placement configuration
struct CameraPlacement {
    position: Vec3,
    look_at: Vec3,
    color: Color,
}

fn spawn_capture_cameras(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Multiple cameras positioned around the scene for triangulation
    let placements = [
        // Camera 0: Front-left, looking at center
        CameraPlacement {
            position: Vec3::new(-50.0, 30.0, -60.0),
            look_at: Vec3::new(0.0, 15.0, 0.0),
            color: Color::srgb(0.8, 0.2, 0.2), // Red
        },
        // Camera 1: Front-right, looking at center
        CameraPlacement {
            position: Vec3::new(50.0, 30.0, -60.0),
            look_at: Vec3::new(0.0, 15.0, 0.0),
            color: Color::srgb(0.2, 0.8, 0.2), // Green
        },
        // Camera 2: Back-center, looking at center (for better triangulation)
        CameraPlacement {
            position: Vec3::new(0.0, 40.0, 70.0),
            look_at: Vec3::new(0.0, 15.0, 0.0),
            color: Color::srgb(0.2, 0.2, 0.8), // Blue
        },
        // Camera 3: High overhead, looking down
        CameraPlacement {
            position: Vec3::new(0.0, 80.0, 0.0),
            look_at: Vec3::new(0.0, 15.0, 0.0),
            color: Color::srgb(0.8, 0.8, 0.2), // Yellow
        },
    ];

    let mesh = meshes.add(Cuboid::new(3.0, 2.0, 4.0));

    for (id, placement) in placements.iter().enumerate() {
        let transform =
            Transform::from_translation(placement.position).looking_at(placement.look_at, Vec3::Y);

        commands.spawn((
            Mesh3d(mesh.clone()),
            MeshMaterial3d(materials.add(placement.color)),
            transform,
            CaptureCamera::with_id(id as u32),
        ));

        println!(
            "Camera {} at {:?}, looking at {:?}",
            id, placement.position, placement.look_at
        );
    }

    println!(
        "Spawned {} capture cameras for triangulation",
        placements.len()
    );
}
