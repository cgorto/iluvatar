//! Capture cameras using real iluvatar-core types
//!
//! This module provides simulated capture cameras that use the real
//! `CameraIntrinsics` and `Fov` types from iluvatar-core.

use bevy::prelude::*;
use glam::{UVec2, Vec2};

use iluvatar_core::{CameraIntrinsics, DistortionModel, Fov};

use crate::sim_config::SimulatorTomlConfig;

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
            horizontal: 2.618, // 150 degrees
            vertical: 2.618,   // 150 degrees
        },
        distortion: DistortionModel::None,
    }
}

/// Palette of colors for camera visualization markers
const CAMERA_COLORS: &[Color] = &[
    Color::srgb(0.8, 0.2, 0.2), // Red
    Color::srgb(0.2, 0.8, 0.2), // Green
    Color::srgb(0.2, 0.2, 0.8), // Blue
    Color::srgb(0.8, 0.8, 0.2), // Yellow
    Color::srgb(0.8, 0.2, 0.8), // Magenta
    Color::srgb(0.2, 0.8, 0.8), // Cyan
    Color::srgb(0.9, 0.5, 0.1), // Orange
    Color::srgb(0.5, 0.9, 0.1), // Lime
];

fn spawn_capture_cameras(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    sim_toml: Option<Res<SimulatorTomlConfig>>,
) {
    let mesh = meshes.add(Cuboid::new(3.0, 2.0, 4.0));

    if let Some(toml) = sim_toml.as_ref() {
        for entry in &toml.cameras {
            let resolution = entry.resolution_uvec2();
            let fov_h = entry.fov_h_rad();
            let fov_v = entry.fov_v_rad();
            let focal_length_x = (resolution.x as f32 / 2.0) / (fov_h / 2.0).tan();
            let focal_length_y = (resolution.y as f32 / 2.0) / (fov_v / 2.0).tan();

            let intrinsics = CameraIntrinsics {
                focal_length: Vec2::new(focal_length_x, focal_length_y),
                principal_point: Vec2::new(resolution.x as f32 / 2.0, resolution.y as f32 / 2.0),
                resolution,
                fov: Fov {
                    horizontal: fov_h,
                    vertical: fov_v,
                },
                distortion: DistortionModel::None,
            };

            let position = entry.position_vec3();
            let look_at = entry.look_at_vec3();
            let transform =
                Transform::from_translation(position).looking_at(look_at, Vec3::Y);
            let color = CAMERA_COLORS[entry.id as usize % CAMERA_COLORS.len()];

            commands.spawn((
                Mesh3d(mesh.clone()),
                MeshMaterial3d(materials.add(color)),
                transform,
                CaptureCamera::new(intrinsics, entry.id),
            ));

            println!(
                "Camera {} at {:?}, looking at {:?} ({}x{}, FOV {:.0}x{:.0}°)",
                entry.id, position, look_at,
                resolution.x, resolution.y,
                fov_h.to_degrees(), fov_v.to_degrees(),
            );
        }

        println!(
            "Spawned {} capture cameras from config",
            toml.cameras.len()
        );
    } else {
        // Default placements for backwards compatibility
        let defaults = [
            (Vec3::new(-50.0, 30.0, -60.0), Vec3::new(0.0, 100.0, 0.0)),
            (Vec3::new(50.0, 30.0, -60.0), Vec3::new(0.0, 100.0, 0.0)),
            (Vec3::new(0.0, 40.0, 70.0), Vec3::new(0.0, 100.0, 0.0)),
            (Vec3::new(0.0, 80.0, 0.0), Vec3::new(0.0, 100.0, 0.0)),
        ];

        for (id, &(position, look_at)) in defaults.iter().enumerate() {
            let transform =
                Transform::from_translation(position).looking_at(look_at, Vec3::Y);
            let color = CAMERA_COLORS[id % CAMERA_COLORS.len()];

            commands.spawn((
                Mesh3d(mesh.clone()),
                MeshMaterial3d(materials.add(color)),
                transform,
                CaptureCamera::with_id(id as u32),
            ));

            println!("Camera {} at {:?}, looking at {:?}", id, position, look_at);
        }

        println!("Spawned {} capture cameras (default layout)", defaults.len());
    }
}
