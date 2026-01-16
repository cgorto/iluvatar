use bevy::prelude::*;
use iluvatar_core::{CameraId, CameraIntrinsics, Fov};

use crate::capture::config::CaptureConfig;

pub struct CamerasPlugin;

impl Plugin for CamerasPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_simulated_cameras);
    }
}

/// Component marking an entity as a simulated Iluvatar camera
#[derive(Component)]
pub struct SimulatedCamera {
    pub camera_id: CameraId,
    pub intrinsics: CameraIntrinsics,
}

/// Spawn simulated cameras at fixed positions
pub fn spawn_simulated_cameras(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    config: Res<CaptureConfig>,
) {
    let camera_positions = [
        (Vec3::new(-100.0, 30.0, -100.0), "Camera 1"),
        (Vec3::new(100.0, 30.0, -100.0), "Camera 2"),
        (Vec3::new(-100.0, 30.0, 100.0), "Camera 3"),
        (Vec3::new(100.0, 30.0, 100.0), "Camera 4"),
        (Vec3::new(0.0, 50.0, 0.0), "Camera 5 (overhead)"),
    ];

    // Build intrinsics that match the actual render target resolution
    let intrinsics = CameraIntrinsics {
        focal_length: glam::Vec2::new(
            config.render_width as f32 * 0.78, // ~90° horizontal FOV
            config.render_height as f32 * 0.78,
        ),
        principal_point: glam::Vec2::new(
            config.render_width as f32 / 2.0,
            config.render_height as f32 / 2.0,
        ),
        resolution: glam::UVec2::new(config.render_width, config.render_height),
        fov: Fov {
            horizontal: std::f32::consts::FRAC_PI_2,
            vertical: std::f32::consts::FRAC_PI_4,
        },
    };

    for (i, (pos, _name)) in camera_positions.iter().enumerate() {
        // Look toward center
        let look_at = Vec3::new(0.0, 20.0, 0.0);
        let transform = Transform::from_translation(*pos).looking_at(look_at, Vec3::Y);

        // Visual representation of camera
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(4.0, 3.0, 5.0))),
            MeshMaterial3d(materials.add(Color::srgb(0.2, 0.2, 0.8))),
            transform,
            SimulatedCamera {
                camera_id: i as CameraId,
                intrinsics,
            },
        ));

        // Camera frustum visualization (simple cone)
        let frustum_mesh = meshes.add(Cone {
            radius: 20.0,
            height: 50.0,
        });

        commands.spawn((
            Mesh3d(frustum_mesh),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgba(0.5, 0.5, 1.0, 0.1),
                alpha_mode: AlphaMode::Blend,
                ..default()
            })),
            Transform::from_translation(*pos + transform.forward() * 25.0).with_rotation(
                transform.rotation * Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
            ),
        ));
    }
}
