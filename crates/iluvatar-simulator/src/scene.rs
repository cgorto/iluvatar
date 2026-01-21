//! Simple scene setup - ground plane and basic lighting

use bevy::{camera_controller::free_camera::FreeCamera, prelude::*};
use bevy_egui::PrimaryEguiContext;

use crate::render_layers::{debug_camera_layers, light_layers, scene_layers};

pub struct ScenePlugin;

impl Plugin for ScenePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_scene);
    }
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Ground plane - large flat surface (layer 0 - scene geometry)
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(2000.0, 2000.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.5, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
        // Scene geometry on layer 0
        scene_layers(),
    ));

    // Simple ambient light
    commands.spawn(AmbientLight {
        color: Color::WHITE,
        brightness: 500.0,
        ..default()
    });

    // Directional light (no shadows for simplicity)
    // Lights must illuminate both scene geometry AND targets
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.5, 0.5, 0.0)),
        // Lights affect layers 0 and 1 (scene + targets)
        light_layers(),
    ));

    // Debug/viewer camera - free flying
    // This camera sees ALL layers (scene, targets, AND debug gizmos)
    // It also hosts the egui UI context and renders last (highest order)
    commands.spawn((
        Camera3d::default(),
        Camera {
            // Render after all render cameras (which have negative orders like -1, -2, -3, -4)
            // This ensures:
            // 1. The 3D scene is rendered correctly
            // 2. The egui UI is drawn on top of the 3D scene
            order: 100,
            ..default()
        },
        FreeCamera::default(),
        Transform::from_xyz(0.0, 80.0, 120.0).looking_at(Vec3::ZERO, Vec3::Y),
        // Debug camera sees everything: layers 0, 1, and 2
        debug_camera_layers(),
        // Explicitly mark this camera as the primary egui context host.
        // Without this, bevy_egui might attach to a render camera (which
        // renders to an image target, not the window) causing the UI to
        // not be visible.
        PrimaryEguiContext,
    ));
}
