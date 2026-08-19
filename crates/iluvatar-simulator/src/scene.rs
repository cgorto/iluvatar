//! Simple scene setup - ground plane and basic lighting

use bevy::{camera_controller::free_camera::FreeCamera, prelude::*};
use bevy_egui::{EguiGlobalSettings, PrimaryEguiContext};

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
    mut egui_global_settings: ResMut<EguiGlobalSettings>,
) {
    // The simulator has multiple off-screen cameras. Disable bevy_egui's
    // automatic camera selection and explicitly attach the UI below.
    egui_global_settings.auto_create_primary_context = false;
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
    // Renders last (highest order) so egui UI is drawn on top of the 3D scene
    commands.spawn((
        Camera3d::default(),
        // Pin egui to the actual viewer camera. Letting bevy_egui choose from the
        // simulator's off-screen capture cameras can attach the UI to a camera
        // without a render graph, leaving the controls invisible.
        PrimaryEguiContext,
        Camera {
            // Render after all render cameras (which have negative orders like -1, -2, -3, -4)
            order: 100,
            ..default()
        },
        FreeCamera::default(),
        Transform::from_xyz(0.0, 260.0, 420.0).looking_at(Vec3::new(0.0, 190.0, 0.0), Vec3::Y),
        // Debug camera sees everything: layers 0, 1, and 2
        debug_camera_layers(),
    ));
}
