//! Simple scene setup - ground plane and basic lighting

use bevy::{camera_controller::free_camera::FreeCamera, prelude::*};
use bevy_egui::{EguiContext, PrimaryEguiContext};

use crate::render_layers::{debug_camera_layers, light_layers, scene_layers};

pub struct ScenePlugin;

impl Plugin for ScenePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_scene)
            .add_systems(First, deduplicate_primary_egui_context);
    }
}

/// Ensure exactly one entity has `PrimaryEguiContext`.
///
/// bevy_egui may add `PrimaryEguiContext` + `EguiContext` to multiple entities
/// (windows, cameras). We keep it on the first entity that has an `EguiContext`
/// (typically the window) and strip it from all others.
///
/// This MUST be an exclusive system (takes `&mut World`) so the removal is immediate,
/// not deferred like `Commands`. Deferred removal would still leave duplicates when
/// `EguiPrimaryContextPass` runs later in the same frame.
fn deduplicate_primary_egui_context(world: &mut World) {
    let all_primaries: Vec<Entity> = world
        .query_filtered::<Entity, (With<PrimaryEguiContext>, With<EguiContext>)>()
        .iter(world)
        .collect();

    // Keep only the first one, remove from the rest
    for entity in all_primaries.iter().skip(1) {
        world.entity_mut(*entity).remove::<PrimaryEguiContext>();
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
    // Renders last (highest order) so egui UI is drawn on top of the 3D scene
    commands.spawn((
        Camera3d::default(),
        Camera {
            // Render after all render cameras (which have negative orders like -1, -2, -3, -4)
            order: 100,
            ..default()
        },
        FreeCamera::default(),
        Transform::from_xyz(0.0, 80.0, 120.0).looking_at(Vec3::ZERO, Vec3::Y),
        // Debug camera sees everything: layers 0, 1, and 2
        debug_camera_layers(),
    ));
}
