//! Simple moving targets

use bevy::prelude::*;

pub struct TargetsPlugin;

impl Plugin for TargetsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_targets)
            .add_systems(Update, move_targets);
    }
}

/// A moving target in the scene
#[derive(Component)]
pub struct Target {
    pub id: u32,
}

/// Defines how a target moves
#[derive(Component)]
pub struct TargetPath {
    pub start: Vec3,
    pub end: Vec3,
    pub speed: f32,
    pub progress: f32,
    pub direction: f32, // 1.0 or -1.0 for ping-pong
}

impl TargetPath {
    pub fn new(start: Vec3, end: Vec3, speed: f32) -> Self {
        Self {
            start,
            end,
            speed,
            progress: 0.0,
            direction: 1.0,
        }
    }

    pub fn current_position(&self) -> Vec3 {
        self.start.lerp(self.end, self.progress)
    }
}

fn spawn_targets(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Target 1: moves along X axis (red sphere)
    let target1_start = Vec3::new(-50.0, 15.0, 0.0);
    let target1_end = Vec3::new(50.0, 15.0, 0.0);

    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(3.0))),
        MeshMaterial3d(materials.add(Color::srgb(1.0, 0.2, 0.2))),
        Transform::from_translation(target1_start),
        Target { id: 1 },
        TargetPath::new(target1_start, target1_end, 0.3),
    ));

    // Target 2: moves along Z axis (blue sphere)
    let target2_start = Vec3::new(0.0, 15.0, -40.0);
    let target2_end = Vec3::new(0.0, 15.0, 40.0);

    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(3.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.2, 0.2, 1.0))),
        Transform::from_translation(target2_start),
        Target { id: 2 },
        TargetPath::new(target2_start, target2_end, 0.25),
    ));

    println!("Spawned 2 targets");
}

fn move_targets(
    time: Res<Time>,
    mut query: Query<(&mut Transform, &mut TargetPath), With<Target>>,
) {
    let dt = time.delta_secs();

    for (mut transform, mut path) in query.iter_mut() {
        // Update progress
        path.progress += path.speed * dt * path.direction;

        // Ping-pong at ends
        if path.progress >= 1.0 {
            path.progress = 1.0;
            path.direction = -1.0;
        } else if path.progress <= 0.0 {
            path.progress = 0.0;
            path.direction = 1.0;
        }

        // Update position
        transform.translation = path.current_position();
    }
}
