//! Simple moving targets
use crate::render_layers::target_layers;
use bevy::prelude::*;

const TARGET_MODEL_PATH: &str = "models/craft_speederD.glb";
const TARGET_SCALE: f32 = 20.0;

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
    pub origin: Vec3,
    pub params: MovementPattern,
    pub speed: f32,
    pub time_offset: f32,
}

#[derive(Clone, Copy)]
pub enum MovementPattern {
    Linear {
        end: Vec3,
    },
    Circle {
        radius: f32,
        axis: Vec3,
    },
    Spiral {
        radius: f32,
        height: f32,
        frequency: f32,
    },
    Lissajous {
        freq_x: f32,
        freq_z: f32,
        amp_x: f32,
        amp_z: f32,
    },
}

pub(crate) fn spawn_target(
    commands: &mut Commands,
    asset_server: &AssetServer,
    id: u32,
    path: TargetPath,
) -> Entity {
    let initial_position = path.current_position(0.0);

    commands
        .spawn((
            SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset(TARGET_MODEL_PATH))),
            Transform {
                translation: initial_position,
                scale: Vec3::splat(TARGET_SCALE),
                ..default()
            },
            Target { id },
            path,
            target_layers(),
        ))
        .id()
}

impl TargetPath {
    pub fn new_linear(start: Vec3, end: Vec3, speed: f32) -> Self {
        Self {
            origin: start,
            params: MovementPattern::Linear { end },
            speed,
            time_offset: 0.0,
        }
    }

    pub fn new_spiral(origin: Vec3, radius: f32, height: f32, speed: f32) -> Self {
        Self {
            origin,
            params: MovementPattern::Spiral {
                radius,
                height,
                frequency: 1.0,
            },
            speed,
            time_offset: 0.0,
        }
    }

    pub fn current_position(&self, time: f32) -> Vec3 {
        let t = (time * self.speed) + self.time_offset;

        match self.params {
            MovementPattern::Linear { end } => {
                // Ping-pong 0..1..0
                let progress = (t.sin() + 1.0) / 2.0;
                self.origin.lerp(end, progress)
            }
            MovementPattern::Circle { radius, axis: _ } => {
                // Simple circle on XZ plane for now
                let x = t.cos() * radius;
                let z = t.sin() * radius;
                self.origin + Vec3::new(x, 0.0, z)
            }
            MovementPattern::Spiral {
                radius,
                height,
                frequency,
            } => {
                let x = (t * frequency).cos() * radius;
                let z = (t * frequency).sin() * radius;
                // Height oscillates
                let y = (t * 0.5).sin() * height;
                self.origin + Vec3::new(x, y, z)
            }
            MovementPattern::Lissajous {
                freq_x,
                freq_z,
                amp_x,
                amp_z,
            } => {
                let x = (t * freq_x).sin() * amp_x;
                let z = (t * freq_z).cos() * amp_z;
                self.origin + Vec3::new(x, 0.0, z)
            }
        }
    }

    pub fn current_velocity(&self, time: f32) -> Vec3 {
        // Numerical differentiation for simplicity
        let dt = 0.01;
        let p0 = self.current_position(time);
        let p1 = self.current_position(time + dt);
        (p1 - p0) / dt
    }
}

fn spawn_targets(mut commands: Commands, asset_server: Res<AssetServer>) {
    // 1. Linear "Patrol" Target - High altitude, wide patrol
    // Moves from x=-400 to x=400 at altitude 200m
    let path = TargetPath::new_linear(
        Vec3::new(-400.0, 200.0, 0.0),
        Vec3::new(400.0, 200.0, 0.0),
        0.2, // Slower relative speed for the long distance
    );
    spawn_target(&mut commands, &asset_server, 1, path);

    // 2. Spiral Swarm - Loitering pattern
    // Larger radius (200m) and height (150m)
    // for i in 0..5 {
    //     let angle = (i as f32) * std::f32::consts::TAU / 5.0;
    //     let offset = Vec3::new(angle.cos() * 100.0, 150.0, angle.sin() * 100.0);

    //     commands.spawn((
    //         SceneRoot(
    //             asset_server.load(GltfAssetLabel::Scene(0).from_asset("models/craft_speederD.glb")),
    //         ),
    //         Transform::from_scale(Vec3::splat(5.0)),
    //         Target { id: 10 + i },
    //         TargetPath {
    //             origin: offset,
    //             params: MovementPattern::Spiral {
    //                 radius: 50.0,
    //                 height: 30.0,
    //                 frequency: 0.5,
    //             },
    //             speed: 0.8,
    //             time_offset: angle,
    //         },
    //         target_layers(),
    //     ));
    // }

    // 3. Chaos Lissajous - Erratic maneuvers across the airspace
    // Covering a large volume 300x300m
    // for i in 0..3 {
    //     commands.spawn((
    //         SceneRoot(
    //             asset_server.load(GltfAssetLabel::Scene(0).from_asset("models/craft_speederD.glb")),
    //         ),
    //         Transform::from_scale(Vec3::splat(5.0)),
    //         Target { id: 20 + i },
    //         TargetPath {
    //             origin: Vec3::new(0.0, 300.0, 0.0), // High altitude origin
    //             params: MovementPattern::Lissajous {
    //                 freq_x: 0.7 + (i as f32 * 0.1),
    //                 freq_z: 0.9 + (i as f32 * 0.1),
    //                 amp_x: 300.0,
    //                 amp_z: 300.0,
    //             },
    //             speed: 0.5,
    //             time_offset: (i as f32) * 10.0,
    //         },
    //         target_layers(),
    //     ));
    // }

    println!("Spawned airport-scale targets");
}

fn move_targets(time: Res<Time>, mut query: Query<(&mut Transform, &TargetPath), With<Target>>) {
    let t = time.elapsed_secs();

    for (mut transform, path) in query.iter_mut() {
        transform.translation = path.current_position(t);

        // Also make them look in the direction of movement
        let vel = path.current_velocity(t);
        if vel.length_squared() > 0.1 {
            // Use look_to (negative z is forward in bevy)
            // We might need to adjust based on model orientation.
            // Assuming model forward is -Z.
            transform.look_to(vel.normalize(), Vec3::Y);
        }
    }
}
