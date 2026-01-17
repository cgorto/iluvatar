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

fn spawn_targets(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let sphere_mesh = meshes.add(Sphere::new(1.5));
    let red_mat = materials.add(Color::srgb(1.0, 0.2, 0.2));
    let blue_mat = materials.add(Color::srgb(0.2, 0.2, 1.0));
    let green_mat = materials.add(Color::srgb(0.2, 1.0, 0.2));
    let purple_mat = materials.add(Color::srgb(0.8, 0.2, 0.8));

    // 1. Linear "Patrol" Targets
    commands.spawn((
        Mesh3d(sphere_mesh.clone()),
        MeshMaterial3d(red_mat.clone()),
        Transform::default(),
        Target { id: 1 },
        TargetPath::new_linear(Vec3::new(-50.0, 15.0, 0.0), Vec3::new(50.0, 15.0, 0.0), 0.5),
    ));

    // 2. Spiral Swarm
    for i in 0..5 {
        let angle = (i as f32) * std::f32::consts::TAU / 5.0;
        let offset = Vec3::new(angle.cos() * 20.0, 15.0, angle.sin() * 20.0);

        commands.spawn((
            Mesh3d(sphere_mesh.clone()),
            MeshMaterial3d(blue_mat.clone()),
            Transform::default(),
            Target { id: 10 + i },
            TargetPath {
                origin: offset,
                params: MovementPattern::Spiral {
                    radius: 10.0,
                    height: 5.0,
                    frequency: 2.0,
                },
                speed: 0.8,
                time_offset: angle, // Desynchronize them
            },
        ));
    }

    // 3. Chaos Lissajous
    for i in 0..3 {
        commands.spawn((
            Mesh3d(sphere_mesh.clone()),
            MeshMaterial3d(purple_mat.clone()),
            Transform::default(),
            Target { id: 20 + i },
            TargetPath {
                origin: Vec3::new(0.0, 25.0, 0.0),
                params: MovementPattern::Lissajous {
                    freq_x: 1.3 + (i as f32 * 0.1),
                    freq_z: 1.7 + (i as f32 * 0.1),
                    amp_x: 30.0,
                    amp_z: 30.0,
                },
                speed: 0.6,
                time_offset: (i as f32) * 10.0,
            },
        ));
    }

    println!("Spawned swarm of targets");
}

fn move_targets(time: Res<Time>, mut query: Query<(&mut Transform, &TargetPath), With<Target>>) {
    let t = time.elapsed_secs();

    for (mut transform, path) in query.iter_mut() {
        transform.translation = path.current_position(t);
    }
}
