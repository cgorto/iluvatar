use bevy::prelude::*;

pub struct TargetsPlugin;

// ============================================================================
// Bezier Path Implementation
// ============================================================================

/// A Bezier curve path defined by control points
#[derive(Clone, Debug)]
pub struct BezierPath {
    pub control_points: Vec<Vec3>,
}

impl BezierPath {
    /// Create a new Bezier path from control points
    pub fn new(control_points: Vec<Vec3>) -> Self {
        assert!(
            control_points.len() >= 2,
            "BezierPath requires at least 2 control points"
        );
        Self { control_points }
    }

    /// Create a simple linear path between two points
    pub fn linear(start: Vec3, end: Vec3) -> Self {
        Self::new(vec![start, end])
    }

    /// Create a quadratic Bezier (3 control points)
    pub fn quadratic(start: Vec3, control: Vec3, end: Vec3) -> Self {
        Self::new(vec![start, control, end])
    }

    /// Create a cubic Bezier (4 control points)
    pub fn cubic(start: Vec3, control1: Vec3, control2: Vec3, end: Vec3) -> Self {
        Self::new(vec![start, control1, control2, end])
    }

    /// Evaluate the curve at parameter t (0.0 to 1.0)
    pub fn evaluate(&self, t: f32) -> Vec3 {
        let t = t.clamp(0.0, 1.0);
        let n = self.control_points.len() - 1;
        let mut result = Vec3::ZERO;

        for (i, point) in self.control_points.iter().enumerate() {
            let basis = bernstein_basis(n, i, t);
            result += *point * basis;
        }

        result
    }

    /// Compute the tangent (direction) at parameter t
    pub fn tangent(&self, t: f32) -> Vec3 {
        // Use finite difference for robustness
        let dt = 0.001;
        let t1 = (t - dt).max(0.0);
        let t2 = (t + dt).min(1.0);

        let p1 = self.evaluate(t1);
        let p2 = self.evaluate(t2);

        (p2 - p1).normalize_or_zero()
    }

    /// Approximate the arc length of the curve by sampling
    pub fn arc_length(&self) -> f32 {
        self.arc_length_between(0.0, 1.0)
    }

    /// Approximate arc length between two t values
    pub fn arc_length_between(&self, t_start: f32, t_end: f32) -> f32 {
        let samples = 100;
        let mut length = 0.0;
        let mut prev = self.evaluate(t_start);

        for i in 1..=samples {
            let t = t_start + (t_end - t_start) * (i as f32 / samples as f32);
            let curr = self.evaluate(t);
            length += prev.distance(curr);
            prev = curr;
        }

        length
    }

    /// Find the t parameter for a given arc length distance from the start
    pub fn t_at_arc_length(&self, target_length: f32) -> f32 {
        let total_length = self.arc_length();
        if total_length == 0.0 {
            return 0.0;
        }

        // Binary search for the t value
        let mut low = 0.0f32;
        let mut high = 1.0f32;

        for _ in 0..20 {
            // 20 iterations gives good precision
            let mid = (low + high) / 2.0;
            let length_at_mid = self.arc_length_between(0.0, mid);

            if length_at_mid < target_length {
                low = mid;
            } else {
                high = mid;
            }
        }

        (low + high) / 2.0
    }
}

/// Compute Bernstein basis polynomial
fn bernstein_basis(n: usize, i: usize, t: f32) -> f32 {
    binomial(n, i) as f32 * t.powi(i as i32) * (1.0 - t).powi((n - i) as i32)
}

/// Compute binomial coefficient (n choose k)
fn binomial(n: usize, k: usize) -> usize {
    if k > n {
        return 0;
    }
    if k == 0 || k == n {
        return 1;
    }
    // Use the multiplicative formula to avoid overflow
    let k = k.min(n - k);
    let mut result = 1usize;
    for i in 0..k {
        result = result * (n - i) / (i + 1);
    }
    result
}

// ============================================================================
// Path Follower Component
// ============================================================================

/// How the path should loop when reaching the end
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum LoopMode {
    /// Stop at the end of the path
    Once,
    /// Loop back to the start
    #[default]
    Loop,
    /// Reverse direction at each end (ping-pong)
    PingPong,
}

/// Component for entities that follow a Bezier path
#[derive(Component)]
pub struct PathFollower {
    /// The path to follow
    pub path: BezierPath,
    /// Movement speed in units per second
    pub speed: f32,
    /// How to handle reaching the end
    pub loop_mode: LoopMode,
    /// Current position on the path (0.0 to 1.0 for arc-length parameterization)
    pub progress: f32,
    /// Current direction (1.0 = forward, -1.0 = backward for PingPong)
    pub direction: f32,
    /// Cached arc length for efficiency
    arc_length: f32,
}

impl PathFollower {
    /// Create a new path follower
    pub fn new(path: BezierPath, speed: f32, loop_mode: LoopMode) -> Self {
        let arc_length = path.arc_length();
        Self {
            path,
            speed,
            loop_mode,
            progress: 0.0,
            direction: 1.0,
            arc_length,
        }
    }

    /// Get the current position on the path
    pub fn current_position(&self) -> Vec3 {
        let t = self.path.t_at_arc_length(self.progress * self.arc_length);
        self.path.evaluate(t)
    }

    /// Get the current tangent direction
    pub fn current_tangent(&self) -> Vec3 {
        let t = self.path.t_at_arc_length(self.progress * self.arc_length);
        self.path.tangent(t) * self.direction
    }

    /// Update progress based on delta time, returns new position
    pub fn advance(&mut self, delta_secs: f32) -> Vec3 {
        if self.arc_length == 0.0 {
            return self.path.evaluate(0.0);
        }

        // Compute progress delta based on speed and arc length
        let progress_delta = (self.speed * delta_secs) / self.arc_length;
        self.progress += progress_delta * self.direction;

        // Handle looping
        match self.loop_mode {
            LoopMode::Once => {
                self.progress = self.progress.clamp(0.0, 1.0);
            }
            LoopMode::Loop => {
                while self.progress > 1.0 {
                    self.progress -= 1.0;
                }
                while self.progress < 0.0 {
                    self.progress += 1.0;
                }
            }
            LoopMode::PingPong => {
                if self.progress >= 1.0 {
                    self.progress = 2.0 - self.progress;
                    self.direction = -1.0;
                } else if self.progress <= 0.0 {
                    self.progress = -self.progress;
                    self.direction = 1.0;
                }
                self.progress = self.progress.clamp(0.0, 1.0);
            }
        }

        self.current_position()
    }
}

impl Plugin for TargetsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ActiveScenario>()
            .add_systems(Startup, spawn_scenario)
            .add_systems(Update, (move_targets, move_path_followers));
    }
}

// ============================================================================
// Test Scenarios
// ============================================================================

/// Specification for a single target in a scenario
#[derive(Clone, Debug)]
pub struct TargetSpec {
    pub motion: MotionSpec,
    pub size: f32,
    pub color: Color,
}

/// Motion specification for a target
#[derive(Clone, Debug)]
pub enum MotionSpec {
    /// Linear back-and-forth motion
    Linear { start: Vec3, end: Vec3, speed: f32 },
    /// Circular motion around a center point
    Circular {
        center: Vec3,
        radius: f32,
        altitude: f32,
        speed: f32,
    },
    /// Follow a Bezier path
    Path {
        points: Vec<Vec3>,
        speed: f32,
        loop_mode: LoopMode,
    },
    /// Stationary target
    Stationary { position: Vec3 },
}

/// A complete test scenario
#[derive(Clone, Debug)]
pub struct TestScenario {
    pub name: String,
    pub targets: Vec<TargetSpec>,
    pub duration: f32,
}

impl TestScenario {
    /// Single target moving linearly
    pub fn single_linear() -> Self {
        Self {
            name: "Single Linear".to_string(),
            targets: vec![TargetSpec {
                motion: MotionSpec::Linear {
                    start: Vec3::new(-100.0, 15.0, 0.0),
                    end: Vec3::new(100.0, 15.0, 0.0),
                    speed: 10.0,
                },
                size: 2.0,
                color: Color::srgb(1.0, 0.2, 0.2),
            }],
            duration: 20.0,
        }
    }

    /// Two targets crossing paths
    pub fn crossing_paths() -> Self {
        Self {
            name: "Crossing Paths".to_string(),
            targets: vec![
                TargetSpec {
                    motion: MotionSpec::Linear {
                        start: Vec3::new(-100.0, 20.0, 0.0),
                        end: Vec3::new(100.0, 20.0, 0.0),
                        speed: 15.0,
                    },
                    size: 2.0,
                    color: Color::srgb(1.0, 0.2, 0.2),
                },
                TargetSpec {
                    motion: MotionSpec::Linear {
                        start: Vec3::new(0.0, 20.0, -100.0),
                        end: Vec3::new(0.0, 20.0, 100.0),
                        speed: 12.0,
                    },
                    size: 2.0,
                    color: Color::srgb(0.2, 1.0, 0.2),
                },
            ],
            duration: 30.0,
        }
    }

    /// Multiple targets at different altitudes
    pub fn altitude_layers() -> Self {
        Self {
            name: "Altitude Layers".to_string(),
            targets: vec![
                TargetSpec {
                    motion: MotionSpec::Circular {
                        center: Vec3::ZERO,
                        radius: 50.0,
                        altitude: 20.0,
                        speed: 1.5,
                    },
                    size: 2.0,
                    color: Color::srgb(1.0, 0.5, 0.0),
                },
                TargetSpec {
                    motion: MotionSpec::Circular {
                        center: Vec3::ZERO,
                        radius: 40.0,
                        altitude: 50.0,
                        speed: -1.0, // Opposite direction
                    },
                    size: 2.5,
                    color: Color::srgb(0.0, 0.5, 1.0),
                },
                TargetSpec {
                    motion: MotionSpec::Linear {
                        start: Vec3::new(-80.0, 100.0, 0.0),
                        end: Vec3::new(80.0, 100.0, 0.0),
                        speed: 20.0,
                    },
                    size: 3.0,
                    color: Color::srgb(1.0, 1.0, 0.0),
                },
            ],
            duration: 60.0,
        }
    }

    /// Complex curved path for testing path following
    pub fn curved_path() -> Self {
        Self {
            name: "Curved Path".to_string(),
            targets: vec![TargetSpec {
                motion: MotionSpec::Path {
                    points: vec![
                        Vec3::new(-80.0, 15.0, -50.0),
                        Vec3::new(-40.0, 40.0, 0.0),
                        Vec3::new(40.0, 25.0, 30.0),
                        Vec3::new(80.0, 15.0, -20.0),
                    ],
                    speed: 8.0,
                    loop_mode: LoopMode::PingPong,
                },
                size: 2.5,
                color: Color::srgb(0.8, 0.2, 0.8),
            }],
            duration: 45.0,
        }
    }

    /// Figure-8 pattern using two connected curves
    pub fn figure_eight() -> Self {
        let r = 40.0;
        let h = 25.0;
        Self {
            name: "Figure Eight".to_string(),
            targets: vec![TargetSpec {
                motion: MotionSpec::Path {
                    points: vec![
                        Vec3::new(0.0, h, 0.0),
                        Vec3::new(r, h, r),
                        Vec3::new(0.0, h, 2.0 * r),
                        Vec3::new(-r, h, r),
                        Vec3::new(0.0, h, 0.0),
                        Vec3::new(r, h, -r),
                        Vec3::new(0.0, h, -2.0 * r),
                        Vec3::new(-r, h, -r),
                        Vec3::new(0.0, h, 0.0),
                    ],
                    speed: 12.0,
                    loop_mode: LoopMode::Loop,
                },
                size: 2.0,
                color: Color::srgb(0.2, 0.8, 0.8),
            }],
            duration: 60.0,
        }
    }

    /// Swarm of multiple targets
    pub fn swarm() -> Self {
        let colors = [
            Color::srgb(1.0, 0.2, 0.2),
            Color::srgb(0.2, 1.0, 0.2),
            Color::srgb(0.2, 0.2, 1.0),
            Color::srgb(1.0, 1.0, 0.2),
            Color::srgb(1.0, 0.2, 1.0),
            Color::srgb(0.2, 1.0, 1.0),
        ];

        let targets: Vec<TargetSpec> = (0..6)
            .map(|i| {
                let radius = 30.0 + (i as f32 * 10.0);
                TargetSpec {
                    motion: MotionSpec::Circular {
                        center: Vec3::ZERO,
                        radius,
                        altitude: 15.0 + (i as f32 * 5.0),
                        speed: 0.5 + (i as f32 * 0.2) * if i % 2 == 0 { 1.0 } else { -1.0 },
                    },
                    size: 1.5,
                    color: colors[i],
                }
            })
            .collect();

        Self {
            name: "Swarm".to_string(),
            targets,
            duration: 90.0,
        }
    }

    /// Get all available scenarios
    pub fn all() -> Vec<TestScenario> {
        vec![
            Self::single_linear(),
            Self::crossing_paths(),
            Self::altitude_layers(),
            Self::curved_path(),
            Self::figure_eight(),
            Self::swarm(),
        ]
    }

    /// Find a scenario by name
    pub fn by_name(name: &str) -> Option<TestScenario> {
        Self::all()
            .into_iter()
            .find(|s| s.name.eq_ignore_ascii_case(name))
    }
}

/// Resource holding the currently active scenario
#[derive(Resource)]
pub struct ActiveScenario {
    pub scenario: TestScenario,
    pub start_time: f64,
}

impl Default for ActiveScenario {
    fn default() -> Self {
        Self {
            scenario: TestScenario::crossing_paths(),
            start_time: 0.0,
        }
    }
}

/// Component marking an entity as a simulated target
#[derive(Component)]
pub struct SimulatedTarget {
    pub ground_truth_id: u32,
    pub velocity: Vec3,
}

/// Component for different motion patterns
#[derive(Component)]
pub enum MotionPattern {
    Linear {
        start: Vec3,
        end: Vec3,
        speed: f32,
    },
    Circular {
        center: Vec3,
        radius: f32,
        speed: f32,
    },
    Random {
        bounds: Vec3,
        speed: f32,
    },
}

/// Spawn initial test targets
fn spawn_scenario(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    scenario: Res<ActiveScenario>,
) {
    tracing::info!("Spawning scenario: {}", scenario.scenario.name);

    for (i, spec) in scenario.scenario.targets.iter().enumerate() {
        let id = (i + 1) as u32;

        // Determine initial position based on motion type
        let initial_pos = match &spec.motion {
            MotionSpec::Linear { start, .. } => *start,
            MotionSpec::Circular {
                center,
                radius,
                altitude,
                ..
            } => Vec3::new(center.x + radius, *altitude, center.z),
            MotionSpec::Path { points, .. } => points.first().copied().unwrap_or(Vec3::ZERO),
            MotionSpec::Stationary { position } => *position,
        };

        // Spawn the target entity
        let mut entity_commands = commands.spawn((
            Mesh3d(meshes.add(Sphere::new(spec.size).mesh())),
            MeshMaterial3d(materials.add(spec.color)),
            Transform::from_translation(initial_pos),
            SimulatedTarget {
                ground_truth_id: id,
                velocity: Vec3::ZERO,
            },
            Name::new(format!("Target {}", id)),
        ));

        // Add appropriate motion component
        match &spec.motion {
            MotionSpec::Linear { start, end, speed } => {
                entity_commands.insert(MotionPattern::Linear {
                    start: *start,
                    end: *end,
                    speed: *speed,
                });
            }
            MotionSpec::Circular {
                center,
                radius,
                altitude,
                speed,
            } => {
                entity_commands.insert(MotionPattern::Circular {
                    center: Vec3::new(center.x, *altitude, center.z),
                    radius: *radius,
                    speed: *speed,
                });
            }
            MotionSpec::Path {
                points,
                speed,
                loop_mode,
            } => {
                let path = BezierPath::new(points.clone());
                entity_commands.insert(PathFollower::new(path, *speed, *loop_mode));
            }
            MotionSpec::Stationary { .. } => {
                // No motion component needed
            }
        }
    }

    tracing::info!(
        "Spawned {} targets for scenario '{}'",
        scenario.scenario.targets.len(),
        scenario.scenario.name
    );
}

/// Update target positions based on their motion patterns
fn move_targets(
    time: Res<Time>,
    mut query: Query<(&mut Transform, &mut SimulatedTarget, &MotionPattern)>,
) {
    let t = time.elapsed_secs();

    for (mut transform, mut target, pattern) in query.iter_mut() {
        match pattern {
            MotionPattern::Linear { start, end, speed } => {
                let direction = (*end - *start).normalize();
                let total_dist = start.distance(*end);
                let progress = (t * speed) % (total_dist * 2.0);

                let pos = if progress < total_dist {
                    *start + direction * progress
                } else {
                    *end - direction * (progress - total_dist)
                };

                target.velocity = if progress < total_dist {
                    direction * *speed
                } else {
                    -direction * *speed
                };

                transform.translation = pos;
            }
            MotionPattern::Circular {
                center,
                radius,
                speed,
            } => {
                let angle = t * speed;
                let x = center.x + angle.cos() * radius;
                let z = center.z + angle.sin() * radius;

                let new_pos = Vec3::new(x, center.y, z);
                target.velocity = (new_pos - transform.translation) / time.delta_secs();
                transform.translation = new_pos;
            }
            MotionPattern::Random { bounds, speed } => {
                // Simple random walk
                let dx = (t * 1.1).sin() * speed * time.delta_secs();
                let dy = (t * 1.3).cos() * speed * time.delta_secs() * 0.1;
                let dz = (t * 0.9).sin() * speed * time.delta_secs();

                target.velocity = Vec3::new(dx, dy, dz) / time.delta_secs();
                transform.translation += Vec3::new(dx, dy, dz);

                // Clamp to bounds
                transform.translation = transform.translation.clamp(-*bounds, *bounds);
            }
        }
    }
}

/// Update target positions for path followers
fn move_path_followers(
    time: Res<Time>,
    mut query: Query<(&mut Transform, &mut SimulatedTarget, &mut PathFollower)>,
) {
    let delta = time.delta_secs();

    for (mut transform, mut target, mut follower) in query.iter_mut() {
        let old_pos = transform.translation;
        let new_pos = follower.advance(delta);

        // Compute velocity from position change
        if delta > 0.0 {
            target.velocity = (new_pos - old_pos) / delta;
        }

        transform.translation = new_pos;

        // Orient the target along the path tangent
        let tangent = follower.current_tangent();
        if tangent.length_squared() > 0.0 {
            transform.look_to(tangent, Vec3::Y);
        }
    }
}
