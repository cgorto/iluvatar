# iluvatar-simulator Implementation Guide

This crate provides a Bevy-based simulation environment for testing the Iluvatar system without physical hardware. It renders a 3D scene with moving targets, simulates camera captures, and can feed data through the detection pipeline for validation.

## Overview

```
iluvatar-simulator/
├── src/
│   ├── lib.rs           # Plugin composition
│   ├── main.rs          # Application entry point
│   ├── scene.rs         # Environment setup
│   ├── targets.rs       # Moving objects
│   ├── cameras.rs       # Simulated cameras
│   └── validation.rs    # Ground truth comparison
└── Cargo.toml
```

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         BEVY APP                                     │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐                │
│  │ ScenePlugin │   │TargetsPlugin│   │CamerasPlugin│                │
│  │ - Ground    │   │ - Spawn     │   │ - Positions │                │
│  │ - Lighting  │   │ - Motion    │   │ - Frustums  │                │
│  │ - Main cam  │   │ - Patterns  │   │ - Capture   │                │
│  └─────────────┘   └─────────────┘   └──────┬──────┘                │
│                                             │                        │
│                                             ▼                        │
│                          ┌──────────────────────────────┐           │
│                          │    Simulated Capture System  │           │
│                          │    - Render from each camera │           │
│                          │    - Compute difference      │           │
│                          │    - Generate contributions  │           │
│                          └──────────────┬───────────────┘           │
│                                         │                            │
│         ┌───────────────────────────────┼───────────────────────────┐
│         ▼                               ▼                           ▼
│  ┌─────────────┐                ┌─────────────┐              ┌──────────┐
│  │ Validation  │                │Network Send │              │ UI/Debug │
│  │ - Ground    │                │ (optional)  │              │ Overlay  │
│  │   truth     │                │ to server   │              │          │
│  │ - Metrics   │                └─────────────┘              └──────────┘
│  └─────────────┘                                                        │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Module Implementation Details

### scene.rs - Environment Setup

This module creates the simulation environment.

#### Current State
- Ground plane
- Ambient and directional lighting
- Main viewing camera

#### TODO: Configurable Terrain

```rust
use bevy::prelude::*;

#[derive(Resource)]
pub struct SceneConfig {
    pub ground_size: f32,
    pub terrain_enabled: bool,
    pub buildings_enabled: bool,
    pub time_of_day: f32,  // 0.0 = midnight, 12.0 = noon
}

impl Default for SceneConfig {
    fn default() -> Self {
        Self {
            ground_size: 1000.0,
            terrain_enabled: false,
            buildings_enabled: false,
            time_of_day: 10.0,
        }
    }
}

fn setup_scene(
    mut commands: Commands,
    config: Res<SceneConfig>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Ground
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(config.ground_size, config.ground_size))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.5, 0.3),
            perceptual_roughness: 0.9,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
        Name::new("Ground"),
    ));

    // Lighting based on time of day
    let sun_angle = (config.time_of_day - 6.0) / 12.0 * std::f32::consts::PI;
    let sun_direction = Vec3::new(0.0, sun_angle.sin(), -sun_angle.cos()).normalize();

    let sun_intensity = (sun_angle.sin() * 10000.0).max(0.0);

    commands.spawn(AmbientLight {
        color: Color::srgb(0.4, 0.4, 0.5),
        brightness: 100.0 + sun_intensity * 0.02,
        affects_lightmapped_meshes: true,
    });

    commands.spawn((
        DirectionalLight {
            illuminance: sun_intensity,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_rotation_arc(Vec3::NEG_Z, sun_direction)),
        Name::new("Sun"),
    ));

    // Optional: Add buildings for occlusion testing
    if config.buildings_enabled {
        spawn_buildings(&mut commands, &mut meshes, &mut materials);
    }
}

fn spawn_buildings(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    let building_positions = [
        (Vec3::new(50.0, 15.0, 50.0), Vec3::new(20.0, 30.0, 20.0)),
        (Vec3::new(-80.0, 25.0, -30.0), Vec3::new(30.0, 50.0, 25.0)),
        (Vec3::new(20.0, 10.0, -60.0), Vec3::new(15.0, 20.0, 15.0)),
    ];

    let building_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.5, 0.5, 0.55),
        ..default()
    });

    for (pos, size) in building_positions {
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(size.x, size.y, size.z))),
            MeshMaterial3d(building_material.clone()),
            Transform::from_translation(pos),
            Name::new("Building"),
            Occluder,  // Mark as occluder for visibility testing
        ));
    }
}

#[derive(Component)]
pub struct Occluder;
```

#### TODO: Camera Controls

Add orbital camera controls for the main view:

```rust
#[derive(Component)]
pub struct OrbitCamera {
    pub focus: Vec3,
    pub distance: f32,
    pub pitch: f32,
    pub yaw: f32,
}

fn orbit_camera_system(
    mut query: Query<(&mut Transform, &mut OrbitCamera)>,
    mouse: Res<ButtonInput<MouseButton>>,
    mut motion: EventReader<MouseMotion>,
    mut scroll: EventReader<MouseWheel>,
) {
    let mut delta = Vec2::ZERO;
    for event in motion.read() {
        delta += event.delta;
    }

    let mut scroll_delta = 0.0;
    for event in scroll.read() {
        scroll_delta += event.y;
    }

    for (mut transform, mut orbit) in query.iter_mut() {
        if mouse.pressed(MouseButton::Right) {
            orbit.yaw -= delta.x * 0.005;
            orbit.pitch = (orbit.pitch - delta.y * 0.005).clamp(-1.5, 1.5);
        }

        orbit.distance = (orbit.distance - scroll_delta * 5.0).clamp(10.0, 500.0);

        let rotation = Quat::from_euler(EulerRot::YXZ, orbit.yaw, orbit.pitch, 0.0);
        transform.translation = orbit.focus + rotation * Vec3::new(0.0, 0.0, orbit.distance);
        transform.look_at(orbit.focus, Vec3::Y);
    }
}
```

---

### targets.rs - Moving Objects

This module manages simulated targets that move through the scene.

#### Current State
- `SimulatedTarget` component with ground truth ID
- `MotionPattern` enum for different movement types
- Basic linear and circular motion

#### TODO: Path-Based Motion

```rust
#[derive(Component)]
pub struct PathFollower {
    pub path: BezierPath,
    pub speed: f32,
    pub loop_mode: LoopMode,
    current_t: f32,
}

pub struct BezierPath {
    pub control_points: Vec<Vec3>,
}

impl BezierPath {
    pub fn evaluate(&self, t: f32) -> Vec3 {
        // Cubic Bezier evaluation
        let n = self.control_points.len() - 1;
        let mut result = Vec3::ZERO;

        for (i, point) in self.control_points.iter().enumerate() {
            let basis = bernstein_basis(n, i, t);
            result += *point * basis;
        }

        result
    }

    pub fn tangent(&self, t: f32) -> Vec3 {
        // Derivative of Bezier curve for velocity direction
        let dt = 0.001;
        let p1 = self.evaluate(t);
        let p2 = self.evaluate((t + dt).min(1.0));
        (p2 - p1).normalize()
    }

    pub fn arc_length(&self) -> f32 {
        // Approximate arc length by sampling
        let samples = 100;
        let mut length = 0.0;
        let mut prev = self.evaluate(0.0);

        for i in 1..=samples {
            let t = i as f32 / samples as f32;
            let curr = self.evaluate(t);
            length += prev.distance(curr);
            prev = curr;
        }

        length
    }
}

pub enum LoopMode {
    Once,
    Loop,
    PingPong,
}

fn path_follower_system(
    time: Res<Time>,
    mut query: Query<(&mut Transform, &mut PathFollower, &mut SimulatedTarget)>,
) {
    for (mut transform, mut follower, mut target) in query.iter_mut() {
        let arc_length = follower.path.arc_length();
        let dt = time.delta_secs() * follower.speed / arc_length;

        follower.current_t += dt;

        match follower.loop_mode {
            LoopMode::Once => {
                follower.current_t = follower.current_t.min(1.0);
            }
            LoopMode::Loop => {
                follower.current_t = follower.current_t.fract();
            }
            LoopMode::PingPong => {
                let cycle = (follower.current_t / 2.0).floor() as i32;
                if cycle % 2 == 0 {
                    follower.current_t = follower.current_t.fract() * 2.0;
                } else {
                    follower.current_t = 2.0 - (follower.current_t.fract() * 2.0);
                }
                follower.current_t = follower.current_t.clamp(0.0, 1.0);
            }
        }

        let new_pos = follower.path.evaluate(follower.current_t);
        let tangent = follower.path.tangent(follower.current_t);

        target.velocity = tangent * follower.speed;
        transform.translation = new_pos;
        transform.look_to(tangent, Vec3::Y);
    }
}

fn bernstein_basis(n: usize, i: usize, t: f32) -> f32 {
    binomial(n, i) as f32 * t.powi(i as i32) * (1.0 - t).powi((n - i) as i32)
}

fn binomial(n: usize, k: usize) -> usize {
    if k > n { return 0; }
    if k == 0 || k == n { return 1; }
    binomial(n - 1, k - 1) + binomial(n - 1, k)
}
```

#### TODO: Spawn Scenarios

```rust
#[derive(Resource)]
pub struct TestScenario {
    pub name: String,
    pub targets: Vec<TargetSpec>,
    pub duration: f32,
}

pub struct TargetSpec {
    pub motion: MotionSpec,
    pub size: f32,
    pub color: Color,
}

pub enum MotionSpec {
    Linear { start: Vec3, end: Vec3, speed: f32 },
    Circular { center: Vec3, radius: f32, altitude: f32, speed: f32 },
    Path { points: Vec<Vec3>, speed: f32, loop_mode: LoopMode },
    Stationary { position: Vec3 },
}

impl TestScenario {
    pub fn single_linear() -> Self {
        Self {
            name: "Single Linear".to_string(),
            targets: vec![TargetSpec {
                motion: MotionSpec::Linear {
                    start: Vec3::new(-100.0, 10.0, 0.0),
                    end: Vec3::new(100.0, 10.0, 0.0),
                    speed: 10.0,
                },
                size: 2.0,
                color: Color::srgb(1.0, 0.0, 0.0),
            }],
            duration: 20.0,
        }
    }

    pub fn multiple_crossing() -> Self {
        Self {
            name: "Multiple Crossing".to_string(),
            targets: vec![
                TargetSpec {
                    motion: MotionSpec::Linear {
                        start: Vec3::new(-100.0, 20.0, 0.0),
                        end: Vec3::new(100.0, 20.0, 0.0),
                        speed: 15.0,
                    },
                    size: 2.0,
                    color: Color::srgb(1.0, 0.0, 0.0),
                },
                TargetSpec {
                    motion: MotionSpec::Linear {
                        start: Vec3::new(0.0, 20.0, -100.0),
                        end: Vec3::new(0.0, 20.0, 100.0),
                        speed: 12.0,
                    },
                    size: 2.0,
                    color: Color::srgb(0.0, 1.0, 0.0),
                },
            ],
            duration: 30.0,
        }
    }

    pub fn occlusion_test() -> Self {
        Self {
            name: "Occlusion Test".to_string(),
            targets: vec![TargetSpec {
                motion: MotionSpec::Path {
                    points: vec![
                        Vec3::new(-50.0, 15.0, 0.0),
                        Vec3::new(0.0, 15.0, 0.0),   // Behind building
                        Vec3::new(50.0, 15.0, 0.0),
                    ],
                    speed: 5.0,
                    loop_mode: LoopMode::PingPong,
                },
                size: 3.0,
                color: Color::srgb(0.0, 0.5, 1.0),
            }],
            duration: 60.0,
        }
    }
}
```

---

### cameras.rs - Simulated Cameras

This module simulates the Iluvatar camera units.

#### Current State
- Camera positions with visual representation
- Frustum cone visualization
- Basic intrinsics

#### TODO: Render-Based Capture

The key feature: render the scene from each camera's viewpoint to simulate actual captures.

```rust
use bevy::render::camera::RenderTarget;
use bevy::render::render_resource::{TextureUsages, Extent3d, TextureDimension, TextureFormat};

#[derive(Component)]
pub struct SimulatedCamera {
    pub camera_id: CameraId,
    pub intrinsics: CameraIntrinsics,
    pub render_target: Handle<Image>,
    pub previous_frame: Option<Vec<u8>>,
}

#[derive(Resource)]
pub struct CameraRenderTargets {
    pub targets: HashMap<CameraId, Handle<Image>>,
}

fn setup_camera_render_targets(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    cameras: Query<(Entity, &SimulatedCamera)>,
) {
    for (entity, sim_cam) in cameras.iter() {
        let size = Extent3d {
            width: sim_cam.intrinsics.resolution.x,
            height: sim_cam.intrinsics.resolution.y,
            depth_or_array_layers: 1,
        };

        let mut image = Image {
            texture_descriptor: TextureDescriptor {
                label: Some("camera_render_target"),
                size,
                dimension: TextureDimension::D2,
                format: TextureFormat::Rgba8UnormSrgb,
                mip_level_count: 1,
                sample_count: 1,
                usage: TextureUsages::TEXTURE_BINDING
                    | TextureUsages::COPY_SRC
                    | TextureUsages::RENDER_ATTACHMENT,
                view_formats: &[],
            },
            ..default()
        };
        image.resize(size);

        let image_handle = images.add(image);

        // Add render camera to the simulated camera entity
        commands.entity(entity).insert(Camera {
            target: RenderTarget::Image(image_handle.clone()),
            ..default()
        });
    }
}
```

#### TODO: Frame Difference Computation

```rust
fn compute_camera_differences(
    mut cameras: Query<&mut SimulatedCamera>,
    images: Res<Assets<Image>>,
    mut output: ResMut<SimulatedFrameOutput>,
) {
    for mut sim_cam in cameras.iter_mut() {
        if let Some(image) = images.get(&sim_cam.render_target) {
            let current_frame = image_to_grayscale(&image.data);

            if let Some(ref previous) = sim_cam.previous_frame {
                // Compute difference
                let mut motion_pixels = Vec::new();
                let threshold = 25u8;

                for (i, (curr, prev)) in current_frame.iter().zip(previous.iter()).enumerate() {
                    let diff = curr.abs_diff(*prev);
                    if diff > threshold {
                        let x = (i as u32) % sim_cam.intrinsics.resolution.x;
                        let y = (i as u32) / sim_cam.intrinsics.resolution.x;
                        motion_pixels.push((x, y, diff));
                    }
                }

                output.frames.push(SimulatedFrame {
                    camera_id: sim_cam.camera_id,
                    motion_pixels,
                    timestamp: now_micros(),
                });
            }

            sim_cam.previous_frame = Some(current_frame);
        }
    }
}

fn image_to_grayscale(rgba: &[u8]) -> Vec<u8> {
    rgba.chunks(4)
        .map(|pixel| {
            // Luminance formula: 0.299*R + 0.587*G + 0.114*B
            ((pixel[0] as f32 * 0.299 +
              pixel[1] as f32 * 0.587 +
              pixel[2] as f32 * 0.114) as u8)
        })
        .collect()
}
```

#### TODO: Generate Voxel Contributions

```rust
fn generate_voxel_contributions(
    simulated_frames: Res<SimulatedFrameOutput>,
    cameras: Query<(&Transform, &SimulatedCamera)>,
    config: Res<SimulationConfig>,
) -> Vec<CameraFrame> {
    let mut camera_frames = Vec::new();

    for frame in &simulated_frames.frames {
        if let Some((transform, sim_cam)) = cameras.iter()
            .find(|(_, sc)| sc.camera_id == frame.camera_id)
        {
            let pose = transform_to_camera_pose(transform, &config.grid_origin);

            let raymarcher = Raymarcher::new(
                sim_cam.intrinsics,
                config.raymarch_config.clone(),
                config.grid_bounds,
                config.voxel_size,
            );

            // Create difference mask from motion pixels
            let mut mask = DifferenceMask::new(
                sim_cam.intrinsics.resolution.x,
                sim_cam.intrinsics.resolution.y,
            );
            for (x, y, intensity) in &frame.motion_pixels {
                mask.set((*y * mask.width + *x) as usize, *intensity);
            }

            let contributions = raymarcher.raymarch(&pose, &mask);

            camera_frames.push(CameraFrame {
                camera_id: frame.camera_id,
                sequence: 0,
                timestamp: frame.timestamp,
                pose,
                contributions,
            });
        }
    }

    camera_frames
}

fn transform_to_camera_pose(transform: &Transform, grid_origin: &GeoPosition) -> CameraPose {
    // Convert Bevy transform to CameraPose
    // Bevy uses Y-up, we use ENU (East-North-Up)
    let position = GeoPosition::from_local_enu(
        Vec3::new(transform.translation.x, transform.translation.z, transform.translation.y),
        grid_origin,
    );

    CameraPose {
        position,
        orientation: transform.rotation,
        timestamp: now_micros(),
        uncertainty: PoseUncertainty::default(),
        status: LocalizationStatus::Nominal,
    }
}
```

---

### validation.rs - Ground Truth Comparison

This module compares detection results against known object positions.

#### Current State
- `ValidationMetrics` resource
- Ground truth collection
- Basic error calculation

#### TODO: Comprehensive Metrics

```rust
#[derive(Resource, Default)]
pub struct ValidationMetrics {
    // Detection metrics
    pub true_positives: u32,
    pub false_positives: u32,
    pub false_negatives: u32,

    // Accuracy metrics
    pub position_errors: Vec<f32>,  // meters
    pub velocity_errors: Vec<f32>,  // m/s

    // Per-target tracking
    pub target_detections: HashMap<u32, TargetDetectionStats>,

    // Ground truth buffer
    ground_truth: VecDeque<GroundTruthSample>,
}

#[derive(Default)]
pub struct TargetDetectionStats {
    pub total_frames: u32,
    pub detected_frames: u32,
    pub average_position_error: f32,
    pub track_id: Option<ObjectId>,
}

impl ValidationMetrics {
    pub fn precision(&self) -> f32 {
        let total = self.true_positives + self.false_positives;
        if total == 0 { 0.0 } else { self.true_positives as f32 / total as f32 }
    }

    pub fn recall(&self) -> f32 {
        let total = self.true_positives + self.false_negatives;
        if total == 0 { 0.0 } else { self.true_positives as f32 / total as f32 }
    }

    pub fn f1_score(&self) -> f32 {
        let p = self.precision();
        let r = self.recall();
        if p + r == 0.0 { 0.0 } else { 2.0 * p * r / (p + r) }
    }

    pub fn mean_position_error(&self) -> Option<f32> {
        if self.position_errors.is_empty() {
            None
        } else {
            Some(self.position_errors.iter().sum::<f32>() / self.position_errors.len() as f32)
        }
    }

    pub fn position_error_percentile(&self, percentile: f32) -> Option<f32> {
        if self.position_errors.is_empty() {
            return None;
        }

        let mut sorted = self.position_errors.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let index = ((percentile / 100.0) * sorted.len() as f32) as usize;
        Some(sorted[index.min(sorted.len() - 1)])
    }

    pub fn report(&self) -> ValidationReport {
        ValidationReport {
            precision: self.precision(),
            recall: self.recall(),
            f1_score: self.f1_score(),
            mean_position_error: self.mean_position_error(),
            p90_position_error: self.position_error_percentile(90.0),
            p99_position_error: self.position_error_percentile(99.0),
            total_targets: self.target_detections.len(),
            detection_rate_by_target: self.target_detections.iter()
                .map(|(id, stats)| (*id, stats.detected_frames as f32 / stats.total_frames as f32))
                .collect(),
        }
    }
}

#[derive(Debug)]
pub struct ValidationReport {
    pub precision: f32,
    pub recall: f32,
    pub f1_score: f32,
    pub mean_position_error: Option<f32>,
    pub p90_position_error: Option<f32>,
    pub p99_position_error: Option<f32>,
    pub total_targets: usize,
    pub detection_rate_by_target: HashMap<u32, f32>,
}
```

#### TODO: Validation System

```rust
fn validate_detections(
    time: Res<Time>,
    mut metrics: ResMut<ValidationMetrics>,
    targets: Query<(&Transform, &SimulatedTarget)>,
    detections: Res<CurrentDetections>,  // From simulated detection pipeline
) {
    let timestamp = time.elapsed_secs_f64();
    let association_threshold = 5.0; // meters

    // Collect current ground truth
    let mut ground_truth: Vec<(u32, Vec3, Vec3)> = targets.iter()
        .map(|(t, st)| (st.ground_truth_id, t.translation, st.velocity))
        .collect();

    // Match detections to ground truth
    let mut matched_gt: HashSet<u32> = HashSet::new();
    let mut matched_det: HashSet<ObjectId> = HashSet::new();

    for detection in &detections.objects {
        let mut best_match: Option<(u32, f32)> = None;

        for (gt_id, gt_pos, _) in &ground_truth {
            if matched_gt.contains(gt_id) {
                continue;
            }

            let distance = detection.centroid.distance(*gt_pos);
            if distance < association_threshold {
                if best_match.is_none() || distance < best_match.unwrap().1 {
                    best_match = Some((*gt_id, distance));
                }
            }
        }

        if let Some((gt_id, distance)) = best_match {
            metrics.true_positives += 1;
            metrics.position_errors.push(distance);

            matched_gt.insert(gt_id);
            matched_det.insert(detection.id);

            // Update per-target stats
            let stats = metrics.target_detections.entry(gt_id).or_default();
            stats.detected_frames += 1;
            stats.track_id = Some(detection.id);
        } else {
            metrics.false_positives += 1;
        }
    }

    // Count false negatives (ground truth not matched)
    for (gt_id, _, _) in &ground_truth {
        if !matched_gt.contains(gt_id) {
            metrics.false_negatives += 1;
        }

        // Update total frames for all targets
        let stats = metrics.target_detections.entry(*gt_id).or_default();
        stats.total_frames += 1;
    }
}
```

---

### UI Overlay

#### TODO: Debug Visualization

```rust
use bevy_egui::{egui, EguiContexts, EguiPlugin};

fn debug_ui(
    mut contexts: EguiContexts,
    metrics: Res<ValidationMetrics>,
    cameras: Query<&SimulatedCamera>,
    targets: Query<&SimulatedTarget>,
) {
    egui::Window::new("Simulation Stats").show(contexts.ctx_mut(), |ui| {
        ui.heading("Targets");
        ui.label(format!("Active targets: {}", targets.iter().count()));

        ui.heading("Cameras");
        ui.label(format!("Simulated cameras: {}", cameras.iter().count()));

        ui.heading("Validation Metrics");
        let report = metrics.report();
        ui.label(format!("Precision: {:.2}%", report.precision * 100.0));
        ui.label(format!("Recall: {:.2}%", report.recall * 100.0));
        ui.label(format!("F1 Score: {:.2}%", report.f1_score * 100.0));

        if let Some(mpe) = report.mean_position_error {
            ui.label(format!("Mean Position Error: {:.2}m", mpe));
        }
        if let Some(p90) = report.p90_position_error {
            ui.label(format!("P90 Position Error: {:.2}m", p90));
        }
    });
}
```

---

## Integration with Pipeline

#### TODO: Network Integration

Send simulated data to a real server for full pipeline testing:

```rust
#[derive(Resource)]
pub struct NetworkIntegration {
    client: Option<QuicClient>,
    enabled: bool,
}

async fn send_to_server(
    mut network: ResMut<NetworkIntegration>,
    frames: Res<SimulatedCameraFrames>,
) {
    if !network.enabled {
        return;
    }

    if let Some(ref mut client) = network.client {
        for frame in &frames.0 {
            if let Err(e) = client.send_frame(frame.clone()).await {
                warn!("Failed to send frame: {}", e);
            }
        }
    }
}
```

---

## Implementation Priority

1. **Phase 1: Basic Simulation**
   - Scene with ground and lighting
   - Moving targets with basic patterns
   - Camera positions and frustums

2. **Phase 2: Render-Based Capture**
   - Render from camera viewpoints
   - Compute frame differences
   - Generate voxel contributions

3. **Phase 3: Validation**
   - Ground truth collection
   - Detection matching
   - Metrics calculation and reporting

4. **Phase 4: Advanced Features**
   - Path-based motion
   - Occlusion testing
   - Network integration
   - UI overlay for debugging

---

## Running the Simulator

```bash
# Basic run
cargo run -p iluvatar-simulator

# With specific scenario
cargo run -p iluvatar-simulator -- --scenario occlusion

# Connected to server
cargo run -p iluvatar-simulator -- --server localhost:4433
```
