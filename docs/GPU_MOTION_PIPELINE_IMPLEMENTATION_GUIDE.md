# GPU Motion Pipeline Implementation Guide

This guide provides a detailed implementation plan for moving the simulator's motion detection pipeline from CPU to GPU, based on the design document at `docs/GPU_MOTION_PIPELINE_DESIGN.md`.

## Current State Analysis

### Overview

The simulator (`crates/iluvatar-simulator/`) is a Bevy 0.18-based testing environment with two modes:
- **Geometric Mode** (`SimulatorPlugin`): Uses mathematical projection to detect targets
- **Render Mode** (`RenderSimulatorPlugin`): Actually renders scenes and does frame differencing

The GPU pipeline work targets **Render Mode**, which currently does frame differencing on CPU.

### Current Pipeline (Render Mode)

```
GPU: Camera renders to Image -> Readback::texture() -> ReadbackComplete event
                                          |
                                          v (3.7MB transfer)
CPU: RGBA data -> rgba_to_grayscale() -> compute_difference() -> filter_noise()
                                                                      |
                                                                      v
CPU: motion_pixels Vec<(u32, u32)> -> ray_direction() -> raymarch_ray() -> voxel grid
```

### Key Files and Their Roles

| File | Lines | Role |
|------|-------|------|
| `render_camera.rs` | 468 | Frame capture, CPU differencing, motion extraction |
| `motion_raymarch.rs` | 109 | Consumes motion_pixels, casts rays |
| `render_layers.rs` | 61 | Layer constants and helpers |
| `scene.rs` | 62 | Scene setup with correct layers |
| `targets.rs` | 204 | Target spawning with LAYER_TARGETS |
| `voxels.rs` | 608 | Voxel grid, raymarching, visualization |
| `tracking.rs` | 460 | Detection, tracking, visualization |
| `debug_ui.rs` | 285 | Egui controls, gizmo layer config |
| `lib.rs` | 134 | Plugin organization |

### What Already Exists

**Phase 1 (Render Layers): COMPLETE**
- `render_layers.rs` defines `LAYER_DEFAULT=0`, `LAYER_TARGETS=1`, `LAYER_DEBUG=2`
- `target_layers()`, `scene_layers()`, `debug_layers()`, etc. helper functions exist
- `scene.rs:29` applies `scene_layers()` to ground plane
- `scene.rs:49` applies `light_layers()` to directional light
- `scene.rs:59` applies `debug_camera_layers()` to free camera
- `targets.rs:143,167,190` applies `target_layers()` to all spawned targets
- `render_camera.rs:268` applies `render_camera_layers()` to render cameras
- `debug_ui.rs:27-33` configures gizmos to use `debug_layers()`

**Phase 2-6: NOT STARTED**
- No GPU compute shaders exist
- No `assets/shaders/` directory in the simulator
- No ping-pong buffer infrastructure
- Frame differencing is entirely CPU-based (`render_camera.rs:300-366`)
- Ray generation is CPU-based (`render_camera.rs:150-165`, `motion_raymarch.rs:81-88`)

---

## Gap Analysis by Phase

### Phase 1: Render Layers

**Current State**: FULLY IMPLEMENTED

The render layer system is complete and working:
- Layer constants defined in `render_layers.rs:24-30`
- All entities properly tagged with correct layers
- Gizmos configured to render only on debug layer
- Render cameras see only layers 0+1, debug camera sees all

**Missing**: Nothing

**Conflicts**: None

### Phase 2: Frame Buffer Infrastructure

**Current State**: PARTIAL

Current implementation (`render_camera.rs:77-100`):
```rust
pub struct RenderCamera {
    pub render_target: Handle<Image>,      // Current frame (GPU texture)
    pub previous_frame: Option<Vec<u8>>,   // Previous frame (CPU Vec!)
    pub difference_mask: Vec<u8>,          // CPU buffer
    pub motion_pixels: Vec<(u32, u32)>,    // CPU buffer
    // ...
}
```

**Missing**:
1. Second GPU texture for previous frame (`previous_frame_handle: Handle<Image>`)
2. GPU texture for difference mask (`difference_mask_handle: Handle<Image>`)
3. Frame copy system (current -> previous after each frame)
4. Proper texture usage flags for compute shader binding

**Conflicts**:
- `RenderCamera` struct needs restructuring
- `handle_readback_complete` observer (`render_camera.rs:375-437`) needs major changes
- The `Readback::texture()` pattern reads the full frame - we'll switch to `Readback::buffer()` for rays only

### Phase 3: Frame Differencing Compute Shader

**Current State**: CPU ONLY

CPU implementation in `render_camera.rs`:
- `rgba_to_grayscale()` at lines 300-313
- `compute_difference()` at lines 316-321
- `filter_noise()` at lines 324-367

**Missing**:
1. `assets/shaders/frame_difference.wgsl` - compute shader
2. `gpu_pipeline.rs` - Bevy render graph integration
3. Compute pipeline setup code
4. Bind group creation for textures

**Conflicts**:
- Must remove CPU grayscale/difference code after GPU version works
- The observer-based readback pattern will be replaced

### Phase 4: Ray Generation Compute Shader

**Current State**: CPU ONLY

CPU implementation spans two files:
- `RenderCamera::ray_direction()` at `render_camera.rs:150-165`
- `motion_based_raymarch()` at `motion_raymarch.rs:40-108`

**Missing**:
1. `assets/shaders/ray_generation.wgsl` - compute shader
2. `GpuRay` struct with proper alignment
3. Atomic ray buffer with header
4. Camera intrinsics uniform buffer
5. Per-frame uniform updates for camera poses

**Conflicts**:
- `motion_based_raymarch` system will consume a `RayBufferResource` instead of querying `RenderCamera::motion_pixels`
- Ray direction calculation in shader must match CPU version exactly

### Phase 5: Buffer Readback

**Current State**: WRONG READBACK TARGET

Currently reads full RGBA frame (`render_camera.rs:270`):
```rust
Readback::texture(render_target_handle.clone()),
```

**Missing**:
1. Ray buffer storage creation
2. `Readback::buffer()` for ray buffer only
3. `RayBufferResource` to hold parsed rays
4. `handle_ray_readback` observer
5. Buffer reset (clear ray count) each frame

**Conflicts**:
- Current texture readback must be removed
- `motion_based_raymarch` input changes from `Query<RenderCamera>` to `Res<RayBufferResource>`

### Phase 6: Cleanup & Optimization

**Current State**: N/A (depends on phases 2-5)

**Missing**:
1. Remove `previous_frame: Option<Vec<u8>>` from `RenderCamera`
2. Remove `difference_mask: Vec<u8>` from `RenderCamera`
3. Remove `motion_pixels: Vec<(u32, u32)>` from `RenderCamera`
4. Remove CPU frame differencing functions
5. Add GPU timing metrics
6. Tune subsample factor based on performance data

---

## Detailed Implementation Plan

### Phase 1: Render Layers

**Status**: ALREADY COMPLETE - No changes needed

The implementation matches the design document exactly. The layer system properly prevents gizmo contamination in render cameras.

**Verification Test**:
```bash
cargo run -p iluvatar-simulator -- --render
# Observe: motion_pixels count should be ~700 (targets only), not 71,000+ (with gizmos)
```

---

### Phase 2: Frame Buffer Infrastructure

#### Files to Modify

**`render_camera.rs`** - Major restructuring

#### Code Changes

**Step 2.1: Add GPU texture handles to RenderCamera**

Location: `render_camera.rs:77-100`

```rust
// BEFORE
#[derive(Component)]
pub struct RenderCamera {
    pub intrinsics: CameraIntrinsics,
    pub camera_id: u32,
    pub render_target: Handle<Image>,
    pub previous_frame: Option<Vec<u8>>,      // CPU buffer - REMOVE
    pub difference_mask: Vec<u8>,              // CPU buffer - REMOVE
    pub motion_pixels: Vec<(u32, u32)>,        // CPU buffer - REMOVE
    pub resolution: UVec2,
    pub difference_threshold: u8,
    pub min_neighbors: u8,
    pub subsample: u32,
}

// AFTER
#[derive(Component)]
pub struct RenderCamera {
    pub intrinsics: CameraIntrinsics,
    pub camera_id: u32,
    /// Current frame render target (camera renders here)
    pub render_target: Handle<Image>,
    /// Previous frame (GPU texture, for differencing)
    pub previous_frame: Handle<Image>,
    /// Difference mask output (GPU texture, R8 format)
    pub difference_mask: Handle<Image>,
    /// Resolution for processing
    pub resolution: UVec2,
    /// Difference threshold (0-255)
    pub difference_threshold: u8,
    /// Subsample factor for ray generation
    pub subsample: u32,
    /// Whether we have a valid previous frame (skip first frame)
    pub has_previous: bool,
}
```

**Step 2.2: Create texture with compute shader usage**

Location: `render_camera.rs:176-190` (modify `create_render_target_image`)

```rust
/// Create a render target image suitable for GPU compute and optional readback
fn create_gpu_texture(resolution: UVec2, format: TextureFormat, for_compute: bool) -> Image {
    let size = Extent3d {
        width: resolution.x,
        height: resolution.y,
        depth_or_array_layers: 1,
    };

    let mut image = Image::new_fill(
        size,
        TextureDimension::D2,
        &[0u8; 4], // Will be resized
        format,
        RenderAssetUsages::RENDER_WORLD,
    );

    // Proper size for the format
    let bytes_per_pixel = match format {
        TextureFormat::Rgba8Unorm | TextureFormat::Rgba8UnormSrgb => 4,
        TextureFormat::R8Uint => 1,
        _ => 4,
    };
    image.data = Some(vec![0u8; (resolution.x * resolution.y) as usize * bytes_per_pixel]);

    let mut usage = TextureUsages::TEXTURE_BINDING;
    
    if for_compute {
        usage |= TextureUsages::STORAGE_BINDING; // Compute shader write
    } else {
        usage |= TextureUsages::RENDER_ATTACHMENT; // Camera renders to it
    }
    
    // For ping-pong copy
    usage |= TextureUsages::COPY_SRC | TextureUsages::COPY_DST;

    image.texture_descriptor.usage = usage;

    image
}

/// Create the render target (camera renders here)
fn create_render_target_image(resolution: UVec2) -> Image {
    let mut image = create_gpu_texture(resolution, TextureFormat::Rgba8Unorm, false);
    // Add sRGB view for proper color rendering
    image.texture_view_descriptor = Some(TextureViewDescriptor {
        format: Some(TextureFormat::Rgba8UnormSrgb),
        ..default()
    });
    image
}

/// Create the previous frame buffer (copy destination)
fn create_previous_frame_image(resolution: UVec2) -> Image {
    create_gpu_texture(resolution, TextureFormat::Rgba8Unorm, true)
}

/// Create the difference mask (compute shader output)
fn create_difference_mask_image(resolution: UVec2) -> Image {
    create_gpu_texture(resolution, TextureFormat::R8Uint, true)
}
```

**Step 2.3: Update spawn_render_cameras**

Location: `render_camera.rs:193-297`

```rust
fn spawn_render_cameras(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    config: Res<RenderCameraConfig>,
) {
    // ... placements unchanged ...

    for (id, placement) in placements.iter().enumerate() {
        // Create all three textures for this camera
        let render_target = images.add(create_render_target_image(config.resolution));
        let previous_frame = images.add(create_previous_frame_image(config.resolution));
        let difference_mask = images.add(create_difference_mask_image(config.resolution));

        let transform =
            Transform::from_translation(placement.position).looking_at(placement.look_at, Vec3::Y);

        let aspect = config.resolution.x as f32 / config.resolution.y as f32;
        let fov_y = 2.0 * ((std::f32::consts::FRAC_PI_4).tan() / aspect).atan();

        // Create intrinsics (unchanged)
        let intrinsics = CameraIntrinsics {
            focal_length: Vec2::new(
                config.resolution.x as f32 * 0.8,
                config.resolution.x as f32 * 0.8,
            ),
            principal_point: Vec2::new(
                config.resolution.x as f32 / 2.0,
                config.resolution.y as f32 / 2.0,
            ),
            resolution: config.resolution,
            fov: Fov {
                horizontal: std::f32::consts::FRAC_PI_2,
                vertical: (config.resolution.y as f32 / config.resolution.x as f32)
                    * std::f32::consts::FRAC_PI_2,
            },
            distortion: DistortionModel::None,
        };

        let render_camera = RenderCamera {
            intrinsics,
            camera_id: id as u32,
            render_target: render_target.clone(),
            previous_frame: previous_frame.clone(),
            difference_mask: difference_mask.clone(),
            resolution: config.resolution,
            difference_threshold: config.difference_threshold,
            subsample: config.subsample,
            has_previous: false,
        };

        // Spawn camera entity - NO READBACK here anymore!
        // Readback will be done on the ray buffer in Phase 5
        commands.spawn((
            Camera3d::default(),
            Camera {
                order: -(id as isize + 1),
                ..default()
            },
            RenderTarget::Image(render_target.clone().into()),
            Projection::Perspective(PerspectiveProjection {
                fov: fov_y,
                aspect_ratio: aspect,
                near: 0.1,
                far: 500.0,
                ..default()
            }),
            transform,
            render_camera,
            render_camera_layers(),
        ));

        info!("Spawned render camera {} with GPU frame buffers", id);

        // Visual marker (unchanged)
        commands.spawn((
            Mesh3d(camera_mesh.clone()),
            MeshMaterial3d(materials.add(placement.color)),
            Transform::from_translation(placement.position),
        ));
    }
}
```

**Step 2.4: Remove CPU frame processing code**

Delete or comment out for now (we'll need it for fallback testing):
- `rgba_to_grayscale()` (lines 300-313)
- `compute_difference()` (lines 316-321)
- `filter_noise()` (lines 324-367)
- `handle_readback_complete()` (lines 375-437)

#### Testing Phase 2

```bash
# Build should succeed with new texture structure
cargo build -p iluvatar-simulator

# Run - should show cameras but NO motion detection (compute shaders not hooked up yet)
cargo run -p iluvatar-simulator -- --render
# Expected: No motion pixels, no voxels (pipeline incomplete)
```

#### Pitfalls

1. **Texture format mismatch**: Bevy's `Image::new_fill` signature may differ. Check Bevy 0.18 docs.
2. **Missing `STORAGE_BINDING`**: Compute shaders require this usage flag
3. **Handle ownership**: The `Handle<Image>` is cloned, not moved - be careful about which handle is used where

---

### Phase 3: Frame Differencing Compute Shader

#### Files to Create

**`assets/shaders/frame_difference.wgsl`**

```wgsl
// Frame difference compute shader
// Computes grayscale difference between current and previous frame

struct DiffParams {
    width: u32,
    height: u32,
    threshold: u32,      // 0-255
    _padding: u32,
}

@group(0) @binding(0) var current_frame: texture_2d<f32>;
@group(0) @binding(1) var previous_frame: texture_2d<f32>;
@group(0) @binding(2) var<uniform> params: DiffParams;
@group(0) @binding(3) var difference_mask: texture_storage_2d<r8uint, write>;

// Standard luminance weights for RGB to grayscale
fn to_grayscale(color: vec4<f32>) -> f32 {
    return dot(color.rgb, vec3<f32>(0.299, 0.587, 0.114));
}

@compute @workgroup_size(16, 16)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let x = global_id.x;
    let y = global_id.y;
    
    // Bounds check
    if (x >= params.width || y >= params.height) {
        return;
    }
    
    let coord = vec2<i32>(i32(x), i32(y));
    
    // Sample both frames
    let current_color = textureLoad(current_frame, coord, 0);
    let previous_color = textureLoad(previous_frame, coord, 0);
    
    // Convert to grayscale (0.0 - 1.0 range)
    let current_gray = to_grayscale(current_color);
    let previous_gray = to_grayscale(previous_color);
    
    // Compute absolute difference, scale to 0-255
    let diff = abs(current_gray - previous_gray) * 255.0;
    
    // Threshold and output
    var output: u32 = 0u;
    if (diff > f32(params.threshold)) {
        output = 255u;
    }
    
    textureStore(difference_mask, coord, vec4<u32>(output, 0u, 0u, 0u));
}
```

#### Files to Create

**`src/gpu_pipeline.rs`** - New file

```rust
//! GPU compute pipeline for motion detection
//!
//! This module sets up:
//! 1. Frame differencing compute shader (per camera)
//! 2. Ray generation compute shader (all cameras -> shared buffer)
//! 3. Buffer readback for CPU consumption

use bevy::{
    prelude::*,
    render::{
        extract_component::ExtractComponent,
        render_graph::{self, RenderGraph, RenderLabel},
        render_resource::*,
        renderer::{RenderContext, RenderDevice, RenderQueue},
        texture::GpuImage,
        Render, RenderApp, RenderSet,
    },
};

use crate::render_camera::RenderCamera;

/// Plugin for GPU motion detection pipeline
pub struct GpuMotionPipelinePlugin;

impl Plugin for GpuMotionPipelinePlugin {
    fn build(&self, app: &mut App) {
        // Load shaders as assets
        let asset_server = app.world().resource::<AssetServer>();
        let diff_shader = asset_server.load("shaders/frame_difference.wgsl");
        
        app.insert_resource(FrameDiffShader(diff_shader));
    }

    fn finish(&self, app: &mut App) {
        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };

        render_app
            .init_resource::<FrameDiffPipeline>()
            .add_systems(
                Render,
                prepare_frame_diff_bind_groups.in_set(RenderSet::PrepareBindGroups),
            );

        // Add compute node to render graph
        let mut render_graph = render_app.world_mut().resource_mut::<RenderGraph>();
        render_graph.add_node(FrameDiffLabel, FrameDiffNode);
        
        // Run after cameras render, before anything else
        render_graph.add_node_edge(bevy::render::graph::CameraDriverLabel, FrameDiffLabel);
    }
}

/// Handle to the frame difference shader
#[derive(Resource)]
struct FrameDiffShader(Handle<Shader>);

/// Uniform buffer for diff parameters
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
struct DiffParams {
    width: u32,
    height: u32,
    threshold: u32,
    _padding: u32,
}

/// The compute pipeline for frame differencing
#[derive(Resource)]
struct FrameDiffPipeline {
    pipeline: CachedComputePipelineId,
    bind_group_layout: BindGroupLayout,
}

impl FromWorld for FrameDiffPipeline {
    fn from_world(world: &mut World) -> Self {
        let render_device = world.resource::<RenderDevice>();
        let shader = world.resource::<FrameDiffShader>().0.clone();
        let pipeline_cache = world.resource::<PipelineCache>();

        let bind_group_layout = render_device.create_bind_group_layout(
            "frame_diff_bind_group_layout",
            &BindGroupLayoutEntries::sequential(
                ShaderStages::COMPUTE,
                (
                    // Current frame (texture_2d<f32>)
                    texture_2d(TextureSampleType::Float { filterable: false }),
                    // Previous frame (texture_2d<f32>)
                    texture_2d(TextureSampleType::Float { filterable: false }),
                    // Params uniform
                    uniform_buffer::<DiffParams>(false),
                    // Difference mask (storage texture)
                    texture_storage_2d(TextureFormat::R8Uint, StorageTextureAccess::WriteOnly),
                ),
            ),
        );

        let pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
            label: Some("frame_diff_pipeline".into()),
            layout: vec![bind_group_layout.clone()],
            shader,
            shader_defs: vec![],
            entry_point: "main".into(),
            push_constant_ranges: vec![],
            zero_initialize_workgroup_memory: false,
        });

        Self {
            pipeline,
            bind_group_layout,
        }
    }
}

/// Bind groups prepared for each camera
#[derive(Resource, Default)]
struct FrameDiffBindGroups {
    groups: Vec<(u32, BindGroup, UVec2)>, // (camera_id, bind_group, resolution)
}

/// Prepare bind groups for each render camera
fn prepare_frame_diff_bind_groups(
    mut bind_groups: ResMut<FrameDiffBindGroups>,
    pipeline: Res<FrameDiffPipeline>,
    render_device: Res<RenderDevice>,
    gpu_images: Res<RenderAssets<GpuImage>>,
    cameras: Query<&RenderCamera>,
) {
    bind_groups.groups.clear();

    for camera in cameras.iter() {
        // Get GPU textures
        let Some(current) = gpu_images.get(&camera.render_target) else {
            continue;
        };
        let Some(previous) = gpu_images.get(&camera.previous_frame) else {
            continue;
        };
        let Some(diff_mask) = gpu_images.get(&camera.difference_mask) else {
            continue;
        };

        // Create uniform buffer for params
        let params = DiffParams {
            width: camera.resolution.x,
            height: camera.resolution.y,
            threshold: camera.difference_threshold as u32,
            _padding: 0,
        };
        let params_buffer = render_device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("diff_params_buffer"),
            contents: bytemuck::cast_slice(&[params]),
            usage: BufferUsages::UNIFORM,
        });

        let bind_group = render_device.create_bind_group(
            "frame_diff_bind_group",
            &pipeline.bind_group_layout,
            &BindGroupEntries::sequential((
                &current.texture_view,
                &previous.texture_view,
                params_buffer.as_entire_binding(),
                &diff_mask.texture_view,
            )),
        );

        bind_groups.groups.push((camera.camera_id, bind_group, camera.resolution));
    }
}

/// Render graph label for frame differencing
#[derive(Debug, Hash, PartialEq, Eq, Clone, RenderLabel)]
struct FrameDiffLabel;

/// Render graph node that runs frame differencing compute shader
struct FrameDiffNode;

impl render_graph::Node for FrameDiffNode {
    fn run(
        &self,
        _graph: &mut render_graph::RenderGraphContext,
        render_context: &mut RenderContext,
        world: &World,
    ) -> Result<(), render_graph::NodeRunError> {
        let pipeline_cache = world.resource::<PipelineCache>();
        let pipeline_res = world.resource::<FrameDiffPipeline>();
        let bind_groups = world.resource::<FrameDiffBindGroups>();

        let Some(pipeline) = pipeline_cache.get_compute_pipeline(pipeline_res.pipeline) else {
            return Ok(()); // Pipeline not ready yet
        };

        let mut pass = render_context
            .command_encoder()
            .begin_compute_pass(&ComputePassDescriptor {
                label: Some("frame_diff_pass"),
                timestamp_writes: None,
            });

        pass.set_pipeline(pipeline);

        for (camera_id, bind_group, resolution) in &bind_groups.groups {
            pass.set_bind_group(0, bind_group, &[]);

            // Dispatch with 16x16 workgroups
            let workgroups_x = (resolution.x + 15) / 16;
            let workgroups_y = (resolution.y + 15) / 16;
            pass.dispatch_workgroups(workgroups_x, workgroups_y, 1);
        }

        Ok(())
    }
}
```

#### Files to Modify

**`lib.rs`** - Add GPU pipeline module

```rust
// Add at top with other mods
mod gpu_pipeline;

// In RenderSimulatorPlugin::build(), add:
.add_plugins(gpu_pipeline::GpuMotionPipelinePlugin)
```

**`Cargo.toml`** - Add bytemuck dependency

```toml
[dependencies]
# ... existing deps ...
bytemuck = { version = "1.14", features = ["derive"] }
```

#### Directory Structure

Create the assets directory:
```bash
mkdir -p crates/iluvatar-simulator/assets/shaders
```

#### Testing Phase 3

```bash
# Should compile with new GPU pipeline code
cargo build -p iluvatar-simulator

# Run with WGPU validation enabled
WGPU_BACKEND=vulkan cargo run -p iluvatar-simulator -- --render

# Debug: Add a system to visualize the difference mask texture
# (Sample it and display as a UI image)
```

#### Pitfalls

1. **Bevy 0.18 API changes**: The render graph API has changed significantly. Check:
   - `RenderLabel` trait vs `RenderGraphLabel`
   - `BindGroupLayoutEntries::sequential` may not exist
   - `RenderAssets<GpuImage>` access patterns

2. **Texture format compatibility**: `R8Uint` for storage may need specific GPU features

3. **Shader compilation errors**: WGSL syntax is strict. Test shader independently with naga.

4. **Missing frame copy**: Phase 3 doesn't copy current->previous yet. Add a separate copy pass:

```rust
// In FrameDiffNode::run(), after compute pass:
// Copy current frame to previous for next iteration
let encoder = render_context.command_encoder();
// encoder.copy_texture_to_texture(...)
```

---

### Phase 4: Ray Generation Compute Shader

#### Files to Create

**`assets/shaders/ray_generation.wgsl`**

```wgsl
// Ray generation compute shader
// Reads difference mask, outputs rays for motion pixels

struct RayBufferHeader {
    ray_count: atomic<u32>,
    max_rays: u32,
    frame: u32,
    _padding: u32,
}

struct GpuRay {
    camera_id: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
    origin: vec3<f32>,
    _pad3: f32,
    direction: vec3<f32>,
    _pad4: f32,
}

struct CameraData {
    camera_id: u32,
    width: u32,
    height: u32,
    subsample: u32,
    position: vec3<f32>,
    _pad0: f32,
    // Rotation matrix rows
    rotation_row0: vec3<f32>,
    _pad1: f32,
    rotation_row1: vec3<f32>,
    _pad2: f32,
    rotation_row2: vec3<f32>,
    _pad3: f32,
    // Camera intrinsics
    fov_half_tan: vec2<f32>,    // tan(fov_h/2), tan(fov_v/2)
    principal_point: vec2<f32>, // cx, cy (normalized)
}

@group(0) @binding(0) var difference_mask: texture_2d<u32>;
@group(0) @binding(1) var<uniform> camera: CameraData;
@group(0) @binding(2) var<storage, read_write> ray_header: RayBufferHeader;
@group(0) @binding(3) var<storage, read_write> rays: array<GpuRay>;

@compute @workgroup_size(16, 16)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    // Apply subsampling
    let x = global_id.x * camera.subsample;
    let y = global_id.y * camera.subsample;
    
    // Bounds check
    if (x >= camera.width || y >= camera.height) {
        return;
    }
    
    // Check if this pixel has motion
    let coord = vec2<i32>(i32(x), i32(y));
    let motion = textureLoad(difference_mask, coord, 0).r;
    
    if (motion == 0u) {
        return;
    }
    
    // Generate ray for this motion pixel
    
    // Convert pixel to normalized coordinates [-1, 1]
    let nx = (f32(x) - camera.principal_point.x * f32(camera.width)) / (f32(camera.width) / 2.0);
    let ny = (f32(y) - camera.principal_point.y * f32(camera.height)) / (f32(camera.height) / 2.0);
    
    // Convert to direction angles using FOV
    let angle_h = nx * camera.fov_half_tan.x;
    let angle_v = ny * camera.fov_half_tan.y;
    
    // Direction in camera local space (-Z is forward in Bevy)
    var local_dir = vec3<f32>(angle_h, -angle_v, -1.0);
    local_dir = normalize(local_dir);
    
    // Rotate to world space using camera rotation matrix
    let rotation = mat3x3<f32>(
        camera.rotation_row0,
        camera.rotation_row1,
        camera.rotation_row2,
    );
    let world_dir = rotation * local_dir;
    
    // Atomically allocate a slot in the ray buffer
    let slot = atomicAdd(&ray_header.ray_count, 1u);
    
    // Check if we have space
    if (slot >= ray_header.max_rays) {
        // Buffer full - roll back and drop this ray
        atomicSub(&ray_header.ray_count, 1u);
        return;
    }
    
    // Write the ray
    rays[slot].camera_id = camera.camera_id;
    rays[slot]._pad0 = 0u;
    rays[slot]._pad1 = 0u;
    rays[slot]._pad2 = 0u;
    rays[slot].origin = camera.position;
    rays[slot]._pad3 = 0.0;
    rays[slot].direction = world_dir;
    rays[slot]._pad4 = 0.0;
}
```

#### Files to Modify

**`gpu_pipeline.rs`** - Add ray generation pipeline

Add these types after the frame diff code:

```rust
/// Maximum rays in the buffer (tune based on expected motion)
const MAX_RAYS: usize = 4096;

/// GPU ray structure - must match WGSL layout exactly
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct GpuRay {
    pub camera_id: u32,
    pub _pad0: u32,
    pub _pad1: u32,
    pub _pad2: u32,
    pub origin: [f32; 3],
    pub _pad3: f32,
    pub direction: [f32; 3],
    pub _pad4: f32,
}

/// Ray buffer header - must match WGSL layout
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct RayBufferHeader {
    pub ray_count: u32,
    pub max_rays: u32,
    pub frame: u32,
    pub _padding: u32,
}

/// Camera uniform data for ray generation
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
struct CameraUniform {
    camera_id: u32,
    width: u32,
    height: u32,
    subsample: u32,
    position: [f32; 3],
    _pad0: f32,
    rotation_row0: [f32; 3],
    _pad1: f32,
    rotation_row1: [f32; 3],
    _pad2: f32,
    rotation_row2: [f32; 3],
    _pad3: f32,
    fov_half_tan: [f32; 2],
    principal_point: [f32; 2],
}

impl CameraUniform {
    fn from_camera(camera: &RenderCamera, transform: &Transform) -> Self {
        let rot = transform.rotation;
        // Convert quaternion to rotation matrix columns
        let right = rot * Vec3::X;
        let up = rot * Vec3::Y;
        let forward = rot * Vec3::NEG_Z;

        Self {
            camera_id: camera.camera_id,
            width: camera.resolution.x,
            height: camera.resolution.y,
            subsample: camera.subsample,
            position: transform.translation.into(),
            _pad0: 0.0,
            rotation_row0: right.into(),
            _pad1: 0.0,
            rotation_row1: up.into(),
            _pad2: 0.0,
            rotation_row2: forward.into(),
            _pad3: 0.0,
            fov_half_tan: [
                (camera.intrinsics.fov.horizontal / 2.0).tan(),
                (camera.intrinsics.fov.vertical / 2.0).tan(),
            ],
            principal_point: [0.5, 0.5], // Normalized to [0,1]
        }
    }
}

/// Resource holding the shared ray buffer
#[derive(Resource)]
pub struct RayBuffer {
    pub buffer: Buffer,
    pub size: usize,
}

impl FromWorld for RayBuffer {
    fn from_world(world: &mut World) -> Self {
        let render_device = world.resource::<RenderDevice>();
        
        let header_size = std::mem::size_of::<RayBufferHeader>();
        let rays_size = MAX_RAYS * std::mem::size_of::<GpuRay>();
        let total_size = header_size + rays_size;

        let buffer = render_device.create_buffer(&BufferDescriptor {
            label: Some("ray_buffer"),
            size: total_size as u64,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_SRC | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        Self {
            buffer,
            size: total_size,
        }
    }
}
```

#### Testing Phase 4

```bash
# Build and run
cargo build -p iluvatar-simulator
cargo run -p iluvatar-simulator -- --render

# Add debug logging for ray count
# In the readback handler, print ray_count from header
```

#### Pitfalls

1. **Rotation matrix orientation**: Bevy uses -Z forward. The matrix must be constructed correctly.

2. **Atomic operations**: WGSL atomics have specific requirements. `atomicAdd` returns the OLD value.

3. **Buffer alignment**: The `GpuRay` struct is 48 bytes. Ensure padding is correct.

4. **Shader dispatch order**: Must run AFTER frame differencing completes.

---

### Phase 5: Buffer Readback

#### Files to Modify

**`gpu_pipeline.rs`** - Add readback handling

```rust
use bevy::render::gpu_readback::{Readback, ReadbackComplete};

/// Resource to hold rays after readback
#[derive(Resource, Default)]
pub struct RayBufferResource {
    pub pending_rays: Vec<(u32, Ray)>, // (camera_id, ray)
    pub last_ray_count: u32,
}

/// System to spawn readback entity
fn setup_ray_readback(
    mut commands: Commands,
    ray_buffer: Res<RayBuffer>,
) {
    commands.spawn((
        Readback::buffer(ray_buffer.buffer.clone()),
        RayBufferReadbackMarker,
    ))
    .observe(handle_ray_readback);
}

#[derive(Component)]
struct RayBufferReadbackMarker;

/// Handle completed ray buffer readback
fn handle_ray_readback(
    event: On<ReadbackComplete>,
    mut ray_res: ResMut<RayBufferResource>,
    sim_config: Res<SimulatorConfig>,
) {
    let data: &[u8] = &**event;
    
    if data.len() < std::mem::size_of::<RayBufferHeader>() {
        warn!("Ray buffer readback too small");
        return;
    }

    // Parse header
    let header: RayBufferHeader = *bytemuck::from_bytes(&data[0..16]);
    let ray_count = header.ray_count.min(MAX_RAYS as u32) as usize;
    
    ray_res.last_ray_count = ray_count as u32;
    ray_res.pending_rays.clear();

    if ray_count == 0 {
        return;
    }

    // Parse rays
    let rays_offset = std::mem::size_of::<RayBufferHeader>();
    let rays_data = &data[rays_offset..];
    let rays: &[GpuRay] = bytemuck::cast_slice(&rays_data[..ray_count * std::mem::size_of::<GpuRay>()]);

    for gpu_ray in rays {
        let ray = Ray::new(
            Vec3::from(gpu_ray.origin),
            Vec3::from(gpu_ray.direction),
            sim_config.ray_intensity,
        );
        ray_res.pending_rays.push((gpu_ray.camera_id, ray));
    }

    debug!("Readback: {} rays from GPU", ray_count);
}
```

**`motion_raymarch.rs`** - Consume from RayBufferResource

```rust
// BEFORE: Query cameras for motion_pixels
pub fn motion_based_raymarch(
    mut grid_res: ResMut<VoxelGridResource>,
    sim_config: Res<SimulatorConfig>,
    tracking_config: Option<Res<crate::tracking::TrackingConfig>>,
    cameras: Query<(&Transform, &RenderCamera)>,  // OLD
) {
    // ... iterates camera.motion_pixels ...
}

// AFTER: Read from shared ray buffer
pub fn motion_based_raymarch(
    mut grid_res: ResMut<VoxelGridResource>,
    sim_config: Res<SimulatorConfig>,
    tracking_config: Option<Res<crate::tracking::TrackingConfig>>,
    mut ray_buffer: ResMut<RayBufferResource>,  // NEW
) {
    // Copy config values
    let raymarch_config = grid_res.config.clone();
    let bounds = grid_res.bounds;
    let voxel_size = sim_config.voxel_size;
    let grid_dims = sim_config.grid_dimensions;

    // Clear grid if configured
    let should_clear = tracking_config
        .as_ref()
        .map(|c| c.clear_grid_each_frame)
        .unwrap_or(false);

    if should_clear {
        grid_res.grid.clear();
    }

    // Reset stats
    grid_res.rays_cast = 0;
    grid_res.voxels_contributed = 0;

    // Process rays from GPU buffer
    let raymarcher = SimulatorRaymarcher::new(&raymarch_config, &bounds, voxel_size);
    let mut contributions: HashMap<(u32, u32, u32), f32> = HashMap::new();

    for (camera_id, ray) in ray_buffer.pending_rays.drain(..) {
        raymarcher.march_ray(&ray, grid_dims, &mut contributions);
        grid_res.rays_cast += 1;
    }

    // Add to voxel grid
    let voxel_contributions: Vec<VoxelContribution> = contributions
        .into_iter()
        .map(|((x, y, z), intensity)| VoxelContribution {
            index: UVec3::new(x, y, z),
            intensity,
        })
        .collect();

    grid_res.voxels_contributed = voxel_contributions.len() as u32;

    // For now, use camera_id 0 for all - could be enhanced to track per-camera
    grid_res.grid.add_camera_contributions(0, &voxel_contributions);
}
```

#### Testing Phase 5

```bash
cargo run -p iluvatar-simulator -- --render

# Should see:
# - Motion detection working
# - Voxels appearing where targets move
# - Tracking working as before
# - Much lower GPU->CPU bandwidth (check with GPU profiler)
```

#### Pitfalls

1. **Readback timing**: `ReadbackComplete` fires ~1-2 frames late. Ensure systems handle this.

2. **Buffer reset**: Must clear `ray_count` to 0 before each frame's compute pass.

3. **Camera ID tracking**: The simplified version uses camera_id=0 for all rays. Enhance to track properly.

---

### Phase 6: Cleanup & Optimization

#### Files to Modify

**`render_camera.rs`** - Remove CPU buffers

```rust
// Final RenderCamera struct (minimal)
#[derive(Component)]
pub struct RenderCamera {
    pub intrinsics: CameraIntrinsics,
    pub camera_id: u32,
    pub render_target: Handle<Image>,
    pub previous_frame: Handle<Image>,
    pub difference_mask: Handle<Image>,
    pub resolution: UVec2,
    pub difference_threshold: u8,
    pub subsample: u32,
    pub has_previous: bool,
}
// NO motion_pixels, NO CPU frame buffers
```

**Delete these functions entirely:**
- `rgba_to_grayscale()`
- `compute_difference()`
- `filter_noise()`
- `handle_readback_complete()` (the texture one)

**`gpu_pipeline.rs`** - Add metrics

```rust
#[derive(Resource, Default)]
pub struct GpuPipelineMetrics {
    pub frame_diff_time_us: f32,
    pub ray_gen_time_us: f32,
    pub readback_time_us: f32,
    pub ray_count: u32,
    pub buffer_utilization: f32, // ray_count / MAX_RAYS
}
```

**`debug_ui.rs`** - Display GPU metrics

Add a new section showing:
- GPU frame diff time
- GPU ray gen time
- Readback time
- Ray buffer utilization
- Bandwidth comparison (old vs new)

#### Testing Phase 6

```bash
# Performance comparison
# Run OLD CPU version:
git stash  # Save GPU changes
cargo run -p iluvatar-simulator -- --render
# Note: bandwidth, latency

# Run NEW GPU version:
git stash pop
cargo run -p iluvatar-simulator -- --render
# Note: bandwidth, latency

# Expected improvement:
# - Bandwidth: 3.7MB -> ~100KB (37x reduction)
# - Latency: ~3-4ms -> ~0.35ms (10x improvement)
```

---

## Design Document Issues

### 1. Bevy Version Mismatch

The design document targets Bevy 0.15, but the simulator uses **Bevy 0.18**. API changes:

| Design (0.15) | Current (0.18) | Fix |
|---------------|----------------|-----|
| `RenderLayers::from_layers(&[...])` | Same | OK |
| `Readback::buffer()` | Check API | Verify signature |
| `ShaderStorageBuffer` | May be renamed | Check render::storage |
| `ComputePipelineDescriptor` | May have changes | Check render_resource |
| `render_graph::Node` trait | Likely changed | Check node implementation |

**Action**: Review Bevy 0.18 render graph and compute shader APIs before implementing.

### 2. Missing First-Frame Handling

The design mentions three options for first frame but doesn't specify implementation. Recommendation:

```rust
// In RenderCamera
pub has_previous: bool,

// In frame diff system
if !camera.has_previous {
    // Copy current to previous, mark as ready
    // Output zero motion this frame
    camera.has_previous = true;
    return;
}
```

### 3. Frame Copy Timing

The design says "copy current -> previous after each frame" but doesn't specify when exactly. The copy must happen:
- AFTER the camera renders to `render_target`
- AFTER the compute shader reads from `render_target`
- BEFORE the next frame's camera render

This requires careful render graph ordering.

### 4. Multi-Camera Ray Buffer

The design shows sequential dispatches per camera, all writing to the same buffer. This works but could be optimized with a batched approach using `global_id.z` for camera index.

### 5. Missing Noise Filtering

The CPU version has `filter_noise()` (morphological filtering). The GPU version skips this. Options:
1. Accept slightly noisier output (simpler)
2. Add a third compute pass for erosion/dilation
3. Do filtering in ray generation shader (check neighbors)

### 6. Subsample Factor in Shader

The design applies subsampling in the ray generation shader dispatch:
```wgsl
let x = global_id.x * camera.subsample;
```

This means the dispatch grid size should be `(width/subsample, height/subsample)`, not `(width, height)`. Update dispatch calculation.

---

## Recommended Implementation Order

### Sprint 1: Foundation (Phase 2) - ~2 days

1. Create `assets/shaders/` directory
2. Modify `RenderCamera` struct to use GPU texture handles
3. Create texture creation helper functions
4. Update `spawn_render_cameras` to create all three textures
5. Remove `Readback::texture()` from camera spawning
6. **Test**: Build succeeds, cameras render but no motion detection

### Sprint 2: Frame Differencing (Phase 3) - ~3 days

1. Create `frame_difference.wgsl` shader
2. Create `gpu_pipeline.rs` with compute pipeline setup
3. Implement `FrameDiffPipeline` resource
4. Implement `FrameDiffNode` render graph node
5. Add frame copy pass (current -> previous)
6. Add debug visualization of difference mask
7. **Test**: Difference mask shows motion as white pixels

### Sprint 3: Ray Generation (Phase 4) - ~3 days

1. Create `ray_generation.wgsl` shader
2. Define `GpuRay`, `RayBufferHeader`, `CameraUniform` structs
3. Create `RayBuffer` resource
4. Implement ray generation pipeline and node
5. Add buffer reset before ray generation
6. **Test**: Ray count matches expected motion pixels

### Sprint 4: Integration (Phase 5) - ~2 days

1. Create `RayBufferResource` for CPU consumption
2. Implement ray buffer readback observer
3. Modify `motion_based_raymarch` to use `RayBufferResource`
4. **Test**: Full pipeline working, tracking results match CPU version

### Sprint 5: Polish (Phase 6) - ~2 days

1. Remove unused CPU buffers and functions
2. Add GPU timing metrics
3. Add metrics to debug UI
4. Performance tuning (subsample factor, buffer size)
5. **Test**: Performance improvement validated

### Total Estimate: ~12 days

---

## Quick Reference: File Changes Summary

| File | Action | Lines Changed |
|------|--------|---------------|
| `render_camera.rs` | Major refactor | ~200 lines modified |
| `motion_raymarch.rs` | Modify input source | ~30 lines |
| `gpu_pipeline.rs` | **NEW** | ~400 lines |
| `lib.rs` | Add module + plugin | ~5 lines |
| `Cargo.toml` | Add bytemuck | ~1 line |
| `assets/shaders/frame_difference.wgsl` | **NEW** | ~50 lines |
| `assets/shaders/ray_generation.wgsl` | **NEW** | ~80 lines |
| `debug_ui.rs` | Add metrics display | ~30 lines |

**Total new code**: ~600 lines
**Total modified code**: ~230 lines
