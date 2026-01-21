# GPU Motion Detection Pipeline Design

## Overview

This document describes a redesigned motion detection pipeline that moves frame differencing and ray generation from CPU to GPU, reducing bandwidth from ~3.7MB/frame to ~100KB/frame while eliminating gizmo contamination through render layers.

### Current Problems

1. **Gizmo Contamination**: Debug visualizations (voxel cubes, track trails, coordinate axes) render to the same layer as targets, causing 71,000+ false motion pixels
2. **High Bandwidth**: 4 cameras x 640x360x4 bytes = 3.7MB/frame GPU->CPU transfer
3. **CPU Frame Differencing**: Full RGBA images transferred, converted to grayscale, diffed on CPU
4. **CPU Ray Generation**: Motion pixels iterated sequentially to generate rays

### New Architecture

```
GPU Side:
  Camera Renders (Layer 1 only)
       |
       v
  [Ping-Pong Frame Buffers per camera]
       |
       v
  [Frame Difference Compute Shader] -> Difference Masks (per camera)
       |
       v
  [Ray Generation Compute Shader] -> Shared Ray Buffer (all cameras)
       |
       v
  [Single GPU Readback]

CPU Side:
  Ray Buffer -> Raymarching -> Voxel Grid -> Detection -> Tracking
```

---

## 1. Render Layer Setup

### Layer Assignments

| Layer | Contents | Viewers |
|-------|----------|---------|
| **Layer 0** | Default layer - ground plane, static scene geometry | Debug camera (FreeCamera) |
| **Layer 1** | Targets - moving objects to be detected | Render cameras, Debug camera |
| **Layer 2** | Debug gizmos - voxels, trails, coordinate axes, track visualizations | Debug camera only |

### Entity Configuration

```rust
// Constants for layer organization
pub const LAYER_DEFAULT: usize = 0;      // Static scene geometry
pub const LAYER_TARGETS: usize = 1;      // Moving targets (what cameras detect)
pub const LAYER_DEBUG: usize = 2;        // Debug visualizations

// Render layer sets
pub const TARGET_LAYERS: RenderLayers = RenderLayers::layer(LAYER_TARGETS);
pub const DEBUG_LAYERS: RenderLayers = RenderLayers::layer(LAYER_DEBUG);
pub const ALL_LAYERS: RenderLayers = RenderLayers::from_layers(&[LAYER_DEFAULT, LAYER_TARGETS, LAYER_DEBUG]);
```

### Camera Configuration

**Render Cameras** (motion detection):
```rust
commands.spawn((
    Camera3d::default(),
    Camera {
        order: -(id as isize + 1),  // Render before main camera
        ..default()
    },
    RenderTarget::Image(render_target_handle.clone().into()),
    // CRITICAL: Only see targets, not debug gizmos
    RenderLayers::layer(LAYER_DEFAULT).with(LAYER_TARGETS),
    transform,
    RenderCamera { ... },
));
```

**Debug Camera** (human viewer):
```rust
commands.spawn((
    Camera3d::default(),
    FreeCamera::default(),
    Transform::from_xyz(0.0, 80.0, 120.0).looking_at(Vec3::ZERO, Vec3::Y),
    // See everything
    RenderLayers::from_layers(&[LAYER_DEFAULT, LAYER_TARGETS, LAYER_DEBUG]),
));
```

### Entity Layer Assignments

**Targets** (`targets.rs`):
```rust
commands.spawn((
    Mesh3d(sphere_mesh.clone()),
    MeshMaterial3d(red_mat.clone()),
    Transform::default(),
    Target { id: 1 },
    TargetPath::new_linear(...),
    // Visible to render cameras
    RenderLayers::layer(LAYER_TARGETS),
));
```

**Scene Geometry** (`scene.rs`):
```rust
// Ground plane - visible to all cameras
commands.spawn((
    Mesh3d(meshes.add(Plane3d::default().mesh().size(500.0, 500.0))),
    MeshMaterial3d(materials.add(...)),
    Transform::from_xyz(0.0, 0.0, 0.0),
    // Default layer 0 (implicit, but explicit is clearer)
    RenderLayers::layer(LAYER_DEFAULT),
));

// Lights need to affect both layers
commands.spawn((
    DirectionalLight { ... },
    Transform::from_rotation(...),
    RenderLayers::layer(LAYER_DEFAULT).with(LAYER_TARGETS),
));
```

**Gizmo Configuration** (`voxels.rs`, `tracking.rs`):
```rust
// Gizmos automatically use the default gizmo config
// We need to configure gizmos to render to layer 2
app.init_gizmo_group::<DefaultGizmoConfigGroup>()
   .insert_resource(GizmoConfigStore::default().with_config(
       DefaultGizmoConfigGroup,
       GizmoConfig {
           render_layers: RenderLayers::layer(LAYER_DEBUG),
           ..default()
       },
   ));
```

---

## 2. Frame Buffer Management

### Per-Camera Frame Buffers

Each render camera needs two frame buffers for ping-pong pattern:

```rust
/// GPU-side frame buffers for a single camera
#[derive(Component)]
pub struct CameraFrameBuffers {
    /// Current frame render target (what camera renders to)
    pub current: Handle<Image>,
    /// Previous frame (copy of last frame for differencing)
    pub previous: Handle<Image>,
    /// Which buffer is "current" (swaps each frame)
    pub ping_pong_index: u32,
}
```

### Buffer Creation

```rust
fn create_frame_buffer(resolution: UVec2) -> Image {
    let size = Extent3d {
        width: resolution.x,
        height: resolution.y,
        depth_or_array_layers: 1,
    };
    
    let mut image = Image::new_uninit(
        size,
        TextureDimension::D2,
        TextureFormat::Rgba8Unorm,
        RenderAssetUsages::RENDER_WORLD,
    );
    
    image.texture_descriptor.usage = 
        TextureUsages::RENDER_ATTACHMENT |  // Camera can render to it
        TextureUsages::TEXTURE_BINDING |    // Shader can sample it
        TextureUsages::COPY_SRC |           // For ping-pong copy
        TextureUsages::COPY_DST;            // For ping-pong copy
    
    image
}
```

### Frame Copy System

After each frame renders, copy current -> previous:

```rust
// In render graph, after camera renders but before compute shaders:
// encoder.copy_texture_to_texture(current, previous)
```

**Alternative**: Use two render targets and swap handles instead of copying. This avoids the copy but requires more bookkeeping.

### First Frame Handling

On the first frame, there's no previous frame to diff against:

```rust
// In compute shader or CPU fallback:
if first_frame {
    // Option 1: Output zero motion (no rays)
    // Option 2: Copy current to previous, skip this frame
    // Option 3: Output all pixels as "motion" (aggressive, may cause noise)
}
```

**Recommended**: Option 1 - Output zero motion. The system will catch up on frame 2.

---

## 3. Frame Differencing Compute Shader

### Shader Inputs/Outputs

**Inputs**:
- `current_frame`: `texture_2d<f32>` - Current rendered frame (RGBA)
- `previous_frame`: `texture_2d<f32>` - Previous rendered frame (RGBA)
- `params`: Uniform buffer with threshold, dimensions

**Outputs**:
- `difference_mask`: `texture_storage_2d<r8uint, write>` - Binary mask (0 or 255)

### WGSL Shader

```wgsl
// frame_difference.wgsl

struct DiffParams {
    width: u32,
    height: u32,
    threshold: u32,      // 0-255, typically 15-30
    _padding: u32,
}

@group(0) @binding(0) var current_frame: texture_2d<f32>;
@group(0) @binding(1) var previous_frame: texture_2d<f32>;
@group(0) @binding(2) var<uniform> params: DiffParams;
@group(0) @binding(3) var difference_mask: texture_storage_2d<r8uint, write>;

// Convert RGB to grayscale using standard luminance weights
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
    
    // Convert to grayscale
    let current_gray = to_grayscale(current_color);
    let previous_gray = to_grayscale(previous_color);
    
    // Compute absolute difference (scaled to 0-255)
    let diff = abs(current_gray - previous_gray) * 255.0;
    
    // Threshold
    var output: u32 = 0u;
    if (diff > f32(params.threshold)) {
        output = 255u;
    }
    
    textureStore(difference_mask, coord, vec4<u32>(output, 0u, 0u, 0u));
}
```

### Dispatch Configuration

```rust
// Dispatch with 16x16 workgroups
let workgroups_x = (width + 15) / 16;
let workgroups_y = (height + 15) / 16;
pass.dispatch_workgroups(workgroups_x, workgroups_y, 1);
```

### Integration with Bevy Render Graph

Create a custom render graph node that runs after all cameras render:

```rust
#[derive(Debug, Hash, PartialEq, Eq, Clone, RenderLabel)]
struct FrameDifferenceLabel;

struct FrameDifferenceNode {
    camera_count: usize,
}

impl render_graph::Node for FrameDifferenceNode {
    fn run(
        &self,
        _graph: &mut render_graph::RenderGraphContext,
        render_context: &mut RenderContext,
        world: &World,
    ) -> Result<(), render_graph::NodeRunError> {
        // For each camera:
        // 1. Set up bind group with current/previous frames
        // 2. Dispatch compute shader
        // 3. Copy current -> previous for next frame
        Ok(())
    }
}
```

---

## 4. Ray Generation Compute Shader

### Shader Design

This is the most complex shader. It needs to:
1. Read the difference mask
2. For each motion pixel, generate a ray
3. Atomically append to a shared buffer

### Ray Buffer Format

```rust
/// A single ray ready for CPU raymarching
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct GpuRay {
    /// Camera ID (0-63)
    pub camera_id: u32,
    /// Padding for alignment
    pub _pad0: u32,
    pub _pad1: u32,
    pub _pad2: u32,
    /// Ray origin (camera position in world space)
    pub origin: [f32; 3],
    pub _pad3: f32,
    /// Ray direction (normalized, in world space)
    pub direction: [f32; 3],
    pub _pad4: f32,
}
// Total: 48 bytes per ray (aligned to 16 bytes)
```

### Atomic Counter for Append Buffer

```rust
/// Header at the start of the ray buffer
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct RayBufferHeader {
    /// Number of rays written (atomic counter)
    pub ray_count: u32,
    /// Maximum rays that can fit
    pub max_rays: u32,
    /// Frame number (for debugging)
    pub frame: u32,
    pub _padding: u32,
}
```

### Full Buffer Layout

```
[RayBufferHeader (16 bytes)]
[GpuRay 0 (48 bytes)]
[GpuRay 1 (48 bytes)]
...
[GpuRay N (48 bytes)]
```

### Camera Intrinsics Uniform

```wgsl
struct CameraData {
    // Camera ID for this dispatch
    camera_id: u32,
    // Image dimensions
    width: u32,
    height: u32,
    // Subsample factor (1 = every pixel, 2 = every other)
    subsample: u32,
    
    // Camera pose (world space)
    position: vec3<f32>,
    _pad0: f32,
    
    // Camera rotation (as quaternion or 3x3 matrix)
    // Using rotation matrix for simplicity
    rotation_row0: vec3<f32>,
    _pad1: f32,
    rotation_row1: vec3<f32>,
    _pad2: f32,
    rotation_row2: vec3<f32>,
    _pad3: f32,
    
    // Camera intrinsics
    focal_length: vec2<f32>,      // fx, fy
    principal_point: vec2<f32>,   // cx, cy
    fov_half_tan: vec2<f32>,      // tan(fov_h/2), tan(fov_v/2)
    _pad4: vec2<f32>,
}
```

### WGSL Ray Generation Shader

```wgsl
// ray_generation.wgsl

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
    rotation_row0: vec3<f32>,
    _pad1: f32,
    rotation_row1: vec3<f32>,
    _pad2: f32,
    rotation_row2: vec3<f32>,
    _pad3: f32,
    focal_length: vec2<f32>,
    principal_point: vec2<f32>,
    fov_half_tan: vec2<f32>,
    _pad4: vec2<f32>,
}

@group(0) @binding(0) var difference_mask: texture_2d<u32>;
@group(0) @binding(1) var<uniform> camera: CameraData;
@group(0) @binding(2) var<storage, read_write> ray_buffer_header: RayBufferHeader;
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
    
    // This pixel has motion! Generate a ray.
    
    // Convert pixel to normalized device coordinates
    let nx = (f32(x) - camera.principal_point.x) / (f32(camera.width) / 2.0);
    let ny = (f32(y) - camera.principal_point.y) / (f32(camera.height) / 2.0);
    
    // Convert to angles
    let angle_h = nx * camera.fov_half_tan.x;
    let angle_v = ny * camera.fov_half_tan.y;
    
    // Direction in camera local space (-Z is forward in Bevy)
    var local_dir = vec3<f32>(angle_h, -angle_v, -1.0);
    local_dir = normalize(local_dir);
    
    // Rotate to world space
    let rotation = mat3x3<f32>(
        camera.rotation_row0,
        camera.rotation_row1,
        camera.rotation_row2,
    );
    let world_dir = rotation * local_dir;
    
    // Atomically allocate a slot in the ray buffer
    let slot = atomicAdd(&ray_buffer_header.ray_count, 1u);
    
    // Check if we have space
    if (slot >= ray_buffer_header.max_rays) {
        // Buffer full - decrement and bail
        atomicSub(&ray_buffer_header.ray_count, 1u);
        return;
    }
    
    // Write the ray
    rays[slot].camera_id = camera.camera_id;
    rays[slot].origin = camera.position;
    rays[slot].direction = world_dir;
}
```

### Dispatch Strategy for Multiple Cameras

Two options:

**Option A: Sequential Dispatches**
```rust
for camera in cameras {
    // Update camera uniform
    // Dispatch ray generation for this camera's difference mask
    pass.dispatch_workgroups(workgroups_x, workgroups_y, 1);
}
```

**Option B: Batched with Camera Index**
```rust
// Single dispatch with camera index in z dimension
pass.dispatch_workgroups(workgroups_x, workgroups_y, camera_count);
// Shader uses global_id.z to index into camera array
```

**Recommended**: Option A for simplicity. The shader is lightweight and dispatch overhead is minimal.

### Buffer Overflow Handling

The atomic counter checks against `max_rays`. If exceeded:
1. The shader decrements and returns (ray is lost)
2. CPU can detect overflow by checking `ray_count > max_rays * 0.9`
3. Optionally increase buffer size or reduce subsample factor

**Sizing**: With 4 cameras at 640x360, worst case is ~920K pixels. With subsample=2 and typical motion coverage of 1%, expect ~2300 rays. A 64KB buffer (1365 rays) should suffice with headroom. Use 128KB (2730 rays) for safety.

---

## 5. Buffer Readback

### Buffer Setup

```rust
fn create_ray_buffer(max_rays: usize) -> ShaderStorageBuffer {
    // Header (16 bytes) + rays (48 bytes each)
    let header_size = std::mem::size_of::<RayBufferHeader>();
    let rays_size = max_rays * std::mem::size_of::<GpuRay>();
    let total_size = header_size + rays_size;
    
    let mut buffer = ShaderStorageBuffer::new(
        &vec![0u8; total_size],
        RenderAssetUsages::RENDER_WORLD,
    );
    
    // Enable readback
    buffer.buffer_description.usage |= BufferUsages::COPY_SRC;
    
    buffer
}
```

### Readback Entity

```rust
commands.spawn((
    Readback::buffer(ray_buffer_handle.clone()),
    RayBufferReadback,
))
.observe(handle_ray_readback);
```

### Readback Handler

```rust
fn handle_ray_readback(
    event: On<ReadbackComplete>,
    mut raymarcher: ResMut<RaymarchState>,
) {
    let data: &[u8] = &**event;
    
    // Parse header
    let header: &RayBufferHeader = bytemuck::from_bytes(&data[0..16]);
    
    let ray_count = header.ray_count.min(MAX_RAYS as u32) as usize;
    
    if ray_count == 0 {
        return;
    }
    
    // Parse rays
    let rays_data = &data[16..];
    let rays: &[GpuRay] = bytemuck::cast_slice(&rays_data[..ray_count * 48]);
    
    // Convert to CPU Ray structs and process
    for gpu_ray in rays {
        let ray = Ray::new(
            Vec3::from(gpu_ray.origin),
            Vec3::from(gpu_ray.direction),
            RAY_INTENSITY,
        );
        
        raymarcher.pending_rays.push((gpu_ray.camera_id, ray));
    }
}
```

### Buffer Reset

Before each frame's compute shaders run, reset the atomic counter:

```rust
// In render graph node, before dispatching ray generation:
// Clear just the header (ray_count = 0)
encoder.clear_buffer(ray_buffer, 0, 4);  // Clear first u32
```

---

## 6. Integration with Existing Code

### Files to Modify

| File | Changes |
|------|---------|
| `render_camera.rs` | Remove CPU frame differencing, add GPU buffer handles |
| `motion_raymarch.rs` | Read from ray buffer instead of motion_pixels Vec |
| `scene.rs` | Add RenderLayers to entities |
| `targets.rs` | Add RenderLayers::layer(LAYER_TARGETS) to spawned targets |
| `voxels.rs` | Configure gizmos to use LAYER_DEBUG |
| `tracking.rs` | Configure gizmos to use LAYER_DEBUG |
| `lib.rs` | Add new GPU pipeline plugin |

### New Files to Create

| File | Purpose |
|------|---------|
| `gpu_pipeline.rs` | GPU compute pipeline setup, render graph nodes |
| `shaders/frame_difference.wgsl` | Frame differencing compute shader |
| `shaders/ray_generation.wgsl` | Ray generation compute shader |
| `render_layers.rs` | Constants and helpers for layer management |

### Files to Potentially Delete

| File | Reason |
|------|--------|
| None | Keep existing code for fallback/comparison |

### RenderCamera Changes

```rust
// Before:
#[derive(Component)]
pub struct RenderCamera {
    pub intrinsics: CameraIntrinsics,
    pub camera_id: u32,
    pub render_target: Handle<Image>,
    pub previous_frame: Option<Vec<u8>>,      // DELETE
    pub difference_mask: Vec<u8>,              // DELETE  
    pub motion_pixels: Vec<(u32, u32)>,        // DELETE
    // ...
}

// After:
#[derive(Component)]
pub struct RenderCamera {
    pub intrinsics: CameraIntrinsics,
    pub camera_id: u32,
    pub render_target: Handle<Image>,
    pub previous_frame_handle: Handle<Image>,  // GPU texture
    pub difference_mask_handle: Handle<Image>, // GPU texture
}
```

### MotionRaymarch Changes

```rust
// Before: Read from camera.motion_pixels
for &(u, v) in &camera.motion_pixels {
    let ray_dir = camera.ray_direction(cam_transform, u, v);
    // ...
}

// After: Read from shared ray buffer resource
pub fn motion_based_raymarch(
    mut grid_res: ResMut<VoxelGridResource>,
    ray_buffer: Res<RayBufferResource>,  // NEW
    sim_config: Res<SimulatorConfig>,
    // cameras no longer needed for rays!
) {
    for (camera_id, ray) in ray_buffer.pending_rays.drain(..) {
        let raymarcher = SimulatorRaymarcher::new(...);
        raymarcher.march_ray(&ray, grid_dims, &mut contributions);
        // ...
    }
}
```

---

## 7. Implementation Order

### Phase 1: Render Layers (No GPU changes)
1. Add layer constants to a new `render_layers.rs`
2. Modify `targets.rs` to add `RenderLayers::layer(LAYER_TARGETS)` to targets
3. Modify `scene.rs` to add `RenderLayers::layer(LAYER_DEFAULT)` to ground/lights
4. Configure gizmos to use `LAYER_DEBUG`
5. Add layer filter to RenderCamera spawning
6. Add `RenderLayers::all()` to debug camera

**Test**: Run simulator, verify gizmos don't appear in camera renders, motion pixel count drops dramatically.

### Phase 2: Frame Buffer Infrastructure
1. Create ping-pong buffer handles in RenderCamera
2. Add frame copy system (can be CPU-side initially)
3. Remove CPU `previous_frame: Option<Vec<u8>>`

**Test**: Verify frame differencing still works (same as before).

### Phase 3: Frame Differencing Shader
1. Create `frame_difference.wgsl`
2. Create compute pipeline in new `gpu_pipeline.rs`
3. Add render graph node that runs after cameras
4. Output to GPU texture, NOT CPU

**Test**: Add debug visualization of difference mask.

### Phase 4: Ray Generation Shader
1. Create `ray_generation.wgsl`
2. Create ray buffer storage
3. Add ray generation to render graph (after frame diff)
4. Add CPU uniform updates for camera poses each frame

**Test**: Verify ray count matches expected motion pixels.

### Phase 5: Buffer Readback
1. Add `Readback::buffer()` for ray buffer
2. Create `RayBufferResource` to hold pending rays
3. Modify `motion_based_raymarch` to consume from buffer

**Test**: Full pipeline working, same tracking results as before.

### Phase 6: Cleanup & Optimization
1. Remove unused CPU buffers from RenderCamera
2. Tune subsample factor based on performance
3. Add metrics (GPU time, ray count, buffer utilization)
4. Optional: Move raymarching to GPU too (future work)

---

## 8. Edge Cases & Error Handling

### Zero Motion Pixels
- Ray buffer header shows `ray_count = 0`
- Raymarching system does nothing (no voxel contributions)
- This is fine - the scene is static

### Buffer Overflow
- Shader atomically detects overflow, drops rays
- CPU can detect by checking `ray_count` vs buffer size
- Mitigation: Increase buffer, increase subsample, or accept dropped rays
- Log warning if overflow detected

### First Frame
- No previous frame exists
- Shader should output zero difference (or skip)
- Implemented via buffer initialization (all zeros)

### Camera Pose Changes
- Uniform buffer updated each frame with new transforms
- No special handling needed

### Camera Hot-Plug (future)
- Currently fixed 4 cameras
- For dynamic cameras: rebuild bind groups when camera count changes

### Resolution Changes
- Not supported mid-run
- Requires recreating all textures and buffers

---

## 9. Performance Estimates

### Current (CPU Pipeline)

| Operation | Data Size | Time (estimated) |
|-----------|-----------|------------------|
| GPU->CPU readback | 3.7 MB | 2-3 ms |
| Grayscale conversion | 921K ops x 4 | 0.5 ms |
| Frame differencing | 921K ops x 4 | 0.3 ms |
| Ray generation | ~2K rays x 4 | 0.1 ms |
| **Total** | | **~3-4 ms** |

### New (GPU Pipeline)

| Operation | Data Size | Time (estimated) |
|-----------|-----------|------------------|
| Frame diff (GPU) | 921K pixels x 4 | 0.1 ms |
| Ray generation (GPU) | ~2K rays | 0.05 ms |
| GPU->CPU readback | ~100 KB | 0.2 ms |
| **Total** | | **~0.35 ms** |

**Bandwidth Reduction**: 3.7 MB -> 100 KB = **37x reduction**

**Latency Reduction**: ~3-4 ms -> ~0.35 ms = **~10x improvement**

---

## 10. Future Enhancements

### GPU Raymarching
Move the DDA algorithm to GPU:
- Input: Ray buffer
- Output: Voxel contribution buffer
- Benefit: Massively parallel, could handle 100K+ rays

### Multi-Resolution Pyramid
For large scenes:
- Low-res difference for initial detection
- High-res refinement around motion areas
- Benefit: Better scaling with resolution

### Temporal Coherence
Use motion vectors from previous frame:
- Predict where motion will be
- Reduce false positives from noise
- Better tracking initialization

### Morphological Filtering
GPU-based erosion/dilation on difference mask:
- Remove single-pixel noise
- Fill holes in target silhouettes
- Done as additional compute pass

---

## Appendix A: Complete Shader Code

See `crates/iluvatar-simulator/assets/shaders/` for final implementations.

## Appendix B: Bevy Version Notes

This design targets **Bevy 0.15**. Key APIs used:
- `RenderLayers` from `bevy::render::view`
- `Readback::buffer()` from `bevy::render::gpu_readback`
- `ShaderStorageBuffer` from `bevy::render::storage`
- Compute pipelines via `ComputePipelineDescriptor`
- Render graph nodes via `render_graph::Node`

For Bevy 0.16+, check for API changes in:
- Render graph structure
- GPU readback system
- Storage buffer creation
