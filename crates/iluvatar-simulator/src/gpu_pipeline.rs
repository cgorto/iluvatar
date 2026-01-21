//! GPU compute pipeline for motion detection
//!
//! This module implements the GPU-accelerated motion detection pipeline:
//! 1. Frame differencing compute shader (per camera)
//! 2. Ray generation compute shader (all cameras -> shared buffer)
//! 3. Buffer readback for CPU consumption
//!
//! ## Pipeline Flow
//!
//! ```text
//! Camera Render -> Frame Diff Compute -> Ray Gen Compute -> Buffer Readback -> CPU Raymarch
//!      |                   |                    |
//!   render_target    difference_mask       ray_buffer
//! ```
//!
//! ## Render Graph Integration
//!
//! The compute shaders run as render graph nodes:
//! - `FrameDiffLabel` runs after `CameraDriverLabel`
//! - `RayGenLabel` runs after `FrameDiffLabel`

use std::borrow::Cow;
use std::sync::atomic::{AtomicU32, Ordering};

use bevy::{
    prelude::*,
    render::{
        extract_component::{ExtractComponent, ExtractComponentPlugin},
        extract_resource::{ExtractResource, ExtractResourcePlugin},
        gpu_readback::{Readback, ReadbackComplete},
        render_asset::RenderAssets,
        render_graph::{self, RenderGraph, RenderLabel},
        render_resource::{
            binding_types::{storage_buffer, texture_2d, texture_storage_2d, uniform_buffer},
            BindGroup, BindGroupEntries, BindGroupLayoutDescriptor, BindGroupLayoutEntries, Buffer,
            BufferBinding, BufferInitDescriptor, BufferUsages, CachedComputePipelineId,
            ComputePassDescriptor, ComputePipelineDescriptor, Extent3d as WgpuExtent3d, Origin3d,
            PipelineCache, ShaderStages, ShaderType, StorageTextureAccess, TexelCopyTextureInfo,
            Texture, TextureAspect, TextureFormat, TextureSampleType,
        },
        renderer::{RenderContext, RenderDevice, RenderQueue},
        storage::{GpuShaderStorageBuffer, ShaderStorageBuffer},
        texture::GpuImage,
        Render, RenderApp, RenderStartup, RenderSystems,
    },
};

use iluvatar_core::Ray;

use crate::render_camera::RenderCamera;
use crate::voxels::SimulatorConfig;

/// Maximum rays in the buffer per frame
/// At 640x360 with 4 cameras and subsample=2, worst case is ~115k pixels
/// But in practice, motion covers much less. 16K should be plenty.
const MAX_RAYS: usize = 16384;

/// Plugin for GPU motion detection pipeline
pub struct GpuMotionPipelinePlugin;

impl Plugin for GpuMotionPipelinePlugin {
    fn build(&self, app: &mut App) {
        // Initialize main world resources
        app.init_resource::<RayBufferResource>()
            .init_resource::<GpuPipelineMetrics>()
            .add_plugins((
                ExtractResourcePlugin::<GpuPipelineConfig>::default(),
                ExtractResourcePlugin::<RayBufferHandle>::default(),
                ExtractComponentPlugin::<GpuRenderCamera>::default(),
            ))
            .add_systems(Startup, setup_ray_readback)
            .add_systems(PostUpdate, sync_gpu_render_cameras);
    }

    fn finish(&self, app: &mut App) {
        // Insert the config resource so it can be extracted
        let config = GpuPipelineConfig::default();
        app.insert_resource(config);

        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };

        render_app
            .add_systems(RenderStartup, init_pipelines)
            .add_systems(
                Render,
                prepare_bind_groups.in_set(RenderSystems::PrepareBindGroups),
            );

        // Add render graph nodes
        let mut render_graph = render_app.world_mut().resource_mut::<RenderGraph>();
        render_graph.add_node(FrameDiffLabel, FrameDiffNode::default());
        render_graph.add_node(RayGenLabel, RayGenNode::default());
        render_graph.add_node(FrameCopyLabel, FrameCopyNode);

        // Order: CameraDriver -> FrameDiff -> RayGen -> FrameCopy
        render_graph.add_node_edge(bevy::render::graph::CameraDriverLabel, FrameDiffLabel);
        render_graph.add_node_edge(FrameDiffLabel, RayGenLabel);
        render_graph.add_node_edge(RayGenLabel, FrameCopyLabel);
    }
}

// =============================================================================
// GPU Structures (must match WGSL exactly with proper alignment)
// =============================================================================

/// GPU ray structure - 48 bytes, 16-byte aligned
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable, ShaderType)]
#[repr(C)]
pub struct GpuRay {
    pub camera_id: u32,
    pub _pad0: u32,
    pub _pad1: u32,
    pub _pad2: u32,
    pub origin: Vec3,
    pub _pad3: f32,
    pub direction: Vec3,
    pub _pad4: f32,
}

/// Ray buffer header - 32 bytes (padded for alignment)
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable, ShaderType)]
#[repr(C)]
pub struct RayBufferHeader {
    pub ray_count: u32,
    pub max_rays: u32,
    pub frame: u32,
    pub _padding0: u32,
    // Additional padding to reach 32-byte alignment for storage buffer offset
    pub _padding1: u32,
    pub _padding2: u32,
    pub _padding3: u32,
    pub _padding4: u32,
}

/// Frame difference parameters uniform - 16 bytes
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable, ShaderType)]
#[repr(C)]
struct DiffParams {
    width: u32,
    height: u32,
    threshold: u32,
    _padding: u32,
}

/// Camera uniform data for ray generation - 96 bytes (6 * 16)
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable, ShaderType)]
#[repr(C)]
struct CameraUniform {
    camera_id: u32,
    width: u32,
    height: u32,
    subsample: u32,
    position: Vec3,
    _pad0: f32,
    rotation_row0: Vec3,
    _pad1: f32,
    rotation_row1: Vec3,
    _pad2: f32,
    rotation_row2: Vec3,
    _pad3: f32,
    fov_half_tan: Vec2,
    principal_point: Vec2,
}

impl CameraUniform {
    fn from_camera(camera: &GpuRenderCamera) -> Self {
        let rot = camera.world_rotation;
        // Convert quaternion to rotation matrix - get the basis vectors
        // Note: In Bevy, camera looks down -Z locally. The shader uses -1 for local_dir.z,
        // so we need to provide +Z as the forward vector so that -1 * forward = -Z direction.
        let right = rot * Vec3::X;
        let up = rot * Vec3::Y;
        let forward = rot * Vec3::Z; // Positive Z so shader's -1 gives correct direction

        Self {
            camera_id: camera.camera_id,
            width: camera.resolution.x,
            height: camera.resolution.y,
            subsample: camera.subsample,
            position: camera.world_position,
            _pad0: 0.0,
            rotation_row0: right,
            _pad1: 0.0,
            rotation_row1: up,
            _pad2: 0.0,
            rotation_row2: forward,
            _pad3: 0.0,
            fov_half_tan: Vec2::new(
                (camera.fov_horizontal / 2.0).tan(),
                (camera.fov_vertical / 2.0).tan(),
            ),
            principal_point: Vec2::new(0.5, 0.5), // Normalized center
        }
    }
}

// =============================================================================
// Main World Resources
// =============================================================================

/// Resource to hold rays after GPU readback for CPU consumption
#[derive(Resource, Default)]
pub struct RayBufferResource {
    /// Rays ready for CPU processing: (camera_id, ray)
    pub pending_rays: Vec<(u32, Ray)>,
    /// Last frame's ray count for debugging
    pub last_ray_count: u32,
}

/// GPU pipeline performance metrics
#[derive(Resource, Default)]
pub struct GpuPipelineMetrics {
    /// Number of rays generated last frame
    pub ray_count: u32,
    /// Buffer utilization (ray_count / MAX_RAYS)
    pub buffer_utilization: f32,
    /// Frame number for tracking
    pub frame: u32,
}

/// Configuration extracted to render world
#[derive(Resource, Default)]
struct GpuPipelineConfig {
    frame_count: AtomicU32,
}

// Manual ExtractResource implementation for GpuPipelineConfig
// since AtomicU32 doesn't implement Clone
impl ExtractResource for GpuPipelineConfig {
    type Source = GpuPipelineConfig;

    fn extract_resource(source: &Self::Source) -> Self {
        Self {
            frame_count: AtomicU32::new(source.frame_count.load(Ordering::Relaxed)),
        }
    }
}

/// Component with GPU-relevant camera data, extracted to render world
#[derive(Component, Clone, ExtractComponent)]
pub struct GpuRenderCamera {
    pub camera_id: u32,
    pub render_target: Handle<Image>,
    pub previous_frame: Handle<Image>,
    pub difference_mask: Handle<Image>,
    pub resolution: UVec2,
    pub difference_threshold: u8,
    pub subsample: u32,
    pub has_previous: bool,
    pub fov_horizontal: f32,
    pub fov_vertical: f32,
    /// Camera world position - must be synced from Transform before extraction
    pub world_position: Vec3,
    /// Camera world rotation - must be synced from Transform before extraction
    pub world_rotation: Quat,
}

/// System to sync RenderCamera to GpuRenderCamera component
fn sync_gpu_render_cameras(
    mut commands: Commands,
    cameras: Query<(Entity, &RenderCamera, &GlobalTransform), Without<GpuRenderCamera>>,
    mut existing: Query<(&RenderCamera, &GlobalTransform, &mut GpuRenderCamera)>,
) {
    // Add GpuRenderCamera to entities that don't have one
    for (entity, camera, transform) in cameras.iter() {
        commands.entity(entity).insert(GpuRenderCamera {
            camera_id: camera.camera_id,
            render_target: camera.render_target.clone(),
            previous_frame: camera.previous_frame.clone(),
            difference_mask: camera.difference_mask.clone(),
            resolution: camera.resolution,
            difference_threshold: camera.difference_threshold,
            subsample: camera.subsample,
            has_previous: camera.has_previous,
            fov_horizontal: camera.intrinsics.fov.horizontal,
            fov_vertical: camera.intrinsics.fov.vertical,
            world_position: transform.translation(),
            world_rotation: transform.to_scale_rotation_translation().1,
        });
    }

    // Update existing GpuRenderCamera components (sync transform every frame)
    for (camera, transform, mut gpu_camera) in existing.iter_mut() {
        gpu_camera.has_previous = camera.has_previous;
        gpu_camera.world_position = transform.translation();
        gpu_camera.world_rotation = transform.to_scale_rotation_translation().1;
    }
}

// =============================================================================
// Render World Resources
// =============================================================================

/// The compute pipelines
#[derive(Resource)]
struct GpuPipelines {
    frame_diff_layout: BindGroupLayoutDescriptor,
    frame_diff_pipeline: CachedComputePipelineId,
    ray_gen_layout: BindGroupLayoutDescriptor,
    ray_gen_pipeline: CachedComputePipelineId,
}

/// Per-camera bind groups for frame differencing
#[derive(Resource, Default)]
struct FrameDiffBindGroups {
    /// (camera_id, bind_group, resolution, has_previous)
    groups: Vec<(u32, BindGroup, UVec2, bool)>,
}

/// Per-camera bind groups for ray generation
#[derive(Resource, Default)]
struct RayGenBindGroups {
    /// (camera_id, bind_group, resolution, subsample)
    groups: Vec<(u32, BindGroup, UVec2, u32)>,
}

/// Per-camera texture info for frame copy operations
#[derive(Resource, Default)]
struct FrameCopyTextures {
    /// (camera_id, current_texture, previous_texture, resolution)
    textures: Vec<FrameCopyInfo>,
}

/// Info needed for a single frame copy operation
struct FrameCopyInfo {
    #[allow(dead_code)]
    camera_id: u32,
    current_texture: Texture,
    previous_texture: Texture,
    resolution: UVec2,
}

/// The shared ray buffer on GPU
#[derive(Resource)]
struct GpuRayBuffer {
    buffer: Buffer,
}

/// Handle to the ray buffer for readback
#[derive(Resource, Clone)]
struct RayBufferHandle(Handle<ShaderStorageBuffer>);

impl ExtractResource for RayBufferHandle {
    type Source = RayBufferHandle;

    fn extract_resource(source: &Self::Source) -> Self {
        source.clone()
    }
}

// =============================================================================
// Setup Systems
// =============================================================================

/// Setup the ray buffer readback in main world
fn setup_ray_readback(mut commands: Commands, mut buffers: ResMut<Assets<ShaderStorageBuffer>>) {
    // Create the ray buffer with enough space for header + rays
    let header_size = std::mem::size_of::<RayBufferHeader>();
    let rays_size = MAX_RAYS * std::mem::size_of::<GpuRay>();
    let total_size = header_size + rays_size;

    // Initialize header
    let header = RayBufferHeader {
        ray_count: 0,
        max_rays: MAX_RAYS as u32,
        frame: 0,
        _padding0: 0,
        _padding1: 0,
        _padding2: 0,
        _padding3: 0,
        _padding4: 0,
    };

    // Create initial data as Vec<u32> for the header, then pad with zeros
    // ShaderStorageBuffer works with ShaderType, so we need to use u32
    let header_words = header_size / 4;
    let total_words = total_size / 4;
    let mut data = vec![0u32; total_words];

    // Copy header bytes as u32s
    let header_bytes = bytemuck::bytes_of(&header);
    let header_u32s: &[u32] = bytemuck::cast_slice(header_bytes);
    data[..header_words].copy_from_slice(header_u32s);

    let mut buffer = ShaderStorageBuffer::from(data);
    buffer.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let handle = buffers.add(buffer);

    commands.insert_resource(RayBufferHandle(handle.clone()));

    // Add the ExtractResource plugin for RayBufferHandle
    // (This is done in the plugin build, but we need the resource first)

    // Spawn readback entity with observer
    commands
        .spawn(Readback::buffer(handle))
        .observe(handle_ray_readback);

    info!(
        "GPU motion pipeline: ray buffer created ({} bytes, max {} rays)",
        total_size, MAX_RAYS
    );
}

/// Handle ray buffer readback completion
fn handle_ray_readback(
    event: On<ReadbackComplete>,
    mut ray_res: ResMut<RayBufferResource>,
    mut metrics: ResMut<GpuPipelineMetrics>,
    sim_config: Option<Res<SimulatorConfig>>,
) {
    let data: &[u8] = &event;

    let header_size = std::mem::size_of::<RayBufferHeader>();
    if data.len() < header_size {
        warn!("Ray buffer readback too small: {} bytes", data.len());
        return;
    }

    // Parse header
    let header: RayBufferHeader = *bytemuck::from_bytes(&data[0..header_size]);
    let ray_count = header.ray_count.min(MAX_RAYS as u32) as usize;

    // Update metrics
    metrics.ray_count = ray_count as u32;
    metrics.buffer_utilization = ray_count as f32 / MAX_RAYS as f32;
    metrics.frame = header.frame;

    ray_res.last_ray_count = ray_count as u32;
    ray_res.pending_rays.clear();

    if ray_count == 0 {
        return;
    }

    // Get ray intensity from config (default to 1.0)
    let ray_intensity = sim_config.map(|c| c.ray_intensity).unwrap_or(1.0);

    // Parse rays
    let rays_data = &data[header_size..];
    let ray_size = std::mem::size_of::<GpuRay>();
    let rays_bytes = ray_count * ray_size;

    if rays_data.len() < rays_bytes {
        warn!(
            "Ray buffer data too small: {} bytes, expected {}",
            rays_data.len(),
            rays_bytes
        );
        return;
    }

    let rays: &[GpuRay] = bytemuck::cast_slice(&rays_data[..rays_bytes]);

    for gpu_ray in rays {
        let origin = gpu_ray.origin;
        let direction = gpu_ray.direction;

        // Skip invalid rays (zero direction)
        if direction.length_squared() < 0.001 {
            continue;
        }

        let ray = Ray::new(origin, direction, ray_intensity);
        ray_res.pending_rays.push((gpu_ray.camera_id, ray));
    }

    debug!(
        "GPU readback: {} rays from {} cameras (frame {})",
        ray_res.pending_rays.len(),
        ray_res
            .pending_rays
            .iter()
            .map(|(id, _)| *id)
            .collect::<std::collections::HashSet<_>>()
            .len(),
        header.frame
    );
}

/// Initialize compute pipelines in render world
fn init_pipelines(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    pipeline_cache: Res<PipelineCache>,
) {
    // Frame difference bind group layout descriptor
    let frame_diff_layout = BindGroupLayoutDescriptor::new(
        "frame_diff_bind_group_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                // binding 0: current_frame (texture_2d<f32>)
                texture_2d(TextureSampleType::Float { filterable: false }),
                // binding 1: previous_frame (texture_2d<f32>)
                texture_2d(TextureSampleType::Float { filterable: false }),
                // binding 2: params (uniform)
                uniform_buffer::<DiffParams>(false),
                // binding 3: difference_mask (storage texture write)
                texture_storage_2d(TextureFormat::R8Uint, StorageTextureAccess::WriteOnly),
            ),
        ),
    );

    // Ray generation bind group layout descriptor
    let ray_gen_layout = BindGroupLayoutDescriptor::new(
        "ray_gen_bind_group_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                // binding 0: difference_mask (texture_2d<u32>)
                texture_2d(TextureSampleType::Uint),
                // binding 1: camera uniform
                uniform_buffer::<CameraUniform>(false),
                // binding 2: ray_header (storage buffer)
                storage_buffer::<RayBufferHeader>(false),
                // binding 3: rays array (storage buffer)
                storage_buffer::<[GpuRay; MAX_RAYS]>(false),
            ),
        ),
    );

    // Load shaders
    let diff_shader = asset_server.load("shaders/frame_difference.wgsl");
    let ray_gen_shader = asset_server.load("shaders/ray_generation.wgsl");

    // Create frame difference pipeline
    let frame_diff_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some(Cow::from("frame_diff_pipeline")),
        layout: vec![frame_diff_layout.clone()],
        shader: diff_shader,
        entry_point: Some(Cow::from("main")),
        ..default()
    });

    // Create ray generation pipeline
    let ray_gen_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some(Cow::from("ray_gen_pipeline")),
        layout: vec![ray_gen_layout.clone()],
        shader: ray_gen_shader,
        entry_point: Some(Cow::from("main")),
        ..default()
    });

    commands.insert_resource(GpuPipelines {
        frame_diff_layout,
        frame_diff_pipeline,
        ray_gen_layout,
        ray_gen_pipeline,
    });

    commands.init_resource::<FrameDiffBindGroups>();
    commands.init_resource::<RayGenBindGroups>();

    info!("GPU motion pipeline: compute pipelines initialized");
}

/// Prepare bind groups for the current frame
#[allow(clippy::too_many_arguments)]
fn prepare_bind_groups(
    mut commands: Commands,
    pipelines: Option<Res<GpuPipelines>>,
    mut frame_diff_bind_groups: ResMut<FrameDiffBindGroups>,
    mut ray_gen_bind_groups: ResMut<RayGenBindGroups>,
    render_device: Res<RenderDevice>,
    pipeline_cache: Res<PipelineCache>,
    _queue: Res<RenderQueue>,
    gpu_images: Res<RenderAssets<GpuImage>>,
    gpu_buffers: Res<RenderAssets<GpuShaderStorageBuffer>>,
    cameras: Query<&GpuRenderCamera>,
    ray_buffer_handle: Option<Res<RayBufferHandle>>,
    config: Option<Res<GpuPipelineConfig>>,
) {
    let Some(pipelines) = pipelines else {
        return;
    };
    let Some(ray_buffer_handle) = ray_buffer_handle else {
        return;
    };
    let Some(config) = config else {
        return;
    };

    // Get ray buffer
    let Some(ray_buffer) = gpu_buffers.get(&ray_buffer_handle.0) else {
        return;
    };

    // Get bind group layouts from pipeline cache
    let frame_diff_layout = pipeline_cache.get_bind_group_layout(&pipelines.frame_diff_layout);
    let ray_gen_layout = pipeline_cache.get_bind_group_layout(&pipelines.ray_gen_layout);

    // Increment frame counter
    let _frame = config.frame_count.fetch_add(1, Ordering::Relaxed);

    // Clear old bind groups
    frame_diff_bind_groups.groups.clear();
    ray_gen_bind_groups.groups.clear();

    // Track textures for frame copy
    let mut frame_copy_textures = FrameCopyTextures::default();

    for camera in cameras.iter() {
        // Get GPU textures for this camera
        let Some(current) = gpu_images.get(&camera.render_target) else {
            continue;
        };
        let Some(previous) = gpu_images.get(&camera.previous_frame) else {
            continue;
        };
        let Some(diff_mask) = gpu_images.get(&camera.difference_mask) else {
            continue;
        };

        // Create diff params buffer
        let diff_params = DiffParams {
            width: camera.resolution.x,
            height: camera.resolution.y,
            threshold: camera.difference_threshold as u32,
            _padding: 0,
        };
        let diff_params_buffer = render_device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("diff_params_buffer"),
            contents: bytemuck::bytes_of(&diff_params),
            usage: BufferUsages::UNIFORM,
        });

        // Create frame diff bind group
        let diff_bind_group = render_device.create_bind_group(
            Some("frame_diff_bind_group"),
            &frame_diff_layout,
            &BindGroupEntries::sequential((
                &current.texture_view,
                &previous.texture_view,
                diff_params_buffer.as_entire_binding(),
                &diff_mask.texture_view,
            )),
        );

        frame_diff_bind_groups.groups.push((
            camera.camera_id,
            diff_bind_group,
            camera.resolution,
            camera.has_previous,
        ));

        // Create camera uniform for ray generation
        let camera_uniform = CameraUniform::from_camera(camera);
        let camera_buffer = render_device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("camera_uniform_buffer"),
            contents: bytemuck::bytes_of(&camera_uniform),
            usage: BufferUsages::UNIFORM,
        });

        // Create ray gen bind group
        // The ray buffer has header at offset 0, rays array starting after header
        let header_size = std::mem::size_of::<RayBufferHeader>() as u64;
        let rays_size = (MAX_RAYS * std::mem::size_of::<GpuRay>()) as u64;

        let ray_gen_bind_group = render_device.create_bind_group(
            Some("ray_gen_bind_group"),
            &ray_gen_layout,
            &BindGroupEntries::sequential((
                &diff_mask.texture_view,
                camera_buffer.as_entire_binding(),
                BufferBinding {
                    buffer: &ray_buffer.buffer,
                    offset: 0,
                    size: Some(std::num::NonZeroU64::new(header_size).unwrap()),
                },
                BufferBinding {
                    buffer: &ray_buffer.buffer,
                    offset: header_size,
                    size: Some(std::num::NonZeroU64::new(rays_size).unwrap()),
                },
            )),
        );

        ray_gen_bind_groups.groups.push((
            camera.camera_id,
            ray_gen_bind_group,
            camera.resolution,
            camera.subsample,
        ));

        // Store texture references for frame copy
        frame_copy_textures.textures.push(FrameCopyInfo {
            camera_id: camera.camera_id,
            current_texture: current.texture.clone(),
            previous_texture: previous.texture.clone(),
            resolution: camera.resolution,
        });
    }

    // Store the ray buffer resource so nodes can access it for clearing
    commands.insert_resource(GpuRayBuffer {
        buffer: ray_buffer.buffer.clone(),
    });

    // Store frame copy textures
    commands.insert_resource(frame_copy_textures);
}

// =============================================================================
// Render Graph Nodes
// =============================================================================

/// Render graph label for frame differencing
#[derive(Debug, Hash, PartialEq, Eq, Clone, RenderLabel)]
struct FrameDiffLabel;

/// Render graph label for ray generation
#[derive(Debug, Hash, PartialEq, Eq, Clone, RenderLabel)]
struct RayGenLabel;

/// State tracking for pipeline loading
#[derive(Default)]
enum PipelineState {
    #[default]
    Loading,
    Ready,
}

/// Render graph node that runs frame differencing compute shader
#[derive(Default)]
struct FrameDiffNode {
    state: PipelineState,
}

impl render_graph::Node for FrameDiffNode {
    fn update(&mut self, world: &mut World) {
        let Some(pipelines) = world.get_resource::<GpuPipelines>() else {
            return;
        };
        let pipeline_cache = world.resource::<PipelineCache>();

        if matches!(self.state, PipelineState::Loading)
            && pipeline_cache
                .get_compute_pipeline(pipelines.frame_diff_pipeline)
                .is_some()
        {
            self.state = PipelineState::Ready;
        }
    }

    fn run(
        &self,
        _graph: &mut render_graph::RenderGraphContext,
        render_context: &mut RenderContext,
        world: &World,
    ) -> Result<(), render_graph::NodeRunError> {
        if !matches!(self.state, PipelineState::Ready) {
            return Ok(());
        }

        let pipelines = world.resource::<GpuPipelines>();
        let pipeline_cache = world.resource::<PipelineCache>();
        let bind_groups = world.resource::<FrameDiffBindGroups>();

        let Some(pipeline) = pipeline_cache.get_compute_pipeline(pipelines.frame_diff_pipeline)
        else {
            return Ok(());
        };

        if bind_groups.groups.is_empty() {
            return Ok(());
        }

        let mut pass =
            render_context
                .command_encoder()
                .begin_compute_pass(&ComputePassDescriptor {
                    label: Some("frame_diff_pass"),
                    ..default()
                });

        pass.set_pipeline(pipeline);

        for (_camera_id, bind_group, resolution, has_previous) in &bind_groups.groups {
            // Skip first frame (no previous frame to compare)
            if !has_previous {
                continue;
            }

            pass.set_bind_group(0, bind_group, &[]);

            // Dispatch with 16x16 workgroups
            let workgroups_x = resolution.x.div_ceil(16);
            let workgroups_y = resolution.y.div_ceil(16);
            pass.dispatch_workgroups(workgroups_x, workgroups_y, 1);
        }

        Ok(())
    }
}

/// Render graph node that runs ray generation compute shader
#[derive(Default)]
struct RayGenNode {
    state: PipelineState,
}

impl render_graph::Node for RayGenNode {
    fn update(&mut self, world: &mut World) {
        let Some(pipelines) = world.get_resource::<GpuPipelines>() else {
            return;
        };
        let pipeline_cache = world.resource::<PipelineCache>();

        if matches!(self.state, PipelineState::Loading)
            && pipeline_cache
                .get_compute_pipeline(pipelines.ray_gen_pipeline)
                .is_some()
        {
            self.state = PipelineState::Ready;
        }
    }

    fn run(
        &self,
        _graph: &mut render_graph::RenderGraphContext,
        render_context: &mut RenderContext,
        world: &World,
    ) -> Result<(), render_graph::NodeRunError> {
        if !matches!(self.state, PipelineState::Ready) {
            return Ok(());
        }

        let pipelines = world.resource::<GpuPipelines>();
        let pipeline_cache = world.resource::<PipelineCache>();
        let bind_groups = world.resource::<RayGenBindGroups>();

        let Some(ray_buffer) = world.get_resource::<GpuRayBuffer>() else {
            return Ok(());
        };

        let Some(pipeline) = pipeline_cache.get_compute_pipeline(pipelines.ray_gen_pipeline) else {
            return Ok(());
        };

        if bind_groups.groups.is_empty() {
            return Ok(());
        }

        // First, clear the ray count in the buffer header
        // We write just the first 4 bytes (ray_count) to 0
        let encoder = render_context.command_encoder();
        encoder.clear_buffer(&ray_buffer.buffer, 0, Some(4));

        let mut pass = encoder.begin_compute_pass(&ComputePassDescriptor {
            label: Some("ray_gen_pass"),
            ..default()
        });

        pass.set_pipeline(pipeline);

        for (_camera_id, bind_group, resolution, subsample) in &bind_groups.groups {
            pass.set_bind_group(0, bind_group, &[]);

            // Dispatch with subsampled dimensions
            // Each thread handles one subsampled pixel
            let dispatch_x = resolution.x.div_ceil(*subsample);
            let dispatch_y = resolution.y.div_ceil(*subsample);
            let workgroups_x = dispatch_x.div_ceil(16);
            let workgroups_y = dispatch_y.div_ceil(16);
            pass.dispatch_workgroups(workgroups_x, workgroups_y, 1);
        }

        Ok(())
    }
}

/// Render graph label for frame copy
#[derive(Debug, Hash, PartialEq, Eq, Clone, RenderLabel)]
struct FrameCopyLabel;

/// Render graph node that copies current frame to previous frame buffer
struct FrameCopyNode;

impl render_graph::Node for FrameCopyNode {
    fn run(
        &self,
        _graph: &mut render_graph::RenderGraphContext,
        render_context: &mut RenderContext,
        world: &World,
    ) -> Result<(), render_graph::NodeRunError> {
        let Some(frame_copy) = world.get_resource::<FrameCopyTextures>() else {
            return Ok(());
        };

        if frame_copy.textures.is_empty() {
            return Ok(());
        }

        let encoder = render_context.command_encoder();

        for info in &frame_copy.textures {
            // Copy the entire current frame texture to previous frame
            // This is needed for the next frame's difference computation
            let copy_size = WgpuExtent3d {
                width: info.resolution.x,
                height: info.resolution.y,
                depth_or_array_layers: 1,
            };

            encoder.copy_texture_to_texture(
                TexelCopyTextureInfo {
                    texture: &info.current_texture,
                    mip_level: 0,
                    origin: Origin3d::ZERO,
                    aspect: TextureAspect::All,
                },
                TexelCopyTextureInfo {
                    texture: &info.previous_texture,
                    mip_level: 0,
                    origin: Origin3d::ZERO,
                    aspect: TextureAspect::All,
                },
                copy_size,
            );
        }

        Ok(())
    }
}
