//! Render-to-texture cameras with GPU motion detection
//!
//! This module implements cameras that actually render the scene and perform
//! pixel-based motion detection using GPU compute shaders.
//!
//! ## Architecture
//!
//! Each `RenderCamera` has:
//! - A Bevy `Camera` component that renders to an `Image` asset (render target)
//! - GPU textures: render_target, previous_frame, difference_mask
//! - GPU compute shaders handle frame differencing and ray generation
//!
//! ## Pipeline (GPU-accelerated)
//!
//! 1. Bevy renders the scene to each camera's render target (GPU)
//! 2. Frame difference compute shader compares current vs previous (GPU)
//! 3. Ray generation compute shader creates rays for motion pixels (GPU)
//! 4. Ray buffer is read back to CPU for voxel raymarching
//! 5. Current frame is copied to previous for next iteration
//!
//! ## Performance Notes
//!
//! - All frame processing happens on GPU (no RGBA readback!)
//! - Only the ray buffer (~100KB max) is read back to CPU
//! - Typical bandwidth reduction: 3.7MB → ~10KB per frame

use bevy::{
    asset::RenderAssetUsages,
    camera::RenderTarget,
    prelude::*,
    render::render_resource::{Extent3d, TextureDimension, TextureFormat, TextureUsages},
};

use iluvatar_core::{CameraIntrinsics, DistortionModel, Fov};

use crate::render_layers::render_camera_layers;

/// Plugin that sets up render-to-texture cameras with GPU motion detection
pub struct RenderCameraPlugin;

impl Plugin for RenderCameraPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<RenderCameraConfig>()
            .add_systems(Startup, spawn_render_cameras)
            .add_systems(Update, (update_has_previous, debug_print_gpu_stats));
    }
}

/// Configuration for render cameras
#[derive(Resource, Clone)]
pub struct RenderCameraConfig {
    /// Resolution for render targets (lower = faster, higher = more accurate)
    pub resolution: UVec2,
    /// Difference threshold (0-255) - pixel changes below this are ignored
    pub difference_threshold: u8,
    /// Subsample factor for motion pixels (1 = all pixels, 2 = every other, etc.)
    /// Higher values reduce ray count but may miss small targets
    pub subsample: u32,
}

impl Default for RenderCameraConfig {
    fn default() -> Self {
        Self {
            resolution: UVec2::new(1280, 720), // Half of 1280x720 for performance
            difference_threshold: 10,          // ~6% brightness change
            subsample: 1,                      // Every other motion pixel
        }
    }
}

/// Component marking a camera that renders to texture for motion detection
#[derive(Component)]
pub struct RenderCamera {
    /// Camera intrinsics for ray generation
    pub intrinsics: CameraIntrinsics,
    /// Unique camera ID (0-63 for bitmask tracking)
    pub camera_id: u32,
    /// Handle to the render target image (camera renders here)
    pub render_target: Handle<Image>,
    /// Handle to previous frame texture (for GPU differencing)
    pub previous_frame: Handle<Image>,
    /// Handle to difference mask texture (GPU compute output)
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

impl RenderCamera {
    /// Convert (u, v) pixel coordinates to normalized [-1, 1] range
    pub fn pixel_to_normalized(&self, u: u32, v: u32) -> (f32, f32) {
        let nx = (u as f32 - self.intrinsics.principal_point.x)
            / (self.intrinsics.resolution.x as f32 / 2.0);
        let ny = (v as f32 - self.intrinsics.principal_point.y)
            / (self.intrinsics.resolution.y as f32 / 2.0);
        (nx, ny)
    }

    /// Generate a ray direction for a given pixel coordinate (CPU fallback)
    pub fn ray_direction(&self, camera_transform: &Transform, u: u32, v: u32) -> Vec3 {
        let (nx, ny) = self.pixel_to_normalized(u, v);

        let half_fov_h = self.intrinsics.fov.horizontal / 2.0;
        let half_fov_v = self.intrinsics.fov.vertical / 2.0;

        let slope_h = nx * half_fov_h.tan();
        let slope_v = ny * half_fov_v.tan();

        // Direction in camera local space (-Z is forward in Bevy)
        let local_dir = Vec3::new(slope_h, -slope_v, -1.0).normalize();

        // Transform to world space
        camera_transform.rotation * local_dir
    }
}

/// Camera placement configuration
struct CameraPlacement {
    position: Vec3,
    look_at: Vec3,
    color: Color,
}

// =============================================================================
// Texture Creation Helpers
// =============================================================================

/// Create a GPU texture with proper usage flags for compute shader access
fn create_gpu_texture(
    resolution: UVec2,
    format: TextureFormat,
    for_compute_write: bool,
    for_render_attachment: bool,
) -> Image {
    let size = Extent3d {
        width: resolution.x,
        height: resolution.y,
        depth_or_array_layers: 1,
    };

    // Determine bytes per pixel for proper data allocation
    let bytes_per_pixel = match format {
        TextureFormat::Rgba8Unorm | TextureFormat::Rgba8UnormSrgb => 4,
        TextureFormat::R8Uint | TextureFormat::R8Unorm => 1,
        _ => 4,
    };

    let data = vec![0u8; (resolution.x * resolution.y) as usize * bytes_per_pixel];

    let mut image = Image::new(
        size,
        TextureDimension::D2,
        data,
        format,
        RenderAssetUsages::RENDER_WORLD,
    );

    // Build usage flags based on requirements
    let mut usage = TextureUsages::TEXTURE_BINDING; // For reading in shaders

    if for_render_attachment {
        usage |= TextureUsages::RENDER_ATTACHMENT; // Camera can render to it
    }

    if for_compute_write {
        usage |= TextureUsages::STORAGE_BINDING; // Compute shader can write
    }

    // Always need COPY_SRC and COPY_DST for ping-pong copy
    usage |= TextureUsages::COPY_SRC | TextureUsages::COPY_DST;

    image.texture_descriptor.usage = usage;

    image
}

/// Create the render target (camera renders here)
fn create_render_target_image(resolution: UVec2) -> Image {
    // Use Rgba8UnormSrgb for the render target - this is what Bevy's Camera3d expects
    // and what the frame difference compute shader can read via texture_2d<f32>
    create_gpu_texture(resolution, TextureFormat::Rgba8UnormSrgb, false, true)
}

/// Create the previous frame buffer (for compute shader reading)
fn create_previous_frame_image(resolution: UVec2) -> Image {
    // Previous frame is read by compute shader, written via copy from render target
    // Must match render target format (Rgba8UnormSrgb) for GPU copy operations
    create_gpu_texture(resolution, TextureFormat::Rgba8UnormSrgb, false, false)
}

/// Create the difference mask (compute shader output, R8Uint for motion flags)
fn create_difference_mask_image(resolution: UVec2) -> Image {
    // Difference mask is written by frame diff shader, read by ray gen shader
    create_gpu_texture(resolution, TextureFormat::R8Uint, true, false)
}

// =============================================================================
// Camera Spawning
// =============================================================================

/// Spawn render cameras positioned around the scene
fn spawn_render_cameras(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    config: Res<RenderCameraConfig>,
) {
    // Camera placements - positioned around the scene
    let placements = [
        CameraPlacement {
            position: Vec3::new(-500.0, 2.0, -500.0),
            look_at: Vec3::new(0.0, 250.0, 0.0),
            color: Color::srgb(0.8, 0.2, 0.2), // Red
        },
        //^^^ this is the trouble maker
        CameraPlacement {
            position: Vec3::new(-250.0, 3.0, -490.0),
            look_at: Vec3::new(0.0, 250.0, 0.0),
            color: Color::srgb(0.8, 0.2, 0.2), // Red
        },
        CameraPlacement {
            position: Vec3::new(500.0, 2.0, -500.0),
            look_at: Vec3::new(0.0, 250.0, 0.0),
            color: Color::srgb(0.2, 0.8, 0.2), // Green
        },
        CameraPlacement {
            position: Vec3::new(500.0, 2.0, 500.0),
            look_at: Vec3::new(0.0, 250.0, 0.0),
            color: Color::srgb(0.2, 0.2, 0.8), // Blue
        },
        CameraPlacement {
            position: Vec3::new(-500.0, 2.0, 500.0),
            look_at: Vec3::new(0.0, 250.0, 0.0),
            color: Color::srgb(0.8, 0.8, 0.2), // Yellow
        },
    ];

    // Mesh for camera visualization (a small box to show where cameras are)
    let camera_mesh = meshes.add(Cuboid::new(3.0, 2.0, 4.0));

    for (id, placement) in placements.iter().enumerate() {
        // Create all three GPU textures for this camera
        let render_target = images.add(create_render_target_image(config.resolution));
        let previous_frame = images.add(create_previous_frame_image(config.resolution));
        let difference_mask = images.add(create_difference_mask_image(config.resolution));

        let transform =
            Transform::from_translation(placement.position).looking_at(placement.look_at, Vec3::Y);

        // Calculate FOV from resolution (fixed horizontal FOV, derive vertical)
        let aspect = config.resolution.x as f32 / config.resolution.y as f32;
        let fov_x = std::f32::consts::FRAC_PI_2;
        let fov_y = 2.0 * ((fov_x / 2.0).tan() / aspect).atan();

        let focal_length_x = (config.resolution.x as f32 / 2.0) / (fov_x / 2.0).tan();
        let focal_length_y = (config.resolution.y as f32 / 2.0) / (fov_y / 2.0).tan();

        // Create camera intrinsics
        let intrinsics = CameraIntrinsics {
            focal_length: Vec2::new(focal_length_x, focal_length_y),
            principal_point: Vec2::new(
                config.resolution.x as f32 / 2.0,
                config.resolution.y as f32 / 2.0,
            ),
            resolution: config.resolution,
            fov: Fov {
                horizontal: fov_x,
                vertical: fov_y,
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

        // Spawn camera entity - NO texture readback (GPU pipeline handles everything)
        commands.spawn((
            Camera3d::default(),
            Camera {
                order: -(id as isize + 1), // Render before main camera (negative order)
                ..default()
            },
            Msaa::Sample8,
            // Render to our texture instead of the window
            RenderTarget::Image(render_target.clone().into()),
            Projection::Perspective(PerspectiveProjection {
                fov: fov_y,
                aspect_ratio: aspect,
                near: 0.1,
                far: 3000.0,
                ..default()
            }),
            transform,
            render_camera,
            // CRITICAL: Render cameras see layers 0 (scene) and 1 (targets), NOT layer 2 (debug)
            render_camera_layers(),
        ));

        // Spawn a visual marker for the camera (so we can see where it is)
        commands.spawn((
            Mesh3d(camera_mesh.clone()),
            MeshMaterial3d(materials.add(placement.color)),
            Transform::from_translation(placement.position),
        ));

        info!(
            "Spawned render camera {} at {:?} with GPU frame buffers ({}x{})",
            id, placement.position, config.resolution.x, config.resolution.y
        );
    }

    info!(
        "Spawned {} render cameras with GPU motion detection pipeline",
        placements.len()
    );
}

// =============================================================================
// Frame Management
// =============================================================================

/// System to mark cameras as having a previous frame after first render
///
/// After the first frame, we set `has_previous = true` so the GPU pipeline
/// knows it can start computing differences.
fn update_has_previous(mut cameras: Query<&mut RenderCamera>, mut frame_count: Local<u32>) {
    *frame_count += 1;

    // Skip first frame (no previous to compare against)
    if *frame_count < 2 {
        return;
    }

    // Mark all cameras as having a valid previous frame
    for mut camera in cameras.iter_mut() {
        if !camera.has_previous {
            camera.has_previous = true;
            debug!("RenderCamera {} now has previous frame", camera.camera_id);
        }
    }
}

/// Debug system to print GPU pipeline stats
fn debug_print_gpu_stats(
    metrics: Option<Res<crate::gpu_pipeline::GpuPipelineMetrics>>,
    ray_buffer: Option<Res<crate::gpu_pipeline::RayBufferResource>>,
    time: Res<Time>,
    mut last_print: Local<f32>,
) {
    let now = time.elapsed_secs();
    if now - *last_print < 1.0 {
        return;
    }
    *last_print = now;

    if let Some(metrics) = metrics
        && metrics.ray_count > 0
    {
        info!(
            "GPU Pipeline: {} rays (buffer {:.1}% full)",
            metrics.ray_count,
            metrics.buffer_utilization * 100.0
        );
    }

    if let Some(ray_buffer) = ray_buffer
        && !ray_buffer.pending_rays.is_empty()
    {
        info!(
            "Ray buffer: {} pending rays for CPU processing",
            ray_buffer.pending_rays.len()
        );
    }
}
