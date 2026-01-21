//! Voxel grid and ray marching using real iluvatar modules
//!
//! This module uses the real `SparseVoxelGrid` from iluvatar-server and the
//! 3D-DDA raymarching algorithm from iluvatar-camera. No more toy implementations!
//!
//! Key integration points:
//! - `SparseVoxelGrid`: DashMap-backed concurrent voxel storage with camera bitmasks
//! - `Ray` and `VoxelContribution`: Core types from iluvatar-core
//! - DDA algorithm: Efficient voxel traversal (O(voxels traversed), not O(ray length))

use bevy::prelude::*;
use glam::UVec3;
use std::collections::HashMap;
use std::sync::Arc;

use iluvatar_core::{
    AttenuationConfig, BoundingBox, GeoPosition, Ray, RaymarchConfig, VoxelContribution,
};
use iluvatar_server::grid::SparseVoxelGrid;
use iluvatar_server::time::Clock;

use crate::camera::CaptureCamera;
use crate::render_camera::RenderCamera;
use crate::targets::Target;

pub struct VoxelsPlugin;

impl Plugin for VoxelsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<VoxelGridResource>()
            .init_resource::<SimulatorConfig>()
            .add_systems(
                Update,
                (
                    project_and_raymarch,
                    decay_voxels,
                    visualize_voxels,
                    print_stats,
                )
                    .chain(),
            );
    }
}

/// Voxel plugin variant that doesn't include the project_and_raymarch system.
/// Used when render cameras provide motion-based raymarching instead.
pub struct VoxelsPluginWithoutRaymarch;

impl Plugin for VoxelsPluginWithoutRaymarch {
    fn build(&self, app: &mut App) {
        app.init_resource::<VoxelGridResource>()
            .init_resource::<SimulatorConfig>()
            .add_systems(
                Update,
                (
                    decay_voxels,
                    visualize_voxels_render_mode,
                    print_stats_render_mode,
                )
                    .chain(),
            );
    }
}

/// Configuration for the simulator's voxel grid and raymarching
#[derive(Resource, Clone)]
pub struct SimulatorConfig {
    /// Size of each voxel in world units (meters)
    pub voxel_size: f32,
    /// Grid dimensions in voxels (x, y, z)
    pub grid_dimensions: UVec3,
    /// Grid origin offset in world space
    pub grid_origin: Vec3,
    /// Decay rate for voxel intensity (per second)
    pub decay_rate: f32,
    /// Maximum ray distance
    pub max_ray_distance: f32,
    /// Ray intensity for detected motion
    pub ray_intensity: f32,
    /// Intensity threshold for visualization
    pub visualization_threshold: f32,
}

impl Default for SimulatorConfig {
    fn default() -> Self {
        Self {
            voxel_size: 2.0,
            grid_dimensions: UVec3::new(500, 250, 500), // 1000m x 500m x 1000m at 2m voxels
            grid_origin: Vec3::new(-500.0, 0.0, -500.0),
            decay_rate: 5.0,
            max_ray_distance: 3000.0,
            ray_intensity: 1.0,
            visualization_threshold: 1.0,
        }
    }
}

/// Wrapper around `SparseVoxelGrid` for use as a Bevy Resource
#[derive(Resource)]
pub struct VoxelGridResource {
    pub grid: Arc<SparseVoxelGrid>,
    pub config: RaymarchConfig,
    pub bounds: BoundingBox,
    /// Stats for debugging
    pub rays_cast: u32,
    pub voxels_contributed: u32,
}

impl FromWorld for VoxelGridResource {
    fn from_world(world: &mut World) -> Self {
        let sim_config = world
            .get_resource::<SimulatorConfig>()
            .cloned()
            .unwrap_or_default();

        // Compute grid bounds from origin and dimensions
        let size = Vec3::new(
            sim_config.grid_dimensions.x as f32 * sim_config.voxel_size,
            sim_config.grid_dimensions.y as f32 * sim_config.voxel_size,
            sim_config.grid_dimensions.z as f32 * sim_config.voxel_size,
        );
        let bounds = BoundingBox::new(sim_config.grid_origin, sim_config.grid_origin + size);

        // Create the real SparseVoxelGrid from iluvatar-server
        // We use a trivial GeoPosition origin since we're working in local coordinates
        let clock = Clock::new();
        let grid = Arc::new(SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            sim_config.grid_dimensions,
            sim_config.voxel_size,
            sim_config.decay_rate,
            clock,
        ));

        // Configure raymarching with linear attenuation
        let config = RaymarchConfig {
            max_distance: sim_config.max_ray_distance,
            step_size: 0.5, // Unused by DDA but kept for API compat
            attenuation: AttenuationConfig::None,
            // {
            //     max_distance: sim_config.max_ray_distance,
            // },
        };

        Self {
            grid,
            config,
            bounds,
            rays_cast: 0,
            voxels_contributed: 0,
        }
    }
}

/// Simulator-specific raymarcher that uses the DDA algorithm
///
/// This is a simplified version of `iluvatar_camera::Raymarcher` that works
/// directly with Bevy world coordinates instead of requiring geographic transforms.
/// It uses the same core 3D-DDA algorithm for mathematically correct voxel traversal.
pub struct SimulatorRaymarcher<'a> {
    config: &'a RaymarchConfig,
    bounds: &'a BoundingBox,
    voxel_size: f32,
}

impl<'a> SimulatorRaymarcher<'a> {
    pub fn new(config: &'a RaymarchConfig, bounds: &'a BoundingBox, voxel_size: f32) -> Self {
        Self {
            config,
            bounds,
            voxel_size,
        }
    }

    /// Check if voxel index is within grid bounds
    #[inline]
    fn in_bounds(&self, ix: i32, iy: i32, iz: i32, dims: UVec3) -> bool {
        ix >= 0
            && iy >= 0
            && iz >= 0
            && (ix as u32) < dims.x
            && (iy as u32) < dims.y
            && (iz as u32) < dims.z
    }

    /// 3D-DDA ray marching algorithm (Amanatides & Woo)
    ///
    /// This is the heart of efficient voxel traversal. Instead of stepping
    /// at fixed intervals along the ray, we compute exactly which voxel
    /// boundary will be crossed next in each axis.
    ///
    /// The algorithm maintains t_max values for each axis - the t parameter
    /// at which the ray crosses the next voxel boundary in that axis.
    /// We always step along the axis with the smallest t_max.
    ///
    /// This gives us O(k) complexity where k is the number of voxels traversed,
    /// compared to O(d/s) for naive stepping where d is distance and s is step size.
    pub fn march_ray(
        &self,
        ray: &Ray,
        grid_dims: UVec3,
        contributions: &mut HashMap<(u32, u32, u32), f32>,
    ) {
        // 1. Ray-AABB intersection to find entry/exit points
        let Some((t_min, t_max)) = iluvatar_core::ray_aabb_intersection(
            ray.origin,
            ray.direction,
            self.bounds.min,
            self.bounds.max,
        ) else {
            return; // Ray misses the grid entirely
        };

        // Clamp to max_distance
        let t_max = t_max.min(self.config.max_distance);
        if t_min > t_max {
            return;
        }

        // 2. Compute starting voxel from entry point
        let start_world = ray.origin + ray.direction * t_min;
        let local = start_world - self.bounds.min;

        let fx = local.x / self.voxel_size;
        let fy = local.y / self.voxel_size;
        let fz = local.z / self.voxel_size;

        // Clamp to valid range (handles edge cases at boundaries)
        let mut ix = (fx.floor() as i32).clamp(0, grid_dims.x as i32 - 1);
        let mut iy = (fy.floor() as i32).clamp(0, grid_dims.y as i32 - 1);
        let mut iz = (fz.floor() as i32).clamp(0, grid_dims.z as i32 - 1);

        // 3. Compute step direction for each axis
        let step_x = if ray.direction.x >= 0.0 { 1i32 } else { -1i32 };
        let step_y = if ray.direction.y >= 0.0 { 1i32 } else { -1i32 };
        let step_z = if ray.direction.z >= 0.0 { 1i32 } else { -1i32 };

        // 4. Compute t_delta: how far along ray to cross one voxel in each axis
        let t_delta_x = iluvatar_core::safe_div(self.voxel_size, ray.direction.x.abs());
        let t_delta_y = iluvatar_core::safe_div(self.voxel_size, ray.direction.y.abs());
        let t_delta_z = iluvatar_core::safe_div(self.voxel_size, ray.direction.z.abs());

        // 5. Compute t_max for each axis: t value at next voxel boundary
        let grid_origin = self.bounds.min;
        let next_boundary_x =
            grid_origin.x + (if step_x > 0 { ix + 1 } else { ix } as f32) * self.voxel_size;
        let next_boundary_y =
            grid_origin.y + (if step_y > 0 { iy + 1 } else { iy } as f32) * self.voxel_size;
        let next_boundary_z =
            grid_origin.z + (if step_z > 0 { iz + 1 } else { iz } as f32) * self.voxel_size;

        let mut t_max_x = iluvatar_core::safe_div(next_boundary_x - ray.origin.x, ray.direction.x);
        let mut t_max_y = iluvatar_core::safe_div(next_boundary_y - ray.origin.y, ray.direction.y);
        let mut t_max_z = iluvatar_core::safe_div(next_boundary_z - ray.origin.z, ray.direction.z);

        let mut t_current = t_min;

        // 6. Walk through the grid using DDA
        while t_current <= t_max && self.in_bounds(ix, iy, iz, grid_dims) {
            // Accumulate contribution for current voxel
            let attenuation = self.config.attenuation.compute(t_current);
            let contribution = (ray.intensity * attenuation).max(0.0);

            contributions
                .entry((ix as u32, iy as u32, iz as u32))
                .and_modify(|v| *v += contribution)
                .or_insert(contribution);

            // Step to next voxel: choose axis with smallest t_max
            if t_max_x < t_max_y && t_max_x < t_max_z {
                ix += step_x;
                t_current = t_max_x;
                t_max_x += t_delta_x;
            } else if t_max_y < t_max_z {
                iy += step_y;
                t_current = t_max_y;
                t_max_y += t_delta_y;
            } else {
                iz += step_z;
                t_current = t_max_z;
                t_max_z += t_delta_z;
            }
        }
    }
}

/// Project targets and cast rays using the real DDA algorithm
pub fn project_and_raymarch(
    mut grid_res: ResMut<VoxelGridResource>,
    sim_config: Res<SimulatorConfig>,
    tracking_config: Option<Res<crate::tracking::TrackingConfig>>,
    cameras: Query<(Entity, &Transform, &CaptureCamera)>,
    targets: Query<&Transform, With<Target>>,
) {
    // Copy config values we need for raymarching to avoid borrow conflicts
    let raymarch_config = grid_res.config.clone();
    let bounds = grid_res.bounds;
    let voxel_size = sim_config.voxel_size;
    let grid_dims = sim_config.grid_dimensions;
    let ray_intensity = sim_config.ray_intensity;

    // Clear the grid if configured to do so (frame-by-frame mode)
    // This eliminates ghost voxels from previous frames - fresh slate!
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

    // Collect contributions from all cameras
    for (_entity, cam_transform, camera) in cameras.iter() {
        // Use the camera's assigned ID (already limited to 0-63)
        let camera_id = u64::from(camera.camera_id % 64);

        let raymarcher = SimulatorRaymarcher::new(&raymarch_config, &bounds, voxel_size);

        let mut contributions: HashMap<(u32, u32, u32), f32> = HashMap::new();
        let mut rays_this_camera = 0u32;

        for target_transform in targets.iter() {
            let target_pos = target_transform.translation;

            // Check if target is visible to this camera
            if let Some((u, v)) = camera.project_point(cam_transform, target_pos) {
                // Target is visible! Create a ray toward it
                let ray_dir = camera.ray_direction(cam_transform, u, v);
                let ray_origin = cam_transform.translation;

                // Create a proper Ray struct from iluvatar-core
                let ray = Ray::new(ray_origin, ray_dir, ray_intensity);

                // March the ray using 3D-DDA
                raymarcher.march_ray(&ray, grid_dims, &mut contributions);
                rays_this_camera += 1;
            }
        }

        // Convert to VoxelContribution and add to grid
        let voxel_contributions: Vec<VoxelContribution> = contributions
            .into_iter()
            .map(|((x, y, z), intensity)| VoxelContribution {
                index: UVec3::new(x, y, z),
                intensity,
            })
            .collect();

        grid_res.rays_cast += rays_this_camera;
        grid_res.voxels_contributed += voxel_contributions.len() as u32;

        // Add contributions to the real SparseVoxelGrid with camera tracking
        grid_res
            .grid
            .add_camera_contributions(camera_id, &voxel_contributions);
    }
}

/// Decay voxels over time using the real grid's decay mechanism
pub fn decay_voxels(grid_res: Res<VoxelGridResource>) {
    // The real SparseVoxelGrid handles time-based decay internally
    grid_res.grid.apply_decay();
}

/// Visualize voxels with gizmos
fn visualize_voxels(
    mut gizmos: Gizmos,
    grid_res: Res<VoxelGridResource>,
    sim_config: Res<SimulatorConfig>,
    vis_config: Option<Res<crate::debug_ui::VisualizationConfig>>,
    cameras: Query<&Transform, With<CaptureCamera>>,
    targets: Query<&Transform, With<Target>>,
) {
    let show_voxels = vis_config.as_ref().is_none_or(|c| c.show_voxels);
    let show_cameras = vis_config.as_ref().is_none_or(|c| c.show_cameras);
    let show_targets = vis_config.as_ref().is_none_or(|c| c.show_targets);
    let show_grid_bounds = vis_config.as_ref().is_none_or(|c| c.show_grid_bounds);

    // Draw grid bounds
    if show_grid_bounds {
        let center = (grid_res.bounds.min + grid_res.bounds.max) * 0.5;
        let size = grid_res.bounds.max - grid_res.bounds.min;
        gizmos.cube(
            Transform::from_translation(center).with_scale(size),
            Color::WHITE,
        );
    }

    // Get voxels for visualization from the real grid
    if show_voxels {
        let max_intensity = grid_res.grid.max_intensity().max(1.0);

        // Use the grid's visualization iterator
        let voxels = grid_res.grid.iter_voxels_for_visualization(
            sim_config.visualization_threshold,
            10000, // Max voxels to render
        );

        for (world_pos, intensity, camera_count) in voxels {
            // Adjust position for grid origin (real grid uses 0-based coordinates)
            let pos = world_pos + sim_config.grid_origin;
            let t = (intensity / max_intensity).clamp(0.0, 1.0);

            // Color based on intensity AND camera count
            // Multi-camera contributions get brighter/warmer colors
            let color = if camera_count >= 2 {
                // Hot colors for multi-camera consensus (the good stuff!)
                Color::srgb(1.0, 1.0 - t * 0.5, 0.0)
            } else if t < 0.5 {
                // Cool colors for single-camera, low intensity
                Color::srgb(t * 2.0, t * 2.0, 1.0 - t * 2.0)
            } else {
                // Warm colors for single-camera, high intensity
                Color::srgb(1.0, 2.0 - t * 2.0, 0.0)
            };

            gizmos.cube(
                Transform::from_translation(pos)
                    .with_scale(Vec3::splat(sim_config.voxel_size * 0.8)),
                color,
            );
        }
    }

    // Draw camera frustum outline
    if show_cameras {
        for cam_transform in cameras.iter() {
            let forward = cam_transform.forward() * 50.0;
            let pos = cam_transform.translation;

            gizmos.line(pos, pos + forward, Color::srgb(0.0, 1.0, 0.0));
            gizmos.sphere(
                Isometry3d::from_translation(pos),
                2.0,
                Color::srgb(0.5, 0.2, 0.8),
            );
        }
    }

    // Draw target positions
    if show_targets {
        for target_transform in targets.iter() {
            gizmos.sphere(
                Isometry3d::from_translation(target_transform.translation),
                4.0,
                Color::srgb(0.0, 1.0, 0.0),
            );
        }
    }
}

/// Print stats periodically
fn print_stats(
    time: Res<Time>,
    grid_res: Res<VoxelGridResource>,
    cameras: Query<Entity, With<CaptureCamera>>,
    targets: Query<(&Transform, &Target, &crate::targets::TargetPath)>,
    mut last_print: Local<f32>,
) {
    let now = time.elapsed_secs();
    if now - *last_print > 1.0 {
        *last_print = now;

        let camera_count = cameras.iter().count();

        println!("\n=== t={:.1}s ===", now);
        println!(
            "Active voxels: {} | Cameras: {}",
            grid_res.grid.active_count(),
            camera_count
        );
        println!(
            "Rays cast: {}, voxels contributed: {}",
            grid_res.rays_cast, grid_res.voxels_contributed
        );

        for (transform, target, path) in targets.iter() {
            let pos = transform.translation;
            let vel = path.current_velocity(now);
            println!(
                "Target {}: pos=({:.1}, {:.1}, {:.1}) vel=({:.1}, {:.1}, {:.1}) |v|={:.1}m/s",
                target.id,
                pos.x,
                pos.y,
                pos.z,
                vel.x,
                vel.y,
                vel.z,
                vel.length()
            );
        }
    }
}

/// Visualize voxels with gizmos (render mode - uses RenderCamera)
fn visualize_voxels_render_mode(
    mut gizmos: Gizmos,
    grid_res: Res<VoxelGridResource>,
    sim_config: Res<SimulatorConfig>,
    vis_config: Option<Res<crate::debug_ui::VisualizationConfig>>,
    cameras: Query<&Transform, With<RenderCamera>>,
    targets: Query<&Transform, With<Target>>,
) {
    let show_voxels = vis_config.as_ref().is_none_or(|c| c.show_voxels);
    let show_cameras = vis_config.as_ref().is_none_or(|c| c.show_cameras);
    let show_targets = vis_config.as_ref().is_none_or(|c| c.show_targets);
    let show_grid_bounds = vis_config.as_ref().is_none_or(|c| c.show_grid_bounds);

    // Draw grid bounds
    if show_grid_bounds {
        let center = (grid_res.bounds.min + grid_res.bounds.max) * 0.5;
        let size = grid_res.bounds.max - grid_res.bounds.min;
        gizmos.cube(
            Transform::from_translation(center).with_scale(size),
            Color::WHITE,
        );
    }

    // Get voxels for visualization from the real grid
    if show_voxels {
        let max_intensity = grid_res.grid.max_intensity().max(1.0);

        // Use the grid's visualization iterator
        let voxels = grid_res.grid.iter_voxels_for_visualization(
            sim_config.visualization_threshold,
            10000, // Max voxels to render
        );

        for (world_pos, intensity, camera_count) in voxels {
            // Adjust position for grid origin (real grid uses 0-based coordinates)
            let pos = world_pos + sim_config.grid_origin;
            let t = (intensity / max_intensity).clamp(0.0, 1.0);

            // Color based on intensity AND camera count
            // Multi-camera contributions get brighter/warmer colors
            let color = if camera_count >= 2 {
                // Hot colors for multi-camera consensus (the good stuff!)
                Color::srgb(1.0, 1.0 - t * 0.5, 0.0)
            } else if t < 0.5 {
                // Cool colors for single-camera, low intensity
                Color::srgb(t * 2.0, t * 2.0, 1.0 - t * 2.0)
            } else {
                // Warm colors for single-camera, high intensity
                Color::srgb(1.0, 2.0 - t * 2.0, 0.0)
            };

            gizmos.cube(
                Transform::from_translation(pos)
                    .with_scale(Vec3::splat(sim_config.voxel_size * 0.8)),
                color,
            );
        }
    }

    // Draw camera frustum outline
    if show_cameras {
        for cam_transform in cameras.iter() {
            let forward = cam_transform.forward() * 50.0;
            let pos = cam_transform.translation;

            gizmos.line(pos, pos + forward, Color::srgb(0.0, 1.0, 0.0));
            gizmos.sphere(
                Isometry3d::from_translation(pos),
                2.0,
                Color::srgb(0.5, 0.2, 0.8),
            );
        }
    }

    // Draw target positions
    if show_targets {
        for target_transform in targets.iter() {
            gizmos.sphere(
                Isometry3d::from_translation(target_transform.translation),
                4.0,
                Color::srgb(0.0, 1.0, 0.0),
            );
        }
    }
}

/// Print stats periodically (render mode - uses RenderCamera)
fn print_stats_render_mode(
    time: Res<Time>,
    grid_res: Res<VoxelGridResource>,
    cameras: Query<&RenderCamera>,
    gpu_metrics: Option<Res<crate::gpu_pipeline::GpuPipelineMetrics>>,
    targets: Query<(&Transform, &Target, &crate::targets::TargetPath)>,
    mut last_print: Local<f32>,
) {
    let now = time.elapsed_secs();
    if now - *last_print > 1.0 {
        *last_print = now;

        let camera_count = cameras.iter().count();
        let total_rays = gpu_metrics.as_ref().map(|m| m.ray_count).unwrap_or(0);

        println!("\n=== t={:.1}s ===", now);
        println!(
            "Active voxels: {} | Cameras: {} | GPU rays: {}",
            grid_res.grid.active_count(),
            camera_count,
            total_rays
        );
        println!(
            "Rays cast: {}, voxels contributed: {}",
            grid_res.rays_cast, grid_res.voxels_contributed
        );

        for (transform, target, path) in targets.iter() {
            let pos = transform.translation;
            let vel = path.current_velocity(now);
            println!(
                "Target {}: pos=({:.1}, {:.1}, {:.1}) vel=({:.1}, {:.1}, {:.1}) |v|={:.1}m/s",
                target.id,
                pos.x,
                pos.y,
                pos.z,
                vel.x,
                vel.y,
                vel.z,
                vel.length()
            );
        }
    }
}
