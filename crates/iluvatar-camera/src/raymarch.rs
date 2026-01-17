use crate::difference::DifferenceMask;
use glam::{UVec3, Vec3};
use iluvatar_core::{
    BoundingBox, CameraIntrinsics, CameraPose, GeoPosition, Ray, RaymarchConfig, VoxelContribution,
    ray_aabb_intersection, safe_div,
};
use std::collections::HashMap;

/// Camera ray generator and raymarcher using 3D-DDA algorithm
///
/// The DDA (Digital Differential Analyzer) algorithm efficiently traverses
/// only the voxels that a ray actually passes through, providing O(N) complexity
/// where N is the number of voxels crossed, rather than the naive step-based
/// approach which may miss or double-count voxels.
pub struct Raymarcher {
    intrinsics: CameraIntrinsics,
    config: RaymarchConfig,
    grid_bounds: BoundingBox,
    grid_origin: Vec3,
    grid_dims: UVec3,
    voxel_size: f32,
    world_origin: GeoPosition,
}

impl Raymarcher {
    pub fn new(
        intrinsics: CameraIntrinsics,
        config: RaymarchConfig,
        grid_bounds: BoundingBox,
        voxel_size: f32,
        world_origin: GeoPosition,
    ) -> Self {
        let size = grid_bounds.size();
        let grid_dims = UVec3::new(
            (size.x / voxel_size).ceil() as u32,
            (size.y / voxel_size).ceil() as u32,
            (size.z / voxel_size).ceil() as u32,
        );
        Self {
            intrinsics,
            config,
            grid_origin: grid_bounds.min,
            grid_bounds,
            grid_dims,
            voxel_size,
            world_origin,
        }
    }

    /// Convert pixel coordinates to normalized device coordinates (-1 to 1)
    fn pixel_to_ndc(&self, x: u32, y: u32) -> (f32, f32) {
        let nx = (x as f32 - self.intrinsics.principal_point.x)
            / (self.intrinsics.resolution.x as f32 / 2.0);
        let ny = (y as f32 - self.intrinsics.principal_point.y)
            / (self.intrinsics.resolution.y as f32 / 2.0);
        (nx, ny)
    }

    /// Generate a ray for a pixel with detected motion
    fn pixel_to_ray(&self, pose: &CameraPose, x: u32, y: u32, intensity: f32) -> Ray {
        let (nx, ny) = self.pixel_to_ndc(x, y);

        // Direction in camera space using pinhole camera model
        // Bevy cameras look down -Z, so we use -1.0 for the forward direction
        let dir_camera = Vec3::new(
            nx * (self.intrinsics.fov.horizontal / 2.0).tan(),
            -ny * (self.intrinsics.fov.vertical / 2.0).tan(),
            -1.0, // Bevy cameras look down -Z
        )
        .normalize();

        // Transform to Bevy world space
        let dir_bevy = pose.orientation * dir_camera;

        // Convert direction from Bevy (Y-up) to ENU (Z-up)
        // Bevy: X=right, Y=up, Z=back → ENU: X=East, Y=North, Z=Up
        let dir_enu = Vec3::new(dir_bevy.x, dir_bevy.z, dir_bevy.y);

        // Camera position in local grid coordinates (already in ENU via to_local_enu)
        let origin = pose.position.to_local_enu(&self.world_origin);

        Ray {
            origin,
            direction: dir_enu.normalize(),
            intensity,
        }
    }

    /// Check if voxel index is within grid bounds
    #[inline]
    fn in_bounds(&self, ix: i32, iy: i32, iz: i32) -> bool {
        ix >= 0
            && iy >= 0
            && iz >= 0
            && (ix as u32) < self.grid_dims.x
            && (iy as u32) < self.grid_dims.y
            && (iz as u32) < self.grid_dims.z
    }

    /// Raymarch from the difference mask, returning voxel contributions
    pub fn raymarch<S>(&self, pose: &CameraPose, mask: &DifferenceMask<S>) -> Vec<VoxelContribution>
    where
        S: AsRef<[u8]>,
    {
        let mut contributions: HashMap<(u32, u32, u32), f32> = HashMap::new();

        for (x, y, intensity) in mask.motion_pixels() {
            let ray = self.pixel_to_ray(pose, x, y, intensity as f32);
            self.march_ray_dda(&ray, &mut contributions);
        }

        contributions
            .into_iter()
            .map(|((x, y, z), intensity)| VoxelContribution {
                index: UVec3::new(x, y, z),
                intensity,
            })
            .collect()
    }

    /// 3D-DDA ray marching algorithm
    ///
    /// This algorithm efficiently walks through the voxel grid by computing
    /// which axis-aligned boundary will be crossed first at each step.
    /// Based on Amanatides & Woo's "A Fast Voxel Traversal Algorithm for Ray Tracing"
    fn march_ray_dda(&self, ray: &Ray, contributions: &mut HashMap<(u32, u32, u32), f32>) {
        // 1. Ray-box intersection to find entry/exit points
        let Some((t_min, t_max)) = ray_aabb_intersection(
            ray.origin,
            ray.direction,
            self.grid_bounds.min,
            self.grid_bounds.max,
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
        let local = start_world - self.grid_origin;

        let fx = local.x / self.voxel_size;
        let fy = local.y / self.voxel_size;
        let fz = local.z / self.voxel_size;

        // Clamp to valid range (handles edge cases at boundaries)
        let mut ix = (fx.floor() as i32).clamp(0, self.grid_dims.x as i32 - 1);
        let mut iy = (fy.floor() as i32).clamp(0, self.grid_dims.y as i32 - 1);
        let mut iz = (fz.floor() as i32).clamp(0, self.grid_dims.z as i32 - 1);

        // 3. Compute step direction for each axis
        let step_x = if ray.direction.x >= 0.0 { 1i32 } else { -1i32 };
        let step_y = if ray.direction.y >= 0.0 { 1i32 } else { -1i32 };
        let step_z = if ray.direction.z >= 0.0 { 1i32 } else { -1i32 };

        // 4. Compute t_delta: how far along ray to cross one voxel in each axis
        let t_delta_x = safe_div(self.voxel_size, ray.direction.x.abs());
        let t_delta_y = safe_div(self.voxel_size, ray.direction.y.abs());
        let t_delta_z = safe_div(self.voxel_size, ray.direction.z.abs());

        // 5. Compute t_max for each axis: t value at next voxel boundary
        let next_boundary_x =
            self.grid_origin.x + (if step_x > 0 { ix + 1 } else { ix } as f32) * self.voxel_size;
        let next_boundary_y =
            self.grid_origin.y + (if step_y > 0 { iy + 1 } else { iy } as f32) * self.voxel_size;
        let next_boundary_z =
            self.grid_origin.z + (if step_z > 0 { iz + 1 } else { iz } as f32) * self.voxel_size;

        let mut t_max_x = safe_div(next_boundary_x - ray.origin.x, ray.direction.x);
        let mut t_max_y = safe_div(next_boundary_y - ray.origin.y, ray.direction.y);
        let mut t_max_z = safe_div(next_boundary_z - ray.origin.z, ray.direction.z);

        let mut t_current = t_min;

        // 6. Walk through the grid using DDA
        while t_current <= t_max && self.in_bounds(ix, iy, iz) {
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

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{UVec2, Vec2};
    use iluvatar_core::Fov;

    fn test_intrinsics() -> CameraIntrinsics {
        CameraIntrinsics {
            focal_length: Vec2::new(500.0, 500.0),
            principal_point: Vec2::new(960.0, 540.0),
            resolution: UVec2::new(1920, 1080),
            fov: Fov {
                horizontal: std::f32::consts::FRAC_PI_2,
                vertical: std::f32::consts::FRAC_PI_4,
            },
        }
    }

    #[test]
    fn test_pixel_to_ndc() {
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
        );

        let (nx, ny) = raymarcher.pixel_to_ndc(960, 540);
        assert!((nx).abs() < 0.01);
        assert!((ny).abs() < 0.01);
    }

    #[test]
    fn test_dda_straight_ray() {
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(10.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
        );

        // Ray shooting straight through grid along X axis
        let ray = Ray {
            origin: Vec3::new(-5.0, 5.0, 5.0),
            direction: Vec3::new(1.0, 0.0, 0.0),
            intensity: 1.0,
        };

        let mut contributions = HashMap::new();
        raymarcher.march_ray_dda(&ray, &mut contributions);

        // Should hit 10 voxels along x=0..9, y=5, z=5
        assert_eq!(contributions.len(), 10);

        // Check all voxels are at y=5, z=5 (clamped to grid range)
        for ((x, y, z), _) in &contributions {
            assert!((*x as i32) >= 0 && (*x as i32) < 10);
            assert_eq!(*y, 5); // Middle of grid in Y
            assert_eq!(*z, 5); // Middle of grid in Z
        }
    }

    #[test]
    fn test_dda_diagonal_ray() {
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(10.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
        );

        // Ray shooting diagonally through grid
        let dir = Vec3::new(1.0, 1.0, 1.0).normalize();
        let ray = Ray {
            origin: Vec3::new(-1.0, -1.0, -1.0),
            direction: dir,
            intensity: 1.0,
        };

        let mut contributions = HashMap::new();
        raymarcher.march_ray_dda(&ray, &mut contributions);

        // Should hit voxels along the diagonal
        assert!(!contributions.is_empty());

        // Check that contributions form a diagonal pattern
        for ((x, y, z), _) in &contributions {
            // On a perfect diagonal, x, y, z should be close
            let diff_xy = (*x as i32 - *y as i32).abs();
            let diff_yz = (*y as i32 - *z as i32).abs();
            assert!(diff_xy <= 1 && diff_yz <= 1);
        }
    }

    #[test]
    fn test_dda_ray_misses_grid() {
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(10.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
        );

        // Ray that completely misses the grid
        let ray = Ray {
            origin: Vec3::new(-5.0, 20.0, 5.0), // Above the grid
            direction: Vec3::new(1.0, 0.0, 0.0),
            intensity: 1.0,
        };

        let mut contributions = HashMap::new();
        raymarcher.march_ray_dda(&ray, &mut contributions);

        // Should hit nothing
        assert!(contributions.is_empty());
    }
}
