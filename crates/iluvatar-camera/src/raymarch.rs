use crate::difference::DifferenceMask;
use glam::{UVec3, Vec3};
use iluvatar_core::{
    BoundingBox, CameraIntrinsics, CameraPose, Ray, RaymarchConfig, VoxelContribution,
};
use std::collections::HashMap;

/// Camera ray generator and raymarcher
pub struct Raymarcher {
    intrinsics: CameraIntrinsics,
    config: RaymarchConfig,
    grid_bounds: BoundingBox,
    grid_origin: Vec3,
    voxel_size: f32,
}

impl Raymarcher {
    pub fn new(
        intrinsics: CameraIntrinsics,
        config: RaymarchConfig,
        grid_bounds: BoundingBox,
        voxel_size: f32,
    ) -> Self {
        Self {
            intrinsics,
            config,
            grid_origin: grid_bounds.min,
            grid_bounds,
            voxel_size,
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

        // Direction in camera space
        let dir_camera = Vec3::new(
            nx * (self.intrinsics.fov.horizontal / 2.0).tan(),
            -ny * (self.intrinsics.fov.vertical / 2.0).tan(),
            1.0,
        )
        .normalize();

        // Transform to world space
        let dir_world = pose.orientation * dir_camera;

        // Camera position in local grid coordinates
        let origin = pose
            .position
            .to_local_enu(&iluvatar_core::GeoPosition::new(0.0, 0.0, 0.0));

        Ray {
            origin,
            direction: dir_world,
            intensity,
        }
    }

    /// Convert world position to voxel index
    fn world_to_voxel(&self, pos: Vec3) -> Option<UVec3> {
        let local = pos - self.grid_origin;
        if local.x < 0.0 || local.y < 0.0 || local.z < 0.0 {
            return None;
        }

        let vx = (local.x / self.voxel_size) as u32;
        let vy = (local.y / self.voxel_size) as u32;
        let vz = (local.z / self.voxel_size) as u32;

        let dims = self.grid_bounds.size() / self.voxel_size;
        if vx >= dims.x as u32 || vy >= dims.y as u32 || vz >= dims.z as u32 {
            return None;
        }

        Some(UVec3::new(vx, vy, vz))
    }

    /// Raymarch from the difference mask, returning voxel contributions
    pub fn raymarch(&self, pose: &CameraPose, mask: &DifferenceMask) -> Vec<VoxelContribution> {
        let mut contributions: HashMap<(u32, u32, u32), f32> = HashMap::new();

        for (x, y, intensity) in mask.motion_pixels() {
            let ray = self.pixel_to_ray(pose, x, y, intensity as f32);
            self.march_ray(&ray, &mut contributions);
        }

        contributions
            .into_iter()
            .map(|((x, y, z), intensity)| VoxelContribution {
                index: UVec3::new(x, y, z),
                intensity,
            })
            .collect()
    }

    fn march_ray(&self, ray: &Ray, contributions: &mut HashMap<(u32, u32, u32), f32>) {
        let mut t = 0.0;

        while t < self.config.max_distance {
            let point = ray.origin + ray.direction * t;

            if let Some(voxel) = self.world_to_voxel(point) {
                let attenuation = self.config.attenuation.compute(t);
                let contribution = ray.intensity * attenuation;

                contributions
                    .entry((voxel.x, voxel.y, voxel.z))
                    .and_modify(|v| *v += contribution)
                    .or_insert(contribution);
            }

            t += self.config.step_size;
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
        );

        let (nx, ny) = raymarcher.pixel_to_ndc(960, 540);
        assert!((nx).abs() < 0.01);
        assert!((ny).abs() < 0.01);
    }
}
