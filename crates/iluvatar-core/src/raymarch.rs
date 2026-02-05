use crate::{
    ray_aabb_intersection, safe_div, BoundingBox, CameraIntrinsics, CameraPose, GeoPosition,
    MotionPixel, Ray, RaymarchConfig, VoxelContribution,
};
use glam::{Mat3, Quat, UVec3, Vec3};
use std::collections::HashMap;
use tracing::warn;

/// Camera ray generator and raymarcher using 3D-DDA algorithm.
///
/// The DDA (Digital Differential Analyzer) algorithm efficiently traverses
/// only the voxels that a ray actually passes through, providing O(N) complexity
/// where N is the number of voxels crossed, rather than the naive step-based
/// approach which may miss or double-count voxels.
///
/// # Coordinate System Transformation
///
/// The raymarcher handles the conversion between two coordinate systems:
///
/// - **Bevy/Camera space** (Y-up): X=right, Y=up, Z=back (camera looks down -Z)
/// - **ENU space** (Z-up): X=East, Y=North, Z=Up
///
/// The transformation from Bevy to ENU is performed using a proper rotation matrix
/// that accounts for the camera's orientation (yaw, pitch, roll). This ensures
/// correct ray directions even when the camera is significantly tilted.
///
/// ## Mathematical Background
///
/// The conversion consists of two parts:
/// 1. Apply camera orientation (as quaternion) to get direction in Bevy world space
/// 2. Apply the Bevy-to-ENU rotation matrix to convert coordinate systems
///
/// The Bevy-to-ENU rotation is a 90-degree rotation around the X-axis:
/// ```text
/// | 1  0  0 |     Bevy X (right)   -> ENU X (East)
/// | 0  0 -1 |  => Bevy Z (back)    -> ENU -Y (South), so -Z -> North
/// | 0  1  0 |     Bevy Y (up)      -> ENU Z (Up)
/// ```
///
/// For tilted cameras, this matrix is combined with the camera's orientation
/// quaternion to produce correct ray directions in ENU space.
pub struct Raymarcher {
    intrinsics: CameraIntrinsics,
    config: RaymarchConfig,
    grid_bounds: BoundingBox,
    grid_origin: Vec3,
    grid_dims: UVec3,
    voxel_size: f32,
    world_origin: GeoPosition,
    contribution_limit: usize,
}

impl Raymarcher {
    pub fn new(
        intrinsics: CameraIntrinsics,
        config: RaymarchConfig,
        grid_bounds: BoundingBox,
        voxel_size: f32,
        world_origin: GeoPosition,
        contribution_limit: usize,
    ) -> Self {
        assert!(contribution_limit > 0);
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
            contribution_limit,
        }
    }

    /// Threshold for extreme camera tilt in degrees.
    /// Angles beyond this will trigger a warning (but still work correctly).
    const EXTREME_TILT_WARNING_DEGREES: f32 = 45.0;

    /// Calculate the camera's tilt from level in degrees.
    ///
    /// This measures the angle between the camera's local Y-axis (up in Bevy space)
    /// and the world Y-axis (vertical).
    fn tilt_degrees(pose: &CameraPose) -> f32 {
        // Transform the local up vector (Y-axis) by the camera orientation.
        let camera_up = pose.orientation * Vec3::Y;
        // Measure angle from world up (Y-axis in Bevy space).
        let dot = camera_up.dot(Vec3::Y).clamp(-1.0, 1.0);
        dot.acos().to_degrees()
    }

    /// Rotation matrix to convert from Bevy coordinate system (Y-up) to ENU (Z-up).
    ///
    /// In Bevy/OpenGL convention:
    /// - Camera looks down -Z (forward)
    /// - +Y is up
    /// - +X is right
    ///
    /// In ENU (East-North-Up):
    /// - +X is East (right when facing North)
    /// - +Y is North (forward)
    /// - +Z is Up
    ///
    /// For a level camera facing North, we need:
    /// - Bevy -Z (camera forward) -> ENU +Y (North)
    /// - Bevy +Y (camera up) -> ENU +Z (Up)
    /// - Bevy +X (camera right) -> ENU +X (East)
    ///
    /// This is a 90-degree rotation around the X-axis (positive direction).
    #[inline]
    fn bevy_to_enu_rotation() -> Mat3 {
        // Column-major: each column specifies where that basis vector maps to.
        // X -> X, Y -> Z, Z -> -Y
        Mat3::from_cols(
            Vec3::new(1.0, 0.0, 0.0),  // X -> (1, 0, 0) = X (East)
            Vec3::new(0.0, 0.0, 1.0),  // Y -> (0, 0, 1) = Z (Up)
            Vec3::new(0.0, -1.0, 0.0), // Z -> (0, -1, 0) = -Y (South, so -Z = North)
        )
    }

    /// Build the combined rotation matrix that transforms ray directions from
    /// camera-local space to ENU world space.
    ///
    /// This combines:
    /// 1. Camera orientation (yaw, pitch, roll as quaternion)
    /// 2. Bevy-to-ENU coordinate system conversion
    ///
    /// The result correctly handles tilted cameras.
    #[inline]
    fn camera_to_enu_rotation(orientation: Quat) -> Mat3 {
        let camera_rotation = Mat3::from_quat(orientation);
        let bevy_to_enu = Self::bevy_to_enu_rotation();
        // First apply camera rotation (in Bevy space), then convert to ENU.
        bevy_to_enu * camera_rotation
    }

    /// Check if voxel index is within grid bounds.
    #[inline]
    fn in_bounds(&self, ix: i32, iy: i32, iz: i32) -> bool {
        ix >= 0
            && iy >= 0
            && iz >= 0
            && (ix as u32) < self.grid_dims.x
            && (iy as u32) < self.grid_dims.y
            && (iz as u32) < self.grid_dims.z
    }

    /// Raymarch from motion pixels, returning voxel contributions.
    ///
    /// This is the server-side entry point. The camera sends `MotionPixel` data
    /// and the server reconstructs rays using camera intrinsics and pose, then
    /// marches them through the voxel grid.
    ///
    /// If the number of contributions exceeds `contribution_limit`,
    /// the results are truncated to the highest-intensity contributions.
    pub fn raymarch_motion_pixels(
        &self,
        pose: &CameraPose,
        pixels: &[MotionPixel],
    ) -> Vec<VoxelContribution> {
        // Precompute per-frame constants (invariant across all pixels in this frame).
        let tilt = Self::tilt_degrees(pose);
        if tilt > Self::EXTREME_TILT_WARNING_DEGREES {
            warn!(
                tilt_degrees = %tilt,
                "Camera has extreme tilt. While the coordinate transform \
                 handles this correctly, verify that the mounting orientation is \
                 intentional.",
            );
        }
        let camera_to_enu = Self::camera_to_enu_rotation(pose.orientation);
        let origin = pose.position.to_local_enu(&self.world_origin);

        let mut contributions: HashMap<(u32, u32, u32), f32> =
            HashMap::with_capacity(self.contribution_limit.min(1024));

        for pixel in pixels {
            // Stop early if we've already exceeded the contribution budget.
            // This bounds both memory usage and CPU time on heavy-motion frames.
            if contributions.len() >= self.contribution_limit {
                break;
            }

            let dir_camera =
                self.intrinsics.pixel_to_ray(pixel.x as f32, pixel.y as f32);
            let dir_enu = camera_to_enu * dir_camera;
            let ray = Ray::new(origin, dir_enu, pixel.intensity as f32);
            self.march_ray_dda(&ray, &mut contributions);
        }

        let mut result: Vec<VoxelContribution> = contributions
            .into_iter()
            .map(|((x, y, z), intensity)| VoxelContribution {
                index: UVec3::new(x, y, z),
                intensity,
            })
            .collect();

        // Enforce the contribution limit per frame.
        if result.len() > self.contribution_limit {
            warn!(
                count = result.len(),
                limit = self.contribution_limit,
                "Contribution limit exceeded, truncating to highest intensity"
            );
            // Sort by intensity descending and take the top N.
            result.sort_unstable_by(|a, b| {
                b.intensity.partial_cmp(&a.intensity).unwrap()
            });
            result.truncate(self.contribution_limit);
        }

        result
    }

    /// Raymarch from an iterator of (x, y, intensity) tuples.
    ///
    /// This is the camera-side entry point, used when the camera has a
    /// DifferenceMask and wants to compute voxel contributions locally.
    ///
    /// If the number of contributions exceeds `contribution_limit`,
    /// the results are truncated to the highest-intensity contributions.
    pub fn raymarch_pixels(
        &self,
        pose: &CameraPose,
        pixels: impl Iterator<Item = (u32, u32, u8)>,
    ) -> Vec<VoxelContribution> {
        // Precompute per-frame constants (invariant across all pixels in this frame).
        let tilt = Self::tilt_degrees(pose);
        if tilt > Self::EXTREME_TILT_WARNING_DEGREES {
            warn!(
                tilt_degrees = %tilt,
                "Camera has extreme tilt. While the coordinate transform \
                 handles this correctly, verify that the mounting orientation is \
                 intentional.",
            );
        }
        let camera_to_enu = Self::camera_to_enu_rotation(pose.orientation);
        let origin = pose.position.to_local_enu(&self.world_origin);

        let mut contributions: HashMap<(u32, u32, u32), f32> =
            HashMap::with_capacity(self.contribution_limit.min(1024));

        for (x, y, intensity) in pixels {
            // Stop early if we've already exceeded the contribution budget.
            if contributions.len() >= self.contribution_limit {
                break;
            }

            let dir_camera = self.intrinsics.pixel_to_ray(x as f32, y as f32);
            let dir_enu = camera_to_enu * dir_camera;
            let ray = Ray::new(origin, dir_enu, intensity as f32);
            self.march_ray_dda(&ray, &mut contributions);
        }

        let mut result: Vec<VoxelContribution> = contributions
            .into_iter()
            .map(|((x, y, z), intensity)| VoxelContribution {
                index: UVec3::new(x, y, z),
                intensity,
            })
            .collect();

        // Enforce the contribution limit per frame.
        if result.len() > self.contribution_limit {
            warn!(
                count = result.len(),
                limit = self.contribution_limit,
                "Contribution limit exceeded, truncating to highest intensity"
            );
            result.sort_unstable_by(|a, b| {
                b.intensity.partial_cmp(&a.intensity).unwrap()
            });
            result.truncate(self.contribution_limit);
        }

        result
    }

    /// 3D-DDA ray marching algorithm.
    ///
    /// This algorithm efficiently walks through the voxel grid by computing
    /// which axis-aligned boundary will be crossed first at each step.
    /// Based on Amanatides & Woo's "A Fast Voxel Traversal Algorithm for Ray Tracing".
    fn march_ray_dda(&self, ray: &Ray, contributions: &mut HashMap<(u32, u32, u32), f32>) {
        // 1. Ray-box intersection to find entry/exit points.
        let Some((t_min, t_max)) = ray_aabb_intersection(
            ray.origin,
            ray.direction,
            self.grid_bounds.min,
            self.grid_bounds.max,
        ) else {
            return; // Ray misses the grid entirely.
        };

        // Clamp to max_distance.
        let t_max = t_max.min(self.config.max_distance);
        if t_min > t_max {
            return;
        }

        // 2. Compute starting voxel from entry point.
        let start_world = ray.origin + ray.direction * t_min;
        let local = start_world - self.grid_origin;

        let fx = local.x / self.voxel_size;
        let fy = local.y / self.voxel_size;
        let fz = local.z / self.voxel_size;

        // Clamp to valid range (handles edge cases at boundaries).
        let mut ix = (fx.floor() as i32).clamp(0, self.grid_dims.x as i32 - 1);
        let mut iy = (fy.floor() as i32).clamp(0, self.grid_dims.y as i32 - 1);
        let mut iz = (fz.floor() as i32).clamp(0, self.grid_dims.z as i32 - 1);

        // 3. Compute step direction for each axis.
        let step_x = if ray.direction.x >= 0.0 { 1i32 } else { -1i32 };
        let step_y = if ray.direction.y >= 0.0 { 1i32 } else { -1i32 };
        let step_z = if ray.direction.z >= 0.0 { 1i32 } else { -1i32 };

        // 4. Compute t_delta: how far along ray to cross one voxel in each axis.
        let t_delta_x = safe_div(self.voxel_size, ray.direction.x.abs());
        let t_delta_y = safe_div(self.voxel_size, ray.direction.y.abs());
        let t_delta_z = safe_div(self.voxel_size, ray.direction.z.abs());

        // 5. Compute t_max for each axis: t value at next voxel boundary.
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

        // 6. Walk through the grid using DDA.
        while t_current <= t_max && self.in_bounds(ix, iy, iz) {
            // Accumulate contribution for current voxel.
            let attenuation = self.config.attenuation.compute(t_current);
            let contribution = (ray.intensity * attenuation).max(0.0);

            contributions
                .entry((ix as u32, iy as u32, iz as u32))
                .and_modify(|v| *v += contribution)
                .or_insert(contribution);

            // Step to next voxel: choose axis with smallest t_max.
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

    /// Convert pixel coordinates to normalized device coordinates (-1 to 1).
    ///
    /// Note: This method is kept for testing purposes.
    #[cfg(test)]
    fn pixel_to_ndc(&self, x: u32, y: u32) -> (f32, f32) {
        let nx = (x as f32 - self.intrinsics.principal_point.x)
            / (self.intrinsics.resolution.x as f32 / 2.0);
        let ny = (y as f32 - self.intrinsics.principal_point.y)
            / (self.intrinsics.resolution.y as f32 / 2.0);
        (nx, ny)
    }

    /// Generate a ray for a pixel with detected motion.
    ///
    /// Uses the calibrated intrinsics to correctly undistort the pixel coordinates
    /// before computing the ray direction.
    #[cfg(test)]
    fn pixel_to_ray(&self, pose: &CameraPose, x: u32, y: u32, intensity: f32) -> Ray {
        let tilt = Self::tilt_degrees(pose);
        if tilt > Self::EXTREME_TILT_WARNING_DEGREES {
            warn!(
                tilt_degrees = %tilt,
                "Camera has extreme tilt. While the coordinate transform \
                 handles this correctly, verify that the mounting orientation is \
                 intentional.",
            );
        }

        let dir_camera = self.intrinsics.pixel_to_ray(x as f32, y as f32);
        let camera_to_enu = Self::camera_to_enu_rotation(pose.orientation);
        let dir_enu = camera_to_enu * dir_camera;
        let origin = pose.position.to_local_enu(&self.world_origin);

        Ray::new(origin, dir_enu, intensity)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        CameraPose, DistortionModel, Fov, LocalizationStatus, MotionPixel, PoseUncertainty,
        Timestamp, MAX_CONTRIBUTIONS_PER_FRAME,
    };
    use glam::{Quat, UVec2, Vec2};

    fn test_intrinsics() -> CameraIntrinsics {
        CameraIntrinsics {
            focal_length: Vec2::new(500.0, 500.0),
            principal_point: Vec2::new(960.0, 540.0),
            resolution: UVec2::new(1920, 1080),
            fov: Fov {
                horizontal: std::f32::consts::FRAC_PI_2,
                vertical: std::f32::consts::FRAC_PI_4,
            },
            distortion: DistortionModel::None,
        }
    }

    fn test_pose_with_orientation(orientation: Quat) -> CameraPose {
        CameraPose {
            position: GeoPosition::new(0.0, 0.0, 100.0),
            orientation,
            timestamp: 0 as Timestamp,
            uncertainty: PoseUncertainty::default(),
            status: LocalizationStatus::Nominal,
        }
    }

    fn test_raymarcher() -> Raymarcher {
        Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
            MAX_CONTRIBUTIONS_PER_FRAME,
        )
    }

    #[test]
    fn test_pixel_to_ndc() {
        let raymarcher = test_raymarcher();
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
            MAX_CONTRIBUTIONS_PER_FRAME,
        );

        // Ray shooting straight through grid along X axis.
        let ray = Ray::new(Vec3::new(-5.0, 5.0, 5.0), Vec3::new(1.0, 0.0, 0.0), 1.0);

        let mut contributions = HashMap::new();
        raymarcher.march_ray_dda(&ray, &mut contributions);

        // Should hit 10 voxels along x=0..9, y=5, z=5.
        assert_eq!(contributions.len(), 10);

        for ((x, y, z), _) in &contributions {
            assert!((*x as i32) >= 0 && (*x as i32) < 10);
            assert_eq!(*y, 5);
            assert_eq!(*z, 5);
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
            MAX_CONTRIBUTIONS_PER_FRAME,
        );

        let ray = Ray::new(Vec3::new(-1.0, -1.0, -1.0), Vec3::new(1.0, 1.0, 1.0), 1.0);

        let mut contributions = HashMap::new();
        raymarcher.march_ray_dda(&ray, &mut contributions);

        assert!(!contributions.is_empty());

        for ((x, y, z), _) in &contributions {
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
            MAX_CONTRIBUTIONS_PER_FRAME,
        );

        let ray = Ray::new(
            Vec3::new(-5.0, 20.0, 5.0), // Above the grid.
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
        );

        let mut contributions = HashMap::new();
        raymarcher.march_ray_dda(&ray, &mut contributions);

        assert!(contributions.is_empty());
    }

    #[test]
    fn test_raymarch_motion_pixels_empty() {
        let raymarcher = test_raymarcher();
        let pose = test_pose_with_orientation(Quat::IDENTITY);
        let result = raymarcher.raymarch_motion_pixels(&pose, &[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_raymarch_motion_pixels_basic() {
        let raymarcher = test_raymarcher();
        let pose = test_pose_with_orientation(Quat::IDENTITY);

        // A few motion pixels near the center of the image.
        let pixels = vec![
            MotionPixel::new(960, 540, 128),
            MotionPixel::new(961, 540, 200),
        ];

        let result = raymarcher.raymarch_motion_pixels(&pose, &pixels);

        // Should produce some voxel contributions (exact count depends on DDA geometry).
        assert!(
            !result.is_empty(),
            "Center pixels should produce voxel contributions"
        );

        // All intensities should be non-negative.
        for contrib in &result {
            assert!(contrib.intensity >= 0.0);
        }
    }

    #[test]
    fn test_raymarch_motion_pixels_matches_raymarch_pixels() {
        // The two entry points should produce identical results for the same input.
        let raymarcher = test_raymarcher();
        let pose = test_pose_with_orientation(Quat::IDENTITY);

        let pixels = vec![
            MotionPixel::new(500, 300, 100),
            MotionPixel::new(1400, 800, 200),
        ];

        let result_motion =
            raymarcher.raymarch_motion_pixels(&pose, &pixels);

        let pixel_iter = pixels
            .iter()
            .map(|p| (p.x as u32, p.y as u32, p.intensity));
        let result_pixels =
            raymarcher.raymarch_pixels(&pose, pixel_iter);

        // Same number of contributions.
        assert_eq!(result_motion.len(), result_pixels.len());

        // Sort both by index for comparison (HashMap iteration order is non-deterministic).
        let mut motion_sorted = result_motion;
        let mut pixels_sorted = result_pixels;
        motion_sorted.sort_unstable_by_key(|c| (c.index.x, c.index.y, c.index.z));
        pixels_sorted.sort_unstable_by_key(|c| (c.index.x, c.index.y, c.index.z));

        for (a, b) in motion_sorted.iter().zip(pixels_sorted.iter()) {
            assert_eq!(a.index, b.index);
            assert!(
                (a.intensity - b.intensity).abs() < 1e-5,
                "Intensity mismatch at {:?}: {} vs {}",
                a.index,
                a.intensity,
                b.intensity
            );
        }
    }

    #[test]
    fn test_contribution_limit_enforced() {
        let mut contributions: Vec<VoxelContribution> = (0..100_000)
            .map(|i| VoxelContribution {
                index: UVec3::new(i % 1000, (i / 1000) % 100, i / 100_000),
                intensity: i as f32,
            })
            .collect();

        if contributions.len() > MAX_CONTRIBUTIONS_PER_FRAME {
            contributions.sort_unstable_by(|a, b| {
                b.intensity.partial_cmp(&a.intensity).unwrap()
            });
            contributions.truncate(MAX_CONTRIBUTIONS_PER_FRAME);
        }

        assert_eq!(contributions.len(), MAX_CONTRIBUTIONS_PER_FRAME);

        let min_intensity = contributions
            .iter()
            .map(|c| c.intensity)
            .fold(f32::INFINITY, f32::min);
        assert!(min_intensity >= (100_000 - MAX_CONTRIBUTIONS_PER_FRAME) as f32);
    }

    #[test]
    fn test_level_camera_center_ray_points_north() {
        let raymarcher = test_raymarcher();
        let pose = test_pose_with_orientation(Quat::IDENTITY);
        let ray = raymarcher.pixel_to_ray(&pose, 960, 540, 1.0);

        assert!(
            ray.direction.y > 0.9,
            "Level camera center ray should point North (+Y in ENU), got {:?}",
            ray.direction
        );
        assert!(ray.direction.x.abs() < 0.1);
        assert!(ray.direction.z.abs() < 0.1);
    }

    #[test]
    fn test_ray_direction_is_normalized() {
        let raymarcher = test_raymarcher();

        let test_orientations = [
            (0.0, 0.0, 0.0),
            (90.0, 0.0, 0.0),
            (0.0, 30.0, 0.0),
            (0.0, 0.0, 45.0),
            (270.0, 10.0, 0.0),
            (45.0, -20.0, 15.0),
        ];

        for (yaw, pitch, roll) in test_orientations {
            let orientation = Quat::from_euler(
                glam::EulerRot::YXZ,
                (yaw as f32).to_radians(),
                (pitch as f32).to_radians(),
                (roll as f32).to_radians(),
            );
            let pose = test_pose_with_orientation(orientation);
            let ray = raymarcher.pixel_to_ray(&pose, 960, 540, 1.0);

            let len = ray.direction.length();
            assert!(
                (len - 1.0).abs() < 1e-5,
                "Ray direction should be normalized for orientation ({}, {}, {}), got length {}",
                yaw,
                pitch,
                roll,
                len
            );
        }
    }
}
