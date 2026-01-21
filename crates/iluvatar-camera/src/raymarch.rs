use crate::difference::DifferenceMask;
use glam::{Mat3, Quat, UVec3, Vec3};
use iluvatar_core::{
    ray_aabb_intersection, safe_div, BoundingBox, CameraIntrinsics, CameraPose, GeoPosition, Ray,
    RaymarchConfig, VoxelContribution, MAX_CONTRIBUTIONS_PER_FRAME,
};
use std::collections::HashMap;
use tracing::warn;

/// Camera ray generator and raymarcher using 3D-DDA algorithm
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
/// The Bevy-to-ENU rotation is a 90° rotation around the X-axis:
/// ```text
/// | 1  0  0 |     Bevy X (right)   → ENU X (East)
/// | 0  0 -1 |  => Bevy Z (back)    → ENU -Y (South), so -Z → North
/// | 0  1  0 |     Bevy Y (up)      → ENU Z (Up)
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

    /// Threshold for extreme camera tilt in degrees.
    /// Angles beyond this will trigger a warning (but still work correctly).
    const EXTREME_TILT_WARNING_DEGREES: f32 = 45.0;

    /// Calculate the camera's tilt from level in degrees.
    ///
    /// This measures the angle between the camera's local Y-axis (up in Bevy space)
    /// and the world Y-axis (vertical).
    fn tilt_degrees(pose: &CameraPose) -> f32 {
        // Transform the local up vector (Y-axis) by the camera orientation
        let camera_up = pose.orientation * Vec3::Y;
        // Measure angle from world up (Y-axis in Bevy space)
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
    /// - Bevy -Z (camera forward) → ENU +Y (North)
    /// - Bevy +Y (camera up) → ENU +Z (Up)
    /// - Bevy +X (camera right) → ENU +X (East)
    ///
    /// This is a 90° rotation around the X-axis (positive direction).
    #[inline]
    fn bevy_to_enu_rotation() -> Mat3 {
        // Column-major: each column specifies where that basis vector maps to
        // X → X, Y → Z, Z → -Y
        Mat3::from_cols(
            Vec3::new(1.0, 0.0, 0.0),  // X → (1, 0, 0) = X (East)
            Vec3::new(0.0, 0.0, 1.0),  // Y → (0, 0, 1) = Z (Up)
            Vec3::new(0.0, -1.0, 0.0), // Z → (0, -1, 0) = -Y (South, so -Z = North)
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
        // First apply camera rotation (in Bevy space), then convert to ENU
        bevy_to_enu * camera_rotation
    }

    /// Convert pixel coordinates to normalized device coordinates (-1 to 1).
    ///
    /// Note: This method is kept for testing purposes. The main `pixel_to_ray`
    /// method now uses `CameraIntrinsics::pixel_to_ray` which handles distortion.
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
    /// before computing the ray direction. This handles radial and tangential
    /// lens distortion for accurate triangulation.
    ///
    /// The ray direction is properly transformed from camera-local coordinates
    /// to ENU world coordinates using the camera's orientation, supporting
    /// arbitrary pitch, roll, and yaw angles.
    fn pixel_to_ray(&self, pose: &CameraPose, x: u32, y: u32, intensity: f32) -> Ray {
        // Warn (once per session) if camera has extreme tilt
        let tilt = Self::tilt_degrees(pose);
        if tilt > Self::EXTREME_TILT_WARNING_DEGREES {
            warn!(
                tilt_degrees = %tilt,
                "Camera has extreme tilt. While the coordinate transform \
                 handles this correctly, verify that the mounting orientation is \
                 intentional.",
            );
        }

        // Use calibrated intrinsics to compute ray direction in camera space.
        // This handles lens distortion correction automatically.
        let dir_camera = self.intrinsics.pixel_to_ray(x as f32, y as f32);

        // Transform from camera-local space to ENU world space using the
        // combined rotation matrix that handles both camera orientation and
        // coordinate system conversion.
        let camera_to_enu = Self::camera_to_enu_rotation(pose.orientation);
        let dir_enu = camera_to_enu * dir_camera;

        // Camera position in local grid coordinates (already in ENU via to_local_enu)
        let origin = pose.position.to_local_enu(&self.world_origin);

        Ray::new(origin, dir_enu, intensity)
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

    /// Raymarch from the difference mask, returning voxel contributions.
    ///
    /// If the number of contributions exceeds `MAX_CONTRIBUTIONS_PER_FRAME`,
    /// the results are truncated to the highest-intensity contributions.
    pub fn raymarch<S>(&self, pose: &CameraPose, mask: &DifferenceMask<S>) -> Vec<VoxelContribution>
    where
        S: AsRef<[u8]>,
    {
        let mut contributions: HashMap<(u32, u32, u32), f32> = HashMap::new();

        for (x, y, intensity) in mask.motion_pixels() {
            let ray = self.pixel_to_ray(pose, x, y, intensity as f32);
            self.march_ray_dda(&ray, &mut contributions);
        }

        let mut result: Vec<VoxelContribution> = contributions
            .into_iter()
            .map(|((x, y, z), intensity)| VoxelContribution {
                index: UVec3::new(x, y, z),
                intensity,
            })
            .collect();

        // Enforce the protocol limit on contributions per frame
        if result.len() > MAX_CONTRIBUTIONS_PER_FRAME {
            warn!(
                count = result.len(),
                limit = MAX_CONTRIBUTIONS_PER_FRAME,
                "Contribution limit exceeded, truncating to highest intensity"
            );
            // Sort by intensity descending and take the top N
            result.sort_unstable_by(|a, b| b.intensity.partial_cmp(&a.intensity).unwrap());
            result.truncate(MAX_CONTRIBUTIONS_PER_FRAME);
        }

        result
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
    use glam::{Quat, UVec2, Vec2};
    use iluvatar_core::{
        CameraPose, DistortionModel, Fov, LocalizationStatus, PoseUncertainty, Timestamp,
    };

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

    /// Create a quaternion from Euler angles (yaw, pitch, roll) in degrees.
    /// Uses the YXZ rotation order (yaw around Y, then pitch around X, then roll around Z).
    ///
    /// Note on Bevy/glam conventions:
    /// - Yaw: Positive Y rotation is counter-clockwise when viewed from above.
    ///   So yaw 90° turns LEFT (to face West), yaw -90° turns RIGHT (to face East).
    /// - Pitch: Positive X rotation tilts the camera UP (nose up, looking toward sky).
    ///   So pitch 10° looks up, pitch -10° looks down.
    /// - Roll: Positive Z rotation rolls the camera counter-clockwise from its POV.
    fn quat_from_euler_degrees(yaw: f32, pitch: f32, roll: f32) -> Quat {
        Quat::from_euler(
            glam::EulerRot::YXZ,
            yaw.to_radians(),
            pitch.to_radians(),
            roll.to_radians(),
        )
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
        let ray = Ray::new(Vec3::new(-5.0, 5.0, 5.0), Vec3::new(1.0, 0.0, 0.0), 1.0);

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
        let ray = Ray::new(Vec3::new(-1.0, -1.0, -1.0), Vec3::new(1.0, 1.0, 1.0), 1.0);

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
        let ray = Ray::new(
            Vec3::new(-5.0, 20.0, 5.0), // Above the grid
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
        );

        let mut contributions = HashMap::new();
        raymarcher.march_ray_dda(&ray, &mut contributions);

        // Should hit nothing
        assert!(contributions.is_empty());
    }

    #[test]
    fn test_contribution_limit_enforced() {
        // Test that contributions are limited to MAX_CONTRIBUTIONS_PER_FRAME
        // We simulate this by directly testing the truncation logic
        let mut contributions: Vec<VoxelContribution> = (0..100_000)
            .map(|i| VoxelContribution {
                index: UVec3::new(i % 1000, (i / 1000) % 100, i / 100_000),
                intensity: i as f32, // Higher index = higher intensity
            })
            .collect();

        // Simulate the truncation logic from raymarch()
        if contributions.len() > MAX_CONTRIBUTIONS_PER_FRAME {
            contributions.sort_unstable_by(|a, b| b.intensity.partial_cmp(&a.intensity).unwrap());
            contributions.truncate(MAX_CONTRIBUTIONS_PER_FRAME);
        }

        // Should be truncated to the limit
        assert_eq!(contributions.len(), MAX_CONTRIBUTIONS_PER_FRAME);

        // Should contain the highest intensity values (indices 99999 down to 100000 - 65536)
        let min_intensity = contributions
            .iter()
            .map(|c| c.intensity)
            .fold(f32::INFINITY, f32::min);
        assert!(min_intensity >= (100_000 - MAX_CONTRIBUTIONS_PER_FRAME) as f32);
    }

    // =========================================================================
    // Tilted Camera Tests
    // =========================================================================

    #[test]
    fn test_bevy_to_enu_rotation_matrix() {
        let rot = Raymarcher::bevy_to_enu_rotation();

        // Test that the rotation matrix correctly maps Bevy axes to ENU axes:
        // Bevy X (right) -> ENU X (East)
        let bevy_x = Vec3::X;
        let enu_from_x = rot * bevy_x;
        assert!(
            (enu_from_x - Vec3::X).length() < 1e-5,
            "Bevy X should map to ENU X (East)"
        );

        // Bevy Y (up) -> ENU Z (Up)
        let bevy_y = Vec3::Y;
        let enu_from_y = rot * bevy_y;
        assert!(
            (enu_from_y - Vec3::Z).length() < 1e-5,
            "Bevy Y should map to ENU Z (Up)"
        );

        // Bevy -Z (forward/camera direction) -> ENU Y (North)
        let bevy_neg_z = -Vec3::Z;
        let enu_from_neg_z = rot * bevy_neg_z;
        assert!(
            (enu_from_neg_z - Vec3::Y).length() < 1e-5,
            "Bevy -Z (forward) should map to ENU Y (North), got {:?}",
            enu_from_neg_z
        );

        // Bevy Z (back) -> ENU -Y (South)
        let bevy_z = Vec3::Z;
        let enu_from_z = rot * bevy_z;
        assert!(
            (enu_from_z - (-Vec3::Y)).length() < 1e-5,
            "Bevy Z (back) should map to ENU -Y (South), got {:?}",
            enu_from_z
        );
    }

    #[test]
    fn test_level_camera_center_ray_points_north() {
        // A level camera looking straight ahead (identity orientation in Bevy)
        // should produce a center ray pointing in the +Y (North) direction in ENU
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
        );

        let pose = test_pose_with_orientation(Quat::IDENTITY);
        let ray = raymarcher.pixel_to_ray(&pose, 960, 540, 1.0);

        // Center ray in camera space points down -Z (Bevy convention)
        // After transformation: should point in +Y direction (North in ENU)
        assert!(
            ray.direction.y > 0.9,
            "Level camera center ray should point North (+Y in ENU), got {:?}",
            ray.direction
        );
        assert!(
            ray.direction.x.abs() < 0.1,
            "Level camera center ray should have minimal East/West component"
        );
        assert!(
            ray.direction.z.abs() < 0.1,
            "Level camera center ray should have minimal Up/Down component"
        );
    }

    #[test]
    fn test_camera_yaw_90_points_west() {
        // Camera rotated 90° yaw (counter-clockwise from above) should point West
        // In Bevy: positive Y rotation is counter-clockwise when viewed from above
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
        );

        // Yaw 90° in Bevy = facing West (turned left from North)
        let pose = test_pose_with_orientation(quat_from_euler_degrees(90.0, 0.0, 0.0));
        let ray = raymarcher.pixel_to_ray(&pose, 960, 540, 1.0);

        // Should point West (-X in ENU)
        assert!(
            ray.direction.x < -0.9,
            "90° yaw camera should point West (-X in ENU), got {:?}",
            ray.direction
        );
    }

    #[test]
    fn test_camera_yaw_negative_90_points_east() {
        // Camera rotated -90° yaw (clockwise from above) should point East
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
        );

        // Yaw -90° in Bevy = facing East (turned right from North)
        let pose = test_pose_with_orientation(quat_from_euler_degrees(-90.0, 0.0, 0.0));
        let ray = raymarcher.pixel_to_ray(&pose, 960, 540, 1.0);

        // Should point East (+X in ENU)
        assert!(
            ray.direction.x > 0.9,
            "-90° yaw camera should point East (+X in ENU), got {:?}",
            ray.direction
        );
    }

    #[test]
    fn test_camera_pitch_up_10_degrees() {
        // Camera with 10° pitch up (positive pitch in Bevy's YXZ tilts nose up)
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
        );

        // Positive pitch = tilting up in Bevy
        let pitch_deg = 10.0_f32;
        let pose = test_pose_with_orientation(quat_from_euler_degrees(0.0, pitch_deg, 0.0));
        let ray = raymarcher.pixel_to_ray(&pose, 960, 540, 1.0);

        // Should still point mostly North, but with upward component
        let expected_z = pitch_deg.to_radians().sin();
        let expected_y = pitch_deg.to_radians().cos();

        assert!(
            (ray.direction.z - expected_z).abs() < 0.05,
            "10° pitch up should have Z ≈ {:.3}, got {:.3}",
            expected_z,
            ray.direction.z
        );
        assert!(
            (ray.direction.y - expected_y).abs() < 0.05,
            "10° pitch up should have Y ≈ {:.3}, got {:.3}",
            expected_y,
            ray.direction.y
        );
    }

    #[test]
    fn test_camera_pitch_down_10_degrees() {
        // Camera with 10° pitch down (negative pitch in Bevy tilts nose down)
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
        );

        // Negative pitch = tilting down in Bevy
        let pitch_deg = 10.0_f32;
        let pose = test_pose_with_orientation(quat_from_euler_degrees(0.0, -pitch_deg, 0.0));
        let ray = raymarcher.pixel_to_ray(&pose, 960, 540, 1.0);

        // Should still point mostly North, but with downward component
        let expected_z = -pitch_deg.to_radians().sin();
        let expected_y = pitch_deg.to_radians().cos();

        assert!(
            (ray.direction.z - expected_z).abs() < 0.05,
            "10° pitch down should have Z ≈ {:.3}, got {:.3}",
            expected_z,
            ray.direction.z
        );
        assert!(
            (ray.direction.y - expected_y).abs() < 0.05,
            "10° pitch down should have Y ≈ {:.3}, got {:.3}",
            expected_y,
            ray.direction.y
        );
    }

    #[test]
    fn test_camera_pitch_up_30_degrees() {
        // Camera with 30° pitch up (positive pitch in Bevy)
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
        );

        // Positive pitch = looking up
        let pitch_deg = 30.0_f32;
        let pose = test_pose_with_orientation(quat_from_euler_degrees(0.0, pitch_deg, 0.0));
        let ray = raymarcher.pixel_to_ray(&pose, 960, 540, 1.0);

        // Should point mostly North, with upward component
        let expected_z = pitch_deg.to_radians().sin();

        assert!(
            (ray.direction.z - expected_z).abs() < 0.05,
            "30° pitch up should have Z ≈ {:.3}, got {:.3}",
            expected_z,
            ray.direction.z
        );
        assert!(
            ray.direction.y > 0.5,
            "30° pitch up should still have positive Y (North), got {:?}",
            ray.direction
        );
    }

    #[test]
    fn test_camera_roll_45_degrees() {
        // Camera with 45° roll (tilted to the side)
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
        );

        // Roll 45°: rotation around the viewing axis
        let pose = test_pose_with_orientation(quat_from_euler_degrees(0.0, 0.0, 45.0));
        let ray = raymarcher.pixel_to_ray(&pose, 960, 540, 1.0);

        // Center ray should still point North (roll doesn't change center direction)
        assert!(
            ray.direction.y > 0.9,
            "45° roll camera center ray should still point North, got {:?}",
            ray.direction
        );

        // But off-center pixels should be rotated
        // A pixel on the right side of the image should now have an upward component
        let ray_right = raymarcher.pixel_to_ray(&pose, 1440, 540, 1.0);
        // With 45° roll, the "right" direction becomes "up-right"
        assert!(
            ray_right.direction.z > 0.1,
            "With 45° roll, right-side pixel should have upward component, got {:?}",
            ray_right.direction
        );
    }

    #[test]
    fn test_combined_yaw_and_pitch() {
        // Test combined yaw and pitch
        // 90° yaw (West) + 10° pitch up (positive in Bevy)
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
        );

        // Yaw 90° = looking West, pitch 10° = tilted up
        let pitch_deg = 10.0_f32;
        let pose = test_pose_with_orientation(quat_from_euler_degrees(90.0, pitch_deg, 0.0));
        let ray = raymarcher.pixel_to_ray(&pose, 960, 540, 1.0);

        // Should point West (-X in ENU) and slightly up (+Z)
        let expected_z = pitch_deg.to_radians().sin();

        assert!(
            ray.direction.x < -0.9,
            "90° yaw should point West (-X), got x={:.3}",
            ray.direction.x
        );
        assert!(
            (ray.direction.z - expected_z).abs() < 0.1,
            "10° pitch up should have Z ≈ {:.3}, got {:.3}",
            expected_z,
            ray.direction.z
        );
    }

    #[test]
    fn test_extreme_pitch_45_degrees_up() {
        // Test 45° pitch up (triggers the warning but should still work)
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
        );

        // Positive pitch = looking up
        let pitch_deg = 45.0_f32;
        let pose = test_pose_with_orientation(quat_from_euler_degrees(0.0, pitch_deg, 0.0));
        let ray = raymarcher.pixel_to_ray(&pose, 960, 540, 1.0);

        // At 45°, horizontal and vertical components should be equal
        let expected = 1.0 / 2.0_f32.sqrt(); // cos(45°) = sin(45°) = 1/√2

        assert!(
            (ray.direction.y - expected).abs() < 0.05,
            "45° pitch up should have Y ≈ {:.3}, got {:.3}",
            expected,
            ray.direction.y
        );
        assert!(
            (ray.direction.z - expected).abs() < 0.05,
            "45° pitch up should have Z ≈ {:.3}, got {:.3}",
            expected,
            ray.direction.z
        );
    }

    #[test]
    fn test_pitch_90_degrees_looking_straight_up() {
        // Extreme case: camera looking straight up
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
        );

        // Pitch 90° = camera looking at zenith (positive pitch = up in Bevy)
        let pose = test_pose_with_orientation(quat_from_euler_degrees(0.0, 90.0, 0.0));
        let ray = raymarcher.pixel_to_ray(&pose, 960, 540, 1.0);

        // Should point straight up (+Z in ENU)
        assert!(
            ray.direction.z > 0.99,
            "90° pitch up should point straight up (+Z), got {:?}",
            ray.direction
        );
    }

    #[test]
    fn test_ray_direction_is_normalized() {
        // Verify that rays are normalized for various orientations
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
        );

        let test_orientations = [
            (0.0, 0.0, 0.0),
            (90.0, 0.0, 0.0),
            (0.0, 30.0, 0.0),
            (0.0, 0.0, 45.0),
            (270.0, 10.0, 0.0),
            (45.0, -20.0, 15.0),
        ];

        for (yaw, pitch, roll) in test_orientations {
            let pose = test_pose_with_orientation(quat_from_euler_degrees(yaw, pitch, roll));
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

    #[test]
    fn test_off_center_pixels_with_tilt() {
        // Verify that off-center pixels are correctly transformed for tilted cameras
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
        );

        // Camera with 10° pitch up, pointing North
        let pose = test_pose_with_orientation(quat_from_euler_degrees(0.0, -10.0, 0.0));

        // Top of image should point even more upward
        let ray_top = raymarcher.pixel_to_ray(&pose, 960, 100, 1.0);
        // Bottom of image should point less upward (or downward)
        let ray_bottom = raymarcher.pixel_to_ray(&pose, 960, 980, 1.0);
        // Center ray for comparison
        let ray_center = raymarcher.pixel_to_ray(&pose, 960, 540, 1.0);

        assert!(
            ray_top.direction.z > ray_center.direction.z,
            "Top pixel should point more upward than center"
        );
        assert!(
            ray_bottom.direction.z < ray_center.direction.z,
            "Bottom pixel should point less upward than center"
        );

        // Left and right pixels should have East/West components
        let ray_left = raymarcher.pixel_to_ray(&pose, 200, 540, 1.0);
        let ray_right = raymarcher.pixel_to_ray(&pose, 1720, 540, 1.0);

        assert!(
            ray_left.direction.x < 0.0,
            "Left pixel should have West (-X) component"
        );
        assert!(
            ray_right.direction.x > 0.0,
            "Right pixel should have East (+X) component"
        );
    }
}
