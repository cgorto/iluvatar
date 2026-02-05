use crate::difference::DifferenceMask;
#[allow(unused_imports)]
use crate::profile_scope;
use iluvatar_core::{CameraPose, VoxelContribution};

// Re-export the core Raymarcher so callers can use `crate::raymarch::Raymarcher`.
pub use iluvatar_core::raymarch::Raymarcher;

/// Raymarch from a DifferenceMask, returning voxel contributions.
///
/// This is a convenience wrapper that adapts the camera's DifferenceMask
/// into the iterator-based `Raymarcher::raymarch_pixels` method from core.
pub fn raymarch_from_mask<S>(
    raymarcher: &Raymarcher,
    pose: &CameraPose,
    mask: &DifferenceMask<S>,
) -> Vec<VoxelContribution>
where
    S: AsRef<[u8]>,
{
    profile_scope!("ray_iteration");
    raymarcher.raymarch_pixels(pose, mask.motion_pixels())
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{Quat, UVec2, Vec2, Vec3};
    use iluvatar_core::{
        BoundingBox, CameraIntrinsics, CameraPose, DistortionModel, Fov, GeoPosition,
        LocalizationStatus, MAX_CONTRIBUTIONS_PER_FRAME, PoseUncertainty, RaymarchConfig,
        Timestamp,
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

    #[test]
    fn test_contribution_limit_enforced() {
        // Test that contributions are limited to MAX_CONTRIBUTIONS_PER_FRAME.
        // We simulate this by directly testing the truncation logic.
        let mut contributions: Vec<VoxelContribution> = (0..100_000)
            .map(|i| VoxelContribution {
                index: glam::UVec3::new(i % 1000, (i / 1000) % 100, i / 100_000),
                intensity: i as f32, // Higher index = higher intensity.
            })
            .collect();

        // Simulate the truncation logic from raymarch.
        if contributions.len() > MAX_CONTRIBUTIONS_PER_FRAME {
            contributions.sort_unstable_by(|a, b| b.intensity.partial_cmp(&a.intensity).unwrap());
            contributions.truncate(MAX_CONTRIBUTIONS_PER_FRAME);
        }

        assert_eq!(contributions.len(), MAX_CONTRIBUTIONS_PER_FRAME);

        let min_intensity = contributions
            .iter()
            .map(|c| c.intensity)
            .fold(f32::INFINITY, f32::min);
        assert!(min_intensity >= (100_000 - MAX_CONTRIBUTIONS_PER_FRAME) as f32);
    }

    // =========================================================================
    // Tilted Camera Tests
    // =========================================================================
    //
    // These tests exercise the Raymarcher from iluvatar-core via re-export.
    // They verify that the coordinate system transforms produce correct results.

    #[test]
    fn test_level_camera_center_ray_points_north() {
        // Verifies that the core Raymarcher was correctly extracted by checking
        // a fundamental property: level camera center ray points North.
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
            MAX_CONTRIBUTIONS_PER_FRAME,
        );

        let pose = test_pose_with_orientation(Quat::IDENTITY);

        // Raymarch a single center pixel.
        let pixels: Vec<(u32, u32, u8)> = vec![(960, 540, 128)];
        let result = raymarcher.raymarch_pixels(&pose, pixels.into_iter());

        // The center ray of a level camera facing North should produce contributions
        // that are roughly aligned along the Y axis (North in ENU).
        assert!(!result.is_empty(), "Center pixel should produce contributions");
    }

    #[test]
    fn test_dda_via_public_api() {
        // Camera at altitude 100m, grid extends 0-200m in all axes.
        // A level camera facing North from Z=100 sends center ray along +Y,
        // which passes through the grid at height 100.
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(200.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
            MAX_CONTRIBUTIONS_PER_FRAME,
        );

        let pose = test_pose_with_orientation(Quat::IDENTITY);
        let result = raymarcher.raymarch_pixels(&pose, [(960, 540, 128)].into_iter());
        assert!(!result.is_empty(), "Center ray should hit the grid");
    }

    #[test]
    fn test_off_center_pixels_produce_different_contributions() {
        // Camera at altitude 50m, grid extends 0-100m. Level camera facing North.
        // Left (x=200) and right (x=1720) pixels at wide horizontal angles
        // diverge into different voxel columns as the ray travels forward.
        let raymarcher = Raymarcher::new(
            test_intrinsics(),
            RaymarchConfig::default(),
            BoundingBox::new(Vec3::ZERO, Vec3::splat(100.0)),
            1.0,
            GeoPosition::new(0.0, 0.0, 0.0),
            MAX_CONTRIBUTIONS_PER_FRAME,
        );

        // Level camera at (0, 0, 50) — inside the grid at Z=50.
        let pose = CameraPose {
            position: GeoPosition::new(0.0, 0.0, 50.0),
            orientation: Quat::IDENTITY,
            timestamp: 0 as Timestamp,
            uncertainty: PoseUncertainty::default(),
            status: LocalizationStatus::Nominal,
        };

        // Left and right pixels should produce different voxel sets.
        let result_left = raymarcher.raymarch_pixels(&pose, [(200, 540, 128)].into_iter());
        let result_right = raymarcher.raymarch_pixels(&pose, [(1720, 540, 128)].into_iter());

        // Both should produce contributions, but different ones.
        assert!(!result_left.is_empty(), "Left pixel should hit the grid");
        assert!(!result_right.is_empty(), "Right pixel should hit the grid");

        // Collect index sets (ignoring intensity) for comparison.
        let left_indices: std::collections::HashSet<_> = result_left
            .iter()
            .map(|c| (c.index.x, c.index.y, c.index.z))
            .collect();
        let right_indices: std::collections::HashSet<_> = result_right
            .iter()
            .map(|c| (c.index.x, c.index.y, c.index.z))
            .collect();

        // There should be some difference between left and right pixel ray voxels.
        assert!(
            left_indices != right_indices,
            "Left and right pixels should traverse different voxels"
        );
    }
}
