//! Validation for data crossing the camera network boundary.
//!
//! Postcard guarantees that bytes have the expected Rust shape; it does not put
//! semantic bounds on vectors, RLE expansion, identifiers, or floating-point data.
//! Every transport calls this module before a message reaches the processing loop.

use glam::Vec3;
use iluvatar_core::{
    CameraIntrinsics, CameraMessage, CameraPose, CameraRegistration, CoordinateMode,
    DistortionModel, GeoPosition, GridConfigMessage, MAX_CONTRIBUTIONS_PER_FRAME,
    MAX_MOTION_PIXELS_PER_FRAME, MotionData, PROTOCOL_VERSION,
};
use thiserror::Error;

pub const MAX_CAMERA_ID: u64 = 63;
pub const MAX_IMAGE_DIMENSION: u32 = 16_384;
pub const MAX_CONTRIBUTION_INTENSITY: f32 = 65_535.0;
const MIN_FOCAL_LENGTH: f32 = 1.0;
const MAX_FOCAL_LENGTH: f32 = 1_000_000.0;
const MAX_DISTORTION_COEFFICIENT: f32 = 100.0;
const MAX_LOCAL_COORDINATE_METERS: f64 = 1_000_000.0;

#[derive(Debug, Error, PartialEq, Eq)]
pub enum ValidationError {
    #[error("unsupported protocol version")]
    Version,
    #[error("camera id is outside the 64-camera protocol limit")]
    CameraId,
    #[error("camera registration contains invalid intrinsics")]
    Intrinsics,
    #[error("camera pose contains invalid coordinates or orientation")]
    Pose,
    #[error("message camera id does not match its connection")]
    Identity,
    #[error("message type was not negotiated for this connection")]
    Capability,
    #[error("message contains too many observations")]
    ObservationLimit,
    #[error("motion pixel is outside the registered image")]
    PixelBounds,
    #[error("voxel contribution is invalid or outside the configured grid")]
    Contribution,
    #[error("message type is not accepted after registration")]
    MessageType,
}

pub fn validate_registration(
    registration: &CameraRegistration,
    coordinate_mode: CoordinateMode,
) -> Result<(), ValidationError> {
    if registration.version != PROTOCOL_VERSION {
        return Err(ValidationError::Version);
    }
    if registration.camera_id > MAX_CAMERA_ID {
        return Err(ValidationError::CameraId);
    }
    if !intrinsics_valid(&registration.intrinsics) {
        return Err(ValidationError::Intrinsics);
    }
    validate_pose(&registration.initial_pose, coordinate_mode)
}

pub fn validate_message(
    message: &CameraMessage,
    registration: &CameraRegistration,
    grid: &GridConfigMessage,
) -> Result<(), ValidationError> {
    match message {
        CameraMessage::Register(_) | CameraMessage::TimeSync { .. } => {
            Err(ValidationError::MessageType)
        }
        CameraMessage::Heartbeat { camera_id, .. } => validate_identity(*camera_id, registration),
        CameraMessage::Frame(frame) => {
            validate_identity(frame.camera_id, registration)?;
            if registration.capabilities.motion_frames {
                return Err(ValidationError::Capability);
            }
            validate_pose(&frame.pose, grid.coordinate_mode)?;
            if frame.contributions.len() > MAX_CONTRIBUTIONS_PER_FRAME {
                return Err(ValidationError::ObservationLimit);
            }
            for contribution in &frame.contributions {
                if !contribution.is_valid()
                    || contribution.intensity > MAX_CONTRIBUTION_INTENSITY
                    || contribution.index.x >= grid.dimensions[0]
                    || contribution.index.y >= grid.dimensions[1]
                    || contribution.index.z >= grid.dimensions[2]
                {
                    return Err(ValidationError::Contribution);
                }
            }
            Ok(())
        }
        CameraMessage::Motion(frame) => {
            validate_identity(frame.camera_id, registration)?;
            if !registration.capabilities.motion_frames {
                return Err(ValidationError::Capability);
            }
            validate_pose(&frame.pose, grid.coordinate_mode)?;
            if matches!(frame.motion, MotionData::RunLength(_))
                && !registration.capabilities.rle_encoding
            {
                return Err(ValidationError::Capability);
            }
            validate_motion(&frame.motion, &registration.intrinsics)
        }
    }
}

fn validate_identity(
    camera_id: u64,
    registration: &CameraRegistration,
) -> Result<(), ValidationError> {
    if camera_id == registration.camera_id {
        Ok(())
    } else {
        Err(ValidationError::Identity)
    }
}

fn validate_motion(
    motion: &MotionData,
    intrinsics: &CameraIntrinsics,
) -> Result<(), ValidationError> {
    match motion {
        MotionData::Sparse(pixels) => {
            if pixels.len() > MAX_MOTION_PIXELS_PER_FRAME {
                return Err(ValidationError::ObservationLimit);
            }
            for pixel in pixels {
                if u32::from(pixel.x) >= intrinsics.resolution.x
                    || u32::from(pixel.y) >= intrinsics.resolution.y
                {
                    return Err(ValidationError::PixelBounds);
                }
            }
        }
        MotionData::RunLength(runs) => {
            let mut pixel_count = 0usize;
            for run in runs {
                let run_end = u32::from(run.x_start) + u32::from(run.length);
                if run.length == 0
                    || run_end > intrinsics.resolution.x
                    || u32::from(run.y) >= intrinsics.resolution.y
                {
                    return Err(ValidationError::PixelBounds);
                }
                pixel_count = pixel_count
                    .checked_add(usize::from(run.length))
                    .ok_or(ValidationError::ObservationLimit)?;
                if pixel_count > MAX_MOTION_PIXELS_PER_FRAME {
                    return Err(ValidationError::ObservationLimit);
                }
            }
        }
    }
    Ok(())
}

fn intrinsics_valid(intrinsics: &CameraIntrinsics) -> bool {
    let resolution = intrinsics.resolution;
    let vectors_valid = intrinsics.focal_length.is_finite()
        && intrinsics.principal_point.is_finite()
        && intrinsics.focal_length.min_element() >= MIN_FOCAL_LENGTH
        && intrinsics.focal_length.max_element() <= MAX_FOCAL_LENGTH
        && intrinsics.principal_point.min_element() >= 0.0;
    let resolution_valid = resolution.x > 0
        && resolution.y > 0
        && resolution.x <= MAX_IMAGE_DIMENSION
        && resolution.y <= MAX_IMAGE_DIMENSION
        && intrinsics.principal_point.x < resolution.x as f32
        && intrinsics.principal_point.y < resolution.y as f32;
    let fov_valid = intrinsics.fov.horizontal.is_finite()
        && intrinsics.fov.vertical.is_finite()
        && intrinsics.fov.horizontal > 0.0
        && intrinsics.fov.vertical > 0.0
        && intrinsics.fov.horizontal < std::f32::consts::PI
        && intrinsics.fov.vertical < std::f32::consts::PI;
    let distortion_valid = match intrinsics.distortion {
        DistortionModel::None => true,
        DistortionModel::OpenCV5 { k1, k2, p1, p2, k3 } => [k1, k2, p1, p2, k3]
            .iter()
            .all(|value| value.is_finite() && value.abs() <= MAX_DISTORTION_COEFFICIENT),
        DistortionModel::KannalaBrandt4 { k1, k2, k3, k4 } => [k1, k2, k3, k4]
            .iter()
            .all(|value| value.is_finite() && value.abs() <= MAX_DISTORTION_COEFFICIENT),
    };
    let rays_valid = if vectors_valid && resolution_valid && fov_valid && distortion_valid {
        let max_x = (resolution.x - 1) as f32;
        let max_y = (resolution.y - 1) as f32;
        [
            (0.0, 0.0),
            (max_x, 0.0),
            (0.0, max_y),
            (max_x, max_y),
            (intrinsics.principal_point.x, intrinsics.principal_point.y),
        ]
        .iter()
        .all(|&(x, y)| {
            let ray = intrinsics.pixel_to_ray(x, y);
            ray.is_finite() && (ray.length_squared() - 1.0).abs() <= 0.01
        })
    } else {
        false
    };
    vectors_valid && resolution_valid && fov_valid && distortion_valid && rays_valid
}

fn validate_pose(pose: &CameraPose, mode: CoordinateMode) -> Result<(), ValidationError> {
    let position_valid = match mode {
        CoordinateMode::Gps => GeoPosition::new_checked(
            pose.position.latitude,
            pose.position.longitude,
            pose.position.altitude,
        )
        .is_ok(),
        CoordinateMode::Local => [
            pose.position.latitude,
            pose.position.longitude,
            pose.position.altitude,
        ]
        .iter()
        .all(|value| value.is_finite() && value.abs() <= MAX_LOCAL_COORDINATE_METERS),
    };
    let orientation_length = pose.orientation.length_squared();
    let orientation_valid = pose.orientation.is_finite()
        && orientation_length.is_finite()
        && (orientation_length - 1.0).abs() <= 0.01;
    let uncertainty_valid = vector_nonnegative_finite(pose.uncertainty.position_stddev)
        && vector_nonnegative_finite(pose.uncertainty.orientation_stddev);

    if position_valid && orientation_valid && uncertainty_valid {
        Ok(())
    } else {
        Err(ValidationError::Pose)
    }
}

fn vector_nonnegative_finite(value: Vec3) -> bool {
    value.is_finite() && value.min_element() >= 0.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{Quat, UVec2, Vec2};
    use iluvatar_core::{
        CameraCapabilities, CameraFrame, Fov, LocalizationStatus, MotionFrame, MotionPixel,
        MotionRun, PoseUncertainty, VoxelContribution,
    };

    fn registration() -> CameraRegistration {
        CameraRegistration {
            version: PROTOCOL_VERSION,
            camera_id: 7,
            intrinsics: CameraIntrinsics {
                focal_length: Vec2::splat(600.0),
                principal_point: Vec2::new(640.0, 360.0),
                resolution: UVec2::new(1280, 720),
                fov: Fov {
                    horizontal: 1.2,
                    vertical: 0.7,
                },
                distortion: DistortionModel::None,
            },
            initial_pose: pose(),
            capabilities: CameraCapabilities::with_motion_frames(),
        }
    }

    fn pose() -> CameraPose {
        CameraPose {
            position: GeoPosition::new(47.6, -122.3, 10.0),
            orientation: Quat::IDENTITY,
            timestamp: 1,
            uncertainty: PoseUncertainty::default(),
            status: LocalizationStatus::Nominal,
        }
    }

    fn grid() -> GridConfigMessage {
        GridConfigMessage {
            origin_lat: 47.6,
            origin_lon: -122.3,
            origin_alt: 0.0,
            dimensions: [100, 100, 50],
            voxel_size: 1.0,
            coordinate_mode: CoordinateMode::Gps,
        }
    }

    fn motion(data: MotionData) -> CameraMessage {
        CameraMessage::Motion(MotionFrame {
            camera_id: 7,
            sequence: 1,
            timestamp: 1,
            pose: pose(),
            motion: data,
        })
    }

    #[test]
    fn valid_motion_is_accepted() {
        let message = motion(MotionData::Sparse(vec![MotionPixel::new(1279, 719, 10)]));
        assert_eq!(validate_message(&message, &registration(), &grid()), Ok(()));
    }

    #[test]
    fn registration_rejects_invalid_identity_and_floats() {
        let mut candidate = registration();
        candidate.camera_id = 64;
        assert_eq!(
            validate_registration(&candidate, CoordinateMode::Gps),
            Err(ValidationError::CameraId)
        );

        candidate = registration();
        candidate.intrinsics.focal_length.x = f32::NAN;
        assert_eq!(
            validate_registration(&candidate, CoordinateMode::Gps),
            Err(ValidationError::Intrinsics)
        );

        candidate = registration();
        candidate.intrinsics.focal_length = Vec2::splat(f32::MIN_POSITIVE);
        assert_eq!(
            validate_registration(&candidate, CoordinateMode::Gps),
            Err(ValidationError::Intrinsics)
        );

        candidate = registration();
        candidate.initial_pose.position.latitude = f64::MAX;
        assert_eq!(
            validate_registration(&candidate, CoordinateMode::Local),
            Err(ValidationError::Pose)
        );
    }

    #[test]
    fn connection_identity_cannot_be_spoofed() {
        let mut message = motion(MotionData::Sparse(Vec::new()));
        let CameraMessage::Motion(frame) = &mut message else {
            unreachable!()
        };
        frame.camera_id = 8;
        assert_eq!(
            validate_message(&message, &registration(), &grid()),
            Err(ValidationError::Identity)
        );
    }

    #[test]
    fn sparse_pixels_must_fit_registered_image() {
        let message = motion(MotionData::Sparse(vec![MotionPixel::new(1280, 0, 10)]));
        assert_eq!(
            validate_message(&message, &registration(), &grid()),
            Err(ValidationError::PixelBounds)
        );
    }

    #[test]
    fn rle_expansion_is_bounded_before_iteration() {
        let run_length = 1280u16;
        let run_count = MAX_MOTION_PIXELS_PER_FRAME / usize::from(run_length) + 1;
        let runs = vec![MotionRun::new(0, 0, run_length, 10); run_count];
        let message = motion(MotionData::RunLength(runs));
        let mut peer = registration();
        peer.capabilities.rle_encoding = true;
        assert_eq!(
            validate_message(&message, &peer, &grid()),
            Err(ValidationError::ObservationLimit)
        );
    }

    #[test]
    fn unnegotiated_rle_is_rejected() {
        let message = motion(MotionData::RunLength(vec![MotionRun::new(0, 0, 10, 1)]));
        assert_eq!(
            validate_message(&message, &registration(), &grid()),
            Err(ValidationError::Capability)
        );
    }

    #[test]
    fn contribution_intensity_cannot_overflow_the_grid() {
        let mut peer = registration();
        peer.capabilities = CameraCapabilities::basic();
        let message = CameraMessage::Frame(CameraFrame {
            camera_id: peer.camera_id,
            sequence: 1,
            timestamp: 1,
            pose: pose(),
            contributions: vec![VoxelContribution {
                index: glam::UVec3::ZERO,
                intensity: f32::MAX,
            }],
        });
        assert_eq!(
            validate_message(&message, &peer, &grid()),
            Err(ValidationError::Contribution)
        );
    }

    #[test]
    fn post_registration_control_messages_are_rejected() {
        let message = CameraMessage::TimeSync { timestamp: 0 };
        assert_eq!(
            validate_message(&message, &registration(), &grid()),
            Err(ValidationError::MessageType)
        );
    }
}
