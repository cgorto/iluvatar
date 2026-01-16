use glam::Quat;
use iluvatar_core::{CameraPose, GeoPosition, LocalizationStatus, PoseUncertainty, Timestamp};
use std::time::{SystemTime, UNIX_EPOCH};
use thiserror::Error;

fn now_micros() -> Timestamp {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_micros() as u64
}

#[derive(Debug, Error)]
pub enum LocalizationError {
    #[error("GPS device not available")]
    GpsUnavailable,
    #[error("IMU device not available")]
    ImuUnavailable,
    #[error("No valid fix")]
    NoFix,
}

/// GPS fallback behavior when signal is degraded
#[derive(Debug, Clone, Copy)]
pub enum GpsFallbackMode {
    /// Continue with IMU dead-reckoning
    DeadReckoning,
    /// Mark data with uncertainty
    MarkUncertain,
    /// Pause contributions
    Pause,
}

/// Abstract localization interface
pub trait Localizer: Send {
    fn get_pose(&mut self) -> Result<CameraPose, LocalizationError>;
    fn status(&self) -> LocalizationStatus;
}

/// Dummy localizer for testing - returns a fixed pose
pub struct DummyLocalizer {
    pose: CameraPose,
}

impl DummyLocalizer {
    pub fn new(position: GeoPosition, orientation: Quat) -> Self {
        Self {
            pose: CameraPose {
                position,
                orientation,
                timestamp: now_micros(),
                uncertainty: PoseUncertainty::default(),
                status: LocalizationStatus::Nominal,
            },
        }
    }

    pub fn with_position(lat: f64, lon: f64, alt: f64) -> Self {
        Self::new(GeoPosition::new(lat, lon, alt), Quat::IDENTITY)
    }
}

impl Localizer for DummyLocalizer {
    fn get_pose(&mut self) -> Result<CameraPose, LocalizationError> {
        self.pose.timestamp = now_micros();
        Ok(self.pose)
    }

    fn status(&self) -> LocalizationStatus {
        self.pose.status
    }
}

/// GPS + IMU fusion localizer.
///
/// **Not yet implemented.** This struct exists as a placeholder for future hardware
/// integration. Currently, `get_pose()` always returns `LocalizationError::GpsUnavailable`.
///
/// For actual deployments, use `DummyLocalizer` with a fixed position configured via
/// the `grid_origin` field in `ProcessingConfig`.
#[allow(dead_code)]
pub struct GpsImuLocalizer {
    last_gps_time: Option<std::time::Instant>,
    last_pose: Option<CameraPose>,
    fallback_mode: GpsFallbackMode,
    dead_reckoning_start: Option<std::time::Instant>,
}

impl GpsImuLocalizer {
    pub fn new(fallback_mode: GpsFallbackMode) -> Self {
        Self {
            last_gps_time: None,
            last_pose: None,
            fallback_mode,
            dead_reckoning_start: None,
        }
    }
}

impl Localizer for GpsImuLocalizer {
    fn get_pose(&mut self) -> Result<CameraPose, LocalizationError> {
        // TODO: Implement real GPS + IMU fusion
        // For now, return error indicating no implementation
        Err(LocalizationError::GpsUnavailable)
    }

    fn status(&self) -> LocalizationStatus {
        if let Some(start) = self.dead_reckoning_start {
            LocalizationStatus::DeadReckoning {
                duration_ms: start.elapsed().as_millis() as u64,
            }
        } else if self.last_pose.is_some() {
            LocalizationStatus::Nominal
        } else {
            LocalizationStatus::Unavailable
        }
    }
}
