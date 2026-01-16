use glam::{UVec2, Vec2};
use iluvatar_core::{CameraId, CameraIntrinsics, Fov, GeoOrigin, RaymarchConfig};
use serde::Deserialize;
use std::path::Path;
use std::time::Duration;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ConfigError {
    #[error("Failed to read config file: {0}")]
    Read(#[from] std::io::Error),
    #[error("Failed to parse config: {0}")]
    Parse(#[from] toml::de::Error),
}

#[derive(Debug, Clone, Deserialize)]
pub struct CameraConfig {
    pub identity: IdentityConfig,
    pub hardware: HardwareConfig,
    pub network: NetworkConfig,
    pub processing: ProcessingConfig,
    pub localization: LocalizationConfig,
}

#[derive(Debug, Clone, Deserialize)]
pub struct IdentityConfig {
    pub camera_id: CameraId,
}

#[derive(Debug, Clone, Deserialize)]
pub struct HardwareConfig {
    pub device: String,
    pub width: u32,
    pub height: u32,
    pub fps: u32,
}

#[derive(Debug, Clone, Deserialize)]
pub struct NetworkConfig {
    pub server_address: String,
    #[serde(default = "default_connection_timeout_secs")]
    pub connection_timeout_secs: u64,
    #[serde(default = "default_frame_buffer_size")]
    pub frame_buffer_size: usize,
}

fn default_connection_timeout_secs() -> u64 {
    30
}

fn default_frame_buffer_size() -> usize {
    4
}

#[derive(Debug, Clone, Deserialize)]
pub struct ProcessingConfig {
    #[serde(default = "default_difference_threshold")]
    pub difference_threshold: u8,
    #[serde(default = "default_motion_threshold_percent")]
    pub motion_threshold_percent: f32,
    #[serde(default)]
    pub raymarch: RaymarchSettings,
    /// Geographic origin for the voxel grid coordinate system.
    ///
    /// This field is required. All camera positions and voxel coordinates are computed
    /// relative to this origin. Typically set to the center of the monitored area.
    pub grid_origin: GeoOrigin,
}

fn default_difference_threshold() -> u8 {
    20
}

fn default_motion_threshold_percent() -> f32 {
    0.1
}

#[derive(Debug, Clone, Deserialize)]
pub struct RaymarchSettings {
    #[serde(default = "default_max_distance")]
    pub max_distance: f32,
    #[serde(default = "default_step_size")]
    pub step_size: f32,
}

impl Default for RaymarchSettings {
    fn default() -> Self {
        Self {
            max_distance: default_max_distance(),
            step_size: default_step_size(),
        }
    }
}

fn default_max_distance() -> f32 {
    500.0
}

fn default_step_size() -> f32 {
    0.5
}

#[derive(Debug, Clone, Deserialize)]
pub struct LocalizationConfig {
    pub gps_device: String,
    #[serde(default = "default_gps_timeout_secs")]
    pub gps_timeout_secs: u64,
}

fn default_gps_timeout_secs() -> u64 {
    60
}

impl CameraConfig {
    pub fn load(path: &Path) -> Result<Self, ConfigError> {
        let content = std::fs::read_to_string(path)?;
        let config: CameraConfig = toml::from_str(&content)?;
        Ok(config)
    }

    pub fn connection_timeout(&self) -> Duration {
        Duration::from_secs(self.network.connection_timeout_secs)
    }

    pub fn gps_timeout(&self) -> Duration {
        Duration::from_secs(self.localization.gps_timeout_secs)
    }

    pub fn frame_interval(&self) -> Duration {
        Duration::from_secs_f64(1.0 / self.hardware.fps as f64)
    }

    pub fn resolution(&self) -> (u32, u32) {
        (self.hardware.width, self.hardware.height)
    }

    pub fn pixel_count(&self) -> u32 {
        self.hardware.width * self.hardware.height
    }

    pub fn motion_pixel_threshold(&self) -> u32 {
        ((self.pixel_count() as f32) * (self.processing.motion_threshold_percent / 100.0)) as u32
    }

    pub fn to_intrinsics(&self) -> CameraIntrinsics {
        let (width, height) = self.resolution();
        let aspect = width as f32 / height as f32;

        // Default FOV assumptions - should ideally come from calibration
        let fov_horizontal = std::f32::consts::FRAC_PI_2; // 90 degrees
        let fov_vertical = fov_horizontal / aspect;

        CameraIntrinsics {
            focal_length: Vec2::new(
                width as f32 / (2.0 * (fov_horizontal / 2.0).tan()),
                height as f32 / (2.0 * (fov_vertical / 2.0).tan()),
            ),
            principal_point: Vec2::new(width as f32 / 2.0, height as f32 / 2.0),
            resolution: UVec2::new(width, height),
            fov: Fov {
                horizontal: fov_horizontal,
                vertical: fov_vertical,
            },
        }
    }

    pub fn to_raymarch_config(&self) -> RaymarchConfig {
        RaymarchConfig {
            max_distance: self.processing.raymarch.max_distance,
            step_size: self.processing.raymarch.step_size,
            ..Default::default()
        }
    }
}
